"""
Robot Agent - Complete robot integration with MuJoCo physics simulation.

This implementation connects all robot components (TaskHandler, PathPlanner, 
MotionExecutor, StateHolder) with the MuJoCo physics engine following our
architecture principles: loose coupling, interface-driven design, and SOLID principles.
"""
import threading
import time
from typing import Optional, Set, Tuple
from dataclasses import dataclass

from interfaces.task_handler_interface import ITaskHandler, Task, TaskType
from interfaces.path_planner_interface import IPathPlanner, Cell
from interfaces.motion_executor_interface import IMotionExecutor
from interfaces.state_holder_interface import IStateHolder
from interfaces.coordinate_system_interface import ICoordinateSystem

from robot.impl.task_handler_impl import TaskHandlerImpl
from robot.impl.path_planner_impl import PathPlannerImpl
from robot.impl.motion_executor_impl import MotionExecutorImpl
from robot.impl.state_holder_impl import StateHolderImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from robot.impl.physics_integration import create_command_adapter, create_state_adapter

from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics


@dataclass
class RobotConfiguration:
    """Robot configuration parameters following Single Responsibility Principle."""
    robot_id: str = "robot_1"
    max_speed: float = 1.0
    position_tolerance: float = 0.1
    control_frequency: float = 10.0  # Hz
    motion_frequency: float = 100.0  # Hz
    start_position: Optional[Tuple[float, float]] = None  # Override warehouse default start position
    cell_size: float = 0.25  # Robot navigation cell size (should divide warehouse cell size evenly)
    
    def get_effective_start_position(self, warehouse_map: WarehouseMap) -> Tuple[float, float]:
        """Get the effective start position, using configuration override or warehouse default."""
        if self.start_position is not None:
            return self.start_position
        return warehouse_map.start_position
    
    def validate_cell_size(self, warehouse_cell_size: float) -> bool:
        """Validate that robot cell size divides warehouse cell size evenly."""
        if warehouse_cell_size <= 0 or self.cell_size <= 0:
            return False
        
        # Check if warehouse cell size is evenly divisible by robot cell size
        scale_factor = warehouse_cell_size / self.cell_size
        return abs(scale_factor - round(scale_factor)) < 1e-10  # Allow for floating point precision


class RobotAgent:
    """
    Complete robot agent integrating all components with MuJoCo physics.
    
    Architecture:
    - Interface-driven: All components accessed via interfaces
    - Loose coupling: Components don't know about each other directly
    - Thread-safe: Proper synchronization for multi-threaded operation
    - SOLID principles: Single responsibility, dependency injection
    
    Threading Model:
    - Control Thread: 10Hz task management
    - Motion Thread: 100Hz motion control  
    - Physics Thread: 1000Hz physics simulation (external)
    """
    
    def __init__(self, 
                 warehouse_map: WarehouseMap,
                 physics: SimpleMuJoCoPhysics,
                 config: Optional[RobotConfiguration] = None):
        """
        Initialize robot agent with dependency injection.
        
        Args:
            warehouse_map: Warehouse layout and obstacles
            physics: MuJoCo physics simulation
            config: Robot configuration parameters
        """
        self.config = config or RobotConfiguration()
        self.warehouse_map = warehouse_map
        self.physics = physics
        
        # Validate configuration
        self._validate_configuration()
        
        # Initialize coordinate system
        self.coordinate_system = self._create_coordinate_system()
        
        # Initialize robot components via dependency injection
        self.state_holder = self._create_state_holder()
        self.path_planner = self._create_path_planner()
        self.motion_executor = self._create_motion_executor()
        self.task_handler = self._create_task_handler()
        
        # Control loop management
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._motion_thread: Optional[threading.Thread] = None
        
        print(f"[RobotAgent] Initialized {self.config.robot_id} with all components")
    
    def _validate_configuration(self) -> None:
        """Validate robot configuration against warehouse constraints."""
        if not self.config.validate_cell_size(self.warehouse_map.grid_size):
            warehouse_size = self.warehouse_map.grid_size
            robot_size = self.config.cell_size
            scale_factor = warehouse_size / robot_size
            raise ValueError(
                f"Robot cell size ({robot_size}m) does not divide warehouse cell size ({warehouse_size}m) evenly. "
                f"Scale factor: {scale_factor}. Use cell sizes like 0.25m, 0.125m, or 0.1m for clean scaling."
            )
        
        # Validate start position if provided
        if self.config.start_position is not None:
            x, y = self.config.start_position
            if not self.warehouse_map.is_walkable(x, y):
                raise ValueError(f"Configured start position ({x}, {y}) is not walkable in warehouse")
    
    def _create_coordinate_system(self) -> ICoordinateSystem:
        """Create coordinate system aligned with warehouse using configuration."""
        # Use configured cell size for perfect scaling
        scale_factor = self.warehouse_map.grid_size / self.config.cell_size
        scaled_width = int(self.warehouse_map.width * scale_factor)
        scaled_height = int(self.warehouse_map.height * scale_factor)
        
        print(f"[RobotAgent] Coordinate system: {self.config.cell_size}m cells, "
              f"scale factor {scale_factor}, grid {scaled_width}x{scaled_height}")
        
        return CoordinateSystemImpl(
            cell_size=self.config.cell_size,
            grid_width=scaled_width,
            grid_height=scaled_height
        )
    
    def _create_state_holder(self) -> IStateHolder:
        """Create state holder connected to physics."""
        return StateHolderImpl(
            robot_id=self.config.robot_id,
            model=self.physics.model,
            data=self.physics.data,
            physics_engine=self.physics  # Add physics engine reference
        )
    
    def _create_path_planner(self) -> IPathPlanner:
        """Create path planner with warehouse obstacles."""
        # Extract obstacles from warehouse map
        obstacles = self._extract_warehouse_obstacles()
        
        return PathPlannerImpl(
            coordinate_system=self.coordinate_system,
            static_obstacles=obstacles
        )
    
    def _create_motion_executor(self) -> IMotionExecutor:
        """Create motion executor connected to physics."""
        return MotionExecutorImpl(
            coordinate_system=self.coordinate_system,
            model=self.physics.model,
            data=self.physics.data,
            robot_id=self.config.robot_id,
            physics_engine=self.physics  # Add physics engine reference
        )
    
    def _create_task_handler(self) -> ITaskHandler:
        """Create task handler with all dependencies."""
        return TaskHandlerImpl(
            state_holder=self.state_holder,
            path_planner=self.path_planner,
            motion_executor=self.motion_executor,
            coordinate_system=self.coordinate_system,
            robot_id=self.config.robot_id
        )
    
    def _extract_warehouse_obstacles(self) -> Set[Cell]:
        """Extract obstacle cells from warehouse map with perfect integer scaling."""
        obstacles = set()
        
        # Calculate scale factor (now guaranteed to be integer)
        scale_factor = int(self.warehouse_map.grid_size / self.config.cell_size)
        
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                cell_type = self.warehouse_map.grid[y, x]
                if cell_type in [1, 2]:  # Walls and shelves are obstacles
                    # Scale obstacle positions with perfect integer mapping
                    # Each warehouse cell becomes scale_factor x scale_factor robot cells
                    for scaled_y in range(y * scale_factor, (y + 1) * scale_factor):
                        for scaled_x in range(x * scale_factor, (x + 1) * scale_factor):
                            obstacles.add(Cell(scaled_x, scaled_y))
        
        return obstacles
    
    def initialize_position(self) -> None:
        """Initialize robot at configured start position."""
        start_pos = self.config.get_effective_start_position(self.warehouse_map)
        self.physics.reset_robot_position(start_pos[0], start_pos[1], 0.0)
        print(f"[RobotAgent] Robot positioned at: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")
    
    # Public interface methods
    def start(self) -> None:
        """Start robot agent control loops."""
        if self._running:
            print(f"[RobotAgent] {self.config.robot_id} already running")
            return
        
        self._running = True
        
        # Start control threads
        self._control_thread = threading.Thread(
            target=self._control_loop,
            name=f"Control-{self.config.robot_id}",
            daemon=True
        )
        
        self._motion_thread = threading.Thread(
            target=self._motion_loop,
            name=f"Motion-{self.config.robot_id}",
            daemon=True
        )
        
        self._control_thread.start()
        self._motion_thread.start()
        
        print(f"[RobotAgent] Started {self.config.robot_id} control loops")
    
    def stop(self) -> None:
        """Stop robot agent control loops."""
        if not self._running:
            return
        
        self._running = False
        
        # Emergency stop motion
        self.motion_executor.emergency_stop()
        
        # Wait for threads to finish
        if self._control_thread:
            self._control_thread.join(timeout=1.0)
        if self._motion_thread:
            self._motion_thread.join(timeout=1.0)
        
        print(f"[RobotAgent] Stopped {self.config.robot_id}")
    
    def assign_task(self, task: Task) -> bool:
        """
        Assign a task to the robot.
        
        Args:
            task: Task to execute
            
        Returns:
            bool: True if task accepted, False if robot busy
        """
        return self.task_handler.start_task(task)
    
    def get_status(self) -> dict:
        """Get comprehensive robot status."""
        task_status = self.task_handler.get_task_status()
        motion_status = self.motion_executor.get_motion_status()
        robot_state = self.state_holder.get_robot_state()
        
        return {
            'robot_id': self.config.robot_id,
            'position': robot_state.position,
            'battery': robot_state.battery_level,
            'task_active': task_status.has_active_task,
            'task_id': task_status.task_id,
            'operational_status': task_status.operational_status.value,
            'task_progress': task_status.progress,
            'motion_status': motion_status.value,
            'current_target': self.motion_executor.get_current_target()
        }
    
    # Control loop implementations
    def _control_loop(self) -> None:
        """Control loop running at 10Hz for task management."""
        control_period = 1.0 / self.config.control_frequency
        
        while self._running:
            start_time = time.time()
            
            try:
                # Update state from physics
                self.state_holder.update_from_simulation()
                
                # Update task execution
                self.task_handler.update_task_execution()
                
            except Exception as e:
                print(f"[RobotAgent] Control loop error: {e}")
            
            # Maintain 10Hz frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, control_period - elapsed)
            time.sleep(sleep_time)
    
    def _motion_loop(self) -> None:
        """Motion loop running at 100Hz for motion control."""
        motion_period = 1.0 / self.config.motion_frequency
        
        while self._running:
            start_time = time.time()
            
            try:
                # Update motion control
                self.motion_executor.update_control_loop()
                
            except Exception as e:
                print(f"[RobotAgent] Motion loop error: {e}")
            
            # Maintain 100Hz frequency  
            elapsed = time.time() - start_time
            sleep_time = max(0, motion_period - elapsed)
            time.sleep(sleep_time)
    
    # Task creation helpers
    def create_pickup_task(self, shelf_id: str, item_id: str = "item_1") -> Task:
        """Create a pick and deliver task."""
        return Task(
            task_id=f"pickup_{shelf_id}_{int(time.time())}",
            task_type=TaskType.PICK_AND_DELIVER,
            shelf_id=shelf_id,
            item_id=item_id,
            priority=1
        )
    
    def create_move_task(self, target_x: float, target_y: float) -> Task:
        """Create a move to position task."""
        return Task(
            task_id=f"move_{int(time.time())}",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(target_x, target_y, 0.0),
            priority=1
        )
    
    def create_charging_task(self) -> Task:
        """Create a charging task."""
        return Task(
            task_id=f"charge_{int(time.time())}",
            task_type=TaskType.MOVE_TO_CHARGING,
            priority=1
        )
    
    # Interface access (for external systems)
    @property
    def task_handler_interface(self) -> ITaskHandler:
        """Access to task handler interface."""
        return self.task_handler
    
    @property  
    def motion_executor_interface(self) -> IMotionExecutor:
        """Access to motion executor interface."""
        return self.motion_executor
    
    @property
    def state_holder_interface(self) -> IStateHolder:
        """Access to state holder interface."""
        return self.state_holder
    
    @property
    def path_planner_interface(self) -> IPathPlanner:
        """Access to path planner interface."""
        return self.path_planner 