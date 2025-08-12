"""
DEPRECATED: LEGACY ROBOT AGENT - DO NOT USE

This module is DEPRECATED and should not be used in new code.
Use robot_agent_lane_based.py instead.

This was the original robot agent implementation that has been superseded by
the lane-based robot agent which provides:
- Better lane-based navigation
- Conflict box management
- Centralized configuration
- Parallel bidding system
- Enhanced database integration

Migration Guide:
- Replace: from robot.robot_agent import RobotAgent
- With: from robot.robot_agent_lane_based import RobotAgent

This file is kept for reference only and may be removed in future versions.
"""
import threading
import time
from typing import Optional, Set, Tuple, List, Dict, Any
from dataclasses import dataclass

from interfaces.task_handler_interface import ITaskHandler, Task, TaskType
from interfaces.path_planner_interface import IPathPlanner
from interfaces.motion_executor_interface import IMotionExecutor
from interfaces.state_holder_interface import IStateHolder
from interfaces.coordinate_system_interface import ICoordinateSystem
from interfaces.lane_follower_interface import ILaneFollower
from interfaces.simulation_data_service_interface import ISimulationDataService

from robot.impl.task_handler_impl import TaskHandlerImpl
from robot.impl.path_planner_graph_impl import PathPlannerGraphImpl
from robot.impl.lane_follower_impl import LaneFollowerImpl
from robot.impl.motion_executor_impl import MotionExecutorImpl
from robot.impl.state_holder_impl import StateHolderImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from robot.impl.physics_integration import create_command_adapter, create_state_adapter
from simulation.simulation_data_service_impl import SimulationDataServiceImpl

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
    
    # Lane-based navigation parameters
    lane_tolerance: float = 0.1  # Meters from center-line
    corner_speed: float = 0.3  # Speed in conflict boxes and turns
    bay_approach_speed: float = 0.2  # Speed when approaching bays
    conflict_box_lock_timeout: float = 30.0  # Seconds to wait for lock
    conflict_box_heartbeat_interval: float = 5.0  # Seconds between heartbeats
    
    def get_effective_start_position(self, warehouse_map: WarehouseMap) -> Tuple[float, float]:
        """Get the effective start position, using configuration override or warehouse default."""
        if self.start_position is not None:
            return self.start_position
        return warehouse_map.start_position
    
    def validate_start_position(self, warehouse_map: WarehouseMap) -> bool:
        """Validate that start position is walkable in warehouse."""
        if self.start_position is not None:
            x, y = self.start_position
            return warehouse_map.is_walkable(x, y)
        return True


class RobotAgent:
    """
    DEPRECATED: LEGACY ROBOT AGENT - DO NOT USE
    
    This class is DEPRECATED and should not be used in new code.
    Use robot_agent_lane_based.RobotAgent instead.
    
    This was the original robot agent implementation that has been superseded by
    the lane-based robot agent which provides better navigation and configuration.
    
    Architecture:
    - Interface-driven: All components accessed via interfaces
    - Loose coupling: Components don't know about each other directly
    - Thread-safe: Proper synchronization for multi-threaded operation
    - SOLID principles: Single responsibility, dependency injection
    - Lane-based navigation: Uses NavigationGraph and conflict box management
    
    Threading Model:
    - Control Thread: 10Hz task management
    - Motion Thread: 100Hz motion control  
    - Physics Thread: 1000Hz physics simulation (external)
    """
    
    def __init__(self, 
                 warehouse_map: WarehouseMap,
                 physics: SimpleMuJoCoPhysics,
                 config: Optional[RobotConfiguration] = None,
                 simulation_data_service: Optional[ISimulationDataService] = None):
        """
        DEPRECATED: LEGACY ROBOT AGENT - DO NOT USE
        
        Initialize robot agent with dependency injection.
        
        WARNING: This class is deprecated. Use robot_agent_lane_based.RobotAgent instead.
        
        Args:
            warehouse_map: Warehouse layout and obstacles
            physics: MuJoCo physics simulation
            config: Robot configuration parameters
            simulation_data_service: Database service (created if not provided)
        """
        import warnings
        warnings.warn(
            "RobotAgent from robot_agent.py is DEPRECATED. "
            "Use robot_agent_lane_based.RobotAgent instead.",
            DeprecationWarning,
            stacklevel=2
        )
        self.config = config or RobotConfiguration()
        self.warehouse_map = warehouse_map
        self.physics = physics
        
        # Validate configuration
        self._validate_configuration()
        
        # Initialize simulation data service (database connectivity)
        self.simulation_data_service = simulation_data_service or self._create_simulation_data_service()
        
        # Initialize coordinate system (simplified for lane-based navigation)
        self.coordinate_system = self._create_coordinate_system()
        
        # Initialize robot components via dependency injection
        self.state_holder = self._create_state_holder()
        self.path_planner = self._create_path_planner()
        self.motion_executor = self._create_motion_executor()
        self.lane_follower = self._create_lane_follower()
        self.task_handler = self._create_task_handler()
        
        # Initialize physics thread manager with state holder for 1kHz synchronization
        from robot.impl.physics_integration import create_physics_thread_manager
        self.physics_manager = create_physics_thread_manager(
            physics=self.physics,
            state_holder=self.state_holder,
            frequency_hz=1000.0,
            db_sync_frequency_hz=10.0
        )
        
        # Control loop management
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._motion_thread: Optional[threading.Thread] = None
        
        print(f"[RobotAgent] Initialized {self.config.robot_id} with lane-based navigation")
        print(f"[RobotAgent] Components: PathPlanner, LaneFollower, MotionExecutor, TaskHandler")
        print(f"[RobotAgent] Database: {type(self.simulation_data_service).__name__}")
    
    def _validate_configuration(self) -> None:
        """Validate robot configuration against warehouse constraints."""
        # Validate start position if provided
        if not self.config.validate_start_position(self.warehouse_map):
            x, y = self.config.start_position
            raise ValueError(f"Configured start position ({x}, {y}) is not walkable in warehouse")
        
        # Validate lane-based parameters
        if self.config.lane_tolerance <= 0:
            raise ValueError("Lane tolerance must be positive")
        if self.config.corner_speed <= 0 or self.config.corner_speed > self.config.max_speed:
            raise ValueError("Corner speed must be positive and not exceed max speed")
        if self.config.bay_approach_speed <= 0 or self.config.bay_approach_speed > self.config.max_speed:
            raise ValueError("Bay approach speed must be positive and not exceed max speed")
    
    def _create_simulation_data_service(self) -> ISimulationDataService:
        """Create simulation data service for database connectivity."""
        try:
            service = SimulationDataServiceImpl(warehouse_map=self.warehouse_map)
            print(f"[RobotAgent] Database service initialized successfully")
            return service
        except Exception as e:
            print(f"[RobotAgent] Warning: Database service initialization failed: {e}")
            print(f"[RobotAgent] Some features may be limited without database connectivity")
            # Return a mock service for testing
            return self._create_mock_simulation_data_service()
    
    def _create_mock_simulation_data_service(self) -> ISimulationDataService:
        """Create a mock simulation data service for testing without database."""
        # This would be a mock implementation for testing
        # For now, we'll raise an error to indicate database is required
        raise NotImplementedError("Database connectivity is required for lane-based navigation")
    
    def _create_coordinate_system(self) -> ICoordinateSystem:
        """Create coordinate system for lane-based navigation."""
        # For lane-based navigation, we use a simple coordinate system
        # that maps world coordinates to navigation points
        return CoordinateSystemImpl(
            cell_size=0.25,  # Standard cell size for compatibility
            grid_width=int(self.warehouse_map.width * self.warehouse_map.grid_size / 0.25),
            grid_height=int(self.warehouse_map.height * self.warehouse_map.grid_size / 0.25)
        )
    
    def _create_state_holder(self) -> IStateHolder:
        """Create state holder connected to physics."""
        return StateHolderImpl(
            robot_id=self.config.robot_id,
            model=self.physics.model,
            data=self.physics.data,
            physics_engine=self.physics
        )
    
    def _create_path_planner(self) -> IPathPlanner:
        """Create lane-based path planner."""
        return PathPlannerGraphImpl(
            simulation_data_service=self.simulation_data_service
        )
    
    def _create_lane_follower(self) -> ILaneFollower:
        """Create lane follower for navigation execution."""
        return LaneFollowerImpl(
            state_holder=self.state_holder,
            motion_executor=self.motion_executor,
            simulation_data_service=self.simulation_data_service,
            robot_id=self.config.robot_id
        )
    
    def _create_motion_executor(self) -> IMotionExecutor:
        """Create motion executor connected to physics."""
        return MotionExecutorImpl(
            coordinate_system=self.coordinate_system,
            model=self.physics.model,
            data=self.physics.data,
            robot_id=self.config.robot_id,
            physics_engine=self.physics
        )
    
    def _create_task_handler(self) -> ITaskHandler:
        """Create task handler with all dependencies."""
        return TaskHandlerImpl(
            state_holder=self.state_holder,
            path_planner=self.path_planner,
            lane_follower=self.lane_follower,
            motion_executor=self.motion_executor,
            coordinate_system=self.coordinate_system,
            simulation_data_service=self.simulation_data_service,
            robot_id=self.config.robot_id
        )
    
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
        
        # Start physics simulation
        self.physics_manager.start()
        print(f"[RobotAgent] Started physics simulation for {self.config.robot_id}")
        
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
        
        # Stop physics simulation
        self.physics_manager.stop()
        print(f"[RobotAgent] Stopped physics simulation for {self.config.robot_id}")
        
        # Emergency stop motion and lane following
        self.motion_executor.emergency_stop()
        self.lane_follower.emergency_stop()
        
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
    
    def calculate_bids(self, tasks: List[Task]) -> List['RobotBid']:
        """
        Calculate bids for multiple tasks in parallel.
        
        This method uses the robot's bid calculator to evaluate all tasks
        and return bids for tasks the robot can execute.
        
        Args:
            tasks: List of tasks to bid on
            
        Returns:
            List[RobotBid]: Bids for tasks the robot can execute
        """
        from interfaces.bidding_system_interface import RobotBid
        
        bids = []
        robot_status = self.get_status()
        
        # Check if robot is available for bidding
        if not self.is_available_for_bidding():
            return bids
        
        # Get robot's current position and state
        robot_position = robot_status.get('position', (0.0, 0.0))
        robot_battery = robot_status.get('battery', 0.0)
        
        for task in tasks:
            try:
                # Check if robot can execute this task
                if self._can_execute_task(task):
                    # Calculate bid using bid calculator
                    bid_value = self._calculate_task_bid(task, robot_position, robot_battery)
                    
                    bid = RobotBid(
                        robot_id=self.config.robot_id,
                        task=task,
                        bid_value=bid_value,
                        bid_metadata={
                            "robot_position": robot_position,
                            "robot_battery": robot_battery,
                            "task_type": task.task_type.value,
                            "parallel_bidding": True
                        }
                    )
                    bids.append(bid)
                    
            except Exception as e:
                print(f"[RobotAgent] Error calculating bid for task {task.task_id}: {e}")
                continue
        
        return bids
    
    def is_available_for_bidding(self) -> bool:
        """
        Check if robot is available for bidding.
        
        Returns:
            bool: True if robot can participate in bidding
        """
        try:
            status = self.get_status()
            
            # Check if robot has an active task
            if status.get('task_active', True):
                return False
            
            # Check operational status
            operational_status = status.get('operational_status', 'error')
            if operational_status in ['error', 'emergency_stop', 'stalled']:
                return False
            
            # Check battery level
            battery_level = status.get('battery', 0.0)
            if battery_level < 20.0:  # 20% battery threshold
                return False
            
            return True
            
        except Exception as e:
            print(f"[RobotAgent] Error checking availability: {e}")
            return False
    
    def get_bid_calculation_statistics(self) -> Dict[str, Any]:
        """
        Get bid calculation statistics for monitoring.
        
        Returns:
            Dict[str, Any]: Bid calculation performance metrics
        """
        # This would track bid calculation performance over time
        # For now, return basic statistics
        return {
            'robot_id': self.config.robot_id,
            'available_for_bidding': self.is_available_for_bidding(),
            'current_position': self.get_status().get('position', (0.0, 0.0)),
            'battery_level': self.get_status().get('battery', 0.0),
            'operational_status': self.get_status().get('operational_status', 'unknown')
        }
    
    def get_status(self) -> dict:
        """Get comprehensive robot status."""
        task_status = self.task_handler.get_task_status()
        motion_status = self.motion_executor.get_motion_status()
        lane_status = self.lane_follower.get_lane_following_status()
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
            'lane_following_status': lane_status.value,
            'current_target': self.motion_executor.get_current_target(),
            'conflict_boxes_held': self.lane_follower.get_held_conflict_boxes()
        }
    
    # Control loop implementations
    def _control_loop(self) -> None:
        """Control loop running at 10Hz for task management."""
        control_period = 1.0 / self.config.control_frequency
        
        while self._running:
            start_time = time.time()
            
            try:
                # State is now updated by physics thread at 1kHz
                # No need to call update_from_simulation() here
                
                # Update task execution
                self.task_handler.update_task_execution()
                
                # Update lane following
                self.lane_follower.update_lane_following()
                
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
    
    @property
    def lane_follower_interface(self) -> ILaneFollower:
        """Access to lane follower interface."""
        return self.lane_follower
    
    @property
    def simulation_data_service_interface(self) -> ISimulationDataService:
        """Access to simulation data service interface."""
        return self.simulation_data_service 

    def _can_execute_task(self, task: Task) -> bool:
        """
        Check if robot can execute a specific task.
        
        Args:
            task: Task to check
            
        Returns:
            bool: True if robot can execute the task
        """
        # Basic task compatibility check
        if task.task_type == TaskType.PICK_AND_DELIVER:
            # Check if robot can reach the shelf
            return True  # Simplified for now
        elif task.task_type == TaskType.MOVE_TO_POSITION:
            # Check if target position is reachable
            target_pos = task.target_position
            if target_pos:
                x, y = target_pos[0], target_pos[1]
                return self.warehouse_map.is_walkable(x, y)
        elif task.task_type == TaskType.MOVE_TO_CHARGING:
            # Check if robot can reach charging station
            return True  # Simplified for now
        
        return False
    
    def _calculate_task_bid(self, task: Task, robot_position: Tuple[float, float], robot_battery: float) -> float:
        """
        Calculate bid value for a specific task.
        
        Args:
            task: Task to bid on
            robot_position: Current robot position
            robot_battery: Current battery level
            
        Returns:
            float: Bid value (lower is better)
        """
        # Simple bid calculation based on distance and battery
        base_bid = 100.0
        
        # Distance factor
        if task.target_position:
            target_x, target_y = task.target_position[0], task.target_position[1]
            distance = ((target_x - robot_position[0])**2 + (target_y - robot_position[1])**2)**0.5
            distance_factor = distance * 10.0  # 10 units per meter
        else:
            distance_factor = 50.0  # Default distance for tasks without position
        
        # Battery factor (prefer robots with higher battery)
        battery_factor = (100.0 - robot_battery) * 0.5  # 0.5 units per % battery
        
        # Task type factor
        if task.task_type == TaskType.PICK_AND_DELIVER:
            task_factor = 20.0  # Pick and deliver is more complex
        elif task.task_type == TaskType.MOVE_TO_CHARGING:
            task_factor = 10.0  # Charging is simple
        else:
            task_factor = 0.0  # Move to position is simple
        
        total_bid = base_bid + distance_factor + battery_factor + task_factor
        
        return total_bid 