"""
Robot Agent - Complete robot integration with MuJoCo physics simulation.

This implementation connects all robot components (TaskHandler, PathPlanner, 
LaneFollower, MotionExecutor, StateHolder) with the MuJoCo physics engine following our
architecture principles: loose coupling, interface-driven design, and SOLID principles.

Updated for lane-based navigation with conflict box management and centralized configuration.
"""
import threading
import time
from typing import Optional, Set, Tuple
from dataclasses import dataclass

from interfaces.task_handler_interface import ITaskHandler, Task, TaskType
from interfaces.path_planner_interface import IPathPlanner
from interfaces.motion_executor_interface import IMotionExecutor
from interfaces.state_holder_interface import IStateHolder
from interfaces.coordinate_system_interface import ICoordinateSystem
from interfaces.lane_follower_interface import ILaneFollower
from interfaces.simulation_data_service_interface import ISimulationDataService
from interfaces.configuration_interface import IConfigurationProvider

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


class RobotAgent:
    """
    Complete robot agent integrating all components with MuJoCo physics.
    
    Architecture:
    - Interface-driven: All components accessed via interfaces
    - Loose coupling: Components don't know about each other directly
    - Thread-safe: Proper synchronization for multi-threaded operation
    - SOLID principles: Single responsibility, dependency injection
    - Lane-based navigation: Uses NavigationGraph and conflict box management
    - Centralized configuration: Uses ConfigurationProvider for all parameters
    
    Threading Model:
    - Control Thread: 10Hz task management
    - Motion Thread: 100Hz motion control  
    - Physics Thread: 1000Hz physics simulation (external)
    """
    
    def __init__(self, 
                 warehouse_map: WarehouseMap,
                 physics: SimpleMuJoCoPhysics,
                 config_provider: IConfigurationProvider,
                 robot_id: str = "robot_1",
                 simulation_data_service: Optional[ISimulationDataService] = None):
        """
        Initialize robot agent with dependency injection.
        
        Args:
            warehouse_map: Warehouse layout and obstacles
            physics: MuJoCo physics simulation
            config_provider: Configuration provider for all parameters
            robot_id: Robot identifier
            simulation_data_service: Database service (created if not provided)
        """
        self.config_provider = config_provider
        self.robot_id = robot_id
        self.warehouse_map = warehouse_map
        self.physics = physics
        
        # Get robot configuration from provider
        self.robot_config = self.config_provider.get_robot_config(robot_id)
        
        # Validate configuration
        self._validate_configuration()
        
        # Initialize simulation data service (database connectivity)
        self.simulation_data_service = simulation_data_service or self._create_simulation_data_service()
        
        # Initialize coordinate system (simplified for lane-based navigation)
        self.coordinate_system = self._create_coordinate_system()
        
        # Initialize robot components via dependency injection
        # Order matters: motion_executor must be created before lane_follower
        self.state_holder = self._create_state_holder()
        self.path_planner = self._create_path_planner()
        self.motion_executor = self._create_motion_executor()
        self.lane_follower = self._create_lane_follower()
        self.task_handler = self._create_task_handler()
        
        # Control loop management
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._motion_thread: Optional[threading.Thread] = None
        
        print(f"[RobotAgent] Initialized {self.robot_id} with lane-based navigation")
        print(f"[RobotAgent] Components: PathPlanner, LaneFollower, MotionExecutor, TaskHandler")
        print(f"[RobotAgent] Database: {type(self.simulation_data_service).__name__}")
        print(f"[RobotAgent] Configuration: {len(self.config_provider.errors)} validation errors")
    
    def _validate_configuration(self) -> None:
        """Validate robot configuration against warehouse constraints."""
        # Check for configuration validation errors
        config_errors = self.config_provider.errors
        if config_errors:
            error_msg = f"Configuration validation errors: {', '.join(config_errors)}"
            raise ValueError(error_msg)
        
        # Validate start position if provided (could be moved to config provider)
        start_pos = self.config_provider.get_value("robot.start_position").value
        if start_pos is not None:
            x, y = start_pos
            if not self.warehouse_map.is_walkable(x, y):
                raise ValueError(f"Configured start position ({x}, {y}) is not walkable in warehouse")
    
    def _create_simulation_data_service(self) -> ISimulationDataService:
        """Create simulation data service for database connectivity."""
        try:
            # Get database configuration from provider
            db_config = self.config_provider.get_database_config()
            service = SimulationDataServiceImpl(
                warehouse_map=self.warehouse_map,
                db_host=db_config.host,
                db_port=db_config.port,
                db_name=db_config.database,
                db_user=db_config.user,
                db_password=db_config.password,
                pool_size=db_config.pool_size
            )
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
            robot_id=self.robot_id,
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
        lane_follower = LaneFollowerImpl(
            state_holder=self.state_holder,
            motion_executor=self.motion_executor,
            simulation_data_service=self.simulation_data_service,
            robot_id=self.robot_id,
            config_provider=self.config_provider
        )
        
        # Update lane follower configuration with robot-specific parameters
        lane_follower.update_config_from_robot(self.robot_config)
        
        return lane_follower
    
    def _create_motion_executor(self) -> IMotionExecutor:
        """Create motion executor connected to physics."""
        motion_executor = MotionExecutorImpl(
            coordinate_system=self.coordinate_system,
            model=self.physics.model,
            data=self.physics.data,
            robot_id=self.robot_id,
            physics_engine=self.physics,
            config_provider=self.config_provider
        )
        
        # Update motion executor configuration with robot-specific parameters
        motion_executor.update_config_from_robot(self.robot_config)
        
        return motion_executor
    
    def _create_task_handler(self) -> ITaskHandler:
        """Create task handler for task execution."""
        task_handler = TaskHandlerImpl(
            robot_id=self.robot_id,
            state_holder=self.state_holder,
            path_planner=self.path_planner,
            lane_follower=self.lane_follower,
            motion_executor=self.motion_executor,
            coordinate_system=self.coordinate_system,
            simulation_data_service=self.simulation_data_service,
            config_provider=self.config_provider
        )
        
        # Update task handler configuration with robot-specific parameters
        task_handler.update_config_from_robot(self.robot_config)
        
        return task_handler
    
    def initialize_position(self) -> None:
        """Initialize robot position in physics simulation."""
        # Get start position from configuration or warehouse default
        start_pos = self.config_provider.get_value("robot.start_position").value
        if start_pos is not None:
            x, y = start_pos
        else:
            x, y = self.warehouse_map.start_position
        
        self.physics.reset_robot_position(x, y, 0.0)
        print(f"[RobotAgent] Initialized position at ({x}, {y})")
    
    def start(self) -> None:
        """Start robot control and motion loops."""
        if self._running:
            print(f"[RobotAgent] Already running")
            return
        
        self._running = True
        
        # Start control thread
        self._control_thread = threading.Thread(
            target=self._control_loop,
            name=f"RobotControl-{self.robot_id}",
            daemon=True
        )
        self._control_thread.start()
        
        # Start motion thread
        self._motion_thread = threading.Thread(
            target=self._motion_loop,
            name=f"RobotMotion-{self.robot_id}",
            daemon=True
        )
        self._motion_thread.start()
        
        print(f"[RobotAgent] Started control and motion threads")
    
    def stop(self) -> None:
        """Stop robot control and motion loops."""
        if not self._running:
            print(f"[RobotAgent] Not running")
            return
        
        self._running = False
        
        # Stop motion execution
        self.motion_executor.stop_execution()
        
        # Wait for threads to finish
        if self._control_thread:
            self._control_thread.join(timeout=1.0)
        if self._motion_thread:
            self._motion_thread.join(timeout=1.0)
        
        print(f"[RobotAgent] Stopped control and motion threads")
    
    def assign_task(self, task: Task) -> bool:
        """Assign a task to the robot."""
        return self.task_handler.start_task(task)
    
    def get_status(self) -> dict:
        """Get comprehensive robot status."""
        return {
            'robot_id': self.robot_id,
            'running': self._running,
            'position': self.state_holder.get_position(),
            'battery_level': self.state_holder.get_battery_level(),
            'current_task': self.task_handler.get_current_task(),
            'task_status': self.task_handler.get_task_status(),
            'motion_status': self.motion_executor.get_motion_status(),
            'lane_following_status': self.lane_follower.get_lane_following_status(),
            'configuration_errors': self.config_provider.errors
        }
    
    def _control_loop(self) -> None:
        """Control loop for task management."""
        control_period = 1.0 / self.robot_config.control_frequency
        
        while self._running:
            try:
                # Update task handler
                self.task_handler.update_task_execution()
                
                # Sleep for control period
                time.sleep(control_period)
                
            except Exception as e:
                print(f"[RobotAgent] Control loop error: {e}")
                time.sleep(control_period)
    
    def _motion_loop(self) -> None:
        """Motion loop for physics integration."""
        motion_period = 1.0 / self.robot_config.motion_frequency
        
        while self._running:
            try:
                # Update motion executor
                self.motion_executor.update_control_loop()
                
                # Sleep for motion period
                time.sleep(motion_period)
                
            except Exception as e:
                print(f"[RobotAgent] Motion loop error: {e}")
                time.sleep(motion_period)
    
    def create_pickup_task(self, shelf_id: str, item_id: str = "item_1") -> Task:
        """Create a pickup task."""
        return Task(
            task_id=f"pickup_{shelf_id}_{item_id}",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id=f"order_{shelf_id}_{item_id}",
            shelf_id=shelf_id,
            item_id=item_id
        )
    
    def create_move_task(self, target_x: float, target_y: float) -> Task:
        """Create a move task."""
        return Task(
            task_id=f"move_{target_x}_{target_y}",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(target_x, target_y, 0.0)
        )
    
    def create_charging_task(self) -> Task:
        """Create a charging task."""
        return Task(
            task_id="charging",
            task_type=TaskType.MOVE_TO_CHARGING,
            bay_id="charging_bay_1"
        )
    
    # Interface properties for external access
    @property
    def task_handler_interface(self) -> ITaskHandler:
        return self.task_handler
    
    @property  
    def motion_executor_interface(self) -> IMotionExecutor:
        return self.motion_executor
    
    @property
    def state_holder_interface(self) -> IStateHolder:
        return self.state_holder
    
    @property
    def path_planner_interface(self) -> IPathPlanner:
        return self.path_planner
    
    @property
    def lane_follower_interface(self) -> ILaneFollower:
        return self.lane_follower
    
    @property
    def simulation_data_service_interface(self) -> ISimulationDataService:
        return self.simulation_data_service 