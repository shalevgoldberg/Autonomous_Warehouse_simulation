"""
Robot Agent - Complete robot integration with MuJoCo physics simulation.

This implementation connects all robot components (TaskHandler, PathPlanner, 
LaneFollower, MotionExecutor, StateHolder) with the MuJoCo physics engine following our
architecture principles: loose coupling, interface-driven design, and SOLID principles.

Updated for lane-based navigation with conflict box management and centralized configuration.
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
from interfaces.configuration_interface import IConfigurationProvider, BidConfig
from interfaces.bid_calculator_interface import IBidCalculator
from interfaces.bidding_system_interface import RobotBid

from robot.impl.task_handler_impl import TaskHandlerImpl
from robot.impl.path_planner_graph_impl import PathPlannerGraphImpl
from robot.impl.lane_follower_impl import LaneFollowerImpl
from robot.impl.motion_executor_impl import MotionExecutorImpl
from robot.impl.state_holder_impl import StateHolderImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from robot.impl.physics_integration import create_command_adapter, create_state_adapter
from robot.impl.bid_calculator_impl import BidCalculatorImpl
from simulation.simulation_data_service_impl import SimulationDataServiceImpl

from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from simulation.shared_mujoco_engine import SharedMuJoCoPhysics
from interfaces.appearance_service_interface import IAppearanceService
from utils.geometry import mapdata_to_warehouse_map


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
    - Parallel bidding: Supports asynchronous bid calculation for task assignment
    
    Threading Model:
    - Control Thread: 10Hz task management
    - Motion Thread: 100Hz motion control  
    - Physics Thread: 1000Hz physics simulation (external)
    - Bidding Thread: On-demand bid calculation (parallel)
    """
    
    def __init__(self, 
                 physics: SimpleMuJoCoPhysics,
                 config_provider: IConfigurationProvider,
                 robot_id: str = "robot_1",
                 simulation_data_service: Optional[ISimulationDataService] = None):
        """
        Initialize robot agent with dependency injection.
        
        Args:
            physics: MuJoCo physics simulation
            config_provider: Configuration provider for all parameters
            robot_id: Robot identifier
            simulation_data_service: Database service (created if not provided)
        """
        self.config_provider = config_provider
        self.robot_id = robot_id
        self.physics = physics
        
        # Get robot configuration from provider
        self.robot_config = self.config_provider.get_robot_config(robot_id)
        
        # Initialize simulation data service (database connectivity)
        self.simulation_data_service = simulation_data_service or self._create_simulation_data_service()
        # Get warehouse map from data service
        map_data = self.simulation_data_service.get_map_data()
        self.warehouse_map = mapdata_to_warehouse_map(map_data)
        # Validate configuration (must be after warehouse_map is set)
        self._validate_configuration()
        
        # Initialize coordinate system (simplified for lane-based navigation)
        self.coordinate_system = self._create_coordinate_system()
        
        # Initialize robot components via dependency injection
        # Order matters: motion_executor must be created before lane_follower
        self.state_holder = self._create_state_holder()
        self.path_planner = self._create_path_planner()
        self.motion_executor = self._create_motion_executor()
        self.lane_follower = self._create_lane_follower()
        self.task_handler = self._create_task_handler()
        
        # Initialize bid calculator for parallel bidding
        self.bid_calculator = self._create_bid_calculator()
        
        # Initialize physics thread manager with state holder for 1kHz synchronization
        from robot.impl.physics_integration import create_physics_thread_manager
        self.physics_manager = create_physics_thread_manager(
            physics=self.physics,
            state_holder=self.state_holder,
            frequency_hz=1000.0,
            db_sync_frequency_hz=10.0
        )

        # Store reference to state_holder in physics for direct updates
        print(f"[RobotAgent] DIAGNOSTIC: Setting _state_holder on physics object")
        print(f"[RobotAgent] DIAGNOSTIC: physics type: {type(self.physics)}")
        print(f"[RobotAgent] DIAGNOSTIC: physics has _state_holder before: {hasattr(self.physics, '_state_holder')}")

        # Set the state holder reference for direct updates
        self.physics._state_holder = self.state_holder

        print(f"[RobotAgent] DIAGNOSTIC: physics has _state_holder after: {hasattr(self.physics, '_state_holder')}")
        print(f"[RobotAgent] DIAGNOSTIC: _state_holder is not None: {self.physics._state_holder is not None}")
        
        # Control loop management
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._motion_thread: Optional[threading.Thread] = None
        
        print(f"[RobotAgent] Initialized {self.robot_id} with lane-based navigation")
        print(f"[RobotAgent] Components: PathPlanner, LaneFollower, MotionExecutor, TaskHandler, BidCalculator")
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
        # Use cell size from configuration to match warehouse map
        cell_size = self.robot_config.cell_size
        return CoordinateSystemImpl(
            cell_size=cell_size,  # Use configured cell size (default 0.5)
            grid_width=self.warehouse_map.width,   # Use actual warehouse dimensions
            grid_height=self.warehouse_map.height  # Use actual warehouse dimensions
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
        # Get appearance service if available (for visual shelf carrying)
        appearance_service = self._get_appearance_service()

        task_handler = TaskHandlerImpl(
            robot_id=self.robot_id,
            state_holder=self.state_holder,
            path_planner=self.path_planner,
            lane_follower=self.lane_follower,
            motion_executor=self.motion_executor,
            coordinate_system=self.coordinate_system,
            simulation_data_service=self.simulation_data_service,
            config_provider=self.config_provider,
            appearance_service=appearance_service
        )
        
        # Update task handler configuration with robot-specific parameters
        task_handler.update_config_from_robot(self.robot_config)
        
        return task_handler

    def _get_appearance_service(self) -> Optional[IAppearanceService]:
        """Get appearance service from physics engine if available."""
        # Check if physics is SharedMuJoCoPhysics which has appearance service
        if isinstance(self.physics, SharedMuJoCoPhysics):
            try:
                appearance_service = self.physics._engine.get_appearance_service()
                if appearance_service is not None:
                    print(f"[RobotAgent] Successfully obtained appearance service for robot {self.robot_id}")
                    return appearance_service
                else:
                    print(f"[RobotAgent] WARNING: SharedMuJoCoEngine returned None appearance service for robot {self.robot_id}")
                    return None
            except Exception as e:
                print(f"[RobotAgent] ERROR: Failed to get appearance service for robot {self.robot_id}: {e}")
                return None
        else:
            print(f"[RobotAgent] WARNING: Robot {self.robot_id} is not using SharedMuJoCoPhysics - appearance service not available")
            return None

    def _create_bid_calculator(self) -> IBidCalculator:
        """Create bid calculator for parallel bidding."""
        # Get bid configuration from provider
        bid_config = self.config_provider.get_bid_config()
        
        bid_calculator = BidCalculatorImpl(
            robot_id=self.robot_id,
            simulation_data_service=self.simulation_data_service,
            max_workers=bid_config.max_parallel_workers
        )
        
        # Configure bid calculator with robot-specific settings
        self._configure_bid_calculator(bid_calculator, bid_config)
        
        return bid_calculator
    
    def _configure_bid_calculator(self, bid_calculator: IBidCalculator, bid_config: BidConfig) -> None:
        """Configure bid calculator with robot-specific settings."""
        from interfaces.bid_calculator_interface import BidFactor, BidFactorWeight
        
        # Set factor weights from configuration
        weights = []
        
        if bid_config.enable_distance_factor:
            weights.append(BidFactorWeight(BidFactor.DISTANCE, bid_config.distance_weight))
        
        if bid_config.enable_battery_factor:
            weights.append(BidFactorWeight(BidFactor.BATTERY_LEVEL, bid_config.battery_weight))
        
        if bid_config.enable_workload_factor:
            weights.append(BidFactorWeight(BidFactor.WORKLOAD, bid_config.workload_weight))
        
        if bid_config.enable_task_type_compatibility_factor:
            weights.append(BidFactorWeight(BidFactor.TASK_TYPE_COMPATIBILITY, bid_config.task_type_compatibility_weight))
        
        if bid_config.enable_robot_capabilities_factor:
            weights.append(BidFactorWeight(BidFactor.ROBOT_CAPABILITIES, bid_config.robot_capabilities_weight))
        
        if bid_config.enable_time_urgency_factor:
            weights.append(BidFactorWeight(BidFactor.TIME_URGENCY, bid_config.time_urgency_weight))
        
        if bid_config.enable_conflict_box_availability_factor:
            weights.append(BidFactorWeight(BidFactor.CONFLICT_BOX_AVAILABILITY, bid_config.conflict_box_availability_weight))
        
        if bid_config.enable_shelf_accessibility_factor:
            weights.append(BidFactorWeight(BidFactor.SHELF_ACCESSIBILITY, bid_config.shelf_accessibility_weight))
        
        # Apply weights to calculator
        if weights:
            bid_calculator.set_bid_factor_weights(weights)
        
        # Disable factors that are not enabled
        if not bid_config.enable_distance_factor:
            bid_calculator.enable_factor(BidFactor.DISTANCE, False)
        
        if not bid_config.enable_battery_factor:
            bid_calculator.enable_factor(BidFactor.BATTERY_LEVEL, False)
        
        if not bid_config.enable_workload_factor:
            bid_calculator.enable_factor(BidFactor.WORKLOAD, False)
        
        if not bid_config.enable_task_type_compatibility_factor:
            bid_calculator.enable_factor(BidFactor.TASK_TYPE_COMPATIBILITY, False)
        
        if not bid_config.enable_robot_capabilities_factor:
            bid_calculator.enable_factor(BidFactor.ROBOT_CAPABILITIES, False)
        
        if not bid_config.enable_time_urgency_factor:
            bid_calculator.enable_factor(BidFactor.TIME_URGENCY, False)
        
        if not bid_config.enable_conflict_box_availability_factor:
            bid_calculator.enable_factor(BidFactor.CONFLICT_BOX_AVAILABILITY, False)
        
        if not bid_config.enable_shelf_accessibility_factor:
            bid_calculator.enable_factor(BidFactor.SHELF_ACCESSIBILITY, False)
    
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
        
        # Start physics simulation
        self.physics_manager.start()
        print(f"[RobotAgent] Started physics simulation for {self.robot_id}")
        
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

        # CRITICAL: Clean up TaskHandler resources BEFORE stopping dependent components
        # This ensures proper release of bay locks and other resources held by active tasks
        # The TaskHandler.emergency_stop() method is thread-safe and handles complete cleanup
        try:
            self.task_handler.emergency_stop()
            print(f"[RobotAgent] TaskHandler cleanup completed for {self.robot_id}")
        except Exception as e:
            print(f"[RobotAgent] Warning: TaskHandler cleanup failed for {self.robot_id}: {e}")
            # Continue with shutdown even if cleanup fails

        # Stop physics simulation
        self.physics_manager.stop()
        print(f"[RobotAgent] Stopped physics simulation for {self.robot_id}")

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
        try:
            accepted = self.task_handler.start_task(task)
            print(f"[RobotAgent] assign_task robot={self.robot_id} task={task.task_id} accepted={accepted}")
            return accepted
        except Exception as e:
            print(f"[RobotAgent] assign_task error robot={self.robot_id} task={getattr(task,'task_id',None)}: {e}")
            return False
    
    # Bid calculation methods for parallel bidding
    
    def calculate_bids(self, tasks: List[Task]) -> List[RobotBid]:
        """
        Calculate bids for multiple tasks in parallel.
        
        This method is called by the controller to get bids from this robot
        for available tasks. It runs bid calculation in parallel threads.
        
        Args:
            tasks: List of tasks to calculate bids for
            
        Returns:
            List[RobotBid]: Bids for all tasks (may be empty if robot unavailable)
        """
        try:
            return self.bid_calculator.calculate_bids(tasks, self.state_holder)
        except Exception as e:
            print(f"[RobotAgent] Error calculating bids: {e}")
            return []
    
    def calculate_single_bid(self, task: Task) -> Optional[RobotBid]:
        """
        Calculate bid for a single task.
        
        Args:
            task: Task to calculate bid for
            
        Returns:
            Optional[RobotBid]: Bid for the task, or None if robot cannot bid
        """
        try:
            return self.bid_calculator.calculate_single_bid(task, self.state_holder)
        except Exception as e:
            print(f"[RobotAgent] Error calculating bid for task {task.task_id}: {e}")
            return None
    
    def is_available_for_bidding(self) -> bool:
        """
        Check if robot is available for bidding.
        
        Returns:
            bool: True if robot can participate in bidding
        """
        try:
            # Do not bid while executing a task
            if hasattr(self, 'task_handler') and self.task_handler is not None:
                if not self.task_handler.is_idle():
                    return False
            
            return self.bid_calculator.is_available_for_bidding(self.state_holder)
        except Exception as e:
            print(f"[RobotAgent] Error checking bidding availability: {e}")
            return False
    
    def get_bid_calculation_statistics(self) -> Dict[str, Any]:
        """
        Get bid calculation statistics.
        
        Returns:
            Dict[str, Any]: Statistics including calculation times, success rates, etc.
        """
        try:
            return self.bid_calculator.get_calculation_statistics()
        except Exception as e:
            print(f"[RobotAgent] Error getting bid statistics: {e}")
            return {}
    
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
            'configuration_errors': self.config_provider.errors,
            'bid_calculation_stats': self.get_bid_calculation_statistics()
        }
    
    def _control_loop(self) -> None:
        """Control loop for task management."""
        control_period = 1.0 / self.robot_config.control_frequency
        try:
            print(f"[RobotAgent] Control loop started for {self.robot_id}")
        except Exception:
            pass
        
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
    
    @property
    def bid_calculator_interface(self) -> IBidCalculator:
        return self.bid_calculator 