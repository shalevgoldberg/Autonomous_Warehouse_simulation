"""
Unit tests for RobotAgent with lane-based navigation.

Tests the integration of all robot components with the new lane-based architecture,
ensuring SOLID principles, interface-driven design, and thread safety.
"""
import unittest
import threading
import time
from unittest.mock import Mock, MagicMock, patch
from typing import Optional, Tuple

from robot.robot_agent_lane_based import RobotAgent
from interfaces.task_handler_interface import Task, TaskType, ITaskHandler
from interfaces.state_holder_interface import IStateHolder
from interfaces.path_planner_interface import IPathPlanner
from interfaces.lane_follower_interface import ILaneFollower, LaneFollowingStatus
from interfaces.motion_executor_interface import IMotionExecutor, MotionStatus
from interfaces.coordinate_system_interface import ICoordinateSystem
from interfaces.simulation_data_service_interface import ISimulationDataService
from interfaces.configuration_interface import IConfigurationProvider, RobotConfig
from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics


class MockConfigurationProvider(IConfigurationProvider):
    """Mock configuration provider for testing."""
    
    def __init__(self):
        self._errors = []
        self._robot_config = RobotConfig(
            robot_id="test_robot",
            max_speed=1.0,
            position_tolerance=0.1,
            control_frequency=10.0,
            motion_frequency=100.0,
            lane_tolerance=0.1,
            corner_speed=0.3,
            bay_approach_speed=0.2,
            conflict_box_lock_timeout=30.0,
            conflict_box_heartbeat_interval=5.0,
            max_linear_velocity=2.0,
            max_angular_velocity=2.0,
            movement_speed=1.5,
            wheel_base=0.3,
            wheel_radius=0.05,
            picking_duration=5.0,
            dropping_duration=3.0,
            charging_threshold=0.2,
            emergency_stop_distance=0.5,
            stall_recovery_timeout=10.0
        )
    
    def get_robot_config(self, robot_id: str) -> RobotConfig:
        return self._robot_config
    
    def get_database_config(self):
        return Mock(
            host="localhost",
            port=5432,
            database="test_db",
            user="test_user",
            password="test_pass",
            pool_size=5,
            connect_timeout=5,
            application_name="test_app"
        )
    
    def get_navigation_config(self):
        return Mock()
    
    def get_task_config(self):
        return Mock()
    
    def get_system_config(self):
        return Mock()
    
    def get_value(self, key: str, default=None):
        if key == "robot.start_position":
            return Mock(value=None)  # Use warehouse default
        return Mock(value=default)
    
    def set_value(self, key: str, value, source=None):
        pass
    
    def reload(self):
        pass
    
    def validate(self):
        return self._errors
    
    @property
    def errors(self):
        return self._errors
    
    @errors.setter
    def errors(self, value):
        self._errors = value


class TestRobotAgentLaneBased(unittest.TestCase):
    """Test cases for lane-based RobotAgent."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create mock warehouse map
        self.warehouse_map = Mock(spec=WarehouseMap)
        self.warehouse_map.width = 10
        self.warehouse_map.height = 10
        self.warehouse_map.grid_size = 0.5
        self.warehouse_map.start_position = (0.0, 0.0)
        self.warehouse_map.is_walkable.return_value = True
        self.warehouse_map.grid = [[0 for _ in range(10)] for _ in range(10)]
        
        # Create mock physics engine
        self.physics = Mock(spec=SimpleMuJoCoPhysics)
        self.physics.model = None
        self.physics.data = None
        self.physics.reset_robot_position.return_value = None
        self.physics.get_robot_pose.return_value = (0.0, 0.0, 0.0)
        
        # Create mock simulation data service
        self.simulation_data_service = Mock(spec=ISimulationDataService)
        
        # Set up return values for commonly used methods
        self.simulation_data_service.get_navigation_graph.return_value = Mock()
        self.simulation_data_service.get_blocked_cells.return_value = {}
        self.simulation_data_service.try_acquire_conflict_box_lock.return_value = True
        self.simulation_data_service.release_conflict_box_lock.return_value = True
        self.simulation_data_service.heartbeat_conflict_box_lock.return_value = True
        self.simulation_data_service.get_shelf_position.return_value = (1.0, 1.0)
        self.simulation_data_service.get_dropoff_zones.return_value = [(5.0, 5.0)]
        self.simulation_data_service.lock_shelf.return_value = True
        self.simulation_data_service.unlock_shelf.return_value = True
        self.simulation_data_service.log_event.return_value = None
        
        # Set up default return values for all abstract methods
        self.simulation_data_service.get_map_data.return_value = Mock()
        self.simulation_data_service.get_lanes.return_value = []
        self.simulation_data_service.get_conflict_boxes.return_value = []
        self.simulation_data_service.report_blocked_cell.return_value = True
        self.simulation_data_service.clear_blocked_cell.return_value = True
        self.simulation_data_service.cleanup_expired_blocks.return_value = 0
        self.simulation_data_service.get_conflict_box_lock_owner.return_value = None
        self.simulation_data_service.cleanup_expired_conflict_box_locks.return_value = 0
        self.simulation_data_service.is_shelf_locked.return_value = False
        self.simulation_data_service.get_shelf_lock_owner.return_value = None
        self.simulation_data_service.get_shelf_info.return_value = None
        self.simulation_data_service.get_item_location.return_value = None
        self.simulation_data_service.update_inventory.return_value = True
        
        # Create mock configuration provider
        self.config_provider = MockConfigurationProvider()
    
    def test_initialization_with_valid_config(self):
        """Test RobotAgent initialization with valid configuration."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot",
                simulation_data_service=self.simulation_data_service
            )
            
            # Verify all components are created
            self.assertIsNotNone(robot.state_holder)
            self.assertIsNotNone(robot.path_planner)
            self.assertIsNotNone(robot.lane_follower)
            self.assertIsNotNone(robot.motion_executor)
            self.assertIsNotNone(robot.task_handler)
            self.assertIsNotNone(robot.simulation_data_service)
            
            # Verify interfaces are properly exposed
            self.assertIsInstance(robot.task_handler_interface, ITaskHandler)
            self.assertIsInstance(robot.motion_executor_interface, IMotionExecutor)
            self.assertIsInstance(robot.state_holder_interface, IStateHolder)
            self.assertIsInstance(robot.path_planner_interface, IPathPlanner)
            self.assertIsInstance(robot.lane_follower_interface, ILaneFollower)
            self.assertIsInstance(robot.simulation_data_service_interface, ISimulationDataService)
    
    def test_initialization_with_invalid_start_position(self):
        """Test RobotAgent initialization with invalid start position."""
        self.warehouse_map.is_walkable.return_value = False
        
        # Configure provider to return invalid start position
        self.config_provider.get_value = lambda key, default=None: Mock(value=(999.0, 999.0) if key == "robot.start_position" else default)
        
        with self.assertRaises(ValueError) as context:
            RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
        
        self.assertIn("not walkable", str(context.exception))
    
    def test_initialization_with_configuration_errors(self):
        """Test RobotAgent initialization with configuration validation errors."""
        self.config_provider.errors = ["Invalid max_speed: must be positive"]
        
        with self.assertRaises(ValueError) as context:
            RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
        
        self.assertIn("Configuration validation errors", str(context.exception))
    
    def test_initialization_with_external_simulation_data_service(self):
        """Test RobotAgent initialization with external simulation data service."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl'):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot",
                simulation_data_service=self.simulation_data_service
            )
            
            # Verify external service is used
            self.assertEqual(robot.simulation_data_service, self.simulation_data_service)
    
    def test_initialize_position(self):
        """Test robot position initialization."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            robot.initialize_position()
            
            # Verify physics engine was called with correct position
            self.physics.reset_robot_position.assert_called_once_with(0.0, 0.0, 0.0)
    
    def test_initialize_position_with_custom_start(self):
        """Test robot position initialization with custom start position."""
        # Configure provider to return custom start position
        self.config_provider.get_value = lambda key, default=None: Mock(value=(2.0, 3.0) if key == "robot.start_position" else default)
        
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            robot.initialize_position()
            
            # Verify physics engine was called with custom position
            self.physics.reset_robot_position.assert_called_once_with(2.0, 3.0, 0.0)
    
    def test_start_and_stop_control_loops(self):
        """Test starting and stopping control loops."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            # Mock component methods
            robot.task_handler.update_task_execution = Mock()
            robot.motion_executor.update_control_loop = Mock()
            robot.motion_executor.stop_execution = Mock()
            
            # Start control loops
            robot.start()
            
            # Verify threads are running
            self.assertTrue(robot._running)
            self.assertIsNotNone(robot._control_thread)
            self.assertIsNotNone(robot._motion_thread)
            if robot._control_thread is not None:
                self.assertTrue(robot._control_thread.is_alive())
            if robot._motion_thread is not None:
                self.assertTrue(robot._motion_thread.is_alive())
            
            # Wait a bit for loops to run
            time.sleep(0.1)
            
            # Verify component methods were called
            robot.task_handler.update_task_execution.assert_called()
            robot.motion_executor.update_control_loop.assert_called()
            
            # Stop control loops
            robot.stop()
            
            # Verify threads are stopped
            self.assertFalse(robot._running)
            robot.motion_executor.stop_execution.assert_called_once()
    
    def test_assign_task(self):
        """Test task assignment."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            # Mock task handler
            robot.task_handler.start_task = Mock(return_value=True)
            
            # Create test task
            task = Task(
                task_id="test_task",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id="order_001",  # Required for PICK_AND_DELIVER tasks
                shelf_id="shelf_1",
                item_id="item_1",
                priority=1
            )
            
            # Assign task
            result = robot.assign_task(task)
            
            # Verify task handler was called
            robot.task_handler.start_task.assert_called_once_with(task)
            self.assertTrue(result)
    
    def test_get_status(self):
        """Test status reporting."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            # Mock component status methods
            robot.task_handler.get_task_status = Mock()
            robot.motion_executor.get_motion_status = Mock(return_value=MotionStatus.IDLE)
            robot.lane_follower.get_lane_following_status = Mock(return_value=LaneFollowingStatus.IDLE)
            robot.state_holder.get_position = Mock(return_value=(0.0, 0.0, 0.0))
            robot.state_holder.get_battery_level = Mock(return_value=1.0)
            robot.task_handler.get_current_task = Mock(return_value=None)
            
            # Mock task status
            mock_task_status = Mock()
            mock_task_status.has_active_task = False
            mock_task_status.task_id = None
            mock_task_status.operational_status.value = "IDLE"
            mock_task_status.progress = 0.0
            robot.task_handler.get_task_status.return_value = mock_task_status
            
            # Get status
            status = robot.get_status()
            
            # Verify status structure
            self.assertIn('robot_id', status)
            self.assertIn('running', status)
            self.assertIn('position', status)
            self.assertIn('battery_level', status)  # Changed from 'battery'
            self.assertIn('current_task', status)
            self.assertIn('task_status', status)
            self.assertIn('motion_status', status)
            self.assertIn('lane_following_status', status)
            self.assertIn('configuration_errors', status)
            
            # Verify values
            self.assertEqual(status['robot_id'], 'test_robot')
            self.assertEqual(status['position'], (0.0, 0.0, 0.0))
            self.assertEqual(status['battery_level'], 1.0)
            self.assertFalse(status['running'])
            self.assertEqual(status['motion_status'], MotionStatus.IDLE)
            self.assertEqual(status['lane_following_status'], LaneFollowingStatus.IDLE)
    
    def test_create_task_helpers(self):
        """Test task creation helper methods."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            # Test pickup task creation
            pickup_task = robot.create_pickup_task("shelf_1", "item_1")
            self.assertEqual(pickup_task.task_type, TaskType.PICK_AND_DELIVER)
            self.assertEqual(pickup_task.shelf_id, "shelf_1")
            self.assertEqual(pickup_task.item_id, "item_1")
            self.assertEqual(pickup_task.priority, 0)  # Default priority is 0
            
            # Test move task creation
            move_task = robot.create_move_task(5.0, 3.0)
            self.assertEqual(move_task.task_type, TaskType.MOVE_TO_POSITION)
            self.assertEqual(move_task.target_position, (5.0, 3.0, 0.0))
            self.assertEqual(move_task.priority, 0)  # Default priority is 0
            
            # Test charging task creation
            charging_task = robot.create_charging_task()
            self.assertEqual(charging_task.task_type, TaskType.MOVE_TO_CHARGING)
            self.assertEqual(charging_task.priority, 0)  # Default priority is 0
    
    def test_interface_access_properties(self):
        """Test interface access properties."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            # Verify all interfaces are accessible
            self.assertIsInstance(robot.task_handler_interface, ITaskHandler)
            self.assertIsInstance(robot.motion_executor_interface, IMotionExecutor)
            self.assertIsInstance(robot.state_holder_interface, IStateHolder)
            self.assertIsInstance(robot.path_planner_interface, IPathPlanner)
            self.assertIsInstance(robot.lane_follower_interface, ILaneFollower)
            self.assertIsInstance(robot.simulation_data_service_interface, ISimulationDataService)
            
            # Verify interfaces point to correct components
            self.assertEqual(robot.task_handler_interface, robot.task_handler)
            self.assertEqual(robot.motion_executor_interface, robot.motion_executor)
            self.assertEqual(robot.state_holder_interface, robot.state_holder)
            self.assertEqual(robot.path_planner_interface, robot.path_planner)
            self.assertEqual(robot.lane_follower_interface, robot.lane_follower)
            self.assertEqual(robot.simulation_data_service_interface, robot.simulation_data_service)
    
    def test_thread_safety(self):
        """Test thread safety of RobotAgent."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            # Mock component methods
            robot.task_handler.update_task_execution = Mock()
            robot.motion_executor.update_control_loop = Mock()
            robot.motion_executor.stop_execution = Mock()
            
            # Start control loops
            robot.start()
            
            # Create multiple threads accessing robot status
            results = []
            errors = []
            
            def access_robot_status():
                try:
                    for _ in range(10):
                        status = robot.get_status()
                        results.append(status['robot_id'])
                        time.sleep(0.01)
                except Exception as e:
                    errors.append(str(e))
            
            # Create multiple threads
            threads = []
            for i in range(5):
                thread = threading.Thread(target=access_robot_status, name=f"StatusThread-{i}")
                threads.append(thread)
                thread.start()
            
            # Wait for threads to complete
            for thread in threads:
                thread.join(timeout=1.0)
            
            # Stop robot
            robot.stop()
            
            # Verify no errors occurred
            self.assertEqual(len(errors), 0, f"Thread safety errors: {errors}")
            self.assertEqual(len(results), 50)  # 5 threads * 10 calls each
            self.assertTrue(all(r == 'test_robot' for r in results))
    
    def test_error_handling_in_control_loops(self):
        """Test error handling in control loops - resilient approach."""
        with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=self.simulation_data_service):
            robot = RobotAgent(
                warehouse_map=self.warehouse_map,
                physics=self.physics,
                config_provider=self.config_provider,
                robot_id="test_robot"
            )
            
            # Mock component methods to simulate intermittent errors
            call_count = 0
            
            def task_error_after_calls(*args, **kwargs):
                nonlocal call_count
                call_count += 1
                if call_count > 5:  # Allow some calls to succeed first
                    raise Exception("Task error")
            
            def motion_error_after_calls(*args, **kwargs):
                nonlocal call_count
                call_count += 1
                if call_count > 3:  # Allow some calls to succeed first
                    raise Exception("Motion error")
            
            robot.task_handler.update_task_execution = Mock(side_effect=task_error_after_calls)
            robot.motion_executor.update_control_loop = Mock(side_effect=motion_error_after_calls)
            
            # Start control loops
            robot.start()
            
            # Wait for some iterations
            time.sleep(0.2)
            
            # Stop robot
            robot.stop()
            
            # Verify that methods were called (resilient error handling)
            self.assertGreater(robot.task_handler.update_task_execution.call_count, 0)
            self.assertGreater(robot.motion_executor.update_control_loop.call_count, 0)


if __name__ == '__main__':
    unittest.main() 