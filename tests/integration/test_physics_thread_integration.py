import time
import threading
import pytest
from unittest.mock import Mock

from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent as LaneBasedRobotAgent
from robot.robot_agent import RobotAgent as RegularRobotAgent
from config.configuration_provider import ConfigurationProvider
from interfaces.task_handler_interface import Task, TaskType

class TestPhysicsThreadIntegration:
    """Integration tests for PhysicsThreadManager in robot agents."""
    
    def setup_method(self):
        """Set up test environment."""
        self.warehouse_map = WarehouseMap(width=10, height=10)
        self.physics = SimpleMuJoCoPhysics(self.warehouse_map)
        
        # Create configuration provider
        self.config_provider = ConfigurationProvider()
        
        # Mock simulation data service to avoid database dependencies
        self.mock_simulation_service = Mock()
        self.mock_simulation_service.get_map_data.return_value = self.warehouse_map
        self.mock_simulation_service.get_inventory_statistics.return_value = {
            'total_shelves': 10,
            'total_items': 5
        }
    
    def test_lane_based_robot_physics_thread_management(self):
        """Test that lane-based robot agent properly manages physics thread."""
        # Create robot agent
        robot = LaneBasedRobotAgent(
            warehouse_map=self.warehouse_map,
            physics=self.physics,
            config_provider=self.config_provider,
            robot_id="test_robot",
            simulation_data_service=self.mock_simulation_service
        )
        
        # Verify physics manager is created
        assert hasattr(robot, 'physics_manager')
        assert robot.physics_manager is not None
        
        # Test start/stop
        assert not robot.physics_manager.is_running()
        
        robot.start()
        time.sleep(0.1)  # Let physics start
        assert robot.physics_manager.is_running()
        
        robot.stop()
        time.sleep(0.1)  # Let physics stop
        assert not robot.physics_manager.is_running()
    
    def test_regular_robot_physics_thread_management(self):
        """Test that regular robot agent properly manages physics thread."""
        from robot.robot_agent import RobotConfiguration
        
        config = RobotConfiguration(robot_id="test_robot")
        
        # Create robot agent
        robot = RegularRobotAgent(
            warehouse_map=self.warehouse_map,
            physics=self.physics,
            config=config,
            simulation_data_service=self.mock_simulation_service
        )
        
        # Verify physics manager is created
        assert hasattr(robot, 'physics_manager')
        assert robot.physics_manager is not None
        
        # Test start/stop
        assert not robot.physics_manager.is_running()
        
        robot.start()
        time.sleep(0.1)  # Let physics start
        assert robot.physics_manager.is_running()
        
        robot.stop()
        time.sleep(0.1)  # Let physics stop
        assert not robot.physics_manager.is_running()
    
    def test_physics_thread_frequency(self):
        """Test that physics thread runs at correct frequency."""
        robot = LaneBasedRobotAgent(
            warehouse_map=self.warehouse_map,
            physics=self.physics,
            config_provider=self.config_provider,
            robot_id="test_robot",
            simulation_data_service=self.mock_simulation_service
        )
        
        # Get initial position
        initial_pos = self.physics.get_robot_pose()
        
        # Start physics
        robot.start()
        time.sleep(0.2)  # Let physics run for 200ms
        
        # Get final position
        final_pos = self.physics.get_physics_state()
        
        # Stop physics
        robot.stop()
        
        # Physics should have updated (even if robot is stationary, timestamp should change)
        assert final_pos.timestamp > initial_pos[2]  # timestamp comparison
        assert robot.physics_manager.is_running() == False
    
    def test_physics_thread_error_handling(self):
        """Test that physics thread handles errors gracefully."""
        # Create a physics engine that will throw errors
        class ErrorPhysics(SimpleMuJoCoPhysics):
            def step_physics(self):
                raise RuntimeError("Test physics error")
        
        error_physics = ErrorPhysics(self.warehouse_map)
        
        robot = LaneBasedRobotAgent(
            warehouse_map=self.warehouse_map,
            physics=error_physics,
            config_provider=self.config_provider,
            robot_id="test_robot",
            simulation_data_service=self.mock_simulation_service
        )
        
        # Should not crash when starting with error-prone physics
        robot.start()
        time.sleep(0.1)
        
        # Should still be able to stop
        robot.stop()
        assert not robot.physics_manager.is_running()
    
    def test_physics_thread_concurrent_access(self):
        """Test that physics thread works with concurrent robot operations."""
        robot = LaneBasedRobotAgent(
            warehouse_map=self.warehouse_map,
            physics=self.physics,
            config_provider=self.config_provider,
            robot_id="test_robot",
            simulation_data_service=self.mock_simulation_service
        )
        
        robot.start()
        time.sleep(0.1)
        
        # Create a task while physics is running
        task = Task(
            task_id="test_task",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(5.0, 5.0, 0.0)
        )
        
        # Should be able to assign task while physics is running
        success = robot.assign_task(task)
        # Note: success may be False if robot is busy, but it shouldn't crash
        
        robot.stop()
        assert not robot.physics_manager.is_running()
    
    def test_physics_thread_cleanup(self):
        """Test that physics thread is properly cleaned up."""
        robot = LaneBasedRobotAgent(
            warehouse_map=self.warehouse_map,
            physics=self.physics,
            config_provider=self.config_provider,
            robot_id="test_robot",
            simulation_data_service=self.mock_simulation_service
        )
        
        # Start and stop multiple times
        for i in range(3):
            robot.start()
            time.sleep(0.05)
            robot.stop()
            time.sleep(0.05)
            assert not robot.physics_manager.is_running()
        
        # Final cleanup
        robot.stop()  # Should be safe to call multiple times
        assert not robot.physics_manager.is_running() 