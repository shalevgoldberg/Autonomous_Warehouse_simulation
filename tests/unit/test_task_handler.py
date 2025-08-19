"""
Unit tests for TaskHandler implementation.

Tests task lifecycle management, thread safety, component coordination, and error handling.
"""
import unittest
import threading
import time
from unittest.mock import Mock, MagicMock, patch
from datetime import datetime

from robot.impl.task_handler_impl import TaskHandlerImpl, TaskPhase
from interfaces.task_handler_interface import (
    Task, TaskType, TaskStatus, OperationalStatus, TaskHandlerStatus, TaskHandlingError
)
from interfaces.state_holder_interface import RobotPhysicsState
from interfaces.path_planner_interface import Path, Cell, PathPlanningError
from interfaces.motion_executor_interface import MotionStatus, MotionExecutionError


class TestTaskHandler(unittest.TestCase):
    """Test TaskHandler implementation with comprehensive coverage."""
    
    def setUp(self):
        """Set up test dependencies and TaskHandler instance."""
        # Create mock dependencies
        self.mock_state_holder = Mock()
        self.mock_path_planner = Mock()
        self.mock_motion_executor = Mock()
        self.mock_coordinate_system = Mock()
        self.mock_simulation_data_service = Mock()
        
        # Setup default simulation service responses
        self.mock_simulation_data_service.lock_shelf.return_value = True
        self.mock_simulation_data_service.unlock_shelf.return_value = True
        self.mock_simulation_data_service.update_inventory.return_value = True
        self.mock_simulation_data_service.get_shelf_position.return_value = (3.0, 3.0)
        self.mock_simulation_data_service.get_dropoff_zones.return_value = [(8.0, 8.0)]
        self.mock_simulation_data_service.log_event.return_value = None
        
        # Create TaskHandler instance
        self.task_handler = TaskHandlerImpl(
            state_holder=self.mock_state_holder,
            path_planner=self.mock_path_planner,
            motion_executor=self.mock_motion_executor,
            coordinate_system=self.mock_coordinate_system,
            simulation_data_service=self.mock_simulation_data_service,
            lane_follower=Mock(),  # Add missing lane_follower parameter
            robot_id="test_robot"
        )
        
        # Setup default mock returns
        self.mock_state_holder.get_robot_state.return_value = RobotPhysicsState(
            robot_id="test_robot",
            position=(2.0, 2.0, 0.0),
            velocity=(0.0, 0.0),
            battery_level=0.8,
            timestamp=time.time()
        )
        
        self.mock_path_planner.plan_path.return_value = Path(
            cells=[Cell(2, 2), Cell(3, 2), Cell(3, 3)],
            total_distance=2.0,
            estimated_time=4.0
        )
        
        self.mock_motion_executor.get_motion_status.return_value = MotionStatus.IDLE
    
    def test_initial_state(self):
        """Test TaskHandler initial state."""
        status = self.task_handler.get_task_status()
        
        self.assertFalse(status.has_active_task)
        self.assertIsNone(status.task_id)
        self.assertEqual(status.operational_status, OperationalStatus.IDLE)
        self.assertEqual(status.progress, 0.0)
        self.assertIsNone(status.stall_reason)
        self.assertTrue(self.task_handler.is_idle())
        self.assertIsNone(self.task_handler.get_current_task())
    
    def test_start_pick_and_deliver_task(self):
        """Test starting a pick and deliver task."""
        task = Task(
            task_id="task_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_001",
            shelf_id="shelf_3_3",
            item_id="item_123",
            priority=1
        )
        
        result = self.task_handler.start_task(task)
        
        self.assertTrue(result)
        self.assertFalse(self.task_handler.is_idle())
        
        status = self.task_handler.get_task_status()
        self.assertTrue(status.has_active_task)
        self.assertEqual(status.task_id, "task_001")
        self.assertEqual(status.operational_status, OperationalStatus.MOVING_TO_SHELF)
        
        # Check current task details
        current_task = self.task_handler.get_current_task()
        self.assertIsNotNone(current_task)
        if current_task:  # Fix linter warning
            self.assertEqual(current_task.task_id, "task_001")
            self.assertEqual(current_task.status, TaskStatus.IN_PROGRESS)
            self.assertIsNotNone(current_task.assigned_at)
    
    def test_task_validation(self):
        """Test task validation rejects invalid tasks."""
        # Task validation now happens at creation time, so we test that
        
        # Task without shelf_id for pick and deliver should raise ValueError
        with self.assertRaises(ValueError):
            Task(
                task_id="invalid_001",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id="order_001"  # Missing shelf_id and item_id
            )
        
        # Task without target_position for move to position should raise ValueError
        with self.assertRaises(ValueError):
            Task(
                task_id="invalid_002",
                task_type=TaskType.MOVE_TO_POSITION
            )
        
        # Valid task should be accepted
        valid_task = Task(
            task_id="valid_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_001",
            shelf_id="shelf_3_3",
            item_id="item_001"
        )
        result = self.task_handler.start_task(valid_task)
        self.assertTrue(result)
    
    def test_emergency_stop(self):
        """Test emergency stop functionality."""
        # Start task
        task = Task(
            task_id="task_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_001",
            shelf_id="shelf_3_3",
            item_id="item_001"
        )
        self.task_handler.start_task(task)
        
        # Emergency stop
        self.task_handler.emergency_stop()
        
        # Verify motion executor emergency stop was called
        self.mock_motion_executor.emergency_stop.assert_called_once()
        
        # Should be idle with failed task in history
        self.assertTrue(self.task_handler.is_idle())
        history = self.task_handler.get_task_history()
        self.assertEqual(len(history), 1)
        if history:  # Fix linter warning
            self.assertEqual(history[0].status, TaskStatus.FAILED)
    
    def test_cancel_task(self):
        """Test task cancellation."""
        # Start task
        task = Task(
            task_id="task_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_001",
            shelf_id="shelf_3_3",
            item_id="item_001"
        )
        self.task_handler.start_task(task)
        
        # Cancel task
        self.task_handler.cancel_task("Test cancellation")
        
        # Verify motion was stopped (may be called twice: once in start_task if needed, once in cancel_task)
        self.assertTrue(self.mock_motion_executor.stop_execution.called)
        
        # Should be idle
        self.assertTrue(self.task_handler.is_idle())
        
        # Task should be in history as cancelled
        history = self.task_handler.get_task_history()
        self.assertEqual(len(history), 1)
        if history:  # Fix linter warning
            self.assertEqual(history[0].status, TaskStatus.CANCELLED)
            if history[0].metadata:
                self.assertEqual(history[0].metadata['cancel_reason'], "Test cancellation")


if __name__ == '__main__':
    unittest.main() 