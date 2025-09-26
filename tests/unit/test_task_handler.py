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
            robot_id="test_robot",
            battery_manager=None  # No battery manager in unit tests
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


    def test_automatic_charging_disabled(self):
        """Test that automatic charging is disabled when configured off."""
        # Disable automatic charging
        self.task_handler.enable_auto_charging(False)

        # Setup battery below threshold
        self.mock_state_holder.get_battery_level.return_value = 0.20  # 20%

        # Mock time to avoid rate limiting
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0

            # First call should set last check time
            self.task_handler._check_automatic_charging()

            # Second call should check battery but not trigger (disabled)
            self.task_handler._check_automatic_charging()

        # Should not have created any task
        self.assertIsNone(self.task_handler._current_task)

    def test_automatic_charging_above_threshold(self):
        """Test that charging is not triggered when battery is above threshold."""
        # Setup battery above threshold
        self.mock_state_holder.get_battery_level.return_value = 0.30  # 30%

        # Mock time to avoid rate limiting
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0

            # Call automatic charging check
            self.task_handler._check_automatic_charging()

        # Should not have created any task
        self.assertIsNone(self.task_handler._current_task)

    def test_automatic_charging_idle_robot(self):
        """Test automatic charging triggers when robot is idle and battery low."""
        # Setup battery below threshold
        self.mock_state_holder.get_battery_level.return_value = 0.20  # 20%

        # Mock time to avoid rate limiting
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0

            # Call automatic charging check
            self.task_handler._check_automatic_charging()

        # Should have created and started a charging task
        self.assertIsNotNone(self.task_handler._current_task)
        self.assertEqual(self.task_handler._current_task.task_type, TaskType.MOVE_TO_CHARGING)
        self.assertTrue(self.task_handler._charging_task_pending)

        # Check task metadata
        metadata = self.task_handler._current_task.metadata
        self.assertIsNotNone(metadata)
        self.assertEqual(metadata['auto_generated'], True)
        self.assertEqual(metadata['battery_level'], 0.20)
        self.assertEqual(metadata['trigger_reason'], "low_battery_auto_trigger")

    def test_automatic_charging_safe_interrupt(self):
        """Test automatic charging triggers when doing safe interrupt tasks."""
        # Start with a safe interrupt task (IDLE_PARK)
        idle_task = Task(
            task_id="idle_task",
            task_type=TaskType.IDLE_PARK,
            bay_id="idle_bay_1",
            target_position=(5.0, 5.0, 0.0)
        )

        # Start the idle task
        self.assertTrue(self.task_handler.start_task(idle_task))

        # Setup battery below threshold
        self.mock_state_holder.get_battery_level.return_value = 0.20  # 20%

        # Mock time to avoid rate limiting
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0

            # Call automatic charging check
            self.task_handler._check_automatic_charging()

        # Should have interrupted the idle task and created charging task
        self.assertIsNotNone(self.task_handler._current_task)
        self.assertEqual(self.task_handler._current_task.task_type, TaskType.MOVE_TO_CHARGING)
        self.assertTrue(self.task_handler._charging_task_pending)

    def test_automatic_charging_no_interrupt_critical(self):
        """Test automatic charging does not interrupt critical operations."""
        # Start with a critical task (PICK_AND_DELIVER)
        pick_task = Task(
            task_id="pick_task",
            task_type=TaskType.PICK_AND_DELIVER,
            shelf_id="shelf_1",
            item_id="item_1",
            order_id="order_1"
        )

        # Start the pick task
        self.assertTrue(self.task_handler.start_task(pick_task))

        # Setup battery below threshold
        self.mock_state_holder.get_battery_level.return_value = 0.20  # 20%

        # Mock time to avoid rate limiting
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0

            # Call automatic charging check
            self.task_handler._check_automatic_charging()

        # Should NOT have interrupted the critical task
        self.assertIsNotNone(self.task_handler._current_task)
        self.assertEqual(self.task_handler._current_task.task_type, TaskType.PICK_AND_DELIVER)
        self.assertFalse(self.task_handler._charging_task_pending)

    def test_automatic_charging_duplicate_prevention(self):
        """Test that duplicate charging tasks are prevented."""
        # Setup battery below threshold
        self.mock_state_holder.get_battery_level.return_value = 0.20  # 20%

        # Mock time to avoid rate limiting
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0

            # First call should trigger charging
            self.task_handler._check_automatic_charging()

            # Verify charging task was created
            self.assertIsNotNone(self.task_handler._current_task)
            self.assertTrue(self.task_handler._charging_task_pending)

            # Second call should not create another task
            self.task_handler._check_automatic_charging()

            # Should still have the same task
            self.assertEqual(self.task_handler._current_task.task_id,
                           "auto_charge_test_robot_100")

    def test_automatic_charging_rate_limiting(self):
        """Test that battery checks are rate limited."""
        # Setup battery below threshold
        self.mock_state_holder.get_battery_level.return_value = 0.20  # 20%

        # First call with time 100
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0
            self.task_handler._check_automatic_charging()

        # Verify first task was created
        self.assertIsNotNone(self.task_handler._current_task)
        first_task_id = self.task_handler._current_task.task_id

        # Second call immediately after (should be rate limited)
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.5  # Only 0.5 seconds later (less than 1 second)
            self.task_handler._check_automatic_charging()

        # Should still have the same task (no new task created due to rate limiting)
        self.assertIsNotNone(self.task_handler._current_task)
        self.assertEqual(self.task_handler._current_task.task_id, first_task_id)

    def test_automatic_charging_task_completion_reset(self):
        """Test that charging task flag is reset when charging completes."""
        # Start a charging task
        charging_task = Task(
            task_id="charging_task",
            task_type=TaskType.MOVE_TO_CHARGING,
            bay_id="charge_bay_1"
        )

        self.assertTrue(self.task_handler.start_task(charging_task))
        self.assertTrue(self.task_handler._charging_task_pending)

        # Simulate task completion
        self.task_handler._complete_task()

        # Charging flag should be reset
        self.assertFalse(self.task_handler._charging_task_pending)

    def test_enable_disable_auto_charging(self):
        """Test enabling and disabling automatic charging."""
        # Should be enabled by default
        self.assertTrue(self.task_handler._auto_charging_enabled)

        # Disable
        self.task_handler.enable_auto_charging(False)
        self.assertFalse(self.task_handler._auto_charging_enabled)

        # Re-enable
        self.task_handler.enable_auto_charging(True)
        self.assertTrue(self.task_handler._auto_charging_enabled)

    def test_set_charging_threshold(self):
        """Test setting charging threshold."""
        # Test valid threshold
        self.task_handler.set_charging_threshold(0.15)
        self.assertEqual(self.task_handler._charging_trigger_threshold, 0.15)

        # Test invalid threshold (too low)
        self.task_handler.set_charging_threshold(-0.1)
        self.assertEqual(self.task_handler._charging_trigger_threshold, 0.15)  # Should remain unchanged

        # Test invalid threshold (too high)
        self.task_handler.set_charging_threshold(1.5)
        self.assertEqual(self.task_handler._charging_trigger_threshold, 0.15)  # Should remain unchanged

    def test_battery_level_error_handling(self):
        """Test error handling when battery level cannot be retrieved."""
        # Mock state holder to raise exception
        self.mock_state_holder.get_battery_level.side_effect = Exception("Battery sensor failure")

        # Call automatic charging check
        self.task_handler._check_automatic_charging()

        # Should not crash and should not create task
        self.assertIsNone(self.task_handler._current_task)

    def test_update_task_execution_includes_charging_check(self):
        """Test that update_task_execution includes automatic charging checks."""
        # Setup mocks
        self.mock_state_holder.get_battery_level.return_value = 0.20

        # Mock time to avoid rate limiting
        with patch('time.time') as mock_time:
            mock_time.return_value = 100.0

            # Call update_task_execution
            self.task_handler.update_task_execution()

        # Should have checked for automatic charging and created task
        self.assertIsNotNone(self.task_handler._current_task)
        self.assertEqual(self.task_handler._current_task.task_type, TaskType.MOVE_TO_CHARGING)


if __name__ == '__main__':
    unittest.main() 