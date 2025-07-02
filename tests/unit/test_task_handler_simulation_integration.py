"""
Unit tests for TaskHandler SimulationDataService integration.

Tests the complete integration of TaskHandler with SimulationDataService including:
- Shelf locking/unlocking during task execution
- Inventory updates when picking items
- KPI event logging throughout task lifecycle
- Error handling and cleanup
- Thread safety of all operations

Follows our established testing principles with comprehensive coverage.
"""
import pytest
import time
import threading
from unittest.mock import Mock, MagicMock, call
from datetime import datetime

from robot.impl.task_handler_impl import TaskHandlerImpl, TaskPhase
from interfaces.task_handler_interface import (
    Task, TaskType, TaskStatus, OperationalStatus, TaskHandlingError
)
from interfaces.simulation_data_service_interface import SimulationDataServiceError


class TestTaskHandlerSimulationIntegration:
    """Test TaskHandler integration with SimulationDataService."""
    
    @pytest.fixture
    def mock_state_holder(self):
        """Mock StateHolder for testing."""
        mock = Mock()
        mock.get_robot_state.return_value = Mock(position=(1.0, 1.0, 0.0))
        mock.get_battery_level.return_value = 0.8
        return mock
    
    @pytest.fixture
    def mock_path_planner(self):
        """Mock PathPlanner for testing."""
        mock = Mock()
        mock_path = Mock()
        mock_path.cells = [Mock(x=1, y=1), Mock(x=2, y=2)]
        mock.plan_path.return_value = mock_path
        return mock
    
    @pytest.fixture
    def mock_motion_executor(self):
        """Mock MotionExecutor for testing."""
        mock = Mock()
        mock.get_motion_status.return_value = Mock()
        return mock
    
    @pytest.fixture
    def mock_coordinate_system(self):
        """Mock CoordinateSystem for testing."""
        return Mock()
    
    @pytest.fixture
    def mock_simulation_data_service(self):
        """Mock SimulationDataService for testing."""
        mock = Mock()
        # Default successful responses
        mock.lock_shelf.return_value = True
        mock.unlock_shelf.return_value = True
        mock.update_inventory.return_value = True
        mock.get_shelf_position.return_value = (3.0, 3.0)
        mock.get_dropoff_zones.return_value = [(8.0, 8.0)]
        mock.log_event.return_value = None
        return mock
    
    @pytest.fixture
    def task_handler(self, mock_state_holder, mock_path_planner, mock_motion_executor, 
                    mock_coordinate_system, mock_simulation_data_service):
        """Create TaskHandler with all mocked dependencies."""
        return TaskHandlerImpl(
            state_holder=mock_state_holder,
            path_planner=mock_path_planner,
            motion_executor=mock_motion_executor,
            coordinate_system=mock_coordinate_system,
            simulation_data_service=mock_simulation_data_service,
            robot_id="test_robot_1"
        )
    
    @pytest.fixture
    def sample_pick_task(self):
        """Create a sample pick and deliver task."""
        return Task(
            task_id="test_task_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_001",
            shelf_id="shelf_A1",
            item_id="book_001",
            quantity_to_pick=2,
            customer_id="customer_1"
        )
    
    def test_task_start_logs_kpi_event(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test that starting a task logs appropriate KPI event."""
        # Start task
        result = task_handler.start_task(sample_pick_task)
        
        # Verify task accepted
        assert result is True
        
        # Verify KPI event logged
        mock_simulation_data_service.log_event.assert_called_with(
            event_type="task_start",
            robot_id="test_robot_1",
            event_data={
                "task_id": "test_task_001",
                "task_type": "pick_and_deliver",
                "order_id": "order_001",
                "shelf_id": "shelf_A1",
                "item_id": "book_001",
                "quantity": 2
            }
        )
    
    def test_shelf_locking_during_picking_phase(self, task_handler, mock_simulation_data_service, 
                                              mock_motion_executor, sample_pick_task):
        """Test shelf locking during picking phase."""
        # Setup motion executor to simulate reaching shelf
        from interfaces.motion_executor_interface import MotionStatus
        mock_motion_executor.get_motion_status.return_value = MotionStatus.REACHED_TARGET
        
        # Start task and advance to picking phase
        task_handler.start_task(sample_pick_task)
        
        # Advance to picking phase
        task_handler._task_phase = TaskPhase.PICKING_ITEM
        task_handler._phase_start_time = None  # Reset to trigger initialization
        task_handler.update_task_execution()
        
        # Verify shelf was locked
        mock_simulation_data_service.lock_shelf.assert_called_with("shelf_A1", "test_robot_1")
        
        # Verify shelf lock KPI event
        expected_calls = [
            call(event_type="task_start", robot_id="test_robot_1", event_data={
                "task_id": "test_task_001",
                "task_type": "pick_and_deliver", 
                "order_id": "order_001",
                "shelf_id": "shelf_A1",
                "item_id": "book_001",
                "quantity": 2
            }),
            call(event_type="shelf_locked", robot_id="test_robot_1", event_data={
                "shelf_id": "shelf_A1",
                "operation": "pick"
            })
        ]
        mock_simulation_data_service.log_event.assert_has_calls(expected_calls)
    
    def test_inventory_update_during_picking(self, task_handler, mock_simulation_data_service,
                                           mock_motion_executor, sample_pick_task):
        """Test inventory update when picking completes."""
        from interfaces.motion_executor_interface import MotionStatus
        mock_motion_executor.get_motion_status.return_value = MotionStatus.REACHED_TARGET
        
        # Start task
        task_handler.start_task(sample_pick_task)
        
        # Advance to picking phase and simulate completion
        task_handler._task_phase = TaskPhase.PICKING_ITEM
        task_handler._phase_start_time = time.time() - 4.0  # Simulate picking duration passed
        task_handler._locked_shelf_id = "shelf_A1"  # Simulate shelf already locked
        
        task_handler.update_task_execution()
        
        # Verify inventory was updated
        mock_simulation_data_service.update_inventory.assert_called_with(
            shelf_id="shelf_A1",
            item_id="book_001",
            operation='remove',
            quantity=2
        )
        
        # Verify shelf was unlocked
        mock_simulation_data_service.unlock_shelf.assert_called_with("shelf_A1", "test_robot_1")
    
    def test_pick_operation_kpi_logging(self, task_handler, mock_simulation_data_service,
                                      mock_motion_executor, sample_pick_task):
        """Test KPI logging for successful pick operation."""
        from interfaces.motion_executor_interface import MotionStatus
        mock_motion_executor.get_motion_status.return_value = MotionStatus.REACHED_TARGET
        
        # Start task and simulate picking completion
        task_handler.start_task(sample_pick_task)
        task_handler._task_phase = TaskPhase.PICKING_ITEM
        task_handler._phase_start_time = time.time() - 4.0
        task_handler._locked_shelf_id = "shelf_A1"
        
        task_handler.update_task_execution()
        
        # Verify pick operation KPI event
        pick_event_call = None
        for call_args in mock_simulation_data_service.log_event.call_args_list:
            if call_args[1]['event_type'] == 'pick_operation':
                pick_event_call = call_args
                break
        
        assert pick_event_call is not None
        assert pick_event_call[1]['event_data'] == {
            "shelf_id": "shelf_A1",
            "item_id": "book_001",
            "quantity": 2,
            "success": True
        }
    
    def test_drop_operation_kpi_logging(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test KPI logging for drop operation."""
        # Start task and advance to dropping phase
        task_handler.start_task(sample_pick_task)
        task_handler._task_phase = TaskPhase.DROPPING_ITEM
        task_handler._phase_start_time = time.time() - 3.0  # Simulate dropping duration passed
        
        task_handler.update_task_execution()
        
        # Verify drop operation KPI event
        drop_event_call = None
        for call_args in mock_simulation_data_service.log_event.call_args_list:
            if call_args[1]['event_type'] == 'drop_operation':
                drop_event_call = call_args
                break
        
        assert drop_event_call is not None
        assert drop_event_call[1]['event_data']['success'] is True
        assert drop_event_call[1]['event_data']['order_id'] == "order_001"
        assert drop_event_call[1]['event_data']['item_id'] == "book_001"
        assert drop_event_call[1]['event_data']['quantity'] == 2
    
    def test_task_completion_kpi_logging(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test KPI logging for task completion."""
        # Start task and complete it
        task_handler.start_task(sample_pick_task)
        task_handler._complete_task()
        
        # Verify task completion KPI event
        completion_event_call = None
        for call_args in mock_simulation_data_service.log_event.call_args_list:
            if call_args[1]['event_type'] == 'task_complete':
                completion_event_call = call_args
                break
        
        assert completion_event_call is not None
        event_data = completion_event_call[1]['event_data']
        assert event_data['task_id'] == "test_task_001"
        assert event_data['order_id'] == "order_001"
        assert event_data['success'] is True
        assert 'duration_seconds' in event_data
    
    def test_shelf_lock_failure_handling(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test handling when shelf lock fails (already locked by another robot)."""
        # Setup shelf lock to fail
        mock_simulation_data_service.lock_shelf.return_value = False
        
        # Start task and advance to picking phase
        task_handler.start_task(sample_pick_task)
        task_handler._task_phase = TaskPhase.PICKING_ITEM
        task_handler._phase_start_time = None  # Trigger initialization
        
        task_handler.update_task_execution()
        
        # Verify task failed due to lock failure
        current_task = task_handler.get_current_task()
        assert current_task is None or current_task.status == TaskStatus.FAILED
        
        # Verify failure KPI event logged
        failure_event_call = None
        for call_args in mock_simulation_data_service.log_event.call_args_list:
            if call_args[1]['event_type'] == 'task_failed':
                failure_event_call = call_args
                break
        
        assert failure_event_call is not None
        assert "already in use" in failure_event_call[1]['event_data']['error']
    
    def test_inventory_update_failure_handling(self, task_handler, mock_simulation_data_service,
                                             mock_motion_executor, sample_pick_task):
        """Test handling when inventory update fails."""
        from interfaces.motion_executor_interface import MotionStatus
        mock_motion_executor.get_motion_status.return_value = MotionStatus.REACHED_TARGET
        
        # Setup inventory update to fail
        mock_simulation_data_service.update_inventory.return_value = False
        
        # Start task and simulate picking completion
        task_handler.start_task(sample_pick_task)
        task_handler._task_phase = TaskPhase.PICKING_ITEM
        task_handler._phase_start_time = time.time() - 4.0
        task_handler._locked_shelf_id = "shelf_A1"
        
        task_handler.update_task_execution()
        
        # Verify failed pick operation KPI event
        pick_event_call = None
        for call_args in mock_simulation_data_service.log_event.call_args_list:
            if call_args[1]['event_type'] == 'pick_operation':
                pick_event_call = call_args
                break
        
        assert pick_event_call is not None
        event_data = pick_event_call[1]['event_data']
        assert event_data['success'] is False
        assert event_data['reason'] == 'inventory_update_failed'
    
    def test_simulation_service_exception_handling(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test handling of SimulationDataService exceptions."""
        # Setup service to raise exception
        mock_simulation_data_service.lock_shelf.side_effect = SimulationDataServiceError("Database connection failed")
        
        # Start task and advance to picking phase
        task_handler.start_task(sample_pick_task)
        task_handler._task_phase = TaskPhase.PICKING_ITEM
        task_handler._phase_start_time = None
        
        task_handler.update_task_execution()
        
        # Verify task failed with appropriate error
        current_task = task_handler.get_current_task()
        assert current_task is None or current_task.status == TaskStatus.FAILED
        
        # Verify failure KPI event logged
        failure_event_call = None
        for call_args in mock_simulation_data_service.log_event.call_args_list:
            if call_args[1]['event_type'] == 'task_failed':
                failure_event_call = call_args
                break
        
        assert failure_event_call is not None
        assert "Shelf locking failed" in failure_event_call[1]['event_data']['error']
    
    def test_shelf_position_integration(self, task_handler, mock_simulation_data_service):
        """Test shelf position retrieval from SimulationDataService."""
        # Test successful position retrieval
        position = task_handler._get_shelf_position("shelf_A1")
        assert position == (3.0, 3.0)
        mock_simulation_data_service.get_shelf_position.assert_called_with("shelf_A1")
        
        # Test fallback when shelf not found
        mock_simulation_data_service.get_shelf_position.return_value = None
        position = task_handler._get_shelf_position("nonexistent_shelf")
        assert position == (3.0, 3.0)  # Fallback position
        
        # Test exception handling
        mock_simulation_data_service.get_shelf_position.side_effect = SimulationDataServiceError("Service error")
        position = task_handler._get_shelf_position("shelf_A1")
        assert position == (3.0, 3.0)  # Fallback position
    
    def test_dropoff_position_integration(self, task_handler, mock_simulation_data_service):
        """Test dropoff position retrieval from SimulationDataService."""
        # Test successful position retrieval
        position = task_handler._get_dropoff_position()
        assert position == (8.0, 8.0)
        mock_simulation_data_service.get_dropoff_zones.assert_called_once()
        
        # Test fallback when no dropoff zones
        mock_simulation_data_service.get_dropoff_zones.return_value = []
        position = task_handler._get_dropoff_position()
        assert position == (8.0, 8.0)  # Fallback position
        
        # Test exception handling
        mock_simulation_data_service.get_dropoff_zones.side_effect = SimulationDataServiceError("Service error")
        position = task_handler._get_dropoff_position()
        assert position == (8.0, 8.0)  # Fallback position
    
    def test_shelf_cleanup_on_task_error(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test that shelves are properly unlocked when task fails."""
        # Start task and simulate shelf being locked
        task_handler.start_task(sample_pick_task)
        task_handler._locked_shelf_id = "shelf_A1"
        
        # Trigger task error
        task_handler._handle_task_error("Test error")
        
        # Verify shelf was unlocked
        mock_simulation_data_service.unlock_shelf.assert_called_with("shelf_A1", "test_robot_1")
        
        # Verify shelf unlock KPI event
        unlock_event_call = None
        for call_args in mock_simulation_data_service.log_event.call_args_list:
            if call_args[1]['event_type'] == 'shelf_unlocked':
                unlock_event_call = call_args
                break
        
        assert unlock_event_call is not None
        assert unlock_event_call[1]['event_data']['shelf_id'] == "shelf_A1"
    
    def test_shelf_cleanup_on_task_reset(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test that shelves are properly unlocked when task state is reset."""
        # Start task and simulate shelf being locked
        task_handler.start_task(sample_pick_task)
        task_handler._locked_shelf_id = "shelf_A1"
        
        # Reset task state
        task_handler._reset_task_state()
        
        # Verify shelf was unlocked
        mock_simulation_data_service.unlock_shelf.assert_called_with("shelf_A1", "test_robot_1")
        
        # Verify locked shelf ID is cleared
        assert task_handler._locked_shelf_id is None
    
    def test_kpi_logging_error_handling(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test that KPI logging errors don't fail the task."""
        # Setup KPI logging to fail
        mock_simulation_data_service.log_event.side_effect = SimulationDataServiceError("Logging failed")
        
        # Start task (should succeed despite logging failure)
        result = task_handler.start_task(sample_pick_task)
        assert result is True
        
        # Verify task is still active
        current_task = task_handler.get_current_task()
        assert current_task is not None
        assert current_task.status == TaskStatus.IN_PROGRESS
    
    def test_thread_safety_of_simulation_service_calls(self, task_handler, mock_simulation_data_service, sample_pick_task):
        """Test thread safety of SimulationDataService integration."""
        # Start task
        task_handler.start_task(sample_pick_task)
        
        # Simulate concurrent access from multiple threads
        results = []
        errors = []
        
        def concurrent_operation():
            try:
                status = task_handler.get_task_status()
                results.append(status.operational_status)
            except Exception as e:
                errors.append(str(e))
        
        # Start multiple threads
        threads = []
        for _ in range(10):
            thread = threading.Thread(target=concurrent_operation)
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Verify no errors occurred
        assert len(errors) == 0
        assert len(results) == 10
        
        # Verify all results are valid
        for result in results:
            assert isinstance(result, OperationalStatus)


class TestTaskHandlerRealWorldScenarios:
    """Test TaskHandler in realistic scenarios."""
    
    @pytest.fixture
    def integrated_task_handler(self):
        """Create TaskHandler with more realistic mock behavior."""
        mock_state_holder = Mock()
        mock_state_holder.get_robot_state.return_value = Mock(position=(1.0, 1.0, 0.0))
        mock_state_holder.get_battery_level.return_value = 0.8
        
        mock_path_planner = Mock()
        mock_path = Mock()
        mock_path.cells = [Mock(x=1, y=1), Mock(x=2, y=2), Mock(x=3, y=3)]
        mock_path_planner.plan_path.return_value = mock_path
        
        mock_motion_executor = Mock()
        from interfaces.motion_executor_interface import MotionStatus
        mock_motion_executor.get_motion_status.return_value = MotionStatus.EXECUTING
        
        mock_coordinate_system = Mock()
        
        mock_simulation_service = Mock()
        mock_simulation_service.lock_shelf.return_value = True
        mock_simulation_service.unlock_shelf.return_value = True
        mock_simulation_service.update_inventory.return_value = True
        mock_simulation_service.get_shelf_position.return_value = (5.0, 5.0)
        mock_simulation_service.get_dropoff_zones.return_value = [(10.0, 10.0)]
        
        return TaskHandlerImpl(
            state_holder=mock_state_holder,
            path_planner=mock_path_planner,
            motion_executor=mock_motion_executor,
            coordinate_system=mock_coordinate_system,
            simulation_data_service=mock_simulation_service,
            robot_id="warehouse_robot_1"
        )
    
    def test_complete_pick_and_deliver_workflow(self, integrated_task_handler):
        """Test complete pick and deliver workflow with all integrations."""
        # Create realistic task
        task = Task(
            task_id="real_task_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="customer_order_123",
            shelf_id="electronics_shelf_A5",
            item_id="smartphone_001",
            quantity_to_pick=1,
            customer_id="customer_456",
            order_priority="high"
        )
        
        # Start task
        result = integrated_task_handler.start_task(task)
        assert result is True
        
        # Verify initial state
        status = integrated_task_handler.get_task_status()
        assert status.has_active_task is True
        assert status.task_id == "real_task_001"
        assert status.operational_status == OperationalStatus.MOVING_TO_SHELF
        
        # Verify task is properly tracked
        current_task = integrated_task_handler.get_current_task()
        assert current_task.task_id == "real_task_001"
        assert current_task.status == TaskStatus.IN_PROGRESS
        assert current_task.assigned_at is not None
    
    def test_multiple_tasks_sequential_execution(self, integrated_task_handler):
        """Test sequential execution of multiple tasks."""
        tasks = [
            Task(
                task_id=f"seq_task_{i:03d}",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id=f"order_{i:03d}",
                shelf_id=f"shelf_{i:03d}",
                item_id=f"item_{i:03d}",
                quantity_to_pick=1
            )
            for i in range(3)
        ]
        
        # Execute tasks sequentially
        for i, task in enumerate(tasks):
            # Start task
            result = integrated_task_handler.start_task(task)
            assert result is True
            
            # Complete task
            integrated_task_handler._complete_task()
            
            # Verify task is in history
            history = integrated_task_handler.get_task_history(limit=5)
            assert len(history) == i + 1
            assert history[-1].task_id == task.task_id
            assert history[-1].status == TaskStatus.COMPLETED
        
        # Verify robot is idle after all tasks
        assert integrated_task_handler.is_idle() is True
    
    def test_task_rejection_when_busy(self, integrated_task_handler):
        """Test that new tasks are rejected when robot is busy."""
        # Start first task
        task1 = Task(
            task_id="busy_test_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_001",
            shelf_id="shelf_001",
            item_id="item_001"
        )
        
        result1 = integrated_task_handler.start_task(task1)
        assert result1 is True
        
        # Try to start second task (should be rejected)
        task2 = Task(
            task_id="busy_test_002",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_002",
            shelf_id="shelf_002",
            item_id="item_002"
        )
        
        result2 = integrated_task_handler.start_task(task2)
        assert result2 is False
        
        # Verify first task is still active
        current_task = integrated_task_handler.get_current_task()
        assert current_task.task_id == "busy_test_001" 