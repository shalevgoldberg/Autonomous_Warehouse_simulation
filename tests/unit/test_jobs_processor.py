"""
Unit tests for JobsProcessor implementation.

Tests the core order processing workflow, inventory allocation,
task creation, and coordination features.
"""
import pytest
import time
import threading
from unittest.mock import Mock, MagicMock, patch
from datetime import datetime, timedelta
from typing import List

from warehouse.impl.jobs_processor_impl import JobsProcessorImpl, InventoryAllocation
from interfaces.jobs_processor_interface import (
    IJobsProcessor, Order, OrderItem, Priority, OrderStatus,
    ProcessingResult, ProcessingStats, JobsProcessorError
)
from interfaces.order_source_interface import IOrderSource
from interfaces.simulation_data_service_interface import ISimulationDataService
from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.task_handler_interface import Task, TaskType, TaskStatus


class TestJobsProcessorImpl:
    """Test JobsProcessor implementation."""
    
    @pytest.fixture
    def mock_order_source(self):
        """Create mock order source."""
        mock = Mock(spec=IOrderSource)
        return mock
    
    @pytest.fixture
    def mock_simulation_data_service(self):
        """Create mock simulation data service."""
        mock = Mock(spec=ISimulationDataService)
        return mock
    
    @pytest.fixture
    def mock_jobs_queue(self):
        """Create mock jobs queue."""
        mock = Mock(spec=IJobsQueue)
        return mock
    
    @pytest.fixture
    def jobs_processor(self, mock_order_source, mock_simulation_data_service, mock_jobs_queue):
        """Create JobsProcessor instance with mocked dependencies."""
        return JobsProcessorImpl(
            order_source=mock_order_source,
            simulation_data_service=mock_simulation_data_service,
            jobs_queue=mock_jobs_queue,
            processing_interval=0.1,  # Fast for testing
            max_concurrent_orders=5
        )
    
    @pytest.fixture
    def sample_order(self):
        """Create a sample order for testing."""
        return Order(
            order_id="test_order_001",
            customer_id="customer_123",
            items=[
                OrderItem(item_id="book_001", quantity=2),
                OrderItem(item_id="phone_001", quantity=1)
            ],
            priority=Priority.NORMAL,
            scheduled_time=datetime.now() - timedelta(minutes=5),
            status=OrderStatus.PENDING
        )
    
    def test_initialization(self, jobs_processor):
        """Test JobsProcessor initialization."""
        assert not jobs_processor.is_processing()
        assert len(jobs_processor.get_active_orders()) == 0
        
        stats = jobs_processor.get_processing_stats()
        assert stats.orders_processed == 0
        assert stats.tasks_created == 0
        assert stats.processing_errors == 0
        assert stats.active_orders == 0
        assert stats.locked_shelves == 0
    
    def test_start_stop_processing(self, jobs_processor):
        """Test starting and stopping the processor thread."""
        # Start processing
        jobs_processor.start_processing()
        assert jobs_processor.is_processing()
        
        # Wait a bit to ensure thread is running
        time.sleep(0.2)
        assert jobs_processor.is_processing()
        
        # Stop processing
        jobs_processor.stop_processing()
        assert not jobs_processor.is_processing()
    
    def test_start_already_running_error(self, jobs_processor):
        """Test error when starting already running processor."""
        jobs_processor.start_processing()
        
        with pytest.raises(JobsProcessorError, match="already running"):
            jobs_processor.start_processing()
        
        jobs_processor.stop_processing()
    
    def test_process_orders_once_no_orders(self, jobs_processor, mock_order_source):
        """Test processing when no orders are available."""
        mock_order_source.get_due_orders.return_value = []
        
        result = jobs_processor.process_orders_once()
        
        assert result.success
        assert result.orders_processed == 0
        assert result.tasks_created == 0
        assert "No due orders" in result.message
        assert result.errors is None
    
    def test_process_single_order_success(self, jobs_processor, mock_order_source, 
                                        mock_simulation_data_service, mock_jobs_queue, 
                                        sample_order):
        """Test successful processing of a single order."""
        # Setup mocks
        mock_order_source.get_due_orders.return_value = [sample_order]
        mock_order_source.update_order_status.return_value = None
        
        # Mock inventory info
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        # Mock shelf locations
        mock_shelf_location = Mock()
        mock_shelf_location.shelf_id = "shelf_A1"
        mock_shelf_location.quantity = 5
        mock_simulation_data_service.get_item_shelf_locations.return_value = [mock_shelf_location]
        
        # Process orders
        result = jobs_processor.process_orders_once()
        
        # Verify results
        assert result.success
        assert result.orders_processed == 1
        assert result.tasks_created == 2  # 2 items in order
        assert "Processed 1 orders" in result.message
        
        # Verify mocks were called
        mock_order_source.get_due_orders.assert_called_once()
        mock_order_source.update_order_status.assert_called_with(
            sample_order.order_id, OrderStatus.IN_PROGRESS
        )
        assert mock_jobs_queue.push_task.call_count == 2
    
    def test_process_order_insufficient_inventory(self, jobs_processor, mock_order_source, 
                                                mock_simulation_data_service, sample_order):
        """Test processing order with insufficient inventory."""
        # Setup mocks
        mock_order_source.get_due_orders.return_value = [sample_order]
        
        # Mock insufficient inventory
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 1  # Less than required
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        # Process orders
        result = jobs_processor.process_orders_once()
        
        # Verify failure
        assert not result.success
        assert result.orders_processed == 0
        assert result.tasks_created == 0
        assert result.errors is not None
        assert len(result.errors) == 1
        assert "Insufficient inventory" in result.errors[0]
    
    def test_process_order_no_shelf_locations(self, jobs_processor, mock_order_source, 
                                            mock_simulation_data_service, sample_order):
        """Test processing order with no shelf locations."""
        # Setup mocks
        mock_order_source.get_due_orders.return_value = [sample_order]
        
        # Mock inventory but no shelf locations
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        mock_simulation_data_service.get_item_shelf_locations.return_value = []
        
        # Process orders
        result = jobs_processor.process_orders_once()
        
        # Verify failure
        assert not result.success
        assert "No shelf locations found" in result.errors[0]
    
    def test_inventory_allocation_tracking(self, jobs_processor):
        """Test inventory allocation data structure."""
        allocation = InventoryAllocation(
            order_id="order_123",
            item_id="book_001",
            shelf_id="shelf_A1",
            quantity_allocated=3,
            quantity_requested=5
        )
        
        assert allocation.order_id == "order_123"
        assert allocation.item_id == "book_001"
        assert allocation.shelf_id == "shelf_A1"
        assert allocation.quantity_allocated == 3
        assert allocation.quantity_requested == 5
        assert not allocation.is_fully_allocated
        
        # Test full allocation
        allocation.quantity_allocated = 5
        assert allocation.is_fully_allocated
        
        # Test over allocation
        allocation.quantity_allocated = 7
        assert allocation.is_fully_allocated
    
    def test_shelf_locking_mechanism(self, jobs_processor, mock_order_source, 
                                   mock_simulation_data_service, mock_jobs_queue):
        """Test shelf locking prevents concurrent access."""
        # Create two orders for same shelf
        order1 = Order(
            order_id="order_001",
            customer_id="customer_1",
            items=[OrderItem(item_id="book_001", quantity=1)],
            priority=Priority.NORMAL,
            scheduled_time=datetime.now(),
            status=OrderStatus.PENDING
        )
        
        order2 = Order(
            order_id="order_002",
            customer_id="customer_2",
            items=[OrderItem(item_id="book_001", quantity=1)],
            priority=Priority.HIGH,
            scheduled_time=datetime.now(),
            status=OrderStatus.PENDING
        )
        
        # Setup mocks
        mock_order_source.get_due_orders.return_value = [order1, order2]
        
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        # Same shelf for both orders
        mock_shelf_location = Mock()
        mock_shelf_location.shelf_id = "shelf_A1"
        mock_shelf_location.quantity = 5
        mock_simulation_data_service.get_item_shelf_locations.return_value = [mock_shelf_location]
        
        # Process orders
        result = jobs_processor.process_orders_once()
        
        # Verify both orders processed but shelf only allocated once initially
        assert result.success
        assert result.orders_processed == 2
        
        # Check that shelf locking is tracked
        stats = jobs_processor.get_processing_stats()
        assert stats.locked_shelves > 0
    
    def test_cancel_order(self, jobs_processor, mock_order_source, 
                         mock_simulation_data_service, mock_jobs_queue, sample_order):
        """Test cancelling an active order."""
        # First process an order to make it active
        mock_order_source.get_due_orders.return_value = [sample_order]
        
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        mock_shelf_location = Mock()
        mock_shelf_location.shelf_id = "shelf_A1"
        mock_shelf_location.quantity = 5
        mock_simulation_data_service.get_item_shelf_locations.return_value = [mock_shelf_location]
        
        # Process order
        jobs_processor.process_orders_once()
        
        # Verify order is active
        active_orders = jobs_processor.get_active_orders()
        assert len(active_orders) == 1
        assert active_orders[0].order_id == sample_order.order_id
        
        # Cancel the order
        success = jobs_processor.cancel_order(sample_order.order_id)
        assert success
        
        # Verify order is no longer active
        active_orders = jobs_processor.get_active_orders()
        assert len(active_orders) == 0
        
        # Verify status update was called
        mock_order_source.update_order_status.assert_called_with(
            sample_order.order_id, OrderStatus.CANCELLED
        )
    
    def test_cancel_nonexistent_order(self, jobs_processor):
        """Test cancelling a non-existent order."""
        success = jobs_processor.cancel_order("nonexistent_order")
        assert not success
    
    def test_priority_based_processing(self, jobs_processor, mock_order_source, 
                                     mock_simulation_data_service, mock_jobs_queue):
        """Test that orders are processed according to priority."""
        # Create orders with different priorities
        urgent_order = Order(
            order_id="urgent_order",
            customer_id="urgent_customer",
            items=[OrderItem(item_id="book_001", quantity=1)],
            priority=Priority.URGENT,
            scheduled_time=datetime.now(),
            status=OrderStatus.PENDING
        )
        
        normal_order = Order(
            order_id="normal_order",
            customer_id="normal_customer",
            items=[OrderItem(item_id="phone_001", quantity=1)],
            priority=Priority.NORMAL,
            scheduled_time=datetime.now(),
            status=OrderStatus.PENDING
        )
        
        # Setup mocks (return normal order first, then urgent)
        mock_order_source.get_due_orders.return_value = [normal_order, urgent_order]
        
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        mock_shelf_location = Mock()
        mock_shelf_location.shelf_id = "shelf_A1"
        mock_shelf_location.quantity = 5
        mock_simulation_data_service.get_item_shelf_locations.return_value = [mock_shelf_location]
        
        # Process orders
        result = jobs_processor.process_orders_once()
        
        # Verify both orders processed
        assert result.success
        assert result.orders_processed == 2
        
        # Check that tasks were created with correct priorities
        created_tasks = []
        for call in mock_jobs_queue.push_task.call_args_list:
            task = call[0][0]  # First argument of each call
            created_tasks.append(task)
        
        # Verify task priorities
        urgent_tasks = [t for t in created_tasks if t.order_priority == "urgent"]
        normal_tasks = [t for t in created_tasks if t.order_priority == "normal"]
        
        assert len(urgent_tasks) == 1
        assert len(normal_tasks) == 1
    
    def test_task_creation_details(self, jobs_processor, mock_order_source, 
                                 mock_simulation_data_service, mock_jobs_queue, sample_order):
        """Test detailed task creation with all fields."""
        # Setup mocks
        mock_order_source.get_due_orders.return_value = [sample_order]
        
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        mock_shelf_location = Mock()
        mock_shelf_location.shelf_id = "shelf_A1"
        mock_shelf_location.quantity = 5
        mock_simulation_data_service.get_item_shelf_locations.return_value = [mock_shelf_location]
        
        # Process orders
        result = jobs_processor.process_orders_once()
        
        # Get created tasks
        created_tasks = []
        for call in mock_jobs_queue.push_task.call_args_list:
            task = call[0][0]
            created_tasks.append(task)
        
        # Verify task details
        assert len(created_tasks) == 2  # 2 items in sample order
        
        for task in created_tasks:
            assert task.task_type == TaskType.PICK_AND_DELIVER
            assert task.order_id == sample_order.order_id
            assert task.shelf_id == "shelf_A1"
            assert task.customer_id == sample_order.customer_id
            assert task.order_priority == sample_order.priority.value.lower()
            assert task.inventory_reserved
            assert task.shelf_locked
            assert task.status == TaskStatus.PENDING
            assert task.created_at is not None
    
    def test_statistics_tracking(self, jobs_processor, mock_order_source, 
                               mock_simulation_data_service, mock_jobs_queue, sample_order):
        """Test statistics tracking functionality."""
        # Initial stats
        initial_stats = jobs_processor.get_processing_stats()
        assert initial_stats.orders_processed == 0
        assert initial_stats.tasks_created == 0
        
        # Setup mocks for successful processing
        mock_order_source.get_due_orders.return_value = [sample_order]
        
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        mock_shelf_location = Mock()
        mock_shelf_location.shelf_id = "shelf_A1"
        mock_shelf_location.quantity = 5
        mock_simulation_data_service.get_item_shelf_locations.return_value = [mock_shelf_location]
        
        # Process orders
        jobs_processor.process_orders_once()
        
        # Check updated stats
        updated_stats = jobs_processor.get_processing_stats()
        assert updated_stats.orders_processed == 1
        assert updated_stats.tasks_created == 2
        assert updated_stats.processing_errors == 0
        assert updated_stats.last_processing_time is not None
        assert updated_stats.average_processing_time > 0
        assert updated_stats.active_orders == 1
        assert updated_stats.locked_shelves == 1
    
    def test_error_handling_and_recovery(self, jobs_processor, mock_order_source, 
                                       mock_simulation_data_service, sample_order):
        """Test error handling and recovery mechanisms."""
        # Setup mock to raise exception
        mock_order_source.get_due_orders.return_value = [sample_order]
        mock_simulation_data_service.get_item_inventory.side_effect = Exception("Database error")
        
        # Process orders (should handle exception gracefully)
        result = jobs_processor.process_orders_once()
        
        # Verify error handling
        assert not result.success
        assert result.orders_processed == 0
        assert result.tasks_created == 0
        assert result.errors is not None
        assert len(result.errors) == 1
        assert "Database error" in result.errors[0]
        
        # Verify stats updated with error
        stats = jobs_processor.get_processing_stats()
        assert stats.processing_errors > 0
    
    def test_max_concurrent_orders_limit(self, jobs_processor, mock_order_source):
        """Test maximum concurrent orders limit."""
        # Create more orders than the limit (5)
        orders = []
        for i in range(7):
            order = Order(
                order_id=f"order_{i:03d}",
                customer_id=f"customer_{i}",
                items=[OrderItem(item_id="book_001", quantity=1)],
                priority=Priority.NORMAL,
                scheduled_time=datetime.now(),
                status=OrderStatus.PENDING
            )
            orders.append(order)
        
        mock_order_source.get_due_orders.return_value = orders
        
        # Verify get_due_orders called with correct limit
        jobs_processor.process_orders_once()
        
        # Should request only up to max_concurrent_orders (5)
        mock_order_source.get_due_orders.assert_called_with(limit=5)
    
    def test_thread_safety(self, jobs_processor, mock_order_source, 
                          mock_simulation_data_service, mock_jobs_queue):
        """Test thread safety of concurrent operations."""
        # Setup mocks
        mock_order_source.get_due_orders.return_value = []
        
        # Start background processing
        jobs_processor.start_processing()
        
        # Perform concurrent operations
        def concurrent_operations():
            for _ in range(10):
                jobs_processor.get_processing_stats()
                jobs_processor.get_active_orders()
                jobs_processor.is_processing()
                time.sleep(0.01)
        
        # Run concurrent operations in multiple threads
        threads = []
        for _ in range(3):
            thread = threading.Thread(target=concurrent_operations)
            threads.append(thread)
            thread.start()
        
        # Wait for threads to complete
        for thread in threads:
            thread.join()
        
        # Stop processing
        jobs_processor.stop_processing()
        
        # Verify no exceptions occurred (test passes if we get here)
        assert True
    
    def test_cleanup_on_shutdown(self, jobs_processor, mock_order_source, 
                                mock_simulation_data_service, mock_jobs_queue, sample_order):
        """Test proper cleanup when shutting down."""
        # Process an order to create allocations
        mock_order_source.get_due_orders.return_value = [sample_order]
        
        mock_inventory_info = Mock()
        mock_inventory_info.available_quantity = 10
        mock_simulation_data_service.get_item_inventory.return_value = mock_inventory_info
        
        mock_shelf_location = Mock()
        mock_shelf_location.shelf_id = "shelf_A1"
        mock_shelf_location.quantity = 5
        mock_simulation_data_service.get_item_shelf_locations.return_value = [mock_shelf_location]
        
        jobs_processor.process_orders_once()
        
        # Verify allocations exist
        stats = jobs_processor.get_processing_stats()
        assert stats.active_orders > 0
        assert stats.locked_shelves > 0
        
        # Start and stop processing (triggers cleanup)
        jobs_processor.start_processing()
        jobs_processor.stop_processing()
        
        # Verify cleanup occurred
        final_stats = jobs_processor.get_processing_stats()
        assert final_stats.active_orders == 0
        assert final_stats.locked_shelves == 0 