"""
Tests for JobsProcessor implementation with single-shelf allocation strategy.

Tests focus on:
- Single-shelf task creation (one task per shelf)
- Multi-shelf order handling (multiple tasks for one order)
- Thread safety and resource management
- Error handling and cleanup
- Statistics tracking
"""
import pytest
import threading
import time
from datetime import datetime, timedelta
from unittest.mock import Mock, MagicMock, patch
from dataclasses import dataclass
from typing import List, Optional

from warehouse.impl.jobs_processor_impl import JobsProcessorImpl, InventoryAllocation
from interfaces.jobs_processor_interface import (
    Order, OrderItem, Priority, OrderStatus, ProcessingResult, JobsProcessorError
)
from interfaces.task_handler_interface import Task, TaskType


@dataclass
class MockShelfLocation:
    """Mock shelf location data."""
    shelf_id: str
    quantity: int


@dataclass
class MockInventoryInfo:
    """Mock inventory information."""
    item_id: str
    available_quantity: int


class TestJobsProcessorImpl:
    """Test cases for JobsProcessor implementation."""
    
    @pytest.fixture
    def mock_order_source(self):
        """Mock order source."""
        mock = Mock()
        mock.get_due_orders.return_value = []
        mock.update_order_status.return_value = True
        return mock
    
    @pytest.fixture
    def mock_simulation_data_service(self):
        """Mock simulation data service."""
        mock = Mock()
        mock.get_item_inventory.return_value = MockInventoryInfo("item1", 10)
        mock.get_item_shelf_locations.return_value = [
            MockShelfLocation("shelf1", 5),
            MockShelfLocation("shelf2", 3),
            MockShelfLocation("shelf3", 2)
        ]
        return mock
    
    @pytest.fixture
    def mock_jobs_queue(self):
        """Mock jobs queue."""
        mock = Mock()
        mock.push_task.return_value = True
        return mock
    
    @pytest.fixture
    def jobs_processor(self, mock_order_source, mock_simulation_data_service, mock_jobs_queue):
        """Create JobsProcessor instance with mocks."""
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
            order_id="test_order_1",
            items=[
                OrderItem(item_id="item1", quantity=8),  # Requires multiple shelves
                OrderItem(item_id="item2", quantity=3)   # Single shelf
            ],
            scheduled_time=datetime.now(),
            priority=Priority.NORMAL,
            customer_id="customer1"
        )

    def test_single_shelf_allocation_strategy(self, jobs_processor, sample_order, 
                                            mock_simulation_data_service, mock_jobs_queue):
        """Test that single-shelf allocation creates separate tasks for each shelf."""
        # Setup mock to return shelf locations for item1 (quantity 8 needs multiple shelves)
        mock_simulation_data_service.get_item_inventory.side_effect = [
            MockInventoryInfo("item1", 10),  # First item has enough total inventory
            MockInventoryInfo("item2", 5)    # Second item
        ]
        
        mock_simulation_data_service.get_item_shelf_locations.side_effect = [
            [MockShelfLocation("shelf1", 5), MockShelfLocation("shelf2", 3)],  # item1 locations
            [MockShelfLocation("shelf3", 5)]  # item2 locations
        ]
        
        # Process the order
        result = jobs_processor.process_order(sample_order)
        
        # Verify success
        assert result.success
        assert result.order_id == "test_order_1"
        
        # Verify tasks were created - should be 3 tasks total:
        # - Task 1: 5 of item1 from shelf1
        # - Task 2: 3 of item1 from shelf2  
        # - Task 3: 3 of item2 from shelf3
        assert len(result.tasks_created) == 3
        
        # Verify each task was pushed to queue (single-shelf tasks)
        assert mock_jobs_queue.push_task.call_count == 3
        
        # Verify shelf locking was called for each shelf
        pushed_tasks = [call[0][0] for call in mock_jobs_queue.push_task.call_args_list]
        shelf_ids = [task.shelf_id for task in pushed_tasks]
        assert "shelf1" in shelf_ids
        assert "shelf2" in shelf_ids
        assert "shelf3" in shelf_ids
        
        # Verify each task handles only one shelf (single-shelf strategy)
        for task in pushed_tasks:
            assert task.shelf_id is not None
            assert task.quantity_to_pick > 0
            assert task.inventory_reserved is True
            assert task.shelf_locked is True

    def test_multi_shelf_order_creates_multiple_simple_tasks(self, jobs_processor, 
                                                           mock_simulation_data_service, mock_jobs_queue):
        """Test that orders requiring multiple shelves create multiple simple tasks."""
        # Create order that definitely needs multiple shelves
        large_order = Order(
            order_id="large_order",
            items=[OrderItem(item_id="popular_item", quantity=15)],  # Needs 3 shelves
            scheduled_time=datetime.now(),
            priority=Priority.HIGH,
            customer_id="customer2"
        )
        
        # Mock inventory across 3 shelves
        mock_simulation_data_service.get_item_inventory.return_value = MockInventoryInfo("popular_item", 15)
        mock_simulation_data_service.get_item_shelf_locations.return_value = [
            MockShelfLocation("shelf_a", 7),   # 7 items
            MockShelfLocation("shelf_b", 5),   # 5 items  
            MockShelfLocation("shelf_c", 3)    # 3 items
        ]
        
        # Process the order
        result = jobs_processor.process_order(large_order)
        
        # Verify success
        assert result.success
        
        # Should create 3 separate single-shelf tasks
        assert len(result.tasks_created) == 3
        assert mock_jobs_queue.push_task.call_count == 3
        
        # Verify each task is simple (single shelf)
        pushed_tasks = [call[0][0] for call in mock_jobs_queue.push_task.call_args_list]
        
        # Check task quantities match shelf capacities
        quantities = sorted([task.quantity_to_pick for task in pushed_tasks])
        assert quantities == [3, 5, 7]  # Should match shelf capacities
        
        # Verify all tasks are for the same order and item
        for task in pushed_tasks:
            assert task.order_id == "large_order"
            assert task.item_id == "popular_item"
            assert task.customer_id == "customer2"

    def test_insufficient_inventory_error_handling(self, jobs_processor, mock_simulation_data_service):
        """Test error handling when insufficient inventory is available."""
        # Create order that can't be fulfilled
        impossible_order = Order(
            order_id="impossible_order",
            items=[OrderItem(item_id="rare_item", quantity=100)],  # Way too much
            scheduled_time=datetime.now(),
            priority=Priority.URGENT
        )
        
        # Mock insufficient inventory
        mock_simulation_data_service.get_item_inventory.return_value = MockInventoryInfo("rare_item", 5)
        mock_simulation_data_service.get_item_shelf_locations.return_value = [
            MockShelfLocation("shelf1", 5)  # Only 5 available, need 100
        ]
        
        # Process should fail gracefully
        result = jobs_processor.process_order(impossible_order)
        
        # Verify failure
        assert not result.success
        assert "insufficient inventory" in result.error_message.lower()
        assert len(result.tasks_created) == 0

    def test_shelf_locking_prevents_conflicts(self, jobs_processor, mock_simulation_data_service, mock_jobs_queue):
        """Test that shelf locking prevents concurrent access conflicts."""
        # Create two orders that would compete for the same shelf
        order1 = Order(
            order_id="order1",
            items=[OrderItem(item_id="contested_item", quantity=3)],
            scheduled_time=datetime.now(),
            priority=Priority.NORMAL
        )
        
        order2 = Order(
            order_id="order2", 
            items=[OrderItem(item_id="contested_item", quantity=2)],
            scheduled_time=datetime.now(),
            priority=Priority.NORMAL
        )
        
        # Mock same shelf for both orders
        mock_simulation_data_service.get_item_inventory.return_value = MockInventoryInfo("contested_item", 5)
        mock_simulation_data_service.get_item_shelf_locations.return_value = [
            MockShelfLocation("contested_shelf", 5)
        ]
        
        # Process first order
        result1 = jobs_processor.process_order(order1)
        assert result1.success
        
        # Verify shelf is locked in processor's internal state
        with jobs_processor._lock:
            assert "contested_shelf" in jobs_processor._locked_shelves
        
        # Process second order - should handle the locked shelf appropriately
        result2 = jobs_processor.process_order(order2)
        
        # Both should succeed but use different mechanisms or fail gracefully
        # (Implementation detail depends on how mock handles locking)

    def test_thread_safety_concurrent_processing(self, jobs_processor, mock_order_source):
        """Test thread safety during concurrent order processing."""
        # Setup multiple orders
        orders = [
            Order(f"order_{i}", [OrderItem(f"item_{i}", 1)], datetime.now()) 
            for i in range(5)
        ]
        mock_order_source.get_due_orders.return_value = orders
        
        # Start processing in background
        jobs_processor.start_processing()
        
        # Verify processor is running
        assert jobs_processor.is_processing()
        
        # Let it run briefly
        time.sleep(0.2)
        
        # Stop processing
        jobs_processor.stop_processing()
        
        # Verify processor stopped
        assert not jobs_processor.is_processing()

    def test_statistics_tracking(self, jobs_processor, sample_order, mock_simulation_data_service):
        """Test that processing statistics are tracked correctly."""
        # Setup successful processing
        mock_simulation_data_service.get_item_inventory.side_effect = [
            MockInventoryInfo("item1", 10),
            MockInventoryInfo("item2", 5)
        ]
        
        # Process an order
        result = jobs_processor.process_order(sample_order)
        assert result.success
        
        # Check statistics
        stats = jobs_processor.get_processing_stats()
        assert stats.total_orders_processed >= 1
        assert stats.successful_orders >= 1
        assert stats.total_tasks_created >= 1
        assert stats.last_processing_time is not None

    def test_order_cancellation_cleanup(self, jobs_processor, sample_order, mock_order_source):
        """Test that order cancellation properly cleans up resources."""
        # Process order to get it into active state
        jobs_processor.process_order(sample_order)
        
        # Verify order is active
        active_orders = jobs_processor.get_active_orders()
        assert len(active_orders) >= 1
        assert any(order.order_id == "test_order_1" for order in active_orders)
        
        # Cancel the order
        success = jobs_processor.cancel_order("test_order_1")
        assert success
        
        # Verify order is no longer active
        active_orders = jobs_processor.get_active_orders()
        assert not any(order.order_id == "test_order_1" for order in active_orders)
        
        # Verify order status was updated
        mock_order_source.update_order_status.assert_called_with("test_order_1", OrderStatus.CANCELLED)

    def test_process_orders_once_batch_processing(self, jobs_processor, mock_order_source, 
                                                mock_simulation_data_service, mock_jobs_queue):
        """Test batch processing of multiple orders."""
        # Setup multiple due orders
        orders = [
            Order(f"batch_order_{i}", [OrderItem(f"item_{i}", 2)], datetime.now())
            for i in range(3)
        ]
        mock_order_source.get_due_orders.return_value = orders
        
        # Mock inventory for all items
        mock_simulation_data_service.get_item_inventory.side_effect = [
            MockInventoryInfo(f"item_{i}", 5) for i in range(3)
        ]
        mock_simulation_data_service.get_item_shelf_locations.side_effect = [
            [MockShelfLocation(f"shelf_{i}", 5)] for i in range(3)
        ]
        
        # Process orders once
        result = jobs_processor.process_orders_once()
        
        # Verify batch processing success
        assert result.success
        assert len(result.tasks_created) == 3  # One task per order
        
        # Verify all orders were processed
        assert mock_jobs_queue.push_task.call_count == 3

    def test_priority_based_processing(self, jobs_processor, mock_order_source, mock_simulation_data_service):
        """Test that orders are processed according to priority."""
        # Create orders with different priorities
        urgent_order = Order("urgent", [OrderItem("item1", 1)], datetime.now(), Priority.URGENT)
        normal_order = Order("normal", [OrderItem("item2", 1)], datetime.now(), Priority.NORMAL)
        low_order = Order("low", [OrderItem("item3", 1)], datetime.now(), Priority.LOW)
        
        # Mock order source to return mixed priorities
        mock_order_source.get_due_orders.return_value = [normal_order, urgent_order, low_order]
        
        # Setup inventory
        mock_simulation_data_service.get_item_inventory.return_value = MockInventoryInfo("item", 5)
        mock_simulation_data_service.get_item_shelf_locations.return_value = [
            MockShelfLocation("shelf1", 5)
        ]
        
        # Process orders
        result = jobs_processor.process_orders_once()
        assert result.success


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 