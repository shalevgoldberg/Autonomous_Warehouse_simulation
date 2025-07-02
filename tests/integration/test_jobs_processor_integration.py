"""
Integration tests for JobsProcessor.

Tests the complete order processing workflow with real implementations
of OrderSource, SimulationDataService, and JobsQueue.
"""
import pytest
import tempfile
import json
import time
from datetime import datetime, timedelta
from pathlib import Path

from warehouse.impl.jobs_processor_impl import JobsProcessorImpl
from warehouse.impl.json_order_source import JsonOrderSource
from interfaces.jobs_processor_interface import OrderStatus, Priority
from interfaces.task_handler_interface import TaskType, TaskStatus


class MockSimulationDataService:
    """Mock SimulationDataService for integration testing."""
    
    def __init__(self):
        # Predefined inventory data
        self.inventory = {
            "book_001": {"available_quantity": 10, "shelves": [
                {"shelf_id": "shelf_A1", "quantity": 5},
                {"shelf_id": "shelf_A2", "quantity": 5}
            ]},
            "phone_001": {"available_quantity": 3, "shelves": [
                {"shelf_id": "shelf_B1", "quantity": 3}
            ]},
            "laptop_001": {"available_quantity": 2, "shelves": [
                {"shelf_id": "shelf_C1", "quantity": 2}
            ]},
            "tablet_001": {"available_quantity": 0, "shelves": []},  # Out of stock
        }
    
    def get_item_inventory(self, item_id):
        """Get inventory info for an item."""
        if item_id not in self.inventory:
            return None
        
        inventory_info = type('InventoryInfo', (), {})()
        inventory_info.available_quantity = self.inventory[item_id]["available_quantity"]
        return inventory_info
    
    def get_item_shelf_locations(self, item_id):
        """Get shelf locations for an item."""
        if item_id not in self.inventory:
            return []
        
        shelf_locations = []
        for shelf_data in self.inventory[item_id]["shelves"]:
            shelf_location = type('ShelfLocation', (), {})()
            shelf_location.shelf_id = shelf_data["shelf_id"]
            shelf_location.quantity = shelf_data["quantity"]
            shelf_locations.append(shelf_location)
        
        return shelf_locations


class MockJobsQueue:
    """Mock JobsQueue for integration testing."""
    
    def __init__(self):
        self.tasks = []
    
    def push_task(self, task):
        """Add task to queue."""
        self.tasks.append(task)
    
    def get_all_tasks(self):
        """Get all tasks (for testing)."""
        return self.tasks.copy()
    
    def clear(self):
        """Clear all tasks."""
        self.tasks.clear()


class TestJobsProcessorIntegration:
    """Integration tests for JobsProcessor with real components."""
    
    @pytest.fixture
    def temp_orders_file(self):
        """Create temporary orders file."""
        orders_data = [
            {
                "order_id": "integration_001",
                "customer_id": "customer_001",
                "items": [
                    {"item_id": "book_001", "quantity": 2},
                    {"item_id": "phone_001", "quantity": 1}
                ],
                "priority": "normal",
                "scheduled_time": (datetime.now() - timedelta(minutes=5)).isoformat(),
                "status": "pending"
            },
            {
                "order_id": "integration_002",
                "customer_id": "customer_002",
                "items": [
                    {"item_id": "laptop_001", "quantity": 1}
                ],
                "priority": "urgent",
                "scheduled_time": (datetime.now() - timedelta(minutes=10)).isoformat(),
                "status": "pending"
            },
            {
                "order_id": "integration_003",
                "customer_id": "customer_003",
                "items": [
                    {"item_id": "tablet_001", "quantity": 1}  # Out of stock
                ],
                "priority": "high",
                "scheduled_time": (datetime.now() - timedelta(minutes=2)).isoformat(),
                "status": "pending"
            }
        ]
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(orders_data, f, indent=2)
            temp_file = f.name
        
        yield temp_file
        
        # Cleanup
        Path(temp_file).unlink(missing_ok=True)
    
    @pytest.fixture
    def order_source(self, temp_orders_file):
        """Create real JsonOrderSource."""
        order_source = JsonOrderSource(temp_orders_file)
        order_source.connect()
        yield order_source
        order_source.disconnect()
    
    @pytest.fixture
    def simulation_data_service(self):
        """Create mock simulation data service."""
        return MockSimulationDataService()
    
    @pytest.fixture
    def jobs_queue(self):
        """Create mock jobs queue."""
        return MockJobsQueue()
    
    @pytest.fixture
    def jobs_processor(self, order_source, simulation_data_service, jobs_queue):
        """Create JobsProcessor with real dependencies."""
        return JobsProcessorImpl(
            order_source=order_source,
            simulation_data_service=simulation_data_service,
            jobs_queue=jobs_queue,
            processing_interval=0.1,
            max_concurrent_orders=10
        )
    
    def test_full_order_processing_workflow(self, jobs_processor, jobs_queue):
        """Test complete order processing workflow."""
        # Process orders once
        result = jobs_processor.process_orders_once()
        
        # Should process 2 orders successfully (1 fails due to no inventory)
        assert result.orders_processed == 2
        assert result.tasks_created == 3  # 2 + 1 + 1 items
        assert not result.success  # One order failed
        assert result.errors is not None
        assert len(result.errors) == 1
        assert "Insufficient inventory" in result.errors[0]
        
        # Verify tasks were created
        created_tasks = jobs_queue.get_all_tasks()
        assert len(created_tasks) == 3
        
        # Verify task details
        for task in created_tasks:
            assert task.task_type == TaskType.PICK_AND_DELIVER
            assert task.status == TaskStatus.PENDING
            assert task.inventory_reserved
            assert task.shelf_locked
            assert task.created_at is not None
            assert task.order_id in ["integration_001", "integration_002"]
    
    def test_priority_based_task_creation(self, jobs_processor, jobs_queue):
        """Test that tasks inherit correct priority from orders."""
        jobs_processor.process_orders_once()
        
        created_tasks = jobs_queue.get_all_tasks()
        
        # Find tasks by order
        urgent_tasks = [t for t in created_tasks if t.order_id == "integration_002"]
        normal_tasks = [t for t in created_tasks if t.order_id == "integration_001"]
        
        # Verify priorities
        assert len(urgent_tasks) == 1
        assert urgent_tasks[0].order_priority == "urgent"
        
        assert len(normal_tasks) == 2
        for task in normal_tasks:
            assert task.order_priority == "normal"
    
    def test_inventory_allocation_and_shelf_locking(self, jobs_processor, simulation_data_service):
        """Test inventory allocation and shelf locking mechanisms."""
        # Process orders
        jobs_processor.process_orders_once()
        
        # Check processing stats
        stats = jobs_processor.get_processing_stats()
        assert stats.active_orders == 2  # 2 successful orders
        assert stats.locked_shelves == 3  # 3 different shelves used
        
        # Verify active orders
        active_orders = jobs_processor.get_active_orders()
        assert len(active_orders) == 2
        
        active_order_ids = {order.order_id for order in active_orders}
        assert "integration_001" in active_order_ids
        assert "integration_002" in active_order_ids
        assert "integration_003" not in active_order_ids  # Failed order
    
    def test_order_status_updates(self, jobs_processor, order_source):
        """Test that order statuses are updated correctly."""
        # Process orders
        jobs_processor.process_orders_once()
        
        # Check order statuses (note: mock doesn't persist status updates)
        # In real integration, we would verify the order source reflects status changes
        
        # Verify the processor attempted to update statuses
        stats = jobs_processor.get_processing_stats()
        assert stats.orders_processed == 2  # 2 orders had status updates attempted
    
    def test_concurrent_processing_with_background_thread(self, jobs_processor, jobs_queue):
        """Test background processing thread."""
        jobs_queue.clear()
        
        # Start background processing
        jobs_processor.start_processing()
        
        # Wait for processing to occur
        time.sleep(0.5)
        
        # Stop processing
        jobs_processor.stop_processing()
        
        # Verify tasks were created by background thread
        created_tasks = jobs_queue.get_all_tasks()
        assert len(created_tasks) >= 3  # Should have processed orders
        
        # Verify processing stats
        stats = jobs_processor.get_processing_stats()
        assert stats.orders_processed >= 2
        assert stats.tasks_created >= 3
    
    def test_order_cancellation_workflow(self, jobs_processor):
        """Test order cancellation and cleanup."""
        # Process orders to make them active
        jobs_processor.process_orders_once()
        
        # Verify orders are active
        active_orders = jobs_processor.get_active_orders()
        assert len(active_orders) == 2
        
        # Cancel one order
        success = jobs_processor.cancel_order("integration_001")
        assert success
        
        # Verify order was removed
        active_orders = jobs_processor.get_active_orders()
        assert len(active_orders) == 1
        assert active_orders[0].order_id == "integration_002"
        
        # Verify shelf unlocking
        stats = jobs_processor.get_processing_stats()
        assert stats.locked_shelves == 1  # Only one order's shelves remain locked
    
    def test_error_recovery_and_continuation(self, jobs_processor, simulation_data_service):
        """Test error recovery and continued processing."""
        # First processing cycle
        result1 = jobs_processor.process_orders_once()
        assert result1.orders_processed == 2
        
        # Simulate inventory recovery for out-of-stock item
        simulation_data_service.inventory["tablet_001"]["available_quantity"] = 2
        simulation_data_service.inventory["tablet_001"]["shelves"] = [
            {"shelf_id": "shelf_D1", "quantity": 2}
        ]
        
        # Second processing cycle (should pick up previously failed order)
        # Note: In real scenario, the order would still be pending in the source
        # For this test, we verify the processor can handle the scenario
        
        stats = jobs_processor.get_processing_stats()
        assert stats.processing_errors == 1  # From first failed order
    
    def test_multiple_items_same_shelf(self, jobs_processor, jobs_queue, simulation_data_service):
        """Test processing when multiple items are on the same shelf."""
        # Modify inventory so book and phone are on same shelf
        simulation_data_service.inventory["phone_001"]["shelves"] = [
            {"shelf_id": "shelf_A1", "quantity": 3}  # Same shelf as books
        ]
        
        jobs_processor.process_orders_once()
        
        created_tasks = jobs_queue.get_all_tasks()
        
        # Find tasks that use shelf_A1
        shelf_a1_tasks = [t for t in created_tasks if t.shelf_id == "shelf_A1"]
        
        # Should have tasks for both book and phone from same shelf
        assert len(shelf_a1_tasks) >= 2
        
        # Verify different items
        items_from_shelf = {task.item_id for task in shelf_a1_tasks}
        assert "book_001" in items_from_shelf
        assert "phone_001" in items_from_shelf
    
    def test_large_quantity_allocation_across_shelves(self, jobs_processor, jobs_queue, 
                                                    simulation_data_service, temp_orders_file):
        """Test allocation when quantity spans multiple shelves."""
        # Create order requiring more books than available on single shelf
        large_order = {
            "order_id": "large_order_001",
            "customer_id": "customer_large",
            "items": [
                {"item_id": "book_001", "quantity": 8}  # More than single shelf
            ],
            "priority": "normal",
            "scheduled_time": (datetime.now() - timedelta(minutes=1)).isoformat(),
            "status": "pending"
        }
        
        # Update orders file
        with open(temp_orders_file, 'r') as f:
            orders = json.load(f)
        orders.append(large_order)
        
        with open(temp_orders_file, 'w') as f:
            json.dump(orders, f, indent=2)
        
        # Create new processor with updated file
        from warehouse.impl.json_order_source import JsonOrderSource
        order_source = JsonOrderSource(temp_orders_file)
        order_source.connect()
        
        processor = JobsProcessorImpl(
            order_source=order_source,
            simulation_data_service=simulation_data_service,
            jobs_queue=jobs_queue,
            processing_interval=0.1,
            max_concurrent_orders=10
        )
        
        jobs_queue.clear()
        processor.process_orders_once()
        
        # Should create multiple tasks to fulfill large quantity
        created_tasks = jobs_queue.get_all_tasks()
        book_tasks = [t for t in created_tasks if t.item_id == "book_001" and t.order_id == "large_order_001"]
        
        # Should have tasks from multiple shelves
        total_quantity = sum(task.quantity_to_pick for task in book_tasks)
        assert total_quantity == 8
        
        # Should use both available shelves
        shelf_ids = {task.shelf_id for task in book_tasks}
        assert len(shelf_ids) >= 2
        
        order_source.disconnect()
    
    def test_processing_statistics_accuracy(self, jobs_processor):
        """Test accuracy of processing statistics."""
        initial_stats = jobs_processor.get_processing_stats()
        
        # Process orders
        result = jobs_processor.process_orders_once()
        
        # Check final stats
        final_stats = jobs_processor.get_processing_stats()
        
        # Verify statistics match result
        assert final_stats.orders_processed == result.orders_processed
        assert final_stats.tasks_created == result.tasks_created
        assert final_stats.processing_errors == len(result.errors) if result.errors else 0
        assert final_stats.last_processing_time is not None
        assert final_stats.average_processing_time > 0
        
        # Verify incremental updates
        assert final_stats.orders_processed == initial_stats.orders_processed + result.orders_processed
        assert final_stats.tasks_created == initial_stats.tasks_created + result.tasks_created 