"""
Unit tests for enhanced Task structure.

Tests the new order context, inventory details, and coordination features
added to support the JobsProcessor workflow.
"""
import pytest
from datetime import datetime, timedelta
from interfaces.task_handler_interface import (
    Task, TaskType, TaskStatus, TaskHandlingError
)


class TestEnhancedTask:
    """Test enhanced Task structure with order context."""
    
    def test_basic_task_creation(self):
        """Test creating a basic task with required fields."""
        task = Task(
            task_id="test_task_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_123",
            shelf_id="shelf_A1",
            item_id="book_001"
        )
        
        assert task.task_id == "test_task_001"
        assert task.task_type == TaskType.PICK_AND_DELIVER
        assert task.order_id == "order_123"
        assert task.shelf_id == "shelf_A1"
        assert task.item_id == "book_001"
        
        # Check defaults
        assert task.order_priority == "normal"
        assert task.customer_id is None
        assert task.quantity_to_pick == 1
        assert task.dropoff_zone == "default_dropoff"
        assert task.status == TaskStatus.PENDING
        assert not task.inventory_reserved
        assert not task.shelf_locked
    
    def test_full_order_task_creation(self):
        """Test creating a complete order-based task."""
        task = Task(
            task_id="task_002",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_456",
            shelf_id="shelf_B2",
            item_id="phone_001",
            order_priority="urgent",
            customer_id="customer_789",
            quantity_to_pick=3,
            dropoff_zone="express_dropoff",
            estimated_duration=120.5,
            inventory_reserved=True,
            shelf_locked=True
        )
        
        assert task.order_priority == "urgent"
        assert task.customer_id == "customer_789"
        assert task.quantity_to_pick == 3
        assert task.dropoff_zone == "express_dropoff"
        assert task.estimated_duration == 120.5
        assert task.inventory_reserved
        assert task.shelf_locked
    
    def test_factory_method_create_pick_and_deliver(self):
        """Test the factory method for creating pick and deliver tasks."""
        task = Task.create_pick_and_deliver_task(
            task_id="factory_001",
            order_id="order_factory",
            shelf_id="shelf_C3",
            item_id="laptop_001",
            quantity_to_pick=2,
            order_priority="high",
            customer_id="vip_customer",
            dropoff_zone="priority_zone"
        )
        
        assert task.task_id == "factory_001"
        assert task.task_type == TaskType.PICK_AND_DELIVER
        assert task.order_id == "order_factory"
        assert task.shelf_id == "shelf_C3"
        assert task.item_id == "laptop_001"
        assert task.quantity_to_pick == 2
        assert task.order_priority == "high"
        assert task.customer_id == "vip_customer"
        assert task.dropoff_zone == "priority_zone"
    
    def test_factory_method_defaults(self):
        """Test factory method with default values."""
        task = Task.create_pick_and_deliver_task(
            task_id="factory_002",
            order_id="order_default",
            shelf_id="shelf_D4",
            item_id="tablet_001"
        )
        
        assert task.quantity_to_pick == 1
        assert task.order_priority == "normal"
        assert task.customer_id is None
        assert task.dropoff_zone == "default_dropoff"
    
    def test_system_task_creation(self):
        """Test creating system tasks (non-order tasks)."""
        # Move to charging task
        charging_task = Task(
            task_id="charging_001",
            task_type=TaskType.MOVE_TO_CHARGING
        )
        
        # Should auto-generate system order_id
        assert charging_task.order_id == "system_move_to_charging_charging_001"
        assert charging_task.shelf_id == ""
        assert charging_task.item_id == ""
        
        # Move to position task
        position_task = Task(
            task_id="position_001",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(10.0, 20.0)
        )
        
        assert position_task.order_id == "system_move_to_position_position_001"
        assert position_task.target_position == (10.0, 20.0)
    
    def test_validation_pick_and_deliver_missing_shelf(self):
        """Test validation fails when PICK_AND_DELIVER task missing shelf_id."""
        with pytest.raises(ValueError, match="PICK_AND_DELIVER tasks require shelf_id"):
            Task(
                task_id="invalid_001",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id="order_123",
                shelf_id="",  # Empty shelf_id
                item_id="book_001"
            )
    
    def test_validation_pick_and_deliver_missing_item(self):
        """Test validation fails when PICK_AND_DELIVER task missing item_id."""
        with pytest.raises(ValueError, match="PICK_AND_DELIVER tasks require item_id"):
            Task(
                task_id="invalid_002",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id="order_123",
                shelf_id="shelf_A1",
                item_id=""  # Empty item_id
            )
    
    def test_validation_pick_and_deliver_missing_order(self):
        """Test validation fails when PICK_AND_DELIVER task missing order_id."""
        with pytest.raises(ValueError, match="PICK_AND_DELIVER tasks require order_id"):
            Task(
                task_id="invalid_003",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id="",  # Empty order_id
                shelf_id="shelf_A1",
                item_id="book_001"
            )
    
    def test_validation_negative_quantity(self):
        """Test validation fails for negative quantity."""
        with pytest.raises(ValueError, match="quantity_to_pick must be positive"):
            Task(
                task_id="invalid_004",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id="order_123",
                shelf_id="shelf_A1",
                item_id="book_001",
                quantity_to_pick=-1
            )
    
    def test_validation_zero_quantity(self):
        """Test validation fails for zero quantity."""
        with pytest.raises(ValueError, match="quantity_to_pick must be positive"):
            Task(
                task_id="invalid_005",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id="order_123",
                shelf_id="shelf_A1",
                item_id="book_001",
                quantity_to_pick=0
            )
    
    def test_validation_move_to_position_missing_target(self):
        """Test validation fails when MOVE_TO_POSITION task missing target_position."""
        with pytest.raises(ValueError, match="MOVE_TO_POSITION tasks require target_position"):
            Task(
                task_id="invalid_006",
                task_type=TaskType.MOVE_TO_POSITION,
                target_position=None
            )
    
    def test_timestamps_auto_creation(self):
        """Test that created_at timestamp is automatically set."""
        before_creation = datetime.now()
        task = Task(
            task_id="time_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_time",
            shelf_id="shelf_T1",
            item_id="time_item"
        )
        after_creation = datetime.now()
        
        assert before_creation <= task.created_at <= after_creation
        assert task.assigned_at is None
        assert task.completed_at is None
    
    def test_task_lifecycle_timestamps(self):
        """Test setting lifecycle timestamps."""
        task = Task(
            task_id="lifecycle_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_lifecycle",
            shelf_id="shelf_L1",
            item_id="lifecycle_item"
        )
        
        # Simulate task assignment
        assignment_time = datetime.now()
        task.assigned_at = assignment_time
        task.status = TaskStatus.IN_PROGRESS
        
        assert task.assigned_at == assignment_time
        assert task.status == TaskStatus.IN_PROGRESS
        
        # Simulate task completion
        completion_time = datetime.now()
        task.completed_at = completion_time
        task.status = TaskStatus.COMPLETED
        
        assert task.completed_at == completion_time
        assert task.status == TaskStatus.COMPLETED
    
    def test_coordination_flags(self):
        """Test inventory and shelf coordination flags."""
        task = Task(
            task_id="coord_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_coord",
            shelf_id="shelf_C1",
            item_id="coord_item"
        )
        
        # Initially false
        assert not task.inventory_reserved
        assert not task.shelf_locked
        
        # Set coordination flags
        task.inventory_reserved = True
        task.shelf_locked = True
        
        assert task.inventory_reserved
        assert task.shelf_locked
    
    def test_metadata_handling(self):
        """Test metadata field functionality."""
        metadata = {
            "priority_reason": "customer_vip",
            "special_instructions": "handle_with_care",
            "estimated_weight": 2.5,
            "fragile": True
        }
        
        task = Task(
            task_id="meta_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_meta",
            shelf_id="shelf_M1",
            item_id="meta_item",
            metadata=metadata
        )
        
        assert task.metadata == metadata
        assert task.metadata["priority_reason"] == "customer_vip"
        assert task.metadata["fragile"] is True
        assert task.metadata["estimated_weight"] == 2.5
    
    def test_legacy_priority_field(self):
        """Test legacy priority field still works."""
        task = Task(
            task_id="legacy_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_legacy",
            shelf_id="shelf_L1",
            item_id="legacy_item",
            priority=8  # Legacy 0-10 scale
        )
        
        assert task.priority == 8
        assert task.order_priority == "normal"  # New priority system
    
    def test_order_priority_levels(self):
        """Test different order priority levels."""
        priorities = ["urgent", "high", "normal", "low"]
        
        for priority in priorities:
            task = Task(
                task_id=f"priority_{priority}",
                task_type=TaskType.PICK_AND_DELIVER,
                order_id=f"order_{priority}",
                shelf_id="shelf_P1",
                item_id="priority_item",
                order_priority=priority
            )
            
            assert task.order_priority == priority
    
    def test_task_string_representation(self):
        """Test that task has reasonable string representation."""
        task = Task(
            task_id="repr_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_repr",
            shelf_id="shelf_R1",
            item_id="repr_item",
            quantity_to_pick=2
        )
        
        task_str = str(task)
        assert "repr_001" in task_str
        assert "PICK_AND_DELIVER" in task_str
        assert "order_repr" in task_str
    
    def test_task_equality(self):
        """Test task equality based on task_id."""
        task1 = Task(
            task_id="equal_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_1",
            shelf_id="shelf_1",
            item_id="item_1"
        )
        
        task2 = Task(
            task_id="equal_001",  # Same task_id
            task_type=TaskType.MOVE_TO_CHARGING,  # Different type
            order_id="order_2",  # Different order
            shelf_id="shelf_2",  # Different shelf
            item_id="item_2"  # Different item
        )
        
        task3 = Task(
            task_id="equal_002",  # Different task_id
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_1",
            shelf_id="shelf_1",
            item_id="item_1"
        )
        
        # Tasks with same task_id should be equal
        assert task1 == task2
        # Tasks with different task_id should not be equal
        assert task1 != task3
    
    def test_task_hash(self):
        """Test task hashing for use in sets and dicts."""
        task1 = Task(
            task_id="hash_001",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_hash",
            shelf_id="shelf_H1",
            item_id="hash_item"
        )
        
        task2 = Task(
            task_id="hash_001",  # Same task_id
            task_type=TaskType.MOVE_TO_CHARGING,
            order_id="different_order",
            shelf_id="different_shelf",
            item_id="different_item"
        )
        
        # Should be able to use in sets
        task_set = {task1, task2}
        assert len(task_set) == 1  # Same task_id, so only one in set
        
        # Should be able to use as dict keys
        task_dict = {task1: "value1", task2: "value2"}
        assert len(task_dict) == 1  # Same task_id, so overwrites
        assert task_dict[task1] == "value2"  # Last value wins 