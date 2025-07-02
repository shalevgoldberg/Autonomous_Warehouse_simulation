"""
Tests for JobsQueue implementation.

Tests focus on:
- Interface compliance (all methods work as specified)
- Thread safety (concurrent access)
- FIFO behavior
- Task tracking and statistics
- Error handling
"""
import pytest
import threading
import time
from datetime import datetime
from unittest.mock import Mock

from warehouse.impl.jobs_queue_impl import JobsQueueImpl
from interfaces.jobs_queue_interface import JobsQueueError
from interfaces.task_handler_interface import Task, TaskType


class TestJobsQueueImpl:
    """Test cases for JobsQueue implementation."""
    
    @pytest.fixture
    def jobs_queue(self):
        """Create a JobsQueue instance for testing."""
        return JobsQueueImpl(max_queue_size=100)
    
    @pytest.fixture
    def sample_task(self):
        """Create a sample task for testing."""
        return Task.create_pick_and_deliver_task(
            order_id="test_order",
            order_priority="normal",
            customer_id="customer_1",
            shelf_id="shelf_A",
            item_id="book_123",
            quantity_to_pick=5,
            dropoff_zone="main_dropoff"
        )
    
    @pytest.fixture
    def multiple_tasks(self):
        """Create multiple tasks for testing."""
        tasks = []
        for i in range(5):
            task = Task.create_pick_and_deliver_task(
                order_id=f"order_{i}",
                order_priority="normal",
                customer_id=f"customer_{i}",
                shelf_id=f"shelf_{i}",
                item_id=f"item_{i}",
                quantity_to_pick=i + 1,
                dropoff_zone="main_dropoff"
            )
            tasks.append(task)
        return tasks

    def test_initialization(self):
        """Test JobsQueue initialization."""
        queue = JobsQueueImpl(max_queue_size=50)
        
        assert queue.get_queue_size() == 0
        assert queue.is_empty()
        assert not queue.is_full()
        assert queue.peek_next_task() is None
        
        stats = queue.get_queue_stats()
        assert stats["current_queue_size"] == 0
        assert stats["max_queue_size"] == 50
        assert stats["total_enqueued"] == 0
        assert stats["total_dequeued"] == 0

    def test_enqueue_task_basic(self, jobs_queue, sample_task):
        """Test basic task enqueuing."""
        # Initially empty
        assert jobs_queue.is_empty()
        
        # Enqueue task
        jobs_queue.enqueue_task(sample_task)
        
        # Verify task is in queue
        assert jobs_queue.get_queue_size() == 1
        assert not jobs_queue.is_empty()
        
        # Verify task can be found
        found_task = jobs_queue.get_task_by_id(sample_task.task_id)
        assert found_task is not None
        assert found_task.task_id == sample_task.task_id

    def test_enqueue_task_validation(self, jobs_queue):
        """Test task enqueuing validation."""
        # Test None task
        with pytest.raises(JobsQueueError, match="Cannot enqueue None task"):
            jobs_queue.enqueue_task(None)
        
        # Test task without task_id
        invalid_task = Task(
            task_id="",  # Empty task_id
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="test"
        )
        with pytest.raises(JobsQueueError, match="Task must have a task_id"):
            jobs_queue.enqueue_task(invalid_task)

    def test_enqueue_duplicate_task_id(self, jobs_queue, sample_task):
        """Test that duplicate task IDs are rejected."""
        # Enqueue first task
        jobs_queue.enqueue_task(sample_task)
        
        # Try to enqueue task with same ID
        duplicate_task = Task.create_pick_and_deliver_task(
            order_id="different_order",
            order_priority="high",
            customer_id="different_customer",
            shelf_id="different_shelf",
            item_id="different_item",
            quantity_to_pick=10,
            dropoff_zone="different_zone"
        )
        duplicate_task.task_id = sample_task.task_id  # Same ID
        
        with pytest.raises(JobsQueueError, match="already exists in queue"):
            jobs_queue.enqueue_task(duplicate_task)

    def test_dequeue_task_basic(self, jobs_queue, sample_task):
        """Test basic task dequeuing."""
        # Enqueue task
        jobs_queue.enqueue_task(sample_task)
        
        # Dequeue task
        dequeued_task = jobs_queue.dequeue_task("robot_1")
        
        # Verify correct task returned
        assert dequeued_task is not None
        assert dequeued_task.task_id == sample_task.task_id
        
        # Verify queue is empty
        assert jobs_queue.is_empty()
        assert jobs_queue.get_queue_size() == 0
        
        # Verify task is no longer findable in queue
        assert jobs_queue.get_task_by_id(sample_task.task_id) is None

    def test_dequeue_task_tracking(self, jobs_queue, sample_task):
        """Test that dequeued tasks are tracked."""
        # Enqueue and dequeue task
        jobs_queue.enqueue_task(sample_task)
        dequeued_task = jobs_queue.dequeue_task("robot_alpha")
        
        # Verify task assignment tracking
        assigned_tasks = jobs_queue.get_assigned_tasks()
        assert sample_task.task_id in assigned_tasks
        assert assigned_tasks[sample_task.task_id] == "robot_alpha"

    def test_dequeue_empty_queue(self, jobs_queue):
        """Test dequeuing from empty queue."""
        # Queue is empty
        assert jobs_queue.is_empty()
        
        # Dequeue should return None
        result = jobs_queue.dequeue_task("robot_1")
        assert result is None

    def test_dequeue_invalid_robot_id(self, jobs_queue, sample_task):
        """Test dequeue with invalid robot ID."""
        jobs_queue.enqueue_task(sample_task)
        
        # Empty robot ID should raise error
        with pytest.raises(JobsQueueError, match="robot_id cannot be empty"):
            jobs_queue.dequeue_task("")

    def test_fifo_behavior(self, jobs_queue, multiple_tasks):
        """Test that queue follows FIFO (First In, First Out) behavior."""
        # Enqueue tasks in order
        for task in multiple_tasks:
            jobs_queue.enqueue_task(task)
        
        # Dequeue tasks and verify order
        dequeued_order = []
        while not jobs_queue.is_empty():
            task = jobs_queue.dequeue_task("robot_test")
            dequeued_order.append(task.task_id)
        
        # Should match original order
        expected_order = [task.task_id for task in multiple_tasks]
        assert dequeued_order == expected_order

    def test_peek_next_task(self, jobs_queue, multiple_tasks):
        """Test peeking at next task without removing it."""
        # Empty queue
        assert jobs_queue.peek_next_task() is None
        
        # Add tasks
        for task in multiple_tasks:
            jobs_queue.enqueue_task(task)
        
        # Peek should return first task
        peeked_task = jobs_queue.peek_next_task()
        assert peeked_task is not None
        assert peeked_task.task_id == multiple_tasks[0].task_id
        
        # Queue size should be unchanged
        assert jobs_queue.get_queue_size() == len(multiple_tasks)
        
        # Peeking multiple times should return same task
        peeked_again = jobs_queue.peek_next_task()
        assert peeked_again.task_id == peeked_task.task_id

    def test_get_pending_tasks(self, jobs_queue, multiple_tasks):
        """Test getting all pending tasks."""
        # Add tasks
        for task in multiple_tasks:
            jobs_queue.enqueue_task(task)
        
        # Get pending tasks
        pending = jobs_queue.get_pending_tasks()
        
        # Should return all tasks in order
        assert len(pending) == len(multiple_tasks)
        for i, task in enumerate(pending):
            assert task.task_id == multiple_tasks[i].task_id

    def test_remove_task(self, jobs_queue, multiple_tasks):
        """Test removing specific task from queue."""
        # Add tasks
        for task in multiple_tasks:
            jobs_queue.enqueue_task(task)
        
        # Remove middle task
        middle_task = multiple_tasks[2]
        result = jobs_queue.remove_task(middle_task.task_id)
        assert result is True
        
        # Verify task is removed
        assert jobs_queue.get_queue_size() == len(multiple_tasks) - 1
        assert jobs_queue.get_task_by_id(middle_task.task_id) is None
        
        # Verify other tasks remain
        remaining_tasks = jobs_queue.get_pending_tasks()
        remaining_ids = [task.task_id for task in remaining_tasks]
        assert middle_task.task_id not in remaining_ids

    def test_remove_nonexistent_task(self, jobs_queue):
        """Test removing task that doesn't exist."""
        result = jobs_queue.remove_task("nonexistent_task")
        assert result is False
        
        # Empty task_id
        result = jobs_queue.remove_task("")
        assert result is False

    def test_clear_queue(self, jobs_queue, multiple_tasks):
        """Test clearing all tasks from queue."""
        # Add tasks
        for task in multiple_tasks:
            jobs_queue.enqueue_task(task)
        
        assert jobs_queue.get_queue_size() == len(multiple_tasks)
        
        # Clear queue
        jobs_queue.clear_queue()
        
        # Verify queue is empty
        assert jobs_queue.is_empty()
        assert jobs_queue.get_queue_size() == 0
        assert jobs_queue.get_pending_tasks() == []

    def test_queue_size_limit(self, jobs_queue):
        """Test queue size limit enforcement."""
        # Create small queue
        small_queue = JobsQueueImpl(max_queue_size=2)
        
        # Add tasks up to limit
        task1 = Task.create_pick_and_deliver_task(
            order_id="order1", order_priority="normal", customer_id="c1",
            shelf_id="s1", item_id="i1", quantity_to_pick=1, dropoff_zone="zone1"
        )
        task2 = Task.create_pick_and_deliver_task(
            order_id="order2", order_priority="normal", customer_id="c2",
            shelf_id="s2", item_id="i2", quantity_to_pick=1, dropoff_zone="zone2"
        )
        
        small_queue.enqueue_task(task1)
        small_queue.enqueue_task(task2)
        assert small_queue.is_full()
        
        # Try to add one more - should fail
        task3 = Task.create_pick_and_deliver_task(
            order_id="order3", order_priority="normal", customer_id="c3",
            shelf_id="s3", item_id="i3", quantity_to_pick=1, dropoff_zone="zone3"
        )
        
        with pytest.raises(JobsQueueError, match="Queue is full"):
            small_queue.enqueue_task(task3)

    def test_get_tasks_by_order(self, jobs_queue):
        """Test getting tasks by order ID."""
        # Create tasks for different orders
        order1_tasks = []
        order2_tasks = []
        
        for i in range(3):
            task1 = Task.create_pick_and_deliver_task(
                order_id="order_1", order_priority="normal", customer_id="c1",
                shelf_id=f"shelf_{i}", item_id=f"item_{i}", quantity_to_pick=1,
                dropoff_zone="zone1"
            )
            task2 = Task.create_pick_and_deliver_task(
                order_id="order_2", order_priority="high", customer_id="c2",
                shelf_id=f"shelf_{i+10}", item_id=f"item_{i+10}", quantity_to_pick=2,
                dropoff_zone="zone2"
            )
            order1_tasks.append(task1)
            order2_tasks.append(task2)
            jobs_queue.enqueue_task(task1)
            jobs_queue.enqueue_task(task2)
        
        # Get tasks by order
        found_order1 = jobs_queue.get_tasks_by_order("order_1")
        found_order2 = jobs_queue.get_tasks_by_order("order_2")
        
        assert len(found_order1) == 3
        assert len(found_order2) == 3
        
        # Verify correct tasks returned
        order1_ids = [task.task_id for task in found_order1]
        order2_ids = [task.task_id for task in found_order2]
        
        for task in order1_tasks:
            assert task.task_id in order1_ids
        for task in order2_tasks:
            assert task.task_id in order2_ids

    def test_thread_safety_concurrent_enqueue_dequeue(self, jobs_queue):
        """Test thread safety with concurrent enqueue/dequeue operations."""
        num_tasks = 50
        num_robots = 5
        
        # Create tasks
        tasks = []
        for i in range(num_tasks):
            task = Task.create_pick_and_deliver_task(
                order_id=f"order_{i}", order_priority="normal", customer_id=f"c_{i}",
                shelf_id=f"shelf_{i}", item_id=f"item_{i}", quantity_to_pick=1,
                dropoff_zone="main_zone"
            )
            tasks.append(task)
        
        # Enqueue tasks in separate thread
        def enqueue_worker():
            for task in tasks:
                jobs_queue.enqueue_task(task)
                time.sleep(0.001)  # Small delay to increase chance of race conditions
        
        # Dequeue tasks in separate threads
        dequeued_tasks = []
        dequeue_lock = threading.Lock()
        
        def dequeue_worker(robot_id):
            while True:
                task = jobs_queue.dequeue_task(robot_id)
                if task is None:
                    time.sleep(0.001)
                    # Check if enqueuing is done and queue is empty
                    if not enqueue_thread.is_alive() and jobs_queue.is_empty():
                        break
                    continue
                
                with dequeue_lock:
                    dequeued_tasks.append(task)
        
        # Start threads
        enqueue_thread = threading.Thread(target=enqueue_worker)
        dequeue_threads = []
        
        for i in range(num_robots):
            thread = threading.Thread(target=dequeue_worker, args=[f"robot_{i}"])
            dequeue_threads.append(thread)
        
        # Start all threads
        enqueue_thread.start()
        for thread in dequeue_threads:
            thread.start()
        
        # Wait for completion
        enqueue_thread.join()
        for thread in dequeue_threads:
            thread.join()
        
        # Verify all tasks were processed
        assert len(dequeued_tasks) == num_tasks
        
        # Verify no duplicates
        task_ids = [task.task_id for task in dequeued_tasks]
        assert len(set(task_ids)) == num_tasks
        
        # Verify queue is empty
        assert jobs_queue.is_empty()

    def test_queue_statistics(self, jobs_queue, multiple_tasks):
        """Test queue statistics tracking."""
        initial_stats = jobs_queue.get_queue_stats()
        assert initial_stats["total_enqueued"] == 0
        assert initial_stats["total_dequeued"] == 0
        
        # Enqueue tasks
        for task in multiple_tasks:
            jobs_queue.enqueue_task(task)
        
        stats_after_enqueue = jobs_queue.get_queue_stats()
        assert stats_after_enqueue["total_enqueued"] == len(multiple_tasks)
        assert stats_after_enqueue["current_queue_size"] == len(multiple_tasks)
        
        # Dequeue some tasks
        jobs_queue.dequeue_task("robot_1")
        jobs_queue.dequeue_task("robot_2")
        
        final_stats = jobs_queue.get_queue_stats()
        assert final_stats["total_enqueued"] == len(multiple_tasks)
        assert final_stats["total_dequeued"] == 2
        assert final_stats["current_queue_size"] == len(multiple_tasks) - 2

    def test_string_representations(self, jobs_queue, sample_task):
        """Test string and repr methods."""
        # Empty queue
        str_empty = str(jobs_queue)
        assert "size=0" in str_empty
        assert "enqueued=0" in str_empty
        assert "dequeued=0" in str_empty
        
        # With task
        jobs_queue.enqueue_task(sample_task)
        str_with_task = str(jobs_queue)
        assert "size=1" in str_with_task
        assert "enqueued=1" in str_with_task
        
        # repr should be same as str
        assert repr(jobs_queue) == str(jobs_queue)

    def test_python_builtin_support(self, jobs_queue, multiple_tasks):
        """Test support for Python built-in functions."""
        # Empty queue
        assert len(jobs_queue) == 0
        assert not bool(jobs_queue)
        
        # With tasks
        for task in multiple_tasks:
            jobs_queue.enqueue_task(task)
        
        assert len(jobs_queue) == len(multiple_tasks)
        assert bool(jobs_queue)


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 