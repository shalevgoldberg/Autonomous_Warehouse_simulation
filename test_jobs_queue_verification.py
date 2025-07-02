"""
JobsQueue Implementation Verification Test.

This test verifies that the JobsQueue implementation works correctly
and follows the interface specification exactly.
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import threading
import time
from warehouse.impl.jobs_queue_impl import JobsQueueImpl
from interfaces.jobs_queue_interface import JobsQueueError
from interfaces.task_handler_interface import Task


def test_basic_functionality():
    """Test basic JobsQueue functionality."""
    print("Testing basic JobsQueue functionality...")
    
    # Create queue
    queue = JobsQueueImpl(max_queue_size=10)
    print("✓ JobsQueue created successfully")
    
    # Test initial state
    assert queue.get_queue_size() == 0
    assert queue.is_empty()
    assert not queue.is_full()
    assert queue.peek_next_task() is None
    print("✓ Initial state is correct")
    
    # Create test task
    task = Task.create_pick_and_deliver_task(
        order_id="test_order",
        order_priority="normal",
        customer_id="customer_1",
        shelf_id="shelf_A",
        item_id="book_123",
        quantity_to_pick=5,
        dropoff_zone="main_dropoff"
    )
    print("✓ Test task created")
    
    # Test enqueue
    queue.enqueue_task(task)
    assert queue.get_queue_size() == 1
    assert not queue.is_empty()
    print("✓ Task enqueued successfully")
    
    # Test peek
    peeked = queue.peek_next_task()
    assert peeked is not None
    assert peeked.task_id == task.task_id
    assert queue.get_queue_size() == 1  # Should not change size
    print("✓ Peek functionality works")
    
    # Test dequeue
    dequeued = queue.dequeue_task("robot_1")
    assert dequeued is not None
    assert dequeued.task_id == task.task_id
    assert queue.get_queue_size() == 0
    assert queue.is_empty()
    print("✓ Task dequeued successfully")
    
    return True


def test_fifo_behavior():
    """Test FIFO (First In, First Out) behavior."""
    print("\nTesting FIFO behavior...")
    
    queue = JobsQueueImpl()
    
    # Create multiple tasks
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
        queue.enqueue_task(task)
    
    print(f"✓ Enqueued {len(tasks)} tasks")
    
    # Dequeue and verify order
    dequeued_order = []
    while not queue.is_empty():
        task = queue.dequeue_task("robot_test")
        dequeued_order.append(task.task_id)
    
    # Should match original order
    expected_order = [task.task_id for task in tasks]
    assert dequeued_order == expected_order
    print("✓ FIFO order maintained correctly")
    
    return True


def test_thread_safety():
    """Test thread safety with concurrent operations."""
    print("\nTesting thread safety...")
    
    queue = JobsQueueImpl(max_queue_size=100)
    num_tasks = 20
    
    # Create tasks
    tasks = []
    for i in range(num_tasks):
        task = Task.create_pick_and_deliver_task(
            order_id=f"order_{i}",
            order_priority="normal",
            customer_id=f"customer_{i}",
            shelf_id=f"shelf_{i}",
            item_id=f"item_{i}",
            quantity_to_pick=1,
            dropoff_zone="main_dropoff"
        )
        tasks.append(task)
    
    # Enqueue tasks in one thread
    def enqueue_worker():
        for task in tasks:
            queue.enqueue_task(task)
            time.sleep(0.001)
    
    # Dequeue tasks in another thread
    dequeued_tasks = []
    dequeue_lock = threading.Lock()
    
    def dequeue_worker():
        while True:
            task = queue.dequeue_task("robot_concurrent")
            if task is None:
                time.sleep(0.001)
                if not enqueue_thread.is_alive() and queue.is_empty():
                    break
                continue
            
            with dequeue_lock:
                dequeued_tasks.append(task)
    
    # Start threads
    enqueue_thread = threading.Thread(target=enqueue_worker)
    dequeue_thread = threading.Thread(target=dequeue_worker)
    
    enqueue_thread.start()
    dequeue_thread.start()
    
    # Wait for completion
    enqueue_thread.join()
    dequeue_thread.join()
    
    # Verify all tasks processed
    assert len(dequeued_tasks) == num_tasks
    assert queue.is_empty()
    print("✓ Thread safety verified - all tasks processed correctly")
    
    return True


def test_error_handling():
    """Test error handling."""
    print("\nTesting error handling...")
    
    queue = JobsQueueImpl(max_queue_size=2)
    
    # Test enqueue None task
    try:
        queue.enqueue_task(None)
        assert False, "Should have raised error"
    except JobsQueueError as e:
        assert "Cannot enqueue None task" in str(e)
        print("✓ None task rejection works")
    
    # Test enqueue task without task_id
    try:
        invalid_task = Task(
            task_id="",
            task_type="PICK_AND_DELIVER",
            order_id="test"
        )
        queue.enqueue_task(invalid_task)
        assert False, "Should have raised error"
    except JobsQueueError as e:
        assert "Task must have a task_id" in str(e)
        print("✓ Empty task_id rejection works")
    
    # Test queue size limit
    task1 = Task.create_pick_and_deliver_task(
        order_id="order1", order_priority="normal", customer_id="c1",
        shelf_id="s1", item_id="i1", quantity_to_pick=1, dropoff_zone="zone1"
    )
    task2 = Task.create_pick_and_deliver_task(
        order_id="order2", order_priority="normal", customer_id="c2",
        shelf_id="s2", item_id="i2", quantity_to_pick=1, dropoff_zone="zone2"
    )
    task3 = Task.create_pick_and_deliver_task(
        order_id="order3", order_priority="normal", customer_id="c3",
        shelf_id="s3", item_id="i3", quantity_to_pick=1, dropoff_zone="zone3"
    )
    
    queue.enqueue_task(task1)
    queue.enqueue_task(task2)
    assert queue.is_full()
    
    try:
        queue.enqueue_task(task3)
        assert False, "Should have raised error"
    except JobsQueueError as e:
        assert "Queue is full" in str(e)
        print("✓ Queue size limit enforcement works")
    
    # Test dequeue with empty robot_id
    try:
        queue.dequeue_task("")
        assert False, "Should have raised error"
    except JobsQueueError as e:
        assert "robot_id cannot be empty" in str(e)
        print("✓ Empty robot_id rejection works")
    
    return True


def test_task_management():
    """Test task management features."""
    print("\nTesting task management...")
    
    queue = JobsQueueImpl()
    
    # Create tasks
    tasks = []
    for i in range(3):
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
        queue.enqueue_task(task)
    
    # Test get_task_by_id
    found_task = queue.get_task_by_id(tasks[1].task_id)
    assert found_task is not None
    assert found_task.task_id == tasks[1].task_id
    print("✓ get_task_by_id works")
    
    # Test get_pending_tasks
    pending = queue.get_pending_tasks()
    assert len(pending) == 3
    print("✓ get_pending_tasks works")
    
    # Test remove_task
    removed = queue.remove_task(tasks[1].task_id)
    assert removed is True
    assert queue.get_queue_size() == 2
    assert queue.get_task_by_id(tasks[1].task_id) is None
    print("✓ remove_task works")
    
    # Test clear_queue
    queue.clear_queue()
    assert queue.is_empty()
    assert queue.get_queue_size() == 0
    print("✓ clear_queue works")
    
    return True


def test_statistics():
    """Test statistics tracking."""
    print("\nTesting statistics...")
    
    queue = JobsQueueImpl()
    
    # Initial stats
    stats = queue.get_queue_stats()
    assert stats["total_enqueued"] == 0
    assert stats["total_dequeued"] == 0
    assert stats["current_queue_size"] == 0
    print("✓ Initial statistics correct")
    
    # Add some tasks
    tasks = []
    for i in range(3):
        task = Task.create_pick_and_deliver_task(
            order_id=f"order_{i}",
            order_priority="normal",
            customer_id=f"customer_{i}",
            shelf_id=f"shelf_{i}",
            item_id=f"item_{i}",
            quantity_to_pick=1,
            dropoff_zone="main_dropoff"
        )
        tasks.append(task)
        queue.enqueue_task(task)
    
    stats = queue.get_queue_stats()
    assert stats["total_enqueued"] == 3
    assert stats["current_queue_size"] == 3
    print("✓ Enqueue statistics correct")
    
    # Dequeue some tasks
    queue.dequeue_task("robot_1")
    queue.dequeue_task("robot_2")
    
    stats = queue.get_queue_stats()
    assert stats["total_enqueued"] == 3
    assert stats["total_dequeued"] == 2
    assert stats["current_queue_size"] == 1
    print("✓ Dequeue statistics correct")
    
    # Test assigned tasks tracking
    assigned = queue.get_assigned_tasks()
    assert len(assigned) == 2
    assert tasks[0].task_id in assigned
    assert tasks[1].task_id in assigned
    print("✓ Task assignment tracking works")
    
    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("JobsQueue Implementation Verification")
    print("=" * 60)
    
    tests = [
        test_basic_functionality,
        test_fifo_behavior,
        test_thread_safety,
        test_error_handling,
        test_task_management,
        test_statistics
    ]
    
    passed = 0
    total = len(tests)
    
    for test_func in tests:
        try:
            if test_func():
                passed += 1
        except Exception as e:
            print(f"✗ {test_func.__name__} failed: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests PASSED! JobsQueue implementation is working correctly.")
        print("\nInterface Compliance Verified:")
        print("✓ enqueue_task() - Thread-safe task addition")
        print("✓ dequeue_task() - FIFO task distribution to robots")
        print("✓ peek_next_task() - Non-destructive queue inspection")
        print("✓ get_queue_size() - Current queue size")
        print("✓ get_pending_tasks() - All queued tasks")
        print("✓ remove_task() - Specific task removal")
        print("✓ clear_queue() - Queue clearing")
        print("✓ get_task_by_id() - Task lookup")
        print("✓ Thread safety - Concurrent access protection")
        print("✓ Error handling - Comprehensive validation")
        return True
    else:
        print("❌ Some tests FAILED. Please check the implementation.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 