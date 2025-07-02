"""Comprehensive JobsQueue test."""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import threading
import time
from warehouse.impl.jobs_queue_impl import JobsQueueImpl
from interfaces.jobs_queue_interface import JobsQueueError
from interfaces.task_handler_interface import Task


def test_interface_compliance():
    """Test all interface methods work correctly."""
    print("Testing interface compliance...")
    
    queue = JobsQueueImpl(max_queue_size=5)
    
    # Test initial state
    assert queue.get_queue_size() == 0
    assert queue.is_empty()
    assert not queue.is_full()
    assert queue.peek_next_task() is None
    assert queue.get_pending_tasks() == []
    print("✓ Initial state correct")
    
    # Create test tasks
    tasks = []
    for i in range(3):
        task = Task.create_pick_and_deliver_task(
            task_id=f"task_{i}",
            order_id=f"order_{i}",
            order_priority="normal",
            customer_id=f"customer_{i}",
            shelf_id=f"shelf_{i}",
            item_id=f"item_{i}",
            quantity_to_pick=i + 1,
            dropoff_zone="main_dropoff"
        )
        tasks.append(task)
    
    # Test enqueue_task
    for task in tasks:
        queue.enqueue_task(task)
    
    assert queue.get_queue_size() == 3
    assert not queue.is_empty()
    print("✓ enqueue_task works")
    
    # Test peek_next_task
    peeked = queue.peek_next_task()
    assert peeked is not None
    assert peeked.task_id == tasks[0].task_id
    assert queue.get_queue_size() == 3  # Size unchanged
    print("✓ peek_next_task works")
    
    # Test get_pending_tasks
    pending = queue.get_pending_tasks()
    assert len(pending) == 3
    assert pending[0].task_id == tasks[0].task_id
    print("✓ get_pending_tasks works")
    
    # Test get_task_by_id
    found = queue.get_task_by_id(tasks[1].task_id)
    assert found is not None
    assert found.task_id == tasks[1].task_id
    print("✓ get_task_by_id works")
    
    # Test dequeue_task
    dequeued = queue.dequeue_task("robot_1")
    assert dequeued is not None
    assert dequeued.task_id == tasks[0].task_id
    assert queue.get_queue_size() == 2
    print("✓ dequeue_task works")
    
    # Test remove_task
    removed = queue.remove_task(tasks[1].task_id)
    assert removed is True
    assert queue.get_queue_size() == 1
    assert queue.get_task_by_id(tasks[1].task_id) is None
    print("✓ remove_task works")
    
    # Test clear_queue
    queue.clear_queue()
    assert queue.is_empty()
    assert queue.get_queue_size() == 0
    print("✓ clear_queue works")
    
    return True


def test_fifo_behavior():
    """Test FIFO ordering."""
    print("\nTesting FIFO behavior...")
    
    queue = JobsQueueImpl()
    
    # Create tasks in specific order
    task_ids = ["first", "second", "third", "fourth"]
    for task_id in task_ids:
        task = Task.create_pick_and_deliver_task(
            task_id=task_id,
            order_id=f"order_{task_id}",
            order_priority="normal",
            customer_id="customer",
            shelf_id="shelf",
            item_id="item",
            quantity_to_pick=1,
            dropoff_zone="zone"
        )
        queue.enqueue_task(task)
    
    # Dequeue and verify order
    dequeued_order = []
    while not queue.is_empty():
        task = queue.dequeue_task("robot_fifo")
        dequeued_order.append(task.task_id)
    
    assert dequeued_order == task_ids
    print("✓ FIFO order maintained")
    
    return True


def test_error_handling():
    """Test error conditions."""
    print("\nTesting error handling...")
    
    queue = JobsQueueImpl(max_queue_size=1)
    
    # Test None task
    try:
        queue.enqueue_task(None)
        assert False, "Should have raised error"
    except JobsQueueError as e:
        assert "Cannot enqueue None task" in str(e)
        print("✓ None task rejection")
    
    # Test empty task_id
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
        print("✓ Empty task_id rejection")
    
    # Test queue full
    task1 = Task.create_pick_and_deliver_task(
        task_id="task1",
        order_id="order1",
        order_priority="normal",
        customer_id="customer",
        shelf_id="shelf",
        item_id="item",
        quantity_to_pick=1,
        dropoff_zone="zone"
    )
    task2 = Task.create_pick_and_deliver_task(
        task_id="task2",
        order_id="order2",
        order_priority="normal",
        customer_id="customer",
        shelf_id="shelf",
        item_id="item",
        quantity_to_pick=1,
        dropoff_zone="zone"
    )
    
    queue.enqueue_task(task1)
    assert queue.is_full()
    
    try:
        queue.enqueue_task(task2)
        assert False, "Should have raised error"
    except JobsQueueError as e:
        assert "Queue is full" in str(e)
        print("✓ Queue full rejection")
    
    # Test empty robot_id
    try:
        queue.dequeue_task("")
        assert False, "Should have raised error"
    except JobsQueueError as e:
        assert "robot_id cannot be empty" in str(e)
        print("✓ Empty robot_id rejection")
    
    return True


def test_thread_safety():
    """Test concurrent operations."""
    print("\nTesting thread safety...")
    
    queue = JobsQueueImpl(max_queue_size=50)
    num_tasks = 20
    
    # Create tasks
    tasks = []
    for i in range(num_tasks):
        task = Task.create_pick_and_deliver_task(
            task_id=f"concurrent_task_{i}",
            order_id=f"order_{i}",
            order_priority="normal",
            customer_id=f"customer_{i}",
            shelf_id=f"shelf_{i}",
            item_id=f"item_{i}",
            quantity_to_pick=1,
            dropoff_zone="zone"
        )
        tasks.append(task)
    
    # Producer thread
    def producer():
        for task in tasks:
            queue.enqueue_task(task)
            time.sleep(0.001)
    
    # Consumer threads
    consumed_tasks = []
    consume_lock = threading.Lock()
    
    def consumer(robot_id):
        while True:
            task = queue.dequeue_task(robot_id)
            if task is None:
                time.sleep(0.001)
                if not producer_thread.is_alive() and queue.is_empty():
                    break
                continue
            
            with consume_lock:
                consumed_tasks.append(task)
    
    # Start threads
    producer_thread = threading.Thread(target=producer)
    consumer_threads = []
    
    for i in range(3):
        thread = threading.Thread(target=consumer, args=[f"robot_{i}"])
        consumer_threads.append(thread)
    
    producer_thread.start()
    for thread in consumer_threads:
        thread.start()
    
    # Wait for completion
    producer_thread.join()
    for thread in consumer_threads:
        thread.join()
    
    # Verify results
    assert len(consumed_tasks) == num_tasks
    assert queue.is_empty()
    
    # Check no duplicates
    task_ids = [task.task_id for task in consumed_tasks]
    assert len(set(task_ids)) == num_tasks
    
    print("✓ Thread safety verified")
    return True


def test_statistics_and_tracking():
    """Test statistics and task tracking."""
    print("\nTesting statistics and tracking...")
    
    queue = JobsQueueImpl()
    
    # Initial stats
    stats = queue.get_queue_stats()
    assert stats["total_enqueued"] == 0
    assert stats["total_dequeued"] == 0
    assert stats["current_queue_size"] == 0
    print("✓ Initial statistics correct")
    
    # Add tasks
    tasks = []
    for i in range(3):
        task = Task.create_pick_and_deliver_task(
            task_id=f"stats_task_{i}",
            order_id=f"order_{i}",
            order_priority="normal",
            customer_id=f"customer_{i}",
            shelf_id=f"shelf_{i}",
            item_id=f"item_{i}",
            quantity_to_pick=1,
            dropoff_zone="zone"
        )
        tasks.append(task)
        queue.enqueue_task(task)
    
    stats = queue.get_queue_stats()
    assert stats["total_enqueued"] == 3
    assert stats["current_queue_size"] == 3
    print("✓ Enqueue statistics correct")
    
    # Dequeue tasks
    queue.dequeue_task("robot_alpha")
    queue.dequeue_task("robot_beta")
    
    stats = queue.get_queue_stats()
    assert stats["total_enqueued"] == 3
    assert stats["total_dequeued"] == 2
    assert stats["current_queue_size"] == 1
    print("✓ Dequeue statistics correct")
    
    # Test assignment tracking
    assigned = queue.get_assigned_tasks()
    assert len(assigned) == 2
    assert tasks[0].task_id in assigned
    assert tasks[1].task_id in assigned
    assert assigned[tasks[0].task_id] == "robot_alpha"
    assert assigned[tasks[1].task_id] == "robot_beta"
    print("✓ Task assignment tracking works")
    
    return True


def test_utility_methods():
    """Test additional utility methods."""
    print("\nTesting utility methods...")
    
    queue = JobsQueueImpl()
    
    # Create tasks for different orders
    order1_tasks = []
    order2_tasks = []
    
    for i in range(2):
        task1 = Task.create_pick_and_deliver_task(
            task_id=f"order1_task_{i}",
            order_id="order_1",
            order_priority="normal",
            customer_id="customer_1",
            shelf_id=f"shelf_{i}",
            item_id=f"item_{i}",
            quantity_to_pick=1,
            dropoff_zone="zone"
        )
        task2 = Task.create_pick_and_deliver_task(
            task_id=f"order2_task_{i}",
            order_id="order_2",
            order_priority="high",
            customer_id="customer_2",
            shelf_id=f"shelf_{i+10}",
            item_id=f"item_{i+10}",
            quantity_to_pick=1,
            dropoff_zone="zone"
        )
        
        order1_tasks.append(task1)
        order2_tasks.append(task2)
        queue.enqueue_task(task1)
        queue.enqueue_task(task2)
    
    # Test get_tasks_by_order
    found_order1 = queue.get_tasks_by_order("order_1")
    found_order2 = queue.get_tasks_by_order("order_2")
    
    assert len(found_order1) == 2
    assert len(found_order2) == 2
    
    order1_ids = [task.task_id for task in found_order1]
    for task in order1_tasks:
        assert task.task_id in order1_ids
    
    print("✓ get_tasks_by_order works")
    
    # Test get_tasks_by_priority
    high_priority_tasks = queue.get_tasks_by_priority("high")
    assert len(high_priority_tasks) == 2
    for task in high_priority_tasks:
        assert task.order_priority == "high"
    
    print("✓ get_tasks_by_priority works")
    
    # Test Python built-ins
    assert len(queue) == 4
    assert bool(queue) is True
    
    queue.clear_queue()
    assert len(queue) == 0
    assert bool(queue) is False
    
    print("✓ Python built-in support works")
    
    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("JobsQueue Comprehensive Test Suite")
    print("=" * 60)
    
    tests = [
        test_interface_compliance,
        test_fifo_behavior,
        test_error_handling,
        test_thread_safety,
        test_statistics_and_tracking,
        test_utility_methods
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
        print("🎉 ALL TESTS PASSED!")
        print("\nJobsQueue Implementation Features Verified:")
        print("✓ Complete interface compliance")
        print("✓ Thread-safe FIFO queue operations")
        print("✓ Comprehensive error handling")
        print("✓ Task tracking and statistics")
        print("✓ Concurrent producer/consumer safety")
        print("✓ Queue management utilities")
        print("✓ Python built-in support (__len__, __bool__)")
        return True
    else:
        print("❌ Some tests failed")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 