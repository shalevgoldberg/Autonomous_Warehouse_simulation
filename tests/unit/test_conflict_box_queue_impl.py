"""
Unit tests for ConflictBoxQueueImpl - Comprehensive queue management testing.

This test suite validates the conflict box queue system including:
- Priority-based queue ordering
- Timeout handling and cleanup
- Lock acquisition and release
- Statistics and monitoring
- Thread safety and error handling
"""
import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock
from typing import Dict, Any

from simulation.conflict_box_queue_impl import (
    ConflictBoxQueueImpl,
    ConflictBoxQueueNotifierImpl
)
from interfaces.conflict_box_queue_interface import (
    QueuePosition,
    LockAcquisitionResult,
    QueueEntry,
    QueueStatus,
    ConflictBoxQueueError
)


class TestConflictBoxQueueImpl(unittest.TestCase):
    """Test cases for ConflictBoxQueueImpl."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Mock database connection
        self.mock_connection = Mock()
        self.mock_cursor = Mock()
        # Proper context manager mock for cursor
        cursor_context = Mock()
        cursor_context.__enter__ = Mock(return_value=self.mock_cursor)
        cursor_context.__exit__ = Mock(return_value=None)
        self.mock_connection.cursor.return_value = cursor_context
        # Create queue instance with mocked database
        with patch('psycopg2.pool.ThreadedConnectionPool') as mock_pool:
            mock_pool.return_value.getconn.return_value = self.mock_connection
            self.queue = ConflictBoxQueueImpl(
                db_password="test_password",
                default_timeout=30.0,
                heartbeat_interval=5.0,
                cleanup_interval=10.0
            )
            self.queue._connection_pool = mock_pool.return_value
    
    def tearDown(self):
        """Clean up test fixtures."""
        if hasattr(self.queue, '_cleanup_stop_event'):
            self.queue._cleanup_stop_event.set()
        if hasattr(self.queue, 'close'):
            self.queue.close()
    
    def test_request_lock_immediate_success(self):
        """Test immediate lock acquisition when box is available."""
        # Mock database responses for immediate acquisition
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            None,  # No current lock
        ]
        self.mock_cursor.rowcount = 1  # Successful lock acquisition
        
        result = self.queue.request_lock("box1", "robot1", priority=1, timeout_seconds=30.0)
        
        self.assertTrue(result.success)
        self.assertEqual(result.position, QueuePosition.LOCK_ACQUIRED)
        self.assertEqual(result.estimated_wait_time, 0.0)
        self.assertEqual(result.queue_position, 0)
    
    def test_request_lock_queue_when_busy(self):
        """Test queueing when conflict box is already locked."""
        # Mock database responses for queueing
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            {'locked_by_robot': 'robot2'},  # Box already locked
        ]
        
        # Mock queue position calculation
        with patch.object(self.queue, '_get_queue_position', return_value=2):
            with patch.object(self.queue, '_calculate_estimated_wait_time', return_value=60.0):
                result = self.queue.request_lock("box1", "robot1", priority=0, timeout_seconds=30.0)
        
        self.assertFalse(result.success)
        self.assertEqual(result.position, QueuePosition.QUEUED)
        self.assertEqual(result.estimated_wait_time, 60.0)
        self.assertEqual(result.queue_position, 2)
    
    def test_request_lock_existing_request_update(self):
        """Test updating timeout for existing request."""
        # Mock existing request
        self.mock_cursor.fetchone.return_value = {'position': 'queued'}
        
        with patch.object(self.queue, '_get_queue_position', return_value=1):
            with patch.object(self.queue, '_calculate_estimated_wait_time', return_value=30.0):
                result = self.queue.request_lock("box1", "robot1", priority=1, timeout_seconds=45.0)
        
        self.assertFalse(result.success)
        self.assertEqual(result.position, QueuePosition.QUEUED)
        self.assertEqual(result.estimated_wait_time, 30.0)
        self.assertEqual(result.queue_position, 1)
    
    def test_request_lock_already_acquired(self):
        """Test when robot already has the lock."""
        # Mock existing lock acquisition
        self.mock_cursor.fetchone.return_value = {'position': 'lock_acquired'}
        
        result = self.queue.request_lock("box1", "robot1", priority=1, timeout_seconds=30.0)
        
        self.assertTrue(result.success)
        self.assertEqual(result.position, QueuePosition.LOCK_ACQUIRED)
        self.assertEqual(result.estimated_wait_time, 0.0)
        self.assertEqual(result.queue_position, 0)
    
    def test_release_lock_success(self):
        """Test successful lock release."""
        # Mock successful lock verification and release
        self.mock_cursor.fetchone.side_effect = [
            {'locked_by_robot': 'robot1'},  # Robot owns the lock
            {'duration': 45.0},  # Lock duration
            {'robot_id': 'robot2', 'priority': 1}  # Next robot in queue
        ]
        
        result = self.queue.release_lock("box1", "robot1")
        
        self.assertTrue(result)
        
        # Verify database operations were called
        self.assertTrue(self.mock_cursor.execute.called)
        self.assertTrue(self.mock_connection.commit.called)
    
    def test_release_lock_not_owner(self):
        """Test release when robot doesn't own the lock."""
        # Mock lock owned by different robot
        self.mock_cursor.fetchone.return_value = None
        
        result = self.queue.release_lock("box1", "robot1")
        
        self.assertFalse(result)
        self.assertTrue(self.mock_connection.rollback.called)
    
    def test_cancel_request_success(self):
        """Test successful request cancellation."""
        self.mock_cursor.rowcount = 1  # Successful cancellation
        
        result = self.queue.cancel_request("box1", "robot1")
        
        self.assertTrue(result)
        self.assertTrue(self.mock_cursor.execute.called)
        self.assertTrue(self.mock_connection.commit.called)
    
    def test_cancel_request_not_found(self):
        """Test cancellation when no pending request exists."""
        self.mock_cursor.rowcount = 0  # No rows affected
        
        result = self.queue.cancel_request("box1", "robot1")
        
        self.assertFalse(result)
    
    def test_get_queue_status(self):
        """Test retrieving queue status."""
        # Mock database responses
        self.mock_cursor.fetchone.side_effect = [
            {'locked_by_robot': 'robot1'},  # Current owner
            {'queue_length': 3, 'next_robot': 'robot2'},  # Queue info
            {'total_wait_time': 180.0, 'successful_acquisitions': 6,
             'total_lock_time': 300.0, 'total_requests': 10}  # Statistics
        ]
        
        status = self.queue.get_queue_status("box1")
        
        self.assertEqual(status.box_id, "box1")
        self.assertEqual(status.current_owner, "robot1")
        self.assertEqual(status.queue_length, 3)
        self.assertEqual(status.next_robot, "robot2")
        self.assertEqual(status.average_wait_time, 30.0)  # 180/6
        self.assertEqual(status.lock_duration_stats['avg'], 50.0)  # 300/6
    
    def test_get_robot_queue_position(self):
        """Test retrieving robot's queue position."""
        # Mock database response - using dict for RealDictCursor compatibility
        self.mock_cursor.fetchone.return_value = {
            'robot_id': 'robot1',
            'box_id': 'box1',
            'priority': 1,
            'queue_time': 1234567890.0,
            'timeout_time': 1234567920.0,
            'position': 'queued',
            'estimated_wait_time': 45.0
        }
        
        entry = self.queue.get_robot_queue_position("box1", "robot1")
        
        self.assertIsNotNone(entry)
        self.assertEqual(entry.robot_id, "robot1")
        self.assertEqual(entry.box_id, "box1")
        self.assertEqual(entry.priority, 1)
        self.assertEqual(entry.position, QueuePosition.QUEUED)
        self.assertEqual(entry.estimated_wait_time, 45.0)
    
    def test_get_robot_queue_position_not_found(self):
        """Test retrieving position when robot not in queue."""
        self.mock_cursor.fetchone.return_value = None
        
        entry = self.queue.get_robot_queue_position("box1", "robot1")
        
        self.assertIsNone(entry)
    
    def test_get_robot_queues(self):
        """Test retrieving all queues for a robot."""
        # Mock database response
        self.mock_cursor.fetchall.return_value = [
            {
                'robot_id': 'robot1',
                'box_id': 'box1',
                'priority': 1,
                'queue_time': 1234567890.0,
                'timeout_time': 1234567920.0,
                'position': 'queued',
                'estimated_wait_time': 45.0
            },
            {
                'robot_id': 'robot1',
                'box_id': 'box2',
                'priority': 2,
                'queue_time': 1234567895.0,
                'timeout_time': 1234567925.0,
                'position': 'lock_acquired',
                'estimated_wait_time': 0.0
            }
        ]
        
        entries = self.queue.get_robot_queues("robot1")
        
        self.assertEqual(len(entries), 2)
        self.assertEqual(entries[0].box_id, "box1")
        self.assertEqual(entries[0].position, QueuePosition.QUEUED)
        self.assertEqual(entries[1].box_id, "box2")
        self.assertEqual(entries[1].position, QueuePosition.LOCK_ACQUIRED)
    
    def test_heartbeat_lock_success(self):
        """Test successful heartbeat."""
        self.mock_cursor.rowcount = 1  # Successful update
        
        result = self.queue.heartbeat_lock("box1", "robot1")
        
        self.assertTrue(result)
        self.assertTrue(self.mock_cursor.execute.called)
        self.assertTrue(self.mock_connection.commit.called)
    
    def test_heartbeat_lock_not_owner(self):
        """Test heartbeat when robot doesn't own lock."""
        self.mock_cursor.rowcount = 0  # No rows updated
        
        result = self.queue.heartbeat_lock("box1", "robot1")
        
        self.assertFalse(result)
    
    def test_cleanup_expired_requests(self):
        """Test cleanup of expired requests."""
        # Mock cleanup results
        self.mock_cursor.rowcount = 5  # 5 expired requests
        
        cleaned_count = self.queue.cleanup_expired_requests()
        
        self.assertEqual(cleaned_count, 15)  # 5 + 5 + 5 (timeouts + locks + old entries)
        self.assertTrue(self.mock_cursor.execute.called)
        self.assertTrue(self.mock_connection.commit.called)
    
    def test_get_queue_statistics(self):
        """Test retrieving comprehensive statistics."""
        # Mock database responses
        self.mock_cursor.fetchone.side_effect = [
            {  # Current stats
                'total_queues': 5,
                'total_waiting_robots': 12,
                'avg_estimated_wait': 35.0,
                'priority_0': 8,
                'priority_1': 3,
                'priority_high': 1
            },
            {  # Historical stats
                'total_requests': 100,
                'successful_acquisitions': 85,
                'timeouts': 10,
                'cancellations': 5,
                'avg_wait_time': 25.0,
                'avg_lock_time': 40.0
            },
            {  # Lock stats
                'active_locks': 8
            }
        ]
        
        stats = self.queue.get_queue_statistics()
        
        self.assertEqual(stats['total_queues'], 5)
        self.assertEqual(stats['total_waiting_robots'], 12)
        self.assertEqual(stats['active_locks'], 8)
        self.assertEqual(stats['average_wait_time'], 25.0)
        self.assertEqual(stats['average_lock_time'], 40.0)
        self.assertEqual(stats['timeout_rate'], 10.0)  # 10/100 * 100
        self.assertEqual(stats['success_rate'], 85.0)  # 85/100 * 100
        self.assertEqual(stats['priority_distribution']['priority_0'], 8)
    
    def test_priority_ordering(self):
        """Test that priority ordering works correctly."""
        # This would require more complex database mocking
        # For now, test the priority queue entry comparison
        from simulation.conflict_box_queue_impl import PriorityQueueEntry
        
        entry1 = PriorityQueueEntry(priority=1, queue_time=1000.0, robot_id="robot1", 
                                   box_id="box1", timeout_time=2000.0)
        entry2 = PriorityQueueEntry(priority=2, queue_time=1001.0, robot_id="robot2", 
                                   box_id="box1", timeout_time=2001.0)
        entry3 = PriorityQueueEntry(priority=1, queue_time=999.0, robot_id="robot3", 
                                   box_id="box1", timeout_time=1999.0)
        
        # Higher priority should come first
        self.assertTrue(entry2 < entry1)
        
        # Same priority, earlier time should come first
        self.assertTrue(entry3 < entry1)
    
    def test_database_error_handling(self):
        """Test handling of database errors."""
        import psycopg2
        
        # Mock database error
        self.mock_cursor.execute.side_effect = psycopg2.Error("Database connection failed")
        
        with self.assertRaises(ConflictBoxQueueError):
            self.queue.request_lock("box1", "robot1")
    
    def test_thread_safety(self):
        """Test thread safety of queue operations."""
        # This is a basic test - full thread safety testing would require more complex setup
        results = []
        errors = []
        
        def worker():
            try:
                # Mock successful operation
                with patch.object(self.queue, '_get_connection'):
                    self.mock_cursor.fetchone.return_value = None
                    self.mock_cursor.rowcount = 1
                    result = self.queue.request_lock(f"box1", f"robot{threading.current_thread().ident}")
                    results.append(result)
            except Exception as e:
                errors.append(e)
        
        # Start multiple threads
        threads = []
        for i in range(5):
            thread = threading.Thread(target=worker)
            threads.append(thread)
            thread.start()
        
        # Wait for completion
        for thread in threads:
            thread.join()
        
        # Verify no errors occurred
        self.assertEqual(len(errors), 0)


class TestConflictBoxQueueNotifierImpl(unittest.TestCase):
    """Test cases for ConflictBoxQueueNotifierImpl."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.notifier = ConflictBoxQueueNotifierImpl()
    
    def test_register_and_notify_callback(self):
        """Test callback registration and notification."""
        callback_data = []
        
        def test_callback(event_type: str, data: Dict[str, Any]):
            callback_data.append((event_type, data))
        
        # Register callback
        self.notifier.register_callback("robot1", test_callback)
        
        # Send notification
        self.notifier.notify_lock_available("box1", "robot1")
        
        # Verify callback was called
        self.assertEqual(len(callback_data), 1)
        event_type, data = callback_data[0]
        self.assertEqual(event_type, "lock_available")
        self.assertEqual(data['box_id'], "box1")
        self.assertIn('timestamp', data)
    
    def test_notify_queue_position_changed(self):
        """Test queue position change notification."""
        callback_data = []
        
        def test_callback(event_type: str, data: Dict[str, Any]):
            callback_data.append((event_type, data))
        
        self.notifier.register_callback("robot1", test_callback)
        self.notifier.notify_queue_position_changed("box1", "robot1", 2, 45.0)
        
        self.assertEqual(len(callback_data), 1)
        event_type, data = callback_data[0]
        self.assertEqual(event_type, "queue_position_changed")
        self.assertEqual(data['box_id'], "box1")
        self.assertEqual(data['new_position'], 2)
        self.assertEqual(data['estimated_wait_time'], 45.0)
    
    def test_notify_request_timeout(self):
        """Test timeout notification."""
        callback_data = []
        
        def test_callback(event_type: str, data: Dict[str, Any]):
            callback_data.append((event_type, data))
        
        self.notifier.register_callback("robot1", test_callback)
        self.notifier.notify_request_timeout("box1", "robot1")
        
        self.assertEqual(len(callback_data), 1)
        event_type, data = callback_data[0]
        self.assertEqual(event_type, "request_timeout")
        self.assertEqual(data['box_id'], "box1")
    
    def test_unregister_callback(self):
        """Test callback unregistration."""
        callback_data = []
        
        def test_callback(event_type: str, data: Dict[str, Any]):
            callback_data.append((event_type, data))
        
        # Register and then unregister
        self.notifier.register_callback("robot1", test_callback)
        self.notifier.unregister_callback("robot1")
        
        # Send notification
        self.notifier.notify_lock_available("box1", "robot1")
        
        # Verify callback was not called
        self.assertEqual(len(callback_data), 0)
    
    def test_no_callback_registered(self):
        """Test notification when no callback is registered."""
        # This should not raise an exception
        self.notifier.notify_lock_available("box1", "robot1")
        self.notifier.notify_queue_position_changed("box1", "robot1", 1, 30.0)
        self.notifier.notify_request_timeout("box1", "robot1")
    
    def test_callback_exception_handling(self):
        """Test handling of exceptions in callbacks."""
        def failing_callback(event_type: str, data: Dict[str, Any]):
            raise Exception("Callback failed")
        
        self.notifier.register_callback("robot1", failing_callback)
        
        # This should not raise an exception
        self.notifier.notify_lock_available("box1", "robot1")


if __name__ == '__main__':
    unittest.main() 