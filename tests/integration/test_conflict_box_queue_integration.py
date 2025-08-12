"""
Integration tests for ConflictBoxQueue with LaneFollower - End-to-end testing.

This test suite validates the integration between the conflict box queue system
and the LaneFollower, ensuring proper coordination and lock management.
"""
import unittest
import time
import threading
from unittest.mock import Mock, patch, MagicMock

from robot.impl.lane_follower_impl import LaneFollowerImpl
from simulation.conflict_box_queue_impl import ConflictBoxQueueImpl
from interfaces.conflict_box_queue_interface import QueuePosition
from interfaces.navigation_types import Route, RouteSegment, Lane, Point, BoxRec
from interfaces.lane_follower_interface import LaneFollowingConfig


class TestConflictBoxQueueIntegration(unittest.TestCase):
    """Integration tests for conflict box queue with LaneFollower."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Mock dependencies
        self.mock_state_holder = Mock()
        self.mock_motion_executor = Mock()
        self.mock_simulation_data_service = Mock()
        self.mock_config_provider = Mock()
        
        # Mock database for queue
        self.mock_connection = Mock()
        self.mock_cursor = Mock()
        self.mock_connection.cursor.return_value.__enter__.return_value = self.mock_cursor
        
        # Create conflict box queue
        with patch('psycopg2.pool.ThreadedConnectionPool') as mock_pool:
            mock_pool.return_value.getconn.return_value = self.mock_connection
            
            self.conflict_box_queue = ConflictBoxQueueImpl(
                db_password="test_password",
                default_timeout=30.0,
                heartbeat_interval=5.0,
                cleanup_interval=10.0
            )
            self.conflict_box_queue._connection_pool = mock_pool.return_value
        
        # Create LaneFollower with queue
        self.lane_follower = LaneFollowerImpl(
            robot_id="robot1",
            state_holder=self.mock_state_holder,
            motion_executor=self.mock_motion_executor,
            simulation_data_service=self.mock_simulation_data_service,
            config_provider=self.mock_config_provider,
            conflict_box_queue=self.conflict_box_queue
        )
        
        # Configure mocks
        self.mock_state_holder.get_position.return_value = (0.0, 0.0, 0.0)
        self.mock_motion_executor.get_motion_status.return_value = Mock()
        
        # Set up basic configuration
        config = LaneFollowingConfig(
            lane_tolerance=0.1,
            max_speed=1.0,
            corner_speed=0.3,
            lock_timeout=30.0,
            heartbeat_interval=5.0
        )
        self.lane_follower.set_config(config)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if hasattr(self.conflict_box_queue, 'close'):
            self.conflict_box_queue.close()
        if hasattr(self.lane_follower, 'stop_following'):
            self.lane_follower.stop_following(force_release=True)
    
    def test_lock_acquisition_with_queue_immediate_success(self):
        """Test immediate lock acquisition through queue system."""
        # Mock immediate success
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            None,  # No current lock
        ]
        self.mock_cursor.rowcount = 1  # Successful lock acquisition
        
        # Try to acquire lock
        success = self.lane_follower.try_acquire_conflict_box_lock("box1", priority=1)
        
        self.assertTrue(success)
        self.assertIn("box1", self.lane_follower.get_held_conflict_boxes())
    
    def test_lock_acquisition_with_queue_queued(self):
        """Test lock acquisition that results in queueing."""
        # Mock queueing scenario
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            {'locked_by_robot': 'robot2'},  # Box already locked
        ]
        
        # Mock queue position calculation
        with patch.object(self.conflict_box_queue, '_get_queue_position', return_value=2):
            with patch.object(self.conflict_box_queue, '_calculate_estimated_wait_time', return_value=60.0):
                success = self.lane_follower.try_acquire_conflict_box_lock("box1", priority=0)
        
        self.assertFalse(success)
        self.assertNotIn("box1", self.lane_follower.get_held_conflict_boxes())
    
    def test_lock_release_with_queue(self):
        """Test lock release through queue system."""
        # First acquire a lock
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            None,  # No current lock
        ]
        self.mock_cursor.rowcount = 1  # Successful acquisition
        
        success = self.lane_follower.try_acquire_conflict_box_lock("box1", priority=1)
        self.assertTrue(success)
        
        # Now release it
        self.mock_cursor.fetchone.side_effect = [
            {'locked_by_robot': 'robot1'},  # Robot owns the lock
            {'duration': 45.0},  # Lock duration
            None  # No next robot in queue
        ]
        
        success = self.lane_follower.release_conflict_box_lock("box1")
        self.assertTrue(success)
        self.assertNotIn("box1", self.lane_follower.get_held_conflict_boxes())
    
    def test_heartbeat_with_queue(self):
        """Test heartbeat functionality with queue system."""
        # Acquire a lock first
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            None,  # No current lock
        ]
        self.mock_cursor.rowcount = 1  # Successful acquisition
        
        success = self.lane_follower.try_acquire_conflict_box_lock("box1", priority=1)
        self.assertTrue(success)
        
        # Mock heartbeat success
        self.mock_cursor.rowcount = 1  # Successful heartbeat
        
        # Start route to trigger heartbeat thread
        route = self._create_test_route_with_conflict_box()
        
        with patch.object(self.lane_follower, '_start_next_segment'):
            self.lane_follower.follow_route(route, "robot1")
        
        # Wait a short time for heartbeat
        time.sleep(0.1)
        
        # Stop following to clean up
        self.lane_follower.stop_following()
    
    def test_route_following_with_conflict_boxes(self):
        """Test complete route following with conflict box management."""
        # Create route with conflict boxes
        route = self._create_test_route_with_conflict_box()
        
        # Mock successful lock acquisition
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            None,  # No current lock
        ]
        self.mock_cursor.rowcount = 1  # Successful acquisition
        
        # Mock motion executor responses
        self.mock_motion_executor.get_motion_status.return_value = Mock()
        
        with patch.object(self.lane_follower, '_start_next_segment'):
            with patch.object(self.lane_follower, '_update_segment_progress'):
                self.lane_follower.follow_route(route, "robot1")
        
        # Verify route is being followed
        self.assertTrue(self.lane_follower.is_following())
        self.assertEqual(self.lane_follower.get_current_route(), route)
        
        # Clean up
        self.lane_follower.stop_following()
    
    def test_emergency_stop_with_queue(self):
        """Test emergency stop releases all locks properly."""
        # Acquire locks
        self.mock_cursor.fetchone.side_effect = [
            None,  # No existing request
            None,  # No current lock
        ] * 4  # For two lock acquisitions
        self.mock_cursor.rowcount = 1
        
        self.lane_follower.try_acquire_conflict_box_lock("box1", priority=1)
        self.lane_follower.try_acquire_conflict_box_lock("box2", priority=1)
        
        # Verify locks are held
        held_boxes = self.lane_follower.get_held_conflict_boxes()
        self.assertIn("box1", held_boxes)
        self.assertIn("box2", held_boxes)
        
        # Mock release success
        self.mock_cursor.fetchone.side_effect = [
            {'locked_by_robot': 'robot1'},  # Robot owns lock1
            {'duration': 30.0},  # Lock duration
            None,  # No next robot
            {'locked_by_robot': 'robot1'},  # Robot owns lock2
            {'duration': 25.0},  # Lock duration
            None,  # No next robot
        ]
        
        # Emergency stop
        self.lane_follower.emergency_stop()
        
        # Verify locks are released
        self.assertEqual(len(self.lane_follower.get_held_conflict_boxes()), 0)
    
    def test_priority_handling_in_queue(self):
        """Test that priority is properly handled in queue system."""
        # Test priority queue entry comparison
        from simulation.conflict_box_queue_impl import PriorityQueueEntry
        
        # Create entries with different priorities
        low_priority = PriorityQueueEntry(
            priority=0, queue_time=1000.0, robot_id="robot1", 
            box_id="box1", timeout_time=2000.0
        )
        high_priority = PriorityQueueEntry(
            priority=2, queue_time=1001.0, robot_id="robot2", 
            box_id="box1", timeout_time=2001.0
        )
        
        # Higher priority should come first
        self.assertTrue(high_priority < low_priority)
        
        # Test same priority, different times
        early_time = PriorityQueueEntry(
            priority=1, queue_time=999.0, robot_id="robot3", 
            box_id="box1", timeout_time=1999.0
        )
        late_time = PriorityQueueEntry(
            priority=1, queue_time=1002.0, robot_id="robot4", 
            box_id="box1", timeout_time=2002.0
        )
        
        # Earlier time should come first for same priority
        self.assertTrue(early_time < late_time)
    
    def test_fallback_to_direct_acquisition(self):
        """Test fallback to direct acquisition when queue is not available."""
        # Create LaneFollower without queue
        lane_follower_no_queue = LaneFollowerImpl(
            robot_id="robot2",
            state_holder=self.mock_state_holder,
            motion_executor=self.mock_motion_executor,
            simulation_data_service=self.mock_simulation_data_service,
            config_provider=self.mock_config_provider,
            conflict_box_queue=None  # No queue system
        )
        
        # Mock direct acquisition success
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.return_value = True
        
        success = lane_follower_no_queue.try_acquire_conflict_box_lock("box1", priority=1)
        
        self.assertTrue(success)
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.assert_called_with(
            "box1", "robot2", 1
        )
    
    def test_queue_statistics_integration(self):
        """Test queue statistics functionality."""
        # Mock statistics response
        self.mock_cursor.fetchone.side_effect = [
            {  # Current stats
                'total_queues': 2,
                'total_waiting_robots': 5,
                'avg_estimated_wait': 25.0,
                'priority_0': 3,
                'priority_1': 2,
                'priority_high': 0
            },
            {  # Historical stats
                'total_requests': 50,
                'successful_acquisitions': 45,
                'timeouts': 3,
                'cancellations': 2,
                'avg_wait_time': 20.0,
                'avg_lock_time': 35.0
            },
            {  # Lock stats
                'active_locks': 3
            }
        ]
        
        stats = self.conflict_box_queue.get_queue_statistics()
        
        self.assertEqual(stats['total_queues'], 2)
        self.assertEqual(stats['total_waiting_robots'], 5)
        self.assertEqual(stats['active_locks'], 3)
        self.assertEqual(stats['success_rate'], 90.0)  # 45/50 * 100
        self.assertEqual(stats['timeout_rate'], 6.0)   # 3/50 * 100
    
    def test_concurrent_lock_requests(self):
        """Test concurrent lock requests from multiple robots."""
        results = []
        errors = []
        
        def robot_worker(robot_id: str):
            try:
                # Create separate LaneFollower for each robot
                lane_follower = LaneFollowerImpl(
                    robot_id=robot_id,
                    state_holder=Mock(),
                    motion_executor=Mock(),
                    simulation_data_service=Mock(),
                    conflict_box_queue=self.conflict_box_queue
                )
                
                # Mock database responses
                with patch.object(self.conflict_box_queue, '_get_connection'):
                    self.mock_cursor.fetchone.return_value = None
                    self.mock_cursor.rowcount = 1 if robot_id == "robot1" else 0
                    
                    success = lane_follower.try_acquire_conflict_box_lock("box1", priority=1)
                    results.append((robot_id, success))
                    
            except Exception as e:
                errors.append((robot_id, e))
        
        # Start multiple robot threads
        threads = []
        for i in range(3):
            robot_id = f"robot{i+1}"
            thread = threading.Thread(target=robot_worker, args=(robot_id,))
            threads.append(thread)
            thread.start()
        
        # Wait for completion
        for thread in threads:
            thread.join()
        
        # Verify no errors and at most one success
        self.assertEqual(len(errors), 0)
        successful_acquisitions = sum(1 for _, success in results if success)
        self.assertLessEqual(successful_acquisitions, 1)
    
    def _create_test_route_with_conflict_box(self) -> Route:
        """Create a test route with conflict boxes."""
        # Create lane with waypoints
        waypoints = [
            Point(0.0, 0.0),
            Point(1.0, 0.0),
            Point(2.0, 0.0)
        ]
        lane = Lane(lane_id="lane1", waypoints=waypoints, width=0.5)
        
        # Create route segment
        segment = RouteSegment(
            segment_id="seg1",
            lane=lane,
            start_point=waypoints[0],
            end_point=waypoints[-1],
            speed_limit=1.0
        )
        
        # Create conflict box
        conflict_box = BoxRec(
            box_id="box1",
            center=Point(1.0, 0.0),
            size=0.8
        )
        
        # Create route
        route = Route(
            segments=[segment],
            total_distance=2.0,
            estimated_time=2.0,
            conflict_boxes=[conflict_box]
        )
        
        return route


class TestConflictBoxQueueErrorHandling(unittest.TestCase):
    """Test error handling in conflict box queue integration."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_state_holder = Mock()
        self.mock_motion_executor = Mock()
        self.mock_simulation_data_service = Mock()
        
        # Create queue with invalid configuration to test error handling
        with patch('psycopg2.pool.ThreadedConnectionPool') as mock_pool:
            mock_pool.side_effect = Exception("Database connection failed")
            
            with self.assertRaises(Exception):
                ConflictBoxQueueImpl(db_password="test_password")
    
    def test_queue_unavailable_fallback(self):
        """Test behavior when queue system is unavailable."""
        # Create LaneFollower with None queue
        lane_follower = LaneFollowerImpl(
            robot_id="robot1",
            state_holder=self.mock_state_holder,
            motion_executor=self.mock_motion_executor,
            simulation_data_service=self.mock_simulation_data_service,
            conflict_box_queue=None
        )
        
        # Mock direct acquisition
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.return_value = True
        self.mock_simulation_data_service.release_conflict_box_lock.return_value = True
        self.mock_simulation_data_service.heartbeat_conflict_box_lock.return_value = True
        
        # Test fallback operations
        success = lane_follower.try_acquire_conflict_box_lock("box1", priority=1)
        self.assertTrue(success)
        
        success = lane_follower.release_conflict_box_lock("box1")
        self.assertTrue(success)
        
        # Verify direct service was called
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.assert_called()
        self.mock_simulation_data_service.release_conflict_box_lock.assert_called()
    
    def test_database_error_propagation(self):
        """Test that database errors are properly propagated."""
        # Mock database connection
        mock_connection = Mock()
        mock_cursor = Mock()
        mock_connection.cursor.return_value.__enter__.return_value = mock_cursor
        
        with patch('psycopg2.pool.ThreadedConnectionPool') as mock_pool:
            mock_pool.return_value.getconn.return_value = mock_connection
            
            queue = ConflictBoxQueueImpl(db_password="test_password")
            queue._connection_pool = mock_pool.return_value
            
            # Mock database error
            import psycopg2
            mock_cursor.execute.side_effect = psycopg2.Error("Connection lost")
            
            # Test that error is properly wrapped and propagated
            with self.assertRaises(Exception):
                queue.request_lock("box1", "robot1")


if __name__ == '__main__':
    unittest.main()