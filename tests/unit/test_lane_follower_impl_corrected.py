"""
Unit tests for LaneFollowerImpl - corrected version with proper interface usage.
"""
import unittest
import threading
import time
import math
from unittest.mock import Mock, MagicMock, patch, call
from typing import List, Tuple, Optional

from robot.impl.lane_follower_impl import LaneFollowerImpl, ConflictBoxState, LaneSegmentProgress
from interfaces.lane_follower_interface import (
    LaneFollowingStatus, ConflictBoxStatus, LaneFollowingConfig,
    LaneFollowingError, ConflictBoxLockError, LaneDeviationError
)
from interfaces.navigation_types import Route, RouteSegment, LaneRec, LaneDirection, Point, BoxRec
from interfaces.motion_executor_interface import MotionStatus
from interfaces.simulation_data_service_interface import SimulationDataServiceError


class TestLaneFollowerImpl(unittest.TestCase):
    """Test suite for LaneFollowerImpl."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.robot_id = "test_robot_001"
        
        # Create mock dependencies
        self.mock_state_holder = Mock()
        self.mock_motion_executor = Mock()
        self.mock_simulation_data_service = Mock()
        
        # Set up mock returns
        self.mock_state_holder.get_position.return_value = (5.0, 5.0)
        self.mock_motion_executor.get_motion_status.return_value = MotionStatus.IDLE
        
        # Create lane follower instance
        self.lane_follower = LaneFollowerImpl(
            robot_id=self.robot_id,
            state_holder=self.mock_state_holder,
            motion_executor=self.mock_motion_executor,
            simulation_data_service=self.mock_simulation_data_service
        )
        
        # Test data
        self.test_lane = LaneRec(
            lane_id="lane_001",
            direction=LaneDirection.NORTH,
            waypoints=[Point(0.0, 0.0), Point(0.0, 10.0)],
            is_goal_only=False
        )
        
        self.test_segment = RouteSegment(
            lane=self.test_lane,
            reverse=False,
            start_point=Point(0.0, 0.0),
            end_point=Point(0.0, 10.0)
        )
        
        self.test_conflict_box = BoxRec(
            box_id="box_001",
            center=Point(5.0, 5.0),
            size=2.0
        )
        
        self.test_route = Route(
            segments=[self.test_segment],
            total_distance=10.0,
            estimated_time=10.0,
            conflict_boxes=[self.test_conflict_box]
        )
    
    def tearDown(self):
        """Clean up after tests."""
        if hasattr(self.lane_follower, '_heartbeat_running'):
            self.lane_follower._heartbeat_running = False
        time.sleep(0.1)  # Allow cleanup
    
    def test_initialization(self):
        """Test proper initialization of lane follower."""
        self.assertEqual(self.lane_follower._robot_id, self.robot_id)
        self.assertEqual(self.lane_follower._status, LaneFollowingStatus.IDLE)
        self.assertIsNone(self.lane_follower._current_route)
        self.assertIsNone(self.lane_follower._current_segment)
        self.assertEqual(len(self.lane_follower._conflict_boxes), 0)
        self.assertEqual(len(self.lane_follower._locked_boxes), 0)
        self.assertFalse(self.lane_follower._emergency_stopped)
    
    def test_follow_route_success(self):
        """Test successful route following initiation."""
        # Act
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        
        # Assert
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.FOLLOWING_LANE)
        self.assertEqual(self.lane_follower._current_route, self.test_route)
        self.assertEqual(self.lane_follower._segment_index, 0)
        self.assertEqual(self.lane_follower._total_distance, 10.0)
        self.assertIn("box_001", self.lane_follower._conflict_boxes)
        
        # Verify motion executor was called
        self.mock_motion_executor.follow_route.assert_called_once()
    
    def test_follow_route_while_active_fails(self):
        """Test that starting route while active raises error."""
        # Arrange
        self.lane_follower._status = LaneFollowingStatus.FOLLOWING_LANE
        
        # Act & Assert
        with self.assertRaises(LaneFollowingError) as context:
            self.lane_follower.follow_route(self.test_route, self.robot_id)
        
        self.assertIn("Cannot start route while status is", str(context.exception))
    
    def test_follow_route_empty_segments_fails(self):
        """Test that starting route with empty segments raises error."""
        # Arrange
        empty_route = Route(segments=[], total_distance=0.0, estimated_time=0.0)
        
        # Act & Assert
        with self.assertRaises(LaneFollowingError) as context:
            self.lane_follower.follow_route(empty_route, self.robot_id)
        
        self.assertIn("Route must have at least one segment", str(context.exception))
    
    def test_stop_following_success(self):
        """Test successful route stopping."""
        # Arrange
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        
        # Act
        self.lane_follower.stop_following()
        
        # Assert
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.IDLE)
        self.assertIsNone(self.lane_follower._current_route)
        self.assertIsNone(self.lane_follower._current_segment)
        self.assertEqual(self.lane_follower._segment_index, 0)
        self.assertFalse(self.lane_follower._emergency_stopped)
        
        # Verify motion executor was stopped
        self.mock_motion_executor.stop_execution.assert_called_once()
    
    def test_emergency_stop(self):
        """Test emergency stop functionality."""
        # Arrange
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        
        # Act
        self.lane_follower.emergency_stop()
        
        # Assert
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.ERROR)
        self.assertTrue(self.lane_follower._emergency_stopped)
        
        # Verify emergency stop was called
        self.mock_motion_executor.emergency_stop.assert_called_once()
    
    def test_configure_lane_following(self):
        """Test lane following configuration."""
        # Arrange
        config = LaneFollowingConfig(
            lane_tolerance=0.2,
            max_speed=1.5,
            corner_speed=0.4
        )
        
        # Act
        self.lane_follower.set_config(config)
        
        # Assert
        retrieved_config = self.lane_follower.get_config()
        self.assertEqual(retrieved_config.lane_tolerance, 0.2)
        self.assertEqual(retrieved_config.max_speed, 1.5)
        self.assertEqual(retrieved_config.corner_speed, 0.4)
    
    def test_conflict_box_lock_acquisition_success(self):
        """Test successful conflict box lock acquisition."""
        # Arrange
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.return_value = True
        
        # Act
        result = self.lane_follower.try_acquire_conflict_box_lock("box_001", priority=5)
        
        # Assert
        self.assertTrue(result)
        self.assertIn("box_001", self.lane_follower._locked_boxes)
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.assert_called_once_with(
            "box_001", self.robot_id, 5
        )
    
    def test_conflict_box_lock_acquisition_failure(self):
        """Test failed conflict box lock acquisition."""
        # Arrange
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.return_value = False
        
        # Act
        result = self.lane_follower.try_acquire_conflict_box_lock("box_001")
        
        # Assert
        self.assertFalse(result)
        self.assertNotIn("box_001", self.lane_follower._locked_boxes)
    
    def test_conflict_box_lock_acquisition_database_error(self):
        """Test conflict box lock acquisition with database error."""
        # Arrange
        self.mock_simulation_data_service.try_acquire_conflict_box_lock.side_effect = \
            SimulationDataServiceError("Database connection failed")
        
        # Act & Assert
        with self.assertRaises(ConflictBoxLockError) as context:
            self.lane_follower.try_acquire_conflict_box_lock("box_001")
        
        self.assertIn("Failed to acquire lock for box_001", str(context.exception))
    
    def test_conflict_box_lock_release_success(self):
        """Test successful conflict box lock release."""
        # Arrange
        self.lane_follower._locked_boxes.add("box_001")
        self.mock_simulation_data_service.release_conflict_box_lock.return_value = True
        
        # Act
        result = self.lane_follower.release_conflict_box_lock("box_001")
        
        # Assert
        self.assertTrue(result)
        self.assertNotIn("box_001", self.lane_follower._locked_boxes)
        self.mock_simulation_data_service.release_conflict_box_lock.assert_called_once_with(
            "box_001", self.robot_id
        )
    
    def test_conflict_box_lock_release_failure(self):
        """Test failed conflict box lock release."""
        # Arrange
        self.mock_simulation_data_service.release_conflict_box_lock.return_value = False
        
        # Act
        result = self.lane_follower.release_conflict_box_lock("box_001")
        
        # Assert
        self.assertFalse(result)
    
    def test_get_conflict_box_status(self):
        """Test conflict box status retrieval."""
        # Arrange
        self.lane_follower._conflict_boxes["box_001"] = ConflictBoxState(
            box_id="box_001",
            status=ConflictBoxStatus.LOCKED
        )
        
        # Act
        status = self.lane_follower.get_conflict_box_status("box_001")
        
        # Assert
        self.assertEqual(status, ConflictBoxStatus.LOCKED)
    
    def test_get_conflict_box_status_unknown_box(self):
        """Test conflict box status for unknown box."""
        # Act
        status = self.lane_follower.get_conflict_box_status("unknown_box")
        
        # Assert
        self.assertEqual(status, ConflictBoxStatus.UNLOCKED)
    
    def test_get_held_conflict_boxes(self):
        """Test getting list of held conflict boxes."""
        # Arrange
        self.lane_follower._locked_boxes.add("box_001")
        self.lane_follower._locked_boxes.add("box_002")
        
        # Act
        held_boxes = self.lane_follower.get_held_conflict_boxes()
        
        # Assert
        self.assertEqual(set(held_boxes), {"box_001", "box_002"})
    
    def test_get_unsafe_boxes(self):
        """Test getting list of unsafe conflict boxes."""
        # Arrange
        self.lane_follower._conflict_boxes["box_001"] = ConflictBoxState(
            box_id="box_001",
            robot_inside=True
        )
        self.lane_follower._conflict_boxes["box_002"] = ConflictBoxState(
            box_id="box_002",
            robot_inside=False
        )
        
        # Act
        unsafe_boxes = self.lane_follower.get_unsafe_boxes()
        
        # Assert
        self.assertEqual(unsafe_boxes, ["box_001"])
    
    def test_report_lane_deviation_within_tolerance(self):
        """Test lane deviation reporting within tolerance."""
        # Arrange
        config = LaneFollowingConfig(lane_tolerance=0.5)
        self.lane_follower.set_config(config)
        
        robot_point = Point(0.0, 0.0)
        lane_center = Point(0.0, 0.0)
        
        # Act (should not raise exception)
        self.lane_follower.report_lane_deviation(0.3, robot_point, lane_center)
        
        # Assert - no exception raised
        self.assertEqual(self.lane_follower._consecutive_deviations, 0)
    
    def test_report_lane_deviation_exceeds_tolerance(self):
        """Test lane deviation reporting exceeding tolerance."""
        # Arrange
        config = LaneFollowingConfig(lane_tolerance=0.5)
        self.lane_follower.set_config(config)
        
        robot_point = Point(0.0, 0.0)
        lane_center = Point(0.0, 0.0)
        
        # Act
        self.lane_follower.report_lane_deviation(0.8, robot_point, lane_center)
        
        # Assert
        self.assertEqual(self.lane_follower._consecutive_deviations, 1)
    
    def test_report_lane_deviation_critical_error(self):
        """Test lane deviation reporting with critical error."""
        # Arrange
        config = LaneFollowingConfig(lane_tolerance=0.5)
        self.lane_follower.set_config(config)
        
        robot_point = Point(0.0, 0.0)
        lane_center = Point(0.0, 0.0)
        
        # Act & Assert
        with self.assertRaises(LaneDeviationError) as context:
            self.lane_follower.report_lane_deviation(1.5, robot_point, lane_center)
        
        self.assertIn("Critical lane deviation", str(context.exception))
    
    def test_is_following(self):
        """Test is_following status check."""
        # Initially not following
        self.assertFalse(self.lane_follower.is_following())
        
        # Start following
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        self.assertTrue(self.lane_follower.is_following())
        
        # Stop following
        self.lane_follower.stop_following()
        self.assertFalse(self.lane_follower.is_following())
    
    def test_get_current_route(self):
        """Test getting current route."""
        # Initially no route
        self.assertIsNone(self.lane_follower.get_current_route())
        
        # Start following
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        self.assertEqual(self.lane_follower.get_current_route(), self.test_route)
        
        # Stop following
        self.lane_follower.stop_following()
        self.assertIsNone(self.lane_follower.get_current_route())
    
    def test_get_current_segment(self):
        """Test getting current segment."""
        # Initially no segment
        self.assertIsNone(self.lane_follower.get_current_segment())
        
        # Start following
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        current_segment = self.lane_follower.get_current_segment()
        self.assertIsNotNone(current_segment)
        self.assertEqual(current_segment.lane.lane_id, "lane_001")
    
    def test_get_lane_following_result(self):
        """Test getting comprehensive following result."""
        # Arrange
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        self.lane_follower._distance_traveled = 3.0
        self.lane_follower._locked_boxes.add("box_001")
        
        # Act
        result = self.lane_follower.get_lane_following_result()
        
        # Assert
        self.assertTrue(result.success)
        self.assertEqual(result.status, LaneFollowingStatus.FOLLOWING_LANE)
        self.assertEqual(result.current_segment_index, 0)
        self.assertIsNone(result.error_message)
    
    def test_get_current_segment_progress(self):
        """Test getting current segment progress."""
        # Arrange
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        if self.lane_follower._current_segment:
            self.lane_follower._current_segment.progress_ratio = 0.75
        
        # Act
        progress = self.lane_follower.get_current_segment_progress()
        
        # Assert
        self.assertEqual(progress, 0.75)
    
    def test_get_total_route_progress(self):
        """Test getting total route progress."""
        # Arrange
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        self.lane_follower._distance_traveled = 6.0
        
        # Act
        progress = self.lane_follower.get_total_route_progress()
        
        # Assert
        self.assertEqual(progress, 0.6)  # 6.0 / 10.0
    
    def test_get_lane_deviation_stats(self):
        """Test getting lane deviation statistics."""
        # Arrange
        self.lane_follower._consecutive_deviations = 3
        
        # Act
        stats = self.lane_follower.get_lane_deviation_stats()
        
        # Assert
        self.assertEqual(stats["consecutive_deviations"], 3)
        self.assertIn("tolerance", stats)
        self.assertIn("last_deviation_time", stats)
    
    def test_update_lane_following_idle_state(self):
        """Test update lane following in idle state."""
        # Act
        self.lane_follower.update_lane_following()
        
        # Assert - should do nothing
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.IDLE)
    
    def test_update_lane_following_emergency_stopped(self):
        """Test update lane following in emergency stopped state."""
        # Arrange
        self.lane_follower._emergency_stopped = True
        
        # Act
        self.lane_follower.update_lane_following()
        
        # Assert - should do nothing
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.IDLE)
    
    def test_update_lane_following_segment_completion(self):
        """Test update lane following with segment completion."""
        # Arrange
        # Set robot position near the lane center to avoid deviation errors
        self.mock_state_holder.get_position.return_value = (0.0, 5.0)
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        self.mock_motion_executor.get_motion_status.return_value = MotionStatus.REACHED_TARGET
        
        # Act
        self.lane_follower.update_lane_following()
        
        # Assert - should complete route (single segment)
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.COMPLETED)
    
    def test_calculate_distance(self):
        """Test distance calculation utility."""
        # Act
        distance = self.lane_follower._calculate_distance((0.0, 0.0), (3.0, 4.0))
        
        # Assert
        self.assertEqual(distance, 5.0)  # 3-4-5 triangle
    
    def test_calculate_segment_distance(self):
        """Test segment distance calculation."""
        # Act
        distance = self.lane_follower._calculate_segment_distance(self.test_segment)
        
        # Assert
        self.assertEqual(distance, 10.0)  # Distance from (0,0) to (0,10)
    
    def test_is_point_in_conflict_box(self):
        """Test point in conflict box detection."""
        # Test point inside box
        inside = self.lane_follower._is_point_in_conflict_box((5.0, 5.0), self.test_conflict_box)
        self.assertTrue(inside)
        
        # Test point outside box
        outside = self.lane_follower._is_point_in_conflict_box((10.0, 10.0), self.test_conflict_box)
        self.assertFalse(outside)
        
        # Test point on edge
        edge = self.lane_follower._is_point_in_conflict_box((6.0, 5.0), self.test_conflict_box)
        self.assertTrue(edge)  # Size 2.0 means half-size 1.0, so (6,5) is on edge
    
    def test_calculate_lane_deviation(self):
        """Test lane deviation calculation."""
        # Arrange - robot at (1.0, 5.0), lane from (0,0) to (0,10)
        robot_pos = (1.0, 5.0)
        
        # Act
        deviation = self.lane_follower._calculate_lane_deviation(robot_pos, self.test_segment)
        
        # Assert
        self.assertEqual(deviation, 1.0)  # Perpendicular distance from (1,5) to line x=0
    
    def test_point_to_line_distance(self):
        """Test point to line distance calculation."""
        # Test perpendicular distance
        distance = self.lane_follower._point_to_line_distance(
            (1.0, 1.0), (0.0, 0.0), (0.0, 2.0)
        )
        self.assertEqual(distance, 1.0)
        
        # Test distance to line endpoint
        distance = self.lane_follower._point_to_line_distance(
            (1.0, 3.0), (0.0, 0.0), (0.0, 2.0)
        )
        self.assertAlmostEqual(distance, math.sqrt(2.0), places=5)
    
    def test_thread_safety(self):
        """Test thread safety of lane follower operations."""
        # Arrange
        results = []
        
        def worker():
            try:
                self.lane_follower.follow_route(self.test_route, self.robot_id)
                results.append("success")
            except Exception as e:
                results.append(f"error: {e}")
        
        # Act - multiple threads trying to start route
        threads = []
        for _ in range(5):
            thread = threading.Thread(target=worker)
            threads.append(thread)
            thread.start()
        
        for thread in threads:
            thread.join()
        
        # Assert - only one should succeed
        success_count = sum(1 for r in results if r == "success")
        error_count = sum(1 for r in results if r.startswith("error"))
        
        self.assertEqual(success_count, 1)
        self.assertEqual(error_count, 4)
    
    def test_heartbeat_thread_lifecycle(self):
        """Test heartbeat thread start and stop."""
        # Arrange
        self.lane_follower.follow_route(self.test_route, self.robot_id)
        
        # Act - heartbeat should start automatically
        time.sleep(0.1)  # Allow thread to start
        
        # Assert
        self.assertTrue(self.lane_follower._heartbeat_running)
        self.assertIsNotNone(self.lane_follower._heartbeat_thread)
        
        # Act - stop following
        self.lane_follower.stop_following()
        time.sleep(0.1)  # Allow thread to stop
        
        # Assert
        self.assertFalse(self.lane_follower._heartbeat_running)
    
    @patch('time.sleep')
    def test_heartbeat_loop_functionality(self, mock_sleep):
        """Test heartbeat loop sends heartbeats."""
        # Arrange
        self.lane_follower._locked_boxes.add("box_001")
        self.lane_follower._heartbeat_running = True
        mock_sleep.side_effect = [None, Exception("Stop loop")]  # Stop after one iteration
        
        # Act
        try:
            self.lane_follower._heartbeat_loop()
        except Exception:
            pass  # Expected from mock
        
        # Assert
        self.mock_simulation_data_service.heartbeat_conflict_box_lock.assert_called_with(
            "box_001", self.robot_id
        )


if __name__ == '__main__':
    unittest.main() 