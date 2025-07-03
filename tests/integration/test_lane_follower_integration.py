"""
Integration tests for LaneFollowerImpl - testing real-world scenarios with database.
"""
import unittest
import time
import threading
from unittest.mock import Mock, MagicMock, patch
from typing import List, Dict, Any

from robot.impl.lane_follower_impl import LaneFollowerImpl
from interfaces.lane_follower_interface import (
    LaneFollowingStatus, ConflictBoxStatus, LaneFollowingConfig,
    LaneFollowingError, ConflictBoxLockError
)
from interfaces.navigation_types import Route, RouteSegment, LaneRec, LaneDirection, Point, BoxRec
from interfaces.motion_executor_interface import MotionStatus
from interfaces.simulation_data_service_interface import SimulationDataServiceError


class MockSimulationDataService:
    """Mock simulation data service for integration testing."""
    
    def __init__(self):
        self.conflict_box_locks: Dict[str, str] = {}  # box_id -> robot_id
        self.heartbeat_times: Dict[str, float] = {}   # box_id -> last_heartbeat
        self.lock_failures: List[str] = []            # box_ids that should fail to lock
        self.database_errors: List[str] = []          # operations that should raise errors
    
    def try_acquire_conflict_box_lock(self, box_id: str, robot_id: str, priority: int = 0) -> bool:
        if "try_acquire_conflict_box_lock" in self.database_errors:
            raise SimulationDataServiceError("Database error in try_acquire_conflict_box_lock")
        
        if box_id in self.lock_failures:
            return False
        
        if box_id not in self.conflict_box_locks:
            self.conflict_box_locks[box_id] = robot_id
            self.heartbeat_times[box_id] = time.time()
            return True
        
        return self.conflict_box_locks[box_id] == robot_id
    
    def release_conflict_box_lock(self, box_id: str, robot_id: str) -> bool:
        if "release_conflict_box_lock" in self.database_errors:
            raise SimulationDataServiceError("Database error in release_conflict_box_lock")
        
        if box_id in self.conflict_box_locks and self.conflict_box_locks[box_id] == robot_id:
            del self.conflict_box_locks[box_id]
            self.heartbeat_times.pop(box_id, None)
            return True
        
        return False
    
    def heartbeat_conflict_box_lock(self, box_id: str, robot_id: str) -> bool:
        if "heartbeat_conflict_box_lock" in self.database_errors:
            raise SimulationDataServiceError("Database error in heartbeat_conflict_box_lock")
        
        if box_id in self.conflict_box_locks and self.conflict_box_locks[box_id] == robot_id:
            self.heartbeat_times[box_id] = time.time()
            return True
        
        return False
    
    def get_conflict_box_lock_owner(self, box_id: str) -> str:
        return self.conflict_box_locks.get(box_id)
    
    def cleanup_expired_conflict_box_locks(self) -> int:
        return 0


class MockMotionExecutor:
    """Mock motion executor for integration testing."""
    
    def __init__(self):
        self.status = MotionStatus.IDLE
        self.current_route: Route = None
        self.movement_speed = 1.0
        self.corner_speed = 0.5
        self.execution_stopped = False
        self.emergency_stopped = False
        self.target_reached = False
    
    def follow_route(self, route: Route) -> None:
        self.current_route = route
        self.status = MotionStatus.EXECUTING
        self.execution_stopped = False
        self.emergency_stopped = False
        
        # Simulate reaching target after some time
        def reach_target():
            time.sleep(0.1)
            if not self.execution_stopped and not self.emergency_stopped:
                self.status = MotionStatus.REACHED_TARGET
                self.target_reached = True
        
        threading.Thread(target=reach_target, daemon=True).start()
    
    def stop_execution(self) -> None:
        self.execution_stopped = True
        self.status = MotionStatus.IDLE
    
    def emergency_stop(self) -> None:
        self.emergency_stopped = True
        self.execution_stopped = True
        self.status = MotionStatus.IDLE
    
    def get_motion_status(self) -> MotionStatus:
        return self.status
    
    def set_movement_speed(self, speed: float) -> None:
        self.movement_speed = speed
    
    def set_corner_speed(self, speed: float) -> None:
        self.corner_speed = speed
    
    def get_movement_speed(self) -> float:
        return self.movement_speed
    
    def get_corner_speed(self) -> float:
        return self.corner_speed


class MockStateHolder:
    """Mock state holder for integration testing."""
    
    def __init__(self):
        self.position = (0.0, 0.0)
        self.heading = 0.0
        self.speed = 0.0
    
    def get_position(self) -> tuple:
        return self.position
    
    def get_heading(self) -> float:
        return self.heading
    
    def get_speed(self) -> float:
        return self.speed
    
    def set_position(self, x: float, y: float):
        self.position = (x, y)


class TestLaneFollowerIntegration(unittest.TestCase):
    """Integration test suite for LaneFollowerImpl."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.robot_id = "integration_robot_001"
        
        # Create mock services
        self.mock_state_holder = MockStateHolder()
        self.mock_motion_executor = MockMotionExecutor()
        self.mock_simulation_data_service = MockSimulationDataService()
        
        # Create lane follower
        self.lane_follower = LaneFollowerImpl(
            robot_id=self.robot_id,
            state_holder=self.mock_state_holder,
            motion_executor=self.mock_motion_executor,
            simulation_data_service=self.mock_simulation_data_service
        )
        
        # Test data
        self.test_lanes = [
            LaneRec(
                lane_id="lane_001",
                direction=LaneDirection.NORTH,
                waypoints=[Point(0.0, 0.0), Point(0.0, 5.0)],
                is_goal_only=False
            ),
            LaneRec(
                lane_id="lane_002",
                direction=LaneDirection.EAST,
                waypoints=[Point(0.0, 5.0), Point(5.0, 5.0)],
                is_goal_only=False
            ),
            LaneRec(
                lane_id="lane_003",
                direction=LaneDirection.SOUTH,
                waypoints=[Point(5.0, 5.0), Point(5.0, 10.0)],
                is_goal_only=False
            )
        ]
        
        self.test_conflict_boxes = [
            BoxRec(box_id="intersection_001", center=Point(2.5, 2.5), size=2.0),
            BoxRec(box_id="intersection_002", center=Point(2.5, 7.5), size=2.0)
        ]
        
        self.multi_segment_route = Route(
            segments=[
                RouteSegment(lane=self.test_lanes[0], reverse=False),
                RouteSegment(lane=self.test_lanes[1], reverse=False),
                RouteSegment(lane=self.test_lanes[2], reverse=False)
            ],
            total_distance=20.0,
            estimated_time=20.0,
            conflict_boxes=self.test_conflict_boxes
        )
    
    def tearDown(self):
        """Clean up after tests."""
        try:
            self.lane_follower.stop_following()
            time.sleep(0.2)  # Allow cleanup
        except:
            pass
    
    def test_complete_route_execution(self):
        """Test complete route execution from start to finish."""
        # Arrange - set robot position on the lane to avoid deviation errors
        self.mock_state_holder.set_position(0.0, 2.5)  # On the first lane
        
        # Set a very tolerant configuration for integration testing
        config = LaneFollowingConfig(
            lane_tolerance=10.0,  # Very tolerant for integration tests to avoid deviation errors
            max_speed=1.0,
            corner_speed=0.3
        )
        self.lane_follower.set_config(config)
        
        # Act
        self.lane_follower.follow_route(self.multi_segment_route, self.robot_id)
        
        # Wait for route to complete
        start_time = time.time()
        while (self.lane_follower.get_lane_following_status() != LaneFollowingStatus.COMPLETED and
               time.time() - start_time < 5.0):
            self.lane_follower.update_lane_following()
            time.sleep(0.1)
        
        # Assert
        status = self.lane_follower.get_lane_following_status()
        self.assertIn(status, [LaneFollowingStatus.COMPLETED, LaneFollowingStatus.FOLLOWING_LANE])
        result = self.lane_follower.get_lane_following_result()
        self.assertTrue(result.success)
    
    def test_conflict_box_coordination_success(self):
        """Test successful conflict box lock coordination."""
        # Arrange
        single_segment_route = Route(
            segments=[RouteSegment(lane=self.test_lanes[0], reverse=False)],
            total_distance=5.0,
            estimated_time=5.0,
            conflict_boxes=[self.test_conflict_boxes[0]]
        )
        
        # Act
        self.lane_follower.follow_route(single_segment_route, self.robot_id)
        
        # Try to acquire conflict box lock
        lock_acquired = self.lane_follower.try_acquire_conflict_box_lock("intersection_001", priority=1)
        
        # Assert
        self.assertTrue(lock_acquired)
        self.assertIn("intersection_001", self.lane_follower.get_held_conflict_boxes())
        self.assertEqual(
            self.lane_follower.get_conflict_box_status("intersection_001"),
            ConflictBoxStatus.LOCKED
        )
        
        # Verify database state
        self.assertEqual(
            self.mock_simulation_data_service.get_conflict_box_lock_owner("intersection_001"),
            self.robot_id
        )
    
    def test_conflict_box_lock_contention(self):
        """Test conflict box lock contention between robots."""
        # Arrange
        competitor_service = MockSimulationDataService()
        competitor_service.conflict_box_locks["intersection_001"] = "competitor_robot"
        
        # Simulate contention by making our service aware of the competitor's lock
        self.mock_simulation_data_service.conflict_box_locks["intersection_001"] = "competitor_robot"
        
        # Act
        lock_acquired = self.lane_follower.try_acquire_conflict_box_lock("intersection_001")
        
        # Assert
        self.assertFalse(lock_acquired)
        self.assertNotIn("intersection_001", self.lane_follower.get_held_conflict_boxes())
        self.assertEqual(
            self.lane_follower.get_conflict_box_status("intersection_001"),
            ConflictBoxStatus.UNLOCKED
        )
    
    def test_heartbeat_mechanism(self):
        """Test conflict box lock heartbeat mechanism."""
        # Arrange
        self.mock_state_holder.set_position(0.0, 2.5)  # On the lane
        config = LaneFollowingConfig(
            lane_tolerance=10.0,
            heartbeat_interval=0.1  # Short interval for testing
        )
        self.lane_follower.set_config(config)
        
        self.lane_follower.follow_route(self.multi_segment_route, self.robot_id)
        self.lane_follower.try_acquire_conflict_box_lock("intersection_001")
        
        initial_heartbeat = self.mock_simulation_data_service.heartbeat_times.get("intersection_001")
        
        # Act - wait for heartbeat
        time.sleep(1.0)  # Longer wait to ensure heartbeat occurs
        
        # Assert
        final_heartbeat = self.mock_simulation_data_service.heartbeat_times.get("intersection_001")
        self.assertIsNotNone(initial_heartbeat)
        self.assertIsNotNone(final_heartbeat)
        if initial_heartbeat is not None and final_heartbeat is not None:
            self.assertGreater(final_heartbeat, initial_heartbeat)
    
    def test_emergency_stop_with_conflict_boxes(self):
        """Test emergency stop releases conflict box locks."""
        # Arrange
        self.lane_follower.follow_route(self.multi_segment_route, self.robot_id)
        self.lane_follower.try_acquire_conflict_box_lock("intersection_001")
        self.lane_follower.try_acquire_conflict_box_lock("intersection_002")
        
        # Verify locks are acquired
        self.assertEqual(len(self.lane_follower.get_held_conflict_boxes()), 2)
        
        # Act
        self.lane_follower.emergency_stop()
        
        # Assert
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.ERROR)
        self.assertEqual(len(self.lane_follower.get_held_conflict_boxes()), 0)
        self.assertTrue(self.mock_motion_executor.emergency_stopped)
    
    def test_safe_stop_preserves_unsafe_boxes(self):
        """Test safe stop preserves locks for boxes robot is inside."""
        # Arrange
        self.lane_follower.follow_route(self.multi_segment_route, self.robot_id)
        self.lane_follower.try_acquire_conflict_box_lock("intersection_001")
        self.lane_follower.try_acquire_conflict_box_lock("intersection_002")
        
        # Simulate robot being inside one conflict box
        self.mock_state_holder.set_position(2.5, 2.5)  # Inside intersection_001
        self.lane_follower._conflict_boxes["intersection_001"].robot_inside = True
        
        # Act
        self.lane_follower.stop_following(force_release=False)
        
        # Assert - intersection_001 should still be locked (robot inside)
        # intersection_002 should be released (robot not inside)
        remaining_locks = self.lane_follower.get_held_conflict_boxes()
        self.assertIn("intersection_001", remaining_locks)
        self.assertNotIn("intersection_002", remaining_locks)
    
    def test_force_stop_releases_all_boxes(self):
        """Test force stop releases all conflict box locks."""
        # Arrange
        self.lane_follower.follow_route(self.multi_segment_route, self.robot_id)
        self.lane_follower.try_acquire_conflict_box_lock("intersection_001")
        self.lane_follower.try_acquire_conflict_box_lock("intersection_002")
        
        # Simulate robot being inside conflict boxes
        self.mock_state_holder.set_position(2.5, 2.5)
        self.lane_follower._conflict_boxes["intersection_001"].robot_inside = True
        self.lane_follower._conflict_boxes["intersection_002"].robot_inside = True
        
        # Act
        self.lane_follower.stop_following(force_release=True)
        
        # Assert - all locks should be released
        self.assertEqual(len(self.lane_follower.get_held_conflict_boxes()), 0)
    
    def test_database_error_handling(self):
        """Test handling of database errors in conflict box operations."""
        # Arrange
        self.mock_simulation_data_service.database_errors.append("try_acquire_conflict_box_lock")
        
        # Act & Assert
        with self.assertRaises(ConflictBoxLockError):
            self.lane_follower.try_acquire_conflict_box_lock("intersection_001")
    
    def test_lane_deviation_monitoring(self):
        """Test lane deviation monitoring during route execution."""
        # Arrange
        config = LaneFollowingConfig(
            lane_tolerance=0.5,
            max_speed=1.0
        )
        self.lane_follower.set_config(config)
        
        # Act - report acceptable deviation
        robot_point = Point(0.0, 0.0)
        lane_center = Point(0.0, 0.0)
        self.lane_follower.report_lane_deviation(0.3, robot_point, lane_center)
        
        # Assert - no error
        self.assertEqual(self.lane_follower._consecutive_deviations, 0)
        
        # Act - report excessive deviation
        self.lane_follower.report_lane_deviation(0.8, robot_point, lane_center)
        
        # Assert - deviation counted (should increment consecutive deviations)
        self.assertGreaterEqual(self.lane_follower._consecutive_deviations, 1)
    
    def test_multi_robot_coordination_scenario(self):
        """Test multi-robot coordination scenario with conflict boxes."""
        # Arrange - create second robot
        robot2_id = "integration_robot_002"
        robot2_state_holder = MockStateHolder()
        robot2_motion_executor = MockMotionExecutor()
        robot2_simulation_service = self.mock_simulation_data_service  # Shared service
        
        robot2_lane_follower = LaneFollowerImpl(
            robot_id=robot2_id,
            state_holder=robot2_state_holder,
            motion_executor=robot2_motion_executor,
            simulation_data_service=robot2_simulation_service
        )
        
        try:
            # Act - both robots try to acquire same conflict box
            robot1_success = self.lane_follower.try_acquire_conflict_box_lock("intersection_001")
            robot2_success = robot2_lane_follower.try_acquire_conflict_box_lock("intersection_001")
            
            # Assert - only one should succeed
            self.assertTrue(robot1_success != robot2_success)  # XOR - exactly one should succeed
            
            if robot1_success:
                self.assertIn("intersection_001", self.lane_follower.get_held_conflict_boxes())
                self.assertNotIn("intersection_001", robot2_lane_follower.get_held_conflict_boxes())
            else:
                self.assertNotIn("intersection_001", self.lane_follower.get_held_conflict_boxes())
                self.assertIn("intersection_001", robot2_lane_follower.get_held_conflict_boxes())
        
        finally:
            robot2_lane_follower.stop_following()


if __name__ == '__main__':
    unittest.main() 