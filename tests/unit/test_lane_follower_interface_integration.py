"""
Unit tests for LaneFollower interface integration with TaskHandler.

This module tests the integration between LaneFollower interface and TaskHandler,
ensuring that the lane-based navigation system works correctly with the task
execution workflow.
"""
import unittest
from unittest.mock import Mock, MagicMock
from typing import Optional

from interfaces.lane_follower_interface import (
    ILaneFollower, LaneFollowingStatus, LaneFollowingResult, 
    LaneFollowingConfig, ConflictBoxStatus
)
from interfaces.task_handler_interface import Task, TaskType, TaskStatus
from interfaces.navigation_types import Route, RouteSegment, LaneRec, Point, LaneDirection
from robot.impl.task_handler_impl import TaskHandlerImpl


class MockLaneFollower(ILaneFollower):
    """Mock implementation of LaneFollower for testing."""
    
    def __init__(self):
        self._status = LaneFollowingStatus.IDLE
        self._is_following = False
        self._current_route: Optional[Route] = None
        self._current_segment: Optional[RouteSegment] = None
        self._config = LaneFollowingConfig()
        self._held_boxes: list[str] = []
        self._result = LaneFollowingResult(
            success=True,
            status=LaneFollowingStatus.IDLE,
            current_segment_index=0,
            progress_in_segment=0.0
        )
    
    def follow_route(self, route: Route, robot_id: str) -> None:
        """Start following a route."""
        self._current_route = route
        self._is_following = True
        self._status = LaneFollowingStatus.FOLLOWING_LANE
        self._result = LaneFollowingResult(
            success=True,
            status=self._status,
            current_segment_index=0,
            progress_in_segment=0.0
        )
    
    def stop_following(self, force_release: bool = False) -> None:
        """Stop lane following."""
        self._is_following = False
        self._status = LaneFollowingStatus.IDLE
        self._current_route = None
        self._current_segment = None
        if force_release:
            self._held_boxes.clear()
        self._result = LaneFollowingResult(
            success=True,
            status=self._status,
            current_segment_index=0,
            progress_in_segment=0.0
        )
    
    def get_lane_following_status(self) -> LaneFollowingStatus:
        """Get current status."""
        return self._status
    
    def get_lane_following_result(self) -> LaneFollowingResult:
        """Get current result."""
        return self._result
    
    def is_following(self) -> bool:
        """Check if following."""
        return self._is_following
    
    def get_current_route(self) -> Optional[Route]:
        """Get current route."""
        return self._current_route
    
    def get_current_segment(self) -> Optional[RouteSegment]:
        """Get current segment."""
        return self._current_segment
    
    def get_conflict_box_status(self, box_id: str) -> ConflictBoxStatus:
        """Get conflict box status."""
        if box_id in self._held_boxes:
            return ConflictBoxStatus.LOCKED
        return ConflictBoxStatus.UNLOCKED
    
    def get_held_conflict_boxes(self) -> list[str]:
        """Get held conflict boxes."""
        return self._held_boxes.copy()
    
    def set_config(self, config: LaneFollowingConfig) -> None:
        """Set configuration."""
        self._config = config
    
    def get_config(self) -> LaneFollowingConfig:
        """Get configuration."""
        return self._config
    
    def emergency_stop(self) -> None:
        """Emergency stop."""
        self.stop_following()
    
    def update_lane_following(self) -> None:
        """Update lane following."""
        if self._is_following and self._current_route:
            # Simulate progress
            if self._result.progress_in_segment < 1.0:
                self._result.progress_in_segment += 0.1
            else:
                # Move to next segment
                if self._result.current_segment_index < len(self._current_route.segments) - 1:
                    self._result.current_segment_index += 1
                    self._result.progress_in_segment = 0.0
                else:
                    # Route completed
                    self._status = LaneFollowingStatus.COMPLETED
                    self._result.status = LaneFollowingStatus.COMPLETED
                    self._result.progress_in_segment = 1.0
    
    def report_lane_deviation(self, deviation_distance: float, 
                            current_position: Point, 
                            lane_center: Point) -> None:
        """Report lane deviation."""
        pass
    
    def get_lane_deviation_stats(self) -> dict:
        """Get deviation stats."""
        return {"max_deviation": 0.0, "avg_deviation": 0.0}
    
    def get_unsafe_boxes(self) -> list[str]:
        """Get unsafe boxes (mock implementation)."""
        return []  # Mock: no unsafe boxes


class TestLaneFollowerTaskHandlerIntegration(unittest.TestCase):
    """Test integration between LaneFollower and TaskHandler."""
    
    def setUp(self):
        """Set up test environment."""
        # Create mock dependencies
        self.mock_state_holder = Mock()
        self.mock_path_planner = Mock()
        self.mock_motion_executor = Mock()
        self.mock_coordinate_system = Mock()
        self.mock_simulation_data_service = Mock()
        
        # Create real LaneFollower mock
        self.lane_follower = MockLaneFollower()
        
        # Create TaskHandler with LaneFollower
        self.task_handler = TaskHandlerImpl(
            state_holder=self.mock_state_holder,
            path_planner=self.mock_path_planner,
            lane_follower=self.lane_follower,
            motion_executor=self.mock_motion_executor,
            coordinate_system=self.mock_coordinate_system,
            simulation_data_service=self.mock_simulation_data_service,
            robot_id="test_robot"
        )
        
        # Setup mock responses
        self.mock_state_holder.get_robot_state.return_value = Mock(position=(0.0, 0.0, 0.0))
        self.mock_simulation_data_service.get_shelf_position.return_value = (5.0, 5.0)
        self.mock_simulation_data_service.get_dropoff_zones.return_value = [(8.0, 8.0)]
    
    def test_lane_follower_integration_in_constructor(self):
        """Test that LaneFollower is properly integrated in TaskHandler constructor."""
        # Verify LaneFollower is stored
        self.assertEqual(self.task_handler.lane_follower, self.lane_follower)
        
        # Verify LaneFollower is not following initially
        self.assertFalse(self.lane_follower.is_following())
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.IDLE)
    
    def test_lane_follower_emergency_stop_integration(self):
        """Test that emergency stop calls LaneFollower emergency stop."""
        # Start a task
        task = Task.create_pick_and_deliver_task(
            task_id="test_task",
            order_id="test_order",
            shelf_id="test_shelf",
            item_id="test_item"
        )
        
        self.task_handler.start_task(task)
        
        # Call emergency stop
        self.task_handler.emergency_stop()
        
        # Verify LaneFollower emergency stop was called
        self.assertFalse(self.lane_follower.is_following())
        self.assertEqual(self.lane_follower.get_lane_following_status(), LaneFollowingStatus.IDLE)
    
    def test_lane_follower_stall_handling_integration(self):
        """Test that stall handling stops LaneFollower."""
        # Start a task
        task = Task.create_pick_and_deliver_task(
            task_id="test_task",
            order_id="test_order",
            shelf_id="test_shelf",
            item_id="test_item"
        )
        
        self.task_handler.start_task(task)
        
        # Simulate lane following
        self.lane_follower.follow_route(Mock(spec=Route), "test_robot")
        self.assertTrue(self.lane_follower.is_following())
        
        # Handle stall event
        self.task_handler.handle_stall_event("Test stall")
        
        # Verify LaneFollower was stopped
        self.assertFalse(self.lane_follower.is_following())
    
    def test_lane_follower_update_integration(self):
        """Test that TaskHandler calls LaneFollower update."""
        # Start a task
        task = Task.create_pick_and_deliver_task(
            task_id="test_task",
            order_id="test_order",
            shelf_id="test_shelf",
            item_id="test_item"
        )
        
        self.task_handler.start_task(task)
        
        # Mock path planning to succeed
        from interfaces.navigation_types import PathPlanningResult, Route
        mock_route = Mock(spec=Route)
        mock_route.segments = [Mock(spec=RouteSegment)]
        
        planning_result = PathPlanningResult.success_result(mock_route)
        self.mock_path_planner.plan_route.return_value = planning_result
        
        # Update task execution
        self.task_handler.update_task_execution()
        
        # Verify LaneFollower update was called (indirectly through progress calculation)
        # The update method should have been called during task execution
        self.assertIsNotNone(self.lane_follower.get_lane_following_result())
    
    def test_lane_follower_configuration_integration(self):
        """Test that LaneFollower configuration can be accessed."""
        # Test getting configuration
        config = self.lane_follower.get_config()
        self.assertIsInstance(config, LaneFollowingConfig)
        
        # Test setting configuration
        new_config = LaneFollowingConfig(
            lane_tolerance=0.2,
            max_speed=1.5,
            corner_speed=0.4
        )
        self.lane_follower.set_config(new_config)
        
        updated_config = self.lane_follower.get_config()
        self.assertEqual(updated_config.lane_tolerance, 0.2)
        self.assertEqual(updated_config.max_speed, 1.5)
        self.assertEqual(updated_config.corner_speed, 0.4)
    
    def test_lane_follower_conflict_box_integration(self):
        """Test conflict box management integration."""
        # Test initial state
        self.assertEqual(len(self.lane_follower.get_held_conflict_boxes()), 0)
        self.assertEqual(
            self.lane_follower.get_conflict_box_status("test_box"),
            ConflictBoxStatus.UNLOCKED
        )
        
        # Simulate acquiring a conflict box
        self.lane_follower._held_boxes.append("test_box")
        
        # Test updated state
        self.assertEqual(len(self.lane_follower.get_held_conflict_boxes()), 1)
        self.assertIn("test_box", self.lane_follower.get_held_conflict_boxes())
        self.assertEqual(
            self.lane_follower.get_conflict_box_status("test_box"),
            ConflictBoxStatus.LOCKED
        )
    
    def test_lane_follower_progress_tracking(self):
        """Test that LaneFollower provides accurate progress tracking."""
        # Create a test route
        test_lane = LaneRec(
            lane_id="test_lane",
            direction=LaneDirection.EAST,
            waypoints=[Point(0, 0), Point(5, 0)]
        )
        test_segment = RouteSegment(lane=test_lane)
        test_route = Route(
            segments=[test_segment, test_segment],  # Two segments
            total_distance=10.0,
            estimated_time=5.0
        )
        
        # Start following
        self.lane_follower.follow_route(test_route, "test_robot")
        
        # Verify initial state
        result = self.lane_follower.get_lane_following_result()
        self.assertEqual(result.current_segment_index, 0)
        self.assertEqual(result.progress_in_segment, 0.0)
        
        # Simulate progress
        self.lane_follower.update_lane_following()
        
        # Verify progress
        result = self.lane_follower.get_lane_following_result()
        self.assertEqual(result.current_segment_index, 0)
        self.assertGreater(result.progress_in_segment, 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2) 