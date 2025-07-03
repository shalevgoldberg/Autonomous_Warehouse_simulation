"""
LaneFollower Implementation - Professional lane-based navigation with conflict box management.

This implementation provides precise lane following with center-line tracking,
conflict box coordination, and comprehensive safety features.
"""
import threading
import time
import math
from typing import List, Optional, Dict, Set, Tuple, Any
from dataclasses import dataclass, field
from enum import Enum
import logging

from interfaces.lane_follower_interface import (
    ILaneFollower, LaneFollowingStatus, ConflictBoxStatus, LaneFollowingResult,
    LaneFollowingConfig, LaneFollowingError, ConflictBoxLockError, LaneDeviationError
)
from interfaces.navigation_types import Route, RouteSegment, Point, BoxRec
from interfaces.state_holder_interface import IStateHolder
from interfaces.motion_executor_interface import IMotionExecutor, MotionStatus
from interfaces.simulation_data_service_interface import ISimulationDataService


@dataclass
class ConflictBoxState:
    """Internal state for conflict box management."""
    box_id: str
    status: ConflictBoxStatus = ConflictBoxStatus.UNLOCKED
    lock_attempt_time: Optional[float] = None
    heartbeat_time: Optional[float] = None
    robot_inside: bool = False
    priority: int = 0


@dataclass
class LaneSegmentProgress:
    """Progress tracking for a lane segment."""
    segment_index: int
    segment: RouteSegment
    progress_ratio: float = 0.0  # 0.0 to 1.0
    distance_remaining: float = 0.0
    current_target: Optional[Point] = None


class LaneFollowerImpl(ILaneFollower):
    """
    Professional lane follower implementation with SOLID principles.
    
    Responsibilities:
    - Precise lane center-line following within configurable tolerances
    - Conflict box lock acquisition, heartbeat, and release management
    - Route segment execution with smooth transitions
    - Safety monitoring and collision avoidance
    - Thread-safe operation with proper synchronization
    """
    
    def __init__(self, 
                 robot_id: str,
                 state_holder: IStateHolder,
                 motion_executor: IMotionExecutor,
                 simulation_data_service: ISimulationDataService):
        """
        Initialize the lane follower.
        
        Args:
            robot_id: Unique identifier for this robot
            state_holder: Robot state management service
            motion_executor: Motion control service
            simulation_data_service: Database and navigation data service
        """
        self._robot_id = robot_id
        self._state_holder = state_holder
        self._motion_executor = motion_executor
        self._simulation_data_service = simulation_data_service
        
        # Thread safety
        self._lock = threading.RLock()
        self._logger = logging.getLogger(f"LaneFollower.{robot_id}")
        
        # Configuration
        self._config = LaneFollowingConfig()
        
        # State management
        self._status = LaneFollowingStatus.IDLE
        self._current_route: Optional[Route] = None
        self._current_segment: Optional[LaneSegmentProgress] = None
        self._segment_index = 0
        
        # Conflict box management
        self._conflict_boxes: Dict[str, ConflictBoxState] = {}
        self._locked_boxes: Set[str] = set()
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._heartbeat_running = False
        
        # Progress tracking
        self._total_distance = 0.0
        self._distance_traveled = 0.0
        self._route_start_time: Optional[float] = None
        
        # Safety monitoring
        self._last_deviation_report_time = 0.0
        self._consecutive_deviations = 0
        
        # Emergency state
        self._emergency_stopped = False
        
        self._logger.info(f"LaneFollower initialized for robot {robot_id}")
    
    def follow_route(self, route: Route, robot_id: str) -> None:
        """Start following a lane-based route."""
        with self._lock:
            if self._status != LaneFollowingStatus.IDLE:
                raise LaneFollowingError(f"Cannot start route while status is {self._status}")
            
            if not route.segments:
                raise LaneFollowingError("Route must have at least one segment")
            
            self._logger.info(f"Starting route with {len(route.segments)} segments")
            
            # Reset state
            self._current_route = route
            self._segment_index = 0
            self._total_distance = route.total_distance
            self._distance_traveled = 0.0
            self._route_start_time = time.time()
            self._emergency_stopped = False
            
            # Initialize conflict box states
            self._conflict_boxes.clear()
            self._locked_boxes.clear()
            if route.conflict_boxes:
                for box in route.conflict_boxes:
                    self._conflict_boxes[box.box_id] = ConflictBoxState(box.box_id)
            
            # Start first segment
            self._start_next_segment()
            
            # Start heartbeat thread
            self._start_heartbeat_thread()
    
    def stop_following(self, force_release: bool = False) -> None:
        """Stop following the current route."""
        with self._lock:
            if self._status == LaneFollowingStatus.IDLE:
                return
            
            self._logger.info(f"Stopping route following (force_release={force_release})")
            
            # Stop motion
            self._motion_executor.stop_execution()
            
            # Stop heartbeat thread
            self._stop_heartbeat_thread()
            
            # Release conflict box locks
            self._release_all_locks(force_release)
            
            # Reset state
            self._status = LaneFollowingStatus.IDLE
            self._current_route = None
            self._current_segment = None
            self._segment_index = 0
            self._emergency_stopped = False
    
    def emergency_stop(self) -> None:
        """Emergency stop - immediately halt all motion and release locks."""
        with self._lock:
            self._logger.warning("Emergency stop activated")
            self._emergency_stopped = True
            
            # Immediate motion stop
            self._motion_executor.emergency_stop()
            
            # Force release all locks (safety critical)
            self._release_all_locks(force_release=True)
            
            # Stop heartbeat
            self._stop_heartbeat_thread()
            
            # Set status
            self._status = LaneFollowingStatus.ERROR
    
    def update_lane_following(self) -> None:
        """Update lane following logic - called from control loop."""
        with self._lock:
            if self._status == LaneFollowingStatus.IDLE or self._emergency_stopped:
                return
            
            try:
                # Update current segment progress
                self._update_segment_progress()
                
                # Check for segment completion
                if self._is_segment_complete():
                    self._complete_current_segment()
                
                # Handle conflict box transitions
                self._update_conflict_box_states()
                
                # Safety monitoring
                self._monitor_lane_deviation()
                
            except Exception as e:
                self._logger.error(f"Error in lane following update: {e}")
                self._handle_error(str(e))
    
    def get_lane_following_status(self) -> LaneFollowingStatus:
        """Get current lane following status."""
        with self._lock:
            return self._status
    
    def get_current_segment_progress(self) -> float:
        """Get progress through current segment (0.0 to 1.0)."""
        with self._lock:
            if self._current_segment is None:
                return 0.0
            return self._current_segment.progress_ratio
    
    def get_total_route_progress(self) -> float:
        """Get total route progress (0.0 to 1.0)."""
        with self._lock:
            if self._total_distance <= 0:
                return 0.0
            return min(1.0, self._distance_traveled / self._total_distance)
    
    def get_lane_following_result(self) -> LaneFollowingResult:
        """Get comprehensive following result."""
        with self._lock:
            return LaneFollowingResult(
                success=self._status not in [LaneFollowingStatus.ERROR, LaneFollowingStatus.BLOCKED],
                status=self._status,
                current_segment_index=self._segment_index,
                progress_in_segment=self.get_current_segment_progress(),
                error_message=None if self._status != LaneFollowingStatus.ERROR else "Lane following error"
            )
    
    def set_config(self, config: LaneFollowingConfig) -> None:
        """Configure lane following parameters."""
        with self._lock:
            self._config = config
            self._logger.info(f"Lane following configured: tolerance={config.lane_tolerance}m, "
                            f"speed={config.max_speed}m/s")
    
    def get_config(self) -> LaneFollowingConfig:
        """Get current lane following configuration."""
        with self._lock:
            return self._config
    
    def is_following(self) -> bool:
        """Check if currently following a route."""
        with self._lock:
            return self._status not in [LaneFollowingStatus.IDLE, LaneFollowingStatus.ERROR]
    
    def get_current_route(self) -> Optional[Route]:
        """Get the current route being followed."""
        with self._lock:
            return self._current_route
    
    def get_current_segment(self) -> Optional[RouteSegment]:
        """Get the current route segment being executed."""
        with self._lock:
            if self._current_segment:
                return self._current_segment.segment
            return None
    
    def get_conflict_box_status(self, box_id: str) -> ConflictBoxStatus:
        """Get status of a specific conflict box."""
        with self._lock:
            if box_id in self._conflict_boxes:
                return self._conflict_boxes[box_id].status
            return ConflictBoxStatus.UNLOCKED
    
    def get_held_conflict_boxes(self) -> List[str]:
        """Get list of conflict boxes currently held by this robot."""
        with self._lock:
            return list(self._locked_boxes)
    
    def get_unsafe_boxes(self) -> List[str]:
        """Get list of conflict boxes where robot is currently inside."""
        with self._lock:
            unsafe_boxes = []
            robot_pos = self._state_holder.get_position()
            
            for box_id, box_state in self._conflict_boxes.items():
                if box_state.robot_inside:
                    unsafe_boxes.append(box_id)
            
            return unsafe_boxes
    
    def report_lane_deviation(self, deviation_distance: float, 
                            current_position: Point, 
                            lane_center: Point) -> None:
        """Report lane deviation for safety monitoring."""
        current_time = time.time()
        
        with self._lock:
            # Rate limiting
            if current_time - self._last_deviation_report_time < 1.0:
                return
            
            self._last_deviation_report_time = current_time
            
            if deviation_distance > self._config.lane_tolerance:
                self._consecutive_deviations += 1
                self._logger.warning(f"Lane deviation: {deviation_distance:.3f}m")
                
                # Critical deviation handling - using reasonable defaults
                max_deviation = 1.0  # 1 meter max deviation
                max_consecutive = 5  # 5 consecutive deviations
                
                if (deviation_distance > max_deviation or 
                    self._consecutive_deviations > max_consecutive):
                    
                    error_msg = (f"Critical lane deviation: {deviation_distance:.3f}m, "
                               f"consecutive: {self._consecutive_deviations}")
                    self._logger.error(error_msg)
                    raise LaneDeviationError(error_msg)
            else:
                self._consecutive_deviations = 0
    
    def get_lane_deviation_stats(self) -> Dict[str, Any]:
        """Get lane deviation statistics."""
        with self._lock:
            return {
                "consecutive_deviations": self._consecutive_deviations,
                "last_deviation_time": self._last_deviation_report_time,
                "tolerance": self._config.lane_tolerance
            }
    
    def try_acquire_conflict_box_lock(self, box_id: str, priority: int = 0) -> bool:
        """Try to acquire a conflict box lock."""
        with self._lock:
            try:
                success = self._simulation_data_service.try_acquire_conflict_box_lock(
                    box_id, self._robot_id, priority)
                
                if success:
                    self._locked_boxes.add(box_id)
                    if box_id in self._conflict_boxes:
                        self._conflict_boxes[box_id].status = ConflictBoxStatus.LOCKED
                        self._conflict_boxes[box_id].heartbeat_time = time.time()
                        self._conflict_boxes[box_id].priority = priority
                    
                    self._logger.info(f"Acquired conflict box lock: {box_id}")
                else:
                    self._logger.debug(f"Failed to acquire conflict box lock: {box_id}")
                
                return success
                
            except Exception as e:
                self._logger.error(f"Error acquiring conflict box lock {box_id}: {e}")
                raise ConflictBoxLockError(f"Failed to acquire lock for {box_id}: {e}")
    
    def release_conflict_box_lock(self, box_id: str) -> bool:
        """Release a conflict box lock."""
        with self._lock:
            try:
                success = self._simulation_data_service.release_conflict_box_lock(
                    box_id, self._robot_id)
                
                if success:
                    self._locked_boxes.discard(box_id)
                    if box_id in self._conflict_boxes:
                        self._conflict_boxes[box_id].status = ConflictBoxStatus.UNLOCKED
                        self._conflict_boxes[box_id].heartbeat_time = None
                    
                    self._logger.info(f"Released conflict box lock: {box_id}")
                else:
                    self._logger.warning(f"Failed to release conflict box lock: {box_id}")
                
                return success
                
            except Exception as e:
                self._logger.error(f"Error releasing conflict box lock {box_id}: {e}")
                return False
    
    # Private methods
    
    def _start_next_segment(self) -> None:
        """Start the next route segment."""
        if not self._current_route or self._segment_index >= len(self._current_route.segments):
            self._complete_route()
            return
        
        segment = self._current_route.segments[self._segment_index]
        self._current_segment = LaneSegmentProgress(
            segment_index=self._segment_index,
            segment=segment,
            progress_ratio=0.0,
            distance_remaining=self._calculate_segment_distance(segment),
            current_target=segment.start_point
        )
        
        self._status = LaneFollowingStatus.FOLLOWING_LANE
        
        # Start motion execution for this segment
        self._execute_segment_motion(segment)
        
        self._logger.info(f"Started segment {self._segment_index + 1}/{len(self._current_route.segments)}")
    
    def _execute_segment_motion(self, segment: RouteSegment) -> None:
        """Execute motion for a route segment."""
        # Set appropriate speed based on segment type
        if self._is_segment_in_conflict_box(segment):
            self._motion_executor.set_corner_speed(self._config.corner_speed)
        else:
            self._motion_executor.set_movement_speed(self._config.max_speed)
        
        # Create a single-segment route for motion executor
        single_segment_route = Route(
            segments=[segment],
            total_distance=self._calculate_segment_distance(segment),
            estimated_time=0.0
        )
        
        self._motion_executor.follow_route(single_segment_route)
    
    def _update_segment_progress(self) -> None:
        """Update progress through current segment."""
        if not self._current_segment:
            return
        
        robot_pos = self._state_holder.get_position()
        target_pos = self._current_segment.current_target
        
        if target_pos:
            # Convert robot position to 2D tuple
            robot_pos_2d = (robot_pos[0], robot_pos[1])
            distance_to_target = self._calculate_distance(robot_pos_2d, (target_pos.x, target_pos.y))
            
            # Update progress based on distance to target
            segment_distance = self._calculate_segment_distance(self._current_segment.segment)
            if segment_distance > 0:
                progress = 1.0 - (distance_to_target / segment_distance)
                self._current_segment.progress_ratio = max(0.0, min(1.0, progress))
                self._current_segment.distance_remaining = distance_to_target
    
    def _is_segment_complete(self) -> bool:
        """Check if current segment is complete."""
        if not self._current_segment:
            return False
        
        motion_status = self._motion_executor.get_motion_status()
        return (motion_status == MotionStatus.REACHED_TARGET or 
                self._current_segment.progress_ratio >= 0.98)
    
    def _complete_current_segment(self) -> None:
        """Complete current segment and move to next."""
        if self._current_segment:
            self._distance_traveled += self._calculate_segment_distance(self._current_segment.segment)
            self._logger.info(f"Completed segment {self._segment_index + 1}")
        
        self._segment_index += 1
        self._start_next_segment()
    
    def _complete_route(self) -> None:
        """Complete the entire route."""
        self._logger.info("Route completed successfully")
        self._status = LaneFollowingStatus.COMPLETED
        
        # Stop motion
        self._motion_executor.stop_execution()
        
        # Release all locks
        self._release_all_locks(force_release=False)
        
        # Stop heartbeat
        self._stop_heartbeat_thread()
    
    def _update_conflict_box_states(self) -> None:
        """Update conflict box states based on robot position."""
        robot_pos = self._state_holder.get_position()
        robot_pos_2d = (robot_pos[0], robot_pos[1])
        
        for box_id, box_state in self._conflict_boxes.items():
            # Check if robot is inside this conflict box
            was_inside = box_state.robot_inside
            is_inside = self._is_robot_in_conflict_box(robot_pos_2d, box_id)
            
            if is_inside != was_inside:
                box_state.robot_inside = is_inside
                if is_inside:
                    self._logger.info(f"Robot entered conflict box: {box_id}")
                    self._status = LaneFollowingStatus.IN_CONFLICT_BOX
                else:
                    self._logger.info(f"Robot exited conflict box: {box_id}")
                    if self._status == LaneFollowingStatus.IN_CONFLICT_BOX:
                        self._status = LaneFollowingStatus.FOLLOWING_LANE
    
    def _monitor_lane_deviation(self) -> None:
        """Monitor lane deviation for safety."""
        if not self._current_segment:
            return
        
        robot_pos = self._state_holder.get_position()
        robot_pos_2d = (robot_pos[0], robot_pos[1])
        deviation = self._calculate_lane_deviation(robot_pos_2d, self._current_segment.segment)
        
        if deviation > self._config.lane_tolerance:
            # Create Point objects for the interface
            robot_point = Point(robot_pos_2d[0], robot_pos_2d[1])
            # Use first waypoint as lane center for simplicity
            lane_center = self._current_segment.segment.lane.waypoints[0]
            self.report_lane_deviation(deviation, robot_point, lane_center)
    
    def _handle_error(self, error_message: str) -> None:
        """Handle lane following errors."""
        self._logger.error(f"Lane following error: {error_message}")
        self._status = LaneFollowingStatus.ERROR
        
        # Stop motion
        self._motion_executor.stop_execution()
        
        # Release locks for safety
        self._release_all_locks(force_release=True)
    
    def _release_all_locks(self, force_release: bool) -> None:
        """Release all conflict box locks."""
        if force_release:
            # Force release all locks
            for box_id in list(self._locked_boxes):
                self.release_conflict_box_lock(box_id)
        else:
            # Safe release - only release locks for boxes robot is not inside
            unsafe_boxes = self.get_unsafe_boxes()
            for box_id in list(self._locked_boxes):
                if box_id not in unsafe_boxes:
                    self.release_conflict_box_lock(box_id)
    
    def _start_heartbeat_thread(self) -> None:
        """Start the heartbeat thread for conflict box locks."""
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            return
        
        self._heartbeat_running = True
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()
    
    def _stop_heartbeat_thread(self) -> None:
        """Stop the heartbeat thread."""
        self._heartbeat_running = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=1.0)
    
    def _heartbeat_loop(self) -> None:
        """Heartbeat loop for conflict box locks."""
        while self._heartbeat_running:
            try:
                with self._lock:
                    for box_id in list(self._locked_boxes):
                        self._simulation_data_service.heartbeat_conflict_box_lock(box_id, self._robot_id)
                
                time.sleep(self._config.heartbeat_interval)
                
            except Exception as e:
                self._logger.error(f"Error in heartbeat loop: {e}")
                time.sleep(1.0)
    
    # Utility methods
    
    def _calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def _calculate_segment_distance(self, segment: RouteSegment) -> float:
        """Calculate total distance of a route segment."""
        if len(segment.lane.waypoints) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(len(segment.lane.waypoints) - 1):
            p1 = segment.lane.waypoints[i]
            p2 = segment.lane.waypoints[i + 1]
            total_distance += self._calculate_distance((p1.x, p1.y), (p2.x, p2.y))
        
        return total_distance
    
    def _is_segment_in_conflict_box(self, segment: RouteSegment) -> bool:
        """Check if segment passes through any conflict box."""
        if not self._current_route or not self._current_route.conflict_boxes:
            return False
        
        # Simple check - could be enhanced with more sophisticated geometry
        for waypoint in segment.lane.waypoints:
            for box in self._current_route.conflict_boxes:
                if self._is_point_in_conflict_box((waypoint.x, waypoint.y), box):
                    return True
        
        return False
    
    def _is_robot_in_conflict_box(self, robot_pos: Tuple[float, float], box_id: str) -> bool:
        """Check if robot is inside a specific conflict box."""
        if not self._current_route or not self._current_route.conflict_boxes:
            return False
        
        for box in self._current_route.conflict_boxes:
            if box.box_id == box_id:
                return self._is_point_in_conflict_box(robot_pos, box)
        
        return False
    
    def _is_point_in_conflict_box(self, point: Tuple[float, float], box: BoxRec) -> bool:
        """Check if a point is inside a conflict box."""
        half_size = box.size / 2.0
        return (abs(point[0] - box.center.x) <= half_size and
                abs(point[1] - box.center.y) <= half_size)
    
    def _calculate_lane_deviation(self, robot_pos: Tuple[float, float], segment: RouteSegment) -> float:
        """Calculate perpendicular distance from robot to lane center line."""
        if len(segment.lane.waypoints) < 2:
            return 0.0
        
        min_deviation = float('inf')
        
        # Check deviation from each line segment in the lane
        for i in range(len(segment.lane.waypoints) - 1):
            p1 = segment.lane.waypoints[i]
            p2 = segment.lane.waypoints[i + 1]
            
            deviation = self._point_to_line_distance(robot_pos, (p1.x, p1.y), (p2.x, p2.y))
            min_deviation = min(min_deviation, deviation)
        
        return min_deviation if min_deviation != float('inf') else 0.0
    
    def _point_to_line_distance(self, point: Tuple[float, float], 
                               line_start: Tuple[float, float], 
                               line_end: Tuple[float, float]) -> float:
        """Calculate perpendicular distance from point to line segment."""
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # Vector from line start to end
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            # Degenerate line segment
            return self._calculate_distance(point, line_start)
        
        # Parameter t for closest point on line
        t = max(0, min(1, ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)))
        
        # Closest point on line segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return self._calculate_distance(point, (closest_x, closest_y)) 