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
from interfaces.configuration_interface import IConfigurationProvider
from interfaces.conflict_box_queue_interface import (
    IConflictBoxQueue,
    LockAcquisitionResult,
    QueuePosition,
    ConflictBoxQueueError
)


@dataclass
class ConflictBoxState:
    """Internal state for conflict box management."""
    box_id: str
    status: ConflictBoxStatus = ConflictBoxStatus.UNLOCKED
    lock_attempt_time: Optional[float] = None
    heartbeat_time: Optional[float] = None
    robot_inside: bool = False
    priority: int = 0
    queue_position: Optional[int] = None
    estimated_wait_time: Optional[float] = None
    lock_acquisition_result: Optional[LockAcquisitionResult] = None


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
                 simulation_data_service: ISimulationDataService,
                 config_provider: Optional[IConfigurationProvider] = None,
                 conflict_box_queue: Optional[IConflictBoxQueue] = None):
        """
        Initialize the lane follower.
        
        Args:
            robot_id: Unique identifier for this robot
            state_holder: Robot state management service
            motion_executor: Motion control service
            simulation_data_service: Database and navigation data service
            config_provider: Configuration provider for lane following parameters
            conflict_box_queue: Queue-based conflict box management service
        """
        self._robot_id = robot_id
        self._state_holder = state_holder
        self._motion_executor = motion_executor
        self._simulation_data_service = simulation_data_service
        self._config_provider = config_provider
        self._conflict_box_queue = conflict_box_queue
        
        # Thread safety
        self._lock = threading.RLock()
        self._logger = logging.getLogger(f"LaneFollower.{robot_id}")
        
        # Configuration - get from provider or use defaults
        if config_provider:
            nav_config = config_provider.get_navigation_config()
            self._config = LaneFollowingConfig(
                lane_tolerance=nav_config.lane_tolerance,
                max_speed=1.0,  # Will be overridden by robot config
                corner_speed=0.3,  # Will be overridden by robot config
                bay_approach_speed=0.2,  # Will be overridden by robot config
                lock_timeout=nav_config.conflict_box_lock_timeout,
                heartbeat_interval=nav_config.conflict_box_heartbeat_interval,
                lock_retry_attempts=3,
                # position_tolerance removed - Motion Executor is single source of truth
                velocity_smoothing_factor=0.1,
                emergency_stop_distance=0.5
            )
        else:
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
        self._max_deviation_observed = 0.0
        self._last_deviation_value = 0.0

        # Stop-and-turn coordination
        self._pending_rotation_heading: Optional[float] = None
        self._is_rotating: bool = False
        
        # Emergency state
        self._emergency_stopped = False
        
        self._logger.info(f"LaneFollower initialized for robot {robot_id}")
    
    def update_config_from_robot(self, robot_config) -> None:
        """Update configuration with robot-specific parameters."""
        if robot_config:
            self._config.max_speed = robot_config.max_speed
            self._config.corner_speed = robot_config.corner_speed
            self._config.bay_approach_speed = robot_config.bay_approach_speed
            # position_tolerance removed - Motion Executor is single source of truth
    
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
                # Handle pending stop-and-turn rotation before moving to next segment
                if self._is_rotating:
                    motion_status = self._motion_executor.get_motion_status()
                    if motion_status not in [MotionStatus.EXECUTING, MotionStatus.REACHED_TARGET]:
                        # Rotation finished (executor reports IDLE after stopping)
                        self._is_rotating = False
                        self._pending_rotation_heading = None
                        # Start the next segment now that heading is aligned
                        self._start_next_segment()
                        return
                    # Still rotating; skip other updates until rotation completes
                    return
                
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
            # Determine if we should suppress logs due to rate limiting, but never suppress counter updates
            rate_limited = (current_time - self._last_deviation_report_time < 1.0)
            if not rate_limited:
                self._last_deviation_report_time = current_time
            
            if deviation_distance > self._config.lane_tolerance:
                # Always count excessive deviations, even within the rate limit window
                self._consecutive_deviations += 1
                if not rate_limited:
                    self._logger.warning(f"Lane deviation: {deviation_distance:.3f}m")
                
                # Critical deviation handling - using reasonable defaults
                max_deviation = 1.5  # 1.5 meter max deviation (increased from 1.0)
                max_consecutive = 8  # 8 consecutive deviations (increased from 5)
                
                if (deviation_distance > max_deviation or 
                    self._consecutive_deviations > max_consecutive):
                    error_msg = (f"Critical lane deviation: {deviation_distance:.3f}m, "
                               f"consecutive: {self._consecutive_deviations}")
                    # Log critical regardless of rate limit to aid diagnostics
                    self._logger.error(error_msg)
                    raise LaneDeviationError(error_msg)
            else:
                # Reset consecutive deviations when within tolerance
                self._consecutive_deviations = 0
    
    def get_lane_deviation_stats(self) -> Dict[str, Any]:
        """Get lane deviation statistics."""
        with self._lock:
            return {
                "consecutive_deviations": self._consecutive_deviations,
                "last_deviation_time": self._last_deviation_report_time,
                "tolerance": self._config.lane_tolerance,
                "max_deviation": self._max_deviation_observed,
                "last_deviation": self._last_deviation_value
            }
    
    def try_acquire_conflict_box_lock(self, box_id: str, priority: int = 0) -> bool:
        """Try to acquire a conflict box lock using queue system."""
        with self._lock:
            try:
                # Use queue system if available, otherwise fallback to direct acquisition
                if self._conflict_box_queue:
                    return self._try_acquire_with_queue(box_id, priority)
                else:
                    return self._try_acquire_direct(box_id, priority)
                    
            except Exception as e:
                self._logger.error(f"Error acquiring conflict box lock {box_id}: {e}")
                raise ConflictBoxLockError(f"Failed to acquire lock for {box_id}: {e}")
    
    def _try_acquire_with_queue(self, box_id: str, priority: int = 0) -> bool:
        """Try to acquire a conflict box lock using the queue system."""
        try:
            # Request lock through queue system
            timeout_seconds = self._config.lock_timeout
            if self._conflict_box_queue is None:
                raise ConflictBoxLockError("Conflict box queue not available")
            
            result = self._conflict_box_queue.request_lock(
                box_id, self._robot_id, priority, timeout_seconds)
            
            # Update conflict box state
            if box_id in self._conflict_boxes:
                state = self._conflict_boxes[box_id]
                state.lock_acquisition_result = result
                state.queue_position = result.queue_position
                state.estimated_wait_time = result.estimated_wait_time
                state.priority = priority
                state.lock_attempt_time = time.time()
                
                if result.success:
                    state.status = ConflictBoxStatus.LOCKED
                    state.heartbeat_time = time.time()
                    self._locked_boxes.add(box_id)
                    self._logger.info(f"Acquired conflict box lock immediately: {box_id}")
                else:
                    state.status = ConflictBoxStatus.QUEUED
                    self._logger.info(f"Queued for conflict box {box_id} at position {result.queue_position} "
                                    f"(estimated wait: {result.estimated_wait_time:.1f}s)")
            
            return result.success
            
        except ConflictBoxQueueError as e:
            self._logger.error(f"Queue error acquiring lock for {box_id}: {e}")
            raise ConflictBoxLockError(f"Queue error for {box_id}: {e}")
    
    def _try_acquire_direct(self, box_id: str, priority: int = 0) -> bool:
        """Try to acquire a conflict box lock directly (fallback method)."""
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
    
    def release_conflict_box_lock(self, box_id: str) -> bool:
        """Release a conflict box lock."""
        with self._lock:
            try:
                # Use queue system if available, otherwise fallback to direct release
                if self._conflict_box_queue:
                    success = self._conflict_box_queue.release_lock(box_id, self._robot_id)
                else:
                    success = self._simulation_data_service.release_conflict_box_lock(
                        box_id, self._robot_id)
                
                if success:
                    self._locked_boxes.discard(box_id)
                    if box_id in self._conflict_boxes:
                        self._conflict_boxes[box_id].status = ConflictBoxStatus.UNLOCKED
                        self._conflict_boxes[box_id].heartbeat_time = None
                        self._conflict_boxes[box_id].queue_position = None
                        self._conflict_boxes[box_id].estimated_wait_time = None
                        self._conflict_boxes[box_id].lock_acquisition_result = None
                    
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
        # Set current target to the segment end point (drive toward endpoint)
        self._current_segment = LaneSegmentProgress(
            segment_index=self._segment_index,
            segment=segment,
            progress_ratio=0.0,
            distance_remaining=self._calculate_segment_distance(segment),
            current_target=segment.end_point
        )
        
        self._status = LaneFollowingStatus.FOLLOWING_LANE
        
        # Before starting motion, if this is the first segment or a sharp turn, pre-rotate
        if self._should_prerotate_for_segment(self._segment_index):
            target_heading = self._segment_heading(self._current_segment.segment)
            self._motion_executor.rotate_to_heading(target_heading)
            self._is_rotating = True
            self._pending_rotation_heading = target_heading
            self._logger.info(f"Pre-rotating to heading {target_heading:.3f} rad before segment {self._segment_index + 1}")
            return

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
        
        # Determine if we need a stop-and-turn before the next segment
        next_index = self._segment_index + 1
        if self._current_route and next_index < len(self._current_route.segments):
            curr_heading = self._segment_heading(self._current_segment.segment)
            next_heading = self._segment_heading(self._current_route.segments[next_index])
            if self._is_turn(curr_heading, next_heading):
                # Advance to next segment index but perform rotation before starting motion
                self._segment_index = next_index
                self._pending_rotation_heading = next_heading
                self._is_rotating = True
                self._motion_executor.rotate_to_heading(next_heading)
                self._logger.info(f"Stop-and-turn: rotating to heading {next_heading:.3f} rad before segment {self._segment_index + 1}")
                # Do not start next segment until rotation completes
                return
        
        # No rotation needed; advance and start next segment immediately
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
        # Track last and max observed deviations
        self._last_deviation_value = deviation
        if deviation > self._max_deviation_observed:
            self._max_deviation_observed = deviation
        
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
                        # Use queue system if available, otherwise fallback to direct heartbeat
                        if self._conflict_box_queue:
                            self._conflict_box_queue.heartbeat_lock(box_id, self._robot_id)
                        else:
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

    def _segment_heading(self, segment: RouteSegment) -> float:
        """Compute heading (radians) of the primary direction of a segment."""
        waypoints = segment.lane.waypoints
        if len(waypoints) >= 2:
            p1 = waypoints[0]
            p2 = waypoints[1]
            return math.atan2(p2.y - p1.y, p2.x - p1.x)
        return 0.0

    def _is_turn(self, heading_a: float, heading_b: float, threshold_rad: float = 0.35) -> bool:
        """Return True if the change in heading suggests a turn (not a straight)."""
        diff = abs((heading_b - heading_a + math.pi) % (2 * math.pi) - math.pi)
        return diff >= threshold_rad

    def _should_prerotate_for_segment(self, seg_index: int) -> bool:
        """Decide if we should pre-rotate before starting the given segment."""
        # Pre-rotate for the first segment if current orientation is far from segment heading
        if seg_index == 0:
            try:
                x, y, theta = self._state_holder.get_position()
            except Exception:
                theta = 0.0
            desired = self._segment_heading(self._current_route.segments[seg_index])
            diff = abs((desired - theta + math.pi) % (2 * math.pi) - math.pi)
            return diff >= 0.35
        
        # For other segments, we will handle rotation between segments in _complete_current_segment
        return False
    
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