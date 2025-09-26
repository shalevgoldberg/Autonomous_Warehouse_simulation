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
from interfaces.configuration_interface import IBusinessConfigurationProvider
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
                 config_provider: Optional[IBusinessConfigurationProvider] = None,
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
        # Pending lock gating for upcoming segment (box we are waiting to acquire)
        self._pending_lock_box_id: Optional[str] = None

        # Segment-based exit tracking (prevents re-acquisition within same segment)
        self._exited_boxes_current_segment: Set[str] = set()

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
        
        # Conflict box pre-acquisition distance (meters). If we are within this
        # distance to the first waypoint inside a not-yet-held conflict box,
        # we will try to acquire its lock before crossing the boundary.
        self._pre_acquire_distance: float = 0.5

        self._logger.info(f"LaneFollower initialized for robot {robot_id}")
        # Diagnostics (robot_2-focused): rate-limited progress/no-progress logging
        self._last_progress_log_time: float = 0.0
        self._last_progress_distance: float = float('inf')
        self._last_progress_check_time: float = 0.0
    
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
                print(f"LaneFollower {self._robot_id}: Warning: route with 0 segments")  
              #  raise LaneFollowingError("Route must have at least one segment") 
            
            self._logger.info(f"Starting route with {len(route.segments)} segments")
            
            # Reset state
            self._current_route = route
            self._segment_index = 0
            self._total_distance = route.total_distance
            self._distance_traveled = 0.0
            self._route_start_time = time.time()
            self._emergency_stopped = False

            # Professional: ensure a clean slate for per-route transient gates
            # - Prevent stale rotation/lock from previous route
            self._is_rotating = False
            self._pending_rotation_heading = None
            self._pending_lock_box_id = None
            try:
                # Clear any latched REACHED_TARGET from prior motion
                if hasattr(self._motion_executor, 'clear_reached_target_flag'):
                    self._motion_executor.clear_reached_target_flag()
            except Exception:
                pass

            # Clear exited boxes tracking for new route
            self._exited_boxes_current_segment.clear()

            # Diagnostics: log pending lock and new route boxes (no behavior change)
            try:
                route_box_ids = [b.box_id for b in route.conflict_boxes] if getattr(route, 'conflict_boxes', None) else []
                self._logger.info(f"[diag] follow_route start: pending_lock_box_id={self._pending_lock_box_id}, route_boxes={route_box_ids}")
            except Exception:
                pass

            # Initialize conflict box states
            # Diagnostics: if any locks are still marked as held here, warn and show DB owner
            if self._locked_boxes:
                try:
                    self._logger.warning(f"[Diag] Starting new route while still holding locks in-memory: {list(self._locked_boxes)}")
                    robot_pos = self._state_holder.get_position()
                    robot_xy = (robot_pos[0], robot_pos[1])

                    for _box in list(self._locked_boxes):
                        try:
                            _owner = self._simulation_data_service.get_conflict_box_lock_owner(_box)
                            self._logger.warning(f"[Diag] DB owner for {_box}: {_owner}")

                            # Show geometry check vs cache discrepancy
                            try:
                                # Get box geometry from SDS
                                all_boxes = self._simulation_data_service.get_conflict_boxes()
                                box_dict = {b.box_id: b for b in all_boxes}
                                if _box in box_dict:
                                    box_rec = box_dict[_box]
                                    computed_inside = self._is_point_in_conflict_box(robot_xy, box_rec)
                                    cached_inside = self._conflict_boxes.get(_box, ConflictBoxState(_box)).robot_inside
                                    print(f"[diag] follow_route: {_box} computed_inside={computed_inside}, cached_inside={cached_inside}, robot_at=({robot_xy[0]:.2f}, {robot_xy[1]:.2f}), box_center=({box_rec.center.x:.2f}, {box_rec.center.y:.2f}) w={box_rec.width:.2f} h={box_rec.height:.2f}")
                            except Exception as _e:
                                self._logger.warning(f"[diag] follow_route: failed to compute geometry for {_box}: {_e}")
                        except Exception as _e:
                            self._logger.warning(f"[Diag] Failed to query DB owner for {_box}: {_e}")
                except Exception:
                    print(f"[diag] follow_route: failed to query DB owner for {_box}: {_e}")
                    pass
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
            
            # Refresh conflict-box states to avoid stale unsafe decisions at shutdown
            if not force_release:
                try:
                    self._update_conflict_box_states()
                except Exception as _e:
                    self._logger.warning(f"[Diag] Failed to refresh conflict-box states before stop-release: {_e}")

            # Release conflict box locks
            self._release_all_locks(force_release)
            
            # Diagnostics: report if a pending lock gate remains set at stop
            try:
                if self._pending_lock_box_id is not None:
                    self._logger.warning(f"[diag] stop_following: pending_lock_box_id still set: {self._pending_lock_box_id}")
            except Exception:
                pass

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
                # Rate-limited entry diagnostics to confirm updates are running
                try:
                    now_diag = time.time()
                    last = getattr(self, "_last_update_diag_time", 0.0)
                    if now_diag - last > 1.5:
                        setattr(self, "_last_update_diag_time", now_diag)
                        ms = self._motion_executor.get_motion_status()
                        ms_val = ms.value if hasattr(ms, 'value') else ms
                        st_val = self._status.value if hasattr(self._status, 'value') else self._status
                        #print(f"[diag][LF] update enter: status={st_val} robot={self._robot_id} is_rotating={self._is_rotating} pending={self._pending_lock_box_id} motion={ms_val} seg={self._segment_index}")
                except Exception:
                    pass
                # If we're waiting on a lock for the current segment, retry acquisition
                if self._pending_lock_box_id is not None and self._current_segment is not None:
                    box_id = self._pending_lock_box_id

                    # Diagnostics: detect stale pending lock not present in current route
                    try:
                        route_box_ids = [b.box_id for b in self._current_route.conflict_boxes] if (self._current_route and self._current_route.conflict_boxes) else []
                        if route_box_ids and box_id not in route_box_ids:
                            self._logger.warning(f"[diag] update_lane_following: pending_lock_box_id={box_id} not in current route boxes={route_box_ids} (seg={self._segment_index + 1})")
                    except Exception:
                        pass

                    acquired = False
                    try:
                        acquired = self.try_acquire_conflict_box_lock(box_id)
                    except Exception as e:
                        self._logger.warning(f"Lock retry failed for {box_id}: {e}")
                        acquired = False

                    if acquired:
                        self._logger.info(f"Acquired deferred conflict box lock: {box_id}; starting segment")
                        self._pending_lock_box_id = None
                        # Now that lock is held, start motion for the segment
                        self._status = LaneFollowingStatus.FOLLOWING_LANE
                        self._execute_segment_motion(self._current_segment.segment)
                        return
                    else:
                        self._logger.debug(f"[diag] update_lane_following: still waiting for lock {box_id}")
                        return
                # Handle pending stop-and-turn rotation before moving to next segment
                if self._is_rotating:
                    motion_status = self._motion_executor.get_motion_status()
                    # Diagnostics kept commented to avoid noise
                    try:
                        if getattr(self, "_pending_rotation_heading", None) is not None:
                            pose = self._state_holder.get_position()
                            theta = pose[2] if len(pose) >= 3 else 0.0
                            target_h = float(self._pending_rotation_heading)
                            err = target_h - theta
                            while err > math.pi:
                                err -= 2 * math.pi
                            while err < -math.pi:
                                err += 2 * math.pi
                            ms_val = motion_status.value if hasattr(motion_status, 'value') else motion_status
                            print(f"[diag][LF] robot={self._robot_id} rotating: motion={ms_val} heading_err={err:.3f} target={target_h:.3f} theta={theta:.3f}")
                    except Exception:
                        pass
                    # Safe gate: if rotation is not actively executing (REACHED_TARGET or IDLE), treat as complete
                    if motion_status != MotionStatus.EXECUTING:
                        self._is_rotating = False
                        self._pending_rotation_heading = None
                        self._start_next_segment()
                        return
                    else:
                        try:
                            ms_val = motion_status.value if hasattr(motion_status, 'value') else motion_status
                            #print(f"[diag][LF] robot={self._robot_id} rotation branch: deferring due to motion_status={ms_val}")
                        except Exception:
                            pass
                        return

                # Update current segment progress
                self._update_segment_progress()

                # Check for segment completion
                if self._is_segment_complete():
                    self._logger.info(f"Segment {self._segment_index + 1} completed - starting next")
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

    def get_pending_conflict_box(self) -> Optional[str]:
        """Get the conflict box ID that this robot is currently waiting to acquire."""
        with self._lock:
            return self._pending_lock_box_id

    def get_unsafe_boxes(self) -> List[str]:
        """Get list of conflict boxes where robot is currently inside."""
        with self._lock:
            unsafe_boxes: List[str] = []
            robot_pos = self._state_holder.get_position()
            robot_xy = (robot_pos[0], robot_pos[1])

            # Prefer fresh, on-demand geometry checks to avoid stale cache at release time
            if self._current_route and self._current_route.conflict_boxes:
                # Build lookup once for efficiency
                route_boxes_by_id = {b.box_id: b for b in self._current_route.conflict_boxes}

                for box_id, box_state in self._conflict_boxes.items():
                    box_rec = route_boxes_by_id.get(box_id)
                    if box_rec is None:
                        # Fallback to cached state if box not in current route list (should be rare)
                        print(f"[diag] get_unsafe_boxes: box {box_id} not in route context (route has {[b.box_id for b in self._current_route.conflict_boxes]}) — using cached robot_inside={box_state.robot_inside}")
                        if box_state.robot_inside:
                            unsafe_boxes.append(box_id)
                        continue

                    is_inside_now = self._is_point_in_conflict_box(robot_xy, box_rec)
                    # Keep state consistent for subsequent logs/decisions
                    box_state.robot_inside = is_inside_now
                    if is_inside_now:
                        unsafe_boxes.append(box_id)

                return unsafe_boxes

            # Fallback: no route context — use cached states
            #print(f"[diag] get_unsafe_boxes: no route context — using cached states for {len(self._conflict_boxes)} boxes")
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
                    robot_pos = self._state_holder.get_position()
                    self._logger.info(f"Acquired conflict box lock immediately: {box_id} at ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
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
            
            robot_pos = self._state_holder.get_position()
            self._logger.info(f"Acquired conflict box lock: {box_id} at ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
        else:
            self._logger.debug(f"Failed to acquire conflict box lock: {box_id}")
        
        return success
    
    def release_conflict_box_lock(self, box_id: str) -> bool:
        """Release a conflict box lock."""
        with self._lock:
            try:
                #print(f"[diag] release_conflict_box_lock: releasing lock {box_id} (queue={'yes' if self._conflict_box_queue else 'no'})")
                # Defensive: if we still think we're inside due to stale cache, recompute now
                if box_id in self._conflict_boxes:
                    try:
                        if self._current_route and self._current_route.conflict_boxes:
                            route_boxes_by_id = {b.box_id: b for b in self._current_route.conflict_boxes}
                            rec = route_boxes_by_id.get(box_id)
                            if rec is not None:
                                pos = self._state_holder.get_position()
                                inside_now = self._is_point_in_conflict_box((pos[0], pos[1]), rec)
                                self._conflict_boxes[box_id].robot_inside = inside_now
                                #if inside_now:
                                    #print(f"[diag] release_conflict_box_lock: recomputed inside=true at release for {box_id} at {pos}")
                    except Exception as _e:
                        self._logger.warning(f"[Diag] Failed recompute inside at release for {box_id}: {_e}")
                        # If recompute failed, add diagnostic about cache state
                        cached_state = self._conflict_boxes.get(box_id, ConflictBoxState(box_id))
                        self._logger.warning(f"[diag] release_conflict_box_lock: proceeding with release despite recompute failure, cached_inside={cached_state.robot_inside}")
                # Use queue system if available, otherwise fallback to direct release
                if self._conflict_box_queue:
                    success = self._conflict_box_queue.release_lock(box_id, self._robot_id)
                else:
                    success = self._simulation_data_service.release_conflict_box_lock(
                        box_id, self._robot_id)
                
                if success:
                    #print(f"[diag] release_conflict_box_lock: backend release success for {box_id}; updating local state")
                    self._locked_boxes.discard(box_id)
                    if box_id in self._conflict_boxes:
                        self._conflict_boxes[box_id].status = ConflictBoxStatus.UNLOCKED
                        self._conflict_boxes[box_id].heartbeat_time = None
                        self._conflict_boxes[box_id].queue_position = None
                        self._conflict_boxes[box_id].estimated_wait_time = None
                        self._conflict_boxes[box_id].lock_acquisition_result = None
                    
                    robot_pos = self._state_holder.get_position()
                    self._logger.info(f"Released conflict box lock: {box_id} at ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
                else:
                    # Add diagnostic detail to understand why release failed
                    owner_hint = None
                    try:
                        if self._conflict_box_queue:
                            hb = self._conflict_box_queue.heartbeat_lock(box_id, self._robot_id)
                            owner_hint = self._robot_id if hb else 'another_robot_or_none'
                    except Exception:
                        print(f"[diag] set owner hint to unknown due to {_e}")
                        owner_hint = 'unknown'
                    # Try to fetch DB-reported owner for higher-fidelity diagnosis
                    db_owner = None
                    try:
                        db_owner = self._simulation_data_service.get_conflict_box_lock_owner(box_id)
                    except Exception as _e:
                        db_owner = f"error: {_e}"
                    self._logger.warning(f"Failed to release conflict box lock: {box_id} (owner_hint={owner_hint}, db_owner={db_owner})")
                
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

        # Clear exited boxes tracking when starting new segment
        self._exited_boxes_current_segment.clear()

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

        # Diagnostics: one-time log of segment end point walkability (robot_2 only)
        if self._robot_id == "warehouse_robot_2":
            try:
                end_pt = segment.end_point
                walkable = self._simulation_data_service.warehouse_map.is_walkable(end_pt.x, end_pt.y) if hasattr(self._simulation_data_service, 'warehouse_map') else True
                self._logger.info(f"Segment target: ({end_pt.x:.3f}, {end_pt.y:.3f}) walkable={walkable}")
            except Exception:
                pass
        
        # Before starting motion, if this is the first segment or a sharp turn, pre-rotate
        if self._should_prerotate_for_segment(self._segment_index):
            target_heading = self._segment_heading(self._current_segment.segment)
            try:
                x, y, theta = self._state_holder.get_position()
            except Exception:
                theta = 0.0
            try:
                #print(f"[diag][LF] robot={self._robot_id} seg {self._segment_index + 1} prerotate: current_theta={theta:.3f} target_heading={target_heading:.3f}")
                pass
            except Exception:
                pass
            self._motion_executor.rotate_to_heading(target_heading)
            self._is_rotating = True
            self._pending_rotation_heading = target_heading
            return

        # Start motion execution for this segment
        self._execute_segment_motion(segment)
    
    def _execute_segment_motion(self, segment: RouteSegment) -> None:
        """Execute motion for a route segment."""
        # Last-minute locking strategy: do not acquire at segment start.
        # We rely on _preacquire_upcoming_box_if_needed() to attempt lock acquisition
        # when approaching the conflict box boundary, preventing far-away stops.

        # Set appropriate speed based on segment type
        segment_in_box = self._is_segment_in_conflict_box(segment)
        if segment_in_box:
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

        # Diagnostics: log when a zero-segment route is created (robot_2 only)
        if self._robot_id == "warehouse_robot_2":
            if single_segment_route.total_distance == 0.0:
                self._logger.warning(f"Created a zero-segment route for segment {self._segment_index + 1}.")
    
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
                # Pre-acquire upcoming conflict box if we are close to its entry point
                self._preacquire_upcoming_box_if_needed(robot_pos_2d, self._current_segment.segment)

                # Diagnostics: robot_2-focused progress/no-progress logging
                if self._robot_id == "warehouse_robot_2":
                    import time as _t
                    now = _t.time()
                    if now - self._last_progress_log_time > 1.0:
                        self._last_progress_log_time = now
                        #self._logger.info(
                        #    f"Progress seg {self._segment_index + 1}: dist={distance_to_target:.3f}m/seg={segment_distance:.3f}m, progress={self._current_segment.progress_ratio:.2%}")
                    # No-progress detection (log only; no behavior change)
                    if now - self._last_progress_check_time > 3.0:
                        if abs(distance_to_target - self._last_progress_distance) < 0.005:
                            self._logger.warning(
                                f"No progress detected for 3s on seg {self._segment_index + 1}: dist≈{distance_to_target:.3f}m")
                        self._last_progress_distance = distance_to_target
                        self._last_progress_check_time = now
    
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
            segment_distance = self._calculate_segment_distance(self._current_segment.segment)
            self._distance_traveled += segment_distance

        # Determine if we need a stop-and-turn before the next segment
        next_index = self._segment_index + 1
        if self._current_route and next_index < len(self._current_route.segments):
            curr_heading = self._segment_heading(self._current_segment.segment)
            next_heading = self._segment_heading(self._current_route.segments[next_index])

            if self._is_turn(curr_heading, next_heading):
                self._segment_index = next_index
                self._pending_rotation_heading = next_heading
                self._is_rotating = True
                self._motion_executor.rotate_to_heading(next_heading)
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
        
        # Proactively refresh conflict-box inside flags once at route end to avoid stale release decisions
        try:
            self._update_conflict_box_states()
        except Exception as _e:
            self._logger.warning(f"[Diag] Failed to refresh conflict-box states before release: {_e}")

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
                    #print(f"[diag] _update_conflict_box_states: on-enter state: held={list(self._locked_boxes)}, pending={self._pending_lock_box_id}")
                    if box_id not in self._locked_boxes:
                        self._logger.warning(
                            f"Entered conflict box {box_id} without lock! held={list(self._locked_boxes)}")
                    self._status = LaneFollowingStatus.IN_CONFLICT_BOX
                else:
                    self._logger.info(f"Robot exited conflict box: {box_id}")
                    #print(f"[diag] _update_conflict_box_states: on-exit state before release: held={list(self._locked_boxes)}, pending={self._pending_lock_box_id}")
                    if box_id in self._locked_boxes:
                        try:
                            # Reconfirm geometry at the moment of exit in case of very tight margins
                            if self._current_route and self._current_route.conflict_boxes:
                                try:
                                    route_boxes_by_id = {b.box_id: b for b in self._current_route.conflict_boxes}
                                    rec = route_boxes_by_id.get(box_id)
                                    if rec is not None:
                                        inside_now = self._is_point_in_conflict_box(robot_pos_2d, rec)
                                        box_state.robot_inside = inside_now
                                        if inside_now:
                                            print(f"[diag] _update_conflict_box_states: exit-trigger recompute suggests still inside {box_id}; deferring release")
                                            return
                                except Exception as _e:
                                    print(f"[diag] _update_conflict_box_states: exit-trigger recompute failed for {box_id}: {_e}")
                            #print(f"[diag] _update_conflict_box_states: attempting release on exit for {box_id}; held={list(self._locked_boxes)}")
                            self.release_conflict_box_lock(box_id)
                            #print(f"[diag] _update_conflict_box_states: release on exit completed for {box_id}; now held={list(self._locked_boxes)}")
                        except Exception as e:
                            self._logger.warning(f"Failed to release conflict box {box_id} on exit: {e}")

                    self._exited_boxes_current_segment.add(box_id)

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
            # Recompute unsafe with fresh geometry to avoid stale state between control loop ticks
            unsafe_boxes = self.get_unsafe_boxes()
            for box_id in list(self._locked_boxes):
                if box_id not in unsafe_boxes:
                    self.release_conflict_box_lock(box_id)
                else:
                    try:
                        pos = self._state_holder.get_position()
                        self._logger.warning(f"[Diag] Safe release skipped for {box_id}: robot still inside box at {pos}")
                        # Add diagnostic details about why this box is considered unsafe
                        if self._current_route and self._current_route.conflict_boxes:
                            route_boxes_by_id = {b.box_id: b for b in self._current_route.conflict_boxes}
                            box_rec = route_boxes_by_id.get(box_id)
                            if box_rec:
                                print(f"[diag] _release_all_locks: {box_id} in route context, recomputed inside={self._is_point_in_conflict_box((pos[0], pos[1]), box_rec)}")
                            else:
                                print(f"[diag] _release_all_locks: {box_id} not in route context, cached inside={self._conflict_boxes.get(box_id, ConflictBoxState(box_id)).robot_inside}")
                        else:
                            cached_state = self._conflict_boxes.get(box_id, ConflictBoxState(box_id))
                            print(f"[diag] _release_all_locks: no route context, cached inside={cached_state.robot_inside}")
                    except Exception:
                        print(f"[diag] _release_all_locks: failed to compute geometry for {box_id}")
                        pass
    
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
            except Exception as _e:
                print(f"[diag] _should_prerotate_for_segment: failed to get position: {_e}")
                theta = 0.0
            desired = self._segment_heading(self._current_route.segments[seg_index])
            diff = abs((desired - theta + math.pi) % (2 * math.pi) - math.pi)
            # Diagnostics: heading decision (robot_2 only to reduce noise)
            if self._robot_id == "warehouse_robot_2":
                self._logger.info(f"Heading check: theta={theta:.3f} desired={desired:.3f} diff={diff:.3f}")
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

    def _preacquire_upcoming_box_if_needed(self, robot_pos: Tuple[float, float], segment: RouteSegment) -> None:
        """If the current segment will cross into a conflict box we don't yet hold,
        and we are near its first waypoint inside that box, attempt to acquire the lock.
        If acquisition fails, defer motion until acquired.
        """
        if self._pending_lock_box_id is not None:
            return
        if not self._current_route or not self._current_route.conflict_boxes:
            return
        # Find the first waypoint in this segment that lies inside any conflict box we do not hold
        candidate_box_id: Optional[str] = None
        candidate_wp: Optional[Point] = None
        for wp in segment.lane.waypoints:
            for box in self._current_route.conflict_boxes:
                if (self._is_point_in_conflict_box((wp.x, wp.y), box) and
                    box.box_id not in self._locked_boxes):

                    # Prevent re-acquisition of boxes exited in current segment
                    if box.box_id in self._exited_boxes_current_segment:
                        self._logger.debug(f"[PreAcquire] Skipping {box.box_id} - already exited in current segment")
                        continue

                    candidate_box_id = box.box_id
                    candidate_wp = wp
                    break
            if candidate_box_id is not None:
                break
        if candidate_box_id is None or candidate_wp is None:
            return
        # If close enough to the boundary waypoint, try to acquire
        distance_to_wp = self._calculate_distance(robot_pos, (candidate_wp.x, candidate_wp.y))
        if distance_to_wp <= max(self._pre_acquire_distance, self._config.lane_tolerance):
            try:
                acquired = self.try_acquire_conflict_box_lock(candidate_box_id)
            except Exception as e:
                self._logger.warning(f"Pre-acquire attempt failed for {candidate_box_id}: {e}")
                acquired = False
            if not acquired:
                # Defer motion and wait for acquisition in the update loop
                self._pending_lock_box_id = candidate_box_id
                self._status = LaneFollowingStatus.APPROACHING_CONFLICT_BOX
                try:
                    self._motion_executor.stop_execution()
                except Exception as _e:
                    print(f"[diag] _preacquire_upcoming_box_if_needed: failed to stop execution: {_e}")
                    pass

    def _get_first_box_for_segment(self, segment: RouteSegment) -> Optional[str]:
        """Return the first conflict box id that this segment intersects (if any).

        Heuristic: iterate waypoints order and return the first box whose area contains
        the waypoint. This matches our grid lanes where waypoints inside a box appear
        contiguous. This avoids acquiring locks far ahead.
        """
        if not self._current_route or not self._current_route.conflict_boxes:
            return None
        route_boxes = self._current_route.conflict_boxes
        for waypoint in segment.lane.waypoints:
            wx, wy = waypoint.x, waypoint.y
            for box in route_boxes:
                if self._is_point_in_conflict_box((wx, wy), box):
                    return box.box_id
        return None
    
    def _is_robot_in_conflict_box(self, robot_pos: Tuple[float, float], box_id: str) -> bool:
        """Check if robot is inside a specific conflict box."""
        if not self._current_route or not self._current_route.conflict_boxes:
            return False
        
        for box in self._current_route.conflict_boxes:
            if box.box_id == box_id:
                return self._is_point_in_conflict_box(robot_pos, box)
        
        return False
    
    def _is_point_in_conflict_box(self, point: Tuple[float, float], box: BoxRec) -> bool:
        """Check if a point is inside a rectangular conflict box (axis-aligned)."""
        # Small margin to avoid sticky boundary cases
        epsilon = 0.02
        half_w = max(0.0, (box.width * 0.5) - epsilon)
        half_h = max(0.0, (box.height * 0.5) - epsilon)
        dx = point[0] - box.center.x
        dy = point[1] - box.center.y
        return (abs(dx) <= half_w and abs(dy) <= half_h)
    
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