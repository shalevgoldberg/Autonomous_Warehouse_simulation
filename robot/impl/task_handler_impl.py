"""
TaskHandler Implementation - Thread-safe task lifecycle management and component coordination.

This implementation manages robot tasks by coordinating StateHolder, PathPlanner, MotionExecutor,
and SimulationDataService to execute pick-and-deliver operations, charging tasks, and movement commands.

Key integrations:
- Shelf locking/unlocking during task execution
- Inventory updates when picking items
- KPI event logging throughout task lifecycle
- Real-time position data from SimulationDataService
"""
import threading
import time
import math
import logging
import uuid
from typing import Optional, List, Tuple
from datetime import datetime
from dataclasses import dataclass, replace
from enum import Enum
from threading import RLock
import weakref

from interfaces.task_handler_interface import (
    ITaskHandler, Task, TaskType, TaskStatus, OperationalStatus,
    TaskHandlerStatus, TaskHandlingError
)
from interfaces.charging_station_manager_interface import ChargingStationAssignment
from interfaces.state_holder_interface import IStateHolder
from interfaces.path_planner_interface import IPathPlanner, Path, Cell, PathPlanningError
from interfaces.motion_executor_interface import IMotionExecutor, MotionStatus, MotionExecutionError
from interfaces.coordinate_system_interface import ICoordinateSystem
from interfaces.simulation_data_service_interface import ISimulationDataService, SimulationDataServiceError
from interfaces.lane_follower_interface import ILaneFollower, LaneFollowingStatus, LaneFollowingError
from interfaces.navigation_types import Route, TaskType as NavTaskType
from interfaces.configuration_interface import IBusinessConfigurationProvider
from interfaces.appearance_service_interface import IAppearanceService
from interfaces.charging_station_manager_interface import (
    IChargingStationManager,
    ChargingStationRequest
)
from interfaces.jobs_processor_interface import Priority
from interfaces.battery_manager_interface import IBatteryManager


class ReadWriteLock:
    """
    A reader-writer lock implementation that allows multiple concurrent readers
    but exclusive writers. This improves performance by allowing all getter methods
    to run concurrently while still protecting writes.
    """
    
    def __init__(self):
        self._read_ready = threading.Condition(threading.RLock())
        self._readers = 0
        
    def acquire_read(self):
        """Acquire a read lock (shared)."""
        self._read_ready.acquire()
        try:
            self._readers += 1
        finally:
            self._read_ready.release()
            
    def release_read(self):
        """Release a read lock."""
        self._read_ready.acquire()
        try:
            self._readers -= 1
            if self._readers == 0:
                self._read_ready.notify_all()
        finally:
            self._read_ready.release()
            
    def acquire_write(self):
        """Acquire a write lock (exclusive)."""
        self._read_ready.acquire()
        while self._readers > 0:
            self._read_ready.wait()
            
    def release_write(self):
        """Release a write lock."""
        self._read_ready.release()


class ReadLock:
    """Context manager for read locks."""
    
    def __init__(self, lock: ReadWriteLock):
        self.lock = lock
        
    def __enter__(self):
        self.lock.acquire_read()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release_read()


class WriteLock:
    """Context manager for write locks."""
    
    def __init__(self, lock: ReadWriteLock):
        self.lock = lock
        
    def __enter__(self):
        self.lock.acquire_write()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release_write()


class TaskPhase(Enum):
    """Internal task execution phases."""
    PLANNING = "planning"
    NAVIGATING_TO_SHELF = "navigating_to_shelf"
    PICKING_ITEM = "picking_item"
    NAVIGATING_TO_DROPOFF = "navigating_to_dropoff"
    DROPPING_ITEM = "dropping_item"
    NAVIGATING_TO_CHARGING = "navigating_to_charging"
    CHARGING_BATTERY = "charging_battery"
    COMPLETED = "completed"
    NAVIGATING_TO_IDLE = "navigating_to_idle"
    WANDERING = "wandering"


# Explicit phase ordering for progress calculation (safer than enum string comparison)
TASK_PHASE_ORDER = [
    TaskPhase.PLANNING,
    TaskPhase.NAVIGATING_TO_SHELF,
    TaskPhase.PICKING_ITEM,
    TaskPhase.NAVIGATING_TO_DROPOFF,
    TaskPhase.DROPPING_ITEM,
    TaskPhase.NAVIGATING_TO_CHARGING,
    TaskPhase.CHARGING_BATTERY,
    TaskPhase.NAVIGATING_TO_IDLE,
    TaskPhase.WANDERING,
    TaskPhase.COMPLETED
]


class TaskHandlerImpl(ITaskHandler):
    """
    Thread-safe implementation of TaskHandler with comprehensive task lifecycle management.
    
    Threading Model:
    - start_task(): CONTROL THREAD ONLY (called by Orchestrator)
    - update_task_execution(): CONTROL THREAD ONLY (called at 10Hz)
    - get_*() methods: ANY THREAD (thread-safe reads)
    - handle_stall_event(): ANY THREAD (emergency/safety use)
    - emergency_stop(): ANY THREAD (emergency use)
    
    Task Coordination:
    Uses dependency injection to coordinate all robot components while maintaining
    clear separation of concerns and SOLID principles.
    
    SimulationDataService Integration:
    - Shelf locking/unlocking during task execution
    - Inventory updates when picking items
    - KPI event logging throughout task lifecycle
    - Real-time position data from warehouse map
    """
    
    def __init__(self,
                 state_holder: IStateHolder,
                 path_planner: IPathPlanner,
                 lane_follower: ILaneFollower,
                 motion_executor: IMotionExecutor,
                 coordinate_system: ICoordinateSystem,
                 simulation_data_service: ISimulationDataService,
                 robot_id: str = "robot_1",
                 config_provider: Optional[IBusinessConfigurationProvider] = None,
                 appearance_service: Optional[IAppearanceService] = None,
                 charging_station_manager: Optional[IChargingStationManager] = None,
                 battery_manager: Optional[IBatteryManager] = None):
        """
        Initialize TaskHandler with required dependencies.

        Args:
            state_holder: Robot state management
            path_planner: Route planning functionality
            lane_follower: Lane-based navigation execution
            motion_executor: Motion execution and control
            coordinate_system: Coordinate conversion utilities
            simulation_data_service: Warehouse data and shelf management
            robot_id: Unique identifier for this robot
            config_provider: Configuration provider for task parameters
            appearance_service: Visual appearance service for carrying indication
            charging_station_manager: Charging station allocation and management
            battery_manager: Battery management for emergency stop and monitoring
        """
        self.state_holder = state_holder
        self.path_planner = path_planner
        self.lane_follower = lane_follower
        self.motion_executor = motion_executor
        self.coordinate_system = coordinate_system
        self.simulation_data_service = simulation_data_service
        self.robot_id = robot_id
        self.config_provider = config_provider
        self.appearance_service = appearance_service
        self.charging_station_manager = charging_station_manager
        self.battery_manager = battery_manager
        
        # Setup logging with robot ID prefix
        self.logger = logging.getLogger(f"TaskHandler.{robot_id}")
        
        # Thread safety - Reader-writer lock for better concurrency
        self._status_lock = ReadWriteLock()
        
        # Task state
        self._current_task: Optional[Task] = None
        self._operational_status = OperationalStatus.IDLE
        self._status_change_time: Optional[float] = None
        # Phase-specific initialization flags
        self._picking_initialized: bool = False
        self._task_phase = TaskPhase.COMPLETED
        self._current_path: Optional[Path] = None
        self._task_start_time: Optional[float] = None

        # Shelf orientation state tracking
        self._shelf_orientation_pending: bool = False
        self._orientation_start_time: Optional[float] = None
        
        # Task history for monitoring
        self._task_history: List[Task] = []
        self._max_history_size = 50
        
        # Stall handling
        self._stall_reason: Optional[str] = None
        self._stall_start_time: Optional[float] = None
        self._stall_retry_count: int = 0
        self._stall_retry_start_time: Optional[float] = None
        
        # Task execution parameters - get from config provider or use defaults
        if config_provider:
            robot_config = config_provider.get_robot_config(robot_id)
            task_config = config_provider.get_task_config()
            self._stall_recovery_timeout = robot_config.stall_recovery_timeout
            # position_tolerance removed - Motion Executor is single source of truth
            self._picking_duration = robot_config.picking_duration
            self._dropping_duration = robot_config.dropping_duration
            self._task_timeout = task_config.task_timeout
            self._retry_attempts = task_config.retry_attempts
            self._retry_delay = task_config.retry_delay
        else:
            # Default values
            self._stall_recovery_timeout = 10.0  # seconds
            # position_tolerance removed - Motion Executor is single source of truth
            self._picking_duration = 3.0   # seconds to simulate picking
            self._dropping_duration = 2.0  # seconds to simulate dropping
            self._task_timeout = 300.0  # seconds
            self._retry_attempts = 3
            self._retry_delay = 5.0  # seconds
        
        # Phase timing for simulation
        self._phase_start_time: Optional[float] = None
        
        # Asynchronous route planning state
        self._planning_thread: Optional[threading.Thread] = None
        self._planned_route: Optional[Route] = None
        self._planning_error: Optional[str] = None
        self._planning_target: Optional[Tuple[float, float]] = None
        
        # Shelf locking state (for cleanup on errors)
        self._locked_shelf_id: Optional[str] = None # legacy - redundant with removing self-lock mechanism
        # Bay locking state
        # _idle_bay_lock_id: holds an idle bay lock while robot physically occupies the bay
        # _locked_bay_id: reserved for resource locks (e.g., charging station id)
        # _reached_idle_bay: tracks if robot has physically reached the idle bay (prevents premature lock release)
        self._idle_bay_lock_id: Optional[str] = None
        self._locked_bay_id: Optional[str] = None
        self._reached_idle_bay: bool = False
        self._bay_lock_last_heartbeat: float = 0.0
        self._bay_lock_heartbeat_interval: float = 5.0
        # IDLE_PARK transition flag (prevents bay lock release during transitions)
        self._transitioning_to_idle: bool = False

        # Automatic charging configuration
        if config_provider:
            task_config = config_provider.get_task_config()
            self._auto_charging_enabled = task_config.auto_charging_enabled
            self._charging_trigger_threshold = task_config.charging_trigger_threshold
            self._battery_check_interval = task_config.battery_check_interval
            self._safe_interrupt_task_types = [TaskType(task_type) for task_type in task_config.safe_interrupt_tasks]
            self._prevent_duplicate_charging = task_config.prevent_duplicate_charging
            # Availability during charging
            try:
                self._charging_availability_threshold = float(task_config.charging_availability_threshold)
            except Exception:
                self.logger.warning("Invalid charging availability threshold, using default")
                self._charging_availability_threshold = 0.60
            # Optional idle/wander configuration
            try:
                idle_wander_enabled = config_provider.get_value("idle.wander.enabled").value
                if isinstance(idle_wander_enabled, bool):
                    self._idle_wander_enabled = idle_wander_enabled
                else:
                    self._idle_wander_enabled = True
            except Exception:
                self._idle_wander_enabled = True
            try:
                retry_val = float(config_provider.get_value("idle.wander.retry_interval_seconds").value)
                self._idle_retry_interval = max(0.5, min(retry_val, 30.0))
            except Exception:
                pass
            try:
                min_dist_val = float(config_provider.get_value("idle.wander.min_target_distance_m").value)
                self._wander_min_target_distance = max(0.0, min_dist_val)
            except Exception:
                pass
        else:
            # Default values for automatic charging
            self._auto_charging_enabled = True
            self._charging_trigger_threshold = 0.25  # 25%
            self._battery_check_interval = 1.0  # Check every 1 second
            self._safe_interrupt_task_types = [TaskType.IDLE_PARK, TaskType.MOVE_TO_POSITION]
            self._prevent_duplicate_charging = True
            # Idle/wander defaults
            self._idle_wander_enabled = True
            self._wander_min_target_distance = 2.0
            # Availability during charging default
            self._charging_availability_threshold = 0.60

        # Automatic charging state
        self._charging_task_pending = False
        self._last_battery_check = 0.0
        # Waiting-for-charging queue state
        self._waiting_for_charging: bool = False
        self._waiting_for_charging_since: float = 0.0
        self._last_charging_recheck: float = 0.0
        # Re-attempt interval while waiting (seconds)
        self._charging_recheck_interval: float = max(1.0, self._battery_check_interval)

        # Wander state (for idle fallback)
        self._is_wandering: bool = False
        self._wandering_target: Optional[Tuple[float, float]] = None
        self._wander_candidates: Optional[List[Tuple[float, float]]] = None
        self._last_idle_retry_time: float = 0.0
        # Default retry interval; may be overridden by config
        self._idle_retry_interval: float = 2.0
        # Minimum required distance for selecting a new wander target
        self._wander_min_target_distance: float = 2.0

        # Single-slot pending task scheduling (consumed on control thread)
        self._pending_start_task: Optional[Task] = None

    def _get_battery_level_info(self) -> str:
        """
        Get formatted battery level information for logging.

        Returns:
            str: Formatted battery level string (e.g., "battery: 85.2%")
        """
        try:
            battery_level = self.state_holder.get_battery_level()
            return f"battery: {battery_level:.1%}"
        except Exception:
            return "battery: unknown"

    def _check_automatic_charging(self) -> None:
        """
        Check battery level and trigger automatic charging if needed.
        Called from update_task_execution at regular intervals.
        """
        # Skip if auto-charging is disabled
        if not self._auto_charging_enabled:
            return

        # Rate limit battery checks
        current_time = time.time()
        if current_time - self._last_battery_check < self._battery_check_interval:
            return

        self._last_battery_check = current_time

        # Get current battery level
        try:
            battery_level = self.state_holder.get_battery_level()
        except Exception as e:
            self.logger.warning(f"Failed to get battery level for auto-charging: {e}")
            return

        # If we're already queued/waiting for charging, periodically re-attempt allocation
        if self._waiting_for_charging:
            if current_time - self._last_charging_recheck >= self._charging_recheck_interval:
                self._last_charging_recheck = current_time
                try:
                    if self.charging_station_manager:
                        robot_state = self.state_holder.get_robot_state()
                        robot_position = (robot_state.position[0], robot_state.position[1])
                        request = ChargingStationRequest(
                            robot_id=self.robot_id,
                            robot_position=robot_position,
                            timestamp=time.time()
                        )
                        assignment = self.charging_station_manager.request_charging_station(request)
                        if assignment:
                            # Start charging task using the preallocated assignment
                            self._start_charging_task_with_preallocated(assignment, battery_level)
                            return
                except Exception as e:
                    # Non-fatal: continue waiting
                    self.logger.debug(f"Waiting for charging - recheck failed: {e}")
            # While waiting, do not trigger duplicate auto-charging tasks
            return

        # Check if charging should be triggered
        if self._should_trigger_automatic_charging(battery_level):
            self._trigger_automatic_charging_task(battery_level)

    def _should_trigger_automatic_charging(self, battery_level: float) -> bool:
        """
        Determine if automatic charging should be triggered.

        Args:
            battery_level: Current battery level (0.0 to 1.0)

        Returns:
            bool: True if charging should be triggered
        """
        # Battery must be below threshold
        if battery_level > self._charging_trigger_threshold:
            return False

        # Prevent duplicate charging tasks if enabled
        if self._prevent_duplicate_charging and self._charging_task_pending:
            return False

        # Robot must be in safe state for charging
        return self._is_robot_available_for_charging()

    def _is_robot_available_for_charging(self) -> bool:
        """
        Check if robot is in a safe state to be interrupted for charging.

        Returns:
            bool: True if robot can be safely interrupted
        """
        # No current task = robot is idle (safe to charge)
        if not self._current_task:
            return True

        # Can interrupt safe task types
        return self._current_task.task_type in self._safe_interrupt_task_types

    def _trigger_automatic_charging_task(self, current_battery: float) -> None:
        """
        Create and assign an automatic charging task.

        Args:
            current_battery: Current battery level when triggering
        """
        try:
            # If robot has a current task, complete it first if it's safe to interrupt
            if (self._current_task and
                self._current_task.task_type in self._safe_interrupt_task_types):
                self.logger.info(f"Completing safe task {self._current_task.task_id} for charging")
                self._complete_task()

            # Try to allocate a charging station immediately
            assignment: Optional[ChargingStationAssignment] = None
            if self.charging_station_manager:
                try:
                    robot_state = self.state_holder.get_robot_state()
                    robot_position = (robot_state.position[0], robot_state.position[1])
                    request = ChargingStationRequest(
                        robot_id=self.robot_id,
                        robot_position=robot_position,
                        timestamp=time.time()
                    )
                    assignment = self.charging_station_manager.request_charging_station(request)
                except Exception as e:
                    self.logger.debug(f"Pre-allocation attempt failed: {e}")

            if assignment is None:
                # No station available right now -> go to idle and wait (do not spam tasks)
                if not self._waiting_for_charging:
                    self._handle_waiting_for_charging()
                return

            # Start with preallocated assignment (sets locked bay id and metadata)
            self._start_charging_task_with_preallocated(assignment, current_battery)

        except Exception as e:
            self.logger.error(f"Error creating automatic charging task: {e}")

    def _check_emergency_stop(self) -> bool:
        """
        Check if emergency stop should be triggered due to battery depletion.

        Returns:
            bool: True if emergency stop should be triggered
        """
        try:
            # Check if emergency stop is already active
            if self._operational_status == OperationalStatus.EMERGENCY_STOP:
                return False  # Don't trigger again if already active

            # Check battery manager for emergency stop condition
            if self.battery_manager and self.battery_manager.is_emergency_stop_active():
                return True

            # Fallback: check battery level directly if no manager
            battery_level = self.state_holder.get_battery_level()
            if battery_level <= 0.0:  # Emergency threshold
                self.logger.critical(
                    f"🚨 EMERGENCY STOP TRIGGERED: Battery depleted to {battery_level:.1%} "
                    f"(direct check - no battery manager)"
                )
                return True

        except Exception as e:
            self.logger.error(f"Error checking emergency stop condition: {e}")

        return False

    def _trigger_emergency_stop(self) -> None:
        """
        Trigger emergency stop due to battery depletion.
        """
        try:
            # Set operational status to emergency stop
            self._operational_status = OperationalStatus.EMERGENCY_STOP

            # Stop all motion immediately
            if hasattr(self, 'motion_executor'):
                self.motion_executor.emergency_stop()

            if hasattr(self, 'lane_follower'):
                self.lane_follower.emergency_stop()

            # Cancel current task if any
            if self._current_task:
                self.logger.warning(f"Emergency stop: Cancelling task {self._current_task.task_id}")
                self._current_task.status = TaskStatus.FAILED
                # Don't call _complete_task as it might reset emergency status

            # Release charging station if held during emergency stop
            if self._locked_bay_id and self.charging_station_manager:
                try:
                    released = self.charging_station_manager.release_charging_station(
                        self._locked_bay_id, self.robot_id
                    )
                    if released:
                        self.logger.info(f"Emergency stop: Released charging station {self._locked_bay_id}")
                    else:
                        self.logger.warning(f"Emergency stop: Failed to release charging station {self._locked_bay_id}")
                except Exception as e:
                    self.logger.error(f"Emergency stop: Error releasing charging station {self._locked_bay_id}: {e}")
                finally:
                    self._locked_bay_id = None
            # Release regular bay lock if held (for idle bays)
            if self._idle_bay_lock_id and self.simulation_data_service:
                try:
                    released = self.simulation_data_service.release_bay_lock(
                        self._idle_bay_lock_id, self.robot_id
                    )
                    if released:
                        self.logger.info(f"Emergency stop: Released bay lock {self._idle_bay_lock_id}")
                    else:
                        self.logger.warning(f"Emergency stop: Failed to release bay lock {self._idle_bay_lock_id}")
                except Exception as e:
                    self.logger.error(f"Emergency stop: Error releasing bay lock {self._idle_bay_lock_id}: {e}")
                finally:
                    self._idle_bay_lock_id = None

            # Clear current task
            self._current_task = None

            self.logger.critical(f"🚨 EMERGENCY STOP ACTIVATED: Robot stopped due to battery depletion ({self._get_battery_level_info()})")

        except Exception as e:
            self.logger.error(f"Error during emergency stop: {e}")

    def enable_auto_charging(self, enabled: bool) -> None:
        """
        Enable or disable automatic charging.

        Args:
            enabled: True to enable, False to disable
        """
        with WriteLock(self._status_lock):
            self._auto_charging_enabled = enabled
            self.logger.info(f"Automatic charging {'enabled' if enabled else 'disabled'}")

    def set_charging_threshold(self, threshold: float) -> None:
        """
        Set the battery threshold for automatic charging.

        Args:
            threshold: Battery level threshold (0.0 to 1.0)
        """
        with WriteLock(self._status_lock):
            if 0.0 <= threshold <= 1.0:
                self._charging_trigger_threshold = threshold
                self.logger.info(f"Charging threshold set to {threshold:.1%}")
            else:
                self.logger.warning(f"Invalid charging threshold: {threshold}")
    
    def set_charging_availability_threshold(self, threshold: float) -> None:
        """
        Set the battery threshold for bidding availability while charging.

        Args:
            threshold: Battery level threshold (0.0 to 1.0)
        """
        with WriteLock(self._status_lock):
            if 0.0 <= threshold <= 1.0:
                self._charging_availability_threshold = threshold
                self.logger.info(f"Charging availability threshold set to {threshold:.1%}")
            else:
                self.logger.warning(f"Invalid charging availability threshold: {threshold}")
    
    def update_config_from_robot(self, robot_config) -> None:
        """Update task parameters with robot-specific configuration."""
        if robot_config:
            self._stall_recovery_timeout = robot_config.stall_recovery_timeout
            # position_tolerance removed - Motion Executor is single source of truth
            self._picking_duration = robot_config.picking_duration
            self._dropping_duration = robot_config.dropping_duration
    
    def get_task_status(self) -> TaskHandlerStatus:
        """
        Get current task execution status.
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            TaskHandlerStatus: Current status including progress and operational state
        """
        with ReadLock(self._status_lock):
            progress = self._calculate_task_progress()
            
            return TaskHandlerStatus(
                has_active_task=self._current_task is not None,
                task_id=self._current_task.task_id if self._current_task else None,
                operational_status=self._operational_status,
                progress=progress,
                stall_reason=self._stall_reason,
                locked_bay_id=self._locked_bay_id
            )
    
    def update_task_execution(self) -> None:
        """
        Update task execution (called at 10Hz from control thread).
        **CONTROL THREAD ONLY**: Must be called from control thread only.

        Handles task lifecycle, waypoint progression, and motion coordination.
        Includes automatic charging monitoring and emergency stop checking.
        """
        with WriteLock(self._status_lock):
            # Consume single-slot scheduled task if idle
            if self._pending_start_task is not None:
                if self.is_idle():
                    try:
                        print(f"[TaskHandler] Consuming scheduled task {self._pending_start_task.task_id}")
                    except Exception:
                        pass
                    # Use start_task as we're on control thread
                    if self.start_task(self._pending_start_task):
                        self._pending_start_task = None
                    else:
                        # Keep it for next cycle; log fallback
                        try:
                            print(f"[TaskHandler] Scheduled task {self._pending_start_task.task_id} not started (robot busy); will retry")
                        except Exception:
                            pass

            # Check for battery emergency stop first (highest priority)
            if self._check_emergency_stop():
                self._trigger_emergency_stop()
                return

            # Check for automatic charging (regardless of current task state)
            self._check_automatic_charging()

            if not self._current_task:
                return

            # Handle stall recovery
            if self._operational_status == OperationalStatus.STALLED:
                self._handle_stall_recovery()
                return

            # Handle emergency stop
            if self._operational_status == OperationalStatus.EMERGENCY_STOP:
                return

            # Update lane following execution (with low-frequency diagnostics)
            try:
                now_diag = time.time()
                last = getattr(self, "_last_lf_call_diag", 0.0)
                if now_diag - last > 2.0:
                    setattr(self, "_last_lf_call_diag", now_diag)
                    lf_status = self.lane_follower.get_lane_following_status()
                    lf_val = lf_status.value if hasattr(lf_status, 'value') else lf_status
                    #print(f"[diag][TaskHandler] robot={self.robot_id} calling update_lane_following | phase={self._task_phase.value if self._task_phase else 'n/a'} status={lf_val}")
            except Exception:
                pass
            self.lane_follower.update_lane_following()

            # Update task execution based on current phase
            self._update_current_phase()

    def schedule_start_task(self, task: Task) -> None:
        """
        Schedule a task to start on the next control-thread cycle when idle.
        Safe to call from any thread. Only a single pending task is kept.
        """
        with WriteLock(self._status_lock):
            # Replace any existing pending task; log replacement to avoid silent fallback
            if self._pending_start_task is not None:
                try:
                    print(f"[TaskHandler] Replacing pending task {self._pending_start_task.task_id} with {task.task_id}")
                except Exception:
                    pass
            else:
                try:
                    print(f"[TaskHandler] Scheduled task {task.task_id} for startup")
                except Exception:
                    pass
            self._pending_start_task = task
    
    def emergency_stop(self) -> None:
        """
        Emergency stop current task.
        **Thread-safe**: Can be called from any thread.
        
        Immediately halts all motion and marks task as failed.
        """
        with WriteLock(self._status_lock):
            # Stop motion and lane following immediately
            self.motion_executor.emergency_stop()
            self.lane_follower.emergency_stop()  # Force release all locks in emergency
            
            # Mark task as failed
            if self._current_task:
                self._current_task.status = TaskStatus.FAILED
                self._current_task.completed_at = datetime.now()
                
                self.logger.warning(f"Emergency stop during task {self._current_task.task_id}")
                self._add_to_history(self._current_task)
            
            # On emergency stop: release reservation exactly once (idempotent)
            try:
                task = self._current_task
                if task and task.inventory_reserved:
                    if task.metadata is None:
                        task.metadata = {}
                    released_flag = task.metadata.get('reservation_released', False)
                    consumed_flag = task.metadata.get('reservation_consumed', False)
                    if not consumed_flag and not released_flag:
                        ok = self.simulation_data_service.release_inventory(
                            task.shelf_id, task.item_id, task.quantity_to_pick
                        )
                        if ok:
                            task.metadata['reservation_released'] = True
                            self.logger.info(
                                f"Task {task.task_id}: Emergency stop - released reservation {task.quantity_to_pick} of {task.item_id} on {task.shelf_id}"
                            )
                        else:
                            self.logger.warning(
                                f"Task {task.task_id}: Emergency stop - release returned False for {task.item_id} on {task.shelf_id} - may be orphaned"
                            )
            except SimulationDataServiceError as e:
                self.logger.error(f"Emergency stop: release inventory failed: {e}")

            # Ensure charging station lock is released if held
            if self._locked_bay_id and self.charging_station_manager:
                try:
                    released = self.charging_station_manager.release_charging_station(
                        self._locked_bay_id, self.robot_id
                    )
                    if released:
                        self.logger.info(f"Emergency stop: Released charging station {self._locked_bay_id}")
                    else:
                        self.logger.warning(f"Emergency stop: Failed to release charging station {self._locked_bay_id}")
                except Exception as e:
                    self.logger.error(f"Emergency stop: Error releasing charging station {self._locked_bay_id}: {e}")
                finally:
                    self._locked_bay_id = None

            # Reset to idle state
            self._reset_task_state(preserve_bay_lock=False)
    
    def start_task(self, task: Task) -> bool:
        """
        Start executing a new task.
        **CONTROL THREAD ONLY**: Called by Orchestrator when task is assigned.
        
        Args:
            task: Task to execute
            
        Returns:
            bool: True if task accepted, False if robot is busy
        """
        with WriteLock(self._status_lock):
            # Allow MOVE_TO_CHARGING task even while waiting for charging
            allow_while_waiting_for_charging = (
                getattr(self, "_waiting_for_charging", False)
                and task.task_type == TaskType.MOVE_TO_CHARGING
            )

            if not allow_while_waiting_for_charging and not self.is_idle():
                # Low-noise diagnostics (debug only) to clarify why task is rejected
                try:
                    if self._logger.isEnabledFor(logging.DEBUG):
                        self.logger.debug(
                            f"Rejecting task {task.task_id} ({task.task_type.value}); "
                            f"is_idle={self._operational_status == OperationalStatus.IDLE and self._current_task is None}, "
                            f"waiting_for_charging={getattr(self, '_waiting_for_charging', False)}"
                        )
                except Exception:
                    pass
                self.logger.info(f"Robot busy, cannot accept task {task.task_id}")
                return False

            # If currently performing IDLE_PARK (including WANDERING), stop it before starting new task
            if self._current_task and self._current_task.task_type == TaskType.IDLE_PARK and task.task_type != TaskType.IDLE_PARK:
                try:
                    print(f"[Wander][{self.robot_id}] Preempting IDLE_PARK for new task {task.task_id}; stopping wander if active")
                except Exception:
                    pass
                # Ensure lane following stops and bay lock (if any) is released
                self._reset_task_state(preserve_bay_lock=False)
            
            # Reset motion executor from emergency stop if needed
            motion_status = self.motion_executor.get_motion_status()
            if motion_status == MotionStatus.ERROR:
                print(f"[TaskHandler] MotionStatus error - Stopping motion executor from emergency stop")
                self.motion_executor.stop_execution()  # Reset from emergency stop
            
            # Validate task
            if not self._validate_task(task):
                self.logger.warning(f"Invalid task {task.task_id}")
                return False
            
            # Set charging task flag if this is a charging task
            if task.task_type == TaskType.MOVE_TO_CHARGING:
                self._charging_task_pending = True

            # Reset IDLE_PARK transition flag when starting a new task
            # (unless this IS the IDLE_PARK task we're transitioning to)
            if task.task_type != TaskType.IDLE_PARK:
                self._transitioning_to_idle = False

            # Start task execution
            self._current_task = task
            self._current_task.status = TaskStatus.IN_PROGRESS
            self._current_task.assigned_at = datetime.now()
            self._task_start_time = time.time()
            self._phase_start_time = time.time()

            # Reset shelf orientation state for new task
            self._shelf_orientation_pending = False
            self._orientation_start_time = None
            
            # Set initial phase and status
            if task.task_type == TaskType.PICK_AND_DELIVER:
                self._task_phase = TaskPhase.PLANNING
                self._update_operational_status(OperationalStatus.MOVING_TO_SHELF)
            elif task.task_type == TaskType.MOVE_TO_CHARGING:
                self._task_phase = TaskPhase.PLANNING
                self._update_operational_status(OperationalStatus.MOVING_TO_CHARGING)
            elif task.task_type == TaskType.MOVE_TO_POSITION:
                self._task_phase = TaskPhase.PLANNING
                self._update_operational_status(OperationalStatus.MOVING_TO_SHELF)  # Generic movement
            elif task.task_type == TaskType.IDLE_PARK:
                # Explicitly mark as moving to idle so status reflects intent during planning
                self._task_phase = TaskPhase.PLANNING
                self._update_operational_status(OperationalStatus.MOVING_TO_IDLE)
            
            # Log task start event
            self._log_kpi_event("task_start", {
                "task_id": task.task_id,
                "task_type": task.task_type.value,
                "order_id": task.order_id,
                "shelf_id": task.shelf_id,
                "item_id": task.item_id,
                "quantity": task.quantity_to_pick
            })
            
            print(f"[TaskHandler] Started task {task.task_id}: {task.task_type.value} ({self._get_battery_level_info()})")
            return True
    
    def get_current_task(self) -> Optional[Task]:
        """
        Get the current active task.
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            Optional[Task]: Current task or None if idle
        """
        with ReadLock(self._status_lock):
            return self._current_task
    
    def handle_stall_event(self, reason: str) -> None:
        """
        Handle robot stall event.
        **Thread-safe**: Called by safety/traffic modules when robot is stuck.
        
        Args:
            reason: Description of why the robot is stalled
        """
        with WriteLock(self._status_lock):
            if self._operational_status == OperationalStatus.IDLE:
                return  # Not executing a task
            
            # Increment retry count if this is a retry
            if self._operational_status == OperationalStatus.STALLED:
                self._stall_retry_count += 1
                print(f"[TaskHandler] Stall retry #{self._stall_retry_count}: {reason}")
            else:
                # First stall, reset retry count
                self._stall_retry_count = 1
                print(f"[TaskHandler] Initial stall detected: {reason}")
            
            self._update_operational_status(OperationalStatus.STALLED)
            self._stall_reason = reason
            self._stall_start_time = time.time()
            self._stall_retry_start_time = time.time()
            
            # Stop motion and lane following immediately
            self.motion_executor.stop_execution()
            self.lane_follower.stop_following(force_release=False)  # Safe stop
    
    def is_idle(self) -> bool:
        """
        Check if robot is available for new tasks.
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            bool: True if robot can accept new tasks
        """
        with ReadLock(self._status_lock):
            # While waiting for charging, the robot is intentionally unavailable for bidding
            if getattr(self, "_waiting_for_charging", False):
                return False
            
            # If currently charging and battery above availability threshold, allow bidding
            try:
                if (self._current_task and self._current_task.task_type == TaskType.MOVE_TO_CHARGING and
                    self._operational_status in (OperationalStatus.CHARGING, OperationalStatus.MOVING_TO_CHARGING)):
                    battery_level = self.state_holder.get_battery_level()
                    if battery_level >= getattr(self, '_charging_availability_threshold', 0.60):
                        return True
            except Exception:
                pass
            # Consider IDLE_PARK (including WANDERING) as available for work
            if self._current_task and self._current_task.task_type == TaskType.IDLE_PARK:
                return True
            return (self._operational_status == OperationalStatus.IDLE and 
                    self._current_task is None)
    
    def cancel_task(self, reason: str = "User cancelled") -> None:
        """
        Cancel the current task.
        **Thread-safe**: Can be called from any thread.
        
        Args:
            reason: Reason for cancellation
        """
        with WriteLock(self._status_lock):
            if not self._current_task:
                raise TaskHandlingError("No active task to cancel")
            
            # Stop motion
            self.motion_executor.stop_execution()
            
            # Mark task as cancelled
            self._current_task.status = TaskStatus.CANCELLED
            self._current_task.completed_at = datetime.now()
            if not self._current_task.metadata:
                self._current_task.metadata = {}
            self._current_task.metadata['cancel_reason'] = reason
            
            print(f"[TaskHandler] Task {self._current_task.task_id} cancelled: {reason}")

            # On cancel: release reservation exactly once (idempotent)
            try:
                task = self._current_task
                if task and task.inventory_reserved:
                    if task.metadata is None:
                        task.metadata = {}
                    released_flag = task.metadata.get('reservation_released', False)
                    consumed_flag = task.metadata.get('reservation_consumed', False)
                    if not consumed_flag and not released_flag:
                        ok = self.simulation_data_service.release_inventory(
                            task.shelf_id, task.item_id, task.quantity_to_pick
                        )
                        if ok:
                            task.metadata['reservation_released'] = True
                            self.logger.info(
                                f"Task {task.task_id}: Released reservation {task.quantity_to_pick} of {task.item_id} on {task.shelf_id} after cancel"
                            )
                        else:
                            self.logger.warning(
                                f"Task {task.task_id}: Release reservation returned False for {task.item_id} on {task.shelf_id} (cancel) - may be orphaned"
                            )
            except SimulationDataServiceError as e:
                self.logger.error(f"Release inventory failed on cancel: {e}")

            # Reset transition flag and clean up
            self._transitioning_to_idle = False
            self._add_to_history(self._current_task)
            self._reset_task_state(preserve_bay_lock=False)
    
    def get_task_history(self, limit: int = 10) -> List[Task]:
        """
        Get recent task history.
        **Thread-safe**: Can be called from any thread.
        
        Args:
            limit: Maximum number of tasks to return
            
        Returns:
            List[Task]: Recent tasks
        """
        with ReadLock(self._status_lock):
            return self._task_history[-limit:] if self._task_history else []
    
    # Internal methods for task execution
    def _update_current_phase(self) -> None:
        """Internal: Update task execution based on current phase."""
        try:
            if self._task_phase == TaskPhase.PLANNING:
                self._handle_planning_phase()
            elif self._task_phase == TaskPhase.NAVIGATING_TO_SHELF:
                self._handle_navigation_phase(OperationalStatus.MOVING_TO_SHELF)
            elif self._task_phase == TaskPhase.PICKING_ITEM:
                self._handle_picking_phase()
            elif self._task_phase == TaskPhase.NAVIGATING_TO_DROPOFF:
                self._handle_navigation_phase(OperationalStatus.MOVING_TO_DROPOFF)
            elif self._task_phase == TaskPhase.DROPPING_ITEM:
                self._handle_dropping_phase()
            elif self._task_phase == TaskPhase.NAVIGATING_TO_CHARGING:
                self._handle_navigation_phase(OperationalStatus.MOVING_TO_CHARGING)
            elif self._task_phase == TaskPhase.CHARGING_BATTERY:
                self._handle_charging_phase()
            elif self._task_phase == TaskPhase.NAVIGATING_TO_IDLE:
                self._handle_navigation_phase(OperationalStatus.MOVING_TO_IDLE)
            elif self._task_phase == TaskPhase.WANDERING:
                self._handle_wandering_phase()
            
        except Exception as e:
            print(f"[TaskHandler] Error in phase {self._task_phase}: {e}")
            self._handle_task_error(str(e))
    
    def _handle_planning_phase(self) -> None:
        """
        Internal: Handle task planning phase with asynchronous path planning.
        
        This method implements the optimization to move path planning outside 
        the critical section. Path planning runs in a background thread to avoid
        blocking the 10Hz control loop and reader threads.
        """
        if not self._current_task:
            return  # No task to plan for
        
        # Check if planning is already in progress
        if self._planning_thread and self._planning_thread.is_alive():
            return  # Planning still in progress
        
        # Check if planning completed
        if self._planned_route is not None:
            # Route planning succeeded
            try:
                print(f"[TaskHandler] Starting lane following with {len(self._planned_route.segments)} segments (robot={self.robot_id})")
                self.lane_follower.follow_route(self._planned_route, self.robot_id)
                
                # Advance to next phase
                if self._current_task.task_type == TaskType.PICK_AND_DELIVER:
                    self._advance_to_phase(TaskPhase.NAVIGATING_TO_SHELF, OperationalStatus.MOVING_TO_SHELF)
                elif self._current_task.task_type == TaskType.MOVE_TO_CHARGING:
                    self._advance_to_phase(TaskPhase.NAVIGATING_TO_CHARGING, OperationalStatus.MOVING_TO_CHARGING)
                    # Release idle bay lock if robot is far from the bay (not navigating TO it)
                    self._release_idle_bay_lock_if_not_at_bay()
                elif self._current_task.task_type == TaskType.MOVE_TO_POSITION:
                    self._advance_to_phase(TaskPhase.NAVIGATING_TO_SHELF, OperationalStatus.MOVING_TO_SHELF)
                elif self._current_task.task_type == TaskType.IDLE_PARK:
                    if self._is_wandering:
                        try:
                            print(f"[Wander][{self.robot_id}] Starting WANDER route with {len(self._planned_route.segments)} segments")
                        except Exception:
                            pass
                        self._advance_to_phase(TaskPhase.WANDERING, OperationalStatus.WANDERING)
                        # KPI: wander_start
                        try:
                            self._log_kpi_event("wander_start", {"robot_id": self.robot_id})
                        except Exception:
                            pass
                    else:
                        self._advance_to_phase(TaskPhase.NAVIGATING_TO_IDLE, OperationalStatus.MOVING_TO_IDLE)
                
                # Clear planning state
                self._planned_route = None
                self._planning_target = None
                
            except LaneFollowingError as e:
                self._handle_task_error(f"Lane following failed: {e}")
            return
        
        # Check if planning failed
        if self._planning_error is not None:
            error_msg = self._planning_error
            self._planning_error = None
            self._planning_target = None
            print(f"[TaskHandler] Path planning failed (robot={self.robot_id}): {error_msg}")
            self._handle_task_error(f"Path planning failed: {error_msg}")
            return
        
        # Start new planning if not already started
        if self._planning_target is None:
            try:
                # During WANDERING phase, handle planning here to avoid double-plans
                if self._task_phase == TaskPhase.WANDERING:
                    # If a new plan is needed (e.g., handler set _planning_target above), do nothing here
                    return
                current_state = self.state_holder.get_robot_state()
                current_position = current_state.position
                
                # --- DEBUG LOGGING: initial robot position ---
                try:
                    start_cell_dbg = self.coordinate_system.world_to_cell(current_position[:2])
                    print(f"[TaskHandler] Initial robot position (robot={self.robot_id}): World{current_position[:2]} Cell({start_cell_dbg.x}, {start_cell_dbg.y})")
                except Exception as dbg_e:
                    print(f"[TaskHandler] Initial position conversion error (robot={self.robot_id}): {dbg_e}")
                
                # Determine target based on task type
                if self._current_task.task_type == TaskType.PICK_AND_DELIVER:
                    if self._current_task.shelf_id:
                        target_position = self._get_shelf_position(self._current_task.shelf_id)
                    else:
                        raise TaskHandlingError("Pick and deliver task missing shelf_id")
                
                elif self._current_task.task_type == TaskType.MOVE_TO_POSITION:
                    target_position = self._current_task.target_position
                    if not target_position:
                        raise TaskHandlingError("Move to position task missing target_position")
                    print(f"[TaskHandler] MOVE_TO_POSITION task target: {target_position}")
                    
                    # --- DEBUG LOGGING: target cell ---
                    try:
                        target_cell_dbg = self.coordinate_system.world_to_cell(target_position[:2])
                        print(f"[TaskHandler] Target cell (robot={self.robot_id}): ({target_cell_dbg.x}, {target_cell_dbg.y})")
                    except Exception as dbg_e:
                        print(f"[TaskHandler] Target position conversion error (robot={self.robot_id}): {dbg_e}")
                
                elif self._current_task.task_type == TaskType.MOVE_TO_CHARGING:
                    target_position = self._get_charging_station_position()
                
                elif self._current_task.task_type == TaskType.IDLE_PARK:
                    # Attempt to acquire idle bay lock first
                    bay_target = self._request_and_wait_for_idle_bay()
                    if self._idle_bay_lock_id:
                        # Bay lock acquired: plan to the bay
                        try:
                            print(f"[Wander][{self.robot_id}] Bay lock acquired {self._idle_bay_lock_id} during planning; heading to bay")
                        except Exception:
                            pass
                        target_position = bay_target
                    else:
                        # No idle bay available; WANDER fallback if enabled
                        if getattr(self, '_idle_wander_enabled', True):
                            try:
                                print(f"[Wander][{self.robot_id}] Enter (planning): no bay available")
                            except Exception:
                                pass
                            wander_target = self._select_wander_target(current_position[:2])
                            if wander_target is not None:
                                self._is_wandering = True
                                self._wandering_target = wander_target
                                target_position = (wander_target[0], wander_target[1])
                                try:
                                    print(f"[Wander][{self.robot_id}] No idle bay available; entering WANDER to {wander_target}")
                                except Exception:
                                    pass
                            else:
                                # No safe wander candidates; wait-in-place
                                try:
                                    print(f"[Wander][{self.robot_id}] No safe wander targets; waiting in place (no idle bay)")
                                except Exception:
                                    pass
                                target_position = bay_target
                                if tuple(target_position[:2]) == tuple(current_position[:2]):
                                    eps = 0.2
                                    target_position = (target_position[0] + eps, target_position[1])
                        else:
                            # Wander disabled; wait-in-place
                            try:
                                print(f"[Wander][{self.robot_id}] Wander disabled; waiting in place (no idle bay)")
                            except Exception:
                                pass
                            target_position = bay_target
                            if tuple(target_position[:2]) == tuple(current_position[:2]):
                                eps = 0.2
                                target_position = (target_position[0] + eps, target_position[1])
                
                else:
                    raise TaskHandlingError(f"Unsupported task type: {self._current_task.task_type}")
                
                # Start asynchronous path planning
                self._planning_target = target_position[:2]
                print(f"[TaskHandler] Starting path planning (robot={self.robot_id}): {current_position[:2]} -> {target_position[:2]}")
                # Choose navigation task type based on current task
                nav_task_type: NavTaskType
                if self._current_task.task_type == TaskType.MOVE_TO_POSITION:
                    nav_task_type = NavTaskType.MOVE_TO_POSITION
                elif self._current_task.task_type == TaskType.MOVE_TO_CHARGING:
                    nav_task_type = NavTaskType.MOVE_TO_CHARGING
                elif self._current_task.task_type == TaskType.IDLE_PARK:
                    # Planning to idle bay uses generic movement semantics
                    nav_task_type = NavTaskType.MOVE_TO_POSITION
                else:
                    nav_task_type = NavTaskType.PICK_AND_DELIVER

                # Guard: don't start planning threads for trivial no-op targets
                if (abs(current_position[0] - target_position[0]) < 1e-6 and
                    abs(current_position[1] - target_position[1]) < 1e-6):
                    print(f"[TaskHandler] Skipping trivial plan: start == target ({target_position[:2]})")
                    self._planning_error = None
                    self._planned_route = None
                    self._planning_target = None
                else:
                    self._planning_thread = threading.Thread(
                        target=self._async_path_planning,
                        args=(current_position[:2], target_position[:2], nav_task_type),
                        name=f"PathPlanning-{self.robot_id}",
                        daemon=True
                    )
                    self._planning_thread.start()
                self.logger.debug(f"Started asynchronous path planning to {target_position[:2]}")
                
            except Exception as e:
                print(f"[TaskHandler] Planning initialization failed (robot={self.robot_id}): {e}")
                self._handle_task_error(f"Planning initialization failed: {e}")
    
    def _handle_navigation_phase(self, expected_status: OperationalStatus) -> None:
        """Internal: Handle navigation phases using lane following."""
        if not self._current_task:
            return  # No task to navigate for

        # Check if robot is leaving idle bay and release lock if so
        self._check_and_release_idle_bay_lock_if_leaving()

        # Check lane following status
        lane_status = self.lane_follower.get_lane_following_status()
        lane_result = self.lane_follower.get_lane_following_result()

        # Debug logs for idle navigation
        if expected_status == OperationalStatus.MOVING_TO_IDLE:
            # print(f"[TaskHandler] DEBUG: IDLE navigation - lane_status={lane_status}, current_phase={self._task_phase}")
            if lane_result:
                # print(f"[TaskHandler] DEBUG: IDLE lane_result - success={lane_result.success}, error={lane_result.error_message}")
                motion_status = self.motion_executor.get_motion_status()
                # print(f"[TaskHandler] DEBUG: IDLE motion_status={motion_status}")

        # Check if robot is close to shelf during MOVING_TO_SHELF
        if (expected_status == OperationalStatus.MOVING_TO_SHELF and
            self._is_close_to_target_shelf()):
            # Transition to APPROACHING_SHELF for relaxed safety margins
            self._operational_status = OperationalStatus.APPROACHING_SHELF
            print(f"[TaskHandler] Transitioned to APPROACHING_SHELF - relaxed safety margins active")

        if lane_status == LaneFollowingStatus.COMPLETED:
            # Navigation completed
            if expected_status in [OperationalStatus.MOVING_TO_SHELF, OperationalStatus.APPROACHING_SHELF]:
                # For PICK_AND_DELIVER tasks, ensure robot is oriented towards shelf before picking
                if self._current_task.task_type == TaskType.PICK_AND_DELIVER:
                    # Start shelf orientation if not already in progress
                    if not self._shelf_orientation_pending:
                        self._start_shelf_orientation()
                        self._shelf_orientation_pending = True
                        self._orientation_start_time = time.time()
                        self.logger.info("Navigation complete, orienting towards shelf before picking")
                    else:
                        # Check if orientation is complete
                        if self._is_shelf_orientation_complete():
                            # Orientation complete - advance to picking phase
                            self._shelf_orientation_pending = False
                            self._orientation_start_time = None
                            self.logger.info("Shelf orientation complete, starting picking operation")
                            self._advance_to_phase(TaskPhase.PICKING_ITEM, OperationalStatus.PICKING)

                            # Reset lane follower status to IDLE so it can start new routes
                            self.lane_follower.stop_following()
                        # Orientation still in progress - stay in navigation phase
                else:
                    # Non-PICK_AND_DELIVER tasks (e.g., MOVE_TO_POSITION) - complete immediately
                    self._complete_task()
                    # Reset lane follower status to IDLE so it can start new routes
                    self.lane_follower.stop_following()
            elif expected_status == OperationalStatus.MOVING_TO_DROPOFF:
                self._advance_to_phase(TaskPhase.DROPPING_ITEM, OperationalStatus.DROPPING)

                # Reset lane follower status to IDLE so it can start new routes
                self.lane_follower.stop_following()
            elif expected_status == OperationalStatus.MOVING_TO_CHARGING:
                self._advance_to_phase(TaskPhase.CHARGING_BATTERY, OperationalStatus.CHARGING)

                # Reset lane follower status to IDLE so it can start new routes
                self.lane_follower.stop_following()
            elif expected_status == OperationalStatus.MOVING_TO_IDLE:
                # Idle zone reached - complete the IDLE_PARK task
                self._reached_idle_bay = True  # Mark that robot has reached the idle bay
                self._complete_task()
                # Remain in waiting mode (unavailable) if waiting for charging
                if self._waiting_for_charging:
                    # Keep operational status IDLE but do not allow bidding (handled in is_idle)
                    self._operational_status = OperationalStatus.IDLE

                # Reset lane follower status to IDLE so it can start new routes
                self.lane_follower.stop_following()
        
        elif lane_status == LaneFollowingStatus.ERROR:
            self._handle_task_error(f"Lane following error: {lane_result.error_message}")
        
        elif lane_status == LaneFollowingStatus.BLOCKED:
            self.handle_stall_event("Path blocked during lane following")

    def _handle_wandering_phase(self) -> None:
        """Internal: Handle WANDER behavior and periodic idle-bay retries.

        - Periodically retry idle bay lock acquisition.
        - If a lock is acquired mid-wander, cancel wander and plan to bay.
        - When current wander route completes, immediately select and plan to a new wander target.
        - Recover from errors/blocks by selecting a different wander target.
        """
        if not self._current_task or not self._is_wandering or self._current_task.task_type != TaskType.IDLE_PARK:
            return

        # Retry idle bay acquisition periodically
        now = time.time()
        if now - self._last_idle_retry_time >= max(0.5, self._idle_retry_interval):
            self._last_idle_retry_time = now
            bay_target = self._request_and_wait_for_idle_bay()
            if self._idle_bay_lock_id:
                # Transition from wander to planning for idle bay
                try:
                    print(f"[Wander][{self.robot_id}] Bay lock acquired {self._idle_bay_lock_id}; switching to IDLE_PARK navigation")
                except Exception:
                    pass
                # KPI: wander_stop (exit due to bay acquisition)
                try:
                    self._log_kpi_event("wander_stop", {"reason": "bay_acquired", "bay_id": self._idle_bay_lock_id})
                except Exception:
                    pass
                # Stop current route safely and transition to PLANNING for bay
                self.lane_follower.stop_following()
                self._is_wandering = False
                self._wandering_target = None
                # Clear any old planning state
                self._planned_route = None
                self._planning_error = None
                self._planning_target = None
                # Set phase to PLANNING so planning handler will compute route to bay
                self._task_phase = TaskPhase.PLANNING
                self._operational_status = OperationalStatus.MOVING_TO_IDLE
                try:
                    print(f"[Wander][{self.robot_id}] Transitioning to PLANNING for idle bay {self._idle_bay_lock_id}")
                except Exception:
                    pass
                return

        # If a wander planning result is ready, start following it (avoid waiting for PLANNING phase)
        if self._planned_route is not None and (self._planning_thread is None or not self._planning_thread.is_alive()):
            # Only start a new route if lane follower is idle
            lf_status = self.lane_follower.get_lane_following_status()
            if lf_status in [LaneFollowingStatus.IDLE, LaneFollowingStatus.COMPLETED]:
                if lf_status == LaneFollowingStatus.COMPLETED:
                    # Ensure clean state before starting a new route
                    self.lane_follower.stop_following()
                try:
                    print(f"[Wander][{self.robot_id}] Planning completed; starting wander route with {len(self._planned_route.segments)} segments")
                except Exception:
                    pass
                try:
                    self.lane_follower.follow_route(self._planned_route, self.robot_id)
                except Exception as e:
                    print(f"[Wander][{self.robot_id}] Failed to start wander route: {e}")
                finally:
                    # Clear planning state regardless of start success to avoid stale loops
                    self._planned_route = None
                    self._planning_target = None
                    self._planning_error = None

        # Monitor lane following status during wander
        lane_status = self.lane_follower.get_lane_following_status()
        lane_result = self.lane_follower.get_lane_following_result()

        if lane_status == LaneFollowingStatus.COMPLETED:
            # Wander target reached — pick a new one and continue wandering
            # Ensure we reset follower state before creating a new plan
            self.lane_follower.stop_following()
            current_xy = self.state_holder.get_robot_state().position[:2]
            next_target = self._select_wander_target(current_xy)
            if next_target is not None:
                self._wandering_target = next_target
                try:
                    print(f"[Wander][{self.robot_id}] Reached wander target; selecting new target {next_target}")
                except Exception:
                    pass
                # KPI: wander_target
                try:
                    self._log_kpi_event("wander_target", {"target": {"x": next_target[0], "y": next_target[1]}})
                except Exception:
                    pass
                # Plan to next wander target
                self._planned_route = None
                self._planning_error = None
                self._planning_target = next_target[:2]
                print(f"[TaskHandler] Starting path planning (robot={self.robot_id}): {current_xy} -> {next_target}")
                self._planning_thread = threading.Thread(
                    target=self._async_path_planning,
                    args=(current_xy, next_target[:2], NavTaskType.MOVE_TO_POSITION),
                    name=f"PathPlanning-{self.robot_id}",
                    daemon=True
                )
                self._planning_thread.start()
                return
            else:
                # No further wander targets — remain in place and keep retrying bays
                try:
                    print(f"[Wander][{self.robot_id}] No additional wander targets; waiting and retrying idle bays")
                except Exception:
                    pass
                # KPI: wander_stop (temporary pause due to no targets)
                try:
                    self._log_kpi_event("wander_stop", {"reason": "no_targets"})
                except Exception:
                    pass
                return

        if lane_status == LaneFollowingStatus.ERROR or lane_status == LaneFollowingStatus.BLOCKED:
            # Recover by choosing a different wander target
            current_xy = self.state_holder.get_robot_state().position[:2]
            alt_target = self._select_wander_target(current_xy)
            if alt_target is not None:
                try:
                    print(f"[Wander][{self.robot_id}] Recovering from {lane_status.value}; re-targeting to {alt_target}")
                except Exception:
                    pass
                self.lane_follower.stop_following()
                self._wandering_target = alt_target
                self._planned_route = None
                self._planning_error = None
                self._planning_target = alt_target[:2]
                # KPI: wander_target after recovery
                try:
                    self._log_kpi_event("wander_target", {"target": {"x": alt_target[0], "y": alt_target[1]}, "reason": "recovery"})
                except Exception:
                    pass
                print(f"[TaskHandler] Starting path planning (robot={self.robot_id}): {current_xy} -> {alt_target}")
                self._planning_thread = threading.Thread(
                    target=self._async_path_planning,
                    args=(current_xy, alt_target[:2], NavTaskType.MOVE_TO_POSITION),
                    name=f"PathPlanning-{self.robot_id}",
                    daemon=True
                )
                self._planning_thread.start()
                return
            else:
                try:
                    print(f"[Wander][{self.robot_id}] Recovery failed; no wander targets available")
                except Exception:
                    pass
                # KPI: wander_stop due to recovery failure
                try:
                    self._log_kpi_event("wander_stop", {"reason": "recovery_failed"})
                except Exception:
                    pass
                # Stay in place; next idle bay retry may succeed
                return

    def _check_and_release_idle_bay_lock_if_leaving(self) -> None:
        """Check if robot is leaving current idle bay and release lock if so."""
        if not self._idle_bay_lock_id:
            return

        # Only check for leaving if robot has actually reached the idle bay
        # This prevents premature lock release during navigation TO the bay
        if not self._reached_idle_bay:
            return

        # Get current robot position
        current_position = self.state_holder.get_robot_state().position[:2]

        # Calculate idle bay position from bay_id
        try:
            parts = self._idle_bay_lock_id.split('_')
            row = int(parts[1])
            col = int(parts[2])
            bay_world_pos = self.coordinate_system.cell_to_world(Cell(x=col, y=row))
            bay_position = (bay_world_pos.x, bay_world_pos.y) if hasattr(bay_world_pos, 'x') else bay_world_pos
        except (ValueError, IndexError, AttributeError):
            # Fallback: get position from simulation data service
            idle_bays = self.simulation_data_service.list_idle_bays()
            bay_position = None
            for bid, pos in idle_bays:
                if bid == self._idle_bay_lock_id:
                    bay_position = pos
                    break
            if not bay_position:
                return

        # Check distance from idle bay
        distance_from_idle_bay = ((current_position[0] - bay_position[0]) ** 2 +
                                  (current_position[1] - bay_position[1]) ** 2) ** 0.5

        # If moving more than 1.0 units away from idle bay, we're leaving it
        if distance_from_idle_bay > 1.0:
            try:
                released = self.simulation_data_service.release_bay_lock(self._idle_bay_lock_id, self.robot_id)
                if released:
                    self.logger.info(f"Released idle bay lock {self._idle_bay_lock_id} (robot leaving bay)")
                    try:
                        print(f"[BayLock][{self.robot_id}] Released idle bay {self._idle_bay_lock_id} (leaving)")
                    except Exception:
                        pass
                else:
                    self.logger.warning(f"Failed to release idle bay lock {self._idle_bay_lock_id}")
            except SimulationDataServiceError as e:
                self.logger.error(f"Error releasing idle bay lock {self._idle_bay_lock_id}: {e}")
            finally:
                # Clear the lock state regardless of release success
                self._idle_bay_lock_id = None

    def _release_idle_bay_lock_if_not_at_bay(self) -> None:
        """Release idle bay lock if robot is far from the bay (not navigating TO it).

        Used when starting charging tasks to ensure idle bay locks are properly released
        if the robot has moved away from the idle bay.
        """
        if not self._idle_bay_lock_id:
            return

        # Get current robot position
        current_position = self.state_holder.get_robot_state().position[:2]

        # Calculate idle bay position from bay_id
        try:
            parts = self._idle_bay_lock_id.split('_')
            row = int(parts[1])
            col = int(parts[2])
            bay_world_pos = self.coordinate_system.cell_to_world(Cell(x=col, y=row))
            bay_position = (bay_world_pos.x, bay_world_pos.y) if hasattr(bay_world_pos, 'x') else bay_world_pos
        except (ValueError, IndexError, AttributeError):
            # Fallback: get position from simulation data service
            idle_bays = self.simulation_data_service.list_idle_bays()
            bay_position = None
            for bid, pos in idle_bays:
                if bid == self._idle_bay_lock_id:
                    bay_position = pos
                    break
            if not bay_position:
                return

        # Check distance from idle bay
        distance_from_idle_bay = ((current_position[0] - bay_position[0]) ** 2 +
                                  (current_position[1] - bay_position[1]) ** 2) ** 0.5

        # If moving more than 1.0 units away from idle bay, release the lock
        # (we're clearly not navigating TO the bay if starting a charging task)
        if distance_from_idle_bay > 1.0:
            try:
                released = self.simulation_data_service.release_bay_lock(self._idle_bay_lock_id, self.robot_id)
                if released:
                    self.logger.info(f"Released idle bay lock {self._idle_bay_lock_id} (starting charging task)")
                    try:
                        print(f"[BayLock][{self.robot_id}] Released idle bay {self._idle_bay_lock_id} (starting charging)")
                    except Exception:
                        pass
                else:
                    self.logger.warning(f"Failed to release idle bay lock {self._idle_bay_lock_id}")
            except SimulationDataServiceError as e:
                self.logger.error(f"Error releasing idle bay lock {self._idle_bay_lock_id}: {e}")
            finally:
                # Clear the lock state regardless of release success
                self._idle_bay_lock_id = None
                self._reached_idle_bay = False  # Reset the reached flag

    def _start_shelf_orientation(self) -> None:
        """
        Internal: Start rotating robot to face the target shelf.

        Calculates the required heading angle to face the shelf and initiates
        the rotation using the motion executor.
        """
        try:
            # Get shelf position from simulation data service
            shelf_position = self._get_shelf_position(self._current_task.shelf_id)
            if not shelf_position:
                self.logger.warning(f"Cannot orient to shelf {self._current_task.shelf_id}: invalid position")
                return

            # Get current robot position and orientation
            robot_state = self.state_holder.get_robot_state()
            robot_position = robot_state.position[:2]  # (x, y)
            current_heading = robot_state.position[2]  # theta (current heading)

            # Calculate target heading to face the shelf
            dx = shelf_position[0] - robot_position[0]
            dy = shelf_position[1] - robot_position[1]
            target_heading = math.atan2(dy, dx)

            # Normalize angle difference to [-pi, pi] for optimal rotation
            heading_error = target_heading - current_heading
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi

            # Only rotate if the error is significant (> 5 degrees)
            if abs(heading_error) > math.radians(5):
                # Start rotation to face the shelf
                self.motion_executor.rotate_to_heading(target_heading)
                self.logger.info(
                    f"Orienting to face shelf {self._current_task.shelf_id} "
                    f"(current: {current_heading:.3f}, target: {target_heading:.3f})"
                )
            else:
                # Already facing the shelf within tolerance
                self.logger.debug(f"Already facing shelf {self._current_task.shelf_id} within tolerance")

        except Exception as e:
            self.logger.error(f"Failed to start shelf orientation: {e}")
            # Don't fail the task for orientation errors - continue to picking

    def _is_shelf_orientation_complete(self) -> bool:
        """
        Internal: Check if shelf orientation rotation is complete.

        Includes timeout protection to prevent indefinite waiting.

        Returns:
            bool: True if orientation is complete, timed out, or not needed
        """
        try:
            # Check for timeout (30 seconds max for orientation)
            if self._orientation_start_time:
                elapsed_time = time.time() - self._orientation_start_time
                if elapsed_time > 30.0:  # 30 second timeout
                    self.logger.warning(f"Shelf orientation timed out after {elapsed_time:.1f}s")
                    # Stop any ongoing motion and consider complete
                    try:
                        self.motion_executor.stop_execution()
                    except Exception:
                        pass  # Ignore errors when stopping
                    return True

            # Check if motion executor is idle (rotation completed)
            motion_status = self.motion_executor.get_motion_status()
            return motion_status == MotionStatus.IDLE
        except Exception as e:
            self.logger.error(f"Error checking orientation completion: {e}")
            # Assume complete on error to avoid blocking
            return True

    def _handle_picking_phase(self) -> None:
        """Internal: Handle item picking phase with shelf locking and inventory management."""
        if not self._current_task:
            return
            
        # Initialize picking phase (use explicit flag instead of _phase_start_time)
        if not getattr(self, "_picking_initialized", False):
            if not self._phase_start_time:
                self._phase_start_time = time.time()
            self._picking_initialized = False
            print("[TaskHandler] Picking phase entering init block")
            
            # Initialize picking visuals (no physical shelf lock)
            if self._current_task.shelf_id and not self._picking_initialized:
                print(f"[TaskHandler] Picking init: shelf_id={self._current_task.shelf_id}")
                if self.appearance_service is not None:
                    try:
                        set_ok = self.appearance_service.set_carrying_appearance(self.robot_id, True)
                        print(f"[TaskHandler] Logical carrying appearance set (GREEN) result: {set_ok}")
                    except Exception as e:
                        self.logger.warning(f"Failed to set carrying appearance: {e}")
                else:
                    print("[TaskHandler] Carrying appearance service unavailable")
                self._picking_initialized = True
        
        # Ensure phase start time is set even if the guard above was skipped
        if not self._phase_start_time:
            self._phase_start_time = time.time()
        elapsed_time = time.time() - self._phase_start_time

        # Robot should remain stationary during picking duration

        if elapsed_time >= self._picking_duration:
            # Picking operation completed - log event (inventory consumed at drop-off)
            if self._current_task.item_id and self._current_task.shelf_id:
                self._log_kpi_event("pick_operation", {
                    "shelf_id": self._current_task.shelf_id,
                    "item_id": self._current_task.item_id,
                    "quantity": self._current_task.quantity_to_pick,
                    "success": True
                })
            
            
            # Plan route to dropoff using lane-based navigation
            try:
                from interfaces.navigation_types import Point, TaskType as NavTaskType
                
                current_state = self.state_holder.get_robot_state()
                dropoff_position = self._get_dropoff_position()
                
                start_point = Point(x=current_state.position[0], y=current_state.position[1])
                target_point = Point(x=dropoff_position[0], y=dropoff_position[1])
                
                planning_result = self.path_planner.plan_route(
                    start=start_point,
                    goal=target_point,
                    task_type=NavTaskType.PICK_AND_DELIVER
                )
                
                if planning_result.success and planning_result.route:
                    self.lane_follower.follow_route(planning_result.route, self.robot_id)
                    self._advance_to_phase(TaskPhase.NAVIGATING_TO_DROPOFF, OperationalStatus.MOVING_TO_DROPOFF)
                else:
                    self._handle_task_error(f"Failed to plan dropoff route: {planning_result.error_message}")
                
            except Exception as e:
                self._handle_task_error(f"Failed to plan dropoff route: {e}")
    
    def _handle_dropping_phase(self) -> None:
        """Internal: Handle item dropping phase with KPI logging."""
        if not self._current_task:
            return
            
        # Simulate dropping operation
        if not self._phase_start_time:
            self._phase_start_time = time.time()
            # print(f"[TaskHandler] DEBUG: Dropping phase started, duration={self._dropping_duration}s")

        elapsed_time = time.time() - self._phase_start_time
        # print(f"[TaskHandler] DEBUG: Dropping progress: {elapsed_time:.1f}s / {self._dropping_duration}s")
        
        if elapsed_time >= self._dropping_duration:
            # Log drop operation event
            self._log_kpi_event("drop_operation", {
                "order_id": self._current_task.order_id,
                "item_id": self._current_task.item_id,
                "quantity": self._current_task.quantity_to_pick,
                "dropoff_zone": self._current_task.dropoff_zone,
                "success": True
            })

            # Consume reserved inventory exactly once (idempotent)
            if self._current_task.inventory_reserved:
                if self._current_task.metadata is None:
                    self._current_task.metadata = {}
                consumed_flag = self._current_task.metadata.get('reservation_consumed', False)
                if not consumed_flag:
                    try:
                        ok = self.simulation_data_service.consume_inventory(
                            self._current_task.shelf_id,
                            self._current_task.item_id,
                            self._current_task.quantity_to_pick
                        )
                        if ok:
                            self._current_task.metadata['reservation_consumed'] = True
                            self.logger.info(
                                f"Task {self._current_task.task_id}: Consumed {self._current_task.quantity_to_pick} of {self._current_task.item_id} from {self._current_task.shelf_id}"
                            )
                        else:
                            self.logger.warning(
                                f"Task {self._current_task.task_id}: Consume failed (insufficient inventory) for {self._current_task.item_id} on {self._current_task.shelf_id} - leaving pending for cleanup"
                            )
                    except SimulationDataServiceError as e:
                        self.logger.error(f"Consume inventory failed: {e}")
                        # Do not fail the task completion at this stage; leave pending for operator cleanup
            
            # Reset carrying appearance (Phase 1: visual shelf carry)
            if self.appearance_service is not None:
                try:
                    result = self.appearance_service.set_carrying_appearance(self.robot_id, False)
                    if result:
                        print(f"[TaskHandler] Robot {self.robot_id} VISUALLY CHANGED TO RED (normal - shelf released)")
                        self.logger.info(f"Reset carrying appearance for robot {self.robot_id}")
                    else:
                        print(f"[TaskHandler] WARNING: Failed to reset carrying appearance for robot {self.robot_id}")
                except Exception as e:
                    print(f"[TaskHandler] ERROR: Exception resetting carrying appearance: {e}")
                    self.logger.warning(f"Failed to reset carrying appearance: {e}")
            else:
                print(f"[TaskHandler] INFO: AppearanceService not available; skipping reset color for {self.robot_id}")

            # No physical shelf unlock; physical lock calls removed in Phase 4

            # Dropping completed, task finished
            self._complete_task()

    
    def _handle_charging_phase(self) -> None:
        """Internal: Handle battery charging phase with proper battery manager integration."""
        current_battery = self.state_holder.get_battery_level()
        self.logger.debug(f"[Charging] Battery: {current_battery:.1%}, BatteryManager: {self.battery_manager is not None}")

        # Send heartbeat to maintain charging station lock
        if self._locked_bay_id and self.charging_station_manager:
            try:
                success = self.charging_station_manager.heartbeat_charging_station(
                    self._locked_bay_id, self.robot_id
                )
                if not success:
                    self.logger.warning(f"Failed to heartbeat charging station {self._locked_bay_id}")
            except Exception as e:
                self.logger.error(f"Error heartbeating charging station {self._locked_bay_id}: {e}")

        # Start charging session if not already charging
        if self.battery_manager:
            is_charging = self.battery_manager.is_charging()
            self.logger.debug(f"[Charging] Battery manager exists, is_charging: {is_charging}")

            if not is_charging:
                self.logger.info("[Charging] Starting charging session...")
                success = self.battery_manager.start_charging_session()
                self.logger.info(f"[Charging] Start charging session result: {success}")
                if not success:
                    self.logger.warning("Failed to start charging session")
                    # Continue anyway - don't fail the task for charging issues
            else:
                self.logger.debug("[Charging] Already charging")
        else:
            self.logger.warning("[Charging] No battery manager available!")

        # Check if charging is complete (battery at 100% or higher)
        if current_battery >= 1.0: #TODO nake it configurable
            self.logger.info(f"[Charging] Battery charged to {current_battery:.1%}, completing task")
            # Stop charging session
            if self.battery_manager and self.battery_manager.is_charging():
                success = self.battery_manager.stop_charging_session()
                self.logger.info(f"[Charging] Stop charging session result: {success}")
                if not success:
                    self.logger.warning("Failed to stop charging session")

            self._complete_task()
    
    def _handle_stall_recovery(self) -> None:
        """
        Internal: Attempt to recover from stall condition with retry logic.
        
        TODO: STALL RECOVERY EXPANSION
        Current implementation uses simple retry logic. Future enhancements should include:
        
        1. **Intelligent Retry Strategies**:
           - Different retry delays based on stall reason (obstacle vs. traffic vs. mechanical)
           - Exponential backoff for repeated stalls
           - Maximum retry limits per task phase
        
        2. **Path Replanning Integration**:
           - Automatic path replanning when stalls occur during navigation
           - Alternative route generation for blocked paths
           - Conflict box lock management during replanning
        
        3. **Stall Classification**:
           - Categorize stalls by type (temporary obstacle, permanent blockage, traffic jam)
           - Different recovery strategies for different stall types
           - Integration with traffic management system
        
        4. **Advanced Recovery Actions**:
           - Backward movement to clear blocked position
           - Sideways movement for narrow passages
           - Communication with other robots for coordinated recovery
        
        5. **Monitoring and Analytics**:
           - Stall frequency tracking per robot and location
           - Performance metrics for recovery success rates
           - Integration with warehouse analytics system
        """
        if not self._stall_start_time:
            return
        
        elapsed_time = time.time() - self._stall_start_time
        retry_elapsed = time.time() - (self._stall_retry_start_time or self._stall_start_time)
        
        # Check if we've exceeded maximum retry attempts
        if self._stall_retry_count > self._retry_attempts:
            self._handle_task_error(f"Maximum stall retries ({self._retry_attempts}) exceeded: {self._stall_reason}")
            return
        
        # Check if we've exceeded the overall stall timeout
        if elapsed_time > self._stall_recovery_timeout:
            self._handle_task_error(f"Stall recovery timeout ({self._stall_recovery_timeout}s): {self._stall_reason}")
            return
        
        # Wait for retry delay before attempting recovery
        if retry_elapsed < self._retry_delay:
            return  # Still waiting for retry delay
        
        # Attempt recovery
        print(f"[TaskHandler] Attempting stall recovery (attempt {self._stall_retry_count}/{self._retry_attempts})")
        
        # Simple recovery: try to resume the current phase
        self._update_operational_status(self._get_status_for_phase(self._task_phase))
        self._stall_reason = None
        self._stall_start_time = None
        self._stall_retry_start_time = None
        # Note: _stall_retry_count will be reset on next stall event
        
        print(f"[TaskHandler] Stall recovery successful: resuming task")
    
    def _advance_to_phase(self, new_phase: TaskPhase, new_status: OperationalStatus) -> None:
        """Internal: Advance to next task phase."""
        # Emit previous phase duration before switching (best-effort)
        try:
            if hasattr(self, "_phase_start_time") and self._phase_start_time:
                prev_phase = getattr(self, "_task_phase", None)
                if prev_phase is not None:
                    duration = max(0.0, time.time() - float(self._phase_start_time))
                    self._log_kpi_event("phase_completed", {
                        "phase_duration_seconds": duration
                    })
        except Exception:
            pass

        self._task_phase = new_phase
        self._update_operational_status(new_status)
        self._phase_start_time = time.time()

        # Emit phase started event (numeric code for compatibility)
        try:
            idx = float(TASK_PHASE_ORDER.index(new_phase)) if new_phase in TASK_PHASE_ORDER else 0.0
            self._log_kpi_event("phase_started", {
                "phase_code": idx
            })
        except Exception:
            pass

        # Stop motion when entering phases that require stopping
        if new_phase in [TaskPhase.PICKING_ITEM, TaskPhase.DROPPING_ITEM]:
            try:
                self.motion_executor.stop_execution()
                phase_name = "PICKING_ITEM" if new_phase == TaskPhase.PICKING_ITEM else "DROPPING_ITEM"
                print(f"[TaskHandler] Motion stopped for {phase_name} phase ({self._get_battery_level_info()})")
            except Exception as e:
                print(f"[TaskHandler] Warning: Failed to stop motion for phase {new_phase}: {e}")

        # Reset per-phase init flags on transition
        if new_phase == TaskPhase.PICKING_ITEM:
            try:
                self._picking_initialized = False
            except Exception:
                pass
        elif new_phase == TaskPhase.DROPPING_ITEM:
            try:
                self._dropping_initialized = False
            except Exception:
                pass

        print(f"[{self.robot_id}][TaskHandler] Advanced to phase: {new_phase.value} ({self._get_battery_level_info()})")
    
    def _complete_task(self) -> None:
        """Internal: Complete the current task with cleanup and state reset."""
        if not self._current_task:
            return
            
        # Mark task as completed
        self._current_task.status = TaskStatus.COMPLETED
        self._current_task.completed_at = datetime.now()
        
        # Log task completion event
        task_duration = time.time() - self._task_start_time if self._task_start_time else 0
        self._log_kpi_event("task_completed", {
            "task_id": self._current_task.task_id,
            "order_id": self._current_task.order_id,
            "duration_seconds": task_duration,
            "phase": self._task_phase.value,
            # Numeric flags for KPI aggregation
            "is_pick_and_deliver": 1.0 if self._current_task.task_type == TaskType.PICK_AND_DELIVER else 0.0,
            # Emit PD-only duration for targeted averaging
            **({"pd_duration_seconds": float(task_duration)} if self._current_task.task_type == TaskType.PICK_AND_DELIVER else {})
        })
        
        print(f"[TaskHandler] Task {self._current_task.task_id} completed successfully ({self._get_battery_level_info()})")
        
        # Add to history
        self._add_to_history(self._current_task)
        
        # Capture the completed task type before resetting state
        completed_task_type = self._current_task.task_type

        # Handle charging task completion
        if completed_task_type == TaskType.MOVE_TO_CHARGING:
            self._charging_task_pending = False
            # Release charging station through proper manager if lock is held
            if self._locked_bay_id and self.charging_station_manager:
                try:
                    released = self.charging_station_manager.release_charging_station(
                        self._locked_bay_id, self.robot_id
                    )
                    if released:
                        self.logger.info(f"Released charging station {self._locked_bay_id}")
                    else:
                        self.logger.warning(f"Failed to release charging station {self._locked_bay_id}")
                except Exception as e:
                    self.logger.error(f"Error releasing charging station {self._locked_bay_id}: {e}")
                finally:
                    # Always clear the lock reference to prevent stale state
                    self._locked_bay_id = None

        # Reset task state first so the robot is truly idle and ready for the next task
        # This prevents "robot busy" when starting the auto IDLE_PARK task
        # Preserve bay lock if transitioning to IDLE_PARK
        preserve_lock = self._transitioning_to_idle
        self._reset_task_state(preserve_bay_lock=preserve_lock)
        
        # Automatically navigate to idle zone after task completion (AFTER state reset)
        # But only for non-IDLE_PARK tasks to prevent infinite loops
        if completed_task_type != TaskType.IDLE_PARK:
            self._navigate_to_idle_zone_after_completion()
    
    def _navigate_to_idle_zone_after_completion(self) -> None:
        """
        Internal: Automatically create and start an IDLE_PARK task after task completion.

        This ensures robots automatically move to idle zones instead of staying in place.
        """
        try:
            # Prevent infinite loop: don't create IDLE_PARK task if we just completed one
            if (self._current_task and
                self._current_task.task_type == TaskType.IDLE_PARK):
                print(f"[TaskHandler] Skipping auto IDLE_PARK creation - just completed IDLE_PARK task")
                return

            # Set flag to preserve bay lock during transition
            self._transitioning_to_idle = True

            try:
                # Create a new IDLE_PARK task using the factory method
                idle_task = Task.create_idle_park_task(
                    task_id=f"idle_park_{uuid.uuid4().hex[:8]}",
                    robot_id=self.robot_id
                )

                print(f"[TaskHandler] Auto-creating IDLE_PARK task: {idle_task.task_id}")

                # Start the idle park task immediately
                # print(f"[TaskHandler] DEBUG: About to start IDLE_PARK task {idle_task.task_id}")
                if self.start_task(idle_task):
                    print(f"[TaskHandler] Successfully started auto IDLE_PARK task ({self._get_battery_level_info()})")
                    # print(f"[TaskHandler] DEBUG: IDLE task started, phase={self._task_phase}, operational_status={self._operational_status}")
                else:
                    print(f"[TaskHandler] Failed to start auto IDLE_PARK task - robot busy")
                    self._transitioning_to_idle = False  # Reset flag on failure

            except Exception as e:
                print(f"[TaskHandler] Error creating auto IDLE_PARK task: {e}")
                self._transitioning_to_idle = False  # Reset flag on error
                # Don't fail the main task completion - this is just a convenience feature

        finally:
            # Flag remains set if task started successfully
            # Will be reset when new task begins or on error
            pass
    
    def _handle_task_error(self, error_message: str) -> None:
        """Internal: Handle task execution error with cleanup and KPI logging."""
        if self._current_task:
            self._current_task.status = TaskStatus.FAILED
            self._current_task.completed_at = datetime.now()
            if not self._current_task.metadata:
                self._current_task.metadata = {}
            self._current_task.metadata['error'] = error_message
            
            # Log task failure event
            task_duration = time.time() - self._task_start_time if self._task_start_time else 0
            self._log_kpi_event("task_failed", {
                "task_id": self._current_task.task_id,
                "order_id": self._current_task.order_id,
            "duration_seconds": task_duration,
                "error": error_message,
            "phase": self._task_phase.value,
            # Numeric flags for KPI aggregation
            "is_pick_and_deliver": 1.0 if self._current_task.task_type == TaskType.PICK_AND_DELIVER else 0.0,
            **({"pd_duration_seconds": float(task_duration)} if self._current_task.task_type == TaskType.PICK_AND_DELIVER else {})
            })
            
            print(f"[TaskHandler] Task {self._current_task.task_id} failed: {error_message}")
            self._add_to_history(self._current_task)
        
        # On failure: release reservation exactly once (idempotent)
        try:
            task = self._current_task
            if task and task.inventory_reserved:
                if task.metadata is None:
                    task.metadata = {}
                released_flag = task.metadata.get('reservation_released', False)
                consumed_flag = task.metadata.get('reservation_consumed', False)
                # Only release if not already consumed and not already released
                if not consumed_flag and not released_flag:
                    ok = self.simulation_data_service.release_inventory(
                        task.shelf_id, task.item_id, task.quantity_to_pick
                    )
                    if ok:
                        task.metadata['reservation_released'] = True
                        self.logger.info(
                            f"Task {task.task_id}: Released reservation {task.quantity_to_pick} of {task.item_id} on {task.shelf_id} after failure"
                        )
                    else:
                        self.logger.warning(
                            f"Task {task.task_id}: Release reservation returned False for {task.item_id} on {task.shelf_id} - may be orphaned"
                        )
        except SimulationDataServiceError as e:
            self.logger.error(f"Release inventory failed after task error: {e}")

        self._reset_task_state(preserve_bay_lock=False)
    
    def _reset_task_state(self, preserve_bay_lock: bool = False) -> None:
        """Internal: Reset task state to idle with cleanup."""
        # CRITICAL: Stop motion executor and lane follower when task completes
        self.motion_executor.stop_execution()
        self.lane_follower.stop_following(force_release=False)  # Safe stop

            # No physical shelf locks to clean; inventory reservations handled at failure/cancel paths

        # Release idle bay lock if held (unless preserving for IDLE_PARK transition)
        # Note: Charging station locks are already released in _complete_task()
        if not preserve_bay_lock and self._idle_bay_lock_id:
            try:
                released = self.simulation_data_service.release_bay_lock(self._idle_bay_lock_id, self.robot_id)
                if released:
                    self.logger.info(f"Released bay lock {self._idle_bay_lock_id}")
                    try:
                        print(f"[BayLock][{self.robot_id}] Released idle bay {self._idle_bay_lock_id}")
                    except Exception:
                        pass
            except SimulationDataServiceError as e:
                self.logger.error(f"Failed to release bay lock {self._idle_bay_lock_id}: {e}")
            finally:
                self._idle_bay_lock_id = None
                self._reached_idle_bay = False  # Reset the reached flag
        
        self._current_task = None
        self._update_operational_status(OperationalStatus.IDLE)
        self._task_phase = TaskPhase.COMPLETED
        self._current_path = None
        self._task_start_time = None
        self._phase_start_time = None
        self._stall_reason = None
        self._stall_start_time = None
        self._stall_retry_count = 0
        self._stall_retry_start_time = None
        
        # Clean up asynchronous planning state
        self._planned_route = None
        self._planning_error = None
        self._planning_target = None
        # Note: _planning_thread will be cleaned up by the thread itself

        # Reset shelf orientation state
        self._shelf_orientation_pending = False
        self._orientation_start_time = None
    
    def _add_to_history(self, task: Task) -> None:
        """Internal: Add task to history with size limit."""
        self._task_history.append(task)
        if len(self._task_history) > self._max_history_size:
            self._task_history.pop(0)
    
    def _calculate_task_progress(self) -> float:
        """Internal: Calculate current task progress."""
        if not self._current_task:
            return 0.0
        
        if self._current_task.task_type == TaskType.PICK_AND_DELIVER:
            phase_weights = {
                TaskPhase.PLANNING: 0.1,
                TaskPhase.NAVIGATING_TO_SHELF: 0.3,
                TaskPhase.PICKING_ITEM: 0.2,
                TaskPhase.NAVIGATING_TO_DROPOFF: 0.3,
                TaskPhase.DROPPING_ITEM: 0.1
            }
        elif self._current_task.task_type == TaskType.MOVE_TO_CHARGING:
            phase_weights = {
                TaskPhase.PLANNING: 0.1,
                TaskPhase.NAVIGATING_TO_CHARGING: 0.4,
                TaskPhase.CHARGING_BATTERY: 0.5
            }
        elif self._current_task.task_type == TaskType.IDLE_PARK:
            # Progress model depends on sub-mode: wandering vs navigating to idle bay
            if self._task_phase == TaskPhase.WANDERING or getattr(self, '_is_wandering', False):
                phase_weights = {
                    TaskPhase.PLANNING: 0.1,
                    TaskPhase.WANDERING: 0.9
                }
            else:
                phase_weights = {
                    TaskPhase.PLANNING: 0.1,
                    TaskPhase.NAVIGATING_TO_IDLE: 0.9
                }
        else:
            phase_weights = {
                TaskPhase.PLANNING: 0.1,
                TaskPhase.NAVIGATING_TO_SHELF: 0.9
            }
        
        # Calculate cumulative progress up to current phase using explicit ordering
        current_phase_index = TASK_PHASE_ORDER.index(self._task_phase) if self._task_phase in TASK_PHASE_ORDER else 0
        total_progress = 0.0
        
        for phase, weight in phase_weights.items():
            phase_index = TASK_PHASE_ORDER.index(phase) if phase in TASK_PHASE_ORDER else 0
            if phase_index < current_phase_index:
                total_progress += weight
            elif phase == self._task_phase:
                # Add partial progress for current phase
                phase_progress = self._get_current_phase_progress()
                total_progress += weight * phase_progress
                break
        
        return min(1.0, total_progress)
    
    def _get_current_phase_progress(self) -> float:
        """Internal: Get progress within current phase."""
        if self._task_phase in [TaskPhase.PICKING_ITEM, TaskPhase.DROPPING_ITEM]:
            if self._phase_start_time:
                elapsed = time.time() - self._phase_start_time
                duration = (self._picking_duration if self._task_phase == TaskPhase.PICKING_ITEM 
                           else self._dropping_duration)
                return min(1.0, elapsed / duration)
        
        elif self._task_phase in [
            TaskPhase.NAVIGATING_TO_SHELF,
            TaskPhase.NAVIGATING_TO_DROPOFF,
            TaskPhase.NAVIGATING_TO_CHARGING,
            TaskPhase.NAVIGATING_TO_IDLE,
            TaskPhase.WANDERING
        ]:
            # Special case: If we've transitioned to charging but still in NAVIGATING_TO_CHARGING phase
            # This can happen due to timing between task handler and lane follower state updates
            if (self._task_phase == TaskPhase.NAVIGATING_TO_CHARGING and
                self._operational_status == OperationalStatus.CHARGING):
                return 1.0  # Navigation complete, charging has started

            # Additional safety: If we're in NAVIGATING_TO_CHARGING and battery manager is charging,
            # we've likely reached the destination
            if (self._task_phase == TaskPhase.NAVIGATING_TO_CHARGING and
                self.battery_manager and self.battery_manager.is_charging()):
                return 1.0  # Navigation complete, charging has started

            # Calculate navigation progress based on lane following
            lane_result = self.lane_follower.get_lane_following_result()
            if lane_result and getattr(lane_result, 'success', False):
                # Use segment progress for more accurate progress calculation
                current_route = self.lane_follower.get_current_route()
                total_segments = len(current_route.segments) if current_route else 1
                segment_progress = getattr(lane_result, 'progress_in_segment', 0.0)
                completed_segments = getattr(lane_result, 'current_segment_index', 0)
                return min(0.95, (completed_segments + segment_progress) / total_segments)
            else:
                try:
                    # Diagnostics: log when we fall back to 0.1 (no progress) during navigation
                    lane_status = self.lane_follower.get_lane_following_status()
                    lr_status = getattr(lane_result, 'status', None)
                    lr_prog = getattr(lane_result, 'progress_in_segment', None) if lane_result else None
                    lr_idx = getattr(lane_result, 'current_segment_index', None) if lane_result else None
                    #print(f"[diag][TaskHandler] nav phase progress fallback 0.1 | phase={self._task_phase.value} | lane_status={getattr(lane_status, 'value', lane_status)} | lr.status={getattr(lr_status, 'value', lr_status)} | seg_idx={lr_idx} | seg_prog={lr_prog}")
                except Exception:
                    pass
                return 0.1  # Just started or error
        
        return 0.0
    
    def _validate_task(self, task: Task) -> bool:
        """Internal: Validate task parameters."""
        if not task.task_id or not task.task_type:
            return False
        
        if task.task_type == TaskType.PICK_AND_DELIVER and not task.shelf_id:
            return False
        
        if task.task_type == TaskType.MOVE_TO_POSITION and not task.target_position:
            return False
        
        return True
    
    def _get_status_for_phase(self, phase: TaskPhase) -> OperationalStatus:
        """Internal: Get operational status for task phase."""
        phase_status_map = {
            TaskPhase.PLANNING: OperationalStatus.IDLE,
            TaskPhase.NAVIGATING_TO_SHELF: OperationalStatus.MOVING_TO_SHELF,
            TaskPhase.PICKING_ITEM: OperationalStatus.PICKING,
            TaskPhase.NAVIGATING_TO_DROPOFF: OperationalStatus.MOVING_TO_DROPOFF,
            TaskPhase.DROPPING_ITEM: OperationalStatus.DROPPING,
            TaskPhase.NAVIGATING_TO_CHARGING: OperationalStatus.MOVING_TO_CHARGING,
            TaskPhase.CHARGING_BATTERY: OperationalStatus.CHARGING,
            TaskPhase.COMPLETED: OperationalStatus.IDLE,
            TaskPhase.NAVIGATING_TO_IDLE: OperationalStatus.MOVING_TO_IDLE
        }
        return phase_status_map.get(phase, OperationalStatus.IDLE)
    
    # SimulationDataService integration methods
    def _get_shelf_position(self, shelf_id: str) -> Tuple[float, float]:
        """Internal: Get shelf position from SimulationDataService."""
        try:
            position = self.simulation_data_service.get_shelf_position(shelf_id)
            if position:
                return position
            else:
                self.logger.warning(f"Shelf {shelf_id} not found, using default position")
                return (3.0, 3.0)  # Fallback position
        except SimulationDataServiceError as e:
            self.logger.error(f"Failed to get shelf position for {shelf_id}: {e}")
            return (3.0, 3.0)  # Fallback position
    
    def _get_dropoff_position(self) -> Tuple[float, float]:
        """Internal: Get dropoff position from SimulationDataService."""
        try:
            dropoff_zones = self.simulation_data_service.get_dropoff_zones()
            if dropoff_zones:
                return dropoff_zones[0]  # Use first available dropoff zone
            else:
                self.logger.warning("No dropoff zones found, using default position")
                return (8.0, 8.0)  # Fallback position
        except SimulationDataServiceError as e:
            self.logger.error(f"Failed to get dropoff zones: {e}")
            return (8.0, 8.0)  # Fallback position

    def _handle_waiting_for_charging(self) -> None:
        """
        Internal: Navigate to idle zone and wait until a charging station becomes available.

        - Creates an IDLE_PARK task to move to an idle zone
        - Sets waiting flags to suppress duplicate charging triggers
        - Periodically re-attempts allocation in _check_automatic_charging
        
        Future enhancements (design notes):
        - Replace polling with event-driven allocation: the ChargingStationManager can
          notify waiting robots when a station is released (observer/callback or thread-safe queue).
        - Add a priority queue (e.g., by battery level and wait time) to improve fairness
          across multiple robots, while preserving lock safety.
        """
        try:
            # If already waiting, do nothing
            if self._waiting_for_charging:
                return

            # Create an IDLE_PARK task to move to idle zone while waiting
            idle_task = Task.create_idle_park_task(
                task_id=f"wait_idle_{self.robot_id}_{int(time.time())}",
                robot_id=self.robot_id,
                bay_id="auto_assigned"
            )

            # Start the idle task
            if self.start_task(idle_task):
                self._waiting_for_charging = True
                self._waiting_for_charging_since = time.time()
                self._last_charging_recheck = 0.0
                self._charging_task_pending = True  # Prevent duplicate auto-charging tasks
                self.logger.info("No charging station available - waiting at idle zone and queuing for availability")
            else:
                # If we cannot start idle task, at least mark waiting to suppress duplicates
                self._waiting_for_charging = True
                self._waiting_for_charging_since = time.time()
                self._last_charging_recheck = 0.0
                self._charging_task_pending = True
                self.logger.warning("Failed to start idle task while waiting for charging; will re-attempt allocation periodically")
        except Exception as e:
            self.logger.error(f"Error entering waiting-for-charging state: {e}")

    def _start_charging_task_with_preallocated(self, assignment: 'ChargingStationAssignment', current_battery: float) -> None:
        """
        Internal: Start a MOVE_TO_CHARGING task with a preallocated target.
        Resets waiting flags and sets locked bay ID.
        """
        try:
            # Set the locked bay ID immediately since we have a confirmed assignment
            self._locked_bay_id = assignment.station.station_id
            self._bay_lock_last_heartbeat = time.time()

            charging_task = Task(
                task_id=f"auto_charge_{self.robot_id}_{int(time.time())}",
                task_type=TaskType.MOVE_TO_CHARGING,
                priority=Priority.HIGH,
                bay_id=assignment.station.station_id,  # Use actual station ID
                metadata={
                    "auto_generated": True,
                    "battery_level": current_battery,
                    "trigger_reason": "low_battery_auto_trigger",
                    "trigger_threshold": self._charging_trigger_threshold,
                    "preallocated_position": assignment.station.position,
                    "station_id": assignment.station.station_id  # Include station ID in metadata
                }
            )

            if self.start_task(charging_task):
                self._charging_task_pending = True
                self._waiting_for_charging = False
                self._waiting_for_charging_since = 0.0
                self._last_charging_recheck = 0.0
                self.logger.info(
                    f"Charging station available - proceeding to charge (battery: {current_battery:.1%})"
                )
            else:
                self.logger.warning("Failed to start charging task after allocation; will remain waiting")
        except Exception as e:
            self.logger.error(f"Error starting charging task with preallocated target: {e}")
    
    def _get_charging_station_position(self) -> Tuple[float, float]:
        """
        Internal: Get optimal charging station position using charging station manager.

        Uses the new charging station manager if available, otherwise falls back
        to the legacy method using map data.
        """
        # If charging task carries a preallocated position, use it directly
        try:
            if self._current_task and self._current_task.metadata:
                prealloc = self._current_task.metadata.get("preallocated_position")
                station_id = self._current_task.metadata.get("station_id")
                if prealloc and station_id:
                    # Set locked bay ID for status reporting
                    self._locked_bay_id = station_id
                    self._bay_lock_last_heartbeat = time.time()
                    return tuple(prealloc)
        except Exception:
            pass

        # If we already have a locked charging bay, route to it without reallocating
        try:
            if self._locked_bay_id and isinstance(self._locked_bay_id, str) and self._locked_bay_id.startswith('charge_'):
                parts = self._locked_bay_id.split('_')
                row = int(parts[1]); col = int(parts[2])
                world = self.coordinate_system.cell_to_world(Cell(x=col, y=row))
                return (world.x, world.y) if hasattr(world, 'x') else world
        except Exception:
            pass

        # Use charging station manager if available
        if self.charging_station_manager:
            try:
                # Get current robot position
                robot_state = self.state_holder.get_robot_state()
                robot_position = (robot_state.position[0], robot_state.position[1])

                # Request charging station allocation
                request = ChargingStationRequest(
                    robot_id=self.robot_id,
                    robot_position=robot_position,
                    timestamp=time.time()
                )
                print(f"TaskHandler _get_charging_station_position request: {request}") # TODO: remove

                assignment = self.charging_station_manager.request_charging_station(request)

                if assignment:
                    # Set the locked bay ID for status reporting
                    self._locked_bay_id = assignment.station.station_id
                    self._bay_lock_last_heartbeat = time.time()
                    self.logger.info(f"Allocated charging station {assignment.station.station_id} at {assignment.station.position}")
                    return assignment.station.position
                else:
                    self.logger.warning("No available charging stations found (planning)")
                    # Return current position as no-op to avoid navigating to occupied stations
                    rs = self.state_holder.get_robot_state()
                    return (rs.position[0], rs.position[1])

            except Exception as e:
                self.logger.error(f"Error using charging station manager: {e}")
                return self._get_fallback_charging_position()
        else:
            # Fallback to legacy method
            return self._get_fallback_charging_position()

    def _get_fallback_charging_position(self) -> Tuple[float, float]:
        """Fallback method to get charging station position from map data."""
        try:
            # Get charging stations from simulation data service
            charging_stations = self.simulation_data_service.list_charging_bays()

            if charging_stations:
                # Return the first available charging station position
                # TODO: Could implement distance-based selection here
                return charging_stations[0][1]  # (bay_id, position) tuple

            # If no stations from service, try map data
            return (8.0, 8.0)  # Default position

        except Exception as e:
            self.logger.error(f"Failed to get charging station position: {e}")
            return (8.0, 8.0)  # Fallback position
    
    def _get_idle_zone_position(self) -> Tuple[float, float]:
        """Internal: Get optimal idle zone position from SimulationDataService."""
        try:
            current_state = self.state_holder.get_robot_state()
            current_position = current_state.position[:2]  # (x, y)
            
            # Get optimal idle zone from simulation data service
            optimal_zone = self.simulation_data_service.get_optimal_idle_zone(
                current_position, self.robot_id
            )
            
            if optimal_zone:
                print(f"[TaskHandler] Selected idle zone: {optimal_zone}")
                return optimal_zone
            else:
                # Fallback: use first available idle zone
                idle_zones = self.simulation_data_service.get_idle_zones()
                if idle_zones:
                    fallback_zone = idle_zones[0]
                    print(f"[TaskHandler] Using fallback idle zone: {fallback_zone}")
                    return fallback_zone
                else:
                    # Last resort: use a safe default position
                    print(f"[TaskHandler] No idle zones available, using default position")
                    return (1.0, 1.0)
                    
        except Exception as e:
            self.logger.error(f"Failed to get idle zone position: {e}")
            # Return a safe fallback position
            return (1.0, 1.0)

    def _request_and_wait_for_idle_bay(self) -> Tuple[float, float]:
        """Request nearest available idle bay with periodic retry; returns bay position.

        Non-blocking: if no bay is available, returns current position so the planner
        doesn't move, and the next planning cycle will retry.
        """
        current_xy = self.state_holder.get_robot_state().position[:2]

        # Heartbeat held bay lock
        now = time.time()
        if self._idle_bay_lock_id and now - self._bay_lock_last_heartbeat > self._bay_lock_heartbeat_interval:
            try:
                if self.simulation_data_service.heartbeat_bay_lock(self._idle_bay_lock_id, self.robot_id):
                    self._bay_lock_last_heartbeat = now
                    try:
                        print(f"[BayLock][{self.robot_id}] Heartbeat idle bay {self._idle_bay_lock_id}")
                    except Exception:
                        pass
            except SimulationDataServiceError:
                pass

        # If already locked, return that bay's position (and refresh heartbeat opportunistically)
        if self._idle_bay_lock_id:
            try:
                parts = self._idle_bay_lock_id.split('_')
                row = int(parts[1]); col = int(parts[2])
                world = self.coordinate_system.cell_to_world(Cell(x=col, y=row))
                # Opportunistic heartbeat to keep lock fresh
                try:
                    self.simulation_data_service.heartbeat_bay_lock(self._idle_bay_lock_id, self.robot_id)
                    self._bay_lock_last_heartbeat = time.time()
                except Exception:
                    pass
                return (world.x, world.y) if hasattr(world, 'x') else world
            except Exception:
                idle_bays = self.simulation_data_service.list_idle_bays()
                for bid, pos in idle_bays:
                    if bid == self._idle_bay_lock_id:
                        return pos
                return self.state_holder.get_robot_state().position[:2]

        # Acquire nearest available idle bay
        try:
            state = self.state_holder.get_robot_state()
            current_xy = state.position[:2]
            candidates = self.simulation_data_service.list_idle_bays()
            if not candidates:
                return current_xy
            candidates.sort(key=lambda item: (item[1][0]-current_xy[0])**2 + (item[1][1]-current_xy[1])**2)
            for bay_id, pos in candidates:
                try:
                    print(f"[BayLock][{self.robot_id}] Trying idle bay {bay_id} at {pos}")
                except Exception:
                    pass
                if self.simulation_data_service.try_acquire_bay_lock(bay_id, self.robot_id):
                    self._idle_bay_lock_id = bay_id
                    self._reached_idle_bay = False  # Reset flag for new bay acquisition
                    self._bay_lock_last_heartbeat = time.time()
                    try:
                        print(f"[BayLock][{self.robot_id}] Acquired idle bay {bay_id} at {pos}")
                    except Exception:
                        pass
                    return pos
            try:
                print(f"[BayLock][{self.robot_id}] No idle bay available; waiting")
            except Exception:
                pass
            return current_xy
        except SimulationDataServiceError as e:
            self.logger.error(f"Idle bay acquisition failed: {e}")
            return self.state_holder.get_robot_state().position[:2]
    
    def _unlock_current_shelf(self) -> None:
        """Internal: No-op. Physical shelf locks are removed in reservation model."""
        self._locked_shelf_id = None
        print ("WARNING: NO-OP NETHOD been called _unlock_current_shelf")
    
    def _log_kpi_event(self, event_type: str, event_data: dict) -> None:
        """Internal: Log KPI event to SimulationDataService."""
        try:
            self.simulation_data_service.log_event(
                event_type=event_type,
                robot_id=self.robot_id,
                event_data=event_data
            )
        except SimulationDataServiceError as e:
            self.logger.error(f"Failed to log KPI event {event_type}: {e}")
            # Don't fail the task for logging errors

    def _update_operational_status(self, new_status: OperationalStatus) -> None:
        """
        Internal: Update operational status and emit KPI status change with duration of previous state.
        """
        if new_status == self._operational_status:
            return
        now = time.time()
        try:
            if self._status_change_time is not None:
                duration = max(0.0, now - self._status_change_time)
                # Emit busy/idle style durations via generic status_change metric
                prev_status = self._operational_status
                is_busy_prev = self._is_busy_status(prev_status)
                self._log_kpi_event("status_change", {
                    "prev_status_code": float(self._status_to_code(prev_status)),
                    "new_status_code": float(self._status_to_code(new_status)),
                    "status_duration_seconds": duration,
                    "busy_duration_seconds": duration if is_busy_prev else 0.0,
                    "idle_duration_seconds": 0.0 if is_busy_prev else duration,
                })
        except Exception:
            pass

        self._operational_status = new_status
        self._status_change_time = now

    def _status_to_code(self, status: OperationalStatus) -> int:
        """Map OperationalStatus to a stable numeric code for KPI storage."""
        ordering = [
            OperationalStatus.IDLE,
            OperationalStatus.MOVING_TO_SHELF,
            OperationalStatus.APPROACHING_SHELF,
            OperationalStatus.PICKING,
            OperationalStatus.MOVING_TO_DROPOFF,
            OperationalStatus.DROPPING,
            OperationalStatus.MOVING_TO_CHARGING,
            OperationalStatus.CHARGING,
            OperationalStatus.MOVING_TO_IDLE,
            OperationalStatus.WANDERING,
            OperationalStatus.STALLED,
            OperationalStatus.ERROR,
            OperationalStatus.EMERGENCY_STOP,
            OperationalStatus.APPROACHING_BAY,
            OperationalStatus.ENTERING_BAY,
            OperationalStatus.IN_BAY,
            OperationalStatus.EXITING_BAY,
        ]
        try:
            return ordering.index(status)
        except ValueError:
            return 0

    def _is_busy_status(self, status: OperationalStatus) -> bool:
        """Classify whether a status is considered busy for KPI purposes."""
        return status in {
            OperationalStatus.MOVING_TO_SHELF,
            OperationalStatus.APPROACHING_SHELF,
            OperationalStatus.PICKING,
            OperationalStatus.MOVING_TO_DROPOFF,
            OperationalStatus.DROPPING,
            OperationalStatus.MOVING_TO_CHARGING,
            OperationalStatus.CHARGING,
            OperationalStatus.MOVING_TO_IDLE,
            OperationalStatus.APPROACHING_BAY,
            OperationalStatus.ENTERING_BAY,
            OperationalStatus.IN_BAY,
            OperationalStatus.EXITING_BAY,
        }
    
    # --- Wander candidate selection helpers ---
    def _get_wander_candidates(self) -> List[Tuple[float, float]]:
        """Build or return cached list of safe wander targets in world coordinates.

        Safe targets are navigation graph nodes that are:
        - Regular lanes (node_id starts with 'lane_')
        - Not conflict boxes (node.is_conflict_box == False)
        - Not special zones (idle/charge/dropoff)
        """
        if self._wander_candidates is not None:
            return self._wander_candidates

        candidates: List[Tuple[float, float]] = []
        try:
            graph = self.simulation_data_service.get_navigation_graph()
            for node_id, node in getattr(graph, 'nodes', {}).items():
                # Exclude special zones and conflict nodes
                if not isinstance(node_id, str):
                    continue
                if node_id.startswith('lane_') and not getattr(node, 'is_conflict_box', False):
                    # Sanity: exclude parking-adjacent single-cell conflict boxes (already covered by is_conflict_box)
                    # Also implicitly excludes 'idle_', 'charge_', 'dropoff_' since we filter by 'lane_'
                    pos = getattr(node, 'position', None)
                    if pos is not None and hasattr(pos, 'x') and hasattr(pos, 'y'):
                        candidates.append((float(pos.x), float(pos.y)))
        except Exception as e:
            self.logger.warning(f"[Wander] Failed to load navigation graph for candidates: {e}")

        # Cache even if empty to avoid repeated DB hits; upstream logic should handle empty
        self._wander_candidates = candidates
        return candidates

    def _select_wander_target(self, current_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Select a wander target far enough from current position, or None if unavailable."""
        candidates = self._get_wander_candidates()
        if not candidates:
            return None

        # Prefer targets beyond the minimum distance to avoid dithering
        min_dist = max(0.0, getattr(self, '_wander_min_target_distance', 2.0))
        far_enough: List[Tuple[float, float]] = []
        cx, cy = current_xy
        for (tx, ty) in candidates:
            dx = tx - cx
            dy = ty - cy
            if (dx * dx + dy * dy) ** 0.5 >= min_dist:
                far_enough.append((tx, ty))

        import random
        pool = far_enough if far_enough else candidates
        return random.choice(pool) if pool else None

    def _async_path_planning(self, start: Tuple[float, float], target: Tuple[float, float], nav_task_type: Optional[NavTaskType] = None) -> None:
        """
        Internal: Asynchronous route planning that runs outside the critical section.
        This prevents the expensive path_planner.plan_route() call from blocking 
        the 10Hz control loop and other reader threads.
        """
        try:
            # Convert tuple to Point objects for lane-based planning
            from interfaces.navigation_types import Point, TaskType as NavTaskType
            
            start_point = Point(x=start[0], y=start[1])
            target_point = Point(x=target[0], y=target[1])
            
            # This runs outside the lock, allowing other operations to continue
            planning_result = self.path_planner.plan_route(
                start=start_point, 
                goal=target_point, 
                task_type=(nav_task_type or NavTaskType.PICK_AND_DELIVER)
            )
            
            # Atomically store the result
            with WriteLock(self._status_lock):
                if self._planning_target == target:  # Ensure this is still the current planning request
                    if planning_result.success and planning_result.route:
                        self._planned_route = planning_result.route
                        self._planning_error = None
                        self.logger.debug(f"Route planning completed: {len(planning_result.route.segments)} segments")
                    else:
                        self._planned_route = None
                        self._planning_error = planning_result.error_message
                        self.logger.error(f"Route planning failed: {planning_result.error_message}")
                
        except Exception as e:
            # Atomically store the error
            with WriteLock(self._status_lock):
                if self._planning_target == target:  # Ensure this is still the current planning request
                    self._planned_route = None
                    self._planning_error = str(e)
                    self.logger.error(f"Route planning failed: {e}")
        
        finally:
            # Clear the planning thread reference
            with WriteLock(self._status_lock):
                if self._planning_target == target:
                    self._planning_thread = None

    def _is_close_to_target_shelf(self) -> bool:
        """
        Check if robot is close enough to target shelf to enable relaxed safety margins.

        Returns True when robot is within APPROACHING_SHELF_DISTANCE of target shelf.
        This allows collision avoidance to use relaxed safety margins for intentional
        shelf approaches while maintaining safety for the rest of the journey.
        """
        if not self._current_task or not hasattr(self._current_task, 'pickup_location'):
            return False

        try:
            # Get current robot position
            current_pos = self.state_holder.get_position()
            if not current_pos:
                return False

            # Get target shelf position
            target_pos = self._current_task.pickup_location
            if not target_pos:
                return False

            # Calculate distance to target shelf
            distance = math.sqrt(
                (current_pos[0] - target_pos[0])**2 +
                (current_pos[1] - target_pos[1])**2
            )

            # Define distance threshold for "approaching shelf"
            APPROACHING_SHELF_DISTANCE = 2.0  # 2 meters - transition to APPROACHING_SHELF status

            return distance <= APPROACHING_SHELF_DISTANCE

        except Exception as e:
            # If position check fails, err on side of caution
            print(f"[TaskHandler] Error checking shelf proximity: {e}")
            return False