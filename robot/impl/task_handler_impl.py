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
from interfaces.state_holder_interface import IStateHolder
from interfaces.path_planner_interface import IPathPlanner, Path, Cell, PathPlanningError
from interfaces.motion_executor_interface import IMotionExecutor, MotionStatus, MotionExecutionError
from interfaces.coordinate_system_interface import ICoordinateSystem
from interfaces.simulation_data_service_interface import ISimulationDataService, SimulationDataServiceError
from interfaces.lane_follower_interface import ILaneFollower, LaneFollowingStatus, LaneFollowingError
from interfaces.navigation_types import Route
from interfaces.configuration_interface import IConfigurationProvider


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
                 config_provider: Optional[IConfigurationProvider] = None):
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
        """
        self.state_holder = state_holder
        self.path_planner = path_planner
        self.lane_follower = lane_follower
        self.motion_executor = motion_executor
        self.coordinate_system = coordinate_system
        self.simulation_data_service = simulation_data_service
        self.robot_id = robot_id
        self.config_provider = config_provider
        
        # Setup logging with robot ID prefix
        self.logger = logging.getLogger(f"TaskHandler.{robot_id}")
        
        # Thread safety - Reader-writer lock for better concurrency
        self._status_lock = ReadWriteLock()
        
        # Task state
        self._current_task: Optional[Task] = None
        self._operational_status = OperationalStatus.IDLE
        self._task_phase = TaskPhase.COMPLETED
        self._current_path: Optional[Path] = None
        self._task_start_time: Optional[float] = None
        
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
            self._charging_threshold = robot_config.charging_threshold
            self._task_timeout = task_config.task_timeout
            self._retry_attempts = task_config.retry_attempts
            self._retry_delay = task_config.retry_delay
        else:
            # Default values
            self._stall_recovery_timeout = 10.0  # seconds
            # position_tolerance removed - Motion Executor is single source of truth
            self._picking_duration = 3.0   # seconds to simulate picking
            self._dropping_duration = 2.0  # seconds to simulate dropping
            self._charging_threshold = 0.2  # charge when below 20%
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
        self._locked_shelf_id: Optional[str] = None
        # Bay locking state
        self._locked_bay_id: Optional[str] = None
        self._bay_lock_last_heartbeat: float = 0.0
        self._bay_lock_heartbeat_interval: float = 5.0
    
    def update_config_from_robot(self, robot_config) -> None:
        """Update task parameters with robot-specific configuration."""
        if robot_config:
            self._stall_recovery_timeout = robot_config.stall_recovery_timeout
            # position_tolerance removed - Motion Executor is single source of truth
            self._picking_duration = robot_config.picking_duration
            self._dropping_duration = robot_config.dropping_duration
            self._charging_threshold = robot_config.charging_threshold
    
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
                stall_reason=self._stall_reason
            )
    
    def update_task_execution(self) -> None:
        """
        Update task execution (called at 10Hz from control thread).
        **CONTROL THREAD ONLY**: Must be called from control thread only.
        
        Handles task lifecycle, waypoint progression, and motion coordination.
        """
        with WriteLock(self._status_lock):
            if not self._current_task:
                return
            
            # Handle stall recovery
            if self._operational_status == OperationalStatus.STALLED:
                self._handle_stall_recovery()
                return
            
            # Handle emergency stop
            if self._operational_status == OperationalStatus.EMERGENCY_STOP:
                return
            
            # Update lane following execution
            self.lane_follower.update_lane_following()
            
            # Update task execution based on current phase
            self._update_current_phase()
    
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
            
            # Reset to idle state
            self._reset_task_state()
    
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
            if not self.is_idle():
                self.logger.info(f"Robot busy, cannot accept task {task.task_id}")
                return False
            
            # Reset motion executor from emergency stop if needed
            motion_status = self.motion_executor.get_motion_status()
            if motion_status == MotionStatus.ERROR:
                self.motion_executor.stop_execution()  # Reset from emergency stop
            
            # Validate task
            if not self._validate_task(task):
                self.logger.warning(f"Invalid task {task.task_id}")
                return False
            
            # Start task execution
            self._current_task = task
            self._current_task.status = TaskStatus.IN_PROGRESS
            self._current_task.assigned_at = datetime.now()
            self._task_start_time = time.time()
            self._phase_start_time = time.time()
            
            # Set initial phase and status
            if task.task_type == TaskType.PICK_AND_DELIVER:
                self._task_phase = TaskPhase.PLANNING
                self._operational_status = OperationalStatus.MOVING_TO_SHELF
            elif task.task_type == TaskType.MOVE_TO_CHARGING:
                self._task_phase = TaskPhase.PLANNING
                self._operational_status = OperationalStatus.MOVING_TO_CHARGING
            elif task.task_type == TaskType.MOVE_TO_POSITION:
                self._task_phase = TaskPhase.PLANNING
                self._operational_status = OperationalStatus.MOVING_TO_SHELF  # Generic movement
            elif task.task_type == TaskType.IDLE_PARK:
                # Explicitly mark as moving to idle so status reflects intent during planning
                self._task_phase = TaskPhase.PLANNING
                self._operational_status = OperationalStatus.MOVING_TO_IDLE
            
            # Log task start event
            self._log_kpi_event("task_start", {
                "task_id": task.task_id,
                "task_type": task.task_type.value,
                "order_id": task.order_id,
                "shelf_id": task.shelf_id,
                "item_id": task.item_id,
                "quantity": task.quantity_to_pick
            })
            
            print(f"[TaskHandler] Started task {task.task_id}: {task.task_type.value}")
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
            
            self._operational_status = OperationalStatus.STALLED
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
            
            # Clean up
            self._add_to_history(self._current_task)
            self._reset_task_state()
    
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
                elif self._current_task.task_type == TaskType.MOVE_TO_POSITION:
                    self._advance_to_phase(TaskPhase.NAVIGATING_TO_SHELF, OperationalStatus.MOVING_TO_SHELF)
                elif self._current_task.task_type == TaskType.IDLE_PARK:
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
                    target_position = self._request_and_wait_for_idle_bay()
                
                else:
                    raise TaskHandlingError(f"Unsupported task type: {self._current_task.task_type}")
                
                # Start asynchronous path planning
                self._planning_target = target_position[:2]
                print(f"[TaskHandler] Starting path planning (robot={self.robot_id}): {current_position[:2]} -> {target_position[:2]}")
                self._planning_thread = threading.Thread(
                    target=self._async_path_planning,
                    args=(current_position[:2], target_position[:2]),
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
            
        # Check lane following status
        lane_status = self.lane_follower.get_lane_following_status()
        lane_result = self.lane_follower.get_lane_following_result()
        
        if lane_status == LaneFollowingStatus.COMPLETED:
            # Navigation completed
            if expected_status == OperationalStatus.MOVING_TO_SHELF:
                if self._current_task.task_type == TaskType.PICK_AND_DELIVER:
                    self._advance_to_phase(TaskPhase.PICKING_ITEM, OperationalStatus.PICKING)
                else:
                    # Simple move to position task completed
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
                self._complete_task()
                
                # Reset lane follower status to IDLE so it can start new routes
                self.lane_follower.stop_following()
        
        elif lane_status == LaneFollowingStatus.ERROR:
            self._handle_task_error(f"Lane following error: {lane_result.error_message}")
        
        elif lane_status == LaneFollowingStatus.BLOCKED:
            self.handle_stall_event("Path blocked during lane following")
    
    def _handle_picking_phase(self) -> None:
        """Internal: Handle item picking phase with shelf locking and inventory management."""
        if not self._current_task:
            return
            
        # Initialize picking phase
        if not self._phase_start_time:
            self._phase_start_time = time.time()
            
            # Lock shelf for exclusive access
            if self._current_task.shelf_id and not self._locked_shelf_id:
                try:
                    lock_success = self.simulation_data_service.lock_shelf(
                        self._current_task.shelf_id, 
                        self.robot_id
                    )
                    
                    if lock_success:
                        self._locked_shelf_id = self._current_task.shelf_id
                        self.logger.info(f"Locked shelf {self._current_task.shelf_id} for picking")
                        
                        # Log shelf lock event
                        self._log_kpi_event("shelf_locked", {
                            "shelf_id": self._current_task.shelf_id,
                            "operation": "pick"
                        })
                    else:
                        # Shelf already locked by another robot
                        self._handle_task_error(f"Could not lock shelf {self._current_task.shelf_id} - already in use")
                        return
                        
                except SimulationDataServiceError as e:
                    self._handle_task_error(f"Shelf locking failed: {e}")
                    return
        
        elapsed_time = time.time() - self._phase_start_time
        
        if elapsed_time >= self._picking_duration:
            # Picking operation completed - update inventory
            if self._current_task.item_id and self._current_task.shelf_id:
                try:
                    # Update inventory (remove picked items)
                    inventory_updated = self.simulation_data_service.update_inventory(
                        shelf_id=self._current_task.shelf_id,
                        item_id=self._current_task.item_id,
                        operation='remove',
                        quantity=self._current_task.quantity_to_pick
                    )
                    
                    if inventory_updated:
                        self.logger.info(f"Picked {self._current_task.quantity_to_pick} of {self._current_task.item_id} from shelf {self._current_task.shelf_id}")
                        
                        # Log pick operation event
                        self._log_kpi_event("pick_operation", {
                            "shelf_id": self._current_task.shelf_id,
                            "item_id": self._current_task.item_id,
                            "quantity": self._current_task.quantity_to_pick,
                            "success": True
                        })
                    else:
                        self.logger.warning(f"Inventory update failed for {self._current_task.item_id} on shelf {self._current_task.shelf_id}")
                        
                        # Log failed pick operation
                        self._log_kpi_event("pick_operation", {
                            "shelf_id": self._current_task.shelf_id,
                            "item_id": self._current_task.item_id,
                            "quantity": self._current_task.quantity_to_pick,
                            "success": False,
                            "reason": "inventory_update_failed"
                        })
                        
                except SimulationDataServiceError as e:
                    self.logger.error(f"Failed to update inventory: {e}")
                    self._handle_task_error(f"Inventory update failed: {e}")
                    return
            
            # Unlock shelf after picking
            self._unlock_current_shelf()
            
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
        
        elapsed_time = time.time() - self._phase_start_time
        
        if elapsed_time >= self._dropping_duration:
            # Log drop operation event
            self._log_kpi_event("drop_operation", {
                "order_id": self._current_task.order_id,
                "item_id": self._current_task.item_id,
                "quantity": self._current_task.quantity_to_pick,
                "dropoff_zone": self._current_task.dropoff_zone,
                "success": True
            })
            
            # Dropping completed, task finished
            self._complete_task()
    
    def _handle_charging_phase(self) -> None:
        """Internal: Handle battery charging phase."""
        current_battery = self.state_holder.get_battery_level()
        
        # Simulate charging (would integrate with actual charging system)
        if current_battery >= 0.9:  # Charged to 90%
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
        self._operational_status = self._get_status_for_phase(self._task_phase)
        self._stall_reason = None
        self._stall_start_time = None
        self._stall_retry_start_time = None
        # Note: _stall_retry_count will be reset on next stall event
        
        print(f"[TaskHandler] Stall recovery successful: resuming task")
    
    def _advance_to_phase(self, new_phase: TaskPhase, new_status: OperationalStatus) -> None:
        """Internal: Advance to next task phase."""
        self._task_phase = new_phase
        self._operational_status = new_status
        self._phase_start_time = time.time()
        print(f"[TaskHandler] Advanced to phase: {new_phase.value}")
    
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
            "phase": self._task_phase.value
        })
        
        print(f"[TaskHandler] Task {self._current_task.task_id} completed successfully")
        
        # Add to history
        self._add_to_history(self._current_task)
        
        # Capture the completed task type before resetting state
        completed_task_type = self._current_task.task_type
        
        # Reset task state first so the robot is truly idle and ready for the next task
        # This prevents "robot busy" when starting the auto IDLE_PARK task
        self._reset_task_state()
        
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
            
            # Create a new IDLE_PARK task using the factory method
            idle_task = Task.create_idle_park_task(
                task_id=f"idle_park_{uuid.uuid4().hex[:8]}",
                robot_id=self.robot_id
            )
            
            print(f"[TaskHandler] Auto-creating IDLE_PARK task: {idle_task.task_id}")
            
            # Start the idle park task immediately
            if self.start_task(idle_task):
                print(f"[TaskHandler] Successfully started auto IDLE_PARK task")
            else:
                print(f"[TaskHandler] Failed to start auto IDLE_PARK task - robot busy")
                
        except Exception as e:
            print(f"[TaskHandler] Error creating auto IDLE_PARK task: {e}")
            # Don't fail the main task completion - this is just a convenience feature
    
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
                "phase": self._task_phase.value
            })
            
            print(f"[TaskHandler] Task {self._current_task.task_id} failed: {error_message}")
            self._add_to_history(self._current_task)
        
        # Clean up any locked shelves
        self._unlock_current_shelf()
        
        self._reset_task_state()
    
    def _reset_task_state(self) -> None:
        """Internal: Reset task state to idle with cleanup."""
        # CRITICAL: Stop motion executor and lane follower when task completes
        self.motion_executor.stop_execution()
        self.lane_follower.stop_following(force_release=False)  # Safe stop
        
        # Clean up shelf locks
        self._unlock_current_shelf()
        # Release bay lock if held
        if self._locked_bay_id:
            try:
                released = self.simulation_data_service.release_bay_lock(self._locked_bay_id, self.robot_id)
                if released:
                    self.logger.info(f"Released bay lock {self._locked_bay_id}")
                    try:
                        print(f"[BayLock][{self.robot_id}] Released idle bay {self._locked_bay_id}")
                    except Exception:
                        pass
            except SimulationDataServiceError as e:
                self.logger.error(f"Failed to release bay lock {self._locked_bay_id}: {e}")
            finally:
                self._locked_bay_id = None
        
        self._current_task = None
        self._operational_status = OperationalStatus.IDLE
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
            # Treat IDLE_PARK as a simple navigation task to idle
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
            TaskPhase.NAVIGATING_TO_IDLE
        ]:
            # Calculate navigation progress based on lane following
            lane_result = self.lane_follower.get_lane_following_result()
            if lane_result.success:
                # Use segment progress for more accurate progress calculation
                current_route = self.lane_follower.get_current_route()
                total_segments = len(current_route.segments) if current_route else 1
                segment_progress = lane_result.progress_in_segment
                completed_segments = lane_result.current_segment_index
                return min(0.95, (completed_segments + segment_progress) / total_segments)
            else:
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
    
    def _get_charging_station_position(self) -> Tuple[float, float]:
        """Internal: Get charging station position from SimulationDataService."""
        try:
            # TODO: Add get_charging_stations() method to SimulationDataService
            # For now, use map data to find charging stations
            map_data = self.simulation_data_service.get_map_data()
            # Look for charging stations in map data
            # This is a placeholder - actual implementation depends on map data structure
            return (0.0, 0.0)  # Fallback position
        except SimulationDataServiceError as e:
            self.logger.error(f"Failed to get charging station position: {e}")
            return (0.0, 0.0)  # Fallback position
    
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
                    return (10.0, 10.0)
                    
        except Exception as e:
            self.logger.error(f"Failed to get idle zone position: {e}")
            # Return a safe fallback position
            return (10.0, 10.0)

    def _request_and_wait_for_idle_bay(self) -> Tuple[float, float]:
        """Request nearest available idle bay with periodic retry; returns bay position.

        Non-blocking: if no bay is available, returns current position so the planner
        doesn't move, and the next planning cycle will retry.
        """
        # Heartbeat held bay lock
        now = time.time()
        if self._locked_bay_id and now - self._bay_lock_last_heartbeat > self._bay_lock_heartbeat_interval:
            try:
                if self.simulation_data_service.heartbeat_bay_lock(self._locked_bay_id, self.robot_id):
                    self._bay_lock_last_heartbeat = now
                    try:
                        print(f"[BayLock][{self.robot_id}] Heartbeat idle bay {self._locked_bay_id}")
                    except Exception:
                        pass
            except SimulationDataServiceError:
                pass

        # If already locked, return that bay's position
        if self._locked_bay_id:
            try:
                parts = self._locked_bay_id.split('_')
                row = int(parts[1]); col = int(parts[2])
                world = self.coordinate_system.cell_to_world(Cell(x=col, y=row))
                return (world.x, world.y) if hasattr(world, 'x') else world
            except Exception:
                idle_bays = self.simulation_data_service.list_idle_bays()
                for bid, pos in idle_bays:
                    if bid == self._locked_bay_id:
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
                    self._locked_bay_id = bay_id
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
        """Internal: Unlock currently locked shelf if any."""
        if self._locked_shelf_id:
            try:
                unlock_success = self.simulation_data_service.unlock_shelf(
                    self._locked_shelf_id, 
                    self.robot_id
                )
                
                if unlock_success:
                    self.logger.info(f"Unlocked shelf {self._locked_shelf_id}")
                    
                    # Log shelf unlock event
                    self._log_kpi_event("shelf_unlocked", {
                        "shelf_id": self._locked_shelf_id
                    })
                else:
                    self.logger.warning(f"Failed to unlock shelf {self._locked_shelf_id} - not owned by this robot")
                    
            except SimulationDataServiceError as e:
                self.logger.error(f"Failed to unlock shelf {self._locked_shelf_id}: {e}")
            finally:
                self._locked_shelf_id = None
    
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
    
    def _async_path_planning(self, start: Tuple[float, float], target: Tuple[float, float]) -> None:
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
                task_type=NavTaskType.PICK_AND_DELIVER
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