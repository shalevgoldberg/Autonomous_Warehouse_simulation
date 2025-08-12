"""
Interface for LaneFollower - executes lane-based navigation with conflict box management.

This interface defines the contract for lane following functionality, which is the bridge
between PathPlanner (route planning) and MotionExecutor (motion control).

Design Principles:
- **Single Responsibility**: Lane following and conflict box management only
- **Interface Segregation**: Focused, specific methods for lane navigation
- **Dependency Inversion**: Depends on abstractions (interfaces), not concretions
- **Open/Closed**: Extensible through interface implementation, closed for modification

Responsibilities:
- Execute lane-based routes segment by segment
- Manage conflict box locks and coordination
- Provide lane center-line following with tolerance
- Handle smooth lane transitions and bay approaches
- Coordinate with MotionExecutor for low-level motion control
"""
from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any
from dataclasses import dataclass
from enum import Enum

from .navigation_types import Route, RouteSegment, Point, LaneRec, BoxRec
from .motion_executor_interface import IMotionExecutor
from .simulation_data_service_interface import ISimulationDataService


class LaneFollowingStatus(Enum):
    """Lane following execution status."""
    IDLE = "idle"
    FOLLOWING_LANE = "following_lane"
    APPROACHING_CONFLICT_BOX = "approaching_conflict_box"
    IN_CONFLICT_BOX = "in_conflict_box"
    EXITING_CONFLICT_BOX = "exiting_conflict_box"
    APPROACHING_BAY = "approaching_bay"
    ENTERING_BAY = "entering_bay"
    IN_BAY = "in_bay"
    EXITING_BAY = "exiting_bay"
    COMPLETED = "completed"
    ERROR = "error"
    BLOCKED = "blocked"


class ConflictBoxStatus(Enum):
    """Conflict box lock status."""
    UNLOCKED = "unlocked"
    LOCKING = "locking"
    LOCKED = "locked"
    UNLOCKING = "unlocking"
    QUEUED = "queued"
    TIMEOUT = "timeout"
    ERROR = "error"


@dataclass
class LaneFollowingResult:
    """Result of lane following operation."""
    success: bool
    status: LaneFollowingStatus
    current_segment_index: int
    progress_in_segment: float  # 0.0 to 1.0
    error_message: Optional[str] = None
    conflict_box_status: Optional[ConflictBoxStatus] = None
    current_position: Optional[Point] = None
    target_position: Optional[Point] = None


@dataclass
class LaneFollowingConfig:
    """Configuration for lane following behavior."""
    # Lane following parameters
    lane_tolerance: float = 0.1  # Meters from center-line
    max_speed: float = 1.0  # Maximum speed in m/s
    corner_speed: float = 0.3  # Speed in conflict boxes and turns
    bay_approach_speed: float = 0.2  # Speed when approaching bays
    
    # Conflict box parameters
    lock_timeout: float = 30.0  # Seconds to wait for lock
    heartbeat_interval: float = 5.0  # Seconds between heartbeats
    lock_retry_attempts: int = 3  # Number of lock retry attempts
    
    # Control parameters
    # Note: position_tolerance removed - Motion Executor is single source of truth
    velocity_smoothing_factor: float = 0.1  # Low-pass filter factor
    emergency_stop_distance: float = 0.5  # Meters for emergency stops


class LaneFollowingError(Exception):
    """Base exception for lane following errors."""
    pass


class ConflictBoxLockError(LaneFollowingError):
    """Raised when conflict box lock acquisition fails."""
    pass


class LaneDeviationError(LaneFollowingError):
    """Raised when robot deviates too far from lane center-line."""
    pass


class ILaneFollower(ABC):
    """
    Interface for lane following functionality.
    
    Single Responsibility: Execute lane-based navigation with conflict box management.
    
    This interface bridges the gap between high-level route planning (PathPlanner)
    and low-level motion control (MotionExecutor). It handles the complex logic
    of following lane center-lines, managing conflict box locks, and coordinating
    smooth transitions between navigation states.
    
    **Thread Safety**: All methods are thread-safe.
    **Threading Model**:
    - follow_route(), stop_following(): CONTROL THREAD (called by TaskHandler)
    - update_lane_following(): CONTROL THREAD ONLY (10Hz within control thread)
    - get_*(), is_following(): ANY THREAD (thread-safe reads)
    - emergency_stop(): ANY THREAD (emergency use)
    """
    
    @abstractmethod
    def follow_route(self, route: Route, robot_id: str) -> None:
        """
        Start following a lane-based route.
        
        Args:
            route: Route to follow with lane segments
            robot_id: Robot identifier for conflict box management
            
        Raises:
            LaneFollowingError: If route following cannot be started
        """
        pass
    
    @abstractmethod
    def stop_following(self, force_release: bool = False) -> None:
        """
        Stop lane following and safely release conflict box locks.
        
        Args:
            force_release: If True, force release all locks (use with caution!)
                          If False, only release locks for boxes robot is not inside
        """
        pass
    
    @abstractmethod
    def get_lane_following_status(self) -> LaneFollowingStatus:
        """
        Get current lane following status.
        
        Returns:
            LaneFollowingStatus: Current status
        """
        pass
    
    @abstractmethod
    def get_lane_following_result(self) -> LaneFollowingResult:
        """
        Get detailed lane following result and progress.
        
        Returns:
            LaneFollowingResult: Current result with progress and status
        """
        pass
    
    @abstractmethod
    def is_following(self) -> bool:
        """
        Check if currently following a route.
        
        Returns:
            bool: True if following a route, False otherwise
        """
        pass
    
    @abstractmethod
    def get_current_route(self) -> Optional[Route]:
        """
        Get the current route being followed.
        
        Returns:
            Optional[Route]: Current route or None if not following
        """
        pass
    
    @abstractmethod
    def get_current_segment(self) -> Optional[RouteSegment]:
        """
        Get the current route segment being executed.
        
        Returns:
            Optional[RouteSegment]: Current segment or None if not following
        """
        pass
    
    @abstractmethod
    def get_conflict_box_status(self, box_id: str) -> ConflictBoxStatus:
        """
        Get the lock status of a specific conflict box.
        
        Args:
            box_id: Conflict box identifier
            
        Returns:
            ConflictBoxStatus: Current lock status
        """
        pass
    
    @abstractmethod
    def get_held_conflict_boxes(self) -> List[str]:
        """
        Get list of conflict boxes currently held by this robot.
        
        Returns:
            List[str]: List of conflict box IDs currently held
        """
        pass
    
    @abstractmethod
    def set_config(self, config: LaneFollowingConfig) -> None:
        """
        Update lane following configuration.
        
        Args:
            config: New configuration parameters
        """
        pass
    
    @abstractmethod
    def get_config(self) -> LaneFollowingConfig:
        """
        Get current lane following configuration.
        
        Returns:
            LaneFollowingConfig: Current configuration
        """
        pass
    
    @abstractmethod
    def emergency_stop(self) -> None:
        """
        Emergency stop - immediately halt lane following and release all locks.
        WARNING: This may release locks while robot is inside conflict boxes!
        """
        pass
    
    @abstractmethod
    def update_lane_following(self) -> None:
        """
        Update lane following execution (called at 10Hz from control thread).
        
        This method handles:
        - Lane center-line following
        - Conflict box lock management
        - Segment transitions
        - Motion coordination with MotionExecutor
        
        **Threading**: CONTROL THREAD ONLY - never call from physics or visualization threads
        """
        pass
    
    @abstractmethod
    def report_lane_deviation(self, deviation_distance: float, 
                            current_position: Point, 
                            lane_center: Point) -> None:
        """
        Report lane deviation for monitoring and debugging.
        
        Args:
            deviation_distance: Distance from lane center-line in meters
            current_position: Current robot position
            lane_center: Nearest point on lane center-line
        """
        pass
    
    @abstractmethod
    def get_lane_deviation_stats(self) -> Dict[str, Any]:
        """
        Get lane deviation statistics for monitoring.
        
        Returns:
            Dict[str, Any]: Statistics including max deviation, average deviation, etc.
        """
        pass
    
    @abstractmethod
    def get_unsafe_boxes(self) -> List[str]:
        """
        Get list of conflict boxes that would be unsafe to release.
        
        Returns:
            List[str]: List of box IDs where robot is currently inside
        """
        pass 