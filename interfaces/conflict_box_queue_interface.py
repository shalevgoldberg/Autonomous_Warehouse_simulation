"""
Conflict Box Queue Management Interface - Queue-based lock acquisition system.

This module provides interfaces for managing conflict box lock queues with priority-based
ordering, timeout handling, and deadlock prevention following SOLID principles.

Design Principles:
- **Single Responsibility**: Each interface has one clear purpose
- **Open/Closed**: Extensible through interfaces, closed for modification
- **Liskov Substitution**: All implementations are interchangeable
- **Interface Segregation**: Focused, specific interfaces
- **Dependency Inversion**: Depend on abstractions, not concretions
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass
from enum import Enum
import time


class QueuePosition(Enum):
    """Position status in the conflict box queue."""
    QUEUED = "queued"
    NEXT_IN_LINE = "next_in_line"
    LOCK_ACQUIRED = "lock_acquired"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"


@dataclass
class QueueEntry:
    """Entry in the conflict box queue."""
    robot_id: str
    box_id: str
    priority: int
    queue_time: float  # Unix timestamp when queued
    timeout_time: float  # Unix timestamp when request times out
    position: QueuePosition
    estimated_wait_time: Optional[float] = None  # Estimated wait time in seconds
    
    def __post_init__(self):
        if self.estimated_wait_time is None:
            self.estimated_wait_time = 0.0


@dataclass
class QueueStatus:
    """Status of a conflict box queue."""
    box_id: str
    current_owner: Optional[str]
    queue_length: int
    is_locked: bool
    next_robot: Optional[str]
    average_wait_time: float  # Average wait time in seconds
    lock_duration_stats: Dict[str, float]  # Min, max, avg lock duration
    entries: List[Dict[str, Any]] = None  # List of queue entries for detailed access
    
    def __post_init__(self):
        if self.entries is None:
            self.entries = []


@dataclass
class LockAcquisitionResult:
    """Result of a lock acquisition attempt."""
    success: bool
    position: QueuePosition
    estimated_wait_time: Optional[float]
    queue_position: Optional[int]  # Position in queue (1-based)
    error_message: Optional[str] = None


class ConflictBoxQueueError(Exception):
    """Raised when conflict box queue operations fail."""
    pass


class IConflictBoxQueue(ABC):
    """
    Interface for conflict box queue management.
    
    Responsibilities:
    - Manage priority-based queues for conflict box access
    - Handle timeout and cancellation of queue entries
    - Provide queue status and statistics
    - Support fair access with priority consideration
    - Prevent deadlocks through timeout mechanisms
    """
    
    @abstractmethod
    def request_lock(self, box_id: str, robot_id: str, priority: int = 0, 
                    timeout_seconds: float = 30.0) -> LockAcquisitionResult:
        """
        Request a lock on a conflict box with queue management.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot requesting the lock
            priority: Priority level (higher = more important)
            timeout_seconds: Maximum time to wait in queue
            
        Returns:
            LockAcquisitionResult: Result of the lock request
            
        Raises:
            ConflictBoxQueueError: If queue operation fails
        """
        pass
    
    @abstractmethod
    def release_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Release a conflict box lock and notify next robot in queue.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot releasing the lock
            
        Returns:
            bool: True if lock was released successfully
            
        Raises:
            ConflictBoxQueueError: If release operation fails
        """
        pass
    
    @abstractmethod
    def cancel_request(self, box_id: str, robot_id: str) -> bool:
        """
        Cancel a pending lock request.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot cancelling the request
            
        Returns:
            bool: True if request was cancelled successfully
            
        Raises:
            ConflictBoxQueueError: If cancellation fails
        """
        pass
    
    @abstractmethod
    def get_queue_status(self, box_id: str) -> QueueStatus:
        """
        Get current status of a conflict box queue.
        
        Args:
            box_id: Conflict box identifier
            
        Returns:
            QueueStatus: Current queue status
            
        Raises:
            ConflictBoxQueueError: If status retrieval fails
        """
        pass
    
    @abstractmethod
    def get_robot_queue_position(self, box_id: str, robot_id: str) -> Optional[QueueEntry]:
        """
        Get a robot's position in a specific queue.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot identifier
            
        Returns:
            Optional[QueueEntry]: Queue entry if robot is in queue, None otherwise
            
        Raises:
            ConflictBoxQueueError: If position retrieval fails
        """
        pass
    
    @abstractmethod
    def get_robot_queues(self, robot_id: str) -> List[QueueEntry]:
        """
        Get all queues that a robot is currently in.
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            List[QueueEntry]: List of queue entries for the robot
            
        Raises:
            ConflictBoxQueueError: If queue retrieval fails
        """
        pass
    
    @abstractmethod
    def heartbeat_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Send heartbeat for an active lock.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot holding the lock
            
        Returns:
            bool: True if heartbeat was successful
            
        Raises:
            ConflictBoxQueueError: If heartbeat fails
        """
        pass
    
    @abstractmethod
    def cleanup_expired_requests(self) -> int:
        """
        Remove expired queue entries and stale locks.
        
        Returns:
            int: Number of entries cleaned up
            
        Raises:
            ConflictBoxQueueError: If cleanup fails
        """
        pass
    
    @abstractmethod
    def get_queue_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive queue statistics.
        
        Returns:
            Dict[str, Any]: Statistics including:
                - total_queues: Number of active queues
                - total_waiting_robots: Number of robots waiting
                - average_wait_time: Average wait time across all queues
                - lock_utilization: Percentage of time locks are held
                - timeout_rate: Percentage of requests that timeout
                - priority_distribution: Distribution of priority levels
                
        Raises:
            ConflictBoxQueueError: If statistics retrieval fails
        """
        pass


class IConflictBoxQueueNotifier(ABC):
    """
    Interface for conflict box queue notifications.
    
    Responsibilities:
    - Notify robots when their queue position changes
    - Notify robots when locks become available
    - Handle timeout notifications
    - Support asynchronous notification patterns
    """
    
    @abstractmethod
    def notify_lock_available(self, box_id: str, robot_id: str) -> None:
        """
        Notify a robot that a lock is now available.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot to notify
            
        Raises:
            ConflictBoxQueueError: If notification fails
        """
        pass
    
    @abstractmethod
    def notify_queue_position_changed(self, box_id: str, robot_id: str, 
                                    new_position: int, estimated_wait_time: float) -> None:
        """
        Notify a robot that their queue position has changed.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot to notify
            new_position: New position in queue (1-based)
            estimated_wait_time: Estimated wait time in seconds
            
        Raises:
            ConflictBoxQueueError: If notification fails
        """
        pass
    
    @abstractmethod
    def notify_request_timeout(self, box_id: str, robot_id: str) -> None:
        """
        Notify a robot that their lock request has timed out.
        
        Args:
            box_id: Conflict box identifier
            robot_id: Robot to notify
            
        Raises:
            ConflictBoxQueueError: If notification fails
        """
        pass
    
    @abstractmethod
    def register_callback(self, robot_id: str, callback: callable) -> None:
        """
        Register a callback function for queue notifications.
        
        Args:
            robot_id: Robot identifier
            callback: Callback function to receive notifications
            
        Raises:
            ConflictBoxQueueError: If registration fails
        """
        pass
    
    @abstractmethod
    def unregister_callback(self, robot_id: str) -> None:
        """
        Unregister a callback function.
        
        Args:
            robot_id: Robot identifier
            
        Raises:
            ConflictBoxQueueError: If unregistration fails
        """
        pass