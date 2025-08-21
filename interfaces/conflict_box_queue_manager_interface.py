"""
Conflict Box Queue Manager Interface

This interface defines the contract for managing conflict box queues
in a multi-robot warehouse system. It follows SOLID principles and
provides a clean, modular approach to queue management.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
from datetime import datetime, timedelta
from .conflict_box_queue_interface import QueueEntry, QueueStatus, LockAcquisitionResult


@dataclass
class QueueMetrics:
    """Queue performance metrics for monitoring and optimization."""
    total_robots: int
    queued_robots: int
    next_in_line_robots: int
    locked_robots: int
    average_wait_time: float
    max_wait_time: float
    priority_distribution: Dict[str, int]


@dataclass
class QueueIntegrityIssue:
    """Represents a queue integrity issue that needs attention."""
    box_id: str
    issue_type: str
    issue_description: str
    robot_count: int
    severity: str


class IConflictBoxQueueManager(ABC):
    """
    Interface for managing conflict box queues.
    
    This service provides high-level queue management operations including:
    - Queue position management
    - Robot promotion logic
    - Queue integrity validation
    - Performance monitoring
    - Cleanup operations
    """
    
    @abstractmethod
    def update_queue_positions(self, box_id: str) -> bool:
        """
        Update queue positions for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            True if positions were updated successfully, False otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If database operation fails
        """
        pass
    
    @abstractmethod
    def promote_next_robot(self, box_id: str) -> Optional[str]:
        """
        Promote the next robot in line for a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            The robot ID that was promoted, or None if no robot to promote
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If promotion operation fails
        """
        pass
    
    @abstractmethod
    def cleanup_expired_entries(self, box_id: Optional[str] = None) -> int:
        """
        Clean up expired queue entries and locks.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            Number of entries cleaned up
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If cleanup operation fails
        """
        pass
    
    @abstractmethod
    def validate_queue_integrity(self, box_id: Optional[str] = None) -> List[QueueIntegrityIssue]:
        """
        Validate queue integrity and report any issues.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            List of integrity issues found
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If validation operation fails
        """
        pass
    
    @abstractmethod
    def get_queue_metrics(self, box_id: Optional[str] = None) -> List[QueueMetrics]:
        """
        Get performance metrics for conflict box queues.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            List of queue metrics
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If metrics retrieval fails
        """
        pass
    
    @abstractmethod
    def get_queue_status(self, box_id: str) -> QueueStatus:
        """
        Get the current status of a conflict box queue.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            Current queue status
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If status retrieval fails
        """
        pass
    
    @abstractmethod
    def is_queue_healthy(self, box_id: str) -> bool:
        """
        Check if a conflict box queue is in a healthy state.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            True if queue is healthy, False otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If health check fails
        """
        pass
    
    @abstractmethod
    def get_queue_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all conflict box queues.
        
        Returns:
            Dictionary containing queue summary information
            
        Raises:
            RuntimeError: If summary retrieval fails
        """
        pass

    @abstractmethod
    def add_robot_to_queue(self, box_id: str, robot_id: str, priority: int = 0) -> int:
        """
        Add a robot to the queue for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier to add to queue
            priority: Priority level for the robot (higher = higher priority)
            
        Returns:
            The position in the queue (1-based, where 1 is next in line)
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If robot is already in queue or database operation fails
        """
        pass

    @abstractmethod
    def remove_robot_from_queue(self, box_id: str, robot_id: str) -> bool:
        """
        Remove a robot from the queue for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier to remove from queue
            
        Returns:
            True if robot was removed successfully, False otherwise
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If database operation fails
        """
        pass

    @abstractmethod
    def calculate_estimated_wait_time(self, box_id: str, robot_id: str) -> float:
        """
        Calculate estimated wait time for a robot in the queue.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier
            
        Returns:
            Estimated wait time in seconds
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If calculation fails
        """
        pass
