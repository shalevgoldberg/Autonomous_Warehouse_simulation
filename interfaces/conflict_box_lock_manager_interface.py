"""
Conflict Box Lock Manager Interface

This interface defines the contract for managing conflict box locks
in a multi-robot warehouse system. It follows SOLID principles and
provides a clean, modular approach to lock management.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
from datetime import datetime, timedelta
from .conflict_box_queue_interface import LockAcquisitionResult


@dataclass
class LockInfo:
    """Information about a conflict box lock."""
    box_id: str
    locked_by_robot: str
    priority: int
    locked_at: datetime
    heartbeat_at: datetime
    lock_timeout_seconds: int
    robot_inside: bool


@dataclass
class LockHealthStatus:
    """Health status of a conflict box lock."""
    box_id: str
    is_healthy: bool
    last_heartbeat: datetime
    time_since_heartbeat: timedelta
    is_expired: bool
    robot_id: str


class IConflictBoxLockManager(ABC):
    """
    Interface for managing conflict box locks.
    
    This service provides high-level lock management operations including:
    - Lock acquisition and release
    - Heartbeat management
    - Lock health monitoring
    - Expired lock cleanup
    - Lock validation
    """
    
    @abstractmethod
    def acquire_lock(self, box_id: str, robot_id: str, priority: int = 0) -> LockAcquisitionResult:
        """
        Attempt to acquire a lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot attempting to acquire the lock
            priority: Priority level for the lock request
            
        Returns:
            LockAcquisitionResult indicating success or failure
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If lock acquisition operation fails
        """
        pass
    
    @abstractmethod
    def release_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Release a lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot releasing the lock
            
        Returns:
            True if lock was released successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If lock release operation fails
        """
        pass
    
    @abstractmethod
    def update_heartbeat(self, box_id: str, robot_id: str) -> bool:
        """
        Update the heartbeat for a lock to indicate the robot is still active.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot holding the lock
            
        Returns:
            True if heartbeat was updated successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If heartbeat update fails
        """
        pass
    
    @abstractmethod
    def get_lock_info(self, box_id: str) -> Optional[LockInfo]:
        """
        Get information about the current lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            LockInfo if a lock exists, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If lock info retrieval fails
        """
        pass
    
    @abstractmethod
    def is_locked(self, box_id: str) -> bool:
        """
        Check if a conflict box is currently locked.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            True if the box is locked, False otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If lock check fails
        """
        pass
    
    @abstractmethod
    def get_locked_by_robot(self, box_id: str) -> Optional[str]:
        """
        Get the robot ID that currently holds the lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            Robot ID if locked, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If robot ID retrieval fails
        """
        pass
    
    @abstractmethod
    def check_lock_health(self, box_id: str) -> Optional[LockHealthStatus]:
        """
        Check the health status of a lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            LockHealthStatus if a lock exists, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If health check fails
        """
        pass
    
    @abstractmethod
    def cleanup_expired_locks(self, box_id: Optional[str] = None) -> int:
        """
        Clean up expired locks based on heartbeat timeout.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            Number of expired locks cleaned up
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If cleanup operation fails
        """
        pass
    
    @abstractmethod
    def validate_locks(self, box_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Validate all locks and report any issues.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            List of lock validation issues found
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If validation operation fails
        """
        pass
    
    @abstractmethod
    def get_locks_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all conflict box locks.
        
        Returns:
            Dictionary containing lock summary information
            
        Raises:
            RuntimeError: If summary retrieval fails
        """
        pass
    
    @abstractmethod
    def set_robot_inside_status(self, box_id: str, robot_id: str, is_inside: bool) -> bool:
        """
        Set whether a robot is inside a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot identifier
            is_inside: Whether the robot is inside the box
            
        Returns:
            True if status was updated successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If status update fails
        """
        pass


