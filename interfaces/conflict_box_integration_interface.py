"""
Conflict Box Integration Service Interface

This interface defines the contract for integrating queue and lock management
services into a cohesive, thread-safe conflict box management system.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any, Tuple
from dataclasses import dataclass
from datetime import datetime, timedelta
from .conflict_box_queue_interface import QueueStatus, LockAcquisitionResult
from .conflict_box_queue_manager_interface import QueueMetrics, QueueIntegrityIssue
from .conflict_box_lock_manager_interface import LockInfo, LockHealthStatus


@dataclass
class ConflictBoxState:
    """Complete state of a conflict box including queue and lock information."""
    box_id: str
    queue_status: QueueStatus
    lock_info: Optional[LockInfo]
    is_healthy: bool
    last_updated: datetime
    performance_metrics: Dict[str, Any]


@dataclass
class IntegrationMetrics:
    """Performance metrics for the integrated conflict box system."""
    total_operations: int
    successful_operations: int
    failed_operations: int
    average_response_time: float
    concurrent_operations: int
    connection_pool_health: Dict[str, Any]
    last_updated: datetime


class IConflictBoxIntegrationService(ABC):
    """
    Interface for integrating conflict box queue and lock management services.
    
    This service provides:
    - Coordinated queue and lock operations
    - Thread-safe conflict box management
    - Performance monitoring and health checks
    - Race condition prevention
    - Comprehensive error handling
    """
    
    @abstractmethod
    def request_conflict_box_access(self, box_id: str, robot_id: str, 
                                  priority: int = 0, timeout_seconds: float = 30.0) -> LockAcquisitionResult:
        """
        Request access to a conflict box with integrated queue and lock management.
        
        This method coordinates:
        1. Queue position management
        2. Lock acquisition when appropriate
        3. Thread-safe operations
        4. Proper error handling
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot requesting access
            priority: Priority level for the request
            timeout_seconds: Maximum time to wait for access
            
        Returns:
            LockAcquisitionResult indicating success or failure
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If the operation fails
        """
        pass
    
    @abstractmethod
    def release_conflict_box_access(self, box_id: str, robot_id: str) -> bool:
        """
        Release access to a conflict box and coordinate next robot promotion.
        
        This method coordinates:
        1. Lock release
        2. Queue position updates
        3. Next robot promotion
        4. Thread-safe operations
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot releasing access
            
        Returns:
            True if access was released successfully, False otherwise
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If the operation fails
        """
        pass
    
    @abstractmethod
    def get_conflict_box_state(self, box_id: str) -> ConflictBoxState:
        """
        Get the complete state of a conflict box including queue and lock information.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            Complete conflict box state
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If state retrieval fails
        """
        pass
    
    @abstractmethod
    def update_robot_heartbeat(self, box_id: str, robot_id: str) -> bool:
        """
        Update the heartbeat for a robot holding a lock on a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot updating the heartbeat
            
        Returns:
            True if heartbeat was updated successfully, False otherwise
            
        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If the operation fails
        """
        pass
    
    @abstractmethod
    def cleanup_expired_entries(self, box_id: Optional[str] = None) -> int:
        """
        Clean up expired queue entries and locks with coordinated cleanup.
        
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
    def validate_system_integrity(self, box_id: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Validate the integrity of the entire conflict box system.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            
        Returns:
            List of integrity issues found
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If validation fails
        """
        pass
    
    @abstractmethod
    def get_system_metrics(self) -> IntegrationMetrics:
        """
        Get comprehensive metrics for the integrated conflict box system.
        
        Returns:
            System-wide performance and health metrics
            
        Raises:
            RuntimeError: If metrics retrieval fails
        """
        pass
    
    @abstractmethod
    def is_system_healthy(self) -> bool:
        """
        Check if the entire conflict box system is healthy.
        
        Returns:
            True if system is healthy, False otherwise
            
        Raises:
            RuntimeError: If health check fails
        """
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """
        Gracefully shutdown the integration service.
        
        This method ensures:
        1. All active operations complete
        2. Connection pools are properly closed
        3. Resources are cleaned up
        4. No data loss occurs
        """
        pass




















