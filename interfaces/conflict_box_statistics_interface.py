"""
Conflict Box Statistics Interface

This interface defines the contract for collecting and analyzing
conflict box statistics in a multi-robot warehouse system. It follows
SOLID principles and provides a clean, modular approach to statistics.
"""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
from datetime import datetime, timedelta


@dataclass
class ConflictBoxStats:
    """Statistics for a specific conflict box."""
    box_id: str
    total_requests: int
    successful_acquisitions: int
    timeouts: int
    cancellations: int
    total_wait_time: float
    total_lock_time: float
    last_updated: datetime


@dataclass
class PerformanceMetrics:
    """Performance metrics for conflict box operations."""
    box_id: str
    average_wait_time: float
    average_lock_time: float
    throughput_per_hour: float
    success_rate: float
    timeout_rate: float
    cancellation_rate: float


@dataclass
class SystemOverview:
    """System-wide overview of conflict box operations."""
    total_conflict_boxes: int
    active_locks: int
    total_queued_robots: int
    system_throughput: float
    average_response_time: float
    overall_success_rate: float


class IConflictBoxStatistics(ABC):
    """
    Interface for collecting and analyzing conflict box statistics.
    
    This service provides comprehensive statistics and analytics including:
    - Performance metrics collection
    - Historical data analysis
    - System health monitoring
    - Trend analysis
    - Reporting capabilities
    """
    
    @abstractmethod
    def record_request(self, box_id: str, robot_id: str, priority: int) -> bool:
        """
        Record a new request for a conflict box.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot making the request
            priority: Priority level of the request
            
        Returns:
            True if request was recorded successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If recording operation fails
        """
        pass
    
    @abstractmethod
    def record_acquisition(self, box_id: str, robot_id: str, wait_time: float) -> bool:
        """
        Record a successful lock acquisition.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot that acquired the lock
            wait_time: Time spent waiting in queue
            
        Returns:
            True if acquisition was recorded successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If recording operation fails
        """
        pass
    
    @abstractmethod
    def record_timeout(self, box_id: str, robot_id: str, wait_time: float) -> bool:
        """
        Record a request timeout.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot that timed out
            wait_time: Time spent waiting before timeout
            
        Returns:
            True if timeout was recorded successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If recording operation fails
        """
        pass
    
    @abstractmethod
    def record_cancellation(self, box_id: str, robot_id: str, wait_time: float) -> bool:
        """
        Record a request cancellation.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot that cancelled
            wait_time: Time spent waiting before cancellation
            
        Returns:
            True if cancellation was recorded successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If recording operation fails
        """
        pass
    
    @abstractmethod
    def record_lock_release(self, box_id: str, robot_id: str, lock_time: float) -> bool:
        """
        Record a lock release.
        
        Args:
            box_id: The conflict box identifier
            robot_id: The robot that released the lock
            lock_time: Time the lock was held
            
        Returns:
            True if release was recorded successfully, False otherwise
            
        Raises:
            ValueError: If box_id or robot_id is invalid
            RuntimeError: If recording operation fails
        """
        pass
    
    @abstractmethod
    def get_box_statistics(self, box_id: str) -> Optional[ConflictBoxStats]:
        """
        Get statistics for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            
        Returns:
            ConflictBoxStats if available, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If statistics retrieval fails
        """
        pass
    
    @abstractmethod
    def get_performance_metrics(self, box_id: str, time_period: Optional[timedelta] = None) -> Optional[PerformanceMetrics]:
        """
        Get performance metrics for a specific conflict box.
        
        Args:
            box_id: The conflict box identifier
            time_period: Optional time period for metrics calculation
            
        Returns:
            PerformanceMetrics if available, None otherwise
            
        Raises:
            ValueError: If box_id is invalid
            RuntimeError: If metrics retrieval fails
        """
        pass
    
    @abstractmethod
    def get_system_overview(self, time_period: Optional[timedelta] = None) -> SystemOverview:
        """
        Get system-wide overview of conflict box operations.
        
        Args:
            time_period: Optional time period for overview calculation
            
        Returns:
            SystemOverview containing system statistics
            
        Raises:
            RuntimeError: If overview retrieval fails
        """
        pass
    
    @abstractmethod
    def get_trend_analysis(self, box_id: str, metric: str, time_period: timedelta) -> List[Dict[str, Any]]:
        """
        Get trend analysis for a specific metric over time.
        
        Args:
            box_id: The conflict box identifier
            metric: The metric to analyze (e.g., 'wait_time', 'success_rate')
            time_period: Time period for trend analysis
            
        Returns:
            List of trend data points
            
        Raises:
            ValueError: If box_id or metric is invalid
            RuntimeError: If trend analysis fails
        """
        pass
    
    @abstractmethod
    def cleanup_old_statistics(self, older_than: timedelta) -> int:
        """
        Clean up old statistics data to maintain performance.
        
        Args:
            older_than: Remove statistics older than this time period
            
        Returns:
            Number of records cleaned up
            
        Raises:
            ValueError: If older_than is invalid
            RuntimeError: If cleanup operation fails
        """
        pass
    
    @abstractmethod
    def export_statistics(self, box_id: Optional[str] = None, format: str = "json") -> str:
        """
        Export statistics data in various formats.
        
        Args:
            box_id: Optional specific conflict box, or None for all boxes
            format: Export format ('json', 'csv', 'xml')
            
        Returns:
            Exported data as string
            
        Raises:
            ValueError: If box_id or format is invalid
            RuntimeError: If export operation fails
        """
        pass







