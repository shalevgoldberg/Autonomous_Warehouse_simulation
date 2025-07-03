"""
Interface for Bay Scheduler - manages bay allocation for charging and idle parking.

This interface defines the bay scheduling service that allocates available bays
to robots for charging and idle parking based on proximity and availability.
"""
from abc import ABC, abstractmethod
from typing import Optional, List, Dict
from dataclasses import dataclass
from enum import Enum


class BayPurpose(Enum):
    """Purpose for bay allocation."""
    CHARGING = "charging"
    IDLE_PARK = "idle_park"


@dataclass
class BayInfo:
    """Information about a bay."""
    bay_id: str
    center_position: tuple  # (x, y) in world coordinates
    purpose: BayPurpose
    is_available: bool
    assigned_robot_id: Optional[str] = None
    assigned_at: Optional[float] = None  # timestamp


@dataclass
class BayRequest:
    """Request for bay allocation."""
    robot_id: str
    purpose: BayPurpose
    robot_position: tuple  # (x, y) in world coordinates
    timestamp: float


class BaySchedulerError(Exception):
    """Raised when bay scheduling operations fail."""
    pass


class IBayScheduler(ABC):
    """
    Interface for bay scheduling functionality.
    
    Responsibilities:
    - Manage bay availability and allocation
    - Find nearest available bay for robot requests
    - Handle bay assignment and release
    - Coordinate with central planner for bay access
    
    **Thread Safety**: All methods are thread-safe.
    **Threading Model**:
    - request_bay/release_bay(): Called from robot control threads
    - get_bay_status(): Called from any thread (read-only)
    """
    
    @abstractmethod
    def request_bay(self, robot_id: str, purpose: BayPurpose, 
                   robot_position: tuple) -> Optional[BayInfo]:
        """
        Request allocation of a bay for a robot.
        
        Args:
            robot_id: Unique identifier for the requesting robot
            purpose: Purpose for bay allocation (charging or idle parking)
            robot_position: Current robot position (x, y) in world coordinates
            
        Returns:
            BayInfo: Allocated bay information, or None if no bay available
            
        Raises:
            BaySchedulerError: If bay allocation fails
        """
        pass
    
    @abstractmethod
    def release_bay(self, bay_id: str, robot_id: str) -> bool:
        """
        Release a previously allocated bay.
        
        Args:
            bay_id: Identifier of bay to release
            robot_id: Robot releasing the bay (must be the assigned robot)
            
        Returns:
            bool: True if bay was successfully released, False otherwise
        """
        pass
    
    @abstractmethod
    def get_bay_status(self, bay_id: str) -> Optional[BayInfo]:
        """
        Get current status of a specific bay.
        
        Args:
            bay_id: Identifier of bay to check
            
        Returns:
            BayInfo: Current bay status, or None if bay doesn't exist
        """
        pass
    
    @abstractmethod
    def get_all_bays(self) -> List[BayInfo]:
        """
        Get status of all bays.
        
        Returns:
            List[BayInfo]: Status of all bays in the system
        """
        pass
    
    @abstractmethod
    def get_available_bays(self, purpose: BayPurpose) -> List[BayInfo]:
        """
        Get list of available bays for a specific purpose.
        
        Args:
            purpose: Purpose for bay allocation
            
        Returns:
            List[BayInfo]: List of available bays for the specified purpose
        """
        pass
    
    @abstractmethod
    def get_scheduler_status(self) -> Dict[str, object]:
        """
        Get bay scheduler status information.
        
        Returns:
            Dict containing scheduler statistics and status
        """
        pass 