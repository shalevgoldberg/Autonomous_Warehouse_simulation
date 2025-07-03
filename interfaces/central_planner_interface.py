"""
Interface for Central Planner - manages lane-based routing and blocked cells.

This interface defines the central planner service that coordinates lane-based
navigation across all robots, handling route requests and blocked cell reports.
"""
from abc import ABC, abstractmethod
from typing import Optional, Dict
from dataclasses import dataclass

from .navigation_types import Route, Point, TaskType


@dataclass
class RouteRequest:
    """Request for a route from central planner."""
    robot_id: str
    start: Point
    goal: Point
    task_type: TaskType
    timestamp: float


@dataclass
class BlockReport:
    """Report of a blocked cell from a robot."""
    robot_id: str
    cell_id: str
    timestamp: float
    block_duration: float = 60.0  # Default 60 seconds


class CentralPlannerError(Exception):
    """Raised when central planner operations fail."""
    pass


class ICentralPlanner(ABC):
    """
    Interface for central planner functionality.
    
    Responsibilities:
    - Manage lane-based route planning for all robots
    - Handle blocked cell reports and unblock notifications
    - Coordinate bay access and scheduling
    - Maintain blocked cell state across the system
    
    **Thread Safety**: All methods are thread-safe.
    **Threading Model**:
    - request_route(): Called from robot control threads
    - report_block/report_unblock(): Called from robot control threads
    - get_blocked_cells(): Called from any thread (read-only)
    """
    
    @abstractmethod
    def request_route(self, robot_id: str, start: Point, goal: Point, 
                     task_type: TaskType) -> Route:
        """
        Request a route from start to goal for a specific robot and task.
        
        Args:
            robot_id: Unique identifier for the requesting robot
            start: Starting position in world coordinates
            goal: Goal position in world coordinates
            task_type: Type of task (affects bay access permissions)
            
        Returns:
            Route: Computed route with lane segments and metadata
            
        Raises:
            CentralPlannerError: If route planning fails
        """
        pass
    
    @abstractmethod
    def report_block(self, robot_id: str, cell_id: str, 
                    block_duration: float = 60.0) -> None:
        """
        Report that a cell/lane is blocked by a robot.
        
        Args:
            robot_id: Robot reporting the block
            cell_id: Identifier of blocked cell/lane
            block_duration: How long the cell should remain blocked (seconds)
        """
        pass
    
    @abstractmethod
    def report_unblock(self, robot_id: str, cell_id: str) -> None:
        """
        Report that a previously blocked cell/lane is now clear.
        
        Args:
            robot_id: Robot reporting the unblock
            cell_id: Identifier of unblocked cell/lane
        """
        pass
    
    @abstractmethod
    def get_blocked_cells(self) -> Dict[str, float]:
        """
        Get current blocked cells information.
        
        Returns:
            Dict mapping cell_id to unblock timestamp
        """
        pass
    
    @abstractmethod
    def cleanup_expired_blocks(self) -> None:
        """
        Remove expired block entries (called periodically).
        """
        pass
    
    @abstractmethod
    def get_planner_status(self) -> Dict[str, object]:
        """
        Get central planner status information.
        
        Returns:
            Dict containing planner statistics and status
        """
        pass 