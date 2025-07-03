"""
Interface for PathPlanner - computes routes using lane-based routing.

This interface has been updated to support the new lane-based navigation system,
replacing cell-based A* pathfinding with lane-based routing.
"""
from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum

from .navigation_types import Route, Point, TaskType, RoutePlanningError


# Legacy support - keep for backward compatibility during transition
@dataclass
class Cell:
    """Represents a cell in the warehouse grid (legacy)."""
    x: int
    y: int
    
    def __hash__(self) -> int:
        return hash((self.x, self.y))
    
    def __eq__(self, other) -> bool:
        if isinstance(other, Cell):
            return self.x == other.x and self.y == other.y
        return False


@dataclass
class Path:
    """Represents a computed path (legacy)."""
    cells: List[Cell]
    total_distance: float
    estimated_time: float


class PathPlanningError(Exception):
    """Raised when path planning fails (legacy)."""
    pass


class IPathPlanner(ABC):
    """
    Interface for lane-based path planning functionality.
    
    Responsibilities:
    - Compute lane-based routes from robot's current position to target
    - Use A* algorithm on lane graph (not cell grid)
    - Handle blocked lanes and bay access control
    
    **Thread Safety**: All methods are thread-safe.
    **Threading Model**:
    - plan_route(): CONTROL THREAD ONLY (called by TaskHandler)
    - update_block(): ANY THREAD (block reports from robots)
    - get_blocked_cells(): ANY THREAD (read-only)
    """
    
    @abstractmethod
    def plan_route(self, start: Point, goal: Point, task_type: TaskType) -> Route:
        """
        Plan a lane-based route from start to goal position.
        
        Args:
            start: Starting position in world coordinates
            goal: Goal position in world coordinates
            task_type: Type of task (affects bay access permissions)
            
        Returns:
            Route: Computed route with lane segments and metadata
            
        Raises:
            RoutePlanningError: If no route can be found
        """
        pass
    
    @abstractmethod
    def update_block(self, cell_id: str, unblock_time: float) -> None:
        """
        Update blocked cell information (called by central planner).
        
        Args:
            cell_id: Identifier of blocked cell/lane
            unblock_time: Timestamp when cell will be unblocked
        """
        pass
    
    @abstractmethod
    def get_blocked_cells(self) -> Dict[str, float]:
        """
        Get current blocked cells information.
        
        Returns:
            Dict mapping cell_id to unblock_time
        """
        pass
    
    # Legacy methods for backward compatibility during transition
    @abstractmethod
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Path:
        """
        Legacy path planning method (deprecated).
        
        This method is kept for backward compatibility during the transition
        to lane-based navigation. New code should use plan_route().
        """
        pass
    
    @abstractmethod
    def is_path_blocked(self, path: Path) -> bool:
        """
        Legacy path blocking check (deprecated).
        
        This method is kept for backward compatibility during the transition
        to lane-based navigation.
        """
        pass
    
 