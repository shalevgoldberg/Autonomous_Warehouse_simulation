"""
Interface for PathPlanner - computes routes using A* algorithm.
"""
from abc import ABC, abstractmethod
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


@dataclass
class Cell:
    """Represents a cell in the warehouse grid."""
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
    """Represents a computed path."""
    cells: List[Cell]
    total_distance: float
    estimated_time: float


class PathPlanningError(Exception):
    """Raised when path planning fails."""
    pass


class IPathPlanner(ABC):
    """
    Interface for path planning functionality.
    
    Responsibilities:
    - Compute cell-wise route from robot's current position to target
    - Use A* algorithm on static map
    - Handle obstacles and blocked paths
    
    **Thread Safety**: All methods are thread-safe.
    **Threading Model**:
    - plan_path(): CONTROL THREAD ONLY (called by TaskHandler)
    - is_path_blocked(): ANY THREAD (read-only check)
    """
    
    @abstractmethod
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Path:
        """
        Plan a path from start to goal position.
        
        Args:
            start: Starting position (x, y) in world coordinates
            goal: Goal position (x, y) in world coordinates
            
        Returns:
            Path: Computed path with cells and metadata
            
        Raises:
            PathPlanningError: If no path can be found
        """
        pass
    
    @abstractmethod
    def is_path_blocked(self, path: Path) -> bool:
        """
        Check if a previously computed path is now blocked.
        
        Args:
            path: Path to check
            
        Returns:
            bool: True if path is blocked, False otherwise
        """
        pass
    
 