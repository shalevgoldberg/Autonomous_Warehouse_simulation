"""
Navigation Types - Lane-based navigation data structures.

This module defines the core data structures for the new lane-based navigation system,
replacing the cell-based path planning with lane-based routing.
"""
from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Dict, Set
from dataclasses import dataclass
from enum import Enum


class LaneDirection(Enum):
    """Lane direction flags as specified in the redesign."""
    NORTH = "N"
    SOUTH = "S" 
    EAST = "E"
    WEST = "W"
    BIDIRECTIONAL = "B"  # Two-way lane


@dataclass
class Point:
    """2D point in world coordinates."""
    x: float
    y: float
    
    def __hash__(self) -> int:
        return hash((self.x, self.y))
    
    def __eq__(self, other) -> bool:
        if isinstance(other, Point):
            return abs(self.x - other.x) < 1e-6 and abs(self.y - other.y) < 1e-6
        return False


@dataclass
class LaneRec:
    """Lane record as defined in lanes.csv."""
    lane_id: str
    direction: LaneDirection
    waypoints: List[Point]  # x0,y0,x1,y1,... from CSV
    is_goal_only: bool = False  # True for bay spurs
    bay_id: Optional[str] = None  # Only for bay spurs
    
    def __post_init__(self):
        """Validate lane data."""
        if len(self.waypoints) < 2:
            raise ValueError(f"Lane {self.lane_id} must have at least 2 waypoints")
        
        if self.is_goal_only and not self.bay_id:
            raise ValueError(f"Goal-only lane {self.lane_id} must have bay_id")


@dataclass
class BoxRec:
    """Conflict box record (axis-aligned rectangle)."""
    box_id: str
    center: Point
    width: float
    height: float
    
    def __post_init__(self):
        """Validate box data."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"Box {self.box_id} dimensions must be positive")


@dataclass
class RouteSegment:
    """Segment of a route along a specific lane."""
    lane: LaneRec
    reverse: bool = False  # True if traveling opposite to lane direction
    start_point: Optional[Point] = None  # Entry point into this segment
    end_point: Optional[Point] = None    # Exit point from this segment
    
    def __post_init__(self):
        """Set default points if not provided."""
        if self.start_point is None and self.lane.waypoints:
            self.start_point = self.lane.waypoints[0]
        if self.end_point is None and self.lane.waypoints:
            self.end_point = self.lane.waypoints[-1]


@dataclass
class Route:
    """Complete route from start to goal using lane segments."""
    segments: List[RouteSegment]
    total_distance: float
    estimated_time: float
    conflict_boxes: Optional[List[BoxRec]] = None  # Conflict boxes this route passes through
    
    def __post_init__(self):
        """Initialize conflict boxes if not provided."""
        if self.conflict_boxes is None:
            self.conflict_boxes = []


@dataclass
class PathPlanningResult:
    """
    Result of path planning operation that handles both success and failure cases.
    
    This allows the system to handle path planning failures gracefully without
    throwing exceptions that could crash the robot system.
    """
    success: bool
    route: Optional[Route] = None
    error_message: Optional[str] = None
    failure_reason: Optional[str] = None
    retry_after: Optional[float] = None  # Seconds to wait before retry
    
    @classmethod
    def success_result(cls, route: Route) -> 'PathPlanningResult':
        """Create a successful path planning result."""
        return cls(success=True, route=route)
    
    @classmethod
    def failure_result(cls, error_message: str, failure_reason: str, 
                      retry_after: Optional[float] = None) -> 'PathPlanningResult':
        """Create a failed path planning result."""
        return cls(
            success=False,
            error_message=error_message,
            failure_reason=failure_reason,
            retry_after=retry_after
        )
    
    def is_retryable(self) -> bool:
        """Check if this failure can be retried."""
        return not self.success and self.retry_after is not None


class TaskType(Enum):
    """Enhanced task types for lane-based navigation."""
    PICK_AND_DELIVER = "pick_and_deliver"
    MOVE_TO_CHARGING = "move_to_charging"
    MOVE_TO_POSITION = "move_to_position"
    IDLE_PARK = "idle_park"  # New: park in idle bay


class NavigationError(Exception):
    """Base exception for navigation errors."""
    pass


class RoutePlanningError(NavigationError):
    """Raised when route planning fails."""
    pass


class LaneNotFoundError(NavigationError):
    """Raised when a required lane is not found."""
    pass 