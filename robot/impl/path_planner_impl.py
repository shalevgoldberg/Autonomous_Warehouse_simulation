"""
PathPlanner Implementation - A* pathfinding with static obstacle avoidance.

This implementation provides thread-safe pathfinding services using the A* algorithm
with support for static obstacles and coordinate system integration.
"""
import heapq
import threading
from typing import Optional, List, Tuple, Set, Dict
from dataclasses import dataclass

from interfaces.path_planner_interface import (
    IPathPlanner, Path, Cell, PathPlanningError
)
from interfaces.coordinate_system_interface import ICoordinateSystem


class PathPlannerImpl(IPathPlanner):
    """
    Thread-safe A* pathfinding implementation.
    
    Features:
    - A* algorithm for optimal pathfinding
    - Static obstacle avoidance
    - Thread-safe obstacle management
    - Performance caching
    """
    
    def __init__(self, coordinate_system: ICoordinateSystem, 
                 static_obstacles: Optional[Set[Cell]] = None):
        """
        Initialize PathPlanner with coordinate system and obstacles.
        
        Args:
            coordinate_system: Coordinate conversion system
            static_obstacles: Set of blocked cells
        """
        self.coordinate_system = coordinate_system
        self.static_obstacles = static_obstacles or set()
        
        # Thread safety for obstacle access
        self._obstacles_lock = threading.RLock()
        
        # Algorithm parameters
        self.movement_cost = 1.0
        self.diagonal_cost = 1.414  # Cost for diagonal movement (√2)
        
        # Cache for performance (thread-safe)
        self._distance_cache: Dict[Tuple[Cell, Cell], float] = {}
        self._cache_lock = threading.Lock()
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Path:
        """
        Plan a path from start to goal position using A*.
        **Thread-safe**: Uses internal locking for obstacle access.
        
        Args:
            start: Starting position (x, y) in world coordinates
            goal: Goal position (x, y) in world coordinates
            
        Returns:
            Path: Computed path with cells and metadata
            
        Raises:
            PathPlanningError: If no path can be found
        """
        print(f"[PathPlanner] === PLANNING PATH ===")
        print(f"[PathPlanner] Start: {start} -> Goal: {goal}")
        
        # Convert to cell coordinates
        start_cell = self.coordinate_system.world_to_cell(start)
        goal_cell = self.coordinate_system.world_to_cell(goal)
        
        print(f"[PathPlanner] Start cell: ({start_cell.x}, {start_cell.y})")
        print(f"[PathPlanner] Goal cell: ({goal_cell.x}, {goal_cell.y})")
        print(f"[PathPlanner] Total obstacles: {len(self.static_obstacles)}")
        
        # Validate positions
        if not self.coordinate_system.is_valid_cell(start_cell):
            raise PathPlanningError(f"Start position {start} is outside grid bounds")
        
        if not self.coordinate_system.is_valid_cell(goal_cell):
            raise PathPlanningError(f"Goal position {goal} is outside grid bounds")
        
        # Check if start or goal are blocked
        with self._obstacles_lock:
            if start_cell in self.static_obstacles:
                print(f"[PathPlanner] ERROR: Start cell ({start_cell.x}, {start_cell.y}) is blocked!")
                raise PathPlanningError(f"Start position {start} is blocked")
            
            if goal_cell in self.static_obstacles:
                print(f"[PathPlanner] ERROR: Goal cell ({goal_cell.x}, {goal_cell.y}) is blocked!")
                raise PathPlanningError(f"Goal position {goal} is blocked")
        
        # Run A* algorithm
        cell_path = self._astar_search(start_cell, goal_cell)
        
        if not cell_path:
            print(f"[PathPlanner] ERROR: No path found!")
            raise PathPlanningError(f"No path found from {start} to {goal}")
        
        print(f"[PathPlanner] SUCCESS: Found path with {len(cell_path)} cells")
        
        # Log the path details
        print(f"[PathPlanner] Path waypoints:")
        for i, cell in enumerate(cell_path):
            world_pos = self.coordinate_system.cell_to_world(cell)
            is_obstacle = cell in self.static_obstacles
            print(f"[PathPlanner]   {i}: Cell({cell.x}, {cell.y}) -> World({world_pos[0]:.3f}, {world_pos[1]:.3f}) {'[OBSTACLE!]' if is_obstacle else '[FREE]'}")
        
        # Check for obstacles in path (this should never happen with working A*)
        obstacles_in_path = [cell for cell in cell_path if cell in self.static_obstacles]
        if obstacles_in_path:
            print(f"[PathPlanner] WARNING: Path contains {len(obstacles_in_path)} obstacles!")
            for obs_cell in obstacles_in_path:
                print(f"[PathPlanner]   Obstacle at Cell({obs_cell.x}, {obs_cell.y})")
        else:
            print(f"[PathPlanner] ✓ Path is obstacle-free")
        
        # Convert to world coordinates and create Path object
        world_path = []
        for cell in cell_path:
            world_pos = self.coordinate_system.cell_to_world(cell)
            world_path.append(world_pos)
        
        # Calculate path metadata
        total_distance = self._calculate_path_distance(world_path)
        estimated_time = self._estimate_travel_time(total_distance)
        
        return Path(
            cells=cell_path,
            total_distance=total_distance,
            estimated_time=estimated_time
        )
    
    def is_path_blocked(self, path: Path) -> bool:
        """
        Check if a previously computed path is now blocked.
        **Thread-safe**: Read-only check with locking.
        
        Args:
            path: Path to check
            
        Returns:
            bool: True if path is blocked, False otherwise
        """
        with self._obstacles_lock:
            for cell in path.cells:
                if cell in self.static_obstacles:
                    return True
        return False
    
    def _astar_search(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        """
        A* pathfinding algorithm implementation.
        
        Args:
            start: Starting cell
            goal: Goal cell
            
        Returns:
            Optional[List[Cell]]: Path as list of cells, or None if no path
        """
        print(f"[PathPlanner] Starting A* from Cell({start.x}, {start.y}) to Cell({goal.x}, {goal.y})")
        
        # Priority queue: (f_cost, (x, y)) - use tuple for hashability
        open_set = [(0.0, (start.x, start.y))]
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        
        # Cost from start to each cell (using tuple keys)
        g_score: Dict[Tuple[int, int], float] = {(start.x, start.y): 0}
        
        # Estimated total cost from start to goal through each cell
        f_score: Dict[Tuple[int, int], float] = {(start.x, start.y): self._heuristic(start, goal)}
        
        # Set of evaluated cells
        closed_set: Set[Tuple[int, int]] = set()
        
        iterations = 0
        
        while open_set:
            iterations += 1
            if iterations > 10000:  # Prevent infinite loops
                print(f"[PathPlanner] A* exceeded max iterations!")
                return None
            
            # Get cell with lowest f_score
            current_f, current_tuple = heapq.heappop(open_set)
            current = Cell(current_tuple[0], current_tuple[1])
            
            if current_tuple in closed_set:
                continue
                
            closed_set.add(current_tuple)
            
            # Check if we reached the goal
            if current == goal:
                print(f"[PathPlanner] A* found goal in {iterations} iterations")
                return self._reconstruct_path_from_tuples(came_from, current_tuple)
            
            # Explore neighbors
            neighbors = self.coordinate_system.get_neighbor_cells(current)
            
            for neighbor in neighbors:
                neighbor_tuple = (neighbor.x, neighbor.y)
                
                if neighbor_tuple in closed_set:
                    continue
                
                # Skip if neighbor is blocked
                with self._obstacles_lock:
                    if neighbor in self.static_obstacles:
                        continue
                
                # Calculate tentative g_score
                tentative_g = g_score[current_tuple] + self.movement_cost
                
                # Check if this path to neighbor is better than previous
                if neighbor_tuple not in g_score or tentative_g < g_score[neighbor_tuple]:
                    came_from[neighbor_tuple] = current_tuple
                    g_score[neighbor_tuple] = tentative_g
                    f_score[neighbor_tuple] = tentative_g + self._heuristic(neighbor, goal)
                    
                    # Add to open set
                    heapq.heappush(open_set, (f_score[neighbor_tuple], neighbor_tuple))
        
        print(f"[PathPlanner] A* failed to find path after {iterations} iterations")
        # No path found
        return None
    
    def _heuristic(self, cell1: Cell, cell2: Cell) -> float:
        """
        Heuristic function for A* (Manhattan distance).
        
        Args:
            cell1: First cell
            cell2: Second cell
            
        Returns:
            float: Heuristic cost estimate
        """
        return self.coordinate_system.distance_between_cells(cell1, cell2)
    
    def _reconstruct_path_from_tuples(self, came_from: Dict[Tuple[int, int], Tuple[int, int]], 
                                     current_tuple: Tuple[int, int]) -> List[Cell]:
        """
        Reconstruct path from came_from dictionary using tuple keys.
        
        Args:
            came_from: Dictionary mapping each cell to its predecessor
            current_tuple: Current cell position as tuple
            
        Returns:
            List[Cell]: Path from start to goal
        """
        path = []
        current = current_tuple
        
        while current is not None:
            path.append(Cell(current[0], current[1]))
            current = came_from.get(current)
        
        path.reverse()
        return path
    
    def _reconstruct_path(self, came_from: Dict[Cell, Cell], current: Cell) -> List[Cell]:
        """
        Reconstruct path from came_from dictionary.
        
        Args:
            came_from: Dictionary mapping each cell to its predecessor
            current: Current cell
            
        Returns:
            List[Cell]: Path from start to goal
        """
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)  # Add the start cell
        
        path.reverse()
        return path
    
    def _calculate_path_distance(self, world_path: List[Tuple[float, float]]) -> float:
        """Calculate total distance of path in world coordinates."""
        if len(world_path) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(world_path)):
            dx = world_path[i][0] - world_path[i-1][0]
            dy = world_path[i][1] - world_path[i-1][1]
            total_distance += (dx*dx + dy*dy)**0.5
        
        return total_distance
    
    def _estimate_travel_time(self, distance: float, speed: float = 0.5) -> float:
        """
        Estimate travel time based on distance and robot speed.
        
        Args:
            distance: Path distance in meters
            speed: Robot speed in m/s
            
        Returns:
            float: Estimated time in seconds
        """
        return distance / speed if speed > 0 else float('inf')
    
    # Obstacle management methods
    def add_static_obstacle(self, cell: Cell) -> None:
        """Add a static obstacle. **Thread-safe**."""
        with self._obstacles_lock:
            self.static_obstacles.add(cell)
    
    def remove_static_obstacle(self, cell: Cell) -> None:
        """Remove a static obstacle. **Thread-safe**."""
        with self._obstacles_lock:
            self.static_obstacles.discard(cell)
    
    def set_static_obstacles(self, obstacles: Set[Cell]) -> None:
        """Set all static obstacles. **Thread-safe**."""
        with self._obstacles_lock:
            self.static_obstacles = obstacles.copy()
    
    def get_static_obstacles(self) -> Set[Cell]:
        """Get copy of static obstacles. **Thread-safe**."""
        with self._obstacles_lock:
            return self.static_obstacles.copy()
    
    def clear_cache(self) -> None:
        """Clear performance cache. **Thread-safe**."""
        with self._cache_lock:
            self._distance_cache.clear() 