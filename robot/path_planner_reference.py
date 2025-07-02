"""
Path Planner Module - Intelligent Robot Navigation

This module handles path planning for robot navigation, including A* pathfinding,
shelf selection optimization, and route optimization. It provides clean interfaces
for the Robot Agent to request paths and shelf selections.
"""

import heapq
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from warehouse.map import WarehouseMap

class PathPlanner:
    """
    Advanced path planning system for robot navigation.
    
    This class implements A* pathfinding with Manhattan-style movement,
    shelf selection optimization, and intelligent route planning to
    minimize robot travel time and optimize warehouse operations.
    """
    
    def __init__(self, warehouse_map: WarehouseMap):
        """
        Initialize the path planner.
        
        Args:
            warehouse_map: Warehouse map containing layout information
        """
        self.warehouse_map = warehouse_map
        
        # Planning parameters
        self.movement_cost = 1.0  # Cost per grid cell
        self.turn_cost = 0.1     # Additional cost for direction changes
        self.debug_enabled = True
        
        # Cache for frequently used calculations
        self._distance_cache: Dict[Tuple[Tuple[int, int], Tuple[int, int]], float] = {}
        
    def plan_path_to_position(self, start: Tuple[float, float], 
                            goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan optimal path from start to goal position.
        
        Args:
            start: Starting position in world coordinates (x, y)
            goal: Goal position in world coordinates (x, y)
            
        Returns:
            List of waypoints in world coordinates, or None if no path found
        """
        if self.debug_enabled:
            print(f"[PathPlanner] Planning path from {start} to {goal}")
        
        # Convert to grid coordinates
        start_grid = self.warehouse_map.world_to_grid(*start)
        goal_grid = self.warehouse_map.world_to_grid(*goal)
        
        if self.debug_enabled:
            print(f"[PathPlanner] Grid coordinates: {start_grid} -> {goal_grid}")
        
        # Validate positions
        start_valid = self._is_valid_position(*start)
        goal_valid = self._is_valid_position(*goal)
        
        if self.debug_enabled:
            print(f"[PathPlanner] Start valid: {start_valid}, Goal valid: {goal_valid}")
        
        # If goal is not walkable (e.g., it's a shelf), find an approach position
        actual_goal = goal
        if not goal_valid:
            if self.debug_enabled:
                print(f"[PathPlanner] Goal position {goal} is not walkable, finding approach position")
            approach_pos = self._find_approach_position(goal)
            if approach_pos:
                actual_goal = approach_pos
                goal_valid = True
                if self.debug_enabled:
                    print(f"[PathPlanner] Found approach position: {actual_goal}")
            else:
                if self.debug_enabled:
                    print(f"[PathPlanner] No valid approach position found for {goal}")
        
        if not start_valid or not goal_valid:
            if self.debug_enabled:
                print("[PathPlanner] Invalid start or goal position")
                if not start_valid:
                    print(f"[PathPlanner] Start position {start} is not walkable")
                if not goal_valid:
                    print(f"[PathPlanner] Goal position {goal} is not walkable")
            return None
        
        # Convert to grid coordinates using actual goal
        goal_grid = self.warehouse_map.world_to_grid(*actual_goal)
        
        # Find grid path using A*
        if self.debug_enabled:
            print(f"[PathPlanner] Starting A* search from {start_grid} to {goal_grid}")
            
        grid_path = self._astar_search(start_grid, goal_grid)
        if not grid_path:
            if self.debug_enabled:
                print("[PathPlanner] A* search failed - no path found")
            return None
        
        if self.debug_enabled:
            print(f"[PathPlanner] A* found grid path with {len(grid_path)} waypoints")
        
        # Convert to world coordinates and optimize
        world_path = self._create_optimized_path(grid_path, start, goal)
        
        if self.debug_enabled:
            print(f"[PathPlanner] Generated path with {len(world_path)} waypoints")
        
        return world_path
    
    def find_closest_accessible_shelf(self, robot_position: Tuple[float, float],
                                    shelf_options: List[Tuple[str, Tuple[float, float]]],
                                    sim_data_service = None) -> Optional[Tuple[str, Tuple[float, float], float]]:
        """
        Find the closest accessible shelf from available options using precomputed approach cells.
        
        Args:
            robot_position: Current robot position (x, y)
            shelf_options: List of (shelf_id, shelf_position) tuples
            sim_data_service: Optional SimDataService instance to get precomputed approach cells
            
        Returns:
            Tuple of (shelf_id, approach_position, estimated_time) or None
        """
        if not shelf_options:
            return None
        
        if self.debug_enabled:
            print(f"[PathPlanner] Evaluating {len(shelf_options)} shelf options")
        
        best_shelf = None
        min_eta = float('inf')
        
        for shelf_id, shelf_position in shelf_options:
            # Try to get precomputed approach cell if service is available
            target_position = shelf_position  # Default to shelf position
            
            if sim_data_service is not None:
                try:
                    closest_approach = sim_data_service.get_closest_approach_cell(shelf_id, robot_position)
                    if closest_approach is not None:
                        target_position = closest_approach
                        if self.debug_enabled:
                            print(f"[PathPlanner] Using precomputed approach cell {closest_approach} for shelf {shelf_id}")
                    else:
                        if self.debug_enabled:
                            print(f"[PathPlanner] No approach cells found for shelf {shelf_id}, using fallback")
                        # Fallback: use the old approach position calculation
                        approach_pos = self._find_approach_position(shelf_position)
                        if approach_pos is not None:
                            target_position = approach_pos
                except Exception as e:
                    if self.debug_enabled:
                        print(f"[PathPlanner] Error getting approach cells for shelf {shelf_id}: {e}")
                    # Fallback: use the old approach position calculation
                    approach_pos = self._find_approach_position(shelf_position)
                    if approach_pos is not None:
                        target_position = approach_pos
            else:
                # Fallback: use the old approach position calculation
                approach_pos = self._find_approach_position(shelf_position)
                if approach_pos is not None:
                    target_position = approach_pos
            
            # Calculate estimated travel time to the target position
            eta = self._calculate_travel_time(robot_position, target_position)
            
            if self.debug_enabled:
                print(f"[PathPlanner] Shelf {shelf_id}: Target at {target_position}, ETA = {eta:.2f}s")
            
            if eta < min_eta:
                min_eta = eta
                best_shelf = (shelf_id, target_position, eta)
        
        if self.debug_enabled and best_shelf:
            print(f"[PathPlanner] Selected shelf {best_shelf[0]} with target at {best_shelf[1]}, ETA {best_shelf[2]:.2f}s")
        
        return best_shelf
    
    def plan_multi_stop_route(self, start: Tuple[float, float],
                            waypoints: List[Tuple[float, float]],
                            end: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan route through multiple waypoints with optimization.
        
        Args:
            start: Starting position
            waypoints: List of intermediate positions to visit
            end: Final destination
            
        Returns:
            Complete optimized route or None if not possible
        """
        if not waypoints:
            return self.plan_path_to_position(start, end)
        
        complete_path = []
        current_pos = start
        
        # Plan path through each waypoint
        for waypoint in waypoints:
            segment = self.plan_path_to_position(current_pos, waypoint)
            if not segment:
                if self.debug_enabled:
                    print(f"[PathPlanner] Failed to plan segment to {waypoint}")
                return None
            
            # Add segment, avoiding duplicate waypoints
            if complete_path:
                complete_path.extend(segment[1:])  # Skip first point to avoid duplication
            else:
                complete_path.extend(segment)
            
            current_pos = waypoint
        
        # Plan final segment to end
        final_segment = self.plan_path_to_position(current_pos, end)
        if not final_segment:
            if self.debug_enabled:
                print(f"[PathPlanner] Failed to plan final segment to {end}")
            return None
        
        complete_path.extend(final_segment[1:])  # Skip first point
        
        if self.debug_enabled:
            print(f"[PathPlanner] Multi-stop route planned with {len(complete_path)} total waypoints")
        
        return complete_path
    
    def _calculate_travel_time(self, start: Tuple[float, float], 
                             end: Tuple[float, float]) -> float:
        """
        Calculate estimated travel time between two positions.
        
        Args:
            start: Starting position
            end: Ending position
            
        Returns:
            Estimated travel time in seconds
        """
        # Use cached distance if available
        start_grid = self.warehouse_map.world_to_grid(*start)
        end_grid = self.warehouse_map.world_to_grid(*end)
        cache_key = (start_grid, end_grid)
        
        if cache_key in self._distance_cache:
            distance = self._distance_cache[cache_key]
        else:
            # Plan actual path to get accurate distance
            path = self.plan_path_to_position(start, end)
            if path:
                distance = self._calculate_path_length(path)
                self._distance_cache[cache_key] = distance
            else:
                # Fallback to Manhattan distance
                distance = abs(start[0] - end[0]) + abs(start[1] - end[1])
        
        # Estimate time based on average robot speed (1.5 units/second)
        average_speed = 1.5
        return distance / average_speed
    
    def _calculate_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total length of a path."""
        if len(path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_length += np.sqrt(dx*dx + dy*dy)
        
        return total_length
    
    def _astar_search(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        A* pathfinding algorithm on grid coordinates.
        
        Args:
            start: Start position in grid coordinates
            goal: Goal position in grid coordinates
            
        Returns:
            Path as list of grid coordinates or None if no path found
        """
        open_set = []
        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        heapq.heappush(open_set, (f_score[start], start))
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            closed_set.add(current)
            
            # Explore neighbors (4-directional movement)
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if neighbor in closed_set:
                    continue
                
                # Check if neighbor is valid and walkable
                neighbor_world = self.warehouse_map.grid_to_world(*neighbor)
                if not self._is_valid_position(*neighbor_world):
                    continue
                
                # Calculate movement cost
                tentative_g = g_score[current] + self.movement_cost
                
                # Add turn cost if direction changed
                if current in came_from:
                    prev_pos = came_from[current]
                    prev_dir = (current[0] - prev_pos[0], current[1] - prev_pos[1])
                    curr_dir = (neighbor[0] - current[0], neighbor[1] - current[1])
                    if prev_dir != curr_dir:
                        tentative_g += self.turn_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None
    
    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic for A* search."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from A* search results."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        path.reverse()
        return path
    
    def _create_optimized_path(self, grid_path: List[Tuple[int, int]], 
                             start: Tuple[float, float],
                             goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Convert grid path to optimized world coordinate path.
        
        Args:
            grid_path: Path in grid coordinates
            start: Original start position in world coordinates
            goal: Original goal position in world coordinates
            
        Returns:
            Optimized path in world coordinates
        """
        if not grid_path:
            return [goal]
        
        # Convert grid path to world coordinates
        world_path = [self.warehouse_map.grid_to_world(*pos) for pos in grid_path]
        
        # Simplify path by removing unnecessary waypoints
        simplified_path = self._simplify_path(world_path)
        
        # Ensure we start and end at exact requested positions
        if simplified_path:
            simplified_path[0] = start
            simplified_path[-1] = goal
        
        return simplified_path
    
    def _simplify_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Simplify path by removing redundant waypoints while maintaining valid movement.
        
        Args:
            path: Original path
            
        Returns:
            Simplified path with only necessary waypoints
        """
        if len(path) <= 2:
            return path
        
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev_point = simplified[-1]
            current_point = path[i]
            next_point = path[i + 1]
            
            # Check if we need this waypoint (direction change)
            if self._is_direction_change(prev_point, current_point, next_point):
                simplified.append(current_point)
        
        simplified.append(path[-1])
        return simplified
    
    def _is_direction_change(self, p1: Tuple[float, float], 
                           p2: Tuple[float, float], 
                           p3: Tuple[float, float]) -> bool:
        """Check if there's a direction change at p2."""
        # Calculate direction vectors
        dir1 = (p2[0] - p1[0], p2[1] - p1[1])
        dir2 = (p3[0] - p2[0], p3[1] - p2[1])
        
        # Normalize directions to handle floating point precision
        def normalize_dir(d):
            if abs(d[0]) > abs(d[1]):
                return (1 if d[0] > 0 else -1, 0)
            else:
                return (0, 1 if d[1] > 0 else -1)
        
        return normalize_dir(dir1) != normalize_dir(dir2)
    
    def _is_valid_position(self, x: float, y: float) -> bool:
        """Check if a position is valid and walkable."""
        return self.warehouse_map.is_walkable(x, y)
    
    def _find_approach_position(self, shelf_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Find a walkable approach position adjacent to a shelf.
        
        Args:
            shelf_pos: Position of the shelf (x, y)
            
        Returns:
            Adjacent walkable position or None if none found
        """
        x, y = shelf_pos
        grid_size = self.warehouse_map.grid_size
        
        # Try positions adjacent to the shelf (4-directional)
        adjacent_offsets = [
            (0, grid_size),      # North
            (grid_size, 0),      # East  
            (0, -grid_size),     # South
            (-grid_size, 0),     # West
        ]
        
        for dx, dy in adjacent_offsets:
            candidate_pos = (x + dx, y + dy)
            if self._is_valid_position(*candidate_pos):
                if self.debug_enabled:
                    print(f"[PathPlanner] Found approach position {candidate_pos} for shelf at {shelf_pos}")
                return candidate_pos
        
        if self.debug_enabled:
            print(f"[PathPlanner] No adjacent approach position found for shelf at {shelf_pos}")
        
        return None
    
    def get_planning_statistics(self) -> Dict[str, Any]:
        """Get path planning statistics."""
        return {
            'cache_size': len(self._distance_cache),
            'movement_cost': self.movement_cost,
            'turn_cost': self.turn_cost,
            'debug_enabled': self.debug_enabled
        }
    
    def clear_cache(self) -> None:
        """Clear the distance calculation cache."""
        self._distance_cache.clear()
    
    def set_debug_mode(self, enabled: bool) -> None:
        """Enable or disable debug output."""
        self.debug_enabled = enabled 