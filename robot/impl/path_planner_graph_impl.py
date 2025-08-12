"""
Graph-based path planner implementation using NavigationGraph.

This implementation uses NavigationGraph from SimulationDataService to plan routes 
between points in the warehouse with dynamic blocked cell support.
"""
import heapq
import time
import logging
from typing import Dict, List, Optional, Set, Tuple
from interfaces.path_planner_interface import IPathPlanner
from interfaces.navigation_types import Point, LaneDirection, LaneRec, Route, RouteSegment, TaskType, PathPlanningResult, RoutePlanningError
from interfaces.simulation_data_service_interface import ISimulationDataService, NavigationGraph

# Configure logging
logger = logging.getLogger(__name__)

class PathPlannerGraphImpl(IPathPlanner):
    """
    Graph-based path planner using NavigationGraph from SimulationDataService.
    
    This implementation follows SOLID principles:
    - Single Responsibility: Focus only on path planning
    - Dependency Inversion: Depends on ISimulationDataService abstraction
    - Open/Closed: Extensible through interface implementations
    
    Uses a NavigationGraph from SimulationDataService to plan routes between points.
    Supports dynamic blocked cells and conflict box awareness.
    Returns PathPlanningResult for graceful failure handling.
    """
    
    def __init__(self, simulation_data_service: ISimulationDataService):
        """
        Initialize path planner with dependency injection.
        
        Args:
            simulation_data_service: Service for navigation data and blocked cells
        """
        self.simulation_data_service = simulation_data_service
        self._graph_cache: Optional[NavigationGraph] = None
        self._cache_timestamp = 0
        self._cache_ttl = 60  # Cache graph for 60 seconds
        self._blocked_cells_cache: Dict[str, float] = {}
        self._blocked_cache_timestamp = 0
        self._blocked_cache_ttl = 5  # Cache blocked cells for 5 seconds
        
        logger.debug("PathPlannerGraphImpl initialized with SimulationDataService")

    def _get_navigation_graph(self) -> Optional[NavigationGraph]:
        """Get navigation graph with caching for performance."""
        current_time = time.time()
        
        if (self._graph_cache is None or 
            current_time - self._cache_timestamp > self._cache_ttl):
            try:
                self._graph_cache = self.simulation_data_service.get_navigation_graph()
                self._cache_timestamp = current_time
                logger.debug("Navigation graph refreshed from SimulationDataService")
            except Exception as e:
                logger.error(f"Failed to load navigation graph: {e}")
                if self._graph_cache is None:
                    # Return failure result instead of raising exception
                    return None
                # Use cached version if available
                logger.warning("Using cached navigation graph due to load failure")
        
        return self._graph_cache

    def _get_blocked_cells(self) -> Dict[str, float]:
        """Get blocked cells with caching for performance."""
        current_time = time.time()
        
        if (current_time - self._blocked_cache_timestamp > self._blocked_cache_ttl):
            try:
                self._blocked_cells_cache = self.simulation_data_service.get_blocked_cells()
                self._blocked_cache_timestamp = current_time
                logger.debug(f"Blocked cells refreshed: {len(self._blocked_cells_cache)} active blocks")
            except Exception as e:
                logger.error(f"Failed to load blocked cells: {e}")
                # Continue with cached version
        
        return self._blocked_cells_cache

    def plan_route(self, start: Point, goal: Point, task_type: TaskType) -> PathPlanningResult:
        """
        Plan a route from start to goal using the navigation graph.
        
        This method returns a PathPlanningResult that handles both success and failure
        cases gracefully, allowing the robot system to handle path planning failures
        without crashing.
        
        Args:
            start: Starting position
            goal: Goal position  
            task_type: Type of task (affects routing decisions)
            
        Returns:
            PathPlanningResult: Result containing either a successful route or failure information
        """
        try:
            graph = self._get_navigation_graph()
            
            # Check if graph loading failed
            if graph is None:
                return PathPlanningResult.failure_result(
                    error_message="Cannot load navigation graph",
                    failure_reason="database_error",
                    retry_after=30.0  # Retry after 30 seconds
                )
            
            # Find closest nodes to start and goal
            start_node_id = self._find_closest_node(start, graph)
            goal_node_id = self._find_closest_node(goal, graph)
            
            if start_node_id is None:
                return PathPlanningResult.failure_result(
                    error_message=f"No drivable node found near start position {start}",
                    failure_reason="start_position_unreachable",
                    retry_after=None  # No retry - position is unreachable
                )
            
            if goal_node_id is None:
                return PathPlanningResult.failure_result(
                    error_message=f"No drivable node found near goal position {goal}",
                    failure_reason="goal_position_unreachable",
                    retry_after=None  # No retry - position is unreachable
                )
            
            logger.debug(f"Planning route from node {start_node_id} to {goal_node_id}")
            
            # Use Dijkstra to find shortest path (forbid idle nodes as intermediates)
            path_node_ids = self._dijkstra(start_node_id, goal_node_id, graph)
            
            if not path_node_ids or len(path_node_ids) < 2:
                # Check if it's due to blocked cells
                blocked_cells = self._get_blocked_cells()
                if blocked_cells:
                    # Suggest retry after some time for blocked cells
                    return PathPlanningResult.failure_result(
                        error_message=f"No valid path found from {start} to {goal}",
                        failure_reason="path_blocked_by_obstacles",
                        retry_after=10.0  # Retry after 10 seconds
                    )
                else:
                    # No path exists even without blocks
                    return PathPlanningResult.failure_result(
                        error_message=f"No valid path found from {start} to {goal}",
                        failure_reason="no_path_exists",
                        retry_after=None  # No retry - no path exists
                    )
            
            # Convert path to RouteSegments
            route = self._create_route_from_path(path_node_ids, graph)
            
            logger.debug(f"Route planned successfully: {len(route.segments)} segments, "
                        f"{route.total_distance:.2f}m, {route.estimated_time:.2f}s")
            return PathPlanningResult.success_result(route)
            
        except Exception as e:
            logger.error(f"Unexpected error during route planning: {e}")
            return PathPlanningResult.failure_result(
                error_message=f"Route planning failed: {e}",
                failure_reason="unexpected_error",
                retry_after=5.0  # Retry after 5 seconds for unexpected errors
            )

    def _create_route_from_path(self, path_node_ids: List[str], graph: NavigationGraph) -> Route:
        """Create a Route object from a list of node IDs."""
        segments = []
        total_distance = 0.0
        conflict_boxes = []
        
        for i in range(1, len(path_node_ids)):
            from_node_id = path_node_ids[i-1]
            to_node_id = path_node_ids[i]
            
            from_node = graph.nodes[from_node_id]
            to_node = graph.nodes[to_node_id]
            
            # Calculate segment distance
            dx = to_node.position.x - from_node.position.x
            dy = to_node.position.y - from_node.position.y
            segment_distance = (dx * dx + dy * dy) ** 0.5
            
            # Determine direction from from_node to to_node
            direction = self._get_direction_between_nodes(from_node.position, to_node.position)
            
            # Create lane record
            lane = LaneRec(
                lane_id=f"{from_node_id}->{to_node_id}",
                direction=direction,
                waypoints=[from_node.position, to_node.position]
            )
            
            # Create route segment
            segment = RouteSegment(
                lane=lane,
                start_point=from_node.position,
                end_point=to_node.position
            )
            
            segments.append(segment)
            total_distance += segment_distance
            
            # Collect conflict boxes along the route
            if from_node.is_conflict_box and from_node.conflict_box_id:
                box = graph.conflict_boxes.get(from_node.conflict_box_id)
                if box and box not in conflict_boxes:
                    conflict_boxes.append(box)
            
            if to_node.is_conflict_box and to_node.conflict_box_id:
                box = graph.conflict_boxes.get(to_node.conflict_box_id)
                if box and box not in conflict_boxes:
                    conflict_boxes.append(box)
        
        # Estimate time based on distance and average speed
        average_speed = 0.5  # m/s
        estimated_time = total_distance / average_speed if total_distance > 0 else 0.0
        
        return Route(
            segments=segments,
            total_distance=total_distance,
            estimated_time=estimated_time,
            conflict_boxes=conflict_boxes
        )
    
    def _find_closest_node(self, point: Point, graph: NavigationGraph) -> Optional[str]:
        """Find the closest node to the given point."""
        min_distance = float('inf')
        closest_node_id = None
        
        for node_id, node in graph.nodes.items():
            dx = point.x - node.position.x
            dy = point.y - node.position.y
            distance = (dx * dx + dy * dy) ** 0.5
            
            if distance < min_distance:
                min_distance = distance
                closest_node_id = node_id
        
        return closest_node_id
    
    def _dijkstra(self, start_node_id: str, goal_node_id: str, graph: NavigationGraph) -> List[str]:
        """Use Dijkstra's algorithm to find shortest path between nodes."""
        # Priority queue: (cost, node_id, path)
        queue = [(0.0, start_node_id, [start_node_id])]
        visited = set()
        blocked_cells = self._get_blocked_cells()
        
        while queue:
            cost, current_node_id, path = heapq.heappop(queue)
            
            if current_node_id == goal_node_id:
                return path
            
            if current_node_id in visited:
                continue
            
            visited.add(current_node_id)
            
            # Explore neighbors
            neighbors = graph.get_neighbors(current_node_id)
            for neighbor_node_id in neighbors:
                if (neighbor_node_id not in visited and 
                    not self._is_node_blocked(neighbor_node_id, blocked_cells)):
                    
                    # Calculate edge cost
                    current_node = graph.nodes[current_node_id]
                    neighbor_node = graph.nodes[neighbor_node_id]
                    
                    dx = neighbor_node.position.x - current_node.position.x
                    dy = neighbor_node.position.y - current_node.position.y
                    edge_cost = (dx * dx + dy * dy) ** 0.5
                    
                    # Forbid idle/charging nodes as intermediates: allow only if neighbor is goal
                    if (neighbor_node_id.startswith('idle_') or neighbor_node_id.startswith('charge_')) \
                        and neighbor_node_id != goal_node_id and current_node_id != start_node_id:
                        continue

                    # Add penalty for conflict boxes to encourage alternative routes
                    if neighbor_node.is_conflict_box:
                        edge_cost *= 1.2  # 20% penalty for conflict boxes
                    
                    heapq.heappush(queue, (cost + edge_cost, neighbor_node_id, path + [neighbor_node_id]))
        
        return []  # No path found
    
    def _get_direction_between_nodes(self, from_pos: Point, to_pos: Point) -> LaneDirection:
        """Determine the direction from one position to another."""
        dx = to_pos.x - from_pos.x
        dy = to_pos.y - from_pos.y
        
        if abs(dx) > abs(dy):
            return LaneDirection.EAST if dx > 0 else LaneDirection.WEST
        else:
            return LaneDirection.NORTH if dy > 0 else LaneDirection.SOUTH
    
    def _is_node_blocked(self, node_id: str, blocked_cells: Dict[str, float]) -> bool:
        """Check if a node is currently blocked."""
        if node_id in blocked_cells:
            unblock_time = blocked_cells[node_id]
            current_time = time.time()
            return current_time < unblock_time
        return False
    
    def update_block(self, cell_id: str, unblock_time: float) -> None:
        """
        Update blocked node information via SimulationDataService.
        
        Args:
            cell_id: Node/cell identifier to block
            unblock_time: Unix timestamp when cell will be unblocked
        """
        try:
            success = self.simulation_data_service.report_blocked_cell(
                cell_id=cell_id,
                robot_id="path_planner",  # Default robot ID for path planner blocks
                unblock_time=unblock_time,
                reason="path_planning_block"
            )
            
            if success:
                # Invalidate blocked cells cache to force refresh
                self._blocked_cache_timestamp = 0
                logger.debug(f"Reported blocked cell {cell_id} until {unblock_time}")
            else:
                logger.warning(f"Failed to report blocked cell {cell_id}")
                
        except Exception as e:
            logger.error(f"Failed to update block for cell {cell_id}: {e}")
    
    def get_blocked_cells(self) -> Dict[str, float]:
        """Get current blocked nodes information."""
        try:
            return self._get_blocked_cells()
        except Exception as e:
            logger.error(f"Failed to get blocked cells: {e}")
            return {}
    
    # Legacy methods for backward compatibility
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """Legacy method - use plan_route instead."""
        start_point = Point(x=start[0], y=start[1])
        goal_point = Point(x=goal[0], y=goal[1])
        
        result = self.plan_route(start_point, goal_point, TaskType.PICK_AND_DELIVER)
        
        if not result.success or result.route is None:
            # For legacy compatibility, raise exception on failure
            raise RoutePlanningError(result.error_message or "Path planning failed")
        
        # Convert route to legacy path format
        path = []
        for segment in result.route.segments:
            path.extend([(wp.x, wp.y) for wp in segment.lane.waypoints])
        
        return path
    
    def is_path_blocked(self, path) -> bool:
        """Legacy method - blocking logic handled in Dijkstra."""
        return False 