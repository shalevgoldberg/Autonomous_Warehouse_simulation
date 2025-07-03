"""
Graph-based path planner implementation using NavigationGraph.

This implementation uses a pre-generated NavigationGraph from the GraphGenerator
to plan routes between points in the warehouse.
"""
import heapq
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple
from interfaces.path_planner_interface import IPathPlanner
from interfaces.navigation_types import Point, LaneDirection, LaneRec, Route, RouteSegment, TaskType, RoutePlanningError
from interfaces.graph_generator_interface import NavigationGraph
from warehouse.impl.graph_generator_impl import GraphGeneratorImpl

WAREHOUSE_CSV = Path("sample_warehouse.csv")

class PathPlannerGraphImpl(IPathPlanner):
    """
    Graph-based path planner using NavigationGraph.
    
    Uses a pre-generated NavigationGraph to plan routes between points.
    The graph includes both lanes and conflict boxes as drivable nodes.
    """
    def __init__(self, graph: Optional[NavigationGraph] = None):
        if graph is None:
            # Generate graph from warehouse CSV file
            generator = GraphGeneratorImpl()
            self.graph = generator.generate_graph(WAREHOUSE_CSV)
        else:
            self.graph = graph
        
        self.blocked_nodes: Dict[str, float] = {}  # node_id -> unblock_time

    def plan_route(self, start: Point, goal: Point, task_type: TaskType) -> Route:
        """Plan a route from start to goal using the navigation graph."""
        # Find closest nodes to start and goal
        start_node_id = self._find_closest_node(start)
        goal_node_id = self._find_closest_node(goal)
        
        if start_node_id is None or goal_node_id is None:
            raise RoutePlanningError("No drivable node found near start or goal")
        
        # Use Dijkstra to find shortest path
        path_node_ids = self._dijkstra(start_node_id, goal_node_id)
        
        if not path_node_ids or len(path_node_ids) < 2:
            raise RoutePlanningError(f"No valid path found from {start} to {goal}")
        
        # Convert path to RouteSegments
        segments = []
        total_distance = 0.0
        
        for i in range(1, len(path_node_ids)):
            from_node_id = path_node_ids[i-1]
            to_node_id = path_node_ids[i]
            
            from_node = self.graph.nodes[from_node_id]
            to_node = self.graph.nodes[to_node_id]
            
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
        
        # Estimate time based on distance and speed
        estimated_time = total_distance / 0.5 if total_distance > 0 else 0.0
        
        return Route(
            segments=segments,
            total_distance=total_distance,
            estimated_time=estimated_time
        )
    
    def _find_closest_node(self, point: Point) -> Optional[str]:
        """Find the closest node to the given point."""
        min_distance = float('inf')
        closest_node_id = None
        
        for node_id, node in self.graph.nodes.items():
            dx = point.x - node.position.x
            dy = point.y - node.position.y
            distance = (dx * dx + dy * dy) ** 0.5
            
            if distance < min_distance:
                min_distance = distance
                closest_node_id = node_id
        
        return closest_node_id
    
    def _dijkstra(self, start_node_id: str, goal_node_id: str) -> List[str]:
        """Use Dijkstra's algorithm to find shortest path between nodes."""
        # Priority queue: (cost, node_id, path)
        queue = [(0.0, start_node_id, [start_node_id])]
        visited = set()
        
        while queue:
            cost, current_node_id, path = heapq.heappop(queue)
            
            if current_node_id == goal_node_id:
                return path
            
            if current_node_id in visited:
                continue
            
            visited.add(current_node_id)
            
            # Explore neighbors
            for neighbor_node_id in self.graph.edges.get(current_node_id, set()):
                if neighbor_node_id not in visited and not self._is_node_blocked(neighbor_node_id):
                    # Calculate edge cost
                    current_node = self.graph.nodes[current_node_id]
                    neighbor_node = self.graph.nodes[neighbor_node_id]
                    
                    dx = neighbor_node.position.x - current_node.position.x
                    dy = neighbor_node.position.y - current_node.position.y
                    edge_cost = (dx * dx + dy * dy) ** 0.5
                    
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
    
    def _is_node_blocked(self, node_id: str) -> bool:
        """Check if a node is currently blocked."""
        # For now, no blocking logic - could be extended later
        return False
    
    def update_block(self, cell_id: str, unblock_time: float) -> None:
        """Update blocked node information."""
        self.blocked_nodes[cell_id] = unblock_time
    
    def get_blocked_cells(self) -> Dict[str, float]:
        """Get current blocked nodes information."""
        return self.blocked_nodes.copy()
    
    def get_graph_statistics(self) -> Dict[str, int]:
        """Get statistics about the navigation graph."""
        generator = GraphGeneratorImpl()
        return generator.get_graph_statistics(self.graph)
    
    def validate_graph(self) -> List[str]:
        """Validate the navigation graph connectivity."""
        generator = GraphGeneratorImpl()
        return generator.validate_connectivity(self.graph)
    
    # Legacy methods for backward compatibility
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """Legacy method - use plan_route instead."""
        raise NotImplementedError("Use plan_route for graph-based planning")
    
    def is_path_blocked(self, path):
        """Legacy method - blocking logic handled in Dijkstra."""
        return False 