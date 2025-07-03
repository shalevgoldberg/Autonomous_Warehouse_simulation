"""
Implementation of Graph Generator - builds navigation graphs from warehouse CSV data.

This implementation processes the original warehouse CSV format where each cell
contains encoded information about its type and directions.
"""
import csv
from pathlib import Path
from typing import Dict, Set, List, Tuple, Optional
from math import sqrt

from interfaces.graph_generator_interface import (
    IGraphGenerator, NavigationGraph, GraphNode, GraphEdge, ConflictBox
)
from interfaces.navigation_types import Point, LaneDirection


class GraphGeneratorImpl(IGraphGenerator):
    """
    Implementation of graph generator for warehouse navigation.
    
    Processes the original warehouse CSV format where:
    - Each cell contains encoded type and directions (e.g., 'jen', 'lw', 'js')
    - 'j' prefix indicates conflict box cells
    - 'l' prefix indicates lane cells
    - Letters after prefix indicate available exit directions
    """
    
    def __init__(self):
        self.cell_size = 0.5  # Standard cell size for distance calculations
    
    def generate_graph(self, warehouse_csv: Path, boxes_csv: Optional[Path] = None) -> NavigationGraph:
        """Generate navigation graph from original warehouse CSV format."""
        if not warehouse_csv.exists():
            raise FileNotFoundError(f"Warehouse CSV not found: {warehouse_csv}")
        
        nodes: Dict[str, GraphNode] = {}
        edges: Dict[str, Set[str]] = {}
        conflict_boxes: Dict[str, ConflictBox] = {}
        position_to_node: Dict[Tuple[float, float], str] = {}
        
        # Load warehouse grid and create nodes
        self._load_warehouse_grid(warehouse_csv, nodes, position_to_node, conflict_boxes)
        
        # Build connections based on exit directions
        self._build_connections(nodes, position_to_node, edges)
        
        return NavigationGraph(
            nodes=nodes,
            edges=edges,
            conflict_boxes=conflict_boxes,
            position_to_node=position_to_node
        )
    
    def _load_warehouse_grid(self, warehouse_csv: Path, nodes: Dict[str, GraphNode],
                           position_to_node: Dict[Tuple[float, float], str],
                           conflict_boxes: Dict[str, ConflictBox]) -> None:
        """Load warehouse grid from CSV and create nodes."""
        grid = []
        
        # Read the grid
        with open(warehouse_csv, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                if row:
                    grid.append(row)
        
        # Process each cell
        conflict_box_counter = 0
        for row_idx, row in enumerate(grid):
            for col_idx, cell_value in enumerate(row):
                if cell_value in ['w', 'd', 'i', 'c']:  # walls, dock, idle, charging
                    continue
                
                # Calculate world position (assuming grid starts at origin)
                x = col_idx * self.cell_size
                y = -row_idx * self.cell_size  # Negative Y for downward rows
                pos = (x, y)
                
                # Parse cell value
                if cell_value.startswith('j'):  # Conflict box
                    directions = self._parse_directions(cell_value[1:])  # Remove 'j' prefix
                    node_id = f"box_{conflict_box_counter}"
                    
                    # Create conflict box node
                    node = GraphNode(
                        node_id=node_id,
                        position=Point(x, y),
                        directions=directions,
                        is_conflict_box=True,
                        conflict_box_id=str(conflict_box_counter)
                    )
                    nodes[node_id] = node
                    position_to_node[pos] = node_id
                    
                    # Create conflict box record
                    conflict_box = ConflictBox(
                        box_id=str(conflict_box_counter),
                        position=Point(x, y),
                        size=self.cell_size,
                        participating_nodes={node_id},
                        directions=directions
                    )
                    conflict_boxes[str(conflict_box_counter)] = conflict_box
                    conflict_box_counter += 1
                    
                elif cell_value.startswith('l'):  # Lane cell
                    directions = self._parse_directions(cell_value[1:])  # Remove 'l' prefix
                    node_id = f"lane_{row_idx}_{col_idx}"
                    
                    # Create lane node
                    node = GraphNode(
                        node_id=node_id,
                        position=Point(x, y),
                        directions=directions,
                        is_conflict_box=False
                    )
                    nodes[node_id] = node
                    position_to_node[pos] = node_id
    
    def _parse_directions(self, direction_str: str) -> Set[LaneDirection]:
        """Parse direction string into LaneDirection set."""
        directions = set()
        for char in direction_str.lower():  # Convert to lowercase for case-insensitive parsing
            if char == 'n':
                directions.add(LaneDirection.NORTH)
            elif char == 's':
                directions.add(LaneDirection.SOUTH)
            elif char == 'e':
                directions.add(LaneDirection.EAST)
            elif char == 'w':
                directions.add(LaneDirection.WEST)
        return directions
    
    def _build_connections(self, nodes: Dict[str, GraphNode],
                          position_to_node: Dict[Tuple[float, float], str],
                          edges: Dict[str, Set[str]]) -> None:
        """Build connections between nodes based on exit directions."""
        for node_id, node in nodes.items():
            edges[node_id] = set()
            
            # For each exit direction, find the target position and connect if it exists
            for direction in node.directions:
                target_pos = self._get_exit_position(node.position, direction)
                target_node_id = position_to_node.get((target_pos.x, target_pos.y))
                
                if target_node_id:
                    edges[node_id].add(target_node_id)
    
    def _get_exit_position(self, position: Point, direction: LaneDirection) -> Point:
        """Calculate the position where the exit direction leads to."""
        if direction == LaneDirection.NORTH:
            return Point(position.x, position.y + self.cell_size)
        elif direction == LaneDirection.SOUTH:
            return Point(position.x, position.y - self.cell_size)
        elif direction == LaneDirection.EAST:
            return Point(position.x + self.cell_size, position.y)
        elif direction == LaneDirection.WEST:
            return Point(position.x - self.cell_size, position.y)
        else:
            raise ValueError(f"Unknown direction: {direction}")
    
    def validate_connectivity(self, graph: NavigationGraph) -> List[str]:
        """Validate that the graph is properly connected."""
        warnings = []
        
        # Check for isolated nodes
        isolated_nodes = []
        for node_id, node in graph.nodes.items():
            if not graph.edges.get(node_id, set()) and not self._has_incoming_edges(node_id, graph.edges):
                isolated_nodes.append(node_id)
        
        if isolated_nodes:
            warnings.append(f"Found {len(isolated_nodes)} isolated nodes: {isolated_nodes[:5]}...")
        
        # Check for dead ends
        dead_ends = []
        for node_id, connections in graph.edges.items():
            if not connections:
                dead_ends.append(node_id)
        
        if dead_ends:
            warnings.append(f"Found {len(dead_ends)} dead end nodes: {dead_ends[:5]}...")
        
        # Check for missing connections based on directions
        missing_connections = []
        for node_id, node in graph.nodes.items():
            for direction in node.directions:
                target_pos = self._get_exit_position(node.position, direction)
                target_node_id = graph.position_to_node.get((target_pos.x, target_pos.y))
                
                if not target_node_id:
                    missing_connections.append(f"{node_id} -> {direction.value} -> ({target_pos.x}, {target_pos.y})")
        
        if missing_connections:
            warnings.append(f"Found {len(missing_connections)} missing target positions: {missing_connections[:3]}...")
        
        return warnings
    
    def _has_incoming_edges(self, target_node_id: str, edges: Dict[str, Set[str]]) -> bool:
        """Check if a node has incoming edges."""
        for source_node_id, connections in edges.items():
            if target_node_id in connections:
                return True
        return False
    
    def get_graph_statistics(self, graph: NavigationGraph) -> Dict[str, int]:
        """Get statistics about the generated graph."""
        total_edges = sum(len(connections) for connections in graph.edges.values())
        conflict_box_nodes = sum(1 for node in graph.nodes.values() if node.is_conflict_box)
        
        return {
            'total_nodes': len(graph.nodes),
            'total_edges': total_edges,
            'conflict_boxes': len(graph.conflict_boxes),
            'conflict_box_nodes': conflict_box_nodes,
            'lane_nodes': len(graph.nodes) - conflict_box_nodes,
            'positions': len(graph.position_to_node)
        } 