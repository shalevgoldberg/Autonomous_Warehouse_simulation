"""
Implementation of Graph Generator - builds navigation graphs from warehouse CSV data.

This implementation processes the original warehouse CSV format where each cell
contains encoded information about its type and directions.
"""
import csv
from pathlib import Path
from typing import Dict, Set, List, Tuple, Optional
from math import sqrt

from interfaces.graph_generator_interface import IGraphGenerator, ConflictBox, GraphEdge
from interfaces.simulation_data_service_interface import NavigationGraph, GraphNode
from interfaces.navigation_types import Point, LaneDirection, BoxRec


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
        edges: Dict[str, List[str]] = {}
        conflict_boxes: Dict[str, ConflictBox] = {}
        position_to_node: Dict[Tuple[float, float], str] = {}
        
        # Load warehouse grid and create nodes
        self._load_warehouse_grid(warehouse_csv, nodes, position_to_node, conflict_boxes)
        
        # Build connections based on exit directions
        self._build_connections(nodes, position_to_node, edges)
        
        # Convert conflict_boxes to BoxRec for NavigationGraph (rectangles)
        boxrec_conflict_boxes = {}
        for box_id, box in conflict_boxes.items():
            # ConflictBox now expected to expose width/height; if not, derive from size as square
            width = getattr(box, 'width', None)
            height = getattr(box, 'height', None)
            if width is None or height is None:
                # Legacy fallback during migration: treat as square of size
                print(f"[diag] _generate_graph: fallback: treating box {box_id} as square of size {box.size}")
                width = box.size
                height = box.size
                
            boxrec_conflict_boxes[box_id] = BoxRec(
                box_id=box.box_id,
                center=box.position,
                width=width,
                height=height
            )
        return NavigationGraph(
            nodes=nodes,
            edges=edges,
            conflict_boxes=boxrec_conflict_boxes,
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
        
        # Preprocess: identify auto-detected junctions (l* cells with multiple inbound sides, excluding parking)
        parking_adjacent_lanes: Set[Tuple[int, int]] = set()
        parking_cells: Set[Tuple[int, int]] = set()
        auto_junctions: Set[Tuple[int, int]] = set()
        if grid:
            # Collect parking cells (idle/charging)
            for r_idx, row in enumerate(grid):
                for c_idx, val in enumerate(row):
                    if val in ['i', '4', 'c', '3']:
                        parking_cells.add((r_idx, c_idx))

            # Helper: in-bounds check per row
            def in_bounds(rr: int, cc: int) -> bool:
                return 0 <= rr < len(grid) and 0 <= cc < len(grid[rr])

            # Helper: parse directions from a cell token (after prefix)
            def parse_dir_set(token: str) -> Set[LaneDirection]:
                return set(self._parse_directions(token))

            # Auto-detect junctions: l* cells with >=2 inbound neighbors (ignoring parking neighbors)
            for r_idx, row in enumerate(grid):
                for c_idx, val in enumerate(row):
                    if not val.startswith('l'):
                        continue
                    inbound = 0
                    # From NORTH neighbor into this cell: neighbor must have SOUTH
                    nr, nc = r_idx - 1, c_idx
                    if in_bounds(nr, nc):
                        nval = grid[nr][nc]
                        if (nr, nc) not in parking_cells and (nval.startswith('l') or nval.startswith('j')):
                            dirs = parse_dir_set(nval[1:])
                            if LaneDirection.SOUTH in dirs:
                                inbound += 1
                    # From SOUTH neighbor
                    nr, nc = r_idx + 1, c_idx
                    if in_bounds(nr, nc):
                        nval = grid[nr][nc]
                        if (nr, nc) not in parking_cells and (nval.startswith('l') or nval.startswith('j')):
                            dirs = parse_dir_set(nval[1:])
                            if LaneDirection.NORTH in dirs:
                                inbound += 1
                    # From WEST neighbor
                    nr, nc = r_idx, c_idx - 1
                    if in_bounds(nr, nc):
                        nval = grid[nr][nc]
                        if (nr, nc) not in parking_cells and (nval.startswith('l') or nval.startswith('j')):
                            dirs = parse_dir_set(nval[1:])
                            if LaneDirection.EAST in dirs:
                                inbound += 1
                    # From EAST neighbor
                    nr, nc = r_idx, c_idx + 1
                    if in_bounds(nr, nc):
                        nval = grid[nr][nc]
                        if (nr, nc) not in parking_cells and (nval.startswith('l') or nval.startswith('j')):
                            dirs = parse_dir_set(nval[1:])
                            if LaneDirection.WEST in dirs:
                                inbound += 1
                    if inbound >= 2:
                        auto_junctions.add((r_idx, c_idx))

            # Mark lane cells that are 4-neighbor adjacent to any parking cell (parking-adjacent lanes)
            for r_idx, row in enumerate(grid):
                for c_idx, val in enumerate(row):
                    if not val.startswith('l'):
                        continue
                    neighbors = [(r_idx-1, c_idx), (r_idx+1, c_idx), (r_idx, c_idx-1), (r_idx, c_idx+1)]
                    for nr, nc in neighbors:
                        if in_bounds(nr, nc) and (nr, nc) in parking_cells:
                            parking_adjacent_lanes.add((r_idx, c_idx))
                            break

        # Process each cell - first collect nodes, then group conflict boxes by connectivity
        # Temporary storage for junction (j*) cells to compute connected components
        j_cells: Dict[Tuple[int, int], Dict[str, any]] = {}
        box_node_counter = 0
        for row_idx, row in enumerate(grid):
            for col_idx, cell_value in enumerate(row):
                # Treat idle ('i'/'4') and charging ('c'/'3') as graph nodes; skip 'w' (walls)
                if cell_value in ['w']:
                    continue
                
                # Calculate world position at the CENTER of the cell (matches WarehouseMap)
                x = (col_idx + 0.5) * self.cell_size
                y = (row_idx + 0.5) * self.cell_size
                pos = (x, y)
                
                # Parse cell value
                if cell_value.startswith('j'):  # Junction cell (part of a conflict box)
                    directions = self._parse_directions(cell_value[1:])
                    node_id = f"box_{box_node_counter}"
                    box_node_counter += 1
                    # Create node now; assign final conflict_box_id after grouping
                    node = GraphNode(
                        node_id=node_id,
                        position=Point(x, y),
                        directions=directions,
                        is_conflict_box=True, # TODO TEMP!!!
                        conflict_box_id=None
                    )
                    nodes[node_id] = node
                    position_to_node[pos] = node_id
                    # Track for grouping (use grid indices for adjacency)
                    j_cells[(row_idx, col_idx)] = {
                        'node_id': node_id,
                        'row': row_idx,
                        'col': col_idx,
                        'position': Point(x, y),
                        'directions': directions,
                    }
                elif cell_value.startswith('l'):  # Lane cell
                    directions = self._parse_directions(cell_value[1:])  # Remove 'l' prefix
                    is_parking_adjacent = (row_idx, col_idx) in parking_adjacent_lanes

                    # If auto-detected as a junction and NOT parking-adjacent -> treat as junction cell
                    if (row_idx, col_idx) in auto_junctions and not is_parking_adjacent:
                        node_id = f"box_{box_node_counter}"
                        box_node_counter += 1
                        node = GraphNode(
                            node_id=node_id,
                            position=Point(x, y),
                            directions=directions,
                            is_conflict_box=True,
                            conflict_box_id=None
                        )
                        nodes[node_id] = node
                        position_to_node[pos] = node_id
                        # Track for grouping as junction
                        j_cells[(row_idx, col_idx)] = {
                            'node_id': node_id,
                            'row': row_idx,
                            'col': col_idx,
                            'position': Point(x, y),
                            'directions': directions,
                        }
                    else:
                        # Regular lane node (with optional single-cell conflict box if parking-adjacent)
                        node_id = f"lane_{row_idx}_{col_idx}"
                        node = GraphNode(
                            node_id=node_id,
                            position=Point(x, y),
                            directions=directions,
                            is_conflict_box=is_parking_adjacent,
                            conflict_box_id=(f"cb_p_{row_idx}_{col_idx}" if is_parking_adjacent else None)
                        )
                        nodes[node_id] = node
                        position_to_node[pos] = node_id
                        if is_parking_adjacent:
                            conflict_box_id = f"cb_p_{row_idx}_{col_idx}"
                            try:
                                conflict_boxes[conflict_box_id] = ConflictBox(
                                    box_id=conflict_box_id,
                                    position=Point(x, y),
                                    width=self.cell_size,
                                    height=self.cell_size,
                                    participating_nodes={node_id},
                                    directions=directions
                                )
                            except TypeError:
                                conflict_boxes[conflict_box_id] = ConflictBox(
                                    box_id=conflict_box_id,
                                    position=Point(x, y),
                                    size=self.cell_size,
                                    participating_nodes={node_id},
                                    directions=directions
                                )
                elif cell_value in ['i', '4']:  # Idle zone as a graph node (non-through)
                    # Idle nodes have no encoded directions; we connect them to adjacent lanes/junctions later
                    node_id = f"idle_{row_idx}_{col_idx}"
                    node = GraphNode(
                        node_id=node_id,
                        position=Point(x, y),
                        directions=[],
                        is_conflict_box=False
                    )
                    nodes[node_id] = node
                    position_to_node[pos] = node_id
                elif cell_value in ['c', '3']:  # Charging zone as a graph node (non-through)
                    # Use grid coordinates for consistent station ID format
                    node_id = f"charge_{row_idx}_{col_idx}"
                    node = GraphNode(
                        node_id=node_id,
                        position=Point(x, y),
                        directions=[],
                        is_conflict_box=False
                    )
                    nodes[node_id] = node
                    position_to_node[pos] = node_id
                elif cell_value == 'd':  # Drop-off zone as a graph node (restricted routing)
                    node_id = f"dropoff_{row_idx}_{col_idx}"
                    node = GraphNode(
                        node_id=node_id,
                        position=Point(x, y),
                        directions=[],
                        is_conflict_box=False
                    )
                    nodes[node_id] = node
                    position_to_node[pos] = node_id

        # Group adjacent j* cells (4-neighbor connectivity) into single conflict boxes
        if j_cells:
            visited: Set[Tuple[int, int]] = set()
            group_index = 0
            for key in list(j_cells.keys()):
                if key in visited:
                    continue
                # BFS/DFS to collect a connected component
                stack = [key]
                component: List[Tuple[int, int]] = []
                min_row = min_col = float('inf')
                max_row = max_col = float('-inf')
                dir_union: Set[LaneDirection] = set()
                node_ids_in_component: Set[str] = set()
                while stack:
                    r, c = stack.pop()
                    if (r, c) in visited or (r, c) not in j_cells:
                        continue
                    visited.add((r, c))
                    component.append((r, c))
                    data = j_cells[(r, c)]
                    min_row = min(min_row, r)
                    min_col = min(min_col, c)
                    max_row = max(max_row, r)
                    max_col = max(max_col, c)
                    dir_union.update(data['directions'])
                    node_ids_in_component.add(data['node_id'])
                    # 4-neighbor adjacency: N, S, E, W
                    neighbors = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
                    for nr, nc in neighbors:
                        if (nr, nc) in j_cells and (nr, nc) not in visited:
                            stack.append((nr, nc))
                if not component:
                    continue
                # Compute group box parameters (AABB-based rectangle)
                width_m = (max_col - min_col + 1) * self.cell_size
                height_m = (max_row - min_row + 1) * self.cell_size
                # Center of the AABB in world coordinates (cell centers are at +0.5)
                center_x = (min_col + max_col + 1) * 0.5 * self.cell_size
                center_y = (min_row + max_row + 1) * 0.5 * self.cell_size
                group_id = f"cb_{group_index}"
                group_index += 1
                # Assign group id to all nodes in this component
                for r, c in component:
                    node_id = j_cells[(r, c)]['node_id']
                    node_ref = nodes[node_id]
                    node_ref.conflict_box_id = group_id
                # Create one conflict box record for the whole component
                # Prefer width/height if ConflictBox supports them; otherwise store size for fallback
                try:
                    conflict_boxes[group_id] = ConflictBox(
                        box_id=group_id,
                        position=Point(center_x, center_y),
                        width=width_m,
                        height=height_m,
                        participating_nodes=node_ids_in_component,
                        directions=list(dir_union)
                    )
                except TypeError:
                    # During migration window if interface still expects size
                    size_m = max(width_m, height_m)
                    conflict_boxes[group_id] = ConflictBox(
                        box_id=group_id,
                        position=Point(center_x, center_y),
                        size=size_m,
                        participating_nodes=node_ids_in_component,
                        directions=list(dir_union)
                    )
    
    def _parse_directions(self, direction_str: str) -> List[LaneDirection]:
        """Parse direction string into LaneDirection set."""
        directions = []
        for char in direction_str.lower():  # Convert to lowercase for case-insensitive parsing
            if char == 'n':
                directions.append(LaneDirection.NORTH)
            elif char == 's':
                directions.append(LaneDirection.SOUTH)
            elif char == 'e':
                directions.append(LaneDirection.EAST)
            elif char == 'w':
                directions.append(LaneDirection.WEST)
        return directions
    
    def _build_connections(self, nodes: Dict[str, GraphNode],
                          position_to_node: Dict[Tuple[float, float], str],
                          edges: Dict[str, List[str]]) -> None:
        """Build connections between nodes based on exit directions."""
        for node_id, node in nodes.items():
            edges[node_id] = []
            
            # For each exit direction, find the target position and connect if it exists
            for direction in node.directions:
                target_pos = self._get_exit_position(node.position, direction)
                target_node_id = position_to_node.get((target_pos.x, target_pos.y))
                
                if target_node_id:
                    edges[node_id].append(target_node_id)

        # Connect idle/charging nodes to adjacent lane/junction nodes (bidirectional)
        for node_id, node in nodes.items():
            if node_id.startswith('idle_') or node_id.startswith('charge_'):
                x, y = node.position.x, node.position.y
                neighbor_positions = [
                    (x, y - self.cell_size),  # N
                    (x, y + self.cell_size),  # S
                    (x + self.cell_size, y),  # E
                    (x - self.cell_size, y),  # W
                ]
                for nx, ny in neighbor_positions:
                    neighbor_id = position_to_node.get((nx, ny))
                    if not neighbor_id:
                        continue
                    if neighbor_id.startswith('lane_') or neighbor_id.startswith('box_'):
                        # idle -> neighbor
                        if neighbor_id not in edges[node_id]:
                            edges[node_id].append(neighbor_id)
                        # neighbor -> idle
                        if neighbor_id not in edges:
                            edges[neighbor_id] = []
                        if node_id not in edges[neighbor_id]:
                            edges[neighbor_id].append(node_id)

        # Connect drop-off nodes to adjacent lane/junction nodes (unidirectional out of dropoff
        # to neighbors that DO NOT have an exit direction pointing back to the dropoff). This
        # preserves reachability to dropoffs via lane directions while preventing bidirectional
        # pairs on the same adjacency.
        for node_id, node in nodes.items():
            if not node_id.startswith('dropoff_'):
                continue
            x, y = node.position.x, node.position.y
            neighbor_positions = [
                (x, y - self.cell_size),  # N
                (x, y + self.cell_size),  # S
                (x + self.cell_size, y),  # E
                (x - self.cell_size, y),  # W
            ]
            for nx, ny in neighbor_positions:
                neighbor_id = position_to_node.get((nx, ny))
                if not neighbor_id:
                    continue
                if not (neighbor_id.startswith('lane_') or neighbor_id.startswith('box_')):
                    continue
                neighbor_node = nodes[neighbor_id]

                # Determine the cardinal direction from neighbor -> dropoff
                if abs(x - neighbor_node.position.x) > abs(y - neighbor_node.position.y):
                    # Horizontal neighbor
                    dir_to_drop = LaneDirection.EAST if x > neighbor_node.position.x else LaneDirection.WEST
                else:
                    # Vertical neighbor
                    dir_to_drop = LaneDirection.SOUTH if y > neighbor_node.position.y else LaneDirection.NORTH

                # If the neighbor has an exit direction back to dropoff, skip creating dropoff -> neighbor edge
                if dir_to_drop in (neighbor_node.directions or []):
                    continue

                # Otherwise, allow one-way exit from dropoff to this neighbor
                if neighbor_id not in edges[node_id]:
                    edges[node_id].append(neighbor_id)
    
    def _get_exit_position(self, position: Point, direction: LaneDirection) -> Point:
        """Calculate the position where the exit direction leads to."""
        if direction == LaneDirection.NORTH:
            return Point(position.x, position.y - self.cell_size)  # Go UP (smaller Y)
        elif direction == LaneDirection.SOUTH:
            return Point(position.x, position.y + self.cell_size)  # Go DOWN (larger Y)
        elif direction == LaneDirection.EAST:
            return Point(position.x + self.cell_size, position.y)  # Go RIGHT
        elif direction == LaneDirection.WEST:
            return Point(position.x - self.cell_size, position.y)  # Go LEFT
        else:
            raise ValueError(f"Unknown direction: {direction}")
    
    def validate_connectivity(self, graph: NavigationGraph) -> List[str]:
        """Validate that the graph is properly connected."""
        warnings = []
        
        # Check for isolated nodes
        isolated_nodes = []
        for node_id, node in graph.nodes.items():
            if not graph.edges.get(node_id, []) and not self._has_incoming_edges(node_id, graph.edges):
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
    
    def _has_incoming_edges(self, target_node_id: str, edges: Dict[str, List[str]]) -> bool:
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