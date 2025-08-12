#!/usr/bin/env python3
"""
Comprehensive navigation graph analysis and validation.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
from warehouse.impl.graph_generator_impl import GraphGeneratorImpl
from pathlib import Path
from interfaces.navigation_types import Point

def analyze_navigation_graph():
    """Analyze navigation graph quality and validate against physical map."""
    print("=== NAVIGATION GRAPH ANALYSIS ===")
    
    # Load warehouse map
    warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
    print(f"Warehouse Map: {warehouse_map.width}x{warehouse_map.height} grid")
    print(f"World bounds: 0.0 to {warehouse_map.width * warehouse_map.grid_size:.1f}m X, 0.0 to {warehouse_map.height * warehouse_map.grid_size:.1f}m Y")
    
    # Generate navigation graph
    graph_generator = GraphGeneratorImpl()
    navigation_graph = graph_generator.generate_graph(Path("sample_warehouse.csv"))
    
    print(f"\n=== NAVIGATION GRAPH STATISTICS ===")
    print(f"Total nodes: {len(navigation_graph.nodes)}")
    print(f"Total edges: {sum(len(edges) for edges in navigation_graph.edges.values())}")
    print(f"Conflict boxes: {len(navigation_graph.conflict_boxes)}")
    
    # Analyze node distribution
    print(f"\n=== NODE ANALYSIS ===")
    node_positions = []
    for node_id, node in navigation_graph.nodes.items():
        node_positions.append((node.position.x, node.position.y))
        print(f"Node {node_id}: ({node.position.x:.1f}, {node.position.y:.1f}) - Directions: {node.directions}")
    
    # Check coverage of walkable areas
    print(f"\n=== WALKABLE AREA COVERAGE ===")
    walkable_cells = []
    for y in range(warehouse_map.height):
        for x in range(warehouse_map.width):
            if warehouse_map.grid[y, x] in [0, 3, 4, 5]:  # Free space, charging, idle, drop-off
                world_x, world_y = warehouse_map.grid_to_world(x, y)
                walkable_cells.append((world_x, world_y))
    
    print(f"Warehouse map walkable cells: {len(walkable_cells)}")
    print(f"Navigation graph nodes: {len(navigation_graph.nodes)}")
    
    # Check if navigation graph covers all walkable areas
    covered_cells = set()
    for node in navigation_graph.nodes.values():
        # Find the closest warehouse cell to this node
        grid_x, grid_y = warehouse_map.world_to_grid(node.position.x, node.position.y)
        if 0 <= grid_x < warehouse_map.width and 0 <= grid_y < warehouse_map.height:
            if warehouse_map.grid[grid_y, grid_x] in [0, 3, 4, 5]:
                covered_cells.add((grid_x, grid_y))
    
    print(f"Navigation graph covers {len(covered_cells)} walkable cells")
    print(f"Coverage ratio: {len(covered_cells)}/{len(walkable_cells)} = {len(covered_cells)/len(walkable_cells)*100:.1f}%")
    
    # Find uncovered walkable areas
    all_walkable = set()
    for y in range(warehouse_map.height):
        for x in range(warehouse_map.width):
            if warehouse_map.grid[y, x] in [0, 3, 4, 5]:
                all_walkable.add((x, y))
    
    uncovered = all_walkable - covered_cells
    print(f"Uncovered walkable cells: {len(uncovered)}")
    if uncovered:
        print("Uncovered areas:")
        for x, y in sorted(uncovered)[:10]:  # Show first 10
            world_x, world_y = warehouse_map.grid_to_world(x, y)
            print(f"  Grid({x}, {y}) -> World({world_x:.1f}, {world_y:.1f})")
    
    # Analyze connectivity
    print(f"\n=== CONNECTIVITY ANALYSIS ===")
    connected_nodes = set()
    disconnected_nodes = set()
    
    for node_id, node in navigation_graph.nodes.items():
        neighbors = navigation_graph.edges.get(node_id, [])
        if neighbors:
            connected_nodes.add(node_id)
        else:
            disconnected_nodes.add(node_id)
    
    print(f"Connected nodes: {len(connected_nodes)}")
    print(f"Disconnected nodes: {len(disconnected_nodes)}")
    
    if disconnected_nodes:
        print("Disconnected nodes:")
        for node_id in sorted(disconnected_nodes)[:5]:
            node = navigation_graph.nodes[node_id]
            print(f"  {node_id}: ({node.position.x:.1f}, {node.position.y:.1f})")
    
    # Test path finding between different areas
    print(f"\n=== PATH FINDING TESTS ===")
    
    # Get some test positions
    test_positions = [
        (1.0, 1.0),   # Start position from test
        (2.8, 3.2),   # Target position from test
        (3.0, 5.0),   # Another test position
        (2.0, 4.0),   # Another test position
    ]
    
    for i, start_pos in enumerate(test_positions):
        for j, target_pos in enumerate(test_positions):
            if i != j:
                print(f"\nTesting path: ({start_pos[0]:.1f}, {start_pos[1]:.1f}) -> ({target_pos[0]:.1f}, {target_pos[1]:.1f})")
                
                # Find closest nodes
                start_node = None
                target_node = None
                min_start_dist = float('inf')
                min_target_dist = float('inf')
                
                for node_id, node in navigation_graph.nodes.items():
                    # Distance to start
                    start_dist = ((node.position.x - start_pos[0])**2 + (node.position.y - start_pos[1])**2)**0.5
                    if start_dist < min_start_dist:
                        min_start_dist = start_dist
                        start_node = node_id
                    
                    # Distance to target
                    target_dist = ((node.position.x - target_pos[0])**2 + (node.position.y - target_pos[1])**2)**0.5
                    if target_dist < min_target_dist:
                        min_target_dist = target_dist
                        target_node = node_id
                
                if start_node and target_node:
                    print(f"  Start node: {start_node} (distance: {min_start_dist:.2f}m)")
                    print(f"  Target node: {target_node} (distance: {min_target_dist:.2f}m)")
                    
                    # Check if path exists
                    if start_node in navigation_graph.edges and target_node in navigation_graph.edges:
                        print(f"  Both nodes have edges - path should be possible")
                    else:
                        print(f"  Path may not exist - missing edges")
                else:
                    print(f"  Could not find suitable nodes")
    
    # Analyze edge quality
    print(f"\n=== EDGE ANALYSIS ===")
    total_edges = 0
    bidirectional_edges = 0
    
    for node_id, edges in navigation_graph.edges.items():
        total_edges += len(edges)
        for target_id in edges:
            # Check if edge is bidirectional
            if target_id in navigation_graph.edges and node_id in navigation_graph.edges[target_id]:
                bidirectional_edges += 1
    
    print(f"Total edges: {total_edges}")
    print(f"Bidirectional edges: {bidirectional_edges}")
    print(f"Unidirectional edges: {total_edges - bidirectional_edges}")
    
    # Check for isolated regions
    print(f"\n=== ISOLATION ANALYSIS ===")
    visited = set()
    regions = []
    
    for node_id in navigation_graph.nodes:
        if node_id not in visited:
            # BFS to find connected component
            region = set()
            queue = [node_id]
            visited.add(node_id)
            
            while queue:
                current = queue.pop(0)
                region.add(current)
                
                for neighbor in navigation_graph.edges.get(current, []):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            
            regions.append(region)
    
    print(f"Number of isolated regions: {len(regions)}")
    for i, region in enumerate(regions):
        print(f"  Region {i+1}: {len(region)} nodes")
        if len(region) <= 3:
            print(f"    Nodes: {list(region)}")

if __name__ == "__main__":
    analyze_navigation_graph() 