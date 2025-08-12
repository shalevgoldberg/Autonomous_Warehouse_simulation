#!/usr/bin/env python3
"""
Debug script to analyze warehouse map and start position validation.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap

def analyze_warehouse_map():
    """Analyze the warehouse map and coordinate conversion."""
    print("=== WAREHOUSE MAP ANALYSIS ===")
    
    # Load the warehouse map
    warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
    
    print(f"Map dimensions: {warehouse_map.width}x{warehouse_map.height}")
    print(f"Grid size: {warehouse_map.grid_size}m")
    print(f"World dimensions: {warehouse_map.width * warehouse_map.grid_size:.1f}m x {warehouse_map.height * warehouse_map.grid_size:.1f}m")
    
    # Print the grid for visualization
    print("\n=== GRID LAYOUT ===")
    char_map = {0: '.', 1: 'w', 2: 's', 3: 'c', 4: 'i', 5: 'd'}
    for y in range(warehouse_map.height):
        row = ""
        for x in range(warehouse_map.width):
            cell_value = warehouse_map.grid[y, x]
            row += char_map.get(cell_value, '?')
        print(f"{y:2d}: {row}")
    
    # Test specific positions
    test_positions = [
        (1.0, 1.0),   # From test - valid
        (3.0, 5.0),   # New valid position
        (2.0, 4.0),   # New valid position
        (4.0, 12.0),  # Old failing position - outside bounds
        (3.0, 12.0),  # Old failing position - outside bounds
        (0.5, 0.5),   # Corner
        (1.5, 1.5),   # Middle of first cell
        (2.0, 2.0),   # Another position
    ]
    
    print(f"\n=== POSITION ANALYSIS ===")
    for world_x, world_y in test_positions:
        grid_x, grid_y = warehouse_map.world_to_grid(world_x, world_y)
        is_walkable = warehouse_map.is_walkable(world_x, world_y)
        cell_value = warehouse_map.grid[grid_y, grid_x] if (0 <= grid_x < warehouse_map.width and 0 <= grid_y < warehouse_map.height) else -1
        
        print(f"World ({world_x:.1f}, {world_y:.1f}) -> Grid ({grid_x}, {grid_y}) -> Cell value: {cell_value} -> Walkable: {is_walkable}")
    
    # Find all walkable positions
    print(f"\n=== WALKABLE POSITIONS ===")
    walkable_count = 0
    for y in range(warehouse_map.height):
        for x in range(warehouse_map.width):
            if warehouse_map.grid[y, x] in [0, 3, 4, 5]:  # Free space, charging, idle, drop-off
                world_x, world_y = warehouse_map.grid_to_world(x, y)
                print(f"Walkable: Grid({x}, {y}) -> World({world_x:.1f}, {world_y:.1f})")
                walkable_count += 1
    
    print(f"\nTotal walkable cells: {walkable_count}")
    
    # Test grid bounds
    print(f"\n=== GRID BOUNDS TEST ===")
    print(f"Grid bounds: ((0, {warehouse_map.width-1}), (0, {warehouse_map.height-1}))")
    
    # Test the specific failing position
    failing_x, failing_y = 4.0, 12.0
    grid_x, grid_y = warehouse_map.world_to_grid(failing_x, failing_y)
    print(f"Failing position ({failing_x}, {failing_y}) -> Grid ({grid_x}, {grid_y})")
    print(f"Grid bounds check: 0 <= {grid_x} < {warehouse_map.width} = {0 <= grid_x < warehouse_map.width}")
    print(f"Grid bounds check: 0 <= {grid_y} < {warehouse_map.height} = {0 <= grid_y < warehouse_map.height}")
    
    if 0 <= grid_x < warehouse_map.width and 0 <= grid_y < warehouse_map.height:
        cell_value = warehouse_map.grid[grid_y, grid_x]
        print(f"Cell value at ({grid_x}, {grid_y}): {cell_value}")
        print(f"Is walkable: {cell_value in [0, 3, 4, 5]}")

if __name__ == "__main__":
    analyze_warehouse_map() 