#!/usr/bin/env python3
"""Debug coordinate system conversions."""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from interfaces.path_planner_interface import Cell

def debug_coordinate_systems():
    print("=== DEBUGGING COORDINATE SYSTEMS ===")
    
    # Create warehouse map
    warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
    print(f"Warehouse: {warehouse_map.width}x{warehouse_map.height} cells")
    print(f"Warehouse grid_size: {warehouse_map.grid_size}m")
    print(f"Warehouse world size: {warehouse_map.width * warehouse_map.grid_size}x{warehouse_map.height * warehouse_map.grid_size}m")
    
    # Create robot coordinate system (same as robot does)
    robot_cell_size = 0.25
    scale_factor = warehouse_map.grid_size / robot_cell_size
    scaled_width = int(warehouse_map.width * scale_factor)
    scaled_height = int(warehouse_map.height * scale_factor)
    
    print(f"\nRobot cell_size: {robot_cell_size}m")
    print(f"Scale factor: {scale_factor}")
    print(f"Robot grid: {scaled_width}x{scaled_height} cells")
    
    robot_coord_system = CoordinateSystemImpl(
        cell_size=robot_cell_size,
        grid_width=scaled_width,
        grid_height=scaled_height
    )
    
    # Test the target coordinates from the demo
    test_targets = [
        ("Far East", 18, 4),
        ("Far North", 18, 22),
        ("Far West", 2, 10),
    ]
    
    print(f"\n=== TESTING TARGET CONVERSIONS ===")
    for name, robot_grid_x, robot_grid_y in test_targets:
        cell = Cell(robot_grid_x, robot_grid_y)
        world_x, world_y = robot_coord_system.cell_to_world(cell)
        
        print(f"{name}: Robot grid ({robot_grid_x}, {robot_grid_y}) -> World ({world_x:.3f}, {world_y:.3f})")
        
        # Check if it's within warehouse bounds
        max_world_x = warehouse_map.width * warehouse_map.grid_size
        max_world_y = warehouse_map.height * warehouse_map.grid_size
        
        in_bounds = (0 <= world_x <= max_world_x) and (0 <= world_y <= max_world_y)
        print(f"  Within warehouse bounds ({max_world_x}x{max_world_y}): {in_bounds}")
        
        # Check if it's walkable
        if in_bounds:
            is_walkable = warehouse_map.is_walkable(world_x, world_y)
            print(f"  Is walkable: {is_walkable}")
        
        print()
    
    # Test some warehouse positions
    print(f"=== TESTING WAREHOUSE POSITIONS ===")
    warehouse_test_positions = [
        (9, 2),   # Far East in warehouse coordinates
        (9, 11),  # Far North in warehouse coordinates
        (1, 5),   # Far West in warehouse coordinates
    ]
    
    for warehouse_x, warehouse_y in warehouse_test_positions:
        # Convert using warehouse map
        warehouse_world_x, warehouse_world_y = warehouse_map.grid_to_world(warehouse_x, warehouse_y)
        print(f"Warehouse grid ({warehouse_x}, {warehouse_y}) -> World ({warehouse_world_x:.3f}, {warehouse_world_y:.3f})")
        
        # Try to convert back using robot coordinate system
        try:
            robot_cell = robot_coord_system.world_to_cell((warehouse_world_x, warehouse_world_y))
            print(f"  Robot equivalent: Cell({robot_cell.x}, {robot_cell.y})")
        except Exception as e:
            print(f"  Robot conversion error: {e}")
        
        print()

if __name__ == "__main__":
    debug_coordinate_systems() 