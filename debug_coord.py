#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap

def debug_coordinates():
    warehouse_map = WarehouseMap()
    print(f"Warehouse: {warehouse_map.width}x{warehouse_map.height}")
    print(f"Grid size: {warehouse_map.grid_size}")
    print(f"Start position: {warehouse_map.start_position}")
    
    world_width = warehouse_map.width * warehouse_map.grid_size
    world_height = warehouse_map.height * warehouse_map.grid_size
    print(f"World size: {world_width}x{world_height}")

if __name__ == "__main__":
    debug_coordinates()