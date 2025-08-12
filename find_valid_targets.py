#!/usr/bin/env python3
"""
Script to find valid target positions within warehouse bounds.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap

def find_valid_targets():
    """Find valid target positions within warehouse bounds."""
    print("=== FINDING VALID TARGET POSITIONS ===")
    
    # Load the warehouse map
    warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
    
    print(f"Warehouse dimensions: {warehouse_map.width}x{warehouse_map.height}")
    print(f"World bounds: 0.0 to {warehouse_map.width * warehouse_map.grid_size:.1f}m X, 0.0 to {warehouse_map.height * warehouse_map.grid_size:.1f}m Y")
    
    # Find walkable positions
    valid_targets = []
    
    for y in range(warehouse_map.height):
        for x in range(warehouse_map.width):
            if warehouse_map.grid[y, x] in [0, 3, 4, 5]:  # Free space, charging, idle, drop-off
                world_x, world_y = warehouse_map.grid_to_world(x, y)
                valid_targets.append((world_x, world_y))
    
    print(f"\nFound {len(valid_targets)} valid target positions:")
    
    # Group by distance from center for better testing
    center_x = (warehouse_map.width * warehouse_map.grid_size) / 2
    center_y = (warehouse_map.height * warehouse_map.grid_size) / 2
    
    # Find targets at different distances
    close_targets = []
    medium_targets = []
    far_targets = []
    
    for x, y in valid_targets:
        distance = ((x - center_x)**2 + (y - center_y)**2)**0.5
        if distance < 1.0:
            close_targets.append((x, y, distance))
        elif distance < 2.5:
            medium_targets.append((x, y, distance))
        else:
            far_targets.append((x, y, distance))
    
    print(f"\nClose targets (< 1.0m from center):")
    for x, y, dist in sorted(close_targets, key=lambda t: t[2])[:5]:
        print(f"  ({x:.1f}, {y:.1f}) - {dist:.1f}m from center")
    
    print(f"\nMedium targets (1.0-2.5m from center):")
    for x, y, dist in sorted(medium_targets, key=lambda t: t[2])[:5]:
        print(f"  ({x:.1f}, {y:.1f}) - {dist:.1f}m from center")
    
    print(f"\nFar targets (> 2.5m from center):")
    for x, y, dist in sorted(far_targets, key=lambda t: t[2])[:5]:
        print(f"  ({x:.1f}, {y:.1f}) - {dist:.1f}m from center")
    
    # Suggest good test targets
    print(f"\n=== SUGGESTED TEST TARGETS ===")
    print("Based on the analysis, here are good test targets:")
    
    # Pick diverse targets
    if close_targets:
        close_target = close_targets[0]
        print(f"Close target: ({close_target[0]:.1f}, {close_target[1]:.1f})")
    
    if medium_targets:
        medium_target = medium_targets[len(medium_targets)//2]
        print(f"Medium target: ({medium_target[0]:.1f}, {medium_target[1]:.1f})")
    
    if far_targets:
        far_target = far_targets[0]
        print(f"Far target: ({far_target[0]:.1f}, {far_target[1]:.1f})")
    
    # Also find targets in different areas
    print(f"\n=== AREA-BASED TARGETS ===")
    
    # Top-left area
    top_left_targets = [(x, y) for x, y in valid_targets if x < 2.0 and y > 3.0]
    if top_left_targets:
        print(f"Top-left area: {top_left_targets[0]}")
    
    # Top-right area  
    top_right_targets = [(x, y) for x, y in valid_targets if x > 3.0 and y > 3.0]
    if top_right_targets:
        print(f"Top-right area: {top_right_targets[0]}")
    
    # Bottom area
    bottom_targets = [(x, y) for x, y in valid_targets if y < 2.0]
    if bottom_targets:
        print(f"Bottom area: {bottom_targets[0]}")

if __name__ == "__main__":
    find_valid_targets() 