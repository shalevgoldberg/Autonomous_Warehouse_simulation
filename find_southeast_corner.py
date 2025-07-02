#!/usr/bin/env python3
"""Find the southeast corner position in the warehouse."""

from warehouse.map import WarehouseMap

def find_southeast_corner():
    """Find a suitable southeast corner position for the robot."""
    print("=== FINDING SOUTHEAST CORNER ===")
    
    # Initialize warehouse
    warehouse = WarehouseMap(width=20, height=15)
    
    print(f"Warehouse size: {warehouse.width}x{warehouse.height} cells")
    print(f"World size: {warehouse.width * warehouse.grid_size}x{warehouse.height * warehouse.grid_size}m")
    
    # Southeast corner would be at high X, high Y coordinates
    # But we need to find a walkable position near there
    
    # Try positions starting from the southeast and moving inward
    candidates = []
    
    # Start from the actual southeast corner and work inward
    for y_offset in range(1, 4):  # Try 1-3 cells from the edge
        for x_offset in range(1, 4):  # Try 1-3 cells from the edge
            # Calculate position from southeast corner
            x = warehouse.width - x_offset
            y = warehouse.height - y_offset
            
            # Convert to world coordinates
            world_x, world_y = warehouse.grid_to_world(x, y)
            
            # Check if walkable
            if warehouse.is_walkable(world_x, world_y):
                candidates.append((world_x, world_y, x, y))
                print(f"Candidate: World({world_x:.1f}, {world_y:.1f}) = Grid({x}, {y})")
    
    if candidates:
        # Choose the first (closest to southeast corner) candidate
        best_pos = candidates[0]
        print(f"\nBest southeast position: World({best_pos[0]:.1f}, {best_pos[1]:.1f}) = Grid({best_pos[2]}, {best_pos[3]})")
        
        # Show what's around this position
        print(f"\nArea around southeast corner:")
        center_x, center_y = best_pos[2], best_pos[3]
        
        for y in range(max(0, center_y - 2), min(warehouse.height, center_y + 3)):
            row = f"Row {y:2d}: "
            for x in range(max(0, center_x - 2), min(warehouse.width, center_x + 3)):
                if x == center_x and y == center_y:
                    row += "[R]"  # Robot position
                else:
                    cell = warehouse.grid[y, x]
                    symbols = {0: " . ", 1: " W ", 2: " S ", 3: " C ", 4: " I ", 5: " D "}
                    row += symbols.get(cell, " ? ")
            print(row)
        
        return best_pos[0], best_pos[1]
    else:
        print("No walkable southeast positions found!")
        return None

if __name__ == "__main__":
    pos = find_southeast_corner()
    if pos:
        print(f"\nUse this position: ({pos[0]}, {pos[1]})") 