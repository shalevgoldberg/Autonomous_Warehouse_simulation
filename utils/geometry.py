import numpy as np
import math

def normalize_angle(angle):
    """Normalize angle to [-pi, pi] range."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def distance_between_points(p1, p2):
    """Calculate Euclidean distance between two 2D points."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def angle_between_points(p1, p2):
    """Calculate angle from p1 to p2."""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

def transform_point(point, origin, angle):
    """Transform a point by rotation and translation."""
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    x = point[0] - origin[0]
    y = point[1] - origin[1]
    new_x = x * cos_a + y * sin_a
    new_y = -x * sin_a + y * cos_a
    return (new_x, new_y)

def inverse_transform_point(point, origin, angle):
    """Inverse transform a point by rotation and translation."""
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    
    x = point[0]
    y = point[1]
    
    new_x = x * cos_a - y * sin_a + origin[0]
    new_y = x * sin_a + y * cos_a + origin[1]
    
    return (new_x, new_y)

def point_in_rectangle(point, rect_center, rect_size, rect_angle):
    """Check if a point is inside a rotated rectangle."""
    # Transform point to rectangle's local coordinate system
    local_point = transform_point(point, rect_center, -rect_angle)
    
    # Check if point is within rectangle bounds
    half_width = rect_size[0] / 2
    half_height = rect_size[1] / 2
    
    return (abs(local_point[0]) <= half_width and 
            abs(local_point[1]) <= half_height)

def line_intersects_rectangle(line_start, line_end, rect_center, rect_size, rect_angle):
    """Check if a line intersects with a rotated rectangle."""
    # Transform line to rectangle's local coordinate system
    local_start = transform_point(line_start, rect_center, -rect_angle)
    local_end = transform_point(line_end, rect_center, -rect_angle)
    
    half_width = rect_size[0] / 2
    half_height = rect_size[1] / 2
    
    # Check intersection with rectangle sides
    rect_corners = [
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height)
    ]
    
    for i in range(4):
        p1 = rect_corners[i]
        p2 = rect_corners[(i + 1) % 4]
        
        if lines_intersect(local_start, local_end, p1, p2):
            return True
    
    return False

def lines_intersect(p1, p2, p3, p4):
    """Check if two line segments intersect."""
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
    
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4) 

def mapdata_to_warehouse_map(map_data):
    """
    Convert MapData (from SimulationDataService) to a WarehouseMap instance.
    Args:
        map_data (MapData): The map data object from the data service.
    Returns:
        WarehouseMap: A reconstructed warehouse map.
    """
    from warehouse.map import WarehouseMap
    warehouse_map = WarehouseMap(width=map_data.width, height=map_data.height)
    warehouse_map.grid_size = map_data.cell_size
    # Clear grid
    warehouse_map.grid[:, :] = 0
    # Set obstacles
    for (x, y) in map_data.obstacles:
        warehouse_map.grid[y, x] = 1
    # Set shelves
    warehouse_map.shelves = dict(map_data.shelves)
    for shelf_id, (x, y) in map_data.shelves.items():
        warehouse_map.grid[y, x] = 2
    # Do not assign dropoff_stations or charging_zones directly; rely on WarehouseMap methods
    return warehouse_map 