"""
CoordinateSystem Implementation - Pure utility for coordinate conversions.

This implementation provides stateless coordinate conversion between world
coordinates and cell/grid coordinates for the warehouse system.
"""
import math
from typing import Tuple

from interfaces.coordinate_system_interface import ICoordinateSystem
from interfaces.path_planner_interface import Cell


class CoordinateSystemImpl(ICoordinateSystem):
    """
    Thread-safe implementation of coordinate system conversions.
    
    Threading Model:
    - ALL methods: ANY THREAD (stateless conversions, fully thread-safe)
    
    This is a pure utility class with no mutable state.
    """
    
    def __init__(self, cell_size: float = 0.5, 
                 grid_origin: Tuple[float, float] = (0.0, 0.0),
                 grid_width: int = 20, grid_height: int = 20):
        """
        Initialize coordinate system.
        
        Args:
            cell_size: Size of each grid cell in world units (meters)
            grid_origin: World coordinates of grid origin (bottom-left corner)
            grid_width: Number of cells in X direction
            grid_height: Number of cells in Y direction
        """
        self.cell_size = cell_size
        self.grid_origin = grid_origin
        self.grid_width = grid_width
        self.grid_height = grid_height
    
    def world_to_cell(self, world_pos: Tuple[float, float]) -> Cell:
        """
        Convert world coordinates to cell coordinates.
        **Thread-safe**: Stateless operation.
        
        Args:
            world_pos: Position in world coordinates (x, y)
            
        Returns:
            Cell: Corresponding cell coordinates
            
        Raises:
            ValueError: If world coordinates are outside valid grid bounds
        """
        world_x, world_y = world_pos
        origin_x, origin_y = self.grid_origin
        
        # Convert to cell coordinates
        cell_x = int((world_x - origin_x) / self.cell_size)
        cell_y = int((world_y - origin_y) / self.cell_size)
        
        # Validate cell coordinates
        cell = Cell(cell_x, cell_y)
        if not self.is_valid_cell(cell):
            raise ValueError(f"World position {world_pos} maps to invalid cell {cell}. "
                           f"Grid bounds: {self.get_grid_bounds()}")
        
        return cell
    
    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        """
        Convert cell coordinates to world coordinates (center of cell).
        **Thread-safe**: Stateless operation.
        
        Args:
            cell: Cell coordinates
            
        Returns:
            Tuple[float, float]: World coordinates (x, y)
            
        Raises:
            ValueError: If cell coordinates are invalid
        """
        if not self.is_valid_cell(cell):
            raise ValueError(f"Invalid cell {cell}. Grid bounds: {self.get_grid_bounds()}")
        
        origin_x, origin_y = self.grid_origin
        
        # Convert to world coordinates (center of cell)
        world_x = origin_x + (cell.x + 0.5) * self.cell_size
        world_y = origin_y + (cell.y + 0.5) * self.cell_size
        
        return (world_x, world_y)
    
    def get_cell_size(self) -> float:
        """
        Get the size of each cell in world units.
        **Thread-safe**: Immutable data.
        
        Returns:
            float: Cell size in meters
        """
        return self.cell_size
    
    def get_grid_bounds(self) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """
        Get the bounds of the grid in cell coordinates.
        **Thread-safe**: Immutable data.
        
        Returns:
            Tuple[Tuple[int, int], Tuple[int, int]]: ((min_x, max_x), (min_y, max_y))
        """
        return ((0, self.grid_width - 1), (0, self.grid_height - 1))
    
    def is_valid_cell(self, cell: Cell) -> bool:
        """
        Check if a cell coordinate is valid (within grid bounds).
        **Thread-safe**: Stateless operation.
        
        Args:
            cell: Cell to check
            
        Returns:
            bool: True if valid, False otherwise
        """
        return (0 <= cell.x < self.grid_width and 
                0 <= cell.y < self.grid_height)
    
    def distance_between_cells(self, cell1: Cell, cell2: Cell) -> float:
        """
        Calculate distance between two cells (Manhattan distance).
        **Thread-safe**: Stateless operation.
        
        Args:
            cell1: First cell
            cell2: Second cell
            
        Returns:
            float: Distance in cell units
        """
        return abs(cell1.x - cell2.x) + abs(cell1.y - cell2.y)
    
    def distance_between_world_points(self, pos1: Tuple[float, float], 
                                    pos2: Tuple[float, float]) -> float:
        """
        Calculate Euclidean distance between two world points.
        **Thread-safe**: Stateless operation.
        
        Args:
            pos1: First position (x, y)
            pos2: Second position (x, y)
            
        Returns:
            float: Distance in world units (meters)
        """
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def get_neighbor_cells(self, cell: Cell) -> list[Cell]:
        """
        Get valid neighboring cells (4-connected).
        **Thread-safe**: Stateless operation.
        
        Args:
            cell: Center cell
            
        Returns:
            list[Cell]: Valid neighboring cells
        """
        neighbors = []
        
        # 4-connected neighbors: up, down, left, right
        for dx, dy in [(0, 1), (0, -1), (-1, 0), (1, 0)]:
            neighbor = Cell(cell.x + dx, cell.y + dy)
            if self.is_valid_cell(neighbor):
                neighbors.append(neighbor)
        
        return neighbors
    
    def normalize_angle(self, angle: float) -> float:
        """
        Normalize angle to [0, 2π] range.
        **Thread-safe**: Pure mathematical function.
        
        Args:
            angle: Angle in radians
            
        Returns:
            float: Normalized angle in [0, 2π] range
        """
        # Normalize to [0, 2π] range
        angle = angle % (2 * math.pi)
        if angle < 0:
            angle += 2 * math.pi
        return angle
    
    def angle_difference(self, angle1: float, angle2: float) -> float:
        """
        Calculate the smallest angular difference between two angles.
        **Thread-safe**: Pure mathematical function.
        
        Args:
            angle1: First angle in radians
            angle2: Second angle in radians
            
        Returns:
            float: Angular difference in radians [-π, π]
        """
        # Normalize both angles to [0, 2π]
        angle1_norm = self.normalize_angle(angle1)
        angle2_norm = self.normalize_angle(angle2)
        
        # Calculate raw difference
        diff = angle2_norm - angle1_norm
        
        # Normalize to [-π, π] range (smallest angular difference)
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
            
        return diff 