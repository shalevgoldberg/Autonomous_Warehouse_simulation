"""
Interface for coordinate system conversions between world and cell coordinates.
"""
from abc import ABC, abstractmethod
from typing import Tuple, List
from .path_planner_interface import Cell


class ICoordinateSystem(ABC):
    """
    Interface for coordinate system conversion functionality.
    
    Responsibilities:
    - Convert between world coordinates and cell/grid coordinates
    - Provide mapping utilities for the warehouse grid system
    
    **Thread Safety**: ALL methods are thread-safe (read-only utility functions).
    **Threading Model**: 
    - ALL methods: ANY THREAD (stateless conversions, can be called from anywhere)
    """
    
    @abstractmethod
    def world_to_cell(self, world_pos: Tuple[float, float]) -> Cell:
        """
        Convert world coordinates to cell coordinates.
        
        Args:
            world_pos: Position in world coordinates (x, y)
            
        Returns:
            Cell: Corresponding cell coordinates
        """
        pass
    
    @abstractmethod
    def cell_to_world(self, cell: Cell) -> Tuple[float, float]:
        """
        Convert cell coordinates to world coordinates (center of cell).
        
        Args:
            cell: Cell coordinates
            
        Returns:
            Tuple[float, float]: World coordinates (x, y)
        """
        pass
    
    @abstractmethod
    def get_cell_size(self) -> float:
        """
        Get the size of each cell in world units.
        
        Returns:
            float: Cell size in meters
        """
        pass
    
    @abstractmethod
    def get_grid_bounds(self) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """
        Get the bounds of the grid in cell coordinates.
        
        Returns:
            Tuple[Tuple[int, int], Tuple[int, int]]: ((min_x, max_x), (min_y, max_y))
        """
        pass
    
    @abstractmethod
    def is_valid_cell(self, cell: Cell) -> bool:
        """
        Check if a cell coordinate is valid (within grid bounds).
        
        Args:
            cell: Cell to check
            
        Returns:
            bool: True if valid, False otherwise
        """
        pass
    
    @abstractmethod
    def get_neighbor_cells(self, cell: Cell) -> List[Cell]:
        """
        Get valid neighboring cells (4-connected).
        
        Args:
            cell: Center cell
            
        Returns:
            List[Cell]: Valid neighboring cells
        """
        pass
    
    @abstractmethod
    def distance_between_cells(self, cell1: Cell, cell2: Cell) -> float:
        """
        Calculate distance between two cells (Manhattan distance).
        
        Args:
            cell1: First cell
            cell2: Second cell
            
        Returns:
            float: Distance in cell units
        """
        pass
    
    @abstractmethod
    def distance_between_world_points(self, pos1: Tuple[float, float], 
                                    pos2: Tuple[float, float]) -> float:
        """
        Calculate Euclidean distance between two world points.
        
        Args:
            pos1: First position (x, y)
            pos2: Second position (x, y)
            
        Returns:
            float: Distance in world units (meters)
        """
        pass 