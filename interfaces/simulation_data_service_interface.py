"""
Interface for SimulationDataService - manages warehouse static data and shelf operations.
"""
from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any, Tuple
from dataclasses import dataclass


@dataclass
class ShelfInfo:
    """Information about a warehouse shelf."""
    shelf_id: str
    position: Tuple[float, float]
    items: List[str]
    capacity: int
    is_locked: bool
    locked_by: Optional[str] = None  # robot_id that locked it


@dataclass
class MapData:
    """Static map data for the warehouse."""
    width: int
    height: int
    cell_size: float
    obstacles: List[Tuple[int, int]]  # Cell coordinates of obstacles
    shelves: Dict[str, Tuple[int, int]]  # shelf_id -> cell coordinates
    dropoff_zones: List[Tuple[int, int]]
    charging_zones: List[Tuple[int, int]]


class SimulationDataServiceError(Exception):
    """Raised when simulation data service operations fail."""
    pass


class ISimulationDataService(ABC):
    """
    Interface for simulation data service functionality.
    
    Responsibilities:
    - Provide REST API for static map access (read-only in Phase 2)
    - Handle shelf inventory queries
    - Manage shelf lock/unlock for safe, exclusive access
    - Store KPI events and logging data
    - Guarantee strong consistency of shared resources
    
    **Thread Safety**: ALL methods are fully thread-safe with database-level consistency.
    **Threading Model**:
    - Map operations: ANY THREAD (read-only data)
    - Shelf operations: CONTROL THREAD (called by TaskHandler)
    - Inventory operations: CONTROL THREAD (JobsProcessor, TaskHandler)
    - log_event(): ANY THREAD (logging from all components)
    """
    
    # Map Operations
    @abstractmethod
    def get_map_data(self) -> MapData:
        """
        Get the static warehouse map data.
        
        Returns:
            MapData: Complete map information
        """
        pass
    
    @abstractmethod
    def get_shelf_position(self, shelf_id: str) -> Optional[Tuple[float, float]]:
        """
        Get the world position of a shelf.
        
        Args:
            shelf_id: Shelf identifier
            
        Returns:
            Optional[Tuple[float, float]]: Shelf position or None if not found
        """
        pass
    
    @abstractmethod
    def get_dropoff_zones(self) -> List[Tuple[float, float]]:
        """
        Get all dropoff zone positions.
        
        Returns:
            List[Tuple[float, float]]: List of dropoff positions
        """
        pass
    
    # Shelf Operations
    @abstractmethod
    def lock_shelf(self, shelf_id: str, robot_id: str) -> bool:
        """
        Lock a shelf for exclusive access by a robot.
        
        Args:
            shelf_id: Shelf to lock
            robot_id: Robot requesting the lock
            
        Returns:
            bool: True if successfully locked, False if already locked
            
        Raises:
            SimulationDataServiceError: If shelf doesn't exist
        """
        pass
    
    @abstractmethod
    def unlock_shelf(self, shelf_id: str, robot_id: str) -> bool:
        """
        Unlock a previously locked shelf.
        
        Args:
            shelf_id: Shelf to unlock
            robot_id: Robot that owns the lock
            
        Returns:
            bool: True if successfully unlocked, False if not locked by this robot
            
        Raises:
            SimulationDataServiceError: If shelf doesn't exist
        """
        pass
    
    @abstractmethod
    def is_shelf_locked(self, shelf_id: str) -> bool:
        """
        Check if a shelf is currently locked.
        
        Args:
            shelf_id: Shelf to check
            
        Returns:
            bool: True if locked, False otherwise
            
        Raises:
            SimulationDataServiceError: If shelf doesn't exist
        """
        pass
    
    @abstractmethod
    def get_shelf_lock_owner(self, shelf_id: str) -> Optional[str]:
        """
        Get the robot ID that currently owns the shelf lock.
        
        Args:
            shelf_id: Shelf to check
            
        Returns:
            Optional[str]: Robot ID that owns the lock, or None if not locked
        """
        pass
    
    # Inventory Operations
    @abstractmethod
    def get_shelf_info(self, shelf_id: str) -> Optional[ShelfInfo]:
        """
        Get complete information about a shelf.
        
        Args:
            shelf_id: Shelf identifier
            
        Returns:
            Optional[ShelfInfo]: Shelf information or None if not found
        """
        pass
    
    @abstractmethod
    def get_item_location(self, item_id: str) -> Optional[str]:
        """
        Find which shelf contains a specific item.
        
        Args:
            item_id: Item to locate
            
        Returns:
            Optional[str]: Shelf ID containing the item, or None if not found
        """
        pass
    
    @abstractmethod
    def update_inventory(self, shelf_id: str, item_id: str, operation: str, quantity: int = 1) -> bool:
        """
        Update inventory for a shelf (add, remove, or move items).
        
        Args:
            shelf_id: Shelf to update
            item_id: Item to update
            operation: Operation type ("add", "remove", "move")
            quantity: Number of items (default 1)
            
        Returns:
            bool: True if inventory was updated successfully, False otherwise
        """
        pass
    
    # KPI Logging
    @abstractmethod
    def log_event(self, event_type: str, robot_id: str, event_data: Dict[str, Any]) -> None:
        """
        Log a KPI event.
        
        Args:
            event_type: Type of event (task_start, task_complete, collision, etc.)
            robot_id: Robot associated with the event
            event_data: Additional event data
        """
        pass 