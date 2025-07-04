"""
Interface for SimulationDataService - manages warehouse static data and shelf operations.
"""
from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any, Tuple
from dataclasses import dataclass

# Import lane-based navigation types
from .navigation_types import LaneRec, BoxRec, Point, LaneDirection


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


@dataclass
class NavigationGraph:
    """Navigation graph for lane-based path planning."""
    nodes: Dict[str, 'GraphNode']  # node_id -> GraphNode
    edges: Dict[str, List[str]]    # node_id -> list of connected node_ids
    conflict_boxes: Dict[str, BoxRec]  # box_id -> BoxRec
    
    def get_neighbors(self, node_id: str) -> List[str]:
        """Get neighboring node IDs for a given node."""
        return self.edges.get(node_id, [])
    
    def get_node(self, node_id: str) -> Optional['GraphNode']:
        """Get a graph node by ID."""
        return self.nodes.get(node_id)


@dataclass
class GraphNode:
    """Node in the navigation graph."""
    node_id: str
    position: Point
    directions: List[LaneDirection]  # Available exit directions
    is_conflict_box: bool = False
    conflict_box_id: Optional[str] = None


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
    - Manage lane-based navigation data (lanes, conflict boxes, blocked cells)
    
    **Thread Safety**: ALL methods are fully thread-safe with database-level consistency.
    **Threading Model**:
    - Map operations: ANY THREAD (read-only data)
    - Shelf operations: CONTROL THREAD (called by TaskHandler)
    - Inventory operations: CONTROL THREAD (JobsProcessor, TaskHandler)
    - Lane operations: CONTROL THREAD (PathPlanner, TaskHandler)
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
    
    # Lane-Based Navigation Operations
    @abstractmethod
    def get_lanes(self) -> List[LaneRec]:
        """
        Get all lane definitions from database.
        
        Returns:
            List[LaneRec]: All lane records
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def get_conflict_boxes(self) -> List[BoxRec]:
        """
        Get all conflict box definitions.
        
        Returns:
            List[BoxRec]: All conflict box records
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def get_navigation_graph(self) -> NavigationGraph:
        """
        Get the complete navigation graph for path planning.
        
        Returns:
            NavigationGraph: Complete graph with nodes, edges, and conflict boxes
            
        Raises:
            SimulationDataServiceError: If graph construction fails
        """
        pass
    
    @abstractmethod
    def report_blocked_cell(self, cell_id: str, robot_id: str, 
                           unblock_time: float, reason: str) -> bool:
        """
        Report that a cell/lane is blocked by a robot.
        
        Args:
            cell_id: Identifier of blocked cell/lane
            robot_id: Robot reporting the block
            unblock_time: Unix timestamp when cell should be unblocked
            reason: Reason for blocking (e.g., "dynamic_obstacle", "robot_stalled")
            
        Returns:
            bool: True if block was reported successfully, False otherwise
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def get_blocked_cells(self) -> Dict[str, float]:
        """
        Get current blocked cells with unblock timestamps.
        
        Returns:
            Dict[str, float]: Mapping of cell_id to unblock_time (Unix timestamp)
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def clear_blocked_cell(self, cell_id: str, robot_id: str) -> bool:
        """
        Clear a blocked cell before its automatic unblock time.
        
        Args:
            cell_id: Identifier of cell to unblock
            robot_id: Robot requesting the unblock (must match blocker)
            
        Returns:
            bool: True if cell was unblocked, False if not blocked or wrong robot
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def cleanup_expired_blocks(self) -> int:
        """
        Remove expired blocked cell entries.
        
        Returns:
            int: Number of expired blocks removed
            
        Raises:
            SimulationDataServiceError: If cleanup operation fails
        """
        pass
    
    # Conflict Box Lock Operations
    @abstractmethod
    def try_acquire_conflict_box_lock(self, box_id: str, robot_id: str, priority: int = 0) -> bool:
        """
        Try to acquire a lock on a conflict box.
        
        Args:
            box_id: Conflict box to lock
            robot_id: Robot requesting the lock
            priority: Lock priority for deadlock resolution (higher = more priority)
            
        Returns:
            bool: True if lock was acquired, False if already locked by another robot
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def release_conflict_box_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Release a conflict box lock.
        
        Args:
            box_id: Conflict box to unlock
            robot_id: Robot that owns the lock
            
        Returns:
            bool: True if lock was released, False if not owned by this robot
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def heartbeat_conflict_box_lock(self, box_id: str, robot_id: str) -> bool:
        """
        Update heartbeat for a conflict box lock.
        
        Args:
            box_id: Conflict box to heartbeat
            robot_id: Robot that owns the lock
            
        Returns:
            bool: True if heartbeat was updated, False if not owned by this robot
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def get_conflict_box_lock_owner(self, box_id: str) -> Optional[str]:
        """
        Get the robot ID that currently owns a conflict box lock.
        
        Args:
            box_id: Conflict box to check
            
        Returns:
            Optional[str]: Robot ID that owns the lock, or None if not locked
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        pass
    
    @abstractmethod
    def cleanup_expired_conflict_box_locks(self) -> int:
        """
        Remove expired conflict box lock entries.
        
        Returns:
            int: Number of expired locks removed
            
        Raises:
            SimulationDataServiceError: If cleanup operation fails
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