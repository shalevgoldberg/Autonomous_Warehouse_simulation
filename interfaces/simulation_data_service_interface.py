"""
Interface for SimulationDataService - manages warehouse static data and shelf operations.
"""
from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any, Tuple
from pathlib import Path
from dataclasses import dataclass, field

# Import lane-based navigation types
from .navigation_types import LaneRec, BoxRec, Point, LaneDirection
from .graph_persistence_interface import GraphPersistenceResult


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
class ItemInventoryInfo:
    """Information about an inventory item (pending-aware)."""
    item_id: str
    available_quantity: int
    total_quantity: int
    reserved_quantity: int
    shelf_locations: List[str]
    name: Optional[str] = None
    description: Optional[str] = None
    category: Optional[str] = None


@dataclass
class ItemShelfLocation:
    """Information about item location on a shelf (pending-aware)."""
    item_id: str
    shelf_id: str
    quantity: int  # Available quantity (quantity - pending)
    last_updated: float  # Unix timestamp


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
    idle_zones: List[Tuple[int, int]]


@dataclass
class NavigationGraph:
    """Navigation graph for lane-based path planning."""
    nodes: Dict[str, 'GraphNode']  # node_id -> GraphNode
    edges: Dict[str, List[str]]    # node_id -> list of connected node_ids
    conflict_boxes: Dict[str, BoxRec]  # box_id -> BoxRec
    position_to_node: Dict[Tuple[float, float], str] = field(default_factory=dict)
    
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
    
    @abstractmethod
    def get_idle_zones(self) -> List[Tuple[float, float]]:
        """
        Get all idle zone positions.
        
        Returns:
            List[Tuple[float, float]]: List of idle zone positions
        """
        pass
    
    @abstractmethod
    def get_optimal_idle_zone(self, robot_position: Tuple[float, float], 
                             robot_id: str) -> Optional[Tuple[float, float]]:
        """
        Get the optimal idle zone for a robot based on position and availability.
        
        Args:
            robot_position: Current robot position (x, y)
            robot_id: Robot identifier for tracking
            
        Returns:
            Optional[Tuple[float, float]]: Optimal idle zone position or None if none available
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

    # Graph Persistence Operations
    @abstractmethod
    def persist_navigation_graph_from_csv(self, csv_path: Path, clear_existing: bool = False) -> GraphPersistenceResult:
        """
        Generate and persist a navigation graph from a warehouse CSV file.

        Args:
            csv_path: Path to the warehouse CSV file.
            clear_existing: If True, clear existing graph tables before persist.

        Returns:
            GraphPersistenceResult with counts of persisted boxes, nodes, and edges.
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

    # Bay (Idle/Charging) Lock Operations
    @abstractmethod
    def list_idle_bays(self) -> List[Tuple[str, Tuple[float, float]]]:
        """
        List idle bay ids with positions. Bay ids should correspond to graph node ids (e.g., idle_r_c).
        Returns: List of (bay_id, position) tuples
        """
        pass

    @abstractmethod
    def list_charging_bays(self) -> List[Tuple[str, Tuple[float, float]]]:
        """
        List charging bay ids with positions. Bay ids should correspond to graph node ids (e.g., charge_r_c).
        Returns: List of (bay_id, position) tuples
        """
        pass

    @abstractmethod
    def try_acquire_bay_lock(self, bay_id: str, robot_id: str, timeout_seconds: int = 600) -> bool:
        """
        Try to acquire an exclusive lock on a bay.
        """
        pass

    @abstractmethod
    def release_bay_lock(self, bay_id: str, robot_id: str) -> bool:
        """
        Release a bay lock if owned by the robot.
        """
        pass

    @abstractmethod
    def heartbeat_bay_lock(self, bay_id: str, robot_id: str) -> bool:
        """
        Heartbeat an existing bay lock to keep it alive.
        """
        pass

    @abstractmethod
    def cleanup_expired_bay_locks(self) -> int:
        """
        Cleanup expired bay locks.
        """
        pass

    @abstractmethod
    def get_bay_lock_info(self, bay_id: str) -> Optional[Dict]:
        """
        Get information about a bay lock.

        Args:
            bay_id: Bay identifier to check

        Returns:
            Dict with lock information if locked, None if unlocked.
            Format: {
                'robot_id': str,
                'locked_at': float,  # timestamp
                'heartbeat_at': float,  # timestamp
                'lock_timeout_seconds': int
            }
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
    def reserve_inventory(self, shelf_id: str, item_id: str, quantity: int) -> bool:
        """
        Reserve (pend) inventory for an item on a specific shelf.
        Increments pending by quantity only if (quantity - pending) >= requested.

        Returns:
            bool: True if reservation succeeded, False if insufficient availability.
        """
        pass

    @abstractmethod
    def release_inventory(self, shelf_id: str, item_id: str, quantity: int) -> bool:
        """
        Release a previously reserved (pending) quantity without consuming stock.
        Decrements pending by quantity when pending >= quantity.

        Returns:
            bool: True if release succeeded, False otherwise.
        """
        pass

    @abstractmethod
    def consume_inventory(self, shelf_id: str, item_id: str, quantity: int) -> bool:
        """
        Consume reserved inventory on successful task completion.
        Atomically decreases both pending and quantity by the consumed amount
        when pending >= quantity and quantity >= amount.

        Returns:
            bool: True if consumption succeeded, False otherwise.
        """
        pass
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
    
    @abstractmethod
    def create_shelves_from_map(self, clear_existing: bool = False) -> int:
        """
        Create shelves in the database from warehouse map data.
        
        Args:
            clear_existing: Whether to clear existing shelves before creating new ones
            
        Returns:
            int: Number of shelves created
            
        Raises:
            SimulationDataServiceError: If shelf creation fails
        """
        pass
    
    @abstractmethod
    def populate_inventory(self, inventory_data: List[Dict[str, Any]],
                          assignment_strategy: str = "random",
                          strategy_options: Optional[Dict[str, Any]] = None) -> int:
        """
        Populate inventory with items and assign them to shelves.

        Args:
            inventory_data: List of inventory data dictionaries with format:
                {
                    'item_id': str,
                    'name': str,
                    'description': str (optional),
                    'category': str (optional),  # Used for category-based assignment
                    'shelf_id': str (optional), # If provided, overrides strategy
                    'quantity': int,
                    'priority': str (optional)  # Used for proximity-based assignment
                }
            assignment_strategy: Strategy for assigning items without explicit shelf_id:
                - "random": Assign randomly from available shelves (default)
                - "load_balanced": Assign to least utilized shelf
                - "category_based": Group items by category and assign to zones
                - "proximity_based": Assign high-priority items closer to picking stations
            strategy_options: Additional configuration for the strategy:
                - category_zones: Dict mapping categories to shelf ID prefixes
                - priority_zones: Dict mapping priorities to shelf ID prefixes
                - max_shelf_capacity: Maximum items per shelf

        Returns:
            int: Number of inventory entries created

        Raises:
            SimulationDataServiceError: If inventory population fails
        """
        pass
    
    @abstractmethod
    def get_inventory_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive inventory statistics.
        
        Returns:
            Dict[str, Any]: Inventory statistics including:
                - total_shelves: Number of shelves
                - total_items: Number of unique items
                - total_quantity: Total quantity across all shelves
                - low_stock_items: Items with quantity < 5
        """
        pass

    @abstractmethod
    def export_inventory_status_csv(self, filename: str) -> bool:
        """
        Export complete inventory status to CSV file.

        Args:
            filename: Output CSV filename

        Returns:
            bool: True on success, False on failure
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