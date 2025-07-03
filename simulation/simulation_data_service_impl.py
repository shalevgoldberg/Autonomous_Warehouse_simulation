"""
SimulationDataService Implementation - Database-backed warehouse data management.

This implementation provides thread-safe access to warehouse data, inventory management,
and shelf locking operations backed by PostgreSQL database.

Design Principles:
- **Thread-Safe**: All methods use database transactions for consistency
- **SOLID**: Single responsibility, dependency injection, interface segregation
- **Error Handling**: Comprehensive exception handling with proper logging
- **Database Transactions**: Atomic operations with proper rollback
- **Connection Pooling**: Efficient database connection management
"""
import os
import logging
import threading
import time
import uuid
import json
from contextlib import contextmanager
from typing import Optional, List, Dict, Any, Tuple
from dataclasses import dataclass

import psycopg2
import psycopg2.pool
from psycopg2.extras import RealDictCursor
import psycopg2.errorcodes as errorcodes

from interfaces.simulation_data_service_interface import (
    ISimulationDataService, 
    ShelfInfo, 
    MapData,
    NavigationGraph,
    GraphNode,
    SimulationDataServiceError
)
from interfaces.navigation_types import LaneRec, BoxRec, Point, LaneDirection
from warehouse.map import WarehouseMap


# Configure logging
logger = logging.getLogger(__name__)


class SimulationDataServiceImpl(ISimulationDataService):
    """
    Thread-safe implementation of SimulationDataService backed by PostgreSQL.
    
    Threading Model:
    - ALL methods: Thread-safe (database transactions provide consistency)
    - Connection pooling: Handles concurrent access automatically
    - Shelf locking: Database-level locking prevents race conditions
    
    Database Design:
    - Uses existing warehouse schema (items, shelves, shelf_inventory, etc.)
    - Shelf locks stored in database with robot_id and timestamp
    - All operations are transactional with proper rollback
    """
    
    def __init__(self, warehouse_map: WarehouseMap, 
                 db_host: str = "localhost",
                 db_port: int = 5432,
                 db_name: str = "warehouse_sim",
                 db_user: str = "postgres",
                 db_password: Optional[str] = None,
                 pool_size: int = 10):
        """
        Initialize SimulationDataService with database connection.
        
        Args:
            warehouse_map: Warehouse layout for map data
            db_host: Database host
            db_port: Database port  
            db_name: Database name
            db_user: Database user
            db_password: Database password (or from WAREHOUSE_DB_PASSWORD env var)
            pool_size: Connection pool size
        """
        self.warehouse_map = warehouse_map
        
        # Get database password from environment if not provided
        if db_password is None:
            db_password = os.getenv('WAREHOUSE_DB_PASSWORD')
            if not db_password:
                raise SimulationDataServiceError(
                    "Database password not provided. Set WAREHOUSE_DB_PASSWORD environment variable."
                )
        
        # Database connection parameters
        self.db_params = {
            'host': db_host,
            'port': db_port,
            'database': db_name,
            'user': db_user,
            'password': db_password,
            'cursor_factory': RealDictCursor,
            'connect_timeout': 10,
            'application_name': 'warehouse_simulation'
        }
        
        # Connection pool for thread safety
        self._connection_pool = None
        self._pool_lock = threading.Lock()
        self.pool_size = pool_size
        
        # Cache for frequently accessed data
        self._map_data_cache = None
        self._cache_lock = threading.RLock()
        self._cache_timestamp = 0
        self._cache_ttl = 300  # 5 minutes cache TTL
        
        # Initialize database connection and schema
        self._initialize_database()
        
        logger.info(f"SimulationDataService initialized with database {db_name}@{db_host}:{db_port}")
    
    def _initialize_database(self) -> None:
        """Initialize database connection pool and verify schema."""
        try:
            # Create connection pool
            with self._pool_lock:
                self._connection_pool = psycopg2.pool.ThreadedConnectionPool(
                    minconn=2,
                    maxconn=self.pool_size,
                    **self.db_params
                )
            
            # Test connection and verify schema
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Verify required tables exist
                    cur.execute("""
                        SELECT table_name FROM information_schema.tables 
                        WHERE table_schema = 'public' 
                        AND table_name IN ('items', 'shelves', 'shelf_inventory', 'robots')
                    """)
                    tables = [row['table_name'] for row in cur.fetchall()]
                    
                    required_tables = {'items', 'shelves', 'shelf_inventory', 'robots'}
                    missing_tables = required_tables - set(tables)
                    
                    if missing_tables:
                        raise SimulationDataServiceError(
                            f"Missing required database tables: {missing_tables}. "
                            f"Please run warehouse_schema.sql to create the schema."
                        )
                    
                    # Add shelf_locks table if it doesn't exist
                    cur.execute("""
                        CREATE TABLE IF NOT EXISTS shelf_locks (
                            shelf_id VARCHAR(36) PRIMARY KEY,
                            robot_id VARCHAR(36) NOT NULL,
                            locked_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            FOREIGN KEY (shelf_id) REFERENCES shelves(shelf_id) ON DELETE CASCADE
                        )
                    """)
                    
                    # Create index for performance
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_shelf_locks_robot_id 
                        ON shelf_locks(robot_id)
                    """)
                    
                    conn.commit()
                    logger.info("Database schema verified and shelf_locks table ensured")
                    
        except psycopg2.Error as e:
            raise SimulationDataServiceError(f"Database initialization failed: {e}")
        except Exception as e:
            raise SimulationDataServiceError(f"Unexpected error during database initialization: {e}")
    
    @contextmanager
    def _get_connection(self):
        """Get database connection from pool with proper cleanup."""
        conn = None
        try:
            with self._pool_lock:
                if self._connection_pool is None:
                    raise SimulationDataServiceError("Database connection pool not initialized")
                conn = self._connection_pool.getconn()
            
            if conn.closed:
                # Connection is closed, get a new one
                with self._pool_lock:
                    self._connection_pool.putconn(conn, close=True)
                    conn = self._connection_pool.getconn()
            
            yield conn
            
        except psycopg2.Error as e:
            if conn:
                conn.rollback()
            raise SimulationDataServiceError(f"Database operation failed: {e}")
        except Exception as e:
            if conn:
                conn.rollback()
            raise SimulationDataServiceError(f"Unexpected database error: {e}")
        finally:
            if conn:
                with self._pool_lock:
                    if self._connection_pool:
                        self._connection_pool.putconn(conn)
    
    # Map Operations
    def get_map_data(self) -> MapData:
        """
        Get the static warehouse map data (cached for performance).
        
        Returns:
            MapData: Complete map information
        """
        # Check cache first
        with self._cache_lock:
            current_time = time.time()
            if (self._map_data_cache is not None and 
                current_time - self._cache_timestamp < self._cache_ttl):
                return self._map_data_cache
        
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Get shelves data
                    cur.execute("""
                        SELECT shelf_id, position_x, position_y 
                        FROM shelves 
                        ORDER BY shelf_id
                    """)
                    shelves_data = cur.fetchall()
                    
                    # Convert to map format
                    shelves = {}
                    for shelf in shelves_data:
                        # Convert world position to cell coordinates (handle Decimal types)
                        pos_x = float(shelf['position_x'])
                        pos_y = float(shelf['position_y'])
                        cell_x = int(pos_x / self.warehouse_map.grid_size)
                        cell_y = int(pos_y / self.warehouse_map.grid_size)
                        shelves[shelf['shelf_id']] = (cell_x, cell_y)
                    
                    # Create map data from warehouse map and database shelves
                    map_data = MapData(
                        width=self.warehouse_map.width,
                        height=self.warehouse_map.height,
                        cell_size=self.warehouse_map.grid_size,
                        obstacles=self.warehouse_map.get_obstacle_cells(),
                        shelves=shelves,
                        dropoff_zones=[(9, 9), (10, 9)],  # TODO: Make configurable
                        charging_zones=[(0, 0), (1, 0)]   # TODO: Make configurable
                    )
                    
                    # Update cache
                    with self._cache_lock:
                        self._map_data_cache = map_data
                        self._cache_timestamp = time.time()
                    
                    return map_data
                    
        except Exception as e:
            logger.error(f"Failed to get map data: {e}")
            raise SimulationDataServiceError(f"Failed to get map data: {e}")
    
    def get_shelf_position(self, shelf_id: str) -> Optional[Tuple[float, float]]:
        """
        Get the world position of a shelf.
        
        Args:
            shelf_id: Shelf identifier
            
        Returns:
            Optional[Tuple[float, float]]: Shelf position or None if not found
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT position_x, position_y 
                        FROM shelves 
                        WHERE shelf_id = %s
                    """, (shelf_id,))
                    
                    result = cur.fetchone()
                    if result:
                        return (float(result['position_x']), float(result['position_y']))
                    return None
                    
        except Exception as e:
            logger.error(f"Failed to get shelf position for {shelf_id}: {e}")
            raise SimulationDataServiceError(f"Failed to get shelf position: {e}")
    
    def get_dropoff_zones(self) -> List[Tuple[float, float]]:
        """
        Get all dropoff zone positions.
        
        Returns:
            List[Tuple[float, float]]: List of dropoff positions
        """
        # TODO: Make this configurable from database
        # For now, return hardcoded positions based on warehouse map
        dropoff_cells = [(9, 9), (10, 9)]
        return [
            (cell[0] * self.warehouse_map.grid_size + self.warehouse_map.grid_size/2,
             cell[1] * self.warehouse_map.grid_size + self.warehouse_map.grid_size/2)
            for cell in dropoff_cells
        ]
    
    # Shelf Operations
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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # First verify shelf exists
                    cur.execute("SELECT 1 FROM shelves WHERE shelf_id = %s", (shelf_id,))
                    if not cur.fetchone():
                        raise SimulationDataServiceError(f"Shelf {shelf_id} does not exist")
                    
                    # Try to acquire lock (INSERT will fail if already locked)
                    try:
                        cur.execute("""
                            INSERT INTO shelf_locks (shelf_id, robot_id, locked_at)
                            VALUES (%s, %s, CURRENT_TIMESTAMP)
                        """, (shelf_id, robot_id))
                        
                        conn.commit()
                        logger.debug(f"Shelf {shelf_id} locked by robot {robot_id}")
                        return True
                        
                    except psycopg2.IntegrityError:
                        # Shelf already locked
                        conn.rollback()
                        logger.debug(f"Shelf {shelf_id} already locked, cannot lock for robot {robot_id}")
                        return False
                        
        except SimulationDataServiceError:
            raise
        except Exception as e:
            logger.error(f"Failed to lock shelf {shelf_id} for robot {robot_id}: {e}")
            raise SimulationDataServiceError(f"Failed to lock shelf: {e}")
    
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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # First verify shelf exists
                    cur.execute("SELECT 1 FROM shelves WHERE shelf_id = %s", (shelf_id,))
                    if not cur.fetchone():
                        raise SimulationDataServiceError(f"Shelf {shelf_id} does not exist")
                    
                    # Try to unlock (only if locked by this robot)
                    cur.execute("""
                        DELETE FROM shelf_locks 
                        WHERE shelf_id = %s AND robot_id = %s
                    """, (shelf_id, robot_id))
                    
                    if cur.rowcount > 0:
                        conn.commit()
                        logger.debug(f"Shelf {shelf_id} unlocked by robot {robot_id}")
                        return True
                    else:
                        logger.debug(f"Shelf {shelf_id} not locked by robot {robot_id}, cannot unlock")
                        return False
                        
        except SimulationDataServiceError:
            raise
        except Exception as e:
            logger.error(f"Failed to unlock shelf {shelf_id} for robot {robot_id}: {e}")
            raise SimulationDataServiceError(f"Failed to unlock shelf: {e}")
    
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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # First verify shelf exists
                    cur.execute("SELECT 1 FROM shelves WHERE shelf_id = %s", (shelf_id,))
                    if not cur.fetchone():
                        raise SimulationDataServiceError(f"Shelf {shelf_id} does not exist")
                    
                    # Check if locked
                    cur.execute("""
                        SELECT 1 FROM shelf_locks WHERE shelf_id = %s
                    """, (shelf_id,))
                    
                    return cur.fetchone() is not None
                    
        except SimulationDataServiceError:
            raise
        except Exception as e:
            logger.error(f"Failed to check lock status for shelf {shelf_id}: {e}")
            raise SimulationDataServiceError(f"Failed to check shelf lock status: {e}")
    
    def get_shelf_lock_owner(self, shelf_id: str) -> Optional[str]:
        """
        Get the robot ID that currently owns the shelf lock.
        
        Args:
            shelf_id: Shelf to check
            
        Returns:
            Optional[str]: Robot ID that owns the lock, or None if not locked
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT robot_id FROM shelf_locks WHERE shelf_id = %s
                    """, (shelf_id,))
                    
                    result = cur.fetchone()
                    return result['robot_id'] if result else None
                    
        except Exception as e:
            logger.error(f"Failed to get lock owner for shelf {shelf_id}: {e}")
            raise SimulationDataServiceError(f"Failed to get shelf lock owner: {e}")
    
    # Inventory Operations
    def get_shelf_info(self, shelf_id: str) -> Optional[ShelfInfo]:
        """
        Get complete information about a shelf.
        
        Args:
            shelf_id: Shelf identifier
            
        Returns:
            Optional[ShelfInfo]: Shelf information or None if not found
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Get shelf basic info
                    cur.execute("""
                        SELECT shelf_id, position_x, position_y 
                        FROM shelves 
                        WHERE shelf_id = %s
                    """, (shelf_id,))
                    
                    shelf_result = cur.fetchone()
                    if not shelf_result:
                        return None
                    
                    # Get shelf inventory
                    cur.execute("""
                        SELECT i.item_id, i.name, si.quantity
                        FROM shelf_inventory si
                        JOIN items i ON si.item_id = i.item_id
                        WHERE si.shelf_id = %s AND si.quantity > 0
                        ORDER BY i.name
                    """, (shelf_id,))
                    
                    inventory_results = cur.fetchall()
                    items = [row['item_id'] for row in inventory_results]
                    
                    # Get lock status
                    cur.execute("""
                        SELECT robot_id FROM shelf_locks WHERE shelf_id = %s
                    """, (shelf_id,))
                    
                    lock_result = cur.fetchone()
                    
                    return ShelfInfo(
                        shelf_id=shelf_id,
                        position=(float(shelf_result['position_x']), float(shelf_result['position_y'])),
                        items=items,
                        capacity=100,  # TODO: Make configurable
                        is_locked=lock_result is not None,
                        locked_by=lock_result['robot_id'] if lock_result else None
                    )
                    
        except Exception as e:
            logger.error(f"Failed to get shelf info for {shelf_id}: {e}")
            raise SimulationDataServiceError(f"Failed to get shelf info: {e}")
    
    def get_item_location(self, item_id: str) -> Optional[str]:
        """
        Find which shelf contains a specific item.
        
        Args:
            item_id: Item to locate
            
        Returns:
            Optional[str]: Shelf ID containing the item, or None if not found
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT si.shelf_id 
                        FROM shelf_inventory si
                        WHERE si.item_id = %s AND si.quantity > 0
                        ORDER BY si.quantity DESC
                        LIMIT 1
                    """, (item_id,))
                    
                    result = cur.fetchone()
                    return result['shelf_id'] if result else None
                    
        except Exception as e:
            logger.error(f"Failed to find location for item {item_id}: {e}")
            raise SimulationDataServiceError(f"Failed to find item location: {e}")
    
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
        if operation not in ['add', 'remove', 'move']:
            raise SimulationDataServiceError(f"Invalid operation: {operation}")
        
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Verify shelf exists
                    cur.execute("SELECT 1 FROM shelves WHERE shelf_id = %s", (shelf_id,))
                    if not cur.fetchone():
                        raise SimulationDataServiceError(f"Shelf {shelf_id} does not exist")
                    
                    # Verify item exists
                    cur.execute("SELECT 1 FROM items WHERE item_id = %s", (item_id,))
                    if not cur.fetchone():
                        raise SimulationDataServiceError(f"Item {item_id} does not exist")
                    
                    if operation == 'add':
                        # Add items to shelf
                        cur.execute("""
                            INSERT INTO shelf_inventory (shelf_id, item_id, quantity)
                            VALUES (%s, %s, %s)
                            ON CONFLICT (shelf_id, item_id)
                            DO UPDATE SET quantity = shelf_inventory.quantity + EXCLUDED.quantity
                        """, (shelf_id, item_id, quantity))
                        
                    elif operation == 'remove':
                        # Remove items from shelf
                        cur.execute("""
                            UPDATE shelf_inventory 
                            SET quantity = GREATEST(quantity - %s, 0)
                            WHERE shelf_id = %s AND item_id = %s
                        """, (quantity, shelf_id, item_id))
                        
                        if cur.rowcount == 0:
                            logger.warning(f"No inventory found for item {item_id} on shelf {shelf_id}")
                            return False
                    
                    # TODO: Implement 'move' operation when needed
                    
                    conn.commit()
                    logger.debug(f"Inventory updated: {operation} {quantity} of {item_id} on shelf {shelf_id}")
                    return True
                    
        except SimulationDataServiceError:
            raise
        except Exception as e:
            logger.error(f"Failed to update inventory: {e}")
            raise SimulationDataServiceError(f"Failed to update inventory: {e}")
    
    # KPI Logging
    def log_event(self, event_type: str, robot_id: str, event_data: Dict[str, Any]) -> None:
        """
        Log a KPI event.
        
        Args:
            event_type: Type of event (task_start, task_complete, collision, etc.)
            robot_id: Robot associated with the event
            event_data: Additional event data
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Store event in simulation_metrics table
                    event_id = str(uuid.uuid4())
                    
                    # Store main event
                    cur.execute("""
                        INSERT INTO simulation_metrics (metric_name, metric_value, simulation_run_id)
                        VALUES (%s, %s, %s)
                    """, (f"{event_type}_{robot_id}", 1.0, event_id))
                    
                    # Store event details as separate metrics
                    for key, value in event_data.items():
                        if isinstance(value, (int, float)):
                            cur.execute("""
                                INSERT INTO simulation_metrics (metric_name, metric_value, simulation_run_id)
                                VALUES (%s, %s, %s)
                            """, (f"{event_type}_{key}", float(value), event_id))
                    
                    conn.commit()
                    logger.debug(f"Logged event: {event_type} for robot {robot_id}")
                    
        except Exception as e:
            # Don't raise exceptions for logging failures
            logger.error(f"Failed to log event {event_type} for robot {robot_id}: {e}")
    
    # Lane-Based Navigation Operations
    def get_lanes(self) -> List[LaneRec]:
        """
        Get all lane definitions from database.
        
        Returns:
            List[LaneRec]: All lane records
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT lane_id, direction, waypoints, is_goal_only, bay_id
                        FROM lanes
                        ORDER BY lane_id
                    """)
                    
                    results = cur.fetchall()
                    lanes = []
                    
                    for row in results:
                        # Parse waypoints from JSON
                        waypoints_data = row['waypoints']
                        waypoints = [Point(x=float(wp[0]), y=float(wp[1])) for wp in waypoints_data]
                        
                        # Parse direction enum
                        direction = LaneDirection(row['direction'])
                        
                        lane = LaneRec(
                            lane_id=row['lane_id'],
                            direction=direction,
                            waypoints=waypoints,
                            is_goal_only=row['is_goal_only'],
                            bay_id=row['bay_id']
                        )
                        lanes.append(lane)
                    
                    logger.debug(f"Loaded {len(lanes)} lanes from database")
                    return lanes
                    
        except Exception as e:
            logger.error(f"Failed to load lanes: {e}")
            raise SimulationDataServiceError(f"Failed to load lanes: {e}")
    
    def get_conflict_boxes(self) -> List[BoxRec]:
        """
        Get all conflict box definitions.
        
        Returns:
            List[BoxRec]: All conflict box records
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT box_id, center_x, center_y, size
                        FROM conflict_boxes
                        ORDER BY box_id
                    """)
                    
                    results = cur.fetchall()
                    boxes = []
                    
                    for row in results:
                        box = BoxRec(
                            box_id=row['box_id'],
                            center=Point(x=float(row['center_x']), y=float(row['center_y'])),
                            size=float(row['size'])
                        )
                        boxes.append(box)
                    
                    logger.debug(f"Loaded {len(boxes)} conflict boxes from database")
                    return boxes
                    
        except Exception as e:
            logger.error(f"Failed to load conflict boxes: {e}")
            raise SimulationDataServiceError(f"Failed to load conflict boxes: {e}")
    
    def get_navigation_graph(self) -> NavigationGraph:
        """
        Get the complete navigation graph for path planning.
        
        Returns:
            NavigationGraph: Complete graph with nodes, edges, and conflict boxes
            
        Raises:
            SimulationDataServiceError: If graph construction fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Load nodes
                    cur.execute("""
                        SELECT node_id, position_x, position_y, directions, 
                               is_conflict_box, conflict_box_id
                        FROM navigation_graph_nodes
                        ORDER BY node_id
                    """)
                    
                    node_results = cur.fetchall()
                    nodes = {}
                    
                    for row in node_results:
                        # Parse directions from JSON
                        directions_data = row['directions']
                        directions = [LaneDirection(d) for d in directions_data]
                        
                        node = GraphNode(
                            node_id=row['node_id'],
                            position=Point(x=float(row['position_x']), y=float(row['position_y'])),
                            directions=directions,
                            is_conflict_box=row['is_conflict_box'],
                            conflict_box_id=row['conflict_box_id']
                        )
                        nodes[row['node_id']] = node
                    
                    # Load edges
                    cur.execute("""
                        SELECT from_node, to_node
                        FROM navigation_graph_edges
                        ORDER BY from_node, to_node
                    """)
                    
                    edge_results = cur.fetchall()
                    edges = {}
                    
                    for row in edge_results:
                        from_node = row['from_node']
                        to_node = row['to_node']
                        
                        if from_node not in edges:
                            edges[from_node] = []
                        edges[from_node].append(to_node)
                    
                    # Load conflict boxes
                    conflict_boxes = {}
                    for box in self.get_conflict_boxes():
                        conflict_boxes[box.box_id] = box
                    
                    graph = NavigationGraph(
                        nodes=nodes,
                        edges=edges,
                        conflict_boxes=conflict_boxes
                    )
                    
                    logger.debug(f"Loaded navigation graph with {len(nodes)} nodes, "
                               f"{sum(len(neighbors) for neighbors in edges.values())} edges, "
                               f"{len(conflict_boxes)} conflict boxes")
                    return graph
                    
        except Exception as e:
            logger.error(f"Failed to load navigation graph: {e}")
            raise SimulationDataServiceError(f"Failed to load navigation graph: {e}")
    
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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Convert Unix timestamp to PostgreSQL timestamp (use local time, not GMT)
                    unblock_timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(unblock_time))
                    
                    # Insert or update blocked cell
                    cur.execute("""
                        INSERT INTO blocked_cells (cell_id, unblock_time, blocked_by_robot, block_reason)
                        VALUES (%s, %s, %s, %s)
                        ON CONFLICT (cell_id)
                        DO UPDATE SET 
                            unblock_time = EXCLUDED.unblock_time,
                            blocked_by_robot = EXCLUDED.blocked_by_robot,
                            block_reason = EXCLUDED.block_reason,
                            created_at = CURRENT_TIMESTAMP
                    """, (cell_id, unblock_timestamp, robot_id, reason))
                    
                    conn.commit()
                    logger.debug(f"Reported blocked cell {cell_id} by robot {robot_id}, "
                               f"unblock at {unblock_timestamp}, reason: {reason}")
                    return True
                    
        except Exception as e:
            logger.error(f"Failed to report blocked cell {cell_id}: {e}")
            raise SimulationDataServiceError(f"Failed to report blocked cell: {e}")
    
    def get_blocked_cells(self) -> Dict[str, float]:
        """
        Get current blocked cells with unblock timestamps.
        
        Returns:
            Dict[str, float]: Mapping of cell_id to unblock_time (Unix timestamp)
            
        Raises:
            SimulationDataServiceError: If database operation fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT cell_id, unblock_time
                        FROM blocked_cells
                        WHERE unblock_time > CURRENT_TIMESTAMP
                        ORDER BY cell_id
                    """)
                    
                    results = cur.fetchall()
                    blocked_cells = {}
                    
                    for row in results:
                        # Convert PostgreSQL timestamp to Unix timestamp
                        unblock_time = row['unblock_time'].timestamp()
                        blocked_cells[row['cell_id']] = unblock_time
                    
                    logger.debug(f"Retrieved {len(blocked_cells)} currently blocked cells")
                    return blocked_cells
                    
        except Exception as e:
            logger.error(f"Failed to get blocked cells: {e}")
            raise SimulationDataServiceError(f"Failed to get blocked cells: {e}")
    
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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Delete blocked cell only if it's blocked by the requesting robot
                    cur.execute("""
                        DELETE FROM blocked_cells
                        WHERE cell_id = %s AND blocked_by_robot = %s
                    """, (cell_id, robot_id))
                    
                    success = cur.rowcount > 0
                    conn.commit()
                    
                    if success:
                        logger.debug(f"Cleared blocked cell {cell_id} by robot {robot_id}")
                    else:
                        logger.debug(f"Cell {cell_id} not blocked by robot {robot_id} or not blocked at all")
                    
                    return success
                    
        except Exception as e:
            logger.error(f"Failed to clear blocked cell {cell_id}: {e}")
            raise SimulationDataServiceError(f"Failed to clear blocked cell: {e}")
    
    def cleanup_expired_blocks(self) -> int:
        """
        Remove expired blocked cell entries.
        
        Returns:
            int: Number of expired blocks removed
            
        Raises:
            SimulationDataServiceError: If cleanup operation fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Use the database function we created
                    cur.execute("SELECT cleanup_expired_blocked_cells()")
                    result = cur.fetchone()
                    deleted_count = result[0] if result else 0
                    
                    conn.commit()
                    logger.debug(f"Cleaned up {deleted_count} expired blocked cells")
                    return deleted_count
                    
        except Exception as e:
            logger.error(f"Failed to cleanup expired blocks: {e}")
            raise SimulationDataServiceError(f"Failed to cleanup expired blocks: {e}")
    
    def close(self) -> None:
        """Close database connections and cleanup resources."""
        try:
            if self._connection_pool:
                with self._pool_lock:
                    self._connection_pool.closeall()
                    self._connection_pool = None
                logger.info("SimulationDataService database connections closed")
        except Exception as e:
            logger.error(f"Error closing database connections: {e}")
    
    def __del__(self):
        """Ensure cleanup on object destruction."""
        self.close()