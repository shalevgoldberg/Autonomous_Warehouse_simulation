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
    SimulationDataServiceError,
    ItemInventoryInfo,
    ItemShelfLocation,
)
from interfaces.graph_persistence_interface import GraphPersistenceResult
from warehouse.impl.graph_persistence_impl import GraphPersistenceImpl
from pathlib import Path
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

                    # Ensure bay_locks table exists (simple exclusive occupancy for bays)
                    cur.execute("""
                        CREATE TABLE IF NOT EXISTS bay_locks (
                            bay_id VARCHAR(64) PRIMARY KEY,
                            robot_id VARCHAR(36) NOT NULL,
                            locked_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            heartbeat_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                            lock_timeout_seconds INTEGER DEFAULT 600
                        )
                    """)
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_bay_locks_robot_id ON bay_locks(robot_id)
                    """)
                    cur.execute("""
                        CREATE INDEX IF NOT EXISTS idx_bay_locks_heartbeat ON bay_locks(heartbeat_at)
                    """)
                    conn.commit()
                    logger.info("Database schema ensured for bay_locks")

                    # Ensure conflict box lock helper functions exist.
                    # Safe: CREATE OR REPLACE and no destructive operations.
                    try:
                        # try_acquire_conflict_box_lock
                        cur.execute(
                            """
                            CREATE OR REPLACE FUNCTION try_acquire_conflict_box_lock(
                                p_box_id VARCHAR(36),
                                p_robot_id VARCHAR(36),
                                p_priority INTEGER DEFAULT 0
                            )
                            RETURNS BOOLEAN AS $$
                            DECLARE
                                lock_acquired BOOLEAN := FALSE;
                            BEGIN
                                -- Attempt to insert lock row; if exists, do nothing
                                INSERT INTO conflict_box_locks (box_id, locked_by_robot, lock_priority)
                                VALUES (p_box_id, p_robot_id, COALESCE(p_priority, 0))
                                ON CONFLICT (box_id) DO NOTHING;

                                -- Check if this robot holds the lock
                                SELECT (locked_by_robot = p_robot_id) INTO lock_acquired
                                FROM conflict_box_locks
                                WHERE box_id = p_box_id;

                                RETURN COALESCE(lock_acquired, FALSE);
                            END;
                            $$ LANGUAGE plpgsql;
                            """
                        )

                        # release_conflict_box_lock
                        cur.execute(
                            """
                            CREATE OR REPLACE FUNCTION release_conflict_box_lock(
                                p_box_id VARCHAR(36),
                                p_robot_id VARCHAR(36)
                            )
                            RETURNS BOOLEAN AS $$
                            DECLARE
                                lock_released BOOLEAN := FALSE;
                            BEGIN
                                DELETE FROM conflict_box_locks
                                WHERE box_id = p_box_id AND locked_by_robot = p_robot_id;
                                GET DIAGNOSTICS lock_released = FOUND;
                                RETURN lock_released;
                            END;
                            $$ LANGUAGE plpgsql;
                            """
                        )

                        # heartbeat_conflict_box_lock
                        cur.execute(
                            """
                            CREATE OR REPLACE FUNCTION heartbeat_conflict_box_lock(
                                p_box_id VARCHAR(36),
                                p_robot_id VARCHAR(36)
                            )
                            RETURNS BOOLEAN AS $$
                            DECLARE
                                heartbeat_updated BOOLEAN := FALSE;
                            BEGIN
                                UPDATE conflict_box_locks
                                SET heartbeat_timestamp = CURRENT_TIMESTAMP
                                WHERE box_id = p_box_id AND locked_by_robot = p_robot_id;
                                GET DIAGNOSTICS heartbeat_updated = FOUND;
                                RETURN heartbeat_updated;
                            END;
                            $$ LANGUAGE plpgsql;
                            """
                        )

                        conn.commit()
                        logger.info("Conflict box lock functions ensured")
                    except Exception as e:
                        conn.rollback()
                        logger.warning(f"Could not ensure conflict box functions (continuing): {e}")
                    
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

    # Internal helpers
    def _scalar(self, row, default=None):
        """Extract a scalar value from a fetchone() row that may be a tuple or dict.

        Returns default if row is None or cannot be parsed.
        """
        if row is None:
            return default
        try:
            # Dict row from RealDictCursor
            if isinstance(row, dict):
                if len(row) == 1:
                    return next(iter(row.values()))
                # Attempt known function keys
                for key in (
                    'try_acquire_conflict_box_lock',
                    'release_conflict_box_lock',
                    'heartbeat_conflict_box_lock',
                    'cleanup_expired_blocked_cells',
                ):
                    if key in row:
                        return row[key]
                # Fallback to first value
                return next(iter(row.values()))
            # Tuple/list row
            if isinstance(row, (list, tuple)):
                return row[0] if row else default
            # Already scalar
            return row
        except Exception:
            return default
    
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
                    # Scan warehouse map for all zone types
                    dropoff_zones = []
                    charging_zones = []
                    idle_zones = []

                    for y in range(self.warehouse_map.height):
                        for x in range(self.warehouse_map.width):
                            cell_value = self.warehouse_map.grid[y, x]
                            if cell_value == 3:  # Charging zone
                                charging_zones.append((x, y))
                            elif cell_value == 4:  # Idle zone
                                idle_zones.append((x, y))
                            elif cell_value == 5:  # Drop-off station
                                dropoff_zones.append((x, y))

                    map_data = MapData(
                        width=self.warehouse_map.width,
                        height=self.warehouse_map.height,
                        cell_size=self.warehouse_map.grid_size,
                        obstacles=self.warehouse_map.get_obstacle_cells(),
                        shelves=shelves,
                        dropoff_zones=dropoff_zones,
                        charging_zones=charging_zones,
                        idle_zones=idle_zones
                    )
                    
                    # Update cache
                    with self._cache_lock:
                        self._map_data_cache = map_data
                        self._cache_timestamp = time.time()
                    
                    return map_data
                    
        except Exception as e:
            logger.error(f"Failed to get map data: {e}")
            raise SimulationDataServiceError(f"Failed to get map data: {e}")

    # Graph persistence API
    def persist_navigation_graph_from_csv(self, csv_path: Path, clear_existing: bool = False) -> GraphPersistenceResult:
        try:
            gp = GraphPersistenceImpl(self)
            result = gp.persist_from_csv(csv_path=csv_path, clear_existing=clear_existing)
            logger.info(
                "Persisted navigation graph from %s (boxes=%d, nodes=%d, edges=%d, cleared=%s)",
                str(csv_path), result.boxes_persisted, result.nodes_persisted, result.edges_persisted, result.cleared_existing
            )
            return result
        except Exception as e:
            logger.error(f"Failed to persist navigation graph: {e}")
            raise SimulationDataServiceError(f"Graph persistence failed: {e}")
    
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
        # Get drop-off stations from warehouse map (which reads from CSV)
        return self.warehouse_map._get_dropoff_stations()
    
    def get_idle_zones(self) -> List[Tuple[float, float]]:
        """
        Get all idle zone positions.
        
        Returns:
            List[Tuple[float, float]]: List of idle zone positions
        """
        # Get idle zones from warehouse map
        return self.warehouse_map._get_idle_zones()

    # ---- Bay (Idle/Charging) Listing ----
    def list_idle_bays(self) -> List[Tuple[str, Tuple[float, float]]]:
        """List idle bays as (bay_id, position). Uses map grid positions to synthesize ids."""
        bays: List[Tuple[str, Tuple[float, float]]] = []
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                if self.warehouse_map.grid[y, x] == 4:
                    bay_id = f"idle_{y}_{x}"
                    bays.append((bay_id, self.warehouse_map.grid_to_world(x, y)))
        return bays

    def list_charging_bays(self) -> List[Tuple[str, Tuple[float, float]]]:
        """List charging bays as (bay_id, position). Uses map grid positions to synthesize ids."""
        bays: List[Tuple[str, Tuple[float, float]]] = []
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                if self.warehouse_map.grid[y, x] == 3:
                    bay_id = f"charge_{y}_{x}"
                    bays.append((bay_id, self.warehouse_map.grid_to_world(x, y)))
        return bays
    
    def get_optimal_idle_zone(self, robot_position: Tuple[float, float], 
                             robot_id: str) -> Optional[Tuple[float, float]]:
        """
        Get the optimal idle zone for a robot based on position and availability.
        
        Simple strategy: choose the closest available idle zone.
        Future enhancements: consider zone occupancy, traffic patterns, etc.
        
        Args:
            robot_position: Current robot position (x, y)
            robot_id: Robot identifier for tracking
            
        Returns:
            Optional[Tuple[float, float]]: Optimal idle zone position or None if none available
        """
        idle_zones = self.get_idle_zones()
        if not idle_zones:
            logger.warning("No idle zones available")
            return None
        
        # Simple strategy: find closest idle zone
        # Future enhancement: consider zone occupancy, traffic patterns, etc.
        closest_zone = None
        min_distance = float('inf')
        
        for zone_pos in idle_zones:
            distance = ((zone_pos[0] - robot_position[0]) ** 2 + 
                       (zone_pos[1] - robot_position[1]) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                closest_zone = zone_pos
        
        logger.debug(f"Selected idle zone {closest_zone} for robot {robot_id} "
                    f"(distance: {min_distance:.2f})")
        return closest_zone

    # ---- Bay Lock Operations ----
    def try_acquire_bay_lock(self, bay_id: str, robot_id: str, timeout_seconds: int = 600) -> bool:
        try:
            # Proactively clear expired locks to avoid stale occupancy
            try:
                self.cleanup_expired_bay_locks()
            except Exception:
                pass
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Attempt insert; if exists, check ownership and refresh heartbeat if owned by this robot
                    try:
                        cur.execute(
                            """
                            INSERT INTO bay_locks (bay_id, robot_id, locked_at, heartbeat_at, lock_timeout_seconds)
                            VALUES (%s, %s, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP, %s)
                            """,
                            (bay_id, robot_id, int(timeout_seconds))
                        )
                        conn.commit()
                        logger.debug("Acquired bay lock %s by %s", bay_id, robot_id)
                        return True
                    except psycopg2.IntegrityError:
                        conn.rollback()
                        # Lock exists; check if it's ours and refresh heartbeat
                        try:
                            cur.execute("SELECT robot_id FROM bay_locks WHERE bay_id = %s", (bay_id,))
                            row = cur.fetchone()
                            if row and row[0] == robot_id:
                                cur.execute(
                                    "UPDATE bay_locks SET heartbeat_at = CURRENT_TIMESTAMP, lock_timeout_seconds = %s WHERE bay_id = %s",
                                    (int(timeout_seconds), bay_id)
                                )
                                conn.commit()
                                logger.debug("Refreshed existing bay lock %s for %s", bay_id, robot_id)
                                return True
                            return False
                        except Exception:
                            conn.rollback()
                            return False
        except Exception as e:
            logger.error("Failed to acquire bay lock %s by %s: %s", bay_id, robot_id, e)
            raise SimulationDataServiceError(f"Failed to acquire bay lock: {e}")

    def release_bay_lock(self, bay_id: str, robot_id: str) -> bool:
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        DELETE FROM bay_locks WHERE bay_id = %s AND robot_id = %s
                        """,
                        (bay_id, robot_id)
                    )
                    released = cur.rowcount > 0
                    conn.commit()
                    print(f"SimulationDataService {bay_id}: release_bay_lock: {released}") # TODO: remove
                    return released
        except Exception as e:
            logger.error("Failed to release bay lock %s by %s: %s", bay_id, robot_id, e)
            raise SimulationDataServiceError(f"Failed to release bay lock: {e}")

    def heartbeat_bay_lock(self, bay_id: str, robot_id: str) -> bool:
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        UPDATE bay_locks SET heartbeat_at = CURRENT_TIMESTAMP WHERE bay_id = %s AND robot_id = %s
                        """,
                        (bay_id, robot_id)
                    )
                    updated = cur.rowcount > 0
                    conn.commit()
                    return updated
        except Exception as e:
            logger.error("Failed to heartbeat bay lock %s by %s: %s", bay_id, robot_id, e)
            raise SimulationDataServiceError(f"Failed to heartbeat bay lock: {e}")

    def cleanup_expired_bay_locks(self) -> int:
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("SELECT COUNT(*) FROM bay_locks WHERE CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)")
                    # Purge expired
                    cur.execute("DELETE FROM bay_locks WHERE CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)")
                    deleted = cur.rowcount
                    conn.commit()
                    return int(deleted)
        except Exception as e:
            logger.error("Failed to cleanup expired bay locks: %s", e)
            raise SimulationDataServiceError(f"Failed to cleanup expired bay locks: {e}")

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
        try:
            with self._get_connection() as conn:
                with conn.cursor(cursor_factory=RealDictCursor) as cur:
                    cur.execute(
                        """
                        SELECT robot_id, locked_at, heartbeat_at, lock_timeout_seconds
                        FROM bay_locks
                        WHERE bay_id = %s
                        """,
                        (bay_id,)
                    )
                    row = cur.fetchone()

                    if row:
                        # Convert datetime objects to timestamps for consistency
                        return {
                            'robot_id': row['robot_id'],
                            'locked_at': row['locked_at'].timestamp() if row['locked_at'] else None,
                            'heartbeat_at': row['heartbeat_at'].timestamp() if row['heartbeat_at'] else None,
                            'lock_timeout_seconds': row['lock_timeout_seconds']
                        }
                    else:
                        # No lock exists for this bay
                        return None

        except Exception as e:
            logger.error("Failed to get bay lock info for %s: %s", bay_id, e)
            raise SimulationDataServiceError(f"Failed to get bay lock info: {e}")

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
                    # Prefer shelves with highest available (quantity - pending)
                    cur.execute("""
                        SELECT si.shelf_id
                        FROM shelf_inventory si
                        WHERE si.item_id = %s AND (si.quantity - si.pending) > 0
                        ORDER BY (si.quantity - si.pending) DESC, si.quantity DESC
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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Clear existing shelves if requested
                    if clear_existing:
                        # Handle foreign key constraints by deleting in proper order
                        cur.execute("DELETE FROM shelf_inventory")
                        # Check if task_shelf_options table exists and clear it
                        cur.execute("""
                            SELECT EXISTS (
                                SELECT FROM information_schema.tables 
                                WHERE table_name = 'task_shelf_options'
                            )
                        """)
                        if cur.fetchone()['exists']:
                            cur.execute("DELETE FROM task_shelf_options")
                        cur.execute("DELETE FROM shelves")
                        logger.info("Cleared existing shelves and inventory")
                    
                    # Get shelf positions from warehouse map
                    shelves_created = 0
                    
                    for shelf_id, (grid_x, grid_y) in self.warehouse_map.shelves.items():
                        # Convert grid coordinates to world coordinates
                        world_x = grid_x * self.warehouse_map.grid_size + self.warehouse_map.grid_size / 2
                        world_y = grid_y * self.warehouse_map.grid_size + self.warehouse_map.grid_size / 2
                        
                        # Insert shelf into database
                        cur.execute("""
                            INSERT INTO shelves (shelf_id, position_x, position_y)
                            VALUES (%s, %s, %s)
                            ON CONFLICT (shelf_id) DO NOTHING
                        """, (shelf_id, world_x, world_y))
                        
                        if cur.rowcount > 0:
                            shelves_created += 1
                    
                    conn.commit()
                    logger.info(f"Created {shelves_created} shelves from warehouse map")
                    return shelves_created
                    
        except Exception as e:
            logger.error(f"Failed to create shelves from map: {e}")
            raise SimulationDataServiceError(f"Shelf creation failed: {e}")
    
    def populate_inventory(self, inventory_data: List[Dict[str, Any]]) -> int:
        """
        Populate inventory with items and assign them to shelves.
        
        Args:
            inventory_data: List of inventory data dictionaries with format:
                {
                    'item_id': str,
                    'name': str,
                    'description': str (optional),
                    'category': str (optional),
                    'shelf_id': str,
                    'quantity': int
                }
            
        Returns:
            int: Number of inventory entries created
            
        Raises:
            SimulationDataServiceError: If inventory population fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    inventory_created = 0
                    
                    for item_data in inventory_data:
                        # Insert item if it doesn't exist (let defaults handle created_at/updated_at)
                        cur.execute("""
                            INSERT INTO items (item_id, name)
                            VALUES (%s, %s)
                            ON CONFLICT (item_id) DO NOTHING
                        """, (item_data['item_id'], item_data['name']))
                        
                        # Insert inventory on shelf
                        cur.execute("""
                            INSERT INTO shelf_inventory (shelf_id, item_id, quantity)
                            VALUES (%s, %s, %s)
                            ON CONFLICT (shelf_id, item_id)
                            DO UPDATE SET quantity = EXCLUDED.quantity
                        """, (
                            item_data['shelf_id'],
                            item_data['item_id'],
                            item_data['quantity']
                        ))
                        
                        if cur.rowcount > 0:
                            inventory_created += 1
                    
                    conn.commit()
                    logger.info(f"Populated {inventory_created} inventory entries")
                    return inventory_created
                    
        except Exception as e:
            logger.error(f"Failed to populate inventory: {e}")
            logger.error(f"Error type: {type(e)}")
            logger.error(f"Error details: {str(e)}")
            raise SimulationDataServiceError(f"Inventory population failed: {e}")
    
    def get_inventory_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive inventory statistics.
        
        Returns:
            Dict[str, Any]: Inventory statistics including:
                - total_shelves: Number of shelves
                - total_items: Number of unique items
                - total_quantity: Total quantity across all shelves
                - items_by_category: Items grouped by category
                - low_stock_items: Items with quantity < 5
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Get basic statistics
                    cur.execute("""
                        SELECT 
                            COUNT(DISTINCT s.shelf_id) as total_shelves,
                            COUNT(DISTINCT i.item_id) as total_items,
                            COALESCE(SUM(si.quantity), 0) as total_quantity
                        FROM shelves s
                        LEFT JOIN shelf_inventory si ON s.shelf_id = si.shelf_id
                        LEFT JOIN items i ON si.item_id = i.item_id
                    """)
                    
                    basic_stats = cur.fetchone()
                    
                    # Get low stock items
                    cur.execute("""
                        SELECT i.item_id, i.name, si.quantity, si.shelf_id
                        FROM shelf_inventory si
                        JOIN items i ON si.item_id = i.item_id
                        WHERE si.quantity < 5
                        ORDER BY si.quantity ASC
                    """)
                    
                    low_stock_items = cur.fetchall()
                    
                    return {
                        'total_shelves': basic_stats['total_shelves'],
                        'total_items': basic_stats['total_items'],
                        'total_quantity': basic_stats['total_quantity'],
                        'low_stock_items': [
                            {
                                'item_id': item['item_id'],
                                'name': item['name'],
                                'quantity': item['quantity'],
                                'shelf_id': item['shelf_id']
                            }
                            for item in low_stock_items
                        ]
                    }
                    
        except Exception as e:
            logger.error(f"Failed to get inventory statistics: {e}")
            raise SimulationDataServiceError(f"Failed to get inventory statistics: {e}")
    
    def get_item_inventory(self, item_id: str) -> Optional['ItemInventoryInfo']:
        """
        Get inventory information for a specific item.
        
        Args:
            item_id: Item identifier
            
        Returns:
            Optional[ItemInventoryInfo]: Item inventory information or None if not found
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Get item details
                    cur.execute("""
                        SELECT i.item_id, i.name, i.category
                        FROM items i
                        WHERE i.item_id = %s
                    """, (item_id,))
                    
                    item = cur.fetchone()
                    if not item:
                        return None
                    
                    # Get inventory across all shelves (pending-aware)
                    cur.execute("""
                        SELECT si.shelf_id, si.quantity, si.pending
                        FROM shelf_inventory si
                        WHERE si.item_id = %s
                    """, (item_id,))
                    
                    shelf_data = cur.fetchall()
                    
                    total_quantity = sum(row['quantity'] for row in shelf_data)
                    available_quantity = sum(max(0, row['quantity'] - row.get('pending', 0)) for row in shelf_data)
                    shelf_locations = [row['shelf_id'] for row in shelf_data]
                    
                    return ItemInventoryInfo(
                        item_id=item_id,
                        available_quantity=available_quantity,
                        total_quantity=total_quantity,
                        reserved_quantity=total_quantity - available_quantity,
                        shelf_locations=shelf_locations
                    )
                    
        except Exception as e:
            logger.error(f"Failed to get item inventory for {item_id}: {e}")
            raise SimulationDataServiceError(f"Failed to get item inventory: {e}")

    # --- Reservation primitives (Phase 2) ---
    def reserve_inventory(self, shelf_id: str, item_id: str, quantity: int) -> bool:
        """
        Atomically reserve (pend) inventory on a shelf when available.
        Returns True on success, False if insufficient availability.
        """
        if quantity <= 0:
            return True
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        UPDATE shelf_inventory
                        SET pending = pending + %s,
                            updated_at = CURRENT_TIMESTAMP
                        WHERE shelf_id = %s AND item_id = %s
                              AND (quantity - pending) >= %s
                        """,
                        (quantity, shelf_id, item_id, quantity)
                    )
                    if cur.rowcount == 1:
                        conn.commit()
                        logger.debug(f"Reserved {quantity} of {item_id} on {shelf_id}")
                        return True
                    conn.rollback()
                    logger.warning(f"Failed to reserve {quantity} of {item_id} on {shelf_id} - insufficient availability")
                    return False
        except Exception as e:
            logger.error(f"Failed to reserve inventory {item_id} on {shelf_id}: {e}")
            raise SimulationDataServiceError(f"reserve_inventory failed: {e}")

    def release_inventory(self, shelf_id: str, item_id: str, quantity: int) -> bool:
        """
        Atomically release (unpend) a previously reserved quantity.
        Returns True on success, False otherwise.
        """
        if quantity <= 0:
            return True
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        UPDATE shelf_inventory
                        SET pending = pending - %s,
                            updated_at = CURRENT_TIMESTAMP
                        WHERE shelf_id = %s AND item_id = %s
                              AND pending >= %s
                        """,
                        (quantity, shelf_id, item_id, quantity)
                    )
                    if cur.rowcount == 1:
                        conn.commit()
                        logger.debug(f"Released {quantity} of {item_id} on {shelf_id}")
                        return True
                    conn.rollback()
                    logger.warning(f"Failed to release {quantity} of {item_id} on {shelf_id} - insufficient pending quantity")
                    return False
        except Exception as e:
            logger.error(f"Failed to release inventory {item_id} on {shelf_id}: {e}")
            raise SimulationDataServiceError(f"release_inventory failed: {e}")

    def consume_inventory(self, shelf_id: str, item_id: str, quantity: int) -> bool:
        """
        Atomically consume reserved inventory on successful completion.
        Returns True on success, False otherwise.
        """
        if quantity <= 0:
            return True
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        UPDATE shelf_inventory
                        SET pending = pending - %s,
                            quantity = quantity - %s,
                            updated_at = CURRENT_TIMESTAMP
                        WHERE shelf_id = %s AND item_id = %s
                              AND pending >= %s AND quantity >= %s
                        """,
                        (quantity, quantity, shelf_id, item_id, quantity, quantity)
                    )
                    if cur.rowcount == 1:
                        conn.commit()
                        logger.info(f"Consumed {quantity} of {item_id} on {shelf_id}")
                        return True
                    conn.rollback()
                    logger.warning(f"Failed to consume {quantity} of {item_id} on {shelf_id} - insufficient pending or total quantity")
                    return False
        except Exception as e:
            logger.error(f"Failed to consume inventory {item_id} on {shelf_id}: {e}")
            raise SimulationDataServiceError(f"consume_inventory failed: {e}")
    
    def get_item_shelf_locations(self, item_id: str) -> List['ItemShelfLocation']:
        """
        Get all shelf locations where a specific item is stored.
        
        Args:
            item_id: Item identifier
            
        Returns:
            List[ItemShelfLocation]: List of shelf locations containing the item
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT si.shelf_id, si.quantity, si.pending
                        FROM shelf_inventory si
                        WHERE si.item_id = %s
                        ORDER BY (si.quantity - si.pending) DESC, si.quantity DESC
                    """, (item_id,))
                    
                    locations = []
                    for row in cur.fetchall():
                        locations.append(ItemShelfLocation(
                            item_id=item_id,
                            shelf_id=row['shelf_id'],
                            quantity=max(0, row['quantity'] - row.get('pending', 0)),
                            last_updated=time.time()
                        ))
                    
                    return locations
                    
        except Exception as e:
            logger.error(f"Failed to get shelf locations for item {item_id}: {e}")
            raise SimulationDataServiceError(f"Failed to get shelf locations: {e}")
    
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
                    deleted_count = self._scalar(result, 0)
                    
                    conn.commit()
                    logger.debug(f"Cleaned up {deleted_count} expired blocked cells")
                    return deleted_count
                    
        except Exception as e:
            logger.error(f"Failed to cleanup expired blocks: {e}")
            raise SimulationDataServiceError(f"Failed to cleanup expired blocks: {e}")
    
    # Conflict Box Lock Operations
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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Use the database function we created
                    cur.execute("SELECT try_acquire_conflict_box_lock(%s, %s, %s)", 
                               (box_id, robot_id, priority))
                    result = cur.fetchone()
                    success = bool(self._scalar(result, False))
                    
                    conn.commit()
                    
                    if success:
                        logger.debug(f"Acquired conflict box lock: {box_id} by robot {robot_id} (priority {priority})")
                    else:
                        logger.debug(f"Failed to acquire conflict box lock: {box_id} by robot {robot_id}")
                    
                    return success
                    
        except Exception as e:
            logger.error(f"Failed to acquire conflict box lock {box_id}: {e}")
            raise SimulationDataServiceError(f"Failed to acquire conflict box lock: {e}")

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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Use the database function we created
                    logger.debug(f"[Diag][SDS] Requesting DB release for {box_id} by {robot_id}")
                    cur.execute("SELECT release_conflict_box_lock(%s, %s)", 
                               (box_id, robot_id))
                    result = cur.fetchone()
                    success = bool(self._scalar(result, False))
                    
                    conn.commit()
                    
                    if success:
                        logger.debug(f"[Diag][SDS] DB release success for {box_id} by {robot_id}")
                    else:
                        logger.debug(f"[Diag][SDS] DB release failed for {box_id} by {robot_id}")
                    
                    return success
                    
        except Exception as e:
            logger.error(f"Failed to release conflict box lock {box_id}: {e}")
            raise SimulationDataServiceError(f"Failed to release conflict box lock: {e}")

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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Use the database function we created
                    cur.execute("SELECT heartbeat_conflict_box_lock(%s, %s)", 
                               (box_id, robot_id))
                    result = cur.fetchone()
                    success = bool(self._scalar(result, False))
                    
                    conn.commit()
                    
                    if success:
                        logger.debug(f"Updated heartbeat for conflict box lock: {box_id} by robot {robot_id}")
                    else:
                        logger.debug(f"Failed to update heartbeat for conflict box lock: {box_id} by robot {robot_id}")
                    
                    return success
                    
        except Exception as e:
            logger.error(f"Failed to heartbeat conflict box lock {box_id}: {e}")
            raise SimulationDataServiceError(f"Failed to heartbeat conflict box lock: {e}")

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
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT locked_by_robot
                        FROM conflict_box_locks
                        WHERE box_id = %s
                    """, (box_id,))
                    
                    result = cur.fetchone()
                    owner = result[0] if result else None
                    
                    logger.debug(f"Conflict box {box_id} lock owner: {owner}")
                    return owner
                    
        except Exception as e:
            logger.error(f"Failed to get conflict box lock owner {box_id}: {e}")
            raise SimulationDataServiceError(f"Failed to get conflict box lock owner: {e}")

    def cleanup_expired_conflict_box_locks(self) -> int:
        """
        Remove expired conflict box lock entries.
        
        Returns:
            int: Number of expired locks removed
            
        Raises:
            SimulationDataServiceError: If cleanup operation fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Use the database function we created
                    cur.execute("SELECT cleanup_expired_conflict_box_locks()")
                    result = cur.fetchone()
                    deleted_count = result[0] if result else 0
                    
                    conn.commit()
                    logger.debug(f"Cleaned up {deleted_count} expired conflict box locks")
                    return deleted_count
                    
        except Exception as e:
            logger.error(f"Failed to cleanup expired conflict box locks: {e}")
            raise SimulationDataServiceError(f"Failed to cleanup expired conflict box locks: {e}")
    
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