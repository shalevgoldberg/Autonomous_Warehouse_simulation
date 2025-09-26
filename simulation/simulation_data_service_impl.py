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
import random
from contextlib import contextmanager
from typing import Optional, List, Dict, Any, Tuple
from dataclasses import dataclass
from urllib.parse import urlparse

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
from utils.database_config import get_database_config, DatabaseConfigError
from interfaces.graph_persistence_interface import GraphPersistenceResult
from warehouse.impl.graph_persistence_impl import GraphPersistenceImpl
from pathlib import Path
from interfaces.navigation_types import LaneRec, BoxRec, Point, LaneDirection
from warehouse.map import WarehouseMap

# KPI recording infrastructure
from kpi.kpi_recorder_db_impl import KpiRecorderDbImpl
from kpi.kpi_recorder_interface import KpiEvent
import csv


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

        Database configuration priority:
        1. DATABASE_URL environment variable (recommended)
        2. Individual WAREHOUSE_DB_* environment variables
        3. Constructor parameters (for backward compatibility)
        
        Args:
            warehouse_map: Warehouse layout for map data
            db_host: Database host (fallback)
            db_port: Database port (fallback)
            db_name: Database name (fallback)
            db_user: Database user (fallback)
            db_password: Database password (fallback)
            pool_size: Connection pool size
        """
        self.warehouse_map = warehouse_map
        
        # Get database configuration using shared utility
        try:
            db_config = get_database_config(
                db_host=db_host,
                db_port=db_port,
                db_name=db_name,
                db_user=db_user,
                db_password=db_password
            )
        except DatabaseConfigError as e:
            raise SimulationDataServiceError(str(e))
        
        # Database connection parameters
        self.db_params = {
            'host': db_config['host'],
            'port': db_config['port'],
            'database': db_config['database'],
            'user': db_config['user'],
            'password': db_config['password'],
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
        
        # Per-simulation run identifier for KPI scoping
        self._current_simulation_run_id: str = str(uuid.uuid4())

        # Initialize KPI recorder (non-blocking, background persistence)
        # Note: We use conservative defaults to avoid introducing new config dependencies here.
        # Skip initialization in CLI mode for faster shutdown
        if os.getenv('CLI_MODE') == '1':
            self._kpi_recorder = None
            logger.info("KPI recorder disabled (CLI mode)")
        else:
            try:
                self._kpi_recorder = KpiRecorderDbImpl(
                    connection_getter=self._get_connection,
                    flush_interval_sec=5.0,  # Reduced from 60.0 for faster CLI shutdown
                    max_batch_size=256,
                    queue_capacity=8192,
                    logger=logging.getLogger(f"{__name__}.KPI"),
                )
                logger.info("KPI recorder initialized (DB-backed)")
            except Exception as e:
                # Fallback: disable recorder; direct DB writes will be used with explicit logging
                self._kpi_recorder = None
                logger.error(f"Failed to initialize KPI recorder, falling back to direct DB writes: {e}")

        logger.info(f"SimulationDataService initialized with database {db_name}@{db_host}:{db_port}")

        # Inject KPI recorder into SharedMuJoCoEngine when present for collision KPIs
        try:
            # Some demos pass the engine into robots and keep a reference accessible
            # If the warehouse map or other subsystem exposes the engine, skip. Here we only set when possible.
            from simulation.shared_mujoco_engine import SharedMuJoCoEngine
            for attr_name in dir(self):
                try:
                    candidate = getattr(self, attr_name)
                    if isinstance(candidate, SharedMuJoCoEngine):
                        try:
                            candidate.set_kpi_recorder(
                                self._kpi_recorder,
                                run_id_provider=lambda: getattr(self, "_current_simulation_run_id", None)
                            )
                            logger.info("Injected KPI recorder into SharedMuJoCoEngine for collision KPIs")
                        except Exception as ie:
                            logger.error(f"Failed to inject KPI recorder into engine: {ie}")
                except Exception:
                    continue
        except Exception:
            # Best-effort only; engines are typically created in demos and can call set_kpi_recorder directly
            pass
    
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
                                INSERT INTO conflict_box_locks (box_id, locked_by_robot, priority)
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
                                SET heartbeat_at = CURRENT_TIMESTAMP
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

                    # Always derive dimensions from the current warehouse_map to avoid stale defaults
                    map_data = MapData(
                        width=int(self.warehouse_map.width),
                        height=int(self.warehouse_map.height),
                        cell_size=float(self.warehouse_map.grid_size),
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
    
    def _select_shelf_for_item(self, item_data: Dict[str, Any],
                              assignment_strategy: str,
                              strategy_options: Optional[Dict[str, Any]] = None,
                              available_shelves: Optional[List[str]] = None) -> str:
        """
        Select appropriate shelf based on assignment strategy.

        Args:
            item_data: Item data dictionary
            assignment_strategy: Strategy to use for assignment
            strategy_options: Additional configuration for the strategy

        Returns:
            str: Selected shelf ID

        Raises:
            SimulationDataServiceError: If no suitable shelf can be found
        """
        try:
            # Prefer shelves provided by caller (DB source of truth)
            if available_shelves is None:
                try:
                    available_shelves = self._get_available_shelves_from_db()
                except Exception as e:
                    logger.warning(f"Failed to load shelves from DB inside selector, falling back to in-memory map: {e}")
                    available_shelves = list(self.warehouse_map.shelves.keys())

            if not available_shelves:
                # As a final compatibility fallback, try in-memory map once more
                logger.warning("No shelves provided/found; falling back to in-memory map shelves (compatibility mode)")
                available_shelves = list(self.warehouse_map.shelves.keys())
            if not available_shelves:
                raise SimulationDataServiceError("No shelves available for inventory assignment")

            if assignment_strategy == "random":
                return self._get_random_shelf(available_shelves)
            elif assignment_strategy == "load_balanced":
                # For now, fall back to random since load balancing isn't implemented yet
                logger.warning(f"Load balanced strategy not yet implemented, falling back to random assignment")
                return self._get_random_shelf(available_shelves)
            elif assignment_strategy == "category_based":
                # For now, fall back to random since category-based isn't implemented yet
                logger.warning(f"Category-based strategy not yet implemented, falling back to random assignment")
                return self._get_random_shelf(available_shelves)
            elif assignment_strategy == "proximity_based":
                # For now, fall back to random since proximity-based isn't implemented yet
                logger.warning(f"Proximity-based strategy not yet implemented, falling back to random assignment")
                return self._get_random_shelf(available_shelves)
            else:
                # This should not happen due to validation above, but just in case
                logger.warning(f"Unknown assignment strategy '{assignment_strategy}', falling back to random assignment")
                return self._get_random_shelf(available_shelves)

        except SimulationDataServiceError:
            raise
        except Exception as e:
            logger.error(f"Failed to select shelf for item {item_data.get('item_id', 'unknown')}: {e}")
            # Final fallback to random assignment
            logger.warning(f"Falling back to random assignment due to error: {e}")
            try:
                # Attempt random from DB; if that fails, map fallback
                fallback_shelves = None
                try:
                    fallback_shelves = self._get_available_shelves_from_db()
                except Exception:
                    fallback_shelves = list(self.warehouse_map.shelves.keys())
                if not fallback_shelves:
                    fallback_shelves = list(self.warehouse_map.shelves.keys())
                return self._get_random_shelf(fallback_shelves)
            except Exception:
                raise SimulationDataServiceError(f"Unable to assign shelf for item {item_data.get('item_id', 'unknown')}")

    def _get_random_shelf(self, available_shelves: List[str]) -> str:
        """
        Select random available shelf.

        Args:
            available_shelves: List of available shelf IDs

        Returns:
            str: Randomly selected shelf ID
        """
        return random.choice(available_shelves)

    def _get_available_shelves_from_db(self) -> List[str]:
        """
        Read available shelf IDs from the database.
        Returns: List of shelf_id strings.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor(cursor_factory=RealDictCursor) as cur:
                    cur.execute("SELECT shelf_id FROM shelves ORDER BY shelf_id")
                    rows = cur.fetchall()
                    return [row['shelf_id'] for row in rows]
        except Exception as e:
            logger.error(f"Failed to read shelves from DB: {e}")
            raise SimulationDataServiceError(f"Failed to read shelves: {e}")

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
        try:
            # Validate strategy options
            if strategy_options is None:
                strategy_options = {}

            # Validate assignment strategy
            valid_strategies = ["random", "load_balanced", "category_based", "proximity_based"]
            if assignment_strategy not in valid_strategies:
                logger.warning(f"Invalid assignment strategy '{assignment_strategy}', falling back to 'random'")
                assignment_strategy = "random"

            # Get available shelves from DB (never rely on in-memory map for data ops)
            try:
                available_shelves = self._get_available_shelves_from_db()
            except Exception as e:
                logger.warning(f"Failed to load shelves from DB, falling back to in-memory map: {e}")
                available_shelves = list(self.warehouse_map.shelves.keys())
            # If DB query succeeded but returned no shelves, try in-memory map as a compatibility fallback
            if not available_shelves:
                logger.warning("No shelves found in DB; falling back to in-memory map shelves (compatibility mode)")
                available_shelves = list(self.warehouse_map.shelves.keys())
            if not available_shelves:
                logger.error("No shelves available for inventory assignment.")
                return 0
            if not available_shelves:
                raise SimulationDataServiceError("No shelves available for inventory assignment")

            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    inventory_created = 0
                    
                    for item_data in inventory_data:
                        # Validate required fields
                        if 'item_id' not in item_data or 'name' not in item_data or 'quantity' not in item_data:
                            logger.warning(f"Skipping invalid item data (missing required fields): {item_data}")
                            continue

                        # Validate quantity is positive
                        if not isinstance(item_data['quantity'], int) or item_data['quantity'] <= 0:
                            logger.warning(f"Skipping item with invalid quantity: {item_data}")
                            continue

                        # Use explicit shelf_id if provided, otherwise apply strategy
                        shelf_id = item_data.get('shelf_id')
                        if not shelf_id:
                            try:
                                shelf_id = self._select_shelf_for_item(
                                    item_data,
                                    assignment_strategy,
                                    strategy_options,
                                    available_shelves=available_shelves
                                )
                                logger.debug(f"Assigned item '{item_data['item_id']}' to shelf '{shelf_id}' using {assignment_strategy} strategy")
                            except Exception as e:
                                logger.error(f"Failed to assign shelf for item '{item_data['item_id']}': {e}")
                                continue

                        # Validate shelf exists
                        if shelf_id not in available_shelves:
                            logger.warning(f"Shelf '{shelf_id}' not found in DB shelves, skipping item '{item_data['item_id']}'")
                            continue

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
                            shelf_id,
                            item_data['item_id'],
                            item_data['quantity']
                        ))
                        
                        if cur.rowcount > 0:
                            inventory_created += 1
                    
                    conn.commit()
                    logger.info(f"Populated {inventory_created} inventory entries using {assignment_strategy} strategy")
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
    
    def export_inventory_status_csv(self, filename: str) -> bool:
        """
        Export complete inventory status including reserved quantities to CSV.

        CSV Format:
        - shelf_id,item_id,item_name,total_quantity,reserved_quantity,available_quantity,created_at,updated_at

        Plus summary statistics at the end.

        Args:
            filename: Output CSV filename

        Returns:
            bool: True on success, False on failure
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor(cursor_factory=RealDictCursor) as cur:
                    # Get complete inventory status with item details
                    cur.execute("""
                        SELECT
                            si.shelf_id,
                            si.item_id,
                            i.name as item_name,
                            si.quantity as total_quantity,
                            COALESCE(si.pending, 0) as reserved_quantity,
                            (si.quantity - COALESCE(si.pending, 0)) as available_quantity,
                            si.created_at,
                            si.updated_at
                        FROM shelf_inventory si
                        JOIN items i ON si.item_id = i.item_id
                        ORDER BY si.shelf_id, si.item_id
                    """)

                    inventory_data = cur.fetchall()

                    # Get summary statistics
                    cur.execute("""
                        SELECT
                            COUNT(DISTINCT si.item_id) as total_items,
                            COUNT(DISTINCT si.shelf_id) as total_shelves,
                            SUM(si.quantity) as total_quantity,
                            SUM(COALESCE(si.pending, 0)) as total_reserved,
                            SUM(si.quantity - COALESCE(si.pending, 0)) as total_available
                        FROM shelf_inventory si
                    """)

                    summary = cur.fetchone()

            # Write CSV file
            import csv
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)

                # Write header
                writer.writerow([
                    'shelf_id', 'item_id', 'item_name',
                    'total_quantity', 'reserved_quantity', 'available_quantity',
                    'created_at', 'updated_at'
                ])

                # Write inventory data
                for row in inventory_data:
                    writer.writerow([
                        row['shelf_id'], row['item_id'], row['item_name'],
                        row['total_quantity'], row['reserved_quantity'], row['available_quantity'],
                        row['created_at'], row['updated_at']
                    ])

                # Write summary statistics
                writer.writerow([])
                writer.writerow(['SUMMARY'])
                writer.writerow(['Total Items', summary['total_items']])
                writer.writerow(['Total Shelves Used', summary['total_shelves']])
                writer.writerow(['Total Quantity', summary['total_quantity']])
                writer.writerow(['Total Reserved', summary['total_reserved']])
                writer.writerow(['Total Available', summary['total_available']])

            logger.info(f"Inventory status exported to {filename}")
            return True

        except Exception as e:
            logger.error(f"Failed to export inventory status to {filename}: {e}")
            return False

    def clear_all_reservations(self) -> int:
        """
        Clear all reserved (pending) quantities across all shelves without consuming stock.
        Returns the number of rows affected (those that previously had pending > 0).
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Count rows that currently have pending > 0
                    cur.execute("SELECT COUNT(*) AS cnt FROM shelf_inventory WHERE COALESCE(pending, 0) > 0")
                    rows_with_pending = cur.fetchone()['cnt']
                    if rows_with_pending == 0:
                        return 0
                    # Zero all pending values
                    cur.execute(
                        """
                        UPDATE shelf_inventory
                        SET pending = 0,
                            updated_at = CURRENT_TIMESTAMP
                        WHERE COALESCE(pending, 0) > 0
                        """
                    )
                    conn.commit()
                    logger.info(f"Cleared reserved quantities on {rows_with_pending} shelf_inventory rows")
                    return rows_with_pending
        except Exception as e:
            logger.error(f"Failed to clear reserved quantities: {e}")
            raise SimulationDataServiceError(f"Failed to clear reserved quantities: {e}")

    def clear_all_inventory(self) -> int:
        """
        Remove all inventory rows from shelf_inventory.
        Returns: number of rows deleted.
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("SELECT COUNT(*) AS cnt FROM shelf_inventory")
                    before = cur.fetchone()['cnt']
                    cur.execute("DELETE FROM shelf_inventory")
                    conn.commit()
                    logger.info(f"Cleared all inventory rows: {before}")
                    return before
        except Exception as e:
            logger.error(f"Failed to clear all inventory: {e}")
            raise SimulationDataServiceError(f"Failed to clear all inventory: {e}")

    def delete_inventory_for_items(self, item_ids: List[str], purge_items: bool = False) -> int:
        """
        Delete inventory rows for the given item_ids. Optionally purge items table entries
        that no longer have inventory.
        Returns: number of shelf_inventory rows deleted.
        """
        if not item_ids:
            logger.warning("delete_inventory_for_items called with empty item_ids; nothing to delete")
            return 0
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT COUNT(*) AS cnt FROM shelf_inventory WHERE item_id = ANY(%s)
                    """, (item_ids,))
                    before = cur.fetchone()['cnt']
                    cur.execute("""
                        DELETE FROM shelf_inventory WHERE item_id = ANY(%s)
                    """, (item_ids,))
                    deleted = cur.rowcount
                    if purge_items:
                        # Delete items that no longer have inventory
                        cur.execute("""
                            DELETE FROM items i
                            WHERE i.item_id = ANY(%s)
                              AND NOT EXISTS (
                                  SELECT 1 FROM shelf_inventory si WHERE si.item_id = i.item_id
                              )
                        """, (item_ids,))
                    conn.commit()
                    logger.info(f"Deleted {deleted} inventory rows for items {item_ids}")
                    return before if before == deleted else deleted
        except Exception as e:
            logger.error(f"Failed to delete inventory for items {item_ids}: {e}")
            raise SimulationDataServiceError(f"Failed to delete inventory for items: {e}")

    def move_inventory(self, item_id: str, from_shelf_id: str, to_shelf_id: str, quantity: int) -> bool:
        """
        Atomically move 'quantity' of item from one shelf to another respecting pending.
        """
        if quantity <= 0:
            logger.warning(f"move_inventory called with non-positive quantity: {quantity}")
            return False
        if from_shelf_id == to_shelf_id:
            logger.warning("move_inventory called with same source and destination shelf; no-op")
            return True
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Ensure shelves exist
                    cur.execute("SELECT 1 FROM shelves WHERE shelf_id = %s", (from_shelf_id,))
                    if not cur.fetchone():
                        logger.warning(f"Source shelf {from_shelf_id} does not exist; aborting move")
                        return False
                    cur.execute("SELECT 1 FROM shelves WHERE shelf_id = %s", (to_shelf_id,))
                    if not cur.fetchone():
                        logger.warning(f"Destination shelf {to_shelf_id} does not exist; aborting move")
                        return False

                    # Decrement from source if sufficient available (quantity - pending)
                    cur.execute(
                        """
                        UPDATE shelf_inventory
                        SET quantity = quantity - %s, updated_at = CURRENT_TIMESTAMP
                        WHERE shelf_id = %s AND item_id = %s
                              AND (quantity - COALESCE(pending, 0)) >= %s
                        """,
                        (quantity, from_shelf_id, item_id, quantity)
                    )
                    if cur.rowcount != 1:
                        conn.rollback()
                        logger.warning(f"Insufficient available to move {quantity} of {item_id} from {from_shelf_id}")
                        return False

                    # Increment destination (insert or update)
                    cur.execute(
                        """
                        INSERT INTO shelf_inventory (shelf_id, item_id, quantity)
                        VALUES (%s, %s, %s)
                        ON CONFLICT (shelf_id, item_id)
                        DO UPDATE SET quantity = shelf_inventory.quantity + EXCLUDED.quantity,
                                      updated_at = CURRENT_TIMESTAMP
                        """,
                        (to_shelf_id, item_id, quantity)
                    )
                    conn.commit()
                    logger.info(f"Moved {quantity} of {item_id} from {from_shelf_id} to {to_shelf_id}")
                    return True
        except Exception as e:
            logger.error(f"Failed to move inventory: {e}")
            raise SimulationDataServiceError(f"Failed to move inventory: {e}")

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
        # Prefer non-blocking KPI recorder when available
        if hasattr(self, "_kpi_recorder") and self._kpi_recorder is not None:
            try:
                self._kpi_recorder.record_event(KpiEvent(
                    event_type=event_type,
                    robot_id=robot_id,
                    event_data=event_data,
                    simulation_run_id=self._current_simulation_run_id,
                ))
                # Ensure visibility for immediate queries (tests, end-of-sim summaries)
                # This is a best-effort synchronous flush and should not be called from high-frequency loops.
                self._kpi_recorder.flush()
                return
            except Exception as e:
                # Explicitly log fallback and proceed with direct DB write
                logger.error(
                    f"KPI recorder flush failed, falling back to direct DB write for event {event_type} (robot {robot_id}): {e}"
                )

        # Fallback path: direct, synchronous DB writes preserving prior semantics
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    event_id = self._current_simulation_run_id

                    cur.execute(
                        """
                        INSERT INTO simulation_metrics (metric_name, metric_value, simulation_run_id)
                        VALUES (%s, %s, %s)
                        """,
                        (f"{event_type}_{robot_id}", 1.0, event_id),
                    )

                    for key, value in event_data.items():
                        if isinstance(value, (int, float)):
                            cur.execute(
                                """
                                INSERT INTO simulation_metrics (metric_name, metric_value, simulation_run_id)
                                VALUES (%s, %s, %s)
                                """,
                                (f"{event_type}_{key}", float(value), event_id),
                            )

                conn.commit()
                logger.debug(f"Logged event (fallback direct write): {event_type} for robot {robot_id}")
        except Exception as e:
            logger.error(f"Failed to log event (fallback) {event_type} for robot {robot_id}: {e}")
    
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
                        SELECT box_id, center_x, center_y, width, height
                        FROM conflict_boxes
                        ORDER BY box_id
                    """)
                    
                    results = cur.fetchall()
                    boxes = []
                    
                    for row in results:
                        box = BoxRec(
                            box_id=row['box_id'],
                            center=Point(x=float(row['center_x']), y=float(row['center_y'])),
                            width=float(row['width']),
                            height=float(row['height'])
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
            #logger.error(f"Failed to get conflict box lock owner {box_id}: {e}")
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
                    # Check if function exists first (for debugging)
                    cur.execute("""
                        SELECT EXISTS (
                            SELECT 1 FROM information_schema.routines
                            WHERE routine_name = 'cleanup_expired_conflict_box_locks'
                            AND routine_type = 'FUNCTION'
                        )
                    """)
                    function_exists = cur.fetchone()[0]
                    logger.info(f"[DEBUG] cleanup_expired_conflict_box_locks function exists: {function_exists}")

                    if not function_exists:
                        logger.error("[DEBUG] CRITICAL: cleanup_expired_conflict_box_locks function not found in schema - this WILL corrupt connection state")
                        return 0

                    # Use the database function we created
                    logger.debug("[DEBUG] Calling cleanup_expired_conflict_box_locks() function")
                    cur.execute("SELECT cleanup_expired_conflict_box_locks()")
                    result = cur.fetchone()
                    logger.debug(f"[DEBUG] cleanup_expired_conflict_box_locks() returned: {result}")
                    deleted_count = result[0] if result else 0

                    conn.commit()
                    logger.debug(f"Cleaned up {deleted_count} expired conflict box locks")
                    return deleted_count

        except Exception as e:
            logger.error(f"[DEBUG] Failed to cleanup expired conflict box locks: {e}")
            logger.error(f"[DEBUG] Exception type: {type(e).__name__}")
            raise SimulationDataServiceError(f"Failed to cleanup expired conflict box locks: {e}")
    
    def clear_all_conflict_box_locks(self) -> int:
        """
        Clear all conflict box lock entries.

        WARNING: This method removes ALL conflict box locks regardless of state.
        Use with caution as it may cause race conditions if locks are actively held.

        Returns:
            int: Number of locks removed

        Raises:
            SimulationDataServiceError: If clear operation fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Count locks before deletion for return value
                    cur.execute("SELECT COUNT(*) FROM conflict_box_locks")
                    count_before = cur.fetchone()
                    initial_count = int(self._scalar(count_before, 0))

                    # Clear all conflict box locks
                    cur.execute("DELETE FROM conflict_box_locks")

                    conn.commit()

                    logger.warning(f"Cleared {initial_count} conflict box locks from database")
                    return initial_count

        except Exception as e:
            logger.error(f"Failed to clear all conflict box locks: {e}")
            raise SimulationDataServiceError(f"Failed to clear all conflict box locks: {e}")

    def close(self) -> None:
        """Close database connections and cleanup resources immediately."""
        try:
            # Stop KPI recorder first
            try:
                if hasattr(self, "_kpi_recorder") and self._kpi_recorder is not None:
                    self._kpi_recorder.stop()
            except Exception as e:
                logger.error(f"Error stopping KPI recorder: {e}")

            if self._connection_pool:
                with self._pool_lock:
                    # Force close all connections immediately
                    self._connection_pool.closeall()
                    self._connection_pool = None
                logger.info("SimulationDataService database connections closed")
        except Exception as e:
            logger.error(f"Error closing database connections: {e}")
    
    def __del__(self):
        """Ensure cleanup on object destruction."""
        self.close()

    # ---------------------
    # KPI Aggregation (Views/Queries)
    # ---------------------
    def get_kpi_overview(self, robot_id: Optional[str] = None, run_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Return a simple KPI overview built from simulation_metrics.

        This is read-only and does not modify schema. It leverages naming
        conventions introduced by Phase 1-2 emitters.
        """
        result: Dict[str, Any] = {
            # Overall
            "tasks_completed": 0.0,
            "tasks_failed": 0.0,
            "task_success_rate": 0.0,
            "avg_task_time_seconds": 0.0,
            # Pick-and-deliver subset
            "pd_tasks_completed": 0.0,
            "pd_tasks_failed": 0.0,
            "pd_success_rate": 0.0,
            "pd_avg_task_time_seconds": 0.0,
            # Time utilization
            "busy_time_seconds": 0.0,
            "avg_idle_percent_per_robot": 0.0,
            "avg_busy_percent_per_robot": 0.0,
            # Collision KPIs
            "collision_count": 0.0,
            "collision_rate_per_min": 0.0,
            "collision_types_robot": 0.0,
            "collision_types_shelf": 0.0,
            "collision_types_wall": 0.0,
        }
        try:
            effective_run_id = run_id or getattr(self, "_current_simulation_run_id", None)
            logger.info(f"[DEBUG] KPI aggregation starting with run_id: {effective_run_id}")

            # Test connection state before running queries
            try:
                with self._get_connection() as test_conn:
                    with test_conn.cursor() as test_cur:
                        test_cur.execute("SELECT 1 as test")
                        test_result = test_cur.fetchone()
                        logger.info(f"[DEBUG] Connection test successful: {test_result}")
            except Exception as test_e:
                logger.error(f"[DEBUG] CRITICAL: Connection test FAILED before KPI queries: {test_e}")
                logger.error("[DEBUG] CRITICAL: Database connection IS corrupted from previous operations")

            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Completed/Failed counts
                    if robot_id:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS completed,
                                       SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS failed
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND metric_name IN (%s, %s)
                                """,
                                (
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                    effective_run_id,
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                ),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS completed,
                                       SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS failed
                                FROM simulation_metrics
                                WHERE metric_name IN (%s, %s)
                                """,
                                (
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                ),
                            )
                    else:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS completed,
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS failed
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND (metric_name LIKE %s OR metric_name LIKE %s)
                                """,
                                (
                                    'task_completed_%',
                                    'task_failed_%',
                                    effective_run_id,
                                    'task_completed_%',
                                    'task_failed_%',
                                ),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS completed,
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS failed
                                FROM simulation_metrics
                                WHERE metric_name LIKE %s OR metric_name LIKE %s
                                """,
                                (
                                    'task_completed_%',
                                    'task_failed_%',
                                    'task_completed_%',
                                    'task_failed_%',
                                ),
                            )
                    raw_row = cur.fetchone()
                    logger.info(f"[DEBUG] KPI fetchone() returned: {raw_row} (type: {type(raw_row)})")
                    row = raw_row or {}
                    logger.info(f"[DEBUG] Processed row: {row} (type: {type(row)})")

                    # Check if row is not a dict (indicates corrupted connection)
                    if not isinstance(row, dict):
                        logger.error(f"[DEBUG] CRITICAL: fetchone() returned non-dict type: {type(row)} = {row}")
                        logger.error("[DEBUG] CRITICAL: This PROVES corrupted database connection state from missing cleanup function")

                    completed = float(row.get("completed", 0) or 0)
                    failed = float(row.get("failed", 0) or 0)
                    total = max(1.0, completed + failed)
                    result["tasks_completed"] = completed
                    result["tasks_failed"] = failed
                    result["task_success_rate"] = (completed / total) * 100.0

                    # Average task time (use per-robot metric when available)
                    if robot_id:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT AVG(metric_value) AS avg_time
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND metric_name = %s
                                """,
                                (effective_run_id, f"task_completed_duration_seconds_{robot_id}"),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT AVG(metric_value) AS avg_time
                                FROM simulation_metrics
                                WHERE metric_name = %s
                                """,
                                (f"task_completed_duration_seconds_{robot_id}",),
                            )
                        r = cur.fetchone() or {}
                        avg_time = r.get("avg_time")
                        if avg_time is None:
                            # Fallback to global if per-robot is absent
                            if effective_run_id:
                                cur.execute(
                                    """
                                    SELECT AVG(metric_value) AS avg_time
                                    FROM simulation_metrics
                                    WHERE simulation_run_id = %s AND metric_name = 'task_completed_duration_seconds'
                                    """,
                                    (effective_run_id,),
                                )
                            else:
                                cur.execute(
                                    """
                                    SELECT AVG(metric_value) AS avg_time
                                    FROM simulation_metrics
                                    WHERE metric_name = 'task_completed_duration_seconds'
                                    """
                                )
                            r = cur.fetchone() or {}
                            avg_time = r.get("avg_time")
                    else:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT AVG(metric_value) AS avg_time
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND metric_name = 'task_completed_duration_seconds'
                                """,
                                (effective_run_id,),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT AVG(metric_value) AS avg_time
                                FROM simulation_metrics
                                WHERE metric_name = 'task_completed_duration_seconds'
                                """
                            )
                        r = cur.fetchone() or {}
                        avg_time = r.get("avg_time")

                    result["avg_task_time_seconds"] = float(avg_time or 0.0)

                    # --- Pick-and-deliver breakdown ---
                    # Completed PD tasks: count events where is_pick_and_deliver == 1
                    if robot_id:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS pd_completed,
                                       SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS pd_failed
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND metric_name IN (%s, %s)
                                """,
                                (
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                    effective_run_id,
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                ),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS pd_completed,
                                       SUM(CASE WHEN metric_name = %s THEN 1 ELSE 0 END) AS pd_failed
                                FROM simulation_metrics
                                WHERE metric_name IN (%s, %s)
                                """,
                                (
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                    f"task_completed_{robot_id}",
                                    f"task_failed_{robot_id}",
                                ),
                            )
                    else:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS pd_completed,
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS pd_failed
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND (metric_name LIKE %s OR metric_name LIKE %s)
                                """,
                                (
                                    'task_completed_%',
                                    'task_failed_%',
                                    effective_run_id,
                                    'task_completed_%',
                                    'task_failed_%',
                                ),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS pd_completed,
                                    SUM(CASE WHEN metric_name LIKE %s THEN 1 ELSE 0 END) AS pd_failed
                                FROM simulation_metrics
                                WHERE metric_name LIKE %s OR metric_name LIKE %s
                                """,
                                (
                                    'task_completed_%',
                                    'task_failed_%',
                                    'task_completed_%',
                                    'task_failed_%',
                                ),
                            )
                    pd_row = cur.fetchone() or {}
                    # We need to filter by PD-only events: we logged a numeric flag is_pick_and_deliver.
                    # Count PD tasks by summing the flag metrics aligned with completion/failure.
                    # Global flag metrics names: task_completed_is_pick_and_deliver, task_failed_is_pick_and_deliver
                    # Per-robot flag metrics names: task_completed_is_pick_and_deliver_<robot>, task_failed_is_pick_and_deliver_<robot>
                    if robot_id:
                        key_completed = f"task_completed_is_pick_and_deliver_{robot_id}"
                        key_failed = f"task_failed_is_pick_and_deliver_{robot_id}"
                    else:
                        key_completed = "task_completed_is_pick_and_deliver"
                        key_failed = "task_failed_is_pick_and_deliver"

                    if effective_run_id:
                        cur.execute(
                            """
                            SELECT
                                COALESCE(SUM(CASE WHEN metric_name = %s THEN metric_value ELSE 0 END), 0) AS pd_completed,
                                COALESCE(SUM(CASE WHEN metric_name = %s THEN metric_value ELSE 0 END), 0) AS pd_failed
                            FROM simulation_metrics
                            WHERE simulation_run_id = %s AND metric_name IN (%s, %s)
                            """,
                            (key_completed, key_failed, effective_run_id, key_completed, key_failed),
                        )
                    else:
                        cur.execute(
                            """
                            SELECT
                                COALESCE(SUM(CASE WHEN metric_name = %s THEN metric_value ELSE 0 END), 0) AS pd_completed,
                                COALESCE(SUM(CASE WHEN metric_name = %s THEN metric_value ELSE 0 END), 0) AS pd_failed
                            FROM simulation_metrics
                            WHERE metric_name IN (%s, %s)
                            """,
                            (key_completed, key_failed, key_completed, key_failed),
                        )
                    pd_counts = cur.fetchone() or {}
                    pd_completed = float(pd_counts.get("pd_completed", 0) or 0)
                    pd_failed = float(pd_counts.get("pd_failed", 0) or 0)
                    pd_total = max(1.0, pd_completed + pd_failed)
                    result["pd_tasks_completed"] = pd_completed
                    result["pd_tasks_failed"] = pd_failed
                    result["pd_success_rate"] = (pd_completed / pd_total) * 100.0

                    # PD average time: use pd_duration_seconds metrics
                    if robot_id:
                        pd_dur_key = f"task_completed_pd_duration_seconds_{robot_id}"
                    else:
                        pd_dur_key = "task_completed_pd_duration_seconds"
                    if effective_run_id:
                        cur.execute(
                            """
                            SELECT AVG(metric_value) AS avg_pd_time
                            FROM simulation_metrics
                            WHERE simulation_run_id = %s AND metric_name = %s
                            """,
                            (effective_run_id, pd_dur_key),
                        )
                    else:
                        cur.execute(
                            """
                            SELECT AVG(metric_value) AS avg_pd_time
                            FROM simulation_metrics
                            WHERE metric_name = %s
                            """,
                            (pd_dur_key,),
                        )
                    rpd = cur.fetchone() or {}
                    result["pd_avg_task_time_seconds"] = float(rpd.get("avg_pd_time", 0.0) or 0.0)

                    # Status durations (busy/idle) derived from status_change events
                    if robot_id:
                        duration_key = f"status_duration_seconds_{robot_id}"
                    else:
                        duration_key = "status_duration_seconds"

                    cur.execute(
                        """
                        SELECT metric_name, SUM(metric_value) AS total
                        FROM simulation_metrics
                        WHERE metric_name IN (%s, %s)
                        GROUP BY metric_name
                        """,
                        (
                            f"status_change_{duration_key}",  # not exact; fallback handled below
                            f"status_change_{duration_key}",
                        ),
                    )
                    # The above query is a placeholder; fallback to summing all status_change duration entries
                    # and partitioning by new_status_code buckets using separate queries.

                    # Busy time: sum durations where new_status_code ∈ {moving_to_*, picking, dropping, charging}
                    # We only have numeric values in simulation_metrics, so query by specific metric names we write:
                    # - status_change_status_duration_seconds (global)
                    # - status_change_status_duration_seconds_<robot_id> (per-robot)
                    busy_sum = 0.0
                    idle_sum = 0.0

                    # Global sums
                    if effective_run_id:
                        cur.execute(
                            """
                            SELECT SUM(metric_value) AS total
                            FROM simulation_metrics
                            WHERE simulation_run_id = %s AND metric_name = 'status_change_status_duration_seconds'
                            """,
                            (effective_run_id,),
                        )
                    else:
                        cur.execute(
                            """
                            SELECT SUM(metric_value) AS total
                            FROM simulation_metrics
                            WHERE metric_name = 'status_change_status_duration_seconds'
                            """
                        )
                    r = cur.fetchone() or {}
                    global_total = float(r.get("total") or 0.0)

                    # Per-robot sums if available
                    if robot_id:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT SUM(metric_value) AS total
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND metric_name = %s
                                """,
                                (effective_run_id, f"status_change_status_duration_seconds_{robot_id}"),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT SUM(metric_value) AS total
                                FROM simulation_metrics
                                WHERE metric_name = %s
                                """,
                                (f"status_change_status_duration_seconds_{robot_id}",),
                            )
                        r = cur.fetchone() or {}
                        robot_total = float(r.get("total") or 0.0)
                        total_status_time = robot_total or global_total
                    else:
                        total_status_time = global_total

                    # Sum busy and idle durations emitted per status change
                    if robot_id:
                        busy_key = f"status_change_busy_duration_seconds_{robot_id}"
                        idle_key = f"status_change_idle_duration_seconds_{robot_id}"
                    else:
                        busy_key = "status_change_busy_duration_seconds"
                        idle_key = "status_change_idle_duration_seconds"

                    # Busy sum
                    if effective_run_id:
                        cur.execute(
                            """
                            SELECT COALESCE(SUM(metric_value), 0) AS total
                            FROM simulation_metrics
                            WHERE simulation_run_id = %s AND metric_name = %s
                            """,
                            (effective_run_id, busy_key),
                        )
                    else:
                        cur.execute(
                            """
                            SELECT COALESCE(SUM(metric_value), 0) AS total
                            FROM simulation_metrics
                            WHERE metric_name = %s
                            """,
                            (busy_key,),
                        )
                    rb = cur.fetchone() or {}
                    busy_sum = float(rb.get("total", 0.0) or 0.0)

                    # Idle sum
                    if effective_run_id:
                        cur.execute(
                            """
                            SELECT COALESCE(SUM(metric_value), 0) AS total
                            FROM simulation_metrics
                            WHERE simulation_run_id = %s AND metric_name = %s
                            """,
                            (effective_run_id, idle_key),
                        )
                    else:
                        cur.execute(
                            """
                            SELECT COALESCE(SUM(metric_value), 0) AS total
                            FROM simulation_metrics
                            WHERE metric_name = %s
                            """,
                            (idle_key,),
                        )
                    ri = cur.fetchone() or {}
                    idle_sum = float(ri.get("total", 0.0) or 0.0)

                    result["busy_time_seconds"] = busy_sum

                    # Busy percent averaged per robot: compute per-robot busy/total and average them
                    try:
                        # Compute per-robot busy% = busy_seconds(robot) / (busy+idle)(robot)
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT
                                    REGEXP_REPLACE(metric_name, '^status_change_busy_duration_seconds_', '') AS robot_id,
                                    SUM(metric_value) AS busy_seconds
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND metric_name LIKE %s
                                GROUP BY 1
                                """,
                                (effective_run_id, 'status_change_busy_duration_seconds_%'),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT
                                    REGEXP_REPLACE(metric_name, '^status_change_busy_duration_seconds_', '') AS robot_id,
                                    SUM(metric_value) AS busy_seconds
                                FROM simulation_metrics
                                WHERE metric_name LIKE %s
                                GROUP BY 1
                                """,
                                ('status_change_busy_duration_seconds_%',),
                            )
                        rows = cur.fetchall() or []
                        # Build map of busy seconds per robot
                        busy_map: Dict[str, float] = {}
                        for rr in rows:
                            if isinstance(rr, dict):
                                busy_map[str(rr.get('robot_id'))] = float(rr.get('busy_seconds', 0.0) or 0.0)

                        # Fetch idle per robot
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT
                                    REGEXP_REPLACE(metric_name, '^status_change_idle_duration_seconds_', '') AS robot_id,
                                    SUM(metric_value) AS idle_seconds
                                FROM simulation_metrics
                                WHERE simulation_run_id = %s AND metric_name LIKE %s
                                GROUP BY 1
                                """,
                                (effective_run_id, 'status_change_idle_duration_seconds_%'),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT
                                    REGEXP_REPLACE(metric_name, '^status_change_idle_duration_seconds_', '') AS robot_id,
                                    SUM(metric_value) AS idle_seconds
                                FROM simulation_metrics
                                WHERE metric_name LIKE %s
                                GROUP BY 1
                                """,
                                ('status_change_idle_duration_seconds_%',),
                            )
                        idle_rows = cur.fetchall() or []
                        idle_map: Dict[str, float] = {}
                        for rr in idle_rows:
                            if isinstance(rr, dict):
                                idle_map[str(rr.get('robot_id'))] = float(rr.get('idle_seconds', 0.0) or 0.0)

                        # Compute average of per-robot busy percentages
                        per_robot_percents: List[float] = []
                        for rid, busy_s in busy_map.items():
                            total_s = busy_s + float(idle_map.get(rid, 0.0))
                            if total_s > 0:
                                per_robot_percents.append((busy_s / total_s) * 100.0)
                        if per_robot_percents:
                            result["avg_busy_percent_per_robot"] = sum(per_robot_percents) / len(per_robot_percents)
                            result["avg_idle_percent_per_robot"] = 100.0 - result["avg_busy_percent_per_robot"]
                    except Exception as _:
                        pass

                    # --- Collision KPIs (Database-level dedup within 1-second buckets) ---
                    # Use per-robot detail rows: metric_name LIKE 'collision_detected_collision_type_%'
                    # Bucket timestamps into 1-second windows and count unique (robot, type, bucket)
                    bucket_seconds = 1.0
                    if robot_id:
                        metric_name = f"collision_detected_collision_type_{robot_id}"
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT COUNT(*) AS cnt FROM (
                                    SELECT
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE simulation_run_id = %s AND metric_name = %s
                                    GROUP BY bucket, type_code
                                ) s
                                """,
                                (bucket_seconds, effective_run_id, metric_name),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT COUNT(*) AS cnt FROM (
                                    SELECT
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE metric_name = %s
                                    GROUP BY bucket, type_code
                                ) s
                                """,
                                (bucket_seconds, metric_name),
                            )
                    else:
                        like = 'collision_detected_collision_type_%'
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT COUNT(*) AS cnt FROM (
                                    SELECT
                                        REGEXP_REPLACE(metric_name, '^collision_detected_collision_type_', '') AS rid,
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE simulation_run_id = %s AND metric_name LIKE %s
                                    GROUP BY rid, bucket, type_code
                                ) s
                                """,
                                (bucket_seconds, effective_run_id, like),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT COUNT(*) AS cnt FROM (
                                    SELECT
                                        REGEXP_REPLACE(metric_name, '^collision_detected_collision_type_', '') AS rid,
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE metric_name LIKE %s
                                    GROUP BY rid, bucket, type_code
                                ) s
                                """,
                                (bucket_seconds, like),
                            )
                    r = cur.fetchone() or {}
                    dedup_count = float(r.get("cnt", 0) or 0)
                    result["collision_count"] = dedup_count

                    # Collision rate per minute based on deduped count and run duration
                    try:
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT EXTRACT(EPOCH FROM (MAX(metric_timestamp) - MIN(metric_timestamp))) AS seconds
                                FROM simulation_metrics WHERE simulation_run_id = %s
                                """,
                                (effective_run_id,),
                            )
                            rr = cur.fetchone() or {}
                            seconds = float(rr.get("seconds", 0.0) or 0.0)
                            minutes = max(1.0 / 60.0, seconds / 60.0)
                            result["collision_rate_per_min"] = dedup_count / minutes if minutes > 0 else 0.0
                    except Exception:
                        pass

                    # Collision type breakdown on deduped events
                    if robot_id:
                        metric_name = f"collision_detected_collision_type_{robot_id}"
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT type_code, COUNT(*) AS cnt FROM (
                                    SELECT
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE simulation_run_id = %s AND metric_name = %s
                                    GROUP BY bucket, type_code
                                ) s GROUP BY type_code
                                """,
                                (bucket_seconds, effective_run_id, metric_name),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT type_code, COUNT(*) AS cnt FROM (
                                    SELECT
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE metric_name = %s
                                    GROUP BY bucket, type_code
                                ) s GROUP BY type_code
                                """,
                                (bucket_seconds, metric_name),
                            )
                    else:
                        like = 'collision_detected_collision_type_%'
                        if effective_run_id:
                            cur.execute(
                                """
                                SELECT type_code, COUNT(*) AS cnt FROM (
                                    SELECT
                                        REGEXP_REPLACE(metric_name, '^collision_detected_collision_type_', '') AS rid,
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE simulation_run_id = %s AND metric_name LIKE %s
                                    GROUP BY rid, bucket, type_code
                                ) s GROUP BY type_code
                                """,
                                (bucket_seconds, effective_run_id, like),
                            )
                        else:
                            cur.execute(
                                """
                                SELECT type_code, COUNT(*) AS cnt FROM (
                                    SELECT
                                        REGEXP_REPLACE(metric_name, '^collision_detected_collision_type_', '') AS rid,
                                        FLOOR(EXTRACT(EPOCH FROM metric_timestamp) / %s)::BIGINT AS bucket,
                                        metric_value::INT AS type_code
                                    FROM simulation_metrics
                                    WHERE metric_name LIKE %s
                                    GROUP BY rid, bucket, type_code
                                ) s GROUP BY type_code
                                """,
                                (bucket_seconds, like),
                            )
                    rows = cur.fetchall() or []
                    type_counts: Dict[int, float] = {}
                    for rowtc in rows:
                        if isinstance(rowtc, dict):
                            type_counts[int(rowtc.get('type_code', 0) or 0)] = float(rowtc.get('cnt', 0) or 0)
                    result["collision_types_robot"] = type_counts.get(1, 0.0)
                    result["collision_types_shelf"] = type_counts.get(2, 0.0)
                    result["collision_types_wall"] = type_counts.get(3, 0.0)
                    # To expose shelf and wall separately, we would need per-type metric names.
                    # For now, keep robot sum and allow total count/rate for visibility.

        except Exception as e:
            logger.error(f"[DEBUG] CRITICAL: Failed to build KPI overview: {e}")
            logger.error(f"[DEBUG] CRITICAL: Exception type: {type(e).__name__}")
            logger.error(f"[DEBUG] CRITICAL: Exception args: {e.args}")
            import traceback
            logger.error(f"[DEBUG] CRITICAL: Full traceback:\n{traceback.format_exc()}")
            logger.error("[DEBUG] CRITICAL: This error proves the connection corruption theory!")
        return result

    def export_kpi_overview_csv(self, overview: Dict[str, Any], file_path: str) -> bool:
        """Export KPI overview dict to a CSV file with simple key/value rows."""
        try:
            with open(file_path, mode="w", newline="") as f:
                writer = csv.writer(f)
                # Include run id as header for clarity
                try:
                    run_id = getattr(self, "_current_simulation_run_id", None)
                    if run_id:
                        writer.writerow(["simulation_run_id", run_id])
                except Exception:
                    pass
                writer.writerow(["Metric", "Value"])
                for k, v in overview.items():
                    writer.writerow([k, v])
            return True
        except Exception as e:
            logger.error(f"Failed to export KPI overview to CSV at {file_path}: {e}")
            return False