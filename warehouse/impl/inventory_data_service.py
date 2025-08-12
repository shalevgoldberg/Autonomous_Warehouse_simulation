"""
Inventory Data Service - Manages warehouse inventory data and shelf creation.

This service is responsible for:
1. Creating shelves in the database from warehouse map data
2. Populating inventory data with items
3. Managing inventory allocation and updates
4. Providing thread-safe access to inventory information

Design Principles:
- **Single Responsibility**: Focuses solely on inventory data management
- **Open/Closed**: Extensible for future inventory strategies
- **Dependency Inversion**: Depends on abstractions (interfaces)
- **Thread Safety**: All operations are thread-safe with database transactions
- **Error Handling**: Comprehensive exception handling with proper logging

Future Enhancements:
- Multiple items per shelf support
- Multiple shelves per item support
- Dynamic inventory allocation strategies
- Inventory forecasting and replenishment
"""
import os
import logging
import threading
import time
import uuid
from contextlib import contextmanager
from typing import Optional, List, Dict, Any, Tuple, Set
from dataclasses import dataclass
from datetime import datetime

import psycopg2
import psycopg2.pool
from psycopg2.extras import RealDictCursor
import psycopg2.errorcodes as errorcodes

from warehouse.map import WarehouseMap
from interfaces.simulation_data_service_interface import SimulationDataServiceError


@dataclass
class InventoryItem:
    """Represents an inventory item with its properties."""
    item_id: str
    name: str
    description: Optional[str] = None
    category: Optional[str] = None
    weight: Optional[float] = None
    dimensions: Optional[Tuple[float, float, float]] = None


@dataclass
class ShelfInventory:
    """Represents inventory on a specific shelf."""
    shelf_id: str
    item_id: str
    quantity: int
    max_capacity: int = 100
    reserved_quantity: int = 0
    
    @property
    def available_quantity(self) -> int:
        """Get available quantity (total - reserved)."""
        return max(0, self.quantity - self.reserved_quantity)
    
    @property
    def is_available(self) -> bool:
        """Check if shelf has available inventory."""
        return self.available_quantity > 0


class InventoryDataServiceError(Exception):
    """Exception raised by InventoryDataService operations."""
    pass


class InventoryDataService:
    """
    Thread-safe inventory data service for warehouse management.
    
    Responsibilities:
    - Create shelves from warehouse map data
    - Populate inventory with items
    - Manage inventory allocation and updates
    - Provide inventory queries and statistics
    
    Threading Model:
    - ALL methods: Thread-safe (database transactions provide consistency)
    - Connection pooling: Handles concurrent access automatically
    - Inventory operations: Atomic with proper rollback
    """
    
    def __init__(self, 
                 warehouse_map: WarehouseMap,
                 db_host: str = "localhost",
                 db_port: int = 5432,
                 db_name: str = "warehouse_sim",
                 db_user: str = "postgres",
                 db_password: Optional[str] = None,
                 pool_size: int = 10):
        """
        Initialize InventoryDataService with database connection.
        
        Args:
            warehouse_map: Warehouse layout for shelf creation
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
                raise InventoryDataServiceError(
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
            'application_name': 'warehouse_inventory'
        }
        
        # Connection pool for thread safety
        self._connection_pool = None
        self._pool_lock = threading.Lock()
        self.pool_size = pool_size
        
        # Cache for frequently accessed data
        self._inventory_cache = {}
        self._cache_lock = threading.RLock()
        self._cache_timestamp = 0
        self._cache_ttl = 300  # 5 minutes cache TTL
        
        # Initialize database connection and schema
        self._initialize_database()
        
        # Logging
        self.logger = logging.getLogger(f"{self.__class__.__name__}")
        self.logger.info(f"InventoryDataService initialized with database {db_name}@{db_host}:{db_port}")
    
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
                        AND table_name IN ('items', 'shelves', 'shelf_inventory')
                    """)
                    tables = [row['table_name'] for row in cur.fetchall()]
                    
                    required_tables = {'items', 'shelves', 'shelf_inventory'}
                    missing_tables = required_tables - set(tables)
                    
                    if missing_tables:
                        raise InventoryDataServiceError(
                            f"Missing required database tables: {missing_tables}. "
                            f"Please run warehouse_schema.sql to create the schema."
                        )
                    
                    self.logger.info("Database schema verified successfully")
                    
        except Exception as e:
            self.logger.error(f"Failed to initialize database: {e}")
            raise InventoryDataServiceError(f"Database initialization failed: {e}")
    
    @contextmanager
    def _get_connection(self):
        """Get a database connection from the pool."""
        if not self._connection_pool:
            raise InventoryDataServiceError("Database connection pool not initialized")
        
        conn = None
        try:
            conn = self._connection_pool.getconn()
            yield conn
        except Exception as e:
            if conn:
                conn.rollback()
            raise
        finally:
            if conn:
                self._connection_pool.putconn(conn)
    
    def create_shelves_from_map(self, clear_existing: bool = False) -> int:
        """
        Create shelves in the database from warehouse map data.
        
        Args:
            clear_existing: Whether to clear existing shelves before creating new ones
            
        Returns:
            int: Number of shelves created
            
        Raises:
            InventoryDataServiceError: If shelf creation fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Clear existing shelves if requested
                    if clear_existing:
                        cur.execute("DELETE FROM shelf_inventory")
                        cur.execute("DELETE FROM shelves")
                        self.logger.info("Cleared existing shelves and inventory")
                    
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
                    self.logger.info(f"Created {shelves_created} shelves from warehouse map")
                    
                    # Clear cache since inventory data changed
                    self._clear_cache()
                    
                    return shelves_created
                    
        except Exception as e:
            self.logger.error(f"Failed to create shelves from map: {e}")
            raise InventoryDataServiceError(f"Shelf creation failed: {e}")
    
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
            InventoryDataServiceError: If inventory population fails
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    inventory_created = 0
                    
                    for item_data in inventory_data:
                        # Insert item if it doesn't exist
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
                    self.logger.info(f"Populated {inventory_created} inventory entries")
                    
                    # Clear cache since inventory data changed
                    self._clear_cache()
                    
                    return inventory_created
                    
        except Exception as e:
            self.logger.error(f"Failed to populate inventory: {e}")
            raise InventoryDataServiceError(f"Inventory population failed: {e}")
    
    def get_shelf_inventory(self, shelf_id: str) -> Optional[ShelfInventory]:
        """
        Get inventory information for a specific shelf.
        
        Args:
            shelf_id: Shelf identifier
            
        Returns:
            Optional[ShelfInventory]: Shelf inventory information or None if not found
        """
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        SELECT si.shelf_id, si.item_id, si.quantity,
                               i.name as item_name
                        FROM shelf_inventory si
                        JOIN items i ON si.item_id = i.item_id
                        WHERE si.shelf_id = %s AND si.quantity > 0
                    """, (shelf_id,))
                    
                    result = cur.fetchone()
                    if not result:
                        return None
                    
                    return ShelfInventory(
                        shelf_id=result['shelf_id'],
                        item_id=result['item_id'],
                        quantity=result['quantity']
                    )
                    
        except Exception as e:
            self.logger.error(f"Failed to get shelf inventory for {shelf_id}: {e}")
            raise InventoryDataServiceError(f"Failed to get shelf inventory: {e}")
    
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
            self.logger.error(f"Failed to find location for item {item_id}: {e}")
            raise InventoryDataServiceError(f"Failed to find item location: {e}")
    
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
            raise InventoryDataServiceError(f"Invalid operation: {operation}")
        
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    # Verify shelf exists
                    cur.execute("SELECT 1 FROM shelves WHERE shelf_id = %s", (shelf_id,))
                    if not cur.fetchone():
                        raise InventoryDataServiceError(f"Shelf {shelf_id} does not exist")
                    
                    # Verify item exists
                    cur.execute("SELECT 1 FROM items WHERE item_id = %s", (item_id,))
                    if not cur.fetchone():
                        raise InventoryDataServiceError(f"Item {item_id} does not exist")
                    
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
                            self.logger.warning(f"No inventory found for item {item_id} on shelf {shelf_id}")
                            return False
                    
                    # TODO: Implement 'move' operation when needed
                    
                    conn.commit()
                    self.logger.debug(f"Inventory updated: {operation} {quantity} of {item_id} on shelf {shelf_id}")
                    
                    # Clear cache since inventory data changed
                    self._clear_cache()
                    
                    return True
                    
        except InventoryDataServiceError:
            raise
        except Exception as e:
            self.logger.error(f"Failed to update inventory: {e}")
            raise InventoryDataServiceError(f"Failed to update inventory: {e}")
    
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
            self.logger.error(f"Failed to get inventory statistics: {e}")
            raise InventoryDataServiceError(f"Failed to get inventory statistics: {e}")
    
    def _clear_cache(self) -> None:
        """Clear the inventory cache."""
        with self._cache_lock:
            self._inventory_cache.clear()
            self._cache_timestamp = 0
    
    def close(self) -> None:
        """Close the database connection pool."""
        if self._connection_pool:
            with self._pool_lock:
                self._connection_pool.closeall()
            self.logger.info("Database connection pool closed") 