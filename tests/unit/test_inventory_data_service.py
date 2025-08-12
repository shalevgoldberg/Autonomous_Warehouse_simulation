"""
Unit tests for Inventory Management through SimulationDataService.

Tests cover:
- Shelf creation from warehouse map
- Inventory population and management
- Thread safety and error handling
- Database operations and caching
- SOLID principles compliance

Test Strategy:
- Mock database connections for unit tests
- Test each method in isolation
- Verify thread safety with concurrent operations
- Test error conditions and edge cases

Architecture Note:
- All inventory operations go through SimulationDataService (unified database interface)
- This follows the Facade Pattern and Single Responsibility Principle
- Components interact with inventory through SimulationDataService, not directly
"""
import unittest
import threading
import time
from unittest.mock import Mock, patch, MagicMock
from typing import List, Dict, Any

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import (
    SimulationDataServiceImpl, 
    SimulationDataServiceError
)


class TestInventoryDataService(unittest.TestCase):
    """Test cases for Inventory Management through SimulationDataService."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create a simple warehouse map for testing
        self.warehouse_map = WarehouseMap(width=10, height=10)
        
        # Mock database connection parameters
        self.db_params = {
            'host': 'localhost',
            'port': 5432,
            'database': 'test_warehouse',
            'user': 'test_user',
            'password': 'test_password',
            'pool_size': 5
        }
        
        # Sample inventory data
        self.sample_inventory = [
            {
                'item_id': 'book-fantasy',
                'name': 'Fantasy Novel',
                'shelf_id': 'shelf_0',
                'quantity': 10
            },
            {
                'item_id': 'electronics-phone',
                'name': 'Smartphone',
                'shelf_id': 'shelf_1',
                'quantity': 5
            }
        ]
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_initialization_success(self, mock_pool):
        """Test successful service initialization."""
        # Mock successful database connection
        mock_pool.return_value = Mock()
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
        
        self.assertIsNotNone(service)
        self.assertEqual(service.warehouse_map, self.warehouse_map)
        mock_pool.assert_called_once()
    
    def test_initialization_missing_password(self):
        """Test initialization fails without database password."""
        with patch.dict('os.environ', {}, clear=True):
            with self.assertRaises(InventoryDataServiceError) as context:
                InventoryDataService(
                    warehouse_map=self.warehouse_map,
                    **{k: v for k, v in self.db_params.items() if k != 'password'}
                )
        
        self.assertIn("Database password not provided", str(context.exception))
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_create_shelves_from_map(self, mock_pool):
        """Test shelf creation from warehouse map."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test shelf creation
            shelves_created = service.create_shelves_from_map()
            
            # Verify results
            self.assertGreater(shelves_created, 0)
            mock_cursor.execute.assert_called()
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_populate_inventory(self, mock_pool):
        """Test inventory population."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test inventory population
            inventory_created = service.populate_inventory(self.sample_inventory)
            
            # Verify results
            self.assertEqual(inventory_created, len(self.sample_inventory))
            mock_cursor.execute.assert_called()
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_get_item_location(self, mock_pool):
        """Test item location lookup."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        # Mock item location result
        mock_cursor.fetchone.return_value = {'shelf_id': 'shelf_0'}
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test item location lookup
            shelf_id = service.get_item_location('book-fantasy')
            
            # Verify results
            self.assertEqual(shelf_id, 'shelf_0')
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_get_item_location_not_found(self, mock_pool):
        """Test item location lookup when item not found."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        # Mock no item found
        mock_cursor.fetchone.return_value = None
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test item location lookup
            shelf_id = service.get_item_location('nonexistent-item')
            
            # Verify results
            self.assertIsNone(shelf_id)
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_update_inventory_add(self, mock_pool):
        """Test adding inventory."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        # Mock shelf and item existence
        mock_cursor.fetchone.side_effect = [{'1': 1}, {'1': 1}]
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test adding inventory
            result = service.update_inventory('shelf_0', 'book-fantasy', 'add', 5)
            
            # Verify results
            self.assertTrue(result)
            mock_cursor.execute.assert_called()
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_update_inventory_remove(self, mock_pool):
        """Test removing inventory."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        # Mock shelf and item existence, and successful removal
        mock_cursor.fetchone.side_effect = [{'1': 1}, {'1': 1}]
        mock_cursor.rowcount = 1
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test removing inventory
            result = service.update_inventory('shelf_0', 'book-fantasy', 'remove', 2)
            
            # Verify results
            self.assertTrue(result)
            mock_cursor.execute.assert_called()
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_update_inventory_invalid_operation(self, mock_pool):
        """Test inventory update with invalid operation."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test invalid operation
            with self.assertRaises(InventoryDataServiceError) as context:
                service.update_inventory('shelf_0', 'book-fantasy', 'invalid_op', 1)
            
            self.assertIn("Invalid operation", str(context.exception))
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_get_inventory_statistics(self, mock_pool):
        """Test inventory statistics retrieval."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_cursor
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        # Mock statistics results
        mock_cursor.fetchone.side_effect = [
            {'total_shelves': 10, 'total_items': 5, 'total_quantity': 100},
            []  # No low stock items
        ]
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test statistics retrieval
            stats = service.get_inventory_statistics()
            
            # Verify results
            self.assertEqual(stats['total_shelves'], 10)
            self.assertEqual(stats['total_items'], 5)
            self.assertEqual(stats['total_quantity'], 100)
            self.assertEqual(len(stats['low_stock_items']), 0)
    
    def test_shelf_inventory_properties(self):
        """Test ShelfInventory dataclass properties."""
        # Create shelf inventory with reserved quantity
        shelf_inv = ShelfInventory(
            shelf_id='shelf_0',
            item_id='book-fantasy',
            quantity=10,
            reserved_quantity=3
        )
        
        # Test properties
        self.assertEqual(shelf_inv.available_quantity, 7)
        self.assertTrue(shelf_inv.is_available)
        
        # Test with no available quantity
        shelf_inv.reserved_quantity = 10
        self.assertEqual(shelf_inv.available_quantity, 0)
        self.assertFalse(shelf_inv.is_available)
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_thread_safety(self, mock_pool):
        """Test thread safety of inventory operations."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        # Mock item location results
        mock_cursor.fetchone.return_value = {'shelf_id': 'shelf_0'}
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test concurrent operations
            results = []
            errors = []
            
            def worker(thread_id):
                try:
                    shelf_id = service.get_item_location('book-fantasy')
                    results.append((thread_id, shelf_id))
                except Exception as e:
                    errors.append((thread_id, e))
            
            # Create multiple threads
            threads = []
            for i in range(5):
                thread = threading.Thread(target=worker, args=(i,))
                threads.append(thread)
                thread.start()
            
            # Wait for all threads to complete
            for thread in threads:
                thread.join()
            
            # Verify results
            self.assertEqual(len(results), 5)
            self.assertEqual(len(errors), 0)
            
            for thread_id, shelf_id in results:
                self.assertEqual(shelf_id, 'shelf_0')
    
    @patch('warehouse.impl.inventory_data_service.psycopg2.pool.ThreadedConnectionPool')
    def test_cache_clearing(self, mock_pool):
        """Test cache clearing functionality."""
        # Mock database connection and cursor
        mock_conn = Mock()
        mock_cursor = Mock()
        mock_conn.cursor.return_value.__enter__.return_value = mock_cursor
        mock_pool.return_value.getconn.return_value = mock_conn
        
        # Mock successful table verification
        mock_cursor.fetchall.return_value = [
            {'table_name': 'items'},
            {'table_name': 'shelves'},
            {'table_name': 'shelf_inventory'}
        ]
        
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'test_password'}):
            service = InventoryDataService(
                warehouse_map=self.warehouse_map,
                **self.db_params
            )
            
            # Test cache clearing
            service._clear_cache()
            
            # Verify cache is cleared
            self.assertEqual(len(service._inventory_cache), 0)
            self.assertEqual(service._cache_timestamp, 0)


if __name__ == '__main__':
    unittest.main() 