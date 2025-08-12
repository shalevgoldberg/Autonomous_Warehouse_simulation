"""
Integration tests for Inventory Management through SimulationDataService - End-to-end testing with PostgreSQL.

These tests require:
1. PostgreSQL database running
2. WAREHOUSE_DB_PASSWORD environment variable set
3. Database schema created (warehouse_schema.sql)

Run with: $env:WAREHOUSE_DB_PASSWORD="renaspolter"; python -m pytest tests/integration/test_inventory_data_integration.py -v

Test Coverage:
- Complete inventory data flow from map to database
- Real database operations and transactions
- Thread safety with concurrent operations
- Error handling and recovery
- Performance and scalability

Architecture Note:
- All inventory operations go through SimulationDataService (unified database interface)
- This follows the Facade Pattern and Single Responsibility Principle
- Components interact with inventory through SimulationDataService, not directly
"""
import os
import unittest
import threading
import time
from typing import List, Dict, Any

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import (
    SimulationDataServiceImpl, 
    SimulationDataServiceError
)


class TestInventoryDataIntegration(unittest.TestCase):
    """Integration tests for Inventory Management through SimulationDataService with real database."""
    
    def setUp(self):
        """Set up test environment with real database connection."""
        # These tests require actual database connection
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("Database not available for integration tests")
        
        # Create a simple warehouse map for testing
        self.warehouse_map = WarehouseMap(width=10, height=10)
        
        # Initialize real SimulationDataService
        self.simulation_data_service = SimulationDataServiceImpl(
            warehouse_map=self.warehouse_map,
            db_host="localhost",
            db_port=5432,
            db_name="warehouse_sim",
            db_user="postgres",
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD')
        )
        
        # Clean up any existing test data
        self._cleanup_test_data()
    
    def tearDown(self):
        """Clean up test data after each test."""
        self._cleanup_test_data()
        if hasattr(self, 'simulation_data_service'):
            self.simulation_data_service.close()
    
    def _cleanup_test_data(self):
        """Clean up test data from database."""
        try:
            with self.simulation_data_service._get_connection() as conn:
                with conn.cursor() as cur:
                    # Remove test inventory
                    cur.execute("DELETE FROM shelf_inventory WHERE item_id LIKE 'test_%'")
                    # Remove test items
                    cur.execute("DELETE FROM items WHERE item_id LIKE 'test_%'")
                    # Remove test shelves
                    cur.execute("DELETE FROM shelves WHERE shelf_id LIKE 'test_%'")
                    conn.commit()
        except Exception as e:
            print(f"Warning: Failed to cleanup test data: {e}")
    
    def test_create_shelves_from_map_integration(self):
        """Test creating shelves from warehouse map with real database."""
        # Create shelves from map
        shelves_created = self.simulation_data_service.create_shelves_from_map(clear_existing=True)
        
        # Verify shelves were created
        self.assertGreater(shelves_created, 0)
        self.assertEqual(shelves_created, len(self.warehouse_map.shelves))
        
        # Verify shelves exist in database
        with self.simulation_data_service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT COUNT(*) as count FROM shelves")
                result = cur.fetchone()
                self.assertEqual(result['count'], shelves_created)
    
    def test_populate_inventory_integration(self):
        """Test populating inventory with real database operations."""
        # Create test inventory data
        test_inventory = [
            {
                'item_id': 'test_book_1',
                'name': 'Test Book 1',
                'shelf_id': 'shelf_0',
                'quantity': 10
            },
            {
                'item_id': 'test_electronics_1',
                'name': 'Test Electronics 1',
                'shelf_id': 'shelf_1',
                'quantity': 5
            }
        ]
        
        # Populate inventory
        inventory_created = self.simulation_data_service.populate_inventory(test_inventory)
        
        # Verify inventory was created
        self.assertEqual(inventory_created, len(test_inventory))
        
        # Verify items exist in database
        with self.simulation_data_service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT COUNT(*) as count FROM items WHERE item_id LIKE 'test_%'")
                result = cur.fetchone()
                self.assertEqual(result['count'], len(test_inventory))
                
                # Verify inventory quantities
                cur.execute("SELECT item_id, quantity FROM shelf_inventory WHERE item_id LIKE 'test_%'")
                inventory_results = cur.fetchall()
                self.assertEqual(len(inventory_results), len(test_inventory))
                
                for result in inventory_results:
                    if result['item_id'] == 'test_book_1':
                        self.assertEqual(result['quantity'], 10)
                    elif result['item_id'] == 'test_electronics_1':
                        self.assertEqual(result['quantity'], 5)
    
    def test_get_item_location_integration(self):
        """Test item location lookup with real database."""
        # Create test inventory
        test_inventory = [
            {
                'item_id': 'test_item_location',
                'name': 'Test Item for Location',
                'shelf_id': 'shelf_2',
                'quantity': 15
            }
        ]
        
        self.simulation_data_service.populate_inventory(test_inventory)
        
        # Test item location lookup
        shelf_id = self.simulation_data_service.get_item_location('test_item_location')
        
        # Verify result
        self.assertEqual(shelf_id, 'shelf_2')
        
        # Test non-existent item
        shelf_id = self.simulation_data_service.get_item_location('nonexistent_item')
        self.assertIsNone(shelf_id)
    
    def test_update_inventory_integration(self):
        """Test inventory updates with real database."""
        # Create test inventory
        test_inventory = [
            {
                'item_id': 'test_update_item',
                'name': 'Test Item for Updates',
                'shelf_id': 'shelf_3',
                'quantity': 20
            }
        ]
        
        self.simulation_data_service.populate_inventory(test_inventory)
        
        # Test adding inventory
        result = self.simulation_data_service.update_inventory('shelf_3', 'test_update_item', 'add', 5)
        self.assertTrue(result)
        
        # Verify quantity increased
        with self.simulation_data_service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT quantity FROM shelf_inventory WHERE item_id = 'test_update_item'")
                result = cur.fetchone()
                self.assertEqual(result['quantity'], 25)
        
        # Test removing inventory
        result = self.simulation_data_service.update_inventory('shelf_3', 'test_update_item', 'remove', 10)
        self.assertTrue(result)
        
        # Verify quantity decreased
        with self.simulation_data_service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT quantity FROM shelf_inventory WHERE item_id = 'test_update_item'")
                result = cur.fetchone()
                self.assertEqual(result['quantity'], 15)
    
    def test_get_inventory_statistics_integration(self):
        """Test inventory statistics with real database."""
        # Create test inventory with multiple items
        test_inventory = [
            {
                'item_id': 'test_stats_1',
                'name': 'Test Stats Item 1',
                'shelf_id': 'shelf_4',
                'quantity': 10
            },
            {
                'item_id': 'test_stats_2',
                'name': 'Test Stats Item 2',
                'shelf_id': 'shelf_5',
                'quantity': 5
            },
            {
                'item_id': 'test_stats_3',
                'name': 'Test Stats Item 3',
                'shelf_id': 'shelf_6',
                'quantity': 3  # Low stock
            }
        ]
        
        self.simulation_data_service.populate_inventory(test_inventory)
        
        # Get statistics
        stats = self.simulation_data_service.get_inventory_statistics()
        
        # Verify statistics
        self.assertIn('total_shelves', stats)
        self.assertIn('total_items', stats)
        self.assertIn('total_quantity', stats)
        self.assertIn('low_stock_items', stats)
        
        # Verify low stock items
        low_stock_items = [item for item in stats['low_stock_items'] if item['item_id'] == 'test_stats_3']
        self.assertEqual(len(low_stock_items), 1)
        self.assertEqual(low_stock_items[0]['quantity'], 3)
    
    def test_concurrent_operations_integration(self):
        """Test concurrent inventory operations with real database."""
        # Create test inventory
        test_inventory = [
            {
                'item_id': 'test_concurrent',
                'name': 'Test Concurrent Item',
                'shelf_id': 'shelf_7',
                'quantity': 50
            }
        ]
        
        self.simulation_data_service.populate_inventory(test_inventory)
        
        # Test concurrent operations
        results = []
        errors = []
        
        def worker(thread_id):
            try:
                # Perform multiple operations
                for i in range(5):
                    # Get item location
                    shelf_id = self.simulation_data_service.get_item_location('test_concurrent')
                    if shelf_id != 'shelf_7':
                        raise Exception(f"Wrong shelf ID: {shelf_id}")
                    
                    # Update inventory
                    result = self.simulation_data_service.update_inventory('shelf_7', 'test_concurrent', 'remove', 1)
                    if not result:
                        raise Exception("Inventory update failed")
                    
                    time.sleep(0.01)  # Small delay to increase concurrency
                
                results.append(thread_id)
            except Exception as e:
                errors.append((thread_id, e))
        
        # Create multiple threads
        threads = []
        for i in range(3):
            thread = threading.Thread(target=worker, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Verify results
        self.assertEqual(len(results), 3)  # All threads should succeed
        self.assertEqual(len(errors), 0)   # No errors should occur
        
        # Verify final inventory quantity (should be reduced by 15: 3 threads * 5 operations)
        with self.simulation_data_service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT quantity FROM shelf_inventory WHERE item_id = 'test_concurrent'")
                result = cur.fetchone()
                self.assertEqual(result['quantity'], 35)  # 50 - 15
    
    def test_error_handling_integration(self):
        """Test error handling with real database operations."""
        # Test updating non-existent shelf
        with self.assertRaises(SimulationDataServiceError) as context:
            self.simulation_data_service.update_inventory('nonexistent_shelf', 'test_item', 'add', 1)
        
        self.assertIn("does not exist", str(context.exception))
        
        # Test updating non-existent item
        with self.assertRaises(SimulationDataServiceError) as context:
            self.simulation_data_service.update_inventory('shelf_0', 'nonexistent_item', 'add', 1)
        
        self.assertIn("does not exist", str(context.exception))
        
        # Test invalid operation
        with self.assertRaises(SimulationDataServiceError) as context:
            self.simulation_data_service.update_inventory('shelf_0', 'test_item', 'invalid_op', 1)
        
        self.assertIn("Invalid operation", str(context.exception))
    
    def test_complete_inventory_flow_integration(self):
        """Test complete inventory flow from map to database to queries."""
        # Step 1: Create shelves from map
        shelves_created = self.simulation_data_service.create_shelves_from_map(clear_existing=True)
        self.assertGreater(shelves_created, 0)
        
        # Step 2: Create comprehensive test inventory
        test_inventory = []
        for i, shelf_id in enumerate(list(self.warehouse_map.shelves.keys())[:5]):
            test_inventory.append({
                'item_id': f'test_flow_item_{i}',
                'name': f'Test Flow Item {i}',
                'shelf_id': shelf_id,
                'quantity': 10 + i * 5
            })
        
        # Step 3: Populate inventory
        inventory_created = self.simulation_data_service.populate_inventory(test_inventory)
        self.assertEqual(inventory_created, len(test_inventory))
        
        # Step 4: Test item location lookups
        for i, item_data in enumerate(test_inventory):
            shelf_id = self.simulation_data_service.get_item_location(item_data['item_id'])
            self.assertEqual(shelf_id, item_data['shelf_id'])
        
        # Step 5: Test inventory updates
        for i, item_data in enumerate(test_inventory):
            # Add some inventory
            result = self.simulation_data_service.update_inventory(
                item_data['shelf_id'], 
                item_data['item_id'], 
                'add', 
                5
            )
            self.assertTrue(result)
            
            # Remove some inventory
            result = self.simulation_data_service.update_inventory(
                item_data['shelf_id'], 
                item_data['item_id'], 
                'remove', 
                2
            )
            self.assertTrue(result)
        
        # Step 6: Get final statistics
        stats = self.simulation_data_service.get_inventory_statistics()
        self.assertGreater(stats['total_shelves'], 0)
        self.assertGreater(stats['total_items'], 0)
        self.assertGreater(stats['total_quantity'], 0)
        
        print(f"Complete flow test results:")
        print(f"  Shelves created: {shelves_created}")
        print(f"  Inventory items: {inventory_created}")
        print(f"  Total quantity: {stats['total_quantity']}")
        print(f"  Low stock items: {len(stats['low_stock_items'])}")


if __name__ == '__main__':
    unittest.main() 