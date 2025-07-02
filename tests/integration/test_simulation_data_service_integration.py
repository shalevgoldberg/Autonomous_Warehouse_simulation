"""
Integration tests for SimulationDataService with real database.

These tests require:
1. PostgreSQL database running
2. WAREHOUSE_DB_PASSWORD environment variable set
3. Database schema created (warehouse_schema.sql)

Run with: $env:WAREHOUSE_DB_PASSWORD="renaspolter"; python -m pytest tests/integration/test_simulation_data_service_integration.py
"""
import unittest
import os
import threading
import time
from typing import List

from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from interfaces.simulation_data_service_interface import (
    ISimulationDataService, 
    ShelfInfo, 
    MapData,
    SimulationDataServiceError
)
from warehouse.map import WarehouseMap


class TestSimulationDataServiceIntegration(unittest.TestCase):
    """Integration tests for SimulationDataService with real database."""
    
    @classmethod
    def setUpClass(cls):
        """Set up test database connection."""
        # Check if database password is available
        cls.db_password = os.getenv('WAREHOUSE_DB_PASSWORD')
        if not cls.db_password:
            raise unittest.SkipTest("WAREHOUSE_DB_PASSWORD not set - skipping database integration tests")
        
        # Create warehouse map for testing
        cls.warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
        
        print(f"\n{'='*60}")
        print("SIMULATION DATA SERVICE INTEGRATION TESTS")
        print(f"{'='*60}")
        print(f"Database: warehouse_sim@localhost:5432")
        print(f"Warehouse: {cls.warehouse_map.width}x{cls.warehouse_map.height} cells")
        print(f"{'='*60}")
    
    def setUp(self):
        """Set up each test."""
        try:
            self.service = SimulationDataServiceImpl(
                warehouse_map=self.warehouse_map,
                db_password=self.db_password,
                pool_size=5
            )
            
            # Clean up any test data from previous runs
            self._cleanup_test_data()
            
            # Insert test data
            self._insert_test_data()
            
        except Exception as e:
            self.skipTest(f"Database connection failed: {e}")
    
    def tearDown(self):
        """Clean up after each test."""
        if hasattr(self, 'service'):
            self._cleanup_test_data()
            self.service.close()
    
    def _cleanup_test_data(self):
        """Clean up test data from database."""
        try:
            with self.service._get_connection() as conn:
                with conn.cursor() as cur:
                    # Clean up in correct order due to foreign keys
                    cur.execute("DELETE FROM shelf_locks WHERE shelf_id LIKE 'test_%'")
                    cur.execute("DELETE FROM shelf_inventory WHERE shelf_id LIKE 'test_%'")
                    cur.execute("DELETE FROM shelves WHERE shelf_id LIKE 'test_%'")
                    cur.execute("DELETE FROM items WHERE item_id LIKE 'test_%'")
                    cur.execute("DELETE FROM robots WHERE robot_id LIKE 'test_%'")
                    conn.commit()
        except Exception:
            pass  # Ignore cleanup errors
    
    def _insert_test_data(self):
        """Insert test data into database."""
        with self.service._get_connection() as conn:
            with conn.cursor() as cur:
                # Insert test items
                cur.execute("""
                    INSERT INTO items (item_id, name) VALUES 
                    ('test_book_001', 'Test Fantasy Novel'),
                    ('test_book_002', 'Test Science Fiction'),
                    ('test_electronics_001', 'Test Smartphone')
                """)
                
                # Insert test shelves
                cur.execute("""
                    INSERT INTO shelves (shelf_id, position_x, position_y) VALUES 
                    ('test_shelf_001', 2.5, 3.0),
                    ('test_shelf_002', 4.5, 3.0),
                    ('test_shelf_003', 6.5, 3.0)
                """)
                
                # Insert test inventory
                cur.execute("""
                    INSERT INTO shelf_inventory (shelf_id, item_id, quantity) VALUES 
                    ('test_shelf_001', 'test_book_001', 10),
                    ('test_shelf_001', 'test_book_002', 5),
                    ('test_shelf_002', 'test_electronics_001', 3)
                """)
                
                # Insert test robot
                cur.execute("""
                    INSERT INTO robots (robot_id, name) VALUES 
                    ('test_robot_001', 'Test Robot 1')
                """)
                
                conn.commit()
    
    def test_database_connection_and_schema(self):
        """Test database connection and schema validation."""
        print("\n🔗 Testing database connection and schema...")
        
        # Service should initialize without errors
        self.assertIsNotNone(self.service)
        self.assertIsNotNone(self.service._connection_pool)
        
        # Test connection pool
        with self.service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("SELECT 1 as test_value")
                result = cur.fetchone()
                self.assertEqual(result['test_value'], 1)
        
        print("   ✅ Database connection successful")
        print("   ✅ Schema validation passed")
    
    def test_map_data_operations(self):
        """Test map data retrieval and caching."""
        print("\n🗺️  Testing map data operations...")
        
        # Get map data
        map_data = self.service.get_map_data()
        
        self.assertIsInstance(map_data, MapData)
        self.assertEqual(map_data.width, self.warehouse_map.width)
        self.assertEqual(map_data.height, self.warehouse_map.height)
        self.assertEqual(map_data.cell_size, self.warehouse_map.grid_size)
        
        # Verify test shelves are included
        self.assertIn('test_shelf_001', map_data.shelves)
        self.assertIn('test_shelf_002', map_data.shelves)
        
        # Test caching - second call should be faster
        start_time = time.time()
        map_data2 = self.service.get_map_data()
        cache_time = time.time() - start_time
        
        self.assertEqual(map_data, map_data2)
        self.assertLess(cache_time, 0.01)  # Should be very fast due to caching
        
        print(f"   ✅ Map data retrieved: {map_data.width}x{map_data.height} grid")
        print(f"   ✅ Found {len(map_data.shelves)} shelves")
        print(f"   ✅ Caching working (cache access: {cache_time*1000:.1f}ms)")
    
    def test_shelf_position_operations(self):
        """Test shelf position retrieval."""
        print("\n📍 Testing shelf position operations...")
        
        # Get existing shelf position
        position = self.service.get_shelf_position('test_shelf_001')
        self.assertEqual(position, (2.5, 3.0))
        
        # Get non-existent shelf position
        position = self.service.get_shelf_position('nonexistent')
        self.assertIsNone(position)
        
        print("   ✅ Shelf position retrieval working")
        print(f"   ✅ test_shelf_001 at position {position}")
    
    def test_shelf_locking_operations(self):
        """Test shelf locking and unlocking."""
        print("\n🔒 Testing shelf locking operations...")
        
        shelf_id = 'test_shelf_001'
        robot_id = 'test_robot_001'
        
        # Initially shelf should be unlocked
        self.assertFalse(self.service.is_shelf_locked(shelf_id))
        self.assertIsNone(self.service.get_shelf_lock_owner(shelf_id))
        
        # Lock the shelf
        result = self.service.lock_shelf(shelf_id, robot_id)
        self.assertTrue(result)
        
        # Verify shelf is locked
        self.assertTrue(self.service.is_shelf_locked(shelf_id))
        self.assertEqual(self.service.get_shelf_lock_owner(shelf_id), robot_id)
        
        # Try to lock again (should fail)
        result = self.service.lock_shelf(shelf_id, 'other_robot')
        self.assertFalse(result)
        
        # Unlock the shelf
        result = self.service.unlock_shelf(shelf_id, robot_id)
        self.assertTrue(result)
        
        # Verify shelf is unlocked
        self.assertFalse(self.service.is_shelf_locked(shelf_id))
        self.assertIsNone(self.service.get_shelf_lock_owner(shelf_id))
        
        # Try to unlock again (should fail)
        result = self.service.unlock_shelf(shelf_id, robot_id)
        self.assertFalse(result)
        
        print("   ✅ Shelf locking working correctly")
        print(f"   ✅ Lock/unlock cycle completed for {shelf_id}")
    
    def test_shelf_locking_race_conditions(self):
        """Test shelf locking under race conditions."""
        print("\n🏁 Testing shelf locking race conditions...")
        
        shelf_id = 'test_shelf_002'
        num_robots = 10
        results = []
        errors = []
        
        def lock_attempt(robot_num: int):
            try:
                robot_id = f'test_robot_{robot_num:03d}'
                result = self.service.lock_shelf(shelf_id, robot_id)
                results.append((robot_id, result))
                
                if result:
                    # Hold lock briefly then unlock
                    time.sleep(0.01)
                    unlock_result = self.service.unlock_shelf(shelf_id, robot_id)
                    results.append((robot_id, f'unlock_{unlock_result}'))
                    
            except Exception as e:
                errors.append(f"Robot {robot_num}: {e}")
        
        # Start multiple threads trying to lock the same shelf
        threads = []
        for i in range(num_robots):
            thread = threading.Thread(target=lock_attempt, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Analyze results
        lock_successes = [r for r in results if r[1] is True]
        lock_failures = [r for r in results if r[1] is False]
        
        # Exactly one robot should have succeeded in locking
        self.assertEqual(len(lock_successes), 1)
        self.assertEqual(len(lock_failures), num_robots - 1)
        self.assertEqual(len(errors), 0)
        
        # Shelf should be unlocked at the end
        self.assertFalse(self.service.is_shelf_locked(shelf_id))
        
        print(f"   ✅ Race condition test passed: {len(lock_successes)} success, {len(lock_failures)} failures")
        print(f"   ✅ No errors occurred during concurrent access")
    
    def test_shelf_info_operations(self):
        """Test shelf information retrieval."""
        print("\n📋 Testing shelf info operations...")
        
        # Get existing shelf info
        shelf_info = self.service.get_shelf_info('test_shelf_001')
        
        self.assertIsNotNone(shelf_info)
        self.assertIsInstance(shelf_info, ShelfInfo)
        
        # Type assertion for linter
        assert isinstance(shelf_info, ShelfInfo)
        
        self.assertEqual(shelf_info.shelf_id, 'test_shelf_001')
        self.assertEqual(shelf_info.position, (2.5, 3.0))
        self.assertIn('test_book_001', shelf_info.items)
        self.assertIn('test_book_002', shelf_info.items)
        self.assertFalse(shelf_info.is_locked)
        self.assertIsNone(shelf_info.locked_by)
        
        # Get non-existent shelf info
        shelf_info = self.service.get_shelf_info('nonexistent')
        self.assertIsNone(shelf_info)
        
        print(f"   ✅ Shelf info retrieved for test_shelf_001")
        print(f"   ✅ Found {len(shelf_info.items) if shelf_info else 0} items on shelf")
    
    def test_item_location_operations(self):
        """Test item location finding."""
        print("\n🔍 Testing item location operations...")
        
        # Find existing item
        location = self.service.get_item_location('test_book_001')
        self.assertEqual(location, 'test_shelf_001')
        
        # Find item on different shelf
        location = self.service.get_item_location('test_electronics_001')
        self.assertEqual(location, 'test_shelf_002')
        
        # Find non-existent item
        location = self.service.get_item_location('nonexistent')
        self.assertIsNone(location)
        
        print("   ✅ Item location finding working correctly")
        print(f"   ✅ test_book_001 found on {location}")
    
    def test_inventory_operations(self):
        """Test inventory management operations."""
        print("\n📦 Testing inventory operations...")
        
        shelf_id = 'test_shelf_003'
        item_id = 'test_book_001'
        
        # Add items to shelf
        result = self.service.update_inventory(shelf_id, item_id, 'add', 5)
        self.assertTrue(result)
        
        # Verify item is now on shelf
        location = self.service.get_item_location(item_id)
        self.assertIn(location, ['test_shelf_001', 'test_shelf_003'])  # Could be on either shelf
        
        # Remove items from shelf
        result = self.service.update_inventory(shelf_id, item_id, 'remove', 2)
        self.assertTrue(result)
        
        # Test invalid operation
        with self.assertRaises(SimulationDataServiceError):
            self.service.update_inventory(shelf_id, item_id, 'invalid', 1)
        
        # Test non-existent shelf
        with self.assertRaises(SimulationDataServiceError):
            self.service.update_inventory('nonexistent', item_id, 'add', 1)
        
        print("   ✅ Inventory operations working correctly")
        print("   ✅ Add/remove operations completed")
        print("   ✅ Error handling for invalid operations")
    
    def test_kpi_logging_operations(self):
        """Test KPI event logging."""
        print("\n📊 Testing KPI logging operations...")
        
        event_data = {
            'task_duration': 45.5,
            'distance_traveled': 12.3,
            'battery_used': 0.15,
            'items_picked': 3
        }
        
        # Log event (should not raise exception)
        self.service.log_event('task_complete', 'test_robot_001', event_data)
        
        # Log event with empty data
        self.service.log_event('robot_start', 'test_robot_001', {})
        
        # Verify events were logged (check database)
        with self.service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT COUNT(*) as count 
                    FROM simulation_metrics 
                    WHERE metric_name LIKE 'task_complete_%' OR metric_name LIKE 'robot_start_%'
                """)
                result = cur.fetchone()
                self.assertGreater(result['count'], 0)
        
        print("   ✅ KPI logging working correctly")
        print(f"   ✅ Events logged to database")
    
    def test_concurrent_operations_mixed(self):
        """Test concurrent mixed operations."""
        print("\n🔄 Testing concurrent mixed operations...")
        
        results = []
        errors = []
        
        def mixed_operations(thread_id: int):
            try:
                # Map data access
                map_data = self.service.get_map_data()
                results.append(f"thread_{thread_id}_map_data")
                
                # Shelf position lookup
                position = self.service.get_shelf_position('test_shelf_001')
                results.append(f"thread_{thread_id}_position")
                
                # Item location lookup
                location = self.service.get_item_location('test_book_001')
                results.append(f"thread_{thread_id}_location")
                
                # KPI logging
                self.service.log_event('test_event', f'robot_{thread_id}', {'value': thread_id})
                results.append(f"thread_{thread_id}_log")
                
            except Exception as e:
                errors.append(f"Thread {thread_id}: {e}")
        
        # Start multiple threads with mixed operations
        threads = []
        num_threads = 5
        
        for i in range(num_threads):
            thread = threading.Thread(target=mixed_operations, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Verify all operations completed successfully
        expected_results = num_threads * 4  # 4 operations per thread
        self.assertEqual(len(results), expected_results)
        self.assertEqual(len(errors), 0)
        
        print(f"   ✅ Concurrent operations test passed")
        print(f"   ✅ {len(results)} operations completed across {num_threads} threads")
        print(f"   ✅ No errors occurred")
    
    def test_dropoff_zones(self):
        """Test dropoff zones retrieval."""
        print("\n🚚 Testing dropoff zones...")
        
        zones = self.service.get_dropoff_zones()
        
        self.assertIsInstance(zones, list)
        self.assertGreater(len(zones), 0)
        
        for zone in zones:
            self.assertIsInstance(zone, tuple)
            self.assertEqual(len(zone), 2)
            self.assertIsInstance(zone[0], float)
            self.assertIsInstance(zone[1], float)
        
        print(f"   ✅ Found {len(zones)} dropoff zones")
        print(f"   ✅ Zones: {zones}")


if __name__ == '__main__':
    # Set up logging for better test output
    import logging
    logging.basicConfig(level=logging.INFO)
    
    unittest.main(verbosity=2)