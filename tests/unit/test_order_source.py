"""
Comprehensive Unit Tests for Order Source Interface and JsonOrderSource Implementation.

Tests cover:
- Interface compliance and validation
- File loading and parsing
- Order format validation
- Smart file watching mechanism
- Priority sorting and filtering
- Thread safety and error handling
- Status tracking and statistics
- Edge cases and error conditions
"""

import unittest
import tempfile
import json
import os
import time
import threading
from datetime import datetime, timedelta
from pathlib import Path
from unittest.mock import patch, mock_open

from interfaces.order_source_interface import (
    IOrderSource, OrderSourceError, OrderSourceType, OrderSourceStatus
)
from interfaces.jobs_processor_interface import (
    Order, OrderItem, OrderStatus, Priority
)
from warehouse.impl.json_order_source import JsonOrderSource


class TestOrderDataStructures(unittest.TestCase):
    """Test Order and OrderItem data structures."""
    
    def test_order_item_creation(self):
        """Test OrderItem creation and validation."""
        # Valid order item
        item = OrderItem(item_id="book_001", quantity=2)
        self.assertEqual(item.item_id, "book_001")
        self.assertEqual(item.quantity, 2)
        
        # Invalid quantity
        with self.assertRaises(ValueError):
            OrderItem(item_id="book_001", quantity=0)
        
        with self.assertRaises(ValueError):
            OrderItem(item_id="book_001", quantity=-1)
    
    def test_order_creation(self):
        """Test Order creation and validation."""
        items = [OrderItem(item_id="book_001", quantity=1)]
        scheduled_time = datetime.now() + timedelta(minutes=5)
        
        # Valid order
        order = Order(
            order_id="test_001",
            items=items,
            scheduled_time=scheduled_time,
            priority=Priority.HIGH,
            customer_id="customer_001"
        )
        
        self.assertEqual(order.order_id, "test_001")
        self.assertEqual(len(order.items), 1)
        self.assertEqual(order.priority, Priority.HIGH)
        self.assertEqual(order.status, OrderStatus.PENDING)
        self.assertIsNotNone(order.created_at)
        
        # Invalid order - no items
        with self.assertRaises(ValueError):
            Order(
                order_id="test_002",
                items=[],
                scheduled_time=scheduled_time
            )
        
        # Invalid order - no order_id
        with self.assertRaises(ValueError):
            Order(
                order_id="",
                items=items,
                scheduled_time=scheduled_time
            )


class TestJsonOrderSourceBasics(unittest.TestCase):
    """Test basic JsonOrderSource functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
        
        # Sample order data
        self.sample_orders = [
            {
                "order_id": "order_001",
                "items": [{"item_id": "book_001", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:00",
                "priority": "normal",
                "customer_id": "customer_001"
            },
            {
                "order_id": "order_002",
                "items": [{"item_id": "phone_001", "quantity": 2}],
                "scheduled_time": "2025-06-29T12:30:15",
                "priority": "high",
                "customer_id": "customer_002"
            }
        ]
        
        # Write sample data to file
        with open(self.test_file, 'w') as f:
            json.dump(self.sample_orders, f)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_initialization(self):
        """Test JsonOrderSource initialization."""
        source = JsonOrderSource(self.test_file, "test")
        
        self.assertEqual(source.file_path, Path(self.test_file))
        self.assertEqual(source.order_id_prefix, "test")
        self.assertFalse(source.is_connected())
    
    def test_connection_lifecycle(self):
        """Test connection and disconnection."""
        source = JsonOrderSource(self.test_file)
        
        # Initially disconnected
        self.assertFalse(source.is_connected())
        
        # Connect
        source.connect()
        self.assertTrue(source.is_connected())
        
        # Disconnect
        source.disconnect()
        self.assertFalse(source.is_connected())
    
    def test_connection_error_missing_file(self):
        """Test connection error when file doesn't exist."""
        source = JsonOrderSource("nonexistent_file.json")
        
        with self.assertRaises(OrderSourceError):
            source.connect()
    
    def test_load_orders_from_file(self):
        """Test loading orders from JSON file."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        orders = source.get_all_orders()
        self.assertEqual(len(orders), 2)
        
        # Check first order
        order1 = next(o for o in orders if o.order_id == "order_001")
        self.assertEqual(order1.order_id, "order_001")
        self.assertEqual(len(order1.items), 1)
        self.assertEqual(order1.items[0].item_id, "book_001")
        self.assertEqual(order1.items[0].quantity, 1)
        self.assertEqual(order1.priority, Priority.NORMAL)
        self.assertEqual(order1.customer_id, "customer_001")
        
        source.disconnect()


class TestOrderValidationAndParsing(unittest.TestCase):
    """Test order format validation and parsing."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
        self.source = JsonOrderSource(self.test_file)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_validate_order_format_valid(self):
        """Test validation of valid order formats."""
        valid_order = {
            "order_id": "order_001",
            "items": [{"item_id": "book_001", "quantity": 1}],
            "scheduled_time": "2025-06-29T12:30:00"
        }
        
        self.assertTrue(self.source.validate_order_format(valid_order))
    
    def test_validate_order_format_invalid(self):
        """Test validation of invalid order formats."""
        # Missing order_id
        invalid_order1 = {
            "items": [{"item_id": "book_001", "quantity": 1}],
            "scheduled_time": "2025-06-29T12:30:00"
        }
        self.assertFalse(self.source.validate_order_format(invalid_order1))
        
        # Missing items
        invalid_order2 = {
            "order_id": "order_001",
            "scheduled_time": "2025-06-29T12:30:00"
        }
        self.assertFalse(self.source.validate_order_format(invalid_order2))
        
        # Missing scheduled_time
        invalid_order3 = {
            "order_id": "order_001",
            "items": [{"item_id": "book_001", "quantity": 1}]
        }
        self.assertFalse(self.source.validate_order_format(invalid_order3))
    
    def test_parse_different_time_formats(self):
        """Test parsing different scheduled_time formats."""
        # ISO format
        order_iso = {
            "order_id": "order_001",
            "items": [{"item_id": "book_001", "quantity": 1}],
            "scheduled_time": "2025-06-29T12:30:00",
            "priority": "normal"
        }
        
        # Timestamp format
        timestamp = time.time() + 3600  # 1 hour from now
        order_timestamp = {
            "order_id": "order_002",
            "items": [{"item_id": "book_002", "quantity": 1}],
            "scheduled_time": timestamp,
            "priority": "high"
        }
        
        # Write both formats to file
        with open(self.test_file, 'w') as f:
            json.dump([order_iso, order_timestamp], f)
        
        self.source.connect()
        orders = self.source.get_all_orders()
        
        self.assertEqual(len(orders), 2)
        
        # Check ISO format parsing
        iso_order = next(o for o in orders if o.order_id == "order_001")
        self.assertEqual(iso_order.scheduled_time, datetime(2025, 6, 29, 12, 30, 0))
        
        # Check timestamp format parsing
        ts_order = next(o for o in orders if o.order_id == "order_002")
        expected_time = datetime.fromtimestamp(timestamp)
        self.assertEqual(ts_order.scheduled_time, expected_time)
        
        self.source.disconnect()
    
    def test_parse_all_priority_levels(self):
        """Test parsing all priority levels."""
        orders_data = [
            {"order_id": "urgent", "items": [{"item_id": "item1", "quantity": 1}], 
             "scheduled_time": "2025-06-29T12:30:00", "priority": "urgent"},
            {"order_id": "high", "items": [{"item_id": "item2", "quantity": 1}], 
             "scheduled_time": "2025-06-29T12:30:00", "priority": "high"},
            {"order_id": "normal", "items": [{"item_id": "item3", "quantity": 1}], 
             "scheduled_time": "2025-06-29T12:30:00", "priority": "normal"},
            {"order_id": "low", "items": [{"item_id": "item4", "quantity": 1}], 
             "scheduled_time": "2025-06-29T12:30:00", "priority": "low"},
            {"order_id": "default", "items": [{"item_id": "item5", "quantity": 1}], 
             "scheduled_time": "2025-06-29T12:30:00"}  # No priority = normal
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(orders_data, f)
        
        self.source.connect()
        orders = self.source.get_all_orders()
        
        priorities = {order.order_id: order.priority for order in orders}
        
        self.assertEqual(priorities["urgent"], Priority.URGENT)
        self.assertEqual(priorities["high"], Priority.HIGH)
        self.assertEqual(priorities["normal"], Priority.NORMAL)
        self.assertEqual(priorities["low"], Priority.LOW)
        self.assertEqual(priorities["default"], Priority.NORMAL)  # Default priority
        
        self.source.disconnect()


class TestSmartFileWatching(unittest.TestCase):
    """Test smart file watching and change detection."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
        
        # Initial orders
        self.initial_orders = [
            {
                "order_id": "order_001",
                "items": [{"item_id": "book_001", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:00",
                "priority": "normal"
            }
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(self.initial_orders, f)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_file_change_detection(self):
        """Test that file changes are detected and orders reloaded."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Initial load
        orders = source.get_all_orders()
        self.assertEqual(len(orders), 1)
        
        # Simulate file change
        time.sleep(0.1)  # Ensure different mtime
        updated_orders = self.initial_orders + [
            {
                "order_id": "order_002",
                "items": [{"item_id": "phone_001", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:15",
                "priority": "high"
            }
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(updated_orders, f)
        
        # Next call should detect change and reload
        orders = source.get_all_orders()
        self.assertEqual(len(orders), 2)
        
        source.disconnect()
    
    def test_no_unnecessary_reloads(self):
        """Test that file isn't reloaded when unchanged."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Get initial stats
        stats1 = source.get_source_stats()
        initial_reload_count = stats1['file_reload_count']
        
        # Multiple calls without file changes
        source.get_all_orders()
        source.get_all_orders()
        source.get_all_orders()
        
        # Should not trigger additional reloads
        stats2 = source.get_source_stats()
        self.assertEqual(stats2['file_reload_count'], initial_reload_count)
        
        source.disconnect()


class TestOrderFiltering(unittest.TestCase):
    """Test order filtering and due order logic."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
        
        # Create orders with different scheduled times and priorities
        now = datetime.now()
        self.orders_data = [
            {
                "order_id": "past_urgent",
                "items": [{"item_id": "item1", "quantity": 1}],
                "scheduled_time": (now - timedelta(minutes=5)).isoformat(),
                "priority": "urgent"
            },
            {
                "order_id": "past_normal",
                "items": [{"item_id": "item2", "quantity": 1}],
                "scheduled_time": (now - timedelta(minutes=3)).isoformat(),
                "priority": "normal"
            },
            {
                "order_id": "future_high",
                "items": [{"item_id": "item3", "quantity": 1}],
                "scheduled_time": (now + timedelta(minutes=5)).isoformat(),
                "priority": "high"
            },
            {
                "order_id": "future_low",
                "items": [{"item_id": "item4", "quantity": 1}],
                "scheduled_time": (now + timedelta(minutes=10)).isoformat(),
                "priority": "low"
            }
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(self.orders_data, f)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_get_due_orders(self):
        """Test getting orders that are due for processing."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        due_orders = source.get_due_orders()
        
        # Should only get past orders (due for processing)
        self.assertEqual(len(due_orders), 2)
        due_ids = [order.order_id for order in due_orders]
        self.assertIn("past_urgent", due_ids)
        self.assertIn("past_normal", due_ids)
        
        # Should be sorted by priority (urgent first)
        self.assertEqual(due_orders[0].order_id, "past_urgent")
        self.assertEqual(due_orders[1].order_id, "past_normal")
        
        source.disconnect()
    
    def test_due_orders_limit(self):
        """Test limiting the number of due orders returned."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        due_orders = source.get_due_orders(limit=1)
        
        # Should only return 1 order (highest priority)
        self.assertEqual(len(due_orders), 1)
        self.assertEqual(due_orders[0].order_id, "past_urgent")
        
        source.disconnect()
    
    def test_due_orders_no_duplicates(self):
        """Test that due orders aren't returned multiple times."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # First call
        due_orders1 = source.get_due_orders()
        self.assertEqual(len(due_orders1), 2)
        
        # Second call should return empty (orders already processed)
        due_orders2 = source.get_due_orders()
        self.assertEqual(len(due_orders2), 0)
        
        source.disconnect()
    
    def test_status_filtering(self):
        """Test filtering orders by status."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # All orders should be pending initially
        pending_orders = source.get_all_orders(OrderStatus.PENDING)
        self.assertEqual(len(pending_orders), 4)
        
        # Update one order status
        source.update_order_status("past_urgent", OrderStatus.PROCESSING)
        
        # Test filtering
        pending_orders = source.get_all_orders(OrderStatus.PENDING)
        processing_orders = source.get_all_orders(OrderStatus.PROCESSING)
        
        self.assertEqual(len(pending_orders), 3)
        self.assertEqual(len(processing_orders), 1)
        self.assertEqual(processing_orders[0].order_id, "past_urgent")
        
        source.disconnect()


class TestOrderManagement(unittest.TestCase):
    """Test order management operations."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
        
        # Simple test order
        self.test_orders = [
            {
                "order_id": "order_001",
                "items": [{"item_id": "book_001", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:00",
                "priority": "normal"
            }
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(self.test_orders, f)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_get_order_by_id(self):
        """Test getting a specific order by ID."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Existing order
        order = source.get_order_by_id("order_001")
        self.assertIsNotNone(order)
        self.assertEqual(order.order_id, "order_001")
        
        # Non-existing order
        order = source.get_order_by_id("nonexistent")
        self.assertIsNone(order)
        
        source.disconnect()
    
    def test_update_order_status(self):
        """Test updating order status."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Update existing order
        success = source.update_order_status(
            "order_001", 
            OrderStatus.PROCESSING,
            {"processing_start": "2025-06-29T12:35:00"}
        )
        self.assertTrue(success)
        
        # Verify update
        order = source.get_order_by_id("order_001")
        self.assertEqual(order.status, OrderStatus.PROCESSING)
        self.assertIn("processing_start", order.metadata)
        
        # Update non-existing order
        success = source.update_order_status("nonexistent", OrderStatus.FAILED)
        self.assertFalse(success)
        
        source.disconnect()
    
    def test_add_order(self):
        """Test adding new orders."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Add new order
        new_order = Order(
            order_id="new_order",
            items=[OrderItem(item_id="new_item", quantity=1)],
            scheduled_time=datetime.now() + timedelta(hours=1),
            priority=Priority.HIGH
        )
        
        success = source.add_order(new_order)
        self.assertTrue(success)
        
        # Verify addition
        orders = source.get_all_orders()
        self.assertEqual(len(orders), 2)
        
        added_order = source.get_order_by_id("new_order")
        self.assertIsNotNone(added_order)
        self.assertEqual(added_order.priority, Priority.HIGH)
        
        # Try to add duplicate order
        success = source.add_order(new_order)
        self.assertFalse(success)
        
        source.disconnect()
    
    def test_remove_order(self):
        """Test removing orders."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Remove existing order
        success = source.remove_order("order_001")
        self.assertTrue(success)
        
        # Verify removal
        orders = source.get_all_orders()
        self.assertEqual(len(orders), 0)
        
        order = source.get_order_by_id("order_001")
        self.assertIsNone(order)
        
        # Remove non-existing order
        success = source.remove_order("nonexistent")
        self.assertFalse(success)
        
        source.disconnect()


class TestStatisticsAndMonitoring(unittest.TestCase):
    """Test statistics and monitoring functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
        
        self.test_orders = [
            {
                "order_id": "order_001",
                "items": [{"item_id": "book_001", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:00",
                "priority": "normal"
            }
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(self.test_orders, f)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_get_source_info(self):
        """Test getting source information."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        info = source.get_source_info()
        
        self.assertEqual(info['type'], OrderSourceType.JSON_FILE.value)
        self.assertEqual(info['file_path'], self.test_file)
        self.assertGreater(info['file_size'], 0)
        self.assertIsInstance(info['last_modified'], datetime)
        self.assertEqual(info['order_count'], 1)
        
        source.disconnect()
    
    def test_get_source_stats(self):
        """Test getting source statistics."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Initial stats
        stats = source.get_source_stats()
        self.assertEqual(stats['total_orders_loaded'], 1)
        self.assertEqual(stats['total_polls'], 0)
        self.assertEqual(stats['file_reload_count'], 1)
        self.assertEqual(stats['error_count'], 0)
        self.assertEqual(stats['pending_orders'], 1)
        self.assertEqual(stats['processed_orders'], 0)
        
        # After polling
        source.get_due_orders()
        stats = source.get_source_stats()
        self.assertEqual(stats['total_polls'], 1)
        self.assertIsNotNone(stats['last_poll_time'])
        
        source.disconnect()
    
    def test_polling_interval(self):
        """Test getting polling interval."""
        source = JsonOrderSource(self.test_file)
        interval = source.get_polling_interval()
        
        self.assertIsInstance(interval, float)
        self.assertGreater(interval, 0)
        self.assertEqual(interval, 2.0)  # Expected default for JSON files


class TestErrorHandling(unittest.TestCase):
    """Test error handling and edge cases."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_invalid_json_file(self):
        """Test handling of invalid JSON files."""
        # Create invalid JSON file
        with open(self.test_file, 'w') as f:
            f.write("{ invalid json")
        
        source = JsonOrderSource(self.test_file)
        
        with self.assertRaises(OrderSourceError):
            source.connect()
    
    def test_non_list_json(self):
        """Test handling of JSON that isn't a list."""
        # Create JSON object instead of list
        with open(self.test_file, 'w') as f:
            json.dump({"not": "a list"}, f)
        
        source = JsonOrderSource(self.test_file)
        
        with self.assertRaises(OrderSourceError):
            source.connect()
    
    def test_operations_when_disconnected(self):
        """Test that operations fail when disconnected."""
        source = JsonOrderSource(self.test_file)
        
        # All operations should fail when not connected
        with self.assertRaises(OrderSourceError):
            source.get_due_orders()
        
        with self.assertRaises(OrderSourceError):
            source.get_all_orders()
        
        with self.assertRaises(OrderSourceError):
            source.get_order_by_id("test")
        
        with self.assertRaises(OrderSourceError):
            source.refresh()
    
    def test_malformed_order_handling(self):
        """Test handling of malformed orders in file."""
        malformed_orders = [
            {
                "order_id": "good_order",
                "items": [{"item_id": "book_001", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:00",
                "priority": "normal"
            },
            {
                "order_id": "bad_order",
                # Missing items and scheduled_time
                "priority": "high"
            },
            {
                "order_id": "another_good_order",
                "items": [{"item_id": "phone_001", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:15",
                "priority": "low"
            }
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(malformed_orders, f)
        
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        # Should load only valid orders
        orders = source.get_all_orders()
        self.assertEqual(len(orders), 2)
        
        order_ids = [order.order_id for order in orders]
        self.assertIn("good_order", order_ids)
        self.assertIn("another_good_order", order_ids)
        self.assertNotIn("bad_order", order_ids)
        
        source.disconnect()


class TestThreadSafety(unittest.TestCase):
    """Test thread safety of JsonOrderSource."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.test_file = os.path.join(self.temp_dir, "test_orders.json")
        
        # Create orders for concurrent testing
        self.test_orders = [
            {
                "order_id": f"order_{i:03d}",
                "items": [{"item_id": f"item_{i}", "quantity": 1}],
                "scheduled_time": "2025-06-29T12:30:00",
                "priority": "normal"
            }
            for i in range(10)
        ]
        
        with open(self.test_file, 'w') as f:
            json.dump(self.test_orders, f)
    
    def tearDown(self):
        """Clean up test fixtures."""
        if os.path.exists(self.test_file):
            os.remove(self.test_file)
        os.rmdir(self.temp_dir)
    
    def test_concurrent_reads(self):
        """Test concurrent read operations."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        results = []
        errors = []
        
        def read_orders():
            try:
                orders = source.get_all_orders()
                results.append(len(orders))
            except Exception as e:
                errors.append(e)
        
        # Start multiple threads
        threads = []
        for _ in range(5):
            thread = threading.Thread(target=read_orders)
            threads.append(thread)
            thread.start()
        
        # Wait for all threads
        for thread in threads:
            thread.join()
        
        # All threads should succeed
        self.assertEqual(len(errors), 0)
        self.assertEqual(len(results), 5)
        self.assertTrue(all(count == 10 for count in results))
        
        source.disconnect()
    
    def test_concurrent_status_updates(self):
        """Test concurrent status update operations."""
        source = JsonOrderSource(self.test_file)
        source.connect()
        
        errors = []
        
        def update_status(order_id, status):
            try:
                source.update_order_status(order_id, status)
            except Exception as e:
                errors.append(e)
        
        # Start multiple threads updating different orders
        threads = []
        for i in range(5):
            thread = threading.Thread(
                target=update_status,
                args=(f"order_{i:03d}", OrderStatus.PROCESSING)
            )
            threads.append(thread)
            thread.start()
        
        # Wait for all threads
        for thread in threads:
            thread.join()
        
        # All threads should succeed
        self.assertEqual(len(errors), 0)
        
        # Verify updates
        for i in range(5):
            order = source.get_order_by_id(f"order_{i:03d}")
            self.assertEqual(order.status, OrderStatus.PROCESSING)
        
        source.disconnect()


if __name__ == '__main__':
    unittest.main() 