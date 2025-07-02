"""
Integration Tests for Order Source with Real sample_orders.json.

Tests the complete Order Source system using the actual sample_orders.json file
to ensure real-world compatibility and end-to-end functionality.
"""

import unittest
import time
from datetime import datetime, timedelta
from pathlib import Path

from warehouse.impl.json_order_source import JsonOrderSource
from interfaces.jobs_processor_interface import OrderStatus, Priority


class TestOrderSourceIntegration(unittest.TestCase):
    """Integration tests using real sample_orders.json file."""
    
    def setUp(self):
        """Set up integration test fixtures."""
        # Use the actual sample_orders.json from project root
        self.sample_file = Path(__file__).parent.parent.parent / "sample_orders.json"
        self.assertTrue(self.sample_file.exists(), 
                       f"sample_orders.json not found at {self.sample_file}")
    
    def test_load_real_sample_orders(self):
        """Test loading orders from real sample_orders.json file."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            # Load all orders
            orders = source.get_all_orders()
            
            # Verify expected number of orders (should match sample_orders.json)
            self.assertEqual(len(orders), 7, "Expected 7 orders in sample_orders.json")
            
            # Verify order IDs are correct
            expected_order_ids = [
                "order_001", "order_002", "order_003", "order_004",
                "order_005", "order_006", "order_007"
            ]
            actual_order_ids = sorted([order.order_id for order in orders])
            self.assertEqual(actual_order_ids, expected_order_ids)
            
            # Verify all orders have required fields
            for order in orders:
                self.assertIsNotNone(order.order_id)
                self.assertGreater(len(order.items), 0)
                self.assertIsNotNone(order.scheduled_time)
                self.assertIsInstance(order.priority, Priority)
                self.assertEqual(order.status, OrderStatus.PENDING)
                self.assertIsNotNone(order.customer_id)
            
            print(f"✅ Successfully loaded {len(orders)} orders from sample_orders.json")
            
        finally:
            source.disconnect()
    
    def test_priority_distribution(self):
        """Test that sample orders have expected priority distribution."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            orders = source.get_all_orders()
            
            # Count priorities
            priority_counts = {}
            for order in orders:
                priority = order.priority
                priority_counts[priority] = priority_counts.get(priority, 0) + 1
            
            # Verify we have different priority levels
            self.assertIn(Priority.URGENT, priority_counts)
            self.assertIn(Priority.HIGH, priority_counts)
            self.assertIn(Priority.NORMAL, priority_counts)
            self.assertIn(Priority.LOW, priority_counts)
            
            print(f"✅ Priority distribution: {priority_counts}")
            
        finally:
            source.disconnect()
    
    def test_item_variety(self):
        """Test that sample orders contain variety of items."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            orders = source.get_all_orders()
            
            # Collect all item types
            item_types = set()
            total_quantity = 0
            
            for order in orders:
                for item in order.items:
                    item_types.add(item.item_id)
                    total_quantity += item.quantity
            
            # Verify variety
            expected_items = {"book_001", "phone_001", "laptop_001", "tablet_001", "audio_001"}
            self.assertTrue(expected_items.issubset(item_types), 
                           f"Expected items {expected_items}, got {item_types}")
            
            self.assertGreater(total_quantity, len(orders), 
                              "Should have some orders with quantity > 1")
            
            print(f"✅ Found {len(item_types)} different item types")
            print(f"✅ Total quantity across all orders: {total_quantity}")
            
        finally:
            source.disconnect()
    
    def test_customer_distribution(self):
        """Test customer distribution in sample orders."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            orders = source.get_all_orders()
            
            # Count orders per customer
            customer_counts = {}
            for order in orders:
                customer = order.customer_id
                customer_counts[customer] = customer_counts.get(customer, 0) + 1
            
            # Verify we have multiple customers
            self.assertGreaterEqual(len(customer_counts), 3, 
                                   "Should have at least 3 different customers")
            
            # Verify some customers have multiple orders
            multiple_order_customers = [c for c, count in customer_counts.items() if count > 1]
            self.assertGreater(len(multiple_order_customers), 0, 
                              "Should have at least one customer with multiple orders")
            
            print(f"✅ Customer distribution: {customer_counts}")
            
        finally:
            source.disconnect()
    
    def test_scheduled_time_sequence(self):
        """Test that orders have reasonable scheduled times."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            orders = source.get_all_orders()
            
            # Get all scheduled times
            scheduled_times = [order.scheduled_time for order in orders]
            scheduled_times.sort()
            
            # Verify times are in sequence (not all the same)
            self.assertNotEqual(scheduled_times[0], scheduled_times[-1], 
                               "Orders should have different scheduled times")
            
            # Verify reasonable time span (not too far apart)
            time_span = scheduled_times[-1] - scheduled_times[0]
            self.assertLess(time_span.total_seconds(), 3600, 
                           "Orders should be scheduled within 1 hour of each other")
            
            print(f"✅ Orders scheduled from {scheduled_times[0]} to {scheduled_times[-1]}")
            print(f"✅ Time span: {time_span}")
            
        finally:
            source.disconnect()
    
    def test_order_processing_simulation(self):
        """Test simulating order processing workflow."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            # Simulate time-based order processing
            print("🔄 Simulating order processing workflow...")
            
            # Get initial stats
            initial_stats = source.get_source_stats()
            print(f"   Initial: {initial_stats['pending_orders']} pending orders")
            
            # Process orders in priority order
            processed_count = 0
            while True:
                # Simulate checking for due orders (in real system, would check time)
                # For this test, we'll process all orders regardless of scheduled_time
                orders = source.get_all_orders(OrderStatus.PENDING)
                if not orders:
                    break
                
                # Sort by priority
                orders.sort(key=lambda o: (
                    0 if o.priority == Priority.URGENT else
                    1 if o.priority == Priority.HIGH else
                    2 if o.priority == Priority.NORMAL else 3
                ))
                
                # Process highest priority order
                order = orders[0]
                print(f"   Processing {order.order_id} (priority: {order.priority.value})")
                
                # Simulate processing steps
                source.update_order_status(order.order_id, OrderStatus.PROCESSING)
                time.sleep(0.01)  # Simulate processing time
                source.update_order_status(order.order_id, OrderStatus.COMPLETED)
                
                processed_count += 1
                
                # Safety break
                if processed_count >= 10:
                    break
            
            # Get final stats
            final_stats = source.get_source_stats()
            completed_orders = source.get_all_orders(OrderStatus.COMPLETED)
            
            print(f"✅ Processed {len(completed_orders)} orders")
            print(f"✅ Final stats: {final_stats['pending_orders']} pending, "
                  f"{len(completed_orders)} completed")
            
            # Verify processing worked
            self.assertGreater(len(completed_orders), 0)
            self.assertLess(final_stats['pending_orders'], initial_stats['pending_orders'])
            
        finally:
            source.disconnect()
    
    def test_source_info_and_stats(self):
        """Test source information and statistics."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            # Get source info
            info = source.get_source_info()
            
            # Verify info structure
            required_fields = ['type', 'file_path', 'file_size', 'last_modified', 'order_count']
            for field in required_fields:
                self.assertIn(field, info)
            
            self.assertEqual(info['type'], 'json_file')
            self.assertGreater(info['file_size'], 0)
            self.assertEqual(info['order_count'], 7)
            
            # Get stats
            stats = source.get_source_stats()
            
            # Verify stats structure
            stats_fields = ['total_orders_loaded', 'total_polls', 'file_reload_count', 
                           'error_count', 'pending_orders', 'processed_orders']
            for field in stats_fields:
                self.assertIn(field, stats)
            
            self.assertEqual(stats['total_orders_loaded'], 7)
            self.assertEqual(stats['error_count'], 0)
            self.assertEqual(stats['pending_orders'], 7)
            
            print(f"✅ Source info: {info}")
            print(f"✅ Source stats: {stats}")
            
        finally:
            source.disconnect()
    
    def test_polling_behavior(self):
        """Test polling behavior and efficiency."""
        source = JsonOrderSource(str(self.sample_file))
        source.connect()
        
        try:
            # Test multiple polls
            print("🔄 Testing polling behavior...")
            
            for i in range(5):
                start_time = time.time()
                orders = source.get_due_orders()  # This increments poll counter
                poll_time = time.time() - start_time
                
                print(f"   Poll {i+1}: {len(orders)} orders in {poll_time:.4f}s")
                
                # Polls should be fast (smart file watching)
                self.assertLess(poll_time, 0.1, "Polls should be fast with smart file watching")
            
            # Check stats after polling
            stats = source.get_source_stats()
            self.assertEqual(stats['total_polls'], 5)
            
            print(f"✅ Completed {stats['total_polls']} polls efficiently")
            
        finally:
            source.disconnect()


if __name__ == '__main__':
    unittest.main() 