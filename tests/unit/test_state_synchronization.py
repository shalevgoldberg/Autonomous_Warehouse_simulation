"""
Unit tests for enhanced state synchronization system.

Tests verify that:
1. Physics thread updates state holder at 1kHz
2. Control loop reads state without calling updates
3. State is properly synchronized across threads
4. Database synchronization works at configurable frequency
"""
import unittest
import threading
import time
from unittest.mock import Mock, patch

from robot.impl.physics_integration import PhysicsThreadManager, create_physics_thread_manager
from robot.impl.state_holder_impl import StateHolderImpl
from simulation.mujoco_env import SimpleMuJoCoPhysics
from warehouse.map import WarehouseMap


class TestStateSynchronization(unittest.TestCase):
    """Test enhanced state synchronization system."""
    
    def setUp(self):
        """Set up test environment."""
        # Create mock warehouse map
        self.warehouse_map = WarehouseMap(
            width=10,
            height=10
        )
        
        # Create physics engine
        self.physics = SimpleMuJoCoPhysics(self.warehouse_map)
        
        # Create state holder
        self.state_holder = StateHolderImpl(
            robot_id="test_robot",
            physics_engine=self.physics
        )
        
        # Create physics thread manager
        self.physics_manager = create_physics_thread_manager(
            physics=self.physics,
            state_holder=self.state_holder,
            frequency_hz=1000.0,
            db_sync_frequency_hz=10.0
        )
    
    def tearDown(self):
        """Clean up test resources."""
        if self.physics_manager.is_running():
            self.physics_manager.stop()
    
    def test_physics_thread_updates_state_holder(self):
        """Test that physics thread updates state holder at 1kHz."""
        # Get initial state
        initial_state = self.state_holder.get_robot_state()
        initial_position = initial_state.position
        
        # Start physics thread
        self.physics_manager.start()
        
        # Wait for physics to run
        time.sleep(0.1)  # 100ms should give ~100 physics steps
        
        # Get updated state
        updated_state = self.state_holder.get_robot_state()
        updated_position = updated_state.position
        
        # Position should have changed (physics simulation)
        self.assertNotEqual(initial_position, updated_position)
        
        # Timestamp should be more recent
        self.assertGreater(updated_state.timestamp, initial_state.timestamp)
        
        # Battery should have drained
        self.assertLess(updated_state.battery_level, initial_state.battery_level)
    
    def test_control_loop_reads_state_without_updates(self):
        """Test that control loop can read state without calling updates."""
        # Start physics thread
        self.physics_manager.start()
        
        # Let physics run for a bit
        time.sleep(0.05)
        
        # Simulate control loop reads (should not call update_from_simulation)
        read_states = []
        for _ in range(10):
            state = self.state_holder.get_robot_state()
            read_states.append(state)
            time.sleep(0.01)  # 10Hz control loop
        
        # All reads should succeed
        self.assertEqual(len(read_states), 10)
        
        # States should be valid
        for state in read_states:
            self.assertIsInstance(state.robot_id, str)
            self.assertIsInstance(state.position, tuple)
            self.assertIsInstance(state.battery_level, float)
            self.assertGreaterEqual(state.battery_level, 0.0)
            self.assertLessEqual(state.battery_level, 1.0)
    
    def test_thread_safety_concurrent_access(self):
        """Test thread safety with concurrent access from multiple threads."""
        # Start physics thread
        self.physics_manager.start()
        
        # Let physics run for a bit
        time.sleep(0.05)
        
        # Simulate multiple threads reading state
        read_results = []
        read_errors = []
        
        def read_state_thread(thread_id):
            try:
                for i in range(20):
                    state = self.state_holder.get_robot_state()
                    read_results.append((thread_id, i, state))
                    time.sleep(0.001)  # 1ms delay
            except Exception as e:
                read_errors.append((thread_id, str(e)))
        
        # Start multiple read threads
        threads = []
        for i in range(5):
            thread = threading.Thread(target=read_state_thread, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # No errors should occur
        self.assertEqual(len(read_errors), 0, f"Read errors: {read_errors}")
        
        # All reads should succeed
        self.assertEqual(len(read_results), 100)  # 5 threads * 20 reads each
        
        # All states should be valid
        for thread_id, read_id, state in read_results:
            self.assertIsInstance(state.robot_id, str)
            self.assertIsInstance(state.position, tuple)
            self.assertIsInstance(state.battery_level, float)
    
    def test_physics_frequency_accuracy(self):
        """Test that physics runs at correct frequency."""
        # Track physics steps
        step_count = 0
        start_time = time.time()
        
        # Mock physics step to count calls
        original_step = self.physics.step_physics
        
        def counted_step():
            nonlocal step_count
            step_count += 1
            original_step()
        
        self.physics.step_physics = counted_step
        
        # Start physics thread
        self.physics_manager.start()
        
        # Run for 1 second
        time.sleep(1.0)
        
        # Stop physics thread
        self.physics_manager.stop()
        
        # Calculate frequency
        elapsed_time = time.time() - start_time
        actual_frequency = step_count / elapsed_time
        
        # Should be close to 1000Hz (allow 40% tolerance for system load and overhead)
        self.assertGreaterEqual(actual_frequency, 600.0)
        self.assertLessEqual(actual_frequency, 1400.0)
        
        print(f"Physics frequency: {actual_frequency:.1f} Hz (target: 1000 Hz)")
    
    def test_database_synchronization_frequency(self):
        """Test database synchronization at configurable frequency."""
        # Create physics manager with 5Hz database sync
        db_sync_manager = PhysicsThreadManager(
            physics=self.physics,
            state_holder=self.state_holder,
            frequency_hz=1000.0,
            db_sync_frequency_hz=5.0
        )
        
        # Track database sync calls
        sync_count = 0
        
        # Mock database sync method
        original_sync = db_sync_manager._sync_state_to_database
        
        def counted_sync():
            nonlocal sync_count
            sync_count += 1
            original_sync()
        
        db_sync_manager._sync_state_to_database = counted_sync
        
        # Start physics thread
        db_sync_manager.start()
        
        # Run for 1 second
        time.sleep(1.0)
        
        # Stop physics thread
        db_sync_manager.stop()
        
        # Should have ~5 database syncs (allow 50% tolerance for timing)
        self.assertGreaterEqual(sync_count, 3)
        self.assertLessEqual(sync_count, 7)
        
        print(f"Database sync calls: {sync_count} (target: ~5)")
    
    def test_state_consistency_across_threads(self):
        """Test that state is consistent across different threads."""
        # Start physics thread
        self.physics_manager.start()
        
        # Let physics run for a bit
        time.sleep(0.1)
        
        # Read state from multiple threads simultaneously
        states = []
        
        def read_state():
            state = self.state_holder.get_robot_state()
            states.append(state)
        
        # Create multiple threads to read state at the same time
        threads = []
        for _ in range(10):
            thread = threading.Thread(target=read_state)
            threads.append(thread)
        
        # Start all threads simultaneously
        for thread in threads:
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # All states should be identical (same timestamp)
        self.assertEqual(len(states), 10)
        
        # Check that all states have similar timestamps (within 5ms for thread scheduling)
        timestamps = [state.timestamp for state in states]
        timestamp_variance = max(timestamps) - min(timestamps)
        self.assertLessEqual(timestamp_variance, 0.005, "Timestamps should be within 5ms")
        
        # Check that all states have same position (atomic read)
        positions = [state.position for state in states]
        self.assertEqual(len(set(positions)), 1, "All states should have same position")
    
    def test_physics_thread_cleanup(self):
        """Test that physics thread cleans up properly."""
        # Start physics thread
        self.physics_manager.start()
        
        # Verify it's running
        self.assertTrue(self.physics_manager.is_running())
        
        # Stop physics thread
        self.physics_manager.stop()
        
        # Verify it's stopped
        self.assertFalse(self.physics_manager.is_running())
        
        # Thread should be cleaned up
        # Note: Internal thread cleanup is implementation detail
        pass
    
    def test_error_handling_in_physics_thread(self):
        """Test error handling in physics thread."""
        # Create physics manager with state holder
        physics_manager = create_physics_thread_manager(
            physics=self.physics,
            state_holder=self.state_holder,
            frequency_hz=1000.0
        )
        
        # Mock physics step to raise exception
        def failing_step():
            raise RuntimeError("Physics simulation error")
        
        self.physics.step_physics = failing_step
        
        # Start physics thread (should handle errors gracefully)
        physics_manager.start()
        
        # Let it run for a bit
        time.sleep(0.1)
        
        # Should still be running despite errors
        self.assertTrue(physics_manager.is_running())
        
        # Stop physics thread
        physics_manager.stop()


if __name__ == '__main__':
    unittest.main() 