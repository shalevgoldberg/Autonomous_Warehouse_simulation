"""
Unit tests for StateHolder implementation.

Tests thread safety, MuJoCo integration, and proper state management.
"""
import unittest
import threading
import time
from unittest.mock import Mock, patch

from robot.impl.state_holder_impl import StateHolderImpl
from interfaces.state_holder_interface import RobotPhysicsState


class TestStateHolderImpl(unittest.TestCase):
    """Test cases for StateHolder implementation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.robot_id = "test_robot_1"
        self.state_holder = StateHolderImpl(self.robot_id)
    
    def test_initialization(self):
        """Test proper initialization."""
        self.assertEqual(self.state_holder.robot_id, self.robot_id)
        
        # Test initial state
        state = self.state_holder.get_robot_state()
        self.assertEqual(state.robot_id, self.robot_id)
        self.assertEqual(state.position, (0.0, 0.0, 0.0))
        self.assertEqual(state.velocity, (0.0, 0.0))
        self.assertEqual(state.battery_level, 1.0)
        self.assertIsInstance(state.timestamp, float)
    
    def test_get_position(self):
        """Test position retrieval."""
        position = self.state_holder.get_position()
        self.assertEqual(position, (0.0, 0.0, 0.0))
        self.assertIsInstance(position, tuple)
        self.assertEqual(len(position), 3)
    
    def test_get_velocity(self):
        """Test velocity retrieval."""
        velocity = self.state_holder.get_velocity()
        self.assertEqual(velocity, (0.0, 0.0))
        self.assertIsInstance(velocity, tuple)
        self.assertEqual(len(velocity), 2)
    
    def test_get_battery_level(self):
        """Test battery level retrieval."""
        battery = self.state_holder.get_battery_level()
        self.assertEqual(battery, 1.0)
        self.assertIsInstance(battery, float)
        self.assertGreaterEqual(battery, 0.0)
        self.assertLessEqual(battery, 1.0)
    
    def test_update_from_simulation(self):
        """Test simulation update (testing mode)."""
        initial_state = self.state_holder.get_robot_state()
        
        # Add small delay to ensure timestamp difference
        import time
        time.sleep(0.001)
        
        # Update from simulation (should simulate movement)
        self.state_holder.update_from_simulation()
        
        updated_state = self.state_holder.get_robot_state()
        
        # Position should have changed (simulated movement)
        self.assertNotEqual(initial_state.position, updated_state.position)
        # Battery should have drained slightly
        self.assertLess(updated_state.battery_level, initial_state.battery_level)
        # Timestamp should be more recent
        self.assertGreater(updated_state.timestamp, initial_state.timestamp)
    
    def test_thread_safety_concurrent_reads(self):
        """Test thread safety for concurrent read operations."""
        results = []
        
        def read_state():
            for _ in range(100):
                state = self.state_holder.get_robot_state()
                results.append(state)
                time.sleep(0.001)  # Small delay
        
        # Start multiple read threads
        threads = []
        for _ in range(5):
            thread = threading.Thread(target=read_state)
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # All reads should succeed
        self.assertEqual(len(results), 500)  # 5 threads * 100 reads each
        
        # All states should be valid
        for state in results:
            self.assertIsInstance(state, RobotPhysicsState)
            self.assertEqual(state.robot_id, self.robot_id)
    
    def test_thread_safety_read_write_concurrency(self):
        """Test thread safety between reads and writes."""
        read_results = []
        update_count = 0
        
        def read_continuously():
            for _ in range(50):
                state = self.state_holder.get_robot_state()
                read_results.append(state)
                time.sleep(0.001)
        
        def update_continuously():
            nonlocal update_count
            for _ in range(25):
                self.state_holder.update_from_simulation()
                update_count += 1
                time.sleep(0.002)
        
        # Start read and write threads
        read_thread = threading.Thread(target=read_continuously)
        write_thread = threading.Thread(target=update_continuously)
        
        read_thread.start()
        write_thread.start()
        
        read_thread.join()
        write_thread.join()
        
        # Verify no corruption
        self.assertEqual(len(read_results), 50)
        self.assertEqual(update_count, 25)
        
        # All states should be valid
        for state in read_results:
            self.assertIsInstance(state, RobotPhysicsState)
            self.assertGreaterEqual(state.battery_level, 0.0)
            self.assertLessEqual(state.battery_level, 1.0)
    
    def test_immutable_state_return(self):
        """Test that returned state is immutable copy."""
        state1 = self.state_holder.get_robot_state()
        state2 = self.state_holder.get_robot_state()
        
        # Should be equal but different objects
        self.assertEqual(state1.robot_id, state2.robot_id)
        self.assertEqual(state1.position, state2.position)
        # Objects should be different instances (defensive copy)
        self.assertIsNot(state1, state2)
    
    def test_battery_drain_simulation(self):
        """Test battery drain over multiple updates."""
        initial_battery = self.state_holder.get_battery_level()
        
        # Multiple updates should drain battery
        for _ in range(100):
            self.state_holder.update_from_simulation()
        
        final_battery = self.state_holder.get_battery_level()
        
        self.assertLess(final_battery, initial_battery)
        self.assertGreaterEqual(final_battery, 0.0)  # Should not go negative
    
    def test_position_simulation(self):
        """Test position changes during simulation."""
        initial_position = self.state_holder.get_position()
        
        # Multiple updates should change position
        for _ in range(10):
            self.state_holder.update_from_simulation()
        
        final_position = self.state_holder.get_position()
        
        # Position should have changed
        self.assertNotEqual(initial_position, final_position)
        
        # Changes should be reasonable (not too large)
        dx = abs(final_position[0] - initial_position[0])
        dy = abs(final_position[1] - initial_position[1])
        self.assertLess(dx, 1.0)  # Less than 1 meter change
        self.assertLess(dy, 1.0)
    
    def test_testing_methods(self):
        """Test testing/debugging helper methods."""
        # Test position setting
        test_position = (5.0, 3.0, 1.57)
        self.state_holder.set_test_position(test_position)
        
        current_position = self.state_holder.get_position()
        self.assertEqual(current_position, test_position)
        
        # Test velocity setting
        test_velocity = (1.5, 0.5)
        self.state_holder.set_test_velocity(test_velocity)
        
        current_velocity = self.state_holder.get_velocity()
        self.assertEqual(current_velocity, test_velocity)
    
    def test_mujoco_integration_initialization(self):
        """Test MuJoCo integration during initialization."""
        # Test without MuJoCo (fallback mode)
        state_holder = StateHolderImpl("test_robot")
        
        # Should work in simulation mode
        state = state_holder.get_robot_state()
        self.assertEqual(state.robot_id, "test_robot")
        self.assertIsInstance(state.position, tuple)
        self.assertIsInstance(state.battery_level, float)
        
        # Test with None MuJoCo objects (should not crash)
        state_holder = StateHolderImpl("test_robot", model=None, data=None)
        state = state_holder.get_robot_state()
        self.assertEqual(state.robot_id, "test_robot")


if __name__ == '__main__':
    unittest.main() 