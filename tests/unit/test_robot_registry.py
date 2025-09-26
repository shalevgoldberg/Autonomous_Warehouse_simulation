"""
Unit tests for Robot Registry implementation.

Tests cover:
- Robot registration and listing
- Runtime state persistence and retrieval
- Bulk operations (clear all positions, charge all to full)
- Error handling and edge cases
- Thread safety and database operations
- SOLID principles compliance

Test Strategy:
- Use real database connections (like existing tests)
- Test each method in isolation
- Verify database state persistence
- Test error conditions and edge cases
- Mock minimal external dependencies

Architecture Note:
- Robot registry provides clean abstraction over database operations
- Tests ensure registry is the single source of truth for robot state
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
from typing import List, Dict, Any, Optional, Tuple

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.impl.robot_registry_impl import RobotRegistryImpl
from interfaces.robot_registry_interface import RobotIdentity, RobotRuntimeState


class TestRobotRegistry(unittest.TestCase):
    """Test cases for Robot Registry functionality."""

    def setUp(self):
        """Set up test fixtures."""
        # Create a simple warehouse map for testing
        self.warehouse_map = WarehouseMap(width=10, height=10)

        # Create simulation data service (uses environment variables)
        with patch.dict('os.environ', {'WAREHOUSE_DB_PASSWORD': 'renaspolter'}):
            self.simulation_data_service = SimulationDataServiceImpl(
                warehouse_map=self.warehouse_map
            )

        # Create robot registry
        self.registry = RobotRegistryImpl(self.simulation_data_service)

        # Test data
        self.test_robot_id = "test_robot_001"
        self.test_robot_name = "Test Robot 001"
        self.test_position = (1.5, 2.5, 0.0)
        self.test_battery = 0.85

    def tearDown(self):
        """Clean up test data."""
        try:
            # Clean up any test robots and their state
            with self.simulation_data_service._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("DELETE FROM robot_runtime_state WHERE robot_id LIKE %s", ('test_robot_%',))
                    cur.execute("DELETE FROM robots WHERE robot_id LIKE %s", ('test_robot_%',))
                    conn.commit()
        except Exception:
            # Ignore cleanup errors in tearDown
            pass

    def test_register_robot_new(self):
        """Test registering a new robot."""
        # Register new robot
        result = self.registry.register_robot(self.test_robot_id, self.test_robot_name)

        # Should return True for newly created
        self.assertTrue(result)

        # Verify robot is in registry
        robots = self.registry.list_registered_robots()
        robot_ids = [r.robot_id for r in robots]
        self.assertIn(self.test_robot_id, robot_ids)

        # Verify name is correct
        robot = next(r for r in robots if r.robot_id == self.test_robot_id)
        self.assertEqual(robot.name, self.test_robot_name)

    def test_register_robot_existing(self):
        """Test registering an already existing robot."""
        # Register robot first time
        self.registry.register_robot(self.test_robot_id, self.test_robot_name)

        # Register same robot second time
        result = self.registry.register_robot(self.test_robot_id, "Different Name")

        # Should return False (already exists)
        self.assertFalse(result)

        # Name should remain unchanged
        robots = self.registry.list_registered_robots()
        robot = next(r for r in robots if r.robot_id == self.test_robot_id)
        self.assertEqual(robot.name, self.test_robot_name)

    def test_register_robot_auto_name(self):
        """Test registering a robot with auto-generated name."""
        import uuid
        auto_robot_id = f"auto_robot_{uuid.uuid4().hex[:8]}"

        # Register with no name
        result = self.registry.register_robot(auto_robot_id)

        # Should succeed
        self.assertTrue(result)

        # Verify auto-generated name
        robots = self.registry.list_registered_robots()
        robot = next(r for r in robots if r.robot_id == auto_robot_id)
        self.assertEqual(robot.name, f"Robot {auto_robot_id.replace('_', ' ').title()}")

    def test_list_registered_robots_empty(self):
        """Test listing robots when none are registered."""
        robots = self.registry.list_registered_robots()
        # Should not include our test robots (cleaned up)
        test_robots = [r for r in robots if r.robot_id.startswith('test_robot_')]
        self.assertEqual(len(test_robots), 0)

    def test_list_registered_robots_deterministic_order(self):
        """Test that robots are listed in deterministic order."""
        # Register multiple robots with different names
        robots_data = [
            ("test_z_robot", "Z Robot"),
            ("test_a_robot", "A Robot"),
            ("test_m_robot", "M Robot"),
        ]

        for robot_id, name in robots_data:
            self.registry.register_robot(robot_id, name)

        # List should be ordered by name, then robot_id
        robots = self.registry.list_registered_robots()
        test_robots = [r for r in robots if r.robot_id.startswith('test_')]

        # Should be sorted by name: A Robot, M Robot, Z Robot
        self.assertEqual(len(test_robots), 3)
        self.assertEqual(test_robots[0].name, "A Robot")
        self.assertEqual(test_robots[1].name, "M Robot")
        self.assertEqual(test_robots[2].name, "Z Robot")

    def test_upsert_robot_state_new(self):
        """Test creating new robot state."""
        # Register robot first
        self.registry.register_robot(self.test_robot_id, self.test_robot_name)

        # Upsert state
        self.registry.upsert_robot_state(self.test_robot_id, self.test_position, self.test_battery)

        # Retrieve and verify
        state = self.registry.get_robot_state(self.test_robot_id)
        self.assertIsNotNone(state)
        self.assertEqual(state.robot_id, self.test_robot_id)
        self.assertEqual(state.position, self.test_position)
        self.assertEqual(state.battery_level, self.test_battery)

    def test_upsert_robot_state_update(self):
        """Test updating existing robot state."""
        # Register and set initial state
        self.registry.register_robot(self.test_robot_id, self.test_robot_name)
        self.registry.upsert_robot_state(self.test_robot_id, self.test_position, self.test_battery)

        # Update position and battery
        new_position = (3.0, 4.0, 1.57)
        new_battery = 0.95
        self.registry.upsert_robot_state(self.test_robot_id, new_position, new_battery)

        # Verify updated
        state = self.registry.get_robot_state(self.test_robot_id)
        self.assertIsNotNone(state)
        self.assertEqual(state.position, new_position)
        self.assertEqual(state.battery_level, new_battery)

    def test_upsert_robot_state_clear_position(self):
        """Test clearing robot position while preserving battery."""
        # Register and set initial state
        self.registry.register_robot(self.test_robot_id, self.test_robot_name)
        self.registry.upsert_robot_state(self.test_robot_id, self.test_position, self.test_battery)

        # Clear position (set to None)
        self.registry.upsert_robot_state(self.test_robot_id, None, self.test_battery)

        # Verify position is None but battery preserved
        state = self.registry.get_robot_state(self.test_robot_id)
        self.assertIsNotNone(state)
        self.assertIsNone(state.position)
        self.assertEqual(state.battery_level, self.test_battery)

    def test_get_robot_state_nonexistent(self):
        """Test getting state for non-existent robot."""
        state = self.registry.get_robot_state("nonexistent_robot")
        self.assertIsNone(state)

    def test_clear_robot_state(self):
        """Test clearing robot position while preserving battery."""
        # Register and set state
        self.registry.register_robot(self.test_robot_id, self.test_robot_name)
        self.registry.upsert_robot_state(self.test_robot_id, self.test_position, self.test_battery)

        # Clear position
        self.registry.clear_robot_state(self.test_robot_id)

        # Verify position cleared but battery preserved
        state = self.registry.get_robot_state(self.test_robot_id)
        self.assertIsNotNone(state)
        self.assertIsNone(state.position)
        self.assertEqual(state.battery_level, self.test_battery)

    def test_clear_all_positions(self):
        """Test clearing positions for all robots."""
        # Register multiple robots with positions
        robot_ids = ["test_robot_001", "test_robot_002", "test_robot_003"]
        for robot_id in robot_ids:
            self.registry.register_robot(robot_id, f"Robot {robot_id}")
            self.registry.upsert_robot_state(robot_id, (1.0, 2.0, 0.0), 0.8)

        # Clear all positions
        affected = self.registry.clear_all_positions()

        # Should affect at least the robots we created (may affect more from previous tests)
        self.assertGreaterEqual(affected, len(robot_ids))

        # Verify our test robots have positions cleared but batteries preserved
        for robot_id in robot_ids:
            state = self.registry.get_robot_state(robot_id)
            self.assertIsNotNone(state)
            self.assertIsNone(state.position)
            self.assertEqual(state.battery_level, 0.8)

    def test_charge_all_to_full(self):
        """Test charging all robots to 100% battery."""
        # Register multiple robots with different battery levels
        robot_ids = ["test_robot_001", "test_robot_002"]
        for robot_id in robot_ids:
            self.registry.register_robot(robot_id, f"Robot {robot_id}")
            self.registry.upsert_robot_state(robot_id, None, 0.5)  # Low battery

        # Charge all to full
        affected = self.registry.charge_all_to_full()

        # Should affect at least the robots we created (may affect more from previous tests)
        self.assertGreaterEqual(affected, len(robot_ids))

        # Verify our test robots are charged to 100%
        for robot_id in robot_ids:
            state = self.registry.get_robot_state(robot_id)
            self.assertIsNotNone(state)
            self.assertEqual(state.battery_level, 1.0)

    def test_error_handling_database_failure(self):
        """Test error handling when database operations fail."""
        # Mock database connection to fail
        with patch.object(self.simulation_data_service, '_get_connection', side_effect=Exception("DB Error")):
            # All operations should return safe defaults and log errors
            result = self.registry.register_robot("test_robot", "Test Robot")
            self.assertFalse(result)

            robots = self.registry.list_registered_robots()
            self.assertEqual(robots, [])

            state = self.registry.get_robot_state("test_robot")
            self.assertIsNone(state)

            self.registry.upsert_robot_state("test_robot", None, 1.0)  # Should not raise

            affected = self.registry.clear_all_positions()
            self.assertEqual(affected, 0)

            affected = self.registry.charge_all_to_full()
            self.assertEqual(affected, 0)

    def test_thread_safety(self):
        """Test that registry operations are thread-safe."""
        import threading
        import time

        # Register a robot
        self.registry.register_robot(self.test_robot_id, self.test_robot_name)

        results = []
        errors = []

        def worker_thread(thread_id):
            try:
                # Each thread performs multiple operations
                for i in range(10):
                    # Upsert state
                    position = (float(thread_id), float(i), 0.0)
                    battery = 0.5 + (i * 0.05)
                    self.registry.upsert_robot_state(self.test_robot_id, position, battery)

                    # Read state
                    state = self.registry.get_robot_state(self.test_robot_id)
                    if state:
                        results.append((thread_id, state.battery_level))

                    time.sleep(0.001)  # Small delay to encourage interleaving
            except Exception as e:
                errors.append((thread_id, str(e)))

        # Start multiple threads
        threads = []
        for i in range(3):
            t = threading.Thread(target=worker_thread, args=(i,))
            threads.append(t)
            t.start()

        # Wait for all threads
        for t in threads:
            t.join()

        # Verify no errors occurred
        self.assertEqual(len(errors), 0, f"Thread errors: {errors}")

        # Verify operations completed (some results should exist)
        self.assertGreater(len(results), 0, "No results from thread operations")

        # Final state should be valid
        final_state = self.registry.get_robot_state(self.test_robot_id)
        self.assertIsNotNone(final_state)
        self.assertIsInstance(final_state.battery_level, float)


if __name__ == '__main__':
    unittest.main()
