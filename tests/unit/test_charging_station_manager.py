"""
Unit Tests for Charging Station Manager Implementation

Comprehensive test suite for charging station management functionality including:
- Station discovery and registration
- Proximity-based allocation
- Lock management integration
- Maintenance mode support
- Thread safety
- Configuration integration
"""

import unittest
import time
import threading
from unittest.mock import Mock, MagicMock, patch

from interfaces.charging_station_manager_interface import (
    IChargingStationManager,
    ChargingStationInfo,
    ChargingStationRequest,
    ChargingStationAssignment,
    ChargingStationStatus,
    ChargingStationError
)
from interfaces.configuration_interface import ChargingStationConfig
from warehouse.impl.charging_station_manager_impl import ChargingStationManagerImpl
from warehouse.map import WarehouseMap


class TestChargingStationManager(unittest.TestCase):
    """Test cases for ChargingStationManagerImpl."""

    def setUp(self):
        """Set up test fixtures."""
        # Create mock configuration provider
        self.mock_config_provider = Mock()
        self.mock_config_provider.get_charging_station_config.return_value = ChargingStationConfig(
            enabled=True,
            auto_discover_stations=True,
            station_lock_timeout=600,
            station_heartbeat_interval=30,
            station_power_watts=150.0,
            station_efficiency=0.95,
            max_search_distance=50.0,
            station_selection_timeout=10.0,
            enable_maintenance_mode=True,
            maintenance_check_interval=300,
            station_health_timeout=60,
            max_stations_per_robot=1,
            station_cache_ttl=60,
            concurrent_allocation_limit=10,
            enable_smart_allocation=False,
            enable_station_priorities=False,
            enable_load_balancing=False
        )

        # Create mock simulation data service
        self.mock_simulation_data_service = Mock()
        self.mock_simulation_data_service.try_acquire_bay_lock.return_value = True
        self.mock_simulation_data_service.release_bay_lock.return_value = True
        self.mock_simulation_data_service.heartbeat_bay_lock.return_value = True

        # Create mock warehouse map
        self.mock_warehouse_map = Mock()
        # Mock charging zones at positions (10, 10) and (20, 20)
        self.mock_warehouse_map._get_charging_zones.return_value = [
            (10.0, 10.0),
            (20.0, 20.0)
        ]
        # Mock world_to_grid method for grid coordinate conversion
        self.mock_warehouse_map.world_to_grid.side_effect = lambda x, y: (int(x / 0.5), int(y / 0.5))

        # Create charging station manager
        self.station_manager = ChargingStationManagerImpl(
            config_provider=self.mock_config_provider,
            simulation_data_service=self.mock_simulation_data_service,
            warehouse_map=self.mock_warehouse_map
        )

    def test_initialization(self):
        """Test charging station manager initialization."""
        self.assertIsInstance(self.station_manager, IChargingStationManager)
        self.assertEqual(len(self.station_manager._stations), 2)  # Two stations discovered

        # Check that stations were created with correct properties
        station_ids = list(self.station_manager._stations.keys())
        self.assertIn("charge_20_20", station_ids)  # (10.0, 10.0) -> grid (20, 20)
        self.assertIn("charge_40_40", station_ids)  # (20.0, 20.0) -> grid (40, 40)

        # Verify all stations have same power and efficiency (simplified)
        for station in self.station_manager._stations.values():
            self.assertEqual(station.max_power_watts, 150.0)
            self.assertEqual(station.efficiency, 0.95)

    def test_station_discovery(self):
        """Test automatic station discovery from warehouse map."""
        # Verify stations were discovered correctly
        station_1 = self.station_manager._stations["charge_20_20"]
        station_2 = self.station_manager._stations["charge_40_40"]

        self.assertEqual(station_1.position, (10.0, 10.0))
        self.assertEqual(station_2.position, (20.0, 20.0))
        self.assertEqual(station_1.status, ChargingStationStatus.AVAILABLE)

    def test_request_charging_station_success(self):
        """Test successful charging station request and allocation."""
        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(10.0, 10.0),
            timestamp=time.time()
        )

        assignment = self.station_manager.request_charging_station(request)

        self.assertIsNotNone(assignment)
        self.assertIsInstance(assignment, ChargingStationAssignment)
        self.assertEqual(assignment.station.station_id, "charge_20_20")  # Closest station
        self.assertAlmostEqual(assignment.distance, 0.0, places=1)  # At same position
        self.assertEqual(assignment.station.position, (10.0, 10.0))

    def test_request_charging_station_no_available(self):
        """Test charging station request when no stations are available."""
        # Mock database to return occupied status for all stations
        def mock_get_status(station_id):
            if station_id in ["charge_20_20", "charge_40_40"]:
                return ChargingStationInfo(
                    station_id=station_id,
                    position=(10.0, 10.0) if station_id == "charge_20_20" else (20.0, 20.0),
                    status=ChargingStationStatus.OCCUPIED,
                    max_power_watts=150.0,
                    efficiency=0.95
                )
            return None

        # Mock the get_charging_station_status method
        original_method = self.station_manager.get_charging_station_status
        self.station_manager.get_charging_station_status = mock_get_status

        try:
            request = ChargingStationRequest(
                robot_id="robot_1",
                robot_position=(10.0, 10.0),
                timestamp=time.time()
            )

            assignment = self.station_manager.request_charging_station(request)
            self.assertIsNone(assignment)
        finally:
            # Restore original method
            self.station_manager.get_charging_station_status = original_method

    def test_request_charging_station_lock_failure(self):
        """Test charging station request when lock acquisition fails."""
        # Mock lock acquisition failure
        self.mock_simulation_data_service.try_acquire_bay_lock.return_value = False

        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(10.0, 10.0),
            timestamp=time.time()
        )

        assignment = self.station_manager.request_charging_station(request)
        self.assertIsNone(assignment)

    def test_release_charging_station(self):
        """Test releasing a charging station."""
        # First allocate a station
        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(10.0, 10.0),
            timestamp=time.time()
        )
        assignment = self.station_manager.request_charging_station(request)
        self.assertIsNotNone(assignment)

        # Now release it
        result = self.station_manager.release_charging_station(
            station_id="charge_20_20",
            robot_id="robot_1"
        )
        self.assertTrue(result)

        # Verify the lock release was called
        self.mock_simulation_data_service.release_bay_lock.assert_called_once_with(
            "charge_20_20", "robot_1"
        )

    def test_heartbeat_charging_station(self):
        """Test heartbeating a charging station assignment."""
        # First allocate a station
        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(10.0, 10.0),
            timestamp=time.time()
        )
        assignment = self.station_manager.request_charging_station(request)
        self.assertIsNotNone(assignment)

        # Heartbeat the station
        result = self.station_manager.heartbeat_charging_station(
            station_id="charge_20_20",
            robot_id="robot_1"
        )
        self.assertTrue(result)

        # Verify the heartbeat was called
        self.mock_simulation_data_service.heartbeat_bay_lock.assert_called_once_with(
            "charge_20_20", "robot_1"
        )

    def test_get_charging_station_status(self):
        """Test getting charging station status."""
        # Test getting status of existing station
        status = self.station_manager.get_charging_station_status("charge_20_20")
        self.assertIsNotNone(status)
        self.assertEqual(status.station_id, "charge_20_20")
        self.assertEqual(status.position, (10.0, 10.0))
        self.assertEqual(status.status, ChargingStationStatus.AVAILABLE)

        # Test getting status of non-existent station
        status = self.station_manager.get_charging_station_status("non_existent")
        self.assertIsNone(status)

    def test_list_available_stations(self):
        """Test listing available stations within constraints."""
        # All stations should be available initially
        available = self.station_manager.list_available_stations(robot_position=(10.0, 10.0))
        self.assertEqual(len(available), 2)

        # Test with distance constraint
        available = self.station_manager.list_available_stations(
            robot_position=(10.0, 10.0),
            max_distance=5.0  # Only very close stations
        )
        self.assertEqual(len(available), 1)  # Only the closest station (at same position)

    def test_get_nearest_available_station(self):
        """Test finding the nearest available station."""
        # From position (10, 10), nearest should be charge_20_20 (at same position)
        nearest = self.station_manager.get_nearest_available_station(robot_position=(10.0, 10.0))
        self.assertIsNotNone(nearest)
        self.assertEqual(nearest.station_id, "charge_20_20")

        # From position (15, 15), both stations are equally distant (~7.07 units)
        # The algorithm returns the first one found, which is charge_20_20
        nearest = self.station_manager.get_nearest_available_station(robot_position=(15.0, 15.0))
        self.assertIsNotNone(nearest)
        self.assertEqual(nearest.station_id, "charge_20_20")  # First station found at equal distance

    def test_get_station_statistics(self):
        """Test getting station statistics."""
        stats = self.station_manager.get_station_statistics()

        self.assertEqual(stats['total_stations'], 2)
        self.assertEqual(stats['available_stations'], 2)
        self.assertEqual(stats['occupied_stations'], 0)
        self.assertEqual(stats['maintenance_stations'], 0)

        # Test after occupying a station
        self.station_manager._stations["charge_20_20"].status = ChargingStationStatus.OCCUPIED
        stats = self.station_manager.get_station_statistics()

        self.assertEqual(stats['available_stations'], 1)
        self.assertEqual(stats['occupied_stations'], 1)

    def test_set_station_maintenance(self):
        """Test setting maintenance mode for stations."""
        # Set maintenance mode
        result = self.station_manager.set_station_maintenance("charge_20_20", True)
        self.assertTrue(result)

        # Verify status changed in internal storage
        station = self.station_manager._stations["charge_20_20"]
        self.assertEqual(station.status, ChargingStationStatus.MAINTENANCE)

        # Clear maintenance mode
        result = self.station_manager.set_station_maintenance("charge_20_20", False)
        self.assertTrue(result)

        # Verify status changed back
        station = self.station_manager._stations["charge_20_20"]
        self.assertEqual(station.status, ChargingStationStatus.AVAILABLE)

    def test_estimate_charging_time(self):
        """Test charging time estimation."""
        # Test with 50% battery
        charging_time = self.station_manager.estimate_charging_time(
            station_id="charge_10_10",
            current_battery=0.5,
            target_battery=1.0
        )

        # Should be a positive time estimate
        self.assertGreater(charging_time, 0)

        # Test with already full battery
        charging_time = self.station_manager.estimate_charging_time(
            station_id="charge_10_10",
            current_battery=1.0,
            target_battery=1.0
        )
        self.assertEqual(charging_time, 0)

        # Test with invalid station
        with self.assertRaises(ChargingStationError):
            self.station_manager.estimate_charging_time(
                station_id="invalid_station",
                current_battery=0.5
            )

    def test_configuration_disabled(self):
        """Test behavior when charging station management is disabled."""
        # Create config with disabled charging stations
        disabled_config = ChargingStationConfig(
            enabled=False,
            auto_discover_stations=False,
            station_lock_timeout=600,
            station_heartbeat_interval=30,
            station_power_watts=150.0,
            station_efficiency=0.95,
            max_search_distance=50.0,
            station_selection_timeout=10.0,
            enable_maintenance_mode=True,
            maintenance_check_interval=300,
            station_health_timeout=60,
            max_stations_per_robot=1,
            station_cache_ttl=60,
            concurrent_allocation_limit=10,
            enable_smart_allocation=False,
            enable_station_priorities=False,
            enable_load_balancing=False
        )

        mock_provider = Mock()
        mock_provider.get_charging_station_config.return_value = disabled_config

        disabled_manager = ChargingStationManagerImpl(
            config_provider=mock_provider,
            simulation_data_service=self.mock_simulation_data_service,
            warehouse_map=self.mock_warehouse_map
        )

        # Should return None for requests when disabled
        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(10.0, 10.0),
            timestamp=time.time()
        )

        assignment = disabled_manager.request_charging_station(request)
        self.assertIsNone(assignment)

    def test_thread_safety(self):
        """Test thread safety of charging station operations."""
        results = []
        errors = []

        def station_worker(thread_id):
            """Worker that performs station operations."""
            try:
                for i in range(10):
                    # Try to allocate a station
                    request = ChargingStationRequest(
                        robot_id=f"robot_{thread_id}_{i}",
                        robot_position=(10.0, 10.0),
                        timestamp=time.time()
                    )

                    assignment = self.station_manager.request_charging_station(request)
                    results.append((thread_id, assignment is not None))

                    # If allocated, release it
                    if assignment:
                        self.station_manager.release_charging_station(
                            assignment.station.station_id,
                            request.robot_id
                        )

            except Exception as e:
                errors.append((thread_id, str(e)))

        # Start multiple threads
        threads = []
        for i in range(3):
            thread = threading.Thread(target=station_worker, args=(i,))
            threads.append(thread)
            thread.start()

        # Wait for completion
        for thread in threads:
            thread.join()

        # Verify no errors occurred
        self.assertEqual(len(errors), 0, f"Thread safety errors: {errors}")

        # Verify reasonable results
        self.assertEqual(len(results), 30)  # 3 threads * 10 iterations

    def test_proximity_based_selection(self):
        """Test that stations are selected based on proximity."""
        # Request from position far from both stations
        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(0.0, 0.0),  # Far from both stations
            timestamp=time.time()
        )

        assignment = self.station_manager.request_charging_station(request)

        # Should allocate the closest station (charge_10_10)
        self.assertIsNotNone(assignment)
        self.assertEqual(assignment.station.station_id, "charge_10_10")

        # Distance should be sqrt((10-0)^2 + (10-0)^2) = 14.14
        expected_distance = (10**2 + 10**2)**0.5
        self.assertAlmostEqual(assignment.distance, expected_distance, places=1)

    def test_simple_distance_allocation(self):
        """Test that stations are allocated based on distance (simplified approach)."""
        # All stations are identical, so should get nearest one
        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(5.0, 5.0),  # Closer to (10,10) than (20,20)
            timestamp=time.time()
        )

        assignment = self.station_manager.request_charging_station(request)

        # Should allocate the closer station
        self.assertIsNotNone(assignment)
        self.assertEqual(assignment.station.station_id, "charge_10_10")
        # Distance should be sqrt((10-5)^2 + (10-5)^2) = sqrt(50) ≈ 7.07
        self.assertAlmostEqual(assignment.distance, 7.07, places=1)

    def test_distance_constraints(self):
        """Test that distance constraints are respected."""
        # Request from very far position
        request = ChargingStationRequest(
            robot_id="robot_1",
            robot_position=(100.0, 100.0),  # Very far from stations
            timestamp=time.time()
        )

        assignment = self.station_manager.request_charging_station(request)

        # Should not allocate any station (too far)
        self.assertIsNone(assignment)


if __name__ == '__main__':
    unittest.main()
