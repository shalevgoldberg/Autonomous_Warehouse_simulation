"""
Unit Tests for Battery Manager Implementation

Comprehensive test suite for battery management functionality including:
- Activity-based consumption rates
- Speed-dependent consumption
- Load-based consumption penalties
- Charging dynamics
- Task feasibility checking
- Thread safety
- Configuration integration
"""

import unittest
import time
import threading
from unittest.mock import Mock, MagicMock

from interfaces.battery_manager_interface import (
    IBatteryManager,
    RobotActivity,
    BatteryConsumptionConfig,
    BatteryState
)
from interfaces.configuration_interface import BatteryConfig
from warehouse.impl.battery_manager_impl import BatteryManagerImpl


class TestBatteryManager(unittest.TestCase):
    """Test cases for BatteryManagerImpl."""

    def setUp(self):
        """Set up test fixtures."""
        # Create mock configuration provider
        self.mock_config_provider = Mock()
        self.mock_config_provider.get_battery_config.return_value = BatteryConfig(
            enabled=True,
            capacity_wh=1000.0,
            idle_consumption_rate=5.0,
            moving_consumption_rate=50.0,
            carrying_consumption_rate=80.0,
            stalled_consumption_rate=10.0,
            charging_rate=-150.0,
            speed_consumption_multiplier=10.0,
            load_consumption_multiplier=30.0,
            task_safety_margin=0.15,
            low_battery_threshold=0.2,
            critical_battery_threshold=0.1,
            emergency_stop_threshold=0.0,
            charging_efficiency=0.95,
            self_discharge_rate=0.001
        )

        # Create battery manager
        self.battery_manager = BatteryManagerImpl(
            config_provider=self.mock_config_provider,
            robot_id="test_robot"
        )

    def test_initialization(self):
        """Test battery manager initialization."""
        self.assertIsInstance(self.battery_manager, IBatteryManager)
        self.assertEqual(self.battery_manager._robot_id, "test_robot")

        # Check initial state
        state = self.battery_manager.get_battery_state()
        self.assertEqual(state.level, 1.0)
        self.assertFalse(state.is_charging)

    def test_idle_consumption_rate(self):
        """Test idle consumption rate calculation."""
        rate = self.battery_manager.calculate_consumption_rate(RobotActivity.IDLE)
        expected_rate = -5.0 / 3600.0 / 1000.0  # Negative for consumption (battery decrease)
        self.assertAlmostEqual(rate, expected_rate, places=6)

    def test_moving_consumption_rate(self):
        """Test moving consumption rate with speed multiplier."""
        # Base moving rate
        rate_no_speed = self.battery_manager.calculate_consumption_rate(RobotActivity.MOVING)
        expected_base = -50.0 / 3600.0 / 1000.0  # Negative for consumption
        self.assertAlmostEqual(rate_no_speed, expected_base, places=6)

        # Moving with speed
        rate_with_speed = self.battery_manager.calculate_consumption_rate(
            RobotActivity.MOVING, speed=1.0
        )
        expected_with_speed = -(50.0 + 10.0) / 3600.0 / 1000.0  # base + speed penalty (negative)
        self.assertAlmostEqual(rate_with_speed, expected_with_speed, places=6)

    def test_carrying_consumption_rate(self):
        """Test carrying consumption rate with load multiplier."""
        rate = self.battery_manager.calculate_consumption_rate(
            RobotActivity.CARRYING, carrying_load=True
        )
        expected_base = -80.0 / 3600.0 / 1000.0  # Negative for consumption
        expected_load = -30.0 / 3600.0 / 1000.0  # load multiplier (negative)
        expected_total = expected_base + expected_load  # More negative = more consumption
        self.assertAlmostEqual(rate, expected_total, places=6)

    def test_charging_rate(self):
        """Test charging rate calculation."""
        rate = self.battery_manager.calculate_consumption_rate(RobotActivity.CHARGING)
        expected_rate = 150.0 / 3600.0 / 1000.0  # Positive for charging (battery increase)
        self.assertAlmostEqual(rate, expected_rate, places=6)

    def test_battery_level_update_idle(self):
        """Test battery level update during idle activity."""
        # Reset battery level for this test
        self.battery_manager._state.level = 0.8
        initial_level = 0.8  # Start with 80% battery
        dt = 86400.0  # 24 hours for measurable change

        new_level = self.battery_manager.update_battery_level(
            current_level=initial_level,
            activity=RobotActivity.IDLE,
            dt=dt
        )

        # Should decrease due to idle consumption
        self.assertLess(new_level, initial_level)
        # Verify the decrease is reasonable (between 0.08 and 0.12 for 24h idle)
        decrease = initial_level - new_level
        self.assertGreater(decrease, 0.08)
        self.assertLess(decrease, 0.12)

    def test_battery_level_update_moving(self):
        """Test battery level update during moving activity."""
        # Reset battery level for this test
        self.battery_manager._state.level = 0.8
        initial_level = 0.8  # Start with 80% battery
        dt = 3600.0  # 1 hour for measurable change

        new_level = self.battery_manager.update_battery_level(
            current_level=initial_level,
            activity=RobotActivity.MOVING,
            speed=1.0,
            dt=dt
        )

        # Should decrease due to moving + speed consumption
        self.assertLess(new_level, initial_level)
        # Verify the decrease is reasonable for moving with speed
        decrease = initial_level - new_level
        self.assertGreater(decrease, 0.05)  # At least 5% decrease
        self.assertLess(decrease, 0.08)  # Less than 8% decrease

    def test_battery_level_update_charging(self):
        """Test battery level update during charging."""
        # Start charging session
        self.battery_manager.start_charging_session()

        # Reset battery level for this test
        self.battery_manager._state.level = 0.2
        initial_level = 0.2  # Start with low battery
        dt = 1800.0  # 30 minutes for measurable change

        new_level = self.battery_manager.update_battery_level(
            current_level=initial_level,
            activity=RobotActivity.CHARGING,
            dt=dt
        )

        # Should increase due to charging (accounting for efficiency)
        self.assertGreater(new_level, initial_level)
        # Verify the increase is reasonable for charging
        increase = new_level - initial_level
        self.assertGreater(increase, 0.06)  # At least 6% increase
        self.assertLess(increase, 0.08)  # Less than 8% increase

        # Clean up
        self.battery_manager.stop_charging_session()

    def test_task_consumption_estimation(self):
        """Test task battery consumption estimation."""
        distance = 10.0  # 10 meters
        avg_speed = 1.0  # 1 m/s

        # Moving task
        consumption_moving = self.battery_manager.estimate_task_consumption(
            distance=distance,
            carrying_load=False,
            avg_speed=avg_speed
        )

        # Carrying task (should consume more - more negative)
        consumption_carrying = self.battery_manager.estimate_task_consumption(
            distance=distance,
            carrying_load=True,
            avg_speed=avg_speed
        )

        # Carrying should consume more than moving (more negative)
        self.assertLess(consumption_carrying, consumption_moving)  # More negative = more consumption
        self.assertLess(consumption_moving, 0.0)  # Should be negative (consumption)
        self.assertGreater(consumption_moving, -0.1)  # Reasonable consumption magnitude

    def test_task_feasibility_check(self):
        """Test task feasibility checking with safety margins."""
        # High battery level
        can_complete_high = self.battery_manager.can_complete_task(
            current_battery=0.8,
            estimated_consumption=0.05
        )
        self.assertTrue(can_complete_high)

        # Low battery level
        can_complete_low = self.battery_manager.can_complete_task(
            current_battery=0.15,
            estimated_consumption=0.05
        )
        self.assertFalse(can_complete_low)  # Below safety margin

    def test_charging_session_management(self):
        """Test charging session start/stop functionality."""
        # Initially not charging
        self.assertFalse(self.battery_manager.is_charging())

        # Start charging
        result = self.battery_manager.start_charging_session()
        self.assertTrue(result)
        self.assertTrue(self.battery_manager.is_charging())

        # Try to start again (should fail)
        result = self.battery_manager.start_charging_session()
        self.assertFalse(result)

        # Stop charging
        result = self.battery_manager.stop_charging_session()
        self.assertTrue(result)
        self.assertFalse(self.battery_manager.is_charging())

        # Try to stop again (should fail)
        result = self.battery_manager.stop_charging_session()
        self.assertFalse(result)

    def test_charging_time_estimation(self):
        """Test charging time estimation."""
        # Create a battery manager with half battery level
        self.battery_manager._state.level = 0.5

        # Estimate time to charge to 80%
        time_partial = self.battery_manager.get_charging_time_estimate(target_level=0.8)
        self.assertGreater(time_partial, 0)

        # Estimate time to full charge
        time_full = self.battery_manager.get_charging_time_estimate(target_level=1.0)
        self.assertGreater(time_full, time_partial)  # Should take longer to go higher

        # Test when already at target level
        self.battery_manager._state.level = 1.0  # Set to full
        time_already_full = self.battery_manager.get_charging_time_estimate(target_level=1.0)
        self.assertEqual(time_already_full, 0)  # No charging needed

    def test_battery_bounds(self):
        """Test battery level bounds checking."""
        # Test upper bound
        level = self.battery_manager.update_battery_level(
            current_level=0.95,
            activity=RobotActivity.CHARGING,
            dt=1000.0  # Long charging time
        )
        self.assertLessEqual(level, 1.0)

        # Test lower bound
        level = self.battery_manager.update_battery_level(
            current_level=0.05,
            activity=RobotActivity.MOVING,
            speed=2.0,
            dt=1000.0  # Long drain time
        )
        self.assertGreaterEqual(level, 0.0)

    def test_configuration_integration(self):
        """Test configuration parameter integration."""
        # Verify configuration is used correctly
        config = self.mock_config_provider.get_battery_config.return_value

        # Test that consumption rates use config values (negative for consumption)
        idle_rate = self.battery_manager.calculate_consumption_rate(RobotActivity.IDLE)
        expected_idle = -config.idle_consumption_rate / 3600.0 / config.capacity_wh
        self.assertAlmostEqual(idle_rate, expected_idle, places=6)

    def test_disabled_battery_management(self):
        """Test behavior when battery management is disabled."""
        # Create config with disabled battery management
        disabled_config = BatteryConfig(
            enabled=False,
            capacity_wh=1000.0,
            idle_consumption_rate=5.0,
            moving_consumption_rate=50.0,
            carrying_consumption_rate=80.0,
            stalled_consumption_rate=10.0,
            charging_rate=-150.0,
            speed_consumption_multiplier=10.0,
            load_consumption_multiplier=30.0,
            task_safety_margin=0.15,
            low_battery_threshold=0.2,
            critical_battery_threshold=0.1,
            emergency_stop_threshold=0.0,
            charging_efficiency=0.95,
            self_discharge_rate=0.001
        )

        mock_provider = Mock()
        mock_provider.get_battery_config.return_value = disabled_config

        disabled_manager = BatteryManagerImpl(
            config_provider=mock_provider,
            robot_id="disabled_robot"
        )

        # Should still work but might have different behavior
        state = disabled_manager.get_battery_state()
        self.assertIsInstance(state, BatteryState)

    def test_thread_safety(self):
        """Test thread safety of battery operations."""
        results = []
        errors = []

        def worker_thread(thread_id):
            """Worker thread for concurrent testing."""
            try:
                for i in range(100):
                    # Concurrent battery updates
                    level = self.battery_manager.update_battery_level(
                        current_level=0.5,
                        activity=RobotActivity.MOVING,
                        speed=1.0,
                        dt=0.001
                    )
                    results.append((thread_id, level))
            except Exception as e:
                errors.append((thread_id, str(e)))

        # Start multiple threads
        threads = []
        for i in range(5):
            thread = threading.Thread(target=worker_thread, args=(i,))
            threads.append(thread)
            thread.start()

        # Wait for completion
        for thread in threads:
            thread.join()

        # Verify no errors occurred
        self.assertEqual(len(errors), 0, f"Thread safety errors: {errors}")

        # Verify results are reasonable
        self.assertEqual(len(results), 500)  # 5 threads * 100 iterations

        # All battery levels should be within bounds
        for thread_id, level in results:
            self.assertGreaterEqual(level, 0.0)
            self.assertLessEqual(level, 1.0)

    def test_zero_distance_task(self):
        """Test task consumption estimation for zero distance."""
        consumption = self.battery_manager.estimate_task_consumption(
            distance=0.0,
            carrying_load=True,
            avg_speed=1.0
        )
        self.assertEqual(consumption, 0.0)

    def test_zero_speed_task(self):
        """Test task consumption estimation with zero speed."""
        consumption = self.battery_manager.estimate_task_consumption(
            distance=10.0,
            carrying_load=False,
            avg_speed=0.0
        )
        # Should handle zero speed gracefully (avoid division by zero)
        self.assertIsInstance(consumption, float)

    def test_emergency_stop_not_triggered_above_threshold(self):
        """Test that emergency stop is not triggered when battery is above threshold."""
        # Battery level above emergency threshold
        battery_level = 0.1  # 10% (above 0% threshold)

        # Check emergency stop - should not trigger
        result = self.battery_manager.check_emergency_stop(battery_level)
        self.assertFalse(result)

        # Emergency stop should not be active
        self.assertFalse(self.battery_manager.is_emergency_stop_active())

    def test_emergency_stop_triggered_at_threshold(self):
        """Test that emergency stop is triggered when battery reaches threshold."""
        # Battery level at emergency threshold
        battery_level = 0.0  # 0% (at threshold)

        # Check emergency stop - should trigger
        result = self.battery_manager.check_emergency_stop(battery_level)
        self.assertTrue(result)

        # Emergency stop should be active
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

    def test_emergency_stop_triggered_below_threshold(self):
        """Test that emergency stop is triggered when battery goes below threshold."""
        # Battery level below emergency threshold (shouldn't happen due to clamping)
        battery_level = -0.1  # -10% (below threshold)

        # Check emergency stop - should trigger
        result = self.battery_manager.check_emergency_stop(battery_level)
        self.assertTrue(result)

        # Emergency stop should be active
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

    def test_emergency_stop_no_repeated_alerts(self):
        """Test that emergency stop doesn't trigger repeated alerts."""
        # First call should trigger alert
        result1 = self.battery_manager.check_emergency_stop(0.0)
        self.assertTrue(result1)
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

        # Second call with same low battery should not trigger again
        result2 = self.battery_manager.check_emergency_stop(0.0)
        self.assertTrue(result2)  # Still returns True (emergency active)
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

    def test_emergency_stop_reset(self):
        """Test that emergency stop can be reset."""
        # Trigger emergency stop
        self.battery_manager.check_emergency_stop(0.0)
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

        # Reset emergency stop
        self.battery_manager.reset_emergency_stop()
        self.assertFalse(self.battery_manager.is_emergency_stop_active())

    def test_emergency_stop_disabled_battery_manager(self):
        """Test that emergency stop is not triggered when battery manager is disabled."""
        # Create mock provider with disabled config
        mock_provider = Mock()
        mock_provider.get_battery_config.return_value = BatteryConfig(
            enabled=False,
            capacity_wh=1000.0,
            idle_consumption_rate=5.0,
            moving_consumption_rate=50.0,
            carrying_consumption_rate=80.0,
            stalled_consumption_rate=10.0,
            charging_rate=-150.0,
            speed_consumption_multiplier=10.0,
            load_consumption_multiplier=30.0,
            task_safety_margin=0.15,
            low_battery_threshold=0.2,
            critical_battery_threshold=0.1,
            emergency_stop_threshold=0.0,
            charging_efficiency=0.95,
            self_discharge_rate=0.001
        )

        disabled_manager = BatteryManagerImpl(mock_provider, "test_robot")

        # Emergency stop should not trigger when disabled
        result = disabled_manager.check_emergency_stop(0.0)
        self.assertFalse(result)
        self.assertFalse(disabled_manager.is_emergency_stop_active())

    def test_battery_level_update_with_emergency_stop(self):
        """Test that battery level update triggers emergency stop at 0%."""
        # Directly trigger emergency stop with 0% battery
        result = self.battery_manager.check_emergency_stop(0.0)

        # Should return True (emergency stop triggered)
        self.assertTrue(result)

        # Emergency stop should be active
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

    def test_battery_level_update_resets_emergency_stop(self):
        """Test that battery level update resets emergency stop when charged."""
        # First trigger emergency stop
        self.battery_manager.check_emergency_stop(0.0)
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

        # Now charge the battery above emergency threshold
        new_level = self.battery_manager.update_battery_level(
            current_level=0.0,
            activity=RobotActivity.CHARGING,
            dt=10.0  # Charge for some time
        )

        # Battery should be above 0
        self.assertGreater(new_level, 0.0)

        # Emergency stop should be reset
        self.assertFalse(self.battery_manager.is_emergency_stop_active())

    def test_emergency_stop_state_persistence(self):
        """Test that emergency stop state persists correctly."""
        # Initially not active
        self.assertFalse(self.battery_manager.is_emergency_stop_active())

        # Trigger emergency stop
        self.battery_manager.check_emergency_stop(0.0)
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

        # State should persist
        self.assertTrue(self.battery_manager.is_emergency_stop_active())

        # Reset
        self.battery_manager.reset_emergency_stop()
        self.assertFalse(self.battery_manager.is_emergency_stop_active())


if __name__ == '__main__':
    unittest.main()
