"""
Battery Manager Implementation - Professional battery consumption and charging management.

This implementation provides sophisticated battery management with activity-based consumption,
charging dynamics, and safety features following SOLID principles and enterprise-grade architecture.

Design Principles:
- **Single Responsibility**: Handles only battery-related calculations and state management
- **Open/Closed**: Extensible consumption models without modifying core logic
- **Liskov Substitution**: Fully implements IBatteryManager interface
- **Interface Segregation**: Focused battery management interface
- **Dependency Inversion**: Depends on abstractions, not concretions
"""

import time
import threading
import logging
from typing import Optional
from dataclasses import dataclass, replace

from interfaces.battery_manager_interface import (
    IBatteryManager,
    RobotActivity,
    BatteryConsumptionConfig,
    BatteryState
)
from interfaces.configuration_interface import BatteryConfig, IBusinessConfigurationProvider


@dataclass
class _InternalBatteryState:
    """Internal battery state for thread-safe management."""
    level: float  # Current battery level (0.0 to 1.0)
    is_charging: bool  # Whether currently charging
    consumption_rate: float  # Current consumption rate (per second)
    last_update: float  # Last update timestamp
    charging_start_time: Optional[float]  # When charging started
    emergency_stop_active: bool  # Whether emergency stop is active due to battery depletion
    _lock: threading.RLock  # Thread synchronization


class BatteryManagerImpl(IBatteryManager):
    """
    Professional battery management implementation with activity-based consumption.

    Features:
    - Activity-based consumption rates (idle, moving, carrying, charging)
    - Speed-dependent consumption for moving activities
    - Load-based consumption penalties
    - Charging efficiency modeling
    - Self-discharge simulation
    - Thread-safe operations
    - Comprehensive safety margins

    Threading Model:
    - All public methods are thread-safe
    - Internal state protected by RLock
    - Suitable for physics thread (1kHz) and control threads
    """

    def __init__(self, config_provider: IBusinessConfigurationProvider, robot_id: str = "default"):
        """
        Initialize battery manager with configuration.

        Args:
            config_provider: Configuration provider for battery parameters
            robot_id: Robot identifier for logging
        """
        self._robot_id = robot_id
        self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}[{robot_id}]")

        # Load configuration
        self._config = config_provider.get_battery_config()

        # Pre-compute derived values for performance (needed for rate conversion)
        self._wh_to_level_factor = 1.0 / self._config.capacity_wh  # Convert Wh to battery level
        self._level_to_wh_factor = self._config.capacity_wh  # Convert battery level to Wh

        # Create consumption configuration from battery config
        # Note: Consumption rates are negative (decrease battery), charging rates are positive (increase battery)
        self._consumption_config = BatteryConsumptionConfig(
            idle_rate=-self._convert_watts_to_rate(self._config.idle_consumption_rate),  # Negative for consumption
            moving_rate=-self._convert_watts_to_rate(self._config.moving_consumption_rate),  # Negative for consumption
            carrying_rate=-self._convert_watts_to_rate(self._config.carrying_consumption_rate),  # Negative for consumption
            stalled_rate=-self._convert_watts_to_rate(self._config.stalled_consumption_rate),  # Negative for consumption
            charging_rate=-self._convert_watts_to_rate(self._config.charging_rate),  # Config is negative, so double negative = positive
            speed_multiplier=-self._convert_watts_to_rate(self._config.speed_consumption_multiplier),  # Negative for consumption
            load_multiplier=-self._convert_watts_to_rate(self._config.load_consumption_multiplier)  # Negative for consumption
        )

        # Initialize internal state
        self._state = _InternalBatteryState(
            level=1.0,  # Start fully charged
            is_charging=False,
            consumption_rate=0.0,
            last_update=time.time(),
            charging_start_time=None,
            emergency_stop_active=False,  # Start with no emergency stop
            _lock=threading.RLock()
        )

        self._logger.info(f"BatteryManager initialized for robot {robot_id} with {self._config.capacity_wh}Wh capacity")

    def get_battery_state(self) -> BatteryState:
        """Get current battery state information."""
        with self._state._lock:
            current_time = time.time()

            # Update consumption rate based on current activity
            consumption_rate = self._calculate_current_consumption_rate()

            return BatteryState(
                level=self._state.level,
                is_charging=self._state.is_charging,
                consumption_rate=consumption_rate,
                last_update=current_time
            )

    def update_battery_level(self, current_level: float, activity: RobotActivity,
                           speed: float = 0.0, carrying_load: bool = False,
                           dt: float = 0.001) -> float:
        """
        Update battery level based on activity and time delta.

        This is the core battery simulation method called at high frequency (1kHz)
        from the physics thread.
        """
        if not self._config.enabled:
            return current_level  # Return unchanged if disabled

        with self._state._lock:
            current_time = time.time()

            # Calculate consumption rate for this activity
            consumption_rate = self.calculate_consumption_rate(activity, speed, carrying_load)

            # Update internal state
            self._state.consumption_rate = consumption_rate
            self._state.last_update = current_time

            # Calculate battery level change
            if self._state.is_charging:
                # Apply charging efficiency
                effective_rate = consumption_rate * self._config.charging_efficiency
            else:
                effective_rate = consumption_rate

            # During emergency stop, stop all battery consumption including self-discharge
            if not self._state.emergency_stop_active:
                # Apply self-discharge (very small effect)
                self_discharge_rate = self._config.self_discharge_rate / 3600.0  # Convert to per-second
                effective_rate += self_discharge_rate
            else:
                # Emergency stop active - minimal consumption only
                # Keep a tiny positive rate to allow for emergency systems
                effective_rate = 0.000001  # Very small positive rate (charging-like)

            # Calculate level change
            level_change = effective_rate * dt

            # Update battery level with bounds checking
            new_level = max(0.0, min(1.0, current_level + level_change))

            # Handle emergency stop logic
            if self._state.emergency_stop_active:
                # Emergency stop is active - prevent battery from dropping below threshold
                # But allow slow recovery if charging
                if new_level > self._config.emergency_stop_threshold + 0.01:  # Small buffer
                    # Battery has recovered sufficiently - reset emergency stop
                    self.reset_emergency_stop()
                elif new_level < self._config.emergency_stop_threshold:
                    # Keep battery at threshold level during emergency stop
                    new_level = self._config.emergency_stop_threshold
            else:
                # Normal operation - check for emergency stop trigger
                if new_level <= self._config.emergency_stop_threshold:
                    # Battery depleted - trigger emergency stop
                    self.check_emergency_stop(new_level)

            # Update internal state
            self._state.level = new_level

            # Log significant changes
            if abs(level_change) > 0.001:  # Log changes > 0.1%
                self._logger.debug(".3f"
                                 f"speed={speed:.2f}, load={carrying_load}, dt={dt:.6f}")

            return new_level

    def calculate_consumption_rate(self, activity: RobotActivity,
                                 speed: float = 0.0,
                                 carrying_load: bool = False) -> float:
        """
        Calculate battery consumption rate for given conditions.

        This is a pure function for consumption rate calculation.
        """
        # Base rate from activity
        if activity == RobotActivity.IDLE:
            base_rate = self._consumption_config.idle_rate
        elif activity == RobotActivity.MOVING:
            base_rate = self._consumption_config.moving_rate
        elif activity == RobotActivity.CARRYING:
            base_rate = self._consumption_config.carrying_rate
        elif activity == RobotActivity.CHARGING:
            base_rate = self._consumption_config.charging_rate
        elif activity == RobotActivity.STALLED:
            base_rate = self._consumption_config.stalled_rate
        else:
            base_rate = self._consumption_config.idle_rate  # Default to idle

        # Apply speed multiplier for moving activities
        if activity in [RobotActivity.MOVING, RobotActivity.CARRYING] and speed > 0:
            speed_penalty = speed * self._consumption_config.speed_multiplier
            base_rate += speed_penalty

        # Apply load multiplier for carrying
        if carrying_load:
            load_penalty = self._consumption_config.load_multiplier
            base_rate += load_penalty

        return base_rate

    def estimate_task_consumption(self, distance: float,
                                carrying_load: bool = False,
                                avg_speed: float = 1.0) -> float:
        """
        Estimate battery consumption for a task.

        Args:
            distance: Distance to travel (meters)
            carrying_load: Whether load will be carried
            avg_speed: Average speed during task (m/s)

        Returns:
            float: Estimated battery consumption (0.0 to 1.0)
        """
        if distance <= 0:
            return 0.0

        # Estimate travel time
        travel_time = distance / max(avg_speed, 0.1)  # Avoid division by zero

        # Use appropriate activity
        activity = RobotActivity.CARRYING if carrying_load else RobotActivity.MOVING

        # Calculate average consumption rate
        avg_rate = self.calculate_consumption_rate(activity, avg_speed, carrying_load)

        # Calculate total energy consumption in watt-hours
        total_wh = avg_rate * travel_time * 3600  # Convert to watt-hours

        # Convert to battery level consumption
        level_consumption = total_wh * self._wh_to_level_factor

        self._logger.debug(".1f"
                         f"load={carrying_load}, consumption={level_consumption:.4f}")

        return level_consumption

    def can_complete_task(self, current_battery: float,
                         estimated_consumption: float,
                         safety_margin: Optional[float] = None) -> bool:
        """
        Check if robot can complete a task with current battery.

        Args:
            current_battery: Current battery level (0.0 to 1.0)
            estimated_consumption: Estimated consumption for task
            safety_margin: Override default safety margin

        Returns:
            bool: True if task can be completed safely
        """
        if safety_margin is None:
            safety_margin = self._config.task_safety_margin

        required_level = estimated_consumption + safety_margin
        can_complete = current_battery >= required_level

        if not can_complete:
            self._logger.warning(".3f"
                               ".3f")

        return can_complete

    def get_charging_time_estimate(self, target_level: float = 1.0) -> float:
        """
        Estimate time required to charge to target level.

        Args:
            target_level: Target battery level (0.0 to 1.0)

        Returns:
            float: Estimated charging time (seconds)
        """
        with self._state._lock:
            current_level = self._state.level

        if current_level >= target_level:
            return 0.0  # Already at or above target

        # Calculate energy needed
        energy_needed_wh = (target_level - current_level) * self._level_to_wh_factor

        # Account for charging efficiency
        effective_energy_needed = energy_needed_wh / self._config.charging_efficiency

        # Calculate time (charging rate is negative, so we use absolute value)
        charging_power = abs(self._config.charging_rate)  # Convert to positive watts
        if charging_power <= 0:
            return float('inf')  # Cannot charge

        charging_time_seconds = effective_energy_needed / charging_power

        return charging_time_seconds

    def start_charging_session(self) -> bool:
        """Start a charging session."""
        with self._state._lock:
            if self._state.is_charging:
                self._logger.warning("Charging session already active")
                return False

            self._state.is_charging = True
            self._state.charging_start_time = time.time()

            self._logger.info("Charging session started")
            return True

    def stop_charging_session(self) -> bool:
        """Stop current charging session."""
        with self._state._lock:
            if not self._state.is_charging:
                self._logger.warning("No active charging session to stop")
                return False

            charging_duration = time.time() - (self._state.charging_start_time or time.time())
            self._state.is_charging = False
            self._state.charging_start_time = None

            self._logger.info(".1f")
            return True

    def is_charging(self) -> bool:
        """Check if robot is currently charging."""
        with self._state._lock:
            return self._state.is_charging

    # Private helper methods

    def _calculate_current_consumption_rate(self) -> float:
        """Calculate current consumption rate based on internal state."""
        # This is a placeholder - in a real implementation, this would track
        # the robot's current activity from motion executor or task handler
        return self._consumption_config.idle_rate  # Default to idle

    def _convert_watts_to_rate(self, watts: float) -> float:
        """
        Convert watts to battery level consumption rate per second.

        Args:
            watts: Power consumption in watts

        Returns:
            float: Battery level consumption rate per second
        """
        # Convert watts to watt-hours per second, then to battery level per second
        wh_per_second = watts / 3600.0  # Convert watts to Wh/s
        level_per_second = wh_per_second * self._wh_to_level_factor
        return level_per_second

    def _convert_rate_to_watts(self, rate: float) -> float:
        """
        Convert battery consumption rate back to watts.

        Args:
            rate: Battery level consumption rate per second

        Returns:
            float: Power consumption in watts
        """
        wh_per_second = rate / self._wh_to_level_factor
        watts = wh_per_second * 3600.0  # Convert Wh/s to watts
        return watts

    def check_emergency_stop(self, current_level: float) -> bool:
        """
        Check if emergency stop should be triggered due to critically low battery.
        """
        with self._state._lock:
            if not self._config.enabled:
                return False

            # Check if battery level is at or below emergency threshold
            if current_level <= self._config.emergency_stop_threshold:
                # Only trigger if not already active (prevent repeated alerts)
                if not self._state.emergency_stop_active:
                    self._state.emergency_stop_active = True
                    self._logger.critical(
                        f"🚨 EMERGENCY STOP TRIGGERED: Battery depleted to {current_level:.1%} "
                        f"(threshold: {self._config.emergency_stop_threshold:.1%})"
                    )
                return True

            return False

    def is_emergency_stop_active(self) -> bool:
        """
        Check if emergency stop is currently active due to battery depletion.
        """
        with self._state._lock:
            return self._state.emergency_stop_active

    def reset_emergency_stop(self) -> None:
        """
        Reset emergency stop state (e.g., after battery has been charged).
        """
        with self._state._lock:
            if self._state.emergency_stop_active:
                self._logger.info("🔋 Emergency stop reset - battery level restored")
                self._state.emergency_stop_active = False

    def get_battery_level(self) -> float:
        """
        Get current battery level.
        
        Returns:
            float: Battery level from 0.0 (empty) to 1.0 (full)
        """
        with self._state._lock:
            return self._state.level

    def has_sufficient_battery_for_task(self, estimated_consumption: float,
                                      safety_margin: float = 0.15) -> bool:
        """
        Check if robot has sufficient battery for a task.
        
        Args:
            estimated_consumption: Estimated battery consumption for task
            safety_margin: Safety margin as fraction of battery (0.0 to 1.0)
            
        Returns:
            bool: True if sufficient battery available
        """
        with self._state._lock:
            if not self._config.enabled:
                return True  # If battery management disabled, assume sufficient
            
            current_level = self._state.level
            required_level = estimated_consumption + safety_margin
            
            return current_level >= required_level
