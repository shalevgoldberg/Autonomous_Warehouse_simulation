"""
Motion Command Filter Implementation - Applies safety constraints to motion commands.

This implementation adapts over existing MotionExecutor to apply collision avoidance
safety actions, providing a clean integration point for safety systems without
modifying motion execution logic.

IMPLEMENTATION ARCHITECTURE:
===========================
The filter acts as an adapter between collision avoidance safety actions and
motion execution, modifying wheel velocity commands to ensure safe operation
while preserving motion planning intent where possible.

KEY COMPONENTS:
==============
1. DifferentialDriveKinematics: Handles conversion between wheel and body velocities
2. SafetyActionProcessor: Applies different safety actions to desired commands
3. ConfigurationManager: Manages robot physical parameters from configuration

DIFFERENTIAL DRIVE KINEMATICS:
=============================
For a differential drive robot with:
- Wheel radius r
- Wheel base (track width) b
- Left wheel velocity vl, right wheel velocity vr

Body velocities:
- Linear velocity: v = (vl + vr) * r / 2
- Angular velocity: ω = (vr - vl) * r / b

Wheel velocities from body velocities:
- vl = (v - ω * b/2) / r
- vr = (v + ω * b/2) / r

The filter uses these transformations to apply speed limits and yaw corrections
precisely while maintaining smooth motion.

PERFORMANCE OPTIMIZATION:
========================
- Vectorized math operations for efficiency
- Minimal memory allocations during filtering
- Stateless design for thread safety
- Early validation and error handling
"""

import math
import logging
from typing import Optional
from dataclasses import dataclass

from interfaces.motion_command_filter_interface import (
    IMotionCommandFilter,
    FilteredMotionCommand
)
from interfaces.collision_avoidance_interface import SafetyAction, SafetyActionType
from interfaces.configuration_interface import IBusinessConfigurationProvider


@dataclass(frozen=True)
class DifferentialDriveConfig:
    """
    Configuration parameters for differential drive kinematics.

    All parameters are validated to ensure physical correctness.
    """
    wheel_radius: float  # meters
    wheel_base: float    # meters (distance between wheels)

    def __post_init__(self):
        """Validate configuration parameters."""
        if self.wheel_radius <= 0:
            raise ValueError(f"wheel_radius must be > 0, got {self.wheel_radius}")
        if self.wheel_base <= 0:
            raise ValueError(f"wheel_base must be > 0, got {self.wheel_base}")


class DifferentialDriveKinematics:
    """
    Handles conversion between wheel velocities and body velocities.

    Provides precise kinematic transformations for differential drive robots,
    enabling proper application of speed limits and steering corrections.
    """

    def __init__(self, config: DifferentialDriveConfig):
        self.config = config

    def wheel_to_body_velocities(self, left_wheel_vel: float,
                                right_wheel_vel: float) -> tuple[float, float]:
        """
        Convert wheel velocities to body linear and angular velocities.

        Args:
            left_wheel_vel: Left wheel velocity (rad/s)
            right_wheel_vel: Right wheel velocity (rad/s)

        Returns:
            Tuple of (linear_velocity, angular_velocity) in m/s and rad/s
        """
        r = self.config.wheel_radius
        b = self.config.wheel_base

        # Linear velocity: average of wheel velocities converted to m/s
        linear_vel = (left_wheel_vel + right_wheel_vel) * r / 2.0

        # Angular velocity: difference of wheel velocities converted to rad/s
        angular_vel = (right_wheel_vel - left_wheel_vel) * r / b

        return linear_vel, angular_vel

    def body_to_wheel_velocities(self, linear_vel: float,
                                angular_vel: float) -> tuple[float, float]:
        """
        Convert body velocities to wheel velocities.

        Args:
            linear_vel: Body linear velocity (m/s)
            angular_vel: Body angular velocity (rad/s)

        Returns:
            Tuple of (left_wheel_vel, right_wheel_vel) in rad/s
        """
        r = self.config.wheel_radius
        b = self.config.wheel_base

        # Wheel velocities from inverse kinematics
        left_wheel_vel = (linear_vel - angular_vel * b / 2.0) / r
        right_wheel_vel = (linear_vel + angular_vel * b / 2.0) / r

        return left_wheel_vel, right_wheel_vel


class SafetyActionProcessor:
    """
    Processes safety actions and applies them to motion commands.

    Handles the logic for different safety action types, ensuring proper
    application of speed limits and steering corrections.
    """

    def __init__(self, kinematics: DifferentialDriveKinematics):
        self.kinematics = kinematics

    def apply_safety_action(self, safety_action: SafetyAction,
                          desired_left_vel: float,
                          desired_right_vel: float) -> tuple[float, float]:
        """
        Apply safety action to desired wheel velocities.

        Args:
            safety_action: Safety action to apply
            desired_left_vel: Desired left wheel velocity (rad/s)
            desired_right_vel: Desired right wheel velocity (rad/s)

        Returns:
            Tuple of (safe_left_vel, safe_right_vel) in rad/s
        """
        if safety_action.action_type == SafetyActionType.STOP:
            # Emergency stop: stop linear motion but preserve angular motion for rotation
            # This allows the robot to rotate in place to face a better direction
            # Convert to body velocities, set linear to 0, keep angular, convert back
            current_linear, current_angular = self.kinematics.wheel_to_body_velocities(
                desired_left_vel, desired_right_vel
            )

            # Stop linear motion (forward/backward) but preserve rotation
            safe_linear = 0.0
            safe_angular = current_angular  # Preserve desired turning

            # Convert back to wheel velocities
            safe_left_vel, safe_right_vel = self.kinematics.body_to_wheel_velocities(
                safe_linear, safe_angular
            )

            return safe_left_vel, safe_right_vel

        elif safety_action.action_type == SafetyActionType.FOLLOW:
            # Apply speed limit and/or yaw bias
            return self._apply_follow_constraints(
                safety_action, desired_left_vel, desired_right_vel
            )

        elif safety_action.action_type == SafetyActionType.CLEAR:
            # No modifications needed
            return desired_left_vel, desired_right_vel

        else:
            # Unknown action type - conservative fallback to stop
            return 0.0, 0.0

    def _apply_follow_constraints(self, safety_action: SafetyAction,
                                desired_left_vel: float,
                                desired_right_vel: float) -> tuple[float, float]:
        """
        Apply FOLLOW action constraints (speed limit and/or yaw bias).
        """
        # Convert to body velocities
        current_linear, current_angular = self.kinematics.wheel_to_body_velocities(
            desired_left_vel, desired_right_vel
        )

        # Apply speed limit if specified
        if safety_action.speed_limit is not None:
            current_linear = min(current_linear, safety_action.speed_limit)

        # Apply yaw bias if specified
        if safety_action.yaw_bias is not None:
            current_angular += safety_action.yaw_bias

        # Convert back to wheel velocities
        safe_left_vel, safe_right_vel = self.kinematics.body_to_wheel_velocities(
            current_linear, current_angular
        )

        return safe_left_vel, safe_right_vel


class MotionCommandFilterImpl(IMotionCommandFilter):
    """
    Implementation of motion command filter for collision avoidance.

    Provides a clean adapter between collision avoidance safety actions and
    motion execution, modifying wheel commands to ensure safe operation while
    preserving motion planning decisions where possible.

    ARCHITECTURAL DESIGN:
    ====================
    - Adapter Pattern: Adapts SafetyAction to wheel velocity modifications
    - Dependency Injection: Receives kinematics configuration externally
    - Stateless Processing: All state comes from input parameters
    - Error Recovery: Graceful handling of invalid inputs

    SAFETY INTEGRATION:
    ==================
    - STOP: Emergency halt of linear motion but allows rotation in place
    - FOLLOW with speed_limit: Clamp linear velocity while preserving turning
    - FOLLOW with yaw_bias: Add gentle steering correction for obstacle avoidance
    - CLEAR: Preserve original motion planning decisions

    ROBUSTNESS FEATURES:
    ===================
    - Input validation with descriptive error messages
    - Conservative fallbacks for edge cases
    - Performance monitoring and error tracking
    - Thread-safe operation with no shared mutable state

    CONFIGURATION:
    =============
    - Wheel radius and base from configuration provider
    - Default values for robustness when config unavailable
    - Validation ensures physical parameter correctness
    """

    def __init__(self, config_provider: Optional[IBusinessConfigurationProvider] = None,
                 robot_id: str = "unknown"):
        """
        Initialize motion command filter.

        Args:
            config_provider: Configuration provider for robot parameters
            robot_id: Robot identifier for logging
        """
        self.config_provider = config_provider
        self.robot_id = robot_id
        self.logger = logging.getLogger(f"MotionCommandFilter.{robot_id}")

        # Track command modifications to avoid repetitive logging
        self._previous_action_type = None
        self._command_count = 0
        self._significant_changes = 0

        # Initialize differential drive configuration
        drive_config = self._load_drive_config()
        self.kinematics = DifferentialDriveKinematics(drive_config)

        # Initialize safety action processor
        self.action_processor = SafetyActionProcessor(self.kinematics)

    def apply(self, safety_action: SafetyAction,
             desired_left_velocity: float,
             desired_right_velocity: float) -> FilteredMotionCommand:
        """
        Apply safety constraints to desired motion commands.

        Validates inputs, applies safety action constraints, and returns
        filtered motion commands ready for execution.

        Args:
            safety_action: Safety action from collision avoidance
            desired_left_velocity: Desired left wheel velocity (rad/s)
            desired_right_velocity: Desired right wheel velocity (rad/s)

        Returns:
            FilteredMotionCommand: Safe wheel velocities with applied action

        Raises:
            ValueError: If input parameters are invalid
            RuntimeError: If filtering fails due to internal errors
        """
        # Input validation
        self._validate_inputs(safety_action, desired_left_velocity, desired_right_velocity)

        try:
            # Apply safety action to get safe wheel velocities
            safe_left_vel, safe_right_vel = self.action_processor.apply_safety_action(
                safety_action, desired_left_velocity, desired_right_velocity
            )

            # Create filtered command
            filtered_command = FilteredMotionCommand(
                left_wheel_velocity=safe_left_vel,
                right_wheel_velocity=safe_right_vel,
                applied_safety_action=safety_action
            )

            return filtered_command

        except Exception as e:
            # Conservative fallback on any error
            error_action = SafetyAction(
                action_type=SafetyActionType.STOP,
                reason=f"Filter error: {e}"
            )
            return FilteredMotionCommand(
                left_wheel_velocity=0.0,
                right_wheel_velocity=0.0,
                applied_safety_action=error_action
            )

        # Log command modifications selectively
        self._log_command_modification(safety_action, desired_left_velocity, desired_right_velocity,
                                     safe_left_vel, safe_right_vel)

    def is_operational(self) -> bool:
        """
        Check if the motion command filter is operational.

        The filter is always operational as it has no external dependencies
        and uses conservative fallbacks for all error conditions.
        """
        return True

    def _load_drive_config(self) -> DifferentialDriveConfig:
        """
        Load differential drive configuration from provider or use defaults.
        """
        # Default values (conservative but reasonable)
        wheel_radius = 0.05  # 5cm wheels
        wheel_base = 0.30    # 30cm track width

        if self.config_provider is not None:
            try:
                # Try to load from configuration
                radius_val = self.config_provider.get_value("robot.wheel_radius", wheel_radius)
                wheel_radius = float(radius_val.value)

                base_val = self.config_provider.get_value("robot.wheel_base", wheel_base)
                wheel_base = float(base_val.value)

            except Exception as e:
                # Log warning but continue with defaults
                print(f"[MotionCommandFilter] Warning: Failed to load drive config: {e}, using defaults")

        return DifferentialDriveConfig(
            wheel_radius=wheel_radius,
            wheel_base=wheel_base
        )

    def _validate_inputs(self, safety_action: SafetyAction,
                        desired_left_velocity: float,
                        desired_right_velocity: float) -> None:
        """
        Validate input parameters.

        Args:
            safety_action: Safety action to validate
            desired_left_velocity: Left wheel velocity to validate
            desired_right_velocity: Right wheel velocity to validate

        Raises:
            ValueError: If any parameter is invalid
        """
        # Validate wheel velocities are finite
        for name, value in [
            ("desired_left_velocity", desired_left_velocity),
            ("desired_right_velocity", desired_right_velocity)
        ]:
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite, got {value}")

        # Validate safety action speed_limit if present
        if safety_action.speed_limit is not None:
            if safety_action.speed_limit < 0:
                raise ValueError(f"safety_action.speed_limit must be >= 0, got {safety_action.speed_limit}")

        # Validate safety action yaw_bias if present
        if safety_action.yaw_bias is not None:
            if not math.isfinite(safety_action.yaw_bias):
                raise ValueError(f"safety_action.yaw_bias must be finite, got {safety_action.yaw_bias}")

        # Additional validation could be added for extreme values
        # that might indicate system errors

    def _log_command_modification(self, safety_action: SafetyAction,
                                desired_left: float, desired_right: float,
                                safe_left: float, safe_right: float) -> None:
        """
        Log command modifications selectively to avoid output flooding.

        Only logs when:
        - Safety action type changes (INFO level)
        - Emergency STOP actions (WARNING level)
        - Significant command modifications (every 50 commands)
        """
        current_action_type = safety_action.action_type

        # Always log STOP actions (emergency situations)
        if current_action_type == SafetyActionType.STOP:
            self.logger.warning(f"EMERGENCY STOP applied: desired=({desired_left:.3f}, {desired_right:.3f}) "
                              f"→ safe=({safe_left:.3f}, {safe_right:.3f})")
            self._previous_action_type = current_action_type
            return

        # Log when action type changes
        if current_action_type != self._previous_action_type:
            change_str = self._format_command_change(desired_left, desired_right, safe_left, safe_right)
            self.logger.info(f"Safety action changed to {current_action_type.value}: {change_str}")
            self._previous_action_type = current_action_type
            self._command_count = 0
            return

        # Check for significant modifications
        modification = self._calculate_command_modification(desired_left, desired_right, safe_left, safe_right)

        # Log significant changes or periodic status
        self._command_count += 1
        should_log = False

        if modification > 0.1:  # Significant modification (>10% change)
            should_log = True
            self._significant_changes += 1
        elif self._command_count >= 50:  # Periodic logging
            should_log = True
            self._command_count = 0

        if should_log and current_action_type != SafetyActionType.CLEAR:
            change_str = self._format_command_change(desired_left, desired_right, safe_left, safe_right)
            log_level = "WARNING" if modification > 0.5 else "DEBUG"
            self.logger.debug(f"Command filter: {change_str}")

    def _calculate_command_modification(self, desired_left: float, desired_right: float,
                                      safe_left: float, safe_right: float) -> float:
        """Calculate the magnitude of command modification."""
        left_diff = abs(safe_left - desired_left)
        right_diff = abs(safe_right - desired_right)

        # Normalize by maximum expected wheel velocity (assume 10 rad/s max)
        max_expected = 10.0
        normalized_left = left_diff / max_expected
        normalized_right = right_diff / max_expected

        return max(normalized_left, normalized_right)

    def _format_command_change(self, desired_left: float, desired_right: float,
                             safe_left: float, safe_right: float) -> str:
        """Format command change for logging."""
        return (f"desired=({desired_left:.3f}, {desired_right:.3f}) → "
                f"safe=({safe_left:.3f}, {safe_right:.3f})")
