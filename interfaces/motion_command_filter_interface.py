"""
Motion Command Filter Interface - Applies safety constraints to motion commands.

This interface provides a clean separation between motion planning and safety enforcement,
allowing collision avoidance systems to modify wheel commands without coupling to
motion execution details.

ARCHITECTURAL OVERVIEW:
======================
The motion command filter sits between the motion planner (which generates desired
wheel commands) and the motion executor (which applies them to the robot). It applies
safety constraints from collision avoidance systems while preserving the motion
planning logic.

DEPENDENCY INVERSION:
====================
- Depends only on SafetyAction interface (not collision avoidance implementation)
- No dependencies on motion planning or execution details
- Clean adapter pattern for safety integration

THREAD SAFETY:
=============
- All methods must be thread-safe for concurrent operation
- Stateless filtering (all state comes from input parameters)
- No mutable internal state

INTEGRATION POINTS:
==================
- Called by motion control loop after motion planning but before execution
- Applied at 10Hz control frequency to match collision avoidance updates
- Can override or modify any aspect of wheel commands for safety
"""

from abc import ABC, abstractmethod
from typing import Tuple
from dataclasses import dataclass

from interfaces.collision_avoidance_interface import SafetyAction


@dataclass(frozen=True)
class FilteredMotionCommand:
    """
    Filtered motion command with safety constraints applied.

    Contains the final wheel velocities that should be sent to the motion executor,
    after safety filtering has been applied to the original desired commands.
    """
    left_wheel_velocity: float
    right_wheel_velocity: float
    applied_safety_action: SafetyAction


class IMotionCommandFilter(ABC):
    """
    Interface for filtering motion commands based on safety constraints.

    Applies collision avoidance safety actions to desired motion commands,
    modifying wheel velocities to ensure safe operation while preserving
    motion planning intent where possible.

    ARCHITECTURAL PRINCIPLES:
    =========================
    - Single Responsibility: Only filters commands based on safety actions
    - Open/Closed: Extensible for different safety action types
    - Liskov Substitution: Any implementation can replace another
    - Interface Segregation: Focused on command filtering only
    - Dependency Inversion: Depends on abstractions, not concretions

    SAFETY PHILOSOPHY:
    ==================
    - STOP overrides all other actions (emergency safety first)
    - FOLLOW allows controlled motion with safety constraints
    - CLEAR preserves original motion planning decisions
    - Safety actions can modify speed, direction, or both

    DIFFERENTIAL DRIVE KINEMATICS:
    ==============================
    The filter understands differential drive kinematics to properly apply
    speed limits and yaw corrections:
    - Linear velocity: v = (vl + vr) * r / 2
    - Angular velocity: ω = (vr - vl) * r / b
    Where r = wheel radius, b = wheel base

    This allows precise control over robot motion components.
    """

    @abstractmethod
    def apply(self, safety_action: SafetyAction,
             desired_left_velocity: float,
             desired_right_velocity: float) -> FilteredMotionCommand:
        """
        Apply safety constraints to desired motion commands.

        Takes a safety action from collision avoidance and the desired wheel
        velocities from motion planning, then returns filtered commands that
        respect safety constraints while preserving motion intent where safe.

        Args:
            safety_action: Safety action from collision avoidance system
            desired_left_velocity: Desired left wheel velocity (rad/s)
            desired_right_velocity: Desired right wheel velocity (rad/s)

        Returns:
            FilteredMotionCommand: Safe wheel velocities with applied action

        Raises:
            ValueError: If input parameters are invalid
            RuntimeError: If filtering fails due to internal errors

        FILTERING LOGIC:
        ================
        1. STOP: Set linear velocity to 0 but preserve angular velocity (allows rotation in place)
        2. FOLLOW with speed_limit: Clamp linear speed to limit, preserve angular velocity
        3. FOLLOW with yaw_bias: Add yaw correction, preserve linear speed
        4. FOLLOW with both: Apply both speed clamp and yaw correction
        5. CLEAR: Return desired velocities unchanged

        DIFFERENTIAL DRIVE TRANSFORMATION:
        ================================
        Speed clamping requires converting wheel velocities to linear/angular
        components, applying limits, then converting back to wheel velocities.

        Yaw bias is added to the angular velocity component.

        PERFORMANCE REQUIREMENTS:
        ========================
        - Filtering time: < 1ms per call
        - Memory usage: Minimal, no allocations during filtering
        - Precision: Maintains velocity precision for smooth motion
        - Frequency: Supports 10Hz control loop updates
        """
        pass

    @abstractmethod
    def is_operational(self) -> bool:
        """
        Check if the motion command filter is operational.

        Verifies that the filter has required configuration and can perform
        command filtering. Should be checked before calling apply().

        Returns:
            bool: True if filter is ready to process commands
        """
        pass
