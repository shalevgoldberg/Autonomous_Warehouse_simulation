"""
Collision Avoidance Service Interface - LiDAR-based collision avoidance system.

This module provides interfaces for collision avoidance capabilities using LiDAR data.
Follows SOLID principles with clear separation of concerns and dependency inversion.

ARCHITECTURAL OVERVIEW:
======================
The collision avoidance system provides egocentric safety logic that:
- Uses LiDAR data to detect obstacles in robot's local frame
- Applies different safety rules for static vs dynamic obstacles
- Provides speed clamping and gentle steering corrections
- Integrates with motion control without modifying path planning

PERFORMANCE CHARACTERISTICS:
===========================
- Real-time operation at 10Hz control loop frequency
- Minimal computational overhead per evaluation
- Thread-safe for concurrent robot operation
- Configurable parameters for different environments

DESIGN DECISIONS:
================
1. Egocentric approach: All logic in robot's local coordinate frame
2. Category-based behavior: Different rules for static vs dynamic obstacles
3. Conservative safety: Treat unknown obstacles as dynamic (worst case)
4. Speed-first control: Clamp speed before considering steering corrections
5. Override architecture: Safety layer overrides planner commands when needed
"""

from abc import ABC, abstractmethod
from typing import Optional
from dataclasses import dataclass
from enum import Enum

from interfaces.lidar_interface import LiDARScan


class SafetyActionType(Enum):
    """Types of safety actions that can be taken."""
    CLEAR = "clear"           # No safety action needed
    FOLLOW = "follow"         # Follow speed limit with optional yaw bias
    STOP = "stop"             # Emergency stop (linear velocity to zero, allows rotation)


@dataclass(frozen=True)
class SafetyAction:
    """
    Safety action recommendation from collision avoidance system.

    Encapsulates the safety decision and any required modifications to robot commands.
    STOP overrides all other actions and sets linear velocity to zero but allows rotation.
    FOLLOW allows speed clamping and gentle steering corrections.
    """

    action_type: SafetyActionType
    """Type of safety action required."""

    speed_limit: Optional[float] = None
    """Maximum allowed speed in m/s (for FOLLOW actions). None means no limit."""

    yaw_bias: Optional[float] = None
    """Yaw angle bias in radians (for FOLLOW actions). None means no bias."""

    reason: Optional[str] = None
    """Human-readable explanation of why this action was taken."""

    def __post_init__(self):
        """Validate safety action consistency."""
        if self.action_type == SafetyActionType.STOP:
            # STOP overrides everything - no speed limit or yaw bias
            if self.speed_limit is not None or self.yaw_bias is not None:
                raise ValueError("STOP action cannot have speed_limit or yaw_bias")
        elif self.action_type == SafetyActionType.FOLLOW:
            # FOLLOW requires speed_limit, yaw_bias is optional
            if self.speed_limit is None:
                raise ValueError("FOLLOW action must have speed_limit")
            if self.speed_limit < 0:
                raise ValueError("speed_limit cannot be negative")
        elif self.action_type == SafetyActionType.CLEAR:
            # CLEAR should have no modifications
            if self.speed_limit is not None or self.yaw_bias is not None:
                raise ValueError("CLEAR action cannot have speed_limit or yaw_bias")


class ICollisionAvoidanceService(ABC):
    """
    Interface for collision avoidance service.

    Provides egocentric collision avoidance logic using LiDAR data.
    Evaluates safety in robot's local coordinate frame and recommends
    appropriate safety actions to prevent collisions.

    ARCHITECTURAL PRINCIPLES:
    =========================
    - Egocentric evaluation: All logic in robot's local coordinate frame
    - Category-aware: Different safety rules for static vs dynamic obstacles
    - Conservative fallback: Unknown obstacles treated as dynamic
    - Override-capable: Can override planner commands when safety requires it

    DEPENDENCY INVERSION:
    ====================
    - Depends only on LiDARScan interface (not concrete implementation)
    - No dependencies on motion control or path planning
    - Clean separation from higher-level coordination systems

    THREAD SAFETY:
    ==============
    - All methods must be thread-safe for concurrent robot operation
    - No mutable state that could cause race conditions
    - Stateless evaluation (all state comes from input parameters)
    """

    @abstractmethod
    def evaluate(self, robot_id: str, desired_speed: float, lidar_scan: LiDARScan) -> SafetyAction:
        """
        Evaluate collision avoidance for a robot based on LiDAR data.

        Performs egocentric safety analysis in the robot's local coordinate frame.
        Considers obstacle categories, positions, and applies appropriate safety rules.

        Args:
            robot_id: Unique identifier of the robot being evaluated
            desired_speed: Robot's desired forward speed in m/s (for headway calculations)
            lidar_scan: Current LiDAR scan data with hit categories

        Returns:
            SafetyAction: Recommended safety action with speed limits and steering corrections

        Raises:
            ValueError: If input parameters are invalid
            RuntimeError: If evaluation fails due to internal errors

        THREAD SAFETY: This method must be thread-safe and stateless.

        EVALUATION PROCESS:
        ==================
        1. Transform LiDAR rays to robot-local coordinates (x_forward, y_lateral)
        2. Apply braking cone and lateral guard filters
        3. Evaluate static obstacles (conservative STOP rules)
        4. Evaluate dynamic obstacles (STOP, FOLLOW with headway, caution sectors)
        5. Determine overall safety action based on most restrictive requirement
        6. Return SafetyAction with appropriate speed limits and steering corrections

        PERFORMANCE REQUIREMENTS:
        ========================
        - Evaluation time: < 5ms per robot
        - Memory usage: Minimal, no allocations during evaluation
        - CPU usage: < 2% of single core at 10Hz evaluation rate
        """
        pass

    @abstractmethod
    def is_operational(self, robot_id: str) -> bool:
        """
        Check if collision avoidance service is operational for a robot.

        Verifies that the service has required configuration and can perform
        safety evaluations. Should be checked before calling evaluate().

        Args:
            robot_id: Unique identifier of the robot to check

        Returns:
            bool: True if service is ready to perform evaluations

        THREAD SAFETY: This method must be thread-safe.
        """
        pass
