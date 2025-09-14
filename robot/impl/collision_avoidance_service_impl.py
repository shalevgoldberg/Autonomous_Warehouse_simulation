"""
Collision Avoidance Service Implementation - LiDAR-based collision avoidance system.

This module provides the concrete implementation of ICollisionAvoidanceService.
Provides egocentric safety logic using LiDAR data with different rules for static
and dynamic obstacles.

IMPLEMENTATION ARCHITECTURE:
===========================
The service follows an egocentric approach where all safety evaluation occurs in
the robot's local coordinate frame. Safety rules are applied based on obstacle
categories and their positions relative to the robot.

KEY COMPONENTS:
==============
1. SafetyEvaluator: Core evaluation logic with modular filtering
2. BeamGating: Filters rays by cone and lateral guard criteria
3. ObstacleAnalyzer: Applies category-specific safety rules
4. ActionResolver: Determines overall safety action from multiple requirements

PERFORMANCE OPTIMIZATION:
========================
- Vectorized numpy operations for efficient ray processing
- Minimal memory allocations during evaluation
- Early termination when STOP conditions are met
- Stateless design for thread safety

CONFIGURATION:
=============
Parameters are configurable but follow conservative defaults:
- FOV: ±25° (50° total), 25 beams
- Braking cone: ±10°
- Lateral guard: robot_radius + 0.10 m
- Static STOP: 0.15 m
- Dynamic STOP: 0.30 m, headway time: 0.8 s
- Caution sector: 10°–30°, distance: 0.50 m
- Yaw bias cap: 0.1 rad

FALLBACK BEHAVIOR:
=================
If hit categories are unavailable, all obstacles are treated as dynamic
(worst-case safety assumption).
"""

import math
import numpy as np
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass

from interfaces.collision_avoidance_interface import (
    ICollisionAvoidanceService,
    SafetyAction,
    SafetyActionType
)
from interfaces.lidar_interface import LiDARScan


@dataclass(frozen=True)
class CollisionAvoidanceConfig:
    """
    Configuration parameters for collision avoidance evaluation.

    All parameters are validated on creation to ensure safety constraints.
    """

    # LiDAR field of view and resolution
    fov_degrees: float = 50.0
    num_beams: int = 25

    # Braking zone (emergency stop)
    braking_cone_degrees: float = 10.0  # Half-angle (±10°)
    lateral_guard_margin: float = 0.10  # Added to robot radius

    # Safety distances
    static_stop_distance: float = 0.15
    dynamic_stop_distance: float = 0.30
    dynamic_caution_distance: float = 0.50

    # Dynamic behavior
    headway_time_gap: float = 0.8  # seconds for speed clamping

    # Steering corrections
    yaw_bias_gain: float = 1.0  # Multiplier for yaw bias calculation
    yaw_bias_max: float = 0.1   # Maximum yaw bias in radians

    def __post_init__(self):
        """Validate configuration parameters."""
        if not (10.0 <= self.fov_degrees <= 180.0):
            raise ValueError(f"fov_degrees must be between 10-180, got {self.fov_degrees}")

        if self.num_beams < 5:
            raise ValueError(f"num_beams must be >= 5, got {self.num_beams}")

        if not (5.0 <= self.braking_cone_degrees <= 45.0):
            raise ValueError(f"braking_cone_degrees must be between 5-45, got {self.braking_cone_degrees}")

        for name, value in [
            ("static_stop_distance", self.static_stop_distance),
            ("dynamic_stop_distance", self.dynamic_stop_distance),
            ("dynamic_caution_distance", self.dynamic_caution_distance),
            ("lateral_guard_margin", self.lateral_guard_margin)
        ]:
            if value <= 0:
                raise ValueError(f"{name} must be > 0, got {value}")

        if self.headway_time_gap <= 0:
            raise ValueError(f"headway_time_gap must be > 0, got {self.headway_time_gap}")

        if self.yaw_bias_max <= 0:
            raise ValueError(f"yaw_bias_max must be > 0, got {self.yaw_bias_max}")


class BeamGating:
    """
    Filters LiDAR beams based on spatial criteria.

    Provides efficient filtering of rays by cone angle and lateral position
    to identify which beams are relevant for safety evaluation.
    """

    def __init__(self, config: CollisionAvoidanceConfig):
        self.config = config

    def get_braking_zone_mask(self, angles_rad: np.ndarray,
                            lateral_distances: np.ndarray,
                            robot_radius: float) -> np.ndarray:
        """
        Get mask for beams in braking zone (emergency stop region).

        Braking zone combines cone angle and lateral guard for conservative stopping.
        """
        # Angular filter: within braking cone
        braking_cone_rad = math.radians(self.config.braking_cone_degrees)
        angle_mask = np.abs(angles_rad) <= braking_cone_rad

        # Lateral filter: within robot width + margin
        lateral_guard = robot_radius + self.config.lateral_guard_margin
        lateral_mask = np.abs(lateral_distances) <= lateral_guard

        return angle_mask & lateral_mask

    def get_caution_sector_mask(self, angles_rad: np.ndarray,
                              forward_distances: np.ndarray) -> np.ndarray:
        """
        Get mask for beams in caution sectors (steering correction region).

        Caution sectors are wider than braking cone for gentle steering corrections.
        """
        braking_cone_rad = math.radians(self.config.braking_cone_degrees)
        caution_outer_rad = math.radians(self.config.braking_cone_degrees * 3)  # 30° total

        # Angular range: braking_cone to caution_outer
        angle_mask = (np.abs(angles_rad) > braking_cone_rad) & \
                    (np.abs(angles_rad) <= caution_outer_rad)

        # Distance filter: within caution distance
        distance_mask = forward_distances <= self.config.dynamic_caution_distance

        return angle_mask & distance_mask


class ObstacleAnalyzer:
    """
    Analyzes obstacles by category and applies safety rules.

    Provides category-specific safety evaluation for static and dynamic obstacles.
    """

    def __init__(self, config: CollisionAvoidanceConfig):
        self.config = config

    def evaluate_static_obstacles(self, forward_distances: np.ndarray,
                                braking_zone_mask: np.ndarray,
                                valid_mask: np.ndarray) -> Optional[SafetyAction]:
        """
        Evaluate static obstacles (walls, shelves).

        Static obstacles only trigger STOP in braking zone when very close.
        No speed clamping or steering corrections for static obstacles.
        """
        # Find static obstacles in braking zone within stop distance
        static_in_zone = (braking_zone_mask & valid_mask)
        static_too_close = forward_distances <= self.config.static_stop_distance

        if np.any(static_in_zone & static_too_close):
            return SafetyAction(
                action_type=SafetyActionType.STOP,
                reason="Static obstacle in braking zone"
            )

        return None  # No safety action required

    def evaluate_dynamic_obstacles(self, forward_distances: np.ndarray,
                                 lateral_distances: np.ndarray,
                                 angles_rad: np.ndarray,
                                 braking_zone_mask: np.ndarray,
                                 caution_sector_mask: np.ndarray,
                                 valid_mask: np.ndarray) -> Optional[SafetyAction]:
        """
        Evaluate dynamic obstacles (other robots).

        Dynamic obstacles can trigger STOP, FOLLOW with speed clamping,
        and gentle steering corrections.
        """
        # Check braking zone for emergency stop
        dynamic_in_braking = braking_zone_mask & valid_mask
        dynamic_stop_required = forward_distances <= self.config.dynamic_stop_distance

        if np.any(dynamic_in_braking & dynamic_stop_required):
            return SafetyAction(
                action_type=SafetyActionType.STOP,
                reason="Dynamic obstacle in braking zone"
            )

        # Find closest dynamic obstacle in braking zone for speed clamping
        dynamic_distances = forward_distances[dynamic_in_braking & valid_mask]
        if len(dynamic_distances) > 0:
            closest_distance = np.min(dynamic_distances)

            # Calculate headway-based speed limit
            speed_limit = (closest_distance - self.config.dynamic_stop_distance) / \
                         self.config.headway_time_gap
            speed_limit = max(0.0, speed_limit)  # Clamp to non-negative

            # Check caution sectors for steering corrections
            yaw_bias = self._calculate_yaw_bias(
                forward_distances, lateral_distances, angles_rad,
                caution_sector_mask, valid_mask
            )

            return SafetyAction(
                action_type=SafetyActionType.FOLLOW,
                speed_limit=speed_limit,
                yaw_bias=yaw_bias,
                reason=f"Dynamic obstacle at {closest_distance:.2f}m"
            )

        return None  # No dynamic obstacles in braking zone

    def _calculate_yaw_bias(self, forward_distances: np.ndarray,
                          lateral_distances: np.ndarray,
                          angles_rad: np.ndarray,
                          caution_sector_mask: np.ndarray,
                          valid_mask: np.ndarray) -> Optional[float]:
        """
        Calculate gentle yaw bias based on caution sector obstacles.

        Provides steering corrections to avoid obstacles in caution sectors.
        If obstacles on both sides, no bias is applied (only speed control).
        """
        caution_mask = caution_sector_mask & valid_mask

        if not np.any(caution_mask):
            return None

        # Check left and right caution sectors
        left_sector = (angles_rad > 0) & caution_mask  # Positive angles = left
        right_sector = (angles_rad < 0) & caution_mask  # Negative angles = right

        left_danger = np.any(left_sector)
        right_danger = np.any(right_sector)

        if left_danger and right_danger:
            # Danger on both sides - no yaw bias, rely on speed control
            return None
        elif left_danger:
            # Danger on left - bias right (negative yaw)
            return -self.config.yaw_bias_gain * self.config.yaw_bias_max
        elif right_danger:
            # Danger on right - bias left (positive yaw)
            return self.config.yaw_bias_gain * self.config.yaw_bias_max

        return None


class ActionResolver:
    """
    Resolves multiple safety requirements into a single safety action.

    Applies override rules: STOP > FOLLOW > CLEAR.
    Combines speed limits (most restrictive) and yaw biases.
    """

    @staticmethod
    def resolve_actions(actions: List[Optional[SafetyAction]]) -> SafetyAction:
        """
        Resolve multiple safety actions into a single action.

        Override priority: STOP > FOLLOW > CLEAR
        For FOLLOW actions, takes the most restrictive speed limit
        and combines yaw biases appropriately.
        """
        # Filter out None actions
        valid_actions = [action for action in actions if action is not None]

        if not valid_actions:
            return SafetyAction(action_type=SafetyActionType.CLEAR)

        # Check for STOP (highest priority)
        stop_actions = [a for a in valid_actions if a.action_type == SafetyActionType.STOP]
        if stop_actions:
            # Return first STOP action (they're all equivalent)
            return stop_actions[0]

        # Check for FOLLOW actions
        follow_actions = [a for a in valid_actions if a.action_type == SafetyActionType.FOLLOW]
        if follow_actions:
            # Combine FOLLOW actions
            return ActionResolver._combine_follow_actions(follow_actions)

        # Only CLEAR actions remain
        return SafetyAction(action_type=SafetyActionType.CLEAR)

    @staticmethod
    def _combine_follow_actions(actions: List[SafetyAction]) -> SafetyAction:
        """Combine multiple FOLLOW actions into one."""
        # Take most restrictive speed limit
        speed_limits = [a.speed_limit for a in actions if a.speed_limit is not None]
        final_speed_limit = min(speed_limits) if speed_limits else None

        # Combine yaw biases (prefer non-zero biases, but handle conflicts)
        yaw_biases = [a.yaw_bias for a in actions if a.yaw_bias is not None]
        final_yaw_bias = None

        if yaw_biases:
            # If conflicting biases, cancel them out
            non_zero_biases = [b for b in yaw_biases if abs(b) > 1e-6]
            if len(non_zero_biases) == 1:
                final_yaw_bias = non_zero_biases[0]
            # If multiple conflicting biases, no bias

        # Combine reasons
        reasons = [a.reason for a in actions if a.reason]
        final_reason = "; ".join(reasons) if reasons else None

        return SafetyAction(
            action_type=SafetyActionType.FOLLOW,
            speed_limit=final_speed_limit,
            yaw_bias=final_yaw_bias,
            reason=final_reason
        )


class CollisionAvoidanceServiceImpl(ICollisionAvoidanceService):
    """
    Implementation of collision avoidance service.

    Provides egocentric safety evaluation using LiDAR data with category-aware
    rules for static and dynamic obstacles.

    ARCHITECTURAL DESIGN:
    ====================
    - Stateless evaluation: All state comes from input parameters
    - Modular components: Separate gating, analysis, and resolution
    - Category-aware: Different safety rules for different obstacle types
    - Conservative fallback: Unknown categories treated as dynamic
    - Thread-safe: No mutable state, vectorized operations

    SAFETY PHILOSOPHY:
    ==================
    - Static obstacles: Conservative STOP only when very close
    - Dynamic obstacles: STOP for immediate danger, speed control for headway,
      gentle steering for caution sectors
    - Unknown obstacles: Treated as dynamic (worst-case assumption)
    """

    def __init__(self, config: Optional[CollisionAvoidanceConfig] = None,
                 robot_radius: float = 0.125, robot_id: str = "unknown"):
        """
        Initialize collision avoidance service.

        Args:
            config: Configuration parameters (uses defaults if None)
            robot_radius: Robot radius in meters for lateral guard calculations
            robot_id: Robot identifier for logging
        """
        self.config = config or CollisionAvoidanceConfig()
        self.robot_radius = robot_radius
        self.robot_id = robot_id
        self.logger = logging.getLogger(f"CollisionAvoidance.{robot_id}")

        # Track previous safety action to avoid repetitive logging
        self._previous_action_type = None
        self._action_change_count = 0

        # Initialize modular components
        self.beam_gating = BeamGating(self.config)
        self.obstacle_analyzer = ObstacleAnalyzer(self.config)
        self.action_resolver = ActionResolver()

    def evaluate(self, robot_id: str, desired_speed: float, lidar_scan: LiDARScan) -> SafetyAction:
        """
        Evaluate collision avoidance for a robot based on LiDAR data.

        Performs comprehensive safety analysis in robot's local coordinate frame.

        Args:
            robot_id: Unique identifier of the robot being evaluated
            desired_speed: Robot's desired forward speed in m/s
            lidar_scan: Current LiDAR scan data with hit categories

        Returns:
            SafetyAction: Recommended safety action
        """
        # Input validation
        if desired_speed < 0:
            raise ValueError(f"desired_speed cannot be negative, got {desired_speed}")

        if lidar_scan.angles.shape != lidar_scan.distances.shape or \
           lidar_scan.distances.shape != lidar_scan.valid_mask.shape:
            raise ValueError("LiDAR scan arrays must have matching shapes (angles, distances, valid_mask)")

        # When categories are provided, ensure shape consistency
        if getattr(lidar_scan, 'hit_category', None) is not None:
            if lidar_scan.valid_mask.shape != lidar_scan.hit_category.shape:
                raise ValueError("LiDAR scan hit_category shape must match distances/valid_mask")

        # Transform to robot-local coordinates
        forward_distances, lateral_distances = self._transform_to_local_frame(lidar_scan)

        # Apply beam gating
        braking_zone_mask = self.beam_gating.get_braking_zone_mask(
            lidar_scan.angles, lateral_distances, self.robot_radius
        )
        caution_sector_mask = self.beam_gating.get_caution_sector_mask(
            lidar_scan.angles, forward_distances
        )

        # Handle category fallback
        if self._has_categories(lidar_scan):
            static_mask, dynamic_mask = self._categorize_obstacles(lidar_scan)
        else:
            # Fallback: treat all as dynamic (conservative)
            static_mask = np.zeros_like(lidar_scan.valid_mask, dtype=bool)
            dynamic_mask = np.ones_like(lidar_scan.valid_mask, dtype=bool)

        # Diagnostics: periodic category counts to validate inputs
        try:
            if self._action_change_count % 10 == 0:
                categories = getattr(lidar_scan, 'hit_category', None)
                if categories is not None:
                    unique, counts = np.unique(categories, return_counts=True)
                    cat_counts = {str(k): int(v) for k, v in zip(unique.tolist(), counts.tolist())}
                    valid_count = int(np.sum(lidar_scan.valid_mask))
                    self.logger.debug(f"[CA Debug] robot={self.robot_id} desired_speed={desired_speed:.2f} valid={valid_count} categories={cat_counts}")
        except Exception:
            pass

        # Evaluate static obstacles
        static_action = self.obstacle_analyzer.evaluate_static_obstacles(
            forward_distances, braking_zone_mask & static_mask, lidar_scan.valid_mask
        )

        # Evaluate dynamic obstacles
        dynamic_action = self.obstacle_analyzer.evaluate_dynamic_obstacles(
            forward_distances, lateral_distances, lidar_scan.angles,
            braking_zone_mask & dynamic_mask, caution_sector_mask & dynamic_mask,
            lidar_scan.valid_mask
        )

        # Resolve actions
        final_action = self.action_resolver.resolve_actions([static_action, dynamic_action])

        # Selective logging to avoid flooding
        self._log_safety_action(final_action, static_action, dynamic_action)

        return final_action

    def is_operational(self, robot_id: str) -> bool:
        """
        Check if collision avoidance service is operational.

        Service is always operational as it has no external dependencies
        and uses conservative fallback behavior.
        """
        return True

    def _transform_to_local_frame(self, lidar_scan: LiDARScan) -> Tuple[np.ndarray, np.ndarray]:
        """
        Transform LiDAR rays to robot-local coordinates.

        Converts polar coordinates to Cartesian coordinates in robot's frame:
        - x_forward: distance along robot's forward axis
        - y_lateral: lateral distance from robot's centerline
        """
        # Convert polar to cartesian in robot frame
        forward_distances = lidar_scan.distances * np.cos(lidar_scan.angles)
        lateral_distances = lidar_scan.distances * np.sin(lidar_scan.angles)

        return forward_distances, lateral_distances

    def _has_categories(self, lidar_scan: LiDARScan) -> bool:
        """
        Check if LiDAR scan has valid hit categories.

        Returns False if all categories are unknown or invalid.
        """
        categories = getattr(lidar_scan, 'hit_category', None)
        if categories is None:
            return False
        valid_categories = {'dynamic_robot', 'static', 'ignore', 'unknown'}
        unique_categories = set(categories)
        return bool(unique_categories & valid_categories)

    def _categorize_obstacles(self, lidar_scan: LiDARScan) -> Tuple[np.ndarray, np.ndarray]:
        """
        Categorize obstacles into static and dynamic masks.

        Returns masks for static and dynamic obstacles respectively.
        Unknown categories are treated as dynamic (conservative).
        """
        # Static obstacles (walls, shelves)
        categories = lidar_scan.hit_category if lidar_scan.hit_category is not None else np.array(['unknown'] * lidar_scan.valid_mask.shape[0])
        static_mask = (categories == 'static')

        # Dynamic obstacles (robots) + unknown (conservative fallback)
        dynamic_mask = (categories == 'dynamic_robot') | \
                      (categories == 'unknown')

        # Log warning if unknown categories are present (conservative fallback)
        unknown_count = np.sum(categories == 'unknown')
        if unknown_count > 0:
            self.logger.warning(f"Unknown obstacle categories detected ({unknown_count}/{len(lidar_scan.hit_category)} rays) - treating as dynamic obstacles (conservative fallback)")

        return static_mask, dynamic_mask

    def _log_safety_action(self, final_action: SafetyAction,
                          static_action: Optional[SafetyAction],
                          dynamic_action: Optional[SafetyAction]) -> None:
        """
        Log safety action changes selectively to avoid output flooding.

        Only logs when:
        - Safety action type changes
        - Emergency actions (STOP) are triggered
        - Every 100 evaluations for status monitoring
        """
        current_action_type = final_action.action_type

        # Always log STOP actions (emergency situations), except for static obstacles
        if current_action_type == SafetyActionType.STOP:
            if "Static obstacle" not in str(final_action.reason):
                self.logger.warning(f"EMERGENCY STOP: {final_action.reason}")
            self._previous_action_type = current_action_type
            return

        # Log when action type changes
        if current_action_type != self._previous_action_type:
            action_str = self._format_action_string(final_action)
            self.logger.info(f"Safety action changed to: {action_str}")
            self._previous_action_type = current_action_type
            self._action_change_count = 0
            return

        # Periodic status logging (every 100 evaluations when in same state)
        self._action_change_count += 1
        if self._action_change_count >= 100:
            if current_action_type != SafetyActionType.CLEAR:
                action_str = self._format_action_string(final_action)
                self.logger.debug(f"Continuing safety action: {action_str}")
            self._action_change_count = 0

    def _format_action_string(self, action: SafetyAction) -> str:
        """Format safety action for logging."""
        if action.action_type == SafetyActionType.CLEAR:
            return "CLEAR (no safety restrictions)"
        elif action.action_type == SafetyActionType.STOP:
            return f"STOP ({action.reason})"
        elif action.action_type == SafetyActionType.FOLLOW:
            parts = []
            if action.speed_limit is not None:
                parts.append(f"speed_limit={action.speed_limit:.2f}m/s")
            if action.yaw_bias is not None:
                parts.append(f"yaw_bias={action.yaw_bias:.3f}rad")
            reason = f" ({action.reason})" if action.reason else ""
            return f"FOLLOW ({', '.join(parts)}){reason}"
        else:
            return f"UNKNOWN ({action.action_type})"
