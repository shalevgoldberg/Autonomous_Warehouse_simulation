"""
LiDAR Sensor Interface - Collision avoidance sensor service.

This module provides interfaces for LiDAR-based collision avoidance,
following SOLID principles with clear separation of concerns.

ARCHITECTURAL OVERVIEW:
======================
The LiDAR system is designed for multi-robot warehouse environments with:
- Per-robot service instances for complete isolation
- Thread-safe operations for concurrent access
- Deterministic scanning with configurable performance tuning
- Integration with MuJoCo physics engine for accurate collision detection

PERFORMANCE CHARACTERISTICS:
===========================
- Scan Frequency: 10Hz (configurable)
- Field of View: 110° forward-facing arc
- Resolution: 1 ray per degree (110 rays total)
- Memory: ~1KB per scan (numpy arrays)
- Thread Safety: Per-robot locking with minimal contention
- Caching: Configurable TTL-based scan result caching

DESIGN DECISIONS:
================
1. Multi-robot Architecture: Each robot gets its own LiDARServiceImpl instance
2. Numpy Arrays: Used for performance but can be replaced with memory-efficient alternatives
3. Per-robot Caching: Prevents cross-contamination between robot scans
4. Thread-safe Interface: All public methods are thread-safe
5. Configuration Inheritance: LiDARConfig imported from configuration_interface (single source of truth)

DEPENDENCIES:
============
- numpy: For efficient numerical operations
- interfaces.configuration_interface: For LiDARConfig (avoid duplication)
"""
from abc import ABC, abstractmethod
from typing import Optional
from dataclasses import dataclass
import numpy as np


@dataclass
class LiDARScan:
    """
    LiDAR scan data for collision avoidance.

    Contains distance measurements in a 110° forward-facing arc with high-resolution
    angular sampling (1 ray per degree). Designed for real-time collision avoidance
    in multi-robot warehouse environments.

    DATA STRUCTURE:
    ===============
    - timestamp: Scan acquisition time (seconds since epoch)
    - angles: Ray angles in radians, shape (110,) numpy array
              Range: [-π/3.6, +π/3.6] radians (-55° to +55° from forward)
              Resolution: ~0.0175 radians per ray (1°)
    - distances: Measured distances in meters, shape (110,) numpy array
                 Valid range: [min_range, max_range] from LiDARConfig
                 Invalid measurements: max_range value
    - valid_mask: Boolean validity mask, shape (110,) numpy array
                  True: Valid measurement within range
                  False: No obstacle detected (max_range) or sensor error

    PERFORMANCE CHARACTERISTICS:
    ============================
    - Memory footprint: ~1KB per scan (3 numpy arrays of 110 float32 values)
    - Creation time: < 1ms
    - Access time: O(1) for individual rays, O(n) for bulk operations
    - Thread safety: Immutable after creation (dataclass frozen behavior)

    USAGE PATTERNS:
    ===============
    # Get all valid distance measurements
    valid_distances = distances[valid_mask]

    # Find closest obstacle
    if valid_distances.size > 0:
        closest_distance = np.min(valid_distances)

    # Check if path is clear in specific angular range
    angle_range = (angles >= min_angle) & (angles <= max_angle)
    clear_path = np.all(distances[angle_range & valid_mask] >= safety_distance)
    """
    timestamp: float
    angles: np.ndarray      # Angles in radians (-55° to +55° from forward)
    distances: np.ndarray   # Distance measurements in meters
    valid_mask: np.ndarray  # Boolean mask indicating valid measurements
    # Optional per-ray hit categories. When not provided, downstream logic should
    # conservatively treat obstacles as dynamic (egocentric fallback).
    hit_category: Optional[np.ndarray] = None  # e.g., 'dynamic_robot'|'static'|'ignore'|'unknown'


# Import LiDARConfig from configuration interface to avoid duplication
from interfaces.configuration_interface import LiDARConfig


class ILiDARService(ABC):
    """
    Interface for LiDAR sensor service.

    Provides collision avoidance capabilities through distance measurements
    in a forward-facing arc. Designed for integration with physics simulation
    and multi-robot warehouse environments.

    ARCHITECTURAL DESIGN:
    ====================
    - Per-robot instances: Each robot gets its own LiDARServiceImpl
    - Thread-safe interface: All methods are thread-safe
    - Asynchronous scanning: perform_scan() called from physics thread
    - Synchronous data access: get_scan_data() can be called from any thread

    THREAD SAFETY GUARANTEES:
    ========================
    - All public methods are thread-safe
    - Internal locking prevents race conditions
    - Per-robot isolation ensures no cross-contamination
    - Configuration updates are atomic

    PERFORMANCE REQUIREMENTS:
    ========================
    - Scan frequency: 10Hz (100ms intervals)
    - Latency: < 10ms per scan operation
    - Memory: < 2KB per robot (scan data + configuration)
    - CPU: < 1% of single core for typical warehouse scenarios
    - Thread contention: Minimal locking time

    ERROR HANDLING:
    ===============
    - Invalid robot_id: Logged warning, safe graceful degradation
    - Physics mode mismatch: Service becomes non-operational
    - Configuration errors: Validation with detailed error messages
    - Scan failures: Logged errors, service continues operating

    IMPLEMENTATION CONTRACT:
    =======================
    All implementations must:
    1. Be thread-safe for concurrent multi-robot operation
    2. Validate robot_id parameters and handle invalid requests
    3. Provide comprehensive health monitoring
    4. Implement proper error recovery mechanisms
    5. Follow performance requirements for real-time operation
    """

    @abstractmethod
    def get_scan_data(self, robot_id: str) -> Optional[LiDARScan]:
        """
        Get the latest LiDAR scan data for a specific robot.

        Retrieves the most recent scan data for the specified robot. Returns None
        if no scan has been performed or if the robot_id is invalid.

        Args:
            robot_id (str): Unique identifier of the robot to get scan data for

        Returns:
            Optional[LiDARScan]: Latest scan data containing timestamp, angles,
            distances, and validity mask, or None if unavailable

        Raises:
            No exceptions - Invalid robot_id is handled gracefully with None return

        Performance:
            - Time Complexity: O(1)
            - Memory: Returns reference to existing data (no copy)
            - Thread Safety: Fully thread-safe, can be called concurrently

        Example:
            scan = lidar_service.get_scan_data("robot_1")
            if scan:
                valid_distances = scan.distances[scan.valid_mask]
                closest_obstacle = np.min(valid_distances) if valid_distances.size > 0 else float('inf')
        """
        pass

    @abstractmethod
    def get_config(self, robot_id: str) -> LiDARConfig:
        """
        Get current LiDAR configuration for a specific robot.

        Returns a copy of the current configuration for the specified robot.
        For invalid robot_id, returns default configuration.

        Args:
            robot_id (str): Unique identifier of the robot to get configuration for

        Returns:
            LiDARConfig: Copy of current sensor configuration with all parameters

        Performance:
            - Time Complexity: O(1)
            - Memory: Returns shallow copy of configuration object
            - Thread Safety: Fully thread-safe

        Configuration Parameters:
            - enabled: Whether LiDAR scanning is active
            - num_rays: Number of rays in scan (typically 110)
            - max_range: Maximum detection distance in meters
            - min_range: Minimum detection distance in meters
            - scan_frequency: Target scans per second (Hz)
            - field_of_view: Angular field of view in degrees
            - safety_distance: Minimum safe distance for collision avoidance
            - enable_scan_caching: Whether to cache scan results
            - scan_cache_ttl: Cache time-to-live in seconds
        """
        pass

    @abstractmethod
    def set_config(self, robot_id: str, config: LiDARConfig) -> None:
        """
        Update LiDAR configuration for a specific robot.

        Applies new configuration to the specified robot. Changes take effect
        on the next scan operation. Invalid configurations are rejected with logging.

        Args:
            robot_id (str): Unique identifier of the robot to update
            config (LiDARConfig): New configuration to apply

        Returns:
            None

        Configuration Validation:
            - num_rays > 0
            - max_range > 0 and > min_range
            - min_range >= 0
            - scan_frequency > 0
            - field_of_view between 0 and 360 degrees
            - safety_distance >= 0

        Performance:
            - Time Complexity: O(1)
            - Thread Safety: Fully thread-safe
            - Effect Timing: Changes applied on next perform_scan() call

        Note:
            Configuration updates are scheduled to avoid interrupting
            in-progress scans, ensuring deterministic behavior.
        """
        pass

    @abstractmethod
    def is_operational(self, robot_id: str) -> bool:
        """
        Check if LiDAR sensor is operational for a specific robot.

        Performs comprehensive health check including physics mode compatibility,
        configuration validity, recent scan availability, and error rate monitoring.

        Args:
            robot_id (str): Unique identifier of the robot to check

        Returns:
            bool: True if sensor is working correctly and ready for operation

        Health Check Criteria:
            - Physics mode: Must be 'mujoco_authoritative'
            - Configuration: Must be valid
            - Recent scans: Must have scans within last second (10Hz requirement)
            - Error rate: Must have < 10 errors to avoid error state
            - Robot validity: Must be a valid robot_id

        Performance:
            - Time Complexity: O(1)
            - Thread Safety: Fully thread-safe
            - Health monitoring is continuous and lightweight

        Example:
            if lidar_service.is_operational("robot_1"):
                # Safe to perform operations
                lidar_service.perform_scan("robot_1")
            else:
                # Handle non-operational state
                print("LiDAR sensor not operational")
        """
        pass

    @abstractmethod
    def perform_scan(self, robot_id: str) -> None:
        """
        Perform a LiDAR scan for a specific robot.

        Executes a complete LiDAR scan operation including raycasting against
        the physics environment. This is a synchronous operation that should
        be called from the physics thread for proper timing synchronization.

        Args:
            robot_id (str): Unique identifier of the robot to perform scan for

        Returns:
            None

        Operation Sequence:
            1. Validate robot_id and operational status
            2. Apply any pending configuration changes
            3. Get current robot pose from physics engine
            4. Generate ray directions (110 rays over 110° arc)
            5. Perform raycasting for collision detection
            6. Store results in LiDARScan format
            7. Update health monitoring statistics

        Performance Requirements:
            - Target Frequency: 10Hz (every 100ms)
            - Maximum Latency: < 10ms per scan
            - Memory Allocation: Minimal (reuses buffers)
            - Thread Safety: Must be called from physics thread only

        Threading Notes:
            - **CRITICAL**: Must be called from physics thread only
            - Physics thread timing ensures proper synchronization
            - Non-physics thread calls will log warnings and return early
            - Concurrent calls for different robots are supported

        Error Handling:
            - Invalid robot_id: Logged warning, operation skipped
            - Physics mode mismatch: Logged error, operation skipped
            - Raycasting failures: Partial results stored, errors logged
            - Configuration errors: Fallback to previous valid config
        """
        pass
