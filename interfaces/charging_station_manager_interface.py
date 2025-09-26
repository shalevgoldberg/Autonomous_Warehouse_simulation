"""
Interface for Charging Station Manager - manages charging station allocation and availability.

This interface defines the charging station management service that allocates available charging
stations to robots based on proximity, availability, and charging requirements. It follows the
same architectural patterns as the bay scheduling system but is specialized for charging operations.

Design Principles:
- **Single Responsibility**: Handles only charging station management and allocation
- **Open/Closed**: Extensible for different charging station types and policies
- **Liskov Substitution**: All implementations are interchangeable
- **Interface Segregation**: Focused charging station management interface
- **Dependency Inversion**: Depends on abstractions, not concretions
"""

from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass
from enum import Enum


class ChargingStationStatus(Enum):
    """Status of a charging station."""
    AVAILABLE = "available"
    OCCUPIED = "occupied"
    OUT_OF_SERVICE = "out_of_service"
    MAINTENANCE = "maintenance"


@dataclass
class ChargingStationInfo:
    """Information about a charging station."""
    station_id: str
    position: Tuple[float, float]  # (x, y) in world coordinates
    status: ChargingStationStatus
    max_power_watts: float
    efficiency: float
    assigned_robot_id: Optional[str] = None
    assigned_at: Optional[float] = None  # timestamp
    last_heartbeat: Optional[float] = None
    lock_timeout_seconds: int = 600


@dataclass
class ChargingStationRequest:
    """Request for charging station allocation."""
    robot_id: str
    robot_position: Tuple[float, float]  # (x, y) in world coordinates
    max_distance: Optional[float] = None  # meters (optional constraint)
    timestamp: Optional[float] = None


@dataclass
class ChargingStationAssignment:
    """Result of charging station allocation."""
    station: ChargingStationInfo
    distance: float  # Distance from robot to station
    estimated_travel_time: float  # Estimated time to reach station
    estimated_charging_time: float  # Estimated time to fully charge
    assigned_at: float


class ChargingStationError(Exception):
    """Raised when charging station operations fail."""
    pass


class IChargingStationManager(ABC):
    """
    Interface for charging station management functionality.

    Responsibilities:
    - Manage charging station availability and allocation
    - Find optimal charging station for robot requests
    - Handle station assignment and release with locking
    - Monitor station health and maintenance status
    - Provide charging station information and statistics

    **Thread Safety**: All methods are thread-safe for concurrent access.
    **Threading Model**:
    - request_station/release_station(): Called from robot control threads
    - get_station_status(): Called from any thread (read-only)
    - heartbeat_station(): Called from robot threads to maintain locks
    """

    @abstractmethod
    def request_charging_station(self, request: ChargingStationRequest) -> Optional[ChargingStationAssignment]:
        """
        Request allocation of an optimal charging station for a robot.

        Args:
            request: Charging station allocation request

        Returns:
            ChargingStationAssignment: Assignment details if successful, None if no suitable station available

        Raises:
            ChargingStationError: If allocation fails due to system errors
        """
        pass

    @abstractmethod
    def release_charging_station(self, station_id: str, robot_id: str) -> bool:
        """
        Release a previously allocated charging station.

        Args:
            station_id: Identifier of station to release
            robot_id: Robot releasing the station (must be the assigned robot)

        Returns:
            bool: True if station was successfully released, False otherwise

        Raises:
            ChargingStationError: If release fails due to system errors
        """
        pass

    @abstractmethod
    def heartbeat_charging_station(self, station_id: str, robot_id: str) -> bool:
        """
        Heartbeat an existing charging station assignment to keep it alive.

        Args:
            station_id: Identifier of assigned station
            robot_id: Robot maintaining the assignment

        Returns:
            bool: True if heartbeat successful, False if assignment lost

        Raises:
            ChargingStationError: If heartbeat fails due to system errors
        """
        pass

    @abstractmethod
    def get_charging_station_status(self, station_id: str) -> Optional[ChargingStationInfo]:
        """
        Get current status of a specific charging station.

        Args:
            station_id: Identifier of station to check

        Returns:
            ChargingStationInfo: Station information, or None if not found

        Raises:
            ChargingStationError: If status retrieval fails
        """
        pass

    @abstractmethod
    def list_available_stations(self, robot_position: Tuple[float, float],
                              max_distance: Optional[float] = None) -> List[ChargingStationInfo]:
        """
        List available charging stations within constraints.

        Args:
            robot_position: Robot's current position (x, y)
            max_distance: Maximum distance from robot (optional)

        Returns:
            List[ChargingStationInfo]: List of available stations

        Raises:
            ChargingStationError: If listing fails
        """
        pass

    @abstractmethod
    def get_nearest_available_station(self, robot_position: Tuple[float, float]) -> Optional[ChargingStationInfo]:
        """
        Find the nearest available charging station.

        Args:
            robot_position: Robot's current position (x, y)

        Returns:
            ChargingStationInfo: Nearest available station, or None if none available

        Raises:
            ChargingStationError: If search fails
        """
        pass

    @abstractmethod
    def get_station_statistics(self) -> Dict[str, int]:
        """
        Get charging station system statistics.

        Returns:
            Dict containing:
            - total_stations: Total number of stations
            - available_stations: Number of available stations
            - occupied_stations: Number of occupied stations
            - maintenance_stations: Number of stations under maintenance

        Raises:
            ChargingStationError: If statistics retrieval fails
        """
        pass

    @abstractmethod
    def set_station_maintenance(self, station_id: str, in_maintenance: bool) -> bool:
        """
        Set maintenance status for a charging station.

        Args:
            station_id: Identifier of station
            in_maintenance: True to put in maintenance, False to bring back online

        Returns:
            bool: True if status was successfully changed

        Raises:
            ChargingStationError: If maintenance status change fails
        """
        pass

    @abstractmethod
    def estimate_charging_time(self, station_id: str, current_battery: float,
                             target_battery: float = 1.0) -> float:
        """
        Estimate time required to charge from current to target battery level.

        Args:
            station_id: Identifier of charging station
            current_battery: Current battery level (0.0 to 1.0)
            target_battery: Target battery level (0.0 to 1.0)

        Returns:
            float: Estimated charging time in seconds

        Raises:
            ChargingStationError: If estimation fails or station not found
        """
        pass
