"""
Charging Station Manager Implementation - Professional charging station allocation and management.

This implementation provides comprehensive charging station management with automatic discovery,
proximity-based allocation, and integration with the existing bay lock system. It follows SOLID
principles and provides thread-safe operations for multi-robot environments.

Design Principles:
- **Single Responsibility**: Handles only charging station management and allocation
- **Open/Closed**: Extensible for different station types and allocation strategies
- **Liskov Substitution**: Fully implements IChargingStationManager interface
- **Interface Segregation**: Focused charging station management interface
- **Dependency Inversion**: Depends on abstractions, not concretions

Key Features:
- Automatic station discovery from warehouse map
- Proximity-based optimal station selection
- Integration with existing bay lock system
- Thread-safe concurrent operations
- Maintenance mode support
- Performance monitoring and statistics
"""

import time
import threading
import logging
import math
from typing import Optional, List, Dict, Tuple, Set
from dataclasses import dataclass, replace

from interfaces.charging_station_manager_interface import (
    IChargingStationManager,
    ChargingStationInfo,
    ChargingStationRequest,
    ChargingStationAssignment,
    ChargingStationStatus,
    ChargingStationError
)
from interfaces.configuration_interface import ChargingStationConfig, IBusinessConfigurationProvider
from interfaces.simulation_data_service_interface import ISimulationDataService
from warehouse.map import WarehouseMap


@dataclass
class _StationCacheEntry:
    """Cache entry for station information."""
    station: ChargingStationInfo
    last_updated: float
    ttl: int  # Time to live in seconds


class ChargingStationManagerImpl(IChargingStationManager):
    """
    Professional implementation of charging station management.

    Features:
    - Automatic discovery of charging stations from warehouse map
    - Proximity-based optimal station allocation
    - Integration with existing bay lock system for thread safety
    - Support for different station types (standard, fast, wireless)
    - Maintenance mode and health monitoring
    - Thread-safe concurrent operations
    - Performance statistics and monitoring

    Threading Model:
    - All public methods are thread-safe
    - Internal state protected by RLock
    - Suitable for high-frequency robot allocation requests
    """

    def __init__(self, config_provider: IBusinessConfigurationProvider,
                 simulation_data_service: ISimulationDataService,
                 warehouse_map: WarehouseMap):
        """
        Initialize charging station manager.

        Args:
            config_provider: Configuration provider for charging station parameters
            simulation_data_service: Service for database operations and bay locks
            warehouse_map: Warehouse map for station discovery
        """
        self._config = config_provider.get_charging_station_config()
        self._simulation_data_service = simulation_data_service
        self._warehouse_map = warehouse_map

        self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        # Thread safety
        self._lock = threading.RLock()

        # Station registry and cache
        self._stations: Dict[str, ChargingStationInfo] = {}
        self._station_cache: Dict[str, _StationCacheEntry] = {}
        self._robot_assignments: Dict[str, Set[str]] = {}  # robot_id -> set of station_ids

        # Performance tracking
        self._allocation_attempts = 0
        self._successful_allocations = 0
        self._allocation_timeouts = 0

        # Initialize stations
        if self._config.enabled:
            self._initialize_stations()
            self._logger.info(f"ChargingStationManager initialized with {len(self._stations)} stations")
    
    def request_charging_station(self, request: ChargingStationRequest) -> Optional[ChargingStationAssignment]:
        """
        Request allocation of an optimal charging station for a robot.

        This method:
        1. Finds available stations within constraints
        2. Selects optimal station based on distance and preferences
        3. Attempts to acquire lock on selected station
        4. Returns assignment details if successful
        """
        print(f"ChargingStationManager request_charging_station: {request}") # TODO: remove
        if not self._config.enabled:
            return None

        with self._lock:
            self._allocation_attempts += 1

            try:
                # Find available stations (force fresh DB status to avoid stale cross-robot caches)
                available_stations = self._find_available_stations(request)
                print(f"Available stations: {[station.station_id for station in available_stations]}")
                if not available_stations:
                    self._logger.debug(f"No available stations for robot {request.robot_id}")
                    return None

                # Select optimal station
                selected_station = self._select_optimal_station(available_stations, request)
                if not selected_station:
                    self._logger.debug(f"No suitable station selected for robot {request.robot_id}")
                    return None

                # Attempt to acquire lock - use the station's canonical ID directly to avoid rounding mismatches
                station_id = selected_station.station_id
                if self._acquire_station_lock(station_id, request.robot_id):
                    # Update assignment tracking
                    if request.robot_id not in self._robot_assignments:
                        self._robot_assignments[request.robot_id] = set()
                    self._robot_assignments[request.robot_id].add(station_id)

                    # Invalidate cache to ensure status is immediately updated
                    if station_id in self._station_cache:
                        del self._station_cache[station_id]

                    # Calculate assignment details
                    distance = self._calculate_distance(request.robot_position, selected_station.position)
                    travel_time = distance / 1.0  # Assume 1 m/s average speed
                    charging_time = self.estimate_charging_time(station_id, 0.2, 1.0)  # Assume 20% to 100%

                    assignment = ChargingStationAssignment(
                        station=selected_station,
                        distance=distance,
                        estimated_travel_time=travel_time,
                        estimated_charging_time=charging_time,
                        assigned_at=time.time()
                    )
                    print(f"Assignment: {assignment}") # TODO: remove

                    self._successful_allocations += 1
                    self._logger.info(f"Allocated station {station_id} to robot {request.robot_id}")
                    print(f"charging_manager: Allocated station {station_id} to robot {request.robot_id}") # TODO: remove
                    return assignment
                else:
                    # Diagnostics: fetch current DB owner (single-line, low-noise)
                    try:
                        lock_info = self._simulation_data_service.get_bay_lock_info(station_id)
                        owner = lock_info.get('robot_id') if lock_info else None
                        self._logger.warning(
                            f"Failed to acquire lock on station {station_id} for {request.robot_id}"
                            + (f"; currently held by {owner}" if owner else "; no owner found")
                        )
                    except Exception:
                        pass
                    return None

            except Exception as e:
                self._logger.error(f"Error allocating charging station for robot {request.robot_id}: {e}")
                raise ChargingStationError(f"Station allocation failed: {e}")

    def release_charging_station(self, station_id: str, robot_id: str) -> bool:
        """Release a previously allocated charging station."""
        with self._lock:
            try:
                # Release the bay lock
                success = self._simulation_data_service.release_bay_lock(station_id, robot_id)
                print(f"ChargingStationManager {station_id}: release_charging_station: {success}") # TODO: remove
                if success:
                    # Update assignment tracking
                    if robot_id in self._robot_assignments:
                        self._robot_assignments[robot_id].discard(station_id)
                        if not self._robot_assignments[robot_id]:
                            del self._robot_assignments[robot_id]

                    # Immediately update internal status model and cache to AVAILABLE
                    station = self._stations.get(station_id)
                    if station:
                        # Respect maintenance mode; otherwise mark available
                        if station.status != ChargingStationStatus.MAINTENANCE:
                            updated_station = replace(
                                station,
                                status=ChargingStationStatus.AVAILABLE,
                                assigned_robot_id=None,
                                assigned_at=None,
                                last_heartbeat=None
                            )
                            self._stations[station_id] = updated_station
                            # Refresh cache so subsequent availability checks reflect release instantly
                            self._station_cache[station_id] = _StationCacheEntry(
                                station=updated_station,
                                last_updated=time.time(),
                                ttl=self._config.station_cache_ttl
                            )
                    else:
                        # If station not found in internal map, ensure cache entry is cleared
                        if station_id in self._station_cache:
                            del self._station_cache[station_id]

                    self._logger.info(f"Released station {station_id} from robot {robot_id}")
                else:
                    self._logger.warning(f"Failed to release station {station_id} from robot {robot_id}")

                return success

            except Exception as e:
                self._logger.error(f"Error releasing charging station {station_id}: {e}")
                raise ChargingStationError(f"Station release failed: {e}")

    def heartbeat_charging_station(self, station_id: str, robot_id: str) -> bool:
        """Heartbeat an existing charging station assignment."""
        with self._lock:
            try:
                # Heartbeat the bay lock
                success = self._simulation_data_service.heartbeat_bay_lock(station_id, robot_id)

                if success:
                    # Update cache timestamp
                    if station_id in self._station_cache:
                        self._station_cache[station_id].last_updated = time.time()

                return success

            except Exception as e:
                self._logger.error(f"Error heartbeating charging station {station_id}: {e}")
                raise ChargingStationError(f"Station heartbeat failed: {e}")

    def shutdown(self) -> None:
        """Gracefully release held charging station locks and clear caches.

        Best-effort: never raises; logs warnings on failures.
        """
        with self._lock:
            try:
                # Collect current assignments and release them
                assignments_snapshot = {
                    robot_id: set(stations)
                    for robot_id, stations in self._robot_assignments.items()
                }
            except Exception:
                assignments_snapshot = {}

            for robot_id, station_ids in assignments_snapshot.items():
                for station_id in list(station_ids):
                    try:
                        self._simulation_data_service.release_bay_lock(station_id, robot_id)
                    except Exception:
                        pass
                    # Update internal maps regardless of DB outcome
                    try:
                        if robot_id in self._robot_assignments:
                            self._robot_assignments[robot_id].discard(station_id)
                            if not self._robot_assignments[robot_id]:
                                del self._robot_assignments[robot_id]
                    except Exception:
                        pass
                    # Refresh station cache to AVAILABLE
                    try:
                        station = self._stations.get(station_id)
                        if station and station.status != ChargingStationStatus.MAINTENANCE:
                            updated_station = replace(
                                station,
                                status=ChargingStationStatus.AVAILABLE,
                                assigned_robot_id=None,
                                assigned_at=None,
                                last_heartbeat=None
                            )
                            self._stations[station_id] = updated_station
                            self._station_cache[station_id] = _StationCacheEntry(
                                station=updated_station,
                                last_updated=time.time(),
                                ttl=self._config.station_cache_ttl
                            )
                        elif station_id in self._station_cache:
                            del self._station_cache[station_id]
                    except Exception:
                        pass
            try:
                self._logger.info("ChargingStationManager shutdown complete: all known locks released")
            except Exception:
                pass

    def get_charging_station_status(self, station_id: str, force_refresh: bool = False) -> Optional[ChargingStationInfo]:
        """Get current status of a specific charging station."""
        with self._lock:
            # Check cache first unless forced refresh requested
            if not force_refresh and station_id in self._station_cache:
                cache_entry = self._station_cache[station_id]
                if time.time() - cache_entry.last_updated < self._config.station_cache_ttl:
                    return cache_entry.station

            # Get fresh status from database
            try:
                # Check if station is locked
                lock_info = self._get_station_lock_info(station_id)

                # Get station info
                station = self._stations.get(station_id)
                if station:
                    # Update status based on lock, but respect internal status for maintenance
                    if station.status == ChargingStationStatus.MAINTENANCE:
                        # Maintenance status takes precedence
                        updated_station = station
                    elif lock_info:
                        print(f"Lock info: {lock_info['robot_id']}") # TODO: remove
                        updated_station = replace(
                            station,
                            status=ChargingStationStatus.OCCUPIED,
                            assigned_robot_id=lock_info.get('robot_id'),
                            assigned_at=lock_info.get('locked_at'),
                            last_heartbeat=lock_info.get('heartbeat_at')
                        )
                    else:
                        updated_station = replace(
                            station,
                            status=ChargingStationStatus.AVAILABLE,
                            assigned_robot_id=None,
                            assigned_at=None,
                            last_heartbeat=None
                        )

                    # Update cache
                    self._station_cache[station_id] = _StationCacheEntry(
                        station=updated_station,
                        last_updated=time.time(),
                        ttl=self._config.station_cache_ttl
                    )

                    # Low-noise diagnostics at debug level only
                    try:
                        if self._logger.isEnabledFor(logging.DEBUG):
                            src = "fresh"
                            self._logger.debug(f"[Charging] status {src} {station_id} -> {updated_station.status.name}")
                    except Exception:
                        pass

                    return updated_station

            except Exception as e:
                self._logger.error(f"Error getting station status for {station_id}: {e}")

            return None

    def list_available_stations(self, robot_position: Tuple[float, float],
                              max_distance: Optional[float] = None) -> List[ChargingStationInfo]:
        """List available charging stations within constraints."""
        if max_distance is None:
            max_distance = self._config.max_search_distance

        with self._lock:
            available_stations = []

            for station_id, station in self._stations.items():
                # Check distance
                distance = self._calculate_distance(robot_position, station.position)
                if distance > max_distance:
                    continue

                # Check availability (normal cache behavior for general listing callers)
                status = self.get_charging_station_status(station_id, force_refresh=False)
                if status and status.status == ChargingStationStatus.AVAILABLE:
                    available_stations.append(status)

            return available_stations

    def get_nearest_available_station(self, robot_position: Tuple[float, float]) -> Optional[ChargingStationInfo]:
        """Find the nearest available charging station."""
        available_stations = self.list_available_stations(
            robot_position=robot_position
        )

        if not available_stations:
            return None

        # Return closest station
        return min(available_stations,
                  key=lambda s: self._calculate_distance(robot_position, s.position))

    def get_station_statistics(self) -> Dict[str, int]:
        """Get charging station system statistics."""
        with self._lock:
            total_stations = len(self._stations)
            available_count = 0
            occupied_count = 0
            maintenance_count = 0

            for station_id, station in self._stations.items():
                if station.status == ChargingStationStatus.AVAILABLE:
                    available_count += 1
                elif station.status == ChargingStationStatus.OCCUPIED:
                    occupied_count += 1
                elif station.status == ChargingStationStatus.MAINTENANCE:
                    maintenance_count += 1

            return {
                'total_stations': total_stations,
                'available_stations': available_count,
                'occupied_stations': occupied_count,
                'maintenance_stations': maintenance_count,
                'allocation_attempts': self._allocation_attempts,
                'successful_allocations': self._successful_allocations,
                'allocation_timeouts': self._allocation_timeouts
            }

    def set_station_maintenance(self, station_id: str, in_maintenance: bool) -> bool:
        """Set maintenance status for a charging station."""
        with self._lock:
            try:
                station = self._stations.get(station_id)
                if not station:
                    return False

                new_status = ChargingStationStatus.MAINTENANCE if in_maintenance else ChargingStationStatus.AVAILABLE

                # Update station status
                updated_station = replace(station, status=new_status)
                self._stations[station_id] = updated_station

                # Clear cache
                if station_id in self._station_cache:
                    del self._station_cache[station_id]

                self._logger.info(f"Set station {station_id} maintenance mode: {in_maintenance}")
                return True

            except Exception as e:
                self._logger.error(f"Error setting maintenance mode for station {station_id}: {e}")
                return False

    def estimate_charging_time(self, station_id: str, current_battery: float,
                             target_battery: float = 1.0) -> float:
        """Estimate time required to charge from current to target battery level."""
        try:
            station = self._stations.get(station_id)
            if not station:
                raise ChargingStationError(f"Station {station_id} not found")

            if current_battery >= target_battery:
                return 0.0

            # Calculate energy needed (in watt-hours)
            energy_needed = (target_battery - current_battery) * 1000.0  # Convert to Wh

            # Account for charging efficiency
            effective_energy_needed = energy_needed / self._config.station_efficiency

            # Calculate time (power in watts)
            charging_time_seconds = effective_energy_needed / station.max_power_watts

            return charging_time_seconds

        except Exception as e:
            self._logger.error(f"Error estimating charging time for station {station_id}: {e}")
            raise ChargingStationError(f"Charging time estimation failed: {e}")

    # Private helper methods

    def _initialize_stations(self) -> None:
        """Initialize charging stations from warehouse map."""
        if not self._config.auto_discover_stations:
            return

        try:
            # Get charging zones from warehouse map
            charging_positions = self._warehouse_map._get_charging_zones()

            for i, position in enumerate(charging_positions):
                # Use grid coordinates to ensure unique station IDs
                grid_x, grid_y = self._warehouse_map.world_to_grid(position[0], position[1])
                station_id = f"charge_{grid_y}_{grid_x}"

                station = ChargingStationInfo(
                    station_id=station_id,
                    position=position,
                    status=ChargingStationStatus.AVAILABLE,
                    max_power_watts=self._config.station_power_watts,
                    efficiency=self._config.station_efficiency,
                    lock_timeout_seconds=self._config.station_lock_timeout
                )

                self._stations[station_id] = station

            self._logger.info(f"Discovered {len(self._stations)} charging stations from warehouse map")

        except Exception as e:
            self._logger.error(f"Error initializing charging stations: {e}")

    def _find_available_stations(self, request: ChargingStationRequest) -> List[ChargingStationInfo]:
        """Find available stations that match the request criteria."""
        max_distance = request.max_distance or self._config.max_search_distance

        available_stations = []
       # print(f"Stations: {self._stations.items()}") # TODO: remove
        for station_id, station in self._stations.items():
            # Check distance constraint
            distance = self._calculate_distance(request.robot_position, station.position)
            if distance > max_distance:
                continue

            # Check availability with forced refresh to avoid stale caches across robot managers
            status_info = self.get_charging_station_status(station_id, force_refresh=True)
            if status_info and status_info.status == ChargingStationStatus.AVAILABLE:
                available_stations.append(status_info)

        return available_stations

    def _select_optimal_station(self, available_stations: List[ChargingStationInfo],
                              request: ChargingStationRequest) -> Optional[ChargingStationInfo]:
        """Select the optimal station from available options (nearest available)."""
        if not available_stations:
            return None

        if len(available_stations) == 1:
            return available_stations[0]

        # Simple: return nearest available station
        return min(available_stations,
                  key=lambda s: self._calculate_distance(request.robot_position, s.position))

    def _acquire_station_lock(self, station_id: str, robot_id: str) -> bool:
        """Acquire a lock on a charging station using the bay lock system."""
        try:
            return self._simulation_data_service.try_acquire_bay_lock(
                bay_id=station_id,
                robot_id=robot_id,
                timeout_seconds=self._config.station_lock_timeout
            )
        except Exception as e:
            self._logger.error(f"Error acquiring station lock for {station_id}: {e}")
            print(f"charging_manager: Error acquiring station lock for {station_id}: {e}") # TODO: remove
            return False

    def _get_station_lock_info(self, station_id: str) -> Optional[Dict]:
        """
        Get lock information for a station from the database.

        Queries the bay_locks table to check if the station is currently locked
        and by which robot. This ensures status reporting is consistent with
        the actual database state.

        Args:
            station_id: The station identifier to check

        Returns:
            Dict with lock information if locked, None if unlocked.
            Format: {
                'robot_id': str,
                'locked_at': float,  # timestamp
                'heartbeat_at': float,  # timestamp
                'lock_timeout_seconds': int
            }
        """
        try:
            # Use the public simulation data service method
            return self._simulation_data_service.get_bay_lock_info(station_id)

        except Exception as e:
            self._logger.error(f"Error retrieving lock info for station {station_id}: {e}")
            # Return None on error to avoid breaking status checks
            return None

    def _calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two positions."""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
