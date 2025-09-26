"""
Configuration Management Interface - Centralized system configuration.

This module provides interfaces for managing all system configuration parameters
following SOLID principles with clear separation of concerns and extensibility.

Design Principles:
- **Single Responsibility**: Each configuration section has one clear purpose
- **Open/Closed**: Extensible through interfaces, closed for modification
- **Liskov Substitution**: All implementations are interchangeable
- **Interface Segregation**: Focused, specific interfaces
- **Dependency Inversion**: Depend on abstractions, not concretions
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass, field
from enum import Enum


class ConfigurationSource(Enum):
    """Configuration source types."""
    ENVIRONMENT = "environment"
    FILE = "file"
    DATABASE = "database"
    DEFAULT = "default"


@dataclass
class ConfigurationValue:
    """A configuration value with metadata."""
    value: Any
    source: ConfigurationSource
    key: str
    description: str
    validation_errors: Optional[List[str]] = None
    
    def __post_init__(self):
        if self.validation_errors is None:
            self.validation_errors = []


@dataclass
class LiDARConfig:
    """LiDAR sensor configuration for collision avoidance."""
    # Enable/disable LiDAR collision avoidance
    enabled: bool = True

    # Sensor specifications
    num_rays: int = 110          # One ray per degree in 110° arc
    max_range: float = 3.0       # Maximum detection range in meters
    min_range: float = 0.2       # Minimum detection range in meters
    scan_frequency: float = 10.0 # Scan frequency in Hz
    field_of_view: float = 110.0 # Field of view in degrees

    # Collision avoidance parameters
    safety_distance: float = 0.8 # Safety distance for collision avoidance in meters

    # Performance tuning (for future optimization)
    enable_scan_caching: bool = True  # Cache scan results between frames
    scan_cache_ttl: float = 0.1      # Cache TTL in seconds (10Hz)


@dataclass
class RobotConfig:
    """Robot-specific configuration parameters."""
    # Basic robot parameters
    robot_id: str
    max_speed: float  # m/s
    position_tolerance: float  # meters
    control_frequency: float  # Hz
    motion_frequency: float  # Hz

    # Physical robot dimensions (meters)
    robot_width: float  # meters (diameter for circular robots)
    robot_height: float  # meters
    robot_length: float  # meters (only used for rectangular robots)

    # Coordinate system parameters
    cell_size: float  # meters per grid cell for coordinate system

    # Lane-based navigation parameters
    lane_tolerance: float  # meters from center-line
    corner_speed: float  # m/s in conflict boxes and turns
    bay_approach_speed: float  # m/s when approaching bays
    conflict_box_lock_timeout: float  # seconds
    conflict_box_heartbeat_interval: float  # seconds

    # Motion control parameters
    max_linear_velocity: float  # m/s
    max_angular_velocity: float  # rad/s
    movement_speed: float  # m/s

    # Wheel parameters (proportional to robot size)
    wheel_base_ratio: float  # wheel_base = robot_width * wheel_base_ratio
    wheel_radius_ratio: float  # wheel_radius = robot_height * wheel_radius_ratio

    # Derived wheel parameters (calculated from ratios and robot dimensions)
    @property
    def wheel_base(self) -> float:
        """Wheel base calculated from robot width and ratio."""
        return self.robot_width * self.wheel_base_ratio

    @property
    def wheel_radius(self) -> float:
        """Wheel radius calculated from robot height and ratio."""
        return self.robot_height * self.wheel_radius_ratio

    # Task execution parameters
    picking_duration: float  # seconds
    dropping_duration: float  # seconds

    # Safety parameters
    emergency_stop_distance: float  # meters
    stall_recovery_timeout: float  # seconds

    # LiDAR collision avoidance parameters
    lidar_config: LiDARConfig = field(default_factory=LiDARConfig)  # LiDAR sensor configuration

    # Startup behavior
    start_in_idle: bool = True


@dataclass
class BatteryConfig:
    """Battery management configuration parameters."""
    # Enable/disable enhanced battery management
    enabled: bool

    # Battery specifications
    capacity_wh: float  # Battery capacity in watt-hours

    # Consumption rates (watts)
    idle_consumption_rate: float  # Watts when idle
    moving_consumption_rate: float  # Base watts when moving
    carrying_consumption_rate: float  # Watts when carrying load
    stalled_consumption_rate: float  # Watts when stalled
    charging_rate: float  # Watts when charging (negative = charging)

    # Multipliers for dynamic consumption
    speed_consumption_multiplier: float  # Additional watts per m/s speed
    load_consumption_multiplier: float  # Additional watts when carrying

    # Safety and thresholds
    task_safety_margin: float  # Safety margin for task battery checks (0.0 to 1.0)
    low_battery_threshold: float  # Threshold for triggering charging (0.0 to 1.0)
    critical_battery_threshold: float  # Threshold for emergency actions (0.0 to 1.0)
    emergency_stop_threshold: float  # Threshold for emergency stop (0.0 to 1.0)

    # Efficiency parameters
    charging_efficiency: float  # Charging efficiency (0.0 to 1.0)
    self_discharge_rate: float  # Self-discharge rate per hour (0.0 to 1.0)


@dataclass
class ChargingStationConfig:
    """Charging station management configuration parameters."""
    # Enable/disable charging station management
    enabled: bool

    # Station discovery and allocation
    auto_discover_stations: bool  # Auto-discover stations from warehouse map
    station_lock_timeout: int  # Lock timeout in seconds
    station_heartbeat_interval: int  # Heartbeat interval in seconds

    # Station capabilities (all stations are similar)
    station_power_watts: float  # Station power output (watts)
    station_efficiency: float  # Charging efficiency (0.0 to 1.0)

    # Allocation preferences (simple: nearest available)
    max_search_distance: float  # Maximum distance to search for stations
    station_selection_timeout: float  # Timeout for station selection (seconds)

    # Maintenance and monitoring (kept for future enhancement)
    enable_maintenance_mode: bool  # Enable maintenance mode support
    maintenance_check_interval: int  # Interval to check for maintenance needs
    station_health_timeout: int  # Timeout for station health checks

    # Performance and scaling (kept for future enhancement)
    max_stations_per_robot: int  # Maximum stations a robot can reserve
    station_cache_ttl: int  # Cache TTL for station information
    concurrent_allocation_limit: int  # Maximum concurrent allocation attempts

    # Future enhancement hooks (reserved for advanced features)
    enable_smart_allocation: bool  # Reserved for future smart allocation algorithms
    enable_station_priorities: bool  # Reserved for future station priority system
    enable_load_balancing: bool  # Reserved for future load balancing features


@dataclass
class DatabaseConfig:
    """Database connection configuration."""
    host: str
    port: int
    database: str
    user: str
    password: Optional[str]
    pool_size: int
    connect_timeout: int
    application_name: str


@dataclass
class NavigationConfig:
    """Navigation system configuration."""
    # Path planning parameters
    path_planning_timeout: float  # seconds
    max_path_length: int  # maximum path segments
    replanning_threshold: float  # distance change to trigger replanning
    
    # Conflict box parameters
    conflict_box_lock_timeout: float  # seconds
    conflict_box_heartbeat_interval: float  # seconds
    conflict_box_priority_levels: int  # number of priority levels
    
    # Blocked cell parameters
    blocked_cell_timeout: float  # seconds
    blocked_cell_cleanup_interval: float  # seconds
    
    # Lane following parameters
    lane_tolerance: float  # meters
    velocity_smoothing_factor: float  # low-pass filter factor
    max_deviation: float  # meters
    max_consecutive_deviations: int


@dataclass
class TaskConfig:
    """Task processing configuration."""
    # Task execution parameters
    max_concurrent_tasks: int
    task_timeout: float  # seconds
    retry_attempts: int
    retry_delay: float  # seconds
    
    # Order processing parameters
    max_concurrent_orders: int
    order_processing_interval: float  # seconds
    inventory_allocation_timeout: float  # seconds
    
    # Bidding parameters
    bidding_timeout: float  # seconds
    min_bid_value: float
    max_bid_value: float
    distance_cost_factor: float
    battery_cost_factor: float

    # Automatic charging parameters
    auto_charging_enabled: bool  # Enable automatic charging when battery low
    charging_trigger_threshold: float  # Battery level threshold for triggering charging (0.0-1.0)
    battery_check_interval: float  # How often to check battery level (seconds)
    safe_interrupt_tasks: List[str]  # Task types that can be safely interrupted for charging
    prevent_duplicate_charging: bool  # Prevent creation of duplicate charging tasks
    # Availability during charging
    charging_availability_threshold: float  # Battery level above which robot can bid while charging (0.0-1.0)


@dataclass
class BidConfig:
    """Bid calculation configuration."""
    # Parallel processing
    max_parallel_workers: int  # Maximum threads for parallel bid calculation
    
    # Factor weights (0.0 to 1.0)
    distance_weight: float
    battery_weight: float
    workload_weight: float
    task_type_compatibility_weight: float
    robot_capabilities_weight: float
    time_urgency_weight: float
    conflict_box_availability_weight: float
    shelf_accessibility_weight: float
    
    # Factor enablement
    enable_distance_factor: bool
    enable_battery_factor: bool
    enable_workload_factor: bool
    enable_task_type_compatibility_factor: bool
    enable_robot_capabilities_factor: bool
    enable_time_urgency_factor: bool
    enable_conflict_box_availability_factor: bool
    enable_shelf_accessibility_factor: bool
    
    # Calculation parameters
    battery_threshold: float  # Minimum battery level for bidding (0.0 to 1.0)
    calculation_timeout: float  # Maximum time per bid calculation (seconds)
    max_distance_normalization: float  # Maximum distance for normalization (meters)
    
    # Performance tuning
    enable_parallel_calculation: bool
    enable_calculation_statistics: bool
    enable_factor_breakdown: bool


@dataclass
class SystemConfig:
    """System-wide configuration."""
    # Logging parameters
    log_level: str
    log_file: Optional[str]
    log_format: str
    
    # Performance parameters
    cache_ttl: float  # seconds
    connection_pool_size: int
    thread_pool_size: int
    
    # Monitoring parameters
    health_check_interval: float  # seconds
    metrics_collection_interval: float  # seconds
    alert_thresholds: Dict[str, float]


class ConfigurationError(Exception):
    """Raised when configuration operations fail."""
    pass


class IBusinessConfigurationProvider(ABC):
    """
    Interface for business configuration providers.

    This interface provides access to domain-specific configuration objects
    used by business logic components (robots, tasks, bidding, etc.).

    Responsibilities:
    - Provide domain-specific configuration objects (RobotConfig, BidConfig, etc.)
    - Load configuration from various sources
    - Validate configuration values
    - Support configuration updates and reloading
    """
    
    @abstractmethod
    def get_robot_config(self, robot_id: str) -> RobotConfig:
        """
        Get robot-specific configuration.
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            RobotConfig: Robot configuration
            
        Raises:
            ConfigurationError: If configuration cannot be loaded
        """
        pass
    
    @abstractmethod
    def get_database_config(self) -> DatabaseConfig:
        """
        Get database configuration.
        
        Returns:
            DatabaseConfig: Database configuration
            
        Raises:
            ConfigurationError: If configuration cannot be loaded
        """
        pass
    
    @abstractmethod
    def get_navigation_config(self) -> NavigationConfig:
        """
        Get navigation system configuration.
        
        Returns:
            NavigationConfig: Navigation configuration
            
        Raises:
            ConfigurationError: If configuration cannot be loaded
        """
        pass
    
    @abstractmethod
    def get_task_config(self) -> TaskConfig:
        """
        Get task processing configuration.
        
        Returns:
            TaskConfig: Task configuration
            
        Raises:
            ConfigurationError: If configuration cannot be loaded
        """
        pass
    
    @abstractmethod
    def get_bid_config(self) -> BidConfig:
        """
        Get bid calculation configuration.
        
        Returns:
            BidConfig: Bid calculation configuration
            
        Raises:
            ConfigurationError: If configuration cannot be loaded
        """
        pass
    
    @abstractmethod
    def get_system_config(self) -> SystemConfig:
        """
        Get system-wide configuration.
        
        Returns:
            SystemConfig: System configuration
            
        Raises:
            ConfigurationError: If configuration cannot be loaded
        """
        pass
    
    @abstractmethod
    def get_value(self, key: str, default: Any = None) -> ConfigurationValue:
        """
        Get a specific configuration value.
        
        Args:
            key: Configuration key (e.g., "robot.max_speed")
            default: Default value if not found
            
        Returns:
            ConfigurationValue: Configuration value with metadata
        """
        pass
    
    @abstractmethod
    def set_value(self, key: str, value: Any, source: ConfigurationSource = ConfigurationSource.DEFAULT) -> None:
        """
        Set a configuration value.
        
        Args:
            key: Configuration key
            value: Configuration value
            source: Source of the configuration
            
        Raises:
            ConfigurationError: If value cannot be set
        """
        pass
    
    @abstractmethod
    def reload(self) -> None:
        """
        Reload configuration from sources.
        
        Raises:
            ConfigurationError: If reload fails
        """
        pass
    
    @abstractmethod
    def validate(self) -> List[str]:
        """
        Validate all configuration values.
        
        Returns:
            List[str]: List of validation errors (empty if valid)
        """
        pass
    
    @property
    @abstractmethod
    def errors(self) -> List[str]:
        """
        Get current configuration validation errors.
        
        Returns:
            List[str]: List of validation errors (empty if valid)
        """
        pass


class IConfigurationValidator(ABC):
    """
    Interface for configuration validation.
    
    Responsibilities:
    - Validate configuration values
    - Provide detailed error messages
    - Support custom validation rules
    """
    
    @abstractmethod
    def validate_robot_config(self, config: RobotConfig) -> List[str]:
        """
        Validate robot configuration.
        
        Args:
            config: Robot configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        pass
    
    @abstractmethod
    def validate_database_config(self, config: DatabaseConfig) -> List[str]:
        """
        Validate database configuration.
        
        Args:
            config: Database configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        pass
    
    @abstractmethod
    def validate_navigation_config(self, config: NavigationConfig) -> List[str]:
        """
        Validate navigation configuration.
        
        Args:
            config: Navigation configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        pass
    
    @abstractmethod
    def validate_task_config(self, config: TaskConfig) -> List[str]:
        """
        Validate task configuration.
        
        Args:
            config: Task configuration to validate
            
        Returns:
            List[str]: List of validation errors (empty if valid)
        """
        pass
    
    @abstractmethod
    def validate_bid_config(self, config: BidConfig) -> List[str]:
        """
        Validate bid calculation configuration.
        
        Args:
            config: Bid configuration to validate
            
        Returns:
            List[str]: List of validation errors (empty if valid)
        """
        pass
    
    @abstractmethod
    def validate_system_config(self, config: SystemConfig) -> List[str]:
        """
        Validate system configuration.
        
        Args:
            config: System configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        pass


class IConfigurationSource(ABC):
    """
    Interface for configuration sources.
    
    Responsibilities:
    - Load configuration from specific sources
    - Support different data formats
    - Handle source-specific errors
    """
    
    @abstractmethod
    def load_configuration(self) -> Dict[str, Any]:
        """
        Load configuration from source.
        
        Returns:
            Dict[str, Any]: Configuration data
            
        Raises:
            ConfigurationError: If loading fails
        """
        pass
    
    @abstractmethod
    def get_source_type(self) -> ConfigurationSource:
        """
        Get the source type.
        
        Returns:
            ConfigurationSource: Source type
        """
        pass
    
    @abstractmethod
    def is_available(self) -> bool:
        """
        Check if source is available.
        
        Returns:
            bool: True if source is available
        """
        pass 