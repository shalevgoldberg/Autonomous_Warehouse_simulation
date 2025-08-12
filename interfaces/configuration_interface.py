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
from dataclasses import dataclass
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
class RobotConfig:
    """Robot-specific configuration parameters."""
    # Basic robot parameters
    robot_id: str
    max_speed: float  # m/s
    position_tolerance: float  # meters
    control_frequency: float  # Hz
    motion_frequency: float  # Hz
    
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
    wheel_base: float  # meters between wheels
    wheel_radius: float  # meters
    
    # Task execution parameters
    picking_duration: float  # seconds
    dropping_duration: float  # seconds
    charging_threshold: float  # battery level (0.0 to 1.0)
    
    # Safety parameters
    emergency_stop_distance: float  # meters
    stall_recovery_timeout: float  # seconds


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


class IConfigurationProvider(ABC):
    """
    Interface for configuration providers.
    
    Responsibilities:
    - Load configuration from various sources
    - Validate configuration values
    - Provide type-safe access to configuration
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