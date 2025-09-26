"""
Configuration Sources - Load configuration from various sources.

This module provides implementations for loading configuration from different sources
including environment variables, configuration files, and database.
"""
import os
import json
import yaml
from typing import Dict, Any, Optional
from pathlib import Path

from interfaces.configuration_interface import (
    IConfigurationSource, ConfigurationSource, ConfigurationError
)


class EnvironmentConfigurationSource(IConfigurationSource):
    """
    Configuration source that loads from environment variables.
    
    Environment variable naming convention:
    - WAREHOUSE_<SECTION>_<KEY> (e.g., WAREHOUSE_ROBOT_MAX_SPEED)
    - WAREHOUSE_<KEY> for system-wide settings
    """
    
    def __init__(self, prefix: str = "WAREHOUSE_"):
        """
        Initialize environment configuration source.
        
        Args:
            prefix: Environment variable prefix
        """
        self.prefix = prefix
    
    def load_configuration(self) -> Dict[str, Any]:
        """
        Load configuration from environment variables.
        
        Returns:
            Dict[str, Any]: Configuration data
            
        Raises:
            ConfigurationError: If loading fails
        """
        try:
            config = {}
            
            # Load all environment variables with the prefix
            for key, value in os.environ.items():
                if key.startswith(self.prefix):
                    # Convert WAREHOUSE_ROBOT_MAX_SPEED to robot.max_speed
                    config_key = self._convert_env_key_to_config_key(key)
                    config_value = self._parse_env_value(value)
                    config[config_key] = config_value
            
            return config
            
        except Exception as e:
            raise ConfigurationError(f"Failed to load environment configuration: {e}")
    
    def get_source_type(self) -> ConfigurationSource:
        """Get the source type."""
        return ConfigurationSource.ENVIRONMENT
    
    def is_available(self) -> bool:
        """Check if source is available."""
        return True  # Environment is always available
    
    def _convert_env_key_to_config_key(self, env_key: str) -> str:
        """
        Convert environment variable key to configuration key.
        
        Args:
            env_key: Environment variable key (e.g., WAREHOUSE_ROBOT_MAX_SPEED)
            
        Returns:
            str: Configuration key (e.g., robot.max_speed)
        """
        # Remove prefix and convert to lowercase
        key = env_key[len(self.prefix):].lower()
        
        # Split by underscore and join with dots
        parts = key.split('_')
        if len(parts) >= 2:
            section = parts[0]
            field = '_'.join(parts[1:])
            return f"{section}.{field}"
        else:
            return key
    
    def _parse_env_value(self, value: str) -> Any:
        """
        Parse environment variable value to appropriate type.
        
        Args:
            value: Environment variable value
            
        Returns:
            Any: Parsed value
        """
        # Try to parse as JSON first
        try:
            return json.loads(value)
        except (json.JSONDecodeError, ValueError):
            pass
        
        # Try to parse as number
        try:
            if '.' in value:
                return float(value)
            else:
                return int(value)
        except ValueError:
            pass
        
        # Try to parse as boolean
        if value.lower() in ('true', 'false'):
            return value.lower() == 'true'
        
        # Return as string
        return value


class FileConfigurationSource(IConfigurationSource):
    """
    Configuration source that loads from configuration files.
    
    Supports JSON and YAML formats.
    """
    
    def __init__(self, file_path: str, file_format: str = "auto"):
        """
        Initialize file configuration source.
        
        Args:
            file_path: Path to configuration file
            file_format: File format ("json", "yaml", or "auto")
        """
        self.file_path = Path(file_path)
        self.file_format = file_format
    
    def load_configuration(self) -> Dict[str, Any]:
        """
        Load configuration from file.
        
        Returns:
            Dict[str, Any]: Configuration data
            
        Raises:
            ConfigurationError: If loading fails
        """
        try:
            if not self.file_path.exists():
                raise ConfigurationError(f"Configuration file not found: {self.file_path}")
            
            # Determine file format
            format_type = self._determine_format()
            
            # Load based on format
            if format_type == "json":
                return self._load_json()
            elif format_type == "yaml":
                return self._load_yaml()
            else:
                raise ConfigurationError(f"Unsupported file format: {format_type}")
                
        except Exception as e:
            raise ConfigurationError(f"Failed to load file configuration: {e}")
    
    def get_source_type(self) -> ConfigurationSource:
        """Get the source type."""
        return ConfigurationSource.FILE
    
    def is_available(self) -> bool:
        """Check if source is available."""
        return self.file_path.exists()
    
    def _determine_format(self) -> str:
        """
        Determine file format based on extension or content.
        
        Returns:
            str: File format ("json" or "yaml")
        """
        if self.file_format != "auto":
            return self.file_format
        
        # Check file extension
        suffix = self.file_path.suffix.lower()
        if suffix == ".json":
            return "json"
        elif suffix in (".yml", ".yaml"):
            return "yaml"
        
        # Try to detect from content
        try:
            with open(self.file_path, 'r') as f:
                first_line = f.readline().strip()
                if first_line.startswith('{'):
                    return "json"
                elif first_line.startswith('-') or ':' in first_line:
                    return "yaml"
        except Exception:
            pass
        
        # Default to JSON
        return "json"
    
    def _load_json(self) -> Dict[str, Any]:
        """Load JSON configuration file."""
        with open(self.file_path, 'r') as f:
            return json.load(f)
    
    def _load_yaml(self) -> Dict[str, Any]:
        """Load YAML configuration file."""
        try:
            import yaml
            with open(self.file_path, 'r') as f:
                return yaml.safe_load(f)
        except ImportError:
            raise ConfigurationError("PyYAML is required for YAML configuration files")


class DatabaseConfigurationSource(IConfigurationSource):
    """
    Configuration source that loads from database.
    
    This is a placeholder for future database-based configuration.
    """
    
    def __init__(self, connection_string: str):
        """
        Initialize database configuration source.
        
        Args:
            connection_string: Database connection string
        """
        self.connection_string = connection_string
    
    def load_configuration(self) -> Dict[str, Any]:
        """
        Load configuration from database.
        
        Returns:
            Dict[str, Any]: Configuration data
            
        Raises:
            ConfigurationError: If loading fails
        """
        # TODO: Implement database configuration loading
        raise ConfigurationError("Database configuration source not yet implemented")
    
    def get_source_type(self) -> ConfigurationSource:
        """Get the source type."""
        return ConfigurationSource.DATABASE
    
    def is_available(self) -> bool:
        """Check if source is available."""
        # TODO: Implement database availability check
        return False


class DefaultConfigurationSource(IConfigurationSource):
    """
    Configuration source that provides default values.
    
    This source provides sensible defaults for all configuration parameters.
    """
    
    def __init__(self):
        """Initialize default configuration source."""
        self._defaults = self._create_default_configuration()
    
    def load_configuration(self) -> Dict[str, Any]:
        """
        Load default configuration.

        Returns:
            Dict[str, Any]: Default configuration data
        """
        return self._defaults.copy()
    
    def get_source_type(self) -> ConfigurationSource:
        """Get the source type."""
        return ConfigurationSource.DEFAULT
    
    def is_available(self) -> bool:
        """Check if source is available."""
        return True  # Defaults are always available
    
    def _create_default_configuration(self) -> Dict[str, Any]:
        """
        Create default configuration values.
        
        Returns:
            Dict[str, Any]: Default configuration
        """
        return {
            # Robot configuration
            "robot.robot_id": "robot_1",
            "robot.max_speed": 1.0,
            "robot.position_tolerance": 0.1,
            "robot.control_frequency": 10.0,
            "robot.motion_frequency": 100.0,

            # Physical robot dimensions (0.25m diameter target)
            "robot.robot_width": 0.25,   # meters (diameter for circular robots)
            "robot.robot_height": 0.12,  # meters
            "robot.robot_length": 0.25,  # meters (same as width for circular)

            "robot.lane_tolerance": 0.2,  # Should be 0.1-0.3m for production use
            "robot.corner_speed": 0.3,
            "robot.bay_approach_speed": 0.2,
            "robot.conflict_box_lock_timeout": 30.0,
            "robot.conflict_box_heartbeat_interval": 5.0,
            "robot.max_linear_velocity": 2.0,
            "robot.max_angular_velocity": 2.0,
            "robot.movement_speed": 1.5,

            # Wheel parameters (proportional to robot size)
            "robot.wheel_base_ratio": 0.8,   # wheel_base = robot_width * 0.8
            "robot.wheel_radius_ratio": 0.25, # wheel_radius = robot_height * 0.25
            "robot.picking_duration": 1.0,
            "robot.dropping_duration": 2.0,
            "robot.charging_threshold": 0.2,
            "robot.emergency_stop_distance": 0.5,
            "robot.stall_recovery_timeout": 10.0,

            # Battery management configuration - TEMPORARILY MODIFIED FOR DEMO
            "battery.enabled": True,  # Enable enhanced battery management
            "battery.capacity_wh": 1000.0,  # Battery capacity in watt-hours
            "battery.idle_consumption_rate": 8000.0,  # TEMP: 50kW when idle (10,000x normal)
            "battery.moving_consumption_rate": 80000.0,  # TEMP: 500kW when moving (10,000x normal)
            "battery.carrying_consumption_rate": 150000.0,  # TEMP: 800kW when carrying (10,000x normal)
            "battery.stalled_consumption_rate": 15000.0,  # TEMP: 100kW when stalled (10,000x normal)
            "battery.charging_rate": -50000.0,  # TEMP: 500W when charging (faster for testing)
            "battery.speed_consumption_multiplier": 100.0,  # TEMP: 1000W per m/s speed (100x normal)
            "battery.load_consumption_multiplier": 3000.0,  # TEMP: 3000W when carrying (100x normal)
            "battery.task_safety_margin": 0.15,  # 15% safety margin for task battery checks
            "battery.low_battery_threshold": 0.2,  
            "battery.critical_battery_threshold": 0.6,  # TEMP: 60% emergency actions (higher for demo visibility)
            "battery.emergency_stop_threshold": 0.01,  
            "battery.charging_efficiency": 0.95,  # 95% charging efficiency
            "battery.self_discharge_rate": 0.1,  # TEMP: 10% per hour self-discharge (100x normal)

            # Charging station management configuration
            "charging_station.enabled": True,  # Enable charging station management
            "charging_station.auto_discover_stations": True,  # Auto-discover from warehouse map
            "charging_station.station_lock_timeout": 600,  # 10 minutes lock timeout
            "charging_station.station_heartbeat_interval": 30,  # 30 seconds heartbeat
            #"charging_station.station_power_watts": 150.0,  # Station power output (watts)
            #"charging_station.station_efficiency": 0.95,  # Charging efficiency (0.95 = 95%)
            # "charging_station.max_search_distance": 50.0,  # Max search distance (meters) - UNUSED
            # "charging_station.station_selection_timeout": 10.0,  # Selection timeout (seconds) - UNUSED
            # "charging_station.enable_maintenance_mode": True,  # Enable maintenance support - UNUSED
            # "charging_station.maintenance_check_interval": 300,  # 5 minutes check interval - UNUSED
            # "charging_station.station_health_timeout": 60,  # 1 minute health timeout - UNUSED
            # "charging_station.max_stations_per_robot": 1,  # Max stations per robot - UNUSED
            # "charging_station.station_cache_ttl": 60,  # 1 minute cache TTL - UNUSED
            # "charging_station.concurrent_allocation_limit": 10,  # Max concurrent allocations - UNUSED
            # "charging_station.enable_smart_allocation": False,  # Reserved for future enhancement - UNUSED
            # "charging_station.enable_station_priorities": False,  # Reserved for future enhancement - UNUSED
            # "charging_station.enable_load_balancing": False,  # Reserved for future enhancement - UNUSED

            # Automatic charging configuration
            "charging.auto_trigger_enabled": True,           # Enable automatic charging
            "charging.trigger_threshold": 0.3,             # TEMP: 95% battery threshold
            "charging.battery_check_interval": 1.0,          # Check battery every 1 second
            "charging.safe_interrupt_tasks": ["idle_park", "move_to_position"],  # Tasks that can be interrupted
            "charging.prevent_duplicates": True,             # Prevent duplicate charging tasks

            # Idle/Wander configuration
            "idle.wander.enabled": True,
            "idle.wander.retry_interval_seconds": 1.5,
            "idle.wander.min_target_distance_m": 1.0,

            # Startup behavior
            "robot.start_in_idle": True,

            # Database configuration (uses environment variables with fallbacks)
            "database.host": os.getenv("WAREHOUSE_DB_HOST", "localhost"),
            "database.port": int(os.getenv("WAREHOUSE_DB_PORT", "5432")),
            "database.database": os.getenv("WAREHOUSE_DB_NAME", "warehouse_sim"),
            "database.user": os.getenv("WAREHOUSE_DB_USER", "postgres"),
            "database.password": os.getenv("WAREHOUSE_DB_PASSWORD"),
            "database.pool_size": int(os.getenv("WAREHOUSE_DB_POOL_SIZE", "10")),
            "database.connect_timeout": int(os.getenv("WAREHOUSE_DB_CONNECT_TIMEOUT", "10")),
            "database.application_name": os.getenv("WAREHOUSE_DB_APP_NAME", "warehouse_simulation"),
            
            # Navigation configuration
            "navigation.path_planning_timeout": 30.0,
            "navigation.max_path_length": 100,
            "navigation.replanning_threshold": 0.5,
            "navigation.conflict_box_lock_timeout": 30.0,
            "navigation.conflict_box_heartbeat_interval": 5.0,
            "navigation.conflict_box_priority_levels": 10,
            "navigation.blocked_cell_timeout": 300.0,
            "navigation.blocked_cell_cleanup_interval": 60.0,
            "navigation.lane_tolerance": 0.2,  # Should be 0.1-0.3m for production use
            "navigation.velocity_smoothing_factor": 0.1,
            "navigation.max_deviation": 2.0,
            "navigation.max_consecutive_deviations": 3,
            
            # Task configuration
            "task.max_concurrent_tasks": 5,
            "task.task_timeout": 300.0,
            "task.retry_attempts": 3,
            "task.retry_delay": 5.0,
            "task.max_concurrent_orders": 10,
            "task.order_processing_interval": 5.0,
            "task.inventory_allocation_timeout": 30.0,
            "task.bidding_timeout": 10.0,
            "task.min_bid_value": 0.1,
            "task.max_bid_value": 1000.0,
            "task.distance_cost_factor": 10.0,
            "task.battery_cost_factor": 5.0,

            # Physics configuration
            "physics.mode": "mujoco_authoritative",  # mujoco_authoritative (default) | kinematic_guard
            "physics.gravity": 0.0,  # Zero gravity for deterministic behavior
            "physics.time_step": 0.001,  # 1 kHz physics step
            "physics.velocity_damping": 0.1,  # Optional velocity damping for mujoco_authoritative mode
            # Robot physical parameters for differential drive kinematics
            "robot.wheel_radius": 0.05,  # meters - radius of robot wheels
            "robot.wheel_base": 0.3,     # meters - distance between left and right wheels

            # Collision Avoidance configuration
            "collision_avoidance.enabled": True,
            #"collision_avoidance.fov_degrees": 50.0,
            #"collision_avoidance.num_beams": 50,
            "collision_avoidance.braking_cone_degrees": 10.0,
            "collision_avoidance.lateral_guard_margin": 0.10,
            "collision_avoidance.static_stop_distance": 0.08,
            "collision_avoidance.dynamic_stop_distance": 0.30,
            "collision_avoidance.dynamic_caution_distance": 0.50,
            "collision_avoidance.headway_time_gap": 0.8,
            "collision_avoidance.yaw_bias_max": 0.1,

            # LiDAR configuration (for collision avoidance)
            "lidar.enabled": True,
            "lidar.num_rays": 50,
            "lidar.max_range": 1.5,
            "lidar.min_range": 0.1,
            "lidar.scan_frequency": 10.0,
            "lidar.field_of_view": 50.0,
            "lidar.safety_distance": 0.5,
            "lidar.enable_scan_caching": True,
            "lidar.scan_cache_ttl": 0.1,


            # Appearance configuration
            "appearance.enabled": True,
            "appearance.carry_color": [0.0, 1.0, 0.0, 1.0],  # Bright GREEN RGBA for carrying state
            "appearance.normal_color": [0.8, 0.2, 0.2, 1.0],  # Default robot color (red)

            # Bid configuration
            "bid.max_parallel_workers": 4,
            "bid.distance_weight": 0.4,
            "bid.battery_weight": 0.3,
            "bid.workload_weight": 0.2,
            "bid.task_type_compatibility_weight": 0.1,
            "bid.robot_capabilities_weight": 0.0,
            "bid.time_urgency_weight": 0.0,
            "bid.conflict_box_availability_weight": 0.0,
            "bid.shelf_accessibility_weight": 0.0,
            "bid.enable_distance_factor": True,
            "bid.enable_battery_factor": True,
            "bid.enable_workload_factor": True,
            "bid.enable_task_type_compatibility_factor": True,
            "bid.enable_robot_capabilities_factor": False,
            "bid.enable_time_urgency_factor": False,
            "bid.enable_conflict_box_availability_factor": False,
            "bid.enable_shelf_accessibility_factor": False,
            "bid.battery_threshold": 0.2,
            "bid.calculation_timeout": 1.0,
            "bid.max_distance_normalization": 20.0,
            "bid.enable_parallel_calculation": True,
            "bid.enable_calculation_statistics": True,
            "bid.enable_factor_breakdown": True,
            
            # System configuration
            "system.log_level": "INFO",
            "system.log_file": None,
            "system.log_format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            "system.cache_ttl": 300.0,
            "system.connection_pool_size": 10,
            "system.thread_pool_size": 4,
            "system.health_check_interval": 30.0,
            "system.metrics_collection_interval": 60.0,
            "system.alert_thresholds": {
                "battery_low": 0.2,
                "task_timeout": 300.0,
                "error_rate": 0.1
            },
            # KPI reporting
            "kpi.export_csv_enabled": True,
            "kpi.export_csv_path": "kpi_results.csv",

 

            # Demo configuration
            "demo.random_robot_placement": True,  # True for random placement, False for manual
            "demo.robot_placement_min_distance": 0.5  # Minimum distance between robots in meters
        } 