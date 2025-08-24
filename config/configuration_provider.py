"""
Configuration Provider Implementation - Centralized configuration composition root.

This provider loads, merges, and validates configuration from all sources (env, file, defaults),
providing type-safe access and supporting reloads and validation.
"""
import threading
from typing import Dict, Any, Optional, List
from interfaces.configuration_interface import (
    IConfigurationProvider, IConfigurationSource, IConfigurationValidator,
    RobotConfig, DatabaseConfig, NavigationConfig, TaskConfig, BidConfig, SystemConfig,
    ConfigurationSource, ConfigurationValue, ConfigurationError
)
from config.configuration_sources import (
    EnvironmentConfigurationSource, FileConfigurationSource, DefaultConfigurationSource
)
from config.configuration_validator import ConfigurationValidatorImpl


class ConfigurationProvider(IConfigurationProvider):
    """
    Centralized configuration provider that merges all sources and validates configuration.
    Thread-safe and supports reloads.
    """
    def __init__(self,
                 config_file: Optional[str] = None,
                 config_file_format: str = "auto",
                 env_prefix: str = "WAREHOUSE_",
                 validator: Optional[IConfigurationValidator] = None):
        self._lock = threading.RLock()
        self._sources: List[IConfigurationSource] = []
        self._config: Dict[str, Any] = {}
        self._overrides: Dict[str, Any] = {}  # <-- Runtime overrides
        self._validator = validator or ConfigurationValidatorImpl()
        self._errors: List[str] = []
        self._init_sources(config_file, config_file_format, env_prefix)
        self.reload()

    def _init_sources(self, config_file, config_file_format, env_prefix):
        # Order: env > file > defaults
        self._sources = [
            EnvironmentConfigurationSource(prefix=env_prefix)
        ]
        if config_file:
            self._sources.append(FileConfigurationSource(config_file, config_file_format))
        self._sources.append(DefaultConfigurationSource())

    def reload(self) -> None:
        """Reload configuration from all sources and validate."""
        with self._lock:
            merged = {}
            for source in reversed(self._sources):  # Defaults first, env last
                try:
                    conf = source.load_configuration()
                    merged.update(conf)
                except ConfigurationError as e:
                    # Only log, don't fail
                    pass
            # Apply runtime overrides last
            merged.update(self._overrides)
            self._config = merged
            self._errors = self.validate()

    def validate(self) -> List[str]:
        """Validate all configuration sections and return errors."""
        errors = []
        try:
            errors.extend(self._validator.validate_robot_config(self.get_robot_config(self._config.get('robot.robot_id', 'robot_1'))))
            errors.extend(self._validator.validate_database_config(self.get_database_config()))
            errors.extend(self._validator.validate_navigation_config(self.get_navigation_config()))
            errors.extend(self._validator.validate_task_config(self.get_task_config()))
            errors.extend(self._validator.validate_system_config(self.get_system_config()))
        except Exception as e:
            errors.append(f"Validation error: {e}")
        return errors

    def get_robot_config(self, robot_id: str) -> RobotConfig:
        c = {**self._config, **self._overrides}  # Always check overrides first
        return RobotConfig(
            robot_id=robot_id,
            max_speed=c.get("robot.max_speed", 1.0),
            position_tolerance=c.get("robot.position_tolerance", 0.1),
            control_frequency=c.get("robot.control_frequency", 10.0),
            motion_frequency=c.get("robot.motion_frequency", 100.0),
            cell_size=c.get("robot.cell_size", 0.5),  # Match warehouse map cell size
            lane_tolerance=c.get("robot.lane_tolerance", 0.2),  # Should be 0.1-0.3m for production use
            corner_speed=c.get("robot.corner_speed", 0.3),
            bay_approach_speed=c.get("robot.bay_approach_speed", 0.2),
            conflict_box_lock_timeout=c.get("robot.conflict_box_lock_timeout", 30.0),
            conflict_box_heartbeat_interval=c.get("robot.conflict_box_heartbeat_interval", 5.0),
            max_linear_velocity=c.get("robot.max_linear_velocity", 2.0),
            max_angular_velocity=c.get("robot.max_angular_velocity", 1.0),  # Reduced from 2.0 to 1.0
            movement_speed=c.get("robot.movement_speed", 1.5),
            wheel_base=c.get("robot.wheel_base", 0.3),
            wheel_radius=c.get("robot.wheel_radius", 0.12),  # Increased from 0.05 to 0.12 (more realistic)
            picking_duration=c.get("robot.picking_duration", 5.0),
            dropping_duration=c.get("robot.dropping_duration", 3.0),
            charging_threshold=c.get("robot.charging_threshold", 0.2),
            emergency_stop_distance=c.get("robot.emergency_stop_distance", 0.5),
            stall_recovery_timeout=c.get("robot.stall_recovery_timeout", 10.0),
        )

    def get_database_config(self) -> DatabaseConfig:
        c = {**self._config, **self._overrides}
        return DatabaseConfig(
            host=c.get("database.host", "localhost"),
            port=c.get("database.port", 5432),
            database=c.get("database.database", "warehouse_sim"),
            user=c.get("database.user", "postgres"),
            password=c.get("database.password", None),
            pool_size=c.get("database.pool_size", 10),
            connect_timeout=c.get("database.connect_timeout", 10),
            application_name=c.get("database.application_name", "warehouse_simulation"),
        )

    def get_navigation_config(self) -> NavigationConfig:
        c = {**self._config, **self._overrides}
        return NavigationConfig(
            path_planning_timeout=c.get("navigation.path_planning_timeout", 30.0),
            max_path_length=c.get("navigation.max_path_length", 100),
            replanning_threshold=c.get("navigation.replanning_threshold", 0.5),
            conflict_box_lock_timeout=c.get("navigation.conflict_box_lock_timeout", 30.0),
            conflict_box_heartbeat_interval=c.get("navigation.conflict_box_heartbeat_interval", 5.0),
            conflict_box_priority_levels=c.get("navigation.conflict_box_priority_levels", 10),
            blocked_cell_timeout=c.get("navigation.blocked_cell_timeout", 300.0),
            blocked_cell_cleanup_interval=c.get("navigation.blocked_cell_cleanup_interval", 60.0),
            lane_tolerance=c.get("navigation.lane_tolerance", 3.0),  # DEMO ONLY: Too soft for production! Should be 0.1-0.3m
            velocity_smoothing_factor=c.get("navigation.velocity_smoothing_factor", 0.1),
            max_deviation=c.get("navigation.max_deviation", 2.0),
            max_consecutive_deviations=c.get("navigation.max_consecutive_deviations", 3),
        )

    def get_task_config(self) -> TaskConfig:
        c = {**self._config, **self._overrides}
        return TaskConfig(
            max_concurrent_tasks=c.get("task.max_concurrent_tasks", 5),
            task_timeout=c.get("task.task_timeout", 300.0),
            retry_attempts=c.get("task.retry_attempts", 3),
            retry_delay=c.get("task.retry_delay", 5.0),
            max_concurrent_orders=c.get("task.max_concurrent_orders", 10),
            order_processing_interval=c.get("task.order_processing_interval", 5.0),
            inventory_allocation_timeout=c.get("task.inventory_allocation_timeout", 30.0),
            bidding_timeout=c.get("task.bidding_timeout", 10.0),
            min_bid_value=c.get("task.min_bid_value", 0.1),
            max_bid_value=c.get("task.max_bid_value", 1000.0),
            distance_cost_factor=c.get("task.distance_cost_factor", 10.0),
            battery_cost_factor=c.get("task.battery_cost_factor", 5.0),
        )

    def get_bid_config(self) -> BidConfig:
        c = {**self._config, **self._overrides}
        return BidConfig(
            # Parallel processing
            max_parallel_workers=c.get("bid.max_parallel_workers", 4),
            
            # Factor weights
            distance_weight=c.get("bid.distance_weight", 0.4),
            battery_weight=c.get("bid.battery_weight", 0.3),
            workload_weight=c.get("bid.workload_weight", 0.2),
            task_type_compatibility_weight=c.get("bid.task_type_compatibility_weight", 0.1),
            robot_capabilities_weight=c.get("bid.robot_capabilities_weight", 0.0),
            time_urgency_weight=c.get("bid.time_urgency_weight", 0.0),
            conflict_box_availability_weight=c.get("bid.conflict_box_availability_weight", 0.0),
            shelf_accessibility_weight=c.get("bid.shelf_accessibility_weight", 0.0),
            
            # Factor enablement
            enable_distance_factor=c.get("bid.enable_distance_factor", True),
            enable_battery_factor=c.get("bid.enable_battery_factor", True),
            enable_workload_factor=c.get("bid.enable_workload_factor", True),
            enable_task_type_compatibility_factor=c.get("bid.enable_task_type_compatibility_factor", True),
            enable_robot_capabilities_factor=c.get("bid.enable_robot_capabilities_factor", False),
            enable_time_urgency_factor=c.get("bid.enable_time_urgency_factor", False),
            enable_conflict_box_availability_factor=c.get("bid.enable_conflict_box_availability_factor", False),
            enable_shelf_accessibility_factor=c.get("bid.enable_shelf_accessibility_factor", False),
            
            # Calculation parameters
            battery_threshold=c.get("bid.battery_threshold", 0.2),
            calculation_timeout=c.get("bid.calculation_timeout", 1.0),
            max_distance_normalization=c.get("bid.max_distance_normalization", 20.0),
            
            # Performance tuning
            enable_parallel_calculation=c.get("bid.enable_parallel_calculation", True),
            enable_calculation_statistics=c.get("bid.enable_calculation_statistics", True),
            enable_factor_breakdown=c.get("bid.enable_factor_breakdown", True),
        )

    def get_system_config(self) -> SystemConfig:
        c = {**self._config, **self._overrides}
        return SystemConfig(
            log_level=c.get("system.log_level", "INFO"),
            log_file=c.get("system.log_file", None),
            log_format=c.get("system.log_format", "%(asctime)s - %(name)s - %(levelname)s - %(message)s"),
            cache_ttl=c.get("system.cache_ttl", 300.0),
            connection_pool_size=c.get("system.connection_pool_size", 10),
            thread_pool_size=c.get("system.thread_pool_size", 4),
            health_check_interval=c.get("system.health_check_interval", 30.0),
            metrics_collection_interval=c.get("system.metrics_collection_interval", 60.0),
            alert_thresholds=c.get("system.alert_thresholds", {"battery_low": 0.2, "task_timeout": 300.0, "error_rate": 0.1}),
        )

    def get_value(self, key: str, default: Any = None) -> ConfigurationValue:
        v = self._overrides.get(key, self._config.get(key, default))
        # For demo, assume all values come from merged source
        return ConfigurationValue(
            value=v,
            source=ConfigurationSource.DEFAULT,  # Could be improved to track source
            key=key,
            description=f"Config value for {key}",
            validation_errors=[]
        )

    def set_value(self, key: str, value: Any, source: ConfigurationSource = ConfigurationSource.DEFAULT) -> None:
        with self._lock:
            self._overrides[key] = value
            self._errors = self.validate()

    @property
    def errors(self) -> List[str]:
        return self._errors.copy() 