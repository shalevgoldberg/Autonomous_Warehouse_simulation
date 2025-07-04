"""
Configuration Validator Implementation - Comprehensive configuration validation.

This implementation provides thorough validation of all configuration parameters
with detailed error messages and support for custom validation rules.
"""
import re
from typing import List, Dict, Any
from dataclasses import dataclass

from interfaces.configuration_interface import (
    IConfigurationValidator, RobotConfig, DatabaseConfig, NavigationConfig,
    TaskConfig, SystemConfig, ConfigurationError
)


@dataclass
class ValidationRule:
    """A validation rule with condition and error message."""
    condition: callable
    error_message: str
    field_name: str


class ConfigurationValidatorImpl(IConfigurationValidator):
    """
    Comprehensive configuration validator implementation.
    
    Responsibilities:
    - Validate all configuration sections
    - Provide detailed error messages
    - Support custom validation rules
    - Ensure configuration consistency
    """
    
    def __init__(self):
        """Initialize validator with validation rules."""
        self._robot_rules = self._create_robot_validation_rules()
        self._database_rules = self._create_database_validation_rules()
        self._navigation_rules = self._create_navigation_validation_rules()
        self._task_rules = self._create_task_validation_rules()
        self._system_rules = self._create_system_validation_rules()
    
    def validate_robot_config(self, config: RobotConfig) -> List[str]:
        """
        Validate robot configuration with comprehensive checks.
        
        Args:
            config: Robot configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        errors = []
        
        # Apply all validation rules
        for rule in self._robot_rules:
            if not rule.condition(config):
                errors.append(f"Robot.{rule.field_name}: {rule.error_message}")
        
        # Cross-field validation
        if config.corner_speed > config.max_speed:
            errors.append("Robot.corner_speed: Corner speed cannot exceed max speed")
        
        if config.bay_approach_speed > config.max_speed:
            errors.append("Robot.bay_approach_speed: Bay approach speed cannot exceed max speed")
        
        if config.movement_speed > config.max_linear_velocity:
            errors.append("Robot.movement_speed: Movement speed cannot exceed max linear velocity")
        
        if config.control_frequency <= 0:
            errors.append("Robot.control_frequency: Control frequency must be positive")
        
        if config.motion_frequency <= 0:
            errors.append("Robot.motion_frequency: Motion frequency must be positive")
        
        if config.motion_frequency < config.control_frequency:
            errors.append("Robot.motion_frequency: Motion frequency must be >= control frequency")
        
        return errors
    
    def validate_database_config(self, config: DatabaseConfig) -> List[str]:
        """
        Validate database configuration.
        
        Args:
            config: Database configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        errors = []
        
        # Apply all validation rules
        for rule in self._database_rules:
            if not rule.condition(config):
                errors.append(f"Database.{rule.field_name}: {rule.error_message}")
        
        # Cross-field validation
        if config.port < 1 or config.port > 65535:
            errors.append("Database.port: Port must be between 1 and 65535")
        
        if config.pool_size < 1:
            errors.append("Database.pool_size: Pool size must be at least 1")
        
        if config.connect_timeout < 1:
            errors.append("Database.connect_timeout: Connect timeout must be at least 1 second")
        
        return errors
    
    def validate_navigation_config(self, config: NavigationConfig) -> List[str]:
        """
        Validate navigation configuration.
        
        Args:
            config: Navigation configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        errors = []
        
        # Apply all validation rules
        for rule in self._navigation_rules:
            if not rule.condition(config):
                errors.append(f"Navigation.{rule.field_name}: {rule.error_message}")
        
        # Cross-field validation
        if config.conflict_box_heartbeat_interval >= config.conflict_box_lock_timeout:
            errors.append("Navigation.conflict_box_heartbeat_interval: Heartbeat interval must be less than lock timeout")
        
        if config.blocked_cell_cleanup_interval <= 0:
            errors.append("Navigation.blocked_cell_cleanup_interval: Cleanup interval must be positive")
        
        if config.velocity_smoothing_factor <= 0 or config.velocity_smoothing_factor > 1:
            errors.append("Navigation.velocity_smoothing_factor: Smoothing factor must be between 0 and 1")
        
        return errors
    
    def validate_task_config(self, config: TaskConfig) -> List[str]:
        """
        Validate task configuration.
        
        Args:
            config: Task configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        errors = []
        
        # Apply all validation rules
        for rule in self._task_rules:
            if not rule.condition(config):
                errors.append(f"Task.{rule.field_name}: {rule.error_message}")
        
        # Cross-field validation
        if config.min_bid_value >= config.max_bid_value:
            errors.append("Task.min_bid_value: Min bid value must be less than max bid value")
        
        if config.retry_delay <= 0:
            errors.append("Task.retry_delay: Retry delay must be positive")
        
        if config.order_processing_interval <= 0:
            errors.append("Task.order_processing_interval: Processing interval must be positive")
        
        return errors
    
    def validate_system_config(self, config: SystemConfig) -> List[str]:
        """
        Validate system configuration.
        
        Args:
            config: System configuration to validate
            
        Returns:
            List[str]: List of validation errors
        """
        errors = []
        
        # Apply all validation rules
        for rule in self._system_rules:
            if not rule.condition(config):
                errors.append(f"System.{rule.field_name}: {rule.error_message}")
        
        # Cross-field validation
        valid_log_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        if config.log_level.upper() not in valid_log_levels:
            errors.append(f"System.log_level: Must be one of {valid_log_levels}")
        
        if config.cache_ttl <= 0:
            errors.append("System.cache_ttl: Cache TTL must be positive")
        
        if config.health_check_interval <= 0:
            errors.append("System.health_check_interval: Health check interval must be positive")
        
        if config.metrics_collection_interval <= 0:
            errors.append("System.metrics_collection_interval: Metrics collection interval must be positive")
        
        return errors
    
    def _create_robot_validation_rules(self) -> List[ValidationRule]:
        """Create validation rules for robot configuration."""
        return [
            ValidationRule(
                lambda c: c.robot_id and len(c.robot_id.strip()) > 0,
                "Robot ID cannot be empty",
                "robot_id"
            ),
            ValidationRule(
                lambda c: c.max_speed > 0,
                "Max speed must be positive",
                "max_speed"
            ),
            ValidationRule(
                lambda c: c.max_speed <= 10.0,
                "Max speed cannot exceed 10 m/s",
                "max_speed"
            ),
            ValidationRule(
                lambda c: c.position_tolerance > 0,
                "Position tolerance must be positive",
                "position_tolerance"
            ),
            ValidationRule(
                lambda c: c.position_tolerance <= 1.0,
                "Position tolerance cannot exceed 1 meter",
                "position_tolerance"
            ),
            ValidationRule(
                lambda c: c.lane_tolerance > 0,
                "Lane tolerance must be positive",
                "lane_tolerance"
            ),
            ValidationRule(
                lambda c: c.lane_tolerance <= 2.0,
                "Lane tolerance cannot exceed 2 meters",
                "lane_tolerance"
            ),
            ValidationRule(
                lambda c: c.corner_speed > 0,
                "Corner speed must be positive",
                "corner_speed"
            ),
            ValidationRule(
                lambda c: c.bay_approach_speed > 0,
                "Bay approach speed must be positive",
                "bay_approach_speed"
            ),
            ValidationRule(
                lambda c: c.conflict_box_lock_timeout > 0,
                "Conflict box lock timeout must be positive",
                "conflict_box_lock_timeout"
            ),
            ValidationRule(
                lambda c: c.conflict_box_heartbeat_interval > 0,
                "Conflict box heartbeat interval must be positive",
                "conflict_box_heartbeat_interval"
            ),
            ValidationRule(
                lambda c: c.max_linear_velocity > 0,
                "Max linear velocity must be positive",
                "max_linear_velocity"
            ),
            ValidationRule(
                lambda c: c.max_angular_velocity > 0,
                "Max angular velocity must be positive",
                "max_angular_velocity"
            ),
            ValidationRule(
                lambda c: c.movement_speed > 0,
                "Movement speed must be positive",
                "movement_speed"
            ),
            ValidationRule(
                lambda c: c.wheel_base > 0,
                "Wheel base must be positive",
                "wheel_base"
            ),
            ValidationRule(
                lambda c: c.wheel_radius > 0,
                "Wheel radius must be positive",
                "wheel_radius"
            ),
            ValidationRule(
                lambda c: c.picking_duration > 0,
                "Picking duration must be positive",
                "picking_duration"
            ),
            ValidationRule(
                lambda c: c.dropping_duration > 0,
                "Dropping duration must be positive",
                "dropping_duration"
            ),
            ValidationRule(
                lambda c: 0 <= c.charging_threshold <= 1,
                "Charging threshold must be between 0 and 1",
                "charging_threshold"
            ),
            ValidationRule(
                lambda c: c.emergency_stop_distance > 0,
                "Emergency stop distance must be positive",
                "emergency_stop_distance"
            ),
            ValidationRule(
                lambda c: c.stall_recovery_timeout > 0,
                "Stall recovery timeout must be positive",
                "stall_recovery_timeout"
            ),
        ]
    
    def _create_database_validation_rules(self) -> List[ValidationRule]:
        """Create validation rules for database configuration."""
        return [
            ValidationRule(
                lambda c: c.host and len(c.host.strip()) > 0,
                "Database host cannot be empty",
                "host"
            ),
            ValidationRule(
                lambda c: c.database and len(c.database.strip()) > 0,
                "Database name cannot be empty",
                "database"
            ),
            ValidationRule(
                lambda c: c.user and len(c.user.strip()) > 0,
                "Database user cannot be empty",
                "user"
            ),
            ValidationRule(
                lambda c: c.application_name and len(c.application_name.strip()) > 0,
                "Application name cannot be empty",
                "application_name"
            ),
        ]
    
    def _create_navigation_validation_rules(self) -> List[ValidationRule]:
        """Create validation rules for navigation configuration."""
        return [
            ValidationRule(
                lambda c: c.path_planning_timeout > 0,
                "Path planning timeout must be positive",
                "path_planning_timeout"
            ),
            ValidationRule(
                lambda c: c.max_path_length > 0,
                "Max path length must be positive",
                "max_path_length"
            ),
            ValidationRule(
                lambda c: c.replanning_threshold > 0,
                "Replanning threshold must be positive",
                "replanning_threshold"
            ),
            ValidationRule(
                lambda c: c.conflict_box_lock_timeout > 0,
                "Conflict box lock timeout must be positive",
                "conflict_box_lock_timeout"
            ),
            ValidationRule(
                lambda c: c.conflict_box_heartbeat_interval > 0,
                "Conflict box heartbeat interval must be positive",
                "conflict_box_heartbeat_interval"
            ),
            ValidationRule(
                lambda c: c.conflict_box_priority_levels > 0,
                "Conflict box priority levels must be positive",
                "conflict_box_priority_levels"
            ),
            ValidationRule(
                lambda c: c.blocked_cell_timeout > 0,
                "Blocked cell timeout must be positive",
                "blocked_cell_timeout"
            ),
            ValidationRule(
                lambda c: c.lane_tolerance > 0,
                "Lane tolerance must be positive",
                "lane_tolerance"
            ),
            ValidationRule(
                lambda c: c.max_deviation > 0,
                "Max deviation must be positive",
                "max_deviation"
            ),
            ValidationRule(
                lambda c: c.max_consecutive_deviations > 0,
                "Max consecutive deviations must be positive",
                "max_consecutive_deviations"
            ),
        ]
    
    def _create_task_validation_rules(self) -> List[ValidationRule]:
        """Create validation rules for task configuration."""
        return [
            ValidationRule(
                lambda c: c.max_concurrent_tasks > 0,
                "Max concurrent tasks must be positive",
                "max_concurrent_tasks"
            ),
            ValidationRule(
                lambda c: c.task_timeout > 0,
                "Task timeout must be positive",
                "task_timeout"
            ),
            ValidationRule(
                lambda c: c.retry_attempts >= 0,
                "Retry attempts must be non-negative",
                "retry_attempts"
            ),
            ValidationRule(
                lambda c: c.max_concurrent_orders > 0,
                "Max concurrent orders must be positive",
                "max_concurrent_orders"
            ),
            ValidationRule(
                lambda c: c.order_processing_interval > 0,
                "Order processing interval must be positive",
                "order_processing_interval"
            ),
            ValidationRule(
                lambda c: c.inventory_allocation_timeout > 0,
                "Inventory allocation timeout must be positive",
                "inventory_allocation_timeout"
            ),
            ValidationRule(
                lambda c: c.bidding_timeout > 0,
                "Bidding timeout must be positive",
                "bidding_timeout"
            ),
            ValidationRule(
                lambda c: c.min_bid_value >= 0,
                "Min bid value must be non-negative",
                "min_bid_value"
            ),
            ValidationRule(
                lambda c: c.max_bid_value > 0,
                "Max bid value must be positive",
                "max_bid_value"
            ),
            ValidationRule(
                lambda c: c.distance_cost_factor >= 0,
                "Distance cost factor must be non-negative",
                "distance_cost_factor"
            ),
            ValidationRule(
                lambda c: c.battery_cost_factor >= 0,
                "Battery cost factor must be non-negative",
                "battery_cost_factor"
            ),
        ]
    
    def _create_system_validation_rules(self) -> List[ValidationRule]:
        """Create validation rules for system configuration."""
        return [
            ValidationRule(
                lambda c: c.log_level and len(c.log_level.strip()) > 0,
                "Log level cannot be empty",
                "log_level"
            ),
            ValidationRule(
                lambda c: c.log_format and len(c.log_format.strip()) > 0,
                "Log format cannot be empty",
                "log_format"
            ),
            ValidationRule(
                lambda c: c.cache_ttl > 0,
                "Cache TTL must be positive",
                "cache_ttl"
            ),
            ValidationRule(
                lambda c: c.connection_pool_size > 0,
                "Connection pool size must be positive",
                "connection_pool_size"
            ),
            ValidationRule(
                lambda c: c.thread_pool_size > 0,
                "Thread pool size must be positive",
                "thread_pool_size"
            ),
            ValidationRule(
                lambda c: c.health_check_interval > 0,
                "Health check interval must be positive",
                "health_check_interval"
            ),
            ValidationRule(
                lambda c: c.metrics_collection_interval > 0,
                "Metrics collection interval must be positive",
                "metrics_collection_interval"
            ),
            ValidationRule(
                lambda c: isinstance(c.alert_thresholds, dict),
                "Alert thresholds must be a dictionary",
                "alert_thresholds"
            ),
        ] 