"""
Configuration Provider Implementation

This module provides a concrete implementation of IConfigurationProvider
that reads configuration from multiple sources including environment variables
and configuration files.
"""

import os
import json
import logging
from typing import Any, Dict, List, Optional, Union
from pathlib import Path
import time

from interfaces.configuration_management_interface import (
    IConfigurationProvider, IConfigurationValidator, IConfigurationSource,
    ConfigurationItem, ConfigurationSection, ConfigurationSource,
    ConfigurationValueType, ConfigurationValidationError
)


class EnvironmentConfigurationSource(IConfigurationSource):
    """Configuration source that reads from environment variables."""
    
    def __init__(self, prefix: str = ""):
        self.prefix = prefix.upper()
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
    
    def load_configuration(self) -> Dict[str, ConfigurationItem]:
        """Load configuration from environment variables."""
        items = {}
        
        for key, value in os.environ.items():
            if self.prefix and not key.startswith(self.prefix):
                continue
                
            # Determine value type
            value_type = self._infer_value_type(value)
            
            # Create configuration item
            item = ConfigurationItem(
                key=key.lower().replace('_', '.'),
                value=value,
                value_type=value_type,
                source=ConfigurationSource.ENVIRONMENT,
                description=f"Environment variable: {key}",
                is_required=False
            )
            items[item.key] = item
        
        self.logger.info(f"Loaded {len(items)} configuration items from environment")
        return items
    
    def save_configuration(self, items: Dict[str, ConfigurationItem]) -> None:
        """Environment variables are read-only, so this is a no-op."""
        self.logger.warning("Cannot save to environment variables - source is read-only")
    
    def get_source_name(self) -> str:
        """Get the name of this configuration source."""
        return f"Environment({self.prefix})"
    
    def is_readonly(self) -> bool:
        """Environment variables are read-only."""
        return True
    
    def _infer_value_type(self, value: str) -> ConfigurationValueType:
        """Infer the type of a configuration value."""
        if value.lower() in ('true', 'false'):
            return ConfigurationValueType.BOOLEAN
        elif value.isdigit():
            return ConfigurationValueType.INTEGER
        elif value.replace('.', '').replace('-', '').isdigit():
            return ConfigurationValueType.FLOAT
        else:
            return ConfigurationValueType.STRING


class FileConfigurationSource(IConfigurationSource):
    """Configuration source that reads from JSON configuration files."""
    
    def __init__(self, file_path: Union[str, Path]):
        self.file_path = Path(file_path)
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
    
    def load_configuration(self) -> Dict[str, ConfigurationItem]:
        """Load configuration from the configuration file."""
        if not self.file_path.exists():
            self.logger.warning(f"Configuration file not found: {self.file_path}")
            return {}
        
        try:
            with open(self.file_path, 'r') as f:
                config_data = json.load(f)
            
            items = self._parse_config_data(config_data)
            self.logger.info(f"Loaded {len(items)} configuration items from {self.file_path}")
            return items
            
        except Exception as e:
            self.logger.error(f"Failed to load configuration from {self.file_path}: {e}")
            return {}
    
    def save_configuration(self, items: Dict[str, ConfigurationItem]) -> None:
        """Save configuration to the file."""
        try:
            # Convert ConfigurationItem objects back to plain data
            config_data = {}
            for key, item in items.items():
                if '.' in key:
                    # Handle nested keys
                    parts = key.split('.')
                    current = config_data
                    for part in parts[:-1]:
                        if part not in current:
                            current[part] = {}
                        current = current[part]
                    current[parts[-1]] = item.value
                else:
                    config_data[key] = item.value
            
            with open(self.file_path, 'w') as f:
                json.dump(config_data, f, indent=2)
            
            self.logger.info(f"Saved configuration to {self.file_path}")
            
        except Exception as e:
            self.logger.error(f"Failed to save configuration to {self.file_path}: {e}")
    
    def get_source_name(self) -> str:
        """Get the name of this configuration source."""
        return f"File({self.file_path.name})"
    
    def is_readonly(self) -> bool:
        """File sources are writable."""
        return False
    
    def _parse_config_data(self, data: Any, prefix: str = "") -> Dict[str, ConfigurationItem]:
        """Recursively parse configuration data into ConfigurationItem objects."""
        items = {}
        
        if isinstance(data, dict):
            for key, value in data.items():
                current_key = f"{prefix}.{key}" if prefix else key
                
                if isinstance(value, dict):
                    # Recursively parse nested dictionaries
                    items.update(self._parse_config_data(value, current_key))
                else:
                    # Create configuration item for leaf values
                    value_type = self._infer_value_type(value)
                    item = ConfigurationItem(
                        key=current_key,
                        value=str(value),
                        value_type=value_type,
                        source=ConfigurationSource.FILE,
                        description=f"Configuration file: {current_key}",
                        is_required=False
                    )
                    items[current_key] = item
        
        return items
    
    def _infer_value_type(self, value: Any) -> ConfigurationValueType:
        """Infer the type of a configuration value."""
        if isinstance(value, bool):
            return ConfigurationValueType.BOOLEAN
        elif isinstance(value, int):
            return ConfigurationValueType.INTEGER
        elif isinstance(value, float):
            return ConfigurationValueType.FLOAT
        else:
            return ConfigurationValueType.STRING


class ConfigurationValidator(IConfigurationValidator):
    """Configuration validator that ensures configuration values meet requirements."""
    
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
    
    def validate_item(self, item: ConfigurationItem) -> List[str]:
        """Validate a single configuration item."""
        errors = []
        
        try:
            # Validate required items
            if item.is_required and not item.value:
                errors.append(f"Required configuration item '{item.key}' has no value")
            
            # Validate value type
            if not self._validate_value_type(item):
                errors.append(f"Configuration item '{item.key}' has invalid type: expected {item.value_type.value}, got '{item.value}'")
            
            # Validate specific item requirements
            item_errors = self._validate_specific_item(item)
            errors.extend(item_errors)
            
        except Exception as e:
            errors.append(f"Error validating configuration item '{item.key}': {e}")
        
        return errors
    
    def validate_section(self, section: ConfigurationSection) -> List[str]:
        """Validate a configuration section."""
        errors = []
        
        # Validate required sections
        if section.is_required and not section.items:
            errors.append(f"Required configuration section '{section.name}' has no items")
        
        # Validate each item in the section
        for item in section.items.values():
            item_errors = self.validate_item(item)
            errors.extend(item_errors)
        
        return errors
    
    def validate_required_items(self, items: Dict[str, ConfigurationItem]) -> List[str]:
        """Validate that all required items are present."""
        errors = []
        
        for key, item in items.items():
            if item.is_required and not item.value:
                errors.append(f"Required configuration item '{key}' has no value")
        
        return errors
    
    def validate_configuration(self, items: List[ConfigurationItem]) -> List[str]:
        """Validate configuration items and return validation errors."""
        errors = []
        
        for item in items:
            item_errors = self.validate_item(item)
            errors.extend(item_errors)
        
        if errors:
            self.logger.warning(f"Configuration validation found {len(errors)} errors")
        else:
            self.logger.info("Configuration validation passed")
        
        return errors
    
    def _validate_value_type(self, item: ConfigurationItem) -> bool:
        """Validate that a configuration item's value matches its declared type."""
        try:
            if item.value_type == ConfigurationValueType.BOOLEAN:
                return item.value.lower() in ('true', 'false', '1', '0', 'yes', 'no')
            elif item.value_type == ConfigurationValueType.INTEGER:
                int(item.value)
                return True
            elif item.value_type == ConfigurationValueType.FLOAT:
                float(item.value)
                return True
            elif item.value_type == ConfigurationValueType.STRING:
                return True  # Strings are always valid
            else:
                return False
        except (ValueError, TypeError):
            return False
    
    def _validate_specific_item(self, item: ConfigurationItem) -> List[str]:
        """Validate specific configuration items based on their keys."""
        errors = []
        
        # Database connection validation
        if 'database' in item.key.lower() or 'db' in item.key.lower():
            if 'password' in item.key.lower() and len(item.value) < 1:
                errors.append(f"Database password for '{item.key}' is too short")
        
        # Connection pool validation
        if 'pool' in item.key.lower() and 'size' in item.key.lower():
            try:
                size = int(item.value)
                if size < 1 or size > 100:
                    errors.append(f"Connection pool size '{item.key}' must be between 1 and 100")
            except ValueError:
                errors.append(f"Connection pool size '{item.key}' must be a valid integer")
        
        # Timeout validation
        if 'timeout' in item.key.lower():
            try:
                timeout = float(item.value)
                if timeout < 0:
                    errors.append(f"Timeout '{item.key}' must be non-negative")
            except ValueError:
                errors.append(f"Timeout '{item.key}' must be a valid number")
        
        return errors


class ConfigurationProviderImpl(IConfigurationProvider):
    """
    Configuration provider implementation that aggregates configuration
    from multiple sources and provides a unified configuration interface.
    
    This follows the Single Responsibility Principle by focusing solely on
    configuration management and the Open/Closed Principle by allowing new
    configuration sources to be added without modifying existing code.
    """
    
    def __init__(self, sources: Optional[List[IConfigurationSource]] = None):
        self.sources = sources or []
        self.validator = ConfigurationValidator()
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        
        # Cache for configuration items
        self._config_cache: Optional[List[ConfigurationItem]] = None
        self._last_cache_time: Optional[float] = 0
        self._cache_ttl = 300  # 5 minutes cache TTL
    
    def add_source(self, source: IConfigurationSource) -> None:
        """Add a configuration source."""
        self.sources.append(source)
        self._invalidate_cache()
        self.logger.info(f"Added configuration source: {type(source).__name__}")
    
    def remove_source(self, source: IConfigurationSource) -> None:
        """Remove a configuration source."""
        if source in self.sources:
            self.sources.remove(source)
            self._invalidate_cache()
            self.logger.info(f"Removed configuration source: {type(source).__name__}")
    
    def get_value(self, key: str, default: Any = None) -> Any:
        """Get a configuration value by key with optional default."""
        item = self.get_configuration_item(key)
        if item is None:
            return default
        
        # Convert value to appropriate type
        return self._convert_value(item.value, item.value_type)
    
    def get_section(self, section_name: str) -> ConfigurationSection:
        """Get a configuration section by name."""
        items = self._get_all_configuration_items()
        
        # Filter items by section
        section_items = {}
        for key, item in items.items():
            if key.startswith(f"{section_name}."):
                # Remove section prefix from key
                item_key = key[len(f"{section_name}."):]
                section_items[item_key] = item
        
        return ConfigurationSection(
            name=section_name,
            items=section_items,
            description=f"Configuration section: {section_name}"
        )
    
    def set_value(self, key: str, value: Any, source: ConfigurationSource = ConfigurationSource.MEMORY) -> None:
        """Set a configuration value."""
        # Find a writable source
        writable_source = None
        for src in self.sources:
            if not src.is_readonly():
                writable_source = src
                break
        
        if writable_source is None:
            self.logger.warning("No writable configuration source available")
            return
        
        # Create configuration item
        value_type = self._infer_value_type(value)
        item = ConfigurationItem(
            key=key,
            value=str(value),
            value_type=value_type,
            source=source,
            description=f"Set by user: {key}",
            is_required=False
        )
        
        # Save to writable source
        current_config = writable_source.load_configuration()
        current_config[key] = item
        writable_source.save_configuration(current_config)
        
        # Invalidate cache
        self._invalidate_cache()
        
        self.logger.info(f"Set configuration value: {key} = {value}")
    
    def has_key(self, key: str) -> bool:
        """Check if a configuration key exists."""
        return self.get_configuration_item(key) is not None
    
    def get_all_keys(self) -> List[str]:
        """Get all configuration keys."""
        items = self._get_all_configuration_items()
        return list(items.keys())
    
    def reload(self) -> None:
        """Reload configuration from all sources."""
        self._invalidate_cache()
        self.logger.info("Configuration reloaded from all sources")
    
    def validate(self) -> List[str]:
        """Validate all configuration items."""
        items = self._get_all_configuration_items()
        return self.validator.validate_configuration(list(items.values()))
    
    def get_configuration_item(self, key: str) -> Optional[ConfigurationItem]:
        """Get a specific configuration item by key."""
        items = self._get_all_configuration_items()
        
        # Find item by key (case-insensitive)
        for item_key, item in items.items():
            if item_key.lower() == key.lower():
                return item
        
        return None
    
    def get_configuration_value(self, key: str, default: Any = None) -> Any:
        """Get a configuration value by key with optional default."""
        item = self.get_configuration_item(key)
        if item is None:
            return default
        
        # Convert value to appropriate type
        return self._convert_value(item.value, item.value_type)
    
    def get_configuration_section(self, section_name: str) -> ConfigurationSection:
        """Get a configuration section by name."""
        items = self._get_all_configuration_items()
        
        # Filter items by section
        section_items = {}
        for key, item in items.items():
            if key.startswith(f"{section_name}."):
                # Remove section prefix from key
                item_key = key[len(f"{section_name}."):]
                section_items[item_key] = item
        
        return ConfigurationSection(
            name=section_name,
            items=section_items,
            description=f"Configuration section: {section_name}"
        )
    
    def get_all_configuration_items(self) -> List[ConfigurationItem]:
        """Get all configuration items from all sources."""
        items = self._get_all_configuration_items()
        return list(items.values())
    
    def validate_configuration(self) -> List[str]:
        """Validate all configuration items."""
        items = self._get_all_configuration_items()
        return self.validator.validate_configuration(items)
    
    def refresh_configuration(self) -> None:
        """Refresh configuration from all sources."""
        self._invalidate_cache()
        self.logger.info("Configuration refreshed from all sources")
    
    def _get_all_configuration_items(self) -> Dict[str, ConfigurationItem]:
        """Get all configuration items with caching."""
        current_time = time.time()
        
        # Check if cache is valid
        if (self._config_cache is not None and 
            current_time - self._last_cache_time < self._cache_ttl):
            return self._config_cache
        
        # Load configuration from all sources
        all_items = {}
        for source in self.sources:
            try:
                items = source.load_configuration()
                all_items.update(items)
            except Exception as e:
                self.logger.error(f"Failed to load configuration from {type(source).__name__}: {e}")
        
        # Update cache
        self._config_cache = all_items
        self._last_cache_time = current_time
        
        self.logger.debug(f"Loaded {len(all_items)} configuration items from {len(self.sources)} sources")
        return all_items
    
    def _invalidate_cache(self) -> None:
        """Invalidate the configuration cache."""
        self._config_cache = None
        self._last_cache_time = 0
    
    def _infer_value_type(self, value: Any) -> ConfigurationValueType:
        """Infer the type of a configuration value."""
        if isinstance(value, bool):
            return ConfigurationValueType.BOOLEAN
        elif isinstance(value, int):
            return ConfigurationValueType.INTEGER
        elif isinstance(value, float):
            return ConfigurationValueType.FLOAT
        elif isinstance(value, list):
            return ConfigurationValueType.LIST
        elif isinstance(value, dict):
            return ConfigurationValueType.DICT
        else:
            return ConfigurationValueType.STRING
    
    def _convert_value(self, value: str, value_type: ConfigurationValueType) -> Any:
        """Convert a string value to the appropriate type."""
        try:
            if value_type == ConfigurationValueType.BOOLEAN:
                return value.lower() in ('true', '1', 'yes')
            elif value_type == ConfigurationValueType.INTEGER:
                return int(value)
            elif value_type == ConfigurationValueType.FLOAT:
                return float(value)
            else:
                return value
        except (ValueError, TypeError):
            self.logger.warning(f"Failed to convert value '{value}' to type {value_type.value}, returning as string")
            return value


# Convenience function to create a default configuration provider
def create_default_configuration_provider() -> ConfigurationProviderImpl:
    """Create a configuration provider with default sources."""
    provider = ConfigurationProviderImpl()
    
    # Add environment variable source
    provider.add_source(EnvironmentConfigurationSource())
    
    # Add file source if config file exists
    config_file = Path("config.json")
    if config_file.exists():
        provider.add_source(FileConfigurationSource(config_file))
    
    return provider
