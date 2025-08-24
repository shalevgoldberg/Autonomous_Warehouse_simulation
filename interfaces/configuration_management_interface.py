"""
Configuration Management Interface

This module provides interfaces for configuration management to separate
configuration concerns from business logic and improve maintainability.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Union, List
from dataclasses import dataclass
from enum import Enum


class ConfigurationSource(Enum):
    """Enumeration of configuration sources."""
    ENVIRONMENT = "environment"
    FILE = "file"
    DATABASE = "database"
    MEMORY = "memory"
    COMMAND_LINE = "command_line"


class ConfigurationValueType(Enum):
    """Enumeration of configuration value types."""
    STRING = "string"
    INTEGER = "integer"
    FLOAT = "float"
    BOOLEAN = "boolean"
    LIST = "list"
    DICT = "dict"


@dataclass
class ConfigurationItem:
    """Represents a configuration item with metadata."""
    key: str
    value: Any
    value_type: ConfigurationValueType
    source: ConfigurationSource
    description: Optional[str] = None
    is_required: bool = False
    default_value: Optional[Any] = None
    validation_rules: Optional[Dict[str, Any]] = None


@dataclass
class ConfigurationSection:
    """Represents a configuration section containing related items."""
    name: str
    items: Dict[str, ConfigurationItem]
    description: Optional[str] = None
    is_required: bool = False


class IConfigurationProvider(ABC):
    """
    Interface for configuration providers.
    
    This follows the Single Responsibility Principle by focusing solely on
    configuration management and the Interface Segregation Principle by
    providing focused configuration operations.
    """
    
    @abstractmethod
    def get_value(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value by key.
        
        Args:
            key: Configuration key (supports dot notation for nested keys)
            default: Default value if key not found
            
        Returns:
            Configuration value or default
        """
        pass
    
    @abstractmethod
    def get_section(self, section_name: str) -> ConfigurationSection:
        """
        Get a configuration section.
        
        Args:
            section_name: Name of the configuration section
            
        Returns:
            Configuration section containing all items
            
        Raises:
            ConfigurationSectionNotFoundError: If section doesn't exist
        """
        pass
    
    @abstractmethod
    def set_value(self, key: str, value: Any, source: ConfigurationSource = ConfigurationSource.MEMORY) -> None:
        """
        Set a configuration value.
        
        Args:
            key: Configuration key
            value: Value to set
            source: Source of the configuration value
        """
        pass
    
    @abstractmethod
    def has_key(self, key: str) -> bool:
        """
        Check if a configuration key exists.
        
        Args:
            key: Configuration key to check
            
        Returns:
            True if key exists, False otherwise
        """
        pass
    
    @abstractmethod
    def get_all_keys(self) -> List[str]:
        """
        Get all configuration keys.
        
        Returns:
            List of all configuration keys
        """
        pass
    
    @abstractmethod
    def reload(self) -> None:
        """Reload configuration from all sources."""
        pass
    
    @abstractmethod
    def validate(self) -> List[str]:
        """
        Validate configuration values.
        
        Returns:
            List of validation error messages (empty if valid)
        """
        pass


class IConfigurationValidator(ABC):
    """
    Interface for configuration validation.
    
    This follows the Single Responsibility Principle by focusing solely on
    configuration validation.
    """
    
    @abstractmethod
    def validate_item(self, item: ConfigurationItem) -> List[str]:
        """
        Validate a single configuration item.
        
        Args:
            item: Configuration item to validate
            
        Returns:
            List of validation error messages (empty if valid)
        """
        pass
    
    @abstractmethod
    def validate_section(self, section: ConfigurationSection) -> List[str]:
        """
        Validate a configuration section.
        
        Args:
            section: Configuration section to validate
            
        Returns:
            List of validation error messages (empty if valid)
        """
        pass
    
    @abstractmethod
    def validate_required_items(self, items: Dict[str, ConfigurationItem]) -> List[str]:
        """
        Validate that all required items are present.
        
        Args:
            items: Dictionary of configuration items
            
        Returns:
            List of validation error messages (empty if valid)
        """
        pass


class IConfigurationSource(ABC):
    """
    Interface for configuration sources.
    
    This follows the Open/Closed Principle by allowing new configuration
    sources to be added without modifying existing code.
    """
    
    @abstractmethod
    def load_configuration(self) -> Dict[str, ConfigurationItem]:
        """
        Load configuration from this source.
        
        Returns:
            Dictionary of configuration items
        """
        pass
    
    @abstractmethod
    def save_configuration(self, items: Dict[str, ConfigurationItem]) -> None:
        """
        Save configuration to this source.
        
        Args:
            items: Configuration items to save
        """
        pass
    
    @abstractmethod
    def get_source_name(self) -> str:
        """
        Get the name of this configuration source.
        
        Returns:
            Source name
        """
        pass
    
    @abstractmethod
    def is_readonly(self) -> bool:
        """
        Check if this source is read-only.
        
        Returns:
            True if read-only, False otherwise
        """
        pass


class ConfigurationError(Exception):
    """Base class for configuration-related errors."""
    pass


class ConfigurationKeyNotFoundError(ConfigurationError):
    """Raised when a configuration key is not found."""
    pass


class ConfigurationSectionNotFoundError(ConfigurationError):
    """Raised when a configuration section is not found."""
    pass


class ConfigurationValidationError(ConfigurationError):
    """Raised when configuration validation fails."""
    pass


class ConfigurationSourceError(ConfigurationError):
    """Raised when there's an error with a configuration source."""
    pass


