"""
Dependency Injection Interface

This module provides interfaces for dependency injection to reduce tight coupling
and improve testability in the conflict box system.
"""

from abc import ABC, abstractmethod
from typing import Type, TypeVar, Any, Dict, Optional
from contextlib import contextmanager

T = TypeVar('T')


class IServiceProvider(ABC):
    """
    Interface for a service provider that manages dependency injection.
    
    This follows the Dependency Inversion Principle by providing an abstraction
    for service resolution and lifecycle management.
    """
    
    @abstractmethod
    def register_service(self, service_type: Type[T], implementation: Type[T], 
                        lifetime: str = 'singleton') -> None:
        """
        Register a service implementation.
        
        Args:
            service_type: The interface/abstract class type
            implementation: The concrete implementation class
            lifetime: Service lifetime ('singleton', 'transient', 'scoped')
        """
        pass
    
    @abstractmethod
    def get_service(self, service_type: Type[T]) -> T:
        """
        Resolve a service instance.
        
        Args:
            service_type: The service type to resolve
            
        Returns:
            An instance of the requested service
            
        Raises:
            ServiceNotRegisteredError: If service is not registered
            ServiceResolutionError: If service cannot be resolved
        """
        pass
    
    @abstractmethod
    def create_scope(self) -> 'IServiceScope':
        """
        Create a new service scope for scoped services.
        
        Returns:
            A new service scope
        """
        pass


class IServiceScope(ABC):
    """
    Interface for a service scope that manages scoped service lifetimes.
    """
    
    @abstractmethod
    def get_service(self, service_type: Type[T]) -> T:
        """
        Resolve a service within this scope.
        
        Args:
            service_type: The service type to resolve
            
        Returns:
            An instance of the requested service
        """
        pass
    
    @abstractmethod
    def dispose(self) -> None:
        """Dispose of the scope and its services."""
        pass


class IServiceFactory(ABC):
    """
    Interface for a service factory that creates service instances.
    """
    
    @abstractmethod
    def create_instance(self, service_type: Type[T], **kwargs) -> T:
        """
        Create a new instance of a service.
        
        Args:
            service_type: The service type to create
            **kwargs: Constructor arguments
            
        Returns:
            A new instance of the service
        """
        pass


class ServiceNotRegisteredError(Exception):
    """Raised when attempting to resolve an unregistered service."""
    pass


class ServiceResolutionError(Exception):
    """Raised when a service cannot be resolved due to dependency issues."""
    pass


class ServiceLifetimeError(Exception):
    """Raised when there's an issue with service lifetime management."""
    pass


