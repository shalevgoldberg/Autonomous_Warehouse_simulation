"""
Dependency Injection Container Implementation

This module provides a concrete implementation of the dependency injection
container to reduce tight coupling and improve testability.
"""

import logging
import threading
from typing import Type, TypeVar, Any, Dict, Optional, Callable
from weakref import WeakValueDictionary

from interfaces.dependency_injection_interface import (
    IServiceProvider, IServiceScope, IServiceFactory,
    ServiceNotRegisteredError, ServiceResolutionError, ServiceLifetimeError
)
from interfaces.collision_avoidance_interface import ICollisionAvoidanceService
from interfaces.motion_command_filter_interface import IMotionCommandFilter
from interfaces.configuration_interface import IBusinessConfigurationProvider
from robot.impl.collision_avoidance_service_impl import CollisionAvoidanceServiceImpl, CollisionAvoidanceConfig
from robot.impl.motion_command_filter_impl import MotionCommandFilterImpl

T = TypeVar('T')


class ServiceDescriptor:
    """Describes a service registration with its lifetime and factory."""
    
    def __init__(self, service_type: Type, implementation: Type, lifetime: str, 
                 factory: Optional[Callable] = None):
        self.service_type = service_type
        self.implementation = implementation
        self.lifetime = lifetime
        self.factory = factory or self._default_factory
        self._instance: Optional[Any] = None
        self._lock = threading.RLock()
    
    def _default_factory(self, **kwargs) -> Any:
        """Default factory that creates instances using constructor."""
        return self.implementation(**kwargs)
    
    def get_instance(self, provider: 'DependencyInjectionContainer', **kwargs) -> Any:
        """Get or create an instance based on lifetime."""
        if self.lifetime == 'singleton':
            return self._get_singleton_instance(provider, **kwargs)
        elif self.lifetime == 'transient':
            return self._create_transient_instance(provider, **kwargs)
        else:
            raise ServiceLifetimeError(f"Unsupported lifetime: {self.lifetime}")
    
    def _get_singleton_instance(self, provider: 'DependencyInjectionContainer', **kwargs) -> Any:
        """Get or create singleton instance."""
        with self._lock:
            if self._instance is None:
                self._instance = self._create_instance(provider, **kwargs)
            return self._instance
    
    def _create_transient_instance(self, provider: 'DependencyInjectionContainer', **kwargs) -> Any:
        """Create a new transient instance."""
        return self._create_instance(provider, **kwargs)
    
    def _create_instance(self, provider: 'DependencyInjectionContainer', **kwargs) -> Any:
        """Create a new instance using the factory."""
        try:
            # Resolve constructor dependencies
            resolved_kwargs = self._resolve_constructor_dependencies(provider, **kwargs)
            return self.factory(**resolved_kwargs)
        except Exception as e:
            raise ServiceResolutionError(f"Failed to create instance of {self.implementation}: {e}")
    
    def _resolve_constructor_dependencies(self, provider: 'DependencyInjectionContainer', 
                                       **kwargs) -> Dict[str, Any]:
        """Resolve constructor dependencies using the provider."""
        resolved_kwargs = kwargs.copy()
        
        # Get constructor signature
        import inspect
        sig = inspect.signature(self.implementation.__init__)
        
        for param_name, param in sig.parameters.items():
            if param_name == 'self':
                continue
                
            # Skip if already provided
            if param_name in resolved_kwargs:
                continue
            
            # Try to resolve from provider if it's a registered type
            if param.annotation != inspect.Parameter.empty:
                try:
                    resolved_kwargs[param_name] = provider.get_service(param.annotation)
                except ServiceNotRegisteredError:
                    # Use default value if available
                    if param.default != inspect.Parameter.empty:
                        resolved_kwargs[param_name] = param.default
                    else:
                        raise ServiceResolutionError(
                            f"Cannot resolve parameter '{param_name}' of type {param.annotation}"
                        )
        
        return resolved_kwargs


class ServiceScope(IServiceScope):
    """Implementation of a service scope for scoped services."""
    
    def __init__(self, container: 'DependencyInjectionContainer'):
        self.container = container
        self.services: Dict[Type, Any] = {}
        self._disposed = False
    
    def get_service(self, service_type: Type[T]) -> T:
        """Get a service within this scope."""
        if self._disposed:
            raise ServiceLifetimeError("Cannot use disposed scope")
        
        if service_type not in self.services:
            self.services[service_type] = self.container._create_scoped_instance(service_type)
        
        return self.services[service_type]
    
    def dispose(self) -> None:
        """Dispose of the scope and its services."""
        if not self._disposed:
            self.services.clear()
            self._disposed = True


class DependencyInjectionContainer(IServiceProvider, IServiceFactory):
    """
    Implementation of a dependency injection container.
    
    This container manages service registration, resolution, and lifecycle
    following SOLID principles and reducing tight coupling.
    """
    
    def __init__(self):
        self._services: Dict[Type, ServiceDescriptor] = {}
        self._lock = threading.RLock()
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
    
    def register_service(self, service_type: Type[T], implementation: Type[T], 
                        lifetime: str = 'singleton') -> None:
        """Register a service implementation."""
        if not issubclass(implementation, service_type):
            raise ValueError(f"{implementation} must implement {service_type}")
        
        with self._lock:
            self._services[service_type] = ServiceDescriptor(
                service_type, implementation, lifetime
            )
            self.logger.info(f"Registered {service_type.__name__} -> {implementation.__name__} ({lifetime})")
    
    def register_factory(self, service_type: Type[T], factory: Callable[..., T], 
                        lifetime: str = 'singleton') -> None:
        """Register a service with a custom factory."""
        with self._lock:
            self._services[service_type] = ServiceDescriptor(
                service_type, service_type, lifetime, factory
            )
            self.logger.info(f"Registered {service_type.__name__} with custom factory ({lifetime})")
    
    def get_service(self, service_type: Type[T]) -> T:
        """Resolve a service instance."""
        with self._lock:
            if service_type not in self._services:
                raise ServiceNotRegisteredError(f"Service {service_type.__name__} is not registered")
            
            descriptor = self._services[service_type]
            return descriptor.get_instance(self)
    
    def create_scope(self) -> IServiceScope:
        """Create a new service scope."""
        return ServiceScope(self)

    def create_instance(self, service_type: Type[T], **kwargs) -> T:
        """Create a new instance of a service."""
        return self._create_instance(service_type, **kwargs)

    def create_collision_avoidance_service(self, robot_id: str,
                                         config_provider: Optional[IBusinessConfigurationProvider] = None) -> ICollisionAvoidanceService:
        """
        Create a collision avoidance service instance for a specific robot.

        Args:
            robot_id: Robot identifier for logging and per-robot state
            config_provider: Configuration provider (resolved if not provided)

        Returns:
            ICollisionAvoidanceService: Configured collision avoidance service
        """
        if config_provider is None:
            self.logger.warning(f"Config provider not provided for collision avoidance service on {robot_id}, resolving automatically")
            config_provider = self.get_service(IBusinessConfigurationProvider)

        # Load collision avoidance configuration
        # Note: get_value returns ConfigurationValue objects, extract the .value field
        ca_config = CollisionAvoidanceConfig(
            fov_degrees=config_provider.get_value("collision_avoidance.fov_degrees", 50.0).value,
            num_beams=config_provider.get_value("collision_avoidance.num_beams", 25).value,
            braking_cone_degrees=config_provider.get_value("collision_avoidance.braking_cone_degrees", 10.0).value,
            lateral_guard_margin=config_provider.get_value("collision_avoidance.lateral_guard_margin", 0.10).value,
            static_stop_distance=config_provider.get_value("collision_avoidance.static_stop_distance", 0.15).value,
            dynamic_stop_distance=config_provider.get_value("collision_avoidance.dynamic_stop_distance", 0.30).value,
            dynamic_caution_distance=config_provider.get_value("collision_avoidance.dynamic_caution_distance", 0.50).value,
            headway_time_gap=config_provider.get_value("collision_avoidance.headway_time_gap", 0.8).value,
            yaw_bias_max=config_provider.get_value("collision_avoidance.yaw_bias_max", 0.1).value
        )

        # Get robot radius for lateral guard
        robot_radius = config_provider.get_value("robot.robot_width", 0.25).value / 2.0

        return CollisionAvoidanceServiceImpl(
            config=ca_config,
            robot_radius=robot_radius,
            robot_id=robot_id
        )

    def create_motion_command_filter(self, robot_id: str,
                                   config_provider: Optional[IBusinessConfigurationProvider] = None) -> IMotionCommandFilter:
        """
        Create a motion command filter instance for a specific robot.

        Args:
            robot_id: Robot identifier for logging
            config_provider: Configuration provider (resolved if not provided)

        Returns:
            IMotionCommandFilter: Configured motion command filter
        """
        if config_provider is None:
            self.logger.warning(f"Config provider not provided for motion command filter on {robot_id}, resolving automatically")
            config_provider = self.get_service(IBusinessConfigurationProvider)

        return MotionCommandFilterImpl(
            config_provider=config_provider,
            robot_id=robot_id
        )
    
    def _create_instance(self, service_type: Type[T], **kwargs) -> T:
        """Internal method to create instances."""
        if service_type not in self._services:
            raise ServiceNotRegisteredError(f"Service {service_type.__name__} is not registered")
        
        descriptor = self._services[service_type]
        return descriptor.get_instance(self, **kwargs)
    
    def _create_scoped_instance(self, service_type: Type[T]) -> T:
        """Create an instance for a scope."""
        if service_type not in self._services:
            raise ServiceNotRegisteredError(f"Service {service_type.__name__} is not registered")
        
        descriptor = self._services[service_type]
        if descriptor.lifetime == 'scoped':
            return descriptor._create_instance(self)
        else:
            # For non-scoped services, delegate to main container
            return self.get_service(service_type)
    
    def get_registered_services(self) -> Dict[str, str]:
        """Get information about registered services for debugging."""
        with self._lock:
            return {
                service_type.__name__: f"{desc.implementation.__name__} ({desc.lifetime})"
                for service_type, desc in self._services.items()
            }
    
    def clear(self) -> None:
        """Clear all registered services (useful for testing)."""
        with self._lock:
            self._services.clear()
            self.logger.info("All services cleared")











