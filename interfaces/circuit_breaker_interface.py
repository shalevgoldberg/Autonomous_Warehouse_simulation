"""
Circuit Breaker Interface

This module provides interfaces for circuit breaker pattern implementation
to improve error recovery and system resilience.
"""

from abc import ABC, abstractmethod
from typing import Any, Callable, Optional, Dict
from enum import Enum
from dataclasses import dataclass
from datetime import datetime, timedelta


class CircuitState(Enum):
    """Enumeration of circuit breaker states."""
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Circuit is open, calls fail fast
    HALF_OPEN = "half_open"  # Testing if service is recovered


@dataclass
class CircuitBreakerMetrics:
    """Metrics for circuit breaker monitoring."""
    total_calls: int = 0
    successful_calls: int = 0
    failed_calls: int = 0
    last_failure_time: Optional[datetime] = None
    last_success_time: Optional[datetime] = None
    current_state: CircuitState = CircuitState.CLOSED
    state_changes: int = 0


class ICircuitBreaker(ABC):
    """
    Interface for circuit breaker implementation.
    
    This follows the Single Responsibility Principle by focusing solely on
    circuit breaker logic and the Interface Segregation Principle by
    providing focused circuit breaker operations.
    """
    
    @abstractmethod
    def call(self, func: Callable[..., Any], *args, **kwargs) -> Any:
        """
        Execute a function with circuit breaker protection.
        
        Args:
            func: Function to execute
            *args: Function arguments
            **kwargs: Function keyword arguments
            
        Returns:
            Function result
            
        Raises:
            CircuitBreakerOpenError: If circuit is open
            CircuitBreakerError: If circuit breaker fails
        """
        pass
    
    @abstractmethod
    def is_open(self) -> bool:
        """
        Check if the circuit breaker is open.
        
        Returns:
            True if circuit is open, False otherwise
        """
        pass
    
    @abstractmethod
    def is_closed(self) -> bool:
        """
        Check if the circuit breaker is closed.
        
        Returns:
            True if circuit is closed, False otherwise
        """
        pass
    
    @abstractmethod
    def is_half_open(self) -> bool:
        """
        Check if the circuit breaker is half-open.
        
        Returns:
            True if circuit is half-open, False otherwise
        """
        pass
    
    @abstractmethod
    def get_state(self) -> CircuitState:
        """
        Get the current circuit breaker state.
        
        Returns:
            Current circuit state
        """
        pass
    
    @abstractmethod
    def get_metrics(self) -> CircuitBreakerMetrics:
        """
        Get circuit breaker metrics.
        
        Returns:
            Circuit breaker metrics
        """
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset the circuit breaker to closed state."""
        pass
    
    @abstractmethod
    def force_open(self) -> None:
        """Force the circuit breaker to open state."""
        pass
    
    @abstractmethod
    def force_close(self) -> None:
        """Force the circuit breaker to closed state."""
        pass


class ICircuitBreakerPolicy(ABC):
    """
    Interface for circuit breaker policies.
    
    This follows the Strategy Pattern and allows different circuit breaker
    behaviors to be configured.
    """
    
    @abstractmethod
    def should_open_circuit(self, metrics: CircuitBreakerMetrics) -> bool:
        """
        Determine if circuit should open based on metrics.
        
        Args:
            metrics: Current circuit breaker metrics
            
        Returns:
            True if circuit should open, False otherwise
        """
        pass
    
    @abstractmethod
    def should_close_circuit(self, metrics: CircuitBreakerMetrics) -> bool:
        """
        Determine if circuit should close based on metrics.
        
        Args:
            metrics: Current circuit breaker metrics
            
        Returns:
            True if circuit should close, False otherwise
        """
        pass
    
    @abstractmethod
    def should_half_open_circuit(self, metrics: CircuitBreakerMetrics) -> bool:
        """
        Determine if circuit should transition to half-open.
        
        Args:
            metrics: Current circuit breaker metrics
            
        Returns:
            True if circuit should half-open, False otherwise
        """
        pass
    
    @abstractmethod
    def get_timeout_duration(self) -> timedelta:
        """
        Get the timeout duration for circuit breaker operations.
        
        Returns:
            Timeout duration
        """
        pass


class ICircuitBreakerMonitor(ABC):
    """
    Interface for circuit breaker monitoring.
    
    This follows the Single Responsibility Principle by focusing solely on
    monitoring and reporting circuit breaker status.
    """
    
    @abstractmethod
    def record_call(self, success: bool, duration: timedelta) -> None:
        """
        Record a circuit breaker call result.
        
        Args:
            success: Whether the call was successful
            duration: Duration of the call
        """
        pass
    
    @abstractmethod
    def record_state_change(self, old_state: CircuitState, new_state: CircuitState) -> None:
        """
        Record a circuit breaker state change.
        
        Args:
            old_state: Previous circuit state
            new_state: New circuit state
        """
        pass
    
    @abstractmethod
    def get_health_status(self) -> Dict[str, Any]:
        """
        Get the health status of the circuit breaker.
        
        Returns:
            Dictionary containing health status information
        """
        pass


class CircuitBreakerError(Exception):
    """Base class for circuit breaker errors."""
    pass


class CircuitBreakerOpenError(CircuitBreakerError):
    """Raised when circuit breaker is open and call is rejected."""
    pass


class CircuitBreakerTimeoutError(CircuitBreakerError):
    """Raised when circuit breaker operation times out."""
    pass


class CircuitBreakerConfigurationError(CircuitBreakerError):
    """Raised when circuit breaker configuration is invalid."""
    pass







