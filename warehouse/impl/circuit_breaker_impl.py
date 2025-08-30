"""
Circuit Breaker Implementation

This module provides concrete implementations of the circuit breaker interfaces
to improve error recovery and system resilience.
"""

import logging
import threading
import time
from typing import Any, Callable, Optional, Dict, List
from datetime import datetime, timedelta
from contextlib import contextmanager

from interfaces.circuit_breaker_interface import (
    ICircuitBreaker, ICircuitBreakerPolicy, ICircuitBreakerMonitor,
    CircuitState, CircuitBreakerMetrics, CircuitBreakerError,
    CircuitBreakerOpenError, CircuitBreakerTimeoutError
)


class DefaultCircuitBreakerPolicy(ICircuitBreakerPolicy):
    """
    Default circuit breaker policy implementation.
    
    This follows the Strategy Pattern and provides configurable thresholds
    for circuit breaker behavior.
    """
    
    def __init__(self, 
                 failure_threshold: int = 5,
                 timeout_duration: timedelta = timedelta(seconds=60),
                 half_open_timeout: timedelta = timedelta(seconds=30)):
        self.failure_threshold = failure_threshold
        self.timeout_duration = timeout_duration
        self.half_open_timeout = half_open_timeout
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
    
    def should_open_circuit(self, metrics: CircuitBreakerMetrics) -> bool:
        """Determine if circuit should open based on metrics."""
        if metrics.failed_calls >= self.failure_threshold:
            self.logger.info(f"Circuit should open: {metrics.failed_calls} failures >= {self.failure_threshold} threshold")
            return True
        return False
    
    def should_close_circuit(self, metrics: CircuitBreakerMetrics) -> bool:
        """Determine if circuit should close based on metrics."""
        # Only close if we're in half-open state and have recent success
        if (metrics.current_state == CircuitState.HALF_OPEN and 
            metrics.last_success_time and 
            metrics.last_success_time > datetime.now() - self.half_open_timeout):
            self.logger.info("Circuit should close: successful recovery in half-open state")
            return True
        return False
    
    def should_half_open_circuit(self, metrics: CircuitBreakerMetrics) -> bool:
        """Determine if circuit should transition to half-open."""
        if (metrics.current_state == CircuitState.OPEN and 
            metrics.last_failure_time and 
            metrics.last_failure_time < datetime.now() - self.timeout_duration):
            self.logger.info("Circuit should half-open: timeout period elapsed")
            return True
        return False
    
    def get_timeout_duration(self) -> timedelta:
        """Get the timeout duration for circuit breaker operations."""
        return self.timeout_duration


class CircuitBreakerMonitor(ICircuitBreakerMonitor):
    """
    Circuit breaker monitor implementation.
    
    This follows the Single Responsibility Principle by focusing solely on
    monitoring and reporting circuit breaker status.
    """
    
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self._call_history: List[Dict[str, Any]] = []
        self._state_change_history: List[Dict[str, Any]] = []
        self._max_history_size = 1000
    
    def record_call(self, success: bool, duration: timedelta) -> None:
        """Record a circuit breaker call result."""
        call_record = {
            'timestamp': datetime.now(),
            'success': success,
            'duration': duration,
            'duration_ms': duration.total_seconds() * 1000
        }
        
        self._call_history.append(call_record)
        
        # Trim history if too long
        if len(self._call_history) > self._max_history_size:
            self._call_history = self._call_history[-self._max_history_size:]
        
        self.logger.debug(f"Recorded call: success={success}, duration={duration}")
    
    def record_state_change(self, old_state: CircuitState, new_state: CircuitState) -> None:
        """Record a circuit breaker state change."""
        state_change = {
            'timestamp': datetime.now(),
            'old_state': old_state,
            'new_state': new_state
        }
        
        self._state_change_history.append(state_change)
        
        # Trim history if too long
        if len(self._state_change_history) > self._max_history_size:
            self._state_change_history = self._state_change_history[-self._max_history_size:]
        
        self.logger.info(f"Circuit state changed: {old_state.value} -> {new_state.value}")
    
    def get_health_status(self) -> Dict[str, Any]:
        """Get the health status of the circuit breaker."""
        if not self._call_history:
            return {
                'status': 'unknown',
                'total_calls': 0,
                'success_rate': 0.0,
                'average_duration_ms': 0.0,
                'last_state_change': None
            }
        
        recent_calls = self._call_history[-100:]  # Last 100 calls
        successful_calls = sum(1 for call in recent_calls if call['success'])
        total_recent_calls = len(recent_calls)
        
        if total_recent_calls > 0:
            success_rate = successful_calls / total_recent_calls
            avg_duration = sum(call['duration_ms'] for call in recent_calls) / total_recent_calls
        else:
            success_rate = 0.0
            avg_duration = 0.0
        
        last_state_change = None
        if self._state_change_history:
            last_state_change = self._state_change_history[-1]
        
        return {
            'status': 'healthy' if success_rate > 0.8 else 'degraded' if success_rate > 0.5 else 'unhealthy',
            'total_calls': len(self._call_history),
            'recent_calls': total_recent_calls,
            'success_rate': success_rate,
            'average_duration_ms': avg_duration,
            'last_state_change': last_state_change,
            'call_history_size': len(self._call_history),
            'state_change_history_size': len(self._state_change_history)
        }


class CircuitBreakerImpl(ICircuitBreaker):
    """
    Circuit breaker implementation that provides automatic error recovery
    and system resilience.
    
    This follows the Single Responsibility Principle by focusing solely on
    circuit breaker logic and the Open/Closed Principle by allowing different
    policies to be configured.
    """
    
    def __init__(self, 
                 policy: Optional[ICircuitBreakerPolicy] = None,
                 monitor: Optional[ICircuitBreakerMonitor] = None):
        self.policy = policy or DefaultCircuitBreakerPolicy()
        self.monitor = monitor or CircuitBreakerMonitor()
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Circuit state
        self._current_state = CircuitState.CLOSED
        self._metrics = CircuitBreakerMetrics()
        self._last_state_change = datetime.now()
        
        # State transition tracking
        self._consecutive_failures = 0
        self._consecutive_successes = 0
    
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
        start_time = datetime.now()
        
        # Check circuit state before execution
        if self.is_open():
            self.logger.warning("Circuit is open, call rejected")
            raise CircuitBreakerOpenError("Circuit breaker is open")
        
        try:
            # Execute the function
            result = func(*args, **kwargs)
            
            # Record successful call
            duration = datetime.now() - start_time
            self._record_success(duration)
            
            return result
            
        except Exception as e:
            # Record failed call
            duration = datetime.now() - start_time
            self._record_failure(duration)
            
            # Re-raise the original exception
            raise
    
    def is_open(self) -> bool:
        """Check if the circuit breaker is open."""
        with self._lock:
            return self._current_state == CircuitState.OPEN
    
    def is_closed(self) -> bool:
        """Check if the circuit breaker is closed."""
        with self._lock:
            return self._current_state == CircuitState.CLOSED
    
    def is_half_open(self) -> bool:
        """Check if the circuit breaker is half-open."""
        with self._lock:
            return self._current_state == CircuitState.HALF_OPEN
    
    def get_state(self) -> CircuitState:
        """Get the current circuit breaker state."""
        with self._lock:
            return self._current_state
    
    def get_metrics(self) -> CircuitBreakerMetrics:
        """Get circuit breaker metrics."""
        with self._lock:
            # Create a copy to avoid external modification
            return CircuitBreakerMetrics(
                total_calls=self._metrics.total_calls,
                successful_calls=self._metrics.successful_calls,
                failed_calls=self._metrics.failed_calls,
                last_failure_time=self._metrics.last_failure_time,
                last_success_time=self._metrics.last_success_time,
                current_state=self._current_state,
                state_changes=self._metrics.state_changes
            )
    
    def reset(self) -> None:
        """Reset the circuit breaker to closed state."""
        with self._lock:
            self._transition_to_state(CircuitState.CLOSED)
            self._consecutive_failures = 0
            self._consecutive_successes = 0
            self.logger.info("Circuit breaker reset to closed state")
    
    def force_open(self) -> None:
        """Force the circuit breaker to open state."""
        with self._lock:
            self._transition_to_state(CircuitState.OPEN)
            self.logger.info("Circuit breaker forced to open state")
    
    def force_close(self) -> None:
        """Force the circuit breaker to closed state."""
        with self._lock:
            self._transition_to_state(CircuitState.CLOSED)
            self.logger.info("Circuit breaker forced to closed state")
    
    def _record_success(self, duration: timedelta) -> None:
        """Record a successful call."""
        with self._lock:
            self._metrics.total_calls += 1
            self._metrics.successful_calls += 1
            self._metrics.last_success_time = datetime.now()
            
            self._consecutive_successes += 1
            self._consecutive_failures = 0
            
            # Check if we should transition states
            self._check_state_transitions()
            
            # Record in monitor
            self.monitor.record_call(True, duration)
    
    def _record_failure(self, duration: timedelta) -> None:
        """Record a failed call."""
        with self._lock:
            self._metrics.total_calls += 1
            self._metrics.failed_calls += 1
            self._metrics.last_failure_time = datetime.now()
            
            self._consecutive_failures += 1
            self._consecutive_successes = 0
            
            # Check if we should transition states
            self._check_state_transitions()
            
            # Record in monitor
            self.monitor.record_call(False, duration)
    
    def _check_state_transitions(self) -> None:
        """Check if circuit breaker should transition states."""
        if self._current_state == CircuitState.CLOSED:
            if self.policy.should_open_circuit(self._metrics):
                self._transition_to_state(CircuitState.OPEN)
        
        elif self._current_state == CircuitState.OPEN:
            if self.policy.should_half_open_circuit(self._metrics):
                self._transition_to_state(CircuitState.HALF_OPEN)
        
        elif self._current_state == CircuitState.HALF_OPEN:
            if self.policy.should_close_circuit(self._metrics):
                self._transition_to_state(CircuitState.CLOSED)
    
    def _transition_to_state(self, new_state: CircuitState) -> None:
        """Transition to a new state."""
        if new_state == self._current_state:
            return
        
        old_state = self._current_state
        self._current_state = new_state
        self._metrics.current_state = new_state
        self._metrics.state_changes += 1
        self._last_state_change = datetime.now()
        
        # Record state change in monitor
        self.monitor.record_state_change(old_state, new_state)
        
        self.logger.info(f"Circuit breaker transitioned from {old_state.value} to {new_state.value}")


# Convenience function to create a circuit breaker with default settings
def create_circuit_breaker(failure_threshold: int = 5,
                          timeout_seconds: int = 60,
                          half_open_timeout_seconds: int = 30) -> CircuitBreakerImpl:
    """Create a circuit breaker with default settings."""
    policy = DefaultCircuitBreakerPolicy(
        failure_threshold=failure_threshold,
        timeout_duration=timedelta(seconds=timeout_seconds),
        half_open_timeout=timedelta(seconds=half_open_timeout_seconds)
    )
    
    return CircuitBreakerImpl(policy=policy)


# Context manager for circuit breaker operations
@contextmanager
def circuit_breaker_context(circuit_breaker: ICircuitBreaker):
    """
    Context manager for circuit breaker operations.
    
    Usage:
        with circuit_breaker_context(circuit_breaker):
            # Your operation here
            result = some_function()
    """
    try:
        yield
    except Exception as e:
        # The circuit breaker will handle the exception
        raise







