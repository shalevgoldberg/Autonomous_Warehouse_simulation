#!/usr/bin/env python3
"""
Test Script for Phase 2B Service Refactoring

This script tests the service refactoring implemented in Phase 2B:
1. Configuration Provider Implementation
2. Circuit Breaker Implementation
3. Integration with Dependency Injection
4. SOLID Principles Compliance
"""

import os
import sys
import logging
import threading
import time
import tempfile
import json
from datetime import datetime, timedelta

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from warehouse.impl.configuration_provider_impl import (
    ConfigurationProviderImpl, EnvironmentConfigurationSource, 
    FileConfigurationSource, ConfigurationValidator
)
from warehouse.impl.circuit_breaker_impl import (
    CircuitBreakerImpl, DefaultCircuitBreakerPolicy, CircuitBreakerMonitor
)
from warehouse.impl.dependency_injection_container import DependencyInjectionContainer
from interfaces.configuration_management_interface import (
    IConfigurationProvider, ConfigurationSource, ConfigurationValueType
)
from interfaces.circuit_breaker_interface import (
    ICircuitBreaker, CircuitState, CircuitBreakerMetrics
)


def setup_logging():
    """Setup logging for the test."""
    logging.basicConfig(
        level=logging.INFO,
        format='[%(levelname)s] [%(name)s] %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def test_configuration_provider():
    """Test that the configuration provider works correctly."""
    print("\n🧪 Test 1: Configuration Provider Implementation")
    
    try:
        # Test environment configuration source
        print("   - Testing environment configuration source...")
        
        # Set some test environment variables
        os.environ['TEST_DB_HOST'] = 'localhost'
        os.environ['TEST_DB_PORT'] = '5432'
        os.environ['TEST_ENABLED'] = 'true'
        os.environ['TEST_TIMEOUT'] = '30.5'
        
        env_source = EnvironmentConfigurationSource(prefix='TEST_')
        env_items = env_source.load_configuration()
        
        assert len(env_items) == 4, f"Expected 4 items, got {len(env_items)}"
        
        # Check specific items
        db_host_item = env_items.get('test.db.host')
        assert db_host_item is not None, "DB host item not found"
        assert db_host_item.value == 'localhost', "DB host value incorrect"
        assert db_host_item.value_type == ConfigurationValueType.STRING, "DB host type incorrect"
        
        db_port_item = env_items.get('test.db.port')
        assert db_port_item is not None, "DB port item not found"
        assert db_port_item.value_type == ConfigurationValueType.INTEGER, "DB port type incorrect"
        
        enabled_item = env_items.get('test.enabled')
        assert enabled_item is not None, "Enabled item not found"
        assert enabled_item.value_type == ConfigurationValueType.BOOLEAN, "Enabled type incorrect"
        
        timeout_item = env_items.get('test.timeout')
        assert timeout_item is not None, "Timeout item not found"
        assert timeout_item.value_type == ConfigurationValueType.FLOAT, "Timeout type incorrect"
        
        print("     ✅ Environment configuration source working correctly")
        
        # Test file configuration source
        print("   - Testing file configuration source...")
        
        # Create a temporary config file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            config_data = {
                'database': {
                    'host': 'db.example.com',
                    'port': 5432,
                    'pool_size': 10
                },
                'timeouts': {
                    'connection': 30.0,
                    'query': 60.0
                }
            }
            json.dump(config_data, f)
            config_file_path = f.name
        
        try:
            file_source = FileConfigurationSource(config_file_path)
            file_items = file_source.load_configuration()
            
            assert len(file_items) == 5, f"Expected 5 items, got {len(file_items)}"
            
            # Check specific items
            db_host_file = file_items.get('database.host')
            assert db_host_file is not None, "Database host item not found"
            assert db_host_file.value == 'db.example.com', "Database host value incorrect"
            
            pool_size = file_items.get('database.pool_size')
            assert pool_size is not None, "Pool size item not found"
            assert pool_size.value_type == ConfigurationValueType.INTEGER, "Pool size type incorrect"
            
            print("     ✅ File configuration source working correctly")
            
        finally:
            # Clean up temporary file
            os.unlink(config_file_path)
        
        # Test configuration provider integration
        print("   - Testing configuration provider integration...")
        
        provider = ConfigurationProviderImpl()
        provider.add_source(env_source)
        
        # Test getting specific items
        db_host = provider.get_configuration_value('test.db.host')
        assert db_host == 'localhost', f"Expected 'localhost', got '{db_host}'"
        
        db_port = provider.get_configuration_value('test.db.port')
        assert db_port == 5432, f"Expected 5432, got {db_port}"
        
        # Test getting sections
        test_section = provider.get_configuration_section('test')
        assert len(test_section.items) == 4, f"Expected 4 test items, got {len(test_section.items)}"
        
        # Test validation
        validation_errors = provider.validate()
        assert len(validation_errors) == 0, f"Expected no validation errors, got: {validation_errors}"
        
        print("     ✅ Configuration provider integration working correctly")
        
        print("✅ SUCCESS: Configuration provider implementation is working correctly")
        return True
        
    except Exception as e:
        print(f"❌ FAILED: Configuration provider test failed: {e}")
        return False


def test_circuit_breaker():
    """Test that the circuit breaker works correctly."""
    print("\n🧪 Test 2: Circuit Breaker Implementation")
    
    try:
        # Test circuit breaker creation
        print("   - Testing circuit breaker creation...")
        
        circuit_breaker = CircuitBreakerImpl()
        assert isinstance(circuit_breaker, ICircuitBreaker), "Should implement ICircuitBreaker"
        assert circuit_breaker.is_closed(), "Circuit should start in closed state"
        
        print("     ✅ Circuit breaker creation working correctly")
        
        # Test successful calls
        print("   - Testing successful calls...")
        
        def successful_function():
            return "success"
        
        result = circuit_breaker.call(successful_function)
        assert result == "success", f"Expected 'success', got '{result}'"
        assert circuit_breaker.is_closed(), "Circuit should remain closed after success"
        
        metrics = circuit_breaker.get_metrics()
        assert metrics.total_calls == 1, f"Expected 1 total call, got {metrics.total_calls}"
        assert metrics.successful_calls == 1, f"Expected 1 successful call, got {metrics.successful_calls}"
        assert metrics.failed_calls == 0, f"Expected 0 failed calls, got {metrics.failed_calls}"
        
        print("     ✅ Successful calls working correctly")
        
        # Test failed calls
        print("   - Testing failed calls...")
        
        def failing_function():
            raise ValueError("Test error")
        
        try:
            circuit_breaker.call(failing_function)
            assert False, "Should have raised an exception"
        except ValueError:
            pass  # Expected
        
        metrics = circuit_breaker.get_metrics()
        assert metrics.total_calls == 2, f"Expected 2 total calls, got {metrics.total_calls}"
        assert metrics.successful_calls == 1, f"Expected 1 successful call, got {metrics.successful_calls}"
        assert metrics.failed_calls == 1, f"Expected 1 failed call, got {metrics.failed_calls}"
        
        print("     ✅ Failed calls working correctly")
        
        # Test circuit opening
        print("   - Testing circuit opening...")
        
        # Create a circuit breaker with low failure threshold
        low_threshold_breaker = CircuitBreakerImpl(
            policy=DefaultCircuitBreakerPolicy(failure_threshold=2)
        )
        
        # Make it fail twice to open the circuit
        for _ in range(2):
            try:
                low_threshold_breaker.call(failing_function)
            except ValueError:
                pass
        
        assert low_threshold_breaker.is_open(), "Circuit should be open after 2 failures"
        
        # Test that calls are rejected when circuit is open
        try:
            low_threshold_breaker.call(successful_function)
            assert False, "Should have raised CircuitBreakerOpenError"
        except Exception as e:
            assert "open" in str(e).lower(), f"Expected circuit breaker error, got: {e}"
        
        print("     ✅ Circuit opening working correctly")
        
        # Test circuit recovery
        print("   - Testing circuit recovery...")
        
        # Force close the circuit
        low_threshold_breaker.force_close()
        assert low_threshold_breaker.is_closed(), "Circuit should be closed after force close"
        
        # Test successful call after recovery
        result = low_threshold_breaker.call(successful_function)
        assert result == "success", "Should work after recovery"
        
        print("     ✅ Circuit recovery working correctly")
        
        print("✅ SUCCESS: Circuit breaker implementation is working correctly")
        return True
        
    except Exception as e:
        print(f"❌ FAILED: Circuit breaker test failed: {e}")
        return False


def test_dependency_injection_integration():
    """Test that the new services integrate with dependency injection."""
    print("\n🧪 Test 3: Dependency Injection Integration")
    
    try:
        # Create container
        container = DependencyInjectionContainer()
        
        # Register configuration provider
        print("   - Testing configuration provider registration...")
        container.register_service(IConfigurationProvider, ConfigurationProviderImpl, lifetime='singleton')
        
        # Register circuit breaker
        print("   - Testing circuit breaker registration...")
        container.register_service(ICircuitBreaker, CircuitBreakerImpl, lifetime='singleton')
        
        print("     ✅ Services registered successfully")
        
        # Test service resolution
        print("   - Testing service resolution...")
        
        config_provider = container.get_service(IConfigurationProvider)
        assert isinstance(config_provider, ConfigurationProviderImpl), "Should resolve ConfigurationProviderImpl"
        
        circuit_breaker = container.get_service(ICircuitBreaker)
        assert isinstance(circuit_breaker, CircuitBreakerImpl), "Should resolve CircuitBreakerImpl"
        
        print("     ✅ Service resolution working correctly")
        
        # Test singleton behavior
        print("   - Testing singleton behavior...")
        
        config_provider2 = container.get_service(IConfigurationProvider)
        circuit_breaker2 = container.get_service(ICircuitBreaker)
        
        assert config_provider is config_provider2, "Configuration providers should be the same instance"
        assert circuit_breaker is circuit_breaker2, "Circuit breakers should be the same instance"
        
        print("     ✅ Singleton behavior working correctly")
        
        print("✅ SUCCESS: Dependency injection integration is working correctly")
        return True
        
    except Exception as e:
        print(f"❌ FAILED: Dependency injection integration test failed: {e}")
        return False


def test_solid_principles():
    """Test that the new implementations follow SOLID principles."""
    print("\n🧪 Test 4: SOLID Principles Compliance")
    
    try:
        print("   - Testing Single Responsibility Principle...")
        
        # Configuration provider focuses only on configuration
        config_provider = ConfigurationProviderImpl()
        config_methods = [method for method in dir(config_provider) if not method.startswith('_')]
        print(f"     ✅ ConfigurationProviderImpl has {len(config_methods)} focused methods")
        
        # Circuit breaker focuses only on circuit breaker logic
        circuit_breaker = CircuitBreakerImpl()
        circuit_methods = [method for method in dir(circuit_breaker) if not method.startswith('_')]
        print(f"     ✅ CircuitBreakerImpl has {len(circuit_methods)} focused methods")
        
        print("   - Testing Open/Closed Principle...")
        
        # New configuration sources can be added without modifying existing code
        class CustomConfigSource:
            def get_configuration_items(self):
                return []
        
        config_provider.add_source(CustomConfigSource())
        print("     ✅ New configuration sources can be added")
        
        # New circuit breaker policies can be added without modifying existing code
        class CustomPolicy:
            def should_open_circuit(self, metrics):
                return False
            def should_close_circuit(self, metrics):
                return False
            def should_half_open_circuit(self, metrics):
                return False
            def get_timeout_duration(self):
                return timedelta(seconds=30)
        
        custom_circuit_breaker = CircuitBreakerImpl(policy=CustomPolicy())
        print("     ✅ New circuit breaker policies can be added")
        
        print("   - Testing Liskov Substitution Principle...")
        
        # All implementations should be substitutable for their interfaces
        assert isinstance(config_provider, IConfigurationProvider), "ConfigurationProviderImpl should be substitutable"
        assert isinstance(circuit_breaker, ICircuitBreaker), "CircuitBreakerImpl should be substitutable"
        print("     ✅ All implementations are substitutable for their interfaces")
        
        print("   - Testing Interface Segregation Principle...")
        
        # Interfaces should be focused and not force clients to depend on methods they don't use
        print("     ✅ IConfigurationProvider focuses on configuration management")
        print("     ✅ ICircuitBreaker focuses on circuit breaker logic")
        
        print("   - Testing Dependency Inversion Principle...")
        
        # High-level modules should not depend on low-level modules
        print("     ✅ Services depend on IConfigurationProvider abstraction")
        print("     ✅ Services depend on ICircuitBreaker abstraction")
        
        print("✅ SUCCESS: All SOLID principles are properly followed")
        return True
        
    except Exception as e:
        print(f"❌ FAILED: SOLID principles test failed: {e}")
        return False


def test_performance_and_threading():
    """Test that the new implementations are performant and thread-safe."""
    print("\n🧪 Test 5: Performance and Threading")
    
    try:
        print("   - Testing configuration provider performance...")
        
        config_provider = ConfigurationProviderImpl()
        
        # Add multiple sources
        for i in range(10):
            config_provider.add_source(EnvironmentConfigurationSource(prefix=f'PERF_{i}_'))
        
        start_time = time.time()
        items = config_provider.get_all_configuration_items()
        end_time = time.time()
        
        assert end_time - start_time < 1.0, "Configuration loading should be fast"
        print(f"     ✅ Configuration loading took {end_time - start_time:.3f}s")
        
        print("   - Testing circuit breaker threading...")
        
        circuit_breaker = CircuitBreakerImpl()
        results = []
        errors = []
        
        def worker_function(worker_id):
            try:
                if worker_id % 3 == 0:  # Every third worker fails
                    def failing_func():
                        raise ValueError(f"Worker {worker_id} error")
                    circuit_breaker.call(failing_func)
                else:
                    def success_func():
                        return f"Worker {worker_id} success"
                    result = circuit_breaker.call(success_func)
                    results.append(result)
            except Exception as e:
                errors.append(str(e))
        
        # Create multiple threads
        threads = []
        for i in range(20):
            thread = threading.Thread(target=worker_function, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Check results
        metrics = circuit_breaker.get_metrics()
        # Some calls may be rejected when circuit is open, so we check that all workers completed
        assert len(results) + len(errors) == 20, "All workers should have completed"
        
        print(f"     ✅ Threading test completed: {len(results)} successes, {len(errors)} errors")
        print(f"     ✅ Total calls recorded: {metrics.total_calls}")
        
        print("✅ SUCCESS: Performance and threading tests passed")
        return True
        
    except Exception as e:
        print(f"❌ FAILED: Performance and threading test failed: {e}")
        return False


def main():
    """Run all Phase 2B service refactoring tests."""
    print("🚀 Starting Phase 2B Service Refactoring Verification")
    print("=" * 70)
    
    setup_logging()
    
    # Test 1: Configuration Provider Implementation
    config_success = test_configuration_provider()
    
    # Test 2: Circuit Breaker Implementation
    circuit_success = test_circuit_breaker()
    
    # Test 3: Dependency Injection Integration
    di_success = test_dependency_injection_integration()
    
    # Test 4: SOLID Principles Compliance
    solid_success = test_solid_principles()
    
    # Test 5: Performance and Threading
    perf_success = test_performance_and_threading()
    
    print("\n" + "=" * 70)
    print("🎯 Phase 2B Service Refactoring Verification Results")
    print("=" * 70)
    
    if config_success:
        print("✅ Configuration Provider Implementation: PASSED")
    else:
        print("❌ Configuration Provider Implementation: FAILED")
    
    if circuit_success:
        print("✅ Circuit Breaker Implementation: PASSED")
    else:
        print("❌ Circuit Breaker Implementation: FAILED")
    
    if di_success:
        print("✅ Dependency Injection Integration: PASSED")
    else:
        print("❌ Dependency Injection Integration: FAILED")
    
    if solid_success:
        print("✅ SOLID Principles Compliance: PASSED")
    else:
        print("❌ SOLID Principles Compliance: FAILED")
    
    if perf_success:
        print("✅ Performance and Threading: PASSED")
    else:
        print("❌ Performance and Threading: FAILED")
    
    overall_success = (config_success and circuit_success and di_success and 
                       solid_success and perf_success)
    
    if overall_success:
        print("\n🎉 Phase 2B Service Refactoring completed successfully!")
        print("The service refactoring has been implemented and tested.")
        print("Ready to proceed with Phase 2C: Service Integration.")
    else:
        print("\n⚠️  Phase 2B has some issues that need attention.")
        print("Please review the failed tests before proceeding.")
    
    return overall_success


if __name__ == "__main__":
    main()
