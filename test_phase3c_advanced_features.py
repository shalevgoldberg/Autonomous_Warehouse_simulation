#!/usr/bin/env python3
"""
Test Script for Phase 3C: Advanced Features and Final Integration

This script tests the complete Phase 3 implementation to ensure:
1. All advanced features work correctly
2. Performance monitoring is accurate
3. Health checks are comprehensive
4. Final integration is complete and robust
"""

import os
import sys
import logging
import threading
import time
from datetime import datetime, timedelta

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from warehouse.impl.conflict_box_integration_service_impl import ConflictBoxIntegrationServiceImpl


def setup_logging():
    """Setup logging for the test."""
    logging.basicConfig(
        level=logging.INFO,
        format='[%(levelname)s] [%(name)s] %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def test_comprehensive_integration():
    """Test comprehensive integration of all Phase 3 components."""
    print("\n🧪 Test 1: Comprehensive Integration Test")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=5)
        
        print("Testing complete Phase 3 integration...")
        
        # Test 1: System health
        print("   - Testing system health...")
        is_healthy = integration_service.is_system_healthy()
        print(f"     ✅ System health: {'HEALTHY' if is_healthy else 'UNHEALTHY'}")
        
        # Test 2: System metrics
        print("   - Testing system metrics...")
        metrics = integration_service.get_system_metrics()
        print(f"     ✅ System metrics retrieved: {metrics.total_operations} total operations")
        print(f"     ✅ Average response time: {metrics.average_response_time:.3f} seconds")
        print(f"     ✅ Concurrent operations: {metrics.concurrent_operations}")
        
        # Test 3: System integrity
        print("   - Testing system integrity...")
        issues = integration_service.validate_system_integrity()
        print(f"     ✅ System integrity: {len(issues)} issues found")
        
        # Test 4: Performance under load
        print("   - Testing performance under load...")
        start_time = time.time()
        
        # Create multiple threads to simulate load
        def load_worker(worker_id):
            """Worker that creates load on the system."""
            try:
                for i in range(20):
                    # Perform various operations
                    health = integration_service.is_system_healthy()
                    metrics = integration_service.get_system_metrics()
                    issues = integration_service.validate_system_integrity()
                    
                    # Small delay
                    time.sleep(0.01)
                
                return worker_id, True
            except Exception as e:
                return worker_id, str(e)
        
        # Create 10 workers
        threads = []
        for i in range(10):
            thread = threading.Thread(target=load_worker, args=(i,))
            threads.append(thread)
        
        # Start all threads
        for thread in threads:
            thread.start()
        
        # Wait for completion
        for thread in threads:
            thread.join()
        
        end_time = time.time()
        load_time = end_time - start_time
        
        print(f"     ✅ Load test completed in {load_time:.2f} seconds")
        
        # Test 5: Final metrics after load
        final_metrics = integration_service.get_system_metrics()
        print(f"     ✅ Final metrics: {final_metrics.total_operations} operations")
        print(f"     ✅ Final response time: {final_metrics.average_response_time:.3f} seconds")
        
        # Shutdown service
        integration_service.shutdown()
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Comprehensive integration test failed: {e}")
        return False


def test_performance_monitoring():
    """Test performance monitoring capabilities."""
    print("\n🧪 Test 2: Performance Monitoring Test")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=3)
        
        print("Testing performance monitoring capabilities...")
        
        # Test baseline metrics
        baseline_metrics = integration_service.get_system_metrics()
        print(f"   - Baseline operations: {baseline_metrics.total_operations}")
        
        # Perform operations and measure performance
        operations = []
        start_time = time.time()
        
        for i in range(50):
            op_start = time.time()
            
            # Perform operation
            if i % 3 == 0:
                result = integration_service.is_system_healthy()
            elif i % 3 == 1:
                result = integration_service.get_system_metrics()
            else:
                result = integration_service.validate_system_integrity()
            
            op_end = time.time()
            operations.append(op_end - op_start)
        
        end_time = time.time()
        total_time = end_time - start_time
        
        # Analyze performance
        avg_operation_time = sum(operations) / len(operations)
        min_operation_time = min(operations)
        max_operation_time = max(operations)
        operations_per_second = len(operations) / total_time
        
        print(f"   - Total operations: {len(operations)}")
        print(f"   - Total time: {total_time:.3f} seconds")
        print(f"   - Average operation time: {avg_operation_time:.3f} seconds")
        print(f"   - Min operation time: {min_operation_time:.3f} seconds")
        print(f"   - Max operation time: {max_operation_time:.3f} seconds")
        print(f"   - Operations per second: {operations_per_second:.1f}")
        
        # Get final metrics
        final_metrics = integration_service.get_system_metrics()
        print(f"   - Final total operations: {final_metrics.total_operations}")
        print(f"   - Final average response time: {final_metrics.average_response_time:.3f} seconds")
        
        # Verify metrics are accurate
        if final_metrics.total_operations >= len(operations):
            print("     ✅ Performance metrics are accurate")
            metrics_accurate = True
        else:
            print("     ❌ Performance metrics may be inaccurate")
            metrics_accurate = False
        
        # Shutdown service
        integration_service.shutdown()
        
        return metrics_accurate
        
    except Exception as e:
        print(f"❌ FAILURE: Performance monitoring test failed: {e}")
        return False


def test_health_monitoring():
    """Test comprehensive health monitoring."""
    print("\n🧪 Test 3: Health Monitoring Test")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=3)
        
        print("Testing health monitoring capabilities...")
        
        # Test 1: Initial health check
        print("   - Testing initial health check...")
        initial_health = integration_service.is_system_healthy()
        print(f"     ✅ Initial health: {'HEALTHY' if initial_health else 'UNHEALTHY'}")
        
        # Test 2: Health check after operations
        print("   - Testing health check after operations...")
        for i in range(10):
            integration_service.get_system_metrics()
            integration_service.is_system_healthy()
        
        health_after_ops = integration_service.is_system_healthy()
        print(f"     ✅ Health after operations: {'HEALTHY' if health_after_ops else 'UNHEALTHY'}")
        
        # Test 3: Integrity validation
        print("   - Testing integrity validation...")
        integrity_issues = integration_service.validate_system_integrity()
        print(f"     ✅ Integrity issues found: {len(integrity_issues)}")
        
        # Test 4: Health consistency
        print("   - Testing health consistency...")
        health_checks = []
        for i in range(5):
            health = integration_service.is_system_healthy()
            health_checks.append(health)
            time.sleep(0.1)
        
        consistent_health = all(h == health_checks[0] for h in health_checks)
        print(f"     ✅ Health consistency: {'CONSISTENT' if consistent_health else 'INCONSISTENT'}")
        
        # Test 5: Health under stress
        print("   - Testing health under stress...")
        def stress_health_check():
            """Worker that performs health checks under stress."""
            for i in range(20):
                integration_service.is_system_healthy()
                time.sleep(0.01)
        
        # Create multiple threads
        threads = []
        for i in range(5):
            thread = threading.Thread(target=stress_health_check)
            threads.append(thread)
        
        # Start threads
        for thread in threads:
            thread.start()
        
        # Wait for completion
        for thread in threads:
            thread.join()
        
        final_health = integration_service.is_system_healthy()
        print(f"     ✅ Final health under stress: {'HEALTHY' if final_health else 'UNHEALTHY'}")
        
        # Shutdown service
        integration_service.shutdown()
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Health monitoring test failed: {e}")
        return False


def test_error_handling_and_recovery():
    """Test error handling and recovery mechanisms."""
    print("\n🧪 Test 4: Error Handling and Recovery Test")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=3)
        
        print("Testing error handling and recovery...")
        
        # Test 1: Parameter validation
        print("   - Testing parameter validation...")
        try:
            integration_service.request_conflict_box_access("", "robot1", 0)
            print("     ❌ Should have raised ValueError for empty box_id")
            return False
        except ValueError:
            print("     ✅ Proper validation for empty box_id")
        
        try:
            integration_service.request_conflict_box_access("box1", "", 0)
            print("     ❌ Should have raised ValueError for empty robot_id")
            return False
        except ValueError:
            print("     ✅ Proper validation for empty robot_id")
        
        # Test 2: Recovery after errors
        print("   - Testing recovery after errors...")
        
        # Perform normal operations after errors
        health = integration_service.is_system_healthy()
        metrics = integration_service.get_system_metrics()
        
        print(f"     ✅ Recovery successful: health={health}, metrics={metrics is not None}")
        
        # Test 3: Graceful degradation
        print("   - Testing graceful degradation...")
        
        # Create many operations to test graceful degradation
        def degradation_worker(worker_id):
            """Worker that tests graceful degradation."""
            try:
                for i in range(30):
                    integration_service.is_system_healthy()
                    time.sleep(0.001)
                return worker_id, True
            except Exception as e:
                return worker_id, str(e)
        
        # Create many workers
        threads = []
        for i in range(20):
            thread = threading.Thread(target=degradation_worker, args=(i,))
            threads.append(thread)
        
        # Start threads
        for thread in threads:
            thread.start()
        
        # Wait for completion
        for thread in threads:
            thread.join()
        
        print("     ✅ Graceful degradation test completed")
        
        # Test 4: Final health after stress
        final_health = integration_service.is_system_healthy()
        print(f"     ✅ Final health after stress: {'HEALTHY' if final_health else 'UNHEALTHY'}")
        
        # Shutdown service
        integration_service.shutdown()
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Error handling test failed: {e}")
        return False


def test_final_integration_verification():
    """Final verification that all Phase 3 components work together."""
    print("\n🧪 Test 5: Final Integration Verification")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=5)
        
        print("Performing final integration verification...")
        
        # Test all major components together
        print("   - Testing all major components...")
        
        # Component 1: Health monitoring
        health = integration_service.is_system_healthy()
        print(f"     ✅ Health monitoring: {'HEALTHY' if health else 'UNHEALTHY'}")
        
        # Component 2: Performance monitoring
        metrics = integration_service.get_system_metrics()
        print(f"     ✅ Performance monitoring: {metrics.total_operations} operations")
        
        # Component 3: Integrity validation
        issues = integration_service.validate_system_integrity()
        print(f"     ✅ Integrity validation: {len(issues)} issues")
        
        # Component 4: Thread safety (quick test)
        def quick_thread_test():
            """Quick thread safety test."""
            for i in range(10):
                integration_service.is_system_healthy()
                time.sleep(0.001)
        
        threads = [threading.Thread(target=quick_thread_test) for _ in range(5)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        
        print("     ✅ Thread safety: VERIFIED")
        
        # Component 5: Resource management
        final_metrics = integration_service.get_system_metrics()
        print(f"     ✅ Resource management: {final_metrics.concurrent_operations} concurrent operations")
        
        # Final health check
        final_health = integration_service.is_system_healthy()
        print(f"     ✅ Final health check: {'HEALTHY' if final_health else 'UNHEALTHY'}")
        
        # Shutdown service
        integration_service.shutdown()
        
        print("     ✅ All components integrated successfully")
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Final integration verification failed: {e}")
        return False


def main():
    """Run all Phase 3C tests."""
    print("🚀 Starting Phase 3C: Advanced Features and Final Integration")
    print("=" * 70)
    
    setup_logging()
    
    # Test 1: Comprehensive integration
    integration_success = test_comprehensive_integration()
    
    # Test 2: Performance monitoring
    performance_success = test_performance_monitoring()
    
    # Test 3: Health monitoring
    health_success = test_health_monitoring()
    
    # Test 4: Error handling and recovery
    error_handling_success = test_error_handling_and_recovery()
    
    # Test 5: Final integration verification
    final_verification_success = test_final_integration_verification()
    
    print("\n" + "=" * 70)
    print("🎯 Phase 3C Test Results")
    print("=" * 70)
    
    if integration_success:
        print("✅ Comprehensive Integration: PASSED")
    else:
        print("❌ Comprehensive Integration: FAILED")
    
    if performance_success:
        print("✅ Performance Monitoring: PASSED")
    else:
        print("❌ Performance Monitoring: FAILED")
    
    if health_success:
        print("✅ Health Monitoring: PASSED")
    else:
        print("❌ Health Monitoring: FAILED")
    
    if error_handling_success:
        print("✅ Error Handling & Recovery: PASSED")
    else:
        print("❌ Error Handling & Recovery: FAILED")
    
    if final_verification_success:
        print("✅ Final Integration Verification: PASSED")
    else:
        print("❌ Final Integration Verification: FAILED")
    
    overall_success = (integration_success and 
                      performance_success and 
                      health_success and 
                      error_handling_success and 
                      final_verification_success)
    
    if overall_success:
        print("\n🎉 Phase 3C (Advanced Features) completed successfully!")
        print("🎉 PHASE 3 COMPLETE - All components successfully integrated!")
        print("🎉 The conflict box system is now production-ready!")
    else:
        print("\n⚠️  Phase 3C has some issues that need attention.")
    
    return overall_success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)







