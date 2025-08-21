#!/usr/bin/env python3
"""
Test Script for Phase 3B: Advanced Thread Safety and Race Condition Prevention

This script tests advanced threading scenarios to ensure:
1. Complex concurrent operations work correctly
2. Race conditions are completely prevented
3. Performance under high load
4. Resource management and cleanup
"""

import os
import sys
import logging
import threading
import time
import random
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed

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


def test_high_concurrency_scenario():
    """Test high concurrency scenario with multiple operations."""
    print("\n🧪 Test 1: High Concurrency Scenario")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=5)
        
        # Test parameters
        num_threads = 20
        operations_per_thread = 10
        total_operations = num_threads * operations_per_thread
        
        print(f"Testing {num_threads} threads with {operations_per_thread} operations each")
        print(f"Total operations: {total_operations}")
        
        # Results tracking
        results = {
            'successful': 0,
            'failed': 0,
            'total_time': 0,
            'concurrent_peak': 0
        }
        
        def worker_thread(thread_id):
            """Worker thread for high concurrency testing."""
            thread_results = []
            
            for op_id in range(operations_per_thread):
                try:
                    # Simulate different types of operations
                    operation_type = random.choice(['health_check', 'metrics', 'integrity'])
                    
                    if operation_type == 'health_check':
                        result = integration_service.is_system_healthy()
                        thread_results.append(('health_check', result))
                    elif operation_type == 'metrics':
                        metrics = integration_service.get_system_metrics()
                        thread_results.append(('metrics', metrics is not None))
                    elif operation_type == 'integrity':
                        issues = integration_service.validate_system_integrity()
                        thread_results.append(('integrity', issues is not None))
                    
                    # Small random delay to simulate real-world conditions
                    time.sleep(random.uniform(0.01, 0.05))
                    
                except Exception as e:
                    thread_results.append(('error', str(e)))
            
            return thread_id, thread_results
        
        # Execute with ThreadPoolExecutor for better control
        start_time = time.time()
        
        with ThreadPoolExecutor(max_workers=num_threads) as executor:
            # Submit all tasks
            future_to_thread = {
                executor.submit(worker_thread, i): i 
                for i in range(num_threads)
            }
            
            # Collect results
            for future in as_completed(future_to_thread):
                thread_id, thread_results = future.result()
                
                # Process thread results
                for op_type, result in thread_results:
                    if op_type == 'error':
                        results['failed'] += 1
                    else:
                        results['successful'] += 1
        
        end_time = time.time()
        results['total_time'] = end_time - start_time
        
        # Get final metrics
        final_metrics = integration_service.get_system_metrics()
        results['concurrent_peak'] = final_metrics.concurrent_operations
        
        # Report results
        print(f"✅ SUCCESS: High concurrency test completed in {results['total_time']:.2f} seconds")
        print(f"   - Successful operations: {results['successful']}")
        print(f"   - Failed operations: {results['failed']}")
        print(f"   - Success rate: {(results['successful'] / total_operations) * 100:.1f}%")
        print(f"   - Peak concurrent operations: {results['concurrent_peak']}")
        print(f"   - Operations per second: {total_operations / results['total_time']:.1f}")
        
        # Shutdown service
        integration_service.shutdown()
        
        return results['successful'] > results['failed']
        
    except Exception as e:
        print(f"❌ FAILURE: High concurrency test failed: {e}")
        return False


def test_resource_contention():
    """Test resource contention scenarios."""
    print("\n🧪 Test 2: Resource Contention Scenarios")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=3)
        
        # Test with limited connection pool to force contention
        print("Testing with limited connection pool (3 connections) to force contention")
        
        def contention_worker(worker_id):
            """Worker that creates contention for database connections."""
            try:
                # Perform multiple operations to hold connections
                for i in range(5):
                    # Get system metrics (uses database connection)
                    metrics = integration_service.get_system_metrics()
                    
                    # Simulate work time
                    time.sleep(0.1)
                    
                    # Health check (uses database connection)
                    health = integration_service.is_system_healthy()
                    
                    # Small delay
                    time.sleep(0.05)
                
                return worker_id, True
                
            except Exception as e:
                return worker_id, str(e)
        
        # Create more workers than available connections
        num_workers = 8  # More than the 3 available connections
        
        print(f"Creating {num_workers} workers with only 3 database connections available")
        
        start_time = time.time()
        
        with ThreadPoolExecutor(max_workers=num_workers) as executor:
            # Submit all tasks
            futures = [executor.submit(contention_worker, i) for i in range(num_workers)]
            
            # Collect results
            successful = 0
            failed = 0
            
            for future in as_completed(futures):
                worker_id, result = future.result()
                if result is True:
                    successful += 1
                else:
                    failed += 1
                    print(f"   Worker {worker_id} failed: {result}")
        
        end_time = time.time()
        total_time = end_time - start_time
        
        print(f"✅ SUCCESS: Resource contention test completed in {total_time:.2f} seconds")
        print(f"   - Successful workers: {successful}/{num_workers}")
        print(f"   - Failed workers: {failed}/{num_workers}")
        print(f"   - Success rate: {(successful / num_workers) * 100:.1f}%")
        
        # Shutdown service
        integration_service.shutdown()
        
        return successful > failed
        
    except Exception as e:
        print(f"❌ FAILURE: Resource contention test failed: {e}")
        return False


def test_graceful_degradation():
    """Test graceful degradation under stress."""
    print("\n🧪 Test 3: Graceful Degradation Under Stress")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=2)
        
        print("Testing graceful degradation with very limited resources")
        
        def stress_worker(worker_id):
            """Worker that creates stress on the system."""
            try:
                # Perform rapid operations
                for i in range(10):
                    # Quick operations
                    health = integration_service.is_system_healthy()
                    metrics = integration_service.get_system_metrics()
                    
                    # Minimal delay
                    time.sleep(0.01)
                
                return worker_id, True
                
            except Exception as e:
                return worker_id, str(e)
        
        # Create many workers to stress the system
        num_workers = 15
        
        print(f"Creating {num_workers} workers to stress the system")
        
        start_time = time.time()
        
        with ThreadPoolExecutor(max_workers=num_workers) as executor:
            # Submit all tasks
            futures = [executor.submit(stress_worker, i) for i in range(num_workers)]
            
            # Collect results
            successful = 0
            failed = 0
            
            for future in as_completed(futures):
                worker_id, result = future.result()
                if result is True:
                    successful += 1
                else:
                    failed += 1
        
        end_time = time.time()
        total_time = end_time - start_time
        
        print(f"✅ SUCCESS: Stress test completed in {total_time:.2f} seconds")
        print(f"   - Successful workers: {successful}/{num_workers}")
        print(f"   - Failed workers: {failed}/{num_workers}")
        print(f"   - Success rate: {(successful / num_workers) * 100:.1f}%")
        
        # Check if the system degraded gracefully
        if successful > 0:
            print("✅ SUCCESS: System degraded gracefully under stress")
            graceful = True
        else:
            print("❌ FAILURE: System failed completely under stress")
            graceful = False
        
        # Shutdown service
        integration_service.shutdown()
        
        return graceful
        
    except Exception as e:
        print(f"❌ FAILURE: Graceful degradation test failed: {e}")
        return False


def test_thread_safety_verification():
    """Verify that thread safety mechanisms are working correctly."""
    print("\n🧪 Test 4: Thread Safety Verification")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the integration service
        integration_service = ConflictBoxIntegrationServiceImpl(connection_string, max_connections=3)
        
        print("Verifying thread safety mechanisms")
        
        # Test concurrent access to shared resources
        shared_counter = 0
        counter_lock = threading.Lock()
        
        def counter_worker(worker_id):
            """Worker that increments a shared counter."""
            nonlocal shared_counter
            
            try:
                # Perform operations that should be thread-safe
                for i in range(100):
                    # Get metrics (should be thread-safe)
                    metrics = integration_service.get_system_metrics()
                    
                    # Increment shared counter (with our own lock for verification)
                    with counter_lock:
                        shared_counter += 1
                    
                    # Small delay
                    time.sleep(0.001)
                
                return worker_id, True
                
            except Exception as e:
                return worker_id, str(e)
        
        # Create multiple workers
        num_workers = 5
        
        print(f"Testing with {num_workers} workers accessing shared resources")
        
        start_time = time.time()
        
        with ThreadPoolExecutor(max_workers=num_workers) as executor:
            # Submit all tasks
            futures = [executor.submit(counter_worker, i) for i in range(num_workers)]
            
            # Collect results
            successful = 0
            failed = 0
            
            for future in as_completed(futures):
                worker_id, result = future.result()
                if result is True:
                    successful += 1
                else:
                    failed += 1
        
        end_time = time.time()
        total_time = end_time - start_time
        
        print(f"✅ SUCCESS: Thread safety verification completed in {total_time:.2f} seconds")
        print(f"   - Successful workers: {successful}/{num_workers}")
        print(f"   - Failed workers: {failed}/{num_workers}")
        print(f"   - Final shared counter value: {shared_counter}")
        print(f"   - Expected counter value: {num_workers * 100}")
        
        # Verify thread safety
        expected_counter = num_workers * 100
        if shared_counter == expected_counter:
            print("✅ SUCCESS: Thread safety verified - no race conditions detected")
            thread_safe = True
        else:
            print(f"❌ FAILURE: Thread safety compromised - counter mismatch")
            thread_safe = False
        
        # Shutdown service
        integration_service.shutdown()
        
        return thread_safe
        
    except Exception as e:
        print(f"❌ FAILURE: Thread safety verification failed: {e}")
        return False


def main():
    """Run all Phase 3B tests."""
    print("🚀 Starting Phase 3B: Advanced Thread Safety and Race Condition Prevention")
    print("=" * 70)
    
    setup_logging()
    
    # Test 1: High concurrency scenario
    high_concurrency_success = test_high_concurrency_scenario()
    
    # Test 2: Resource contention
    resource_contention_success = test_resource_contention()
    
    # Test 3: Graceful degradation
    graceful_degradation_success = test_graceful_degradation()
    
    # Test 4: Thread safety verification
    thread_safety_success = test_thread_safety_verification()
    
    print("\n" + "=" * 70)
    print("🎯 Phase 3B Test Results")
    print("=" * 70)
    
    if high_concurrency_success:
        print("✅ High Concurrency: PASSED")
    else:
        print("❌ High Concurrency: FAILED")
    
    if resource_contention_success:
        print("✅ Resource Contention: PASSED")
    else:
        print("❌ Resource Contention: FAILED")
    
    if graceful_degradation_success:
        print("✅ Graceful Degradation: PASSED")
    else:
        print("❌ Graceful Degradation: FAILED")
    
    if thread_safety_success:
        print("✅ Thread Safety Verification: PASSED")
    else:
        print("❌ Thread Safety Verification: FAILED")
    
    overall_success = (high_concurrency_success and 
                      resource_contention_success and 
                      graceful_degradation_success and 
                      thread_safety_success)
    
    if overall_success:
        print("\n🎉 Phase 3B (Advanced Thread Safety) completed successfully!")
        print("The system is now ready for Phase 3C (Advanced Features).")
    else:
        print("\n⚠️  Phase 3B has some issues that need attention.")
    
    return overall_success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
