#!/usr/bin/env python3
"""
Debug script to test timeout configuration and identify hanging issues.
"""
import time
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from unittest.mock import Mock

def slow_operation(operation_id, delay=0.1):
    """Simulate a slow operation that might cause hanging."""
    print(f"Starting operation {operation_id}")
    time.sleep(delay)
    print(f"Completed operation {operation_id}")
    return f"result_{operation_id}"

def test_timeout_behavior():
    """Test how timeouts behave with different configurations."""
    print("=== Testing Timeout Behavior ===")
    
    # Test 1: Short timeout (1 second)
    print("\n1. Testing with 1 second timeout:")
    start_time = time.time()
    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = {
                executor.submit(slow_operation, i, 0.5): i 
                for i in range(4)
            }
            
            results = []
            for future in as_completed(futures, timeout=1.0):
                try:
                    result = future.result(timeout=0.5)
                    results.append(result)
                    print(f"Got result: {result}")
                except Exception as e:
                    print(f"Future failed: {e}")
                    future.cancel()
            
            # Cancel any remaining futures
            for future in futures:
                if not future.done():
                    future.cancel()
                    print(f"Cancelled future for operation {futures[future]}")
                    
    except Exception as e:
        print(f"Timeout test failed: {e}")
    
    end_time = time.time()
    print(f"Test 1 completed in {end_time - start_time:.3f}s")
    
    # Test 2: Longer timeout (5 seconds) - this might hang on slow computers
    print("\n2. Testing with 5 second timeout:")
    start_time = time.time()
    try:
        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = {
                executor.submit(slow_operation, i, 0.1): i 
                for i in range(8)
            }
            
            results = []
            for future in as_completed(futures, timeout=5.0):
                try:
                    result = future.result(timeout=1.0)
                    results.append(result)
                    print(f"Got result: {result}")
                except Exception as e:
                    print(f"Future failed: {e}")
                    future.cancel()
            
            # Cancel any remaining futures
            for future in futures:
                if not future.done():
                    future.cancel()
                    print(f"Cancelled future for operation {futures[future]}")
                    
    except Exception as e:
        print(f"Timeout test failed: {e}")
    
    end_time = time.time()
    print(f"Test 2 completed in {end_time - start_time:.3f}s")
    
    print("\n=== Timeout Test Complete ===")

if __name__ == "__main__":
    test_timeout_behavior() 