#!/usr/bin/env python3
"""
Test Script for Phase 2: Conflict Box Lock Manager Service

This script tests the newly implemented ConflictBoxLockManager service
to ensure it integrates correctly with the Phase 1 database functions.
"""

import os
import sys
import logging
from datetime import datetime

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from warehouse.impl.conflict_box_lock_manager_impl import ConflictBoxLockManagerImpl


def setup_logging():
    """Setup logging for the test."""
    logging.basicConfig(
        level=logging.INFO,
        format='[%(levelname)s] [%(name)s] %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def test_lock_manager_initialization():
    """Test the lock manager initialization."""
    print("\n🧪 Test 1: Lock Manager Initialization")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the lock manager
        lock_manager = ConflictBoxLockManagerImpl(connection_string)
        
        print("✅ SUCCESS: ConflictBoxLockManager initialized successfully")
        return lock_manager
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to initialize ConflictBoxLockManager: {e}")
        return None


def test_lock_status_checks(lock_manager):
    """Test basic lock status checking operations."""
    print("\n🧪 Test 2: Lock Status Checks")
    
    try:
        # Test with a known conflict box ID
        test_box_id = "0"  # Using the first conflict box from our data
        
        # Check if box is locked
        is_locked = lock_manager.is_locked(test_box_id)
        print(f"✅ SUCCESS: Lock status check completed for box {test_box_id}")
        print(f"   - Is locked: {is_locked}")
        
        # Get robot ID if locked
        locked_by_robot = lock_manager.get_locked_by_robot(test_box_id)
        print(f"   - Locked by robot: {locked_by_robot}")
        
        # Get lock info
        lock_info = lock_manager.get_lock_info(test_box_id)
        if lock_info:
            print(f"   - Lock info retrieved: {lock_info.locked_by_robot} (priority: {lock_info.priority})")
        else:
            print("   - No lock info available (box not locked)")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to check lock status: {e}")
        return False


def test_lock_health_monitoring(lock_manager):
    """Test lock health monitoring functionality."""
    print("\n🧪 Test 3: Lock Health Monitoring")
    
    try:
        # Test with a known conflict box ID
        test_box_id = "0"
        
        # Check lock health
        health_status = lock_manager.check_lock_health(test_box_id)
        
        if health_status:
            print(f"✅ SUCCESS: Lock health check completed for box {test_box_id}")
            print(f"   - Is healthy: {health_status.is_healthy}")
            print(f"   - Is expired: {health_status.is_expired}")
            print(f"   - Time since heartbeat: {health_status.time_since_heartbeat}")
            print(f"   - Robot ID: {health_status.robot_id}")
        else:
            print(f"✅ SUCCESS: No lock found for box {test_box_id} (expected if not locked)")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to check lock health: {e}")
        return False


def test_lock_validation(lock_manager):
    """Test lock validation functionality."""
    print("\n🧪 Test 4: Lock Validation")
    
    try:
        # Validate all locks
        validation_issues = lock_manager.validate_locks()
        
        print(f"✅ SUCCESS: Lock validation completed. Found {len(validation_issues)} issues.")
        
        # Display issues if any
        if validation_issues:
            for issue in validation_issues:
                print(f"   - {issue['severity']}: {issue['issue_type']} on box {issue['box_id']}")
                print(f"     Description: {issue['description']}")
        else:
            print("   - No validation issues found - all locks are healthy!")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to validate locks: {e}")
        return False


def test_lock_summary_generation(lock_manager):
    """Test lock summary generation functionality."""
    print("\n🧪 Test 5: Lock Summary Generation")
    
    try:
        # Generate locks summary
        summary = lock_manager.get_locks_summary()
        
        print("✅ SUCCESS: Locks summary generated successfully")
        print(f"   - Total locks: {summary['total_locks']}")
        print(f"   - Expired locks: {summary['expired_locks']}")
        print(f"   - Healthy locks: {summary['healthy_locks']}")
        print(f"   - System health: {summary['system_health']}")
        
        # Display priority distribution if available
        if summary['priority_distribution']:
            print("   - Priority distribution:")
            for priority, count in summary['priority_distribution'].items():
                print(f"     Priority {priority}: {count} locks")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to generate locks summary: {e}")
        return False


def test_cleanup_operations(lock_manager):
    """Test cleanup operations."""
    print("\n🧪 Test 6: Cleanup Operations")
    
    try:
        # Test cleanup for all conflict boxes
        cleaned_count = lock_manager.cleanup_expired_locks()
        
        print(f"✅ SUCCESS: Cleanup completed successfully. Cleaned {cleaned_count} expired locks.")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to perform cleanup operations: {e}")
        return False


def test_lock_acquisition_simulation(lock_manager):
    """Test lock acquisition simulation (without actually acquiring locks)."""
    print("\n🧪 Test 7: Lock Acquisition Simulation")
    
    try:
        # Test with a known conflict box ID
        test_box_id = "0"
        test_robot_id = "test_robot_001"
        
        # Check current lock status
        is_locked = lock_manager.is_locked(test_box_id)
        print(f"✅ SUCCESS: Lock acquisition simulation completed for box {test_box_id}")
        print(f"   - Current lock status: {'LOCKED' if is_locked else 'UNLOCKED'}")
        
        if is_locked:
            locked_by_robot = lock_manager.get_locked_by_robot(test_box_id)
            print(f"   - Currently locked by: {locked_by_robot}")
            print(f"   - Test robot {test_robot_id} would need higher priority to acquire lock")
        else:
            print(f"   - Test robot {test_robot_id} could acquire lock immediately")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to simulate lock acquisition: {e}")
        return False


def main():
    """Main test function."""
    print("🚀 Phase 2 Testing: Conflict Box Lock Manager Service")
    print("=" * 60)
    
    # Setup logging
    setup_logging()
    
    # Test results tracking
    test_results = []
    
    # Test 1: Initialization
    lock_manager = test_lock_manager_initialization()
    test_results.append(('Initialization', lock_manager is not None))
    
    if lock_manager is None:
        print("\n❌ CRITICAL: Cannot proceed with tests - lock manager initialization failed")
        return False
    
    # Test 2: Lock Status Checks
    test_results.append(('Lock Status Checks', test_lock_status_checks(lock_manager)))
    
    # Test 3: Lock Health Monitoring
    test_results.append(('Lock Health Monitoring', test_lock_health_monitoring(lock_manager)))
    
    # Test 4: Lock Validation
    test_results.append(('Lock Validation', test_lock_validation(lock_manager)))
    
    # Test 5: Lock Summary Generation
    test_results.append(('Lock Summary Generation', test_lock_summary_generation(lock_manager)))
    
    # Test 6: Cleanup Operations
    test_results.append(('Cleanup Operations', test_cleanup_operations(lock_manager)))
    
    # Test 7: Lock Acquisition Simulation
    test_results.append(('Lock Acquisition Simulation', test_lock_acquisition_simulation(lock_manager)))
    
    # Test Summary
    print("\n" + "=" * 60)
    print("📊 PHASE 2 LOCK MANAGER TEST RESULTS SUMMARY")
    print("=" * 60)
    
    passed_tests = 0
    total_tests = len(test_results)
    
    for test_name, result in test_results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"{test_name:.<30} {status}")
        if result:
            passed_tests += 1
    
    print("-" * 60)
    print(f"Total Tests: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")
    print(f"Success Rate: {(passed_tests / total_tests) * 100:.1f}%")
    
    if passed_tests == total_tests:
        print("\n🎉 ALL TESTS PASSED! Phase 2 Lock Manager is working correctly.")
        print("Ready to proceed with next Phase 2 components.")
        return True
    else:
        print(f"\n⚠️  {total_tests - passed_tests} test(s) failed. Please investigate before proceeding.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
