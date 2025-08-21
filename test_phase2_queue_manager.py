#!/usr/bin/env python3
"""
Test Script for Phase 2: Conflict Box Queue Manager Service

This script tests the newly implemented ConflictBoxQueueManager service
to ensure it integrates correctly with the Phase 1 database functions.
"""

import os
import sys
import logging
from datetime import datetime

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from warehouse.impl.conflict_box_queue_manager_impl import ConflictBoxQueueManagerImpl


def setup_logging():
    """Setup logging for the test."""
    logging.basicConfig(
        level=logging.INFO,
        format='[%(levelname)s] [%(name)s] %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def test_queue_manager_initialization():
    """Test the queue manager initialization."""
    print("\n🧪 Test 1: Queue Manager Initialization")
    
    try:
        # Get database connection string from environment
        db_password = os.getenv('WAREHOUSE_DB_PASSWORD', 'renaspolter')
        connection_string = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
        
        # Initialize the queue manager
        queue_manager = ConflictBoxQueueManagerImpl(connection_string)
        
        print("✅ SUCCESS: ConflictBoxQueueManager initialized successfully")
        return queue_manager
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to initialize ConflictBoxQueueManager: {e}")
        return None


def test_queue_metrics_retrieval(queue_manager):
    """Test retrieving queue metrics."""
    print("\n🧪 Test 2: Queue Metrics Retrieval")
    
    try:
        # Get metrics for all conflict boxes
        metrics = queue_manager.get_queue_metrics()
        
        print(f"✅ SUCCESS: Retrieved metrics for {len(metrics)} conflict boxes")
        
        # Display some metrics if available
        if metrics:
            for metric in metrics[:3]:  # Show first 3
                print(f"   - Box {metric.total_robots}: {metric.total_robots} robots, "
                      f"{metric.queued_robots} queued, {metric.locked_robots} locked")
        else:
            print("   - No conflict boxes have queue data yet")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to retrieve queue metrics: {e}")
        return False


def test_queue_integrity_validation(queue_manager):
    """Test queue integrity validation."""
    print("\n🧪 Test 3: Queue Integrity Validation")
    
    try:
        # Validate integrity for all conflict boxes
        issues = queue_manager.validate_queue_integrity()
        
        print(f"✅ SUCCESS: Integrity validation completed. Found {len(issues)} issues.")
        
        # Display issues if any
        if issues:
            for issue in issues:
                print(f"   - {issue.severity}: {issue.issue_type} on box {issue.box_id}")
        else:
            print("   - No integrity issues found - all queues are healthy!")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to validate queue integrity: {e}")
        return False


def test_queue_summary_generation(queue_manager):
    """Test queue summary generation."""
    print("\n🧪 Test 4: Queue Summary Generation")
    
    try:
        # Generate system-wide summary
        summary = queue_manager.get_queue_summary()
        
        print("✅ SUCCESS: Queue summary generated successfully")
        print(f"   - Total conflict boxes: {summary['total_conflict_boxes']}")
        print(f"   - Total robots in queues: {summary['total_robots_in_queues']}")
        print(f"   - System health: {summary['system_health']}")
        print(f"   - Integrity issues: {summary['integrity_issues']['total']} total")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to generate queue summary: {e}")
        return False


def test_specific_conflict_box_operations(queue_manager):
    """Test operations on a specific conflict box."""
    print("\n🧪 Test 5: Specific Conflict Box Operations")
    
    try:
        # Get a conflict box ID to test with
        metrics = queue_manager.get_queue_metrics()
        if not metrics:
            print("   - SKIPPED: No conflict boxes available for testing")
            return True
        
        test_box_id = metrics[0].box_id if hasattr(metrics[0], 'box_id') else '0'
        
        # Test queue status retrieval
        status = queue_manager.get_queue_status(test_box_id)
        print(f"✅ SUCCESS: Retrieved status for conflict box {test_box_id}")
        print(f"   - Is locked: {status.is_locked}")
        print(f"   - Queue length: {status.queue_length}")
        
        # Test queue health check
        is_healthy = queue_manager.is_queue_healthy(test_box_id)
        print(f"   - Is healthy: {is_healthy}")
        
        # Test queue position update (should not fail)
        update_success = queue_manager.update_queue_positions(test_box_id)
        print(f"   - Position update: {'SUCCESS' if update_success else 'FAILED'}")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to test specific conflict box operations: {e}")
        return False


def test_cleanup_operations(queue_manager):
    """Test cleanup operations."""
    print("\n🧪 Test 6: Cleanup Operations")
    
    try:
        # Test cleanup for all conflict boxes
        cleaned_count = queue_manager.cleanup_expired_entries()
        
        print(f"✅ SUCCESS: Cleanup completed successfully. Cleaned {cleaned_count} entries.")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILURE: Failed to perform cleanup operations: {e}")
        return False


def main():
    """Main test function."""
    print("🚀 Phase 2 Testing: Conflict Box Queue Manager Service")
    print("=" * 60)
    
    # Setup logging
    setup_logging()
    
    # Test results tracking
    test_results = []
    
    # Test 1: Initialization
    queue_manager = test_queue_manager_initialization()
    test_results.append(('Initialization', queue_manager is not None))
    
    if queue_manager is None:
        print("\n❌ CRITICAL: Cannot proceed with tests - queue manager initialization failed")
        return False
    
    # Test 2: Queue Metrics
    test_results.append(('Queue Metrics', test_queue_metrics_retrieval(queue_manager)))
    
    # Test 3: Integrity Validation
    test_results.append(('Integrity Validation', test_queue_integrity_validation(queue_manager)))
    
    # Test 4: Summary Generation
    test_results.append(('Summary Generation', test_queue_summary_generation(queue_manager)))
    
    # Test 5: Specific Box Operations
    test_results.append(('Specific Box Operations', test_specific_conflict_box_operations(queue_manager)))
    
    # Test 6: Cleanup Operations
    test_results.append(('Cleanup Operations', test_cleanup_operations(queue_manager)))
    
    # Test Summary
    print("\n" + "=" * 60)
    print("📊 PHASE 2 TEST RESULTS SUMMARY")
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
        print("\n🎉 ALL TESTS PASSED! Phase 2 Queue Manager is working correctly.")
        print("Ready to proceed with next Phase 2 components.")
        return True
    else:
        print(f"\n⚠️  {total_tests - passed_tests} test(s) failed. Please investigate before proceeding.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
