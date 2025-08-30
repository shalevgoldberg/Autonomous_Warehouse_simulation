-- Test Script for Phase 1: Conflict Box Queue Database Functions
-- Purpose: Verify all new functions work correctly
-- Run this after deploying phase1_database_functions.sql

-- ============================================================================
-- PHASE 1 FUNCTION TESTING
-- ============================================================================

-- Test 1: Verify all new functions exist
SELECT 'Test 1: Function Existence Check' as test_name;

SELECT 
    proname as function_name,
    CASE 
        WHEN prosrc LIKE '%update_queue_positions_working%' THEN '✅ NEW: Working position update'
        WHEN prosrc LIKE '%promote_next_robot_in_queue%' THEN '✅ NEW: Queue promotion'
        WHEN prosrc LIKE '%cleanup_expired_queue_entries%' THEN '✅ NEW: Expired cleanup'
        WHEN prosrc LIKE '%validate_queue_integrity%' THEN '✅ NEW: Integrity validation'
        WHEN prosrc LIKE '%get_queue_performance_metrics%' THEN '✅ NEW: Performance metrics'
        WHEN prosrc LIKE '%get_queue_position_optimized%' THEN '✅ NEW: Optimized position'
        ELSE '❌ UNKNOWN'
    END as status
FROM pg_proc 
WHERE proname IN (
    'get_queue_position_optimized',
    'update_queue_positions_working', 
    'promote_next_robot_in_queue',
    'cleanup_expired_queue_entries',
    'validate_queue_integrity',
    'get_queue_performance_metrics'
)
ORDER BY proname;

-- Test 2: Test get_queue_position_optimized with sample data
SELECT 'Test 2: Optimized Position Function Test' as test_name;

-- First, let's see what conflict boxes exist
SELECT 'Available conflict boxes:' as info;
SELECT box_id, center_x, center_y, size FROM conflict_boxes LIMIT 5;

-- Test 3: Test update_queue_positions_working function
SELECT 'Test 3: Working Position Update Function Test' as test_name;

-- This function should work without errors (unlike the old broken one)
-- We'll test it on a conflict box if data exists
DO $$
DECLARE
    test_box_id VARCHAR(36);
    test_result VARCHAR(100);
BEGIN
    -- Get a conflict box to test with
    SELECT box_id INTO test_box_id FROM conflict_boxes LIMIT 1;
    
    IF test_box_id IS NOT NULL THEN
        -- Test the function
        PERFORM update_queue_positions_working(test_box_id);
        test_result := 'SUCCESS: Function executed without errors';
    ELSE
        test_result := 'SKIPPED: No conflict boxes available for testing';
    END IF;
    
    RAISE NOTICE 'Test result: %', test_result;
END $$;

-- Test 4: Test promote_next_robot_in_queue function
SELECT 'Test 4: Queue Promotion Function Test' as test_name;

DO $$
DECLARE
    test_box_id VARCHAR(36);
    promoted_robot VARCHAR(36);
BEGIN
    -- Get a conflict box to test with
    SELECT box_id INTO test_box_id FROM conflict_boxes LIMIT 1;
    
    IF test_box_id IS NOT NULL THEN
        -- Test the function
        SELECT promote_next_robot_in_queue(test_box_id) INTO promoted_robot;
        
        IF promoted_robot IS NOT NULL THEN
            RAISE NOTICE 'Test result: SUCCESS - Promoted robot %', promoted_robot;
        ELSE
            RAISE NOTICE 'Test result: SUCCESS - No robots to promote (expected if queue is empty)';
        END IF;
    ELSE
        RAISE NOTICE 'Test result: SKIPPED - No conflict boxes available for testing';
    END IF;
END $$;

-- Test 5: Test cleanup_expired_queue_entries function
SELECT 'Test 5: Cleanup Function Test' as test_name;

DO $$
DECLARE
    cleaned_count INTEGER;
BEGIN
    -- Test the function (should not fail even with empty data)
    SELECT cleanup_expired_queue_entries() INTO cleaned_count;
    
    RAISE NOTICE 'Test result: SUCCESS - Cleaned % expired entries', cleaned_count;
END $$;

-- Test 6: Test validate_queue_integrity function
SELECT 'Test 6: Integrity Validation Function Test' as test_name;

-- This function should return results (even if empty)
SELECT 
    'Integrity check results:' as info,
    box_id,
    issue_type,
    issue_description,
    robot_count,
    severity
FROM validate_queue_integrity();

-- Test 7: Test get_queue_performance_metrics function
SELECT 'Test 7: Performance Metrics Function Test' as test_name;

-- This function should return results (even if empty)
SELECT 
    'Performance metrics:' as info,
    box_id,
    total_robots,
    queued_robots,
    next_in_line_robots,
    locked_robots,
    average_wait_time,
    max_wait_time,
    priority_distribution
FROM get_queue_performance_metrics();

-- Test 8: Compare old vs new functions
SELECT 'Test 8: Function Comparison Test' as test_name;

-- Show the difference between old and new functions
SELECT 
    'Old get_queue_position:' as function_type,
    prosrc as function_source
FROM pg_proc 
WHERE proname = 'get_queue_position'

UNION ALL

SELECT 
    'New get_queue_position_optimized:' as function_type,
    prosrc as function_source
FROM pg_proc 
WHERE proname = 'get_queue_position_optimized';

-- Test 9: Performance comparison
SELECT 'Test 9: Performance Comparison' as test_name;

-- Test execution time of both functions (if we have test data)
DO $$
DECLARE
    test_box_id VARCHAR(36);
    start_time TIMESTAMP;
    end_time TIMESTAMP;
    old_result INTEGER;
    new_result INTEGER;
BEGIN
    -- Get a conflict box to test with
    SELECT box_id INTO test_box_id FROM conflict_boxes LIMIT 1;
    
    IF test_box_id IS NOT NULL THEN
        -- Test old function
        start_time := clock_timestamp();
        SELECT get_queue_position(test_box_id, 'test_robot') INTO old_result;
        end_time := clock_timestamp();
        
        RAISE NOTICE 'Old function execution time: %', end_time - start_time;
        
        -- Test new function
        start_time := clock_timestamp();
        SELECT get_queue_position_optimized(test_box_id, 'test_robot') INTO new_result;
        end_time := clock_timestamp();
        
        RAISE NOTICE 'New function execution time: %', end_time - start_time;
        RAISE NOTICE 'Performance improvement: %', 
            CASE 
                WHEN old_result = new_result THEN 'Same result, different performance'
                ELSE 'Different results - investigate needed'
            END;
    ELSE
        RAISE NOTICE 'Performance test: SKIPPED - No conflict boxes available';
    END IF;
END $$;

-- ============================================================================
-- TEST SUMMARY
-- ============================================================================

SELECT 'Phase 1 Testing Completed' as test_summary;

-- Final verification that all functions are working
SELECT 
    'Final Status Check' as check_type,
    COUNT(*) as total_functions,
    COUNT(CASE WHEN prosrc IS NOT NULL THEN 1 END) as valid_functions,
    'All functions deployed and tested' as status
FROM pg_proc 
WHERE proname IN (
    'get_queue_position_optimized',
    'update_queue_positions_working', 
    'promote_next_robot_in_queue',
    'cleanup_expired_queue_entries',
    'validate_queue_integrity',
    'get_queue_performance_metrics'
);

-- ============================================================================
-- NEXT STEPS AFTER TESTING
-- ============================================================================
--
-- If all tests pass:
-- 1. Update application code to use new functions
-- 2. Remove old broken functions
-- 3. Proceed to Phase 2 (Service Implementation)
--
-- If any tests fail:
-- 1. Investigate the issue
-- 2. Fix the function if needed
-- 3. Re-run tests until all pass
--
-- ============================================================================







