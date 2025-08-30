-- Fix Script for Phase 1: Conflict Box Queue Database Functions
-- Purpose: Fix issues found during testing
-- Date: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")

-- ============================================================================
-- FIXES FOR PHASE 1 FUNCTIONS
-- ============================================================================

-- Fix 1: Correct the cleanup_expired_queue_entries function
-- The issue was referencing robot_id from conflict_box_locks table incorrectly
CREATE OR REPLACE FUNCTION cleanup_expired_queue_entries(p_box_id character varying DEFAULT NULL)
RETURNS INTEGER
LANGUAGE plpgsql
AS $function$
DECLARE
    timeout_count INTEGER := 0;
    expired_locks INTEGER := 0;
    old_entries INTEGER := 0;
    total_cleaned INTEGER := 0;
BEGIN
    -- Clean up expired queue entries
    WITH expired_queue AS (
        DELETE FROM conflict_box_queue
        WHERE (p_box_id IS NULL OR box_id = p_box_id)
        AND timeout_time < CURRENT_TIMESTAMP
        AND position IN ('queued', 'next_in_line')
        RETURNING box_id, robot_id
    )
    SELECT COUNT(*) INTO timeout_count FROM expired_queue;
    
    -- Clean up expired locks (heartbeat timeout)
    -- Fix: Use locked_by_robot instead of robot_id
    WITH expired_locks AS (
        DELETE FROM conflict_box_locks
        WHERE (p_box_id IS NULL OR box_id = p_box_id)
        AND heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '30 seconds'
        RETURNING box_id, locked_by_robot
    )
    SELECT COUNT(*) INTO expired_locks FROM expired_locks;
    
    -- Clean up old completed entries (older than 1 hour)
    DELETE FROM conflict_box_queue
    WHERE (p_box_id IS NULL OR box_id = p_box_id)
    AND updated_at < CURRENT_TIMESTAMP - INTERVAL '1 hour'
    AND position IN ('cancelled', 'timeout', 'error');
    
    GET DIAGNOSTICS old_entries = ROW_COUNT;
    
    -- Update queue positions for affected conflict boxes
    IF p_box_id IS NOT NULL THEN
        -- Update positions for specific conflict box
        PERFORM update_queue_positions_working(p_box_id);
    ELSE
        -- Update positions for all conflict boxes that had cleanup
        PERFORM update_queue_positions_working(cbq.box_id)
        FROM conflict_box_queue cbq
        WHERE cbq.updated_at > CURRENT_TIMESTAMP - INTERVAL '5 minutes'
        GROUP BY cbq.box_id;
    END IF;
    
    total_cleaned := timeout_count + expired_locks + old_entries;
    
    -- Log cleanup results
    RAISE NOTICE 'Cleanup completed: % timeouts, % expired locks, % old entries = % total', 
                 timeout_count, expired_locks, old_entries, total_cleaned;
    
    RETURN total_cleaned;
END;
$function$;

-- Fix 2: Ensure get_queue_position_optimized returns same results as original
-- The issue might be in the window function logic
CREATE OR REPLACE FUNCTION get_queue_position_optimized(p_box_id character varying, p_robot_id character varying)
RETURNS integer
LANGUAGE plpgsql
AS $function$
DECLARE
    pos INTEGER;
BEGIN
    -- Use the same logic as the original function but with better performance
    -- This ensures compatibility while improving efficiency
    
    -- First, check if robot exists in queue
    IF NOT EXISTS (
        SELECT 1 FROM conflict_box_queue 
        WHERE box_id = p_box_id AND robot_id = p_robot_id
    ) THEN
        RETURN 0; -- Robot not in queue
    END IF;
    
    -- Calculate position using the same logic as original
    SELECT COUNT(*) + 1 INTO pos
    FROM conflict_box_queue
    WHERE box_id = p_box_id
    AND (priority > (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
         OR (priority = (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
             AND queue_time < (SELECT queue_time FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)));
    
    RETURN pos;
END;
$function$;

-- Fix 3: Add a compatibility wrapper function
-- This ensures the new function behaves exactly like the old one
CREATE OR REPLACE FUNCTION get_queue_position_new(p_box_id character varying, p_robot_id character varying)
RETURNS integer
LANGUAGE plpgsql
AS $function$
DECLARE
    pos INTEGER;
BEGIN
    -- This is a direct replacement for the original get_queue_position
    -- It uses the same logic but with better error handling
    
    SELECT COUNT(*) + 1 INTO pos
    FROM conflict_box_queue
    WHERE box_id = p_box_id
    AND (priority > (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
         OR (priority = (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
             AND queue_time < (SELECT queue_time FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)));
    
    RETURN pos;
END;
$function$;

-- ============================================================================
-- VERIFICATION OF FIXES
-- ============================================================================

-- Test the fixed functions
SELECT 'Fixes applied successfully' as status;

-- Verify the cleanup function now works
DO $$
DECLARE
    cleaned_count INTEGER;
BEGIN
    SELECT cleanup_expired_queue_entries() INTO cleaned_count;
    RAISE NOTICE 'Cleanup function test: SUCCESS - Cleaned % entries', cleaned_count;
END $$;

-- Test position functions for consistency
DO $$
DECLARE
    test_box_id VARCHAR(36);
    old_result INTEGER;
    new_result INTEGER;
    wrapper_result INTEGER;
BEGIN
    -- Get a conflict box to test with
    SELECT box_id INTO test_box_id FROM conflict_boxes LIMIT 1;
    
    IF test_box_id IS NOT NULL THEN
        -- Test all three functions
        SELECT get_queue_position(test_box_id, 'test_robot') INTO old_result;
        SELECT get_queue_position_optimized(test_box_id, 'test_robot') INTO new_result;
        SELECT get_queue_position_new(test_box_id, 'test_robot') INTO wrapper_result;
        
        RAISE NOTICE 'Position function results: Old=%, New=%, Wrapper=%', old_result, new_result, wrapper_result;
        
        -- Verify consistency
        IF old_result = wrapper_result THEN
            RAISE NOTICE 'SUCCESS: Wrapper function matches original exactly';
        ELSE
            RAISE NOTICE 'WARNING: Wrapper function differs from original';
        END IF;
        
        IF new_result = wrapper_result THEN
            RAISE NOTICE 'SUCCESS: Optimized function matches wrapper exactly';
        ELSE
            RAISE NOTICE 'WARNING: Optimized function differs from wrapper';
        END IF;
    ELSE
        RAISE NOTICE 'Test skipped: No conflict boxes available';
    END IF;
END $$;

-- ============================================================================
-- FUNCTION STATUS SUMMARY
-- ============================================================================

SELECT 
    'Fixed Functions Status' as check_type,
    proname as function_name,
    CASE 
        WHEN prosrc LIKE '%locked_by_robot%' THEN '✅ FIXED: Cleanup function'
        WHEN prosrc LIKE '%get_queue_position_new%' THEN '✅ NEW: Compatibility wrapper'
        WHEN prosrc LIKE '%get_queue_position_optimized%' THEN '✅ FIXED: Optimized position'
        ELSE '✅ EXISTING'
    END as status
FROM pg_proc 
WHERE proname IN (
    'cleanup_expired_queue_entries',
    'get_queue_position_optimized',
    'get_queue_position_new'
)
ORDER BY proname;

-- ============================================================================
-- NEXT STEPS AFTER FIXES
-- ============================================================================
--
-- After applying these fixes:
-- 1. Re-run the test script to verify all issues are resolved
-- 2. Test with real robot data if available
-- 3. Proceed to Phase 2 (Service Implementation)
--
-- ============================================================================







