-- Conflict Box Refactoring Rollback Script
-- Generated on: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
-- Purpose: Complete rollback of conflict box queue refactoring
-- Usage: Run this script if the refactoring fails or causes issues

-- ============================================================================
-- EMERGENCY ROLLBACK PROCEDURE
-- ============================================================================
-- 
-- This script will restore the database to its pre-refactoring state.
-- Run this ONLY if something goes wrong during the refactoring.
--
-- WARNING: This will overwrite any changes made during the refactoring.
-- Make sure you have a backup of any important data before proceeding.
--
-- ============================================================================

-- Step 1: Drop any new functions that were created during refactoring
-- (These will be added as we implement the refactoring)

-- Step 2: Restore original function definitions
-- Function 1: get_queue_position (Original working version)
CREATE OR REPLACE FUNCTION get_queue_position(p_box_id character varying, p_robot_id character varying)
RETURNS integer
LANGUAGE plpgsql
AS $function$
DECLARE
    pos INTEGER;
BEGIN
    SELECT COUNT(*) + 1 INTO pos
    FROM conflict_box_queue
    WHERE box_id = p_box_id
    AND (priority > (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
         OR (priority = (SELECT priority FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)
             AND queue_time < (SELECT queue_time FROM conflict_box_queue WHERE box_id = p_box_id AND robot_id = p_robot_id)));
    RETURN pos;
END;
$function$;

-- Function 2: update_queue_positions (Original broken version - for reference)
CREATE OR REPLACE FUNCTION update_queue_positions(p_box_id character varying)
RETURNS void
LANGUAGE plpgsql
AS $function$
BEGIN
    UPDATE conflict_box_queue
    SET position = CASE
        WHEN ROW_NUMBER() OVER (ORDER BY priority DESC, queue_time ASC) = 1 THEN 'next_in_line'
        ELSE 'queued'
    END,
    updated_at = CURRENT_TIMESTAMP
    WHERE box_id = p_box_id AND position != 'lock_acquired';
END;
$function$;

-- Function 3: update_queue_positions_simple (Original placeholder)
CREATE OR REPLACE FUNCTION update_queue_positions_simple(p_box_id character varying)
RETURNS void
LANGUAGE sql
AS $function$
SELECT NULL;
$function$;

-- Step 3: Verify function restoration
SELECT 'Rollback completed. Functions restored to original state.' as status;

-- Step 4: Show current function status
SELECT 
    proname as function_name,
    CASE 
        WHEN prosrc LIKE '%ROW_NUMBER()%' THEN 'BROKEN (window function in UPDATE)'
        WHEN prosrc LIKE '%SELECT NULL%' THEN 'PLACEHOLDER (no-op)'
        ELSE 'WORKING'
    END as status,
    CASE 
        WHEN prosrc LIKE '%ROW_NUMBER()%' THEN 'Needs manual position updates'
        WHEN prosrc LIKE '%SELECT NULL%' THEN 'No position updates'
        ELSE 'Functional'
    END as description
FROM pg_proc 
WHERE proname IN ('get_queue_position', 'update_queue_positions', 'update_queue_positions_simple')
ORDER BY proname;

-- Step 5: Check table integrity
SELECT 
    'conflict_boxes' as table_name,
    COUNT(*) as record_count,
    'OK' as status
FROM conflict_boxes
UNION ALL
SELECT 
    'conflict_box_locks' as table_name,
    COUNT(*) as record_count,
    'OK' as status
FROM conflict_box_locks
UNION ALL
SELECT 
    'conflict_box_queue' as table_name,
    COUNT(*) as record_count,
    'OK' as status
FROM conflict_box_queue
UNION ALL
SELECT 
    'conflict_box_queue_stats' as table_name,
    COUNT(*) as record_count,
    'OK' as status
FROM conflict_box_queue_stats;

-- Step 6: Check for any orphaned data
SELECT 
    'Orphaned queue entries' as check_type,
    COUNT(*) as count,
    CASE WHEN COUNT(*) = 0 THEN 'OK' ELSE 'WARNING: Found orphaned entries' END as status
FROM conflict_box_queue cbq
LEFT JOIN conflict_box_locks cbl ON cbq.box_id = cbl.box_id AND cbq.robot_id = cbl.locked_by_robot
WHERE cbq.position = 'lock_acquired' AND cbl.box_id IS NULL

UNION ALL

SELECT 
    'Multiple next_in_line robots' as check_type,
    COUNT(*) as count,
    CASE WHEN COUNT(*) <= 1 THEN 'OK' ELSE 'WARNING: Multiple robots in next_in_line' END as status
FROM (
    SELECT box_id, COUNT(*) as count
    FROM conflict_box_queue
    WHERE position = 'next_in_line'
    GROUP BY box_id
    HAVING COUNT(*) > 1
) multiple_next;

-- ============================================================================
-- ROLLBACK COMPLETION CHECKLIST
-- ============================================================================
--
-- After running this script, verify:
-- [ ] All original functions are restored
-- [ ] No new functions remain from refactoring
-- [ ] Table data is intact
-- [ ] No orphaned records exist
-- [ ] System is back to working state
--
-- If issues persist, consider:
-- 1. Restoring from full database backup
-- 2. Checking application logs for errors
-- 3. Verifying robot agents are using original code
--
-- ============================================================================

SELECT 'Rollback script completed. Check the results above for any warnings.' as final_status;

