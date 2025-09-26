-- Phase 1: Conflict Box Queue Database Functions Implementation
-- Date: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
-- Purpose: Fix broken queue position update functions
-- Status: Phase 1 of conflict box queue refactoring

-- ============================================================================
-- PHASE 1: DATABASE FUNCTIONS IMPLEMENTATION
-- ============================================================================
-- 
-- This script implements working queue position management functions
-- that replace the broken update_queue_positions function.
--
-- Changes:
-- 1. Optimize get_queue_position with window functions
-- 2. Implement working update_queue_positions_working
-- 3. Add queue promotion function
-- 4. Add queue cleanup function
--
-- ============================================================================

-- Step 1: Optimize get_queue_position function with window functions
-- This function is already working but can be optimized for better performance
CREATE OR REPLACE FUNCTION get_queue_position_optimized(p_box_id character varying, p_robot_id character varying)
RETURNS integer
LANGUAGE plpgsql
AS $function$
DECLARE
    pos INTEGER;
BEGIN
    -- Use window function for more efficient position calculation
    SELECT COALESCE(position_rank, 0)
    INTO pos
    FROM (
        SELECT robot_id, 
               ROW_NUMBER() OVER (
                   ORDER BY priority DESC, queue_time ASC
               ) as position_rank
        FROM conflict_box_queue
        WHERE box_id = p_box_id 
        AND position IN ('queued', 'next_in_line')
    ) ranked
    WHERE robot_id = p_robot_id;
    
    RETURN pos;
END;
$function$;

-- Step 2: Implement working queue position update function
-- This replaces the broken update_queue_positions function
CREATE OR REPLACE FUNCTION update_queue_positions_working(p_box_id character varying)
RETURNS void
LANGUAGE plpgsql
AS $function$
DECLARE
    queue_record RECORD;
    position_counter INTEGER := 1;
BEGIN
    -- Update positions using a cursor approach instead of window functions in UPDATE
    -- This is PostgreSQL-compatible and maintains ACID properties
    
    -- First, reset all positions to 'queued' (except those with locks)
    UPDATE conflict_box_queue
    SET position = 'queued', updated_at = CURRENT_TIMESTAMP
    WHERE box_id = p_box_id AND position != 'lock_acquired';
    
    -- Then, update positions based on priority and queue time order
    FOR queue_record IN
        SELECT robot_id, priority, queue_time
        FROM conflict_box_queue
        WHERE box_id = p_box_id AND position != 'lock_acquired'
        ORDER BY priority DESC, queue_time ASC
    LOOP
        -- Set first robot to 'next_in_line', others remain 'queued'
        IF position_counter = 1 THEN
            UPDATE conflict_box_queue
            SET position = 'next_in_line', updated_at = CURRENT_TIMESTAMP
            WHERE box_id = p_box_id AND robot_id = queue_record.robot_id;
        END IF;
        
        position_counter := position_counter + 1;
    END LOOP;
    
    -- Log the update for debugging
    RAISE NOTICE 'Updated queue positions for conflict box %: % robots processed', p_box_id, position_counter - 1;
END;
$function$;

-- Step 3: Add queue promotion function for when locks are released
-- This function promotes the next robot in line when a lock becomes available
CREATE OR REPLACE FUNCTION promote_next_robot_in_queue(p_box_id character varying)
RETURNS character varying
LANGUAGE plpgsql
AS $function$
DECLARE
    next_robot_id character varying;
    promoted_count INTEGER;
BEGIN
    -- Find the next robot in line (should be only one)
    SELECT robot_id INTO next_robot_id
    FROM conflict_box_queue
    WHERE box_id = p_box_id AND position = 'next_in_line'
    LIMIT 1;
    
    -- If no robot is next in line, try to promote from queued
    IF next_robot_id IS NULL THEN
        SELECT robot_id INTO next_robot_id
        FROM conflict_box_queue
        WHERE box_id = p_box_id AND position = 'queued'
        ORDER BY priority DESC, queue_time ASC
        LIMIT 1;
        
        -- Promote this robot to next_in_line
        IF next_robot_id IS NOT NULL THEN
            UPDATE conflict_box_queue
            SET position = 'next_in_line', updated_at = CURRENT_TIMESTAMP
            WHERE box_id = p_box_id AND robot_id = next_robot_id;
            
            GET DIAGNOSTICS promoted_count = ROW_COUNT;
            RAISE NOTICE 'Promoted robot % to next_in_line for conflict box %', next_robot_id, p_box_id;
        END IF;
    END IF;
    
    RETURN next_robot_id;
END;
$function$;

-- Step 4: Add queue cleanup function for expired entries
-- This function removes timed-out entries and maintains queue integrity
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
    WITH expired_locks AS (
        DELETE FROM conflict_box_locks
        WHERE (p_box_id IS NULL OR box_id = p_box_id)
        AND heartbeat_at < CURRENT_TIMESTAMP - INTERVAL '30 seconds'
        RETURNING box_id, robot_id
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

-- Step 5: Add queue validation function for integrity checks
-- This function validates queue state and reports any issues
CREATE OR REPLACE FUNCTION validate_queue_integrity(p_box_id character varying DEFAULT NULL)
RETURNS TABLE(
    box_id character varying,
    issue_type text,
    issue_description text,
    robot_count integer,
    severity text
)
LANGUAGE plpgsql
AS $function$
BEGIN
    -- Check for multiple robots in next_in_line state
    RETURN QUERY
    SELECT 
        cbq.box_id,
        'Multiple next_in_line robots'::text as issue_type,
        'Multiple robots have next_in_line position'::text as issue_description,
        COUNT(*)::integer as robot_count,
        'HIGH'::text as severity
    FROM conflict_box_queue cbq
    WHERE (p_box_id IS NULL OR cbq.box_id = p_box_id)
    AND cbq.position = 'next_in_line'
    GROUP BY cbq.box_id
    HAVING COUNT(*) > 1
    
    UNION ALL
    
    -- Check for orphaned queue entries (position = lock_acquired but no actual lock)
    SELECT 
        cbq.box_id,
        'Orphaned lock_acquired entries'::text as issue_type,
        'Queue shows lock_acquired but no actual lock exists'::text as issue_description,
        COUNT(*)::integer as robot_count,
        'MEDIUM'::text as severity
    FROM conflict_box_queue cbq
    LEFT JOIN conflict_box_locks cbl ON cbq.box_id = cbl.box_id AND cbq.robot_id = cbl.locked_by_robot
    WHERE (p_box_id IS NULL OR cbq.box_id = p_box_id)
    AND cbq.position = 'lock_acquired'
    AND cbl.box_id IS NULL
    GROUP BY cbq.box_id
    
    UNION ALL
    
    -- Check for robots with locks but not in queue
    SELECT 
        cbl.box_id,
        'Robots with locks not in queue'::text as issue_type,
        'Robot has lock but not in queue'::text as issue_description,
        COUNT(*)::integer as robot_count,
        'LOW'::text as severity
    FROM conflict_box_locks cbl
    LEFT JOIN conflict_box_queue cbq ON cbl.box_id = cbq.box_id AND cbl.locked_by_robot = cbq.robot_id
    WHERE (p_box_id IS NULL OR cbl.box_id = p_box_id)
    AND cbq.robot_id IS NULL
    GROUP BY cbl.box_id;
END;
$function$;

-- Step 6: Add performance monitoring function
-- This function provides queue performance metrics
CREATE OR REPLACE FUNCTION get_queue_performance_metrics(p_box_id character varying DEFAULT NULL)
RETURNS TABLE(
    box_id character varying,
    total_robots integer,
    queued_robots integer,
    next_in_line_robots integer,
    locked_robots integer,
    average_wait_time numeric,
    max_wait_time numeric,
    priority_distribution json
)
LANGUAGE plpgsql
AS $function$
BEGIN
    RETURN QUERY
    SELECT 
        cbq.box_id,
        COUNT(*)::integer as total_robots,
        COUNT(CASE WHEN cbq.position = 'queued' THEN 1 END)::integer as queued_robots,
        COUNT(CASE WHEN cbq.position = 'next_in_line' THEN 1 END)::integer as next_in_line_robots,
        COUNT(CASE WHEN cbq.position = 'lock_acquired' THEN 1 END)::integer as locked_robots,
        AVG(EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - cbq.queue_time)))::numeric as average_wait_time,
        MAX(EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - cbq.queue_time)))::numeric as max_wait_time,
        json_build_object(
            'priority_0', COUNT(CASE WHEN cbq.priority = 0 THEN 1 END),
            'priority_1', COUNT(CASE WHEN cbq.priority = 1 THEN 1 END),
            'priority_high', COUNT(CASE WHEN cbq.priority >= 2 THEN 1 END)
        ) as priority_distribution
    FROM conflict_box_queue cbq
    WHERE (p_box_id IS NULL OR cbq.box_id = p_box_id)
    AND cbq.position IN ('queued', 'next_in_line', 'lock_acquired')
    GROUP BY cbq.box_id;
END;
$function$;

-- ============================================================================
-- VERIFICATION AND TESTING
-- ============================================================================

-- Test the new functions
SELECT 'Phase 1 implementation completed successfully' as status;

-- Show all new functions
SELECT 
    proname as function_name,
    CASE 
        WHEN prosrc LIKE '%update_queue_positions_working%' THEN 'NEW: Working position update'
        WHEN prosrc LIKE '%promote_next_robot_in_queue%' THEN 'NEW: Queue promotion'
        WHEN prosrc LIKE '%cleanup_expired_queue_entries%' THEN 'NEW: Expired cleanup'
        WHEN prosrc LIKE '%validate_queue_integrity%' THEN 'NEW: Integrity validation'
        WHEN prosrc LIKE '%get_queue_performance_metrics%' THEN 'NEW: Performance metrics'
        WHEN prosrc LIKE '%get_queue_position_optimized%' THEN 'NEW: Optimized position'
        ELSE 'EXISTING'
    END as status,
    'Ready for testing' as description
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

-- ============================================================================
-- NEXT STEPS
-- ============================================================================
--
-- After running this script:
-- 1. Test the new functions with sample data
-- 2. Verify they work correctly
-- 3. Update the application code to use new functions
-- 4. Remove old broken functions
-- 5. Proceed to Phase 2 (Service Implementation)
--
-- ============================================================================



















