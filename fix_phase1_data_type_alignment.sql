-- Fix Phase 1: Data Type Alignment and Logic Consistency
-- Date: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
-- Purpose: Fix data type mismatches and align position logic with database schema
-- Status: Critical fix for Phase 1 functions

-- ============================================================================
-- IMMEDIATE FIXES: DATA TYPE ALIGNMENT
-- ============================================================================
-- 
-- This script fixes the critical data type mismatches between:
-- 1. Database schema (string-based positions: 'queued', 'next_in_line', 'lock_acquired')
-- 2. Phase 1 functions (which were returning integers)
-- 3. Service layer expectations
--
-- Changes:
-- 1. Fix get_queue_position_optimized to return string positions
-- 2. Standardize position logic across all functions
-- 3. Ensure consistency with database schema
-- 4. Add proper error handling and transaction management
--
-- ============================================================================

-- Fix 1: Correct get_queue_position_optimized to return string position
-- This function should return the actual position value from the database, not an integer
CREATE OR REPLACE FUNCTION get_queue_position_optimized(p_box_id character varying, p_robot_id character varying)
RETURNS character varying
LANGUAGE plpgsql
AS $function$
DECLARE
    v_position character varying;
BEGIN
    -- Get the actual position value from the database
    SELECT position
    INTO v_position
    FROM conflict_box_queue
    WHERE box_id = p_box_id AND robot_id = p_robot_id;
    
    -- Return the position if found, otherwise return 'not_found'
    IF v_position IS NOT NULL THEN
        RETURN v_position;
    ELSE
        RETURN 'not_found';
    END IF;
    
EXCEPTION
    WHEN OTHERS THEN
        RAISE EXCEPTION 'Error getting queue position: %', SQLERRM;
END;
$function$;

-- Fix 2: Correct update_queue_positions_working with proper transaction handling
CREATE OR REPLACE FUNCTION update_queue_positions_working(p_box_id character varying)
RETURNS void
LANGUAGE plpgsql
AS $function$
DECLARE
    queue_record RECORD;
    position_counter INTEGER := 1;
    v_error_message TEXT;
BEGIN
    -- Start transaction
    BEGIN
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
        
    EXCEPTION
        WHEN OTHERS THEN
            -- Rollback transaction on error
            v_error_message := 'Error updating queue positions: ' || SQLERRM;
            RAISE EXCEPTION '%', v_error_message;
    END;
END;
$function$;

-- Fix 3: Correct promote_next_robot_in_queue with proper error handling
CREATE OR REPLACE FUNCTION promote_next_robot_in_queue(p_box_id character varying)
RETURNS character varying
LANGUAGE plpgsql
AS $function$
DECLARE
    next_robot_id character varying;
    promoted_count INTEGER;
    v_error_message TEXT;
BEGIN
    -- Start transaction
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
        
    EXCEPTION
        WHEN OTHERS THEN
            -- Rollback transaction on error
            v_error_message := 'Error promoting next robot: ' || SQLERRM;
            RAISE EXCEPTION '%', v_error_message;
    END;
END;
$function$;

-- Fix 4: Correct cleanup_expired_queue_entries with proper transaction handling
CREATE OR REPLACE FUNCTION cleanup_expired_queue_entries(p_box_id character varying DEFAULT NULL)
RETURNS INTEGER
LANGUAGE plpgsql
AS $function$
DECLARE
    timeout_count INTEGER := 0;
    expired_locks INTEGER := 0;
    old_entries INTEGER := 0;
    total_cleaned INTEGER := 0;
    v_error_message TEXT;
BEGIN
    -- Start transaction
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
        
    EXCEPTION
        WHEN OTHERS THEN
            -- Rollback transaction on error
            v_error_message := 'Error during cleanup: ' || SQLERRM;
            RAISE EXCEPTION '%', v_error_message;
    END;
END;
$function$;

-- Fix 5: Add a new function to get queue position as integer (for backward compatibility)
-- This function returns the numeric position in the queue (1-based)
CREATE OR REPLACE FUNCTION get_queue_position_numeric(p_box_id character varying, p_robot_id character varying)
RETURNS integer
LANGUAGE plpgsql
AS $function$
DECLARE
    v_position INTEGER;
BEGIN
    -- Use window function to calculate numeric position
    SELECT position_rank
    INTO v_position
    FROM (
        SELECT robot_id, 
               ROW_NUMBER() OVER (
                   PARTITION BY box_id
                   ORDER BY priority DESC, queue_time ASC
               ) as position_rank
        FROM conflict_box_queue
        WHERE box_id = p_box_id
        AND position IN ('queued', 'next_in_line')
    ) ranked
    WHERE robot_id = p_robot_id;
    
    -- Return position if found, otherwise return 0
    RETURN COALESCE(v_position, 0);
    
EXCEPTION
    WHEN OTHERS THEN
        RAISE EXCEPTION 'Error getting numeric queue position: %', SQLERRM;
END;
$function$;

-- Fix 6: Add a function to get queue statistics with proper position handling
CREATE OR REPLACE FUNCTION get_queue_statistics(p_box_id character varying DEFAULT NULL)
RETURNS TABLE(
    box_id character varying,
    total_robots integer,
    queued_robots integer,
    next_in_line_robots integer,
    lock_acquired_robots integer,
    average_wait_time numeric,
    max_wait_time numeric
) AS
$function$
BEGIN
    RETURN QUERY
    SELECT
        cbq.box_id,
        COUNT(cbq.robot_id)::integer as total_robots,
        COUNT(CASE WHEN cbq.position = 'queued' THEN 1 END)::integer as queued_robots,
        COUNT(CASE WHEN cbq.position = 'next_in_line' THEN 1 END)::integer as next_in_line_robots,
        COUNT(CASE WHEN cbq.position = 'lock_acquired' THEN 1 END)::integer as lock_acquired_robots,
        COALESCE(AVG(EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - cbq.queue_time))), 0)::numeric as average_wait_time,
        COALESCE(MAX(EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - cbq.queue_time))), 0)::numeric as max_wait_time
    FROM conflict_box_queue cbq
    WHERE (p_box_id IS NULL OR cbq.box_id = p_box_id)
    GROUP BY cbq.box_id;
END;
$function$ LANGUAGE plpgsql;

-- ============================================================================
-- VERIFICATION QUERIES
-- ============================================================================

-- Verify function signatures
SELECT 
    proname as function_name,
    pg_get_function_result(oid) as return_type,
    pg_get_function_arguments(oid) as arguments
FROM pg_proc 
WHERE proname IN (
    'get_queue_position_optimized',
    'update_queue_positions_working', 
    'promote_next_robot_in_queue',
    'cleanup_expired_queue_entries',
    'get_queue_position_numeric',
    'get_queue_statistics'
)
ORDER BY proname;

-- Verify position values in database
SELECT DISTINCT position FROM conflict_box_queue ORDER BY position;

-- ============================================================================
-- MIGRATION NOTES
-- ============================================================================
--
-- IMPORTANT: These changes ensure that:
-- 1. All functions now return data types that match the database schema
-- 2. Position logic is consistent across all functions
-- 3. Error handling and transaction management is improved
-- 4. Backward compatibility is maintained where possible
--
-- The service layer should now work correctly with these functions.
-- ============================================================================




















