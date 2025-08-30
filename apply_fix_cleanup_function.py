#!/usr/bin/env python3
"""
Apply a fix to the cleanup_expired_queue_entries PostgreSQL function.

This script corrects the RETURNING list for expired lock deletions to use
locked_by_robot instead of the non-existent robot_id column.
"""

import os
import sys
import psycopg2


SQL_FIX = r"""
CREATE OR REPLACE FUNCTION cleanup_expired_queue_entries(p_box_id character varying DEFAULT NULL)
RETURNS INTEGER
LANGUAGE plpgsql
AS $function$
DECLARE
    timeout_count INTEGER := 0;
    expired_locks_count INTEGER := 0;
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
            RETURNING box_id, locked_by_robot
        )
        SELECT COUNT(*) INTO expired_locks_count FROM expired_locks;

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

        total_cleaned := timeout_count + expired_locks_count + old_entries;

        -- Log cleanup results
        RAISE NOTICE 'Cleanup completed: % timeouts, % expired locks, % old entries = % total', 
                     timeout_count, expired_locks_count, old_entries, total_cleaned;

        RETURN total_cleaned;

    EXCEPTION
        WHEN OTHERS THEN
            -- Rollback transaction on error
            v_error_message := 'Error during cleanup: ' || SQLERRM;
            RAISE EXCEPTION '%', v_error_message;
    END;
END;
$function$;
"""


def main() -> int:
    db_password = os.getenv("WAREHOUSE_DB_PASSWORD", "renaspolter")
    dsn = f"host=localhost user=postgres password={db_password} dbname=warehouse_sim"
    try:
        with psycopg2.connect(dsn) as conn:
            with conn.cursor() as cur:
                cur.execute(SQL_FIX)
                conn.commit()
        print("✅ Applied cleanup_expired_queue_entries fix successfully")
        return 0
    except Exception as e:
        print(f"❌ Failed to apply fix: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())









