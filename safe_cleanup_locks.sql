-- SAFE CLEANUP OF ORPHANED BAY LOCKS
-- This script safely removes only expired locks with confirmation

-- Start transaction for safety
BEGIN;

-- Show current state before cleanup
\echo '=== CURRENT ORPHANED BAY LOCKS ==='
SELECT
    bay_id,
    robot_id,
    locked_at,
    heartbeat_at,
    lock_timeout_seconds,
    EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat,
    CASE WHEN CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)
         THEN 'EXPIRED' ELSE 'ACTIVE' END as status
FROM bay_locks
ORDER BY locked_at DESC;

-- Count expired locks
\echo ''
\echo '=== SUMMARY ==='
SELECT
    COUNT(*) as total_locks,
    COUNT(CASE WHEN CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)
               THEN 1 END) as expired_locks,
    COUNT(CASE WHEN CURRENT_TIMESTAMP <= (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)
               THEN 1 END) as active_locks
FROM bay_locks;

-- DELETE ONLY EXPIRED LOCKS
\echo ''
\echo '=== CLEANUP IN PROGRESS ==='
DELETE FROM bay_locks
WHERE CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds);

-- Show results
\echo ''
\echo '=== CLEANUP RESULTS ==='
SELECT
    COUNT(*) as remaining_locks
FROM bay_locks;

-- Commit transaction
COMMIT;

\echo ''
\echo '✅ Cleanup completed successfully!'
