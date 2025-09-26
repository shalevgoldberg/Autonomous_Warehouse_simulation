-- Safe cleanup of orphaned bay locks
-- This script only deletes locks that are confirmed to be expired

-- First, show what will be deleted
SELECT
    bay_id,
    robot_id,
    locked_at,
    heartbeat_at,
    lock_timeout_seconds,
    EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat
FROM bay_locks
WHERE CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds)
ORDER BY locked_at DESC;

-- Count how many will be deleted
SELECT COUNT(*) as expired_locks_count
FROM bay_locks
WHERE CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds);

-- Proceed with cleanup (uncomment the DELETE below to actually clean up)
-- DELETE FROM bay_locks
-- WHERE CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds);

-- Verify cleanup
-- SELECT COUNT(*) as remaining_locks FROM bay_locks;
