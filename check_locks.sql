SELECT
    bay_id,
    robot_id,
    locked_at,
    heartbeat_at,
    lock_timeout_seconds,
    EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - heartbeat_at)) as seconds_since_heartbeat,
    CURRENT_TIMESTAMP > (heartbeat_at + INTERVAL '1 second' * lock_timeout_seconds) as is_expired
FROM bay_locks
ORDER BY locked_at DESC;
