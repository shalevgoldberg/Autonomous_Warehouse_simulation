-- Minimal conflict box locking functions for lane-based navigation
-- Safe to re-run: uses CREATE OR REPLACE and does not modify existing tables.
-- Aligned with existing schema columns: priority, locked_at, heartbeat_at.

SET search_path = public;

CREATE OR REPLACE FUNCTION try_acquire_conflict_box_lock(
    p_box_id VARCHAR,
    p_robot_id VARCHAR,
    p_priority INTEGER DEFAULT 0
)
RETURNS BOOLEAN AS $$
DECLARE
    v_locked_by VARCHAR;
    v_existing_priority INTEGER;
BEGIN
    -- Lock existing row for this box (if any) to ensure atomic decision
    SELECT locked_by_robot, priority
      INTO v_locked_by, v_existing_priority
      FROM conflict_box_locks
     WHERE box_id = p_box_id
     FOR UPDATE;

    IF NOT FOUND THEN
        -- No existing lock: create one
        INSERT INTO conflict_box_locks (box_id, locked_by_robot, priority, locked_at, heartbeat_at)
        VALUES (p_box_id, p_robot_id, p_priority, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP);
        RETURN TRUE;
    END IF;

    -- Already held by same robot
    IF v_locked_by = p_robot_id THEN
        RETURN TRUE;
    END IF;

    -- Priority preemption (higher wins)
    IF COALESCE(p_priority, 0) > COALESCE(v_existing_priority, 0) THEN
        UPDATE conflict_box_locks
           SET locked_by_robot = p_robot_id,
               priority = p_priority,
               locked_at = CURRENT_TIMESTAMP,
               heartbeat_at = CURRENT_TIMESTAMP
         WHERE box_id = p_box_id;
        RETURN TRUE;
    END IF;

    RETURN FALSE;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION release_conflict_box_lock(
    p_box_id VARCHAR,
    p_robot_id VARCHAR
)
RETURNS BOOLEAN AS $$
DECLARE
    v_count INTEGER;
BEGIN
    DELETE FROM conflict_box_locks
     WHERE box_id = p_box_id AND locked_by_robot = p_robot_id;
    GET DIAGNOSTICS v_count = ROW_COUNT;
    RETURN v_count > 0;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION heartbeat_conflict_box_lock(
    p_box_id VARCHAR,
    p_robot_id VARCHAR
)
RETURNS BOOLEAN AS $$
DECLARE
    v_count INTEGER;
BEGIN
    UPDATE conflict_box_locks
       SET heartbeat_at = CURRENT_TIMESTAMP
     WHERE box_id = p_box_id AND locked_by_robot = p_robot_id;
    GET DIAGNOSTICS v_count = ROW_COUNT;
    RETURN v_count > 0;
END;
$$ LANGUAGE plpgsql;



