-- Conflict Box Functions Backup Script
-- Generated on: $(Get-Date -Format "yyyy-MM-dd HH:mm:ss")
-- Purpose: Backup current function definitions before refactoring
-- Usage: Run this script to restore functions if needed

-- Function 1: get_queue_position
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

-- Function 2: update_queue_positions
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

-- Function 3: update_queue_positions_simple
CREATE OR REPLACE FUNCTION update_queue_positions_simple(p_box_id character varying)
RETURNS void
LANGUAGE sql
AS $function$
SELECT NULL;
$function$;

-- Verification queries
SELECT 'Functions restored successfully' as status;
SELECT proname, prosrc FROM pg_proc WHERE proname IN ('get_queue_position', 'update_queue_positions', 'update_queue_positions_simple');

