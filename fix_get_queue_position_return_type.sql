-- Fix the return type of get_queue_position_optimized function
DROP FUNCTION IF EXISTS get_queue_position_optimized(character varying, character varying);

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







