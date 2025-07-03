-- Lane-Based Navigation Schema Extension
-- PostgreSQL extension for autonomous warehouse lane-based navigation
-- This extends the existing warehouse_schema.sql

-- Enable UUID extension (if not already enabled)
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- 1. Lanes Table - Store lane definitions from CSV
CREATE TABLE IF NOT EXISTS lanes (
    lane_id VARCHAR(36) PRIMARY KEY,
    direction VARCHAR(10) NOT NULL CHECK (direction IN ('N', 'S', 'E', 'W', 'B')),
    waypoints JSONB NOT NULL, -- Array of [x,y] coordinates from CSV
    is_goal_only BOOLEAN DEFAULT FALSE, -- True for bay spurs
    bay_id VARCHAR(36), -- Only for bay spurs
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 2. Conflict Boxes Table - Store conflict box definitions
CREATE TABLE IF NOT EXISTS conflict_boxes (
    box_id VARCHAR(36) PRIMARY KEY,
    center_x DECIMAL(10,3) NOT NULL,
    center_y DECIMAL(10,3) NOT NULL,
    size DECIMAL(5,3) NOT NULL CHECK (size > 0),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 3. Navigation Graph Nodes Table - Store graph nodes for path planning
CREATE TABLE IF NOT EXISTS navigation_graph_nodes (
    node_id VARCHAR(36) PRIMARY KEY,
    position_x DECIMAL(10,3) NOT NULL,
    position_y DECIMAL(10,3) NOT NULL,
    directions JSONB NOT NULL, -- Available exit directions
    is_conflict_box BOOLEAN DEFAULT FALSE,
    conflict_box_id VARCHAR(36) REFERENCES conflict_boxes(box_id),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 4. Navigation Graph Edges Table - Store graph edges for path planning
CREATE TABLE IF NOT EXISTS navigation_graph_edges (
    edge_id VARCHAR(36) PRIMARY KEY DEFAULT uuid_generate_v4(),
    from_node VARCHAR(36) NOT NULL REFERENCES navigation_graph_nodes(node_id),
    to_node VARCHAR(36) NOT NULL REFERENCES navigation_graph_nodes(node_id),
    direction VARCHAR(10) NOT NULL CHECK (direction IN ('N', 'S', 'E', 'W')),
    distance DECIMAL(10,3) NOT NULL CHECK (distance > 0),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(from_node, to_node)
);

-- 5. Blocked Cells Table - Store dynamically blocked lanes/cells
CREATE TABLE IF NOT EXISTS blocked_cells (
    cell_id VARCHAR(36) PRIMARY KEY,
    unblock_time TIMESTAMP NOT NULL,
    blocked_by_robot VARCHAR(36) REFERENCES robots(robot_id),
    block_reason VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 6. Shelf Locks Table - Enhanced shelf locking for lane-based navigation
CREATE TABLE IF NOT EXISTS shelf_locks (
    shelf_id VARCHAR(36) PRIMARY KEY REFERENCES shelves(shelf_id),
    locked_by_robot VARCHAR(36) NOT NULL REFERENCES robots(robot_id),
    lock_timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    heartbeat_timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lock_timeout_seconds INTEGER DEFAULT 300 -- 5 minutes default timeout
);

-- Performance Indexes
CREATE INDEX IF NOT EXISTS idx_lanes_direction ON lanes(direction);
CREATE INDEX IF NOT EXISTS idx_lanes_bay_id ON lanes(bay_id) WHERE bay_id IS NOT NULL;
CREATE INDEX IF NOT EXISTS idx_conflict_boxes_position ON conflict_boxes(center_x, center_y);
CREATE INDEX IF NOT EXISTS idx_navigation_graph_nodes_position ON navigation_graph_nodes(position_x, position_y);
CREATE INDEX IF NOT EXISTS idx_navigation_graph_nodes_conflict_box ON navigation_graph_nodes(conflict_box_id) WHERE conflict_box_id IS NOT NULL;
CREATE INDEX IF NOT EXISTS idx_navigation_graph_edges_from_node ON navigation_graph_edges(from_node);
CREATE INDEX IF NOT EXISTS idx_navigation_graph_edges_to_node ON navigation_graph_edges(to_node);
CREATE INDEX IF NOT EXISTS idx_blocked_cells_unblock_time ON blocked_cells(unblock_time);
CREATE INDEX IF NOT EXISTS idx_blocked_cells_robot ON blocked_cells(blocked_by_robot);
CREATE INDEX IF NOT EXISTS idx_shelf_locks_robot ON shelf_locks(locked_by_robot);
CREATE INDEX IF NOT EXISTS idx_shelf_locks_heartbeat ON shelf_locks(heartbeat_timestamp);

-- Triggers for updating timestamps
CREATE TRIGGER IF NOT EXISTS update_lanes_updated_at BEFORE UPDATE ON lanes
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER IF NOT EXISTS update_conflict_boxes_updated_at BEFORE UPDATE ON conflict_boxes
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER IF NOT EXISTS update_navigation_graph_nodes_updated_at BEFORE UPDATE ON navigation_graph_nodes
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER IF NOT EXISTS update_navigation_graph_edges_updated_at BEFORE UPDATE ON navigation_graph_edges
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Views for Common Queries
-- View for lanes with waypoint details
CREATE OR REPLACE VIEW lanes_detailed AS
SELECT 
    lane_id,
    direction,
    waypoints,
    is_goal_only,
    bay_id,
    jsonb_array_length(waypoints) as waypoint_count,
    created_at,
    updated_at
FROM lanes;

-- View for conflict boxes with associated nodes
CREATE OR REPLACE VIEW conflict_boxes_with_nodes AS
SELECT 
    cb.box_id,
    cb.center_x,
    cb.center_y,
    cb.size,
    COUNT(ngn.node_id) as associated_nodes,
    cb.created_at,
    cb.updated_at
FROM conflict_boxes cb
LEFT JOIN navigation_graph_nodes ngn ON cb.box_id = ngn.conflict_box_id
GROUP BY cb.box_id, cb.center_x, cb.center_y, cb.size, cb.created_at, cb.updated_at;

-- View for navigation graph connectivity
CREATE OR REPLACE VIEW navigation_graph_connectivity AS
SELECT 
    n.node_id,
    n.position_x,
    n.position_y,
    n.is_conflict_box,
    COUNT(e_out.edge_id) as outgoing_edges,
    COUNT(e_in.edge_id) as incoming_edges,
    COUNT(e_out.edge_id) + COUNT(e_in.edge_id) as total_connections
FROM navigation_graph_nodes n
LEFT JOIN navigation_graph_edges e_out ON n.node_id = e_out.from_node
LEFT JOIN navigation_graph_edges e_in ON n.node_id = e_in.to_node
GROUP BY n.node_id, n.position_x, n.position_y, n.is_conflict_box;

-- View for currently blocked cells
CREATE OR REPLACE VIEW active_blocked_cells AS
SELECT 
    cell_id,
    blocked_by_robot,
    block_reason,
    unblock_time,
    EXTRACT(EPOCH FROM (unblock_time - CURRENT_TIMESTAMP)) as seconds_until_unblock,
    created_at
FROM blocked_cells
WHERE unblock_time > CURRENT_TIMESTAMP;

-- View for expired shelf locks (for cleanup)
CREATE OR REPLACE VIEW expired_shelf_locks AS
SELECT 
    sl.shelf_id,
    sl.locked_by_robot,
    sl.lock_timestamp,
    sl.heartbeat_timestamp,
    sl.lock_timeout_seconds,
    EXTRACT(EPOCH FROM (CURRENT_TIMESTAMP - sl.heartbeat_timestamp)) as seconds_since_heartbeat
FROM shelf_locks sl
WHERE CURRENT_TIMESTAMP > (sl.heartbeat_timestamp + INTERVAL '1 second' * sl.lock_timeout_seconds);

-- Functions for lane-based navigation
-- Function to cleanup expired blocked cells
CREATE OR REPLACE FUNCTION cleanup_expired_blocked_cells()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER;
BEGIN
    DELETE FROM blocked_cells WHERE unblock_time <= CURRENT_TIMESTAMP;
    GET DIAGNOSTICS deleted_count = ROW_COUNT;
    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- Function to cleanup expired shelf locks
CREATE OR REPLACE FUNCTION cleanup_expired_shelf_locks()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER;
BEGIN
    DELETE FROM shelf_locks 
    WHERE CURRENT_TIMESTAMP > (heartbeat_timestamp + INTERVAL '1 second' * lock_timeout_seconds);
    GET DIAGNOSTICS deleted_count = ROW_COUNT;
    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- Function to get blocked cells for a specific robot
CREATE OR REPLACE FUNCTION get_blocked_cells_for_robot(p_robot_id VARCHAR(36))
RETURNS TABLE(cell_id VARCHAR(36), unblock_time TIMESTAMP, block_reason VARCHAR(255)) AS $$
BEGIN
    RETURN QUERY
    SELECT bc.cell_id, bc.unblock_time, bc.block_reason
    FROM blocked_cells bc
    WHERE bc.unblock_time > CURRENT_TIMESTAMP
    AND (bc.blocked_by_robot != p_robot_id OR bc.blocked_by_robot IS NULL);
END;
$$ LANGUAGE plpgsql;

-- Comments for documentation
COMMENT ON TABLE lanes IS 'Lane definitions for lane-based navigation system';
COMMENT ON TABLE conflict_boxes IS 'Conflict boxes for intersection management';
COMMENT ON TABLE navigation_graph_nodes IS 'Navigation graph nodes for path planning';
COMMENT ON TABLE navigation_graph_edges IS 'Navigation graph edges connecting nodes';
COMMENT ON TABLE blocked_cells IS 'Dynamically blocked lanes/cells for obstacle avoidance';
COMMENT ON TABLE shelf_locks IS 'Enhanced shelf locking with heartbeat mechanism';

COMMENT ON COLUMN lanes.waypoints IS 'JSON array of [x,y] coordinate pairs defining the lane path';
COMMENT ON COLUMN navigation_graph_nodes.directions IS 'JSON array of available exit directions (N,S,E,W)';
COMMENT ON COLUMN blocked_cells.unblock_time IS 'Timestamp when the cell will be automatically unblocked';
COMMENT ON COLUMN shelf_locks.heartbeat_timestamp IS 'Last heartbeat from robot holding the lock'; 