-- Task Management System Schema
-- Add these tables to the existing warehouse database schema

-- Raw Orders Queue
CREATE TABLE IF NOT EXISTS raw_orders (
    order_id TEXT PRIMARY KEY,
    item_id TEXT NOT NULL REFERENCES items(item_id),
    quantity INTEGER NOT NULL CHECK (quantity > 0),
    status TEXT NOT NULL DEFAULT 'pending' CHECK (status IN ('pending', 'processing', 'fulfilled', 'cancelled')),
    creation_time TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    completion_time TIMESTAMP,
    
    CONSTRAINT fk_item FOREIGN KEY (item_id) REFERENCES items(item_id)
);

-- Tasks
CREATE TABLE IF NOT EXISTS tasks (
    task_id TEXT PRIMARY KEY,
    order_id TEXT NOT NULL REFERENCES raw_orders(order_id),
    robot_id TEXT,
    shelf_id TEXT NOT NULL REFERENCES shelves(shelf_id),
    item_id TEXT NOT NULL REFERENCES items(item_id),
    quantity INTEGER NOT NULL CHECK (quantity > 0),
    status TEXT NOT NULL DEFAULT 'pending' CHECK (status IN ('pending', 'assigned', 'in_progress', 'completed', 'failed')),
    creation_time TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    start_time TIMESTAMP,
    completion_time TIMESTAMP,
    approach_position_x FLOAT NOT NULL,
    approach_position_y FLOAT NOT NULL,
    dropoff_position_x FLOAT NOT NULL,
    dropoff_position_y FLOAT NOT NULL,
    
    CONSTRAINT fk_order FOREIGN KEY (order_id) REFERENCES raw_orders(order_id),
    CONSTRAINT fk_shelf FOREIGN KEY (shelf_id) REFERENCES shelves(shelf_id),
    CONSTRAINT fk_item FOREIGN KEY (item_id) REFERENCES items(item_id)
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_raw_orders_status ON raw_orders(status);
CREATE INDEX IF NOT EXISTS idx_tasks_status ON tasks(status);
CREATE INDEX IF NOT EXISTS idx_tasks_robot_id ON tasks(robot_id);
CREATE INDEX IF NOT EXISTS idx_tasks_order_id ON tasks(order_id);

-- Metrics tracking view
CREATE OR REPLACE VIEW task_metrics AS
SELECT
    COUNT(*) AS total_tasks,
    SUM(CASE WHEN status = 'pending' THEN 1 ELSE 0 END) AS pending_tasks,
    SUM(CASE WHEN status = 'assigned' THEN 1 ELSE 0 END) AS assigned_tasks,
    SUM(CASE WHEN status = 'in_progress' THEN 1 ELSE 0 END) AS in_progress_tasks,
    SUM(CASE WHEN status = 'completed' THEN 1 ELSE 0 END) AS completed_tasks,
    SUM(CASE WHEN status = 'failed' THEN 1 ELSE 0 END) AS failed_tasks,
    AVG(CASE 
        WHEN status = 'completed' AND completion_time IS NOT NULL 
        THEN EXTRACT(EPOCH FROM (completion_time - start_time)) 
        ELSE NULL 
    END) AS avg_completion_time_seconds
FROM tasks;

-- Order fulfillment metrics view
CREATE OR REPLACE VIEW order_metrics AS
SELECT
    COUNT(*) AS total_orders,
    SUM(CASE WHEN status = 'pending' THEN 1 ELSE 0 END) AS pending_orders,
    SUM(CASE WHEN status = 'processing' THEN 1 ELSE 0 END) AS processing_orders,
    SUM(CASE WHEN status = 'fulfilled' THEN 1 ELSE 0 END) AS fulfilled_orders,
    SUM(CASE WHEN status = 'cancelled' THEN 1 ELSE 0 END) AS cancelled_orders,
    AVG(CASE 
        WHEN status = 'fulfilled' AND completion_time IS NOT NULL 
        THEN EXTRACT(EPOCH FROM (completion_time - creation_time)) 
        ELSE NULL 
    END) AS avg_fulfillment_time_seconds
FROM raw_orders;

-- Robot performance metrics view
CREATE OR REPLACE VIEW robot_metrics AS
SELECT
    robot_id,
    COUNT(*) AS total_tasks,
    SUM(CASE WHEN status = 'completed' THEN 1 ELSE 0 END) AS completed_tasks,
    SUM(CASE WHEN status = 'failed' THEN 1 ELSE 0 END) AS failed_tasks,
    AVG(CASE 
        WHEN status = 'completed' AND completion_time IS NOT NULL AND start_time IS NOT NULL
        THEN EXTRACT(EPOCH FROM (completion_time - start_time)) 
        ELSE NULL 
    END) AS avg_task_time_seconds
FROM tasks
WHERE robot_id IS NOT NULL
GROUP BY robot_id;

-- Functions for working with tasks

-- Function to get the next available task for a robot
CREATE OR REPLACE FUNCTION get_next_task(p_robot_id TEXT)
RETURNS SETOF tasks AS $$
BEGIN
    -- First check if robot already has an active task
    RETURN QUERY
    SELECT *
    FROM tasks
    WHERE robot_id = p_robot_id AND status IN ('assigned', 'in_progress')
    LIMIT 1;
    
    -- If no rows returned, get the next pending task
    IF NOT FOUND THEN
        RETURN QUERY
        SELECT *
        FROM tasks
        WHERE status = 'pending'
        ORDER BY creation_time ASC
        LIMIT 1;
    END IF;
END;
$$ LANGUAGE plpgsql; 