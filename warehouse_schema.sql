-- Warehouse Simulation Database Schema
-- PostgreSQL database for autonomous warehouse logistics simulation

-- Create database (run this separately if needed)
-- CREATE DATABASE warehouse_sim;

-- Connect to the database
-- \c warehouse_sim;

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- 1. Items Table - Store item definitions
CREATE TABLE items (
    item_id VARCHAR(36) PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 2. Shelves Table - Store shelf information
CREATE TABLE shelves (
    shelf_id VARCHAR(36) PRIMARY KEY,
    position_x DECIMAL(10,3) NOT NULL,
    position_y DECIMAL(10,3) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 3. Shelf Inventory Table - Store items on each shelf
CREATE TABLE shelf_inventory (
    id SERIAL PRIMARY KEY,
    shelf_id VARCHAR(36) NOT NULL,
    item_id VARCHAR(36) NOT NULL,
    quantity INTEGER NOT NULL DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (shelf_id) REFERENCES shelves(shelf_id) ON DELETE CASCADE,
    FOREIGN KEY (item_id) REFERENCES items(item_id) ON DELETE CASCADE,
    UNIQUE(shelf_id, item_id)
);

-- 4. Robots Table - Store only robot definitions
CREATE TABLE robots (
    robot_id VARCHAR(36) PRIMARY KEY,
    name VARCHAR(255),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 5. Warehouse Map Table - Store warehouse layout
CREATE TABLE warehouse_map (
    id SERIAL PRIMARY KEY,
    width INTEGER NOT NULL,
    height INTEGER NOT NULL,
    grid_size DECIMAL(5,3) NOT NULL,
    grid_data JSONB NOT NULL, -- Store the numpy grid as JSON
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- 6. Shelf Approach Cells Table - Store precomputed approach positions for each shelf
CREATE TABLE shelf_approach_cells (
    id SERIAL PRIMARY KEY,
    shelf_id VARCHAR(36) NOT NULL,
    approach_x DECIMAL(10,3) NOT NULL,
    approach_y DECIMAL(10,3) NOT NULL,
    distance_to_shelf DECIMAL(10,3) NOT NULL,
    grid_x INTEGER NOT NULL,
    grid_y INTEGER NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (shelf_id) REFERENCES shelves(shelf_id) ON DELETE CASCADE,
    UNIQUE(shelf_id, grid_x, grid_y)
);

-- 7. Simulation Metrics Table - Store simulation performance data
CREATE TABLE simulation_metrics (
    id SERIAL PRIMARY KEY,
    metric_name VARCHAR(100) NOT NULL,
    metric_value DECIMAL(15,3),
    metric_timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    simulation_run_id VARCHAR(36)
);

-- Indexes for Performance
CREATE INDEX idx_shelf_inventory_shelf_id ON shelf_inventory(shelf_id);
CREATE INDEX idx_shelf_inventory_item_id ON shelf_inventory(item_id);
CREATE INDEX idx_shelves_position ON shelves(position_x, position_y);
CREATE INDEX idx_shelf_approach_cells_shelf_id ON shelf_approach_cells(shelf_id);
CREATE INDEX idx_shelf_approach_cells_position ON shelf_approach_cells(approach_x, approach_y);
CREATE INDEX idx_shelf_approach_cells_distance ON shelf_approach_cells(shelf_id, distance_to_shelf);
CREATE INDEX idx_simulation_metrics_name ON simulation_metrics(metric_name);
CREATE INDEX idx_simulation_metrics_timestamp ON simulation_metrics(metric_timestamp);
CREATE INDEX idx_simulation_metrics_run_id ON simulation_metrics(simulation_run_id);

-- Views for Common Queries
-- View for shelf inventory with item names
CREATE VIEW shelf_inventory_view AS
SELECT 
    s.shelf_id,
    s.position_x,
    s.position_y,
    i.item_id,
    i.name as item_name,
    si.quantity,
    si.updated_at
FROM shelves s
JOIN shelf_inventory si ON s.shelf_id = si.shelf_id
JOIN items i ON si.item_id = i.item_id;

-- View for total inventory by item
CREATE VIEW total_inventory_view AS
SELECT 
    i.item_id,
    i.name,
    SUM(si.quantity) as total_quantity,
    COUNT(DISTINCT s.shelf_id) as shelf_count
FROM items i
LEFT JOIN shelf_inventory si ON i.item_id = si.item_id
LEFT JOIN shelves s ON si.shelf_id = s.shelf_id
GROUP BY i.item_id, i.name;

-- Triggers for Data Integrity
-- Trigger function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Create triggers for all tables with updated_at columns
CREATE TRIGGER update_items_updated_at BEFORE UPDATE ON items
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_shelves_updated_at BEFORE UPDATE ON shelves
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_shelf_inventory_updated_at BEFORE UPDATE ON shelf_inventory
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_robots_updated_at BEFORE UPDATE ON robots
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- Insert sample data for testing
INSERT INTO items (item_id, name) VALUES 
    ('book-fantasy', 'Fantasy Novel'),
    ('book-scifi', 'Science Fiction'),
    ('book-mystery', 'Mystery Novel'),
    ('electronics-phone', 'Smartphone'),
    ('electronics-laptop', 'Laptop'),
    ('electronics-tablet', 'Tablet'),
    ('clothing-tshirt', 'T-Shirt'),
    ('clothing-jeans', 'Jeans'),
    ('clothing-jacket', 'Jacket');

-- Insert sample robot
INSERT INTO robots (robot_id, name) VALUES 
    ('robot_001', 'Warehouse Robot 1');

-- Create comments for documentation
COMMENT ON TABLE items IS 'Catalog of all items that can be stored in the warehouse';
COMMENT ON TABLE shelves IS 'Physical shelf locations in the warehouse';
COMMENT ON TABLE shelf_inventory IS 'Current inventory quantities on each shelf';
COMMENT ON TABLE shelf_approach_cells IS 'Precomputed approach positions for each shelf to optimize path planning';
COMMENT ON TABLE robots IS 'Robot definitions and configurations';
COMMENT ON TABLE warehouse_map IS 'Warehouse layout and grid configuration';
COMMENT ON TABLE simulation_metrics IS 'Performance metrics collected during simulation runs';

-- Grant permissions (adjust as needed for your setup)
-- GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO your_user;
-- GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO your_user; 