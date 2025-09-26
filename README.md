# Autonomous Warehouse Simulation System

**Autonomous Warehouse v2** is a comprehensive multi-robot warehouse simulation system demonstrating advanced concepts in robotics, artificial intelligence, and software engineering. The system features autonomous robots navigating a warehouse environment, processing orders, managing inventory, and coordinating through sophisticated algorithms.

## 📋 Prerequisites

### Software Dependencies
- **Python**: 3.11+
- **PostgreSQL**: 12+ with psycopg2 driver
- **MuJoCo**: 2.3.3+ (physics simulation engine)
- **Git**: For version control

## 🛠️ Installation

### Step 1: Environment Setup

#### 1.1 Clone the Repository
```bash
git clone <repository-url>
cd autonomous-warehouse-v2
```

#### 1.2 Create Virtual Environment
```bash
# Windows PowerShell
python -m venv venv311
.\venv311\Scripts\Activate.ps1

# Linux/macOS
python3 -m venv venv311
source venv311/bin/activate
```

#### 1.3 Install Dependencies
```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### Step 2: Database Setup

#### 2.1 Install PostgreSQL
- **Windows**: Download from [PostgreSQL Official Site](https://www.postgresql.org/download/windows/)
- **Ubuntu/Debian**: `sudo apt-get install postgresql postgresql-contrib`
- **macOS**: `brew install postgresql`

#### 2.2 Create Database
```bash
# Connect to PostgreSQL as superuser
psql -U postgres

# Create database and user (REPLACE with your own secure credentials)
CREATE DATABASE warehouse_sim;
CREATE USER your_username WITH PASSWORD 'your_secure_password';
GRANT ALL PRIVILEGES ON DATABASE warehouse_sim TO your_username;
ALTER USER your_username CREATEDB;
\q
```

#### 2.3 Configure Database Connection
```bash
# RECOMMENDED: Use DATABASE_URL (Single environment variable - easiest setup)
# Windows PowerShell
$env:DATABASE_URL="postgresql://postgres:your_secure_password@localhost:5432/warehouse_sim"

# Windows Command Prompt
set DATABASE_URL="postgresql://postgres:your_secure_password@localhost:5432/warehouse_sim"

# Linux/macOS
export DATABASE_URL="postgresql://postgres:your_secure_password@localhost:5432/warehouse_sim"

# ALTERNATIVE: Use individual environment variables (if you prefer)
# Windows PowerShell
$env:WAREHOUSE_DB_HOST="localhost"
$env:WAREHOUSE_DB_PORT="5432"
$env:WAREHOUSE_DB_NAME="warehouse_sim"
$env:WAREHOUSE_DB_USER="postgres"
$env:WAREHOUSE_DB_PASSWORD="your_secure_password"

# Replace your_secure_password with your actual PostgreSQL password
```

#### 2.4 Initialize Database Schema
```bash
# Run the schema setup script (use your database username)
psql -U your_username -d warehouse_sim -f warehouse_schema_lane_based.sql
```

### Step 3: MuJoCo Installation

#### 3.1 Install MuJoCo
```bash
# Using pip (recommended)
pip install mujoco==2.3.3

# Verify installation
python -c "import mujoco; print('MuJoCo version:', mujoco.__version__)"
```

#### 3.2 MuJoCo License
- MuJoCo requires a license key for versions 2.1+
- Obtain a free academic license from [MuJoCo Website](https://mujoco.org/)
- Set the license key as environment variable: `MUJOCO_LICENSE_KEY`

### Step 4: Warehouse Data Setup

#### 4.1 Generate Warehouse from CSV 
```bash
# Generate complete warehouse data from CSV layout file
# This creates navigation graph, shelves, and populates inventory. default:
# extended_warehouse.csv
python load_new_warehouse.py --warehouse YOUR_WAREHOUSE.csv

# Available warehouse layouts:
# - sample_warehouse.csv (small demo layout)
# - extended_warehouse.csv (larger warehouse with more shelves)
# - custom_layout.csv (create your own warehouse layout)

# The script will generate:
# - Navigation graph (conflict boxes, nodes, edges)
# - Shelves from warehouse layout
# - Demo inventory with sample items
# - Export inventory status to CSV
```

#### 4.2 Inventory Management CLI
The `populate_inventory_data.py` script is a command-line interface (CLI) tool for managing warehouse inventory after the initial warehouse setup. It supports operations like adding, deleting, moving, and exporting inventory data. The script only works with pre-existing shelves in the database and does not create or modify shelves.

**Prerequisites**:
- Database connection must be configured (see Step 2.3).
- Shelves must exist in the database (run `load_new_warehouse.py` first or ensure shelves are present).
- The script uses JSON files for input (no CSV support).
- For faster execution, the script disables the KPI recorder internally.

**Available Commands**:
- **`add-sample`**: Add curated sample inventory items with random shelf assignment.
  ```bash
  # Windows PowerShell
  $env:WAREHOUSE_DB_PASSWORD="your_secure_password"; python populate_inventory_data.py add-sample
  # Linux/macOS
  export WAREHOUSE_DB_PASSWORD="your_secure_password"; python3 populate_inventory_data.py add-sample
  ```
  - Adds sample items (e.g., books, electronics, clothing) with realistic quantities.
  - Uses random assignment strategy (default).
  - Expected Output: Logs the number of entries added (e.g., "Added sample inventory entries: 12").

- **`add`**: Add inventory from a JSON file.
  ```bash
  # Windows PowerShell
  $env:WAREHOUSE_DB_PASSWORD="your_secure_password"; python populate_inventory_data.py add --file path\to\items.json
  # Linux/macOS
  export WAREHOUSE_DB_PASSWORD="your_secure_password"; python3 populate_inventory_data.py add --file path/to/items.json
  ```
  - JSON Format Example: `[{ "item_id": "laptop_001", "name": "Gaming Laptop", "quantity": 10, "shelf_id": "optional", "category": "electronics" }]`.
  - Validates records and assigns items to shelves using the specified strategy (default: random).
  - Expected Output: Logs the number of entries added or validation errors.

- **`delete-all`**: Delete all inventory entries.
  ```bash
  $env:WAREHOUSE_DB_PASSWORD="your_secure_password"; python populate_inventory_data.py delete-all
  ```
  - Removes all rows from the `shelf_inventory` table.
  - Expected Output: Logs the number of deleted rows (e.g., "Deleted all inventory rows: 20").

- **`delete-items`**: Delete inventory for specific item IDs.
  ```bash
  $env:WAREHOUSE_DB_PASSWORD="your_secure_password"; python populate_inventory_data.py delete-items --item-ids item_A,item_B --purge-items
  ```
  - Supports comma-separated item IDs and optional purging of items with no remaining inventory.
  - Expected Output: Logs the number of deleted rows.

- **`export`**: Export inventory status to a CSV file.
  ```bash
  $env:WAREHOUSE_DB_PASSWORD="your_secure_password"; python populate_inventory_data.py export --file inventory_status.csv
  ```
  - Generates a CSV with inventory data (including reserved quantities).
  - Expected Output: Logs success and file path (e.g., "Exported inventory CSV: inventory_status.csv").

- **`move`**: Move a quantity of an item between shelves.
  ```bash
  $env:WAREHOUSE_DB_PASSWORD="your_secure_password"; python populate_inventory_data.py move --item-id item_A --from-shelf shelf_1_1 --to-shelf shelf_2_1 --quantity 5
  ```
  - Updates inventory by decrementing from one shelf and incrementing another.
  - Expected Output: Logs success or failure (e.g., "Moved 5 of item_A from shelf_1_1 to shelf_2_1").

**Notes**:
- All commands require a valid database connection. If shelves don't exist, the script will log an error and exit.
- Logging: The script logs all operations, warnings, and errors to the console (e.g., for invalid JSON or missing fields).
- Strategy Options: For `add-sample` and `add`, you can specify `--strategy` (e.g., `--strategy load_balanced`), but "random" is the default.
- Safety: The script is read-only for shelves and uses transactions for data integrity. No mocks are used—everything interacts with the real database.
- Performance: The script disables the KPI recorder for faster shutdown, so there's no delay on exit.

#### 4.3 Quick Setup Test
```bash
# Test database connection (optional)
python -c "
from utils.database_config import get_database_config
try:
    config = get_database_config()
    print('✅ Database configuration loaded successfully')
    print(f'   Host: {config[\"host\"]}:{config[\"port\"]}')
    print(f'   Database: {config[\"database\"]}')
    print(f'   User: {config[\"user\"]}')
except Exception as e:
    print(f'❌ Database configuration error: {e}')
    print('   Please check your DATABASE_URL or WAREHOUSE_DB_* environment variables')
"
```

#### 4.4 Robot Registry Setup and Configuration

The **Robot Registry** is a database-backed system for managing robot identities and their persistent runtime state (position and battery level). This enables robots to maintain their state across simulation runs and supports deterministic robot selection.

##### Database Schema Overview

The robot registry uses two main tables:

**`robots` table:**
- `robot_id`: Unique identifier (36-character UUID, MuJoCo-safe format)
- `name`: Human-friendly name (optional)
- `created_at`, `updated_at`: Timestamps

**`robot_runtime_state` table:**
- `robot_id`: Foreign key to robots table
- `pos_x`, `pos_y`, `theta`: Position data (NULL when unplaced)
- `battery_level`: Battery percentage (0.0-1.0, persists across runs)

##### Robot Management CLI

The `robot_registry_cli.py` provides comprehensive robot management:

```bash
# List all registered robots
python -m warehouse.robot_registry_cli list

# Register individual robot
python -m warehouse.robot_registry_cli register --id "robot_001" --name "Alpha"

# Register multiple robots (auto-generates MuJoCo-safe IDs)
python -m warehouse.robot_registry_cli register-bulk --count 3 --name-prefix "DemoRobot"

# Delete specific robot
python -m warehouse.robot_registry_cli delete --id "robot_001"

# Delete all robots
python -m warehouse.robot_registry_cli delete-all

# Charge all robots to 100% battery
python -m warehouse.robot_registry_cli charge-all

# Clear positions for all robots (keeps battery levels)
python -m warehouse.robot_registry_cli clear-positions
```

##### Automatic Sample Fleet Creation

The `load_new_warehouse.py` script automatically creates a sample robot fleet if no robots exist:

```bash
# Load warehouse and auto-create 3 sample robots if none exist
python load_new_warehouse.py --warehouse sample_warehouse.csv
```

**Sample Output:**
```
[INFO] No robots found in database - creating sample fleet
[SUCCESS] Created sample robot fleet: 3 robots
[INFO]    🤖 Sample robots: ['SampleRobot DB3A5A77', 'SampleRobot 817B1373', 'SampleRobot 67C860B9']
[INFO]    🆔 Robot IDs: ['db3a5a77...', '817b1373...', '67c860b9...']
```

##### Robot ID Format

Robot IDs are generated as 32-character hexadecimal strings (no hyphens) for MuJoCo compatibility:
- **Format**: `uuid.uuid4().hex` (e.g., `db3a5a77c823400c90b3bfbca7b49ce5`)
- **MuJoCo-Safe**: No special characters that could break joint/body naming
- **Collision-Free**: Ensures unique identifiers across all robots

##### Integration with Simulations

The robot registry automatically:
- **Persists positions**: Robots remember their last positions between runs
- **Manages battery levels**: Battery state survives simulation restarts
- **Supports selective runs**: Choose specific robots or all registered robots
- **Handles warehouse resets**: Clears positions when loading new warehouses

**Example: Run with specific robots**
```bash
# Run with first 2 robots from registry
python demo_multi_robot_orders_simulation.py --robots 2 --placement persisted

# Run with all registered robots
python demo_multi_robot_orders_simulation.py --robots 0 --placement persisted
```

##### Troubleshooting Robot Registry

**"No robots found" when running simulation:**
```bash
# Check registered robots
python -m warehouse.robot_registry_cli list

# Create sample fleet if empty
python -m warehouse.robot_registry_cli register-bulk --count 3 --name-prefix "Robot"
```

**"MuJoCo joint names not found" errors:**
- Robot IDs are automatically MuJoCo-safe (hexadecimal format)
- If you registered robots manually, ensure IDs contain only alphanumeric characters
- Regenerate IDs: delete and re-register robots with the CLI

**Robot positions not persisting:**
```bash
# Check current positions
python -c "
from warehouse.impl.robot_registry_impl import RobotRegistryImpl
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap

wm = WarehouseMap()
sds = SimulationDataServiceImpl(wm)
registry = RobotRegistryImpl(sds)

for robot in registry.list_registered_robots():
    state = registry.get_robot_state(robot.robot_id)
    if state and state.position:
        print(f'{robot.robot_id}: {state.position}')
    else:
        print(f'{robot.robot_id}: no position')
"
```

**Database permission errors:**
```bash
# Ensure your database user has permissions on robot tables
psql -U your_username -d warehouse_sim
GRANT ALL ON robots TO your_username;
GRANT ALL ON robot_runtime_state TO your_username;
\q
```

## 🚀 Running the System

### Basic Multi-Robot Demo
```bash
# Windows PowerShell
.\venv311\Scripts\python.exe demo_multi_robot_orders_simulation.py

# Linux/macOS
python3 demo_multi_robot_orders_simulation.py
```

### Advanced Options
```bash
# Custom robot count and placement
.\venv311\Scripts\python.exe demo_multi_robot_orders_simulation.py --robots 3 --placement random

# Manual robot placement
.\venv311\Scripts\python.exe demo_multi_robot_orders_simulation.py --robots 2 --placement manual

# Persisted placement (uses robot registry - recommended)
.\venv311\Scripts\python.exe demo_multi_robot_orders_simulation.py --robots 0 --placement persisted
```

### Robot Registry Integration

The robot registry enables persistent robot state across simulation runs:

```bash
# 1. First, ensure robots are registered (if not, warehouse loading auto-creates sample fleet)
python -m warehouse.robot_registry_cli list

# 2. Run simulation with persisted robots (uses all registered robots)
.\venv311\Scripts\python.exe demo_multi_robot_orders_simulation.py --robots 0 --placement persisted

# 3. Run simulation with specific number of robots from registry
.\venv311\Scripts\python.exe demo_multi_robot_orders_simulation.py --robots 2 --placement persisted

# 4. Manage robots between runs
python -m warehouse.robot_registry_cli charge-all  # Charge all robots
python -m warehouse.robot_registry_cli clear-positions  # Reset all positions
```

### Other Demo Scripts
```bash
# Single robot demo
.\venv311\Scripts\python.exe demo_robot_task_simulation.py

# Configuration integration demo
.\venv311\Scripts\python.exe demo_configuration_integration_complete.py

# End-to-end multi-robot demo
.\venv311\Scripts\python.exe demo_end_to_end_multi_robot.py
```

## 🔧 Troubleshooting

### Database Connection Issues

**Error: "Database password not configured"**
```bash
# Solution: Set DATABASE_URL with your actual password
$env:DATABASE_URL="postgresql://postgres:YOUR_ACTUAL_PASSWORD@localhost:5432/warehouse_sim"

# Replace YOUR_ACTUAL_PASSWORD with your real PostgreSQL password
```

**Error: "Connection refused" or "Host not found"**
```bash
# Check if PostgreSQL is running
# Windows: Services > PostgreSQL > Start
# macOS/Linux: sudo systemctl start postgresql

# Verify PostgreSQL is listening on correct port
psql -U postgres -h localhost -p 5432 -d warehouse_sim
```

**Error: "Database warehouse_sim does not exist"**
```bash
# Create the database (run as PostgreSQL superuser)
psql -U postgres
CREATE DATABASE warehouse_sim;
GRANT ALL PRIVILEGES ON DATABASE warehouse_sim TO postgres;
\q

# Then run the schema setup
psql -U postgres -d warehouse_sim -f warehouse_schema_lane_based.sql
```

### MuJoCo License Issues
```bash
# Set license key
$env:MUJOCO_LICENSE_KEY="your-license-key"

# Verify installation
python -c "import mujoco; print('MuJoCo OK')"
```

### Import Errors
```bash
# Ensure virtual environment is activated
.\venv311\Scripts\Activate.ps1

# Reinstall dependencies
pip install -r requirements.txt --force-reinstall
```

### Robot Registry Issues

**"No robots found" or empty robot list:**
```bash
# Check if robots are registered
python -m warehouse.robot_registry_cli list

# Create sample fleet if needed
python -m warehouse.robot_registry_cli register-bulk --count 3 --name-prefix "Robot"

# Or load warehouse (auto-creates sample fleet if none exist)
python load_new_warehouse.py --warehouse sample_warehouse.csv
```

**Robot positions not persisting between runs:**
```bash
# Verify positions are being saved
python -m warehouse.robot_registry_cli list  # Should show robot names

# Check specific robot state
python -c "
from warehouse.impl.robot_registry_impl import RobotRegistryImpl
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap

wm = WarehouseMap()
sds = SimulationDataServiceImpl(wm)
registry = RobotRegistryImpl(sds)

for robot in registry.list_registered_robots():
    state = registry.get_robot_state(robot.robot_id)
    print(f'{robot.robot_id}: position={state.position if state else None}, battery={state.battery_level if state else None}')
"
```

**MuJoCo joint/body name errors:**
- Robot IDs are automatically generated as MuJoCo-safe (hexadecimal format)
- If manually registered, ensure IDs contain only alphanumeric characters
- Fix: Delete and re-register robots using the CLI

**Database permission errors with robot tables:**
```bash
# Grant permissions to your database user
psql -U your_username -d warehouse_sim
GRANT ALL ON robots TO your_username;
GRANT ALL ON robot_runtime_state TO your_username;
GRANT ALL ON warehouse_map TO your_username;  # If using load_new_warehouse.py
\q
```

**Simulation fails to start with robot registry:**
```bash
# 1. Verify database connection
python -c "
from utils.database_config import get_database_config
config = get_database_config()
print('✅ Database config OK')
"

# 2. Check warehouse data exists
python -c "
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap
wm = WarehouseMap()
sds = SimulationDataServiceImpl(wm)
boxes = sds.get_conflict_boxes()
print(f'✅ Navigation graph: {len(boxes)} boxes found')
"

# 3. Check robots exist
python -m warehouse.robot_registry_cli list
```

## 📚 Project Structure

```
autonomous-warehouse-v2/
├── config/                 # Configuration management
├── interfaces/             # Interface definitions (SOLID architecture)
├── robot/                  # Robot agent implementations
├── simulation/             # Physics and visualization
├── warehouse/              # Warehouse management and data models
│   ├── robot_registry_cli.py      # Robot registry command-line interface
│   └── impl/                      # Robot registry implementation
├── tests/                  # Unit and integration tests
├── utils/                  # Utility functions
├── *.py                    # Demo and utility scripts
├── *.csv                   # Warehouse layouts and sample data
├── *.json                  # Sample orders and configuration
├── *.sql                   # Database schemas and utilities
└── *.md                    # Documentation
```

---

**Project Status**: ✅ Complete - Ready for demonstration and evaluation  
**Last Updated**: September 2025  
**Version**: 2.0 - Lane-Based Navigation with Multi-Robot Coordination
