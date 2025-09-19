## Project Context (Current) – Autonomous Warehouse (Lane-Based, Passive MuJoCo Viewer)

### Executive Summary
- **Goal**: Simulate autonomous warehouse logistics with clean, interface-driven architecture.
- **Physics**: Dual-mode physics system supporting both kinematic and MuJoCo authoritative modes at 1 kHz.
- **Visualization**: MuJoCo used as a passive viewer with physics engine integration; we write `qpos` directly.
- **Navigation**: Lane-based planning and following; **CONFLICT BOX SYSTEM FULLY IMPLEMENTED**.
- **DB**: PostgreSQL via `SimulationDataService`; graph persistence supported.
- **Threads**: Physics 1 kHz, Motion 100 Hz, Control 10 Hz, Visualization ~30 FPS.
- **Bay Occupancy**: Idle/charging bays modeled with exclusive occupancy via database-backed bay locks.

### What Works Now
- **Single-robot task loop**: external tasks → queue → bidding/assignment → execution → completion → automatic IDLE_PARK.
- **Passive MuJoCo visualization** rendering the warehouse and robot pose live with appearance service integration.
- **Path planning on a navigation graph** (from CSV) with lane semantics.
- **Motion executor with differential-drive kinematics**, atomic wheel commands.
- **Dual physics modes**: Kinematic guard mode (default) and MuJoCo authoritative mode with planar joints and velocity actuators.
- **Physics thread manager (1 kHz)** updating `StateHolder` for consumers with SharedMuJoCoEngine support.
- **Database layer scaffolding** with connection pooling and graph persistence.
- **Complete task lifecycle**: PICK_AND_DELIVER tasks automatically chain to IDLE_PARK for idle zone navigation.
- **Logical inventory system**: Database-backed item transfer without physical shelf attachment, including reservation-based allocation to prevent double-booking and atomic inventory operations.
- ** CONFLICT BOX QUEUE SYSTEM**: Fully implemented with database-backed queue management, thread-safe operations, and comprehensive testing.
- **Multi-robot coordination**: Controller and bidding system distribute tasks across multiple robots with battery level monitoring.
- **Idle/Charging bays as resources**: Robots park at the nearest available idle bay, with intelligent waiting strategies including WANDER mode when bays are occupied.
- **Battery management system**: Activity-based consumption, charging station allocation, emergency stop, and configurable thresholds.
- **Enhanced configuration management**: Cleaned configuration with unused parameters commented out for clarity.
- **Inventory management system**: Reservation-based allocation with atomic database operations, preventing double-booking and ensuring consistency.

### Key Constraints and Demo Relaxations

---

## System Architecture and Module Interactions

### High-Level Architecture
```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           External Interface Layer                          │
├─────────────────────────────────────────────────────────────────────────────┤
│  Order Source  │  Jobs Queue  │  Bidding System  │  KPI Monitoring      │
└─────────────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Robot Coordination Layer                          │
├─────────────────────────────────────────────────────────────────────────────┤
│  Robot Agent  │  Task Handler │  Path Planner   │  Lane Follower        │
│               │               │                 │                        │
│  ┌─────────┐  │  ┌─────────┐  │  ┌─────────┐   │  ┌─────────────────┐   │
│  │Physics  │  │  │Task     │  │  │Graph-   │   │  │ Conflict Box   │   │
│  │Thread   │  │  │Lifecycle│  │  │Based    │   │  │Queue System     │   │
│  │Manager  │  │  │         │  │  │Planning │   │  │  │   │
│  └─────────┘  │  └─────────┘  │  └─────────┘   │  └─────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Motion Control Layer                              │
├─────────────────────────────────────────────────────────────────────────────┤
│  Motion Executor  │  State Holder  │  Coordinate System  │  Bid Calculator  │
│                   │                 │                     │                 │
│  ┌─────────────┐  │  ┌──────────┐   │  ┌─────────────────┐│  ┌──────────┐   │
│  │Differential │  │  │Thread-   │   │  │Grid/World      ││  │Parallel  │   │
│  │Drive        │  │  │Safe      │   │  │Coordinate      ││  │Bid       │   │
│  │Control      │  │  │State     │   │  │Conversion      ││  │Processing│   │
│  └─────────────┘  │  └──────────┘   │  └─────────────────┘│  └──────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Physics & Visualization Layer                     │
├─────────────────────────────────────────────────────────────────────────────┤
│  Simple Physics  │  MuJoCo Viewer  │  Warehouse Map    │  Graph Generator  │
│                  │                  │                   │                   │
│  ┌─────────────┐ │  ┌─────────────┐ │  ┌─────────────┐  │  ┌─────────────┐  │
│  │Kinematic    │ │  │Passive      │ │  │CSV-Based    │  │  │Lane/Conflict│  │
│  │Integration  │ │  │Rendering    │ │  │Layout       │  │  │Box Graph    │  │
│  │             │ │  │             │ │  │             │  │  │             │  │
│  └─────────────┘ │  └─────────────┘ │  └─────────────┘  │  └─────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Data Persistence Layer                            │
├─────────────────────────────────────────────────────────────────────────────┤
│  Simulation Data Service  │  Graph Persistence  │  Inventory Management   │
│                           │                     │                         │
│  ┌─────────────────────┐  │  ┌─────────────────┐│  ┌─────────────────┐   │
│  │PostgreSQL +         │  │  │Navigation       ││  │Shelf Locking    │   │
│  │Connection Pooling   │  │  │Graph Storage    ││  │& Inventory     │   │
│  │Conflict Box DB   │  │  │                 ││  │                 │   │
│  └─────────────────────┘  │  └─────────────────┘│  └─────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Module Interaction Flow
1. **Task Creation**: External orders → `JobsProcessor` → `JobsQueue`
2. **Inventory Reservation**: `JobsProcessor` → `SimulationDataService.reserve_inventory()` → Atomic reservation
3. **Task Assignment**: `BiddingSystem` → `RobotAgent.bid_calculator` → Task award
4. **Task Execution**: `TaskHandler` → `PathPlanner` → `LaneFollower` → `MotionExecutor`
5. **Inventory Consumption**: `TaskHandler` → `SimulationDataService.consume_inventory()` → Atomic consumption on success
6. **Inventory Release**: `TaskHandler` → `SimulationDataService.release_inventory()` → Atomic release on failure
7. **Motion Control**: `MotionExecutor` → `SimpleMuJoCoPhysics` → `StateHolder`
8. **State Synchronization**: `PhysicsThreadManager` (1kHz) → `StateHolder` → All consumers
9. **Visualization**: `StateHolder` → `MujocoVisualization` → MuJoCo passive viewer
10. **Database**: All components → `SimulationDataService` → PostgreSQL
11. ** Conflict Box Management**: `LaneFollower` → `ConflictBoxIntegrationService` → Queue/Lock managers → PostgreSQL
12. **Bay Lock Management**: `TaskHandler` → `SimulationDataService` bay lock APIs → PostgreSQL
13. **Battery Management**: `StateHolder` → `BatteryManager` → Activity-based consumption calculations
14. **Charging Station Allocation**: `TaskHandler` → `ChargingStationManager` → Proximity-based station selection
15. **Idle Bay Management**: `TaskHandler` → Bay lock acquisition → WANDER mode (when bays occupied) → Resume IDLE_PARK when bay available

### Threads and Rates
- **Physics** (1 kHz): `robot.impl.physics_integration.PhysicsThreadManager` calls `SimpleMuJoCoPhysics.step_physics()` and `StateHolder.update_from_simulation()`.
- **Motion** (100 Hz): `MotionExecutorImpl.update_control_loop()` computes wheel velocities.
- **Control** (10 Hz): `TaskHandlerImpl.update_task_execution()` manages tasks and lane following.
- **Visualization** (~30 FPS): `VisualizationThread` calls `MujocoVisualization.visualize()`.

---

## Design Principles

### SOLID Principles Implementation
1. **Single Responsibility**: Each component has one clear purpose
   - `MotionExecutor`: Only handles motion control and wheel commands
   - `PathPlanner`: Only handles route planning and graph navigation
   - `LaneFollower`: Only handles lane following and conflict box coordination
   - `StateHolder`: Only manages robot state and thread-safe access
   -  'Conflict Box Services': Separate managers for queue, locks, and integration

2. **Open/Closed**: Extensible through interfaces, closed for modification
   - All components accessed via interfaces (`IMotionExecutor`, `IPathPlanner`, etc.)
   - New implementations can be added without modifying existing code
   - Configuration-driven behavior changes
   - ** New architectural interfaces**: Dependency Injection, Configuration Management, Circuit Breaker
     - **Configuration Management**: `IBusinessConfigurationProvider` provides domain-specific config objects (RobotConfig, BidConfig) with metadata for business logic; `IConfigurationManager` provides generic key-value access for technical infrastructure

3. **Liskov Substitution**: All implementations are interchangeable
   - Mock implementations for testing
   - Production implementations for deployment
   - Interface contracts ensure compatibility

4. **Interface Segregation**: Focused, specific interfaces
   - `IMotionExecutor`: Motion-specific methods only
   - `IPathPlanner`: Path planning methods only
   - `ILaneFollower`: Lane following methods only
   - Conflict Box Interfaces: `IConflictBoxQueueManager`, `IConflictBoxLockManager`, `IConflictBoxIntegrationService`

5. **Dependency Inversion**: Depend on abstractions, not concretions
   - Robot agent depends on interfaces, not concrete implementations
   - Dependency injection through constructor parameters
   - Easy to swap implementations for testing or different scenarios
   - **✅ Dependency Injection Container**: `DependencyInjectionContainer` for service management

### Quality Standards
- **Comprehensive Testing**: Unit tests for all components, integration tests for workflows
- **Modularity**: Loose coupling, high cohesion, clear boundaries
- **Scalability**: Design for multi-robot coordination and future expansion
- **Thread Safety**: All components thread-safe with proper synchronization
- **Error Handling**: Graceful degradation, comprehensive logging, exception safety
- **Performance Monitoring**: Built-in metrics, health checks, and circuit breaker patterns

### Architecture Patterns
- **Interface-Driven Design**: All major components defined by interfaces
- **Dependency Injection**: Components receive dependencies through constructors
- **Factory Pattern**: Component creation centralized in factory methods
- **Observer Pattern**: State changes propagate through StateHolder
- **Command Pattern**: Motion commands as atomic operations
- **Strategy Pattern**: Different path planning and motion strategies
- **Circuit Breaker Pattern**: Automatic error recovery and system resilience
- **Configuration Management**: Centralized, validated configuration system

---

## Physics and Visualization

### Physics (Dual-mode system)
- **Kinematic Guard Mode** (default): `simulation/mujoco_env.py` → `SimpleMuJoCoPhysics` (kinematic, differential drive).
- **MuJoCo Authoritative Mode**: `simulation/shared_mujoco_engine.py` → `SharedMuJoCoEngine` with planar joints and velocity actuators.
- Both modes: Integrate wheel commands, check walkability via `WarehouseMap.is_walkable`, update pose at 1 kHz.
- Kinematic mode: No forces/contacts/gravity; deterministic for simplicity.
- Authoritative mode: MuJoCo physics simulation with joints, actuators, and configurable damping/gains.

### Visualization (MuJoCo passive viewer)
- Modules: `simulation/mujoco_visualization.py`, `simulation/multi_robot_mujoco_visualization.py`, `simulation/mujoco_model_factory.py`, `simulation/visualization_thread.py`.
- MJCF is generated from `WarehouseMap`; robot appearance controlled by `AppearanceService` (visual feedback for carrying state).
- Viewer is created and updated on the visualization thread; writes joint `qpos` from `StateHolder`, calls `mj_forward`, and `sync()`.
- Coordinate frame: MJCF scene centered at origin; physics positions shifted by half extents for proper alignment.

Implications:
- Physics mode configurable via `physics.mode` setting ("kinematic_guard" or "mujoco_authoritative").
- Visual feedback integrated through appearance service (robots turn green when carrying items).
- Collisions enforced by physics layer; special cells (drop-off, charging, idle) are visual-only markers.

---

## Navigation Pipeline (Lane-Based)

1. Path Planning: `robot.impl.path_planner_graph_impl.PathPlannerGraphImpl`
   - Uses `ISimulationDataService` to load `NavigationGraph` generated from CSV by `warehouse.impl.graph_generator_impl`.
2. Lane Following: `robot.impl.lane_follower_impl.LaneFollowerImpl`
   - Follows lane center-lines within tolerance; integrates with MotionExecutor.
   - **✅ Conflict Box Integration**: Automatically enters/exits conflict boxes during navigation.
3. Motion Control: `robot.impl.motion_executor_impl.MotionExecutorImpl`
   - Converts segment/target to wheel velocities; writes atomically to physics.
4. Physics Integration: `simulation/mujoco_env.SimpleMuJoCoPhysics`
   - Integrates pose; updates `IStateHolder`.
5. Visualization: `simulation.MujocoVisualization`
   - Reads `IStateHolder`; writes free-joint pose to MuJoCo viewer.

Parking Management:
- Idle/charging bays are included as graph nodes (non-through). During IDLE_PARK, TaskHandler acquires a bay lock via SimulationDataService, heartbeats while parked, and releases on completion.
- **WANDER Mode**: When idle bays are occupied, robots enter WANDER operational status, randomly selecting walkable lane positions (excluding conflict boxes, drop-off, idle, and charging zones) for navigation. While wandering, robots periodically attempt to acquire idle bay locks and can accept new tasks or charging requests. Wandering prevents pathway blockage during idle bay waiting.

Supporting Services:
- Jobs flow: `warehouse.impl.jobs_processor_impl` → `warehouse.impl.jobs_queue_impl`.
- DB-backed navigation data and inventory: `simulation/simulation_data_service_impl.py`.
- Conflict Box Services: `warehouse/impl/conflict_box_*_impl.py` for queue and lock management.

---

## Database Integration
- Service: `SimulationDataServiceImpl` (PostgreSQL, psycopg2, connection pooling).
- Responsibilities: Map data, shelf locking, inventory queries, navigation graph persistence.
- Env var for password: set before running tests or demos.
  - PowerShell: `$env:WAREHOUSE_DB_PASSWORD="renaspolter"`

Graph Persistence:
- `warehouse.impl.graph_persistence_impl.GraphPersistenceImpl` persists lanes, nodes, edges, and conflict boxes derived via `GraphGeneratorImpl`.

**Conflict Box Database Schema**:
- `conflict_box_queue`: Robot queue positions and priorities
- `conflict_box_locks`: Active locks with timeouts and heartbeats
- `conflict_boxes`: Physical conflict box locations
- `conflict_box_queue_stats`: Performance metrics and statistics
- PostgreSQL functions for atomic queue operations and position management

**Bay Locks Schema**:
- `bay_locks`: Exclusive occupancy for idle/charging bays (`bay_id`, `locked_by_robot`, timestamps, timeout)
- Helper functions for cleanup/acquire/release/heartbeat; service also exposes idle/charging bay listings
- Integrated with WANDER mode: robots periodically attempt bay lock acquisition while wandering, transitioning back to IDLE_PARK navigation when successful

**Inventory Management Schema**:
- `shelf_inventory`: Item storage with quantity tracking (`shelf_id`, `item_id`, `quantity`, `pending`)
- `pending` column tracks reserved but not yet consumed inventory to prevent double-booking
- Atomic operations for reserve/release/consume with database-level consistency
- Fallback allocation when primary shelves are insufficient

**Battery Management Integration**:
- Battery consumption calculations integrated with `StateHolder` physics updates
- Charging station allocation using existing bay lock infrastructure
- Emergency stop functionality with configurable thresholds
- Battery level monitoring in all demo logging

---

## Configuration
- Source: `config/configuration_sources.py` (defaults), `config/configuration_provider.py` (typed access + validation).
- Demo overrides: lane tolerance set to 3.0 m to avoid frequent deviation aborts (too soft for production).
- Validator: `config/configuration_validator.py` may warn/fail when tolerance > 2.0 m; adjust when returning to strict operation.
- Enhanced Configuration: `warehouse/impl/configuration_provider_impl.py` with multiple sources
- Configuration Management: Cleaned configuration with unused parameters commented out for clarity and maintainability

Key params (illustrative):
- Physics: mode ("kinematic_guard" or "mujoco_authoritative"), gravity, time step, velocity damping.
- Robot: cell size, speed caps, wheel base/radius, position tolerance.
- Navigation: lane tolerance, velocity smoothing, blocked cell handling.
- Threading: control/motion/physics frequencies.
- conflict Box: Lock timeouts, heartbeat intervals, queue priorities.
- Battery: consumption rates, charging rates, safety thresholds, emergency stop levels.
- Charging: station allocation, automatic triggering, safety margins.
- Appearance: carrying color, normal color for visual feedback.

---

## Running
- Demo: `demo_robot_task_simulation.py`
  - Starts robot agent (spawns physics + control + motion), initializes visualization thread, positions robot, enqueues demo tasks.
  - Demonstrates complete task lifecycle: SHELF → DROP-OFF → IDLE ZONE (3 tasks for faster testing).
  - Conflict Box Integration: Shows real-time conflict box entry/exit during navigation.
  - Battery Management: Real-time battery level monitoring and emergency stop demonstration.
  - Command (PowerShell):
    - Set DB password: `$env:WAREHOUSE_DB_PASSWORD="renaspolter"`
    - Run: `./venv311/Scripts/python.exe demo_robot_task_simulation.py`

Notes:
- Close older MuJoCo windows before re-running to avoid stale viewers.
- Viewer is passive; HUD time advances, but no internal stepping.

---

## Known Issues and Notes
- Demo lane tolerance (3.0 m) is intentionally lax; tighten to 0.1–0.3 m for production behavior.
- Viewer must be initialized and updated on the same thread with proper locking (done).
- Without MuJoCo dynamics, the robot can visually pass through obstacles; actual collision logic is map-based.
 - Charging navigation currently uses basic target selection; bay lock patterns are available for charging workflows as needed.

---

## Future Roadmap

### Core System Enhancements
- **Sensors**: Add LIDAR via MuJoCo ray casting against MJCF geometry (still passive), feed into `IStateHolder` and safety systems
- **Tolerance Management**: Tighten lane deviation tolerances, ensure validator alignment, expand integration tests around deviation and lane switching
- **Graph Persistence**: Persist navigation graph from CSV on startup via `GraphPersistenceImpl` when DB is available
- **Battery Management**: enhanced charging stations allocation, availabilty for tasks before fully charged
- **collision avoidance** in the physics authorative mode, colission avoidance using sensors.
- **bidding system** advanced bidding system that calculates various factors
- **warehouse layout designing api** - semiautomatic lane and graph generator using user input via designated ui
- **Kpi display** - statistics, logs, and performance tracking
- **Configuration Management**: ✅ Cleaned configuration with unused parameters commented out

### Multi-Robot Coordination
- **Fleet Management**: Extend system to handle multiple robots with coordinated task execution
- **Traffic Coordination**:implement enhanced traffic rules, collision anoidance, and more
- **Resource Allocation**: Develop intelligent task distribution and robot assignment algorithms
- **Collision Avoidance**: Multi-robot path planning with dynamic obstacle avoidance

### Advanced Task Management
- **Multi-Task Coordination**: Support for robots handling multiple concurrent tasks
- **Task Dependencies**: Implement task chains and dependency management
- **Priority Queuing**: Dynamic task prioritization based on urgency and robot capabilities
- **Task Optimization**: Route optimization for multiple tasks and robots
- **WANDER Mode**: ✅ Implemented intelligent idle bay waiting with random navigation to prevent pathway blockage

### Enhanced Bidding System
- **Real-Time Bidding**: Implement live bidding with dynamic task reallocation
- **Bid Optimization**: Advanced bid calculation algorithms considering multiple factors
- **Market-Based Coordination**: Auction-based task distribution with price discovery
- **Bid Analytics**: Comprehensive bidding statistics and performance metrics

### User Interface and Tools
- **Graph Generator UI**: Interactive web-based interface for creating and editing warehouse navigation graphs
- **Real-Time Monitoring**: Dashboard for monitoring robot status, task progress, and system health
- **Configuration Management**: User-friendly interface for system parameter tuning
- **Visualization Tools**: Enhanced 3D visualization with multiple robot support

### Operational Features
- **Charging Management**: ✅ Intelligent charging station assignment and battery management implemented
- **Idle Bay Management**: ✅ WANDER mode implemented for intelligent waiting when idle bays are occupied
- **Bidding System Integration**: ✅ Enhanced availability checking during WANDER mode for new task acceptance
- **KPI Logging**: Comprehensive performance metrics, task completion rates, and efficiency analysis
- **Predictive Maintenance**: Robot health monitoring and maintenance scheduling
- **Error Recovery**: Advanced error handling and automatic recovery mechanisms

### Optional Advanced Features
- **Restocking Station**: Automated restocking tasks and inventory management
- **Dynamic Obstacles**: Real-time obstacle detection and dynamic path replanning

### MuJoCo Physics Migration
- **Benefits**: True contacts, (optional) joints (e.g., carrying shelves), richer sensors, realistic physics
- **Costs**: Tuning masses/inertia/friction/damping/contacts; retuning control; replacing map-collision with MuJoCo collision filters
- **Incremental Path**: Keep passive viewer; introduce MuJoCo collision-only scene for raycasts; then selectively enable dynamics for the robot while maintaining interface contracts

---

## Key Modules (Quick Reference)
- Physics: `simulation/mujoco_env.py`, `simulation/shared_mujoco_engine.py`, `robot/impl/physics_integration.py`
- Visualization: `simulation/mujoco_visualization.py`, `simulation/multi_robot_mujoco_visualization.py`, `simulation/mujoco_model_factory.py`, `simulation/visualization_thread.py`
- Appearance Service: `interfaces/appearance_service_interface.py`, `simulation/shared_mujoco_engine.py` (integrated)
- Robot Agent: `robot/robot_agent_lane_based.py`
- Motion: `robot/impl/motion_executor_impl.py`
- Lane Following: `robot/impl/lane_follower_impl.py`
- Path Planner: `robot/impl/path_planner_graph_impl.py`
- Task Handler: `robot/impl/task_handler_impl.py` (includes IDLE_PARK task chaining, WANDER mode for intelligent idle bay waiting, and inventory consumption/release operations)
- Map: `warehouse/map.py` (coordinate-based shelf IDs: `shelf_x_y`)
- Graph Generation/Persistence: `warehouse/impl/graph_generator_impl.py`, `warehouse/impl/graph_persistence_impl.py`
- Jobs: `warehouse/impl/jobs_processor_impl.py` (includes reservation-based inventory allocation), `warehouse/impl/jobs_queue_impl.py`
- Multi-robot Controller: `warehouse/impl/robot_controller_impl.py`
- Bidding: `warehouse/impl/transparent_bidding_system_impl.py`
- DB Service: `simulation/simulation_data_service_impl.py` (includes inventory management with reservation-based allocation)
- Config: `config/configuration_sources.py`, `config/configuration_provider.py`, `config/configuration_validator.py`
- Conflict Box System: `warehouse/impl/conflict_box_*_impl.py`, `interfaces/conflict_box_*_interface.py`
- Battery Management: `warehouse/impl/battery_manager_impl.py`, `interfaces/battery_manager_interface.py`
- Charging Station Management: `warehouse/impl/charging_station_manager_impl.py`, `interfaces/charging_station_manager_interface.py`
- Architectural Services: `warehouse/impl/dependency_injection_container.py`, `warehouse/impl/configuration_provider_impl.py`, `warehouse/impl/circuit_breaker_impl.py`

---

## Technical Notes and Setup

### Database Setup
- **Database**: PostgreSQL (warehouse_sim)
- **Password**: `renaspolter` (set as environment variable)
- **Connection**: localhost:5432
- **Schema**: Run `warehouse_schema_lane_based.sql` for lane-based navigation tables

#### Environment Variable Setup
**PowerShell:**
```powershell
$env:WAREHOUSE_DB_PASSWORD="renaspolter"
```

**Command Prompt:**
```cmd
set WAREHOUSE_DB_PASSWORD=renaspolter
```

**Linux/macOS:**
```bash
export WAREHOUSE_DB_PASSWORD="renaspolter"
```

### Running the Simulation

#### 1. Basic Demo with Visualization
```powershell
# Set database password
$env:WAREHOUSE_DB_PASSWORD="renaspolter"

# Run the main demo
.\venv311\Scripts\python.exe demo_robot_task_simulation.py
```

**Expected Output:**
- MuJoCo viewer window opens showing warehouse and yellow robot
- Robot moves between positions following lane-based navigation
- Conflict Box: Real-time logging of conflict box operations

#### 2. Alternative Demo Scripts
```powershell
# Configuration integration demo
.\venv311\Scripts\python.exe demo_configuration_integration_complete.py

# Simple movement test
.\venv311\Scripts\python.exe test_movement_simple.py

# Multi-robot coordination demo with battery monitoring (example: 4 robots)
.\venv311\Scripts\python.exe demo_multi_robot_simulation.py --robots 4

# End-to-end multi-robot demo with battery logging
.\venv311\Scripts\python.exe demo_end_to_end_multi_robot.py

# Multi-robot demo with order processing and inventory management
.\venv311\Scripts\python.exe demo_multi_robot_orders_simulation.py
```

#### 3. Running Tests
```powershell
# Unit tests
.\venv311\Scripts\pytest.exe tests\unit

# Integration tests
.\venv311\Scripts\pytest.exe tests\integration

# Specific test file
.\venv311\Scripts\pytest.exe tests\integration\test_robot_end_to_end_parametrized.py

# Battery management tests
.\venv311\Scripts\pytest.exe tests/unit/test_battery_manager.py
.\venv311\Scripts\pytest.exe tests/integration/test_battery_management_integration.py

# Conflict Box Tests
.\venv311\Scripts\python.exe test_phase3a_integration_service.py
.\venv311\Scripts\python.exe test_phase3b_advanced_threading.py
.\venv311\Scripts\python.exe test_phase3c_advanced_features.py
```

### Configuration Management

#### Key Configuration Files
- `config/configuration_sources.py`: Default values and overrides (unused parameters commented out)
- `config/configuration_provider.py`: Type-safe configuration access
- `config/configuration_validator.py`: Validation rules
- 'Enhanced Configuration': `warehouse/impl/configuration_provider_impl.py` with multiple sources

#### Important Parameters
```python
# Physics mode (default: "kinematic_guard")
"physics.mode": "kinematic_guard"           # or "mujoco_authoritative"
"physics.gravity": 0.0                      # Gravity for physics simulation
"physics.time_step": 0.001                  # Physics time step (1 kHz)
"physics.velocity_damping": 0.0             # Joint damping

# Lane tolerance (demo: 3.0m, production: 0.1-0.3m)
"robot.lane_tolerance": 3.0
"navigation.lane_tolerance": 3.0

# Threading frequencies
"robot.physics_frequency": 1000.0  # 1 kHz
"robot.control_frequency": 10.0    # 10 Hz
"robot.motion_frequency": 100.0    # 100 Hz

# Robot parameters
"robot.max_speed": 1.0             # m/s
"robot.cell_size": 0.5             # meters

# Battery parameters
"battery.idle_consumption_rate": 50000.0   # Extreme drain for demo visibility
"battery.low_battery_threshold": 0.8       # 80% triggers bidding exclusion
"battery.emergency_stop_threshold": 0.4    # 40% triggers emergency stop

# Charging parameters
"charging.trigger_threshold": 0.85         # 85% triggers automatic charging
"charging.battery_check_interval": 1.0     # Check every 1 second

# Appearance parameters
"appearance.carrying_color": [0.0, 1.0, 0.0, 1.0]  # Green when carrying
"appearance.normal_color": [0.8, 0.2, 0.2, 1.0]    # Red when not carrying
```

### Development Environment

#### Virtual Environment
```powershell
# Activate virtual environment
.\venv311\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt

# Check MuJoCo installation
python -c "import mujoco; print('MuJoCo version:', mujoco.__version__)"
```

#### Required Dependencies
- **Python**: 3.11+
- **MuJoCo**: 2.3.0+
- **PostgreSQL**: 12+ with psycopg2
- **Other**: numpy, threading, time, logging

### Debugging and Monitoring

#### Console Logging
- **Task Execution**: `[TaskHandler]` prefix
- **Motion Control**: `[MotionExecutor]` prefix
- **Physics**: `[Physics]` prefix
- **Lane Following**: `[LaneFollower]` prefix
- **Conflict Box**: `[INFO] [LaneFollower.robot_X] Robot entered/exited conflict box: X`
 - **Bay Locks**: `[BayLock][robot_X] Trying/Acquired/Heartbeat/Released idle bay <bay_id>`



### Performance Tuning

#### Thread Frequencies
- **Physics**: 1000 Hz (1ms) - Critical for smooth motion
- **Motion**: 100 Hz (10ms) - Balance between responsiveness and CPU usage
- **Control**: 10 Hz (100ms) - Task management and coordination
- **Visualization**: 30 FPS (33ms) - Smooth visual updates

#### Memory Management
- **Connection Pooling**: Default 10 connections, adjust based on robot count
- **State Caching**: StateHolder caches recent physics state
- **Graph Persistence**: Navigation graph loaded once, cached in memory
- **Conflict Box**: Connection pooling for database operations, performance monitoring

### Integration Points

#### External Systems
- **Order Source**: JSON files, APIs, databases
- **KPI Monitoring**: PostgreSQL tables for performance metrics
- **Inventory Management**: Shelf locking and item tracking
- **Graph Generation**: CSV-based warehouse layout processing
- ** Conflict Box Management**: Database-backed queue and lock system

#### API Endpoints (Future)
- **REST API**: Task submission and status monitoring
- **WebSocket**: Real-time robot state updates
- **GraphQL**: Flexible data querying for complex operations

---

## How to Use This Context in Future Chats
- Physics mode is configurable via `physics.mode` ("kinematic_guard" or "mujoco_authoritative").
- Kinematic mode uses `SimpleMuJoCoPhysics`; authoritative mode uses `SharedMuJoCoEngine`.
- Visual feedback integrated through `AppearanceService` (robots turn green when carrying).
- Physics/state truth comes from physics engine and `IStateHolder`.
- For DB work, go through `SimulationDataService` (and set `WAREHOUSE_DB_PASSWORD`).
- For navigation issues, check `LaneFollowerImpl` and `PathPlannerGraphImpl` first, and validate configuration.
- **✅ Architectural Services**: Dependency injection, configuration management, and circuit breaker patterns available.
- **✅ Battery Management**: Activity-based consumption with emergency stop and configurable thresholds.
- **✅ Charging Stations**: Proximity-based allocation integrated with existing bay lock system.
- **✅ Configuration Management**: Cleaned configuration with unused parameters commented out.
- **✅ Logical Attachment**: Database-backed item transfer without physical shelf attachment.
- **✅ WANDER Mode**: Intelligent idle bay waiting with random navigation to prevent pathway blockage.
- **✅ Inventory Management**: Reservation-based allocation system with atomic database operations, preventing double-booking and ensuring consistency across concurrent operations.
- Use the technical notes above for setup, running, and troubleshooting.




