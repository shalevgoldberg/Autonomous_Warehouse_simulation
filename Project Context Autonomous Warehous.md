# Project Context: Autonomous Warehouse Logistics Simulation in MuJoCo

## Database Access
For access to the database run: `$env:WAREHOUSE_DB_PASSWORD="renaspolter";`

## Project Overview
For my B.A final project, I am building a simulation for autonomous warehouse logistics using the MuJoCo physics engine. The goal is to create a decentralized, scalable multi-robot system where robots perform item picking and delivery tasks in a dynamic warehouse environment.

## Key System Details
- **Robots**: Simulated as two-wheel differential drive agents, each with local state (pose, speed, battery, current task) and modules for path planning (A*), traffic rules (keep-right, intersection yielding), and motion control.
- **Task Management**: Tasks are generated externally, distributed via a jobs queue, and assigned through a decentralized auction (bidding) process. The orchestrator/arbiter only awards tasks; it does not manage robot traffic or routes.
- **Robot Coordination**: Each robot manages its own state, communicates with peers over a broadcast status bus (pub/sub), and avoids collisions using both shared beacons and local safety rules.
- **Warehouse World**: Includes a static map (walls, shelves, charging zones) managed by a Simulation Data Service (REST API). Robots lock and unlock shelves through this service.
- **Multithreading**: The simulation uses multiple threads: high-frequency physics (1 kHz), mid-frequency robot control and communication (10 Hz), and visualization (20–30 FPS).
- **KPI Logging**: Implemented for task events, collisions, and performance metrics for later analysis and visualization.
- **Constraints**: System must run efficiently on standard hardware (laptop/desktop), with a small fleet (~4 robots) and modest warehouse size.

## System Architecture

### Layer 1: Robot Internals
| Module | One-line Role |
|--------|---------------|
| StateHolder | Stores smoothed pose, speed, battery (updated every physics tick). |
| PathPlanner | A* on static map → returns full cell route. |
| TaskHandler | Owns current task & route buffer; triggers re-plan, manages shelf locks. |
| TrafficRuleControl | Applies keep-right, yield, following-distance each 100 ms. |
| MotionExecutor | Converts "move to next cell" into wheel torques (PID). |
| Communicator | Publishes/receives 10 Hz status beacons to/from Communication Bus. |
| OccupancyMap | In-memory dict of nearby robots, updated from beacons. |

### Layer 2: System Services
| Module | One-line Role |
|--------|---------------|
| RawJobRouter | Partitions external orders to the proper Job-Processor stream. |
| JobsProcessor | Converts raw orders to robot tasks, updates inventory, pushes to Jobs-Queue. |
| JobsQueue | Topic robots listen to; supports multiple partitions. |
| Orchestrator | Collects bids, sends TaskAward to winning robot. |
| SimulationDataService | REST API for static map, shelf inventory, shelf-lock, KPI logging. |
| KPI Log | Table where events (TaskStart, TaskDone, stalls, collisions) are stored. |

### Layer 3: Infrastructure
| Module | One-line Role |
|--------|---------------|
| CommunicationBus | Pub/Sub channel; ≤ 50 kB/s total fleet traffic. |
| PostgreSQL DB | Persistent store for shelves, locks, KPI events. |
| Visualization | Reads KPI data + map to show live dashboard. |

## Database Architecture

### Current State (Phase 2)
- **Primary Database**: PostgreSQL
- **Database Password**: `renaspolter`
- **Schema**: `items`, `shelves`, `shelf_inventory`, `robots`, `warehouse_map`, `shelf_locks`, `kpi_logs`
- **Performance**: Adequate for single robot - no caching needed
- **Architecture**: Clean, direct database queries with connection pooling

### Database Design Principles
- **No premature optimization**: Current Phase 2 doesn't need caching
- **Single robot**: No database contention or performance bottlenecks
- **Direct queries**: Fast enough for current load without complexity
- **Connection pooling**: Already implemented in SimulationDataService

### Future Database Evolution (Phase 3+)

#### Phase 3: Multi-Robot Database Architecture
**Complexity**: Medium (1-2 weeks implementation)

**New Components**:
- **Redis pub/sub** for real-time robot state coordination
- **Robot telemetry** with partitioned tables and retention policies
- **Bidding statistics** with aggregate-only persistence
- **Event-driven obstacles** instead of full grid updates

**Schema Additions**:
```sql
-- Robot telemetry with partitioning
CREATE TABLE robot_telemetry (
    robot_id VARCHAR(36),
    timestamp TIMESTAMP,
    pose JSONB,
    battery DECIMAL(5,2),
    speed DECIMAL(5,2)
) PARTITION BY RANGE (timestamp);

-- Bidding aggregates (raw data in cache, aggregates in DB)
CREATE TABLE bidding_aggregates (
    period_start TIMESTAMP,
    avg_latency_ms DECIMAL(8,2),
    total_bids INTEGER,
    successful_bids INTEGER
);

-- Dynamic obstacle events
CREATE TABLE obstacle_events (
    timestamp TIMESTAMP,
    x DECIMAL(10,3),
    y DECIMAL(10,3),
    obstacle_type VARCHAR(50),
    event_type VARCHAR(20)
);
```

**Key Insights**:
- **Phase 2**: No caching needed - current architecture is appropriate
- **Phase 3**: Redis integration for multi-robot coordination
- **Performance**: Only optimize when you have actual problems
- **Architecture**: Keep it simple until complexity is justified

## Development Phases

| Phase | What Works | New Pieces Turned On | "Done" Test |
|-------|------------|---------------------|-------------|
| **Layer 1 – Solo-Core** | 1 robot, 1 hard-coded task. | MotionExecutor, StateHolder, PathPlanner, mini map in code. | Shelf delivered, no crash. |
| **Layer 2 – Warehouse-Core** | Same robot handles many tasks via JobsQueue; shelf locks via SimulationDataService & DB. | JobsProcessor, JobsQueue, shelf lock API. | 10 tasks executed sequentially; lock prevents double-pick. |
| **Layer 3 – Fleet-Core** | Multiple robots, collision-free traffic, task auctioning. | Communicator, OccupancyMap, TrafficRuleControl, Orchestrator, CommunicationBus. | 4 robots run 5 min, zero contacts, tasks keep flowing. |
| **Layer 4 – Scalable Fleet** | Multiple job orchestrators, Multiple job queues, Multiple robot models | Deployment configs / Helm values. | Throughput scales ~linearly with robot count. |

## Current Status (Phase 2)

### ✅ Completed Components
- **Single robot** executing multiple tasks successfully
- **JobsProcessor** converting orders to robot tasks
- **JobsQueue** distributing tasks to robot
- **SimulationDataService** managing inventory and shelf locks
- **PathPlanner** with A* pathfinding working correctly
- **MotionExecutor** controlling robot movement
- **Database** performing adequately without caching
- **Architecture** clean and maintainable
- **Robot task execution** - robot successfully completes tasks and reaches targets
- **Console-based simulation** - system runs without visualization
- **Core task flow** - orders → tasks → robot execution → completion

### 🔄 In Progress
- **SimulationDataService integration** with task flow - shelf locking and inventory management during task execution

### ❌ Remaining Tasks (Phase 2)
1. **Database testing** - comprehensive testing of database operations and integration
2. **Visualization** - restore or implement new visualization system
3. **Git integration** - connect project to version control system

### �� Phase 2 Completion Checklist
- [ ] Complete SimulationDataService integration with task flow
- [ ] Implement comprehensive database testing
- [ ] Add visualization system (MuJoCo or alternative)
- [ ] Set up Git repository and version control
- [ ] Final testing and validation of complete Phase 2 system

## Key Architectural Decisions

### Database Strategy
- **Phase 2**: Direct PostgreSQL queries - no caching needed
- **Phase 3**: Redis + PostgreSQL hybrid for multi-robot coordination
- **Principle**: Don't optimize prematurely - current performance is adequate

### Caching Philosophy
- **No caching in Phase 2**: Single robot doesn't create contention
- **Logical patterns**: Inventory changes with every task, paths are unique
- **Performance**: System runs smoothly without optimization
- **Complexity**: Avoid adding complexity without meaningful benefits

### Future Considerations
- **Redis integration** only when moving to multi-robot (Phase 3)
- **Table partitioning** for log retention policies
- **Event-driven architecture** for dynamic obstacles
- **Aggregate-only persistence** for high-frequency data

## Technical Constraints
- **Hardware**: Standard laptop/desktop performance
- **Robot Fleet**: Small scale (~4 robots)
- **Warehouse Size**: Modest dimensions
- **Real-time Requirements**: 10Hz control loops
- **Database**: PostgreSQL with connection pooling
- **Database Password**: `renaspolter`

## Development Guidelines
- **SOLID Principles**: Maintain clean architecture
- **Interface-driven design**: Loose coupling between components
- **Thread safety**: All database operations are thread-safe
- **Error handling**: Comprehensive exception handling
- **Testing**: Unit and integration tests for all components
- **Documentation**: Clear interfaces and implementation details

## Immediate Next Steps
1. **Complete SimulationDataService integration** - ensure shelf locking and inventory updates work during task execution
2. **Database testing** - verify all database operations work correctly with real data
3. **Visualization** - implement or restore visualization for better debugging and demonstration
4. **Git setup** - establish version control for the project

---

**Note**: This project follows the principle of "premature optimization is the root of all evil." The current Phase 2 architecture is clean, simple, and performs adequately. Database optimizations should only be implemented when moving to Phase 3 multi-robot coordination where they provide actual value.

**Database Access**: Always use `$env:WAREHOUSE_DB_PASSWORD="renaspolter";` when working with the database.