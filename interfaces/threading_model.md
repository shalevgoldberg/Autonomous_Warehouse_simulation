# Threading Model for Autonomous Warehouse System

## Thread Types and Frequencies

### 1. **Physics Thread (1 kHz)**
- **Purpose**: MuJoCo physics simulation
- **Frequency**: 1000 Hz (every 1ms)
- **Components**:
  - `mujoco.mj_step()` - Physics simulation step
  - `StateHolder.update_from_simulation()` - Read physics state

### 2. **Control Thread (10 Hz)**
- **Purpose**: Robot decision making and control
- **Frequency**: 10 Hz (every 100ms)
- **Components**:
  - `TaskHandler.*()` - All task management
  - `PathPlanner.plan_path*()` - Path planning
  - `MotionExecutor.execute_*()` - Motion commands
  - `MotionExecutor.update_control_loop()` - PID control (100Hz within this thread)

### 3. **JobsProcessor Thread (Continuous)**
- **Purpose**: Order processing
- **Frequency**: Continuous polling/event-driven
- **Components**:
  - `JobsProcessor.process_next_order()` - Process orders

### 4. **Visualization Thread (20-30 FPS)**
- **Purpose**: Display and rendering
- **Frequency**: 20-30 Hz
- **Components**:
  - `Visualization.visualize()` - Render display

## Thread Safety Contracts

### ✅ **Thread-Safe for Concurrent Access:**
- `StateHolder.get_*()` - Read physics state from any thread
- `JobsQueue.*()` - All methods (thread-safe queue)
- `SimulationDataService.*()` - All methods (database-backed)
- `CoordinateSystem.*()` - All methods (stateless utility)

### ⚠️ **Single Thread Only:**
- `StateHolder.update_from_simulation()` - **PHYSICS THREAD ONLY**
- `TaskHandler.*()` - **CONTROL THREAD ONLY**
- `PathPlanner.plan_path*()` - **CONTROL THREAD ONLY**
- `MotionExecutor.execute_*()` - **CONTROL THREAD ONLY**
- `Visualization.visualize()` - **VISUALIZATION THREAD ONLY**

### 🚨 **Emergency Access:**
- `MotionExecutor.emergency_stop()` - **ANY THREAD** (safety override)

## Data Flow Between Threads

```
Physics Thread (1kHz):
    MuJoCo.mj_step() ← data.ctrl[wheel_commands] ← MotionExecutor (ATOMIC!)
    MuJoCo → StateHolder.update_from_simulation()
                    ↓ (thread-safe writes)
Control Thread (10Hz):
    StateHolder.get_position() → TaskHandler → PathPlanner → MotionExecutor
    MotionExecutor.set_wheel_commands_atomic() → data.ctrl[wheel_commands] (ATOMIC!)
                    ↓ (thread-safe reads)
Visualization Thread (30Hz):
    StateHolder.get_position() → Visualization.visualize()

JobsProcessor Thread:
    External Orders → JobsProcessor → JobsQueue
                                        ↓ (thread-safe queue)
Control Thread:
    TaskHandler.request_new_task() → JobsQueue.dequeue_task()
```

## Implementation Guidelines

### 1. **Immutable Data Structures**
- `RobotPhysicsState` - Immutable snapshots
- `Task`, `Path`, `MapData` - Immutable data

### 2. **Thread-Safe Implementations Must Provide:**
- Internal locking mechanisms
- Atomic operations for critical sections
- Database transactions for SimulationDataService
- Thread-safe collections for JobsQueue

### 3. **Atomic Command Writes**
- `MotionExecutor.set_wheel_commands_atomic()` - Must prevent race conditions with physics thread
- Use locking, double-buffering, or atomic data structures
- Physics thread must never read partially-written commands

### 4. **Error Handling**
- Thread-safe error propagation
- Emergency stop mechanisms
- Graceful degradation under thread contention

## Phase 2 Threading Setup

```python
# Main application thread coordination
def main():
    # Physics thread
    physics_thread = Thread(target=run_physics_loop, args=[1000])  # 1kHz
    
    # Control thread  
    control_thread = Thread(target=run_control_loop, args=[10])    # 10Hz
    
    # Visualization thread
    viz_thread = Thread(target=run_visualization_loop, args=[30])  # 30Hz
    
    # JobsProcessor thread
    processor_thread = Thread(target=run_jobs_processor)           # Continuous
    
    # Start all threads
    physics_thread.start()
    control_thread.start()
    viz_thread.start()
    processor_thread.start()
```

This threading model ensures:
- **Physics stability** at high frequency
- **Control responsiveness** at appropriate frequency  
- **Visual smoothness** for human perception
- **Order processing** without blocking other operations
- **Thread safety** for all shared resources 