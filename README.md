# Autonomous Warehouse Logistics Simulation - Phase 1

## Overview
This is Phase 1 of a decentralized, multi-robot warehouse simulator using MuJoCo. Phase 1 implements a single robot with hard-coded tasks, focusing on core motion and path planning capabilities.

## Phase 1 Features
- Single robot with differential drive
- Hard-coded task execution
- A* path planning on static map
- Motion control with PID
- State management (pose, speed, battery)
- Mini warehouse map in code
- MuJoCo physics simulation

## Installation
```bash
pip install -r requirements.txt
```

## Running Phase 1
```bash
python main.py
```

## Project Structure
```
├── main.py                 # Main simulation entry point
├── robot/
│   ├── __init__.py
│   ├── state_holder.py     # Robot state management
│   ├── path_planner.py     # A* path planning
│   ├── motion_executor.py  # PID motion control
│   └── robot_agent.py      # Main robot agent
├── warehouse/
│   ├── __init__.py
│   ├── map.py             # Static warehouse map
│   └── shelf.py           # Shelf management
├── simulation/
│   ├── __init__.py
│   ├── mujoco_env.py      # MuJoCo environment setup
│   └── visualizer.py      # Real-time visualization
└── utils/
    ├── __init__.py
    └── geometry.py        # Geometry utilities
```

## Success Criteria
- Robot successfully delivers shelf to destination
- No crashes or physics errors
- Smooth motion with collision avoidance
- Real-time performance on standard hardware

## Next Phases
- Phase 2: Multi-task handling via JobsQueue
- Phase 3: Multi-robot fleet with collision avoidance
- Phase 4: Scalable fleet with multiple orchestrators 