#!/usr/bin/env python3
import sys
sys.path.append('.')
from demo_multi_robot_simulation import MultiRobotSimulationManager

print('=== TESTING STATEHOLDER REFERENCE SETTING ===')

# Create simulation manager
sim = MultiRobotSimulationManager(robot_count=1)

print('\n--- CHECKING STATEHOLDER REFERENCE ---')
for robot in sim.robots:
    physics = robot.robot.physics
    has_state_holder = hasattr(physics, '_state_holder')
    state_holder_ref = getattr(physics, '_state_holder', None) if has_state_holder else None
    print(f'{robot.robot_id} physics has _state_holder: {has_state_holder}')
    print(f'{robot.robot_id} _state_holder is not None: {state_holder_ref is not None}')
    if state_holder_ref:
        print(f'{robot.robot_id} _state_holder type: {type(state_holder_ref)}')

print('\n✅ Reference check completed!')
