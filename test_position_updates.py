#!/usr/bin/env python3
import sys
sys.path.append('.')
from demo_multi_robot_simulation import MultiRobotSimulationManager

print('=== TESTING POSITION UPDATE FLOW ===')

# Create simulation manager
sim = MultiRobotSimulationManager(robot_count=1)

print('\n--- CALLING START_SIMULATION ---')
sim.start_simulation()

print('\n--- WAITING FOR POSITION UPDATES ---')
import time
for i in range(3):
    time.sleep(1)
    print(f'Waited {i+1} seconds...')

print('\n--- CHECKING FINAL POSITIONS ---')
for robot in sim.robots:
    try:
        pose = robot.robot.physics.get_pose()
        print(f'{robot.robot_id} physics pose: ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})')

        status = robot.robot.get_status()
        pos_from_status = status.get('position', [0, 0, 0])
        print(f'{robot.robot_id} status position: ({pos_from_status[0]:.3f}, {pos_from_status[1]:.3f}, {pos_from_status[2]:.3f})')
    except Exception as e:
        print(f'{robot.robot_id}: Error - {e}')

print('\n✅ Test complete')
