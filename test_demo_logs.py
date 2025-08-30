#!/usr/bin/env python3
"""
Test script to check if SharedMuJoCoEngine logs appear in demo
"""

import sys
sys.path.append('.')

print('=== TESTING SHARED MUJOCO ENGINE LOGS IN DEMO ===')

from demo_multi_robot_simulation import MultiRobotSimulationManager

print('Creating simulation manager (this should trigger SharedMuJoCoEngine logs)...')

# Create simulation manager - this should show engine logs
sim = MultiRobotSimulationManager(robot_count=1)

print('✅ Simulation manager created successfully')
print('Check above for SharedMuJoCoEngine logs...')
