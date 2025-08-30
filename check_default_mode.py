#!/usr/bin/env python3
from config.configuration_provider import ConfigurationProvider

config = ConfigurationProvider()
mode = config.get_value('physics.mode', 'kinematic_guard')
default_mode = mode.value if hasattr(mode, 'value') else mode

print(f'Default physics mode: {default_mode}')
print('')
print('This is why you don\'t see SharedMuJoCoEngine logs!')
print('The logs only appear when using mujoco_authoritative mode.')
print('')
print('To see the logs, switch to mujoco_authoritative mode.')
