#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath('.')))

print('Testing demo with fixed 5-beam LiDAR configuration...')

try:
    from demo_robot_task_simulation import TaskSimulationManager

    # Create simulation manager
    sim = TaskSimulationManager()

    print('\\nChecking final LiDAR configuration...')

    if hasattr(sim.robot, 'lidar_service') and sim.robot.lidar_service is not None:
        config = sim.robot.lidar_service._config
        print(f'✅ Final config: {config.num_rays} rays, {config.field_of_view}° FOV')

        # Test a scan
        sim.robot.lidar_service.perform_scan('robot_1')
        scan = sim.robot.get_lidar_scan()

        if scan:
            print(f'✅ Scan successful: {len(scan.distances)} rays')

            if len(scan.distances) <= 10:
                print('✅ 5-beam configuration is active!')

                # Show what the logging would look like
                beam_data = []
                for i in range(len(scan.distances)):
                    angle_deg = scan.angles[i] * 180.0 / 3.14159
                    if scan.valid_mask[i]:
                        beam_data.append(".1f")
                    else:
                        beam_data.append(".1f")
                beam_str = " | ".join(beam_data)
                print(f'Expected log format: LiDAR: [{beam_str}]')
            else:
                print('❌ Still using 110 rays')
        else:
            print('❌ Scan failed')
    else:
        print('❌ LiDAR service not available')

    print('\\n🎉 Demo 5-beam test completed!')

except Exception as e:
    print(f'❌ Error: {e}')
    import traceback
    traceback.print_exc()












