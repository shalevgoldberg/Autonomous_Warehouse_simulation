#!/usr/bin/env python3
"""
Simple Long Distance Robot Movement Demo - Console Only
Shows robot moving across warehouse without MuJoCo visualization conflicts.
"""
import time
import threading
from unittest.mock import MagicMock
from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from interfaces.configuration_interface import RobotConfig
from interfaces.task_handler_interface import Task, TaskType


class SimpleLongDistanceDemo:
    """Simple demo showing long-distance robot movement."""
    
    def __init__(self):
        """Initialize demo."""
        print("[*] Initializing Simple Long Distance Demo")
        
        # Initialize system
        self.warehouse_map = WarehouseMap()
        self.physics = SimpleMuJoCoPhysics(self.warehouse_map)
        # Create robot configuration
        robot_config = RobotConfig(
            robot_id="demo_robot",
            max_speed=2.0,
            control_frequency=10.0,
            motion_frequency=100.0,
            cell_size=1.0,
            lane_tolerance=0.2,
            corner_speed=1.0,
            bay_approach_speed=0.5,
            conflict_box_lock_timeout=5.0,
            conflict_box_heartbeat_interval=1.0,
            max_linear_velocity=2.0,
            max_angular_velocity=2.0,
            movement_speed=2.0,
            wheel_base=0.5,
            wheel_radius=0.1,
            picking_duration=2.0,
            dropping_duration=2.0,
            position_tolerance=0.1,
            charging_threshold=0.2,
            emergency_stop_distance=0.5,
            stall_recovery_timeout=10.0
        )
        
        # Create mock configuration provider
        config_provider = MagicMock()
        config_provider.get_robot_config.return_value = robot_config
        config_provider.get_database_config.return_value = MagicMock()
        config_provider.get_value.return_value = MagicMock(value=(0.0, 0.0))
        config_provider.errors = []
        
        # Create bid configuration
        bid_config = MagicMock()
        bid_config.distance_weight = 0.3
        bid_config.battery_weight = 0.2
        bid_config.workload_weight = 0.2
        bid_config.task_type_compatibility_weight = 0.1
        bid_config.robot_capabilities_weight = 0.2
        bid_config.time_urgency_weight = 0.1
        bid_config.conflict_box_availability_weight = 0.1
        bid_config.shelf_accessibility_weight = 0.1
        bid_config.enable_distance_factor = True
        bid_config.enable_battery_factor = True
        bid_config.enable_workload_factor = True
        bid_config.enable_task_type_compatibility_factor = True
        bid_config.enable_robot_capabilities_factor = True
        bid_config.enable_time_urgency_factor = True
        bid_config.enable_conflict_box_availability_factor = True
        bid_config.enable_shelf_accessibility_factor = True
        bid_config.battery_threshold = 0.2
        bid_config.calculation_timeout = 5.0
        bid_config.max_distance_normalization = 100.0
        bid_config.enable_parallel_calculation = True
        bid_config.enable_calculation_statistics = True
        bid_config.enable_factor_breakdown = True
        bid_config.max_parallel_workers = 4
        config_provider.get_bid_config.return_value = bid_config
        
        # Create mock simulation data service
        mock_sim_service = MagicMock()
        mock_sim_service.get_map_data.return_value = MagicMock(
            width=10,
            height=15,
            cell_size=1.0,
            start_position=(0.0, 0.0),
            walkable_cells=[(x, y) for x in range(10) for y in range(15)],
            blocked_cells=[],
            obstacles=[],
            shelves={},
            dropoff_zones=[(5.0, 5.0)]
        )
        mock_sim_service.get_navigation_graph.return_value = MagicMock()
        mock_sim_service.get_blocked_cells.return_value = {}
        mock_sim_service.try_acquire_conflict_box_lock.return_value = True
        mock_sim_service.release_conflict_box_lock.return_value = True
        mock_sim_service.heartbeat_conflict_box_lock.return_value = True
        mock_sim_service.get_shelf_position.return_value = (1.0, 1.0)
        mock_sim_service.get_dropoff_zones.return_value = [(5.0, 5.0)]
        mock_sim_service.lock_shelf.return_value = True
        mock_sim_service.unlock_shelf.return_value = True
        mock_sim_service.log_event.return_value = None
        
        self.robot = RobotAgent(
            physics=self.physics,
            config_provider=config_provider,
            simulation_data_service=mock_sim_service
        )
        
        # Physics thread control
        from robot.impl.physics_integration import create_physics_thread_manager
        self.physics_manager = create_physics_thread_manager(self.physics, frequency_hz=1000.0)
        
        print(f"   [OK] Warehouse: {self.warehouse_map.width}x{self.warehouse_map.height}")
        print(f"   [OK] Robot initialized at: {self.physics.get_robot_pose()}")
    
    def start_physics(self):
        """Start physics loop."""
        self.physics_manager.start()
        print("   [OK] Physics loop started at 1000Hz")
    
    def stop_physics(self):
        """Stop physics loop."""
        self.physics_manager.stop()
        print("   [OK] Physics loop stopped")
    
    def create_long_distance_tasks(self):
        """Create tasks that traverse the warehouse."""
        # Use proper grid-to-world conversion for valid positions
        grid_targets = [
            ("Far East", 9, 2),      # Far right, walkable row
            ("Far North", 9, 11),    # Far right, top
            ("Far West", 1, 5),      # Far left, middle
            ("Center", 5, 8),        # Center, walkable
        ]
        
        tasks = []
        for name, grid_x, grid_y in grid_targets:
            world_x, world_y = self.warehouse_map.grid_to_world(grid_x, grid_y)
            
            task = Task(
                task_id=f"move_to_{name.lower().replace(' ', '_')}",
                task_type=TaskType.MOVE_TO_POSITION,
                target_position=(world_x, world_y, 0.0),
                priority=1,
                metadata={"target_name": name}
            )
            tasks.append((name, task))
        
        return tasks
    
    def run(self):
        """Run the complete demo."""
        try:
            # Start systems
            self.start_physics()
            self.robot.start()
            
            # Create tasks
            tasks = self.create_long_distance_tasks()
            
            print(f"\n[*] Running {len(tasks)} long-distance movement tasks:")
            
            for i, (target_name, task) in enumerate(tasks):
                print(f"\n--- Task {i+1}: Move to {target_name} ---")
                
                # Get current position
                current_pos = self.physics.get_robot_pose()
                target_pos = task.target_position
                
                # Calculate distance
                distance = ((target_pos[0] - current_pos[0])**2 + 
                           (target_pos[1] - current_pos[1])**2)**0.5
                
                print(f"   From: ({current_pos[0]:.1f}, {current_pos[1]:.1f})")
                print(f"   To:   ({target_pos[0]:.1f}, {target_pos[1]:.1f})")
                print(f"   🚀 Distance: {distance:.1f} meters")
                
                # Assign task
                success = self.robot.assign_task(task)
                if not success:
                    print(f"   [X] Failed to assign task - robot may be busy")
                    # Force reset robot to idle state
                    print(f"   🔄 Forcing robot to emergency stop and reset...")
                    self.robot.task_handler.emergency_stop()
                    time.sleep(1.0)  # Give time for reset
                    
                    # Try again
                    success = self.robot.assign_task(task)
                    if not success:
                        print(f"   ❌ Still failed to assign task after reset")
                        continue
                    else:
                        print(f"   ✅ Task assigned successfully after reset")
                
                # Monitor task execution
                task_start_time = time.time()
                max_task_time = 60.0  # Extended timeout for long-distance tasks
                last_pos = current_pos
                
                while time.time() - task_start_time < max_task_time:
                    # Get status
                    status = self.robot.get_status()
                    pos = self.physics.get_robot_pose()
                    
                    # Calculate movement since last check
                    movement = ((pos[0] - last_pos[0])**2 + (pos[1] - last_pos[1])**2)**0.5
                    last_pos = pos
                    
                    print(f"   [{time.time() - task_start_time:.1f}s] "
                          f"Pos: ({pos[0]:.1f}, {pos[1]:.1f}) | "
                          f"Status: {status['operational_status']} | "
                          f"Progress: {status['task_progress']:.1%} | "
                          f"Movement: {movement:.2f}m")
                    
                    # Check if task completed
                    if not status['task_active']:
                        final_pos = self.physics.get_robot_pose()
                        final_distance = ((target_pos[0] - final_pos[0])**2 + 
                                          (target_pos[1] - final_pos[1])**2)**0.5
                          
                        print(f"   [OK] Task completed!")
                        print(f"   [*] Final position: ({final_pos[0]:.1f}, {final_pos[1]:.1f})")
                        print(f"   [*] Distance to target: {final_distance:.2f}m")
                        break
                    
                    time.sleep(0.5)  # Update every 0.5 seconds
                
                else:
                    # Timeout occurred - force reset robot state
                    print(f"   ⚠️  Task timeout after {max_task_time}s")
                    print(f"   🔄 Forcing robot to stop and reset to idle state...")
                    self.robot.task_handler.emergency_stop()
                    time.sleep(1.0)  # Give time for reset
                    
                    # Verify robot is now idle
                    status = self.robot.get_status()
                    print(f"   📊 Robot status after reset: {status['operational_status']}")
                
                # Brief pause between tasks
                time.sleep(1.0)
            
            print(f"\n🎉 Demo completed! Robot traversed the warehouse successfully.")
            
        except KeyboardInterrupt:
            print("\n[!] Demo interrupted by user")
        
        except Exception as e:
            print(f"[!] Error during demo: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # Cleanup
            print(f"\n[*] Demo completed! Robot traversed the warehouse successfully.")
            self.stop_physics()
            self.robot.stop()


def main():
    """Main demo function."""
    print("=== SIMPLE LONG DISTANCE ROBOT MOVEMENT DEMO ===")
    
    try:
        demo = SimpleLongDistanceDemo()
        demo.run()
    except KeyboardInterrupt:
        print("\n[!] Demo interrupted by user")
    except Exception as e:
        print(f"[!] Demo failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n=== Demo finished ===")


if __name__ == "__main__":
    main() 