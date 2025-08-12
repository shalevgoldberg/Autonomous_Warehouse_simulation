#!/usr/bin/env python3
"""
Simple Long Distance Robot Movement Demo - Console Only
Shows robot moving across warehouse without MuJoCo visualization conflicts.
"""
import time
import threading
from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent import RobotAgent, RobotConfiguration
from interfaces.task_handler_interface import Task, TaskType


class SimpleLongDistanceDemo:
    """Simple demo showing long-distance robot movement."""
    
    def __init__(self):
        """Initialize demo."""
        print("[*] Initializing Simple Long Distance Demo")
        
        # Initialize system
        self.warehouse_map = WarehouseMap()
        self.physics = SimpleMuJoCoPhysics(self.warehouse_map)
        self.robot = RobotAgent(
            warehouse_map=self.warehouse_map,
            physics=self.physics,
            config=RobotConfiguration(robot_id="demo_robot")
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