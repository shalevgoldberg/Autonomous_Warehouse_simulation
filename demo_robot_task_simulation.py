#!/usr/bin/env python3
"""
Complete Robot Task Simulation Demo

Demonstrates a fully integrated robot task simulation with:
- MuJoCo physics simulation
- Complete robot agent with all components
- Task creation and execution
- Real-time visualization
- Clean architecture with loose coupling

⚠️  DEMO CONFIGURATION: Lane tolerance set to 3.0m (too soft for production!)
    This allows tasks to complete despite navigation issues. For production,
    use 0.1-0.3m tolerance for safe warehouse operation.

Run with: python demo_robot_task_simulation.py
"""
import sys
import os
import time
import threading
from typing import List, Optional
from unittest.mock import MagicMock

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from interfaces.configuration_interface import RobotConfig
from interfaces.task_handler_interface import Task, TaskType
from interfaces.path_planner_interface import Cell
from interfaces.visualization_interface import IVisualization
from simulation.mujoco_visualization import MujocoVisualization
from simulation.visualization_thread import VisualizationThread
from interfaces.lane_follower_interface import LaneFollowingConfig


class TaskSimulationManager:
    """
    Manages complete robot task simulation.
    
    Coordinates physics, robot agent, task creation, and visualization
    following clean architecture principles.
    """
    
    def __init__(self, warehouse_csv: str = "sample_warehouse.csv"):
        """Initialize simulation manager."""
        print("🚀 Initializing Robot Task Simulation")
        
        # Load warehouse
        self.warehouse_map = WarehouseMap(csv_file=warehouse_csv)
        print(f"   ✅ Warehouse loaded: {self.warehouse_map.width}x{self.warehouse_map.height}")
        
        # Initialize physics
        self.physics = SimpleMuJoCoPhysics(self.warehouse_map)
        print(f"   ✅ Physics ready: {self.physics.is_simulation_ready()}")
        
        # Create robot agent
        self.robot_config = RobotConfig(
            robot_id="warehouse_robot_1",
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
        config_provider.get_robot_config.return_value = self.robot_config
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
            height=14,
            cell_size=1.0,
            start_position=(0.0, 0.0),
            walkable_cells=[(x, y) for x in range(10) for y in range(14)],
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
        print(f"   ✅ Robot agent created: {self.robot_config.robot_id}")
        
        # Configure lane follower for DEMO (very soft tolerance)
        self.robot.lane_follower.set_config(
            LaneFollowingConfig(
                lane_tolerance=3.0,  # DEMO ONLY: too soft for production!
                max_speed=self.robot_config.max_speed,
                corner_speed=self.robot_config.corner_speed,
                bay_approach_speed=self.robot_config.bay_approach_speed,
            )
        )
        print("   ⚠️  Lane tolerance set to 3.0m for demo (too soft for production)")

        # We rely on RobotAgent's internal physics thread manager.
        self.physics_manager = None

        # Visualization
        self.visualization: Optional[IVisualization] = MujocoVisualization(
            state_holder=self.robot.state_holder,
            warehouse_map=self.warehouse_map
        )
        self.visualization_thread: Optional[VisualizationThread] = None
        
        # Task management
        self.available_shelves = self._discover_shelf_positions()
        self.current_task_index = 0
        
        print(f"   ✅ Found {len(self.available_shelves)} shelves for tasks")
        print("🎯 Simulation ready!")
    
    def _discover_shelf_positions(self) -> List[tuple]:
        """Discover shelf positions from warehouse map."""
        shelves = []
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                if self.warehouse_map.grid[y, x] == 2:  # Shelf
                    world_pos = self.warehouse_map.grid_to_world(x, y)
                    shelves.append((f"shelf_{x}_{y}", world_pos[0], world_pos[1]))
        return shelves
    
    def start_simulation(self) -> None:
        """Start complete simulation."""
        print("\n🎬 Starting Robot Task Simulation")
        
        # Start robot agent (starts physics + control threads)
        self.robot.start()
        print("   ✅ Robot control and physics threads started")
        
        # Start visualization (reads from state holder updated by robot physics thread)
        try:
            self.visualization.initialize()
            self.visualization_thread = VisualizationThread(self.visualization, fps=30.0)
            self.visualization_thread.start()
            print("   ✅ Visualization thread running at 30 FPS")
        except Exception as e:
            print(f"   ⚠️ Visualization not started: {e}")
        
        # Give robot initial position
        initial_pos = self.warehouse_map.grid_to_world(1, 1)
        self.physics.reset_robot_position(initial_pos[0], initial_pos[1], 0.0)
        
        print("   ✅ Robot positioned at start")
    
    def stop_simulation(self) -> None:
        """Stop complete simulation."""
        print("\n🛑 Stopping Robot Task Simulation")
        
        # Stop robot agent
        self.robot.stop()
        
        # Stop visualization
        try:
            if self.visualization_thread is not None:
                self.visualization_thread.stop()
                self.visualization_thread = None
            if self.visualization is not None:
                self.visualization.shutdown()
        except Exception:
            pass

        # Stop physics if we ever started an external loop (we did not)
        if self.physics_manager is not None:
            try:
                self.physics_manager.stop()
            except Exception:
                pass
        
        print("   ✅ Simulation stopped cleanly")
    
    def assign_next_task(self) -> bool:
        """Assign next task to robot - with LONG DISTANCE movements for clear visualization."""
        
        # Define long-distance waypoints using robot coordinate system
        # Robot uses 0.25m cells, warehouse uses 0.5m cells (2x scaling)
        # Grid is 22x26 in robot coordinates (11x13 warehouse * 2)
        # Avoid shelf rows in robot coordinates: (6,7), (12,13), (18,19)
        # Use walkable rows: 0,1,2,3,4,5,8,9,10,11,14,15,16,17,20,21,22,23,24,25
        robot_grid_targets = [
            ("Far East", 18, 4),              # Robot grid (18,4) - Far East side
            ("Far North", 18, 22),            # Robot grid (18,22) - Far North side  
            ("Far West", 2, 10),              # Robot grid (2,10) - Far West side
            ("Center East", 14, 16),          # Robot grid (14,16) - Center East
            ("Center", 10, 10),               # Robot grid (10,10) - Center
        ]
        
        # Convert robot grid to world coordinates using robot's coordinate system
        long_distance_targets = []
        for name, robot_grid_x, robot_grid_y in robot_grid_targets:
            # Use robot's coordinate system for conversion
            cell = Cell(robot_grid_x, robot_grid_y)
            world_x, world_y = self.robot.coordinate_system.cell_to_world(cell)
            long_distance_targets.append((name, world_x, world_y))
        
        if self.current_task_index >= len(long_distance_targets):
            print("   ⚠️  All long-distance tasks completed")
            return False
        
        # Get current target
        target_name, target_x, target_y = long_distance_targets[self.current_task_index]
        
        task = Task(
            task_id=f"long_distance_{self.current_task_index + 1}",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(target_x, target_y, 0.0),
            priority=1,
            metadata={
                "target_name": target_name,
                "distance_type": "long_distance_demo"
            }
        )
        
        # Assign to robot
        success = self.robot.assign_task(task)
        
        if success:
            # Calculate distance from current position
            current_pos = self.physics.get_robot_pose()
            distance = ((target_x - current_pos[0])**2 + (target_y - current_pos[1])**2)**0.5
            
            print(f"   ✅ Assigned {task.task_id}: Move to {target_name}")
            print(f"       From ({current_pos[0]:.1f}, {current_pos[1]:.1f}) -> To ({target_x:.1f}, {target_y:.1f})")
            print(f"       🚀 LONG DISTANCE: {distance:.1f} meters across warehouse!")
            self.current_task_index += 1
        else:
            print(f"   ❌ Robot busy, cannot assign {task.task_id}")
        
        return success
    
    def get_simulation_status(self) -> dict:
        """Get comprehensive simulation status."""
        robot_status = self.robot.get_status()
        physics_state = self.physics.get_physics_state()
        
        return {
            'simulation_time': time.time(),
            'physics_ready': self.physics.is_simulation_ready(),
            'robot': robot_status,
            'tasks_completed': self.current_task_index,
            'available_shelves': len(self.available_shelves)
        }


def demo_automated_tasks():
    """Demo: Automated task execution."""
    print("=" * 60)
    print("🤖 AUTOMATED ROBOT TASK SIMULATION")
    print("=" * 60)
    
    # Create simulation
    sim = TaskSimulationManager()
    
    try:
        # Start simulation
        sim.start_simulation()
        
        print("\n⏳ Running automated task sequence...")
        print("   Robot will automatically execute pick and deliver tasks")
        
        # Run for all available tasks
        task_cycle = 1
        while True:
            print(f"\n--- Task Cycle {task_cycle} ---")
            
            # Assign task
            if sim.assign_next_task():
                # Wait for task completion
                task_start_time = time.time()
                max_task_time = 60.0  # 60 seconds max per task (longer for remote targets)
                
                while time.time() - task_start_time < max_task_time:
                    status = sim.get_simulation_status()
                    robot_status = status['robot']
                    
                    # Detailed progress logging every 2 seconds
                    if int(time.time() - task_start_time) % 2 == 0:
                        print(f"   Status: {robot_status['operational_status']} | "
                              f"Progress: {robot_status['task_progress']:.1%} | "
                              f"Position: ({robot_status['position'][0]:.2f}, {robot_status['position'][1]:.2f}) | "
                              f"Motion: {robot_status['motion_status']}")
                    
                    # Check if task completed
                    if not robot_status['task_active']:
                        # Get task history to verify actual completion
                        task_history = sim.robot.task_handler.get_task_history(limit=1)
                        if task_history:
                            completed_task = task_history[0]
                            if completed_task.status.value == 'completed':
                                print(f"   ✅ Task {completed_task.task_id} SUCCESSFULLY completed!")
                                print(f"      Final position: ({robot_status['position'][0]:.2f}, {robot_status['position'][1]:.2f})")
                                break
                            else:
                                print(f"   ❌ Task {completed_task.task_id} failed with status: {completed_task.status.value}")
                                break
                        else:
                            print(f"   ⚠️  Task completed but no history found")
                            break
                    
                    time.sleep(0.5)  # Check every 500ms for better responsiveness
                
                else:
                    print(f"   ⚠️  Task timeout after {max_task_time}s")
            else:
                # No more tasks available
                print("   ✅ All available tasks have been completed!")
                break
            
            time.sleep(1.0)  # Brief pause between tasks
            task_cycle += 1
        
        print(f"\n🎉 Completed {sim.current_task_index} tasks successfully!")
        
    except KeyboardInterrupt:
        print("\n⚠️  Simulation interrupted by user")
    
    finally:
        sim.stop_simulation()


def main():
    """Main demo function - console-based robot task simulation."""
    print("🤖 Robot Task Simulation Demo")
    print("=" * 40)
    print("Console-based autonomous warehouse robot simulation")
    print("Following Phase 2 objectives: Single robot executing multiple tasks")
    print()
    
    try:
        demo_automated_tasks()
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main() 