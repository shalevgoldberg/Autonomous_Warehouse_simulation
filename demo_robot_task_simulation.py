#!/usr/bin/env python3
"""
Complete Robot Task Simulation Demo - PICK_AND_DELIVER Edition

Demonstrates a fully integrated robot task simulation with:
- MuJoCo physics simulation
- Complete robot agent with all components
- PICK_AND_DELIVER task execution (shelf → drop-off → idle)
- Real database integration with inventory management
- Real-time visualization
- LiDAR collision avoidance system with live scan data logging
- Clean architecture with loose coupling

⚠️  DEMO CONFIGURATION: Lane tolerance set to 3.0m (too soft for production!)
    This allows tasks to complete despite navigation issues. For production,
    use 0.1-0.3m tolerance for safe warehouse operation.

LiDAR Features in Demo:
- Real-time scan data logging (every 3 seconds)
- Individual beam distance data (when ≤10 rays)
- Obstacle detection and distance reporting
- LiDAR operational status monitoring
- Detailed scan performance summaries per task
- Configured for 5 beams for optimal logging visibility

Run with: python demo_robot_task_simulation.py
"""
import sys
import os
import time
import threading
from typing import List, Optional
from unittest.mock import MagicMock
import logging
import numpy as np

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
from config.configuration_provider import ConfigurationProvider
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from pathlib import Path


class TaskSimulationManager:
    """
    Manages complete robot task simulation.
    
    Coordinates physics, robot agent, task creation, and visualization
    following clean architecture principles.
    """
    
    def __init__(self, warehouse_csv: str = "sample_warehouse.csv"):
        """Initialize simulation manager."""
        print("🚀 Initializing Robot Task Simulation - PICK_AND_DELIVER Edition")
        
        # Load warehouse
        self.warehouse_map = WarehouseMap(csv_file=warehouse_csv)
        print(f"   ✅ Warehouse loaded: {self.warehouse_map.width}x{self.warehouse_map.height}")
        
        # Create real configuration provider (needed before shared engine)
        self.config_provider = ConfigurationProvider()
        print("   ✅ Configuration provider initialized")

        # Initialize physics (prefer shared engine so appearance/contacts flow through)
        try:
            from simulation.shared_mujoco_engine import SharedMuJoCoEngine, SharedMuJoCoPhysics
            shared_engine = SharedMuJoCoEngine(self.warehouse_map, physics_dt=0.001, enable_time_gating=True,
                                              config_provider=self.config_provider)
            # Use same starting position as multi-robot demo for consistency
            world_pos = self.warehouse_map.grid_to_world(1, 1)
            shared_engine.register_robot("robot_1", (world_pos[0], world_pos[1], 0.0))
            shared_engine.initialize()
            self.physics = SharedMuJoCoPhysics(shared_engine, robot_id="robot_1")
        except Exception as e:
            print(f"   ⚠️  SharedMuJoCoEngine not available ({e}); falling back to SimpleMuJoCoPhysics")
            self.physics = SimpleMuJoCoPhysics(self.warehouse_map, robot_id="robot_1")
        print(f"   ✅ Physics ready: {self.physics.is_simulation_ready()}")
        
        # Configuration provider already initialized above
        
        # Create real simulation data service
        self.simulation_data_service = SimulationDataServiceImpl(self.warehouse_map)
        print("   ✅ Simulation data service initialized")
        
        # Persist grouped navigation graph (from CSV) into DB to ensure up-to-date conflict boxes
        try:
            result = self.simulation_data_service.persist_navigation_graph_from_csv(
                Path(warehouse_csv), clear_existing=True
            )
            print(f"   ✅ Navigation graph persisted: boxes={result.boxes_persisted}, "
                  f"nodes={result.nodes_persisted}, edges={result.edges_persisted}")
        except Exception as e:
            print(f"   ⚠️  Warning: Could not persist navigation graph: {e}")
        
        # Populate inventory for demo
        self._populate_demo_inventory()
        
        # Create robot agent with real services using configuration provider
        # This ensures all parameters including new robot dimensions are properly loaded
        self.robot_config = self.config_provider.get_robot_config("warehouse_robot_1")

        # Override demo-specific parameters for stability
        from interfaces.configuration_interface import RobotConfig
        self.robot_config = RobotConfig(
            robot_id="warehouse_robot_1",
            # Use configured robot dimensions
            robot_width=self.robot_config.robot_width,
            robot_height=self.robot_config.robot_height,
            robot_length=self.robot_config.robot_length,
            # Use proportional wheel parameters
            wheel_base_ratio=self.robot_config.wheel_base_ratio,
            wheel_radius_ratio=self.robot_config.wheel_radius_ratio,
            # Demo-specific overrides for stability
            max_speed=0.5,  # supposed to be 0.5, reduced
            control_frequency=10.0,
            motion_frequency=100.0,
            cell_size=1.0,
            lane_tolerance=0.2,
            corner_speed=0.3,  # Reduced for stability
            bay_approach_speed=0.2,  # Reduced for stability
            conflict_box_lock_timeout=5.0,
            conflict_box_heartbeat_interval=1.0,
            max_linear_velocity=0.5,  # Reduced for stability with lower actuator gains
            max_angular_velocity=0.5,  # Reduced for stability
            movement_speed=0.5,  # Reduced for stability with lower actuator gains
            picking_duration=2.0,
            dropping_duration=2.0,
            position_tolerance=0.1,
            emergency_stop_distance=0.5,
            stall_recovery_timeout=10.0
        )
        
        # Ensure robot id exists in DB for FK constraints used by conflict box locks
        try:
            with self.simulation_data_service._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        "INSERT INTO robots (robot_id, name) VALUES (%s, %s) ON CONFLICT (robot_id) DO NOTHING",
                        ("robot_1", "Robot 1"),
                    )
                    conn.commit()
        except Exception:
            pass

        # Create robot agent with real simulation data service
        self.robot = RobotAgent(
            physics=self.physics,
            config_provider=self.config_provider,
            simulation_data_service=self.simulation_data_service
        )
        print(f"   ✅ Robot agent created: {self.robot_config.robot_id}")

        # Configure LiDAR for demo with fewer rays for better logging
        if hasattr(self.robot, 'lidar_service') and self.robot.lidar_service is not None:
            try:
                # Create demo config with 5 rays for detailed logging
                from interfaces.lidar_interface import LiDARConfig
                demo_lidar_config = LiDARConfig(
                    enabled=True,
                    num_rays=5,  # Reduced for demo logging
                    max_range=3.0,
                    min_range=0.2,
                    scan_frequency=10.0,
                    field_of_view=90.0,  # 90° spread over 5 rays
                    safety_distance=0.8,
                    enable_scan_caching=True,
                    scan_cache_ttl=0.1
                )

                # Update the LiDAR service with demo config
                self.robot.lidar_service.set_config("robot_1", demo_lidar_config)

                # Wait for configuration to be eligible for application (100ms minimum)
                import time
                time.sleep(0.15)  # Wait 150ms to be safe

                # Apply the configuration
                if hasattr(self.robot.lidar_service, '_apply_pending_config'):
                    self.robot.lidar_service._apply_pending_config()

                print(f"   🔍 LiDAR configured for demo: {demo_lidar_config.num_rays} rays, "
                      f"{demo_lidar_config.max_range}m range, {demo_lidar_config.field_of_view}° FOV")
            except Exception as e:
                print(f"   🔍 LiDAR: Demo configuration failed ({str(e)[:50]}...)")
        else:
            print("   🔍 LiDAR: Not available (requires mujoco_authoritative physics)")

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
            warehouse_map=self.warehouse_map,
            robot_config=self.robot_config
        )
        self.visualization_thread: Optional[VisualizationThread] = None
        
        # Task management
        self.available_shelves = self._discover_shelf_positions()
        self.current_task_index = 0
        
        print(f"   ✅ Found {len(self.available_shelves)} shelves for tasks")
        print("🎯 Simulation ready!")
    
    def _populate_demo_inventory(self) -> None:
        """Populate shelves with demo inventory for PICK_AND_DELIVER tasks."""
        print("   📦 Populating demo inventory...")
        
        try:
            # Create shelves from warehouse map
            shelves_created = self.simulation_data_service.create_shelves_from_map(clear_existing=True)
            print(f"      ✅ Created {shelves_created} shelves from warehouse map")
            
            # Create demo inventory data based on actual warehouse layout
            # From sample_warehouse.csv: shelves are in rows 3, 6, 9 (0-indexed)
            # Warehouse is 10x14, with shelves from x=3 to x=7
            # Shelf IDs now use grid coordinates: shelf_x_y
            demo_inventory = [
                {'item_id': 'laptop_001', 'name': 'Gaming Laptop', 'shelf_id': 'shelf_3_3', 'quantity': 5},
                {'item_id': 'phone_001', 'name': 'Smartphone', 'shelf_id': 'shelf_4_3', 'quantity': 10},
                {'item_id': 'tablet_001', 'name': 'Tablet', 'shelf_id': 'shelf_6_3', 'quantity': 8},
                {'item_id': 'headphones_001', 'name': 'Wireless Headphones', 'shelf_id': 'shelf_7_3', 'quantity': 15},
                {'item_id': 'keyboard_001', 'name': 'Mechanical Keyboard', 'shelf_id': 'shelf_3_6', 'quantity': 12},
                {'item_id': 'mouse_001', 'name': 'Gaming Mouse', 'shelf_id': 'shelf_4_6', 'quantity': 20},
                {'item_id': 'monitor_001', 'name': '4K Monitor', 'shelf_id': 'shelf_6_6', 'quantity': 6},
                {'item_id': 'speaker_001', 'name': 'Bluetooth Speaker', 'shelf_id': 'shelf_7_6', 'quantity': 9},
                {'item_id': 'camera_001', 'name': 'Action Camera', 'shelf_id': 'shelf_3_9', 'quantity': 7},
                {'item_id': 'drone_001', 'name': 'Quadcopter Drone', 'shelf_id': 'shelf_4_9', 'quantity': 4},
                {'item_id': 'gaming_console_001', 'name': 'Gaming Console', 'shelf_id': 'shelf_6_9', 'quantity': 3},
                {'item_id': 'vr_headset_001', 'name': 'VR Headset', 'shelf_id': 'shelf_7_9', 'quantity': 8},
            ]
            
            # Populate inventory
            inventory_created = self.simulation_data_service.populate_inventory(demo_inventory)
            print(f"      ✅ Populated {inventory_created} inventory items")
            
            # Get inventory statistics
            stats = self.simulation_data_service.get_inventory_statistics()
            print(f"      📊 Inventory stats: {stats['total_items']} items across {stats['total_shelves']} shelves")
            
        except Exception as e:
            print(f"      ⚠️  Warning: Could not populate inventory: {e}")
            print("      Continuing with mock inventory...")
    
    def _discover_shelf_positions(self) -> List[tuple]:
        """Discover shelf positions from warehouse map."""
        shelves = []
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                if self.warehouse_map.grid[y, x] == 2:  # Shelf
                    world_pos = self.warehouse_map.grid_to_world(x, y)
                    shelf_id = f"shelf_{x}_{y}"
                    shelves.append((shelf_id, world_pos[0], world_pos[1]))
        return shelves
    
    def start_simulation(self) -> None:
        """Start complete simulation."""
        print("\n🎬 Starting Robot Task Simulation")

        # Ensure LaneFollower direct lock acquisition/release logs are visible
        # Configure only the LaneFollower logger to avoid impacting other components (e.g., MuJoCo viewer)
        try:
            # Configure the specific robot's LaneFollower logger to avoid global side-effects
            lane_logger = logging.getLogger(f"LaneFollower.{self.robot.robot_id}")
            lane_logger.setLevel(logging.INFO)
            # Attach a console handler once (avoid duplicates on reruns)
            if not any(isinstance(h, logging.StreamHandler) for h in lane_logger.handlers):
                handler = logging.StreamHandler()
                handler.setLevel(logging.INFO)
                handler.setFormatter(logging.Formatter("[%(levelname)s] [%(name)s] %(message)s"))
                lane_logger.addHandler(handler)
        except Exception:
            # Never fail demo due to logging setup
            pass
        
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
        initial_pos = self.warehouse_map.grid_to_world(5, 4)
        self.physics.reset_robot_position(initial_pos[0], initial_pos[1], 0.0)

        print("   ✅ Robot positioned at start")

        # Initial LiDAR test
        if hasattr(self.robot, 'lidar_service') and self.robot.lidar_service is not None:
            try:
                # Give LiDAR a moment to initialize
                time.sleep(0.1)
                initial_scan = self.robot.get_lidar_scan()
                if initial_scan is not None:
                    valid_points = initial_scan.valid_mask.sum()
                    total_rays = len(initial_scan.distances)

                    if total_rays <= 10 and valid_points > 0:
                        # Show individual beam data for few rays
                        beam_data = []
                        for i in range(total_rays):
                            angle_deg = initial_scan.angles[i] * 180.0 / 3.14159
                            if initial_scan.valid_mask[i]:
                                beam_data.append(f"{angle_deg:.0f}°:{initial_scan.distances[i]:.1f}m")
                            else:
                                beam_data.append(f"{angle_deg:.0f}°:invalid")
                        beam_str = " | ".join(beam_data)
                        print(f"   🔍 LiDAR test scan: [{beam_str}] - System ready!")
                    else:
                        print(f"   🔍 LiDAR test scan: {valid_points}/{total_rays} rays valid - System ready!")
                else:
                    print("   🔍 LiDAR: Initializing scan data...")
            except Exception as e:
                print(f"   🔍 LiDAR: Initialization check failed ({str(e)[:30]}...)")
        else:
            print("   🔍 LiDAR: Not available for this simulation")
    
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
        """Assign next PICK_AND_DELIVER task to robot."""
        
        if self.current_task_index >= len(self.available_shelves):
            print("   ⚠️  All PICK_AND_DELIVER tasks completed")
            return False
        
        # Get current shelf target
        shelf_id, shelf_x, shelf_y = self.available_shelves[self.current_task_index]
        
        # Map shelf IDs to item IDs based on our inventory
        shelf_to_item_map = {
            'shelf_3_3': 'laptop_001',
            'shelf_4_3': 'phone_001',
            'shelf_6_3': 'tablet_001',
            'shelf_7_3': 'headphones_001',
            'shelf_3_6': 'keyboard_001',
            'shelf_4_6': 'mouse_001',
            'shelf_6_6': 'monitor_001',
            'shelf_7_6': 'speaker_001',
            'shelf_3_9': 'camera_001',
            'shelf_4_9': 'drone_001',
            'shelf_6_9': 'gaming_console_001',
            'shelf_7_9': 'vr_headset_001',
        }
        
        # Get the item ID for this shelf
        item_id = shelf_to_item_map.get(shelf_id, f"item_{self.current_task_index + 1}")
        
        # Create PICK_AND_DELIVER task
        task = Task.create_pick_and_deliver_task(
            task_id=f"pick_deliver_{self.current_task_index + 1}",
            order_id=f"demo_order_{self.current_task_index + 1}",
            shelf_id=shelf_id,
            item_id=item_id,
            quantity_to_pick=1,
            order_priority="normal",
            customer_id=f"demo_customer_{self.current_task_index + 1}",
            dropoff_zone="default_dropoff"
        )
        
        # Assign to robot
        success = self.robot.task_handler.start_task(task)
        
        if success:
            # Calculate distance from current position
            current_pos = self.physics.get_robot_pose()
            distance = ((shelf_x - current_pos[0])**2 + (shelf_y - current_pos[1])**2)**0.5
            
            print(f"   ✅ Assigned {task.task_id}: Pick from {shelf_id}")
            print(f"       From ({current_pos[0]:.1f}, {current_pos[1]:.1f}) -> To shelf at ({shelf_x:.1f}, {shelf_y:.1f})")
            print(f"       📦 PICK_AND_DELIVER: {distance:.1f} meters to shelf!")
            print(f"       🎯 Task: Pick 1x {task.item_id} from {shelf_id}")
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
    """Demo: Automated PICK_AND_DELIVER task execution."""
    print("=" * 60)
    print("🤖 AUTOMATED ROBOT PICK_AND_DELIVER SIMULATION")
    print("=" * 60)
    
    # Create simulation
    sim = TaskSimulationManager()
    
    try:
        # Start simulation
        sim.start_simulation()
        
        print("\n⏳ Running automated PICK_AND_DELIVER task sequence...")
        print("   Robot will automatically execute: SHELF → DROP-OFF → IDLE ZONE")
        
        # Run fewer tasks to keep the demo short
        max_tasks = 2
        task_cycle = 1
        while task_cycle <= max_tasks:
            print(f"\n--- Task Cycle {task_cycle}/{max_tasks} ---")
            
            # Assign task
            if sim.assign_next_task():
                # Wait for task completion
                task_start_time = time.time()
                max_task_time = 120.0  # 120 seconds max per task (longer for PICK_AND_DELIVER)
                
                while time.time() - task_start_time < max_task_time:
                    status = sim.get_simulation_status()
                    robot_status = status['robot']
                    
                    # Detailed progress logging every 3 seconds
                    if int(time.time() - task_start_time) % 3 == 0:
                        # Get status from task handler instead of robot status
                        task_status = robot_status.get('task_status')
                        if task_status:
                            operational_status = task_status.operational_status.value
                            progress = task_status.progress
                        else:
                            operational_status = 'unknown'
                            progress = 0.0
                        
                        position = robot_status.get('position', [0.0, 0.0, 0.0])
                        motion_status = robot_status.get('motion_status', 'unknown')
                        
                        # Get battery level from robot status
                        battery_level = robot_status.get('battery_level', 0.0)

                        # Get conflict box status
                        acquired_boxes = robot_status.get('acquired_conflict_boxes', [])
                        pending_box = robot_status.get('pending_conflict_box')

                        # Format conflict box info
                        if acquired_boxes:
                            conflict_info = f"Locks: {acquired_boxes}"
                        else:
                            conflict_info = "Locks: None"

                        if pending_box:
                            conflict_info += f" | Pending: {pending_box}"
                        else:
                            conflict_info += " | Pending: None"

                        # Get LiDAR scan data
                        lidar_info = "LiDAR: N/A"
                        if hasattr(sim.robot, 'lidar_service') and sim.robot.lidar_service is not None:
                            try:
                                lidar_scan = sim.robot.get_lidar_scan()
                                if lidar_scan is not None:
                                    valid_points = lidar_scan.valid_mask.sum()
                                    total_rays = len(lidar_scan.distances)

                                    if valid_points > 0:
                                        distances = lidar_scan.distances[lidar_scan.valid_mask]
                                        min_dist = distances.min()
                                        max_dist = distances.max()

                                        # For few rays, show individual beam data
                                        if total_rays <= 10:
                                            beam_distances = []
                                            for i in range(total_rays):
                                                angle_deg = lidar_scan.angles[i] * 180.0 / 3.14159
                                                if lidar_scan.valid_mask[i]:
                                                    beam_distances.append(f"{angle_deg:.0f}°:{lidar_scan.distances[i]:.2f}m")
                                                else:
                                                    beam_distances.append(f"{angle_deg:.0f}°:invalid")
                                            beam_str = " | ".join(beam_distances)
                                            lidar_info = f"LiDAR: [{beam_str}]"
                                        else:
                                            # For many rays, show summary
                                            lidar_info = f"LiDAR: {valid_points}/{total_rays} valid, {min_dist:.2f}-{max_dist:.2f}m"
                                    else:
                                        lidar_info = f"LiDAR: No obstacles ({total_rays} rays)"
                                elif sim.robot.is_lidar_operational():
                                    lidar_info = "LiDAR: Scanning..."
                                else:
                                    lidar_info = "LiDAR: Offline"
                            except Exception as e:
                                lidar_info = f"LiDAR: Error ({str(e)[:20]}...)"

                        print(f"   Status: {operational_status} | "
                              f"Progress: {progress:.1%} | "
                              f"Position: ({position[0]:.2f}, {position[1]:.2f}) | "
                              f"Motion: {motion_status} | "
                              f"Battery: {battery_level:.1%} | "
                              f"{conflict_info} | "
                              f"{lidar_info}")
                    
                    # Check if task completed
                    task_status = robot_status.get('task_status')
                    if task_status and not task_status.has_active_task:
                        # Get task history to verify actual completion
                        task_history = sim.robot.task_handler.get_task_history(limit=1)
                        if task_history:
                            completed_task = task_history[0]
                            if completed_task.status.value == 'completed':
                                # Safely get final position
                                final_position = robot_status.get('position', [0.0, 0.0, 0.0])
                                print(f"   ✅ Task {completed_task.task_id} SUCCESSFULLY completed!")
                                print(f"      Final position: ({final_position[0]:.2f}, {final_position[1]:.2f})")
                                print(f"      🎯 PICK_AND_DELIVER: Shelf → Drop-off → Idle Zone")

                                # Log LiDAR performance summary for completed task
                                if hasattr(sim.robot, 'lidar_service') and sim.robot.lidar_service is not None:
                                    try:
                                        final_scan = sim.robot.get_lidar_scan()
                                        if final_scan is not None:
                                            valid_points = final_scan.valid_mask.sum()
                                            total_rays = len(final_scan.distances)

                                            if valid_points > 0:
                                                distances = final_scan.distances[final_scan.valid_mask]
                                                avg_distance = distances.mean()

                                                # Show detailed beam data for few rays
                                                if total_rays <= 10:
                                                    beam_details = []
                                                    for i in range(total_rays):
                                                        angle_deg = final_scan.angles[i] * 180.0 / 3.14159
                                                        if final_scan.valid_mask[i]:
                                                            beam_details.append(f"{angle_deg:.0f}°:{final_scan.distances[i]:.2f}m")
                                                        else:
                                                            beam_details.append(f"{angle_deg:.0f}°:invalid")
                                                    beam_summary = " | ".join(beam_details)
                                                    print(f"      🔍 LiDAR Summary: [{beam_summary}]")
                                                else:
                                                    print(f"      🔍 LiDAR Summary: {valid_points}/{total_rays} rays valid, "
                                                          f"avg distance: {avg_distance:.2f}m")
                                            else:
                                                print(f"      🔍 LiDAR Summary: No obstacles detected in final scan ({total_rays} rays)")
                                        else:
                                            print("      🔍 LiDAR Summary: No scan data available")
                                    except Exception as e:
                                        print(f"      🔍 LiDAR Summary: Error retrieving final scan ({str(e)[:30]}...)")
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
                print("   ❌ Failed to assign task - robot busy or no more shelves")
                break
            
            time.sleep(1.0)  # Brief pause between tasks
            task_cycle += 1
        
        print(f"\n🎉 Completed {task_cycle - 1} PICK_AND_DELIVER tasks successfully!")
        print("   📦 Full lifecycle: SHELF → DROP-OFF → IDLE ZONE")

        # KPI Overview (Phase 5)
        try:
            export_enabled = sim.config_provider.get_value("kpi.export_csv_enabled", True).value
            export_path = sim.config_provider.get_value("kpi.export_csv_path", "kpi_results.csv").value
        except Exception:
            export_enabled = True
            export_path = "kpi_results.csv"

        try:
            overview = sim.simulation_data_service.get_kpi_overview()
            print("\n📊 KPI SUMMARY:")
            print(f"   ✅ Tasks Completed: {overview.get('tasks_completed', 0)}")
            print(f"   ⏱️  Avg Task Time: {overview.get('avg_task_time_seconds', 0.0):.2f}s")
            print(f"   📈 Success Rate: {overview.get('task_success_rate', 0.0):.1f}%")
            print(f"   🧩 PD Completed: {overview.get('pd_tasks_completed', 0)} | PD Failed: {overview.get('pd_tasks_failed', 0)} | PD Success: {overview.get('pd_success_rate', 0):.1f}% | PD Avg Time: {overview.get('pd_avg_task_time_seconds', 0):.1f}s")
            if export_enabled:
                if sim.simulation_data_service.export_kpi_overview_csv(overview, export_path):
                    print(f"   💾 KPI CSV exported: {export_path}")
                else:
                    print("   ⚠️  KPI CSV export failed (see logs)")
        except Exception as e:
            print(f"   ⚠️  KPI overview unavailable: {e}")
        
    except KeyboardInterrupt:
        print("\n⚠️  Simulation interrupted by user")
    
    finally:
        sim.stop_simulation()


def main():
    """Main demo function - PICK_AND_DELIVER robot task simulation."""
    print("🤖 Robot PICK_AND_DELIVER Task Simulation Demo")
    print("=" * 50)
    print("Complete autonomous warehouse robot simulation")
    print("Demonstrating full task lifecycle:")
    print("   📍 Navigate to Shelf")
    print("   🤖 Pick Item")
    print("   🚚 Navigate to Drop-off")
    print("   📦 Drop Item")
    print("   🏠 Navigate to Idle Zone")
    print()
    
    # Enable motion executor verbose logging for debugging
    from robot.impl.motion_executor_impl import MotionExecutorImpl
    MotionExecutorImpl.set_verbose_logging(False)
    
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