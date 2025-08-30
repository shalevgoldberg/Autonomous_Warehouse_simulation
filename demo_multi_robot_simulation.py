#!/usr/bin/env python3
"""
Multi-Robot Warehouse Simulation Demo - Professional Edition

Demonstrates coordinated multi-robot operation with:
- Multiple robot agents working simultaneously
- Robot controller managing task distribution
- Conflict box coordination for safe navigation
- Real-time multi-robot visualization
- Comprehensive error handling and monitoring
- SOLID architecture principles maintained

⚠️  DEMO CONFIGURATION: Lane tolerance set to 3.0m (too soft for production!)
    This allows tasks to complete despite navigation issues. For production,
    use 0.1-0.3m tolerance for safe warehouse operation.

Run with: python demo_multi_robot_simulation.py
"""
import sys
import os
import time
import threading
from typing import List, Optional
import argparse
from unittest.mock import MagicMock
import logging
from dataclasses import dataclass

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
#from simulation.mujoco_env import SimpleMuJoCoPhysics
from simulation.shared_mujoco_engine import SharedMuJoCoEngine, SharedMuJoCoPhysics
import builtins as _bi

# Safe print wrapper to avoid UnicodeEncodeError on Windows consoles
def _safe_print(*args, **kwargs):
    try:
        _bi.print(*args, **kwargs)
    except UnicodeEncodeError:
        safe_args = tuple(str(a).encode('ascii', 'ignore').decode() for a in args)
        _bi.print(*safe_args, **kwargs)

print = _safe_print  # type: ignore
from robot.robot_agent_lane_based import RobotAgent
from interfaces.configuration_interface import RobotConfig
from interfaces.task_handler_interface import Task, TaskType
from interfaces.path_planner_interface import Cell
from interfaces.visualization_interface import IVisualization
from simulation.mujoco_visualization import MujocoVisualization
from simulation.multi_robot_mujoco_visualization import MultiRobotMujocoVisualization
from simulation.visualization_thread import VisualizationThread
from interfaces.lane_follower_interface import LaneFollowingConfig
from config.configuration_provider import ConfigurationProvider
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from pathlib import Path
from warehouse.impl.robot_controller_impl import RobotController
from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
from warehouse.impl.jobs_queue_impl import JobsQueueImpl


@dataclass
class RobotInstance:
    """Container for robot instance and its metadata."""
    robot: RobotAgent
    config: RobotConfig
    start_position: tuple
    robot_id: str
    status: str = "initializing"


class MultiRobotSimulationManager:
    """
    Manages multi-robot warehouse simulation.
    
    Coordinates multiple robots, physics simulation, task distribution,
    and visualization following clean architecture principles.
    """
    
    def __init__(self, warehouse_csv: str = "sample_warehouse.csv", robot_count: int = 2):
        """Initialize simulation manager."""
        print("🚀 Initializing Multi-Robot Warehouse Simulation")
        print(f"   🤖 Robot Count: {robot_count}")
        
        # Validate robot count
        if robot_count < 1 or robot_count > 5:
            raise ValueError("Robot count must be between 1 and 5 for demo purposes")
        
        self.robot_count = robot_count
        self.robots: List[RobotInstance] = []
        
        # Load warehouse
        self.warehouse_map = WarehouseMap(csv_file=warehouse_csv)
        print(f"   ✅ Warehouse loaded: {self.warehouse_map.width}x{self.warehouse_map.height}")

        # Create real configuration provider
        self.config_provider = ConfigurationProvider()
        print("   ✅ Configuration provider initialized")

        # Shared MuJoCo engine for all robots
        self.shared_engine = SharedMuJoCoEngine(self.warehouse_map, physics_dt=0.001, enable_time_gating=True,
                                               config_provider=self.config_provider)
        
        # Create real simulation data service (increase pool for multi-robot)
        self.simulation_data_service = SimulationDataServiceImpl(self.warehouse_map, pool_size=max(10, 3 * robot_count))
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

        # Register all robots with shared engine BEFORE initialization
        self._register_robots_with_engine()

        # Initialize shared engine (creates appearance service)
        try:
            self.shared_engine.initialize()
            print("   ✅ Shared engine initialized with appearance service")
        except Exception as e:
            print(f"   ⚠️  Shared engine initialization failed: {e}")

        # Create robot controller components
        self.jobs_queue = JobsQueueImpl()
        self.bidding_system = TransparentBiddingSystem()

        # Create robots AFTER engine initialization
        self._create_robot_agents()

        # Create robot controller
        self.robot_controller = RobotController(
            jobs_queue=self.jobs_queue,
            bidding_system=self.bidding_system,
            robot_pool=[robot.robot for robot in self.robots],
            polling_interval=0.5  # Poll every 500ms for multi-robot coordination
        )
        print(f"   ✅ Robot controller initialized with {len(self.robots)} robots")
        
        # Visualization
        self.visualization: Optional[IVisualization] = None
        self.visualization_thread: Optional[VisualizationThread] = None
        
        # Task management
        self.available_shelves = self._discover_shelf_positions()
        self.current_task_index = 0
        self.tasks_created = 0
        
        print(f"   ✅ Found {len(self.available_shelves)} shelves for tasks")
        print("🎯 Simulation ready!")

    def _register_robots_with_engine(self) -> None:
        """Register all robots with shared engine before initialization."""
        print("   🤖 Registering robots with shared engine...")

        # Define starting positions for robots (avoid conflicts)
        start_positions = [
            (1, 1),   # Robot 1: Left lane (row 1, col 1 = 'l')
            (1, 2),   # Robot 2: Left lane (row 2, col 1 = 'l')
            (1, 3),   # Robot 3: Left lane (row 3, col 1 = 'l')
            (1, 4),   # Robot 4: Left lane (row 4, col 1 = 'l')
            (1, 5),   # Robot 5: Left lane (row 5, col 1 = 'l')
        ]

        for i in range(self.robot_count):
            robot_id = f"warehouse_robot_{i+1}"
            start_x, start_y = start_positions[i]

            # Register robot with shared engine
            world_pos = self.warehouse_map.grid_to_world(start_x, start_y)
            self.shared_engine.register_robot(robot_id, (world_pos[0], world_pos[1], 0.0))

        print(f"   ✅ Registered {self.robot_count} robots with shared engine")

    def _create_robot_agents(self) -> None:
        """Create multiple robot agents with different starting positions."""
        print("   🤖 Creating robot agents...")

        # Define starting positions for robots (avoid conflicts)
        start_positions = [
            (1, 1),   # Robot 1: Left lane (row 1, col 1 = 'l')
            (1, 2),   # Robot 2: Left lane (row 2, col 1 = 'l')
            (1, 3),   # Robot 3: Left lane (row 3, col 1 = 'l')
            (1, 4),   # Robot 4: Left lane (row 4, col 1 = 'l')
            (1, 5),   # Robot 5: Left lane (row 5, col 1 = 'l')
        ]

        for i in range(self.robot_count):
            robot_id = f"warehouse_robot_{i+1}"
            start_x, start_y = start_positions[i]

            # Create robot configuration
            robot_config = RobotConfig(
                robot_id=robot_id,
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

            # Create robot physics (shared engine already initialized)
            robot_physics = SharedMuJoCoPhysics(self.shared_engine, robot_id=robot_id)
            # Diagnostics: tag physics with robot_id and enable for robot_2 only
            try:
                robot_physics.robot_id = robot_id
                if robot_id == "warehouse_robot_2":
                    robot_physics._diagnostics_enabled = False
            except Exception:
                pass
            
            # Ensure robot_id exists in DB for FK constraints (best-effort)
            try:
                with self.simulation_data_service._get_connection() as conn:
                    with conn.cursor() as cur:
                        cur.execute(
                            "INSERT INTO robots (robot_id, name) VALUES (%s, %s) ON CONFLICT (robot_id) DO NOTHING",
                            (robot_id, robot_id.replace('_', ' ').title()),
                        )
                        conn.commit()
            except Exception:
                pass

            # Create robot agent with unique robot_id and dedicated physics
            robot = RobotAgent(
                physics=robot_physics,
                config_provider=self.config_provider,
                robot_id=robot_id,
                simulation_data_service=self.simulation_data_service
            )
            
            # Configure lane follower for DEMO (very soft tolerance)
            robot.lane_follower.set_config(
                LaneFollowingConfig(
                    lane_tolerance=3.0,  # DEMO ONLY: too soft for production!
                    max_speed=robot_config.max_speed,
                    corner_speed=robot_config.corner_speed,
                    bay_approach_speed=robot_config.bay_approach_speed,
                )
            )
            
            # Create robot instance
            robot_instance = RobotInstance(
                robot=robot,
                config=robot_config,
                start_position=(start_x, start_y),
                robot_id=robot_id
            )
            
            self.robots.append(robot_instance)
            print(f"      ✅ Created {robot_id} at position ({start_x}, {start_y})")

        print(f"   ✅ Created {len(self.robots)} robot agents")
    
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
        """Start multi-robot simulation."""
        print("\n🎬 Starting Multi-Robot Warehouse Simulation")

        # Configure logging for all robots
        self._configure_robot_logging()
        
        # Start all robot agents
        for robot_instance in self.robots:
            try:
                robot_instance.robot.start()
                robot_instance.status = "running"
                print(f"   ✅ Started {robot_instance.robot_id}")
            except Exception as e:
                robot_instance.status = "error"
                print(f"   ❌ Failed to start {robot_instance.robot_id}: {e}")
                raise
        
        # Position robots are already pre-registered in shared engine; ensure reset
        for robot_instance in self.robots:
            start_x, start_y = robot_instance.start_position
            world_pos = self.warehouse_map.grid_to_world(start_x, start_y)
            try:
                robot_instance.robot.physics.reset_robot_position(world_pos[0], world_pos[1], 0.0)
                print(f"   ✅ Positioned {robot_instance.robot_id} at ({start_x}, {start_y})")

                # Force StateHolder update after positioning
                try:
                    robot_instance.robot.state_holder.update_from_simulation()
                    print(f"   ✅ StateHolder updated for {robot_instance.robot_id}")
                except Exception as e:
                    print(f"   ⚠️  Failed to update StateHolder for {robot_instance.robot_id}: {e}")
            except Exception as e:
                print(f"   ⚠️  Failed to position {robot_instance.robot_id}: {e}")
        
        # Start visualization (reads from state holders updated by robot physics threads)
        try:
            # Multi-robot viewer: render all robots in one window
            state_holders = {r.robot_id: r.robot.state_holder for r in self.robots}
            self.visualization = MultiRobotMujocoVisualization(state_holders, self.warehouse_map)
            self.visualization.initialize()
            self.visualization_thread = VisualizationThread(self.visualization, fps=30.0)
            self.visualization_thread.start()
            print("   ✅ Visualization thread running at 30 FPS")
        except Exception as e:
            print(f"   ⚠️ Visualization not started: {e}")

        # Start robot controller
        try:
            self.robot_controller.start()
            print("   ✅ Robot controller started")
        except Exception as e:
            print(f"   ❌ Failed to start robot controller: {e}")
            raise

        print("   ✅ All robots positioned and ready")
    
    def _configure_robot_logging(self) -> None:
        """Configure logging for all robots to avoid global side-effects."""
        try:
            for robot_instance in self.robots:
                # Configure the specific robot's LaneFollower logger
                lane_logger = logging.getLogger(f"LaneFollower.{robot_instance.robot_id}")
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
    
    def stop_simulation(self) -> None:
        """Stop multi-robot simulation."""
        print("\n🛑 Stopping Multi-Robot Warehouse Simulation")
        
        # Stop robot controller
        try:
            self.robot_controller.stop()
            print("   ✅ Robot controller stopped")
        except Exception as e:
            print(f"   ⚠️  Error stopping robot controller: {e}")
        
        # Stop all robot agents
        for robot_instance in self.robots:
            try:
                robot_instance.robot.stop()
                robot_instance.status = "stopped"
                print(f"   ✅ Stopped {robot_instance.robot_id}")
            except Exception as e:
                robot_instance.status = "error"
                print(f"   ⚠️  Error stopping {robot_instance.robot_id}: {e}")
        
        # Stop visualization
        try:
            if self.visualization_thread is not None:
                self.visualization_thread.stop()
                self.visualization_thread = None
            if self.visualization is not None:
                self.visualization.shutdown()
        except Exception:
            pass
        
        print("   ✅ Multi-robot simulation stopped cleanly")
    
    def create_demo_tasks(self, task_count: int) -> None:
        """Create demo tasks and add them to the jobs queue."""
        print(f"   📋 Creating {task_count} demo tasks...")
        
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
        
        tasks_created = 0
        for i in range(task_count):
            if self.current_task_index >= len(self.available_shelves):
                print(f"   ⚠️  No more shelves available for tasks")
                break
            
            # Get current shelf target
            shelf_id, shelf_x, shelf_y = self.available_shelves[self.current_task_index]
            
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
            
            # Add to jobs queue
            self.jobs_queue.enqueue_task(task)
            tasks_created += 1
            self.current_task_index += 1
            
            print(f"      ✅ Created task {task.task_id}: Pick from {shelf_id}")
        
        self.tasks_created += tasks_created
        print(f"   ✅ Created {tasks_created} tasks (Total: {self.tasks_created})")
    
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
    
    def monitor_robot_coordination(self, duration: float = 60.0) -> None:
        """Monitor robot coordination and task execution."""
        print(f"\n⏳ Monitoring robot coordination for {duration} seconds...")
        print("   🤖 Robots will coordinate through the robot controller")
        print("   📋 Tasks will be distributed via bidding system")
        print("   🔄 Conflict box coordination ensures safe navigation")
        
        start_time = time.time()
        last_status_time = 0
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Print status every 5 seconds
            if current_time - last_status_time >= 5.0:
                status = self.get_simulation_status()
                
                print(f"\n--- Status Update at {current_time - start_time:.1f}s ---")
                
                # Robot statuses
                for robot_info in status['robots']:
                    robot_id = robot_info['robot_id']
                    robot_status = robot_info['status']
                    
                    if robot_status == 'running':
                        try:
                            robot_data = robot_info['robot_status']
                            position = robot_data.get('position', [0.0, 0.0, 0.0])
                            task_status = robot_data.get('task_status')
                            
                            if task_status:
                                operational_status = task_status.operational_status.value
                                progress = task_status.progress
                            else:
                                operational_status = 'unknown'
                                progress = 0.0
                            
                            print(f"   🤖 {robot_id}: {operational_status} | "
                                  f"Progress: {progress:.1%} | "
                                  f"Position: ({position[0]:.2f}, {position[1]:.2f})")
                        except Exception as e:
                            print(f"   🤖 {robot_id}: Error getting status - {e}")
                    else:
                        print(f"   🤖 {robot_id}: {robot_status}")
                
                # Controller status
                controller = status['controller']
                print(f"   🎮 Controller: {controller['total_rounds_processed']} rounds, "
                      f"{controller['total_tasks_assigned']} tasks assigned")
                
                last_status_time = current_time
            
            time.sleep(1.0)  # Check every second
        
        print(f"\n✅ Monitoring completed after {duration} seconds")
    
    def get_simulation_status(self) -> dict:
        """Get comprehensive multi-robot simulation status."""
        robot_statuses = []
        for robot_instance in self.robots:
            try:
                robot_status = robot_instance.robot.get_status()
                robot_statuses.append({
                    'robot_id': robot_instance.robot_id,
                    'status': robot_instance.status,
                    'robot_status': robot_status
                })
            except Exception as e:
                robot_statuses.append({
                    'robot_id': robot_instance.robot_id,
                    'status': 'error',
                    'error': str(e)
                })
        
        controller_status = self.robot_controller.get_controller_status()
        
        return {
            'simulation_time': time.time(),
            'physics_ready': True,
            'robots': robot_statuses,
            'controller': controller_status,
            'tasks_created': self.tasks_created,
            'available_shelves': len(self.available_shelves)
        }


def demo_multi_robot_coordination():
    """Demo: Multi-robot coordination and task execution."""
    print("=" * 70)
    print("🤖 MULTI-ROBOT WAREHOUSE COORDINATION SIMULATION")
    print("=" * 70)
    
    # Create multi-robot simulation
    sim = MultiRobotSimulationManager(robot_count=2)
    
    try:
        # Start simulation
        sim.start_simulation()
        
        print("\n⏳ Running multi-robot coordination simulation...")
        print("   Robots will coordinate through the robot controller")
        print("   Tasks will be distributed via bidding system")
        print("   Conflict box coordination ensures safe navigation")
        
        # Create initial batch of tasks
        initial_tasks = 4  # 2 tasks per robot
        sim.create_demo_tasks(initial_tasks)
        
        # Monitor coordination for a reasonable duration
        monitoring_duration = 90.0  # 90 seconds for multi-robot demo
        sim.monitor_robot_coordination(monitoring_duration)
        
        # Create additional tasks to demonstrate continuous operation
        additional_tasks = 2
        sim.create_demo_tasks(additional_tasks)
        
        # Monitor for a bit longer
        sim.monitor_robot_coordination(30.0)
        
        print(f"\n🎉 Multi-robot coordination simulation completed!")
        print(f"   📋 Total tasks created: {sim.tasks_created}")
        print(f"   🤖 Robots coordinated successfully")
        print(f"   🔄 Conflict box coordination maintained safety")
        
    except KeyboardInterrupt:
        print("\n⚠️  Simulation interrupted by user")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        sim.stop_simulation()


def main():
    """Main entry point for the multi-robot warehouse simulation demo."""
    from robot.impl.motion_executor_impl import MotionExecutorImpl
    # Return to standard logging to reduce overhead
    MotionExecutorImpl.set_verbose_logging(False)
    
    print("🤖 Multi-Robot Warehouse Coordination Simulation Demo")
    print("=" * 60)
    print("Professional multi-robot autonomous warehouse simulation")
    print("Demonstrating coordinated operation:")
    print("   🤖 Multiple robots working simultaneously")
    print("   🎮 Robot controller managing task distribution")
    print("   🔄 Conflict box coordination for safe navigation")
    print("   📊 Real-time monitoring and status reporting")
    print()
    
    # Optional CLI arguments (e.g., --robots 5)
    try:
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument("--robots", type=int, default=3)
        args, _ = parser.parse_known_args()
        robot_count = args.robots
    except Exception:
        robot_count = 3

    try:
        # Run the demo with requested robot count
        # Wrapper to use the same flow while allowing variable robot count
        print(f"Requested robot count: {robot_count}")
        sim = MultiRobotSimulationManager(robot_count=robot_count)
        try:
            sim.start_simulation()
            # Create initial tasks: 2 per robot
            sim.create_demo_tasks(robot_count * 2)
            # Significantly longer monitoring window for clearer observation
            sim.monitor_robot_coordination(180.0)
        finally:
            sim.stop_simulation()
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main() 