#!/usr/bin/env python3
"""
Multi-Robot Warehouse Simulation Demo - Professional Edition with Order Processing

Demonstrates coordinated multi-robot operation with:
- Multiple robot agents working simultaneously
- Robot controller managing task distribution
- Conflict box coordination for safe navigation
- Real-time multi-robot visualization
- Comprehensive error handling and monitoring
- SOLID architecture principles maintained
- Integrated order processing from sample_orders.json

⚠️  DEMO CONFIGURATION: Lane tolerance set to 3.0m (too soft for production!)
    This allows tasks to complete despite navigation issues. For production,
    use 0.1-0.3m tolerance for safe warehouse operation.

Run with: python demo_multi_robot_orders_simulation.py
"""
import sys
import os
import time
import threading
from typing import List, Optional, Tuple
import argparse
from unittest.mock import MagicMock
import logging
from dataclasses import dataclass

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
from warehouse.impl.json_order_source import JsonOrderSource
from warehouse.impl.jobs_processor_impl import JobsProcessorImpl, JobsProcessorError
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
        
        # Validate robot coun
        if robot_count < 1 or robot_count > 15:
            raise ValueError("Robot count must be between 1 and 15 for demo purposes")
        
        self.robot_count = robot_count
        self.robots: List[RobotInstance] = []
        
        # Load warehouse
        self.warehouse_map = WarehouseMap(csv_file=warehouse_csv)
        print(f"   ✅ Warehouse loaded: {self.warehouse_map.width}x{self.warehouse_map.height}")

        # Create real configuration provider
        self.config_provider = ConfigurationProvider()
        print("   ✅ Configuration provider initialized")

        # Override random placement to use manual placement
        self.config_provider.set_value("demo.random_robot_placement", True)
        print("   ✅ Random robot placement disabled (manual placement enabled)")

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
        self.bidding_system = TransparentBiddingSystem(config_provider=self.config_provider)

        # Initialize order processing pipeline
        try:
            self.order_source = JsonOrderSource("sample_orders.json")
            self.order_source.connect()
            print("   ✅ Order source initialized and connected to sample_orders.json")

            self.jobs_processor = JobsProcessorImpl(
                order_source=self.order_source,
                simulation_data_service=self.simulation_data_service,
                jobs_queue=self.jobs_queue,
                config_provider=self.config_provider
            )
            print("   ✅ Jobs processor initialized with order processing pipeline")
        except Exception as e:
            print(f"   ❌ Failed to initialize order processing pipeline: {e}")
            logging.error(f"EXCEPTION: Order processing pipeline initialization failed: {e}", exc_info=True)
            raise

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
        
        # Task management (now handled by order processor)
        self.tasks_created = 0
        
        print("🎯 Simulation ready!")

    def _find_random_walkable_positions(self, count: int) -> List[Tuple[int, int]]:
        """
        Find random walkable positions that avoid conflict boxes.

        Args:
            count: Number of positions to find

        Returns:
            List of (grid_x, grid_y) tuples for walkable positions
        """
        import random
        from interfaces.navigation_types import Point

        # Get all walkable cells (grid coordinates)
        walkable_cells = []
        for grid_y in range(self.warehouse_map.height):
            for grid_x in range(self.warehouse_map.width):
                # Check if cell is walkable (free space, charging, idle, or drop-off)
                if self.warehouse_map.grid[grid_y, grid_x] in [0, 3, 4, 5]:
                    # Convert to world coordinates for conflict box checking
                    world_pos = self.warehouse_map.grid_to_world(grid_x, grid_y)
                    walkable_cells.append((grid_x, grid_y, world_pos[0], world_pos[1]))

        # Get conflict boxes from database
        conflict_boxes = []
        try:
            conflict_boxes = self.simulation_data_service.get_conflict_boxes()
        except Exception as e:
            print(f"   ⚠️  Could not load conflict boxes, proceeding without conflict box avoidance: {e}")

        # Filter out cells that are in conflict boxes
        filtered_cells = []

        for grid_x, grid_y, world_x, world_y in walkable_cells:
            cell_safe = True

            # Check if cell is in any conflict box
            for box in conflict_boxes:
                distance = ((world_x - box.center.x) ** 2 + (world_y - box.center.y) ** 2) ** 0.5
                if distance <= (box.size / 2.0):  # Add buffer distance
                    cell_safe = False
                    break

            if cell_safe:
                filtered_cells.append((grid_x, grid_y))

        # Randomly select positions
        if len(filtered_cells) < count:
            print(f"   ⚠️  Only {len(filtered_cells)} safe walkable cells available, using all of them")
            return filtered_cells

        return random.sample(filtered_cells, count)

    def _register_robots_with_engine(self) -> None:
        """Register all robots with shared engine before initialization."""
        print("   🤖 Registering robots with shared engine...")

        # Check if random placement is enabled
        random_placement_config = self.config_provider.get_value("demo.random_robot_placement", True)
        random_placement = random_placement_config.value if hasattr(random_placement_config, 'value') else random_placement_config

        if random_placement:
            print("   🎲 Using random robot placement...")
            # Find random walkable positions
            random_positions = self._find_random_walkable_positions(self.robot_count)

            for i in range(self.robot_count):
                robot_id = f"warehouse_robot_{i+1}"
                if i < len(random_positions):
                    grid_x, grid_y = random_positions[i]
                    world_pos = self.warehouse_map.grid_to_world(grid_x, grid_y)
                    print(f"      🎯 {robot_id} -> Grid ({grid_x}, {grid_y}) -> World ({world_pos[0]:.2f}, {world_pos[1]:.2f})")
                    self.shared_engine.register_robot(robot_id, (world_pos[0], world_pos[1], 0.0))
                else:
                    print(f"      ⚠️  No safe position found for {robot_id}, skipping")
        else:
            print("   📍 Using manual robot placement...")
            # Define starting positions for robots (specific coordinates)
            start_positions = [
                (4.25,3.25),   # Robot 1: Cell (2, 3)
                (4.25, 3.75),   # Robot 2: Cell (4, 1)
                (4.25, 2.75),     # Robot 3: Default position
                (4.25, 1.75),     # Robot 4: Default position
                (4.25, 2.25),     # Robot 5: Default position
            ]

            for i in range(self.robot_count):
                robot_id = f"warehouse_robot_{i+1}"
                if i < len(start_positions):
                    start_x, start_y = start_positions[i]
                    print(f"      📍 {robot_id} -> World ({start_x:.2f}, {start_y:.2f})")
                else:
                    # Fallback for additional robots
                    start_x, start_y = 1.0, float(i)
                    print(f"      📍 {robot_id} -> Fallback ({start_x:.2f}, {start_y:.2f})")

                # Register robot with shared engine
                self.shared_engine.register_robot(robot_id, (start_x, start_y, 0.0))

        print(f"   ✅ Registered {self.robot_count} robots with shared engine")

    def _create_robot_agents(self) -> None:
        """Create multiple robot agents with different starting positions."""
        print("   🤖 Creating robot agents...")

        # Get positions from shared engine (already registered)
        random_placement_config = self.config_provider.get_value("demo.random_robot_placement", True)
        random_placement = random_placement_config.value if hasattr(random_placement_config, 'value') else random_placement_config

        for i in range(self.robot_count):
            robot_id = f"warehouse_robot_{i+1}"

            # Get starting position from shared engine
            try:
                start_pose = self.shared_engine.get_pose(robot_id)
                start_x, start_y = start_pose[0], start_pose[1]
                grid_pos = self.warehouse_map.world_to_grid(start_x, start_y)
                start_grid_x, start_grid_y = int(grid_pos[0]), int(grid_pos[1])
            except Exception as e:
                print(f"      ⚠️  Could not get position for {robot_id}: {e}")
                # Fallback to manual positions if registration fails
                fallback_positions = [
                    (2.75, 1.25),   # Robot 1: Cell (5, 2)
                    (2.25, 0.75),   # Robot 2: Cell (4, 1)
                    (1.0, 1.0),     # Robot 3: Default position
                    (1.0, 2.0),     # Robot 4: Default position
                    (1.0, 3.0),     # Robot 5: Default position
                ]
                if i < len(fallback_positions):
                    start_x, start_y = fallback_positions[i]
                else:
                    start_x, start_y = 1.0, float(i)

            # Get base configuration from provider and customize for multi-robot demo
            base_config = self.config_provider.get_robot_config(robot_id)

            # Create robot configuration with demo-specific overrides
            # Match single-robot motion parameters for stability
            robot_config = RobotConfig(
                robot_id=robot_id,
                # Use provider values for physical dimensions and ratios
                robot_width=base_config.robot_width,
                robot_height=base_config.robot_height,
                robot_length=base_config.robot_length,
                wheel_base_ratio=base_config.wheel_base_ratio,
                wheel_radius_ratio=base_config.wheel_radius_ratio,
                # Demo-specific motion parameters (reduced for stability)
                max_speed=0.5,  # Match single-robot stability settings
                position_tolerance=base_config.position_tolerance,
                control_frequency=base_config.control_frequency,
                motion_frequency=base_config.motion_frequency,
                cell_size=base_config.cell_size,
                # Demo-specific navigation parameters (softer for demo)
                lane_tolerance=0.5,  # DEMO ONLY: very soft tolerance
                corner_speed=base_config.corner_speed,
                bay_approach_speed=base_config.bay_approach_speed,
                # Use provider values for coordination
                conflict_box_lock_timeout=base_config.conflict_box_lock_timeout,
                conflict_box_heartbeat_interval=base_config.conflict_box_heartbeat_interval,
                # Demo-specific motion parameters (reduced for stability)
                max_linear_velocity=0.5,  # Match single-robot stability settings
                max_angular_velocity=0.5,  # Match single-robot stability settings
                movement_speed=0.5,  # Match single-robot stability settings
                # Use provider values for timing
                picking_duration=base_config.picking_duration,
                dropping_duration=base_config.dropping_duration,
                emergency_stop_distance=base_config.emergency_stop_distance,
                stall_recovery_timeout=base_config.stall_recovery_timeout,
                # Use provider LiDAR config
                lidar_config=base_config.lidar_config
            )

            # Create robot physics (shared engine already initialized)
            robot_physics = SharedMuJoCoPhysics(self.shared_engine, robot_id=robot_id)
            # Tag physics with robot_id for proper identification
            try:
                robot_physics.robot_id = robot_id
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
                    lane_tolerance=robot_config.lane_tolerance,  # Uses the 3.0m demo tolerance set above
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
                # Add missing items from sample_orders.json
                {'item_id': 'book_001', 'name': 'Programming Book', 'shelf_id': 'shelf_3_3', 'quantity': 15},
                {'item_id': 'audio_001', 'name': 'Audio Equipment', 'shelf_id': 'shelf_4_6', 'quantity': 6},
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
            try:
                # Get current position from shared engine
                current_pose = self.shared_engine.get_pose(robot_instance.robot_id)
                start_x, start_y = current_pose[0], current_pose[1]

                robot_instance.robot.physics.reset_robot_position(start_x, start_y, 0.0)
                grid_pos = self.warehouse_map.world_to_grid(start_x, start_y)
                grid_x, grid_y = int(grid_pos[0]), int(grid_pos[1])
                print(f"   ✅ Positioned {robot_instance.robot_id} at grid ({grid_x}, {grid_y}) -> world ({start_x:.2f}, {start_y:.2f})")

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
            robot_configs = {r.robot_id: r.config for r in self.robots}
            self.visualization = MultiRobotMujocoVisualization(state_holders, self.warehouse_map, robot_configs)
            self.visualization.initialize()
            self.visualization_thread = VisualizationThread(self.visualization, fps=30.0)
            self.visualization_thread.start()
            print("   ✅ Visualization thread running at 30 FPS")
        except Exception as e:
            print(f"   ⚠️ Visualization not started: {e}")

        # Start order processor (must start before robot controller for task availability)
        try:
            self.jobs_processor.start_processing()
            print("   ✅ Order processor started and polling for orders")
        except JobsProcessorError as e:
            if "already running" in str(e):
                print("   ⚠️  Order processor already running, continuing...")
                logging.warning(f"Order processor start skipped: {e}")
            else:
                print(f"   ❌ Failed to start order processor: {e}")
                logging.error(f"EXCEPTION: Order processor startup failed: {e}", exc_info=True)
                raise
        except Exception as e:
            print(f"   ❌ Failed to start order processor: {e}")
            logging.error(f"EXCEPTION: Order processor startup failed: {e}", exc_info=True)
            raise

        # Start robot controller
        try:
            self.robot_controller.start()
            print("   ✅ Robot controller started")
        except Exception as e:
            print(f"   ❌ Failed to start robot controller: {e}")
            logging.error(f"EXCEPTION: Robot controller startup failed: {e}", exc_info=True)
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
        
        # Best-effort: clean up charging/idle bay locks for all robots before stopping threads
        try:
            for robot_instance in self.robots:
                try:
                    handler = getattr(robot_instance.robot, 'task_handler', None)
                    if handler is None:
                        continue
                    locked_id = getattr(handler, '_locked_bay_id', None)
                    if not locked_id:
                        continue
                    # Route releases based on lock type to keep caches consistent
                    if locked_id.startswith('charge_'):
                        # Prefer manager when available for cache invalidation
                        manager = getattr(handler, 'charging_station_manager', None)
                        if manager is not None:
                            try:
                                manager.release_charging_station(locked_id, robot_instance.robot_id)
                            except Exception:
                                # Fall back to direct release if manager path fails
                                try:
                                    handler.simulation_data_service.release_bay_lock(locked_id, robot_instance.robot_id)
                                except Exception:
                                    pass
                        else:
                            try:
                                handler.simulation_data_service.release_bay_lock(locked_id, robot_instance.robot_id)
                            except Exception:
                                pass
                    else:
                        # Idle bay or other bay lock
                        try:
                            handler.simulation_data_service.release_bay_lock(locked_id, robot_instance.robot_id)
                        except Exception:
                            pass
                    # Clear handler reference so status reflects unlocked state
                    try:
                        handler._locked_bay_id = None
                    except Exception:
                        pass
                except Exception:
                    # Never fail shutdown due to cleanup attempts
                    pass
        except Exception:
            # Continue with shutdown regardless
            pass

        # Stop order processor first (graceful shutdown)
        try:
            self.jobs_processor.stop_processing()
            print("   ✅ Order processor stopped")
        except Exception as e:
            print(f"   ⚠️  Error stopping order processor: {e}")
            logging.error(f"EXCEPTION: Order processor shutdown failed: {e}", exc_info=True)

        # Stop robot controller
        try:
            self.robot_controller.stop()
            print("   ✅ Robot controller stopped")
        except Exception as e:
            print(f"   ⚠️  Error stopping robot controller: {e}")
            logging.error(f"EXCEPTION: Robot controller shutdown failed: {e}", exc_info=True)
        
        # Stop all robot agents
        for robot_instance in self.robots:
            try:
                robot_instance.robot.stop()
                robot_instance.status = "stopped"
                print(f"   ✅ Stopped {robot_instance.robot_id}")
            except Exception as e:
                robot_instance.status = "error"
                print(f"   ⚠️  Error stopping {robot_instance.robot_id}: {e}")
                logging.error(f"EXCEPTION: Robot {robot_instance.robot_id} shutdown failed: {e}", exc_info=True)
        
        # Stop visualization
        try:
            if self.visualization_thread is not None:
                self.visualization_thread.stop()
                self.visualization_thread = None
            if self.visualization is not None:
                self.visualization.shutdown()
        except Exception as e:
            logging.error(f"EXCEPTION: Visualization shutdown failed: {e}", exc_info=True)

        # Disconnect order source
        try:
            if hasattr(self, 'order_source') and self.order_source.is_connected():
                self.order_source.disconnect()
                print("   ✅ Order source disconnected")
        except Exception as e:
            logging.error(f"EXCEPTION: Order source disconnect failed: {e}", exc_info=True)
        
        # KPI Overview (Phase 5)
        try:
            cfg = self.config_provider
            export_enabled = cfg.get_value("kpi.export_csv_enabled", True).value
            export_path = cfg.get_value("kpi.export_csv_path", "kpi_results.csv").value
        except Exception:
            export_enabled = True
            export_path = "kpi_results.csv"

        try:
            overview = self.simulation_data_service.get_kpi_overview()
            print("\n📊 KPI SUMMARY:")
            print(f"   ✅ Tasks Completed: {overview.get('tasks_completed', 0)}")
            print(f"   ❌ Tasks Failed: {overview.get('tasks_failed', 0)}")
            print(f"   📈 Success Rate: {overview.get('task_success_rate', 0):.1f}%")
            print(f"   ⏱️  Avg Task Time: {overview.get('avg_task_time_seconds', 0):.1f}s")
            print(f"   🧩 PD Completed: {overview.get('pd_tasks_completed', 0)} | PD Failed: {overview.get('pd_tasks_failed', 0)} | PD Success: {overview.get('pd_success_rate', 0):.1f}% | PD Avg Time: {overview.get('pd_avg_task_time_seconds', 0):.1f}s")
            print(f"   💼 Busy Time: {overview.get('busy_time_seconds', 0):.1f}s")
            print(f"   📊 Avg Busy % per Robot: {overview.get('avg_busy_percent_per_robot', 0):.1f}%")
            print(f"   🆓 Avg Idle % per Robot: {overview.get('avg_idle_percent_per_robot', 0):.1f}%")

            if export_enabled:
                if self.simulation_data_service.export_kpi_overview_csv(overview, export_path):
                    print(f"   💾 KPI CSV exported: {export_path}")
                else:
                    print(f"   ⚠️  KPI CSV export failed: {export_path}")
        except Exception as e:
            print(f"   ⚠️  KPI summary unavailable: {e}")

        print("   ✅ Multi-robot simulation stopped cleanly")
    
    def create_demo_tasks(self, task_count: int) -> None:
        """
        Process orders from sample_orders.json file through the complete order processing pipeline.

        This replaces hardcoded task creation with proper order processing that includes:
        - Order loading from JSON file
        - Inventory validation and shelf lookup
        - Proper task creation with metadata
        - Order status tracking
        """
        print(f"   📋 Processing orders from sample_orders.json through complete pipeline...")

        try:
            # Check if processor is already running
            if self.jobs_processor.is_processing_started():
                print("   ℹ️  Order processor already started, checking current status...")
            else:
                # Start the order processing pipeline
                self.jobs_processor.start_processing()
                print("   ✅ Order processor started and polling for orders")

            # Allow time for order processing to create tasks
            time.sleep(3.0)

            # Check processing results
            processing_stats = self.jobs_processor.get_processing_stats()
            tasks_queued = self.jobs_queue.get_queue_size()

            print(f"   📊 Order Processing Results:")
            print(f"      Orders processed: {processing_stats.orders_processed}")
            print(f"      Tasks created: {processing_stats.tasks_created}")
            print(f"      Tasks queued: {tasks_queued}")
            print(f"      Processor state: {self.jobs_processor.get_processing_state()}")

            if processing_stats.tasks_created > 0:
                print(f"   ✅ Successfully processed orders into {processing_stats.tasks_created} tasks")
                self.tasks_created = processing_stats.tasks_created
            else:
                print("   ⚠️  No tasks were created from orders - check inventory and order format")

        except JobsProcessorError as e:
            if "already running" in str(e):
                print("   ⚠️  Order processor already running from simulation startup")
                logging.info(f"Order processor start skipped: {e}")
            else:
                print(f"   ❌ Failed to process orders: {e}")
                logging.error(f"EXCEPTION: Order processing failed: {e}", exc_info=True)
                raise
        except Exception as e:
            print(f"   ❌ Failed to process orders: {e}")
            logging.error(f"EXCEPTION: Order processing failed: {e}", exc_info=True)
            raise
    
    
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
                            
                            # Get battery level from robot data
                            battery_level = robot_data.get('battery_level', 0.0)

                            # Get conflict box status
                            acquired_boxes = robot_data.get('acquired_conflict_boxes', [])
                            pending_box = robot_data.get('pending_conflict_box')

                            # Get charging station lock status
                            locked_bay_id = robot_data.get('locked_bay_id')

                            # Format conflict box info
                            if acquired_boxes:
                                conflict_info = f"Locks: {acquired_boxes}"
                            else:
                                conflict_info = "Locks: None"

                            if pending_box:
                                conflict_info += f" | Pending: {pending_box}"
                            else:
                                conflict_info += " | Pending: None"

                            # Add charging station lock info
                            if locked_bay_id:
                                conflict_info += f" | Charging: {locked_bay_id}"
                            else:
                                conflict_info += " | Charging: None"

                            print(f"   🤖 {robot_id}: {operational_status} | "
                                  f"Progress: {progress:.1%} | "
                                  f"Position: ({position[0]:.2f}, {position[1]:.2f}) | "
                                  f"Battery: {battery_level:.1%} | "
                                  f"{conflict_info}")
                        except Exception as e:
                            print(f"   🤖 {robot_id}: Error getting status - {e}")
                    else:
                        print(f"   🤖 {robot_id}: {robot_status}")
                
                # Controller status
                controller = status['controller']
                print(f"   🎮 Controller: {controller['total_rounds_processed']} rounds, "
                      f"{controller['total_tasks_assigned']} tasks assigned")
                
                # Order processing status
                try:
                    processor_stats = self.jobs_processor.get_processing_stats()
                    print(f"   📋 Orders: {processor_stats.orders_processed} processed, "
                          f"{processor_stats.tasks_created} tasks created")
                except Exception as e:
                    logging.error(f"EXCEPTION: Failed to get processor stats: {e}", exc_info=True)
                    print("   📋 Orders: Status unavailable")

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
            'tasks_created': self.tasks_created
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
        
        # Start order processing (orders will be processed from sample_orders.json)
        print("\n📋 Starting order processing from sample_orders.json...")
        sim.create_demo_tasks(0)  # This now starts the order processor, parameter ignored
        
        # Monitor coordination for a reasonable duration
        monitoring_duration = 120.0  # Extended time for order processing
        sim.monitor_robot_coordination(monitoring_duration)
        
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
            # Order processing is now handled automatically by the order processor
            # Orders will be loaded from sample_orders.json and processed into tasks
            print(f"\n📋 Order processing will load orders from sample_orders.json automatically")
            # Extended monitoring for order processing and task execution
            sim.monitor_robot_coordination(180.0)  # Extended time for full order processing
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