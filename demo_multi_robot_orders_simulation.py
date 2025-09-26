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
- Integrated order processing from configurable JSON orders file
- Support for different warehouse layouts via CSV configuration

CLI ARGUMENTS:
    --robots N              Number of robots to simulate
    --placement MODE        Robot placement: 'random' or 'manual' (default: random)
    --orders FILE           JSON orders file to process (default: sample_orders.json)

WAREHOUSE MANAGEMENT:
    Warehouse data is automatically managed. If no warehouse data exists in the database,
    the demo will automatically detect this and run load_new_warehouse.py to generate it.

EXAMPLES:
    python demo_multi_robot_orders_simulation.py                    # Auto-detect and manage warehouse
    python demo_multi_robot_orders_simulation.py --robots 5 --placement manual
    python demo_multi_robot_orders_simulation.py --orders my_orders.json

    # To load a custom warehouse first (if needed):
    python load_new_warehouse.py --warehouse custom_layout.csv
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
import hashlib

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
from warehouse.impl.robot_registry_impl import RobotRegistryImpl
from interfaces.robot_registry_interface import RobotRuntimeState, RobotIdentity


@dataclass
class RobotInstance:
    """Container for robot instance and its metadata."""
    robot: RobotAgent
    config: RobotConfig
    start_position: tuple
    robot_id: str
    status: str = "initializing"


class WarehouseDataMissingError(RuntimeError):
    """Exception raised when warehouse data is missing from database."""
    pass

def _calculate_file_hash(file_path: str) -> str:
    """Calculate SHA256 hash of a file for change detection."""
    import hashlib
    try:
        with open(file_path, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()
    except Exception:
        return ""


class MultiRobotSimulationManager:
    """
    Manages multi-robot warehouse simulation.
    
    Coordinates multiple robots, physics simulation, task distribution,
    and visualization following clean architecture principles.
    """
    
    def __init__(self, robot_count: int = 0, placement_mode: str = "persisted", orders_file: str = "sample_orders.json"):
        """Initialize simulation manager."""
        print("🚀 Initializing Multi-Robot Warehouse Simulation")
        print(f"   🤖 Robot Count: {robot_count}")
        print(f"   📋 Orders File: {orders_file}")
        print("   💾 Data Persistence: Enabled - using existing data")

        # Validate robot count
        #if robot_count < 1 or robot_count > 15:
        #    raise ValueError("Robot count must be between 1 and 15 for demo purposes")

        # robot_count == 0 means: use all robots from registry
        self.robot_count = robot_count
        # placement_mode in {persisted, random, manual}
        self.placement_mode = placement_mode
        self.orders_file = orders_file
        self.robots: List[RobotInstance] = []
        # Selected robots (identities) for this run
        self._selected_robots: List[RobotIdentity] = []

        # Try to initialize simulation with existing warehouse data
        try:
            self._initialize_simulation_components()
            print("   ✅ Warehouse data found and loaded successfully")
            print("   💾 Using existing warehouse data")
        except WarehouseDataMissingError:
            print("   ⚠️  No warehouse data found in database")
            print("🔄 FALLBACK: Auto-generating warehouse with default CSV: extended_warehouse.csv")
            print("🔄 FALLBACK: Running load_new_warehouse.py --warehouse extended_warehouse.csv")
            print("🔄 FALLBACK: Waiting for warehouse generation to complete...")

            # Auto-generate warehouse data
            self._generate_warehouse_data_fallback()

            # Retry initialization with newly generated data
            print("🔄 FALLBACK: Retrying simulation initialization with generated warehouse data...")
            self._initialize_simulation_components()
            print("✅ FALLBACK: Warehouse generation completed successfully")
            print("✅ FALLBACK: Demo can now continue with generated warehouse data")

        # Verify warehouse data integrity after initialization
        self._verify_warehouse_data()

        print("🎯 Simulation ready!")

    def _initialize_simulation_components(self) -> None:
        """
        Initialize all simulation components assuming warehouse data exists.

        This method should only be called after warehouse data has been verified
        to exist in the database. It will fail if warehouse data is missing.
        """
        try:
            # Initialize with a minimal temporary map to gain DB access
            temp_map = WarehouseMap()  # default dimensions, not used for generation
            self.simulation_data_service = SimulationDataServiceImpl(temp_map, pool_size=max(10, 3 * self.robot_count))
            print("   ✅ Simulation data service initialized")

            # Retrieve warehouse metadata from DB
            source_csv, expected_hash = self._get_warehouse_metadata_from_db()
            if not source_csv:
                print("   ⚠️  No warehouse metadata found in database")
                raise WarehouseDataMissingError("Warehouse metadata missing")

            if not os.path.exists(source_csv):
                print(f"   ⚠️  Warehouse CSV not found on disk: {source_csv}")
                print("   🔄 FALLBACK: The demo will trigger regeneration")
                raise WarehouseDataMissingError(f"Source CSV missing: {source_csv}")

            # Validate hash when available
            if expected_hash:
                current_hash = _calculate_file_hash(source_csv)
                if current_hash and current_hash != expected_hash:
                    print("   ⚠️  Warehouse CSV hash mismatch detected")
                    print(f"      Expected: {expected_hash[:8]}... | Current: {current_hash[:8]}...")
                    print("   🔄 FALLBACK: The demo will trigger regeneration")
                    raise WarehouseDataMissingError("Warehouse CSV has changed since data generation")

            # Load verified warehouse
            self.warehouse_map = WarehouseMap(csv_file=source_csv)
            print(f"   ✅ Warehouse loaded: {self.warehouse_map.width}x{self.warehouse_map.height} from {source_csv}")

            # Ensure the simulation data service uses the verified warehouse map
            # and invalidate any cached MapData built from the temporary map
            try:
                self.simulation_data_service.warehouse_map = self.warehouse_map
                if hasattr(self.simulation_data_service, "_cache_lock"):
                    with self.simulation_data_service._cache_lock:
                        self.simulation_data_service._map_data_cache = None
                        self.simulation_data_service._cache_timestamp = 0
            except Exception as e:
                print(f"   ⚠️  Warning: Failed to synchronize data service map: {e}")

            # Create real configuration provider
            self.config_provider = ConfigurationProvider()
            print("   ✅ Configuration provider initialized")

            # Set robot placement mode for legacy consumers
            random_placement = self.placement_mode == "random"
            self.config_provider.set_value("demo.random_robot_placement", random_placement)

            if self.placement_mode == "random":
                print("   ✅ Random robot placement enabled")
            elif self.placement_mode == "manual":
                print("   ✅ Manual robot placement enabled (fixed positions)")
            else:
                print("   ✅ Persisted robot placement enabled (DB-backed)")

            # Shared MuJoCo engine for all robots (uses verified map)
            self.shared_engine = SharedMuJoCoEngine(self.warehouse_map, physics_dt=0.001, enable_time_gating=True,
                                                   config_provider=self.config_provider)

            # Test if warehouse data exists by checking conflict boxes
            print("🔍 Testing warehouse data availability...")
            conflict_boxes = self.simulation_data_service.get_conflict_boxes()
            if not conflict_boxes:
                print("   ⚠️  No conflict boxes found - warehouse data may be missing")
                raise WarehouseDataMissingError("No warehouse data found in database - navigation graph missing")

            print(f"   ✅ Warehouse data verified: {len(conflict_boxes)} conflict boxes found")

            # Clear any orphaned conflict box locks from previous simulation runs
            try:
                cleared_count = self.simulation_data_service.clear_all_conflict_box_locks()
                if cleared_count > 0:
                    print(f"   🧹 Cleared {cleared_count} orphaned conflict box locks from previous runs")
                else:
                    print("   ✅ No orphaned conflict box locks found")
            except Exception as e:
                print(f"   ⚠️  Warning: Could not clear conflict box locks: {e}")

            # (moved) Robot selection and engine registration happen after registry init below

            # Initialize robot registry (DB-backed)
            try:
                self.robot_registry = RobotRegistryImpl(self.simulation_data_service)
                print("   ✅ Robot registry initialized")
            except Exception as e:
                print(f"   ❌ Failed to initialize robot registry: {e}")
                logging.error(f"EXCEPTION: Robot registry initialization failed: {e}", exc_info=True)
                raise

            # Select robots for this run via registry (deterministic by name)
            self._select_robots_via_registry()

            # Register all selected robots with shared engine BEFORE initialization
            self._register_robots_with_engine()

            # Initialize shared engine (creates appearance service)
            try:
                self.shared_engine.initialize()
                print("   ✅ Shared engine initialized with appearance service")
            except Exception as e:
                print(f"   ⚠️  Shared engine initialization failed: {e}")

            # Wire KPI recorder to engine for collision KPIs (authoritative mode only)
            try:
                if hasattr(self.simulation_data_service, "_kpi_recorder") and self.simulation_data_service._kpi_recorder is not None:
                    self.shared_engine.set_kpi_recorder(
                        self.simulation_data_service._kpi_recorder,
                        run_id_provider=lambda: getattr(self.simulation_data_service, "_current_simulation_run_id", None)
                    )
            except Exception as e:
                logging.error(f"EXCEPTION: Failed to inject KPI recorder into shared engine: {e}", exc_info=True)

            # Create robot controller components
            self.jobs_queue = JobsQueueImpl()
            self.bidding_system = TransparentBiddingSystem(config_provider=self.config_provider)

            # Initialize order processing pipeline
            try:
                self.order_source = JsonOrderSource(self.orders_file)
                self.order_source.connect()
                print(f"   ✅ Order source initialized and connected to {self.orders_file}")

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

        except WarehouseDataMissingError:
            # Re-raise our custom exception for the caller to handle
            raise
        except Exception as e:
            # Check if this might be related to missing warehouse data
            error_msg = str(e).lower()
            if any(keyword in error_msg for keyword in ['conflict', 'navigation', 'database', 'table', 'schema', 'connection']):
                print(f"   ❌ Simulation initialization failed - possibly due to missing warehouse data: {e}")
                raise WarehouseDataMissingError(f"Simulation initialization failed: {e}") from e
            else:
                # Re-raise other exceptions as-is
                raise

    def _get_warehouse_metadata_from_db(self) -> Tuple[str, str]:
        """
        Retrieve warehouse metadata (source CSV and its hash) from database.

        Returns:
            (source_csv_file, source_csv_hash) or ("", "") when unavailable
        """
        try:
            with self.simulation_data_service._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute(
                        """
                        SELECT source_csv_file, source_csv_hash
                        FROM warehouse_map
                        ORDER BY id DESC
                        LIMIT 1
                        """
                    )
                    row = cur.fetchone()
                    if not row:
                        return "", ""
                    source_csv = row.get('source_csv_file') if isinstance(row, dict) else row[0]
                    csv_hash = row.get('source_csv_hash') if isinstance(row, dict) else row[1]
                    return source_csv or "", csv_hash or ""
        except Exception as e:
            print(f"   ⚠️  Could not retrieve warehouse metadata: {e}")
            print("   🔄 FALLBACK: Proceeding without metadata - may trigger regeneration")
            return "", ""

    def _generate_warehouse_data_fallback(self) -> None:
        """
        Generate warehouse data as a fallback when no data exists.

        This method calls the load_new_warehouse.py script to generate warehouse data
        and handles all error cases gracefully.
        """
        import subprocess
        import sys

        try:
            result = subprocess.run([
                sys.executable, "load_new_warehouse.py",
                "--warehouse", "extended_warehouse.csv"
            ], check=True, capture_output=True, text=True)

            # Log success if there's output
            if result.stdout:
                for line in result.stdout.strip().split('\n'):
                    if line.strip():
                        print(f"   📋 {line}")

        except subprocess.CalledProcessError as e:
            print(f"❌ FALLBACK: Warehouse generation failed: {e}")
            if e.stderr:
                print(f"❌ FALLBACK: Error details: {e.stderr}")
            raise RuntimeError("Failed to generate warehouse data. Please run load_new_warehouse.py manually.")


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
                # Axis-aligned rectangle containment
                half_w = box.width * 0.5
                half_h = box.height * 0.5
                if (abs(world_x - box.center.x) <= half_w and
                    abs(world_y - box.center.y) <= half_h):
                    cell_safe = False
                    break

            if cell_safe:
                filtered_cells.append((grid_x, grid_y))

        # Randomly select positions
        if len(filtered_cells) < count:
            print(f"   ⚠️  Only {len(filtered_cells)} safe walkable cells available, using all of them")
            return filtered_cells

        return random.sample(filtered_cells, count)

    def _find_random_walkable_positions_excluding(self, count: int, exclude_cells: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Variant of random position finder that excludes specified grid cells.
        """
        import random
        from interfaces.navigation_types import Point

        # Get all walkable cells (grid coordinates)
        walkable_cells = []
        for grid_y in range(self.warehouse_map.height):
            for grid_x in range(self.warehouse_map.width):
                if self.warehouse_map.grid[grid_y, grid_x] in [0, 3, 4, 5]:
                    world_pos = self.warehouse_map.grid_to_world(grid_x, grid_y)
                    walkable_cells.append((grid_x, grid_y, world_pos[0], world_pos[1]))

        # Get conflict boxes (best-effort)
        try:
            conflict_boxes = self.simulation_data_service.get_conflict_boxes()
        except Exception as e:
            print(f"   ⚠️  Could not load conflict boxes, proceeding without conflict box avoidance: {e}")
            conflict_boxes = []

        # Build exclusion set for quick checks
        exclude_set = set(exclude_cells)

        filtered_cells: List[Tuple[int, int]] = []
        for grid_x, grid_y, world_x, world_y in walkable_cells:
            if (grid_x, grid_y) in exclude_set:
                continue
            cell_safe = True
            for box in conflict_boxes:
                half_w = box.width * 0.5
                half_h = box.height * 0.5
                if (abs(world_x - box.center.x) <= half_w and
                    abs(world_y - box.center.y) <= half_h):
                    cell_safe = False
                    break
            if cell_safe:
                filtered_cells.append((grid_x, grid_y))

        if len(filtered_cells) < count:
            print(f"   ⚠️  Only {len(filtered_cells)} safe walkable cells available after exclusion, using all of them")
            return filtered_cells
        return random.sample(filtered_cells, count)

    def _register_robots_with_engine(self) -> None:
        """Register selected robots with shared engine, honoring placement mode."""
        print("   🤖 Registering robots with shared engine...")

        selected_ids = [ri.robot_id for ri in self._selected_robots]

        if self.placement_mode == "random":
            print("   🎲 Using random robot placement...")
            random_cells = self._find_random_walkable_positions(len(selected_ids))
            for idx, robot_id in enumerate(selected_ids):
                if idx < len(random_cells):
                    gx, gy = random_cells[idx]
                    wx, wy = self.warehouse_map.grid_to_world(gx, gy)
                    print(f"      🎯 {robot_id} -> Grid ({gx}, {gy}) -> World ({wx:.2f}, {wy:.2f})")
                    self.shared_engine.register_robot(robot_id, (wx, wy, 0.0))
                else:
                    print(f"      ⚠️  No safe position found for {robot_id}, skipping")
        elif self.placement_mode == "manual":
            print("   📍 Using manual robot placement...")
            start_positions = [
                (6.25,2.25),
                (2.75, 9.25),
                (1.75, 8.25),
                (5.75, 1.25),
                (2.25, 2.75),
                (1.75, 1.25),
                (1.25, 6.25),
                (4.75, 5.75),
                (5.25, 8.25),
                (5.75, 4.75),
                (3.25, 2.25),
                (3.25, 1.75),
                (3.25, 1.25),
                (3.25, 0.75),
                (3.25, 0.25),
            ]
            for idx, robot_id in enumerate(selected_ids):
                if idx < len(start_positions):
                    sx, sy = start_positions[idx]
                else:
                    sx, sy = 1.0, float(idx)
                print(f"      📍 {robot_id} -> World ({sx:.2f}, {sy:.2f})")
                self.shared_engine.register_robot(robot_id, (sx, sy, 0.0))
        else:
            print("   🧭 Using persisted robot placement (DB-backed)...")
            # Build reserved cells from robots with persisted positions
            reserved_cells: List[Tuple[int, int]] = []
            robots_needing_random: List[str] = []

            # First pass: register robots with valid persisted positions
            for ri in self._selected_robots:
                state = self.robot_registry.get_robot_state(ri.robot_id)
                if state and state.position is not None:
                    x, y, th = state.position
                    # Validate walkability
                    try:
                        walkable = self.warehouse_map.is_walkable(x, y)
                    except Exception:
                        walkable = True
                    if walkable:
                        gx, gy = self.warehouse_map.world_to_grid(x, y)
                        reserved_cells.append((int(gx), int(gy)))
                        print(f"      🗂️  {ri.robot_id} -> Persisted World ({x:.2f}, {y:.2f})")
                        self.shared_engine.register_robot(ri.robot_id, (x, y, float(th)))
                    else:
                        print(f"      ⚠️  {ri.robot_id} persisted position not walkable, reassigning randomly")
                        robots_needing_random.append(ri.robot_id)
                else:
                    robots_needing_random.append(ri.robot_id)

            # Second pass: assign non-colliding random cells for remaining robots
            if robots_needing_random:
                needed = len(robots_needing_random)
                random_cells = self._find_random_walkable_positions_excluding(needed, reserved_cells)
                for idx, robot_id in enumerate(robots_needing_random):
                    if idx < len(random_cells):
                        gx, gy = random_cells[idx]
                        wx, wy = self.warehouse_map.grid_to_world(gx, gy)
                        reserved_cells.append((gx, gy))
                        print(f"      🎯 {robot_id} -> Random Grid ({gx}, {gy}) -> World ({wx:.2f}, {wy:.2f})")
                        self.shared_engine.register_robot(robot_id, (wx, wy, 0.0))
                    else:
                        print(f"      ⚠️  No safe random position found for {robot_id}, skipping registration")

        print(f"   ✅ Registered {len(selected_ids)} robots with shared engine")

    def _create_robot_agents(self) -> None:
        """Create robot agents for the selected robots with current engine positions."""
        print("   🤖 Creating robot agents...")

        for robot_id in [ri.robot_id for ri in self._selected_robots]:

            # Get starting position from shared engine
            try:
                start_pose = self.shared_engine.get_pose(robot_id)
                start_x, start_y = start_pose[0], start_pose[1]
                grid_pos = self.warehouse_map.world_to_grid(start_x, start_y)
                start_grid_x, start_grid_y = int(grid_pos[0]), int(grid_pos[1])
            except Exception as e:
                print(f"      ⚠️  Could not get position for {robot_id}: {e}")
                # Fallback to manual-like positions if registration fails
                fallback_positions = [
                    (2.75, 1.25),
                    (2.25, 0.75),
                    (1.0, 1.0),
                    (1.0, 2.0),
                    (1.0, 3.0),
                ]
                idx = [ri.robot_id for ri in self._selected_robots].index(robot_id)
                if idx < len(fallback_positions):
                    start_x, start_y = fallback_positions[idx]
                else:
                    start_x, start_y = 1.0, float(idx)

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
                lane_tolerance=0.3,  
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
            
            # Identity is managed via RobotRegistry; no direct inserts here

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

    def _select_robots_via_registry(self) -> None:
        """Select robots for this run from registry (deterministic by name)."""
        try:
            all_identities = self.robot_registry.list_registered_robots()
        except Exception as e:
            print(f"   ❌ Failed to list registered robots: {e}")
            logging.error(f"EXCEPTION: Robot selection failed: {e}", exc_info=True)
            raise

        total = len(all_identities)
        if self.robot_count and self.robot_count > 0:
            if self.robot_count > total:
                print(f"   ❌ Requested {self.robot_count} robots but only {total} are registered. Aborting.")
                raise RuntimeError("Insufficient registered robots for requested run")
            selected = all_identities[:self.robot_count]
            unselected = all_identities[self.robot_count:]
            # Nullify positions for unused robots as requested
            try:
                for ri in unselected:
                    self.robot_registry.clear_robot_state(ri.robot_id)
            except Exception as e:
                logging.error(f"EXCEPTION: Failed to nullify positions for unselected robots: {e}", exc_info=True)
        else:
            selected = all_identities

        self._selected_robots = selected
        self.robot_count = len(selected)
        print(f"   ✅ Selected {self.robot_count} robots for this run")
    
    def _verify_warehouse_data(self) -> None:
        """Verify that warehouse data exists and is valid."""
        print("🔍 Verifying warehouse data integrity...")

        try:
            # Check conflict boxes
            conflict_boxes = self.simulation_data_service.get_conflict_boxes()
            if conflict_boxes:
                print(f"   ✅ Navigation graph verified: {len(conflict_boxes)} conflict boxes found")
            else:
                print("   ⚠️  No conflict boxes found - this may indicate incomplete warehouse data")
                print("   ℹ️  The demo will continue, but path planning may be limited")

            # Check inventory statistics
            stats = self.simulation_data_service.get_inventory_statistics()
            print(f"   ✅ Inventory verified: {stats['total_items']} items across {stats['total_shelves']} shelves")

            # Export initial inventory status
            if self.simulation_data_service.export_inventory_status_csv("initial_inventory_status.csv"):
                print("   📊 Initial inventory status exported: initial_inventory_status.csv")
            else:
                print("   ⚠️  Failed to export initial inventory status")

        except Exception as e:
            print(f"   ❌ Warehouse data verification failed: {e}")
            print("   🔄 This may be due to missing warehouse data - demo will attempt to continue")
            # Don't raise exception - let demo continue even with incomplete data
    
    
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
        
        # Persist final runtime state (positions and battery) before tearing down visualization/engine
        try:
            for robot_instance in self.robots:
                try:
                    status = robot_instance.robot.get_status()
                    pos = status.get('position', (0.0, 0.0, 0.0))
                    battery = float(status.get('battery_level', 1.0))
                    self.robot_registry.upsert_robot_state(robot_instance.robot_id, (float(pos[0]), float(pos[1]), float(pos[2])), battery)
                except Exception as e:
                    logging.error(f"EXCEPTION: Failed to persist state for {robot_instance.robot_id}: {e}", exc_info=True)
        except Exception as e:
            logging.error(f"EXCEPTION: Persisting final runtime state failed: {e}", exc_info=True)

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
            print(f"   🧩 PD Tasks Completed: {overview.get('pd_tasks_completed', 0)} | PD Tasks Failed: {overview.get('pd_tasks_failed', 0)} | PD Success: {overview.get('pd_success_rate', 0):.1f}% | PD Avg Time: {overview.get('pd_avg_task_time_seconds', 0):.1f}s")
            print(f"   ✅ Overall Tasks Completed: {overview.get('tasks_completed', 0)}")
            print(f"   ❌ Tasks Failed: {overview.get('tasks_failed', 0)}")
            print(f"   📈 Success Rate: {overview.get('task_success_rate', 0):.1f}%")
            #print(f"   💼 Busy Time: {overview.get('busy_time_seconds', 0):.1f}s")
            print(f"   📊 Avg Busy % per Robot: {overview.get('avg_busy_percent_per_robot', 0):.1f}%")
            print(f"   🆓 Avg Idle % per Robot: {overview.get('avg_idle_percent_per_robot', 0):.1f}%")
            print(f"   💥 Collisions: {overview.get('collision_count', 0)}")

            if export_enabled:
                if self.simulation_data_service.export_kpi_overview_csv(overview, export_path):
                    print(f"   💾 KPI CSV exported: {export_path}")
                else:
                    print(f"   ⚠️  KPI CSV export failed: {export_path}")
        except Exception as e:
            print(f"   ⚠️  KPI summary unavailable: {e}")

        # Clear any remaining reserved quantities (pending) before exporting final inventory
        try:
            cleared_rows = self.simulation_data_service.clear_all_reservations()
            if cleared_rows > 0:
                print(f"\n🧼 Cleared reserved quantities on {cleared_rows} inventory rows")
            else:
                print("\n🧼 No reserved quantities to clear")
        except Exception as e:
            print(f"\n⚠️  Failed to clear reserved quantities: {e}")

        # Export final inventory status
        print("\n📊 Exporting Final Inventory Status...")
        if self.simulation_data_service.export_inventory_status_csv("final_inventory_status.csv"):
            print("      📊 Final inventory status exported: final_inventory_status.csv")
        else:
            print("      ⚠️  Failed to export final inventory status")

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





def main():
    """Main entry point for the multi-robot warehouse simulation demo."""
    from robot.impl.motion_executor_impl import MotionExecutorImpl
    # Return to standard logging to reduce overhead
    MotionExecutorImpl.set_verbose_logging(False)

    print("🤖 Multi-Robot Warehouse Coordination Simulation Demo")
    print("=" * 60)
    print("Professional multi-robot autonomous warehouse simulation")
    print("Demonstrating coordinated operation with configurable:")
    print("   🤖 Multiple robots working simultaneously")
    print("   🎮 Robot controller managing task distribution")
    print("   🔄 Conflict box coordination for safe navigation")
    print("   📊 Real-time monitoring and status reporting")
    print("   🏭 Configurable warehouse layouts")
    print("   📋 Configurable order processing")
    print()
    
    # Optional CLI arguments (e.g., --robots 5 --placement persisted --orders sample_orders.json)
    try:
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument("--robots", type=int, default=0,
                          help="Number of robots to simulate (0 = all registered; default: 0)")
        parser.add_argument("--placement", choices=["persisted", "random", "manual"], default="persisted",
                          help="Robot placement mode: 'persisted' (DB-backed), 'random', or 'manual' (default: persisted)")
        parser.add_argument("--orders", type=str, default="sample_orders.json",
                          help="JSON orders file to process (default: sample_orders.json)")
        args, _ = parser.parse_known_args()
        robot_count = args.robots
        placement_mode = args.placement
        orders_file = args.orders
    except Exception:
        robot_count = 0
        placement_mode = "persisted"
        orders_file = "sample_orders.json"

    try:
        # Run the demo with requested robot count and placement mode
        # Wrapper to use the same flow while allowing variable robot count
        print(f"Requested robot count: {robot_count}")
        print(f"Robot placement mode: {placement_mode}")
        print(f"Orders file: {orders_file}")
        print("Data persistence: enabled (warehouse data auto-managed)")
        sim = MultiRobotSimulationManager(robot_count=robot_count, placement_mode=placement_mode,
                                         orders_file=orders_file)
        try:
            sim.start_simulation()
            # Order processing is now handled automatically by the order processor
            # Orders will be loaded from the specified orders file and processed into tasks
            print(f"\n📋 Order processing will load orders from {orders_file} automatically")
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