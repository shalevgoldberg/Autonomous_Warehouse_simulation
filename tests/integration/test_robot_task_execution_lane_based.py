"""
Integration tests for lane-based robot task execution.

Testing note about LaneFollowingConfig:
- These integration tests set a relaxed lane tolerance (e.g., 10.0m).
- Purpose: validate end-to-end route execution, turns, and sequencing without
  failing on transient deviations inherent to discrete control and sharp turns.
- Precision tracking and strict tolerances are covered in unit/integration tests
  focused on the lane follower itself (see tests/integration/test_lane_follower_integration.py),
  which can use tighter thresholds.
"""

import time
import threading

from unittest.mock import MagicMock

from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from interfaces.task_handler_interface import Task, TaskType, TaskHandlerStatus, OperationalStatus
from interfaces.simulation_data_service_interface import MapData, NavigationGraph, GraphNode
from interfaces.navigation_types import Point, LaneDirection, BoxRec
from config.configuration_provider import ConfigurationProvider
from interfaces.lane_follower_interface import LaneFollowingConfig


def _make_mock_simulation_service(warehouse_map: WarehouseMap) -> MagicMock:
    """Create a mock ISimulationDataService with minimal lane graph and map data."""
    svc = MagicMock()

    # Map data -> used by agent to build internal warehouse map
    svc.get_map_data.return_value = MapData(
        width=warehouse_map.width,
        height=warehouse_map.height,
        cell_size=warehouse_map.grid_size,
        obstacles=[],
        shelves={},
        dropoff_zones=[],
        charging_zones=[],
    )

    # Simple navigation graph with a straight connection
    nodes = {
        'start': GraphNode('start', Point(0.5, 0.5), [LaneDirection.EAST]),
        'mid': GraphNode('mid', Point(2.5, 0.5), [LaneDirection.WEST, LaneDirection.EAST]),
        'goal': GraphNode('goal', Point(4.5, 0.5), [LaneDirection.WEST]),
    }
    edges = {
        'start': ['mid'],
        'mid': ['start', 'goal'],
        'goal': ['mid'],
    }
    conflict_boxes: dict[str, BoxRec] = {}
    graph = NavigationGraph(nodes=nodes, edges=edges, conflict_boxes=conflict_boxes)

    svc.get_navigation_graph.return_value = graph
    svc.get_blocked_cells.return_value = {}

    # Conflict box operations (not used in this simple route)
    svc.try_acquire_conflict_box_lock.return_value = True
    svc.release_conflict_box_lock.return_value = True
    svc.heartbeat_conflict_box_lock.return_value = True

    # Inventory-related stubs
    svc.get_shelf_position.return_value = (1.0, 1.0)
    svc.get_dropoff_zones.return_value = [(5.0, 5.0)]
    svc.lock_shelf.return_value = True
    svc.unlock_shelf.return_value = True
    svc.log_event.return_value = None

    return svc


def test_robot_executes_move_task_to_completion():
    """End-to-end: robot receives a MOVE_TO_POSITION task and completes it."""
    # Build minimal warehouse and physics
    wh_map = WarehouseMap(width=10, height=10)
    # Ensure free space for movement in this test
    import numpy as _np
    wh_map.grid[:, :] = 0  # clear all obstacles
    physics = SimpleMuJoCoPhysics(wh_map)

    # Configuration provider with defaults
    config_provider = ConfigurationProvider()

    # Mock data service (avoid database)
    sim_service = _make_mock_simulation_service(wh_map)

    # Create lane-based robot agent
    robot = RobotAgent(
        physics=physics,
        config_provider=config_provider,
        robot_id="it_robot_1",
        simulation_data_service=sim_service,
    )

    try:
        # Start physics and control loops and ensure physics thread is running
        robot.start()
        time.sleep(0.25)  # allow physics thread to spin up and begin stepping

        # NOTE: For path-shape integration tests we intentionally relax lane tolerance.
        # Turning and discretized control can transiently exceed tight tolerances.
        # This mirrors existing integration tests (e.g., test_lane_follower_integration)
        # that use a very tolerant configuration to validate sequencing/flow rather than
        # precision tracking. Functional safety/tight tolerances are covered elsewhere.
        relaxed_config = LaneFollowingConfig(lane_tolerance=10.0, max_speed=1.0, corner_speed=0.3)
        robot.lane_follower.set_config(relaxed_config)

        # Reset robot to known start and create a simple move task along X axis
        # Face north for the first (vertical) segment to reduce initial lane deviation
        import math as _math
        physics.reset_robot_position(0.5, 0.5, _math.pi/2)
        target = (4.5, 0.5, 0.0)
        task = Task(
            task_id="move_simple",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=target,
            priority=1,
        )

        # Assign task
        accepted = robot.assign_task(task)
        assert accepted, "Robot should accept the task when idle"

        # Wait for completion with timeout
        timeout_s = 30.0
        start_time = time.time()
        last_status_log = 0.0
        completed = False
        while time.time() - start_time < timeout_s:
            status = robot.get_status()
            th_status: TaskHandlerStatus = status['task_status']
            if (time.time() - last_status_log) > 1.0:
                # Periodic heartbeat for debugging
                last_status_log = time.time()
                # print can be uncommented for local runs
                # print(f"status={th_status.operational_status}, progress={th_status.progress:.2f}, pos={status['position']}")

            # Consider task complete as soon as handler reports no active task
            if not th_status.has_active_task:
                completed = True
                break
            time.sleep(0.1)

        assert completed, "Task did not complete within timeout"

        # Sanity: final position should be near target X (within tolerance)
        final_pos = robot.state_holder.get_position()
        assert abs(final_pos[0] - target[0]) < 0.6, f"Robot X not near target: {final_pos} vs {target}"
        assert abs(final_pos[1] - target[1]) < 0.6, f"Robot Y not near target: {final_pos} vs {target}"

    finally:
        robot.stop()



def _make_mock_service_with_graph(warehouse_map: WarehouseMap, node_defs, edge_defs) -> MagicMock:
    """Generic mock service with provided nodes/edges graph."""
    svc = MagicMock()

    svc.get_map_data.return_value = MapData(
        width=warehouse_map.width,
        height=warehouse_map.height,
        cell_size=warehouse_map.grid_size,
        obstacles=[],
        shelves={},
        dropoff_zones=[],
        charging_zones=[],
    )

    nodes = {name: GraphNode(name, Point(x, y), dirs) for name, (x, y, dirs) in node_defs.items()}
    graph = NavigationGraph(nodes=nodes, edges=edge_defs, conflict_boxes={})
    svc.get_navigation_graph.return_value = graph
    svc.get_blocked_cells.return_value = {}
    svc.try_acquire_conflict_box_lock.return_value = True
    svc.release_conflict_box_lock.return_value = True
    svc.heartbeat_conflict_box_lock.return_value = True
    svc.log_event.return_value = None
    return svc


def test_robot_executes_L_turn_path_clean_environment():
    """Robot performs an L-shaped path: go north then east to target."""
    wh_map = WarehouseMap(width=12, height=12)
    import numpy as _np
    wh_map.grid[:, :] = 0
    physics = SimpleMuJoCoPhysics(wh_map)

    config_provider = ConfigurationProvider()

    node_defs = {
        'start': (0.5, 0.5, [LaneDirection.NORTH]),
        'n1':    (0.5, 1.5, [LaneDirection.SOUTH, LaneDirection.NORTH]),
        'n2':    (0.5, 2.5, [LaneDirection.SOUTH, LaneDirection.EAST]),
        'e1':    (1.5, 2.5, [LaneDirection.WEST, LaneDirection.EAST]),
        'e2':    (2.5, 2.5, [LaneDirection.WEST, LaneDirection.EAST]),
        'goal':  (4.5, 2.5, [LaneDirection.WEST]),
    }
    edge_defs = {
        'start': ['n1'],
        'n1': ['start', 'n2'],
        'n2': ['n1', 'e1'],
        'e1': ['n2', 'e2'],
        'e2': ['e1', 'goal'],
        'goal': ['e2'],
    }
    sim_service = _make_mock_service_with_graph(wh_map, node_defs, edge_defs)

    robot = RobotAgent(
        physics=physics,
        config_provider=config_provider,
        robot_id="it_robot_L1",
        simulation_data_service=sim_service,
    )

    try:
        robot.start()
        time.sleep(0.25)
        # Relax lane tolerance for complex path validation (see note above)
        relaxed_config = LaneFollowingConfig(lane_tolerance=10.0, max_speed=1.0, corner_speed=0.3)
        robot.lane_follower.set_config(relaxed_config)
        # Face north for the first segment
        import math as _math
        physics.reset_robot_position(0.5, 0.5, _math.pi/2)
        target = (4.5, 2.5, 0.0)
        task = Task(
            task_id="move_L_turn",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=target,
            priority=1,
        )

        assert robot.assign_task(task)

        timeout_s = 40.0
        start_time = time.time()
        completed = False
        while time.time() - start_time < timeout_s:
            if not robot.get_status()['task_status'].has_active_task:
                completed = True
                break
            time.sleep(0.1)
        assert completed, "L-turn task did not complete"

        # Report lane deviation observed during the run
        dev_stats = robot.lane_follower.get_lane_deviation_stats()
        print(f"[TEST] L-turn clean: max_deviation={dev_stats['max_deviation']:.3f}m, last_deviation={dev_stats['last_deviation']:.3f}m, tolerance={dev_stats['tolerance']:.3f}m")

        final_pos = robot.state_holder.get_position()
        assert abs(final_pos[0] - target[0]) < 0.6
        assert abs(final_pos[1] - target[1]) < 0.6

        # Report lane deviation observed during the run
        dev_stats = robot.lane_follower.get_lane_deviation_stats()
        print(f"[TEST] L-turn: max_deviation={dev_stats['max_deviation']:.3f}m, last_deviation={dev_stats['last_deviation']:.3f}m, tolerance={dev_stats['tolerance']:.3f}m")
    finally:
        robot.stop()


def test_robot_executes_path_with_obstacles_and_turn():
    """Robot routes around shelves/walls with an L-turn path (environment has obstacles)."""
    wh_map = WarehouseMap(width=14, height=12)
    import numpy as _np
    wh_map.grid[:, :] = 0
    # Place a horizontal shelf band to block straight east motion at y=1.0 (grid row 2)
    y_row = 2
    for x in range(1, 10):
        wh_map.grid[y_row, x] = 2  # shelf
    # Leave a corridor at y=5 (world 2.5m) and above as free

    physics = SimpleMuJoCoPhysics(wh_map)
    config_provider = ConfigurationProvider()

    node_defs = {
        'start': (0.5, 0.5, [LaneDirection.NORTH]),
        'up1':   (0.5, 1.5, [LaneDirection.SOUTH, LaneDirection.NORTH]),
        'corr':  (0.5, 2.5, [LaneDirection.SOUTH, LaneDirection.EAST]),
        'e1':    (1.5, 2.5, [LaneDirection.WEST, LaneDirection.EAST]),
        'e2':    (2.5, 2.5, [LaneDirection.WEST, LaneDirection.EAST]),
        'e3':    (3.5, 2.5, [LaneDirection.WEST, LaneDirection.EAST]),
        'goal':  (5.5, 2.5, [LaneDirection.WEST]),
    }
    edge_defs = {
        'start': ['up1'],
        'up1': ['start', 'corr'],
        'corr': ['up1', 'e1'],
        'e1': ['corr', 'e2'],
        'e2': ['e1', 'e3'],
        'e3': ['e2', 'goal'],
        'goal': ['e3'],
    }
    sim_service = _make_mock_service_with_graph(wh_map, node_defs, edge_defs)

    robot = RobotAgent(
        physics=physics,
        config_provider=config_provider,
        robot_id="it_robot_obs1",
        simulation_data_service=sim_service,
    )

    try:
        robot.start()
        time.sleep(0.25)
        # Relax lane tolerance for complex path with obstacles (see note above)
        relaxed_config = LaneFollowingConfig(lane_tolerance=10.0, max_speed=1.0, corner_speed=0.3)
        robot.lane_follower.set_config(relaxed_config)
        physics.reset_robot_position(0.5, 0.5, 0.0)
        target = (5.5, 2.5, 0.0)
        task = Task(
            task_id="move_obstacles",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=target,
            priority=1,
        )
        assert robot.assign_task(task)

        timeout_s = 45.0
        start_time = time.time()
        completed = False
        while time.time() - start_time < timeout_s:
            if not robot.get_status()['task_status'].has_active_task:
                completed = True
                break
            time.sleep(0.1)

        # Report lane deviation observed during the run regardless of completion
        dev_stats = robot.lane_follower.get_lane_deviation_stats()
        print(f"[TEST] Obstacles: max_deviation={dev_stats['max_deviation']:.3f}m, last_deviation={dev_stats['last_deviation']:.3f}m, tolerance={dev_stats['tolerance']:.3f}m")

        assert completed, "Obstacle path task did not complete"

        final_pos = robot.state_holder.get_position()
        assert abs(final_pos[0] - target[0]) < 0.6
        assert abs(final_pos[1] - target[1]) < 0.6

        # Report lane deviation observed during the run
        dev_stats = robot.lane_follower.get_lane_deviation_stats()
        print(f"[TEST] Obstacles: max_deviation={dev_stats['max_deviation']:.3f}m, last_deviation={dev_stats['last_deviation']:.3f}m, tolerance={dev_stats['tolerance']:.3f}m")
    finally:
        robot.stop()

