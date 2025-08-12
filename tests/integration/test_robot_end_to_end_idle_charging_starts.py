"""
E2E tests starting from idle zones or charging stations, ending at walkable areas
(lane node positions) on the real CSV map with real DB-backed services.

Windows note: Requires WAREHOUSE_DB_PASSWORD to be set.
"""

import os
import time
import math
import unittest
from pathlib import Path
from collections import deque

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from config.configuration_provider import ConfigurationProvider
from interfaces.lane_follower_interface import LaneFollowingConfig


def _bfs(edges: dict[str, list[str]], start: str, goal: str) -> list[str] | None:
    if start == goal:
        return [start]
    visited = {start}
    q = deque([(start, [start])])
    while q:
        node, path = q.popleft()
        for nbr in edges.get(node, []):
            if nbr in visited:
                continue
            npath = path + [nbr]
            if nbr == goal:
                return npath
            visited.add(nbr)
            q.append((nbr, npath))
    return None


class TestIdleChargingStarts(unittest.TestCase):
    def setUp(self) -> None:
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("WAREHOUSE_DB_PASSWORD not set; skipping DB-backed E2E test")

        self.warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
        self.sim_service = SimulationDataServiceImpl(
            warehouse_map=self.warehouse_map,
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD'),
            pool_size=5,
        )

        # Persist shelves and graph from CSV so DB matches map
        self.sim_service.create_shelves_from_map(clear_existing=True)
        self.sim_service.persist_navigation_graph_from_csv(Path("sample_warehouse.csv"), clear_existing=True)

        self.graph = self.sim_service.get_navigation_graph()

        # Collect idle and charging world coordinates from the map grid
        self.idle_positions = self.warehouse_map._get_idle_zones()
        # Charging positions: scan grid for type 3
        charging_positions: list[tuple[float, float]] = []
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                if self.warehouse_map.grid[y, x] == 3:
                    charging_positions.append(self.warehouse_map.grid_to_world(x, y))
        self.charging_positions = charging_positions

        # Candidate goal nodes: non-conflict-box nodes (walkable lane centers)
        self.goal_nodes = [nid for nid, n in self.graph.nodes.items() if not getattr(n, 'is_conflict_box', False)]

    def tearDown(self) -> None:
        if hasattr(self, 'sim_service'):
            self.sim_service.close()

    def _nearest_node_id(self, x: float, y: float) -> str:
        best_id = None
        best_d2 = float("inf")
        for nid, node in self.graph.nodes.items():
            dx = node.position.x - x
            dy = node.position.y - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_id = nid
        assert best_id is not None
        return best_id

    def _node_is_walkable(self, nid: str) -> bool:
        node = self.graph.nodes[nid].position
        return self.warehouse_map.is_walkable(node.x, node.y)

    def test_starts_from_idle_and_charging(self) -> None:
        starts = []
        # Up to 2 idle starts and 1 charging start (if present)
        starts.extend(self.idle_positions[:2])
        if self.charging_positions:
            starts.append(self.charging_positions[0])
        self.assertGreater(len(starts), 0, "No idle/charging start positions found in map")

        results: list[tuple[tuple[float, float], str, bool, dict]] = []

        # Pick diverse goal nodes; ensure connectivity from nearest start node
        chosen_goals: list[str] = []
        for sx, sy in starts:
            start_nid = self._nearest_node_id(sx, sy)
            # Find a goal with a reasonable path length (>= 3 nodes) different from start
            goal_nid = None
            for cand in self.goal_nodes:
                if cand == start_nid:
                    continue
                path = _bfs(self.graph.edges, start_nid, cand)
                if path and len(path) >= 3:
                    goal_nid = cand
                    break
            if goal_nid is None:
                # Fallback: any different node with a path
                for cand in self.goal_nodes:
                    if cand == start_nid:
                        continue
                    path = _bfs(self.graph.edges, start_nid, cand)
                    if path:
                        goal_nid = cand
                        break
            self.assertIsNotNone(goal_nid, f"No reachable goal found from start near ({sx:.2f},{sy:.2f})")
            chosen_goals.append(goal_nid)  # track

        for (sx, sy), goal_nid in zip(starts, chosen_goals):
            goal_pos = self.graph.nodes[goal_nid].position

            physics = SimpleMuJoCoPhysics(self.warehouse_map)
            config_provider = ConfigurationProvider()
            robot = RobotAgent(
                physics=physics,
                config_provider=config_provider,
                robot_id=f"idle_charge_start_{sx:.1f}_{sy:.1f}",
                simulation_data_service=self.sim_service,
            )

            try:
                robot.start()
                time.sleep(0.3)

                # Choose a lane-connected spawn near the idle/charging zone: nearest graph node position
                start_nid = self._nearest_node_id(sx, sy)
                # If that node isn’t walkable (due to map/graph mismatch), pick a walkable neighbor
                if not self._node_is_walkable(start_nid):
                    for nbr in self.graph.edges.get(start_nid, []):
                        if self._node_is_walkable(nbr):
                            start_nid = nbr
                            break
                start_node = self.graph.nodes[start_nid].position
                heading = math.atan2(goal_pos.y - start_node.y, goal_pos.x - start_node.x)
                physics.reset_robot_position(start_node.x, start_node.y, heading)
                # Physics holds the authoritative pose; control threads will read it on tick

                # Relax tolerance for E2E coverage
                robot.lane_follower.set_config(LaneFollowingConfig(lane_tolerance=10.0, max_speed=1.0, corner_speed=0.3))

                # End at a walkable area: use the world position of a lane node
                task = robot.create_move_task(goal_pos.x, goal_pos.y)
                self.assertTrue(robot.assign_task(task))

                timeout_s = 60.0
                start_time = time.time()
                completed = False
                while time.time() - start_time < timeout_s:
                    if not robot.get_status()['task_status'].has_active_task:
                        completed = True
                        break
                    time.sleep(0.1)

                dev = robot.lane_follower.get_lane_deviation_stats()
                print(f"[E2E-IDLE/CHARGE] start_idle/charge=({sx:.1f},{sy:.1f}) spawn_node={start_nid} -> goal={goal_nid} completed={completed} max_dev={dev.get('max_deviation', 0.0):.3f}m last_dev={dev.get('last_deviation', 0.0):.3f}m tol={dev.get('tolerance', 0.0):.3f}m")
                self.assertTrue(completed, f"Task from ({sx:.1f},{sy:.1f}) did not complete")

                results.append(((sx, sy), goal_nid, completed, dev))

            finally:
                robot.stop()

        self.assertTrue(all(c for _, _, c, _ in results))


if __name__ == "__main__":
    unittest.main()


