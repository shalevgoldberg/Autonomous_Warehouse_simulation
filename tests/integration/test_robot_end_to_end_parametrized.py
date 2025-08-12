"""
Parameterized end-to-end runs over multiple valid start/goal nodes from the
graph generated from sample_warehouse.csv. Uses the real DB through
SimulationDataServiceImpl, and reports per-case lane deviation stats.

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


def _shortest_path(edges: dict[str, list[str]], start: str, goal: str) -> list[str] | None:
    if start == goal:
        return [start]
    visited = set([start])
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


def _pick_start_goal_pairs(graph, desired_pairs: int = 3) -> list[tuple[str, str, list[str]]]:
    # Consider only non-conflict-box nodes as viable start/goal candidates
    nodes = [nid for nid, n in graph.nodes.items() if not getattr(n, 'is_conflict_box', False)]
    pairs: list[tuple[str, str, list[str]]] = []
    # Choose starts with outgoing edges
    candidate_starts = [n for n in nodes if len(graph.edges.get(n, [])) > 0]
    for start in sorted(candidate_starts):
        # Prefer a goal at path length 4-8 if possible, else any reachable different node
        goals = [n for n in nodes if n != start]
        for goal in goals:
            path = _shortest_path(graph.edges, start, goal)
            if not path or len(path) < 3:
                continue
            if 4 <= len(path) <= 8:
                pairs.append((start, goal, path))
                break
        if len(pairs) >= desired_pairs:
            break
    # If not enough pairs found, relax length constraint
    if len(pairs) < desired_pairs:
        for start in sorted(candidate_starts):
            for goal in nodes:
                if goal == start:
                    continue
                path = _shortest_path(graph.edges, start, goal)
                if path and len(path) >= 3:
                    triplet = (start, goal, path)
                    if triplet not in pairs:
                        pairs.append(triplet)
                        if len(pairs) >= desired_pairs:
                            break
            if len(pairs) >= desired_pairs:
                break
    return pairs


class TestRobotEndToEndParametrized(unittest.TestCase):
    def setUp(self) -> None:
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("WAREHOUSE_DB_PASSWORD not set; skipping DB-backed E2E param test")

        self.warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
        self.sim_service = SimulationDataServiceImpl(
            warehouse_map=self.warehouse_map,
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD'),
            pool_size=5,
        )

        # Ensure shelves and graph are persisted from the CSV
        self.sim_service.create_shelves_from_map(clear_existing=True)
        self.sim_service.persist_navigation_graph_from_csv(Path("sample_warehouse.csv"), clear_existing=True)

        # Retrieve graph to enumerate valid nodes/edges
        self.graph = self.sim_service.get_navigation_graph()

    def tearDown(self) -> None:
        if hasattr(self, 'sim_service'):
            self.sim_service.close()

    def test_multiple_start_goal_pairs_complete(self) -> None:
        pairs = _pick_start_goal_pairs(self.graph, desired_pairs=5)
        self.assertGreaterEqual(len(pairs), 1, "No valid start/goal pairs found in graph")

        all_completed: list[tuple[str, str, bool, dict]] = []

        for (start_id, goal_id, path) in pairs:
            start_node = self.graph.nodes[start_id]
            goal_node = self.graph.nodes[goal_id]

            physics = SimpleMuJoCoPhysics(self.warehouse_map)
            config_provider = ConfigurationProvider()
            robot = RobotAgent(
                physics=physics,
                config_provider=config_provider,
                robot_id=f"param_robot_{start_id}_{goal_id}",
                simulation_data_service=self.sim_service,
            )

            try:
                robot.start()
                time.sleep(0.3)

                # Relax tolerance for E2E coverage; tight precision is covered elsewhere
                robot.lane_follower.set_config(LaneFollowingConfig(lane_tolerance=10.0, max_speed=1.0, corner_speed=0.3))

                # Initial heading aligned with first segment if possible
                if len(path) >= 2:
                    n0 = self.graph.nodes[path[0]].position
                    n1 = self.graph.nodes[path[1]].position
                    heading = math.atan2(n1.y - n0.y, n1.x - n0.x)
                else:
                    heading = 0.0

                physics.reset_robot_position(start_node.position.x, start_node.position.y, heading)

                task = robot.create_move_task(goal_node.position.x, goal_node.position.y)
                accepted = robot.assign_task(task)
                self.assertTrue(accepted, f"Robot should accept task {start_id}->{goal_id}")

                timeout_s = 60.0
                start_time = time.time()
                completed = False
                while time.time() - start_time < timeout_s:
                    status = robot.get_status()
                    if not status['task_status'].has_active_task:
                        completed = True
                        break
                    time.sleep(0.1)

                dev = robot.lane_follower.get_lane_deviation_stats()
                print(f"[E2E-PARAM] {start_id}->{goal_id} path_len={len(path)} completed={completed} max_dev={dev.get('max_deviation', 0.0):.3f}m last_dev={dev.get('last_deviation', 0.0):.3f}m tol={dev.get('tolerance', 0.0):.3f}m")

                all_completed.append((start_id, goal_id, completed, dev))

                self.assertTrue(completed, f"Task {start_id}->{goal_id} did not complete")

            finally:
                robot.stop()

        # Optional overall assertion: all completed already checked individually
        self.assertTrue(all(c for _, _, c, _ in all_completed))


if __name__ == "__main__":
    unittest.main()


