"""
Parameterized end-to-end runs over multiple valid start/goal nodes from the
graph generated from sample_warehouse.csv. Uses the real DB through
SimulationDataServiceImpl, and reports per-case lane deviation stats.

Windows note: Requires WAREHOUSE_DB_PASSWORD to be set.

Features:
- Real-time MuJoCo visualization of robot movements
- Database-backed navigation graph
- Multiple robot path testing with visual feedback
- Performance metrics and lane deviation statistics

Visualization Configuration:
- Set ENABLE_VISUALIZATION = True to see real-time robot movements in MuJoCo viewer
- Set ENABLE_VISUALIZATION = False for faster testing without visualization
- Adjust VISUALIZATION_FPS for smoother or faster visualization updates
- Use test_multiple_start_goal_pairs_no_visualization() for fast testing

Motion Logging Configuration:
- Set ENABLE_VERBOSE_MOTION_LOGS = False to reduce verbose motion output (default)
- Set ENABLE_VERBOSE_MOTION_LOGS = True to see detailed motion calculations
- Use test_multiple_start_goal_pairs_with_verbose_logs() for debugging motion issues

Usage Examples:
1. With visualization (default):
   python -m pytest tests/integration/test_robot_end_to_end_parametrized.py::TestRobotEndToEndParametrized::test_multiple_start_goal_pairs_complete -v -s

2. Without visualization (faster):
   python -m pytest tests/integration/test_robot_end_to_end_parametrized.py::TestRobotEndToEndParametrized::test_multiple_start_goal_pairs_no_visualization -v -s

3. With verbose motion logging (for debugging):
   python -m pytest tests/integration/test_robot_end_to_end_parametrized.py::TestRobotEndToEndParametrized::test_multiple_start_goal_pairs_with_verbose_logs -v -s

4. Custom configuration:
   - Modify class variables: ENABLE_VISUALIZATION, VISUALIZATION_FPS, MAX_ROBOTS_TO_TEST
   - Adjust TASK_TIMEOUT_SECONDS for longer/shorter robot paths
   - Change STATUS_UPDATE_INTERVAL for more/fewer status updates
   - Set ENABLE_VERBOSE_MOTION_LOGS for motion debugging

Visualization Features:
- 3D MuJoCo viewer showing warehouse layout and robot movements
- Real-time robot position tracking and path visualization
- Camera follows the primary robot automatically
- 30 FPS smooth visualization updates
- Multi-robot support (tracks primary robot, shows all robot paths)
"""

import os
import time
import math
import unittest
from pathlib import Path
from collections import deque
from typing import Optional

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from config.configuration_provider import ConfigurationProvider
from interfaces.lane_follower_interface import LaneFollowingConfig
from interfaces.visualization_interface import IVisualization
from simulation.mujoco_visualization import MujocoVisualization
from simulation.visualization_thread import VisualizationThread


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
    # Configuration: Set to False to disable visualization for faster testing
    ENABLE_VISUALIZATION = True
    
    # Visualization settings
    VISUALIZATION_FPS = 30.0  # Frames per second for visualization
    
    # Test parameters
    MAX_ROBOTS_TO_TEST = 5  # Maximum number of robot paths to test
    TASK_TIMEOUT_SECONDS = 60.0  # Timeout for each robot task
    STATUS_UPDATE_INTERVAL = 5.0  # Status update interval in seconds
    
    # Logging configuration
    ENABLE_VERBOSE_MOTION_LOGS = False  # Set to True to see detailed motion logs
    
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
        
        # Initialize visualization components
        self.visualization: Optional[IVisualization] = None
        self.visualization_thread: Optional[VisualizationThread] = None
        self.robots: list[RobotAgent] = []
        
        print("🚀 End-to-End Test with Visualization Setup Complete!")
        print(f"   📊 Navigation Graph: {len(self.graph.nodes)} nodes, {sum(len(neighbors) for neighbors in self.graph.edges.values())} edges")
        print(f"   🏭 Warehouse Map: {self.warehouse_map.width}x{self.warehouse_map.height} grid")
        print(f"   📦 Shelves: {len(self.warehouse_map.shelves)}")
        
        # Configure motion logging
        from robot.impl.motion_executor_impl import MotionExecutorImpl
        MotionExecutorImpl.set_verbose_logging(self.ENABLE_VERBOSE_MOTION_LOGS)
        print(f"   📝 Motion logging: {'VERBOSE' if self.ENABLE_VERBOSE_MOTION_LOGS else 'QUIET'}")

    def tearDown(self) -> None:
        # Stop all robots
        for robot in self.robots:
            try:
                robot.stop()
            except Exception as e:
                print(f"⚠️ Warning: Failed to stop robot: {e}")
        
        # Stop visualization
        if self.visualization_thread is not None:
            self.visualization_thread.stop()
            self.visualization_thread = None
        if self.visualization is not None:
            try:
                self.visualization.shutdown()
            except Exception as e:
                print(f"⚠️ Warning: Failed to shutdown visualization: {e}")
            self.visualization = None
        
        # Close simulation service
        if hasattr(self, 'sim_service'):
            self.sim_service.close()
        
        print("🧹 Test cleanup completed")
    
    def _setup_visualization_for_robots(self, robots: list[RobotAgent]) -> None:
        """Setup visualization to track multiple robots."""
        if not robots or not self.ENABLE_VISUALIZATION:
            return
            
        try:
            primary_robot = robots[0]
            if self.visualization is None:
                self.visualization = MujocoVisualization(
                    state_holder=primary_robot.state_holder_interface,
                    warehouse_map=self.warehouse_map
                )
                self.visualization.initialize()
                self.visualization_thread = VisualizationThread(self.visualization, fps=self.VISUALIZATION_FPS)
                self.visualization_thread.start()
                print(f"   🎬 Multi-robot visualization started at {self.VISUALIZATION_FPS} FPS")
                print(f"   📍 Tracking {len(robots)} robots (primary: {primary_robot.robot_id})")
            else:
                # Already initialized; just switch tracked robot
                self.visualization.set_state_holder(primary_robot.state_holder_interface)
        except Exception as e:
            print(f"   ⚠️ Visualization failed to start: {e}")
            print("   📊 Test will continue without visualization")
    
    def _update_visualization_robot(self, robot: RobotAgent) -> None:
        """Update visualization to track a specific robot."""
        if self.visualization is None or not self.ENABLE_VISUALIZATION:
            return
            
        try:
            self.visualization.set_state_holder(robot.state_holder_interface)
            print(f"   🎥 Visualization now tracking robot: {robot.robot_id}")
        except Exception as e:
            print(f"   ⚠️ Visualization update failed: {e}")

    def test_multiple_start_goal_pairs_complete(self) -> None:
        pairs = _pick_start_goal_pairs(self.graph, desired_pairs=self.MAX_ROBOTS_TO_TEST)
        self.assertGreaterEqual(len(pairs), 1, "No valid start/goal pairs found in graph")
        
        print(f"🎯 Testing {len(pairs)} start/goal pairs with visualization...")
        print(f"   ⚙️ Configuration: visualization={'ON' if self.ENABLE_VISUALIZATION else 'OFF'}, FPS={self.VISUALIZATION_FPS}, timeout={self.TASK_TIMEOUT_SECONDS}s")
        
        # Initialize visualization with the first robot's state holder
        # We'll update this as robots are created
        self.visualization = None
        self.visualization_thread = None

        all_completed: list[tuple[str, str, bool, dict]] = []

        for i, (start_id, goal_id, path) in enumerate(pairs):
            print(f"\n🤖 Robot {i+1}/{len(pairs)}: {start_id} → {goal_id} (path length: {len(path)})")
            
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
            
            # Add robot to tracking list
            self.robots.append(robot)
            
            # Initialize visualization for this robot if not already done
            if self.visualization is None:
                self._setup_visualization_for_robots(self.robots)
            else:
                # Update visualization to track this robot
                self._update_visualization_robot(robot)

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
                print(f"   📍 Robot positioned at ({start_node.position.x:.2f}, {start_node.position.y:.2f})")

                task = robot.create_move_task(goal_node.position.x, goal_node.position.y)
                accepted = robot.assign_task(task)
                self.assertTrue(accepted, f"Robot should accept task {start_id}->{goal_id}")
                
                print(f"   🎯 Task assigned: move to ({goal_node.position.x:.2f}, {goal_node.position.y:.2f})")

                timeout_s = self.TASK_TIMEOUT_SECONDS
                start_time = time.time()
                completed = False
                last_status_time = start_time
                
                while time.time() - start_time < timeout_s:
                    status = robot.get_status()
                    current_time = time.time()
                    
                    # Print status updates every STATUS_UPDATE_INTERVAL seconds
                    if current_time - last_status_time >= self.STATUS_UPDATE_INTERVAL:
                        pos = status['position']
                        print(f"   📊 Status: pos=({pos[0]:.2f}, {pos[1]:.2f}), task_active={status['task_status'].has_active_task}")
                        last_status_time = current_time
                    
                    # Check if original task completed (either no active task or idle park task started)
                    current_task = status['task_status'].task_id
                    if not status['task_status'].has_active_task:
                        completed = True
                        break
                    elif current_task and current_task.startswith('idle_park_'):
                        # Original task completed and idle park task started - this is success
                        completed = True
                        break
                    time.sleep(0.1)

                dev = robot.lane_follower.get_lane_deviation_stats()
                print(f"   ✅ Task completed: {completed}")
                print(f"   📏 Lane deviation: max={dev.get('max_deviation', 0.0):.3f}m, last={dev.get('last_deviation', 0.0):.3f}m")
                print(f"[E2E-PARAM] {start_id}->{goal_id} path_len={len(path)} completed={completed} max_dev={dev.get('max_deviation', 0.0):.3f}m last_dev={dev.get('last_deviation', 0.0):.3f}m tol={dev.get('tolerance', 0.0):.3f}m")

                all_completed.append((start_id, goal_id, completed, dev))

                self.assertTrue(completed, f"Task {start_id}->{goal_id} did not complete")

            finally:
                robot.stop()

        # Optional overall assertion: all completed already checked individually
        self.assertTrue(all(c for _, _, c, _ in all_completed))
        
        print(f"\n🎉 All {len(all_completed)} robot tasks completed successfully!")
        print("   📊 Final Statistics:")
        for start_id, goal_id, completed, dev in all_completed:
            print(f"      {start_id} → {goal_id}: {'✅' if completed else '❌'} (max_dev: {dev.get('max_deviation', 0.0):.3f}m)")
    
    def test_multiple_start_goal_pairs_no_visualization(self) -> None:
        """Run the same test without visualization for faster execution."""
        # Temporarily disable visualization for this test
        original_setting = self.ENABLE_VISUALIZATION
        self.ENABLE_VISUALIZATION = False
        
        try:
            print("🚀 Running end-to-end test WITHOUT visualization for faster execution...")
            self.test_multiple_start_goal_pairs_complete()
        finally:
            # Restore original setting
            self.ENABLE_VISUALIZATION = original_setting
    
    def test_multiple_start_goal_pairs_with_verbose_logs(self) -> None:
        """Run the same test with verbose motion logging enabled for debugging."""
        # Temporarily enable verbose motion logging
        original_logging = self.ENABLE_VERBOSE_MOTION_LOGS
        self.ENABLE_VERBOSE_MOTION_LOGS = True
        
        try:
            print("🔍 Running end-to-end test WITH verbose motion logging for debugging...")
            # Reconfigure motion logging
            from robot.impl.motion_executor_impl import MotionExecutorImpl
            MotionExecutorImpl.set_verbose_logging(True)
            self.test_multiple_start_goal_pairs_complete()
        finally:
            # Restore original setting
            self.ENABLE_VERBOSE_MOTION_LOGS = original_logging
            from robot.impl.motion_executor_impl import MotionExecutorImpl
            MotionExecutorImpl.set_verbose_logging(False)


if __name__ == "__main__":
    # Configuration: Set ENABLE_VISUALIZATION = False for faster testing without visualization
    # Set ENABLE_VISUALIZATION = True to see real-time robot movements in MuJoCo viewer
    unittest.main()


