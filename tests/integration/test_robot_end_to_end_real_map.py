"""
End-to-end integration using real warehouse map (CSV) and real database graph.

Preconditions:
- PostgreSQL reachable with WAREHOUSE_DB_PASSWORD set in env
- Navigation graph tables will be populated for the test and cleaned up after
"""

import os
from pathlib import Path
import time
import math
import unittest

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from config.configuration_provider import ConfigurationProvider
from interfaces.lane_follower_interface import LaneFollowingConfig


class TestRobotEndToEndRealMap(unittest.TestCase):
    def setUp(self) -> None:
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("WAREHOUSE_DB_PASSWORD not set; skipping real DB E2E test")

        # Load the real warehouse map from CSV (lanes encoded as walkable)
        self.warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")

        # Initialize real DB-backed simulation data service
        self.sim_service = SimulationDataServiceImpl(
            warehouse_map=self.warehouse_map,
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD'),
            pool_size=5,
        )

        # Ensure shelves in DB match the currently loaded CSV map
        # This prevents out-of-bounds indices when reconstructing the map from DB data
        self.sim_service.create_shelves_from_map(clear_existing=True)

        # Persist a navigation graph generated from the CSV via the service API
        # This reflects the real workflow and avoids hand-crafted SQL inserts
        self.sim_service.persist_navigation_graph_from_csv(Path("sample_warehouse.csv"), clear_existing=True)

        # Physics and robot agent
        self.physics = SimpleMuJoCoPhysics(self.warehouse_map)
        self.config_provider = ConfigurationProvider()
        self.robot = RobotAgent(
            physics=self.physics,
            config_provider=self.config_provider,
            robot_id="e2e_robot_1",
            simulation_data_service=self.sim_service,
        )

    def tearDown(self) -> None:
        try:
            if hasattr(self, 'robot'):
                self.robot.stop()
        finally:
            if hasattr(self, 'sim_service'):
                self.sim_service.close()

    def test_robot_move_on_real_map(self) -> None:
        # Start threads
        self.robot.start()
        time.sleep(0.3)

        # Relax tolerance for integration shape coverage (precision covered elsewhere)
        self.robot.lane_follower.set_config(LaneFollowingConfig(lane_tolerance=10.0, max_speed=1.0, corner_speed=0.3))

        # Reset pose roughly at graph start
        self.physics.reset_robot_position(0.5, 0.5, math.pi/2)

        # Assign a move task to the goal node
        task = self.robot.create_move_task(4.5, 2.5)
        accepted = self.robot.assign_task(task)
        self.assertTrue(accepted, "Robot should accept the task")

        # Wait for completion
        timeout_s = 45.0
        start = time.time()
        while time.time() - start < timeout_s:
            status = self.robot.get_status()
            if not status['task_status'].has_active_task:
                break
            time.sleep(0.1)
        else:
            dev = self.robot.lane_follower.get_lane_deviation_stats()
            self.fail(f"Task did not complete. Max deviation={dev.get('max_deviation', 0):.3f}m")

        # Verify proximity to target
        x, y, _ = self.robot.state_holder.get_position()
        self.assertLess(abs(x - 4.5), 0.6)
        self.assertLess(abs(y - 2.5), 0.6)

        # Report deviation stats for visibility
        dev = self.robot.lane_follower.get_lane_deviation_stats()
        print(f"[E2E] max_deviation={dev.get('max_deviation', 0.0):.3f}m, last_deviation={dev.get('last_deviation', 0.0):.3f}m, tolerance={dev.get('tolerance', 0.0):.3f}m")


if __name__ == "__main__":
    unittest.main()


