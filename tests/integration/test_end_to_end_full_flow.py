"""
End-to-end flow test with DB:
- External script appends orders into a JSON file (order source)
- JobsProcessor reads orders, creates tasks, enqueues into JobsQueue
- RobotController processes bidding, assigns to RobotAgent
- Robot executes with DB-backed SimulationDataService (graph, shelves, inventory)

Windows note: set WAREHOUSE_DB_PASSWORD before running.
"""

import os
import json
import time
import math
import uuid
import shutil
import tempfile
import subprocess
import unittest
from pathlib import Path

from warehouse.map import WarehouseMap
from warehouse.impl.json_order_source import JsonOrderSource
from warehouse.impl.jobs_queue_impl import JobsQueueImpl
from warehouse.impl.robot_controller_impl import RobotController
from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
from warehouse.impl.jobs_processor_impl import JobsProcessorImpl
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent_lane_based import RobotAgent
from config.configuration_provider import ConfigurationProvider
from interfaces.lane_follower_interface import LaneFollowingConfig


def _write_external_order_script(script_path: Path) -> None:
    script = r"""
import json, sys, datetime
file_path = sys.argv[1]
order_id = sys.argv[2]
item_id = sys.argv[3]
# Write naive ISO datetime (no timezone) to avoid aware/naive comparison issues
now = datetime.datetime.now().isoformat()
order = {
  "order_id": order_id,
  "items": [{"item_id": item_id, "quantity": 1}],
  "scheduled_time": now,
  "priority": "normal",
  "metadata": {"source": "external_script"}
}
data = []
try:
    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
except Exception:
    data = []
data.append(order)
with open(file_path, 'w', encoding='utf-8') as f:
    json.dump(data, f)
print(f"Appended order {order_id} for {item_id}")
"""
    script_path.write_text(script, encoding='utf-8')


class TestEndToEndFullFlow(unittest.TestCase):
    def setUp(self) -> None:
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("WAREHOUSE_DB_PASSWORD not set; skipping DB-backed E2E flow test")

        # DB-backed services and map/graph
        self.warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
        self.svc = SimulationDataServiceImpl(
            warehouse_map=self.warehouse_map,
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD'),
            pool_size=5,
        )
        self.svc.create_shelves_from_map(clear_existing=True)
        self.svc.persist_navigation_graph_from_csv(Path("sample_warehouse.csv"), clear_existing=True)

        # Choose three shelves from the map and populate inventory for three items
        shelf_ids = list(self.warehouse_map.shelves.keys())[:3]
        items = [
            {"item_id": "item_A", "name": "Item A", "shelf_id": shelf_ids[0], "quantity": 5},
            {"item_id": "item_B", "name": "Item B", "shelf_id": shelf_ids[1], "quantity": 5},
            {"item_id": "item_C", "name": "Item C", "shelf_id": shelf_ids[2], "quantity": 5},
        ]
        self.svc.populate_inventory(items)

        # Temp orders JSON and external script
        self.temp_dir = Path(tempfile.mkdtemp(prefix="e2e_orders_"))
        self.orders_file = self.temp_dir / "orders.json"
        self.orders_file.write_text("[]", encoding='utf-8')
        self.external_script = self.temp_dir / "append_order.py"
        _write_external_order_script(self.external_script)

        # Order source and queue/processor
        self.order_source = JsonOrderSource(str(self.orders_file))
        self.order_source.connect()
        self.jobs_queue = JobsQueueImpl()
        self.jobs_processor = JobsProcessorImpl(
            order_source=self.order_source,
            simulation_data_service=self.svc,
            jobs_queue=self.jobs_queue,
        )

        # Robot + controller
        self.physics = SimpleMuJoCoPhysics(self.warehouse_map)
        self.config_provider = ConfigurationProvider()
        self.robot = RobotAgent(
            physics=self.physics,
            config_provider=self.config_provider,
            robot_id=f"robot_{uuid.uuid4().hex[:6]}",
            simulation_data_service=self.svc,
        )
        self.robot.start()
        time.sleep(0.3)
        self.robot.lane_follower.set_config(LaneFollowingConfig(lane_tolerance=10.0, max_speed=1.0, corner_speed=0.3))

        # Wrap robot to satisfy RobotController's legacy expectation of robot.config.robot_id
        class _RobotAdapter:
            def __init__(self, robot):
                self._robot = robot
                self.config = type("Cfg", (), {"robot_id": robot.robot_id})()
            def __getattr__(self, name):
                return getattr(self._robot, name)

        self.controller = RobotController(
            jobs_queue=self.jobs_queue,
            robot_pool=[_RobotAdapter(self.robot)],
            config_provider=self.config_provider,
            bidding_system=TransparentBiddingSystem(),
            polling_interval=0.5,
        )

    def tearDown(self) -> None:
        try:
            if hasattr(self, 'controller'):
                self.controller.stop()
        except Exception:
            pass
        try:
            if hasattr(self, 'robot'):
                self.robot.stop()
        except Exception:
            pass
        try:
            if hasattr(self, 'order_source'):
                self.order_source.disconnect()
        except Exception:
            pass
        try:
            if hasattr(self, 'svc'):
                self.svc.close()
        except Exception:
            pass
        try:
            if hasattr(self, 'temp_dir') and self.temp_dir.exists():
                shutil.rmtree(self.temp_dir, ignore_errors=True)
        except Exception:
            pass

    def _append_order_external(self, item_id: str) -> str:
        order_id = f"ord_{uuid.uuid4().hex[:8]}"
        cmd = [
            "python",
            str(self.external_script),
            str(self.orders_file),
            order_id,
            item_id,
        ]
        subprocess.check_call(cmd)
        return order_id

    def _process_orders_into_queue(self) -> None:
        # Force a processing pass to enqueue tasks
        self.jobs_processor.process_orders_once()

    def _run_controller_until_idle(self, timeout_s: float = 60.0) -> bool:
        start = time.time()
        completed = False
        while time.time() - start < timeout_s:
            self.controller.process_single_round()
            status = self.robot.get_status()
            # status['task_status'] is a dataclass-like object; check attribute directly
            task_status = status.get('task_status')
            has_active = bool(getattr(task_status, 'has_active_task', False))
            if not has_active:
                completed = True
                break
            time.sleep(0.2)
        return completed

    def test_full_flow_three_orders_via_external_script(self) -> None:
        items = ["item_A", "item_B", "item_C"]
        results = []
        for item in items:
            oid = self._append_order_external(item)
            # Ensure order source sees file change
            self.order_source.refresh()
            # Process into queue
            self._process_orders_into_queue()
            # Run controller/bidding/assignment → robot executes
            done = self._run_controller_until_idle(timeout_s=90.0)
            dev = self.robot.lane_follower.get_lane_deviation_stats()
            print(f"[FULL-E2E] order={oid} item={item} completed={done} max_dev={dev.get('max_deviation',0.0):.3f}m last_dev={dev.get('last_deviation',0.0):.3f}m")
            self.assertTrue(done, f"Order {oid} not completed")
            results.append((oid, item, done, dev))

        # All three should complete
        self.assertTrue(all(r[2] for r in results))


if __name__ == "__main__":
    unittest.main()


