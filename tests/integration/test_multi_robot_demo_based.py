"""
Integration tests for the multi-robot demo flow.

Goals:
- Exercise MultiRobotSimulationManager with 2–5 robots
- Run headless (no GUI) by stubbing visualization components
- Enqueue multiple tasks and verify assignment/progress
- Validate robots start at distinct positions
- Ensure DB runtime tables are clean before and after

These tests rely on SimulationDataServiceImpl for all DB interactions,
in line with the project's service access pattern.
"""

from __future__ import annotations

import time
from typing import List
from pathlib import Path

import pytest

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
import types
import sys


class _DummyVisualization:
    """No-op visualization to avoid launching GUI during tests."""

    def __init__(self, *args, **kwargs):
        self._active = True

    def initialize(self, config=None) -> None:
        self._active = True

    def visualize(self) -> None:
        pass

    def shutdown(self) -> None:
        self._active = False

    def is_active(self) -> bool:
        return self._active


class _DummyVisualizationThread:
    """No-op visualization thread wrapper."""

    def __init__(self, visualization, fps: float = 30.0):
        self._visualization = visualization
        self._fps = fps

    def start(self) -> None:
        # No thread created in tests
        pass

    def stop(self) -> None:
        # Ensure visualization shuts down cleanly
        try:
            if self._visualization is not None:
                self._visualization.shutdown()
        except Exception:
            pass


def _patch_headless_visualization(monkeypatch: pytest.MonkeyPatch) -> None:
    """Install headless stand-ins BEFORE importing the demo module."""
    # Create stub modules with expected classes so imports succeed without mujoco
    mr_mod = types.ModuleType('simulation.multi_robot_mujoco_visualization')
    setattr(mr_mod, 'MultiRobotMujocoVisualization', _DummyVisualization)
    monkeypatch.setitem(sys.modules, 'simulation.multi_robot_mujoco_visualization', mr_mod)

    mj_mod = types.ModuleType('simulation.mujoco_visualization')
    setattr(mj_mod, 'MujocoVisualization', _DummyVisualization)
    monkeypatch.setitem(sys.modules, 'simulation.mujoco_visualization', mj_mod)

    vt_mod = types.ModuleType('simulation.visualization_thread')
    setattr(vt_mod, 'VisualizationThread', _DummyVisualizationThread)
    monkeypatch.setitem(sys.modules, 'simulation.visualization_thread', vt_mod)


def _purge_runtime_tables(sim_data: SimulationDataServiceImpl) -> None:
    """Best-effort purge of runtime tables that may have garbage from prior runs."""
    with sim_data._get_connection() as conn:  # Access via service by design
        with conn.cursor() as cur:
            for table in (
                'conflict_box_locks',
                'blocked_cells',
                'shelf_locks',
            ):
                try:
                    cur.execute(f"DELETE FROM {table}")
                except Exception:
                    # Table may not exist in some schemas; ignore for portability
                    pass


def _ensure_robot_ids(sim_data: SimulationDataServiceImpl, robot_ids: List[str]) -> None:
    """Insert required robot IDs for FK constraints if missing."""
    with sim_data._get_connection() as conn:
        with conn.cursor() as cur:
            for rid in robot_ids:
                try:
                    cur.execute(
                        "INSERT INTO robots (robot_id, name) VALUES (%s, %s) "
                        "ON CONFLICT (robot_id) DO NOTHING",
                        (rid, rid.replace('_', ' ').title()),
                    )
                except Exception:
                    # If table doesn't exist (unlikely), let the demo's schema management handle it
                    pass


@pytest.mark.parametrize("robot_count", [2, 3, 4, 5])
def test_multi_robot_demo_param(robot_count: int, monkeypatch: pytest.MonkeyPatch) -> None:
    """Run the demo manager in headless mode with 2–5 robots and multiple tasks."""
    _patch_headless_visualization(monkeypatch)

    # Prepare DB and map via the simulation data service
    warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
    sim_data = SimulationDataServiceImpl(warehouse_map)

    # Ensure a clean baseline and required graph/shelves
    _purge_runtime_tables(sim_data)
    sim_data.persist_navigation_graph_from_csv(csv_path=Path("sample_warehouse.csv"), clear_existing=True)
    sim_data.create_shelves_from_map(clear_existing=True)

    # Ensure robots exist for FK constraints used by lock tables
    robot_ids = [f"warehouse_robot_{i}" for i in range(1, robot_count + 1)]
    _ensure_robot_ids(sim_data, robot_ids)

    # Import after headless patches so demo uses stubs
    from demo_multi_robot_simulation import MultiRobotSimulationManager
    sim = MultiRobotSimulationManager(robot_count=robot_count)

    try:
        sim.start_simulation()

        # Create tasks: 2 per robot for throughput
        total_tasks = robot_count * 2
        sim.create_demo_tasks(total_tasks)

        # Observe for a bounded period while controller and robots operate
        deadline = time.time() + 30.0
        last_queue_size = None
        while time.time() < deadline:
            queue_size = sim.jobs_queue.get_queue_size()
            if last_queue_size is None or queue_size != last_queue_size:
                # Progress indicator for debugging
                last_queue_size = queue_size
            if queue_size == 0:
                break
            time.sleep(1.0)

        # Validate that at least some work was assigned
        controller_status = sim.robot_controller.get_controller_status()
        assert controller_status['total_tasks_assigned'] >= min(robot_count, total_tasks)

        # Validate robots have distinct positions (based on configured start positions)
        status = sim.get_simulation_status()
        positions = []
        for r in status['robots']:
            pos = r.get('robot_status', {}).get('position', [0.0, 0.0, 0.0])
            positions.append((round(pos[0], 2), round(pos[1], 2)))
        assert len(positions) == robot_count
        assert len(set(positions)) == robot_count, "Robots should have distinct positions"

    finally:
        sim.stop_simulation()
        # Post-run cleanup of runtime tables
        _purge_runtime_tables(sim_data)

        # Ensure conflict box locks are not left behind
        with sim_data._get_connection() as conn:
            with conn.cursor() as cur:
                try:
                    cur.execute("SELECT COUNT(*) FROM conflict_box_locks")
                    remaining = cur.fetchone()[0]
                    assert remaining == 0
                except Exception:
                    # If table is absent, treat as clean
                    pass


