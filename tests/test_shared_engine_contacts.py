import math

import pytest

from warehouse.map import WarehouseMap
from simulation.shared_mujoco_engine import SharedMuJoCoEngine, SharedMuJoCoPhysics


def _simple_map():
    # 10x10 empty map with one shelf ahead of robot 1
    m = WarehouseMap(width=10, height=10)
    # place a shelf at grid (3,3)
    m.grid[3, 3] = 2
    return m


def test_robot_blocks_on_shelf_contact_shared_world():
    m = _simple_map()
    engine = SharedMuJoCoEngine(m, physics_dt=0.001, enable_time_gating=False)
    # Register one robot facing +y near the shelf
    x0, y0 = m.grid_to_world(3, 2)
    engine.register_robot("r1", (x0, y0, math.pi / 2))
    engine.initialize()
    p1 = SharedMuJoCoPhysics(engine, robot_id="r1")

    # Command forward motion toward shelf
    p1.set_wheel_velocities(5.0, 5.0)
    initial = p1.get_robot_pose()
    for _ in range(200):
        p1.step_physics()
    final = p1.get_robot_pose()

    # Robot should be blocked before penetrating the shelf cell y=3
    shelf_y = m.grid_to_world(3, 3)[1]
    assert final[1] < shelf_y - 0.1
    # No sideways drift
    assert abs(final[0] - initial[0]) < 0.05


def test_robot_robot_nonpenetration_shared_world():
    m = WarehouseMap(width=10, height=10)
    engine = SharedMuJoCoEngine(m, physics_dt=0.001, enable_time_gating=False)
    # Two robots facing each other along +y / -y
    xA, yA = m.grid_to_world(5, 4)
    xB, yB = m.grid_to_world(5, 6)
    engine.register_robot("A", (xA, yA, math.pi / 2))
    engine.register_robot("B", (xB, yB, -math.pi / 2))
    engine.initialize()
    pA = SharedMuJoCoPhysics(engine, robot_id="A")
    pB = SharedMuJoCoPhysics(engine, robot_id="B")

    # Drive toward each other
    pA.set_wheel_velocities(5.0, 5.0)
    pB.set_wheel_velocities(5.0, 5.0)
    for _ in range(400):
        # Any adapter can invoke step; engine time-gates internally
        pA.step_physics()
        pB.step_physics()

    fA = pA.get_robot_pose()
    fB = pB.get_robot_pose()
    # Ensure they did not pass through each other; A is still below B in y
    assert fA[1] < fB[1] - 0.2


def test_attach_shelf_moves_with_robot_shared_world():
    m = WarehouseMap(width=10, height=10)
    # Place a shelf near (4,4)
    m.grid[4, 4] = 2
    engine = SharedMuJoCoEngine(m, physics_dt=0.001, enable_time_gating=False)
    x0, y0 = m.grid_to_world(4, 4)
    engine.register_robot("r1", (x0 - 0.4, y0, 0.0))
    engine.initialize()
    p = SharedMuJoCoPhysics(engine, robot_id="r1")

    # Attach the shelf by its body name in world
    shelf_id = "shelf_4_4"
    ok = p.attach_shelf_to_robot(shelf_id, (0.3, 0.0, 0.0))
    assert ok

    # Move forward; shelf should follow
    p.set_wheel_velocities(5.0, 5.0)
    for _ in range(300):
        p.step_physics()

    rx, ry, rt = p.get_robot_pose()
    # Read shelf pose from engine and verify relative offset in x (approx)
    sp = engine.get_body_pose(shelf_id)
    assert sp is not None
    sx, sy, st = sp
    assert abs((sx - rx) - 0.3) < 0.1


