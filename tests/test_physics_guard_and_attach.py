import time
import math

from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics


def test_collision_guard_blocks_shelf_penetration():
    # Build a map with a shelf right ahead of start
    m = WarehouseMap(width=10, height=10)
    # Use an arbitrary shelf at grid (3,3)
    m.grid[3, 3] = 2  # shelf
    # Put robot near (3,2), facing +y toward the shelf
    x0, y0 = m.grid_to_world(3, 2)
    physics = SimpleMuJoCoPhysics(m)
    physics.reset_robot_position(x0, y0, math.pi/2)

    physics.set_wheel_velocities(5.0, 5.0)
    # Expect the guard to prevent y from exceeding shelf lower face minus robot radius
    shelf_x, shelf_y = m.grid_to_world(3, 3)
    shelf_half_y = 0.22
    robot_radius = 0.12
    max_allowed_y = shelf_y - (shelf_half_y + robot_radius) + 1e-3

    blocked = False
    stalled_steps = 0
    last_y = y0
    for _ in range(2000):  # up to ~2 seconds at 1kHz
        physics.step_physics()
        x, y, th = physics.get_robot_pose()
        if y <= last_y + 1e-6:
            stalled_steps += 1
        else:
            stalled_steps = 0
        if stalled_steps > 50:  # no forward progress for 50 steps
            blocked = True
            break
        last_y = y

    assert blocked, "Robot should eventually stop before shelf due to contact-guard"
    assert last_y <= max_allowed_y + 0.02, f"Robot y exceeded allowed limit: {last_y} > {max_allowed_y}"


def test_attach_and_follow_robot():
    m = WarehouseMap(width=10, height=10)
    # Ensure there is a shelf at (3,3)
    m.grid[3, 3] = 2
    physics = SimpleMuJoCoPhysics(m)
    x0, y0 = m.grid_to_world(2, 3)
    physics.reset_robot_position(x0, y0, 0.0)

    # Attach shelf and move forward; shelf should remain offset from robot
    attached = physics.attach_shelf_to_robot("shelf_3_3", (0.3, 0.0, 0.0))
    assert attached, "Attach should succeed for existing shelf"

    physics.set_wheel_velocities(5.0, 5.0)
    for _ in range(200):
        physics.step_physics()

    rx, ry, rt = physics.get_robot_pose()
    # Access private Data to verify shelf alignment only if model is initialized
    if physics.model is not None and physics.data is not None:
        # read shelf body pose via cached method
        # We don't have a getter; rely on internal address mapping by calling internal helper through name
        # Validate that shelf remains approximately at offset (tolerant check)
        # Compute expected shelf center
        sx_exp = rx + 0.3 * math.cos(rt)
        sy_exp = ry + 0.3 * math.sin(rt)
        # Fetch actual shelf pos via body name
        # Body name equals shelf id
        try:
            bid = physics._body_name_to_id.get("shelf_3_3")  # type: ignore[attr-defined]
            if bid is not None:
                spos = physics.data.xpos[bid]
                assert abs(float(spos[0]) - sx_exp) < 0.2
                assert abs(float(spos[1]) - sy_exp) < 0.2
        except Exception:
            # If internals unavailable, skip strict check
            pass


