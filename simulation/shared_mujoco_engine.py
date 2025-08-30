"""
Shared MuJoCo Engine - Single world for multi-robot contact and shelf handling.

Design goals:
- One MuJoCo world shared by all robots (deterministic, scalable contact checks)
- Keep kinematic simplicity: integrate wheel commands to candidate poses
- Enforce non-penetration via MuJoCo contact queries (mj_forward) in shared scene
- Provide per-robot adapter compatible with existing physics interfaces

This is an incremental step toward full minimal-dynamics (Option B). It centralizes
scene management while preserving the current kinematic stepping model so that
robot code and interfaces remain unchanged. In a later phase we can switch stepping
from kinematic+contact-guard to mj_step with planar joints and velocity actuators
without changing external interfaces.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List, Set, Union

import numpy as np
import mujoco

from warehouse.map import WarehouseMap
from interfaces.appearance_service_interface import IAppearanceService
from interfaces.configuration_interface import IConfigurationProvider

# Type aliases for complex data structures
RobotPlanarQPosMap = Dict[str, Dict[str, int]]  # robot_id -> {'x': addr, 'y': addr, 'yaw': addr}
RobotActuatorMap = Dict[str, Dict[str, int]]    # robot_id -> {'x': id, 'y': id, 'yaw': id}


@dataclass(frozen=True)
class RobotKinematicState:
    position: Tuple[float, float, float]  # x, y, theta
    velocity: Tuple[float, float, float]  # vx_world, vy_world, omega
    timestamp: float


class SharedMuJoCoEngine:
    """
    Shared MuJoCo world managing all robots and shelves.

    Threading model:
    - step(): called by any robot's physics adapter (multiple threads possible)
      Internally synchronized, at most one world-step per dt (time-gated).
    - getters/setters: thread-safe.
    """

    def __init__(self, warehouse_map: WarehouseMap, physics_dt: float = 0.001, enable_time_gating: bool = False,
                 config_provider: Optional[IConfigurationProvider] = None):
        self._warehouse_map = warehouse_map
        self._dt = physics_dt
        self._lock = threading.RLock()
        self._model: Optional[mujoco.MjModel] = None
        self._data: Optional[mujoco.MjData] = None
        # Optional: enable conservative time-gating so that multiple adapters do not
        # oversubscribe the world stepping. Disabled by default to keep tests deterministic.
        self._enable_time_gating: bool = bool(enable_time_gating)
        self._config_provider = config_provider

        # Physics mode configuration (will be loaded from config)
        self._physics_mode: str = "kinematic_guard"  # Default mode
        self._physics_gravity: float = 0.0
        self._physics_velocity_damping: float = 0.1

        # Robot physical parameters (will be loaded from config)
        self._robot_wheel_radius: float = 0.05  # meters
        self._robot_wheel_base: float = 0.3     # meters

        # Load physics configuration
        self._load_physics_configuration()

        # Robot registry
        self._robots: Dict[str, Dict[str, object]] = {}
        # Per-robot command (left/right wheel rad/s)
        self._wheel_cmd: Dict[str, Tuple[float, float]] = {}
        # Shelf attachment removed in Phase 1 - now using logical attachment only
        # Per-robot detachment in progress flag to prevent race conditions
        self._detachment_in_progress: Dict[str, bool] = {}

        # Per-robot appearance state
        self._robot_carrying_appearance: Dict[str, bool] = {}
        self._appearance_service: Optional[IAppearanceService] = None

        # Cached ids with proper type hints
        self._body_name_to_id: Dict[str, int] = {}
        self._geom_name_to_id: Dict[str, int] = {}
        self._robot_geom_ids: Dict[str, Set[int]] = {}
        self._robot_free_qpos_adr: Dict[str, Optional[int]] = {}
        self._robot_planar_qpos_adr: RobotPlanarQPosMap = {}
        self._shelf_free_qpos_adr: Dict[str, int] = {}
        self._shelf_geom_ids_by_id: Dict[str, Set[int]] = {}
        self._robot_actuator_ids: RobotActuatorMap = {}

        # Timing state (used only when time-gating is enabled)
        self._last_step_time = time.perf_counter()

    # ---------------- World build & registration ----------------
    def register_robot(self, robot_id: str, initial_pose: Tuple[float, float, float]) -> None:
        with self._lock:
            if robot_id in self._robots:
                return
            # Store kinematic state and command
            self._robots[robot_id] = {
                "state": RobotKinematicState(position=initial_pose, velocity=(0.0, 0.0, 0.0), timestamp=time.time())
            }
            self._wheel_cmd[robot_id] = (0.0, 0.0)
            # Shelf attachment removed in Phase 1
            self._detachment_in_progress[robot_id] = False
            self._robot_carrying_appearance[robot_id] = False

    def initialize(self) -> None:
        """Build MuJoCo model with all registered robots and warehouse elements."""
        with self._lock:
            xml = self._build_world_xml()
            self._model = mujoco.MjModel.from_xml_string(xml)
            self._data = mujoco.MjData(self._model)
            self._cache_indices()
            # Write initial robot poses
            for robot_id in self._robots.keys():
                x, y, th = self._robots[robot_id]["state"].position  # type: ignore[index]
                self._write_robot_pose(robot_id, x, y, th)
            mujoco.mj_forward(self._model, self._data)

            # Initialize appearance service
            self._appearance_service = AppearanceService(self, self._warehouse_map, self._config_provider)

    def is_ready(self) -> bool:
        with self._lock:
            return self._model is not None and self._data is not None

    def get_appearance_service(self) -> Optional[IAppearanceService]:
        """Get the appearance service for managing robot visual appearance."""
        with self._lock:
            return self._appearance_service

    def get_physics_mode(self) -> str:
        """Get the current physics mode."""
        with self._lock:
            return self._physics_mode

    def set_physics_mode(self, mode: str) -> bool:
        """
        Set the physics mode (for testing/advanced configuration).

        Args:
            mode: Either "kinematic_guard" or "mujoco_authoritative"

        Returns:
            bool: True if mode was set successfully
        """
        with self._lock:
            if mode in ["kinematic_guard", "mujoco_authoritative"]:
                old_mode = self._physics_mode
                self._physics_mode = mode
                if hasattr(self, '_authoritative_mode_logged'):
                    delattr(self, '_authoritative_mode_logged')  # Reset logging flag
                print(f"[SharedMuJoCoEngine] Physics mode changed: {old_mode} → {mode}")
                return True
            else:
                print(f"[SharedMuJoCoEngine] WARNING: Invalid physics mode '{mode}'")
                return False

    def _load_physics_configuration(self) -> None:
        """Load physics configuration from provider or use defaults."""
        if self._config_provider is not None:
            try:
                physics_mode_val = self._config_provider.get_value("physics.mode", "mujoco_authoritative")
                self._physics_mode = physics_mode_val.value if hasattr(physics_mode_val, 'value') else physics_mode_val

                # Explicit logging of physics mode
                print(f"[SharedMuJoCoEngine] Physics mode loaded: {self._physics_mode}")
                if self._physics_mode == "mujoco_authoritative":
                    print(f"[SharedMuJoCoEngine] Using MuJoCo Authoritative Physics - Real physics solver active")
                elif self._physics_mode == "kinematic_guard":
                    print(f"[SharedMuJoCoEngine] Using Kinematic Guard Physics - Rule-based collision detection")

                gravity_val = self._config_provider.get_value("physics.gravity", 0.0)
                self._physics_gravity = gravity_val.value if hasattr(gravity_val, 'value') else gravity_val

                damping_val = self._config_provider.get_value("physics.velocity_damping", 0.1)
                self._physics_velocity_damping = damping_val.value if hasattr(damping_val, 'value') else damping_val

                # Load robot physical parameters
                wheel_radius_val = self._config_provider.get_value("robot.wheel_radius", 0.05)
                self._robot_wheel_radius = wheel_radius_val.value if hasattr(wheel_radius_val, 'value') else wheel_radius_val

                wheel_base_val = self._config_provider.get_value("robot.wheel_base", 0.3)
                self._robot_wheel_base = wheel_base_val.value if hasattr(wheel_base_val, 'value') else wheel_base_val

                # Validate physics mode
                if self._physics_mode not in ["kinematic_guard", "mujoco_authoritative"]:
                    print(f"[SharedMuJoCoEngine] WARNING: Invalid physics mode '{self._physics_mode}', using 'mujoco_authoritative'")
                    self._physics_mode = "mujoco_authoritative"

                # Validate robot parameters
                if self._robot_wheel_radius <= 0:
                    print(f"[SharedMuJoCoEngine] WARNING: Invalid wheel radius {self._robot_wheel_radius}, using default 0.05")
                    self._robot_wheel_radius = 0.05

                if self._robot_wheel_base <= 0:
                    print(f"[SharedMuJoCoEngine] WARNING: Invalid wheel base {self._robot_wheel_base}, using default 0.3")
                    self._robot_wheel_base = 0.3

            except Exception as e:
                print(f"[SharedMuJoCoEngine] WARNING: Failed to load physics config: {e}, using defaults")
        else:
            # Use defaults (already set in constructor)
            pass

    def _build_world_xml(self) -> str:
        wm = self._warehouse_map
        world_width = wm.width * wm.grid_size
        world_height = wm.height * wm.grid_size

        robots_xml: List[str] = []
        for idx, robot_id in enumerate(sorted(self._robots.keys())):
            stored_pos = self._robots[robot_id]["state"].position
            x, y, theta = stored_pos
            # print(f"[SharedMuJoCoEngine] DIAGNOSTIC: Building XML for {robot_id}, stored position: ({x:.3f}, {y:.3f}, {theta:.3f})")
            z = 0.10 + 0.01 * (idx % 5)
            body = f"robot_{robot_id}"
            if self._physics_mode == "mujoco_authoritative":
                # print(f"[SharedMuJoCoEngine] DIAGNOSTIC: Generating XML for {robot_id} body at (0,0,{z:.3f}), joints will handle positioning")
                robots_xml.append(
                    f"""
    <body name='{body}' pos='0 0 {z}'>
      <joint name='planar_x_{robot_id}' type='slide' axis='1 0 0' range='-10 10' damping='0.1'/>
      <joint name='planar_y_{robot_id}' type='slide' axis='0 1 0' range='-10 10' damping='0.1'/>
      <joint name='yaw_{robot_id}' type='hinge' axis='0 0 1' range='-3.14159 3.14159' damping='0.1'/>
      <geom name='robot_geom_{robot_id}' type='cylinder' size='0.12 0.06' rgba='0.8 0.2 0.2 1'/>
    </body>
                    """.strip()
                )
            else:
                free = f"free_{robot_id}"
                # One cylinder geom per robot for contact classification
                robots_xml.append(
                    f"""
    <body name='{body}' pos='{x} {y} {z}'>
      <freejoint name='{free}'/>
      <geom name='robot_geom_{robot_id}' type='cylinder' size='0.12 0.06' rgba='0.8 0.2 0.2 1'/>
    </body>
                    """.strip()
                )

        cells_xml = self._generate_cells_xml()
        robots_xml_joined = "\n    ".join(robots_xml)
        # Build actuators section if in authoritative mode
        actuators_xml = ""
        if self._physics_mode == "mujoco_authoritative":
            actuator_elems: List[str] = []
            for robot_id in sorted(self._robots.keys()):
                actuator_elems.append(
                    f"""
  <velocity name='act_x_{robot_id}' joint='planar_x_{robot_id}' kv='100.0'/>
  <velocity name='act_y_{robot_id}' joint='planar_y_{robot_id}' kv='100.0'/>
  <velocity name='act_yaw_{robot_id}' joint='yaw_{robot_id}' kv='15.0'/>
                    """.strip()
                )
            actuators_xml = "\n    ".join(actuator_elems)
        actuator_block = f"\n  <actuator>\n    {actuators_xml}\n  </actuator>\n" if actuators_xml else ""
        return f"""<?xml version="1.0" encoding="UTF-8"?>
<mujoco model='warehouse_shared_world'>
  <compiler angle='radian' coordinate='local'/>
  <option timestep='{self._dt}' gravity='0 0 0' integrator='Euler'/>
  <worldbody>
    <geom name='floor' type='plane' size='{world_width/2} {world_height/2} 0.1' pos='{world_width/2} {world_height/2} 0' rgba='0.9 0.9 0.9 1'/>
    {cells_xml}
    {robots_xml_joined}
  </worldbody>{actuator_block}</mujoco>
"""

    def _generate_cells_xml(self) -> str:
        wm = self._warehouse_map
        xml_parts: List[str] = []
        for y in range(wm.height):
            for x in range(wm.width):
                cell = int(wm.grid[y, x])
                world_x = (x + 0.5) * wm.grid_size
                world_y = (y + 0.5) * wm.grid_size
                if cell == 1:  # wall
                    xml_parts.append(self._geom_xml(f"wall_{x}_{y}", world_x, world_y, 0.25, 0.25, 0.25, "0.5 0.5 0.5 1"))
                elif cell == 2:  # shelf as movable body (freejoint) so we can weld-move
                    xml_parts.append(
                        """
    <body name='{name}' pos='{wx} {wy} {z}'>
      <freejoint name='free_{gx}_{gy}'/>
      <geom name='shelf_geom_{gx}_{gy}' type='box' size='0.22 0.22 0.6' rgba='0.2 0.8 0.2 1'/>
    </body>
                        """.strip().format(name=f"shelf_{x}_{y}", gx=x, gy=y, wx=world_x, wy=world_y, z=0.6)
                    )
                elif cell == 3:  # charging (visual-only marker)
                    xml_parts.append(self._geom_xml(f"charge_{x}_{y}", world_x, world_y, 0.3, 0.3, 0.05, "1.0 0.8 0.0 1", visual_only=True))
                elif cell == 4:  # idle (visual-only marker)
                    xml_parts.append(self._geom_xml(f"idle_{x}_{y}", world_x, world_y, 0.3, 0.3, 0.05, "0.5 0.8 1.0 1", visual_only=True))
                elif cell == 5:  # dropoff (visual-only marker)
                    xml_parts.append(self._geom_xml(f"dropoff_{x}_{y}", world_x, world_y, 0.3, 0.3, 0.1, "1.0 0.4 0.4 1", visual_only=True))
        return "\n    ".join(xml_parts)

    def _geom_xml(self, name: str, x: float, y: float, sx: float, sy: float, sz: float, rgba: str, visual_only: bool = False) -> str:
        """Create geometry XML with optional collision disabling for visual-only markers."""
        if visual_only:
            # Visual-only geometry: no collision detection
            return (
                f"<geom name='{name}' type='box' pos='{x} {y} {sz}' size='{sx} {sy} {sz}' rgba='{rgba}' "
                f"contype='0' conaffinity='0'/>"
            )
        else:
            # Physical geometry with collision detection
            return (
                f"<geom name='{name}' type='box' pos='{x} {y} {sz}' size='{sx} {sy} {sz}' rgba='{rgba}'/>"
            )

    def _cache_indices(self) -> None:
        assert self._model is not None
        # name maps
        self._body_name_to_id.clear()
        self._geom_name_to_id.clear()
        for i in range(int(self._model.nbody)):
            nm = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, i)
            if nm:
                self._body_name_to_id[nm] = int(i)
        for i in range(int(self._model.ngeom)):
            nm = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if nm:
                self._geom_name_to_id[nm] = int(i)
        # robots
        self._robot_geom_ids.clear()
        self._robot_free_qpos_adr.clear()
        self._robot_planar_qpos_adr.clear()
        self._robot_actuator_ids.clear()

        for robot_id in self._robots.keys():
            gname = f"robot_geom_{robot_id}"
            gid = self._geom_name_to_id.get(gname)
            if gid is not None:
                self._robot_geom_ids[robot_id] = {gid}
            else:
                self._robot_geom_ids[robot_id] = set()

            # Cache joint addresses based on physics mode
            if self._physics_mode == "mujoco_authoritative":
                # Cache planar joint addresses for authoritative mode
                try:
                    jx_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, f"planar_x_{robot_id}")
                    jy_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, f"planar_y_{robot_id}")
                    jyaw_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, f"yaw_{robot_id}")

                    if int(jx_id) >= 0 and int(jy_id) >= 0 and int(jyaw_id) >= 0:
                        x_addr = int(self._model.jnt_qposadr[jx_id])
                        y_addr = int(self._model.jnt_qposadr[jy_id])
                        yaw_addr = int(self._model.jnt_qposadr[jyaw_id])
                        self._robot_planar_qpos_adr[robot_id] = {
                            'x': x_addr,
                            'y': y_addr,
                            'yaw': yaw_addr
                        }
                        # print(f"[SharedMuJoCoEngine] DIAGNOSTIC: Set up planar joints for {robot_id}: x={x_addr}, y={y_addr}, yaw={yaw_addr}")
                    else:
                        self._robot_planar_qpos_adr[robot_id] = None
                        # print(f"[SharedMuJoCoEngine] DIAGNOSTIC: Failed to set up planar joints for {robot_id}: jx_id={int(jx_id)}, jy_id={int(jy_id)}, jyaw_id={int(jyaw_id)}")
                except Exception:
                    self._robot_planar_qpos_adr[robot_id] = None

                # Cache actuator addresses for authoritative mode
                try:
                    ax_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_x_{robot_id}")
                    ay_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_y_{robot_id}")
                    ayaw_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_yaw_{robot_id}")

                    if int(ax_id) >= 0 and int(ay_id) >= 0 and int(ayaw_id) >= 0:
                        self._robot_actuator_ids[robot_id] = {
                            'x': int(ax_id),
                            'y': int(ay_id),
                            'yaw': int(ayaw_id)
                        }
                    else:
                        self._robot_actuator_ids[robot_id] = None
                except Exception:
                    self._robot_actuator_ids[robot_id] = None
            else:
                # Cache free joint addresses for kinematic mode
                try:
                    jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, f"free_{robot_id}")
                    if int(jid) >= 0:
                        self._robot_free_qpos_adr[robot_id] = int(self._model.jnt_qposadr[jid])
                    else:
                        self._robot_free_qpos_adr[robot_id] = None
                except Exception:
                    self._robot_free_qpos_adr[robot_id] = None
        # shelves
        self._shelf_free_qpos_adr.clear()
        self._shelf_geom_ids_by_id.clear()
        for name, bid in self._body_name_to_id.items():
            if name.startswith("shelf_"):
                parts = name.split("_")
                if len(parts) == 3:
                    gx, gy = parts[1], parts[2]
                    gname = f"shelf_geom_{gx}_{gy}"
                    gid = self._geom_name_to_id.get(gname)
                    if gid is not None:
                        self._shelf_geom_ids_by_id.setdefault(name, set()).add(int(gid))
                try:
                    jname = f"free_{parts[1]}_{parts[2]}"
                    jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                    if int(jid) >= 0:
                        self._shelf_free_qpos_adr[name] = int(self._model.jnt_qposadr[jid])
                except Exception:
                    pass

    # ---------------- Per-robot APIs ----------------
    def set_wheel_velocities(self, robot_id: str, left: float, right: float) -> None:
        with self._lock:
            if robot_id in self._wheel_cmd:
                self._wheel_cmd[robot_id] = (left, right)

                # Wire to actuators in authoritative mode
                if self._physics_mode == "mujoco_authoritative":
                    # Always update actuators - even for zero velocities to ensure stopping
                    targets = self._wheel_velocities_to_actuator_targets(robot_id, left, right)
                    success = self._set_actuator_targets(robot_id, *targets)
                    if not success:
                        print(f"[SharedMuJoCoEngine] WARNING: Failed to set actuator targets for robot {robot_id}")

    def get_pose(self, robot_id: str) -> Tuple[float, float, float]:
        """
        Get the current pose of a robot.

        Thread-safe method that provides pose information with fallback mechanisms.
        In authoritative mode, tries to read from MuJoCo planar joints first,
        then falls back to kinematic state.

        Args:
            robot_id: The robot identifier

        Returns:
            Tuple[float, float, float]: (x, y, theta) current robot pose

        Raises:
            KeyError: If robot_id is not registered
        """
        with self._lock:
            # Validate robot exists
            if robot_id not in self._robots:
                raise KeyError(f"Robot {robot_id} is not registered")

            # In authoritative mode, try to read from planar joints first
            if self._physics_mode == "mujoco_authoritative":
                planar_pose = self._read_planar_pose(robot_id)
                if planar_pose is not None:
                    return planar_pose
                else:
                    # Log that planar pose reading failed, but continue with fallback
                    print(f"[SharedMuJoCoEngine] WARNING: Failed to read planar pose for robot {robot_id} in authoritative mode, using kinematic fallback")

            # Fallback to kinematic state (always available)
            robot_state = self._robots[robot_id]["state"]  # type: ignore[index]
            if not isinstance(robot_state, RobotKinematicState):
                raise RuntimeError(f"Invalid state type for robot {robot_id}: {type(robot_state)}")

            return robot_state.position

    def get_body_pose(self, body_name: str) -> Optional[Tuple[float, float, float]]:
        """Public helper: read a body's world pose (x,y,theta)."""
        with self._lock:
            if self._model is None or self._data is None:
                return None
            try:
                bid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                if int(bid) < 0:
                    return None
                pos = self._data.xpos[int(bid)]
                quat = self._data.xquat[int(bid)]
                # yaw from quaternion
                w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
                siny_cosp = 2 * (w * z + x * y)
                cosy_cosp = 1 - 2 * (y * y + z * z)
                import math
                yaw = math.atan2(siny_cosp, cosy_cosp)
                return (float(pos[0]), float(pos[1]), float(yaw))
            except Exception:
                return None

    def reset_pose(self, robot_id: str, x: float, y: float, theta: float) -> None:
        # print(f"[SharedMuJoCoEngine] DIAGNOSTIC: reset_pose called for {robot_id} with position ({x}, {y}, {theta})")
        with self._lock:
            self._robots[robot_id]["state"] = RobotKinematicState(
                position=(x, y, theta), velocity=(0.0, 0.0, 0.0), timestamp=time.time()
            )
            if self._model is not None and self._data is not None:
                # print(f"[SharedMuJoCoEngine] DIAGNOSTIC: Calling _write_robot_pose for {robot_id}")
                self._write_robot_pose(robot_id, x, y, theta)

                # Shelf attachment removed in Phase 1 - no longer align shelves with robots

                mujoco.mj_forward(self._model, self._data)
            else:
                pass  # Model not ready yet

        # Update all StateHolders that might be interested in this robot
        # This ensures the robot's position is immediately reflected in status reports
        if hasattr(self, '_state_holders_to_update'):
            for state_holder in self._state_holders_to_update:
                try:
                    state_holder.update_from_simulation()
                except Exception as e:
                    print(f"[SharedMuJoCoEngine] WARNING: Failed to update StateHolder: {e}")

    # attach_shelf method removed in Phase 1 - using logical attachment only

    # detach_shelf method removed in Phase 1 - using logical attachment only

    # ---------------- Stepping (kinematic + collision guard) ----------------
    def step(self) -> None:
        """Advance world by one dt at most, centralized across all robots."""
        with self._lock:
            if self._model is None or self._data is None:
                return
            # Optional conservative time-gating to avoid duplicate world steps
            if self._enable_time_gating:
                now = time.perf_counter()
                if (now - self._last_step_time) < self._dt:
                    return
                self._last_step_time = now

            # Handle physics modes
            if self._physics_mode == "mujoco_authoritative":
                self._step_mujoco_authoritative()
            else:  # kinematic_guard (default)
                self._step_kinematic_guard()

    def _step_kinematic_guard(self) -> None:
        """Step using kinematic physics with collision guard (current behavior)."""
        # For deterministic processing order
        robot_ids = sorted(self._robots.keys())

        for robot_id in robot_ids:
            # Read current state & command
            st = self._robots[robot_id]["state"]  # type: ignore[index]
            assert isinstance(st, RobotKinematicState)
            x, y, theta = st.position
            left, right = self._wheel_cmd.get(robot_id, (0.0, 0.0))

            # Differential drive kinematics (use configured parameters for consistency)
            wheel_radius = self._robot_wheel_radius
            wheel_base = self._robot_wheel_base
            linear_vel = (left + right) * wheel_radius / 2.0
            angular_vel = (right - left) * wheel_radius / wheel_base

            # Calculate movement using current heading (theta), then update heading
            attempted_x = x + linear_vel * np.cos(theta) * self._dt
            attempted_y = y + linear_vel * np.sin(theta) * self._dt
            new_theta = theta + angular_vel * self._dt

            # Clamp to bounds
            max_x = self._warehouse_map.width * self._warehouse_map.grid_size - 0.2
            max_y = self._warehouse_map.height * self._warehouse_map.grid_size - 0.2
            clamped_x = max(0.2, min(attempted_x, max_x))
            clamped_y = max(0.2, min(attempted_y, max_y))

            # Map walkability (logical obstacles)
            walkable = self._warehouse_map.is_walkable(clamped_x, clamped_y)
            # Physical contact check inside shared world
            blocked_by_contact = self._would_collide(robot_id, clamped_x, clamped_y, new_theta)

            # shelf_id reference removed in Phase 1

            if not walkable or blocked_by_contact:
                # Reject move
                accepted_x, accepted_y, accepted_theta = x, y, theta
                vx, vy, omg = 0.0, 0.0, 0.0
            else:
                accepted_x, accepted_y, accepted_theta = clamped_x, clamped_y, new_theta
                vx, vy, omg = linear_vel * np.cos(new_theta), linear_vel * np.sin(new_theta), angular_vel
                # Commit pose to world
                self._write_robot_pose(robot_id, accepted_x, accepted_y, accepted_theta)
                # Shelf attachment removed in Phase 1 - shelves remain stationary

            # Update internal state snapshot
            self._robots[robot_id]["state"] = RobotKinematicState(
                position=(accepted_x, accepted_y, accepted_theta),
                velocity=(vx, vy, omg),
                timestamp=time.time(),
            )

        mujoco.mj_forward(self._model, self._data)

    def _step_mujoco_authoritative(self) -> None:
        """
        Step using MuJoCo authoritative physics (Phase 2).

        Uses MuJoCo physics solver for realistic robot dynamics and contacts.
        No manual collision detection - MuJoCo handles all interactions.
        """
        # For now, log that we're in authoritative mode
        if not hasattr(self, '_authoritative_mode_logged'):
            print(f"[SharedMuJoCoEngine] Physics mode: {self._physics_mode} (using real MuJoCo physics)")
            self._authoritative_mode_logged = True

        # For deterministic processing order
        robot_ids = sorted(self._robots.keys())

        # Process wheel commands and set actuator targets
        for robot_id in robot_ids:
            left, right = self._wheel_cmd.get(robot_id, (0.0, 0.0))
            if left != 0.0 or right != 0.0:  # Only update if there's movement
                targets = self._wheel_velocities_to_actuator_targets(robot_id, left, right)
                success = self._set_actuator_targets(robot_id, *targets)
                if not success:
                    print(f"[SharedMuJoCoEngine] WARNING: Failed to set actuator targets for robot {robot_id}")

        # Run MuJoCo physics simulation (contacts handled by MuJoCo)
        mujoco.mj_step(self._model, self._data)

        # Read resulting poses and update internal state
        for robot_id in robot_ids:
            planar_pose = self._read_planar_pose(robot_id)
            if planar_pose is not None:
                x, y, theta = planar_pose
                # Update internal kinematic state for compatibility
                self._robots[robot_id]["state"] = RobotKinematicState(
                    position=(x, y, theta),
                    velocity=(0.0, 0.0, 0.0),  # Could compute from qvel if needed
                    timestamp=time.time(),
                )
            else:
                # Fallback to kinematic if planar pose unavailable
                print(f"[SharedMuJoCoEngine] WARNING: Could not read planar pose for robot {robot_id}, using kinematic state")

        # Shelf attachment handling removed in Phase 1 - shelves remain stationary

    # ---------------- Actuator Control Methods ----------------

    def _wheel_velocities_to_actuator_targets(self, robot_id: str, left_vel: float, right_vel: float) -> Tuple[float, float, float]:
        """
        Convert differential drive wheel velocities to planar actuator targets.

        This method implements differential drive kinematics to convert individual wheel
        velocities into planar joint velocities for MuJoCo actuation.

        Coordinate System Assumptions:
        - World coordinates: +X = East, +Y = North, +Z = Up
        - Robot local coordinates: +X = Forward, +Y = Left, +Z = Up
        - Pose angle (theta) is measured CCW from +X axis
        - Wheel velocities are in rad/s, positive = forward rotation

        Args:
            robot_id: The robot identifier
            left_vel: Left wheel velocity (rad/s)
            right_vel: Right wheel velocity (rad/s)

        Returns:
            Tuple[float, float, float]: (x_dot, y_dot, yaw_dot) in world coordinates (m/s, m/s, rad/s)

        Raises:
            ValueError: If wheel_base is zero or negative
        """
        # Validate robot parameters to prevent division by zero
        if self._robot_wheel_base <= 0:
            raise ValueError(f"Invalid wheel_base {self._robot_wheel_base} for robot {robot_id}. Must be positive.")

        if self._robot_wheel_radius <= 0:
            raise ValueError(f"Invalid wheel_radius {self._robot_wheel_radius} for robot {robot_id}. Must be positive.")

        # Get current pose for coordinate transformation
        current_pose = self.get_pose(robot_id)
        current_theta = current_pose[2] if current_pose else 0.0

        # Differential drive kinematics using configured parameters
        wheel_radius = self._robot_wheel_radius  # meters
        wheel_base = self._robot_wheel_base     # meters (distance between wheels)

        # Convert wheel velocities to linear and angular velocities
        # Linear velocity = average wheel velocity * wheel radius
        linear_vel = (left_vel + right_vel) * wheel_radius / 2.0
        # Angular velocity = wheel velocity difference * wheel radius / wheel base
        angular_vel = (right_vel - left_vel) * wheel_radius / wheel_base

        # Convert to world-frame velocities (planar coordinates)
        # MuJoCo's planar joints expect world-frame velocities
        target_x_dot = linear_vel * math.cos(current_theta)
        target_y_dot = linear_vel * math.sin(current_theta)
        target_yaw_dot = angular_vel

        return (target_x_dot, target_y_dot, target_yaw_dot)

    def _set_actuator_targets(self, robot_id: str, x_dot: float, y_dot: float, yaw_dot: float) -> bool:
        """
        Set velocity actuator targets for planar joints.

        Thread-safe method with defensive checks to prevent race conditions and data corruption.
        Validates input values and actuator configuration before setting targets.

        Args:
            robot_id: The robot identifier
            x_dot: Target x-axis velocity (m/s)
            y_dot: Target y-axis velocity (m/s)
            yaw_dot: Target yaw angular velocity (rad/s)

        Returns:
            bool: True if actuators were set successfully, False otherwise
        """
        # Defensive checks to prevent race conditions
        if self._data is None:
            return False

        # Safely get actuator IDs with validation
        actuator_ids = self._robot_actuator_ids.get(robot_id)
        if actuator_ids is None:
            return False

        # Validate that all required actuator IDs are present and valid
        required_keys = ['x', 'y', 'yaw']
        if not all(key in actuator_ids for key in required_keys):
            return False

        # Additional validation of actuator ID values
        for key in required_keys:
            act_id = actuator_ids[key]
            if act_id is None or act_id < 0 or act_id >= len(self._data.ctrl):
                return False

        # Validate input values are reasonable
        for val_name, val in [('x_dot', x_dot), ('y_dot', y_dot), ('yaw_dot', yaw_dot)]:
            if not (math.isfinite(val)):
                print(f"[SharedMuJoCoEngine] WARNING: Invalid actuator target {val_name}={val} for robot {robot_id}")
                return False

        try:
            # Atomic write of all actuator targets to ensure consistency
            self._data.ctrl[actuator_ids['x']] = x_dot
            self._data.ctrl[actuator_ids['y']] = y_dot
            self._data.ctrl[actuator_ids['yaw']] = yaw_dot
            return True
        except (IndexError, ValueError) as e:
            print(f"[SharedMuJoCoEngine] WARNING: Failed to set actuator targets for robot {robot_id}: {e}")
            return False
        except Exception as e:
            print(f"[SharedMuJoCoEngine] WARNING: Unexpected error setting actuator targets for robot {robot_id}: {e}")
            return False

    def _read_planar_pose(self, robot_id: str) -> Optional[Tuple[float, float, float]]:
        """
        Read pose from planar joint positions.

        This method is thread-safe when called from synchronized contexts.
        Includes defensive checks to prevent race conditions and data corruption.

        Args:
            robot_id: The robot identifier

        Returns:
            Optional[Tuple[float, float, float]]: (x, y, theta) or None if not available
        """
        # Defensive checks to prevent race conditions
        if self._data is None:
            return None

        # Safely get the qpos addresses with additional validation
        qpos_adr = self._robot_planar_qpos_adr.get(robot_id)
        if qpos_adr is None:
            print(f"[SharedMuJoCoEngine] ERROR: No planar qpos addresses found for robot {robot_id}")
            return None

        # Validate that all required joint addresses are present and valid
        required_keys = ['x', 'y', 'yaw']
        if not all(key in qpos_adr for key in required_keys):
            return None

        # Additional validation of address values
        for key in required_keys:
            addr = qpos_adr[key]
            if addr is None or addr < 0 or addr >= len(self._data.qpos):
                return None

        try:
            # Atomic read of all joint positions to ensure consistency
            x = float(self._data.qpos[qpos_adr['x']])
            y = float(self._data.qpos[qpos_adr['y']])
            yaw = float(self._data.qpos[qpos_adr['yaw']])

            # Validate that values are reasonable (not NaN or infinite)
            for val_name, val in [('x', x), ('y', y), ('yaw', yaw)]:
                if not (math.isfinite(val)):
                    print(f"[SharedMuJoCoEngine] ERROR: Invalid {val_name} value: {val} for robot {robot_id}")
                    return None

            # Log unusual positions that might indicate issues
            if abs(x) > 10.0 or abs(y) > 10.0:
                print(f"[SharedMuJoCoEngine] WARNING: Robot {robot_id} at unusual position ({x:.3f}, {y:.3f}, {yaw:.3f})")

            return (x, y, yaw)
        except (IndexError, ValueError, TypeError) as e:
            # Log the error for debugging but don't crash
            print(f"[SharedMuJoCoEngine] WARNING: Failed to read planar pose for robot {robot_id}: {e}")
            return None
        except Exception as e:
            # Catch any other unexpected exceptions
            print(f"[SharedMuJoCoEngine] WARNING: Unexpected error reading planar pose for robot {robot_id}: {e}")
            return None

    # ---------------- Internal helpers ----------------
    def _write_robot_pose(self, robot_id: str, x: float, y: float, theta: float) -> None:
        body = f"robot_{robot_id}"

        # Check which addressing mode we're using
        if self._physics_mode == "mujoco_authoritative":
            # In authoritative mode, the body is positioned in XML, joints should be absolute
            planar_adr = self._robot_planar_qpos_adr.get(robot_id)
            if planar_adr is not None:
                # Write to planar joint positions - joints are absolute positions since body is at (0,0,z)
                if self._data is not None and 'x' in planar_adr and 'y' in planar_adr and 'yaw' in planar_adr:
                    try:
                        self._data.qpos[planar_adr['x']] = x  # Absolute x position
                        self._data.qpos[planar_adr['y']] = y  # Absolute y position
                        self._data.qpos[planar_adr['yaw']] = theta  # Absolute yaw
                        print(f"[SharedMuJoCoEngine] Robot {robot_id} positioned at ({x:.3f}, {y:.3f}, {theta:.3f})")
                    except Exception as e:
                        print(f"[SharedMuJoCoEngine] ERROR: Failed to write planar pose for {robot_id}: {e}")
                else:
                    print(f"[SharedMuJoCoEngine] ERROR: Cannot write planar pose - data is None or missing joints")
            else:
                print(f"[SharedMuJoCoEngine] ERROR: No planar joints found for {robot_id}")
        else:
            # In kinematic mode, use free joints
            adr = self._robot_free_qpos_adr.get(robot_id)
            if adr is None:
                return
            self._write_qpos_at(adr, x, y, 0.1, theta)

    def _write_body_pose(self, body_name: str, x: float, y: float, theta: float) -> None:
        adr = self._shelf_free_qpos_adr.get(body_name)
        if adr is None:
            return
        self._write_qpos_at(adr, x, y, 0.6, theta)

    def _write_qpos_at(self, adr: int, x: float, y: float, z: float, theta: float) -> None:
        """
        Write pose data to qpos array at specified address.

        Thread-safe method with defensive checks to prevent data corruption.
        Writes 7DOF pose (x, y, z, quaternion) to the qpos array.

        Args:
            adr: Starting address in qpos array
            x: X position
            y: Y position
            z: Z position
            theta: Rotation angle around Z axis
        """
        if self._data is None:
            return

        # Validate address range
        if adr < 0 or adr + 6 >= len(self._data.qpos):
            print(f"[SharedMuJoCoEngine] WARNING: Invalid qpos address {adr}, ignoring write")
            return

        # Validate input values
        for val_name, val in [('x', x), ('y', y), ('z', z), ('theta', theta)]:
            if not math.isfinite(val):
                print(f"[SharedMuJoCoEngine] WARNING: Invalid {val_name}={val}, ignoring write")
                return

        try:
            half = theta / 2.0
            # Atomic write of all pose components
            self._data.qpos[adr + 0] = x
            self._data.qpos[adr + 1] = y
            self._data.qpos[adr + 2] = z
            self._data.qpos[adr + 3] = math.cos(half)
            self._data.qpos[adr + 4] = 0.0
            self._data.qpos[adr + 5] = 0.0
            self._data.qpos[adr + 6] = math.sin(half)
        except (IndexError, ValueError) as e:
            print(f"[SharedMuJoCoEngine] WARNING: Failed to write qpos at address {adr}: {e}")
        except Exception as e:
            print(f"[SharedMuJoCoEngine] WARNING: Unexpected error writing qpos at address {adr}: {e}")

    def _would_collide(self, robot_id: str, x: float, y: float, theta: float) -> bool:
        if self._model is None or self._data is None:
            return False

        # Write candidate pose for this robot
        prev_pose = self.get_pose(robot_id)
        adr = self._robot_free_qpos_adr.get(robot_id)
        if adr is None:
            return False

        self._write_qpos_at(adr, x, y, 0.1, theta)
        mujoco.mj_forward(self._model, self._data)

        try:
            # Contacts are symmetrical; look for any contact involving robot geoms
            my_geom_ids = self._robot_geom_ids.get(robot_id, set())
            # Shelf attachment removed in Phase 1 - no attached_ids to ignore

            for i in range(int(self._data.ncon)):
                c = self._data.contact[i]
                g1 = int(c.geom1)
                g2 = int(c.geom2)

                # if contact involves this robot
                robot_involved = (g1 in my_geom_ids) or (g2 in my_geom_ids)
                if not robot_involved:
                    continue

                other = g2 if (g1 in my_geom_ids) else g1

                # Shelf attachment removed in Phase 1 - no longer ignore attached shelves

                # classify other by name
                try:
                    other_name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_GEOM, other)
                except Exception:
                    other_name = None

                if not other_name:
                    return True

                if other_name.startswith("wall_") or other_name.startswith("shelf_") or other_name.startswith("shelf_geom_") or other_name.startswith("robot_geom_"):
                    # robot vs world/shelf/robot -> collision
                    return True

            return False

        finally:
            # Restore previous pose to avoid committing candidate on rejection
            self._write_robot_pose(robot_id, prev_pose[0], prev_pose[1], prev_pose[2])
            mujoco.mj_forward(self._model, self._data)


class AppearanceService(IAppearanceService):
    """
    Service for managing robot visual appearance changes.

    Provides thread-safe methods to toggle robot material/rgba when carrying shelves.
    Integrates with SharedMuJoCoEngine for visual feedback without affecting physics.
    """

    def __init__(self, engine: SharedMuJoCoEngine, warehouse_map: WarehouseMap,
                 config_provider: Optional[IConfigurationProvider] = None):
        """
        Initialize appearance service.

        Args:
            engine: The shared MuJoCo engine instance
            warehouse_map: Warehouse map for configuration access
            config_provider: Configuration provider for appearance settings
        """
        self._engine = engine
        self._warehouse_map = warehouse_map
        self._config_provider = config_provider
        self._lock = threading.RLock()

        # Load configuration
        self._load_configuration()

    def _load_configuration(self) -> None:
        """Load appearance configuration from provider or use defaults."""
        if self._config_provider is not None:
            try:
                enabled_val = self._config_provider.get_value("appearance.enabled", True)
                self._enabled = enabled_val.value if hasattr(enabled_val, 'value') else enabled_val

                carry_color_val = self._config_provider.get_value("appearance.carry_color", [1.0, 0.5, 0.0, 1.0])
                carry_color_list = carry_color_val.value if hasattr(carry_color_val, 'value') else carry_color_val

                normal_color_val = self._config_provider.get_value("appearance.normal_color", [0.8, 0.2, 0.2, 1.0])
                normal_color_list = normal_color_val.value if hasattr(normal_color_val, 'value') else normal_color_val

                self._carry_color = tuple(carry_color_list)
                self._normal_color = tuple(normal_color_list)
            except Exception:
                # Fall back to defaults on configuration error
                self._enabled = True
                self._carry_color = (1.0, 0.5, 0.0, 1.0)  # Orange RGBA
                self._normal_color = (0.8, 0.2, 0.2, 1.0)  # Default red
        else:
            # Use defaults
            self._enabled = True
            self._carry_color = (1.0, 0.5, 0.0, 1.0)  # Orange RGBA
            self._normal_color = (0.8, 0.2, 0.2, 1.0)  # Default red

    def configure(self, enabled: bool = True, carry_color: Optional[Tuple[float, float, float, float]] = None,
                  normal_color: Optional[Tuple[float, float, float, float]] = None) -> None:
        """
        Configure appearance service settings.

        Args:
            enabled: Whether appearance service is enabled
            carry_color: RGBA color for carrying state
            normal_color: RGBA color for normal state
        """
        with self._lock:
            self._enabled = enabled
            if carry_color is not None:
                self._carry_color = carry_color
            if normal_color is not None:
                self._normal_color = normal_color

    def set_carrying_appearance(self, robot_id: str, carrying: bool) -> bool:
        """
        Set robot appearance to indicate carrying state.

        Args:
            robot_id: The robot identifier
            carrying: True to show carrying appearance, False to revert to normal

        Returns:
            bool: True if appearance was successfully changed, False otherwise
        """
        if not self._enabled:
            return False

        with self._lock:
            if robot_id not in self._engine._robot_carrying_appearance:
                return False

            current_carrying = self._engine._robot_carrying_appearance[robot_id]
            if current_carrying == carrying:
                return True  # Already in desired state

            # Update appearance state
            self._engine._robot_carrying_appearance[robot_id] = carrying

            # Apply visual change if engine is ready
            if self._engine.is_ready():
                result = self._apply_robot_color(robot_id, carrying)
                if result:
                    color_name = "GREEN (carrying)" if carrying else "RED (normal)"
                    print(f"[AppearanceService] Robot {robot_id} changed to {color_name}")
                return result

            return True

    def is_carrying_appearance(self, robot_id: str) -> bool:
        """
        Check if robot currently has carrying appearance.

        Args:
            robot_id: The robot identifier

        Returns:
            bool: True if robot has carrying appearance, False otherwise
        """
        with self._lock:
            return self._engine._robot_carrying_appearance.get(robot_id, False)

    def get_carrying_color(self) -> Tuple[float, float, float, float]:
        """
        Get the RGBA color used for carrying appearance.

        Returns:
            Tuple[float, float, float, float]: RGBA values (0.0-1.0)
        """
        with self._lock:
            return self._carry_color

    def is_enabled(self) -> bool:
        """
        Check if appearance service is enabled.

        Returns:
            bool: True if service is enabled, False otherwise
        """
        with self._lock:
            return self._enabled

    def _apply_robot_color(self, robot_id: str, carrying: bool) -> bool:
        """
        Apply color change to robot geometry.

        Args:
            robot_id: The robot identifier
            carrying: True for carrying color, False for normal color

        Returns:
            bool: True if color was applied successfully
        """
        try:
            if self._engine._model is None or self._engine._data is None:
                return False

            # Get robot geometry IDs
            robot_geom_ids = self._engine._robot_geom_ids.get(robot_id, set())
            if not robot_geom_ids:
                return False

            # Choose color
            color = self._carry_color if carrying else self._normal_color

            # Apply color to all robot geometries
            for geom_id in robot_geom_ids:
                if geom_id < len(self._engine._model.geom_rgba):
                    self._engine._model.geom_rgba[geom_id] = np.array(color)

            return True

        except Exception:
            return False


class SharedMuJoCoPhysics:
    """
    Per-robot adapter exposing a SimpleMuJoCoPhysics-like API backed by SharedMuJoCoEngine.

    Only minimal surface compatible methods are provided to integrate with the existing robot
    stack without changes. Step calls are deduplicated via the engine's internal time-gating.
    """

    def __init__(self, engine: SharedMuJoCoEngine, robot_id: str = "robot_1"):
        self._engine = engine
        self.robot_id = robot_id

        # Initialize with warehouse default if available
        # NOTE: Caller must call reset_robot_position for explicit start pose
        self._engine.register_robot(robot_id, (0.0, 0.0, 0.0))
        # Engine initialization is coordinated by the caller (so all robots
        # are registered before building the world). Do not auto-initialize here.

        # Wheel commands
        self._cmd_lock = threading.Lock()
        self._left_wheel_vel = 0.0
        self._right_wheel_vel = 0.0

    # --- Compatibility surface ---
    @property
    def model(self):  # for StateHolder wiring
        return getattr(self._engine, "_model", None)

    @property
    def data(self):  # for StateHolder wiring
        return getattr(self._engine, "_data", None)

    def is_simulation_ready(self) -> bool:
        return self._engine.is_ready()

    def set_wheel_velocities(self, left_vel: float, right_vel: float) -> None:
        with self._cmd_lock:
            self._left_wheel_vel = left_vel
            self._right_wheel_vel = right_vel
        self._engine.set_wheel_velocities(self.robot_id, left_vel, right_vel)

    def step_physics(self) -> None:
        # Delegate a world step (time-gated inside engine)
        self._engine.step()

    def get_robot_pose(self) -> Tuple[float, float, float]:
        return self._engine.get_pose(self.robot_id)

    def get_pose(self) -> Tuple[float, float, float]:
        """Alias for get_robot_pose to match expected interface."""
        return self.get_robot_pose()

    def get_physics_state(self):
        pos = self._engine.get_pose(self.robot_id)
        # Derive simple velocity approximation is not required by current readers
        return RobotKinematicState(position=pos, velocity=(0.0, 0.0, 0.0), timestamp=time.time())

    def reset_robot_position(self, x: float, y: float, theta: float = 0.0) -> None:
        self._engine.reset_pose(self.robot_id, x, y, theta)

        # Immediately update the StateHolder to reflect the new position
        # This ensures status reports show the correct position
        if hasattr(self, '_state_holder') and self._state_holder is not None:
            try:
                self._state_holder.update_from_simulation()
            except Exception as e:
                print(f"[SharedMuJoCoPhysics] ERROR: Failed to update StateHolder after position reset: {e}")

    # Shelf attachment methods removed in Phase 1 - using logical attachment only


