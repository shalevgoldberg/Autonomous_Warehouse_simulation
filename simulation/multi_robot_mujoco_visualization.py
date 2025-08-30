"""
Multi-robot MuJoCo visualization.

Renders multiple robots in a single MuJoCo passive viewer by maintaining a
separate freejoint body for each robot and updating its qpos slice from the
robot's IStateHolder.

Threading model matches IVisualization expectations: model/data are created in
initialize(), the viewer is launched lazily in visualize() on the visualization
thread, and shutdown() clears resources.
"""

from typing import Dict, Any, Optional
import math

import mujoco
from mujoco import viewer

from interfaces.visualization_interface import IVisualization, VisualizationError
from interfaces.state_holder_interface import IStateHolder
from warehouse.map import WarehouseMap


class MultiRobotMujocoVisualization(IVisualization):
    """
    MuJoCo viewer-based visualization that supports multiple robots.

    - Creates a MuJoCo model (floor + warehouse geoms + one body per robot)
    - Updates each robot's qpos block from its IStateHolder
    - Uses mujoco.viewer passive viewer, single window
    """

    def __init__(self, state_holders: Dict[str, IStateHolder], warehouse_map: WarehouseMap):
        if not state_holders:
            raise VisualizationError("state_holders must be a non-empty dict of robot_id -> IStateHolder")
        self._state_holders: Dict[str, IStateHolder] = state_holders
        self.warehouse_map = warehouse_map
        self._active = False
        self._model: Optional[mujoco.MjModel] = None
        self._data: Optional[mujoco.MjData] = None
        self._viewer = None
        self._viewer_launched = False
        # Map robot_id -> qpos start index
        self._robot_qpos_addr: Dict[str, int] = {}
        # Map robot_id -> z offset to avoid visual overlap/z-fighting
        self._robot_z_offset: Dict[str, float] = {}
        # Appearance caches
        self._robot_geom_ids: Dict[str, int] = {}
        self._last_carrying_state: Dict[str, bool] = {}

    def initialize(self, config: Optional[Dict[str, Any]] = None) -> None:
        try:
            xml = self._build_world_xml()
            self._model = mujoco.MjModel.from_xml_string(xml)
            self._data = mujoco.MjData(self._model)

            # Diagnostics: verify expected robot bodies/joints exist
            try:
                expected = len(self._state_holders)
                print(f"[MultiRobotMujocoVisualization] Building visualization for {expected} robots: {list(self._state_holders.keys())}")
                for robot_id in self._state_holders.keys():
                    body_name = f"robot_{robot_id}"
                    bid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                    if int(bid) == -1:
                        print(f"[MultiRobotMujocoVisualization] Warning: body '{body_name}' not found in model")
            except Exception:
                pass

            # Resolve qpos base address for each robot using named freejoints
            for idx, robot_id in enumerate(self._state_holders.keys()):
                joint_name = f"joint_{robot_id}"
                # mj_name2id returns -1 when name not found (no exception)
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if int(jid) == -1:
                    raise VisualizationError(f"Joint '{joint_name}' not found in MuJoCo model")
                qpos_addr = int(self._model.jnt_qposadr[jid])
                self._robot_qpos_addr[robot_id] = qpos_addr
                # Assign a small per-robot z offset to ensure visibility separation
                self._robot_z_offset[robot_id] = 0.10 + 0.01 * (idx % 5)
                # Cache robot body geom id for appearance updates
                try:
                    gid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, f"robot_body_{robot_id}")
                    if int(gid) >= 0:
                        self._robot_geom_ids[robot_id] = int(gid)
                except Exception:
                    pass

            # Initialize each robot pose from its state holder
            for robot_id, holder in self._state_holders.items():
                try:
                    x, y, theta = holder.get_position()
                    self._write_robot_qpos(robot_id, x, y, theta)
                except Exception:
                    # Non-fatal: continue with default pose
                    pass
            mujoco.mj_forward(self._model, self._data)

            # Viewer will be created on visualization thread lazily
            self._active = True
            # Diagnostics: print resolved mapping and model sizes
            try:
                print(f"[MultiRobotMujocoVisualization] Model bodies={int(self._model.nbody)} joints={int(self._model.njnt)} qpos={int(self._model.nq)}")
                mapping_str = ", ".join(f"{rid}:{addr}" for rid, addr in self._robot_qpos_addr.items())
                print(f"[MultiRobotMujocoVisualization] qpos addr mapping: {mapping_str}")
            except Exception:
                pass
            print("[MultiRobotMujocoVisualization] Initialized; viewer will start on visualization thread")
        except Exception as e:
            raise VisualizationError(f"Failed to initialize multi-robot MuJoCo visualization: {e}")

    def visualize(self) -> None:
        if not self._active or self._model is None or self._data is None:
            return
        try:
            # Lazily create the viewer once on the visualization thread
            if not self._viewer_launched:
                self._viewer = viewer.launch_passive(self._model, self._data)
                self._viewer_launched = True
                # Standardize camera to top-down centered view for multi-robot visibility
                try:
                    world_width = self.warehouse_map.width * self.warehouse_map.grid_size
                    world_height = self.warehouse_map.height * self.warehouse_map.grid_size
                    if hasattr(self._viewer, 'cam'):
                        self._viewer.cam.lookat[0] = world_width / 2.0
                        self._viewer.cam.lookat[1] = world_height / 2.0
                        self._viewer.cam.lookat[2] = 0.0
                        self._viewer.cam.elevation = -90.0
                        self._viewer.cam.azimuth = 0.0
                        self._viewer.cam.distance = max(world_width, world_height) * 1.2
                except Exception:
                    pass

            # Update all robots' poses
            for robot_id, holder in self._state_holders.items():
                try:
                    x, y, theta = holder.get_position()
                    self._write_robot_qpos(robot_id, x, y, theta)
                    # Appearance: bright GREEN when carrying, otherwise base color (no change)
                    try:
                        carrying = self._is_carrying_state(holder)
                        last = self._last_carrying_state.get(robot_id)
                        if last is None or last != carrying:
                            gid = self._robot_geom_ids.get(robot_id)
                            if gid is not None:
                                carry_rgba = (0.0, 1.0, 0.0, 1.0)
                                if carrying:
                                    self._model.geom_rgba[gid] = carry_rgba
                                # else: leave original per-robot color
                            self._last_carrying_state[robot_id] = carrying
                            # Avoid duplicate color-change logs; engine/TaskHandler already log
                    except Exception:
                        pass
                except Exception:
                    # Keep rendering others even if one fails
                    pass

            mujoco.mj_forward(self._model, self._data)

            # Ensure the passive viewer presents the updated frame
            if self._viewer is not None and hasattr(self._viewer, "sync"):
                try:
                    self._viewer.sync()
                except Exception:
                    pass
        except Exception as e:
            print(f"[MultiRobotMujocoVisualization] Visualization error: {e}")

    def shutdown(self) -> None:
        try:
            self._active = False
            self._viewer = None
            self._viewer_launched = False
            self._model = None
            self._data = None
            print("[MultiRobotMujocoVisualization] MuJoCo viewer shutdown")
        except Exception as e:
            print(f"[MultiRobotMujocoVisualization] Shutdown error: {e}")

    def is_active(self) -> bool:
        return self._active

    # Internal helpers
    def _write_robot_qpos(self, robot_id: str, x: float, y: float, theta: float) -> None:
        addr = self._robot_qpos_addr.get(robot_id)
        if addr is None:
            return
        # qpos layout for a freejoint: [x, y, z, qw, qx, qy, qz]
        self._data.qpos[addr + 0] = x
        self._data.qpos[addr + 1] = y
        # Use per-robot z to avoid overlap and improve visibility
        self._data.qpos[addr + 2] = self._robot_z_offset.get(robot_id, 0.1)
        half = theta / 2.0
        self._data.qpos[addr + 3] = math.cos(half)
        self._data.qpos[addr + 4] = 0.0
        self._data.qpos[addr + 5] = 0.0
        self._data.qpos[addr + 6] = math.sin(half)

    def _is_carrying_state(self, state_holder: IStateHolder) -> bool:
        """Infer carrying state by checking appearance service flag, with fallback to attached shelf."""
        pe = getattr(state_holder, "physics_engine", None)
        if pe is None:
            return False

        # SharedMuJoCoPhysics path: check appearance service flag first
        try:
            engine = getattr(pe, "_engine", None)
            if engine is not None:
                rid = getattr(state_holder, "robot_id", None)
                if rid is not None:
                    # Prefer explicit appearance flag; fallback to attached shelf (Phase 1 compatibility)
                    try:
                        flag = engine._robot_carrying_appearance.get(rid, False)
                        if flag:
                            return True
                    except Exception:
                        pass
                    # Fallback to attached shelf for backward compatibility
                    try:
                        shelf = engine._attached_shelf_id.get(rid)
                        return shelf is not None
                    except Exception:
                        pass
        except Exception:
            pass

        # SimpleMuJoCoPhysics path: attached shelf implies carrying
        try:
            attached = getattr(pe, "_attached_shelf_id", None)
            if attached:
                return True
        except Exception:
            pass

        return False

    def _build_world_xml(self) -> str:
        """Build MuJoCo XML with floor, warehouse cells, and freejoint robot bodies."""
        world_width = self.warehouse_map.width * self.warehouse_map.grid_size
        world_height = self.warehouse_map.height * self.warehouse_map.grid_size
        cells_xml = self._generate_cells_xml()
        robots_xml = self._generate_robots_xml()
        # Place a fixed top-down camera centered on the floor to ensure all robots are in view
        cam_height = max(world_width, world_height) * 1.5
        return f"""<?xml version='1.0'?>
<mujoco model='warehouse_world_multi'>
  <compiler angle='radian' coordinate='local'/>
  <option timestep='0.01' gravity='0 0 0' integrator='Euler'/>
  <worldbody>
    <geom name='floor' type='plane' size='{world_width/2} {world_height/2} 0.1' pos='{world_width/2} {world_height/2} 0' rgba='0.9 0.9 0.9 1'/>
    <camera name='topdown' mode='fixed' pos='{world_width/2} {world_height/2} {cam_height}' euler='-1.5708 0 0'/>
    {cells_xml}
    {robots_xml}
  </worldbody>
</mujoco>
"""

    def _generate_cells_xml(self) -> str:
        grid = getattr(self.warehouse_map, 'grid', None)
        if grid is None:
            return ""
        width = self.warehouse_map.width
        height = self.warehouse_map.height
        grid_size = self.warehouse_map.grid_size
        xml_parts = []
        for y in range(height):
            for x in range(width):
                cell_type = grid[y, x]
                if cell_type == 0:
                    continue
                world_x = (x + 0.5) * grid_size
                world_y = (y + 0.5) * grid_size
                if cell_type == 1:  # Wall
                    xml_parts.append(self._geom_xml(f"wall_{x}_{y}", world_x, world_y, 0.25, 0.25, 0.25, "0.5 0.5 0.5 1"))
                elif cell_type == 2:  # Shelf
                    xml_parts.append(self._geom_xml(f"shelf_{x}_{y}", world_x, world_y, 0.2, 0.2, 0.5, "0.2 0.8 0.2 1"))
                elif cell_type == 3:  # Charging zone
                    xml_parts.append(self._geom_xml(f"charge_{x}_{y}", world_x, world_y, 0.25, 0.25, 0.05, "1.0 0.8 0.0 1"))
                elif cell_type == 4:  # Idle zone
                    xml_parts.append(self._geom_xml(f"idle_{x}_{y}", world_x, world_y, 0.25, 0.25, 0.05, "0.5 0.8 1.0 1"))
                elif cell_type == 5:  # Drop-off station
                    xml_parts.append(self._geom_xml(f"dropoff_{x}_{y}", world_x, world_y, 0.25, 0.25, 0.1, "1.0 0.4 0.4 1"))
        return "\n    ".join(xml_parts)

    def _generate_robots_xml(self) -> str:
        # Distinct colors for multiple robots (cycles if more requested)
        robot_colors = [
            "0.8 0.2 0.2 1",  # red
            "0.2 0.2 0.8 1",  # blue
            "0.2 0.8 0.2 1",  # green
            "0.0 0.0 0.0 1",  # black (replaces yellow for max contrast)
            "0.8 0.2 0.8 1",  # magenta
            "0.2 0.8 0.8 1",  # cyan
        ]
        xml_parts = []
        for idx, robot_id in enumerate(self._state_holders.keys()):
            color = robot_colors[idx % len(robot_colors)]
            body_name = f"robot_{robot_id}"
            joint_name = f"joint_{robot_id}"
            site_name = f"robot_site_{robot_id}"
            # Use each robot's current pose as initial body position
            try:
                x, y, _ = self._state_holders[robot_id].get_position()
            except Exception:
                x, y = self.warehouse_map.start_position
            z = 0.10 + 0.01 * (idx % 5)
            xml_parts.append(
                f"""
    <body name='{body_name}' pos='{x} {y} {z}'>
      <freejoint name='{joint_name}'/>
      <geom name='robot_body_{robot_id}' type='cylinder' size='0.1 0.05' rgba='{color}'/>
      <site name='{site_name}' pos='0 0 0' size='0.01'/>
      <geom name='robot_direction_{robot_id}' type='cylinder' size='0.01 0.15' pos='0.15 0 0.05' rgba='1.0 1.0 0.0 1'/>
    </body>
                """.strip()
            )
        return "\n    ".join(xml_parts)

    def _geom_xml(self, name: str, x: float, y: float, sx: float, sy: float, sz: float, rgba: str) -> str:
        return (
            f"<geom name='{name}' type='box' pos='{x} {y} {sz}' size='{sx} {sy} {sz}' rgba='{rgba}'/>"
        )



