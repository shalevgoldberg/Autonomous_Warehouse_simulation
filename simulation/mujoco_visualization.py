"""
MuJoCo-based visualization for warehouse simulation.

This implementation opens a real MuJoCo viewer window and updates the model's
pose from the robot's IStateHolder. Physics remains decoupled; this only
renders based on current state. A single viewer instance is reused; the
tracked robot can be switched via set_state_holder().
"""

from typing import Dict, Any, Optional
import math

import mujoco
from mujoco import viewer

from interfaces.visualization_interface import IVisualization, VisualizationError
from interfaces.state_holder_interface import IStateHolder
from warehouse.map import WarehouseMap


class MujocoVisualization(IVisualization):
    """
    MuJoCo viewer-based visualization for integration tests and demos.
    - Creates a MuJoCo model (floor + warehouse geoms + robot with freejoint)
    - Updates qpos from IStateHolder (x, y, theta -> xyz + quaternion)
    - Uses mujoco.viewer passive viewer and keeps a single window alive
    """
    
    def __init__(self, state_holder: IStateHolder, warehouse_map: WarehouseMap):
        self.state_holder = state_holder
        self.warehouse_map = warehouse_map
        self._active = False
        self._model: Optional[mujoco.MjModel] = None
        self._data: Optional[mujoco.MjData] = None
        self._viewer = None
        self._viewer_launched = False  # Guard against multiple windows
        # Cached ids and appearance state
        self._robot_geom_id: Optional[int] = None
        self._robot_dir_geom_id: Optional[int] = None
        self._last_carrying_state: Optional[bool] = None
    
    def set_state_holder(self, state_holder: IStateHolder) -> None:
        """Switch the tracked robot without recreating the viewer."""
        self.state_holder = state_holder
    
    def initialize(self, config: Optional[Dict[str, Any]] = None) -> None:
        try:
            xml = self._build_world_xml()
            self._model = mujoco.MjModel.from_xml_string(xml)
            self._data = mujoco.MjData(self._model)
            # Cache robot geom ids for fast color toggling
            try:
                self._robot_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "robot_body")
                self._robot_dir_geom_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, "robot_direction")
            except Exception:
                self._robot_geom_id = None
                self._robot_dir_geom_id = None
            # Set initial pose from current physics/state to avoid visual jump to MJCF default
            try:
                x, y, theta = self.state_holder.get_position()
                self._data.qpos[0] = x
                self._data.qpos[1] = y
                self._data.qpos[2] = 0.1
                half = theta / 2.0
                self._data.qpos[3] = math.cos(half)
                self._data.qpos[4] = 0.0
                self._data.qpos[5] = 0.0
                self._data.qpos[6] = math.sin(half)
                mujoco.mj_forward(self._model, self._data)
            except Exception:
                # Non-fatal: continue with default pose
                pass
            # Do NOT create the viewer here. It must be created and updated on the
            # same thread (the visualization thread). We will lazily launch it in visualize().
            self._active = True
            print("[MujocoVisualization] Initialized (model/data ready); viewer will be created on visualization thread")
        except Exception as e:
            raise VisualizationError(f"Failed to initialize MuJoCo visualization: {e}")
    
    def visualize(self) -> None:
        if not self._active or self._model is None or self._data is None:
            return
        try:
            # Lazily create the viewer once on the visualization thread
            if not self._viewer_launched:
                self._viewer = viewer.launch_passive(self._model, self._data)
                self._viewer_launched = True
            # Update pose from state holder
            x, y, theta = self.state_holder.get_position()
            # qpos layout: [x, y, z, qw, qx, qy, qz]
            self._data.qpos[0] = x
            self._data.qpos[1] = y
            self._data.qpos[2] = 0.1
            half = theta / 2.0
            self._data.qpos[3] = math.cos(half)
            self._data.qpos[4] = 0.0
            self._data.qpos[5] = 0.0
            self._data.qpos[6] = math.sin(half)
            # Apply appearance (bold color) based on carrying state without rebuilding model
            try:
                carrying = self._is_carrying_state()
                if carrying != self._last_carrying_state:
                    # Bright GREEN when carrying, RED when not
                    carry_rgba = (0.0, 1.0, 0.0, 1.0)
                    normal_rgba = (0.8, 0.2, 0.2, 1.0)
                    rgba = carry_rgba if carrying else normal_rgba
                    if self._robot_geom_id is not None and self._robot_geom_id >= 0:
                        self._model.geom_rgba[self._robot_geom_id] = rgba
                    if self._robot_dir_geom_id is not None and self._robot_dir_geom_id >= 0:
                        self._model.geom_rgba[self._robot_dir_geom_id] = rgba
                    self._last_carrying_state = carrying
                    # Avoid duplicate color-change logs; engine/TaskHandler already log
            except Exception:
                # Never fail visualize due to appearance update
                pass
            mujoco.mj_forward(self._model, self._data)
            # Ensure the passive viewer presents the updated frame
            if self._viewer is not None and hasattr(self._viewer, "sync"):
                try:
                    self._viewer.sync()
                except Exception:
                    pass
        except Exception as e:
            print(f"[MujocoVisualization] Visualization error: {e}")
    
    def shutdown(self) -> None:
        try:
            self._active = False
            # We cannot programmatically close the passive viewer; it will close when
            # the process exits or the user closes the window. Prevent relaunch.
            self._viewer = None
            self._viewer_launched = False
            self._model = None
            self._data = None
            print("[MujocoVisualization] MuJoCo viewer shutdown")
        except Exception as e:
            print(f"[MujocoVisualization] Shutdown error: {e}")
    
    def is_active(self) -> bool:
        return self._active
    
    def _build_world_xml(self) -> str:
        """Build MuJoCo XML with floor, warehouse cells, and a freejoint robot."""
        world_width = self.warehouse_map.width * self.warehouse_map.grid_size
        world_height = self.warehouse_map.height * self.warehouse_map.grid_size
        cells_xml = self._generate_cells_xml()
        return f"""<?xml version='1.0'?>
<mujoco model='warehouse_world'>
  <compiler angle='radian' coordinate='local'/>
  <option timestep='0.01' gravity='0 0 0' integrator='Euler'/>
  <worldbody>
    <geom name='floor' type='plane' size='{world_width/2} {world_height/2} 0.1' pos='{world_width/2} {world_height/2} 0' rgba='0.9 0.9 0.9 1'/>
    {cells_xml}
    <body name='robot_base' pos='{self.warehouse_map.grid_to_world(1, 1)[0]} {self.warehouse_map.grid_to_world(1, 1)[1]} 0.1'>
      <freejoint/>
      <geom name='robot_body' type='cylinder' size='0.1 0.05' rgba='0.8 0.2 0.2 1'/>
      <site name='robot_site' pos='0 0 0' size='0.01'/>
      <!-- Rotation indicator line pointing in the direction the robot is facing -->
      <geom name='robot_direction' type='cylinder' size='0.01 0.15' pos='0.15 0 0.05' rgba='1.0 1.0 0.0 1'/>
    </body>
  </worldbody>
</mujoco>
"""

    def _is_carrying_state(self) -> bool:
        """Infer carrying state from the physics engine attached to the state holder.
        Works for SimpleMuJoCoPhysics by checking attached shelf, and is safe when absent.
        """
        pe = getattr(self.state_holder, "physics_engine", None)
        if pe is None:
            return False
        # SimpleMuJoCoPhysics path: attached shelf implies carrying
        try:
            attached = getattr(pe, "_attached_shelf_id", None)
            if attached:
                return True
        except Exception:
            pass
        # SharedMuJoCoPhysics path: engine holds per-robot attached shelf ids
        try:
            engine = getattr(pe, "_engine", None)
            if engine is not None:
                rid = getattr(self.state_holder, "robot_id", None)
                if rid is not None:
                    # Prefer explicit appearance flag; fallback to attached shelf
                    try:
                        flag = engine._robot_carrying_appearance.get(rid, False)
                        if flag:
                            return True
                    except Exception:
                        pass
                    shelf = engine._attached_shelf_id.get(rid)
                    return shelf is not None
        except Exception:
            pass
        return False
    
    def _generate_cells_xml(self) -> str:
        """Generate MuJoCo geoms for walls, shelves, charging, idle, drop-offs from grid."""
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
                    xml_parts.append(self._geom_xml(f"dropoff_{x}_{y}", world_x, world_y, 0.25, 0.25, 0.05, "1.0 0.4 0.4 1"))
        return "\n    ".join(xml_parts)
    
    def _geom_xml(self, name: str, x: float, y: float, sx: float, sy: float, sz: float, rgba: str) -> str:
        return (
            f"<geom name='{name}' type='box' pos='{x} {y} {sz}' size='{sx} {sy} {sz}' rgba='{rgba}'/>"
        )
