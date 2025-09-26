"""
MuJoCo Model Factory for Warehouse Visualization

Generates MuJoCo XML (MJCF) describing the warehouse scene and robot, based on
the `WarehouseMap`. This module is intentionally independent from the physics
engine to follow Single Responsibility Principle and allow the visualization to
operate decoupled from kinematic physics.
"""
from typing import Tuple

from warehouse.map import WarehouseMap
from interfaces.configuration_interface import RobotConfig


def generate_warehouse_mjcf(warehouse_map: WarehouseMap, physics_dt: float = 0.001,
                           robot_config: RobotConfig = None) -> str:
    """Generate a MuJoCo XML string describing the warehouse and a simple robot.

    Args:
        warehouse_map: The warehouse layout to visualize.
        physics_dt: Physics timestep to embed in the MJCF for viewer timing.
        robot_config: Robot configuration with physical dimensions. If None, uses defaults.

    Returns:
        MJCF XML as a string.
    """
    # Use provided robot config or create default
    if robot_config is None:
        from config.configuration_provider import ConfigurationProvider
        provider = ConfigurationProvider()
        robot_config = provider.get_robot_config("robot_1")

    # Extract robot dimensions
    robot_radius = robot_config.robot_width / 2.0  # Convert diameter to radius
    robot_height = robot_config.robot_height
    wheel_base = robot_config.wheel_base
    wheel_radius = robot_config.wheel_radius
    world_width = warehouse_map.width * warehouse_map.grid_size
    world_height = warehouse_map.height * warehouse_map.grid_size
    start_x = warehouse_map.start_position[0] - world_width / 2
    start_y = warehouse_map.start_position[1] - world_height / 2

    xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<mujoco model="simple_warehouse">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="{physics_dt}" gravity="0 0 0" integrator="Euler"/>
  
  <default>
    <geom type="box" rgba="0.8 0.8 0.8 1"/>
  </default>
  
  <worldbody>
    <!-- Floor centered at origin so physics (world) coords map directly -->
    <geom name="floor" type="plane" size="{world_width/2} {world_height/2} 0.1" 
          pos="0 0 0" rgba="0.9 0.9 0.9 1"/>
    
    <!-- Robot with differential drive (visualization-only) -->
    <body name="robot_base" pos="{start_x} {start_y} 0.1">
      <freejoint name="base_free"/>
      <geom name="robot_body" type="cylinder" size="{robot_radius} {robot_height/2}" rgba="1.0 0.9 0.0 1"/>
      <site name="robot_site" pos="0 0 0" size="0.02" rgba="1.0 1.0 0.0 1"/>
      
      <!-- Left wheel (proportional to robot size) -->
      <body name="left_wheel_body" pos="-{wheel_base/2} 0 0">
        <joint name="left_wheel" type="hinge" axis="0 1 0" limited="false"/>
        <geom name="left_wheel_geom" type="cylinder" size="{wheel_radius} {wheel_radius/2}" rgba="0.1 0.1 0.1 1"/>
      </body>

      <!-- Right wheel (proportional to robot size) -->
      <body name="right_wheel_body" pos="{wheel_base/2} 0 0">
        <joint name="right_wheel" type="hinge" axis="0 1 0" limited="false"/>
        <geom name="right_wheel_geom" type="cylinder" size="{wheel_radius} {wheel_radius/2}" rgba="0.1 0.1 0.1 1"/>
      </body>

      <!-- Orientation arrow/marker for visible heading -->
      <geom name="heading_marker" type="box" pos="{robot_radius + 0.02} 0 {robot_height/2 + 0.01}" size="0.04 0.01 0.02" rgba="1.0 0.2 0.2 1"/>
    </body>
    
    <!-- Warehouse elements -->
    {generate_warehouse_elements_xml(warehouse_map)}
  </worldbody>
  
  <actuator>
    <motor name="left_wheel_motor" joint="left_wheel" gear="1"/>
    <motor name="right_wheel_motor" joint="right_wheel" gear="1"/>
  </actuator>
  
  <sensor>
    <framepos name="robot_pos" objtype="site" objname="robot_site"/>
    <framequat name="robot_quat" objtype="site" objname="robot_site"/>
  </sensor>
</mujoco>"""
    return xml


def generate_warehouse_elements_xml(warehouse_map: WarehouseMap) -> str:
    """Generate the MJCF for static warehouse elements from the grid.

    Walls, shelves, charging/idle/drop-off zones are encoded as simple geoms.
    """
    elements_xml = ""
    grid = warehouse_map.grid
    grid_size = warehouse_map.grid_size
    height, width = grid.shape

    for y in range(height):
        for x in range(width):
            cell_type = int(grid[y, x])
            world_x = (x + 0.5) * grid_size - (warehouse_map.width * grid_size) / 2
            world_y = (y + 0.5) * grid_size - (warehouse_map.height * grid_size) / 2
            if cell_type == 1:  # Wall
                elements_xml += (
                    f'<geom name="wall_{x}_{y}" pos="{world_x} {world_y} 0.25" '
                    f'size="0.25 0.25 0.25" rgba="0.5 0.5 0.5 1" contype="1" conaffinity="1"/>'
                    "\n"
                )
            elif cell_type == 2:  # Shelf
                elements_xml += (
                    f'<geom name="shelf_{x}_{y}" pos="{world_x} {world_y} 0.6" '
                    f'size="0.22 0.22 0.6" rgba="0.2 0.8 0.2 1" contype="1" conaffinity="1"/>'
                    "\n"
                )
            elif cell_type == 3:  # Charging zone
                elements_xml += (
                    f'<geom name="charge_{x}_{y}" pos="{world_x} {world_y} 0.05" '
                    f'size="0.3 0.3 0.05" rgba="1.0 0.8 0.0 1" contype="1" conaffinity="1"/>'
                    "\n"
                )
            elif cell_type == 4:  # Idle zone
                elements_xml += (
                    f'<geom name="idle_{x}_{y}" pos="{world_x} {world_y} 0.05" '
                    f'size="0.3 0.3 0.05" rgba="0.5 0.8 1.0 1" contype="1" conaffinity="1"/>'
                    "\n"
                )
            elif cell_type == 5:  # Drop-off station
                elements_xml += (
                    f'<geom name="dropoff_{x}_{y}" pos="{world_x} {world_y} 0.05" '
                    f'size="0.3 0.3 0.05" rgba="1.0 0.4 0.4 1" contype="1" conaffinity="1"/>'
                    "\n"
                )
    return elements_xml


def yaw_to_quaternion(theta: float) -> Tuple[float, float, float, float]:
    """Convert yaw angle (rotation around Z) to a quaternion (w, x, y, z)."""
    import math
    w = math.cos(theta / 2.0)
    z = math.sin(theta / 2.0)
    return (w, 0.0, 0.0, z)


