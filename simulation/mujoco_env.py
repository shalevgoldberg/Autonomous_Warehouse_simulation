"""
Simple MuJoCo Physics Environment for Warehouse Simulation.

This implementation provides basic physics simulation with clean interfaces,
following our architecture principles: simple, thread-safe, and testable.
"""
import mujoco
import numpy as np
import threading
import time
from typing import Tuple, Dict, Any, Optional, List
from dataclasses import dataclass

from warehouse.map import WarehouseMap


@dataclass(frozen=True)
class PhysicsState:
    """Immutable physics state snapshot."""
    position: Tuple[float, float, float]  # x, y, theta
    velocity: Tuple[float, float, float]  # linear_x, linear_y, angular_z
    timestamp: float


class SimpleMuJoCoPhysics:
    """
    Simple MuJoCo physics integration for warehouse simulation.
    
    Design Principles:
    - **Simple**: No complex dynamics, direct position integration
    - **Thread-safe**: Atomic state updates
    - **Interface-driven**: Clean separation from robot logic
    - **Testable**: Mockable physics for unit tests
    
    Threading Model:
    - Physics step: Called at 1kHz from physics thread
    - State reads: Thread-safe from any thread
    - Wheel commands: Atomic updates from control thread
    """
    
    def __init__(self, warehouse_map: WarehouseMap):
        """
        Initialize simple MuJoCo physics.
        
        Args:
            warehouse_map: Warehouse layout for collision detection
        """
        self.warehouse_map = warehouse_map
        
        # Physics parameters
        self._physics_dt = 0.001  # 1kHz physics timestep
        self._wheel_radius = 0.05  # meters
        self._wheel_base = 0.3  # meters between wheels
        
        # Robot state (thread-safe)
        self._state_lock = threading.RLock()
        self._current_state = PhysicsState(
            position=(warehouse_map.start_position[0], warehouse_map.start_position[1], 0.0),
            velocity=(0.0, 0.0, 0.0),
            timestamp=time.time()
        )
        
        # Wheel commands (atomic)
        self._command_lock = threading.Lock()
        self._left_wheel_vel = 0.0
        self._right_wheel_vel = 0.0
        
        # MuJoCo model and data
        self.model = None
        self.data = None
        self._initialize_mujoco()
    
    def _initialize_mujoco(self) -> None:
        """
        Initialize MuJoCo model for visualization.
        **DISABLED**: MuJoCo visualization causes physics conflicts and stack overflow.
        Our kinematic physics works perfectly without it.
        """
        # DISABLED: MuJoCo visualization causes stack overflow and physics instability
        # The kinematic physics simulation works perfectly without MuJoCo dynamics
        self.model = None
        self.data = None
        print("[MuJoCo] Visualization disabled to prevent physics conflicts")
        return
        
        # OLD CODE (commented out to prevent conflicts):
        # try:
        #     xml_content = self._generate_warehouse_xml()
        #     self.model = mujoco.MjModel.from_xml_string(xml_content)
        #     self.data = mujoco.MjData(self.model)
        #     print(f"[MuJoCo] Initialized with {self.model.nq} DOF, {self.model.nbody} bodies")
        # except Exception as e:
        #     print(f"[MuJoCo] Failed to initialize: {e}")
        #     self.model = None
        #     self.data = None
    
    def _generate_warehouse_xml(self) -> str:
        """Generate MuJoCo XML from warehouse map."""
        # Calculate world size
        world_width = self.warehouse_map.width * self.warehouse_map.grid_size
        world_height = self.warehouse_map.height * self.warehouse_map.grid_size
        
        xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<mujoco model="simple_warehouse">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="{self._physics_dt}" gravity="0 0 0" integrator="Euler"/>
  
  <default>
    <geom type="box" rgba="0.8 0.8 0.8 1"/>
  </default>
  
  <worldbody>
    <!-- Floor -->
    <geom name="floor" type="plane" size="{world_width/2} {world_height/2} 0.1" 
          pos="{world_width/2} {world_height/2} 0" rgba="0.9 0.9 0.9 1"/>
    
    <!-- Robot with differential drive -->
    <body name="robot_base" pos="{self.warehouse_map.start_position[0]} {self.warehouse_map.start_position[1]} 0.1">
      <freejoint/>
      <geom name="robot_body" type="cylinder" size="0.1 0.05" rgba="0.8 0.2 0.2 1"/>
      <site name="robot_site" pos="0 0 0" size="0.01"/>
      
      <!-- Left wheel -->
      <body name="left_wheel_body" pos="-0.15 0 0">
        <joint name="left_wheel" type="hinge" axis="0 1 0" limited="false"/>
        <geom name="left_wheel_geom" type="cylinder" size="0.03 0.01" rgba="0.3 0.3 0.3 1"/>
      </body>
      
      <!-- Right wheel -->
      <body name="right_wheel_body" pos="0.15 0 0">
        <joint name="right_wheel" type="hinge" axis="0 1 0" limited="false"/>
        <geom name="right_wheel_geom" type="cylinder" size="0.03 0.01" rgba="0.3 0.3 0.3 1"/>
      </body>
    </body>
    
    <!-- Warehouse elements -->
    {self._generate_warehouse_elements_xml()}
  </worldbody>
  
  <actuator>
    <!-- Wheel motors for differential drive -->
    <motor name="left_wheel_motor" joint="left_wheel" gear="1"/>
    <motor name="right_wheel_motor" joint="right_wheel" gear="1"/>
  </actuator>
  
  <sensor>
    <framepos name="robot_pos" objtype="site" objname="robot_site"/>
    <framequat name="robot_quat" objtype="site" objname="robot_site"/>
  </sensor>
</mujoco>"""
        return xml
    
    def _generate_warehouse_elements_xml(self) -> str:
        """Generate XML for all warehouse elements from the grid."""
        elements_xml = ""
        
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                cell_type = self.warehouse_map.grid[y, x]
                world_x = (x + 0.5) * self.warehouse_map.grid_size
                world_y = (y + 0.5) * self.warehouse_map.grid_size
                
                if cell_type == 1:  # Wall
                    elements_xml += f'<geom name="wall_{x}_{y}" pos="{world_x} {world_y} 0.25" size="0.25 0.25 0.25" rgba="0.5 0.5 0.5 1"/>\n'
                elif cell_type == 2:  # Shelf
                    elements_xml += f'<geom name="shelf_{x}_{y}" pos="{world_x} {world_y} 0.5" size="0.2 0.2 0.5" rgba="0.2 0.8 0.2 1"/>\n'
                elif cell_type == 3:  # Charging zone
                    elements_xml += f'<geom name="charge_{x}_{y}" pos="{world_x} {world_y} 0.05" size="0.25 0.25 0.05" rgba="1.0 0.8 0.0 1"/>\n'
                elif cell_type == 4:  # Idle zone
                    elements_xml += f'<geom name="idle_{x}_{y}" pos="{world_x} {world_y} 0.05" size="0.25 0.25 0.05" rgba="0.5 0.8 1.0 1"/>\n'
                elif cell_type == 5:  # Drop-off station
                    elements_xml += f'<geom name="dropoff_{x}_{y}" pos="{world_x} {world_y} 0.1" size="0.25 0.25 0.1" rgba="1.0 0.4 0.4 1"/>\n'
        
        return elements_xml
    
    def set_wheel_velocities(self, left_vel: float, right_vel: float) -> None:
        """
        Set wheel velocities atomically.
        **Thread-safe**: Can be called from control thread.
        
        Args:
            left_vel: Left wheel velocity (rad/s)
            right_vel: Right wheel velocity (rad/s)
        """
        with self._command_lock:
            self._left_wheel_vel = left_vel
            self._right_wheel_vel = right_vel
            # Store for debugging
            self._last_wheel_commands = (left_vel, right_vel, time.time())
    
    def step_physics(self) -> None:
        """
        Step physics simulation (called at 1kHz).
        **Thread-safe**: Should only be called from physics thread.
        
        Performs simple kinematic integration without complex dynamics.
        """
        with self._command_lock:
            left_vel = self._left_wheel_vel
            right_vel = self._right_wheel_vel
        
        # Simple differential drive kinematics
        linear_vel = (left_vel + right_vel) * self._wheel_radius / 2.0
        angular_vel = (right_vel - left_vel) * self._wheel_radius / self._wheel_base
        
        with self._state_lock:
            current_pos = self._current_state.position
            x, y, theta = current_pos
            
            # Simple Euler integration
            new_theta = theta + angular_vel * self._physics_dt
            new_x = x + linear_vel * np.cos(new_theta) * self._physics_dt
            new_y = y + linear_vel * np.sin(new_theta) * self._physics_dt
            
            # Basic collision detection (stay within warehouse bounds)
            new_x = max(0.2, min(new_x, self.warehouse_map.width * self.warehouse_map.grid_size - 0.2))
            new_y = max(0.2, min(new_y, self.warehouse_map.height * self.warehouse_map.grid_size - 0.2))
            
            # Check collision with walls and shelves
            if not self.warehouse_map.is_walkable(new_x, new_y):
                # Collision detected, don't update position
                new_x, new_y = x, y
                linear_vel = 0.0
                angular_vel = 0.0
            
            # Update state
            self._current_state = PhysicsState(
                position=(new_x, new_y, new_theta),
                velocity=(linear_vel * np.cos(new_theta), linear_vel * np.sin(new_theta), angular_vel),
                timestamp=time.time()
            )
            
            # Update MuJoCo visualization if available
            if self.model is not None and self.data is not None:
                self._update_mujoco_visualization(new_x, new_y, new_theta)
    
    def _update_mujoco_visualization(self, x: float, y: float, theta: float) -> None:
        """
        Update MuJoCo model for visualization (internal use only).
        **DISABLED**: MuJoCo visualization disabled to prevent physics conflicts.
        """
        # DISABLED: No MuJoCo model to update
        return
        
        # OLD CODE (commented out):
        # try:
        #     # Update robot position in MuJoCo
        #     self.data.qpos[0] = x
        #     self.data.qpos[1] = y
        #     self.data.qpos[2] = 0.1  # Fixed height
        #     
        #     # Convert angle to quaternion (rotation around z-axis)
        #     self.data.qpos[3] = np.cos(theta / 2)  # w
        #     self.data.qpos[4] = 0.0                # x
        #     self.data.qpos[5] = 0.0                # y
        #     self.data.qpos[6] = np.sin(theta / 2)  # z
        #     
        #     # Zero velocities (we control position directly)
        #     self.data.qvel[:] = 0.0
        #     
        #     # Forward kinematics for visualization
        #     mujoco.mj_forward(self.model, self.data)
        #     
        # except Exception as e:
        #     # Fail silently for visualization issues
        #     pass
    
    def get_physics_state(self) -> PhysicsState:
        """
        Get current physics state.
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            PhysicsState: Immutable snapshot of current state
        """
        with self._state_lock:
            return self._current_state
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """
        Get robot pose (x, y, theta).
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            Tuple[float, float, float]: Current robot pose
        """
        with self._state_lock:
            return self._current_state.position
    
    def is_simulation_ready(self) -> bool:
        """Check if simulation is ready."""
        return self.model is not None and self.data is not None
    
    def reset_robot_position(self, x: float, y: float, theta: float = 0.0) -> None:
        """
        Reset robot to specified position.
        **Thread-safe**: Can be called from any thread.
        """
        with self._state_lock:
            self._current_state = PhysicsState(
                position=(x, y, theta),
                velocity=(0.0, 0.0, 0.0),
                timestamp=time.time()
            )
            
            # Update MuJoCo if available
            if self.model is not None and self.data is not None:
                self._update_mujoco_visualization(x, y, theta)


class MuJoCoWarehouseEnv:
    """
    Legacy compatibility wrapper for existing code.
    **Deprecated**: Use SimpleMuJoCoPhysics directly for new code.
    """
    
    def __init__(self, warehouse_map: WarehouseMap, robot_agent=None):
        """Initialize with legacy interface."""
        print("[MuJoCo] Using legacy wrapper - consider migrating to SimpleMuJoCoPhysics")
        self.warehouse_map = warehouse_map
        self.robot_agent = robot_agent
        
        # Use new simple physics engine
        self.physics = SimpleMuJoCoPhysics(warehouse_map)
        
        # Legacy properties
        self.model = self.physics.model
        self.data = self.physics.data
        self.dt = 0.001
        self.control_dt = 0.01
        self.wheel_radius = 0.05
        self.wheel_base = 0.3
    
    def step(self, left_wheel_speed: Optional[float] = None, 
             right_wheel_speed: Optional[float] = None, 
             target_angle: Optional[float] = None) -> dict:
        """Legacy step method."""
        if target_angle is not None:
            # Handle snap-to-angle (legacy feature)
            current_x, current_y, _ = self.physics.get_robot_pose()
            self.physics.reset_robot_position(current_x, current_y, target_angle)
            
            if self.robot_agent and hasattr(self.robot_agent, 'state_holder'):
                self.robot_agent.state_holder.update_pose(current_x, current_y, target_angle)
                self.robot_agent.state_holder.update_speed(0.0, 0.0)
        
        return {
                'robot_pos': [current_x, current_y, 0.1],
                'robot_theta': target_angle,
                'linear_speed': 0.0,
                'angular_speed': 0.0
            }
        
        # Normal wheel command processing
        left_vel = left_wheel_speed if left_wheel_speed is not None else 0.0
        right_vel = right_wheel_speed if right_wheel_speed is not None else 0.0
        
        self.physics.set_wheel_velocities(left_vel, right_vel)
        self.physics.step_physics()
        
        # Update robot agent if provided
        if self.robot_agent and hasattr(self.robot_agent, 'state_holder'):
            state = self.physics.get_physics_state()
            x, y, theta = state.position
            vx, vy, omega = state.velocity
            linear_speed = np.sqrt(vx*vx + vy*vy)
            
            self.robot_agent.state_holder.update_pose(x, y, theta)
            self.robot_agent.state_holder.update_speed(linear_speed, omega)
        
        state = self.physics.get_physics_state()
        return {
            'robot_pos': [state.position[0], state.position[1], 0.1],
            'robot_theta': state.position[2],
            'linear_speed': np.sqrt(state.velocity[0]**2 + state.velocity[1]**2),
            'angular_speed': state.velocity[2]
        }
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Legacy pose getter."""
        return self.physics.get_robot_pose()
    
    def render(self) -> None:
        """Legacy render method (no-op)."""
        pass 
