"""
StateHolder Implementation - Thread-safe robot physics state management.

This implementation reads state directly from MuJoCo simulation and provides
thread-safe access to robot physics data for other components.
"""
import threading
import time
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass, replace

from interfaces.state_holder_interface import IStateHolder, RobotPhysicsState


class StateHolderImpl(IStateHolder):
    """
    Thread-safe implementation of StateHolder for MuJoCo integration.
    
    Threading Model:
    - update_from_simulation(): Called from PHYSICS THREAD (1kHz)
    - get_*() methods: Called from ANY THREAD (thread-safe reads)
    
    Uses double-buffering to ensure atomic reads of complete state.
    """
    
    def __init__(self, robot_id: str, model=None, data=None, physics_engine=None):
        """
        Initialize StateHolder.
        
        Args:
            robot_id: Unique identifier for this robot
            model: MuJoCo model (mjModel) - can be None for testing
            data: MuJoCo data (mjData) - can be None for testing
            physics_engine: Physics engine to get real robot position from
        """
        self.robot_id = robot_id
        self.model = model
        self.data = data
        self.physics_engine = physics_engine
        
        # Thread safety: Double buffering for atomic reads
        self._state_lock = threading.RLock()
        self._current_state = RobotPhysicsState(
            robot_id=robot_id,
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0),
            battery_level=1.0,
            timestamp=time.time()
        )
        
        # MuJoCo entity IDs (cached for performance)
        self._robot_body_id: Optional[int] = None
        self._left_wheel_joint_id: Optional[int] = None
        self._right_wheel_joint_id: Optional[int] = None
        
        # Robot physical parameters
        self._wheel_radius = 0.05  # meters
        self._wheel_base = 0.2     # meters between wheels
        
        # Battery simulation
        self._battery_drain_rate = 0.0001  # per simulation step
        
        # Initialize MuJoCo IDs if model is provided
        if self.model is not None:
            self._initialize_mujoco_ids()
    
    def _initialize_mujoco_ids(self) -> None:
        """Initialize MuJoCo entity IDs for fast access."""
        try:
            import mujoco
            
            # Get robot body ID
            self._robot_body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, "robot_base"
            )
            
            # Get wheel joint IDs
            self._left_wheel_joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel"
            )
            self._right_wheel_joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel"
            )
            
        except Exception as e:
            # Fallback for testing without MuJoCo
            print(f"[StateHolder] Warning: Could not initialize MuJoCo IDs: {e}")
            self._robot_body_id = 0
            self._left_wheel_joint_id = 0
            self._right_wheel_joint_id = 1
    
    def get_robot_state(self) -> RobotPhysicsState:
        """
        Get the current physics state of the robot.
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            RobotPhysicsState: Immutable snapshot of current physics state
        """
        with self._state_lock:
            # Return copy of current state (immutable)
            return replace(self._current_state)
    
    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current robot position (x, y, theta).
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            Tuple[float, float, float]: Current position and orientation
        """
        with self._state_lock:
            return self._current_state.position
    
    def get_velocity(self) -> Tuple[float, float]:
        """
        Get current robot velocity (linear, angular).
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            Tuple[float, float]: Current linear and angular velocities
        """
        with self._state_lock:
            return self._current_state.velocity
    
    def get_battery_level(self) -> float:
        """
        Get current battery level.
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            float: Battery level from 0.0 to 1.0
        """
        with self._state_lock:
            return self._current_state.battery_level
    
    def update_from_simulation(self) -> None:
        """
        Update state information from MuJoCo simulation.
        **PHYSICS THREAD ONLY**: Must be called from physics thread at 1kHz.
        **Not thread-safe**: Never call from multiple threads simultaneously.
        """
        # Read from MuJoCo (or simulate for testing)
        position = self._read_position_from_mujoco()
        velocity = self._read_velocity_from_mujoco()
        battery_level = self._update_battery_level()
        
        # Atomic update of complete state
        new_state = RobotPhysicsState(
            robot_id=self.robot_id,
            position=position,
            velocity=velocity,
            battery_level=battery_level,
            timestamp=time.time()
        )
        
        with self._state_lock:
            self._current_state = new_state
    
    def _read_position_from_mujoco(self) -> Tuple[float, float, float]:
        """Read robot position from physics engine or MuJoCo."""
        # First try to get position from physics engine (preferred)
        if self.physics_engine is not None:
            try:
                return self.physics_engine.get_robot_pose()
            except Exception as e:
                print(f"[StateHolder] Error reading from physics engine: {e}")
        
        # Fallback to MuJoCo if available
        if self.data is not None and self._robot_body_id is not None:
            try:
                # Read position from MuJoCo
                pos = self.data.xpos[self._robot_body_id]
                quat = self.data.xquat[self._robot_body_id]
                
                # Convert quaternion to euler angle (theta)
                theta = self._quaternion_to_euler(quat)
                
                return (float(pos[0]), float(pos[1]), float(theta))
                
            except Exception as e:
                print(f"[StateHolder] Error reading position from MuJoCo: {e}")
        
        # Testing mode: simulate small movement if no physics engine or MuJoCo
        if self.physics_engine is None and self.data is None:
            with self._state_lock:
                current_pos = self._current_state.position
                # Simulate small forward movement (0.001m per step)
                new_x = current_pos[0] + 0.001
                new_y = current_pos[1] + 0.001
                new_theta = current_pos[2] + 0.01  # Small rotation
                return (new_x, new_y, new_theta)
        
        # Last resort: return current position (no movement simulation)
        with self._state_lock:
            return self._current_state.position
    
    def _read_velocity_from_mujoco(self) -> Tuple[float, float]:
        """Read robot velocity from MuJoCo or simulate for testing."""
        if self.data is None or self._left_wheel_joint_id is None:
            # Testing mode - simulate velocity
            return (0.1, 0.0)  # Slow forward movement
        
        try:
            # Read wheel velocities
            left_vel = self.data.qvel[self._left_wheel_joint_id]
            right_vel = self.data.qvel[self._right_wheel_joint_id]
            
            # Convert to linear/angular velocity using differential drive kinematics
            linear_vel = (left_vel + right_vel) * self._wheel_radius / 2.0
            angular_vel = (right_vel - left_vel) * self._wheel_radius / self._wheel_base
            
            return (float(linear_vel), float(angular_vel))
            
        except Exception as e:
            print(f"[StateHolder] Error reading velocity from MuJoCo: {e}")
            with self._state_lock:
                return self._current_state.velocity
    
    def _update_battery_level(self) -> float:
        """Update battery level (simulated drain)."""
        with self._state_lock:
            current_battery = self._current_state.battery_level
            
        # Simple battery drain simulation
        new_battery = max(0.0, current_battery - self._battery_drain_rate)
        return new_battery
    
    def _quaternion_to_euler(self, quat) -> float:
        """
        Convert quaternion to euler angle (yaw/theta).
        
        Args:
            quat: Quaternion [w, x, y, z]
            
        Returns:
            float: Yaw angle in radians
        """
        import math
        
        w, x, y, z = quat
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    # Testing and debugging methods
    def set_test_position(self, position: Tuple[float, float, float]) -> None:
        """Set position for testing (not part of interface)."""
        with self._state_lock:
            self._current_state = replace(
                self._current_state,
                position=position,
                timestamp=time.time()
            )
    
    def set_test_velocity(self, velocity: Tuple[float, float]) -> None:
        """Set velocity for testing (not part of interface)."""
        with self._state_lock:
            self._current_state = replace(
                self._current_state,
                velocity=velocity,
                timestamp=time.time()
            ) 