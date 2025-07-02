"""
Physics Integration Adapters - Clean interfaces between robot components and MuJoCo.

These adapters maintain loose coupling by providing clean interfaces between
robot components and the physics simulation, following dependency injection principles.
"""
from abc import ABC, abstractmethod
from typing import Tuple, Protocol
import time

from simulation.mujoco_env import SimpleMuJoCoPhysics


class IPhysicsStateProvider(Protocol):
    """Protocol for physics state reading."""
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get robot pose (x, y, theta)."""
        ...
    
    def get_physics_state(self):
        """Get complete physics state."""
        ...


class IPhysicsCommandSink(Protocol):
    """Protocol for physics command writing."""
    
    def set_wheel_velocities(self, left_vel: float, right_vel: float) -> None:
        """Set wheel velocities."""
        ...
    
    def step_physics(self) -> None:
        """Step physics simulation."""
        ...


class PhysicsStateAdapter:
    """
    Adapter for reading state from MuJoCo physics.
    
    Provides clean interface for StateHolder to read robot state
    without direct coupling to MuJoCo implementation.
    """
    
    def __init__(self, physics: SimpleMuJoCoPhysics):
        """Initialize with physics simulation."""
        self.physics = physics
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get current robot pose from physics."""
        return self.physics.get_robot_pose()
    
    def get_physics_state(self):
        """Get complete physics state."""
        return self.physics.get_physics_state()
    
    def get_timestamp(self) -> float:
        """Get current simulation timestamp."""
        return time.time()


class PhysicsCommandAdapter:
    """
    Adapter for writing commands to MuJoCo physics.
    
    Provides clean interface for MotionExecutor to send wheel commands
    without direct coupling to MuJoCo implementation.
    """
    
    def __init__(self, physics: SimpleMuJoCoPhysics):
        """Initialize with physics simulation."""
        self.physics = physics
    
    def set_wheel_velocities(self, left_vel: float, right_vel: float) -> None:
        """Send wheel velocity commands to physics."""
        self.physics.set_wheel_velocities(left_vel, right_vel)
    
    def is_simulation_ready(self) -> bool:
        """Check if physics simulation is ready."""
        return self.physics.is_simulation_ready()


# Integration helper functions
def create_state_adapter(physics: SimpleMuJoCoPhysics) -> PhysicsStateAdapter:
    """Factory function for state adapter."""
    return PhysicsStateAdapter(physics)


def create_command_adapter(physics: SimpleMuJoCoPhysics) -> PhysicsCommandAdapter:
    """Factory function for command adapter."""
    return PhysicsCommandAdapter(physics) 