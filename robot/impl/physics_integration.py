"""
Physics Integration Adapters - Clean interfaces between robot components and MuJoCo.

These adapters maintain loose coupling by providing clean interfaces between
robot components and the physics simulation, following dependency injection principles.
"""
from abc import ABC, abstractmethod
from typing import Tuple, Protocol, Optional
import time

from simulation.mujoco_env import SimpleMuJoCoPhysics
from interfaces.state_holder_interface import IStateHolder


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


class IPhysicsThreadManager(Protocol):
    """Protocol for physics thread management."""
    def start(self) -> None:
        """Start the physics simulation thread."""
        ...
    def stop(self) -> None:
        """Stop the physics simulation thread."""
        ...
    def is_running(self) -> bool:
        """Check if the physics thread is running."""
        ...


class PhysicsThreadManager(IPhysicsThreadManager):
    """
    Manages the physics simulation thread for continuous physics updates.
    Follows SOLID principles and is interface-driven for testability and scalability.
    Enhanced with state synchronization:
    - Physics simulation at 1kHz
    - State holder updates at 1kHz
    - Database synchronization at configurable frequency
    """
    def __init__(self, physics: IPhysicsCommandSink, 
                 state_holder: Optional[IStateHolder] = None,
                 frequency_hz: float = 1000.0,
                 db_sync_frequency_hz: float = 10.0):
        self._physics = physics
        self._state_holder = state_holder
        self._frequency_hz = frequency_hz
        self._db_sync_frequency_hz = db_sync_frequency_hz
        self._running = False
        self._thread = None
        self._lock = None
        self._step_count = 0

    def start(self) -> None:
        import threading
        if self._running:
            return
        self._running = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._physics_loop, name="PhysicsThread", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._lock = None

    def is_running(self) -> bool:
        return self._running

    def _physics_loop(self):
        import time
        period = 1.0 / self._frequency_hz
        db_sync_period = 1.0 / self._db_sync_frequency_hz
        last_db_sync = time.perf_counter()
        next_step = time.perf_counter() + period
        
        while self._running:
            try:
                # Step physics simulation
                self._physics.step_physics()
                # Update state holder (1kHz synchronization)
                if self._state_holder is not None:
                    self._state_holder.update_from_simulation()
                # Database synchronization at lower frequency
                now = time.perf_counter()
                if now - last_db_sync >= db_sync_period:
                    self._sync_state_to_database()
                    last_db_sync = now
                self._step_count += 1
            except Exception as e:
                # Log only every 1000 errors to avoid I/O overhead
                if self._step_count % 1000 == 0:
                    print(f"[PhysicsThreadManager] Physics step error: {e}")
            # Drift-compensated sleep
            next_step += period
            sleep_time = next_step - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # If we're behind, skip sleep to catch up
                next_step = time.perf_counter() + period

    def _sync_state_to_database(self):
        """Synchronize current state to database."""
        if self._state_holder is None:
            return
        try:
            state = self._state_holder.get_robot_state()
            # TODO: Implement database synchronization
            # For now, do not log to avoid I/O overhead
            pass
        except Exception as e:
            # Log only every 1000 errors to avoid I/O overhead
            if self._step_count % 1000 == 0:
                print(f"[PhysicsThreadManager] Database sync error: {e}")


def create_physics_thread_manager(physics: IPhysicsCommandSink, 
                                state_holder: Optional[IStateHolder] = None,
                                frequency_hz: float = 1000.0,
                                db_sync_frequency_hz: float = 10.0) -> IPhysicsThreadManager:
    """Factory function for physics thread manager."""
    return PhysicsThreadManager(physics, state_holder, frequency_hz, db_sync_frequency_hz) 