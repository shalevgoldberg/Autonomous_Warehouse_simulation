"""
Interface for MotionExecutor - executes motion plans via MuJoCo commands.
"""
from abc import ABC, abstractmethod
from typing import Tuple, Optional
from dataclasses import dataclass
from enum import Enum

from .path_planner_interface import Path, Cell


class MotionStatus(Enum):
    """Motion execution status."""
    IDLE = "idle"
    EXECUTING = "executing"
    REACHED_TARGET = "reached_target"
    BLOCKED = "blocked"
    ERROR = "error"


@dataclass
class MotionCommand:
    """Low-level motion command."""
    left_wheel_velocity: float
    right_wheel_velocity: float
    duration: float


class MotionExecutionError(Exception):
    """Raised when motion execution fails."""
    pass


class IMotionExecutor(ABC):
    """
    Interface for motion execution functionality.
    
    Responsibilities:
    - Execute motion plans step-by-step
    - Convert "move to next cell" into wheel torques (PID control)
    - Write low-level commands to MuJoCo
    - Handle motion safety and collision avoidance
    
    **Thread Safety**: All methods are thread-safe.
    **Threading Model**:
    - execute_*(), stop_execution(): CONTROL THREAD (called by TaskHandler)
    - update_control_loop(): CONTROL THREAD ONLY (100Hz within control thread)
    - get_*(), is_at_target(): ANY THREAD (thread-safe reads)
    - emergency_stop(): ANY THREAD (emergency use)
    """
    
    @abstractmethod
    def execute_path(self, path: Path) -> None:
        """
        Start executing a planned path.
        
        Args:
            path: Path to execute
            
        Raises:
            MotionExecutionError: If execution cannot be started
        """
        pass
    
    @abstractmethod
    def execute_single_move(self, target_cell: Cell) -> None:
        """
        Execute a single move to the target cell.
        
        Args:
            target_cell: Target cell to move to
            
        Raises:
            MotionExecutionError: If move cannot be executed
        """
        pass
    
    @abstractmethod
    def stop_execution(self) -> None:
        """
        Stop current motion execution immediately.
        """
        pass
    
    @abstractmethod
    def get_motion_status(self) -> MotionStatus:
        """
        Get current motion execution status.
        
        Returns:
            MotionStatus: Current status
        """
        pass
    
    @abstractmethod
    def is_at_target(self, target: Tuple[float, float], tolerance: float = 0.1) -> bool:
        """
        Check if robot has reached the target position.
        
        Args:
            target: Target position (x, y)
            tolerance: Position tolerance in meters
            
        Returns:
            bool: True if at target, False otherwise
        """
        pass
    
    @abstractmethod
    def get_current_target(self) -> Optional[Tuple[float, float]]:
        """
        Get the current target position being moved to.
        
        Returns:
            Optional[Tuple[float, float]]: Current target or None if idle
        """
        pass
    
    @abstractmethod
    def set_movement_speed(self, speed: float) -> None:
        """
        Set the movement speed for the robot.
        
        Args:
            speed: Movement speed in m/s
        """
        pass
    
    @abstractmethod
    def get_movement_speed(self) -> float:
        """
        Get current movement speed.
        
        Returns:
            float: Current movement speed in m/s
        """
        pass
    
    @abstractmethod
    def emergency_stop(self) -> None:
        """
        Emergency stop - immediately halt all motion.
        """
        pass
    
    @abstractmethod
    def update_control_loop(self) -> None:
        """
        Update the control loop - should be called at mid-frequency (100Hz).
        Handles PID control and sends commands to MuJoCo.
        
        **Atomicity**: All wheel commands must be written atomically to prevent
        race conditions with physics thread. Implementation must ensure physics
        thread never reads partially-written commands.
        """
        pass
    
    @abstractmethod
    def set_wheel_commands_atomic(self, left_vel: float, right_vel: float) -> None:
        """
        Set wheel velocities atomically to prevent race conditions.
        
        **Critical**: This method must ensure both wheel commands are written
        atomically relative to physics thread reads. Use appropriate locking
        or double-buffering to prevent partial command reads.
        
        Args:
            left_vel: Left wheel velocity command
            right_vel: Right wheel velocity command
        """
        pass

    @abstractmethod
    def clear_reached_target_flag(self) -> None:
        """
        Clear the reached target flag after TaskHandler has processed it.
        This allows the motion executor to return to normal status reporting.
        """
        pass 