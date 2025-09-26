"""
Interface for StateHolder - maintains robot state information.
"""
from abc import ABC, abstractmethod
from typing import Tuple, Optional, Dict, Any, TYPE_CHECKING
from dataclasses import dataclass
from enum import Enum

# Forward reference for LiDARScan to avoid circular imports
if TYPE_CHECKING:
    from interfaces.lidar_interface import LiDARScan


@dataclass
class RobotPhysicsState:
    """Physics-related robot state information."""
    robot_id: str
    position: Tuple[float, float, float]  # x, y, theta
    velocity: Tuple[float, float]  # linear, angular
    battery_level: float  # 0.0 to 1.0
    timestamp: float = 0.0
    additional_data: Optional[Dict[str, Any]] = None

    # LiDAR sensor data (Phase 2.1)
    lidar_scan: Optional['LiDARScan'] = None  # Latest LiDAR scan data


class IStateHolder(ABC):
    """
    Interface for managing robot state information.
    
    Responsibilities:
    - Maintain up-to-date robot status (pose, speed, battery, current task)
    - Read data directly from MuJoCo simulation
    - Provide thread-safe access to state information
    
    **Thread Safety**: All getter methods are thread-safe for concurrent reads.
    **Threading Model**:
    - update_from_simulation(): PHYSICS THREAD ONLY (1kHz)
    - get_*() methods: ANY THREAD (Control, Visualization)
    """
    
    @abstractmethod
    def get_robot_state(self) -> RobotPhysicsState:
        """
        Get the current physics state of the robot.
        **Thread-safe**: Can be called from any thread.
        
        Returns:
            RobotPhysicsState: Immutable snapshot of current physics state
        """
        pass
    
    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current robot position (x, y, theta).
        
        Returns:
            Tuple[float, float, float]: Current position and orientation
        """
        pass
    
    @abstractmethod
    def get_velocity(self) -> Tuple[float, float]:
        """
        Get current robot velocity (linear, angular).
        
        Returns:
            Tuple[float, float]: Current linear and angular velocities
        """
        pass
    
    @abstractmethod
    def get_battery_level(self) -> float:
        """
        Get current battery level.
        
        Returns:
            float: Battery level from 0.0 to 1.0
        """
        pass
    
    @abstractmethod
    def update_from_simulation(self) -> None:
        """
        Update state information from MuJoCo simulation.
        **PHYSICS THREAD ONLY**: Must be called from physics thread at 1kHz.
        **Not thread-safe**: Never call from multiple threads simultaneously.
        """
        pass

    # LiDAR sensor data access (Phase 2.1)
    @abstractmethod
    def get_lidar_scan(self) -> Optional['LiDARScan']:
        """
        Get the latest LiDAR scan data.
        **Thread-safe**: Can be called from any thread.

        Returns:
            LiDARScan: Latest LiDAR scan data, or None if not available
        """
        pass

    @abstractmethod
    def update_lidar_scan(self, scan: 'LiDARScan') -> None:
        """
        Update LiDAR scan data in the state.
        **PHYSICS THREAD ONLY**: Should be called from physics thread.
        **Not thread-safe**: Never call from multiple threads simultaneously.

        Args:
            scan: New LiDAR scan data to store
        """
        pass 