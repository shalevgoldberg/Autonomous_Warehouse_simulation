"""
Battery Manager Interface - Professional battery management contract.

This interface defines the contract for battery management services including:
- Activity-based consumption calculation
- Battery level updates and monitoring
- Charging session management
- Emergency stop functionality
- Safety margin calculations

Follows SOLID principles with clear separation of concerns.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class RobotActivity(Enum):
    """Robot activity types affecting battery consumption."""
    IDLE = "idle"           # Robot is stationary and idle
    MOVING = "moving"       # Robot is moving without load
    CARRYING = "carrying"   # Robot is moving with load
    CHARGING = "charging"   # Robot is charging
    STALLED = "stalled"     # Robot is stuck/obstructed


@dataclass(frozen=True)
class BatteryConsumptionConfig:
    """Configuration for battery consumption rates."""
    idle_rate: float         # Consumption rate when idle (per second)
    moving_rate: float       # Consumption rate when moving (per second)
    carrying_rate: float     # Consumption rate when carrying (per second)
    stalled_rate: float      # Consumption rate when stalled (per second)
    charging_rate: float     # Charging rate when charging (per second, positive)
    speed_multiplier: float  # Additional consumption per m/s speed
    load_multiplier: float   # Additional consumption when carrying load


@dataclass(frozen=True)
class BatteryState:
    """Current battery state information."""
    level: float             # Current battery level (0.0 to 1.0)
    is_charging: bool        # Whether currently charging
    consumption_rate: float  # Current consumption rate (per second)
    last_update: float       # Last update timestamp


class IBatteryManager(ABC):
    """
    Battery management interface for robot power systems.
    
    Provides comprehensive battery management including:
    - Real-time consumption calculation based on robot activity
    - Battery level updates with safety bounds
    - Charging session management
    - Emergency stop functionality
    - Safety margin calculations for task planning
    """
    
    @abstractmethod
    def get_battery_state(self) -> BatteryState:
        """
        Get current battery state.
        
        Returns:
            BatteryState: Current battery information
        """
        pass
    
    @abstractmethod
    def get_battery_level(self) -> float:
        """
        Get current battery level.
        
        Returns:
            float: Battery level from 0.0 (empty) to 1.0 (full)
        """
        pass
    
    @abstractmethod
    def calculate_consumption_rate(self, activity: RobotActivity, 
                                 speed: float = 0.0, 
                                 carrying_load: bool = False) -> float:
        """
        Calculate battery consumption rate for given activity.
        
        Args:
            activity: Robot activity type
            speed: Current movement speed in m/s
            carrying_load: Whether robot is carrying a load
            
        Returns:
            float: Consumption rate in battery level per second
                  (negative = consumption, positive = charging)
        """
        pass
    
    @abstractmethod
    def update_battery_level(self, current_level: float, 
                           activity: RobotActivity,
                           speed: float = 0.0, 
                           carrying_load: bool = False,
                           dt: float = 0.001) -> float:
        """
        Update battery level based on activity and time.
        
        Args:
            current_level: Current battery level (0.0 to 1.0)
            activity: Robot activity type
            speed: Current movement speed in m/s
            carrying_load: Whether robot is carrying a load
            dt: Time delta in seconds
            
        Returns:
            float: New battery level (0.0 to 1.0)
        """
        pass
    
    @abstractmethod
    def start_charging_session(self) -> bool:
        """
        Start a charging session.
        
        Returns:
            bool: True if charging started successfully
        """
        pass
    
    @abstractmethod
    def stop_charging_session(self) -> bool:
        """
        Stop the current charging session.
        
        Returns:
            bool: True if charging stopped successfully
        """
        pass
    
    @abstractmethod
    def is_charging(self) -> bool:
        """
        Check if robot is currently charging.
        
        Returns:
            bool: True if charging, False otherwise
        """
        pass
    
    @abstractmethod
    def get_charging_time_estimate(self, target_level: float, 
                                 current_level: Optional[float] = None) -> float:
        """
        Estimate time to reach target battery level.
        
        Args:
            target_level: Target battery level (0.0 to 1.0)
            current_level: Current battery level (if None, use current state)
            
        Returns:
            float: Estimated time in seconds
        """
        pass
    
    @abstractmethod
    def has_sufficient_battery_for_task(self, estimated_consumption: float,
                                      safety_margin: float = 0.15) -> bool:
        """
        Check if robot has sufficient battery for a task.
        
        Args:
            estimated_consumption: Estimated battery consumption for task
            safety_margin: Safety margin as fraction of battery (0.0 to 1.0)
            
        Returns:
            bool: True if sufficient battery available
        """
        pass
    
    @abstractmethod
    def check_emergency_stop(self, current_level: float) -> bool:
        """
        Check if emergency stop should be triggered due to critically low battery.
        
        Args:
            current_level: Current battery level
            
        Returns:
            bool: True if emergency stop should be triggered
        """
        pass
    
    @abstractmethod
    def is_emergency_stop_active(self) -> bool:
        """
        Check if emergency stop is currently active due to battery depletion.
        
        Returns:
            bool: True if emergency stop is active
        """
        pass
    
    @abstractmethod
    def reset_emergency_stop(self) -> None:
        """
        Reset emergency stop state (e.g., after battery has been charged).
        """
        pass
