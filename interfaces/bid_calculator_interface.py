"""
Interface for Bid Calculator - robot task bidding functionality.

The Bid Calculator is responsible for:
1. Calculating bid values for robot-task combinations
2. Considering multiple factors (distance, battery, workload, etc.)
3. Providing extensible framework for new bidding factors
4. Supporting different bidding strategies and algorithms

Design Principles:
- **Single Responsibility**: Only handles bid calculation logic
- **Open/Closed**: Open for extension (new factors), closed for modification
- **Dependency Inversion**: Depends on abstractions (StateHolder, Task)
- **Interface Segregation**: Clean, focused interface
- **Thread Safety**: All methods are thread-safe for concurrent access
"""
from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum

from .task_handler_interface import Task
from .state_holder_interface import IStateHolder
from .bidding_system_interface import RobotBid


class BidFactor(Enum):
    """Available bidding factors that can be considered."""
    DISTANCE = "distance"
    BATTERY_LEVEL = "battery_level"
    WORKLOAD = "workload"
    TASK_TYPE_COMPATIBILITY = "task_type_compatibility"
    ROBOT_CAPABILITIES = "robot_capabilities"
    TIME_URGENCY = "time_urgency"
    CONFLICT_BOX_AVAILABILITY = "conflict_box_availability"
    SHELF_ACCESSIBILITY = "shelf_accessibility"


@dataclass
class BidFactorWeight:
    """Weight configuration for a bidding factor."""
    factor: BidFactor
    weight: float  # 0.0 to 1.0, where 1.0 is maximum importance
    enabled: bool = True
    
    def __post_init__(self):
        """Validate bid factor weight."""
        if not 0.0 <= self.weight <= 1.0:
            raise ValueError(f"Bid factor weight must be between 0.0 and 1.0, got {self.weight}")


@dataclass
class BidCalculationContext:
    """Context information for bid calculation."""
    robot_id: str
    robot_position: Tuple[float, float, float]
    battery_level: float
    current_task: Optional[Task]
    operational_status: str
    robot_capabilities: Dict[str, Any]
    warehouse_map_info: Optional[Dict[str, Any]] = None
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class BidCalculationResult:
    """Result of a bid calculation."""
    robot_id: str
    task: Task
    total_bid_value: float
    factor_breakdown: Dict[BidFactor, float]  # Individual factor contributions
    calculation_time: float  # Time taken to calculate in seconds
    metadata: Optional[Dict[str, Any]] = None
    timestamp: datetime = field(default_factory=datetime.now)
    
    def __post_init__(self):
        """Validate bid calculation result."""
        if self.total_bid_value < 0:
            raise ValueError(f"Bid value must be non-negative, got {self.total_bid_value}")
        if self.calculation_time < 0:
            raise ValueError(f"Calculation time must be non-negative, got {self.calculation_time}")


class IBidCalculator(ABC):
    """
    Interface for bid calculation functionality.
    
    Responsibilities:
    - Calculate bid values for robot-task combinations
    - Support multiple configurable factors
    - Provide extensible framework for new factors
    - Support different bidding strategies
    - Ensure thread-safe operation
    
    **Thread Safety**: All methods are thread-safe for concurrent access.
    **Threading Model**:
    - calculate_bids(): Called from robot bidding threads
    - calculate_single_bid(): Called from robot bidding threads
    - get_*() methods: ANY THREAD (configuration and monitoring)
    
    Integration Points:
    - **StateHolder**: For robot state information
    - **Task**: For task requirements and constraints
    - **SimulationDataService**: For warehouse data (optional)
    - **ConfigurationProvider**: For bid factor weights and settings
    """
    
    @abstractmethod
    def calculate_bids(self, tasks: List[Task], state_holder: IStateHolder) -> List[RobotBid]:
        """
        Calculate bids for multiple tasks.
        
        This method calculates bids for all provided tasks using the robot's
        current state. It's designed for parallel execution where the controller
        sends multiple tasks to a robot for bidding.
        
        Args:
            tasks: List of tasks to calculate bids for
            state_holder: Robot state holder for current status
            
        Returns:
            List[RobotBid]: Bids for all tasks (may be empty if robot unavailable)
            
        Raises:
            BidCalculationError: If bid calculation fails critically
        """
        pass
    
    @abstractmethod
    def calculate_single_bid(self, task: Task, state_holder: IStateHolder) -> Optional[RobotBid]:
        """
        Calculate bid for a single task.
        
        Args:
            task: Task to calculate bid for
            state_holder: Robot state holder for current status
            
        Returns:
            Optional[RobotBid]: Bid for the task, or None if robot cannot bid
            
        Raises:
            BidCalculationError: If bid calculation fails
        """
        pass
    
    @abstractmethod
    def is_available_for_bidding(self, state_holder: IStateHolder) -> bool:
        """
        Check if robot is available for bidding.
        
        This method determines if the robot can participate in bidding
        based on its current state (battery, operational status, etc.).
        
        Args:
            state_holder: Robot state holder for current status
            
        Returns:
            bool: True if robot can participate in bidding
        """
        pass
    
    @abstractmethod
    def set_bid_factor_weights(self, weights: List[BidFactorWeight]) -> None:
        """
        Set weights for different bidding factors.
        
        Args:
            weights: List of factor weights to apply
            
        Raises:
            BidCalculationError: If weights are invalid
        """
        pass
    
    @abstractmethod
    def get_bid_factor_weights(self) -> List[BidFactorWeight]:
        """
        Get current bid factor weights.
        
        Returns:
            List[BidFactorWeight]: Current factor weight configuration
        """
        pass
    
    @abstractmethod
    def enable_factor(self, factor: BidFactor, enabled: bool = True) -> None:
        """
        Enable or disable a specific bidding factor.
        
        Args:
            factor: Factor to enable/disable
            enabled: True to enable, False to disable
        """
        pass
    
    @abstractmethod
    def get_supported_factors(self) -> List[BidFactor]:
        """
        Get list of supported bidding factors.
        
        Returns:
            List[BidFactor]: Factors supported by this calculator
        """
        pass
    
    @abstractmethod
    def get_calculation_statistics(self) -> Dict[str, Any]:
        """
        Get bid calculation statistics.
        
        Returns:
            Dict[str, Any]: Statistics including calculation times, success rates, etc.
        """
        pass


class BidCalculationError(Exception):
    """Exception raised when bid calculation fails."""
    pass 