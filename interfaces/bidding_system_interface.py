"""
Interface for Bidding System - robot task assignment and coordination.

The Bidding System is responsible for:
1. Collecting bids from available robots for pending tasks
2. Selecting winning bids based on assignment criteria
3. Coordinating task distribution across multiple robots
4. Supporting both simple (transparent) and advanced bidding strategies

Design Principles:
- **Single Responsibility**: Only handles bid collection and selection
- **SOLID**: Interface segregation, dependency injection ready
- **Thread Safety**: All methods are thread-safe for concurrent access
- **Extensibility**: Supports both simple and complex bidding algorithms
- **Separation of Concerns**: Clean separation from task execution and queue management
"""
from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any, TYPE_CHECKING
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum

from .task_handler_interface import Task

if TYPE_CHECKING:
    from robot.robot_agent_lane_based import RobotAgent


class BidStatus(Enum):
    """Status of a robot bid."""
    PENDING = "pending"
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    EXPIRED = "expired"


class BiddingStrategy(Enum):
    """Available bidding strategies."""
    TRANSPARENT = "transparent"  # Single robot - automatic assignment
    ROUND_ROBIN = "round_robin"  # Simple rotation
    DISTANCE_BASED = "distance_based"  # Closest robot wins
    BATTERY_AWARE = "battery_aware"  # Consider battery levels
    LOAD_BALANCED = "load_balanced"  # Balance workload across robots
    HYBRID = "hybrid"  # Combination of multiple factors


@dataclass
class RobotBid:
    """
    Represents a robot's bid for a specific task.
    
    Contains all information needed for bid evaluation and assignment.
    """
    robot_id: str  # Robot identifier instead of robot object for cleaner separation
    task: Task
    bid_value: float  # Numeric bid value (lower is better for costs, higher for benefits)
    bid_timestamp: datetime = field(default_factory=datetime.now)
    bid_metadata: Optional[Dict[str, Any]] = None  # Additional bid information
    status: BidStatus = BidStatus.PENDING
    
    def __post_init__(self):
        """Validate bid data."""
        if self.bid_value < 0:
            raise ValueError("Bid value must be non-negative")
        if not self.robot_id or not self.task:
            raise ValueError("Robot ID and task must be provided")


@dataclass
class TaskAssignment:
    """
    Represents the assignment of a task to a robot.
    
    Result of the bidding process - which robot gets which task.
    """
    robot_id: str  # Robot identifier instead of robot object
    task: Task
    assignment_timestamp: datetime = field(default_factory=datetime.now)
    assignment_metadata: Optional[Dict[str, Any]] = None  # Assignment details
    winning_bid_value: Optional[float] = None  # Value of the winning bid
    
    def __post_init__(self):
        """Validate assignment data."""
        if not self.robot_id or not self.task:
            raise ValueError("Robot ID and task must be provided")


@dataclass
class BiddingRound:
    """
    Represents a complete bidding round with all bids and results.
    
    Useful for monitoring, debugging, and performance analysis.
    """
    round_id: str
    available_tasks: List[Task]
    submitted_bids: List[RobotBid]
    winning_assignments: List[TaskAssignment]
    round_timestamp: datetime = field(default_factory=datetime.now)
    round_duration: Optional[float] = None  # Time taken for the round
    round_metadata: Optional[Dict[str, Any]] = None  # Additional round information


@dataclass
class BiddingStats:
    """Statistics about bidding system performance."""
    total_rounds: int
    total_bids_submitted: int
    total_assignments_made: int
    average_round_duration: float
    average_bids_per_round: float
    assignment_success_rate: float  # Percentage of tasks successfully assigned
    last_round_timestamp: Optional[datetime] = None
    active_strategy: BiddingStrategy = BiddingStrategy.TRANSPARENT


class BiddingSystemError(Exception):
    """Raised when bidding system operations fail."""
    pass


class IBiddingSystem(ABC):
    """
    Interface for bidding system functionality.
    
    Responsibilities:
    - Collect bids from available robots for pending tasks
    - Evaluate bids using configurable strategies
    - Select winning bids and create task assignments
    - Track bidding performance and statistics
    - Support multiple bidding strategies (transparent, distance-based, etc.)
    
    **Thread Safety**: All methods are thread-safe for concurrent access.
    **Threading Model**:
    - collect_bids(): ORCHESTRATOR THREAD (called by RobotController)
    - select_winning_bids(): ORCHESTRATOR THREAD (called by RobotController)
    - get_*() methods: ANY THREAD (monitoring and status)
    
    Integration Points:
    - **JobsQueue**: For getting available tasks
    - **Robot Pool**: For getting available robots (RobotAgent instances)
    - **RobotController**: For task assignment coordination
    - **Monitoring Systems**: For performance tracking
    """
    
    @abstractmethod
    def collect_bids(self, available_tasks: List[Task], available_robots: List['RobotAgent']) -> List[RobotBid]:
        """
        Collect bids from available robots for the given tasks.
        
        This method asks each available robot to evaluate and bid on tasks
        they are capable of executing. Robots may decline to bid on tasks
        they cannot handle or are not suitable for.
        
        Args:
            available_tasks: List of tasks available for bidding
            available_robots: List of RobotAgent instances that can participate in bidding
            
        Returns:
            List[RobotBid]: All bids submitted by robots
            
        Raises:
            BiddingSystemError: If bid collection fails critically
        """
        pass
    
    @abstractmethod
    def select_winning_bids(self, bids: List[RobotBid]) -> List[TaskAssignment]:
        """
        Select winning bids and create task assignments.
        
        This method evaluates all submitted bids using the configured
        bidding strategy and selects the best assignments. The selection
        algorithm depends on the current strategy (transparent, distance-based, etc.).
        
        Args:
            bids: List of bids to evaluate
            
        Returns:
            List[TaskAssignment]: Winning assignments (robot-task pairs)
            
        Raises:
            BiddingSystemError: If bid selection fails
        """
        pass
    
    @abstractmethod
    def process_bidding_round(self, available_tasks: List[Task], 
                            available_robots: List['RobotAgent']) -> BiddingRound:
        """
        Process a complete bidding round from collection to assignment.
        
        This is a convenience method that combines collect_bids() and
        select_winning_bids() into a single operation, returning a
        complete BiddingRound with all information.
        
        Args:
            available_tasks: List of tasks available for bidding
            available_robots: List of RobotAgent instances that can participate in bidding
            
        Returns:
            BiddingRound: Complete round information including bids and assignments
            
        Raises:
            BiddingSystemError: If bidding round processing fails
        """
        pass
    
    @abstractmethod
    def set_bidding_strategy(self, strategy: BiddingStrategy) -> None:
        """
        Set the bidding strategy to use for task assignment.
        
        Different strategies optimize for different goals:
        - TRANSPARENT: Simple assignment for single robot scenarios
        - ROUND_ROBIN: Fair rotation among robots
        - DISTANCE_BASED: Minimize travel distance
        - BATTERY_AWARE: Consider robot battery levels
        - LOAD_BALANCED: Balance workload across robots
        - HYBRID: Combination of multiple factors
        
        Args:
            strategy: Bidding strategy to use
            
        Raises:
            BiddingSystemError: If strategy is not supported
        """
        pass
    
    @abstractmethod
    def get_current_strategy(self) -> BiddingStrategy:
        """
        Get the currently active bidding strategy.
        
        Returns:
            BiddingStrategy: Current strategy being used
        """
        pass
    
    @abstractmethod
    def get_bidding_stats(self) -> BiddingStats:
        """
        Get comprehensive bidding system statistics.
        
        Returns:
            BiddingStats: Performance statistics and metrics
        """
        pass
    
    @abstractmethod
    def get_recent_bidding_rounds(self, limit: int = 10) -> List[BiddingRound]:
        """
        Get recent bidding rounds for monitoring and analysis.
        
        Args:
            limit: Maximum number of rounds to return
            
        Returns:
            List[BiddingRound]: Recent bidding rounds
        """
        pass
    
    @abstractmethod
    def reset_statistics(self) -> None:
        """
        Reset all bidding statistics to zero.
        
        Useful for testing or when starting a new operational period.
        """
        pass
    
    @abstractmethod
    def is_robot_available_for_task(self, robot: 'RobotAgent', task: Task) -> bool:
        """
        Check if a robot is available and suitable for a specific task.
        
        This method evaluates robot capabilities, current status, and
        task requirements to determine if the robot can bid on the task.
        
        Args:
            robot: RobotAgent instance to check
            task: Task to check compatibility with
            
        Returns:
            bool: True if robot can bid on the task
        """
        pass
    
    @abstractmethod
    def calculate_bid_value(self, robot: 'RobotAgent', task: Task) -> float:
        """
        Calculate the bid value for a robot-task combination.
        
        This method implements the core bidding logic, calculating
        a numeric value that represents how suitable the robot is
        for the task. Lower values typically indicate better fits
        (e.g., lower cost, shorter distance).
        
        Args:
            robot: RobotAgent instance to evaluate
            task: Task to evaluate
            
        Returns:
            float: Bid value (lower is typically better)
            
        Raises:
            BiddingSystemError: If bid calculation fails
        """
        pass 