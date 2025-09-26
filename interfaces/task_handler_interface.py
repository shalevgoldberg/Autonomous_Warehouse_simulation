"""
Interface for TaskHandler - manages robot tasks and coordinates with external services.
"""
from abc import ABC, abstractmethod
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
from datetime import datetime

from .lane_follower_interface import ILaneFollower


class OperationalStatus(Enum):
    """Robot operational status - managed by TaskHandler."""
    IDLE = "idle"
    MOVING_TO_SHELF = "moving_to_shelf"
    APPROACHING_SHELF = "approaching_shelf"  # Close to shelf (≤2m), relaxed safety margins
    PICKING = "picking"
    MOVING_TO_DROPOFF = "moving_to_dropoff"
    DROPPING = "dropping"
    MOVING_TO_CHARGING = "moving_to_charging"
    CHARGING = "charging"
    MOVING_TO_IDLE = "moving_to_idle"
    WANDERING = "wandering"
    STALLED = "stalled"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"
    # New states for bay handling
    APPROACHING_BAY = "approaching_bay"
    ENTERING_BAY = "entering_bay"
    IN_BAY = "in_bay"
    EXITING_BAY = "exiting_bay"


class TaskType(Enum):
    """Types of robot tasks."""
    PICK_AND_DELIVER = "pick_and_deliver"
    MOVE_TO_CHARGING = "move_to_charging"
    MOVE_TO_POSITION = "move_to_position"
    IDLE_PARK = "idle_park"  # New: park in idle bay


class TaskStatus(Enum):
    """Task execution status."""
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class Task:
    """
    Enhanced robot task with order context and inventory details.
    
    This enhanced structure supports the JobsProcessor workflow by including
    order traceability, inventory specifics, and coordination metadata.
    """
    # Required fields (no defaults)
    task_id: str
    task_type: TaskType
    
    # Context fields (required for order tasks, optional for system tasks)
    order_id: str = ""  # Which order this task belongs to
    shelf_id: str = ""  # Specific shelf to pick from
    item_id: str = ""   # Specific item to pick
    
    # Optional fields with defaults
    order_priority: str = "normal"  # URGENT, HIGH, NORMAL, LOW from Priority enum
    customer_id: Optional[str] = None
    quantity_to_pick: int = 1  # How many items to pick
    dropoff_zone: str = "default_dropoff"  # Where to deliver items
    estimated_duration: Optional[float] = None  # Estimated time in seconds
    target_position: Optional[tuple] = None  # For MOVE_TO_POSITION tasks
    bay_id: Optional[str] = None  # For bay-related tasks (charging, idle)
    priority: int = 0  # Legacy priority (0-10 scale)
    created_at: datetime = field(default_factory=datetime.now)
    assigned_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    status: TaskStatus = TaskStatus.PENDING
    inventory_reserved: bool = False  # Has inventory been allocated?
    shelf_locked: bool = False        # Is shelf locked for this task?
    metadata: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        """Validate task data after creation."""
        # Basic validation
        if self.quantity_to_pick <= 0:
            raise ValueError("quantity_to_pick must be positive")
        
        # Task type specific validation
        if self.task_type == TaskType.PICK_AND_DELIVER:
            if not self.shelf_id:
                raise ValueError("PICK_AND_DELIVER tasks require shelf_id")
            if not self.item_id:
                raise ValueError("PICK_AND_DELIVER tasks require item_id")
            if not self.order_id:
                raise ValueError("PICK_AND_DELIVER tasks require order_id")
        
        if self.task_type == TaskType.MOVE_TO_POSITION:
            if not self.target_position:
                raise ValueError("MOVE_TO_POSITION tasks require target_position")
        
        if self.task_type in [TaskType.MOVE_TO_CHARGING, TaskType.IDLE_PARK]:
            if not self.bay_id:
                raise ValueError(f"{self.task_type.value} tasks require bay_id")
        
        # For non-order tasks, generate system order_id if empty
        if self.task_type in [TaskType.MOVE_TO_CHARGING, TaskType.MOVE_TO_POSITION]:
            if not self.order_id:
                self.order_id = f"system_{self.task_type.value}_{self.task_id}"
    
    @classmethod
    def create_pick_and_deliver_task(cls, task_id: str, order_id: str, 
                                   shelf_id: str, item_id: str, 
                                   quantity_to_pick: int = 1,
                                   order_priority: str = "normal",
                                   customer_id: Optional[str] = None,
                                   dropoff_zone: str = "default_dropoff") -> 'Task':
        """
        Factory method to create pick and deliver tasks from orders.
        
        Args:
            task_id: Unique task identifier
            order_id: Source order identifier
            shelf_id: Shelf to pick from
            item_id: Item to pick
            quantity_to_pick: Number of items to pick
            order_priority: Order priority level
            customer_id: Customer identifier
            dropoff_zone: Where to deliver items
            
        Returns:
            Task: Configured pick and deliver task
        """
        return cls(
            task_id=task_id,
            task_type=TaskType.PICK_AND_DELIVER,
            order_id=order_id,
            shelf_id=shelf_id,
            item_id=item_id,
            quantity_to_pick=quantity_to_pick,
            order_priority=order_priority,
            customer_id=customer_id,
            dropoff_zone=dropoff_zone
        )
    
    @classmethod
    def create_idle_park_task(cls, task_id: str, robot_id: str,
                             bay_id: Optional[str] = None) -> 'Task':
        """
        Factory method to create idle park tasks.
        
        Args:
            task_id: Unique task identifier
            robot_id: Robot identifier
            bay_id: Specific idle zone bay (optional, will be auto-assigned if None)
            
        Returns:
            Task: Configured idle park task
        """
        return cls(
            task_id=task_id,
            task_type=TaskType.IDLE_PARK,
            order_id=f"system_idle_{robot_id}",
            bay_id=bay_id or "auto_assigned",
            priority=0  # Low priority system task
        )
    
    def __eq__(self, other) -> bool:
        """Tasks are equal if they have the same task_id."""
        if not isinstance(other, Task):
            return False
        return self.task_id == other.task_id
    
    def __hash__(self) -> int:
        """Hash based on task_id for use in sets and dicts."""
        return hash(self.task_id)


@dataclass
class TaskHandlerStatus:
    """TaskHandler status information."""
    has_active_task: bool
    task_id: Optional[str]
    operational_status: OperationalStatus
    progress: float  # 0.0 to 1.0
    stall_reason: Optional[str] = None
    locked_bay_id: Optional[str] = None  # Current charging/idle bay lock


class TaskHandlingError(Exception):
    """Raised when task handling fails."""
    pass


class ITaskHandler(ABC):
    """
    Interface for task handling functionality.
    
    Responsibilities:
    - Execute assigned tasks from external systems (Orchestrator/JobsQueue)
    - Manage operational status and task lifecycle
    - Coordinate with SimulationDataService for shelf operations
    - Coordinate with PathPlanner and MotionExecutor
    
    **Thread Safety**: All methods are thread-safe internally.
    **Threading Model**: 
    - ALL methods: CONTROL THREAD ONLY (10Hz)
    - Never call from physics or visualization threads
    """
    
    @abstractmethod
    def get_task_status(self) -> TaskHandlerStatus:
        """
        Get current task execution status.
        
        Returns:
            TaskHandlerStatus: Current status including progress and operational state
        """
        pass
    
    @abstractmethod
    def update_task_execution(self) -> None:
        """
        Update task execution (called at 10Hz from control thread).
        Handles task lifecycle, waypoint progression, and motion coordination.
        """
        pass
    
    @abstractmethod
    def emergency_stop(self) -> None:
        """
        Emergency stop current task.
        Immediately halts all motion and marks task as failed.
        """
        pass
    
    @abstractmethod
    def start_task(self, task: Task) -> bool:
        """
        Start executing a new task.
        Called by Orchestrator or main robot controller when a task is assigned.
        
        Args:
            task: Task to execute
            
        Returns:
            bool: True if task accepted, False if robot is busy
        """
        pass
    
    @abstractmethod
    def get_current_task(self) -> Optional[Task]:
        """
        Get the current active task.
        Useful for logging, monitoring, or UI components.
        
        Returns:
            Optional[Task]: Current task or None if idle
        """
        pass
    
    @abstractmethod
    def handle_stall_event(self, reason: str) -> None:
        """
        Handle robot stall event.
        Called by local safety or traffic modules when robot is stuck/blocked.
        
        Args:
            reason: Description of why the robot is stalled
        """
        pass
    
    @abstractmethod
    def is_idle(self) -> bool:
        """
        Check if robot is available for new tasks.
        Used by Orchestrator or task assigner.
        
        Returns:
            bool: True if robot can accept new tasks
        """
        pass
    
    @abstractmethod
    def cancel_task(self, reason: str = "User cancelled") -> None:
        """
        Cancel the current task.
        
        Args:
            reason: Reason for cancellation
            
        Raises:
            TaskHandlingError: If no active task
        """
        pass
    
    @abstractmethod
    def get_task_history(self, limit: int = 10) -> List[Task]:
        """
        Get recent task history.
        
        Args:
            limit: Maximum number of tasks to return
            
        Returns:
            List[Task]: Recent tasks
        """
        pass 