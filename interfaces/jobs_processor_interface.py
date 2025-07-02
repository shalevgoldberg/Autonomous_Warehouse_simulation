"""
Interface for JobsProcessor - converts external orders into actionable robot tasks.

The JobsProcessor is responsible for:
1. Pulling orders from external sources (JSON files, APIs, databases)
2. Converting orders into executable robot tasks
3. Managing inventory allocation and shelf locking
4. Coordinating with SimulationDataService for warehouse data
5. Pushing tasks to JobsQueue for robot assignment

Design Principles:
- **Single Responsibility**: Only handles order-to-task conversion
- **SOLID**: Dependency injection for external services
- **Thread Safety**: All methods are thread-safe for concurrent access
- **Error Handling**: Comprehensive exception handling with proper logging
"""
from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from .task_handler_interface import Task


class Priority(Enum):
    """Order priority levels."""
    LOW = "low"
    NORMAL = "normal"
    HIGH = "high"
    URGENT = "urgent"


class OrderStatus(Enum):
    """Order processing status."""
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class OrderItem:
    """Represents a single item in an order."""
    item_id: str
    quantity: int
    
    def __post_init__(self):
        if self.quantity <= 0:
            raise ValueError("Quantity must be positive")


@dataclass
class Order:
    """
    Represents an external customer order.
    
    This is the input format that JobsProcessor receives from external systems.
    Can be loaded from JSON, received via API, or pulled from database.
    """
    order_id: str
    items: List[OrderItem]
    scheduled_time: datetime  # When order should enter the system
    priority: Priority = Priority.NORMAL
    customer_id: Optional[str] = None
    deadline: Optional[datetime] = None
    status: OrderStatus = OrderStatus.PENDING
    created_at: datetime = field(default_factory=datetime.now)
    metadata: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        if not self.items:
            raise ValueError("Order must contain at least one item")
        if not self.order_id:
            raise ValueError("Order must have an order_id")


@dataclass
class TaskItem:
    """Represents an item within a robot task."""
    item_id: str
    quantity: int
    shelf_id: str  # Which shelf to pick from


@dataclass
class ProcessingResult:
    """Result of order processing operation."""
    success: bool
    order_id: str
    tasks_created: List[str]  # List of task IDs created
    error_message: Optional[str] = None
    processing_time: Optional[float] = None


@dataclass
class ProcessingStats:
    """Statistics about JobsProcessor performance."""
    total_orders_processed: int
    successful_orders: int
    failed_orders: int
    total_tasks_created: int
    average_processing_time: float
    orders_per_minute: float
    last_processing_time: Optional[datetime] = None


class JobsProcessorError(Exception):
    """Raised when jobs processor operations fail."""
    pass


class IJobsProcessor(ABC):
    """
    Interface for jobs processor functionality.
    
    Responsibilities:
    - Pull external orders from order sources (files, APIs, databases)
    - Convert orders into actionable robot tasks using SimulationDataService
    - Manage inventory allocation and shelf locking
    - Push tasks into JobsQueue for robot assignment
    - Track processing statistics and performance
    
    **Thread Safety**: All methods are thread-safe for concurrent access.
    **Threading Model**:
    - process_next_order(): DEDICATED PROCESSOR THREAD (continuous polling)
    - start/stop_processing(): MAIN/CONTROL THREAD (service management)
    - get_*() methods: ANY THREAD (monitoring and status)
    
    Integration Points:
    - **SimulationDataService**: For item locations, shelf locking, inventory updates
    - **JobsQueue**: For task assignment and distribution
    - **External Order Source**: For pulling raw orders (JSON, API, DB)
    """
    
    @abstractmethod
    def process_next_order(self) -> Optional[ProcessingResult]:
        """
        Pull and process the next available order from external source.
        This is the main processing method called continuously.
        
        Workflow:
        1. Check for orders due for processing (scheduled_time <= now)
        2. Select highest priority order
        3. For each item in order:
           a. Find shelf with sufficient inventory using SimulationDataService
           b. Lock shelf to prevent conflicts
           c. Create robot task for pick operation
           d. Update inventory allocation
        4. Push all tasks to JobsQueue
        5. Mark order as completed
        
        Returns:
            Optional[ProcessingResult]: Result of processing operation, 
                                      None if no orders available
            
        Raises:
            JobsProcessorError: If order processing fails critically
        """
        pass
    
    @abstractmethod
    def process_order(self, order: Order) -> ProcessingResult:
        """
        Process a specific order immediately.
        Useful for testing or manual order injection.
        
        Args:
            order: Order to process
            
        Returns:
            ProcessingResult: Result of processing operation
            
        Raises:
            JobsProcessorError: If order processing fails
        """
        pass
    
    @abstractmethod
    def start_processing(self) -> None:
        """
        Start the continuous order processing service.
        Begins polling for orders and processing them automatically.
        """
        pass
    
    @abstractmethod
    def stop_processing(self) -> None:
        """
        Stop the order processing service.
        Completes current order processing then stops polling.
        """
        pass
    
    @abstractmethod
    def is_processing(self) -> bool:
        """
        Check if the processor is currently active.
        
        Returns:
            bool: True if processing orders, False otherwise
        """
        pass
    
    @abstractmethod
    def get_processing_stats(self) -> ProcessingStats:
        """
        Get comprehensive statistics about order processing performance.
        
        Returns:
            ProcessingStats: Processing performance metrics
        """
        pass
    
    @abstractmethod
    def get_pending_orders(self) -> List[Order]:
        """
        Get all orders waiting to be processed.
        
        Returns:
            List[Order]: Orders that are pending processing
        """
        pass
    
    @abstractmethod
    def get_order_status(self, order_id: str) -> Optional[OrderStatus]:
        """
        Get the status of a specific order.
        
        Args:
            order_id: ID of order to check
            
        Returns:
            Optional[OrderStatus]: Order status or None if not found
        """
        pass
    
    @abstractmethod
    def cancel_order(self, order_id: str, reason: str = "User cancelled") -> bool:
        """
        Cancel a pending order.
        
        Args:
            order_id: ID of order to cancel
            reason: Reason for cancellation
            
        Returns:
            bool: True if order was cancelled, False if not found or already processed
        """
        pass
    
    @abstractmethod
    def add_order(self, order: Order) -> bool:
        """
        Add a new order to the processing queue.
        Useful for API endpoints or manual order injection.
        
        Args:
            order: Order to add
            
        Returns:
            bool: True if order was added successfully
            
        Raises:
            JobsProcessorError: If order is invalid or cannot be added
        """
        pass
    
    @abstractmethod
    def get_processing_errors(self) -> List[Dict[str, Any]]:
        """
        Get recent processing errors for debugging and monitoring.
        
        Returns:
            List[Dict[str, Any]]: Recent error information with timestamps
        """
        pass 