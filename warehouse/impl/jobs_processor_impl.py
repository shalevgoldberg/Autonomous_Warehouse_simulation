"""
JobsProcessor implementation - converts orders to robot tasks.

This implementation handles the core workflow:
1. Poll OrderSource for due orders
2. Look up inventory using SimulationDataService
3. Create robot tasks with proper coordination (single-shelf allocation strategy)
4. Push tasks to JobsQueue for robot assignment

SINGLE-SHELF ALLOCATION STRATEGY:
- Each robot task handles only one shelf (keeps robot logic simple)
- For items requiring multiple shelves, creates separate tasks for different robots
- Achieves order completion through task composition rather than complex multi-shelf tasks
- Enables better parallelization and cleaner error handling

Follows SOLID principles with dependency injection and clean separation of concerns.
"""
import logging
import threading
import time
import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Set, Any
from dataclasses import dataclass, field

from interfaces.jobs_processor_interface import (
    IJobsProcessor, Order, OrderItem, Priority, OrderStatus,
    ProcessingResult, ProcessingStats, JobsProcessorError
)
from interfaces.order_source_interface import IOrderSource
from interfaces.simulation_data_service_interface import ISimulationDataService
from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.task_handler_interface import Task, TaskType


@dataclass
class InventoryAllocation:
    """Tracks inventory allocation for an order."""
    order_id: str
    item_id: str
    shelf_id: str
    quantity_allocated: int
    quantity_requested: int
    allocation_time: datetime = field(default_factory=datetime.now)
    
    @property
    def is_fully_allocated(self) -> bool:
        """Check if the full requested quantity is allocated."""
        return self.quantity_allocated >= self.quantity_requested


class JobsProcessorImpl(IJobsProcessor):
    """
    Production implementation of JobsProcessor.
    
    Features:
    - Thread-safe order processing with dedicated processor thread
    - Smart inventory allocation with shelf locking
    - Priority-based order processing
    - Comprehensive error handling and recovery
    - Statistics tracking and monitoring
    - Graceful shutdown handling
    """
    
    def __init__(self, 
                 order_source: IOrderSource,
                 simulation_data_service: ISimulationDataService,
                 jobs_queue: IJobsQueue,
                 processing_interval: float = 5.0,
                 max_concurrent_orders: int = 10):
        """
        Initialize JobsProcessor with dependencies.
        
        Args:
            order_source: Source of customer orders
            simulation_data_service: Warehouse inventory and shelf management
            jobs_queue: Queue for robot task distribution
            processing_interval: How often to check for new orders (seconds)
            max_concurrent_orders: Maximum orders to process simultaneously
        """
        self._order_source = order_source
        self._simulation_data_service = simulation_data_service
        self._jobs_queue = jobs_queue
        self._processing_interval = processing_interval
        self._max_concurrent_orders = max_concurrent_orders
        
        # Thread management
        self._processor_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()
        self._lock = threading.RLock()
        
        # Processing state
        self._active_orders: Dict[str, Order] = {}
        self._inventory_allocations: Dict[str, List[InventoryAllocation]] = {}
        self._locked_shelves: Set[str] = set()
        
        # Statistics
        self._stats = ProcessingStats(
            total_orders_processed=0,
            successful_orders=0,
            failed_orders=0,
            total_tasks_created=0,
            average_processing_time=0.0,
            orders_per_minute=0.0,
            last_processing_time=None
        )
        
        # Logging
        self._logger = logging.getLogger(f"{self.__class__.__name__}")
        self._logger.info("JobsProcessor initialized with %d max concurrent orders", 
                         max_concurrent_orders)
    
    def start_processing(self) -> None:
        """Start the background order processing thread."""
        with self._lock:
            if self._processor_thread and self._processor_thread.is_alive():
                raise JobsProcessorError("JobsProcessor is already running")
            
            self._shutdown_event.clear()
            self._processor_thread = threading.Thread(
                target=self._processor_loop,
                name="JobsProcessor",
                daemon=True
            )
            self._processor_thread.start()
            self._logger.info("JobsProcessor started")
    
    def stop_processing(self) -> None:
        """Stop the background processing thread gracefully."""
        with self._lock:
            if not self._processor_thread or not self._processor_thread.is_alive():
                self._logger.warning("JobsProcessor is not running")
                return
            
            self._logger.info("Stopping JobsProcessor...")
            self._shutdown_event.set()
            
            # Wait for thread to finish with timeout
            self._processor_thread.join(timeout=10.0)
            if self._processor_thread.is_alive():
                self._logger.error("JobsProcessor thread did not stop gracefully")
            else:
                self._logger.info("JobsProcessor stopped")
            
            # Cleanup allocated resources
            self._cleanup_allocations()
    
    def process_orders_once(self) -> ProcessingResult:
        """
        Process orders once (manual trigger).
        
        Returns:
            ProcessingResult: Results of the processing attempt
        """
        start_time = time.time()
        
        try:
            # Get due orders from source
            due_orders = self._order_source.get_due_orders(
                limit=self._max_concurrent_orders - len(self._active_orders)
            )
            
            if not due_orders:
                            return ProcessingResult(
                success=True,
                order_id="",
                tasks_created=[],
                processing_time=time.time() - start_time
            )
            
            # Process each order
            total_tasks_created = 0
            orders_processed = 0
            errors = []
            
            for order in due_orders:
                try:
                    if order.order_id in self._active_orders:
                        continue  # Already processing this order
                    
                    tasks_created = self._process_single_order(order)
                    total_tasks_created += tasks_created
                    orders_processed += 1
                    
                    # Update order status
                    self._order_source.update_order_status(
                        order.order_id, OrderStatus.PROCESSING
                    )
                    
                except Exception as e:
                    error_msg = f"Failed to process order {order.order_id}: {str(e)}"
                    self._logger.error(error_msg, exc_info=True)
                    errors.append(error_msg)
                    
                    # Update order status to failed
                    try:
                        self._order_source.update_order_status(
                            order.order_id, OrderStatus.FAILED
                        )
                    except Exception:
                        pass  # Don't let status update failures block processing
            
            # Update statistics
            processing_time = time.time() - start_time
            self._update_stats(orders_processed, total_tasks_created, 
                             len(errors), processing_time)
            
            success = len(errors) == 0
            message = f"Processed {orders_processed} orders, created {total_tasks_created} tasks"
            if errors:
                message += f" with {len(errors)} errors"
            
            return ProcessingResult(
                success=success,
                order_id="batch_processing",
                tasks_created=[f"task_{i}" for i in range(total_tasks_created)],
                processing_time=processing_time,
                error_message="; ".join(errors) if errors else None
            )
            
        except Exception as e:
            error_msg = f"Critical error in order processing: {str(e)}"
            self._logger.error(error_msg, exc_info=True)
            self._stats.failed_orders += 1
            
            return ProcessingResult(
                success=False,
                order_id="batch_processing_error",
                tasks_created=[],
                processing_time=time.time() - start_time,
                error_message=error_msg
            )
    
    def get_processing_stats(self) -> ProcessingStats:
        """Get current processing statistics."""
        with self._lock:
            # Calculate orders per minute
            orders_per_minute = 0.0
            if self._stats.last_processing_time:
                time_diff = (datetime.now() - self._stats.last_processing_time).total_seconds()
                if time_diff > 0:
                    orders_per_minute = (self._stats.total_orders_processed * 60.0) / time_diff
            
            return ProcessingStats(
                total_orders_processed=self._stats.total_orders_processed,
                successful_orders=self._stats.successful_orders,
                failed_orders=self._stats.failed_orders,
                total_tasks_created=self._stats.total_tasks_created,
                average_processing_time=self._stats.average_processing_time,
                orders_per_minute=orders_per_minute,
                last_processing_time=self._stats.last_processing_time
            )
    
    def get_active_orders(self) -> List[Order]:
        """Get list of currently active orders being processed."""
        with self._lock:
            return list(self._active_orders.values())
    
    def cancel_order(self, order_id: str) -> bool:
        """
        Cancel an active order and clean up its resources.
        
        Args:
            order_id: ID of order to cancel
            
        Returns:
            bool: True if order was cancelled, False if not found
        """
        with self._lock:
            if order_id not in self._active_orders:
                return False
            
            # Clean up allocations for this order
            self._cleanup_order_allocations(order_id)
            
            # Remove from active orders
            del self._active_orders[order_id]
            
            # Update order status
            try:
                self._order_source.update_order_status(order_id, OrderStatus.CANCELLED)
            except Exception as e:
                self._logger.error(f"Failed to update cancelled order status: {e}")
            
            self._logger.info(f"Order {order_id} cancelled")
            return True
    
    def is_processing(self) -> bool:
        """Check if the processor is currently running."""
        with self._lock:
            return (self._processor_thread is not None and 
                   self._processor_thread.is_alive() and 
                   not self._shutdown_event.is_set())
    
    # Private implementation methods
    
    def _processor_loop(self) -> None:
        """Main processing loop running in background thread."""
        self._logger.info("JobsProcessor loop started")
        
        while not self._shutdown_event.is_set():
            try:
                # Process orders once
                result = self.process_orders_once()
                
                if result.orders_processed > 0:
                    self._logger.info(f"Processed {result.orders_processed} orders, "
                                    f"created {result.tasks_created} tasks in "
                                    f"{result.processing_time:.2f}s")
                
                # Wait for next processing cycle
                self._shutdown_event.wait(self._processing_interval)
                
            except Exception as e:
                self._logger.error(f"Unexpected error in processor loop: {e}", exc_info=True)
                # Continue processing after error
                self._shutdown_event.wait(self._processing_interval)
        
        self._logger.info("JobsProcessor loop ended")
    
    def _process_single_order(self, order: Order) -> int:
        """
        Process a single order into robot tasks.
        
        Args:
            order: Order to process
            
        Returns:
            int: Number of tasks created
            
        Raises:
            JobsProcessorError: If order processing fails
        """
        self._logger.debug(f"Processing order {order.order_id}")
        
        # Add to active orders
        with self._lock:
            self._active_orders[order.order_id] = order
            self._inventory_allocations[order.order_id] = []
        
        tasks_created = 0
        
        try:
            # Process each item in the order
            for order_item in order.items:
                # Find inventory for this item
                inventory_info = self._simulation_data_service.get_item_inventory(
                    order_item.item_id
                )
                
                if not inventory_info or inventory_info.available_quantity < order_item.quantity:
                    raise JobsProcessorError(
                        f"Insufficient inventory for item {order_item.item_id}: "
                        f"requested {order_item.quantity}, available {inventory_info.available_quantity if inventory_info else 0}"
                    )
                
                # Allocate inventory and create tasks
                tasks_for_item = self._allocate_and_create_tasks(order, order_item, inventory_info)
                tasks_created += len(tasks_for_item)
                
                # Push tasks to queue
                for task in tasks_for_item:
                    self._jobs_queue.enqueue_task(task)
            
            self._logger.info(f"Successfully processed order {order.order_id}, created {tasks_created} tasks")
            return tasks_created
            
        except Exception as e:
            # Clean up on failure
            self._cleanup_order_allocations(order.order_id)
            with self._lock:
                if order.order_id in self._active_orders:
                    del self._active_orders[order.order_id]
            raise JobsProcessorError(f"Failed to process order {order.order_id}: {str(e)}") from e
    
    def _allocate_and_create_tasks(self, order: Order, order_item: OrderItem, 
                                 inventory_info) -> List[Task]:
        """
        Allocate inventory and create robot tasks for an order item.
        
        Uses single-shelf allocation strategy: each robot task handles only one shelf.
        This keeps robot logic simple and enables better parallelization across multiple robots.
        
        For items that require multiple shelves, creates separate tasks that can be assigned
        to different robots, achieving order completion through task composition.
        
        Args:
            order: Source order
            order_item: Item to process
            inventory_info: Available inventory information
            
        Returns:
            List[Task]: Created robot tasks (one task per shelf)
        """
        tasks = []
        remaining_quantity = order_item.quantity
        
        # Get shelf locations for this item
        shelf_locations = self._simulation_data_service.get_item_shelf_locations(
            order_item.item_id
        )
        
        if not shelf_locations:
            raise JobsProcessorError(f"No shelf locations found for item {order_item.item_id}")
        
        # Sort shelves by available quantity (prioritize fuller shelves)
        shelf_locations.sort(key=lambda x: x.quantity, reverse=True)
        
        # Create one task per shelf (single-shelf allocation strategy)
        for shelf_location in shelf_locations:
            if remaining_quantity <= 0:
                break
            
            # Check if shelf is already locked
            with self._lock:
                if shelf_location.shelf_id in self._locked_shelves:
                    continue
            
            # Determine how much to pick from this shelf (single-shelf task)
            quantity_to_pick = min(remaining_quantity, shelf_location.quantity)
            
            # Lock the shelf for this task
            with self._lock:
                self._locked_shelves.add(shelf_location.shelf_id)
            
            # Create allocation record
            allocation = InventoryAllocation(
                order_id=order.order_id,
                item_id=order_item.item_id,
                shelf_id=shelf_location.shelf_id,
                quantity_allocated=quantity_to_pick,
                quantity_requested=order_item.quantity
            )
            
            with self._lock:
                self._inventory_allocations[order.order_id].append(allocation)
            
            # Create single-shelf robot task (simple for robot controllers)
            task = Task.create_pick_and_deliver_task(
                task_id=f"task_{uuid.uuid4().hex[:8]}",
                order_id=order.order_id,
                shelf_id=shelf_location.shelf_id,
                item_id=order_item.item_id,
                quantity_to_pick=quantity_to_pick,
                order_priority=order.priority.value.lower(),
                customer_id=order.customer_id
            )
            
            # Set coordination flags
            task.inventory_reserved = True
            task.shelf_locked = True
            
            tasks.append(task)
            remaining_quantity -= quantity_to_pick
            
            self._logger.debug(f"Created single-shelf task {task.task_id} for {quantity_to_pick} of "
                             f"{order_item.item_id} from shelf {shelf_location.shelf_id}")
        
        if remaining_quantity > 0:
            raise JobsProcessorError(
                f"Could not allocate sufficient inventory for {order_item.item_id}: "
                f"still need {remaining_quantity} items"
            )
        
        return tasks
    
    def _cleanup_order_allocations(self, order_id: str) -> None:
        """Clean up inventory allocations for a specific order."""
        with self._lock:
            if order_id in self._inventory_allocations:
                # Unlock shelves allocated to this order
                for allocation in self._inventory_allocations[order_id]:
                    self._locked_shelves.discard(allocation.shelf_id)
                
                # Remove allocation records
                del self._inventory_allocations[order_id]
                
                self._logger.debug(f"Cleaned up allocations for order {order_id}")
    
    def _cleanup_allocations(self) -> None:
        """Clean up all inventory allocations."""
        with self._lock:
            self._locked_shelves.clear()
            self._inventory_allocations.clear()
            self._active_orders.clear()
            self._logger.info("All inventory allocations cleaned up")
    
    def _update_stats(self, orders_processed: int, tasks_created: int, 
                     errors: int, processing_time: float) -> None:
        """Update processing statistics."""
        with self._lock:
            self._stats.total_orders_processed += orders_processed
            self._stats.total_tasks_created += tasks_created
            self._stats.failed_orders += errors
            self._stats.successful_orders += (orders_processed - errors)
            self._stats.last_processing_time = datetime.now()
            
            # Update average processing time (exponential moving average)
            if self._stats.average_processing_time == 0.0:
                self._stats.average_processing_time = processing_time
            else:
                alpha = 0.1  # Smoothing factor
                self._stats.average_processing_time = (
                    alpha * processing_time + 
                    (1 - alpha) * self._stats.average_processing_time
                )
    
    # Additional interface methods
    
    def process_next_order(self) -> Optional[ProcessingResult]:
        """
        Pull and process the next available order from external source.
        
        Returns:
            Optional[ProcessingResult]: Result of processing operation, 
                                      None if no orders available
        """
        # Get the next due order
        due_orders = self._order_source.get_due_orders(limit=1)
        if not due_orders:
            return None
        
        order = due_orders[0]
        
        # Check if already processing this order
        with self._lock:
            if order.order_id in self._active_orders:
                return None
        
        # Process the order
        start_time = time.time()
        try:
            tasks_created = self._process_single_order(order)
            
            # Update order status
            self._order_source.update_order_status(
                order.order_id, OrderStatus.PROCESSING
            )
            
            # Create task IDs list
            task_ids = []
            # Note: In real implementation, we'd track task IDs during creation
            # For now, generate placeholder IDs
            for i in range(tasks_created):
                task_ids.append(f"task_{order.order_id}_{i}")
            
            processing_time = time.time() - start_time
            self._update_stats(1, tasks_created, 0, processing_time)
            
            return ProcessingResult(
                success=True,
                order_id=order.order_id,
                tasks_created=task_ids,
                processing_time=processing_time
            )
            
        except Exception as e:
            error_msg = f"Failed to process order {order.order_id}: {str(e)}"
            self._logger.error(error_msg, exc_info=True)
            
            # Update order status to failed
            try:
                self._order_source.update_order_status(
                    order.order_id, OrderStatus.FAILED
                )
            except Exception:
                pass
            
            processing_time = time.time() - start_time
            self._update_stats(0, 0, 1, processing_time)
            
            return ProcessingResult(
                success=False,
                order_id=order.order_id,
                tasks_created=[],
                error_message=error_msg,
                processing_time=processing_time
            )
    
    def process_order(self, order: Order) -> ProcessingResult:
        """
        Process a specific order immediately.
        
        Args:
            order: Order to process
            
        Returns:
            ProcessingResult: Result of processing operation
        """
        start_time = time.time()
        
        try:
            tasks_created = self._process_single_order(order)
            
            # Create task IDs list
            task_ids = []
            for i in range(tasks_created):
                task_ids.append(f"task_{order.order_id}_{i}")
            
            processing_time = time.time() - start_time
            self._update_stats(1, tasks_created, 0, processing_time)
            
            return ProcessingResult(
                success=True,
                order_id=order.order_id,
                tasks_created=task_ids,
                processing_time=processing_time
            )
            
        except Exception as e:
            error_msg = f"Failed to process order {order.order_id}: {str(e)}"
            self._logger.error(error_msg, exc_info=True)
            
            processing_time = time.time() - start_time
            self._update_stats(0, 0, 1, processing_time)
            
            return ProcessingResult(
                success=False,
                order_id=order.order_id,
                tasks_created=[],
                error_message=error_msg,
                processing_time=processing_time
            )
    
    def get_pending_orders(self) -> List[Order]:
        """
        Get all orders waiting to be processed.
        
        Returns:
            List[Order]: Orders that are pending processing
        """
        # Get all orders with pending status
        all_orders = self._order_source.get_all_orders()
        return [order for order in all_orders if order.status == OrderStatus.PENDING]
    
    def get_order_status(self, order_id: str) -> Optional[OrderStatus]:
        """
        Get the status of a specific order.
        
        Args:
            order_id: ID of order to check
            
        Returns:
            Optional[OrderStatus]: Order status or None if not found
        """
        order = self._order_source.get_order_by_id(order_id)
        return order.status if order else None
    
    def add_order(self, order: Order) -> bool:
        """
        Add a new order to the processing queue.
        
        Args:
            order: Order to add
            
        Returns:
            bool: True if order was added successfully
        """
        try:
            return self._order_source.add_order(order)
        except Exception as e:
            self._logger.error(f"Failed to add order {order.order_id}: {e}")
            raise JobsProcessorError(f"Failed to add order: {e}") from e
    
    def get_processing_errors(self) -> List[Dict[str, Any]]:
        """
        Get recent processing errors for debugging and monitoring.
        
        Returns:
            List[Dict[str, Any]]: Recent error information with timestamps
        """
        # Return basic error info from stats
        # In a full implementation, we'd maintain a detailed error log
        return [
            {
                "error_count": self._stats.processing_errors,
                "last_error_time": self._stats.last_processing_time,
                "message": "Processing errors occurred (detailed logging available in logs)"
            }
        ] if self._stats.processing_errors > 0 else [] 