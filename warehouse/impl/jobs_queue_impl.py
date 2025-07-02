"""
JobsQueue implementation - thread-safe task distribution for robots.

This implementation provides:
- Thread-safe FIFO task queue
- Task distribution to robots via dequeue_task()
- Queue management and monitoring
- In-memory implementation for Phase 2

Follows SOLID principles with clean separation of concerns.
"""
import logging
import threading
from collections import deque
from typing import Optional, List, Dict, Any
from datetime import datetime

from interfaces.jobs_queue_interface import IJobsQueue, JobsQueueError
from interfaces.task_handler_interface import Task


class JobsQueueImpl(IJobsQueue):
    """
    Production implementation of JobsQueue.
    
    Features:
    - Thread-safe FIFO task queue using deque and RLock
    - Task distribution tracking (which robot got which task)
    - Queue monitoring and statistics
    - Task lifecycle management
    - Comprehensive error handling
    """
    
    def __init__(self, max_queue_size: int = 1000):
        """
        Initialize JobsQueue.
        
        Args:
            max_queue_size: Maximum number of tasks in queue (prevents memory issues)
        """
        self._max_queue_size = max_queue_size
        
        # Thread-safe task queue (FIFO)
        self._task_queue: deque[Task] = deque()
        self._lock = threading.RLock()
        
        # Task tracking
        self._task_index: Dict[str, Task] = {}  # task_id -> Task for fast lookup
        self._assigned_tasks: Dict[str, str] = {}  # task_id -> robot_id
        
        # Statistics
        self._total_enqueued = 0
        self._total_dequeued = 0
        self._queue_created_at = datetime.now()
        
        # Logging
        self._logger = logging.getLogger(f"{self.__class__.__name__}")
        self._logger.info("JobsQueue initialized with max size %d", max_queue_size)
    
    def enqueue_task(self, task: Task) -> None:
        """
        Add a task to the queue.
        
        Args:
            task: Task to add to queue
            
        Raises:
            JobsQueueError: If task cannot be enqueued
        """
        if not task:
            raise JobsQueueError("Cannot enqueue None task")
        
        if not task.task_id:
            raise JobsQueueError("Task must have a task_id")
        
        with self._lock:
            # Check queue size limit
            if len(self._task_queue) >= self._max_queue_size:
                raise JobsQueueError(f"Queue is full (max size: {self._max_queue_size})")
            
            # Check for duplicate task IDs
            if task.task_id in self._task_index:
                raise JobsQueueError(f"Task with ID {task.task_id} already exists in queue")
            
            # Add to queue and index
            self._task_queue.append(task)
            self._task_index[task.task_id] = task
            self._total_enqueued += 1
            
            self._logger.debug(f"Enqueued task {task.task_id} for order {task.order_id}")
    
    def dequeue_task(self, robot_id: str) -> Optional[Task]:
        """
        Get the next available task for a robot.
        
        Args:
            robot_id: ID of the robot requesting work
            
        Returns:
            Optional[Task]: Next task if available, None otherwise
        """
        if not robot_id:
            raise JobsQueueError("robot_id cannot be empty")
        
        with self._lock:
            if not self._task_queue:
                return None
            
            # Get next task (FIFO)
            task = self._task_queue.popleft()
            
            # Update tracking
            self._assigned_tasks[task.task_id] = robot_id
            self._total_dequeued += 1
            
            # Remove from index (task is no longer in queue)
            del self._task_index[task.task_id]
            
            self._logger.debug(f"Dequeued task {task.task_id} to robot {robot_id}")
            return task
    
    def peek_next_task(self) -> Optional[Task]:
        """
        Peek at the next task without removing it from queue.
        
        Returns:
            Optional[Task]: Next task if available, None otherwise
        """
        with self._lock:
            if not self._task_queue:
                return None
            return self._task_queue[0]
    
    def get_queue_size(self) -> int:
        """
        Get the current size of the queue.
        
        Returns:
            int: Number of tasks in queue
        """
        with self._lock:
            return len(self._task_queue)
    
    def get_pending_tasks(self) -> List[Task]:
        """
        Get all pending tasks in the queue.
        
        Returns:
            List[Task]: List of pending tasks (copy of queue contents)
        """
        with self._lock:
            return list(self._task_queue)
    
    def remove_task(self, task_id: str) -> bool:
        """
        Remove a specific task from the queue.
        
        Args:
            task_id: ID of task to remove
            
        Returns:
            bool: True if task was removed, False if not found
        """
        if not task_id:
            return False
        
        with self._lock:
            if task_id not in self._task_index:
                return False
            
            # Find and remove task from queue
            task_to_remove = self._task_index[task_id]
            try:
                self._task_queue.remove(task_to_remove)
                del self._task_index[task_id]
                self._logger.debug(f"Removed task {task_id} from queue")
                return True
            except ValueError:
                # Task not in queue (shouldn't happen due to index, but be safe)
                self._logger.warning(f"Task {task_id} was in index but not in queue")
                if task_id in self._task_index:
                    del self._task_index[task_id]
                return False
    
    def clear_queue(self) -> None:
        """
        Clear all tasks from the queue.
        """
        with self._lock:
            cleared_count = len(self._task_queue)
            self._task_queue.clear()
            self._task_index.clear()
            # Note: We don't clear _assigned_tasks as those are already assigned to robots
            
            if cleared_count > 0:
                self._logger.info(f"Cleared {cleared_count} tasks from queue")
    
    def get_task_by_id(self, task_id: str) -> Optional[Task]:
        """
        Get a specific task by ID.
        
        Args:
            task_id: Task ID to look for
            
        Returns:
            Optional[Task]: Task if found, None otherwise
        """
        if not task_id:
            return None
        
        with self._lock:
            return self._task_index.get(task_id)
    
    # Additional utility methods for monitoring and debugging
    
    def get_queue_stats(self) -> Dict[str, Any]:
        """
        Get comprehensive queue statistics.
        
        Returns:
            Dict[str, Any]: Queue statistics
        """
        with self._lock:
            return {
                "current_queue_size": len(self._task_queue),
                "max_queue_size": self._max_queue_size,
                "total_enqueued": self._total_enqueued,
                "total_dequeued": self._total_dequeued,
                "total_assigned": len(self._assigned_tasks),
                "queue_created_at": self._queue_created_at,
                "queue_utilization": len(self._task_queue) / self._max_queue_size if self._max_queue_size > 0 else 0.0
            }
    
    def get_assigned_tasks(self) -> Dict[str, str]:
        """
        Get mapping of assigned tasks to robots.
        
        Returns:
            Dict[str, str]: task_id -> robot_id mapping
        """
        with self._lock:
            return dict(self._assigned_tasks)
    
    def get_tasks_by_order(self, order_id: str) -> List[Task]:
        """
        Get all tasks for a specific order.
        
        Args:
            order_id: Order ID to search for
            
        Returns:
            List[Task]: Tasks belonging to the order
        """
        if not order_id:
            return []
        
        with self._lock:
            return [task for task in self._task_queue if task.order_id == order_id]
    
    def get_tasks_by_priority(self, priority: str) -> List[Task]:
        """
        Get all tasks with specific priority.
        
        Args:
            priority: Priority level to search for
            
        Returns:
            List[Task]: Tasks with the specified priority
        """
        if not priority:
            return []
        
        with self._lock:
            return [task for task in self._task_queue 
                   if hasattr(task, 'order_priority') and task.order_priority == priority]
    
    def is_empty(self) -> bool:
        """
        Check if queue is empty.
        
        Returns:
            bool: True if queue is empty
        """
        with self._lock:
            return len(self._task_queue) == 0
    
    def is_full(self) -> bool:
        """
        Check if queue is full.
        
        Returns:
            bool: True if queue is at max capacity
        """
        with self._lock:
            return len(self._task_queue) >= self._max_queue_size
    
    def __len__(self) -> int:
        """Support len() function."""
        return self.get_queue_size()
    
    def __bool__(self) -> bool:
        """Support bool() function - True if queue has tasks."""
        return not self.is_empty()
    
    def __str__(self) -> str:
        """String representation."""
        with self._lock:
            return f"JobsQueue(size={len(self._task_queue)}/{self._max_queue_size}, " \
                   f"enqueued={self._total_enqueued}, dequeued={self._total_dequeued})"
    
    def __repr__(self) -> str:
        """Detailed representation."""
        return self.__str__() 