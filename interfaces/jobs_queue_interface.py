"""
Interface for JobsQueue - main interface for robots to receive work.
"""
from abc import ABC, abstractmethod
from typing import Optional, List
from .task_handler_interface import Task


class JobsQueueError(Exception):
    """Raised when jobs queue operations fail."""
    pass


class IJobsQueue(ABC):
    """
    Interface for job queue functionality.
    
    Responsibilities:
    - Act as main interface for robots to receive work
    - Support FIFO-style task handling
    - Handle task distribution to robots
    - For Phase 2: in-memory implementation, no database
    
    **Thread Safety**: ALL methods are fully thread-safe for concurrent access.
    **Threading Model**:
    - enqueue_task(): CONTROL THREAD (JobsProcessor thread)
    - dequeue_task(): CONTROL THREAD (TaskHandler from any robot)
    - All other methods: ANY THREAD (monitoring, management)
    """
    
    @abstractmethod
    def enqueue_task(self, task: Task) -> None:
        """
        Add a task to the queue.
        
        Args:
            task: Task to add to queue
            
        Raises:
            JobsQueueError: If task cannot be enqueued
        """
        pass
    
    @abstractmethod
    def dequeue_task(self, robot_id: str) -> Optional[Task]:
        """
        Get the next available task for a robot.
        
        Args:
            robot_id: ID of the robot requesting work
            
        Returns:
            Optional[Task]: Next task if available, None otherwise
        """
        pass
    
    @abstractmethod
    def peek_next_task(self) -> Optional[Task]:
        """
        Peek at the next task without removing it from queue.
        
        Returns:
            Optional[Task]: Next task if available, None otherwise
        """
        pass
    
    @abstractmethod
    def get_queue_size(self) -> int:
        """
        Get the current size of the queue.
        
        Returns:
            int: Number of tasks in queue
        """
        pass
    
    @abstractmethod
    def get_pending_tasks(self) -> List[Task]:
        """
        Get all pending tasks in the queue.
        
        Returns:
            List[Task]: List of pending tasks
        """
        pass
    
    @abstractmethod
    def remove_task(self, task_id: str) -> bool:
        """
        Remove a specific task from the queue.
        
        Args:
            task_id: ID of task to remove
            
        Returns:
            bool: True if task was removed, False if not found
        """
        pass
    
    @abstractmethod
    def clear_queue(self) -> None:
        """
        Clear all tasks from the queue.
        """
        pass
    
    @abstractmethod
    def get_task_by_id(self, task_id: str) -> Optional[Task]:
        """
        Get a specific task by ID.
        
        Args:
            task_id: Task ID to look for
            
        Returns:
            Optional[Task]: Task if found, None otherwise
        """
        pass 