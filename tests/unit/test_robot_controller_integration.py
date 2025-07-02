"""
Integration tests for Robot Controller with Jobs Queue and Bidding System.

Tests the complete workflow:
1. Jobs queue provides tasks
2. Robot controller polls queue
3. Bidding system assigns tasks
4. Robots receive assignments

This tests the integration between all components following our
established principles and interface contracts.
"""
import pytest
import time
from typing import List, Dict, Any
from unittest.mock import Mock, patch
from datetime import datetime

from warehouse.impl.robot_controller_impl import RobotController, RobotControllerError
from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
from warehouse.impl.jobs_queue_impl import JobsQueueImpl
from interfaces.task_handler_interface import Task, TaskType, TaskStatus
from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.bidding_system_interface import IBiddingSystem


class MockRobotAgent:
    """Mock RobotAgent for integration testing."""
    
    def __init__(self, robot_id: str, position: tuple = (0.0, 0.0), 
                 battery: float = 80.0, is_idle: bool = True):
        self._robot_id = robot_id
        self._position = position
        self._battery = battery
        self._is_idle = is_idle
        self._assigned_tasks = []
        self._completed_tasks = []
        
        # Mock config to match RobotAgent interface
        self.config = Mock()
        self.config.robot_id = robot_id
        
        # Mock task handler interface
        self.task_handler_interface = Mock()
        self.task_handler_interface.is_idle.return_value = is_idle
        
        # Task completion simulation
        self._task_completion_thread = None
        self._stop_completion_thread = False
    
    def get_robot_id(self) -> str:
        return self._robot_id
    
    def get_status(self) -> Dict[str, Any]:
        return {
            'robot_id': self._robot_id,
            'position': self._position,
            'battery': self._battery,
            'task_active': not self._is_idle,
            'operational_status': 'idle' if self._is_idle else 'busy'
        }
    
    def assign_task(self, task: Task) -> bool:
        if not self._is_idle:
            return False
        
        # Accept the task
        self._is_idle = False
        self._assigned_tasks.append(task)
        self.task_handler_interface.is_idle.return_value = False
        
        # Simulate task completion after a short delay
        self._simulate_task_completion(task)
        
        return True
    
    def _simulate_task_completion(self, task: Task) -> None:
        """Simulate task completion by marking it complete and making robot available."""
        import threading
        
        def complete_task():
            # Mark task as completed immediately
            task.status = TaskStatus.COMPLETED
            task.completed_at = datetime.now()
            
            # Move task from assigned to completed
            if task in self._assigned_tasks:
                self._assigned_tasks.remove(task)
            self._completed_tasks.append(task)
            
            # Make robot available again
            self._is_idle = True
            self.task_handler_interface.is_idle.return_value = True
            
            print(f"[MockRobotAgent] {self._robot_id} completed task {task.task_id}")
        
        # Start completion thread (immediate completion)
        completion_thread = threading.Thread(target=complete_task, daemon=True)
        completion_thread.start()
    
    def get_assigned_tasks(self) -> List[Task]:
        return self._assigned_tasks.copy()
    
    def get_completed_tasks(self) -> List[Task]:
        return self._completed_tasks.copy()


class TestRobotControllerIntegration:
    """Test Robot Controller integration with Jobs Queue and Bidding System."""
    
    def setup_method(self):
        """Set up test fixtures."""
        # Create real implementations for integration testing
        self.jobs_queue = JobsQueueImpl()
        self.bidding_system = TransparentBiddingSystem()
        
        # Create mock robots
        self.robot1 = MockRobotAgent("robot_1", position=(0.0, 0.0), battery=80.0)
        self.robot2 = MockRobotAgent("robot_2", position=(5.0, 5.0), battery=60.0)
        self.robot_pool = [self.robot1, self.robot2]
        
        # Create robot controller
        self.controller = RobotController(
            jobs_queue=self.jobs_queue,
            bidding_system=self.bidding_system,
            robot_pool=self.robot_pool,
            polling_interval=0.1  # Fast polling for tests
        )
        
        # Create test tasks
        self.task1 = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        self.task2 = Task.create_pick_and_deliver_task("task_2", "order_2", "shelf_2", "item_2")
        self.task3 = Task.create_pick_and_deliver_task("task_3", "order_3", "shelf_3", "item_3")
    
    def test_initialization(self):
        """Test robot controller initialization."""
        status = self.controller.get_controller_status()
        
        assert status['running'] is False
        assert status['total_rounds_processed'] == 0
        assert status['total_tasks_assigned'] == 0
        assert status['robot_count'] == 2
        assert status['polling_interval'] == 0.1
    
    def test_robot_status_summary(self):
        """Test getting robot status summary."""
        robot_statuses = self.controller.get_robot_status_summary()
        
        assert len(robot_statuses) == 2
        
        # Check robot 1 status
        robot1_status = next(s for s in robot_statuses if s['robot_id'] == 'robot_1')
        assert robot1_status['robot_id'] == 'robot_1'
        assert robot1_status['available_for_bidding'] is True
        assert robot1_status['status']['battery'] == 80.0
        
        # Check robot 2 status
        robot2_status = next(s for s in robot_statuses if s['robot_id'] == 'robot_2')
        assert robot2_status['robot_id'] == 'robot_2'
        assert robot2_status['available_for_bidding'] is True
        assert robot2_status['status']['battery'] == 60.0
    
    def test_add_remove_robot(self):
        """Test adding and removing robots from the pool."""
        new_robot = MockRobotAgent("robot_3")
        
        # Add robot
        self.controller.add_robot(new_robot)
        status = self.controller.get_controller_status()
        assert status['robot_count'] == 3
        
        # Remove robot
        self.controller.remove_robot(new_robot)
        status = self.controller.get_controller_status()
        assert status['robot_count'] == 2
    
    def test_process_single_round_no_tasks(self):
        """Test processing a single round with no tasks in queue."""
        round_data = self.controller.process_single_round()
        
        assert round_data is None
        
        status = self.controller.get_controller_status()
        assert status['total_rounds_processed'] == 0
        assert status['total_tasks_assigned'] == 0
    
    def test_process_single_round_with_tasks(self):
        """Test processing a single round with tasks in queue."""
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        self.jobs_queue.enqueue_task(self.task2)
        
        # Process round
        round_data = self.controller.process_single_round()
        
        assert round_data is not None
        assert len(round_data.available_tasks) == 2
        assert len(round_data.submitted_bids) == 4  # 2 robots × 2 tasks
        assert len(round_data.winning_assignments) == 2  # One assignment per task
        
        # Check statistics
        status = self.controller.get_controller_status()
        assert status['total_rounds_processed'] == 1
        assert status['total_tasks_assigned'] == 2
    
    def test_task_assignment_to_robots(self):
        """Test that tasks are actually assigned to robots."""
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        self.jobs_queue.enqueue_task(self.task2)
        
        # Process round
        self.controller.process_single_round()
        
        # Check that robots received tasks
        robot1_tasks = self.robot1.get_assigned_tasks()
        robot2_tasks = self.robot2.get_assigned_tasks()
        
        assert len(robot1_tasks) == 1
        assert len(robot2_tasks) == 1
        
        # Check that robots are no longer idle
        assert not self.robot1._is_idle
        assert not self.robot2._is_idle
    
    def test_busy_robot_not_assigned(self):
        """Test that busy robots are not assigned new tasks."""
        # Make robot 1 busy
        self.robot1._is_idle = False
        self.robot1.task_handler_interface.is_idle.return_value = False
        
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        self.jobs_queue.enqueue_task(self.task2)
        
        # Process round
        round_data = self.controller.process_single_round()
        
        assert round_data is not None
        assert len(round_data.submitted_bids) == 2  # Only robot 2 bids (robot 1 is busy)
        assert len(round_data.winning_assignments) == 2  # Both tasks assigned to robot 2
        
        # Check that only robot 2 received tasks
        robot1_tasks = self.robot1.get_assigned_tasks()
        robot2_tasks = self.robot2.get_assigned_tasks()
        
        assert len(robot1_tasks) == 0
        assert len(robot2_tasks) == 2
    
    def test_low_battery_robot_not_assigned(self):
        """Test that robots with low battery are not assigned tasks."""
        # Set robot 1 to low battery
        self.robot1._battery = 15.0
        
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        self.jobs_queue.enqueue_task(self.task2)
        
        # Process round
        round_data = self.controller.process_single_round()
        
        assert round_data is not None
        assert len(round_data.submitted_bids) == 2  # Only robot 2 bids (robot 1 has low battery)
        assert len(round_data.winning_assignments) == 2  # Both tasks assigned to robot 2
        
        # Check that only robot 2 received tasks
        robot1_tasks = self.robot1.get_assigned_tasks()
        robot2_tasks = self.robot2.get_assigned_tasks()
        
        assert len(robot1_tasks) == 0
        assert len(robot2_tasks) == 2
    
    def test_controller_start_stop(self):
        """Test starting and stopping the controller."""
        # Start controller
        self.controller.start()
        
        status = self.controller.get_controller_status()
        assert status['running'] is True
        assert status['controller_start_time'] is not None
        
        # Stop controller
        self.controller.stop()
        
        status = self.controller.get_controller_status()
        assert status['running'] is False
    
    def test_continuous_operation(self):
        """Test continuous operation with multiple rounds."""
        # Start controller
        self.controller.start()
        
        # Add tasks periodically
        time.sleep(0.05)  # Wait for first poll
        self.jobs_queue.enqueue_task(self.task1)
        
        time.sleep(0.05)  # Wait for second poll
        self.jobs_queue.enqueue_task(self.task2)
        
        time.sleep(0.05)  # Wait for third poll
        self.jobs_queue.enqueue_task(self.task3)
        
        # Stop controller
        self.controller.stop()
        
        # Check that tasks were processed
        status = self.controller.get_controller_status()
        assert status['total_rounds_processed'] > 0
        assert status['total_tasks_assigned'] > 0
    
    def test_error_handling_invalid_robot(self):
        """Test error handling with invalid robot data."""
        # Create robot that raises exception
        invalid_robot = Mock()
        invalid_robot.get_robot_id.side_effect = Exception("Robot error")
        invalid_robot.get_status.side_effect = Exception("Status error")
        
        # Add invalid robot to pool
        self.controller.add_robot(invalid_robot)
        
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        
        # Process round (should handle errors gracefully)
        round_data = self.controller.process_single_round()
        
        # Should still process with valid robots
        assert round_data is not None
        assert len(round_data.winning_assignments) > 0
    
    def test_error_handling_queue_failure(self):
        """Test error handling when jobs queue fails."""
        # Create queue that raises exception
        failing_queue = Mock(spec=IJobsQueue)
        failing_queue.dequeue_task.side_effect = Exception("Queue error")
        
        # Create controller with failing queue
        controller = RobotController(
            jobs_queue=failing_queue,
            bidding_system=self.bidding_system,
            robot_pool=self.robot_pool
        )
        
        # Process round (should handle errors gracefully)
        round_data = controller.process_single_round()
        
        # Should return None due to queue error
        assert round_data is None
    
    def test_error_handling_bidding_failure(self):
        """Test error handling when bidding system fails."""
        # Create bidding system that raises exception
        failing_bidding_system = Mock(spec=IBiddingSystem)
        failing_bidding_system.process_bidding_round.side_effect = Exception("Bidding error")
        
        # Create controller with failing bidding system
        controller = RobotController(
            jobs_queue=self.jobs_queue,
            bidding_system=failing_bidding_system,
            robot_pool=self.robot_pool
        )
        
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        
        # Process round (should handle errors gracefully)
        round_data = controller.process_single_round()
        
        # Should return None due to bidding error
        assert round_data is None
    
    def test_multiple_rounds_statistics(self):
        """Test statistics tracking across multiple rounds."""
        # Process multiple rounds
        for i in range(3):
            self.jobs_queue.enqueue_task(Task.create_pick_and_deliver_task(
                f"task_{i}", f"order_{i}", f"shelf_{i}", f"item_{i}"
            ))
            self.controller.process_single_round()
        
        # Check statistics
        status = self.controller.get_controller_status()
        assert status['total_rounds_processed'] == 3
        assert status['total_tasks_assigned'] == 3
        assert status['last_round_timestamp'] is not None
    
    def test_robot_finding_by_id(self):
        """Test finding robots by ID for task assignment."""
        # Test finding existing robot
        robot = self.controller._find_robot_by_id("robot_1")
        assert robot is not None
        assert robot.get_robot_id() == "robot_1"
        
        # Test finding non-existent robot
        robot = self.controller._find_robot_by_id("non_existent")
        assert robot is None
    
    def test_edge_case_empty_robot_pool(self):
        """Test behavior with empty robot pool."""
        # Create controller with empty robot pool
        controller = RobotController(
            jobs_queue=self.jobs_queue,
            bidding_system=self.bidding_system,
            robot_pool=[]
        )
        
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        
        # Process round
        round_data = controller.process_single_round()
        
        # Should return None due to no available robots
        assert round_data is None
    
    def test_edge_case_all_robots_busy(self):
        """Test behavior when all robots are busy."""
        # Make all robots busy
        self.robot1._is_idle = False
        self.robot1.task_handler_interface.is_idle.return_value = False
        self.robot2._is_idle = False
        self.robot2.task_handler_interface.is_idle.return_value = False
        
        # Add tasks to queue
        self.jobs_queue.enqueue_task(self.task1)
        self.jobs_queue.enqueue_task(self.task2)
        
        # Process round
        round_data = self.controller.process_single_round()
        
        # Should return None due to no available robots
        assert round_data is None
    
    def test_thread_safety(self):
        """Test thread safety of the controller."""
        results = []
        errors = []
        
        def worker(worker_id: int):
            try:
                # Add task to queue
                task = Task.create_pick_and_deliver_task(
                    f"task_{worker_id}", f"order_{worker_id}", 
                    f"shelf_{worker_id}", f"item_{worker_id}"
                )
                self.jobs_queue.enqueue_task(task)
                
                # Process round
                round_data = self.controller.process_single_round()
                results.append(round_data)
                
            except Exception as e:
                errors.append(e)
        
        # Create multiple threads
        import threading
        threads = []
        for i in range(5):
            thread = threading.Thread(target=worker, args=(i,))
            threads.append(thread)
        
        # Start all threads
        for thread in threads:
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Verify no errors occurred
        assert len(errors) == 0, f"Thread safety test failed with errors: {errors}"
        
        # Verify all rounds were processed
        assert len(results) == 5
        
        # Verify statistics are consistent
        status = self.controller.get_controller_status()
        assert status['total_rounds_processed'] == 5 