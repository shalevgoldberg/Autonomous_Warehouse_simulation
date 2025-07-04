"""
Robot Controller Implementation - orchestrates bidding, jobs queue, and robot agents.

This controller coordinates the entire robot task assignment workflow:
1. Polls the jobs queue for available tasks
2. Collects available robots from the robot pool
3. Uses the bidding system to assign tasks to robots
4. Monitors task execution and robot status

Design Principles:
- **Single Responsibility**: Only handles coordination and orchestration
- **Open/Closed**: Extensible for different bidding strategies and robot types
- **Liskov Substitution**: Works with any IBiddingSystem implementation
- **Interface Segregation**: Depends only on required interfaces
- **Dependency Inversion**: Depends on abstractions, not concrete implementations

Thread Safety: All methods are thread-safe for concurrent access.
"""
import threading
import time
from typing import List, Optional, Dict, Any
from datetime import datetime

from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.bidding_system_interface import IBiddingSystem, BiddingRound, TaskAssignment
from interfaces.task_handler_interface import Task
from interfaces.configuration_interface import IConfigurationProvider
from robot.robot_agent import RobotAgent


class RobotControllerError(Exception):
    """Raised when robot controller operations fail."""
    pass


class RobotController:
    """
    Robot Controller - orchestrates task assignment and robot coordination.
    
    This controller manages the complete workflow from task discovery
    to robot assignment, providing a clean interface for warehouse
    automation systems.
    
    **Thread Safety**: All methods are thread-safe for concurrent access.
    **Threading Model**:
    - start(): MAIN THREAD (initialization)
    - stop(): MAIN THREAD (shutdown)
    - _control_loop(): CONTROL THREAD (continuous operation)
    - get_*() methods: ANY THREAD (status queries)
    
    Integration Points:
    - **JobsQueue**: For getting available tasks
    - **BiddingSystem**: For task assignment decisions
    - **Robot Pool**: For getting available robots
    - **Monitoring Systems**: For performance tracking
    """
    
    def __init__(self, 
                 jobs_queue: IJobsQueue,
                 bidding_system: IBiddingSystem,
                 robot_pool: List[Any],
                 config_provider: Optional[IConfigurationProvider] = None,
                 polling_interval: Optional[float] = None):
        """
        Initialize the robot controller.
        
        Args:
            jobs_queue: Queue containing available tasks
            bidding_system: Bidding system for task assignment
            robot_pool: List of available robot agents
            config_provider: Configuration provider for system parameters
            polling_interval: How often to poll for new tasks (seconds)
        """
        self.jobs_queue = jobs_queue
        self.bidding_system = bidding_system
        self.robot_pool = robot_pool
        self.config_provider = config_provider
        
        # Get configuration from provider or use provided/default values
        if config_provider:
            system_config = config_provider.get_system_config()
            self.polling_interval = polling_interval or system_config.health_check_interval
            # Use reasonable defaults for robot controller specific parameters
            self.max_robots_per_round = 10
            self.max_tasks_per_round = 20
            self.round_timeout = 30.0
        else:
            # Use provided values or defaults
            self.polling_interval = polling_interval or 1.0
            self.max_robots_per_round = 10
            self.max_tasks_per_round = 20
            self.round_timeout = 30.0
        
        # Control loop management
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        
        # Statistics and monitoring
        self._total_rounds_processed = 0
        self._total_tasks_assigned = 0
        self._last_round_timestamp: Optional[datetime] = None
        self._controller_start_time: Optional[datetime] = None
        
        print(f"[RobotController] Initialized with {len(robot_pool)} robots")
    
    def update_config_from_provider(self) -> None:
        """Update controller parameters from configuration provider."""
        if self.config_provider:
            system_config = self.config_provider.get_system_config()
            self.polling_interval = system_config.health_check_interval
            # Keep existing values for robot controller specific parameters
            # These could be added to SystemConfig in the future if needed
    
    def start(self) -> None:
        """
        Start the robot controller.
        
        This begins the continuous control loop that polls for tasks
        and coordinates robot assignments.
        """
        with self._lock:
            if self._running:
                print("[RobotController] Already running")
                return
            
            self._running = True
            self._controller_start_time = datetime.now()
            
            # Start control thread
            self._control_thread = threading.Thread(
                target=self._control_loop,
                name="RobotController",
                daemon=True
            )
            self._control_thread.start()
            
            print("[RobotController] Started control loop")
    
    def stop(self) -> None:
        """
        Stop the robot controller.
        
        This gracefully shuts down the control loop and waits for
        the thread to complete.
        """
        with self._lock:
            if not self._running:
                return
            
            self._running = False
            
            # Wait for control thread to finish
            if self._control_thread:
                self._control_thread.join(timeout=5.0)
            
            print("[RobotController] Stopped")
    
    def get_controller_status(self) -> Dict[str, Any]:
        """
        Get comprehensive controller status.
        
        Returns:
            Dict[str, Any]: Controller status information
        """
        with self._lock:
            return {
                'running': self._running,
                'total_rounds_processed': self._total_rounds_processed,
                'total_tasks_assigned': self._total_tasks_assigned,
                'last_round_timestamp': self._last_round_timestamp,
                'controller_start_time': self._controller_start_time,
                'robot_count': len(self.robot_pool),
                'polling_interval': self.polling_interval
            }
    
    def get_robot_status_summary(self) -> List[Dict[str, Any]]:
        """
        Get status summary for all robots.
        
        Returns:
            List[Dict[str, Any]]: Status information for each robot
        """
        with self._lock:
            robot_statuses = []
            for robot in self.robot_pool:
                try:
                    status = robot.get_status()
                    robot_statuses.append({
                        'robot_id': robot.config.robot_id,
                        'status': status,
                        'available_for_bidding': self._is_robot_available(robot)
                    })
                except Exception as e:
                    print(f"[RobotController] Error getting status for robot: {e}")
                    robot_statuses.append({
                        'robot_id': 'unknown',
                        'status': {'error': str(e)},
                        'available_for_bidding': False
                    })
            
            return robot_statuses
    
    def add_robot(self, robot: Any) -> None:
        """
        Add a robot to the controller's robot pool.
        
        Args:
            robot: Robot agent to add
        """
        with self._lock:
            if robot not in self.robot_pool:
                self.robot_pool.append(robot)
                print(f"[RobotController] Added robot {robot.config.robot_id}")
    
    def remove_robot(self, robot: Any) -> None:
        """
        Remove a robot from the controller's robot pool.
        
        Args:
            robot: Robot agent to remove
        """
        with self._lock:
            if robot in self.robot_pool:
                self.robot_pool.remove(robot)
                print(f"[RobotController] Removed robot {robot.config.robot_id}")
    
    def process_single_round(self) -> Optional[BiddingRound]:
        """
        Process a single bidding round manually.
        
        This method can be called externally to trigger a bidding round
        without waiting for the polling interval.
        
        Returns:
            Optional[BiddingRound]: Bidding round results, or None if no tasks/robots
        """
        with self._lock:
            try:
                return self._process_bidding_round()
            except Exception as e:
                print(f"[RobotController] Error in manual round processing: {e}")
                return None
    
    def _control_loop(self) -> None:
        """
        Main control loop for continuous task assignment.
        
        This loop runs continuously while the controller is active,
        polling for tasks and coordinating robot assignments.
        """
        print(f"[RobotController] Control loop started, polling every {self.polling_interval}s")
        
        while self._running:
            try:
                # Process bidding round
                round_data = self._process_bidding_round()
                
                if round_data:
                    print(f"[RobotController] Processed round {round_data.round_id} with "
                          f"{len(round_data.winning_assignments)} assignments")
                
                # Wait for next polling interval
                time.sleep(self.polling_interval)
                
            except Exception as e:
                print(f"[RobotController] Error in control loop: {e}")
                time.sleep(self.polling_interval)  # Continue despite errors
        
        print("[RobotController] Control loop stopped")
    
    def _process_bidding_round(self) -> Optional[BiddingRound]:
        """
        Process a complete bidding round.
        
        This method:
        1. Gets available tasks from the jobs queue
        2. Gets available robots from the robot pool
        3. Runs the bidding system to assign tasks
        4. Assigns tasks to robots
        5. Updates statistics
        
        Returns:
            Optional[BiddingRound]: Bidding round results, or None if no tasks/robots
        """
        # Get available tasks from jobs queue
        available_tasks = self._get_available_tasks()
        if not available_tasks:
            return None
        
        # Get available robots
        available_robots = self._get_available_robots()
        if not available_robots:
            print("[RobotController] No available robots for bidding")
            return None
        
        # Process bidding round
        try:
            round_data = self.bidding_system.process_bidding_round(
                available_tasks, available_robots
            )
            
            # Assign tasks to robots
            self._assign_tasks_to_robots(round_data.winning_assignments)
            
            # Update statistics
            self._update_statistics(round_data)
            
            return round_data
            
        except Exception as e:
            print(f"[RobotController] Error processing bidding round: {e}")
            return None
    
    def _get_available_tasks(self) -> List[Task]:
        """
        Get available tasks from the jobs queue.
        
        Returns:
            List[Task]: Available tasks for bidding
        """
        try:
            # Get all pending tasks from the queue
            tasks = self.jobs_queue.get_pending_tasks()
            
            if tasks:
                print(f"[RobotController] Retrieved {len(tasks)} tasks from queue")
            
            return tasks
            
        except Exception as e:
            print(f"[RobotController] Error getting tasks from queue: {e}")
            return []
    
    def _get_available_robots(self) -> List[Any]:
        """
        Get available robots from the robot pool.
        
        Returns:
            List[Any]: Available robots for bidding
        """
        available_robots = []
        
        for robot in self.robot_pool:
            if self._is_robot_available(robot):
                available_robots.append(robot)
        
        return available_robots
    
    def _is_robot_available(self, robot: Any) -> bool:
        """
        Check if a robot is available for task assignment.
        
        Args:
            robot: Robot to check
            
        Returns:
            bool: True if robot is available
        """
        try:
            # Check if robot is idle (no active task)
            status = robot.get_status()
            if status.get('task_active', True):  # Default to True for safety
                return False
            
            # Check operational status
            operational_status = status.get('operational_status', 'error')
            if operational_status in ['error', 'emergency_stop', 'stalled']:
                return False
            
            # Check battery level
            battery_level = status.get('battery', 0.0)
            if battery_level < 20.0:  # 20% battery threshold
                return False
            
            return True
            
        except Exception as e:
            print(f"[RobotController] Error checking robot availability: {e}")
            return False
    
    def _assign_tasks_to_robots(self, assignments: List[TaskAssignment]) -> None:
        """
        Assign tasks to robots based on bidding results.
        
        Args:
            assignments: List of task assignments from bidding system
        """
        successful_assignments = 0
        
        for assignment in assignments:
            try:
                # Find the robot by ID
                robot = self._find_robot_by_id(assignment.robot_id)
                if robot is None:
                    print(f"[RobotController] Robot {assignment.robot_id} not found for assignment")
                    continue
                
                # Assign the task to the robot
                if robot.assign_task(assignment.task):
                    successful_assignments += 1
                    print(f"[RobotController] Assigned task {assignment.task.task_id} to robot {assignment.robot_id}")
                else:
                    print(f"[RobotController] Failed to assign task {assignment.task.task_id} to robot {assignment.robot_id}")
                
            except Exception as e:
                print(f"[RobotController] Error assigning task {assignment.task.task_id} to robot {assignment.robot_id}: {e}")
        
        print(f"[RobotController] Successfully assigned {successful_assignments}/{len(assignments)} tasks")
    
    def _find_robot_by_id(self, robot_id: str) -> Optional[Any]:
        """
        Find a robot in the pool by its ID.
        
        Args:
            robot_id: Robot identifier
            
        Returns:
            Optional[Any]: Robot agent, or None if not found
        """
        for robot in self.robot_pool:
            try:
                if robot.config.robot_id == robot_id:
                    return robot
            except Exception:
                continue
        
        return None
    
    def _update_statistics(self, round_data: BiddingRound) -> None:
        """
        Update controller statistics with bidding round data.
        
        Args:
            round_data: Bidding round data
        """
        self._total_rounds_processed += 1
        self._total_tasks_assigned += len(round_data.winning_assignments)
        self._last_round_timestamp = round_data.round_timestamp 