"""
Robot Controller Implementation - orchestrates parallel asynchronous bidding and robot coordination.

This controller coordinates the entire robot task assignment workflow using parallel bidding:
1. Polls the jobs queue for available tasks
2. Sends tasks to all available robots simultaneously for parallel bid calculation
3. Collects bids from robots asynchronously
4. Selects winning bids and assigns tasks to robots
5. Monitors task execution and robot status

Design Principles:
- **Single Responsibility**: Only handles coordination and orchestration
- **Open/Closed**: Extensible for different bidding strategies and robot types
- **Liskov Substitution**: Works with any robot agent that implements the bidding interface
- **Interface Segregation**: Depends only on required interfaces
- **Dependency Inversion**: Depends on abstractions, not concrete implementations
- **Parallel Processing**: Uses concurrent bid calculation for scalability

Thread Safety: All methods are thread-safe for concurrent access.
"""
import threading
import time
import asyncio
from typing import List, Optional, Dict, Any, Tuple
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass

from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.bidding_system_interface import IBiddingSystem, BiddingRound, TaskAssignment, RobotBid
from interfaces.task_handler_interface import Task
from interfaces.configuration_interface import IConfigurationProvider, BidConfig
from robot.robot_agent_lane_based import RobotAgent


@dataclass
class ParallelBiddingResult:
    """Result of parallel bidding round."""
    round_id: str
    available_tasks: List[Task]
    available_robots: List[Any]
    all_bids: List[RobotBid]
    winning_assignments: List[TaskAssignment]
    round_duration: float
    robot_bid_times: Dict[str, float]
    successful_robots: int
    failed_robots: int
    round_metadata: Dict[str, Any]


class RobotControllerError(Exception):
    """Raised when robot controller operations fail."""
    pass


class RobotController:
    """
    Robot Controller - orchestrates parallel asynchronous task assignment and robot coordination.
    
    This controller manages the complete workflow from task discovery
    to robot assignment using parallel bidding for improved performance
    and scalability.
    
    **Thread Safety**: All methods are thread-safe for concurrent access.
    **Threading Model**:
    - start(): MAIN THREAD (initialization)
    - stop(): MAIN THREAD (shutdown)
    - _control_loop(): CONTROL THREAD (continuous operation)
    - _process_parallel_bidding(): CONTROL THREAD (parallel bid collection)
    - get_*() methods: ANY THREAD (status queries)
    
    Integration Points:
    - **JobsQueue**: For getting available tasks
    - **Robot Pool**: For parallel bid calculation
    - **Bidding System**: For bid selection and assignment (optional)
    - **Monitoring Systems**: For performance tracking
    """
    
    def __init__(self, 
                 jobs_queue: IJobsQueue,
                 robot_pool: List[Any],
                 config_provider: Optional[IConfigurationProvider] = None,
                 bidding_system: Optional[IBiddingSystem] = None,
                 polling_interval: Optional[float] = None):
        """
        Initialize the robot controller.
        
        Args:
            jobs_queue: Queue containing available tasks
            robot_pool: List of available robot agents
            config_provider: Configuration provider for system parameters
            bidding_system: Optional bidding system for bid selection (can use built-in)
            polling_interval: How often to poll for new tasks (seconds)
        """
        self.jobs_queue = jobs_queue
        self.robot_pool = robot_pool
        self.config_provider = config_provider
        self.bidding_system = bidding_system
        
        # Get configuration from provider or use provided/default values
        if config_provider:
            system_config = config_provider.get_system_config()
            bid_config = config_provider.get_bid_config()
            self.polling_interval = polling_interval or system_config.health_check_interval
            self.max_parallel_workers = bid_config.max_parallel_workers
            self.bidding_timeout = bid_config.calculation_timeout
            self.max_robots_per_round = 20
            self.max_tasks_per_round = 50
            self.round_timeout = 5.0  # Reduced from 30.0 to 5.0 seconds for faster tests
        else:
            # Use provided values or defaults
            self.polling_interval = polling_interval or 1.0
            self.max_parallel_workers = 4
            self.bidding_timeout = 5.0  # Increased from 1.0 to 5.0 seconds
            self.max_robots_per_round = 20
            self.max_tasks_per_round = 50
            self.round_timeout = 3.0  # Reduced from 10.0 to 3.0 seconds for faster tests
        
        # Control loop management
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        
        # Thread pool for parallel bid calculation
        self._thread_pool = ThreadPoolExecutor(max_workers=self.max_parallel_workers)
        
        # Statistics and monitoring
        self._total_rounds_processed = 0
        self._total_tasks_assigned = 0
        self._total_bids_collected = 0
        self._average_bid_time = 0.0
        self._last_round_timestamp: Optional[datetime] = None
        self._controller_start_time: Optional[datetime] = None
        
        print(f"[RobotController] Initialized with {len(robot_pool)} robots")
        print(f"[RobotController] Parallel bidding with {self.max_parallel_workers} workers")
        print(f"[RobotController] Bidding timeout: {self.bidding_timeout}s")
    
    def update_config_from_provider(self) -> None:
        """Update controller parameters from configuration provider."""
        if self.config_provider:
            system_config = self.config_provider.get_system_config()
            bid_config = self.config_provider.get_bid_config()
            self.polling_interval = system_config.health_check_interval
            self.max_parallel_workers = bid_config.max_parallel_workers
            self.bidding_timeout = bid_config.calculation_timeout
            
            # Update thread pool if needed
            if self._thread_pool._max_workers != self.max_parallel_workers:
                self._thread_pool.shutdown(wait=True)
                self._thread_pool = ThreadPoolExecutor(max_workers=self.max_parallel_workers)
    
    def start(self) -> None:
        """
        Start the robot controller.
        
        This begins the continuous control loop that polls for tasks
        and coordinates parallel robot bidding.
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
            
            # Shutdown thread pool
            self._thread_pool.shutdown(wait=True)
            
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
                'total_bids_collected': self._total_bids_collected,
                'average_bid_time': self._average_bid_time,
                'last_round_timestamp': self._last_round_timestamp,
                'controller_start_time': self._controller_start_time,
                'robot_count': len(self.robot_pool),
                'polling_interval': self.polling_interval,
                'max_parallel_workers': self.max_parallel_workers,
                'bidding_timeout': self.bidding_timeout
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
                    bid_stats = robot.get_bid_calculation_statistics()
                    
                    # Determine availability from status to avoid potential deadlock
                    available_for_bidding = (
                        not status.get('task_active', True) and
                        status.get('operational_status', 'error') == 'idle' and
                        status.get('battery', 0.0) > 20.0
                    )
                    
                    robot_statuses.append({
                        'robot_id': self._get_robot_id(robot),
                        'status': status,
                        'available_for_bidding': available_for_bidding,
                        'bid_calculation_stats': bid_stats
                    })
                except Exception as e:
                    print(f"[RobotController] Error getting status for robot: {e}")
                    robot_statuses.append({
                        'robot_id': self._get_robot_id(robot),
                        'status': {'error': str(e)},
                        'available_for_bidding': False,
                        'bid_calculation_stats': {}
                    })
            
            return robot_statuses
    
    def add_robot(self, robot: Any) -> None:
        """
        Add a robot to the pool.
        
        Args:
            robot: Robot agent to add
        """
        with self._lock:
            if robot not in self.robot_pool:
                self.robot_pool.append(robot)
                print(f"[RobotController] Added robot {self._get_robot_id(robot)} to pool")
    
    def remove_robot(self, robot: Any) -> None:
        """
        Remove a robot from the pool.
        
        Args:
            robot: Robot agent to remove
        """
        with self._lock:
            if robot in self.robot_pool:
                self.robot_pool.remove(robot)
                print(f"[RobotController] Removed robot {self._get_robot_id(robot)} from pool")
    
    def process_single_round(self) -> Optional[ParallelBiddingResult]:
        """
        Process a single bidding round manually.
        
        This method can be called externally to trigger a bidding round
        without waiting for the control loop.
        
        Returns:
            Optional[ParallelBiddingResult]: Bidding round results, or None if no tasks/robots
        """
        with self._lock:
            try:
                return self._process_parallel_bidding_round()
            except Exception as e:
                print(f"[RobotController] Error in manual round processing: {e}")
                return None
    
    def _control_loop(self) -> None:
        """
        Main control loop for continuous task assignment.
        
        This loop runs continuously while the controller is active,
        polling for tasks and coordinating parallel robot bidding.
        """
        print(f"[RobotController] Control loop started, polling every {self.polling_interval}s")
        
        while self._running:
            try:
                # Process parallel bidding round
                round_data = self._process_parallel_bidding_round()
                
                if round_data:
                    # Support both internal and external round structures
                    bids_count = len(getattr(round_data, 'all_bids', []) or getattr(round_data, 'submitted_bids', []) or [])
                    duration = getattr(round_data, 'round_duration', 0.0) or 0.0
                    print(f"[RobotController] Processed round {round_data.round_id} with "
                          f"{len(round_data.winning_assignments)} assignments from "
                          f"{bids_count} bids in {duration:.3f}s")
                
                # Wait for next polling interval
                time.sleep(self.polling_interval)
                
            except Exception as e:
                print(f"[RobotController] Error in control loop: {e}")
                time.sleep(self.polling_interval)  # Continue despite errors
        
        print("[RobotController] Control loop stopped")
    
    def _process_parallel_bidding_round(self) -> Optional[Any]:
        """
        Process a complete parallel bidding round.
        
        This method:
        1. Gets available tasks from the jobs queue
        2. Gets available robots from the robot pool
        3. Sends tasks to all robots simultaneously for parallel bid calculation
        4. Collects bids from robots asynchronously
        5. Selects winning bids and assigns tasks to robots
        6. Updates statistics
        
        Returns:
            Optional[ParallelBiddingResult]: Bidding round results, or None if no tasks/robots
        """
        # Get exactly ONE task from the jobs queue (queue semantics: FIFO, per-task bidding)
        available_tasks = self._get_next_task_for_bidding()
        if not available_tasks:
            return None
        
        # Get available robots
        available_robots = self._get_available_robots()
        if not available_robots:
            print("[RobotController] No available robots for bidding")
            return None
        
        # Limit tasks and robots for performance
        available_tasks = available_tasks[:self.max_tasks_per_round]
        available_robots = available_robots[:self.max_robots_per_round]
        
        print(f"[RobotController] Starting parallel bidding with {len(available_tasks)} tasks and {len(available_robots)} robots")
        
        # Process parallel bidding round
        try:
            # If a bidding system is provided, delegate to it for SOLID compliance
            if self.bidding_system is not None:
                bidding_round = self.bidding_system.process_bidding_round(available_tasks, available_robots)
                # Assign tasks per bidding results
                self._assign_tasks_to_robots(bidding_round.winning_assignments)
                # Update statistics
                self._update_statistics(bidding_round)
                return bidding_round
            else:
                # Fallback to internal parallel bidding
                round_data = self._collect_parallel_bids(available_tasks, available_robots)
                # Select winning bids
                winning_assignments = self._select_winning_bids(round_data.all_bids)
                round_data.winning_assignments = winning_assignments
                # Assign tasks to robots and update the queue accordingly
                self._assign_tasks_to_robots(winning_assignments)
                # Update statistics
                self._update_statistics(round_data)
                return round_data
        except Exception as e:
            print(f"[RobotController] Error processing parallel bidding round: {e}")
            return None

    def _get_next_task_for_bidding(self) -> List[Task]:
        """
        Retrieve the next task for bidding, preserving strict queue semantics.

        Returns:
            List[Task]: A single-element list containing the next task, or empty if none.
        """
        try:
            next_task = None
            # Prefer peek to avoid losing task if assignment fails
            if hasattr(self.jobs_queue, 'peek_next_task'):
                next_task = self.jobs_queue.peek_next_task()
            else:
                # Fallback: get all pending and take the first (if interface lacks peek)
                pending = self.jobs_queue.get_pending_tasks()
                next_task = pending[0] if pending else None
            if next_task is not None:
                return [next_task]
            return []
        except Exception as e:
            print(f"[RobotController] Error peeking next task from queue: {e}")
            return []
    
    def _collect_parallel_bids(self, tasks: List[Task], robots: List[Any]) -> ParallelBiddingResult:
        """
        Collect bids from all robots in parallel.
        
        Args:
            tasks: Available tasks for bidding
            robots: Available robots for bidding
            
        Returns:
            ParallelBiddingResult: Results of parallel bid collection
        """
        round_id = f"round_{int(time.time())}_{len(robots)}_robots"
        start_time = time.time()
        
        # Submit bid calculation tasks to thread pool
        future_to_robot = {}
        for robot in robots:
            future = self._thread_pool.submit(self._get_robot_bids, robot, tasks)
            future_to_robot[future] = robot
        
        # Collect results as they complete
        all_bids = []
        robot_bid_times = {}
        successful_robots = 0
        failed_robots = 0
        
        try:
            for future in as_completed(future_to_robot, timeout=self.round_timeout):
                robot = future_to_robot[future]
                robot_id = self._get_robot_id(robot)
                
                try:
                    robot_bids, bid_time = future.result(timeout=self.bidding_timeout)
                    all_bids.extend(robot_bids)
                    robot_bid_times[robot_id] = bid_time
                    successful_robots += 1
                    print(f"[RobotController] Robot {robot_id} submitted {len(robot_bids)} bids in {bid_time:.3f}s")
                    
                except Exception as e:
                    failed_robots += 1
                    robot_bid_times[robot_id] = -1.0  # Indicate failure
                    print(f"[RobotController] Robot {robot_id} failed to submit bids: {e}")
                    
        except TimeoutError:
            # Handle timeout for the entire round
            print(f"[RobotController] Bidding round timed out after {self.round_timeout}s")
            # Mark remaining robots as failed and cancel their futures
            for future in future_to_robot:
                if not future.done():
                    robot = future_to_robot[future]
                    robot_id = self._get_robot_id(robot)
                    if robot_id not in robot_bid_times:
                        failed_robots += 1
                        robot_bid_times[robot_id] = -1.0
                        print(f"[RobotController] Robot {robot_id} timed out")
                    # Cancel the future to prevent hanging
                    future.cancel()
        
        # CRITICAL FIX: Ensure all futures are properly handled to prevent hanging
        for future in future_to_robot:
            if not future.done():
                robot = future_to_robot[future]
                robot_id = self._get_robot_id(robot)
                if robot_id not in robot_bid_times:
                    failed_robots += 1
                    robot_bid_times[robot_id] = -1.0
                    print(f"[RobotController] Robot {robot_id} cancelled/failed")
                # Cancel the future to prevent hanging
                future.cancel()
        
        round_duration = time.time() - start_time
        
        return ParallelBiddingResult(
            round_id=round_id,
            available_tasks=tasks,
            available_robots=robots,
            all_bids=all_bids,
            winning_assignments=[],  # Will be filled by caller
            round_duration=round_duration,
            robot_bid_times=robot_bid_times,
            successful_robots=successful_robots,
            failed_robots=failed_robots,
            round_metadata={
                "parallel_workers": self.max_parallel_workers,
                "bidding_timeout": self.bidding_timeout,
                "successful_robots": successful_robots,
                "failed_robots": failed_robots
            }
        )
    
    def _get_robot_bids(self, robot: Any, tasks: List[Task]) -> Tuple[List[RobotBid], float]:
        """
        Get bids from a single robot for the given tasks.
        
        Args:
            robot: Robot to get bids from
            tasks: Tasks to bid on
            
        Returns:
            Tuple[List[RobotBid], float]: Bids and calculation time
        """
        start_time = time.time()
        
        try:
            # Use robot's bid calculator for parallel calculation
            bids = robot.calculate_bids(tasks)
            calculation_time = time.time() - start_time
            
            return bids, calculation_time
            
        except Exception as e:
            calculation_time = time.time() - start_time
            print(f"[RobotController] Error getting bids from robot {self._get_robot_id(robot)}: {e}")
            return [], calculation_time
    
    def _select_winning_bids(self, bids: List[RobotBid]) -> List[TaskAssignment]:
        """
        Select winning bids from all collected bids.
        
        This method implements a simple greedy algorithm:
        1. Group bids by task
        2. For each task, select the robot with the lowest bid value
        3. Ensure each robot gets at most one task per round
        
        Args:
            bids: All bids collected from robots
            
        Returns:
            List[TaskAssignment]: Winning assignments
        """
        if not bids:
            return []
        
        # Group bids by task
        task_bids: Dict[str, List[RobotBid]] = {}
        for bid in bids:
            task_id = bid.task.task_id
            if task_id not in task_bids:
                task_bids[task_id] = []
            task_bids[task_id].append(bid)
        
        # Select winning bids
        assignments = []
        assigned_robots = set()
        
        # Process tasks in order
        for task_id, task_bid_list in task_bids.items():
            if not task_bid_list:
                continue
            
            # Find the best bid for this task (lowest value)
            best_bid = min(task_bid_list, key=lambda b: b.bid_value)
            
            # Only assign if robot hasn't been assigned yet
            if best_bid.robot_id not in assigned_robots:
                assigned_robots.add(best_bid.robot_id)
                
                assignment = TaskAssignment(
                    robot_id=best_bid.robot_id,
                    task=best_bid.task,
                    winning_bid_value=best_bid.bid_value,
                    assignment_metadata={
                        "total_bids_for_task": len(task_bid_list),
                        "winning_bid_index": task_bid_list.index(best_bid),
                        "parallel_bidding": True
                    }
                )
                assignments.append(assignment)
        
        print(f"[RobotController] Selected {len(assignments)} winning assignments from {len(bids)} bids")
        return assignments
    
    def _get_available_tasks(self) -> List[Task]:
        """
        Deprecated in favor of _get_next_task_for_bidding().
        Kept for backward compatibility; returns a single next task when possible.
        """
        return self._get_next_task_for_bidding()
    
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
            # Use robot's own availability check
            if hasattr(robot, 'is_available_for_bidding'):
                return robot.is_available_for_bidding()
            
            # Fallback to status-based check
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
                    # Remove task from the queue now that it has been successfully assigned
                    try:
                        if hasattr(self.jobs_queue, 'remove_task'):
                            self.jobs_queue.remove_task(assignment.task.task_id)
                    except Exception as e:
                        print(f"[RobotController] Warning: failed to remove task {assignment.task.task_id} from queue: {e}")
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
                if self._get_robot_id(robot) == robot_id:
                    return robot
            except Exception:
                continue
        
        return None
    
    def _update_statistics(self, round_data: Any) -> None:
        """
        Update controller statistics with bidding round data.
        
        Args:
            round_data: Bidding round data
        """
        self._total_rounds_processed += 1
        # Count assignments regardless of structure
        assignments = getattr(round_data, 'winning_assignments', []) or []
        self._total_tasks_assigned += len(assignments)
        # Count bids depending on structure
        if hasattr(round_data, 'all_bids') and getattr(round_data, 'all_bids') is not None:
            self._total_bids_collected += len(round_data.all_bids)
        elif hasattr(round_data, 'submitted_bids') and getattr(round_data, 'submitted_bids') is not None:
            self._total_bids_collected += len(round_data.submitted_bids)
        self._last_round_timestamp = datetime.now()
        
        # Update average bid time when available (internal-only metric)
        robot_bid_times = getattr(round_data, 'robot_bid_times', None)
        if isinstance(robot_bid_times, dict):
            successful_times = [t for t in robot_bid_times.values() if t > 0]
            if successful_times:
                if self._average_bid_time == 0:
                    self._average_bid_time = sum(successful_times) / len(successful_times)
                else:
                    self._average_bid_time = 0.9 * self._average_bid_time + 0.1 * (sum(successful_times) / len(successful_times))

    def _get_robot_id(self, robot: Any) -> str:
        """
        Get robot ID safely, handling both mock and real robot implementations.
        
        Args:
            robot: Robot agent (mock or real)
            
        Returns:
            str: Robot identifier
        """
        try:
            # Try to get robot ID from config attribute (mock robots)
            if hasattr(robot, 'config') and hasattr(robot.config, 'robot_id'):
                return robot.config.robot_id
            # Try to get robot ID directly (real robots)
            elif hasattr(robot, 'robot_id'):
                return robot.robot_id
            # Fallback to string representation
            else:
                return str(robot)
        except Exception:
            return str(robot) 