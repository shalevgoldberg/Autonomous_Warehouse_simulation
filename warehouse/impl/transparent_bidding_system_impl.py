"""
Transparent Bidding System Implementation.

This implementation provides a simple, transparent bidding mechanism where:
- Each available robot bids on all tasks they can execute
- The system automatically assigns tasks to available robots
- No complex bidding algorithms - just direct assignment
- Perfect for single-robot scenarios or simple multi-robot coordination

Design Principles:
- **Single Responsibility**: Only handles transparent bidding logic
- **Open/Closed**: Extensible for future bidding strategies
- **Liskov Substitution**: Implements IBiddingSystem contract
- **Interface Segregation**: Clean interface implementation
- **Dependency Inversion**: Depends on abstractions (RobotAgent, Task)

Thread Safety: All methods are thread-safe for concurrent access.
"""
import threading
import time
import uuid
from typing import List, Optional, Dict, Any, TYPE_CHECKING
from datetime import datetime

from interfaces.bidding_system_interface import (
    IBiddingSystem, RobotBid, TaskAssignment, BiddingRound, 
    BiddingStats, BiddingStrategy, BidStatus, BiddingSystemError
)
from interfaces.task_handler_interface import Task

if TYPE_CHECKING:
    from robot.robot_agent import RobotAgent

class TransparentBiddingSystem(IBiddingSystem):
    """
    Transparent bidding system implementation.
    
    This system implements a simple bidding strategy where:
    1. All available robots bid on all tasks they can execute
    2. Each robot calculates a simple cost (distance-based)
    3. Tasks are assigned to the first available robot
    4. No complex optimization - just direct assignment
    
    Perfect for:
    - Single robot scenarios
    - Simple multi-robot coordination
    - Testing and development
    - Scenarios where simplicity is preferred over optimization
    """
    
    def __init__(self):
        """Initialize the transparent bidding system."""
        self._strategy = BiddingStrategy.TRANSPARENT
        self._stats = BiddingStats(
            total_rounds=0,
            total_bids_submitted=0,
            total_assignments_made=0,
            average_round_duration=0.0,
            average_bids_per_round=0.0,
            assignment_success_rate=0.0
        )
        self._recent_rounds: List[BiddingRound] = []
        self._max_round_history = 50
        self._lock = threading.RLock()  # Reentrant lock for thread safety
        
        print("[TransparentBiddingSystem] Initialized with transparent strategy")
    
    def collect_bids(self, available_tasks: List[Task], 
                    available_robots: List['RobotAgent']) -> List[RobotBid]:
        """
        Collect bids from available robots for the given tasks.
        
        In transparent bidding, each available robot bids on all tasks
        they can execute. The bid value is calculated based on simple
        distance and availability metrics.
        
        Args:
            available_tasks: List of tasks available for bidding
            available_robots: List of RobotAgent instances that can participate
            
        Returns:
            List[RobotBid]: All bids submitted by robots
            
        Raises:
            BiddingSystemError: If bid collection fails critically
        """
        with self._lock:
            try:
                bids = []
                
                for robot in available_robots:
                    try:
                        robot_id = robot.config.robot_id
                    except Exception as e:
                        raise BiddingSystemError(f"Failed to get robot_id: {e}") from e
                    # Check if robot is available for bidding
                    if not self.is_robot_available_for_task(robot, None):
                        continue
                    
                    for task in available_tasks:
                        # Check if robot can execute this specific task
                        if self.is_robot_available_for_task(robot, task):
                            try:
                                bid_value = self.calculate_bid_value(robot, task)
                                bid = RobotBid(
                                    robot_id=robot_id,
                                    task=task,
                                    bid_value=bid_value,
                                    bid_metadata={
                                        "strategy": "transparent",
                                        "robot_position": robot.get_status().get('position', (0, 0)),
                                        "robot_battery": robot.get_status().get('battery', 0.0)
                                    }
                                )
                                bids.append(bid)
                            except Exception as e:
                                print(f"[TransparentBiddingSystem] Failed to calculate bid for robot {robot_id} on task {task.task_id}: {e}")
                                continue
                
                print(f"[TransparentBiddingSystem] Collected {len(bids)} bids from {len(available_robots)} robots for {len(available_tasks)} tasks")
                return bids
                
            except Exception as e:
                raise BiddingSystemError(f"Failed to collect bids: {e}") from e
    
    def select_winning_bids(self, bids: List[RobotBid]) -> List[TaskAssignment]:
        """
        Select winning bids and create task assignments.
        
        Hybrid approach:
        1. Group bids by task
        2. For each task, prefer robots not yet assigned (one-to-one preference)
        3. If no unassigned robots available, allow already-assigned robots
        4. Create assignments for all selected bids
        
        Args:
            bids: List of bids to evaluate
            
        Returns:
            List[TaskAssignment]: Winning assignments (robot-task pairs)
            
        Raises:
            BiddingSystemError: If bid selection fails
        """
        with self._lock:
            try:
                assignments = []
                assigned_robots = set()
                
                # Group bids by task
                task_bids: Dict[str, List[RobotBid]] = {}
                for bid in bids:
                    task_id = bid.task.task_id
                    if task_id not in task_bids:
                        task_bids[task_id] = []
                    task_bids[task_id].append(bid)
                
                # Process tasks in order, preferring unassigned robots
                for task_id, task_bid_list in task_bids.items():
                    if not task_bid_list:
                        continue
                    
                    # First, try to find a robot that hasn't been assigned yet
                    available_bids = [b for b in task_bid_list if b.robot_id not in assigned_robots]
                    
                    # If no unassigned robots available, use any robot (fallback)
                    if not available_bids:
                        available_bids = task_bid_list
                    
                    # Find the bid with the lowest value
                    winning_bid = min(available_bids, key=lambda b: b.bid_value)
                    winning_bid.status = BidStatus.ACCEPTED
                    
                    # Only track as assigned if this is the robot's first assignment
                    if winning_bid.robot_id not in assigned_robots:
                        assigned_robots.add(winning_bid.robot_id)
                    
                    # Create assignment
                    assignment = TaskAssignment(
                        robot_id=winning_bid.robot_id,
                        task=winning_bid.task,
                        winning_bid_value=winning_bid.bid_value,
                        assignment_metadata={
                            "strategy": "transparent",
                            "total_bids_for_task": len(task_bid_list),
                            "winning_bid_index": task_bid_list.index(winning_bid),
                            "robot_already_assigned": winning_bid.robot_id in assigned_robots
                        }
                    )
                    assignments.append(assignment)
                
                print(f"[TransparentBiddingSystem] Selected {len(assignments)} winning assignments from {len(bids)} bids")
                return assignments
                
            except Exception as e:
                raise BiddingSystemError(f"Failed to select winning bids: {e}") from e
    
    def process_bidding_round(self, available_tasks: List[Task], 
                            available_robots: List['RobotAgent']) -> BiddingRound:
        """
        Process a complete bidding round from collection to assignment.
        
        This method combines collect_bids() and select_winning_bids()
        into a single operation, tracking timing and updating statistics.
        
        Args:
            available_tasks: List of tasks available for bidding
            available_robots: List of RobotAgent instances that can participate
            
        Returns:
            BiddingRound: Complete round information including bids and assignments
            
        Raises:
            BiddingSystemError: If bidding round processing fails
        """
        with self._lock:
            try:
                round_id = f"round_{uuid.uuid4().hex[:8]}"
                start_time = time.time()
                
                # Collect bids
                bids = self.collect_bids(available_tasks, available_robots)
                
                # Select winning bids
                assignments = self.select_winning_bids(bids)
                
                # Calculate round duration
                round_duration = time.time() - start_time
                
                # Create bidding round
                round_data = BiddingRound(
                    round_id=round_id,
                    available_tasks=available_tasks,
                    submitted_bids=bids,
                    winning_assignments=assignments,
                    round_duration=round_duration,
                    round_metadata={
                        "strategy": self._strategy.value,
                        "robot_count": len(available_robots),
                        "task_count": len(available_tasks)
                    }
                )
                
                # Update statistics
                self._update_statistics(round_data)
                
                # Store in recent rounds
                self._recent_rounds.append(round_data)
                if len(self._recent_rounds) > self._max_round_history:
                    self._recent_rounds.pop(0)
                
                print(f"[TransparentBiddingSystem] Completed bidding round {round_id} in {round_duration:.3f}s")
                return round_data
                
            except Exception as e:
                raise BiddingSystemError(f"Failed to process bidding round: {e}") from e
    
    def set_bidding_strategy(self, strategy: BiddingStrategy) -> None:
        """
        Set the bidding strategy to use for task assignment.
        
        For transparent bidding system, only TRANSPARENT strategy is supported.
        
        Args:
            strategy: Bidding strategy to use
            
        Raises:
            BiddingSystemError: If strategy is not supported
        """
        with self._lock:
            if strategy != BiddingStrategy.TRANSPARENT:
                raise BiddingSystemError(
                    f"TransparentBiddingSystem only supports TRANSPARENT strategy, got {strategy.value}"
                )
            self._strategy = strategy
            print(f"[TransparentBiddingSystem] Strategy set to {strategy.value}")
    
    def get_current_strategy(self) -> BiddingStrategy:
        """
        Get the currently active bidding strategy.
        
        Returns:
            BiddingStrategy: Current strategy being used
        """
        with self._lock:
            return self._strategy
    
    def get_bidding_stats(self) -> BiddingStats:
        """
        Get comprehensive bidding system statistics.
        
        Returns:
            BiddingStats: Performance statistics and metrics
        """
        with self._lock:
            return self._stats
    
    def get_recent_bidding_rounds(self, limit: int = 10) -> List[BiddingRound]:
        """
        Get recent bidding rounds for monitoring and analysis.
        
        Args:
            limit: Maximum number of rounds to return
            
        Returns:
            List[BiddingRound]: Recent bidding rounds
        """
        with self._lock:
            return self._recent_rounds[-limit:] if self._recent_rounds else []
    
    def reset_statistics(self) -> None:
        """
        Reset all bidding statistics to zero.
        
        Useful for testing or when starting a new operational period.
        """
        with self._lock:
            self._stats = BiddingStats(
                total_rounds=0,
                total_bids_submitted=0,
                total_assignments_made=0,
                average_round_duration=0.0,
                average_bids_per_round=0.0,
                assignment_success_rate=0.0
            )
            self._recent_rounds.clear()
            print("[TransparentBiddingSystem] Statistics reset")
    
    def is_robot_available_for_task(self, robot: 'RobotAgent', task: Optional[Task]) -> bool:
        """
        Check if a robot is available and suitable for a specific task.
        
        For transparent bidding, a robot is available if:
        1. It's idle (no active task)
        2. It has sufficient battery
        3. It's not in error state
        
        Args:
            robot: RobotAgent instance to check
            task: Task to check compatibility with (optional)
            
        Returns:
            bool: True if robot can bid on the task
        """
        try:
            # Get robot status
            status = robot.get_status()
            
            # Check if robot is idle
            if status.get('task_active', True):  # Default to True for safety
                return False
            
            # Check battery level (simple threshold)
            battery_level = status.get('battery', 0.0)
            if battery_level < 20.0:  # 20% battery threshold
                return False
            
            # Check operational status
            operational_status = status.get('operational_status', 'error')
            if operational_status in ['error', 'emergency_stop', 'stalled']:
                return False
            
            # For specific task, check if robot can execute it
            if task is not None:
                # Simple check: robot can execute any task if it's available
                # In a real implementation, you might check task-specific requirements
                pass
            
            return True
            
        except Exception as e:
            print(f"[TransparentBiddingSystem] Error checking robot availability: {e}")
            return False
    
    def calculate_bid_value(self, robot: 'RobotAgent', task: Task) -> float:
        """
        Calculate the bid value for a robot-task combination.
        
        For transparent bidding, the bid value is based on:
        1. Distance from robot to task location
        2. Robot battery level (lower battery = higher cost)
        3. Robot-specific variation to ensure fair distribution
        
        Args:
            robot: RobotAgent instance to evaluate
            task: Task to evaluate
            
        Returns:
            float: Bid value (lower is better)
            
        Raises:
            BiddingSystemError: If bid calculation fails
        """
        try:
            # Get robot status
            status = robot.get_status()
            robot_position = status.get('position', (0.0, 0.0))
            battery_level = status.get('battery', 100.0)
            
            # Get robot ID for variation
            robot_id = robot.config.robot_id
            
            # Calculate distance to task (simplified)
            # For PICK_AND_DELIVER tasks, we'd need shelf coordinates
            # For now, use a simple heuristic based on task type
            distance_cost = 0.0
            if task.task_type.value == "pick_and_deliver":
                # Simple distance calculation (in real implementation, get shelf coordinates)
                distance_cost = 10.0  # Base distance cost
            elif task.task_type.value == "move_to_position":
                if task.target_position:
                    # Calculate actual distance
                    target_x, target_y = task.target_position[0], task.target_position[1]
                    robot_x, robot_y = robot_position
                    distance_cost = ((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2) ** 0.5
                else:
                    distance_cost = 5.0  # Default distance cost
            else:
                distance_cost = 5.0  # Default for other task types
            
            # Battery factor (lower battery = higher cost)
            battery_factor = max(1.0, (100.0 - battery_level) / 20.0)  # 1.0 to 5.0 range
            
            # Robot-specific variation to ensure fair distribution
            # Use hash of robot ID to create consistent but different values per robot
            robot_variation = (hash(robot_id) % 100) / 1000.0  # Small variation (0.0 to 0.099)
            
            # Calculate final bid value
            bid_value = distance_cost * battery_factor + robot_variation
            
            return max(0.1, bid_value)  # Ensure positive bid value
            
        except Exception as e:
            raise BiddingSystemError(f"Failed to calculate bid value: {e}") from e
    
    def _update_statistics(self, round_data: BiddingRound) -> None:
        """
        Update bidding statistics with new round data.
        
        Args:
            round_data: Bidding round data to incorporate
        """
        # Update basic counts
        self._stats.total_rounds += 1
        self._stats.total_bids_submitted += len(round_data.submitted_bids)
        self._stats.total_assignments_made += len(round_data.winning_assignments)
        
        # Update averages
        if self._stats.total_rounds > 0:
            round_duration = round_data.round_duration if round_data.round_duration is not None else 0.0
            self._stats.average_round_duration = (
                (self._stats.average_round_duration * (self._stats.total_rounds - 1) + 
                 round_duration) / self._stats.total_rounds
            )
            self._stats.average_bids_per_round = (
                self._stats.total_bids_submitted / self._stats.total_rounds
            )
        
        # Update success rate
        if len(round_data.available_tasks) > 0:
            success_rate = len(round_data.winning_assignments) / len(round_data.available_tasks)
            self._stats.assignment_success_rate = (
                (self._stats.assignment_success_rate * (self._stats.total_rounds - 1) + 
                 success_rate) / self._stats.total_rounds
            )
        
        # Update last round timestamp
        self._stats.last_round_timestamp = round_data.round_timestamp 