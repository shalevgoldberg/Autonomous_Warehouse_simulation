"""
Integration tests for Robot Controller parallel bidding functionality.

This test suite verifies that the robot controller correctly implements
parallel asynchronous bidding with proper task assignment and robot coordination.
"""
import pytest
import time
import threading
from unittest.mock import Mock, MagicMock, patch
from typing import List, Dict, Any

from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.bidding_system_interface import IBiddingSystem
from interfaces.task_handler_interface import Task, TaskType
from interfaces.configuration_interface import IConfigurationProvider, SystemConfig, BidConfig
from warehouse.impl.robot_controller_impl import RobotController, ParallelBiddingResult
from robot.robot_agent import RobotAgent, RobotConfiguration
from warehouse.map import WarehouseMap
# from simulation.mujoco_env import SimpleMuJoCoPhysics  # Moved below


class MockJobsQueue:
    """Mock jobs queue for testing."""
    
    def __init__(self, tasks: List[Task] = None):
        self.tasks = tasks or []
        self._lock = threading.Lock()
    
    def get_pending_tasks(self) -> List[Task]:
        """Get pending tasks."""
        with self._lock:
            return self.tasks.copy()
    
    def add_task(self, task: Task) -> None:
        """Add a task to the queue."""
        with self._lock:
            self.tasks.append(task)
    
    def remove_task(self, task_id: str) -> None:
        """Remove a task from the queue."""
        with self._lock:
            self.tasks = [t for t in self.tasks if t.task_id != task_id]


class MockRobotAgent:
    """Mock robot agent for testing parallel bidding."""
    
    def __init__(self, robot_id: str, position: tuple = (0.0, 0.0), battery: float = 100.0):
        self.config = Mock()
        self.config.robot_id = robot_id
        self._position = position
        self._battery = battery
        self._task_active = False
        self._operational_status = 'idle'
        self._lock = threading.Lock()
    
    def calculate_bids(self, tasks: List[Task]) -> List[Any]:
        """Calculate bids for tasks."""
        import time
        from interfaces.bidding_system_interface import RobotBid
        
        # Check availability without lock to avoid deadlock
            if not self.is_available_for_bidding():
                return []
            
            # Simulate some calculation time
            time.sleep(0.01)  # 10ms delay to simulate real calculation
            
            bids = []
            for task in tasks:
                # Simple bid calculation based on distance
                if task.target_position:
                    target_x, target_y = task.target_position[0], task.target_position[1]
                    distance = ((target_x - self._position[0])**2 + (target_y - self._position[1])**2)**0.5
                    bid_value = distance * 10.0 + (100.0 - self._battery) * 0.5
                else:
                    bid_value = 50.0 + (100.0 - self._battery) * 0.5
                
                bid = RobotBid(
                    robot_id=self.config.robot_id,
                    task=task,
                    bid_value=bid_value,
                    bid_metadata={
                        "robot_position": self._position,
                        "robot_battery": self._battery,
                        "parallel_bidding": True
                    }
                )
                bids.append(bid)
            
            return bids
    
    def is_available_for_bidding(self) -> bool:
        """Check if robot is available for bidding."""
        with self._lock:
            return (not self._task_active and 
                   self._operational_status == 'idle' and 
                   self._battery > 20.0)
    
    def get_bid_calculation_statistics(self) -> Dict[str, Any]:
        """Get bid calculation statistics."""
        # Remove lock to prevent deadlock with is_available_for_bidding
            return {
                'robot_id': self.config.robot_id,
                'available_for_bidding': self.is_available_for_bidding(),
                'current_position': self._position,
                'battery_level': self._battery,
                'operational_status': self._operational_status
            }
    
    def get_status(self) -> Dict[str, Any]:
        """Get robot status."""
        with self._lock:
            return {
                'robot_id': self.config.robot_id,
                'position': self._position,
                'battery': self._battery,
                'task_active': self._task_active,
                'operational_status': self._operational_status
            }
    
    def assign_task(self, task: Task) -> bool:
        """Assign a task to the robot."""
        with self._lock:
            if self._task_active:
                return False
            
            self._task_active = True
            return True
    
    def set_position(self, position: tuple) -> None:
        """Set robot position."""
        with self._lock:
            self._position = position
    
    def set_battery(self, battery: float) -> None:
        """Set robot battery level."""
        with self._lock:
            self._battery = battery
    
    def set_operational_status(self, status: str) -> None:
        """Set operational status."""
        with self._lock:
            self._operational_status = status


class MockConfigurationProvider:
    """Mock configuration provider for testing."""
    
    def __init__(self):
        self.system_config = SystemConfig(
            log_level="INFO",
            log_file=None,
            log_format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            cache_ttl=300.0,
            connection_pool_size=10,
            thread_pool_size=4,
            health_check_interval=1.0,
            metrics_collection_interval=60.0,
            alert_thresholds={"battery_low": 0.2, "task_timeout": 300.0, "error_rate": 0.1}
        )
        self.bid_config = BidConfig(
            max_parallel_workers=4,
            distance_weight=0.4,
            battery_weight=0.3,
            workload_weight=0.2,
            task_type_compatibility_weight=0.1,
            robot_capabilities_weight=0.0,
            time_urgency_weight=0.0,
            conflict_box_availability_weight=0.0,
            shelf_accessibility_weight=0.0,
            enable_distance_factor=True,
            enable_battery_factor=True,
            enable_workload_factor=True,
            enable_task_type_compatibility_factor=True,
            enable_robot_capabilities_factor=False,
            enable_time_urgency_factor=False,
            enable_conflict_box_availability_factor=False,
            enable_shelf_accessibility_factor=False,
            battery_threshold=0.2,
            calculation_timeout=1.0,  # Reduced from 5.0 to 1.0 for faster tests
            max_distance_normalization=20.0,
            enable_parallel_calculation=True,
            enable_calculation_statistics=True,
            enable_factor_breakdown=True
        )
    
    def get_system_config(self) -> SystemConfig:
        return self.system_config
    
    def get_bid_config(self) -> BidConfig:
        return self.bid_config


class TestRobotControllerParallelBidding:
    """Test robot controller parallel bidding functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        from simulation.mujoco_env import SimpleMuJoCoPhysics  # Import moved here
        # Create mock components
        self.jobs_queue = MockJobsQueue()
        self.config_provider = MockConfigurationProvider()
        
        # Create test tasks
        self.tasks = [
            Task(task_id="task_1", task_type=TaskType.MOVE_TO_POSITION, target_position=(5.0, 5.0), priority=1),
            Task(task_id="task_2", task_type=TaskType.MOVE_TO_POSITION, target_position=(10.0, 10.0), priority=1),
            Task(task_id="task_3", task_type=TaskType.PICK_AND_DELIVER, shelf_id="shelf_1", item_id="item_1", order_id="order_1", priority=2)
        ]
        
        # Create mock robots
        self.robots = [
            MockRobotAgent("robot_1", position=(0.0, 0.0), battery=100.0),
            MockRobotAgent("robot_2", position=(2.0, 2.0), battery=80.0),
            MockRobotAgent("robot_3", position=(8.0, 8.0), battery=60.0)
        ]
        
        # Add tasks to queue
        for task in self.tasks:
            self.jobs_queue.add_task(task)
        
        # Create robot controller
        self.controller = RobotController(
            jobs_queue=self.jobs_queue,
            robot_pool=self.robots,
            config_provider=self.config_provider
        )
    
    def test_controller_initialization(self):
        """Test robot controller initialization with parallel bidding."""
        assert self.controller.max_parallel_workers == 4
        assert self.controller.bidding_timeout == 1.0  # Should match calculation_timeout from config
        assert self.controller.max_robots_per_round == 20
        assert self.controller.max_tasks_per_round == 50
        assert len(self.controller.robot_pool) == 3
    
    def test_parallel_bid_collection(self):
        """Test parallel bid collection from multiple robots."""
        # Process a single bidding round
        round_result = self.controller.process_single_round()
        
        assert round_result is not None
        assert round_result.round_id is not None
        assert len(round_result.available_tasks) == 3
        assert len(round_result.available_robots) == 3
        assert len(round_result.all_bids) > 0
        assert round_result.successful_robots == 3
        assert round_result.failed_robots == 0
        assert round_result.round_duration > 0
        
        # Check that all robots submitted bids
        robot_ids = {bid.robot_id for bid in round_result.all_bids}
        assert robot_ids == {"robot_1", "robot_2", "robot_3"}
        
        # Check bid times are recorded
        assert len(round_result.robot_bid_times) == 3
        for robot_id in ["robot_1", "robot_2", "robot_3"]:
            assert robot_id in round_result.robot_bid_times
            assert round_result.robot_bid_times[robot_id] > 0
    
    def test_winning_bid_selection(self):
        """Test selection of winning bids from parallel collection."""
        round_result = self.controller.process_single_round()
        
        assert round_result is not None
        assert len(round_result.winning_assignments) > 0
        
        # Check that each assignment has valid data
        for assignment in round_result.winning_assignments:
            assert assignment.robot_id in ["robot_1", "robot_2", "robot_3"]
            assert assignment.task.task_id in ["task_1", "task_2", "task_3"]
            assert assignment.winning_bid_value > 0
            assert "parallel_bidding" in assignment.assignment_metadata
    
    def test_robot_availability_checking(self):
        """Test robot availability checking for bidding."""
        # Make one robot unavailable
        self.robots[0].set_operational_status('error')
        
        round_result = self.controller.process_single_round()
        
        assert round_result is not None
        assert round_result.successful_robots == 2
        assert round_result.failed_robots == 0  # Robot is filtered out, not failed
        
        # Check that only available robots submitted bids
        robot_ids = {bid.robot_id for bid in round_result.all_bids}
        assert robot_ids == {"robot_2", "robot_3"}
    
    def test_low_battery_robot_filtering(self):
        """Test filtering of robots with low battery."""
        # Set one robot to low battery
        self.robots[1].set_battery(15.0)  # Below 20% threshold
        
        round_result = self.controller.process_single_round()
        
        assert round_result is not None
        assert round_result.successful_robots == 2
        
        # Check that low battery robot didn't submit bids
        robot_ids = {bid.robot_id for bid in round_result.all_bids}
        assert "robot_2" not in robot_ids
        assert robot_ids == {"robot_1", "robot_3"}
    
    def test_busy_robot_filtering(self):
        """Test filtering of robots with active tasks."""
        # Make one robot busy
        self.robots[2]._task_active = True
        
        round_result = self.controller.process_single_round()
        
        assert round_result is not None
        assert round_result.successful_robots == 2
        
        # Check that busy robot didn't submit bids
        robot_ids = {bid.robot_id for bid in round_result.all_bids}
        assert "robot_3" not in robot_ids
        assert robot_ids == {"robot_1", "robot_2"}
    
    def test_empty_task_queue(self):
        """Test behavior when no tasks are available."""
        # Clear task queue
        self.jobs_queue.tasks.clear()
        
        round_result = self.controller.process_single_round()
        
        assert round_result is None
    
    def test_no_available_robots(self):
        """Test behavior when no robots are available."""
        # Make all robots unavailable
        for robot in self.robots:
            robot.set_operational_status('error')
        
        round_result = self.controller.process_single_round()
        
        assert round_result is None
    
    def test_controller_statistics(self):
        """Test controller statistics tracking."""
        # Add more tasks for multiple rounds
        additional_tasks = [
            Task(task_id="task_4", task_type=TaskType.MOVE_TO_POSITION, target_position=(15.0, 15.0), priority=1),
            Task(task_id="task_5", task_type=TaskType.MOVE_TO_POSITION, target_position=(20.0, 20.0), priority=1),
            Task(task_id="task_6", task_type=TaskType.PICK_AND_DELIVER, shelf_id="shelf_2", item_id="item_2", order_id="order_2", priority=2),
        ]
        
        for task in additional_tasks:
            self.jobs_queue.add_task(task)
        
        # Process a few rounds
        for round_num in range(3):
            print(f"[TEST] Processing round {round_num + 1}")
            self.controller.process_single_round()
            
            # Reset robot task status for next round (simulate task completion)
            for robot in self.robots:
                robot._task_active = False
        
        status = self.controller.get_controller_status()
        
        assert status['total_rounds_processed'] == 3
        assert status['total_tasks_assigned'] > 0
        assert status['total_bids_collected'] > 0
        assert status['average_bid_time'] > 0
        assert status['robot_count'] == 3
        assert status['max_parallel_workers'] == 4
        assert status['bidding_timeout'] == 1.0  # Should match calculation_timeout from config
    
    def test_robot_status_summary(self):
        """Test robot status summary generation."""
        status_summary = self.controller.get_robot_status_summary()
        
        assert len(status_summary) == 3
        
        for robot_status in status_summary:
            assert 'robot_id' in robot_status
            assert 'status' in robot_status
            assert 'available_for_bidding' in robot_status
            assert 'bid_calculation_stats' in robot_status
            
            # Check bid calculation stats
            bid_stats = robot_status['bid_calculation_stats']
            assert 'robot_id' in bid_stats
            assert 'available_for_bidding' in bid_stats
            assert 'current_position' in bid_stats
            assert 'battery_level' in bid_stats
            assert 'operational_status' in bid_stats
    
    def test_configuration_update(self):
        """Test configuration update from provider."""
        # Update configuration
        self.config_provider.bid_config.max_parallel_workers = 6
        self.config_provider.bid_config.calculation_timeout = 2.0
        
        self.controller.update_config_from_provider()
        
        assert self.controller.max_parallel_workers == 6
        assert self.controller.bidding_timeout == 2.0
    
    def test_parallel_bidding_performance(self):
        """Test parallel bidding performance with multiple robots."""
        # Add more robots to test parallel performance
        additional_robots = [
            MockRobotAgent(f"robot_{i}", position=(i*2.0, i*2.0), battery=90.0)
            for i in range(4, 8)
        ]
        
        for robot in additional_robots:
            self.controller.add_robot(robot)
        
        # Add more tasks
        additional_tasks = [
            Task(task_id=f"task_{i}", task_type=TaskType.MOVE_TO_POSITION, 
                 target_position=(i*3.0, i*3.0), priority=1)
            for i in range(4, 8)
        ]
        
        for task in additional_tasks:
            self.jobs_queue.add_task(task)
        
        # Process bidding round
        start_time = time.time()
        round_result = self.controller.process_single_round()
        end_time = time.time()
        
        assert round_result is not None
        assert round_result.successful_robots == 7  # All robots available
        assert len(round_result.all_bids) > 0
        
        # Check that parallel processing is working (should be faster than sequential)
        processing_time = end_time - start_time
        assert processing_time < 2.0  # Should complete within timeout
    
    def test_robot_pool_management(self):
        """Test adding and removing robots from the pool."""
        # Add a new robot
        new_robot = MockRobotAgent("robot_new", position=(15.0, 15.0), battery=95.0)
        self.controller.add_robot(new_robot)
        
        assert len(self.controller.robot_pool) == 4
        
        # Process a round to verify new robot participates
        round_result = self.controller.process_single_round()
        assert round_result is not None
        
        robot_ids = {bid.robot_id for bid in round_result.all_bids}
        assert "robot_new" in robot_ids
        
        # Remove a robot
        self.controller.remove_robot(self.robots[0])
        assert len(self.controller.robot_pool) == 3
        
        # Process another round
        round_result = self.controller.process_single_round()
        assert round_result is not None
        
        robot_ids = {bid.robot_id for bid in round_result.all_bids}
        assert "robot_1" not in robot_ids


if __name__ == "__main__":
    pytest.main([__file__]) 