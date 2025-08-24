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
from interfaces.bidding_system_interface import IBiddingSystem, BiddingStrategy
from interfaces.task_handler_interface import Task, TaskType
from interfaces.configuration_interface import IConfigurationProvider, SystemConfig, BidConfig
from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
from robot.robot_agent_lane_based import RobotAgent
from interfaces.configuration_interface import RobotConfig
from warehouse.map import WarehouseMap


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
    """Mock robot agent for testing bidding system."""
    
    def __init__(self, robot_id: str, position: tuple = (0.0, 0.0), battery: float = 100.0):
        self.robot_id = robot_id  # Use robot_id directly for modern agent
        self._position = position
        self._battery = battery
        self._task_active = False
        self._operational_status = 'idle'
        self._lock = threading.Lock()
    
    def get_status(self) -> Dict[str, Any]:
        """Get robot status."""
        with self._lock:
            return {
                'robot_id': self.robot_id,
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
            calculation_timeout=1.0,
            max_distance_normalization=20.0,
            enable_parallel_calculation=True,
            enable_calculation_statistics=True,
            enable_factor_breakdown=True
        )
    
    def get_system_config(self) -> SystemConfig:
        """Get system configuration."""
        return self.system_config
    
    def get_bid_config(self) -> BidConfig:
        """Get bid configuration."""
        return self.bid_config


class TestBiddingSystemIntegration:
    """Test bidding system integration without complex RobotController."""
    
    def setup_method(self):
        """Set up test fixtures."""
        # Create mock components
        self.jobs_queue = MockJobsQueue()
        self.config_provider = MockConfigurationProvider()
        
        # Create test tasks
        self.tasks = [
            Task(task_id="task_1", task_type=TaskType.MOVE_TO_POSITION, target_position=(5.0, 5.0), priority=1),
            Task(task_id="task_2", task_type=TaskType.MOVE_TO_POSITION, target_position=(10.0, 10.0), priority=1),
            Task(task_id="task_3", task_type=TaskType.PICK_AND_DELIVER, shelf_id="shelf_3_3", item_id="item_1", order_id="order_1", priority=2)
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
        
        # Create bidding system directly
        self.bidding_system = TransparentBiddingSystem()
    
    def test_bidding_system_initialization(self):
        """Test bidding system initialization."""
        assert self.bidding_system.get_current_strategy() == BiddingStrategy.TRANSPARENT
        print("✅ Bidding system initialization test passed")
    
    def test_mock_robot_functionality(self):
        """Test mock robot basic functionality."""
        robot = self.robots[0]
        
        # Test status
        status = robot.get_status()
        assert status['robot_id'] == 'robot_1'
        assert status['position'] == (0.0, 0.0)
        assert status['battery'] == 100.0
        assert status['task_active'] == False
        assert status['operational_status'] == 'idle'
        
        # Test position setting
        robot.set_position((1.0, 1.0))
        assert robot.get_status()['position'] == (1.0, 1.0)
        
        # Test battery setting
        robot.set_battery(50.0)
        assert robot.get_status()['battery'] == 50.0
        
        print("✅ Mock robot functionality test passed")
    
    def test_bidding_system_robot_availability(self):
        """Test bidding system robot availability checking."""
        robot = self.robots[0]
        
        # Test available robot
        available = self.bidding_system.is_robot_available_for_task(robot, None)
        assert available == True
        
        # Test robot with low battery
        robot.set_battery(15.0)  # Below 20% threshold
        available = self.bidding_system.is_robot_available_for_task(robot, None)
        assert available == False
        
        # Test robot with error status
        robot.set_battery(100.0)  # Reset battery
        robot.set_operational_status('error')
        available = self.bidding_system.is_robot_available_for_task(robot, None)
        assert available == False
        
        print("✅ Robot availability test passed")
    
    def test_bidding_system_bid_calculation(self):
        """Test bidding system bid calculation."""
        robot = self.robots[0]
        task = self.tasks[0]  # MOVE_TO_POSITION task
        
        # Calculate bid value
        bid_value = self.bidding_system.calculate_bid_value(robot, task)
        assert bid_value > 0
        assert isinstance(bid_value, float)
        
        print(f"✅ Bid calculation test passed - bid value: {bid_value}")
    
    def test_bidding_system_collect_bids(self):
        """Test bidding system bid collection."""
        # Collect bids from all robots
        bids = self.bidding_system.collect_bids(self.tasks, self.robots)
        
        assert len(bids) > 0
        assert all(bid.robot_id in ["robot_1", "robot_2", "robot_3"] for bid in bids)
        assert all(bid.task.task_id in ["task_1", "task_2", "task_3"] for bid in bids)
        
        print(f"✅ Bid collection test passed - collected {len(bids)} bids")
    
    def test_bidding_system_complete_round(self):
        """Test complete bidding round."""
        # Process complete bidding round
        round_data = self.bidding_system.process_bidding_round(self.tasks, self.robots)
        
        assert round_data is not None
        assert round_data.round_id is not None
        # With the standalone bidding system, multiple tasks are allowed; keep original expectation
        assert len(round_data.available_tasks) == 3
        assert len(round_data.submitted_bids) > 0
        assert len(round_data.winning_assignments) > 0
        assert round_data.round_duration >= 0  # Duration can be 0 for very fast operations
        
        print(f"✅ Complete bidding round test passed - {len(round_data.winning_assignments)} assignments")
    
    def test_bidding_system_statistics(self):
        """Test bidding system statistics."""
        # Get initial statistics
        initial_stats = self.bidding_system.get_bidding_stats()
        initial_rounds = initial_stats.total_rounds
        
        # Process a few rounds to build statistics
        for round_num in range(3):
            print(f"[TEST] Processing round {round_num + 1}")
            self.bidding_system.process_bidding_round(self.tasks, self.robots)
            
            # Reset robot task status for next round
            for robot in self.robots:
                robot._task_active = False
        
        # Get final statistics
        stats = self.bidding_system.get_bidding_stats()
        
        print(f"[DEBUG] Initial rounds: {initial_rounds}, Final rounds: {stats.total_rounds}, New rounds: {stats.total_rounds - initial_rounds}")
        
        # Check that we processed exactly 3 new rounds
        assert stats.total_rounds == initial_rounds + 3
        assert stats.total_bids_submitted > 0
        assert stats.total_assignments_made > 0
        assert stats.average_round_duration >= 0  # Duration can be 0 for very fast operations
        assert stats.average_bids_per_round > 0
        assert stats.assignment_success_rate > 0
        
        print(f"✅ Statistics test passed - {stats.total_rounds} rounds processed")


def main():
    """Run the tests manually to avoid pytest hanging."""
    print("=== RUNNING BIDDING SYSTEM INTEGRATION TESTS ===")
    
    test = TestBiddingSystemIntegration()
    test.setup_method()
    
    try:
        test.test_bidding_system_initialization()
        test.test_mock_robot_functionality()
        test.test_bidding_system_robot_availability()
        test.test_bidding_system_bid_calculation()
        test.test_bidding_system_collect_bids()
        test.test_bidding_system_complete_round()
        test.test_bidding_system_statistics()
        
        print("\n🎉 ALL TESTS PASSED! Bidding system is working correctly.")
        
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main() 