"""
Unit tests for Bidding System Interface.

Tests the data structures, enums, and interface contracts defined in the
bidding system interface. Ensures proper validation and behavior.
"""
import pytest
from datetime import datetime
from typing import List, Dict, Any, Optional, Tuple
from unittest.mock import Mock

from interfaces.bidding_system_interface import (
    BidStatus, BiddingStrategy, RobotBid, TaskAssignment, 
    BiddingRound, BiddingStats, BiddingSystemError, IBiddingSystem
)
from interfaces.task_handler_interface import Task, TaskType, OperationalStatus, TaskHandlerStatus


class MockRobotAgent:
    """Mock RobotAgent implementation for testing."""
    
    def __init__(self, robot_id: str, is_idle: bool = True):
        self._robot_id = robot_id
        self._is_idle = is_idle
        self._position = (0.0, 0.0)
        self._battery_level = 80.0
        self._max_speed = 1.0
        self._max_payload = 10.0
        
        # Mock task handler interface
        self.task_handler_interface = Mock()
        self.task_handler_interface.is_idle.return_value = is_idle
        self.task_handler_interface.get_task_status.return_value = TaskHandlerStatus(
            has_active_task=not is_idle,
            task_id=None if is_idle else "task_1",
            operational_status=OperationalStatus.IDLE if is_idle else OperationalStatus.MOVING_TO_SHELF,
            progress=0.0
        )
    
    def get_robot_id(self) -> str:
        return self._robot_id
    
    def assign_task(self, task: Task) -> bool:
        if not self._is_idle:
            return False
        self._is_idle = False
        self.task_handler_interface.is_idle.return_value = False
        return True
    
    def get_status(self) -> Dict[str, Any]:
        return {
            'robot_id': self._robot_id,
            'position': self._position,
            'battery': self._battery_level,
            'task_active': not self._is_idle,
            'operational_status': 'idle' if self._is_idle else 'busy'
        }
    
    def is_available_for_bidding(self) -> bool:
        return self._is_idle
    
    def calculate_task_cost(self, task: Task) -> float:
        if not self._is_idle:
            raise ValueError("Cannot execute task")
        return 10.0  # Mock cost


class TestBidStatus:
    """Test BidStatus enum."""
    
    def test_bid_status_values(self):
        """Test that all bid status values are defined."""
        assert BidStatus.PENDING.value == "pending"
        assert BidStatus.ACCEPTED.value == "accepted"
        assert BidStatus.REJECTED.value == "rejected"
        assert BidStatus.EXPIRED.value == "expired"
    
    def test_bid_status_enumeration(self):
        """Test that all expected statuses are present."""
        expected_statuses = {"pending", "accepted", "rejected", "expired"}
        actual_statuses = {status.value for status in BidStatus}
        assert actual_statuses == expected_statuses


class TestBiddingStrategy:
    """Test BiddingStrategy enum."""
    
    def test_bidding_strategy_values(self):
        """Test that all bidding strategy values are defined."""
        assert BiddingStrategy.TRANSPARENT.value == "transparent"
        assert BiddingStrategy.ROUND_ROBIN.value == "round_robin"
        assert BiddingStrategy.DISTANCE_BASED.value == "distance_based"
        assert BiddingStrategy.BATTERY_AWARE.value == "battery_aware"
        assert BiddingStrategy.LOAD_BALANCED.value == "load_balanced"
        assert BiddingStrategy.HYBRID.value == "hybrid"
    
    def test_bidding_strategy_enumeration(self):
        """Test that all expected strategies are present."""
        expected_strategies = {
            "transparent", "round_robin", "distance_based", 
            "battery_aware", "load_balanced", "hybrid"
        }
        actual_strategies = {strategy.value for strategy in BiddingStrategy}
        assert actual_strategies == expected_strategies


class TestRobotBid:
    """Test RobotBid data structure."""
    
    def test_robot_bid_creation(self):
        """Test basic RobotBid creation."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        bid = RobotBid(robot_id="robot_1", task=task, bid_value=15.5)
        
        assert bid.robot_id == "robot_1"
        assert bid.task == task
        assert bid.bid_value == 15.5
        assert bid.status == BidStatus.PENDING
        assert isinstance(bid.bid_timestamp, datetime)
        assert bid.bid_metadata is None
    
    def test_robot_bid_with_metadata(self):
        """Test RobotBid creation with metadata."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        metadata = {"distance": 10.5, "battery_cost": 5.2}
        bid = RobotBid(
            robot_id="robot_1", 
            task=task, 
            bid_value=20.0,
            bid_metadata=metadata
        )
        
        assert bid.bid_metadata == metadata
    
    def test_robot_bid_validation_negative_value(self):
        """Test that negative bid values are rejected."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        
        with pytest.raises(ValueError, match="Bid value must be non-negative"):
            RobotBid(robot_id="robot_1", task=task, bid_value=-5.0)
    
    def test_robot_bid_validation_missing_robot_id(self):
        """Test that missing robot_id is rejected."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        
        with pytest.raises(ValueError, match="Robot ID and task must be provided"):
            RobotBid(robot_id="", task=task, bid_value=10.0)
    
    def test_robot_bid_validation_missing_task(self):
        """Test that missing task is rejected."""
        with pytest.raises(ValueError, match="Robot ID and task must be provided"):
            RobotBid(robot_id="robot_1", task=None, bid_value=10.0)
    
    def test_robot_bid_equality(self):
        """Test RobotBid equality comparison."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        
        bid1 = RobotBid(robot_id="robot_1", task=task, bid_value=10.0)
        bid2 = RobotBid(robot_id="robot_1", task=task, bid_value=10.0)
        bid3 = RobotBid(robot_id="robot_2", task=task, bid_value=10.0)
        
        assert bid1 == bid2
        assert bid1 != bid3


class TestTaskAssignment:
    """Test TaskAssignment data structure."""
    
    def test_task_assignment_creation(self):
        """Test basic TaskAssignment creation."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        assignment = TaskAssignment(robot_id="robot_1", task=task)
        
        assert assignment.robot_id == "robot_1"
        assert assignment.task == task
        assert isinstance(assignment.assignment_timestamp, datetime)
        assert assignment.assignment_metadata is None
        assert assignment.winning_bid_value is None
    
    def test_task_assignment_with_metadata(self):
        """Test TaskAssignment creation with metadata."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        metadata = {"strategy": "distance_based", "round_id": "round_1"}
        assignment = TaskAssignment(
            robot_id="robot_1", 
            task=task,
            assignment_metadata=metadata,
            winning_bid_value=15.5
        )
        
        assert assignment.assignment_metadata == metadata
        assert assignment.winning_bid_value == 15.5
    
    def test_task_assignment_validation_missing_robot_id(self):
        """Test that missing robot_id is rejected."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        
        with pytest.raises(ValueError, match="Robot ID and task must be provided"):
            TaskAssignment(robot_id="", task=task)
    
    def test_task_assignment_validation_missing_task(self):
        """Test that missing task is rejected."""
        with pytest.raises(ValueError, match="Robot ID and task must be provided"):
            TaskAssignment(robot_id="robot_1", task=None)


class TestBiddingRound:
    """Test BiddingRound data structure."""
    
    def test_bidding_round_creation(self):
        """Test basic BiddingRound creation."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        bid = RobotBid(robot_id="robot_1", task=task, bid_value=10.0)
        assignment = TaskAssignment(robot_id="robot_1", task=task)
        
        round_data = BiddingRound(
            round_id="round_1",
            available_tasks=[task],
            submitted_bids=[bid],
            winning_assignments=[assignment]
        )
        
        assert round_data.round_id == "round_1"
        assert round_data.available_tasks == [task]
        assert round_data.submitted_bids == [bid]
        assert round_data.winning_assignments == [assignment]
        assert isinstance(round_data.round_timestamp, datetime)
        assert round_data.round_duration is None
        assert round_data.round_metadata is None
    
    def test_bidding_round_with_metadata(self):
        """Test BiddingRound creation with metadata."""
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        bid = RobotBid(robot_id="robot_1", task=task, bid_value=10.0)
        assignment = TaskAssignment(robot_id="robot_1", task=task)
        metadata = {"strategy": "transparent", "robot_count": 1}
        
        round_data = BiddingRound(
            round_id="round_1",
            available_tasks=[task],
            submitted_bids=[bid],
            winning_assignments=[assignment],
            round_duration=0.5,
            round_metadata=metadata
        )
        
        assert round_data.round_duration == 0.5
        assert round_data.round_metadata == metadata


class TestBiddingStats:
    """Test BiddingStats data structure."""
    
    def test_bidding_stats_creation(self):
        """Test basic BiddingStats creation."""
        stats = BiddingStats(
            total_rounds=10,
            total_bids_submitted=25,
            total_assignments_made=20,
            average_round_duration=0.5,
            average_bids_per_round=2.5,
            assignment_success_rate=0.8
        )
        
        assert stats.total_rounds == 10
        assert stats.total_bids_submitted == 25
        assert stats.total_assignments_made == 20
        assert stats.average_round_duration == 0.5
        assert stats.average_bids_per_round == 2.5
        assert stats.assignment_success_rate == 0.8
        assert stats.last_round_timestamp is None
        assert stats.active_strategy == BiddingStrategy.TRANSPARENT
    
    def test_bidding_stats_with_timestamp(self):
        """Test BiddingStats creation with timestamp."""
        timestamp = datetime.now()
        stats = BiddingStats(
            total_rounds=5,
            total_bids_submitted=15,
            total_assignments_made=12,
            average_round_duration=0.3,
            average_bids_per_round=3.0,
            assignment_success_rate=0.8,
            last_round_timestamp=timestamp,
            active_strategy=BiddingStrategy.DISTANCE_BASED
        )
        
        assert stats.last_round_timestamp == timestamp
        assert stats.active_strategy == BiddingStrategy.DISTANCE_BASED


class TestBiddingSystemError:
    """Test BiddingSystemError exception."""
    
    def test_bidding_system_error_creation(self):
        """Test BiddingSystemError creation."""
        error = BiddingSystemError("Bidding failed")
        assert str(error) == "Bidding failed"
    
    def test_bidding_system_error_inheritance(self):
        """Test that BiddingSystemError inherits from Exception."""
        error = BiddingSystemError("Test error")
        assert isinstance(error, Exception)


class TestIBiddingSystem:
    """Test IBiddingSystem interface contract."""
    
    def test_interface_is_abstract(self):
        """Test that IBiddingSystem cannot be instantiated."""
        with pytest.raises(TypeError):
            IBiddingSystem()
    
    def test_interface_has_required_methods(self):
        """Test that IBiddingSystem has all required abstract methods."""
        required_methods = {
            'collect_bids',
            'select_winning_bids', 
            'process_bidding_round',
            'set_bidding_strategy',
            'get_current_strategy',
            'get_bidding_stats',
            'get_recent_bidding_rounds',
            'reset_statistics',
            'is_robot_available_for_task',
            'calculate_bid_value'
        }
        
        actual_methods = {
            method for method in dir(IBiddingSystem) 
            if not method.startswith('_')
        }
        
        # Check that all required methods are present
        for method in required_methods:
            assert method in actual_methods, f"Missing method: {method}"
    
    def test_interface_method_signatures(self):
        """Test that interface methods have correct signatures."""
        # This test ensures the interface methods are properly defined
        # with correct type hints and documentation
        
        # Test that collect_bids signature is correct
        import inspect
        sig = inspect.signature(IBiddingSystem.collect_bids)
        params = list(sig.parameters.keys())
        
        # Should have self, available_tasks, available_robots
        assert params == ['self', 'available_tasks', 'available_robots']
        
        # Test return type annotation (check it's a List type)
        assert "List" in str(sig.return_annotation)


class TestMockRobotAgent:
    """Test MockRobotAgent implementation."""
    
    def test_mock_robot_agent_creation(self):
        """Test MockRobotAgent creation."""
        robot = MockRobotAgent("robot_1")
        
        assert robot.get_robot_id() == "robot_1"
        assert robot.is_available_for_bidding() is True
        assert robot.task_handler_interface.is_idle() is True
    
    def test_mock_robot_agent_status(self):
        """Test MockRobotAgent status."""
        robot = MockRobotAgent("robot_1")
        status = robot.get_status()
        
        assert status['robot_id'] == "robot_1"
        assert status['position'] == (0.0, 0.0)
        assert status['battery'] == 80.0
        assert status['task_active'] is False
        assert status['operational_status'] == 'idle'
    
    def test_mock_robot_agent_task_execution(self):
        """Test MockRobotAgent task execution."""
        robot = MockRobotAgent("robot_1")
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        
        assert robot.is_available_for_bidding() is True
        assert robot.calculate_task_cost(task) == 10.0
        assert robot.assign_task(task) is True
        assert robot.is_available_for_bidding() is False
    
    def test_mock_robot_agent_busy_status(self):
        """Test MockRobotAgent when busy."""
        robot = MockRobotAgent("robot_1", is_idle=False)
        task = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        
        assert robot.is_available_for_bidding() is False
        assert robot.assign_task(task) is False
        
        with pytest.raises(ValueError, match="Cannot execute task"):
            robot.calculate_task_cost(task)
    
    def test_mock_robot_agent_task_handler_interface(self):
        """Test MockRobotAgent task handler interface."""
        robot = MockRobotAgent("robot_1")
        task_status = robot.task_handler_interface.get_task_status()
        
        assert task_status.has_active_task is False
        assert task_status.task_id is None
        assert task_status.operational_status == OperationalStatus.IDLE
        assert task_status.progress == 0.0 