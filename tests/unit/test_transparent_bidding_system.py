"""
Unit tests for Transparent Bidding System Implementation.

Tests the transparent bidding system implementation, including:
- Bid collection and selection logic
- Statistics tracking and updates
- Thread safety
- Error handling
- Edge cases and boundary conditions
"""
import pytest
import threading
import time
from typing import List, Dict, Any
from unittest.mock import Mock, patch

from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
from interfaces.bidding_system_interface import (
    BiddingStrategy, BidStatus, BiddingSystemError
)
from interfaces.task_handler_interface import Task, TaskType, OperationalStatus, TaskHandlerStatus


class MockRobotAgent:
    """Mock RobotAgent for testing transparent bidding system."""
    
    def __init__(self, robot_id: str, position: tuple = (0.0, 0.0), 
                 battery: float = 80.0, is_idle: bool = True):
        self._robot_id = robot_id
        self._position = position
        self._battery = battery
        self._is_idle = is_idle
        self._operational_status = OperationalStatus.IDLE if is_idle else OperationalStatus.MOVING_TO_SHELF
        
        # Mock task handler interface
        self.task_handler_interface = Mock()
        self.task_handler_interface.is_idle.return_value = is_idle
        self.task_handler_interface.get_task_status.return_value = TaskHandlerStatus(
            has_active_task=not is_idle,
            task_id=None if is_idle else "task_1",
            operational_status=self._operational_status,
            progress=0.0
        )
    
    def get_robot_id(self) -> str:
        return self._robot_id
    
    def get_status(self) -> Dict[str, Any]:
        return {
            'robot_id': self._robot_id,
            'position': self._position,
            'battery': self._battery,
            'task_active': not self._is_idle,
            'operational_status': self._operational_status.value
        }
    
    def assign_task(self, task: Task) -> bool:
        if not self._is_idle:
            return False
        self._is_idle = False
        self._operational_status = OperationalStatus.MOVING_TO_SHELF
        self.task_handler_interface.is_idle.return_value = False
        return True


class TestTransparentBiddingSystem:
    """Test TransparentBiddingSystem implementation."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.bidding_system = TransparentBiddingSystem()
        self.robot1 = MockRobotAgent("robot_1", position=(0.0, 0.0), battery=80.0)
        self.robot2 = MockRobotAgent("robot_2", position=(5.0, 5.0), battery=60.0)
        self.task1 = Task.create_pick_and_deliver_task("task_1", "order_1", "shelf_1", "item_1")
        self.task2 = Task.create_pick_and_deliver_task("task_2", "order_2", "shelf_2", "item_2")
    
    def test_initialization(self):
        """Test transparent bidding system initialization."""
        assert self.bidding_system.get_current_strategy() == BiddingStrategy.TRANSPARENT
        
        stats = self.bidding_system.get_bidding_stats()
        assert stats.total_rounds == 0
        assert stats.total_bids_submitted == 0
        assert stats.total_assignments_made == 0
        assert stats.average_round_duration == 0.0
        assert stats.average_bids_per_round == 0.0
        assert stats.assignment_success_rate == 0.0
    
    def test_set_bidding_strategy_transparent(self):
        """Test setting transparent bidding strategy."""
        self.bidding_system.set_bidding_strategy(BiddingStrategy.TRANSPARENT)
        assert self.bidding_system.get_current_strategy() == BiddingStrategy.TRANSPARENT
    
    def test_set_bidding_strategy_unsupported(self):
        """Test that unsupported strategies are rejected."""
        with pytest.raises(BiddingSystemError, match="only supports TRANSPARENT strategy"):
            self.bidding_system.set_bidding_strategy(BiddingStrategy.DISTANCE_BASED)
    
    def test_is_robot_available_for_task_idle_robot(self):
        """Test robot availability check for idle robot."""
        assert self.bidding_system.is_robot_available_for_task(self.robot1, None) is True
        assert self.bidding_system.is_robot_available_for_task(self.robot1, self.task1) is True
    
    def test_is_robot_available_for_task_busy_robot(self):
        """Test robot availability check for busy robot."""
        busy_robot = MockRobotAgent("robot_3", is_idle=False)
        assert self.bidding_system.is_robot_available_for_task(busy_robot, None) is False
    
    def test_is_robot_available_for_task_low_battery(self):
        """Test robot availability check for robot with low battery."""
        low_battery_robot = MockRobotAgent("robot_4", battery=15.0)
        assert self.bidding_system.is_robot_available_for_task(low_battery_robot, None) is False
    
    def test_is_robot_available_for_task_error_state(self):
        """Test robot availability check for robot in error state."""
        error_robot = MockRobotAgent("robot_5")
        error_robot._operational_status = OperationalStatus.ERROR
        assert self.bidding_system.is_robot_available_for_task(error_robot, None) is False
    
    def test_calculate_bid_value_pick_and_deliver(self):
        """Test bid value calculation for pick and deliver task."""
        bid_value = self.bidding_system.calculate_bid_value(self.robot1, self.task1)
        assert bid_value > 0
        assert isinstance(bid_value, float)
    
    def test_calculate_bid_value_move_to_position(self):
        """Test bid value calculation for move to position task."""
        move_task = Task(
            task_id="move_1",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(10.0, 10.0)
        )
        bid_value = self.bidding_system.calculate_bid_value(self.robot1, move_task)
        assert bid_value > 0
        assert isinstance(bid_value, float)
    
    def test_calculate_bid_value_battery_factor(self):
        """Test that battery level affects bid value."""
        high_battery_robot = MockRobotAgent("robot_6", battery=90.0)
        low_battery_robot = MockRobotAgent("robot_7", battery=30.0)
        
        high_bid = self.bidding_system.calculate_bid_value(high_battery_robot, self.task1)
        low_bid = self.bidding_system.calculate_bid_value(low_battery_robot, self.task1)
        
        # Lower battery should result in higher bid value (worse)
        assert low_bid > high_bid
    
    def test_collect_bids_single_robot_single_task(self):
        """Test bid collection with single robot and single task."""
        bids = self.bidding_system.collect_bids([self.task1], [self.robot1])
        
        assert len(bids) == 1
        bid = bids[0]
        assert bid.robot_id == "robot_1"
        assert bid.task == self.task1
        assert bid.bid_value > 0
        assert bid.status == BidStatus.PENDING
        assert bid.bid_metadata is not None
        assert bid.bid_metadata["strategy"] == "transparent"
    
    def test_collect_bids_multiple_robots_multiple_tasks(self):
        """Test bid collection with multiple robots and tasks."""
        bids = self.bidding_system.collect_bids([self.task1, self.task2], [self.robot1, self.robot2])
        
        assert len(bids) == 4  # 2 robots × 2 tasks
        robot_ids = {bid.robot_id for bid in bids}
        task_ids = {bid.task.task_id for bid in bids}
        
        assert robot_ids == {"robot_1", "robot_2"}
        assert task_ids == {"task_1", "task_2"}
    
    def test_collect_bids_no_available_robots(self):
        """Test bid collection when no robots are available."""
        busy_robot = MockRobotAgent("robot_8", is_idle=False)
        bids = self.bidding_system.collect_bids([self.task1], [busy_robot])
        
        assert len(bids) == 0
    
    def test_collect_bids_no_tasks(self):
        """Test bid collection with no tasks."""
        bids = self.bidding_system.collect_bids([], [self.robot1])
        
        assert len(bids) == 0
    
    def test_select_winning_bids_single_task(self):
        """Test winning bid selection for single task."""
        bid1 = Mock()
        bid1.robot_id = "robot_1"
        bid1.task = self.task1
        bid1.bid_value = 10.0
        bid1.status = BidStatus.PENDING
        
        bid2 = Mock()
        bid2.robot_id = "robot_2"
        bid2.task = self.task1
        bid2.bid_value = 5.0  # Lower bid value
        bid2.status = BidStatus.PENDING
        
        assignments = self.bidding_system.select_winning_bids([bid1, bid2])
        
        assert len(assignments) == 1
        assignment = assignments[0]
        assert assignment.robot_id == "robot_2"  # Lower bid value wins
        assert assignment.task == self.task1
        assert assignment.winning_bid_value == 5.0
        assert bid2.status == BidStatus.ACCEPTED
    
    def test_select_winning_bids_multiple_tasks(self):
        """Test winning bid selection for multiple tasks."""
        # Task 1 bids
        bid1_task1 = Mock()
        bid1_task1.robot_id = "robot_1"
        bid1_task1.task = self.task1
        bid1_task1.bid_value = 10.0
        bid1_task1.status = BidStatus.PENDING
        
        bid2_task1 = Mock()
        bid2_task1.robot_id = "robot_2"
        bid2_task1.task = self.task1
        bid2_task1.bid_value = 5.0
        bid2_task1.status = BidStatus.PENDING
        
        # Task 2 bids
        bid1_task2 = Mock()
        bid1_task2.robot_id = "robot_1"
        bid1_task2.task = self.task2
        bid1_task2.bid_value = 3.0
        bid1_task2.status = BidStatus.PENDING
        
        bid2_task2 = Mock()
        bid2_task2.robot_id = "robot_2"
        bid2_task2.task = self.task2
        bid2_task2.bid_value = 8.0
        bid2_task2.status = BidStatus.PENDING
        
        assignments = self.bidding_system.select_winning_bids([
            bid1_task1, bid2_task1, bid1_task2, bid2_task2
        ])
        
        assert len(assignments) == 2
        
        # Check task 1 assignment
        task1_assignment = next(a for a in assignments if a.task == self.task1)
        assert task1_assignment.robot_id == "robot_2"  # Lower bid (5.0)
        
        # Check task 2 assignment
        task2_assignment = next(a for a in assignments if a.task == self.task2)
        assert task2_assignment.robot_id == "robot_1"  # Lower bid (3.0)
    
    def test_process_bidding_round_complete_flow(self):
        """Test complete bidding round processing."""
        round_data = self.bidding_system.process_bidding_round(
            [self.task1, self.task2], [self.robot1, self.robot2]
        )
        
        assert round_data.round_id is not None
        assert len(round_data.available_tasks) == 2
        assert len(round_data.submitted_bids) == 4  # 2 robots × 2 tasks
        assert len(round_data.winning_assignments) == 2  # One assignment per task
        assert round_data.round_duration >= 0  # Duration can be 0 for fast operations
        assert round_data.round_metadata["strategy"] == "transparent"
    
    def test_process_bidding_round_statistics_update(self):
        """Test that statistics are updated after bidding round."""
        initial_stats = self.bidding_system.get_bidding_stats()
        assert initial_stats.total_rounds == 0
        
        self.bidding_system.process_bidding_round([self.task1], [self.robot1])
        
        updated_stats = self.bidding_system.get_bidding_stats()
        assert updated_stats.total_rounds == 1
        assert updated_stats.total_bids_submitted == 1
        assert updated_stats.total_assignments_made == 1
        assert updated_stats.average_round_duration >= 0  # Duration can be 0 for fast operations
        assert updated_stats.average_bids_per_round == 1.0
        assert updated_stats.assignment_success_rate == 1.0
    
    def test_get_recent_bidding_rounds(self):
        """Test retrieval of recent bidding rounds."""
        # Process multiple rounds
        for i in range(3):
            self.bidding_system.process_bidding_round([self.task1], [self.robot1])
        
        recent_rounds = self.bidding_system.get_recent_bidding_rounds(limit=2)
        assert len(recent_rounds) == 2
        
        recent_rounds = self.bidding_system.get_recent_bidding_rounds(limit=10)
        assert len(recent_rounds) == 3
    
    def test_reset_statistics(self):
        """Test statistics reset functionality."""
        # Process a round to populate statistics
        self.bidding_system.process_bidding_round([self.task1], [self.robot1])
        
        # Verify statistics are populated
        stats = self.bidding_system.get_bidding_stats()
        assert stats.total_rounds > 0
        
        # Reset statistics
        self.bidding_system.reset_statistics()
        
        # Verify statistics are reset
        reset_stats = self.bidding_system.get_bidding_stats()
        assert reset_stats.total_rounds == 0
        assert reset_stats.total_bids_submitted == 0
        assert reset_stats.total_assignments_made == 0
        assert reset_stats.average_round_duration == 0.0
        assert reset_stats.average_bids_per_round == 0.0
        assert reset_stats.assignment_success_rate == 0.0
        
        # Verify recent rounds are cleared
        recent_rounds = self.bidding_system.get_recent_bidding_rounds()
        assert len(recent_rounds) == 0
    
    def test_thread_safety(self):
        """Test thread safety of the bidding system."""
        results = []
        errors = []
        
        def worker(worker_id: int):
            try:
                robot = MockRobotAgent(f"robot_{worker_id}")
                task = Task.create_pick_and_deliver_task(f"task_{worker_id}", f"order_{worker_id}", f"shelf_{worker_id}", f"item_{worker_id}")
                
                # Process bidding round
                round_data = self.bidding_system.process_bidding_round([task], [robot])
                results.append(round_data)
                
            except Exception as e:
                errors.append(e)
        
        # Create multiple threads
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
        stats = self.bidding_system.get_bidding_stats()
        assert stats.total_rounds == 5
    
    def test_error_handling_invalid_robot(self):
        """Test error handling with invalid robot data."""
        invalid_robot = Mock()
        invalid_robot.get_robot_id.return_value = "invalid_robot"
        invalid_robot.get_status.side_effect = Exception("Robot error")
        
        # Should handle gracefully and skip invalid robot
        bids = self.bidding_system.collect_bids([self.task1], [invalid_robot])
        assert len(bids) == 0
    
    def test_error_handling_bid_calculation_failure(self):
        """Test error handling when bid calculation fails."""
        robot_with_error = Mock()
        robot_with_error.get_robot_id.return_value = "error_robot"
        robot_with_error.get_status.side_effect = Exception("Status error")
        
        with pytest.raises(BiddingSystemError, match="Failed to calculate bid value"):
            self.bidding_system.calculate_bid_value(robot_with_error, self.task1)
    
    def test_error_handling_collection_failure(self):
        """Test error handling when bid collection fails."""
        robot_with_error = Mock()
        robot_with_error.get_robot_id.side_effect = Exception("Robot ID error")
        
        with pytest.raises(BiddingSystemError, match="Failed to collect bids"):
            self.bidding_system.collect_bids([self.task1], [robot_with_error])
    
    def test_error_handling_selection_failure(self):
        """Test error handling when bid selection fails."""
        invalid_bid = Mock()
        invalid_bid.task = None  # Invalid bid
        
        with pytest.raises(BiddingSystemError, match="Failed to select winning bids"):
            self.bidding_system.select_winning_bids([invalid_bid])
    
    def test_edge_case_empty_inputs(self):
        """Test edge cases with empty inputs."""
        # Empty tasks, empty robots
        bids = self.bidding_system.collect_bids([], [])
        assert len(bids) == 0
        
        assignments = self.bidding_system.select_winning_bids([])
        assert len(assignments) == 0
        
        round_data = self.bidding_system.process_bidding_round([], [])
        assert len(round_data.submitted_bids) == 0
        assert len(round_data.winning_assignments) == 0
    
    def test_edge_case_single_robot_no_tasks(self):
        """Test edge case with single robot but no tasks."""
        bids = self.bidding_system.collect_bids([], [self.robot1])
        assert len(bids) == 0
    
    def test_edge_case_single_task_no_robots(self):
        """Test edge case with single task but no robots."""
        bids = self.bidding_system.collect_bids([self.task1], [])
        assert len(bids) == 0
    
    def test_bid_value_always_positive(self):
        """Test that bid values are always positive."""
        robot = MockRobotAgent("test_robot", battery=100.0)
        task = Task.create_pick_and_deliver_task("test_task", "test_order", "test_shelf", "test_item")
        
        bid_value = self.bidding_system.calculate_bid_value(robot, task)
        assert bid_value > 0
        assert bid_value >= 0.1  # Minimum bid value
    
    def test_round_id_uniqueness(self):
        """Test that bidding round IDs are unique."""
        round1 = self.bidding_system.process_bidding_round([self.task1], [self.robot1])
        round2 = self.bidding_system.process_bidding_round([self.task2], [self.robot2])
        
        assert round1.round_id != round2.round_id 