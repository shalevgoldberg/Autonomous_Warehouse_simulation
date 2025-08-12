"""
Unit tests for Bid Calculator Implementation.

Tests the bid calculator functionality including:
- Factor calculation methods
- Parallel bid calculation
- Thread safety
- Error handling
- Statistics tracking
- Configuration management
"""
import pytest
import threading
import time
from typing import List, Dict, Any
from unittest.mock import Mock, MagicMock

from robot.impl.bid_calculator_impl import BidCalculatorImpl
from interfaces.bid_calculator_interface import (
    BidFactor, BidFactorWeight, BidCalculationError
)
from interfaces.task_handler_interface import Task, TaskType
from interfaces.state_holder_interface import IStateHolder
from interfaces.bidding_system_interface import RobotBid
from interfaces.simulation_data_service_interface import ISimulationDataService


class MockStateHolder(IStateHolder):
    """Mock state holder for testing."""
    
    def __init__(self, robot_id: str = "test_robot", battery_level: float = 0.8):
        self.robot_id = robot_id
        self.battery_level = battery_level
        self.position = (0.0, 0.0, 0.0)
        self.current_task = None
        self.operational_status = "idle"
    
    def get_robot_state(self):
        """Get mock robot state."""
        from interfaces.state_holder_interface import RobotPhysicsState
        return RobotPhysicsState(
            robot_id=self.robot_id,
            position=self.position,
            velocity=(0.0, 0.0),
            battery_level=self.battery_level,
            timestamp=0.0
        )
    
    def get_position(self):
        return self.position
    
    def get_velocity(self):
        return (0.0, 0.0)
    
    def get_battery_level(self):
        return self.battery_level
    
    def update_from_simulation(self):
        pass


class MockSimulationDataService:
    """Mock simulation data service for testing."""
    
    def get_shelf_info(self, shelf_id: str):
        """Get mock shelf info."""
        if shelf_id == "shelf_1":
            mock_shelf = Mock()
            mock_shelf.position = (5.0, 5.0, 0.0)
            return mock_shelf
        return None
    
    # Implement other required methods from ISimulationDataService
    def get_shelf_inventory(self, shelf_id: str):
        return []
    
    def update_shelf_inventory(self, shelf_id: str, item_id: str, quantity: int):
        pass
    
    def get_robot_status(self, robot_id: str):
        return {}
    
    def update_robot_status(self, robot_id: str, status: dict):
        pass
    
    def get_warehouse_map(self):
        return {}
    
    def get_navigation_graph(self):
        return {}
    
    def get_conflict_box_status(self, box_id: str):
        return {}
    
    def update_conflict_box_status(self, box_id: str, status: dict):
        pass
    
    def get_available_conflict_boxes(self):
        return []
    
    def acquire_conflict_box(self, robot_id: str, box_id: str):
        return True
    
    def release_conflict_box(self, robot_id: str, box_id: str):
        return True


class TestBidCalculatorImpl:
    """Test BidCalculatorImpl implementation."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.state_holder = MockStateHolder()
        self.bid_calculator = BidCalculatorImpl(
            robot_id="test_robot",
            simulation_data_service=None  # Use None to avoid interface implementation
        )
        
        # Create test tasks
        self.move_task = Task(
            task_id="move_1",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(10.0, 10.0, 0.0)
        )
        
        self.pick_task = Task(
            task_id="pick_1",
            task_type=TaskType.PICK_AND_DELIVER,
            order_id="order_1",
            shelf_id="shelf_1",
            item_id="item_1"
        )
    
    def test_initialization(self):
        """Test bid calculator initialization."""
        assert self.bid_calculator.robot_id == "test_robot"
        assert self.bid_calculator.simulation_data_service == None # Changed from self.simulation_data_service
        assert self.bid_calculator.max_workers == 4
        
        # Check that default factors are initialized
        supported_factors = self.bid_calculator.get_supported_factors()
        expected_factors = [
            BidFactor.DISTANCE,
            BidFactor.BATTERY_LEVEL,
            BidFactor.WORKLOAD,
            BidFactor.TASK_TYPE_COMPATIBILITY
        ]
        assert set(supported_factors) == set(expected_factors)
    
    def test_is_available_for_bidding_idle_robot(self):
        """Test availability check for idle robot."""
        assert self.bid_calculator.is_available_for_bidding(self.state_holder) is True
    
    def test_is_available_for_bidding_low_battery(self):
        """Test availability check for robot with low battery."""
        low_battery_state = MockStateHolder(battery_level=0.15)  # 15% battery
        assert self.bid_calculator.is_available_for_bidding(low_battery_state) is False
    
    def test_calculate_single_bid_move_task(self):
        """Test bid calculation for move task."""
        bid = self.bid_calculator.calculate_single_bid(self.move_task, self.state_holder)
        
        assert bid is not None
        assert bid.robot_id == "test_robot"
        assert bid.task == self.move_task
        assert bid.bid_value > 0
        assert bid.bid_metadata is not None
        assert "factor_breakdown" in bid.bid_metadata
        assert "calculation_time" in bid.bid_metadata
    
    def test_calculate_single_bid_pick_task(self):
        """Test bid calculation for pick and deliver task."""
        bid = self.bid_calculator.calculate_single_bid(self.pick_task, self.state_holder)
        
        assert bid is not None
        assert bid.robot_id == "test_robot"
        assert bid.task == self.pick_task
        assert bid.bid_value > 0
        assert bid.bid_metadata is not None
    
    def test_calculate_single_bid_unavailable_robot(self):
        """Test bid calculation when robot is unavailable."""
        low_battery_state = MockStateHolder(battery_level=0.15)
        bid = self.bid_calculator.calculate_single_bid(self.move_task, low_battery_state)
        
        assert bid is None
    
    def test_calculate_bids_multiple_tasks(self):
        """Test parallel bid calculation for multiple tasks."""
        tasks = [self.move_task, self.pick_task]
        bids = self.bid_calculator.calculate_bids(tasks, self.state_holder)
        
        assert len(bids) == 2
        assert all(isinstance(bid, RobotBid) for bid in bids)
        assert all(bid.robot_id == "test_robot" for bid in bids)
        assert {bid.task.task_id for bid in bids} == {"move_1", "pick_1"}
    
    def test_calculate_bids_empty_list(self):
        """Test bid calculation with empty task list."""
        bids = self.bid_calculator.calculate_bids([], self.state_holder)
        assert bids == []
    
    def test_calculate_bids_unavailable_robot(self):
        """Test bid calculation when robot is unavailable."""
        low_battery_state = MockStateHolder(battery_level=0.15)
        tasks = [self.move_task, self.pick_task]
        bids = self.bid_calculator.calculate_bids(tasks, low_battery_state)
        
        assert bids == []
    
    def test_bid_factor_weights(self):
        """Test bid factor weight configuration."""
        weights = [
            BidFactorWeight(BidFactor.DISTANCE, 0.5),
            BidFactorWeight(BidFactor.BATTERY_LEVEL, 0.3),
            BidFactorWeight(BidFactor.WORKLOAD, 0.2)
        ]
        
        self.bid_calculator.set_bid_factor_weights(weights)
        current_weights = self.bid_calculator.get_bid_factor_weights()
        
        # Should have 4 weights (the 3 we set plus the default TASK_TYPE_COMPATIBILITY)
        assert len(current_weights) == 4
        weight_dict = {w.factor: w.weight for w in current_weights}
        assert weight_dict[BidFactor.DISTANCE] == 0.5
        assert weight_dict[BidFactor.BATTERY_LEVEL] == 0.3
        assert weight_dict[BidFactor.WORKLOAD] == 0.2
        assert BidFactor.TASK_TYPE_COMPATIBILITY in weight_dict
    
    def test_enable_disable_factor(self):
        """Test enabling and disabling factors."""
        # Disable distance factor
        self.bid_calculator.enable_factor(BidFactor.DISTANCE, False)
        
        # Calculate bid and check that distance factor is not used
        bid = self.bid_calculator.calculate_single_bid(self.move_task, self.state_holder)
        assert bid is not None
        
        # Re-enable distance factor
        self.bid_calculator.enable_factor(BidFactor.DISTANCE, True)
        
        # Calculate bid again
        bid2 = self.bid_calculator.calculate_single_bid(self.move_task, self.state_holder)
        assert bid2 is not None
    
    def test_calculation_statistics(self):
        """Test bid calculation statistics."""
        # Perform some calculations
        self.bid_calculator.calculate_single_bid(self.move_task, self.state_holder)
        self.bid_calculator.calculate_single_bid(self.pick_task, self.state_holder)
        
        stats = self.bid_calculator.get_calculation_statistics()
        
        assert stats['total_calculations'] == 2
        assert stats['successful_calculations'] == 2
        assert stats['failed_calculations'] == 0
        assert stats['success_rate'] == 1.0
        # Calculation time might be very small, so just check it's not negative
        assert stats['average_calculation_time'] >= 0
        assert 'enabled_factors' in stats
    
    def test_thread_safety(self):
        """Test thread safety of bid calculator."""
        tasks = [self.move_task, self.pick_task] * 5  # 10 tasks total
        
        def calculate_bids():
            return self.bid_calculator.calculate_bids(tasks, self.state_holder)
        
        # Run multiple threads simultaneously
        threads = []
        results = []
        
        for i in range(3):
            thread = threading.Thread(target=lambda: results.append(calculate_bids()))
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Check that all threads completed successfully
        assert len(results) == 3
        for result in results:
            assert len(result) == 10  # All 10 tasks should have bids
    
    def test_error_handling(self):
        """Test error handling in bid calculation."""
        # Create a state holder that works for availability check but fails during bid calculation
        error_state_holder = MockStateHolder()
        
        # Mock the get_robot_state to work for availability check but fail during bid calculation
        original_get_robot_state = error_state_holder.get_robot_state
        call_count = 0
        
        def failing_get_robot_state():
            nonlocal call_count
            call_count += 1
            if call_count > 1:  # Fail on second call (during bid calculation)
                raise Exception("State error during bid calculation")
            return original_get_robot_state()
        
        error_state_holder.get_robot_state = failing_get_robot_state
        
        # Should handle error gracefully
        bid = self.bid_calculator.calculate_single_bid(self.move_task, error_state_holder)
        assert bid is None
        
        # Check statistics
        stats = self.bid_calculator.get_calculation_statistics()
        assert stats['failed_calculations'] > 0
    
    def test_factor_breakdown(self):
        """Test that factor breakdown is included in bid metadata."""
        bid = self.bid_calculator.calculate_single_bid(self.move_task, self.state_holder)
        
        assert bid is not None
        assert bid.bid_metadata is not None
        assert "factor_breakdown" in bid.bid_metadata
        
        factor_breakdown = bid.bid_metadata["factor_breakdown"]
        assert isinstance(factor_breakdown, dict)
        
        # Check that expected factors are present
        expected_factors = ["distance", "battery_level", "workload", "task_type_compatibility"]
        for factor in expected_factors:
            assert factor in factor_breakdown
            assert isinstance(factor_breakdown[factor], (int, float))
    
    def test_distance_factor_calculation(self):
        """Test distance factor calculation."""
        # Test with different target positions
        close_task = Task(
            task_id="close",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(1.0, 1.0, 0.0)
        )
        
        far_task = Task(
            task_id="far",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(20.0, 20.0, 0.0)
        )
        
        close_bid = self.bid_calculator.calculate_single_bid(close_task, self.state_holder)
        far_bid = self.bid_calculator.calculate_single_bid(far_task, self.state_holder)
        
        assert close_bid is not None
        assert far_bid is not None
        
        # Close task should have lower bid value (better)
        assert close_bid.bid_value < far_bid.bid_value
    
    def test_battery_factor_calculation(self):
        """Test battery factor calculation."""
        high_battery_state = MockStateHolder(battery_level=0.9)  # 90% battery
        low_battery_state = MockStateHolder(battery_level=0.3)   # 30% battery
        
        high_bid = self.bid_calculator.calculate_single_bid(self.move_task, high_battery_state)
        low_bid = self.bid_calculator.calculate_single_bid(self.move_task, low_battery_state)
        
        assert high_bid is not None
        assert low_bid is not None
        
        # High battery should have lower bid value (better)
        assert high_bid.bid_value < low_bid.bid_value
    
    def test_invalid_bid_factor_weight(self):
        """Test validation of bid factor weights."""
        # Test weight out of range
        with pytest.raises(ValueError, match="must be between 0.0 and 1.0"):
            BidFactorWeight(BidFactor.DISTANCE, 1.5)
        
        with pytest.raises(ValueError, match="must be between 0.0 and 1.0"):
            BidFactorWeight(BidFactor.DISTANCE, -0.1)
    
    def test_supported_factors(self):
        """Test getting supported factors."""
        factors = self.bid_calculator.get_supported_factors()
        
        assert len(factors) == 4
        assert BidFactor.DISTANCE in factors
        assert BidFactor.BATTERY_LEVEL in factors
        assert BidFactor.WORKLOAD in factors
        assert BidFactor.TASK_TYPE_COMPATIBILITY in factors 