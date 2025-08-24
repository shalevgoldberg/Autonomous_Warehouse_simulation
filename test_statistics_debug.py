#!/usr/bin/env python3
"""
Debug test for bidding system statistics.
"""
import sys
import os

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_statistics_debug():
    """Test statistics tracking step by step."""
    print("=== STATISTICS DEBUG TEST ===")
    
    try:
        from warehouse.impl.transparent_bidding_system_impl import TransparentBiddingSystem
        from interfaces.task_handler_interface import Task, TaskType
        
        # Create bidding system
        bidding_system = TransparentBiddingSystem()
        
        # Check initial stats
        initial_stats = bidding_system.get_bidding_stats()
        print(f"Initial stats: {initial_stats.total_rounds} rounds, {initial_stats.total_bids_submitted} bids")
        
        # Create test task and robot
        task = Task(
            task_id="test_task",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(5.0, 5.0),
            priority=1
        )
        
        class MockRobot:
            def __init__(self, robot_id: str):
                self.robot_id = robot_id
                self._position = (0.0, 0.0)
                self._battery = 100.0
                self._task_active = False
                self._operational_status = 'idle'
            
            def get_status(self):
                return {
                    'robot_id': self.robot_id,
                    'position': self._position,
                    'battery': self._battery,
                    'task_active': self._task_active,
                    'operational_status': self._operational_status
                }
        
        robot = MockRobot("test_robot")
        
        # Process one round
        print("\nProcessing first round...")
        round_data = bidding_system.process_bidding_round([task], [robot])
        print(f"Round completed: {round_data.round_id}")
        
        # Check stats after first round
        stats_after_1 = bidding_system.get_bidding_stats()
        print(f"After 1 round: {stats_after_1.total_rounds} rounds, {stats_after_1.total_bids_submitted} bids")
        
        # Process second round
        print("\nProcessing second round...")
        round_data2 = bidding_system.process_bidding_round([task], [robot])
        print(f"Round completed: {round_data2.round_id}")
        
        # Check stats after second round
        stats_after_2 = bidding_system.get_bidding_stats()
        print(f"After 2 rounds: {stats_after_2.total_rounds} rounds, {stats_after_2.total_bids_submitted} bids")
        
        # Process third round
        print("\nProcessing third round...")
        round_data3 = bidding_system.process_bidding_round([task], [robot])
        print(f"Round completed: {round_data3.round_id}")
        
        # Check stats after third round
        stats_after_3 = bidding_system.get_bidding_stats()
        print(f"After 3 rounds: {stats_after_3.total_rounds} rounds, {stats_after_3.total_bids_submitted} bids")
        
        print("\n✅ Statistics debug test completed")
        return True
        
    except Exception as e:
        print(f"❌ Statistics debug test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_statistics_debug()
