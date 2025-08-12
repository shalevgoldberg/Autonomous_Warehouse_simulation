#!/usr/bin/env python3
"""
Debug script to test robot controller step by step.
"""
import time
import threading
from unittest.mock import Mock, MagicMock
from typing import List, Dict, Any

from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.task_handler_interface import Task, TaskType
from interfaces.configuration_interface import IConfigurationProvider, SystemConfig, BidConfig
from warehouse.impl.robot_controller_impl import RobotController, ParallelBiddingResult

class SimpleMockRobot:
    """Simplified mock robot for debugging."""
    
    def __init__(self, robot_id: str):
        self.config = Mock()
        self.config.robot_id = robot_id
        self._position = (0.0, 0.0)
        self._battery = 100.0
        self._task_active = False
        self._operational_status = 'idle'
    
    def calculate_bids(self, tasks: List[Task]) -> List[Any]:
        """Calculate bids for tasks."""
        print(f"[DEBUG] Robot {self.config.robot_id} calculating bids for {len(tasks)} tasks")
        
        # Simulate some calculation time
        time.sleep(0.01)  # 10ms delay
        
        from interfaces.bidding_system_interface import RobotBid
        bids = []
        for task in tasks:
            bid = RobotBid(
                robot_id=self.config.robot_id,
                task=task,
                bid_value=10.0,
                bid_metadata={"debug": True}
            )
            bids.append(bid)
        
        print(f"[DEBUG] Robot {self.config.robot_id} completed bid calculation")
        return bids
    
    def is_available_for_bidding(self) -> bool:
        """Check if robot is available for bidding."""
        return (not self._task_active and 
               self._operational_status == 'idle' and 
               self._battery > 20.0)
    
    def get_bid_calculation_statistics(self) -> Dict[str, Any]:
        """Get bid calculation statistics."""
        return {
            'robot_id': self.config.robot_id,
            'available_for_bidding': self.is_available_for_bidding(),
            'current_position': self._position,
            'battery_level': self._battery,
            'operational_status': self._operational_status
        }
    
    def get_status(self) -> Dict[str, Any]:
        """Get robot status."""
        return {
            'robot_id': self.config.robot_id,
            'position': self._position,
            'battery': self._battery,
            'task_active': self._task_active,
            'operational_status': self._operational_status
        }

class SimpleMockJobsQueue:
    """Simplified mock jobs queue for debugging."""
    
    def __init__(self):
        self.tasks = [
            Task(task_id="task_1", task_type=TaskType.MOVE_TO_POSITION, target_position=(5.0, 5.0), priority=1),
            Task(task_id="task_2", task_type=TaskType.MOVE_TO_POSITION, target_position=(10.0, 10.0), priority=1),
        ]
    
    def get_pending_tasks(self) -> List[Task]:
        """Get pending tasks."""
        return self.tasks.copy()

class SimpleMockConfigProvider:
    """Simplified mock configuration provider for debugging."""
    
    def __init__(self):
        self.system_config = SystemConfig(
            log_level="INFO",
            log_file=None,
            log_format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            cache_ttl=300.0,
            connection_pool_size=10,
            thread_pool_size=2,  # Reduced for debugging
            health_check_interval=1.0,
            metrics_collection_interval=60.0,
            alert_thresholds={"battery_low": 0.2, "task_timeout": 300.0, "error_rate": 0.1}
        )
        self.bid_config = BidConfig(
            max_parallel_workers=2,  # Reduced for debugging
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
            calculation_timeout=2.0,  # Increased for debugging
            max_distance_normalization=20.0,
            enable_parallel_calculation=True,
            enable_calculation_statistics=True,
            enable_factor_breakdown=True
        )
    
    def get_system_config(self) -> SystemConfig:
        return self.system_config
    
    def get_bid_config(self) -> BidConfig:
        return self.bid_config

def test_single_round():
    """Test a single bidding round with debug output."""
    print("[DEBUG] Starting robot controller debug test")
    
    # Create components
    jobs_queue = SimpleMockJobsQueue()
    config_provider = SimpleMockConfigProvider()
    robots = [
        SimpleMockRobot("robot_1"),
        SimpleMockRobot("robot_2"),
    ]
    
    print(f"[DEBUG] Created {len(robots)} robots and {len(jobs_queue.tasks)} tasks")
    
    # Create controller
    controller = RobotController(
        jobs_queue=jobs_queue,
        robot_pool=robots,
        config_provider=config_provider
    )
    
    print(f"[DEBUG] Controller created with {controller.max_parallel_workers} workers")
    print(f"[DEBUG] Bidding timeout: {controller.bidding_timeout}s")
    print(f"[DEBUG] Round timeout: {controller.round_timeout}s")
    
    # Test single round
    print("[DEBUG] Starting single round processing...")
    start_time = time.time()
    
    try:
        result = controller.process_single_round()
        end_time = time.time()
        
        if result:
            print(f"[DEBUG] Round completed in {end_time - start_time:.3f}s")
            print(f"[DEBUG] Successful robots: {result.successful_robots}")
            print(f"[DEBUG] Failed robots: {result.failed_robots}")
            print(f"[DEBUG] Total bids: {len(result.all_bids)}")
            print(f"[DEBUG] Winning assignments: {len(result.winning_assignments)}")
        else:
            print("[DEBUG] Round returned None")
            
    except Exception as e:
        print(f"[DEBUG] Exception during round processing: {e}")
        import traceback
        traceback.print_exc()
    
    print("[DEBUG] Test completed")

if __name__ == "__main__":
    test_single_round() 