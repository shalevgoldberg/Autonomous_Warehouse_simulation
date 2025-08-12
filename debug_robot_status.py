#!/usr/bin/env python3
"""
Debug script to test robot status summary functionality.
"""
import time
import threading
from unittest.mock import Mock
from typing import List, Dict, Any

from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.task_handler_interface import Task, TaskType
from interfaces.configuration_interface import IConfigurationProvider, SystemConfig, BidConfig
from warehouse.impl.robot_controller_impl import RobotController

class SimpleMockRobot:
    """Simplified mock robot for debugging status."""
    
    def __init__(self, robot_id: str):
        self.config = Mock()
        self.config.robot_id = robot_id
        self._position = (0.0, 0.0)
        self._battery = 100.0
        self._task_active = False
        self._operational_status = 'idle'
        self._lock = threading.Lock()
    
    def get_status(self) -> Dict[str, Any]:
        """Get robot status."""
        print(f"[DEBUG] Robot {self.config.robot_id} get_status() called")
        with self._lock:
            return {
                'robot_id': self.config.robot_id,
                'position': self._position,
                'battery': self._battery,
                'task_active': self._task_active,
                'operational_status': self._operational_status
            }
    
    def get_bid_calculation_statistics(self) -> Dict[str, Any]:
        """Get bid calculation statistics."""
        print(f"[DEBUG] Robot {self.config.robot_id} get_bid_calculation_statistics() called")
        with self._lock:
            return {
                'robot_id': self.config.robot_id,
                'available_for_bidding': self.is_available_for_bidding(),
                'current_position': self._position,
                'battery_level': self._battery,
                'operational_status': self._operational_status
            }
    
    def is_available_for_bidding(self) -> bool:
        """Check if robot is available for bidding."""
        print(f"[DEBUG] Robot {self.config.robot_id} is_available_for_bidding() called")
        with self._lock:
            return (not self._task_active and 
                   self._operational_status == 'idle' and 
                   self._battery > 20.0)

class SimpleMockJobsQueue:
    """Simplified mock jobs queue for debugging."""
    
    def __init__(self):
        self.tasks = []
    
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
            thread_pool_size=2,
            health_check_interval=1.0,
            metrics_collection_interval=60.0,
            alert_thresholds={"battery_low": 0.2, "task_timeout": 300.0, "error_rate": 0.1}
        )
        self.bid_config = BidConfig(
            max_parallel_workers=2,
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
            calculation_timeout=2.0,
            max_distance_normalization=20.0,
            enable_parallel_calculation=True,
            enable_calculation_statistics=True,
            enable_factor_breakdown=True
        )
    
    def get_system_config(self) -> SystemConfig:
        return self.system_config
    
    def get_bid_config(self) -> BidConfig:
        return self.bid_config

def test_robot_status_summary():
    """Test robot status summary functionality."""
    print("[DEBUG] Starting robot status summary debug test")
    
    # Create components
    jobs_queue = SimpleMockJobsQueue()
    config_provider = SimpleMockConfigProvider()
    robots = [
        SimpleMockRobot("robot_1"),
        SimpleMockRobot("robot_2"),
        SimpleMockRobot("robot_3"),
    ]
    
    print(f"[DEBUG] Created {len(robots)} robots")
    
    # Create controller
    controller = RobotController(
        jobs_queue=jobs_queue,
        robot_pool=robots,
        config_provider=config_provider
    )
    
    print(f"[DEBUG] Controller created")
    
    # Test robot status summary
    print("[DEBUG] Calling get_robot_status_summary()...")
    start_time = time.time()
    
    try:
        status_summary = controller.get_robot_status_summary()
        end_time = time.time()
        
        print(f"[DEBUG] Status summary completed in {end_time - start_time:.3f}s")
        print(f"[DEBUG] Got {len(status_summary)} robot statuses")
        
        for i, robot_status in enumerate(status_summary):
            print(f"[DEBUG] Robot {i+1}: {robot_status['robot_id']}")
            print(f"[DEBUG]   Available: {robot_status['available_for_bidding']}")
            print(f"[DEBUG]   Status keys: {list(robot_status['status'].keys())}")
            print(f"[DEBUG]   Bid stats keys: {list(robot_status['bid_calculation_stats'].keys())}")
        
    except Exception as e:
        print(f"[DEBUG] Exception during status summary: {e}")
        import traceback
        traceback.print_exc()
    
    print("[DEBUG] Test completed")

if __name__ == "__main__":
    test_robot_status_summary() 