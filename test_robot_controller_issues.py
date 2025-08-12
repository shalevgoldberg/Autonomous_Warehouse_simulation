#!/usr/bin/env python3
"""
Comprehensive test to check if the robot controller issues from the review are still actual.
"""
import time
import threading
import gc
import psutil
import os
from unittest.mock import Mock
from typing import List, Dict, Any

from interfaces.jobs_queue_interface import IJobsQueue
from interfaces.task_handler_interface import Task, TaskType
from interfaces.configuration_interface import IConfigurationProvider, SystemConfig, BidConfig
from warehouse.impl.robot_controller_impl import RobotController

class TestRobot:
    """Test robot for checking timeout and performance issues."""
    
    def __init__(self, robot_id: str, delay: float = 0.01):
        self.config = Mock()
        self.config.robot_id = robot_id
        self._position = (0.0, 0.0)
        self._battery = 100.0
        self._task_active = False
        self._operational_status = 'idle'
        self._delay = delay
    
    def calculate_bids(self, tasks: List[Task]) -> List[Any]:
        """Calculate bids with configurable delay."""
        time.sleep(self._delay)  # Simulate processing time
        
        from interfaces.bidding_system_interface import RobotBid
        bids = []
        for task in tasks:
            bid = RobotBid(
                robot_id=self.config.robot_id,
                task=task,
                bid_value=10.0,
                bid_metadata={"test": True}
            )
            bids.append(bid)
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
    
    def assign_task(self, task: Task) -> bool:
        """Assign a task to the robot."""
        if self._task_active:
            return False
        self._task_active = True
        return True

class TestJobsQueue:
    """Test jobs queue."""
    
    def __init__(self):
        self.tasks = [
            Task(task_id="task_1", task_type=TaskType.MOVE_TO_POSITION, target_position=(5.0, 5.0), priority=1),
            Task(task_id="task_2", task_type=TaskType.MOVE_TO_POSITION, target_position=(10.0, 10.0), priority=1),
            Task(task_id="task_3", task_type=TaskType.PICK_AND_DELIVER, shelf_id="shelf_1", item_id="item_1", order_id="order_1", priority=2),
        ]
    
    def get_pending_tasks(self) -> List[Task]:
        """Get pending tasks."""
        return self.tasks.copy()

class TestConfigProvider:
    """Test configuration provider with different timeout scenarios."""
    
    def __init__(self, calculation_timeout: float = 1.0, max_workers: int = 4):
        self.system_config = SystemConfig(
            log_level="INFO",
            log_file=None,
            log_format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            cache_ttl=300.0,
            connection_pool_size=10,
            thread_pool_size=max_workers,
            health_check_interval=1.0,
            metrics_collection_interval=60.0,
            alert_thresholds={"battery_low": 0.2, "task_timeout": 300.0, "error_rate": 0.1}
        )
        self.bid_config = BidConfig(
            max_parallel_workers=max_workers,
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
            calculation_timeout=calculation_timeout,
            max_distance_normalization=20.0,
            enable_parallel_calculation=True,
            enable_calculation_statistics=True,
            enable_factor_breakdown=True
        )
    
    def get_system_config(self) -> SystemConfig:
        return self.system_config
    
    def get_bid_config(self) -> BidConfig:
        return self.bid_config

def get_thread_count():
    """Get current thread count for the process."""
    return threading.active_count()

def get_memory_usage():
    """Get current memory usage."""
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / 1024 / 1024  # MB

def test_timeout_configuration_conflicts():
    """Test 1: Check for timeout configuration conflicts."""
    print("\n=== TEST 1: Timeout Configuration Conflicts ===")
    
    # Test different timeout configurations
    test_cases = [
        (0.5, "Very short timeout"),
        (1.0, "Short timeout"),
        (2.0, "Medium timeout"),
        (5.0, "Long timeout"),
    ]
    
    for timeout, description in test_cases:
        print(f"\nTesting {description} ({timeout}s):")
        
        config_provider = TestConfigProvider(calculation_timeout=timeout)
        jobs_queue = TestJobsQueue()
        robots = [TestRobot(f"robot_{i}", delay=0.01) for i in range(3)]
        
        controller = RobotController(
            jobs_queue=jobs_queue,
            robot_pool=robots,
            config_provider=config_provider
        )
        
        print(f"  - Controller bidding_timeout: {controller.bidding_timeout}s")
        print(f"  - Controller round_timeout: {controller.round_timeout}s")
        print(f"  - Config calculation_timeout: {config_provider.bid_config.calculation_timeout}s")
        
        # Check for conflicts
        if controller.bidding_timeout != config_provider.bid_config.calculation_timeout:
            print(f"  ❌ CONFLICT: Controller timeout ({controller.bidding_timeout}s) != Config timeout ({config_provider.bid_config.calculation_timeout}s)")
        else:
            print(f"  ✅ No conflict: Timeouts match")
        
        # Test round processing
        start_time = time.time()
        result = controller.process_single_round()
        end_time = time.time()
        
        if result:
            print(f"  ✅ Round completed in {end_time - start_time:.3f}s")
            print(f"  ✅ Successful robots: {result.successful_robots}")
        else:
            print(f"  ❌ Round failed or timed out")

def test_thread_pool_management():
    """Test 2: Check for thread pool management issues and resource leaks."""
    print("\n=== TEST 2: Thread Pool Management ===")
    
    initial_threads = get_thread_count()
    initial_memory = get_memory_usage()
    
    print(f"Initial threads: {initial_threads}")
    print(f"Initial memory: {initial_memory:.2f} MB")
    
    # Create multiple controllers and process rounds
    controllers = []
    for i in range(5):
        config_provider = TestConfigProvider(calculation_timeout=1.0, max_workers=2)
        jobs_queue = TestJobsQueue()
        robots = [TestRobot(f"robot_{j}", delay=0.01) for j in range(3)]
        
        controller = RobotController(
            jobs_queue=jobs_queue,
            robot_pool=robots,
            config_provider=config_provider
        )
        controllers.append(controller)
        
        # Process multiple rounds
        for round_num in range(3):
            result = controller.process_single_round()
            if result:
                print(f"Controller {i}, Round {round_num + 1}: {result.successful_robots} robots successful")
        
        # Reset robots for next round
        for robot in robots:
            robot._task_active = False
    
    # Check for resource leaks
    mid_threads = get_thread_count()
    mid_memory = get_memory_usage()
    
    print(f"After processing: {mid_threads} threads, {mid_memory:.2f} MB")
    
    # Clean up controllers
    for controller in controllers:
        controller.stop()
    
    # Force garbage collection
    gc.collect()
    time.sleep(1)  # Allow time for cleanup
    
    final_threads = get_thread_count()
    final_memory = get_memory_usage()
    
    print(f"After cleanup: {final_threads} threads, {final_memory:.2f} MB")
    
    # Check for leaks
    thread_leak = final_threads - initial_threads
    memory_leak = final_memory - initial_memory
    
    if thread_leak > 5:  # Allow some variance
        print(f"❌ THREAD LEAK: {thread_leak} extra threads")
    else:
        print(f"✅ No significant thread leak")
    
    if memory_leak > 10:  # Allow 10MB variance
        print(f"❌ MEMORY LEAK: {memory_leak:.2f} MB")
    else:
        print(f"✅ No significant memory leak")

def test_test_reliability():
    """Test 3: Check for test reliability issues (hanging, timeouts)."""
    print("\n=== TEST 3: Test Reliability ===")
    
    # Test with different robot delays to simulate performance issues
    test_cases = [
        (0.001, "Very fast robots"),
        (0.01, "Fast robots"),
        (0.1, "Slow robots"),
        (0.5, "Very slow robots"),
    ]
    
    for delay, description in test_cases:
        print(f"\nTesting {description} (delay={delay}s):")
        
        config_provider = TestConfigProvider(calculation_timeout=1.0)
        jobs_queue = TestJobsQueue()
        robots = [TestRobot(f"robot_{i}", delay=delay) for i in range(3)]
        
        controller = RobotController(
            jobs_queue=jobs_queue,
            robot_pool=robots,
            config_provider=config_provider
        )
        
        # Test with timeout
        start_time = time.time()
        try:
            result = controller.process_single_round()
            end_time = time.time()
            
            if result:
                print(f"  ✅ Completed in {end_time - start_time:.3f}s")
                print(f"  ✅ Successful robots: {result.successful_robots}")
                
                # Check if it completed within reasonable time
                if end_time - start_time < 2.0:  # Should complete within 2 seconds
                    print(f"  ✅ Performance acceptable")
                else:
                    print(f"  ⚠️  Performance issue: took {end_time - start_time:.3f}s")
            else:
                print(f"  ❌ Round failed or timed out")
                
        except Exception as e:
            print(f"  ❌ Exception: {e}")
        
        controller.stop()

def test_concurrent_access():
    """Test 4: Check for concurrent access issues."""
    print("\n=== TEST 4: Concurrent Access ===")
    
    config_provider = TestConfigProvider(calculation_timeout=1.0)
    jobs_queue = TestJobsQueue()
    robots = [TestRobot(f"robot_{i}", delay=0.01) for i in range(3)]
    
    controller = RobotController(
        jobs_queue=jobs_queue,
        robot_pool=robots,
        config_provider=config_provider
    )
    
    # Test concurrent access to status methods
    def concurrent_status_access():
        for _ in range(10):
            status = controller.get_controller_status()
            robot_status = controller.get_robot_status_summary()
            time.sleep(0.001)
    
    # Start multiple threads
    threads = []
    for i in range(5):
        thread = threading.Thread(target=concurrent_status_access, name=f"StatusThread_{i}")
        threads.append(thread)
        thread.start()
    
    # Wait for all threads to complete
    for thread in threads:
        thread.join(timeout=5.0)
        if thread.is_alive():
            print(f"❌ Thread {thread.name} did not complete within timeout")
        else:
            print(f"✅ Thread {thread.name} completed successfully")
    
    controller.stop()

if __name__ == "__main__":
    print("=== ROBOT CONTROLLER ISSUES CHECK ===")
    print("Checking if the issues mentioned in the review are still actual...")
    
    test_timeout_configuration_conflicts()
    test_thread_pool_management()
    test_test_reliability()
    test_concurrent_access()
    
    print("\n=== SUMMARY ===")
    print("All tests completed. Check output above for any issues.") 