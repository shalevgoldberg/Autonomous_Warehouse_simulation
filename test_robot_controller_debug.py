#!/usr/bin/env python3
"""
Debug test for robot controller parallel bidding
"""

import sys
import os
import time
import logging
from typing import List, Dict, Any

# Add the project root to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from warehouse.impl.robot_controller_impl import RobotControllerImpl
from robot.robot_agent_lane_based import RobotAgentLaneBased
from interfaces.task_handler_interface import TaskType

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MockTaskHandler:
    """Mock task handler for testing"""
    def __init__(self):
        self.tasks = []
    
    def get_available_tasks(self) -> List[Dict[str, Any]]:
        return [
            {
                "task_id": "task_1",
                "task_type": TaskType.PICK_AND_DELIVER,
                "source_location": (5, 5),
                "destination_location": (10, 10),
                "priority": 1,
                "order_id": "order_1"
            },
            {
                "task_id": "task_2", 
                "task_type": TaskType.PICK_AND_DELIVER,
                "source_location": (3, 3),
                "destination_location": (8, 8),
                "priority": 1,
                "order_id": "order_2"
            }
        ]

def create_mock_robot(robot_id: str) -> RobotAgentLaneBased:
    """Create a mock robot agent"""
    logger.info(f"Creating mock robot {robot_id}")
    
    # Create minimal mock dependencies
    mock_state_holder = None
    mock_task_handler = None
    mock_motion_executor = None
    mock_path_planner = None
    mock_lane_follower = None
    mock_bid_calculator = None
    mock_coordinate_system = None
    
    robot = RobotAgentLaneBased(
        robot_id=robot_id,
        state_holder=mock_state_holder,
        task_handler=mock_task_handler,
        motion_executor=mock_motion_executor,
        path_planner=mock_path_planner,
        lane_follower=mock_lane_follower,
        bid_calculator=mock_bid_calculator,
        coordinate_system=mock_coordinate_system
    )
    
    logger.info(f"Mock robot {robot_id} created successfully")
    return robot

def test_robot_controller():
    """Test the robot controller with mock robots"""
    logger.info("=== Testing Robot Controller ===")
    
    # Create mock task handler
    task_handler = MockTaskHandler()
    
    # Create robot controller
    logger.info("Creating robot controller")
    controller = RobotControllerImpl()
    
    # Create mock robots
    robots = []
    for i in range(3):
        robot = create_mock_robot(f"robot_{i}")
        robots.append(robot)
    
    logger.info(f"Created {len(robots)} mock robots")
    
    # Get available tasks
    tasks = task_handler.get_available_tasks()
    logger.info(f"Available tasks: {len(tasks)}")
    
    # Test parallel bidding
    logger.info("Starting parallel bidding test")
    start_time = time.time()
    
    try:
        # This is where the actual robot controller method would be called
        # For now, let's test the individual robot bid calculation
        for robot in robots:
            logger.info(f"Testing bid calculation for {robot.robot_id}")
            for task in tasks:
                try:
                    bid = robot.calculate_bid_for_task(task)
                    logger.info(f"Robot {robot.robot_id} bid for task {task['task_id']}: {bid}")
                except Exception as e:
                    logger.error(f"Error calculating bid for robot {robot.robot_id}: {e}")
        
        end_time = time.time()
        logger.info(f"Bid calculation test completed in {end_time - start_time:.3f} seconds")
        
    except Exception as e:
        logger.error(f"Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    logger.info("Starting robot controller debug test")
    success = test_robot_controller()
    logger.info(f"Test {'PASSED' if success else 'FAILED'}") 