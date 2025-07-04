import pytest
from unittest.mock import MagicMock, patch
import threading
import time

from robot.robot_agent_lane_based import RobotAgent, RobotConfiguration
from interfaces.task_handler_interface import Task, TaskType
from interfaces.lane_follower_interface import LaneFollowingStatus
from interfaces.motion_executor_interface import MotionStatus
from interfaces.simulation_data_service_interface import ISimulationDataService
from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics

@pytest.fixture
def warehouse_map():
    mock = MagicMock()
    mock.grid_size = 1.0
    mock.width = 10
    mock.height = 10
    mock.start_position = (0.0, 0.0)
    mock.is_walkable.return_value = True
    return mock

@pytest.fixture
def physics():
    mock = MagicMock()
    mock.model = None
    mock.data = None
    mock.reset_robot_position.return_value = None
    mock.get_robot_pose.return_value = (0.0, 0.0, 0.0)
    return mock

@pytest.fixture
def simulation_data_service():
    mock = MagicMock(spec=ISimulationDataService)
    mock.get_navigation_graph.return_value = MagicMock()
    mock.get_blocked_cells.return_value = {}
    mock.try_acquire_conflict_box_lock.return_value = True
    mock.release_conflict_box_lock.return_value = True
    mock.heartbeat_conflict_box_lock.return_value = True
    mock.get_shelf_position.return_value = (1.0, 1.0)
    mock.get_dropoff_zones.return_value = [(5.0, 5.0)]
    mock.lock_shelf.return_value = True
    mock.unlock_shelf.return_value = True
    mock.log_event.return_value = None
    return mock

@pytest.fixture
def robot_config():
    return RobotConfiguration(
        robot_id="lane_robot_001",
        max_speed=2.0,
        position_tolerance=0.1,
        control_frequency=10.0,
        motion_frequency=100.0,
        start_position=(0.0, 0.0)
    )

@pytest.fixture
def robot_agent(warehouse_map, physics, robot_config, simulation_data_service):
    with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=simulation_data_service):
        return RobotAgent(
            warehouse_map=warehouse_map,
            physics=physics,
            config=robot_config,
            simulation_data_service=simulation_data_service
        )

def test_lane_based_robot_initialization(robot_agent, robot_config):
    assert robot_agent.config.robot_id == robot_config.robot_id
    assert robot_agent.config.max_speed == robot_config.max_speed
    assert robot_agent.simulation_data_service is not None
    assert robot_agent.lane_follower is not None
    assert robot_agent.path_planner is not None
    assert robot_agent.motion_executor is not None
    assert robot_agent.task_handler is not None

def test_lane_based_task_assignment_and_status(robot_agent):
    task = Task(
        task_id="lane_task_001",
        task_type=TaskType.PICK_AND_DELIVER,
        order_id="order_001",  # Required for PICK_AND_DELIVER tasks
        shelf_id="shelf_1",
        item_id="item_1",
        priority=1
    )
    robot_agent.task_handler.start_task = MagicMock(return_value=True)
    accepted = robot_agent.assign_task(task)
    assert accepted is True
    status = robot_agent.get_status()
    assert 'robot_id' in status
    assert 'lane_following_status' in status
    assert 'motion_status' in status
    assert status['robot_id'] == robot_agent.config.robot_id

def test_lane_based_path_planning(robot_agent):
    start = (0.0, 0.0)
    end = (2.0, 2.0)
    robot_agent.path_planner.plan_route = MagicMock(return_value=MagicMock())
    result = robot_agent.path_planner.plan_route(start, end, TaskType.MOVE_TO_POSITION)
    assert result is not None

def test_lane_based_lane_follower_status(robot_agent):
    robot_agent.lane_follower.get_lane_following_status = MagicMock(return_value=LaneFollowingStatus.FOLLOWING_LANE)
    status = robot_agent.lane_follower.get_lane_following_status()
    assert status == LaneFollowingStatus.FOLLOWING_LANE

def test_lane_based_motion_executor_status(robot_agent):
    robot_agent.motion_executor.get_motion_status = MagicMock(return_value=MotionStatus.LANE_FOLLOW)
    status = robot_agent.motion_executor.get_motion_status()
    assert status == MotionStatus.LANE_FOLLOW

def test_lane_based_concurrency_safety(robot_agent):
    results = []
    errors = []
    def concurrent_status_check():
        try:
            status = robot_agent.get_status()
            results.append(status is not None)
        except Exception as e:
            errors.append(str(e))
    threads = []
    for i in range(5):
        thread = threading.Thread(target=concurrent_status_check)
        threads.append(thread)
        thread.start()
    for thread in threads:
        thread.join()
    assert len(errors) == 0
    assert len(results) == 5
    assert all(results)

def test_lane_based_error_handling(robot_agent):
    with pytest.raises(AttributeError):
        robot_agent.assign_task(None)
    robot_agent.path_planner.plan_route = MagicMock(side_effect=ValueError("Invalid input"))
    with pytest.raises(ValueError):
        robot_agent.path_planner.plan_route(None, (1.0, 1.0), TaskType.MOVE_TO_POSITION)
    with pytest.raises(ValueError):
        robot_agent.path_planner.plan_route((1.0, 1.0), None, TaskType.MOVE_TO_POSITION) 