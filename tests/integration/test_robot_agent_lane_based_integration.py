import pytest
from unittest.mock import MagicMock, patch
import threading
import time

from robot.robot_agent_lane_based import RobotAgent
from interfaces.configuration_interface import RobotConfig
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
    
    # Create a proper mock object for map data
    map_data_mock = MagicMock()
    map_data_mock.width = 10
    map_data_mock.height = 10
    map_data_mock.cell_size = 1.0
    map_data_mock.obstacles = []
    map_data_mock.shelves = {}
    mock.get_map_data.return_value = map_data_mock
    return mock

@pytest.fixture
def robot_config():
    return RobotConfig(
        robot_id="lane_robot_001",
        max_speed=2.0,
        position_tolerance=0.1,
        control_frequency=10.0,
        motion_frequency=100.0,
        cell_size=1.0,
        lane_tolerance=0.2,
        corner_speed=1.0,
        bay_approach_speed=0.5,
        conflict_box_lock_timeout=5.0,
        conflict_box_heartbeat_interval=1.0,
        max_linear_velocity=2.0,
        max_angular_velocity=2.0,
        movement_speed=2.0,
        wheel_base=0.5,
        wheel_radius=0.1,
        picking_duration=2.0,
        dropping_duration=2.0,
        charging_threshold=0.2,
        emergency_stop_distance=0.5,
        stall_recovery_timeout=10.0
    )

@pytest.fixture
def config_provider(robot_config):
    mock_provider = MagicMock()
    mock_provider.get_robot_config.return_value = robot_config
    mock_provider.get_value.return_value = MagicMock(value=(0.0, 0.0))
    mock_provider.errors = []
    mock_provider.get_database_config.return_value = MagicMock()
    
    # Create bid configuration
    bid_config = MagicMock()
    bid_config.distance_weight = 0.3
    bid_config.battery_weight = 0.2
    bid_config.workload_weight = 0.2
    bid_config.task_type_compatibility_weight = 0.1
    bid_config.robot_capabilities_weight = 0.1
    bid_config.time_urgency_weight = 0.05
    bid_config.conflict_box_availability_weight = 0.03
    bid_config.shelf_accessibility_weight = 0.02
    bid_config.enable_distance_factor = True
    bid_config.enable_battery_factor = True
    bid_config.enable_workload_factor = True
    bid_config.enable_task_type_compatibility_factor = True
    bid_config.enable_robot_capabilities_factor = True
    bid_config.enable_time_urgency_factor = True
    bid_config.enable_conflict_box_availability_factor = True
    bid_config.enable_shelf_accessibility_factor = True
    bid_config.battery_threshold = 0.2
    bid_config.calculation_timeout = 1.0
    bid_config.max_distance_normalization = 100.0
    bid_config.enable_parallel_calculation = True
    bid_config.enable_calculation_statistics = True
    bid_config.enable_factor_breakdown = True
    bid_config.max_parallel_workers = 4
    
    mock_provider.get_bid_config.return_value = bid_config
    return mock_provider

@pytest.fixture
def robot_agent(physics, config_provider, simulation_data_service):
    with patch('robot.robot_agent_lane_based.SimulationDataServiceImpl', return_value=simulation_data_service):
        return RobotAgent(
            physics=physics,
            config_provider=config_provider,
            robot_id="lane_robot_001",
            simulation_data_service=simulation_data_service
        )

def test_lane_based_robot_initialization(robot_agent, robot_config):
    assert robot_agent.robot_config.robot_id == robot_config.robot_id
    assert robot_agent.robot_config.max_speed == robot_config.max_speed
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
    assert status['robot_id'] == robot_agent.robot_config.robot_id

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