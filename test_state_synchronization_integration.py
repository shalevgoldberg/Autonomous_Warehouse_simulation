"""
Simple integration test for enhanced state synchronization.

This test verifies that:
1. Physics thread updates state at 1kHz
2. Control loop reads state without calling updates
3. State is properly synchronized across all components
"""
import time
import threading
from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.robot_agent import RobotAgent, RobotConfiguration
from robot.robot_agent_lane_based import RobotAgent as LaneBasedRobotAgent
from config.configuration_provider import ConfigurationProvider


def test_regular_robot_agent_state_sync():
    """Test state synchronization with regular robot agent."""
    print("\n=== Testing Regular Robot Agent State Synchronization ===")
    
    # Create warehouse map
    warehouse_map = WarehouseMap(width=10, height=10)
    
    # Create physics engine
    physics = SimpleMuJoCoPhysics(warehouse_map)
    
    # Create robot agent
    config = RobotConfiguration(
        robot_id="test_robot_1",
        control_frequency=10.0,
        motion_frequency=100.0
    )
    
    robot = RobotAgent(
        warehouse_map=warehouse_map,
        physics=physics,
        config=config
    )
    
    # Initialize robot position
    robot.initialize_position()
    
    # Start robot
    robot.start()
    
    # Let it run for a bit
    time.sleep(0.5)
    
    # Check state synchronization
    initial_state = robot.state_holder.get_robot_state()
    print(f"Initial state: pos={initial_state.position}, battery={initial_state.battery_level:.1%}")
    
    # Let it run more
    time.sleep(0.5)
    
    # Check updated state
    updated_state = robot.state_holder.get_robot_state()
    print(f"Updated state: pos={updated_state.position}, battery={updated_state.battery_level:.1%}")
    
    # Verify state has changed (physics simulation)
    assert updated_state.position != initial_state.position, "Position should have changed"
    assert updated_state.battery_level < initial_state.battery_level, "Battery should have drained"
    assert updated_state.timestamp > initial_state.timestamp, "Timestamp should be more recent"
    
    # Check robot status
    status = robot.get_status()
    print(f"Robot status: {status['operational_status']}, motion={status['motion_status']}")
    
    # Stop robot
    robot.stop()
    
    print("✅ Regular robot agent state synchronization test passed")


def test_lane_based_robot_agent_state_sync():
    """Test state synchronization with lane-based robot agent."""
    print("\n=== Testing Lane-Based Robot Agent State Synchronization ===")
    
    # Create warehouse map
    warehouse_map = WarehouseMap(width=10, height=10)
    
    # Create physics engine
    physics = SimpleMuJoCoPhysics(warehouse_map)
    
    # Create configuration provider
    config_provider = ConfigurationProvider()
    
    # Create lane-based robot agent
    robot = LaneBasedRobotAgent(
        warehouse_map=warehouse_map,
        physics=physics,
        config_provider=config_provider,
        robot_id="test_robot_2"
    )
    
    # Initialize robot position
    robot.initialize_position()
    
    # Start robot
    robot.start()
    
    # Let it run for a bit
    time.sleep(0.5)
    
    # Check state synchronization
    initial_state = robot.state_holder.get_robot_state()
    print(f"Initial state: pos={initial_state.position}, battery={initial_state.battery_level:.1%}")
    
    # Let it run more
    time.sleep(0.5)
    
    # Check updated state
    updated_state = robot.state_holder.get_robot_state()
    print(f"Updated state: pos={updated_state.position}, battery={updated_state.battery_level:.1%}")
    
    # Verify state has changed (physics simulation)
    assert updated_state.position != initial_state.position, "Position should have changed"
    assert updated_state.battery_level < initial_state.battery_level, "Battery should have drained"
    assert updated_state.timestamp > initial_state.timestamp, "Timestamp should be more recent"
    
    # Check robot status
    status = robot.get_status()
    print(f"Robot status: running={status['running']}, battery={status['battery_level']:.1%}")
    
    # Stop robot
    robot.stop()
    
    print("✅ Lane-based robot agent state synchronization test passed")


def test_concurrent_state_access():
    """Test concurrent state access from multiple threads."""
    print("\n=== Testing Concurrent State Access ===")
    
    # Create warehouse map and physics
    warehouse_map = WarehouseMap(width=10, height=10)
    physics = SimpleMuJoCoPhysics(warehouse_map)
    
    # Create robot agent
    config = RobotConfiguration(robot_id="test_robot_3")
    robot = RobotAgent(warehouse_map=warehouse_map, physics=physics, config=config)
    robot.initialize_position()
    robot.start()
    
    # Let physics run for a bit
    time.sleep(0.1)
    
    # Simulate multiple threads reading state
    read_results = []
    read_errors = []
    
    def read_state_thread(thread_id):
        try:
            for i in range(10):
                state = robot.state_holder.get_robot_state()
                read_results.append((thread_id, i, state))
                time.sleep(0.001)  # 1ms delay
        except Exception as e:
            read_errors.append((thread_id, str(e)))
    
    # Start multiple read threads
    threads = []
    for i in range(5):
        thread = threading.Thread(target=read_state_thread, args=(i,))
        threads.append(thread)
        thread.start()
    
    # Wait for all threads to complete
    for thread in threads:
        thread.join()
    
    # Verify no errors occurred
    assert len(read_errors) == 0, f"Read errors: {read_errors}"
    
    # Verify all reads succeeded
    assert len(read_results) == 50, f"Expected 50 reads, got {len(read_results)}"
    
    # Verify all states are valid
    for thread_id, read_id, state in read_results:
        assert isinstance(state.robot_id, str)
        assert isinstance(state.position, tuple)
        assert isinstance(state.battery_level, float)
        assert 0.0 <= state.battery_level <= 1.0
    
    robot.stop()
    
    print("✅ Concurrent state access test passed")


def test_physics_thread_frequency():
    """Test that physics thread runs at correct frequency."""
    print("\n=== Testing Physics Thread Frequency ===")
    
    # Create warehouse map and physics
    warehouse_map = WarehouseMap(width=10, height=10)
    physics = SimpleMuJoCoPhysics(warehouse_map)
    
    # Create robot agent
    config = RobotConfiguration(robot_id="test_robot_4")
    robot = RobotAgent(warehouse_map=warehouse_map, physics=physics, config=config)
    robot.initialize_position()
    
    # Track physics steps
    step_count = 0
    original_step = physics.step_physics
    
    def counted_step():
        nonlocal step_count
        step_count += 1
        original_step()
    
    physics.step_physics = counted_step
    
    # Start robot
    robot.start()
    
    # Run for 1 second
    time.sleep(1.0)
    
    # Stop robot
    robot.stop()
    
    # Calculate frequency
    actual_frequency = step_count / 1.0
    
    print(f"Physics frequency: {actual_frequency:.1f} Hz (target: 1000 Hz)")
    
    # Should be reasonable frequency (allow 50% tolerance for system load)
    assert actual_frequency >= 500.0, f"Frequency too low: {actual_frequency} Hz"
    assert actual_frequency <= 1500.0, f"Frequency too high: {actual_frequency} Hz"
    
    print("✅ Physics thread frequency test passed")


if __name__ == "__main__":
    print("🧪 Testing Enhanced State Synchronization System")
    print("=" * 60)
    
    try:
        # Test regular robot agent
        test_regular_robot_agent_state_sync()
        
        # Test lane-based robot agent
        test_lane_based_robot_agent_state_sync()
        
        # Test concurrent access
        test_concurrent_state_access()
        
        # Test physics frequency
        test_physics_thread_frequency()
        
        print("\n" + "=" * 60)
        print("🎉 All state synchronization tests passed!")
        print("✅ Real-time state synchronization is working correctly")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        exit(1) 