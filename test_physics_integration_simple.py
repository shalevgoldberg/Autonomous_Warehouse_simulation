#!/usr/bin/env python3
"""
Simple test to verify PhysicsThreadManager integration in the main flow.
"""
import time
from warehouse.map import WarehouseMap
from simulation.mujoco_env import SimpleMuJoCoPhysics
from robot.impl.physics_integration import create_physics_thread_manager

def test_physics_integration():
    """Test that PhysicsThreadManager integrates correctly with the main flow."""
    print("🧪 Testing PhysicsThreadManager Integration")
    
    # Create warehouse and physics
    warehouse_map = WarehouseMap(width=10, height=10)
    physics = SimpleMuJoCoPhysics(warehouse_map)
    
    # Create physics thread manager
    physics_manager = create_physics_thread_manager(physics, frequency_hz=1000.0)
    
    print(f"✅ Physics manager created: {type(physics_manager).__name__}")
    
    # Test start/stop
    assert not physics_manager.is_running()
    print("✅ Initial state: not running")
    
    # Start physics
    physics_manager.start()
    time.sleep(0.1)  # Let it run for 100ms
    assert physics_manager.is_running()
    print("✅ Physics started successfully")
    
    # Check that physics is actually running
    initial_state = physics.get_physics_state()
    time.sleep(0.1)  # Let it run for another 100ms
    final_state = physics.get_physics_state()
    
    # Timestamp should have updated
    assert final_state.timestamp > initial_state.timestamp
    print("✅ Physics simulation is running (timestamp updated)")
    
    # Stop physics
    physics_manager.stop()
    time.sleep(0.1)  # Let it stop
    assert not physics_manager.is_running()
    print("✅ Physics stopped successfully")
    
    print("🎉 PhysicsThreadManager integration test PASSED!")

if __name__ == "__main__":
    test_physics_integration() 