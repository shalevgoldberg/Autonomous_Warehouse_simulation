import time
import threading
import pytest
from robot.impl.physics_integration import PhysicsThreadManager, IPhysicsCommandSink

class MockPhysics(IPhysicsCommandSink):
    def __init__(self):
        self.step_count = 0
        self.error_on_step = False
        self.lock = threading.Lock()
    def set_wheel_velocities(self, left_vel, right_vel):
        pass
    def step_physics(self):
        with self.lock:
            if self.error_on_step:
                raise RuntimeError("Physics error!")
            self.step_count += 1

def test_physics_thread_manager_runs_and_stops():
    physics = MockPhysics()
    manager = PhysicsThreadManager(physics, frequency_hz=200.0)  # 200Hz for test speed
    assert not manager.is_running()
    manager.start()
    time.sleep(0.05)  # Let it run for 50ms
    manager.stop()
    assert not manager.is_running()
    # Should have run at least a few steps
    with physics.lock:
        assert physics.step_count > 0

def test_physics_thread_manager_frequency():
    physics = MockPhysics()
    manager = PhysicsThreadManager(physics, frequency_hz=100.0)  # 100Hz
    manager.start()
    time.sleep(0.12)  # Let it run for 120ms
    manager.stop()
    with physics.lock:
        # Should be close to 12 steps (allow some margin)
        assert 8 <= physics.step_count <= 20

def test_physics_thread_manager_error_handling():
    physics = MockPhysics()
    physics.error_on_step = True
    manager = PhysicsThreadManager(physics, frequency_hz=50.0)
    manager.start()
    time.sleep(0.03)
    manager.stop()
    # Should not crash or hang
    assert not manager.is_running() 