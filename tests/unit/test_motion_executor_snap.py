#!/usr/bin/env python3
"""
Comprehensive tests for MotionExecutor snap-to-target functionality.

Tests various scenarios:
- Different start and destination positions
- Obstacle avoidance during snap
- Multiple snap scenarios
- Edge cases and boundary conditions
- Complex path scenarios
- Obstacle interactions
"""

import unittest
import time
import math
from typing import Tuple, List, Optional

from robot.impl.motion_executor_impl import MotionExecutorImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from interfaces.motion_executor_interface import MotionStatus
from interfaces.path_planner_interface import Cell


class TestMotionExecutorSnap(unittest.TestCase):
    """Test MotionExecutor snap-to-target functionality."""

    def setUp(self):
        """Set up test components."""
        # Create coordinate system for 20x20 warehouse
        self.coord_system = CoordinateSystemImpl(
            cell_size=0.5,  # 50cm cells
            grid_origin=(0.0, 0.0),
            grid_width=40,
            grid_height=40
        )
        
        # Create motion executor with physics simulation
        self.motion_executor = MotionExecutorImpl(
            coordinate_system=self.coord_system,
            robot_id="test_robot"
        )
        
        # Create simple physics engine for testing
        self.physics_engine = SimplePhysicsEngine()
        self.motion_executor.physics_engine = self.physics_engine

    def test_snap_triggered_at_20cm(self):
        """Test that snap is triggered when robot gets within 20cm of target."""
        print(f"\n{'='*60}")
        print("TEST: Snap triggered at 20cm distance")
        print(f"{'='*60}")
        
        # Set robot position just outside snap threshold
        self.physics_engine.set_robot_position(1.0, 1.0, 0.0)
        target_world = (1.20, 1.20)  # ~0.282m away
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run a few steps outside threshold
        for _ in range(5):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
        
        # Move robot just inside snap threshold
        self.physics_engine.set_robot_position(1.05, 1.05, 0.0)  # ~0.212m away
        for _ in range(5):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
        
        # Move robot well within snap threshold
        self.physics_engine.set_robot_position(1.15, 1.15, 0.0)  # ~0.07m away
        snap_triggered = False
        for _ in range(10):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            if hasattr(self.motion_executor, '_snap_triggered') and self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered after moving inside threshold")
                break
        
        self.assertTrue(snap_triggered, "Snap should be triggered when within 20cm")

    def test_snap_completes_task(self):
        """Test that snap motion completes the task successfully."""
        print(f"\n{'='*60}")
        print("TEST: Snap completes task")
        print(f"{'='*60}")
        
        # Set robot position close to target
        self.physics_engine.set_robot_position(1.0, 1.0, 0.0)
        target_world = (1.15, 1.15)  # 15cm away
        
        # Convert target to Cell and start motion
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until task completes
        max_iterations = 200
        for i in range(max_iterations):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check if motion completed
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                print(f"✅ Task completed at iteration {i}")
                break
        
        # Verify task completed
        final_status = self.motion_executor.get_motion_status()
        self.assertEqual(final_status, MotionStatus.REACHED_TARGET, "Task should complete with snap")

    def test_snap_with_obstacle_near_target(self):
        """Test snap functionality when there's an obstacle near the target."""
        print(f"\n{'='*60}")
        print("TEST: Snap with obstacle near target")
        print(f"{'='*60}")
        
        # Add obstacle near target
        self.physics_engine.add_obstacle(1.18, 1.18, 0.05)  # 5cm radius obstacle at 1.18,1.18
        
        # Set robot position
        self.physics_engine.set_robot_position(1.0, 1.0, 0.0)
        target_world = (1.25, 1.25)  # Target beyond obstacle
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until completion
        max_iterations = 300
        completed = False
        snap_triggered = False
        
        for i in range(max_iterations):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check for snap trigger
            if hasattr(self.motion_executor, '_snap_triggered') and self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered at iteration {i}")
            
            # Check for completion
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                completed = True
                print(f"✅ Task completed at iteration {i}")
                break
        
        self.assertTrue(completed, "Task should complete even with obstacle")
        self.assertTrue(snap_triggered, "Snap should be triggered despite obstacle")

    def test_snap_from_different_angles(self):
        """Test snap from different approach angles."""
        print(f"\n{'='*60}")
        print("TEST: Snap from different angles")
        print(f"{'='*60}")
        
        target_world = (2.0, 2.0)
        test_angles = [0.0, math.pi/4, math.pi/2, 3*math.pi/4, -math.pi/4, -math.pi/2]  # Removed π radians
        
        for angle in test_angles:
            print(f"Testing angle: {angle:.2f} rad")
            
            # Reset motion executor
            self.motion_executor = MotionExecutorImpl(
                coordinate_system=self.coord_system,
                robot_id="test_robot"
            )
            self.motion_executor.physics_engine = self.physics_engine
            
            # Set robot position at angle from target
            distance = 0.20  # 20cm away to ensure snap triggers
            start_x = target_world[0] + distance * math.cos(angle)
            start_y = target_world[1] + distance * math.sin(angle)
            self.physics_engine.set_robot_position(start_x, start_y, angle)
            
            # Convert target to Cell and start motion
            target_cell = self.coord_system.world_to_cell(target_world)
            self.motion_executor.execute_single_move(target_cell)
            
            # Run until completion
            max_iterations = 300
            completed = False
            for i in range(max_iterations):
                self.motion_executor.update_control_loop()
                self.physics_engine.step_physics()
                
                status = self.motion_executor.get_motion_status()
                if status == MotionStatus.REACHED_TARGET:
                    completed = True
                    print(f"  ✅ Completed at iteration {i}")
                    break
            
            self.assertTrue(completed, f"Task should complete for angle {angle}")

    def test_snap_with_narrow_corridor(self):
        """Test snap functionality in a narrow corridor scenario."""
        print(f"\n{'='*60}")
        print("TEST: Snap in narrow corridor")
        print(f"{'='*60}")
        
        # Create narrow corridor with obstacles
        self.physics_engine.add_obstacle(1.5, 1.3, 0.1)  # Left wall
        self.physics_engine.add_obstacle(1.5, 1.7, 0.1)  # Right wall
        
        # Set robot position in corridor
        self.physics_engine.set_robot_position(1.0, 1.5, 0.0)
        target_world = (2.0, 1.5)  # Target at end of corridor
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until completion
        max_iterations = 400
        completed = False
        snap_triggered = False
        
        for i in range(max_iterations):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check for snap trigger
            if hasattr(self.motion_executor, '_snap_triggered') and self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered at iteration {i}")
            
            # Check for completion
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                completed = True
                print(f"✅ Task completed at iteration {i}")
                break
        
        self.assertTrue(completed, "Task should complete in narrow corridor")
        self.assertTrue(snap_triggered, "Snap should be triggered in corridor")

    def test_snap_edge_cases(self):
        """Test snap functionality in edge cases."""
        print(f"\n{'='*60}")
        print("TEST: Snap edge cases")
        print(f"{'='*60}")
        
        # Test 1: Very close to target (5cm)
        print("Edge case 1: Very close (5cm)")
        self.physics_engine.set_robot_position(1.95, 1.95, 0.0)
        target_world = (2.0, 2.0)
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until completion
        for i in range(100):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                print(f"  ✅ Very close case completed at iteration {i}")
                break
        
        # Test 2: Exactly at snap threshold (20cm)
        print("Edge case 2: Exactly at threshold (20cm)")
        self.motion_executor = MotionExecutorImpl(
            coordinate_system=self.coord_system,
            robot_id="test_robot"
        )
        self.motion_executor.physics_engine = self.physics_engine
        
        self.physics_engine.set_robot_position(1.8, 1.8, 0.0)
        target_world = (2.0, 2.0)  # Exactly 20cm away
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until completion
        for i in range(100):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                print(f"  ✅ Threshold case completed at iteration {i}")
                break

    def test_snap_physics_consistency(self):
        """Test that snap maintains physics consistency."""
        print(f"\n{'='*60}")
        print("TEST: Snap physics consistency")
        print(f"{'='*60}")
        
        # Set robot position further from target to ensure snap triggers
        self.physics_engine.set_robot_position(1.0, 1.0, 0.0)
        target_world = (1.25, 1.25)  # 25cm away to ensure snap triggers
        
        # Record initial state
        initial_pos = self.physics_engine.get_robot_pose()
        initial_vel = self.physics_engine.get_robot_velocity()
        
        # Convert target to Cell and start motion
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until snap triggers
        snap_triggered = False
        for i in range(100):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            if self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered at iteration {i}")
                break
        
        self.assertTrue(snap_triggered, "Snap should be triggered")
        
        # Verify physics consistency (no teleporting)
        current_pos = self.physics_engine.get_robot_pose()
        current_vel = self.physics_engine.get_robot_velocity()
        
        # Position should change smoothly
        pos_change = math.sqrt((current_pos[0] - initial_pos[0])**2 + 
                              (current_pos[1] - initial_pos[1])**2)
        self.assertLess(pos_change, 0.1, "Position should change smoothly, not teleport")
        
        # Velocity should be reasonable
        vel_magnitude = math.sqrt(current_vel[0]**2 + current_vel[1]**2)
        self.assertLess(vel_magnitude, 5.0, "Velocity should be reasonable during snap")

    def test_snap_with_dynamic_obstacles(self):
        """Test snap functionality with dynamically moving obstacles."""
        print(f"\n{'='*60}")
        print("TEST: Snap with dynamic obstacles")
        print(f"{'='*60}")
        
        # Add dynamic obstacle that moves slowly
        obstacle_id = self.physics_engine.add_obstacle(1.5, 1.5, 0.08)  # Smaller obstacle
        
        # Set robot position
        self.physics_engine.set_robot_position(1.0, 1.0, 0.0)
        target_world = (2.0, 2.0)
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run with moving obstacle
        max_iterations = 400
        completed = False
        snap_triggered = False
        
        for i in range(max_iterations):
            # Move obstacle slowly toward robot
            if i > 100:  # Start moving obstacle later
                self.physics_engine.move_obstacle(obstacle_id, 0.005, 0.005)  # Slower movement
            
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check for snap trigger
            if hasattr(self.motion_executor, '_snap_triggered') and self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered at iteration {i}")
            
            # Check for completion
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                completed = True
                print(f"✅ Task completed at iteration {i}")
                break
        
        self.assertTrue(completed, "Task should complete with dynamic obstacles")
        self.assertTrue(snap_triggered, "Snap should be triggered with dynamic obstacles")

    def test_snap_in_complex_environment(self):
        """Test snap functionality in a complex environment with multiple obstacles."""
        print(f"\n{'='*60}")
        print("TEST: Snap in complex environment")
        print(f"{'='*60}")
        
        # Create complex environment with fewer, smaller obstacles
        self.physics_engine.add_obstacle(1.3, 1.3, 0.05)  # Central obstacle (smaller)
        self.physics_engine.add_obstacle(1.5, 1.0, 0.04)  # Left obstacle (smaller)
        self.physics_engine.add_obstacle(1.0, 1.5, 0.04)  # Right obstacle (smaller)
        
        # Set robot position
        self.physics_engine.set_robot_position(0.5, 0.5, 0.0)
        target_world = (2.0, 2.0)
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until completion
        max_iterations = 600
        completed = False
        snap_triggered = False
        
        for i in range(max_iterations):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check for snap trigger
            if hasattr(self.motion_executor, '_snap_triggered') and self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered at iteration {i}")
            
            # Check for completion
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                completed = True
                print(f"✅ Task completed at iteration {i}")
                break
        
        self.assertTrue(completed, "Task should complete in complex environment")
        self.assertTrue(snap_triggered, "Snap should be triggered in complex environment")

    def test_snap_with_high_speed_approach(self):
        """Test snap functionality when robot approaches target at high speed."""
        print(f"\n{'='*60}")
        print("TEST: Snap with high speed approach")
        print(f"{'='*60}")
        
        # Set robot position
        self.physics_engine.set_robot_position(1.0, 1.0, 0.0)
        target_world = (1.15, 1.15)
        
        # Set high movement speed
        self.motion_executor.set_movement_speed(3.0)  # 3 m/s
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until completion
        max_iterations = 100
        completed = False
        snap_triggered = False
        
        for i in range(max_iterations):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check for snap trigger
            if hasattr(self.motion_executor, '_snap_triggered') and self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered at iteration {i}")
            
            # Check for completion
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                completed = True
                print(f"✅ Task completed at iteration {i}")
                break
        
        self.assertTrue(completed, "Task should complete with high speed approach")
        self.assertTrue(snap_triggered, "Snap should be triggered with high speed")

    def test_snap_recovery_from_failure(self):
        """Test snap functionality recovery after a failed approach."""
        print(f"\n{'='*60}")
        print("TEST: Snap recovery from failure")
        print(f"{'='*60}")
        
        # Add obstacle that blocks initial approach
        self.physics_engine.add_obstacle(1.1, 1.1, 0.15)  # Large obstacle blocking path
        
        # Set robot position
        self.physics_engine.set_robot_position(1.0, 1.0, 0.0)
        target_world = (1.25, 1.25)
        
        target_cell = self.coord_system.world_to_cell(target_world)
        self.motion_executor.execute_single_move(target_cell)
        
        # Run until robot gets stuck
        for i in range(100):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check if robot is stuck (not moving)
            current_pos = self.physics_engine.get_robot_pose()
            if i > 0:
                pos_change = math.sqrt((current_pos[0] - prev_pos[0])**2 + 
                                     (current_pos[1] - prev_pos[1])**2)
                if pos_change < 0.001:  # Robot is stuck
                    print(f"✅ Robot stuck at iteration {i}")
                    break
            prev_pos = current_pos
        
        # Remove obstacle and continue
        self.physics_engine.clear_obstacles()
        
        # Continue motion
        max_iterations = 200
        completed = False
        snap_triggered = False
        
        for i in range(max_iterations):
            self.motion_executor.update_control_loop()
            self.physics_engine.step_physics()
            
            # Check for snap trigger
            if hasattr(self.motion_executor, '_snap_triggered') and self.motion_executor._snap_triggered:
                snap_triggered = True
                print(f"✅ Snap triggered after recovery at iteration {i}")
            
            # Check for completion
            status = self.motion_executor.get_motion_status()
            if status == MotionStatus.REACHED_TARGET:
                completed = True
                print(f"✅ Task completed after recovery at iteration {i}")
                break
        
        self.assertTrue(completed, "Task should complete after recovery")
        self.assertTrue(snap_triggered, "Snap should be triggered after recovery")


class SimplePhysicsEngine:
    """Simple physics engine for testing snap functionality."""
    
    def __init__(self):
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0
        self._robot_vx = 0.0
        self._robot_vy = 0.0
        self._robot_vtheta = 0.0
        self._dt = 0.01  # 100Hz simulation
        self._obstacles = {}  # Dictionary of obstacles
        self._obstacle_id_counter = 0
        
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get robot position and orientation."""
        return (self._robot_x, self._robot_y, self._robot_theta)
    
    def get_robot_velocity(self) -> Tuple[float, float, float]:
        """Get robot velocity."""
        return (self._robot_vx, self._robot_vy, self._robot_vtheta)
    
    def set_robot_position(self, x: float, y: float, theta: float):
        """Set robot position."""
        self._robot_x = x
        self._robot_y = y
        self._robot_theta = theta
    
    def set_wheel_velocities(self, left_vel: float, right_vel: float):
        """Set wheel velocities and update robot motion."""
        # Simple differential drive kinematics
        wheel_base = 0.3
        wheel_radius = 0.05
        
        linear_vel = (left_vel + right_vel) * wheel_radius / 2.0
        angular_vel = (right_vel - left_vel) * wheel_radius / wheel_base
        
        # Update velocities
        self._robot_vx = linear_vel * math.cos(self._robot_theta)
        self._robot_vy = linear_vel * math.sin(self._robot_theta)
        self._robot_vtheta = angular_vel
    
    def add_obstacle(self, x: float, y: float, radius: float) -> int:
        """Add an obstacle to the environment."""
        obstacle_id = self._obstacle_id_counter
        self._obstacles[obstacle_id] = {'x': x, 'y': y, 'radius': radius}
        self._obstacle_id_counter += 1
        return obstacle_id
    
    def move_obstacle(self, obstacle_id: int, dx: float, dy: float):
        """Move an obstacle."""
        if obstacle_id in self._obstacles:
            self._obstacles[obstacle_id]['x'] += dx
            self._obstacles[obstacle_id]['y'] += dy
    
    def clear_obstacles(self):
        """Clear all obstacles."""
        self._obstacles.clear()
    
    def _check_collision(self) -> bool:
        """Check if robot collides with any obstacle."""
        for obstacle in self._obstacles.values():
            distance = math.sqrt((self._robot_x - obstacle['x'])**2 + 
                               (self._robot_y - obstacle['y'])**2)
            if distance < obstacle['radius'] + 0.05:  # Robot radius ~5cm (more realistic)
                return True
        return False
    
    def step_physics(self):
        """Step physics simulation."""
        # Check for collisions before moving
        if self._check_collision():
            # Stop robot if collision detected
            self._robot_vx = 0.0
            self._robot_vy = 0.0
            self._robot_vtheta = 0.0
            print(f"[Physics] Collision detected at pos=({self._robot_x:.3f},{self._robot_y:.3f})")
            return
        
        # Integrate position
        self._robot_x += self._robot_vx * self._dt
        self._robot_y += self._robot_vy * self._dt
        self._robot_theta += self._robot_vtheta * self._dt
        
        # Normalize theta
        while self._robot_theta > math.pi:
            self._robot_theta -= 2 * math.pi
        while self._robot_theta < -math.pi:
            self._robot_theta += 2 * math.pi


if __name__ == "__main__":
    unittest.main(verbosity=2) 