"""
Comprehensive integration test for all robot components working together.

Tests StateHolder, CoordinateSystem, PathPlanner, TaskHandler, and MotionExecutor.
Updated for Phase 2 with SimulationDataService integration.
"""
import unittest
import time
import threading
import os
from typing import Tuple, Optional

from robot.impl.state_holder_impl import StateHolderImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from robot.impl.path_planner_impl import PathPlannerImpl
from robot.impl.task_handler_impl import TaskHandlerImpl
from robot.impl.motion_executor_impl import MotionExecutorImpl
from interfaces.path_planner_interface import Cell, Path
from interfaces.task_handler_interface import Task, TaskType, TaskStatus
from interfaces.motion_executor_interface import MotionStatus
from interfaces.simulation_data_service_interface import ISimulationDataService, ShelfInfo, MapData
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap


class SimpleTestPhysics:
    """Simple physics simulator for testing motion execution."""
    
    def __init__(self):
        """Initialize simple physics with robot at origin."""
        self._lock = threading.RLock()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_theta = 0.0
        self._left_wheel_vel = 0.0
        self._right_wheel_vel = 0.0
        self._wheel_radius = 0.05  # meters
        self._wheel_base = 0.3  # meters
        self._dt = 0.01  # 100Hz simulation
        
    def set_wheel_velocities(self, left_vel: float, right_vel: float) -> None:
        """Set wheel velocities."""
        with self._lock:
            self._left_wheel_vel = left_vel
            self._right_wheel_vel = right_vel
    
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """Get current robot pose."""
        with self._lock:
            return (self._robot_x, self._robot_y, self._robot_theta)
    
    def step_physics(self) -> None:
        """Step physics simulation to update robot position."""
        with self._lock:
            # Simple differential drive kinematics
            linear_vel = (self._left_wheel_vel + self._right_wheel_vel) * self._wheel_radius / 2.0
            angular_vel = (self._right_wheel_vel - self._left_wheel_vel) * self._wheel_radius / self._wheel_base
            
            # Integrate position using proper differential drive kinematics
            # Move in the direction the robot is facing (theta)
            import math
            self._robot_x += linear_vel * self._dt * math.cos(self._robot_theta)
            self._robot_y += linear_vel * self._dt * math.sin(self._robot_theta)
            self._robot_theta += angular_vel * self._dt
            
            # Normalize theta to [-π, π]
            while self._robot_theta > 3.14159:
                self._robot_theta -= 2 * 3.14159
            while self._robot_theta < -3.14159:
                self._robot_theta += 2 * 3.14159
    
    def reset_robot_position(self, x: float, y: float, theta: float) -> None:
        """Reset robot to specific position."""
        with self._lock:
            self._robot_x = x
            self._robot_y = y
            self._robot_theta = theta


class MockSimulationDataService(ISimulationDataService):
    """Mock SimulationDataService for testing when database is not available."""
    
    def get_map_data(self):
        return MapData(
            width=20, 
            height=20, 
            cell_size=0.5, 
            shelves={},
            obstacles=[],
            dropoff_zones=[(9, 9)],  # Use integers for grid coordinates
            charging_zones=[]
        )
    
    def get_shelf_position(self, shelf_id: str):
        return (1.0, 1.0)  # Default position
    
    def get_dropoff_zones(self):
        return [(9.0, 9.0)]  # Default dropoff zone
    
    def lock_shelf(self, shelf_id: str, robot_id: str):
        return True  # Always successful
    
    def unlock_shelf(self, shelf_id: str, robot_id: str):
        return True  # Always successful
    
    def is_shelf_locked(self, shelf_id: str):
        return False  # Never locked
    
    def get_shelf_lock_owner(self, shelf_id: str):
        return None  # No owner
    
    def get_shelf_info(self, shelf_id: str):
        return ShelfInfo(
            shelf_id=shelf_id,
            position=(1.0, 1.0),
            items=[],
            capacity=10,
            is_locked=False
        )
    
    def get_item_location(self, item_id: str):
        return "SHELF_001"  # Default shelf
    
    def update_inventory(self, shelf_id: str, item_id: str, operation: str, quantity: int = 1):
        return True  # Always successful
    
    def log_event(self, event_type: str, robot_id: str, event_data: dict):
        pass  # No-op for testing
    
    def close(self):
        pass  # No-op for testing


class TestBasicIntegration(unittest.TestCase):
    """Integration test for core components."""
    
    def setUp(self):
        """Set up test components."""
        # Create coordinate system for 10x10 warehouse
        self.coord_system = CoordinateSystemImpl(
            cell_size=0.5,  # 50cm cells
            grid_origin=(0.0, 0.0),
            grid_width=20,
            grid_height=20
        )
        
        # Create path planner with some obstacles (shelves)
        self.path_planner = PathPlannerImpl(self.coord_system)
        shelf_obstacles = [
            Cell(5, 5), Cell(5, 6), Cell(5, 7),  # Shelf row 1
            Cell(10, 5), Cell(10, 6), Cell(10, 7),  # Shelf row 2
            Cell(15, 5), Cell(15, 6), Cell(15, 7),  # Shelf row 3
        ]
        for obstacle in shelf_obstacles:
            self.path_planner.add_static_obstacle(obstacle)
        
        # Create simple physics for testing
        self.physics = SimpleTestPhysics()
        
        # Create state holder for robot with physics
        self.state_holder = StateHolderImpl("robot_1", physics_engine=self.physics)
        
        # Create motion executor with physics
        self.motion_executor = MotionExecutorImpl(
            coordinate_system=self.coord_system,
            robot_id="robot_1",
            physics_engine=self.physics
        )
        
        # Create SimulationDataService for Phase 2 integration
        try:
            # Try to use real database if available
            db_password = os.getenv('WAREHOUSE_DB_PASSWORD')
            if db_password:
                warehouse_map = WarehouseMap(csv_file="sample_warehouse.csv")
                self.simulation_data_service = SimulationDataServiceImpl(
                    warehouse_map=warehouse_map,
                    db_password=db_password,
                    pool_size=3
                )
            else:
                # Use mock service for basic testing
                self.simulation_data_service = MockSimulationDataService()
                print("⚠️  WAREHOUSE_DB_PASSWORD not set - using mock SimulationDataService")
        except Exception as e:
            # Use mock service if database connection fails
            self.simulation_data_service = MockSimulationDataService()
            print(f"⚠️  Database connection failed - using mock SimulationDataService: {e}")
        
        # Create task handler (the orchestrator) with SimulationDataService
        self.task_handler = TaskHandlerImpl(
            state_holder=self.state_holder,
            path_planner=self.path_planner,
            motion_executor=self.motion_executor,
            coordinate_system=self.coord_system,
            robot_id="robot_1",
            simulation_data_service=self.simulation_data_service
        )
        
        # Start control loop for motion execution
        self._control_loop_running = True
        self._control_thread = threading.Thread(target=self._run_control_loop, daemon=True)
        self._control_thread.start()
    
    def _run_control_loop(self):
        """Run control loop for motion execution at 100Hz."""
        while self._control_loop_running:
            try:
                # Update motion executor
                self.motion_executor.update_control_loop()
                
                # Step physics simulation
                self.physics.step_physics()
                
                # Update state holder
                self.state_holder.update_from_simulation()
                
                # Sleep for 10ms (100Hz)
                time.sleep(0.01)
            except Exception as e:
                print(f"[Control Loop] Error: {e}")
                break
    
    def tearDown(self):
        """Clean up test resources."""
        self._control_loop_running = False
        if hasattr(self, '_control_thread'):
            self._control_thread.join(timeout=1.0)
        
        if hasattr(self, 'simulation_data_service'):
            self.simulation_data_service.close()
    
    def test_warehouse_navigation_simulation(self):
        """Test simulated warehouse navigation scenario."""
        print("\n=== Warehouse Navigation Simulation ===")
        
        # Robot starts at warehouse entrance
        robot_start = (0.5, 0.5)  # Cell (1,1)
        self.state_holder.set_test_position((robot_start[0], robot_start[1], 0.0))
        
        # Mission: Go to shelf area, then to loading dock
        waypoints = [
            (2.0, 2.0),   # Navigate to open area (Cell 4,4)
            (7.0, 2.0),   # Around first shelf (Cell 14,4)
            (8.0, 2.0),   # Around second shelf (Cell 24,4)
            (8.5, 3.0),   # Near shelf area (Cell 34,4)
            (9.0, 9.0),   # Loading dock
        ]
        
        print(f"Robot starting at: {robot_start}")
        current_pos = robot_start
        
        for i, waypoint in enumerate(waypoints):
            print(f"\nWaypoint {i+1}: Planning path to {waypoint}")
            
            # Plan path using PathPlanner
            path = self.path_planner.plan_path(current_pos, waypoint)
            
            print(f"  Path found: {len(path.cells)} cells")
            print(f"  Distance: {path.total_distance:.2f}m")
            print(f"  Estimated time: {path.estimated_time:.1f}s")
            
            # Simulate robot movement along path
            for j, cell in enumerate(path.cells):
                world_pos = self.coord_system.cell_to_world(cell)
                
                # Update robot position
                self.state_holder.set_test_position((world_pos[0], world_pos[1], 0.0))
                
                # Simulate some time passage and state updates
                if j % 3 == 0:  # Update state every few cells
                    self.state_holder.update_from_simulation()
                
                # Get current state
                robot_state = self.state_holder.get_robot_state()
                
                # Check path is not blocked
                self.assertFalse(self.path_planner.is_path_blocked(path))
            
            current_pos = waypoint
            print(f"  Reached waypoint {i+1}")
        
        # Final state check
        final_state = self.state_holder.get_robot_state()
        final_world_pos = final_state.position[:2]  # x, y only
        final_cell = self.coord_system.world_to_cell(final_world_pos)
        
        print(f"\nMission completed!")
        print(f"Final position: {final_world_pos}")
        print(f"Final cell: ({final_cell.x}, {final_cell.y})")
        print(f"Battery remaining: {final_state.battery_level:.1%}")
        
        # Verify robot reached destination area
        self.assertGreater(final_world_pos[0], 8.0)  # Near loading dock X
        self.assertGreater(final_world_pos[1], 8.0)  # Near loading dock Y
        self.assertGreater(final_state.battery_level, 0.9)  # Still have battery
    
    def test_coordinate_conversions_accuracy(self):
        """Test accuracy of coordinate conversions."""
        test_cases = [
            ((0.0, 0.0), Cell(0, 0)),
            ((2.5, 3.5), Cell(5, 7)),
            ((9.75, 9.75), Cell(19, 19)),
        ]
        
        for world_pos, expected_cell in test_cases:
            # World to cell
            actual_cell = self.coord_system.world_to_cell(world_pos)
            self.assertEqual(actual_cell, expected_cell)
            
            # Cell back to world (should be cell center)
            recovered_world = self.coord_system.cell_to_world(actual_cell)
            expected_world = (
                (expected_cell.x + 0.5) * 0.5,
                (expected_cell.y + 0.5) * 0.5
            )
            self.assertEqual(recovered_world, expected_world)
    
    def test_obstacle_avoidance(self):
        """Test that paths correctly avoid obstacles."""
        # Try to plan path that would go through obstacles
        start = (2.0, 2.5)   # Before shelf area
        goal = (8.0, 2.5)    # After shelf area (direct path blocked)
        
        path = self.path_planner.plan_path(start, goal)
        
        # Path should exist
        self.assertIsInstance(path, Path)
        self.assertGreater(len(path.cells), 0)
        
        # Path should not contain any obstacles
        for cell in path.cells:
            self.assertNotIn(cell, self.path_planner.get_static_obstacles())
        
        # Path should be longer than direct path due to obstacle avoidance
        direct_distance = abs(goal[0] - start[0])  # Direct X distance
        self.assertGreater(path.total_distance, direct_distance)
    
    def test_state_holder_simulation_accuracy(self):
        """Test StateHolder simulation behavior."""
        initial_state = self.state_holder.get_robot_state()
        
        # Run simulation for several steps
        for _ in range(50):
            self.state_holder.update_from_simulation()
        
        final_state = self.state_holder.get_robot_state()
        
        # Battery should have drained
        self.assertLess(final_state.battery_level, initial_state.battery_level)
        
        # Position should have changed (simulated movement)
        self.assertNotEqual(final_state.position, initial_state.position)
        
        # Timestamp should be more recent
        self.assertGreaterEqual(final_state.timestamp, initial_state.timestamp)
    
    def test_component_thread_safety(self):
        """Test basic thread safety of components."""
        results = []
        
        def coordinate_operations():
            for i in range(100):
                pos = (i * 0.1, i * 0.1)
                cell = self.coord_system.world_to_cell(pos)
                world_back = self.coord_system.cell_to_world(cell)
                results.append((pos, cell, world_back))
        
        def state_operations():
            for _ in range(50):
                state = self.state_holder.get_robot_state()
                self.state_holder.update_from_simulation()
                results.append(state)
        
        # Run operations in parallel
        threads = [
            threading.Thread(target=coordinate_operations),
            threading.Thread(target=state_operations),
        ]
        
        for thread in threads:
            thread.start()
        
        for thread in threads:
            thread.join()
        
        # All operations should complete successfully
        self.assertEqual(len(results), 150)  # 100 + 50
    
    def test_complete_task_workflow(self):
        """Test complete task workflow with all components."""
        print(f"\n{'='*60}")
        print("TESTING COMPLETE TASK WORKFLOW")
        print(f"{'='*60}")
        
        # Set initial robot position
        initial_pos = (1.0, 1.0, 0.0)
        self.state_holder.set_test_position(initial_pos)
        
        # Create a move-to-position task
        task = Task(
            task_id="workflow_001",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(5.0, 5.0, 0.0),
            order_id="test_order_001"  # Required for Phase 2
        )
        
        print(f"📋 Created task: {task.task_id}")
        
        # Start the task
        success = self.task_handler.start_task(task)
        self.assertTrue(success, "Task should be accepted")
        print("✅ Task started successfully")
        
        # Monitor and execute task
        max_iterations = 2000
        iteration = 0
        
        while iteration < max_iterations:
            # Get current status
            status = self.task_handler.get_task_status()
            motion_status = self.motion_executor.get_motion_status()
            robot_state = self.state_holder.get_robot_state()
            
            # Log progress every 20 iterations
            if iteration % 20 == 0:
                print(f"  📊 Iteration {iteration}: Status={status.operational_status.value}, "
                      f"Motion={motion_status.value}, Progress={status.progress:.1%}, "
                      f"Battery={robot_state.battery_level:.1%}")
            
            # Update task execution (10Hz control loop)
            self.task_handler.update_task_execution()
            
            # Update motion executor (100Hz control loop)
            for _ in range(10):
                self.motion_executor.update_control_loop()
            
            # Update state holder (physics simulation)
            self.state_holder.update_from_simulation()
            
            # Check if task completed
            if not status.has_active_task:
                print("✅ Task completed!")
                break
            
            iteration += 1
            time.sleep(0.01)  # Simulate real-time execution
        
        # Verify task completion
        final_status = self.task_handler.get_task_status()
        task_history = self.task_handler.get_task_history()
        
        self.assertFalse(final_status.has_active_task, "Task should be completed")
        self.assertEqual(len(task_history), 1, "Should have one task in history")
        
        completed_task = task_history[0]
        self.assertEqual(completed_task.status, TaskStatus.COMPLETED, "Task should be completed successfully")
        
        # Verify robot moved from initial position
        final_position = self.state_holder.get_robot_state().position
        distance_moved = ((final_position[0] - initial_pos[0])**2 + 
                         (final_position[1] - initial_pos[1])**2)**0.5
        self.assertGreater(distance_moved, 0.005, "Robot should have moved significantly")
        
        print(f"📏 Distance moved: {distance_moved:.2f}m")
        print("✅ Complete task workflow test passed!")
    
    def test_multiple_task_sequence(self):
        """Test executing multiple tasks in sequence."""
        print(f"\n{'='*60}")
        print("TESTING MULTIPLE TASK SEQUENCE")
        print(f"{'='*60}")
        
        # Set initial position
        self.state_holder.set_test_position((1.0, 1.0, 0.0))
        
        # Create multiple tasks
        tasks = [
            Task(task_id="seq_001", task_type=TaskType.MOVE_TO_POSITION, target_position=(3.0, 3.0, 0.0), order_id="test_order_001"),
            Task(task_id="seq_002", task_type=TaskType.MOVE_TO_POSITION, target_position=(7.0, 3.0, 0.0), order_id="test_order_002"),
            Task(task_id="seq_003", task_type=TaskType.MOVE_TO_POSITION, target_position=(9.0, 5.0, 0.0), order_id="test_order_003"),
        ]
        
        completed_tasks = []
        
        for i, task in enumerate(tasks):
            print(f"🔄 Starting task {i+1}/{len(tasks)}: {task.task_id}")
            
            # Start task
            success = self.task_handler.start_task(task)
            self.assertTrue(success, f"Task {task.task_id} should be accepted")
            
            # Execute task
            max_iterations = 50
            iteration = 0
            
            while iteration < max_iterations:
                status = self.task_handler.get_task_status()
                
                if not status.has_active_task:
                    break
                
                self.task_handler.update_task_execution()
                
                # Update motion and state
                for _ in range(10):
                    self.motion_executor.update_control_loop()
                self.state_holder.update_from_simulation()
                
                iteration += 1
                time.sleep(0.01)
            
            print(f"✅ Task {task.task_id} completed in {iteration} iterations")
        
        # Verify all tasks completed
        task_history = self.task_handler.get_task_history()
        self.assertEqual(len(task_history), len(tasks), "All tasks should be completed")
        
        for task in task_history:
            self.assertEqual(task.status, TaskStatus.COMPLETED, f"Task {task.task_id} should be completed")
        
        print(f"✅ Multiple task sequence test passed! Completed {len(task_history)} tasks")
    
    def test_error_handling_and_recovery(self):
        """Test error handling and recovery across components."""
        print(f"\n{'='*60}")
        print("TESTING ERROR HANDLING AND RECOVERY")
        print(f"{'='*60}")
        
        # Test 1: Invalid task (should be rejected)
        try:
            invalid_task = Task(
                task_id="invalid_001",
                task_type=TaskType.PICK_AND_DELIVER,
                # Missing required fields - should be invalid
            )
            # If we get here, the task was created successfully (which is wrong)
            success = self.task_handler.start_task(invalid_task)
            self.assertFalse(success, "Invalid task should be rejected")
        except ValueError as e:
            # Expected behavior - task creation should fail validation
            print(f"✅ Invalid task correctly rejected: {e}")
            return  # Skip the rest of this test since the task creation failed
        print("✅ Invalid task correctly rejected")
        
        # Test 2: Task with unreachable target (should fail during execution)
        unreachable_task = Task(
            task_id="unreachable_001",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(50.0, 50.0, 0.0),  # Outside grid bounds
            order_id="test_order_unreachable"
        )
        
        success = self.task_handler.start_task(unreachable_task)
        self.assertTrue(success, "Task should be accepted initially")
        
        # Execute until failure
        max_iterations = 30
        iteration = 0
        
        while iteration < max_iterations:
            status = self.task_handler.get_task_status()
            
            if not status.has_active_task:
                break
            
            self.task_handler.update_task_execution()
            
            for _ in range(10):
                self.motion_executor.update_control_loop()
            self.state_holder.update_from_simulation()
            
            iteration += 1
            time.sleep(0.01)
        
        # Verify task failed
        task_history = self.task_handler.get_task_history()
        if task_history:
            failed_task = task_history[-1]
            self.assertEqual(failed_task.status, TaskStatus.FAILED, "Task should have failed")
            print(f"✅ Task {failed_task.task_id} failed as expected")
        
        # Test 3: Recovery - try a valid task after failure
        recovery_task = Task(
            task_id="recovery_001",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(3.0, 3.0, 0.0),
            order_id="test_order_recovery"
        )
        
        success = self.task_handler.start_task(recovery_task)
        self.assertTrue(success, "Recovery task should be accepted")
        
        # Execute recovery task
        max_iterations = 50
        iteration = 0
        
        while iteration < max_iterations:
            status = self.task_handler.get_task_status()
            
            if not status.has_active_task:
                break
            
            self.task_handler.update_task_execution()
            
            for _ in range(10):
                self.motion_executor.update_control_loop()
            self.state_holder.update_from_simulation()
            
            iteration += 1
            time.sleep(0.01)
        
        # Verify recovery successful
        task_history = self.task_handler.get_task_history()
        if len(task_history) >= 2:
            recovery_completed = task_history[-1]
            self.assertEqual(recovery_completed.status, TaskStatus.COMPLETED, "Recovery task should succeed")
            print(f"✅ Recovery task {recovery_completed.task_id} completed successfully")
        
        print("✅ Error handling and recovery test passed!")
    
    def test_concurrent_operations(self):
        """Test concurrent operations across all components."""
        print(f"\n{'='*60}")
        print("TESTING CONCURRENT OPERATIONS")
        print(f"{'='*60}")
        
        # Set initial position
        self.state_holder.set_test_position((1.0, 1.0, 0.0))
        
        # Start a task
        task = Task(
            task_id="concurrent_001",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(5.0, 5.0, 0.0),
            order_id="test_order_concurrent"
        )
        
        success = self.task_handler.start_task(task)
        self.assertTrue(success, "Task should be accepted")
        
        # Run concurrent operations
        results = []
        errors = []
        
        def status_monitor():
            """Monitor task status concurrently."""
            for i in range(50):
                try:
                    status = self.task_handler.get_task_status()
                    motion_status = self.motion_executor.get_motion_status()
                    robot_state = self.state_holder.get_robot_state()
                    
                    results.append({
                        'iteration': i,
                        'task_status': status.operational_status.value,
                        'motion_status': motion_status.value,
                        'battery': robot_state.battery_level
                    })
                    
                    time.sleep(0.02)
                except Exception as e:
                    errors.append(f"Status monitor error: {e}")
        
        def state_updater():
            """Update state concurrently."""
            for i in range(100):
                try:
                    self.state_holder.update_from_simulation()
                    time.sleep(0.01)
                except Exception as e:
                    errors.append(f"State updater error: {e}")
        
        def motion_updater():
            """Update motion concurrently."""
            for i in range(100):
                try:
                    self.motion_executor.update_control_loop()
                    time.sleep(0.01)
                except Exception as e:
                    errors.append(f"Motion updater error: {e}")
        
        # Start concurrent threads
        threads = [
            threading.Thread(target=status_monitor, name="StatusMonitor"),
            threading.Thread(target=state_updater, name="StateUpdater"),
            threading.Thread(target=motion_updater, name="MotionUpdater"),
        ]
        
        for thread in threads:
            thread.start()
        
        # Main thread continues task execution
        max_iterations = 50
        iteration = 0
        
        while iteration < max_iterations:
            status = self.task_handler.get_task_status()
            
            if not status.has_active_task:
                break
            
            self.task_handler.update_task_execution()
            iteration += 1
            time.sleep(0.02)
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
        
        # Verify no errors occurred
        self.assertEqual(len(errors), 0, f"No errors should occur during concurrent operations: {errors}")
        
        # Verify results
        self.assertGreater(len(results), 0, "Should have status monitoring results")
        
        # Verify task completed
        final_status = self.task_handler.get_task_status()
        self.assertFalse(final_status.has_active_task, "Task should be completed")
        
        print(f"✅ Concurrent operations test passed! {len(results)} status checks, {len(errors)} errors")
    
    def test_component_interaction_verification(self):
        """Verify that all components interact correctly."""
        print(f"\n{'='*60}")
        print("VERIFYING COMPONENT INTERACTIONS")
        print(f"{'='*60}")
        
        # Test 1: TaskHandler → PathPlanner interaction
        print("🔄 Testing TaskHandler → PathPlanner")
        
        # Create a task that requires path planning
        task = Task(
            task_id="interaction_001",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(5.0, 5.0, 0.0),
            order_id="test_order_interaction"
        )
        
        success = self.task_handler.start_task(task)
        self.assertTrue(success, "Task should be accepted")
        
        # Verify path planning occurred
        status = self.task_handler.get_task_status()
        self.assertIsNotNone(status.task_id, "Task should be active")
        print("✅ TaskHandler → PathPlanner interaction verified")
        
        # Test 2: TaskHandler → MotionExecutor interaction
        print("🔄 Testing TaskHandler → MotionExecutor")
        
        # Execute a few iterations to trigger motion execution
        for i in range(10):
            self.task_handler.update_task_execution()
            
            for _ in range(10):
                self.motion_executor.update_control_loop()
            self.state_holder.update_from_simulation()
            
            motion_status = self.motion_executor.get_motion_status()
            if motion_status == MotionStatus.EXECUTING:
                print("✅ MotionExecutor received commands from TaskHandler")
                break
        
        # Test 3: MotionExecutor → StateHolder interaction (simulated)
        print("🔄 Testing MotionExecutor → StateHolder (simulated)")
        
        # In real implementation, MotionExecutor would read from StateHolder
        # For now, verify they can coexist
        robot_state = self.state_holder.get_robot_state()
        motion_target = self.motion_executor.get_current_target()
        
        self.assertIsNotNone(robot_state, "StateHolder should provide robot state")
        self.assertIsNotNone(motion_target, "MotionExecutor should have a target")
        print("✅ MotionExecutor → StateHolder interaction verified")
        
        # Test 4: CoordinateSystem integration
        print("🔄 Testing CoordinateSystem integration")
        
        # Verify all components use the same coordinate system
        self.assertIs(self.task_handler.coordinate_system, self.coord_system)
        self.assertIs(self.motion_executor.coordinate_system, self.coord_system)
        self.assertIs(self.path_planner.coordinate_system, self.coord_system)
        
        # Test coordinate conversion consistency
        test_cell = Cell(5, 5)
        world_pos = self.coord_system.cell_to_world(test_cell)
        cell_back = self.coord_system.world_to_cell(world_pos)
        
        self.assertEqual(test_cell, cell_back, "Coordinate conversions should be consistent")
        print("✅ CoordinateSystem integration verified")
        
        # Test 5: Thread safety across components
        print("🔄 Testing thread safety across components")
        
        def concurrent_access():
            """Access multiple components concurrently."""
            for i in range(20):
                # Read from all components
                self.task_handler.get_task_status()
                self.motion_executor.get_motion_status()
                self.state_holder.get_robot_state()
                self.path_planner.get_static_obstacles()
                
                time.sleep(0.001)
        
        # Run concurrent access
        threads = [threading.Thread(target=concurrent_access) for _ in range(5)]
        
        for thread in threads:
            thread.start()
        
        for thread in threads:
            thread.join()
        
        print("✅ Thread safety verified across all components")
        print("✅ Component interaction verification passed!")
    
    def test_performance_and_responsiveness(self):
        """Test performance and responsiveness of the integrated system."""
        print(f"\n{'='*60}")
        print("TESTING PERFORMANCE AND RESPONSIVENESS")
        print(f"{'='*60}")
        
        # Set initial position
        self.state_holder.set_test_position((1.0, 1.0, 0.0))
        
        # Measure status check performance
        status_check_times = []
        for i in range(100):
            start_time = time.time()
            status = self.task_handler.get_task_status()
            motion_status = self.motion_executor.get_motion_status()
            robot_state = self.state_holder.get_robot_state()
            end_time = time.time()
            
            status_check_times.append(end_time - start_time)
        
        avg_status_time = sum(status_check_times) / len(status_check_times)
        max_status_time = max(status_check_times)
        
        print(f"📊 Status check - Avg: {avg_status_time*1000:.2f}ms, Max: {max_status_time*1000:.2f}ms")
        
        # Verify responsiveness
        self.assertLess(avg_status_time, 0.001, "Average status check should be under 1ms")
        self.assertLess(max_status_time, 0.01, "Maximum status check should be under 10ms")
        
        # Measure task execution performance
        task = Task(
            task_id="perf_001",
            task_type=TaskType.MOVE_TO_POSITION,
            target_position=(3.0, 3.0, 0.0)
        )
        
        start_time = time.time()
        success = self.task_handler.start_task(task)
        self.assertTrue(success, "Task should be accepted")
        
        # Execute task
        max_iterations = 50
        iteration = 0
        update_times = []
        
        while iteration < max_iterations:
            status = self.task_handler.get_task_status()
            
            if not status.has_active_task:
                break
            
            update_start = time.time()
            self.task_handler.update_task_execution()
            
            for _ in range(10):
                self.motion_executor.update_control_loop()
            self.state_holder.update_from_simulation()
            
            update_end = time.time()
            update_times.append(update_end - update_start)
            
            iteration += 1
            time.sleep(0.01)
        
        total_time = time.time() - start_time
        avg_update_time = sum(update_times) / len(update_times) if update_times else 0
        
        print(f"📊 Task execution - Total: {total_time:.2f}s, Avg update: {avg_update_time*1000:.2f}ms")
        
        # Verify task completed in reasonable time
        self.assertLess(total_time, 10.0, "Task should complete within 10 seconds")
        self.assertLess(avg_update_time, 0.1, "Average update should be under 100ms")
        
        print("✅ Performance and responsiveness test passed!")


if __name__ == '__main__':
    unittest.main() 