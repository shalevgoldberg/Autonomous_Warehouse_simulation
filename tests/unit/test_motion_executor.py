"""
Unit tests for MotionExecutor implementation.

Tests motion execution, wheel commands, path following, and thread safety.
"""
import unittest
import threading
import time
import math
from unittest.mock import Mock, patch

from robot.impl.motion_executor_impl import MotionExecutorImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from interfaces.motion_executor_interface import MotionStatus, MotionExecutionError
from interfaces.path_planner_interface import Path, Cell


class TestMotionExecutorImpl(unittest.TestCase):
    """Test cases for MotionExecutor implementation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.coord_system = CoordinateSystemImpl(
            cell_size=1.0,
            grid_origin=(0.0, 0.0),
            grid_width=10,
            grid_height=10
        )
        self.motion_executor = MotionExecutorImpl(
            coordinate_system=self.coord_system,
            robot_id="test_robot"
        )
    
    def test_initialization(self):
        """Test proper initialization."""
        self.assertEqual(self.motion_executor.coordinate_system, self.coord_system)
        self.assertEqual(self.motion_executor.robot_id, "test_robot")
        self.assertEqual(self.motion_executor.get_motion_status(), MotionStatus.IDLE)
        self.assertIsInstance(self.motion_executor._status_lock, type(threading.RLock()))
        self.assertIsInstance(self.motion_executor._command_lock, type(threading.Lock()))
    
    def test_initial_status(self):
        """Test initial motion status."""
        status = self.motion_executor.get_motion_status()
        
        self.assertEqual(status, MotionStatus.IDLE)
        
        # Should have no target initially
        self.assertIsNone(self.motion_executor.get_current_target())
        
        # Should have default speed
        self.assertEqual(self.motion_executor.get_movement_speed(), 0.5)
    
    def test_single_move_execution(self):
        """Test single move execution."""
        target_cell = Cell(3, 4)
        target_world = self.coord_system.cell_to_world(target_cell)
        
        self.motion_executor.execute_single_move(target_cell)
        
        self.assertEqual(self.motion_executor.get_motion_status(), MotionStatus.EXECUTING)
        self.assertEqual(self.motion_executor.get_current_target(), target_world)
    
    def test_path_execution(self):
        """Test path execution."""
        path = Path(
            cells=[Cell(0, 0), Cell(1, 0), Cell(2, 0)],
            total_distance=2.0,
            estimated_time=4.0
        )
        
        self.motion_executor.execute_path(path)
        
        self.assertEqual(self.motion_executor.get_motion_status(), MotionStatus.EXECUTING)
        self.assertIsNotNone(self.motion_executor.get_current_target())
    
    def test_stop_execution(self):
        """Test stop execution."""
        self.motion_executor.execute_single_move(Cell(2, 2))
        self.assertEqual(self.motion_executor.get_motion_status(), MotionStatus.EXECUTING)
        
        self.motion_executor.stop_execution()
        self.assertEqual(self.motion_executor.get_motion_status(), MotionStatus.IDLE)
    
    def test_emergency_stop(self):
        """Test emergency stop."""
        self.motion_executor.execute_single_move(Cell(2, 2))
        self.motion_executor.emergency_stop()
        self.assertEqual(self.motion_executor.get_motion_status(), MotionStatus.ERROR)
    
    def test_speed_control(self):
        """Test speed control."""
        self.motion_executor.set_movement_speed(1.2)
        self.assertEqual(self.motion_executor.get_movement_speed(), 1.2)
        
        # Test clamping
        self.motion_executor.set_movement_speed(5.0)
        self.assertLessEqual(self.motion_executor.get_movement_speed(), 2.0)
        
        self.motion_executor.set_movement_speed(-1.0)
        self.assertGreaterEqual(self.motion_executor.get_movement_speed(), 0.0)
    
    def test_error_handling(self):
        """Test error handling."""
        # Test invalid cell
        with self.assertRaises(MotionExecutionError):
            self.motion_executor.execute_single_move(Cell(100, 100))
        
        # Test empty path
        empty_path = Path(cells=[], total_distance=0.0, estimated_time=0.0)
        with self.assertRaises(MotionExecutionError):
            self.motion_executor.execute_path(empty_path)


if __name__ == '__main__':
    unittest.main()
