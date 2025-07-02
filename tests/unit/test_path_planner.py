"""
Unit tests for PathPlanner implementation.

Tests A* pathfinding, obstacle avoidance, and thread safety.
"""
import unittest
import threading
import time
import math
from unittest.mock import Mock, patch

from robot.impl.path_planner_impl import PathPlannerImpl
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from interfaces.path_planner_interface import Path, Cell, PathPlanningError


class TestPathPlannerImpl(unittest.TestCase):
    """Test cases for PathPlanner implementation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.coord_system = CoordinateSystemImpl(
            cell_size=1.0,
            grid_origin=(0.0, 0.0),
            grid_width=10,
            grid_height=10
        )
        self.path_planner = PathPlannerImpl(self.coord_system)
    
    def test_initialization(self):
        """Test proper initialization."""
        self.assertEqual(self.path_planner.coordinate_system, self.coord_system)
        self.assertEqual(len(self.path_planner.get_static_obstacles()), 0)
    
    def test_simple_path_planning(self):
        """Test simple path planning without obstacles."""
        start = (0.5, 0.5)
        goal = (2.5, 2.5)
        
        path = self.path_planner.plan_path(start, goal)
        
        self.assertIsInstance(path, Path)
        self.assertGreater(len(path.cells), 0)
        self.assertGreater(path.total_distance, 0.0)
        self.assertGreater(path.estimated_time, 0.0)
    
    def test_obstacle_avoidance(self):
        """Test path planning with obstacles."""
        # Add obstacle
        obstacle = Cell(1, 1)
        self.path_planner.add_static_obstacle(obstacle)
        
        start = (0.5, 0.5)
        goal = (2.5, 2.5)
        
        path = self.path_planner.plan_path(start, goal)
        
        # Path should not contain obstacle
        self.assertNotIn(obstacle, path.cells)
    
    def test_no_path_available(self):
        """Test when no path is available."""
        # Block the entire path
        for x in range(10):
            self.path_planner.add_static_obstacle(Cell(x, 1))
        
        start = (0.5, 0.5)
        goal = (2.5, 2.5)
        
        with self.assertRaises(PathPlanningError):
            self.path_planner.plan_path(start, goal)
    
    def test_thread_safety(self):
        """Test thread safety of path planning."""
        results = []
        
        def plan_paths():
            for i in range(10):
                start = (i * 0.5, i * 0.5)
                goal = ((i + 2) * 0.5, (i + 2) * 0.5)
                try:
                    path = self.path_planner.plan_path(start, goal)
                    results.append(f"path_{i}")
                except PathPlanningError:
                    results.append(f"blocked_{i}")
        
        threads = [threading.Thread(target=plan_paths) for _ in range(3)]
        
        for thread in threads:
            thread.start()
        
        for thread in threads:
            thread.join()
        
        self.assertEqual(len(results), 30)  # 3 threads * 10 paths


if __name__ == '__main__':
    unittest.main()
