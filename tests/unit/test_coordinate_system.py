"""
Unit tests for CoordinateSystem implementation.

Tests coordinate conversions, grid operations, and thread safety.
"""
import unittest
import threading
import math
from robot.impl.coordinate_system_impl import CoordinateSystemImpl
from interfaces.path_planner_interface import Cell


class TestCoordinateSystemImpl(unittest.TestCase):
    """Test cases for CoordinateSystem implementation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.coord_system = CoordinateSystemImpl(
            cell_size=0.5,
            grid_origin=(0.0, 0.0),
            grid_width=20,
            grid_height=20
        )
    
    def test_initialization(self):
        """Test proper initialization."""
        self.assertEqual(self.coord_system.cell_size, 0.5)
        self.assertEqual(self.coord_system.grid_origin, (0.0, 0.0))
        self.assertEqual(self.coord_system.grid_width, 20)
        self.assertEqual(self.coord_system.grid_height, 20)
    
    def test_world_to_cell_conversion(self):
        """Test world to cell coordinate conversion."""
        # Test origin conversion
        cell = self.coord_system.world_to_cell((0.0, 0.0))
        self.assertEqual(cell, Cell(0, 0))
        
        # Test positive coordinates
        cell = self.coord_system.world_to_cell((1.0, 1.5))
        self.assertEqual(cell, Cell(2, 3))  # 1.0/0.5=2, 1.5/0.5=3
        
        # Test with different grid origin
        coord_system = CoordinateSystemImpl(
            cell_size=1.0,
            grid_origin=(-5.0, -5.0),
            grid_width=10,
            grid_height=10
        )
        cell = coord_system.world_to_cell((0.0, 0.0))
        self.assertEqual(cell, Cell(5, 5))  # (0-(-5))/1.0 = 5
    
    def test_cell_to_world_conversion(self):
        """Test cell to world coordinate conversion."""
        # Test origin conversion (center of cell)
        world_pos = self.coord_system.cell_to_world(Cell(0, 0))
        self.assertEqual(world_pos, (0.25, 0.25))  # Center of first cell
        
        # Test other cells
        world_pos = self.coord_system.cell_to_world(Cell(2, 3))
        self.assertEqual(world_pos, (1.25, 1.75))  # (2+0.5)*0.5, (3+0.5)*0.5
        
        # Test with different grid origin
        coord_system = CoordinateSystemImpl(
            cell_size=1.0,
            grid_origin=(-5.0, -5.0),
            grid_width=10,
            grid_height=10
        )
        world_pos = coord_system.cell_to_world(Cell(5, 5))
        self.assertEqual(world_pos, (0.5, 0.5))  # -5 + (5+0.5)*1.0 = 0.5
    
    def test_round_trip_conversion(self):
        """Test world->cell->world conversion accuracy."""
        test_positions = [
            (0.0, 0.0),
            (1.0, 1.0),
            (5.5, 3.2),
            (9.9, 9.9)
        ]
        
        for original_pos in test_positions:
            cell = self.coord_system.world_to_cell(original_pos)
            recovered_pos = self.coord_system.cell_to_world(cell)
            
            # Should recover to cell center
            expected_x = (int(original_pos[0] / 0.5) + 0.5) * 0.5
            expected_y = (int(original_pos[1] / 0.5) + 0.5) * 0.5
            
            self.assertAlmostEqual(recovered_pos[0], expected_x, places=6)
            self.assertAlmostEqual(recovered_pos[1], expected_y, places=6)
    
    def test_get_cell_size(self):
        """Test cell size retrieval."""
        self.assertEqual(self.coord_system.get_cell_size(), 0.5)
        
        # Test with different size
        coord_system = CoordinateSystemImpl(cell_size=1.0)
        self.assertEqual(coord_system.get_cell_size(), 1.0)
    
    def test_get_grid_bounds(self):
        """Test grid bounds retrieval."""
        bounds = self.coord_system.get_grid_bounds()
        expected = ((0, 19), (0, 19))  # 20x20 grid, 0-indexed
        self.assertEqual(bounds, expected)
        
        # Test with different size
        coord_system = CoordinateSystemImpl(grid_width=10, grid_height=15)
        bounds = coord_system.get_grid_bounds()
        expected = ((0, 9), (0, 14))
        self.assertEqual(bounds, expected)
    
    def test_is_valid_cell(self):
        """Test cell validity checking."""
        # Valid cells
        self.assertTrue(self.coord_system.is_valid_cell(Cell(0, 0)))
        self.assertTrue(self.coord_system.is_valid_cell(Cell(10, 10)))
        self.assertTrue(self.coord_system.is_valid_cell(Cell(19, 19)))
        
        # Invalid cells
        self.assertFalse(self.coord_system.is_valid_cell(Cell(-1, 0)))
        self.assertFalse(self.coord_system.is_valid_cell(Cell(0, -1)))
        self.assertFalse(self.coord_system.is_valid_cell(Cell(20, 0)))
        self.assertFalse(self.coord_system.is_valid_cell(Cell(0, 20)))
        self.assertFalse(self.coord_system.is_valid_cell(Cell(100, 100)))
    
    def test_distance_between_cells(self):
        """Test Manhattan distance between cells."""
        # Same cell
        distance = self.coord_system.distance_between_cells(Cell(5, 5), Cell(5, 5))
        self.assertEqual(distance, 0.0)
        
        # Adjacent cells
        distance = self.coord_system.distance_between_cells(Cell(5, 5), Cell(6, 5))
        self.assertEqual(distance, 1.0)
        
        distance = self.coord_system.distance_between_cells(Cell(5, 5), Cell(5, 6))
        self.assertEqual(distance, 1.0)
        
        # Diagonal cells (Manhattan distance)
        distance = self.coord_system.distance_between_cells(Cell(0, 0), Cell(3, 4))
        self.assertEqual(distance, 7.0)  # |3-0| + |4-0| = 7
    
    def test_distance_between_world_points(self):
        """Test Euclidean distance between world points."""
        # Same point
        distance = self.coord_system.distance_between_world_points((0, 0), (0, 0))
        self.assertEqual(distance, 0.0)
        
        # Horizontal distance
        distance = self.coord_system.distance_between_world_points((0, 0), (3, 0))
        self.assertEqual(distance, 3.0)
        
        # Vertical distance
        distance = self.coord_system.distance_between_world_points((0, 0), (0, 4))
        self.assertEqual(distance, 4.0)
        
        # Diagonal distance (Pythagorean theorem)
        distance = self.coord_system.distance_between_world_points((0, 0), (3, 4))
        self.assertEqual(distance, 5.0)  # 3-4-5 triangle
    
    def test_get_neighbor_cells(self):
        """Test neighbor cell generation."""
        # Center cell (all 4 neighbors valid)
        neighbors = self.coord_system.get_neighbor_cells(Cell(10, 10))
        expected = [Cell(10, 11), Cell(10, 9), Cell(9, 10), Cell(11, 10)]
        self.assertEqual(len(neighbors), 4)
        for neighbor in expected:
            self.assertIn(neighbor, neighbors)
        
        # Corner cell (only 2 neighbors valid)
        neighbors = self.coord_system.get_neighbor_cells(Cell(0, 0))
        expected = [Cell(0, 1), Cell(1, 0)]
        self.assertEqual(len(neighbors), 2)
        for neighbor in expected:
            self.assertIn(neighbor, neighbors)
        
        # Edge cell (3 neighbors valid)
        neighbors = self.coord_system.get_neighbor_cells(Cell(0, 10))
        self.assertEqual(len(neighbors), 3)
        
        # All neighbors should be valid
        for neighbor in neighbors:
            self.assertTrue(self.coord_system.is_valid_cell(neighbor))
    
    def test_normalize_angle(self):
        """Test angle normalization to [0, 2?] convention."""
        # Already normalized
        self.assertAlmostEqual(self.coord_system.normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(self.coord_system.normalize_angle(math.pi/2), math.pi/2)
        self.assertAlmostEqual(self.coord_system.normalize_angle(math.pi), math.pi)
        
        # Needs normalization to [0, 2?]
        self.assertAlmostEqual(self.coord_system.normalize_angle(2*math.pi), 0.0)
        self.assertAlmostEqual(self.coord_system.normalize_angle(-2*math.pi), 0.0)
        self.assertAlmostEqual(self.coord_system.normalize_angle(3*math.pi), math.pi)
        self.assertAlmostEqual(self.coord_system.normalize_angle(-3*math.pi), math.pi)
        self.assertAlmostEqual(self.coord_system.normalize_angle(-math.pi/2), 3*math.pi/2)
        
        # Test edge cases
        self.assertAlmostEqual(self.coord_system.normalize_angle(4*math.pi), 0.0)
        self.assertAlmostEqual(self.coord_system.normalize_angle(-4*math.pi), 0.0)
    
    def test_angle_difference(self):
        """Test angular difference calculation."""
        # Same angle
        diff = self.coord_system.angle_difference(0.0, 0.0)
        self.assertAlmostEqual(diff, 0.0)
        
        # Simple differences
        diff = self.coord_system.angle_difference(0.0, math.pi/2)
        self.assertAlmostEqual(diff, math.pi/2)
        
        diff = self.coord_system.angle_difference(math.pi/2, 0.0)
        self.assertAlmostEqual(diff, -math.pi/2)
        
        # Wrap-around cases
        diff = self.coord_system.angle_difference(-math.pi + 0.1, math.pi - 0.1)
        self.assertAlmostEqual(diff, -0.2, places=6)
    
    def test_thread_safety(self):
        """Test thread safety of coordinate operations."""
        results = []
        
        def perform_conversions():
            for i in range(100):
                # World to cell conversion
                world_pos = (i * 0.1, i * 0.1)
                cell = self.coord_system.world_to_cell(world_pos)
                
                # Cell to world conversion
                world_back = self.coord_system.cell_to_world(cell)
                
                # Distance calculations
                distance = self.coord_system.distance_between_world_points(
                    world_pos, world_back
                )
                
                results.append((cell, world_back, distance))
        
        # Start multiple threads
        threads = []
        for _ in range(5):
            thread = threading.Thread(target=perform_conversions)
            threads.append(thread)
            thread.start()
        
        # Wait for completion
        for thread in threads:
            thread.join()
        
        # All operations should succeed
        self.assertEqual(len(results), 500)  # 5 threads * 100 operations
        
        # Results should be valid
        for cell, world_pos, distance in results:
            self.assertIsInstance(cell, Cell)
            self.assertIsInstance(world_pos, tuple)
            self.assertIsInstance(distance, float)
            self.assertGreaterEqual(distance, 0.0)
    
    def test_edge_cases(self):
        """Test edge cases and boundary conditions."""
        # Test coordinates that are actually outside the grid bounds
        with self.assertRaises(ValueError):
            self.coord_system.world_to_cell((10.0, 10.0))  # Outside grid
        
        with self.assertRaises(ValueError):
            self.coord_system.world_to_cell((20.0, 20.0))  # Far outside grid
        
        # Valid edge cases
        cell = self.coord_system.world_to_cell((0.0, 0.0))  # Grid origin
        self.assertEqual(cell, Cell(0, 0))
        
        cell = self.coord_system.world_to_cell((9.75, 9.75))  # Grid corner
        self.assertEqual(cell, Cell(19, 19))
        
        # Test negative coordinates (should work with default origin)
        cell = self.coord_system.world_to_cell((-0.1, -0.1))
        self.assertIsInstance(cell, Cell)
        
        # Very small cell size with appropriate grid bounds
        coord_system = CoordinateSystemImpl(cell_size=0.001, grid_width=2000, grid_height=2000)
        cell = coord_system.world_to_cell((1.0, 1.0))
        self.assertIsInstance(cell, Cell)
    
    def test_custom_configurations(self):
        """Test with different coordinate system configurations."""
        # Large cells
        coord_system = CoordinateSystemImpl(
            cell_size=2.0,
            grid_origin=(0.0, 0.0),
            grid_width=5,
            grid_height=5
        )
        
        cell = coord_system.world_to_cell((3.0, 3.0))
        self.assertEqual(cell, Cell(1, 1))  # 3.0/2.0 = 1.5 -> 1
        
        world_pos = coord_system.cell_to_world(Cell(1, 1))
        self.assertEqual(world_pos, (3.0, 3.0))  # (1+0.5)*2.0 = 3.0
        
        # Offset origin
        coord_system = CoordinateSystemImpl(
            cell_size=1.0,
            grid_origin=(-10.0, -10.0),
            grid_width=20,
            grid_height=20
        )
        
        cell = coord_system.world_to_cell((0.0, 0.0))
        self.assertEqual(cell, Cell(10, 10))
        
        world_pos = coord_system.cell_to_world(Cell(10, 10))
        self.assertEqual(world_pos, (0.5, 0.5))  # Center of cell at origin


if __name__ == '__main__':
    unittest.main() 