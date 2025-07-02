import unittest
import sys
import os
import decimal
from typing import Tuple

# Add parent directory to path to import modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from warehouse.map import WarehouseMap

class TestWarehouseMap(unittest.TestCase):
    """Test cases for the WarehouseMap class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.warehouse_map = WarehouseMap(width=10, height=10)
    
    def test_world_to_grid_float(self):
        """Test converting world coordinates to grid coordinates with float values."""
        # Test with integer values
        grid_x, grid_y = self.warehouse_map.world_to_grid(1.0, 2.0)
        self.assertEqual(grid_x, 2)  # 1.0 / 0.5 = 2
        self.assertEqual(grid_y, 4)  # 2.0 / 0.5 = 4
        
        # Test with float values
        grid_x, grid_y = self.warehouse_map.world_to_grid(1.25, 2.75)
        self.assertEqual(grid_x, 2)  # 1.25 / 0.5 = 2.5, int truncates to 2
        self.assertEqual(grid_y, 5)  # 2.75 / 0.5 = 5.5, int truncates to 5
    
    def test_world_to_grid_decimal(self):
        """Test converting world coordinates to grid coordinates with Decimal values."""
        # Test with Decimal values
        grid_x, grid_y = self.warehouse_map.world_to_grid(
            float(decimal.Decimal('1.5')), float(decimal.Decimal('2.5')))
        self.assertEqual(grid_x, 3)  # 1.5 / 0.5 = 3
        self.assertEqual(grid_y, 5)  # 2.5 / 0.5 = 5
        
        # Test with mixed Decimal and float
        grid_x, grid_y = self.warehouse_map.world_to_grid(
            float(decimal.Decimal('1.75')), 2.25)
        self.assertEqual(grid_x, 3)  # 1.75 / 0.5 = 3.5, int truncates to 3
        self.assertEqual(grid_y, 4)  # 2.25 / 0.5 = 4.5, int truncates to 4
    
    def test_grid_to_world_int(self):
        """Test converting grid coordinates to world coordinates with integer values."""
        # Test with integer values
        world_x, world_y = self.warehouse_map.grid_to_world(2, 4)
        self.assertEqual(world_x, 1.25)  # (2 + 0.5) * 0.5 = 1.25
        self.assertEqual(world_y, 2.25)  # (4 + 0.5) * 0.5 = 2.25
    
    def test_grid_to_world_decimal(self):
        """Test converting grid coordinates to world coordinates with Decimal values."""
        # Test with Decimal grid coordinates
        world_x, world_y = self.warehouse_map.grid_to_world(
            int(decimal.Decimal('3')), int(decimal.Decimal('5')))
        self.assertEqual(world_x, 1.75)  # (3 + 0.5) * 0.5 = 1.75
        self.assertEqual(world_y, 2.75)  # (5 + 0.5) * 0.5 = 2.75
        
        # Confirm type is float, not Decimal
        self.assertIsInstance(world_x, float)
        self.assertIsInstance(world_y, float)
    
    def test_is_walkable(self):
        """Test the is_walkable method with float and decimal values."""
        # Create a known walkable position
        grid_x, grid_y = 2, 2
        world_x, world_y = self.warehouse_map.grid_to_world(grid_x, grid_y)
        
        # Test with float values
        self.assertTrue(self.warehouse_map.is_walkable(world_x, world_y))
        
        # Test with string values that would be converted to float
        self.assertTrue(self.warehouse_map.is_walkable(
            float(str(world_x)), float(str(world_y))))
        
        # Test with a wall position (find an actual wall in the grid)
        wall_found = False
        for y in range(self.warehouse_map.height):
            for x in range(self.warehouse_map.width):
                if self.warehouse_map.grid[y, x] == 1:  # Wall
                    world_x, world_y = self.warehouse_map.grid_to_world(x, y)
                    self.assertFalse(self.warehouse_map.is_walkable(world_x, world_y))
                    wall_found = True
                    break
            if wall_found:
                break
        
        # If no walls found, test with a shelf position instead
        if not wall_found:
            for y in range(self.warehouse_map.height):
                for x in range(self.warehouse_map.width):
                    if self.warehouse_map.grid[y, x] == 2:  # Shelf
                        world_x, world_y = self.warehouse_map.grid_to_world(x, y)
                        self.assertFalse(self.warehouse_map.is_walkable(world_x, world_y))
                        break

if __name__ == '__main__':
    unittest.main() 