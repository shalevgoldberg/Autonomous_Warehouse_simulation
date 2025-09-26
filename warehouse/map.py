import numpy as np
import random
import csv
import os
from typing import List, Tuple, Dict, Optional
from warehouse.shelf import Shelf

class WarehouseMap:
    """Static warehouse map with walls, shelves, charging zones, idle zones, and drop-off stations."""
    
    @staticmethod
    def _read_csv_dimensions(csv_file: str) -> Tuple[int, int]:
        """Read dimensions from CSV file.
        
        Returns:
            Tuple of (width, height) or (0, 0) if file can't be read
        """
        try:
            with open(csv_file, 'r', newline='', encoding='utf-8-sig') as file:
                reader = csv.reader(file)
                rows = list(reader)
            
            if not rows:
                return (0, 0)
            
            csv_height = len(rows)
            csv_width = max(len(row) for row in rows) if rows else 0
            
            return (csv_width, csv_height)
        except Exception as e:
            return (0, 0)

    def __init__(self, width: int = 20, height: int = 15, csv_file: Optional[str] = None):
        # If CSV file is provided, read its dimensions first
        if csv_file and os.path.exists(csv_file):
            csv_width, csv_height = self._read_csv_dimensions(csv_file)
            if csv_width > 0 and csv_height > 0:
                print(f"CSV file detected: using dimensions {csv_width}x{csv_height}")
                width = csv_width
                height = csv_height
            else:
                print(f"WARNING: Could not read valid dimensions from CSV file, using defaults {width}x{height}")
        
        self.width = width
        self.height = height
        self.grid_size = 0.5  # meters per grid cell
        
        # Initialize empty grid (0 = free space, 1 = wall, 2 = shelf, 3 = charging, 4 = idle zone, 5 = drop-off)
        self.grid = np.zeros((height, width), dtype=int)
        
        # Initialize shelves dictionary (grid positions)
        self.shelves = {}
        # Initialize shelf objects (Shelf instances)
        self.shelf_objects = {}  # shelf_id -> Shelf
        
        # Define warehouse layout - either from CSV or generated
        if csv_file and os.path.exists(csv_file):
            self._load_from_csv(csv_file)
        else:
            # Define warehouse layout - order matters to avoid conflicts
            # Guard against too-small dimensions causing random-range errors
            try:
                self._create_walls()
                self._create_charging_zones()
                self._create_idle_zones()
                self._create_dropoff_stations()
                self._create_shelves()  # Create shelves last to avoid conflicts
            except ValueError as e:
                # Fallback: initialize empty layout without randomized placements
                print(f"WARNING: Minimal map dimensions prevented procedural layout ({e}); using empty layout")
        
        # Define key locations (dynamic based on size)
        self.shelf_positions = self._get_shelf_positions()
        self.dropoff_stations = self._get_dropoff_stations()
        self.idle_zones = self._get_idle_zones()
        self.start_position = self._get_start_position()

        # Create Shelf objects for each shelf cell
        for shelf_id, (x, y) in self.shelves.items():
            world_x = x * self.grid_size + self.grid_size / 2
            world_y = y * self.grid_size + self.grid_size / 2
            self.shelf_objects[shelf_id] = Shelf(shelf_id, (world_x, world_y))
    
    def _load_from_csv(self, csv_file: str):
        """Load warehouse layout from CSV file.
        
        CSV format:
        - Each cell represents a warehouse grid cell
        - Values: '.' or '0' = free space, 'w' or '1' = wall, 's' or '2' = shelf, 
                 'c' or '3' = charging, 'i' or '4' = idle zone, 'd' or '5' = drop-off
        - Lane/junction encodings (e.g., 'ls', 'lw', 'lne', 'js', etc.) are treated as walkable
        
        Shelf ID Convention:
        - Shelves are named using their grid coordinates: shelf_{x}_{y}
        - This provides meaningful, human-readable shelf identification
        - Example: shelf_3_3 is located at grid position (3, 3)
        
        Future Adapter Pattern Implementation:
        If you need to support legacy shelf ID formats (e.g., shelf_1, shelf_2) in the future,
        you can implement an adapter pattern here:
        
        ```python
        class ShelfIDAdapter:
            def __init__(self, warehouse_map):
                self.warehouse_map = warehouse_map
                self.legacy_id_map = {}  # legacy_id -> coordinate_id mapping
                
            def get_shelf_position(self, shelf_id: str) -> Tuple[float, float]:
                if shelf_id.startswith("shelf_") and "_" in shelf_id:
                    # Coordinate-based ID (e.g., shelf_3_3)
                    return self.warehouse_map.get_shelf_position(shelf_id)
                else:
                    # Legacy ID (e.g., shelf_1) - convert to coordinate-based
                    coordinate_id = self.legacy_id_map.get(shelf_id)
                    if coordinate_id:
                        return self.warehouse_map.get_shelf_position(coordinate_id)
                    else:
                        raise ValueError(f"Unknown shelf ID format: {shelf_id}")
        ```
        """
        try:
            with open(csv_file, 'r', newline='', encoding='utf-8-sig') as file:
                reader = csv.reader(file)
                rows = list(reader)
            
            if not rows:
                raise ValueError("CSV file is empty")
            
            # Mapping from CSV values to grid values
            value_map = {
                '.': 0, '0': 0, '': 0,  # Free space
                'w': 1, '1': 1,         # Wall
                's': 2, '2': 2,         # Shelf
                'c': 3, '3': 3,         # Charging zone
                'i': 4, '4': 4,         # Idle zone
                'd': 5, '5': 5          # Drop-off station
            }
            lane_prefixes = ('l', 'j')  # Lane and junction prefixes
            
            # Load the grid
            for y, row in enumerate(rows):
                if y >= self.height:
                    break
                for x, cell_value in enumerate(row):
                    if x >= self.width:
                        break
                    
                    # Clean and normalize cell value, handle BOM and other encoding issues
                    cell_value = str(cell_value).strip().lower()
                    # Remove BOM and other invisible characters
                    cell_value = ''.join(char for char in cell_value if ord(char) >= 32)
                    
                    if cell_value in value_map:
                        self.grid[y, x] = value_map[cell_value]
                        # Create shelf entry if it's a shelf
                        if value_map[cell_value] == 2:  # Shelf
                            # Use grid coordinates for shelf IDs when loading from CSV
                            # This makes shelf IDs more meaningful and matches demo expectations
                            shelf_id = f"shelf_{x}_{y}"
                            self.shelves[shelf_id] = (x, y)
                    elif cell_value.startswith(lane_prefixes):
                        # Treat all lane/junction encodings as walkable (free space)
                        self.grid[y, x] = 0
                    else:
                        # Default to free space for unknown values
                        self.grid[y, x] = 0
                        if cell_value:  # Only warn for non-empty unknown values
                            print(f"Warning: Unknown cell value '{cell_value}' at ({x}, {y}), treating as free space")
            print(f"Successfully loaded warehouse layout from {csv_file}")
            print(f"Dimensions: {self.width}x{self.height}")
            print(f"Shelves: {len(self.shelves)}")
            print(f"Walls: {np.sum(self.grid == 1)}")
            print(f"Charging zones: {np.sum(self.grid == 3)}")
            print(f"Idle zones: {np.sum(self.grid == 4)}")
            print(f"Drop-off stations: {np.sum(self.grid == 5)}")
            
        except Exception as e:
            print(f"Error loading CSV file {csv_file}: {e}")
            print("Falling back to generated warehouse layout")
            # Fall back to generated layout
            self._create_walls()
            self._create_charging_zones()
            self._create_idle_zones()
            self._create_dropoff_stations()
            self._create_shelves()
    
    def save_to_csv(self, csv_file: str):
        """Save current warehouse layout to CSV file.
        
        Saves the warehouse grid in a human-readable format with letters.
        """
        try:
            # Mapping from grid values to CSV characters
            char_map = {
                0: '.',  # Free space
                1: 'w',  # Wall
                2: 's',  # Shelf
                3: 'c',  # Charging zone
                4: 'i',  # Idle zone
                5: 'd'   # Drop-off station
            }
            
            with open(csv_file, 'w', newline='') as file:
                writer = csv.writer(file)
                for y in range(self.height):
                    row = []
                    for x in range(self.width):
                        cell_value = self.grid[y, x]
                        row.append(char_map.get(cell_value, '.'))
                    writer.writerow(row)
            
            print(f"Warehouse layout saved to {csv_file}")
            
        except Exception as e:
            print(f"Error saving CSV file {csv_file}: {e}")
    
    def _create_walls(self):
        """Create 3 random wall cells instead of border walls."""
        # No border walls - remove the old wall creation code
        
        # Create 3 random wall cells
        wall_count = 0
        attempts = 0
        max_attempts = 100  # Prevent infinite loop
        
        while wall_count < 3 and attempts < max_attempts:
            # Generate random position
            x = random.randint(1, self.width - 2)  # Avoid edges
            y = random.randint(1, self.height - 2)  # Avoid edges
            
            # Check if position is empty
            if self.grid[y, x] == 0:
                self.grid[y, x] = 1  # Wall cell
                wall_count += 1
            
            attempts += 1

    def _create_shelves(self):
        """Create shelf lines with 2-cell-wide aisles between them, leaving 2 cells walkable at sides."""
        # Start from edge since there are no border walls, but leave some space
        shelf_start_x = 2  # Leave 2 cells walkable on left side
        shelf_end_x = self.width - 2  # Leave 2 cells walkable on right side
        shelf_start_y = 1
        shelf_end_y = self.height - 3  # Reserve bottom 2 rows for idle zones

        # Parameters for shelf/aisle pattern
        shelf_line_width = 1  # 1 cell of shelves
        aisle_width = 2       # 2 cells of aisle (meets requirement)

        y = shelf_start_y
        while y < shelf_end_y:
            # Place shelf line
            for x in range(shelf_start_x, shelf_end_x):
                if self.grid[y, x] == 0:  # Only place if not already occupied
                    # Use coordinate-based shelf IDs for consistency with CSV loading
                    # This ensures both CSV and generated warehouses use the same naming convention
                    shelf_id = f"shelf_{x}_{y}"
                    self.shelves[shelf_id] = (x, y)
                    self.grid[y, x] = 2  # Shelf cell
            
            # Move to next shelf line (skip aisle)
            y += shelf_line_width + aisle_width
        
    def _create_charging_zones(self):
        """Create charging zones."""
        # Charging zones (represented as 3 in grid)
        if self.width >= 3 and self.height >= 2:
            # Place charging zones in a safe area (avoid conflicts with walls)
            for attempt_y in range(1, min(3, self.height - 1)):
                for attempt_x in range(1, min(4, self.width - 1)):
                    if self.grid[attempt_y, attempt_x] == 0:
                        self.grid[attempt_y, attempt_x] = 3
                        if attempt_x + 1 < self.width - 1 and self.grid[attempt_y, attempt_x + 1] == 0:
                            self.grid[attempt_y, attempt_x + 1] = 3
                        return  # Exit after placing charging zones
        
    def _create_idle_zones(self):
        """Create idle zones in a line instead of 2x2."""
        # Idle zones (represented as 4 in grid)
        if self.width >= 8 and self.height >= 6:
            # Create idle zones in a horizontal line at the bottom
            # Reserve the bottom 2 rows for idle zones and free space
            idle_y = self.height - 2  # Second to last row
            idle_start_x = 1
            idle_count = min(5, self.width - 2)  # Create up to 5 idle zones in a line
            
            placed_count = 0
            for x in range(idle_start_x, idle_start_x + idle_count):
                if x < self.width - 1 and self.grid[idle_y, x] == 0:  # Check bounds and availability
                    self.grid[idle_y, x] = 4
                    placed_count += 1
                    
            # If we couldn't place enough in the bottom row, try the row above
            if placed_count < 3:
                idle_y = self.height - 3
                for x in range(idle_start_x, idle_start_x + idle_count):
                    if x < self.width - 1 and self.grid[idle_y, x] == 0 and placed_count < 5:
                        self.grid[idle_y, x] = 4
                        placed_count += 1
        
    def _create_dropoff_stations(self):
        """Create drop-off stations where shelves are delivered."""
        # Drop-off stations (represented as 5 in grid)
        if self.width >= 8 and self.height >= 6:
            # Create drop-off station on the right side
            dropoff_x = self.width - 2
            dropoff_y = self.height // 2
            
            # Ensure it's not overlapping with other elements
            if self.grid[dropoff_y, dropoff_x] == 0:
                self.grid[dropoff_y, dropoff_x] = 5
            else:
                # Find alternative position
                for y in range(1, self.height - 1):
                    if self.grid[y, dropoff_x] == 0:
                        self.grid[y, dropoff_x] = 5
                        break
        
    def _get_dropoff_stations(self) -> List[Tuple[float, float]]:
        """Get drop-off station positions."""
        stations = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 5:  # Drop-off station
                    # Return the center of the cell instead of the corner
                    world_x = (x + 0.5) * self.grid_size
                    world_y = (y + 0.5) * self.grid_size
                    stations.append((world_x, world_y))
        return stations
    
    def _get_charging_zones(self) -> List[Tuple[float, float]]:
        """Get charging zone positions."""
        charging_positions = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 3:  # Charging zone
                    # Return the center of the cell instead of the corner
                    world_x = (x + 0.5) * self.grid_size
                    world_y = (y + 0.5) * self.grid_size
                    charging_positions.append((world_x, world_y))
        return charging_positions
        
    def _get_idle_zones(self) -> List[Tuple[float, float]]:
        """Get idle zone positions."""
        idle_positions = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 4:  # Idle zone
                    # Return the center of the cell instead of the corner
                    world_x = (x + 0.5) * self.grid_size
                    world_y = (y + 0.5) * self.grid_size
                    idle_positions.append((world_x, world_y))
        return idle_positions
        
    def _get_shelf_positions(self) -> List[Tuple[float, float]]:
        """Get list of shelf positions."""
        shelf_positions = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 2:
                    # Return the center of the cell instead of the corner
                    world_x = (x + 0.5) * self.grid_size
                    world_y = (y + 0.5) * self.grid_size
                    shelf_positions.append((world_x, world_y))
        return shelf_positions

    def get_shelf_position(self, shelf_id: str) -> Optional[Tuple[float, float]]:
        """
        Get the world position of a specific shelf by its ID.

        Args:
            shelf_id: Shelf identifier in format "shelf_x_y"

        Returns:
            Optional[Tuple[float, float]]: Shelf world position or None if not found
        """
        if shelf_id in self.shelves:
            grid_x, grid_y = self.shelves[shelf_id]
            # Convert grid coordinates to world coordinates (center of cell)
            world_x = (grid_x + 0.5) * self.grid_size
            world_y = (grid_y + 0.5) * self.grid_size
            return (world_x, world_y)
        return None
    
    def is_walkable(self, x: float, y: float) -> bool:
        """Check if a position is walkable (not wall or shelf)."""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        
        if (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return self.grid[grid_y, grid_x] in [0, 3, 4, 5]  # Free space, charging, idle, or drop-off
        return False
    
    def is_shelf(self, x: float, y: float) -> bool:
        """Check if a position contains a shelf."""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        
        if (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return self.grid[grid_y, grid_x] == 2
        return False
    
    def is_charging_zone(self, x: float, y: float) -> bool:
        """Check if a position is a charging zone."""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        
        if (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return self.grid[grid_y, grid_x] == 3
        return False
    
    def is_idle_zone(self, x: float, y: float) -> bool:
        """Check if a position is an idle zone."""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        
        if (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return self.grid[grid_y, grid_x] == 4
        return False
    
    def is_dropoff_station(self, x: float, y: float) -> bool:
        """Check if a position is a drop-off station."""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        
        if (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return self.grid[grid_y, grid_x] == 5
        return False
    
    def get_neighbors(self, x: float, y: float) -> List[Tuple[float, float]]:
        """Get walkable neighbors of a position."""
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), 
                     (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        for dx, dy in directions:
            new_x = x + dx * self.grid_size
            new_y = y + dy * self.grid_size
            if self.is_walkable(new_x, new_y):
                neighbors.append((new_x, new_y))
        
        return neighbors
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        # Ensure we're working with floats, not Decimal
        x, y = float(x), float(y)
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        # Ensure we're working with ints/floats, not Decimal
        grid_x, grid_y = int(grid_x), int(grid_y)
        world_x = (grid_x + 0.5) * self.grid_size
        world_y = (grid_y + 0.5) * self.grid_size
        return (world_x, world_y)
    
    def get_obstacle_cells(self) -> List[Tuple[int, int]]:
        """Get list of obstacle cells (walls and shelves) in grid coordinates."""
        obstacles = []
        for y in range(self.height):
            for x in range(self.width):
                cell_type = self.grid[y, x]
                if cell_type in [1, 2]:  # Walls and shelves are obstacles
                    obstacles.append((x, y))
        return obstacles
    
    def _get_start_position(self) -> Tuple[float, float]:
        """Get start position at southeast corner of warehouse."""
        # Southeast corner position - start from the southeast and work inward
        southeast_candidates = []
        
        # Try positions starting from the southeast corner and moving inward
        for y_offset in range(1, 4):  # Try 1-3 cells from the edge
            for x_offset in range(1, 4):  # Try 1-3 cells from the edge
                # Calculate position from southeast corner
                x = self.width - x_offset
                y = self.height - y_offset
                
                # Convert to world coordinates
                world_x, world_y = self.grid_to_world(x, y)
                
                # Check if walkable
                if self.is_walkable(world_x, world_y):
                    southeast_candidates.append((world_x, world_y))
        
        # Return the first (closest to southeast corner) candidate
        if southeast_candidates:
            return southeast_candidates[0]
        
        # Fallback to original logic if southeast corner isn't available
        if self.width >= 8:
            # For larger warehouses, try positions near the charging zone
            candidate_positions = [
                (1.0, 1.0),  # Near charging zone
                (1.0, 2.0),  # Near charging zone
                (2.0, 1.0),  # Alternative position
                (3.0, 1.0),  # Alternative position
            ]
            
            for pos in candidate_positions:
                if self.is_walkable(*pos):
                    return pos
            
            # If none of the preferred positions work, find any walkable position
            for y in range(1, self.height - 1):
                for x in range(1, self.width - 1):
                    world_x = x * self.grid_size
                    world_y = y * self.grid_size
                    if self.is_walkable(world_x, world_y):
                        return (world_x, world_y)
        else:
            # Small warehouse - start in free area
            return (1.0, 1.0)
        
        # Final fallback
        return (1.0, 1.0) 