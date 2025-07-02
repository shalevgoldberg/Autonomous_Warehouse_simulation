# CSV Warehouse Layout Feature

This warehouse simulation now supports loading custom warehouse layouts from CSV files, giving you complete control over the warehouse design.

## Usage

### Command Line Options

```bash
# Run with default generated layout
python main.py

# Run with sample CSV layout
python main.py --demo-csv

# Run with custom CSV file
python main.py --csv my_warehouse.csv

# Generate and save a warehouse layout to CSV
python main.py --save-csv my_layout.csv
```

### CSV Format

The CSV file represents the warehouse as a grid where each cell can contain:

- `.` or `0` = Free space (passable)
- `w` or `1` = Wall (obstacle)
- `s` or `2` = Shelf (storage)
- `c` or `3` = Charging zone
- `i` or `4` = Idle zone
- `d` or `5` = Drop-off station

### Example CSV Layout

```csv
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,c,c,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,d,.
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,.,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,.,.
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,.,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,.,.
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,.,s,s,s,s,s,s,w,s,s,s,s,s,s,s,s,s,s,s,s,s,.,.
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,.,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,.,.
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,.,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,s,.,.
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,i,i,i,i,i,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.
.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,
```

### Features

1. **Flexible Sizing**: CSV files can be any rectangular size - the warehouse will automatically resize to match
2. **Automatic Shelf Creation**: Shelf cells (`s`) automatically create shelf objects with random inventory
3. **Validation**: Invalid layouts fall back to generated layouts with error messages
4. **Statistics**: Loading shows detailed statistics about the warehouse composition
5. **Export**: Any generated warehouse can be saved to CSV for future use

### Sample Files

- `sample_warehouse.csv` - A 25x15 demonstration warehouse
- Use `--save-csv filename.csv` to create your own templates

### Design Tips

1. **Ensure Connectivity**: Make sure robots can reach all areas
2. **Aisle Width**: Keep aisles at least 2 cells wide for smooth navigation
3. **Charging Access**: Place charging zones near high-traffic areas
4. **Idle Zones**: Position idle zones away from busy shelf areas
5. **Drop-off Placement**: Put drop-off stations near warehouse exits

### Technical Details

- Grid coordinates start from (0,0) at top-left
- World coordinates place (0.75, 0.75) at the center of grid cell (1,1)
- Robots navigate using A* pathfinding on the grid
- Invalid CSV files automatically fall back to generated layouts 