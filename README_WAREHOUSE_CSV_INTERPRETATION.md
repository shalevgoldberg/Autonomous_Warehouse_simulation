# Warehouse CSV Interpretation Guide

## Overview

This document explains how warehouse CSV files are interpreted and processed by the autonomous warehouse system. The CSV represents a 2D grid where each cell contains encoded information about the warehouse layout, navigation paths, and special zones.

## CSV Structure

The CSV file represents a **rectangular grid** where:
- **North is at Row 0** (top of the file)
- **Each cell is configurable size** (default: 0.5m × 0.5m) in the physical world
- **World coordinates**: (0,0) at top-left, positive X right, positive Y down
- **Grid dimensions**: Configurable via warehouse map configuration

## Cell Value Encoding

### Basic Elements
- `w` = **Wall** (non-walkable, blocks navigation)
- `s` = **Shelf** (non-walkable, storage location)

### Navigation Elements
- `l*` = **Lane cell** (walkable navigation path)
- `j*` = **Junction cell** (conflict box intersection)

### Special Zones
- `c` or `3` = **Charging zone** (is a graph node; see routing rules below)
- `i` or `4` = **Idle zone** (is a graph node; see routing rules below)
- `d` or `5` = **Drop-off station** (destination; currently not a graph node)

## Direction Encoding

### Lane Cells (l*)
The character after `l` indicates the **exit directions** from that cell:

- `ls` = Lane with **South** exit
- `lw` = Lane with **West** exit
- `le` = Lane with **East** exit
- `ln` = Lane with **North** exit
- `lne` = Lane with **North** and **East** exits
- `les` = Lane with **East** and **South** exits
- `lse` = Lane with **South** and **East** exits

### Junction Cells (j*)
The character after `j` indicates the **exit directions** from that conflict box:

- `js` = Junction with **South** exit
- `jn` = Junction with **North** exit
- `je` = Junction with **East** exit
- `jnw` = Junction with **North** and **West** exits
- `jws` = Junction with **West** and **South** exits
- `jse` = Junction with **South** and **East** exits
- `jen` = Junction with **East** and **North** exits
- `jes` = Junction with **East** and **South** exits
- `jne` = Junction with **North** and **East** exits

## Processing Logic

### 1. Navigation Graph Generation

The `GraphGeneratorImpl` processes the CSV as follows:

```python
# Skip non-navigation cells
if cell_value in ['w', 'd', 'i', 'c']:  # walls, dock, idle, charging
    continue  # These are NOT navigation nodes

# Process lane cells
if cell_value.startswith('l'):
    directions = parse_directions(cell_value[1:])
    # Create navigation node with directions

# Process junction cells  
if cell_value.startswith('j'):
    directions = parse_directions(cell_value[1:])
    # Create conflict box node with directions
```

### 2. Special Zone Handling

- Idle (`i`) and Charging (`c`) zones are included as graph nodes. They are connected bidirectionally to adjacent lane/junction nodes for natural starts/ends.
- These zone nodes are treated as non-through nodes: the planner forbids using them as intermediate waypoints. They are allowed only as start or goal.
- Drop-off (`d`) remains destination-only (not a graph node) in this iteration. It can be promoted similarly later.

Alternate policy: Instead of forbidding traversal through zone nodes, apply a high traversal cost to edges entering these nodes so the planner avoids them except when necessary (start/goal). This can be enabled later if desired.

### 3. Conflict Box Processing

Conflict boxes (`j*` cells) are processed specially:

- Each junction cell becomes a `ConflictBox` object
- The cell gets `is_conflict_box=True` flag
- Conflict boxes require **locking mechanism** for multi-robot coordination
- Robots must acquire locks before entering conflict boxes

### 4. Direction Parsing

The `_parse_directions()` method converts direction strings to `LaneDirection` enums:

```python
def _parse_directions(direction_str: str) -> List[LaneDirection]:
    directions = []
    for char in direction_str.lower():
        if char == 'n': directions.append(LaneDirection.NORTH)
        elif char == 's': directions.append(LaneDirection.SOUTH)
        elif char == 'e': directions.append(LaneDirection.EAST)
        elif char == 'w': directions.append(LaneDirection.WEST)
    return directions
```

## Coordinate System

### Grid Coordinates
- **Grid**: (0,0) at top-left, positive X right, positive Y down
- **Cell size**: 0.5m × 0.5m

### World Coordinates  
- **World**: (0,0) at top-left, positive X right, positive Y down
- **Conversion**: `world_x = grid_x * 0.5`, `world_y = grid_y * 0.5`

### Example
- Grid position (2, 3) = World position (1.0m, 1.5m)



## Validation Rules

### Walkable Cells
A position is considered walkable if the grid cell contains:
- `0` = Free space
- `3` = Charging zone  
- `4` = Idle zone
- `5` = Drop-off station
- Any `l*` = Lane cell
- Any `j*` = Junction cell

### Non-walkable Cells
- `1` = Wall
- `2` = Shelf

## Example CSV Interpretation

```
w,w,w,w,w,w,w,w,w,w          # Row 0: All walls
w,ls,lw,lw,lw,lw,lw,lw,lw,w  # Row 1: West lane with South exit, then West lanes
w,ls,lne,le,le,les,le,le,ln,w # Row 2: Mixed directions
```

**Row 1 Analysis:**
- `(1,1)`: `ls` = Lane with South exit → connects to `(1,2)`
- `(2,1)`: `lw` = Lane with West exit → connects to `(1,1)`  
- `(3,1)`: `lw` = Lane with West exit → connects to `(2,1)`

**Destination Example:**
- Charging zone at `(8,12)`: Path planner finds closest navigation node (e.g., `(7,12)`)
- Robot navigates to `(7,12)`, then moves to actual destination `(8,12)`
- For return journey: Robot starts from `(7,12)` (closest node) to next destination

## Integration with Robot Navigation

### 1. Path Planning
- Uses `NavigationGraph` with nodes and edges
- Dijkstra's algorithm for shortest path calculation
- Respects conflict box locking
- Handles blocked cells dynamically

### 2. Destination Handling
The path planner handles destinations (charging, idle, drop-off zones) through **closest node approximation**:

- **To destinations**: Finds the closest navigation node to the destination point
- **From destinations**: Uses the same closest node as a starting point
- **No special routing**: Destinations are treated as regular world coordinates
- **Fallback behavior**: If no nearby navigation node exists, path planning fails gracefully

### 3. Lane Following
- Robots follow lane directions during navigation
- Conflict boxes require lock acquisition
- Special zones are valid destinations

### 4. Coordinate Conversion
- Robot positions in world coordinates
- Path planning uses grid coordinates
- Continuous conversion between systems

## Configuration

### Cell Size
- **Default**: 0.5m per grid cell
- **Configurable**: Via `robot.cell_size` in configuration
- **Impact**: Affects coordinate system and navigation precision

### Conflict Box Settings
- **Lock timeout**: Configurable via `robot.conflict_box_lock_timeout`
- **Heartbeat interval**: Configurable via `robot.conflict_box_heartbeat_interval`
- **Priority levels**: Configurable via `navigation.conflict_box_priority_levels`

### Navigation Settings
- **Graph cache TTL**: Configurable for performance optimization
- **Blocked cells cache TTL**: Configurable for real-time updates
- **Conflict box penalty**: Configurable penalty for routing through conflict boxes

## Future Improvements

1. **Enhanced Validation**: Better error checking for CSV format
2. **Dynamic Zones**: Support for runtime zone changes
3. **Performance Optimization**: Efficient graph traversal algorithms
4. **Advanced Routing**: Multi-objective path planning (distance, time, energy) 