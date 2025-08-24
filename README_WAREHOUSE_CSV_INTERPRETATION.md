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
- `d` or `5` = **Drop-off station** (is a graph node with restricted routing; see routing rules below)

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

The `GraphGeneratorImpl` processes the CSV in multiple phases:

#### Phase 1: Preprocessing
- Scan the grid to identify all idle/charging cells (`i`, `4`, `c`, `3`)
- Mark lane cells (`l*`) that are 4-neighbor adjacent to any parking cell
- These parking-adjacent lanes will become single-cell conflict boxes

#### Phase 2: Node Creation
```python
# Skip non-navigation cells
if cell_value in ['w']:  # walls only
    continue

# Process lane cells
if cell_value.startswith('l'):
    directions = parse_directions(cell_value[1:])
    is_parking_adjacent = check_4_neighbor_parking_adjacency()
    if is_parking_adjacent:
        # Create single-cell conflict box with cb_p_r_c identifier
        create_conflict_box_node(directions, is_conflict_box=True)
    else:
        # Create regular lane node
        create_lane_node(directions, is_conflict_box=False)

# Process junction cells  
if cell_value.startswith('j'):
    directions = parse_directions(cell_value[1:])
    # Create conflict box node (will be grouped later)
    create_conflict_box_node(directions, is_conflict_box=True)

# Process parking zones
if cell_value in ['i', '4', 'c', '3']:
    # Create graph node for idle/charging zones
    create_parking_node()

# Process drop-off zones
if cell_value == 'd':
    # Create graph node for drop-off zones (restricted routing)
    create_dropoff_node()
```

#### Phase 3: Conflict Box Grouping
- Group adjacent junction cells (4-neighbor) into connected components
- Assign `cb_*` identifiers to grouped junction boxes
- Parking-adjacent lanes retain their individual `cb_p_r_c` identifiers

### 2. Special Zone Handling

- Idle (`i`) and Charging (`c`) zones are included as graph nodes. They are connected bidirectionally to adjacent lane/junction nodes for natural starts/ends.
- Drop-off (`d`) zones are included as graph nodes with unidirectional connectivity to adjacent lanes/junctions.
- These zone nodes are treated as non-through nodes: the planner forbids using them as intermediate waypoints. They are allowed only as start or goal.

Alternate policy: Instead of forbidding traversal through zone nodes, apply a high traversal cost to edges entering these nodes so the planner avoids them except when necessary (start/goal). This can be enabled later if desired.

#### Drop-off Zone Connectivity Rules
Drop-off zones (`d` cells) are connected to adjacent lane/junction nodes with specific rules to prevent routing loops:

- **Unidirectional edges**: Drop-off zones only have outbound edges to adjacent lanes/junctions
- **Direction-based filtering**: A drop-off zone connects to a neighbor only if the neighbor's encoded directions do NOT point back to the drop-off
- **Example**: If `dropoff_12_1` is adjacent to `lane_11_1` (marked as `ln` for North), and North points toward the drop-off, then no edge is created from drop-off to lane
- **Result**: Robots can route TO drop-off zones efficiently, but cannot create circular routes through drop-offs

### 3. Conflict Box Processing

The system processes conflict boxes in two categories:

#### Junction Conflict Boxes (`j*` cells)
- Adjacent junction cells (4-neighbor connectivity) are grouped into single conflict boxes
- Each group becomes a `ConflictBox` object with `is_conflict_box=True` flag
- Grouped boxes use `cb_*` identifiers and cover the entire connected component
- These require **locking mechanism** for multi-robot coordination

#### Parking-Adjacent Lane Conflict Boxes
- Lane cells (`l*`) adjacent to idle/charging zones are automatically marked as single-cell conflict boxes
- Each parking-adjacent lane gets its own `ConflictBox` with `cb_p_r_c` identifier (r=row, c=col)
- These cells have `is_conflict_box=True` flag and `size = cell_size`
- Single-cell boxes prevent parking-adjacent lanes from being merged into large intersection areas
- Robots must acquire locks before entering any conflict box (grouped or single-cell)

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

**Conflict Box Analysis:**
- Junction cells in rows 5-6, 8-9, 11-12 are grouped by 4-neighbor connectivity
- Lane cells adjacent to idle zones (row 13) become single-cell conflict boxes
- Example: `l*` cell at `(4,12)` adjacent to idle zone `(4,13)` becomes conflict box `cb_p_4_12`

**Destination Example:**
- Charging zone at `(8,12)`: Path planner finds closest navigation node (e.g., `(7,12)`)
- Robot navigates to `(7,12)`, then moves to actual destination `(8,12)`
- For return journey: Robot starts from `(7,12)` (closest node) to next destination

**Drop-off Zone Analysis:**
- Drop-off zone at `(1,12)`: Becomes graph node `dropoff_12_1` at world position (0.75, 6.25)
- Connects to `lane_12_2` (East neighbor) because lane_12_2 has `ln` (North) direction, not pointing back to drop-off
- Does NOT connect to `lane_11_1` (North neighbor) because lane_11_1 has `ln` (North) direction pointing toward drop-off
- Robots can route TO drop-off via lane directions, and FROM drop-off via the unidirectional edge to `lane_12_2`

## Integration with Robot Navigation

### 1. Path Planning
- Uses `NavigationGraph` with nodes and edges
- Dijkstra's algorithm for shortest path calculation
- Respects conflict box locking
- Handles blocked cells dynamically

### 2. Destination Handling
The path planner handles destinations through **direct graph routing** for special zones and **closest node approximation** for other destinations:

- **Special zones (idle, charging, drop-off)**: Are graph nodes, so robots route directly to/from them
- **Other destinations**: Path planner finds the closest navigation node to the destination point
- **Routing restrictions**: Special zone nodes cannot be used as intermediate waypoints in routes
- **Fallback behavior**: If no nearby navigation node exists, path planning fails gracefully

### 3. Lane Following
- Robots follow lane directions during navigation
- All conflict boxes (grouped junctions and single-cell parking-adjacent) require lock acquisition
- Lock acquisition follows the same protocol regardless of box type
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
- **Box types**: System automatically handles both grouped junction boxes (`cb_*`) and single-cell parking-adjacent boxes (`cb_p_*`)

### Navigation Settings
- **Graph cache TTL**: Configurable for performance optimization
- **Blocked cells cache TTL**: Configurable for real-time updates
- **Conflict box penalty**: Configurable penalty for routing through conflict boxes

## Future Improvements

1. **Enhanced Validation**: Better error checking for CSV format
2. **Dynamic Zones**: Support for runtime zone changes
3. **Performance Optimization**: Efficient graph traversal algorithms
4. **Advanced Routing**: Multi-objective path planning (distance, time, energy)
5. **Conflict Box Optimization**: Fine-tune grouping algorithms for complex intersection patterns
6. **Parking Zone Integration**: Enhanced coordination between parking-adjacent lanes and zone access
7. **Advanced Drop-off Routing**: Enhanced algorithms for drop-off zone access patterns and traffic flow optimization 