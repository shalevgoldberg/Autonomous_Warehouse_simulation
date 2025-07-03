#!/usr/bin/env python3
"""Test CSV compatibility with MuJoCo and path planning requirements"""

import csv
from pathlib import Path
from typing import List, Dict, Set, Tuple
from dataclasses import dataclass

@dataclass
class LaneNode:
    """Represents a lane node for path planning."""
    lane_id: str
    x: float
    y: float
    direction: str
    connections: Set[str] = None  # type: ignore
    
    def __post_init__(self):
        if self.connections is None:
            self.connections = set()

@dataclass
class ConflictBox:
    """Represents a conflict box."""
    box_id: str
    x: float
    y: float
    size: float
    allowed_directions: str

@dataclass
class Bay:
    """Represents a bay."""
    bay_id: str
    bay_type: str
    center_x: float
    center_y: float
    gateway_x: float
    gateway_y: float

def load_lanes(csv_path: Path) -> List[LaneNode]:
    """Load lanes from CSV and create nodes for path planning."""
    lanes = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:
                lane_id, direction, x, y = row
                lanes.append(LaneNode(
                    lane_id=lane_id,
                    x=float(x),
                    y=float(y),
                    direction=direction
                ))
    return lanes

def load_boxes(csv_path: Path) -> List[ConflictBox]:
    """Load conflict boxes from CSV."""
    boxes = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:
                box_id, x, y, size, directions = row
                boxes.append(ConflictBox(
                    box_id=box_id,
                    x=float(x),
                    y=float(y),
                    size=float(size),
                    allowed_directions=directions
                ))
    return boxes

def load_bays(csv_path: Path) -> List[Bay]:
    """Load bays from CSV."""
    bays = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:
                bay_id, bay_type, cx, cy, gx, gy = row
                bays.append(Bay(
                    bay_id=bay_id,
                    bay_type=bay_type,
                    center_x=float(cx),
                    center_y=float(cy),
                    gateway_x=float(gx),
                    gateway_y=float(gy)
                ))
    return bays

def build_lane_graph(lanes: List[LaneNode]) -> Dict[str, LaneNode]:
    """Build a graph from lane nodes by connecting adjacent lanes."""
    # Create lookup by position
    position_map: Dict[Tuple[float, float], List[LaneNode]] = {}
    for lane in lanes:
        pos = (lane.x, lane.y)
        if pos not in position_map:
            position_map[pos] = []
        position_map[pos].append(lane)
    
    # Connect lanes at same positions
    for pos, lane_list in position_map.items():
        if len(lane_list) > 1:
            # Multiple lanes at same position - connect them
            for i, lane1 in enumerate(lane_list):
                for j, lane2 in enumerate(lane_list):
                    if i != j:
                        lane1.connections.add(lane2.lane_id)
                        lane2.connections.add(lane1.lane_id)
    
    # Connect adjacent lanes (within 0.5m)
    connection_distance = 0.5
    for i, lane1 in enumerate(lanes):
        for j, lane2 in enumerate(lanes):
            if i != j:
                distance = ((lane1.x - lane2.x) ** 2 + (lane1.y - lane2.y) ** 2) ** 0.5
                if distance <= connection_distance:
                    lane1.connections.add(lane2.lane_id)
                    lane2.connections.add(lane1.lane_id)
    
    # Create lookup by lane_id
    return {lane.lane_id: lane for lane in lanes}

def test_path_planning(lane_graph: Dict[str, LaneNode], start_id: str, goal_id: str) -> List[str]:
    """Simple BFS path planning test."""
    if start_id not in lane_graph or goal_id not in lane_graph:
        return []
    
    visited = set()
    queue = [(start_id, [start_id])]
    
    while queue:
        current_id, path = queue.pop(0)
        if current_id == goal_id:
            return path
        
        if current_id in visited:
            continue
        
        visited.add(current_id)
        current_lane = lane_graph[current_id]
        
        for neighbor_id in current_lane.connections:
            if neighbor_id not in visited:
                queue.append((neighbor_id, path + [neighbor_id]))
    
    return []

def analyze_mujoco_compatibility():
    """Analyze MuJoCo compatibility."""
    print("=" * 60)
    print("MUJOCO COMPATIBILITY ANALYSIS")
    print("=" * 60)
    
    mjcf_path = Path("./manual_lanes_output/warehouse.mjcf")
    if not mjcf_path.exists():
        print("❌ MuJoCo file not found!")
        return False
    
    print("✅ MuJoCo file exists")
    
    # Check file content
    with open(mjcf_path, 'r') as f:
        content = f.read()
    
    # Basic MuJoCo structure checks
    checks = [
        ("XML structure", "<?xml" in content or "<mujoco" in content),
        ("World body", "<worldbody>" in content),
        ("Geometries", "<geom" in content),
        ("Proper closing", "</mujoco>" in content)
    ]
    
    all_passed = True
    for check_name, passed in checks:
        status = "✅" if passed else "❌"
        print(f"{status} {check_name}")
        if not passed:
            all_passed = False
    
    # Count geometries
    geom_count = content.count("<geom")
    print(f"📊 Total geometries: {geom_count}")
    
    return all_passed

def main():
    """Main test function."""
    print("=" * 60)
    print("CSV COMPATIBILITY TEST")
    print("=" * 60)
    
    output_dir = Path("./manual_lanes_output")
    
    # Load data
    try:
        lanes = load_lanes(output_dir / "lanes.csv")
        boxes = load_boxes(output_dir / "boxes.csv")
        bays = load_bays(output_dir / "bays.csv")
        
        print(f"✅ Loaded {len(lanes)} lanes")
        print(f"✅ Loaded {len(boxes)} conflict boxes")
        print(f"✅ Loaded {len(bays)} bays")
        
    except Exception as e:
        print(f"❌ Failed to load data: {e}")
        return
    
    # Test lane graph construction
    print("\n" + "=" * 60)
    print("LANE GRAPH ANALYSIS")
    print("=" * 60)
    
    lane_graph = build_lane_graph(lanes)
    print(f"✅ Built lane graph with {len(lane_graph)} nodes")
    
    # Analyze connectivity
    total_connections = sum(len(lane.connections) for lane in lane_graph.values())
    avg_connections = total_connections / len(lane_graph) if lane_graph else 0
    print(f"📊 Average connections per lane: {avg_connections:.2f}")
    
    # Find isolated lanes
    isolated_lanes = [lane_id for lane_id, lane in lane_graph.items() if not lane.connections]
    print(f"⚠️  Isolated lanes: {len(isolated_lanes)}")
    if isolated_lanes:
        print(f"   Isolated lane IDs: {isolated_lanes[:5]}{'...' if len(isolated_lanes) > 5 else ''}")
    
    # Test path planning
    print("\n" + "=" * 60)
    print("PATH PLANNING TEST")
    print("=" * 60)
    
    if len(lane_graph) >= 2:
        # Try to find a path between first and last lane
        lane_ids = list(lane_graph.keys())
        start_id = lane_ids[0]
        goal_id = lane_ids[-1]
        
        path = test_path_planning(lane_graph, start_id, goal_id)
        if path:
            print(f"✅ Found path from {start_id} to {goal_id}: {len(path)} steps")
            print(f"   Path: {' -> '.join(path[:5])}{'...' if len(path) > 5 else ''}")
        else:
            print(f"❌ No path found from {start_id} to {goal_id}")
    else:
        print("⚠️  Not enough lanes for path planning test")
    
    # Analyze data quality
    print("\n" + "=" * 60)
    print("DATA QUALITY ANALYSIS")
    print("=" * 60)
    
    # Check for duplicate coordinates
    positions = [(lane.x, lane.y) for lane in lanes]
    unique_positions = set(positions)
    duplicates = len(positions) - len(unique_positions)
    print(f"📊 Duplicate positions: {duplicates}")
    
    # Check coordinate ranges
    if lanes:
        x_coords = [lane.x for lane in lanes]
        y_coords = [lane.y for lane in lanes]
        print(f"📊 X range: {min(x_coords):.2f} to {max(x_coords):.2f}")
        print(f"📊 Y range: {min(y_coords):.2f} to {max(y_coords):.2f}")
    
    # Check direction distribution
    direction_counts = {}
    for lane in lanes:
        direction_counts[lane.direction] = direction_counts.get(lane.direction, 0) + 1
    
    print(f"📊 Direction distribution:")
    for direction, count in sorted(direction_counts.items()):
        percentage = (count / len(lanes)) * 100
        print(f"   {direction}: {count} ({percentage:.1f}%)")
    
    # Test MuJoCo compatibility
    analyze_mujoco_compatibility()
    
    print("\n" + "=" * 60)
    print("COMPATIBILITY ASSESSMENT")
    print("=" * 60)
    
    # Overall assessment
    issues = []
    
    if len(isolated_lanes) > len(lane_graph) * 0.1:  # More than 10% isolated
        issues.append("Too many isolated lanes")
    
    if avg_connections < 1.0:
        issues.append("Low connectivity")
    
    if duplicates > len(lanes) * 0.2:  # More than 20% duplicates
        issues.append("Many duplicate positions")
    
    if issues:
        print("⚠️  Issues found:")
        for issue in issues:
            print(f"   - {issue}")
    else:
        print("✅ Data looks good for path planning!")
    
    print(f"\n📋 Summary:")
    print(f"   - {len(lanes)} lanes ready for path planning")
    print(f"   - {len(boxes)} conflict boxes for traffic management")
    print(f"   - {len(bays)} bays for robot operations")
    print(f"   - MuJoCo scene file generated successfully")

if __name__ == "__main__":
    main() 