#!/usr/bin/env python3
"""Print lane data in readable format for verification"""

import csv
from pathlib import Path

def print_lanes(csv_path: Path):
    """Print lanes in a readable format"""
    print("=" * 60)
    print("LANES")
    print("=" * 60)
    print("ID | Direction | X      | Y      | Description")
    print("-" * 60)
    
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:  # Skip empty rows
                lane_id, direction, x, y = row
                print(f"{lane_id:2} | {direction:9} | {x:6} | {y:6} | Lane cell at ({x}, {y})")

def print_boxes(csv_path: Path):
    """Print conflict boxes in a readable format"""
    print("\n" + "=" * 60)
    print("CONFLICT BOXES")
    print("=" * 60)
    print("ID | X      | Y      | Size   | Allowed Directions")
    print("-" * 60)
    
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:  # Skip empty rows
                box_id, x, y, size, directions = row
                print(f"{box_id:2} | {x:6} | {y:6} | {size:6} | {directions}")

def print_bays(csv_path: Path):
    """Print bays in a readable format"""
    print("\n" + "=" * 60)
    print("BAYS")
    print("=" * 60)
    print("ID | Type | Center X | Center Y | Gateway X | Gateway Y")
    print("-" * 60)
    
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:  # Skip empty rows
                bay_id, bay_type, cx, cy, gx, gy = row
                type_name = {"D": "Dropoff", "I": "Idle", "C": "Charging"}.get(bay_type, bay_type)
                print(f"{bay_id:2} | {type_name:7} | {cx:8} | {cy:8} | {gx:9} | {gy:9}")

def analyze_lane_distribution(lanes_csv: Path):
    """Analyze the distribution of lane directions"""
    print("\n" + "=" * 60)
    print("LANE DIRECTION ANALYSIS")
    print("=" * 60)
    
    directions = {}
    with open(lanes_csv, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if row:
                direction = row[1]
                directions[direction] = directions.get(direction, 0) + 1
    
    total = sum(directions.values())
    print(f"Total lanes: {total}")
    for direction, count in sorted(directions.items()):
        percentage = (count / total) * 100
        print(f"{direction}: {count:3} lanes ({percentage:5.1f}%)")

def main():
    output_dir = Path("./manual_lanes_output")
    
    lanes_file = output_dir / "lanes.csv"
    boxes_file = output_dir / "boxes.csv"
    bays_file = output_dir / "bays.csv"
    
    if not lanes_file.exists():
        print("Error: lanes.csv not found!")
        return
    
    print_lanes(lanes_file)
    print_boxes(boxes_file)
    print_bays(bays_file)
    analyze_lane_distribution(lanes_file)
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    # Count lanes
    with open(lanes_file, 'r', encoding='utf-8') as f:
        lane_count = len([row for row in csv.reader(f) if row])
    
    # Count boxes
    with open(boxes_file, 'r', encoding='utf-8') as f:
        box_count = len([row for row in csv.reader(f) if row])
    
    # Count bays
    with open(bays_file, 'r', encoding='utf-8') as f:
        bay_count = len([row for row in csv.reader(f) if row])
    
    print(f"Total Lanes: {lane_count}")
    print(f"Total Boxes: {box_count}")
    print(f"Total Bays:  {bay_count}")

if __name__ == "__main__":
    main() 