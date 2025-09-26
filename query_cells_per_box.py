#!/usr/bin/env python3
"""
Query and display which cells belong to which conflict box.
"""

import os
import sys
import psycopg2
from psycopg2.extras import RealDictCursor
import json

def get_cells_per_conflict_box():
    """Query navigation graph nodes and map them to conflict boxes."""
    # Get database configuration
    db_host = os.getenv('WAREHOUSE_DB_HOST', 'localhost')
    db_port = int(os.getenv('WAREHOUSE_DB_PORT', '5432'))
    db_name = os.getenv('WAREHOUSE_DB_NAME', 'warehouse_sim')
    db_user = os.getenv('WAREHOUSE_DB_USER', 'postgres')
    db_password = os.getenv('WAREHOUSE_DB_PASSWORD')

    if not db_password:
        print("❌ WAREHOUSE_DB_PASSWORD environment variable not set!")
        print("   Set with: $env:WAREHOUSE_DB_PASSWORD='renaspolter'")
        return {}

    try:
        # Connect to database
        conn = psycopg2.connect(
            host=db_host,
            port=db_port,
            database=db_name,
            user=db_user,
            password=db_password,
            connect_timeout=10
        )

        cursor = conn.cursor(cursor_factory=RealDictCursor)

        # Query all navigation graph nodes with their conflict box associations
        cursor.execute("""
            SELECT
                node_id,
                position_x,
                position_y,
                directions,
                is_conflict_box,
                conflict_box_id,
                created_at
            FROM navigation_graph_nodes
            WHERE conflict_box_id IS NOT NULL
            ORDER BY conflict_box_id, node_id
        """)

        nodes = cursor.fetchall()

        # Group nodes by conflict box
        boxes_cells = {}

        for node in nodes:
            box_id = node['conflict_box_id']
            if box_id not in boxes_cells:
                boxes_cells[box_id] = {
                    'box_id': box_id,
                    'nodes': [],
                    'grid_cells': []
                }

            # Convert world position to grid cell (each cell is 0.5m, grid starts at 0,0)
            grid_x = int(round(float(node['position_x']) / 0.5))
            grid_y = int(round(float(node['position_y']) / 0.5))

            node_info = {
                'node_id': node['node_id'],
                'world_pos': f"({node['position_x']}, {node['position_y']})",
                'grid_pos': f"({grid_x}, {grid_y})",
                'directions': node['directions'],
                'is_conflict_box': node['is_conflict_box']
            }

            boxes_cells[box_id]['nodes'].append(node_info)
            boxes_cells[box_id]['grid_cells'].append((grid_x, grid_y))

        cursor.close()
        conn.close()

        return boxes_cells

    except Exception as e:
        print(f"❌ Database error: {e}")
        return {}

def main():
    """Main function to display cell-to-box mappings."""
    print("🔍 Mapping Cells to Conflict Boxes")
    print("=" * 60)

    # Set the password if not already set
    if not os.getenv('WAREHOUSE_DB_PASSWORD'):
        os.environ['WAREHOUSE_DB_PASSWORD'] = 'renaspolter'
        print("🔑 Set WAREHOUSE_DB_PASSWORD=renaspolter")

    boxes_cells = get_cells_per_conflict_box()

    if not boxes_cells:
        print("❌ No data found!")
        return

    print(f"\n📦 Found {len(boxes_cells)} Conflict Boxes with Cell Mappings:")
    print("-" * 100)

    # Sort boxes by ID for consistent output
    sorted_boxes = sorted(boxes_cells.items())

    for box_id, box_data in sorted_boxes:
        nodes = box_data['nodes']
        grid_cells = box_data['grid_cells']

        print(f"\n🔹 {box_id} ({len(nodes)} nodes, {len(grid_cells)} cells):")
        print("-" * 50)

        # Get conflict box details
        print(f"   📍 Grid Cells: {', '.join([f'({x},{y})' for x,y in sorted(grid_cells)])}")
        print("   📋 Nodes:")

        for node in nodes:
            directions = node['directions']
            if isinstance(directions, list):
                dir_str = ''.join(directions)
            else:
                dir_str = str(directions)

            print(f"      • {node['node_id']}: Grid{node['grid_pos']} World{node['world_pos']} → {dir_str}")

    print("\n✅ Cell-to-Box Mapping Complete!")

    # Summary statistics
    total_nodes = sum(len(box_data['nodes']) for box_data in boxes_cells.values())
    total_cells = sum(len(box_data['grid_cells']) for box_data in boxes_cells.values())
    unique_cells = len(set((x,y) for box_data in boxes_cells.values() for x,y in box_data['grid_cells']))

    print(f"\n📊 SUMMARY:")
    print(f"   • Total Conflict Boxes: {len(boxes_cells)}")
    print(f"   • Total Navigation Nodes: {total_nodes}")
    print(f"   • Total Grid Cells Covered: {total_cells}")
    print(f"   • Unique Grid Cells: {unique_cells}")

if __name__ == "__main__":
    main()
