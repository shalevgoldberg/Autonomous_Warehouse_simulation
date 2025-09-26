#!/usr/bin/env python3
"""
Query all navigation graph nodes to show exact cell-to-conflict-box mapping.
"""

import os
import sys
import psycopg2
from psycopg2.extras import RealDictCursor

def query_all_navigation_nodes():
    """Query all navigation graph nodes with their conflict box associations."""
    # Get database configuration
    db_host = os.getenv('WAREHOUSE_DB_HOST', 'localhost')
    db_port = int(os.getenv('WAREHOUSE_DB_PORT', '5432'))
    db_name = os.getenv('WAREHOUSE_DB_NAME', 'warehouse_sim')
    db_user = os.getenv('WAREHOUSE_DB_USER', 'postgres')
    db_password = os.getenv('WAREHOUSE_DB_PASSWORD')

    if not db_password:
        print("❌ WAREHOUSE_DB_PASSWORD environment variable not set!")
        print("   Set with: $env:WAREHOUSE_DB_PASSWORD='renaspolter'")
        return

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

        # Query ALL navigation graph nodes
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
            ORDER BY node_id
        """)

        nodes = cursor.fetchall()

        print("🔍 COMPLETE NAVIGATION GRAPH NODES")
        print("=" * 100)
        print(f"Total nodes: {len(nodes)}")
        print()

        # Group by conflict box
        conflict_boxes = {}
        regular_nodes = []

        for node in nodes:
            box_id = node['conflict_box_id']
            if box_id:
                if box_id not in conflict_boxes:
                    conflict_boxes[box_id] = []
                conflict_boxes[box_id].append(node)
            else:
                regular_nodes.append(node)

        # Show conflict box nodes
        print("🎯 CONFLICT BOX NODES:")
        print("-" * 100)

        sorted_boxes = sorted(conflict_boxes.keys())
        for box_id in sorted_boxes:
            box_nodes = conflict_boxes[box_id]
            print(f"\n🔹 {box_id} ({len(box_nodes)} nodes):")

            for node in box_nodes:
                world_x = float(node['position_x'])
                world_y = float(node['position_y'])
                directions = node['directions']
                if isinstance(directions, list):
                    dir_str = ''.join(directions)
                else:
                    dir_str = str(directions)

                print(f"   📍 {node['node_id']}: World({world_x:.3f}, {world_y:.3f}) → {dir_str}")

        print(f"\n📊 SUMMARY:")
        print(f"   • Total nodes: {len(nodes)}")
        print(f"   • Conflict box nodes: {sum(len(nodes) for nodes in conflict_boxes.values())}")
        print(f"   • Regular navigation nodes: {len(regular_nodes)}")
        print(f"   • Conflict boxes: {len(conflict_boxes)}")

        cursor.close()
        conn.close()

    except Exception as e:
        print(f"❌ Database error: {e}")

if __name__ == "__main__":
    query_all_navigation_nodes()
