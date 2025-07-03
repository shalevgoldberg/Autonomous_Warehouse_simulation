"""
Debug script to check if conflict boxes are correctly marked and positioned.
"""
from warehouse.impl.graph_generator_impl import GraphGeneratorImpl
from pathlib import Path

def debug_conflict_boxes():
    """Debug conflict box marking and positioning."""
    print("CONFLICT BOX DEBUG")
    print("=" * 50)
    
    # Generate graph
    generator = GraphGeneratorImpl()
    graph = generator.generate_graph(Path("sample_warehouse.csv"))
    
    # Check all conflict boxes
    print("All Conflict Boxes:")
    print("-" * 30)
    for box_id, box in graph.conflict_boxes.items():
        directions = ''.join(d.value for d in box.directions)
        print(f"Box {box_id}: ({box.position.x}, {box.position.y}) directions: {directions}")
        
        # Check if the node is correctly marked
        node_id = list(box.participating_nodes)[0]
        node = graph.nodes[node_id]
        print(f"  Node {node_id}: is_conflict_box={node.is_conflict_box}, conflict_box_id={node.conflict_box_id}")
        
        # Check connections
        connections = graph.edges.get(node_id, set())
        print(f"  Connections: {list(connections)}")
        print()
    
    # Check specific positions from sample_warehouse.csv
    print("Checking specific positions from CSV:")
    print("-" * 40)
    
    # Read the CSV to see what should be at each position
    import csv
    with open("sample_warehouse.csv", 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        grid = list(reader)
    
    # Check each conflict box position
    for box_id, box in graph.conflict_boxes.items():
        row_idx = int(-box.position.y / 0.5)  # Convert Y back to row
        col_idx = int(box.position.x / 0.5)   # Convert X back to column
        
        if 0 <= row_idx < len(grid) and 0 <= col_idx < len(grid[row_idx]):
            csv_value = grid[row_idx][col_idx]
            directions = ''.join(d.value for d in box.directions)
            print(f"Box {box_id}: CSV[{row_idx}][{col_idx}] = '{csv_value}' → parsed as '{directions}'")
            
            # Check if the parsing is correct
            if csv_value.startswith('j'):
                expected_directions = csv_value[1:]  # Remove 'j' prefix
                if directions.lower() == expected_directions:
                    print(f"  ✅ Correctly parsed")
                else:
                    print(f"  ❌ Mismatch: expected '{expected_directions}', got '{directions}'")
            else:
                print(f"  ❌ Should not be a conflict box (doesn't start with 'j')")
        else:
            print(f"Box {box_id}: Position ({box.position.x}, {box.position.y}) outside grid bounds")
        print()
    
    # Check if all 'j' cells in CSV are represented as conflict boxes
    print("Checking if all 'j' cells in CSV are conflict boxes:")
    print("-" * 50)
    conflict_box_positions = set()
    for box in graph.conflict_boxes.values():
        conflict_box_positions.add((box.position.x, box.position.y))
    
    j_cells_in_csv = []
    for row_idx, row in enumerate(grid):
        for col_idx, cell_value in enumerate(row):
            if cell_value.startswith('j'):
                x = col_idx * 0.5
                y = -row_idx * 0.5
                j_cells_in_csv.append((x, y, cell_value))
    
    print(f"Found {len(j_cells_in_csv)} 'j' cells in CSV:")
    for x, y, cell_value in j_cells_in_csv:
        if (x, y) in conflict_box_positions:
            print(f"  ✅ ({x}, {y}) '{cell_value}' → correctly marked as conflict box")
        else:
            print(f"  ❌ ({x}, {y}) '{cell_value}' → NOT marked as conflict box")
    
    # Summary
    print(f"\nSUMMARY:")
    print(f"- Total conflict boxes in graph: {len(graph.conflict_boxes)}")
    print(f"- Total 'j' cells in CSV: {len(j_cells_in_csv)}")
    print(f"- Conflict box positions: {len(conflict_box_positions)}")
    print(f"- J cells in CSV: {len(j_cells_in_csv)}")

if __name__ == "__main__":
    debug_conflict_boxes() 