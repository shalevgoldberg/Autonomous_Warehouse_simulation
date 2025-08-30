#!/usr/bin/env python3
"""Analyze conflict boxes from updated CSV with auto-detection."""

from pathlib import Path
from warehouse.impl.graph_generator_impl import GraphGeneratorImpl

def main():
    csv_path = Path('sample_warehouse.csv')
    G = GraphGeneratorImpl()
    graph = G.generate_graph(csv_path)
    
    print('=== CONFLICT BOX ANALYSIS ===')
    print(f'Total conflict boxes: {len(graph.conflict_boxes)}')
    print()
    
    print('=== GROUPED JUNCTION BOXES (cb_*) ===')
    grouped = [k for k in graph.conflict_boxes if str(k).startswith('cb_') and not str(k).startswith('cb_p_')]
    print(f'Found {len(grouped)} grouped boxes')
    for box_id in grouped:
        box = graph.conflict_boxes[box_id]
        print(f'{box_id}: center=({box.center.x:.1f}, {box.center.y:.1f}), size={box.size:.1f}m')
    print()
    
    print('=== PARKING-ADJACENT SINGLES (cb_p_*) ===')
    singles = [k for k in graph.conflict_boxes if str(k).startswith('cb_p_')]
    print(f'Found {len(singles)} single boxes')
    for box_id in singles:
        box = graph.conflict_boxes[box_id]
        print(f'{box_id}: center=({box.center.x:.1f}, {box.center.y:.1f}), size={box.size:.1f}m')
    print()
    
    print('=== DETAILED NODE LOCATIONS ===')
    for node_id, node in graph.nodes.items():
        if node.is_conflict_box:
            print(f'{node_id}: pos=({node.position.x:.1f}, {node.position.y:.1f}), box_id={node.conflict_box_id}')

if __name__ == '__main__':
    main()




