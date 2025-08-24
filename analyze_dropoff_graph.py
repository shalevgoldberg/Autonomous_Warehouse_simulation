import sys
from pathlib import Path

from warehouse.impl.graph_generator_impl import GraphGeneratorImpl


def main():
    csv_path = Path("sample_warehouse.csv")
    if not csv_path.exists():
        print("sample_warehouse.csv not found")
        sys.exit(1)

    gen = GraphGeneratorImpl()
    graph = gen.generate_graph(csv_path)

    drop_nodes = [nid for nid in graph.nodes.keys() if nid.startswith("dropoff_")]
    print("=== DROP-OFF GRAPH ANALYSIS ===")
    print(f"Drop-off nodes: {len(drop_nodes)}")
    for nid in sorted(drop_nodes):
        node = graph.nodes[nid]
        neighbors = graph.edges.get(nid, [])
        print(f"{nid} at ({node.position.x:.2f},{node.position.y:.2f}) -> {neighbors}")

    # Sanity: ensure no neighbor has reverse edge to dropoff unless lane directions encode it
    rev_edges = []
    for nid in drop_nodes:
        for nb in graph.edges.get(nid, []):
            if nid in graph.edges.get(nb, []):
                rev_edges.append((nid, nb))
    if rev_edges:
        print("WARNING: Found bidirectional edges with dropoffs:")
        for a, b in rev_edges:
            print(f"  {a} <-> {b}")
    else:
        print("No bidirectional edges from dropoffs detected (as expected).")


if __name__ == "__main__":
    main()


