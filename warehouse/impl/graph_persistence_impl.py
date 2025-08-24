"""
Graph persistence implementation.

Persists a navigation graph (conflict boxes, nodes, edges) into the database
in the correct order. Generation is delegated to GraphGeneratorImpl.

Design:
- SOLID: single responsibility (persistence only); depends on abstractions
- Interface-driven: implements IGraphPersistence
- Loose coupling: accepts SimulationDataService via its interface
"""
from pathlib import Path
from typing import Optional
import json

from interfaces.graph_persistence_interface import IGraphPersistence, GraphPersistenceResult
from interfaces.simulation_data_service_interface import ISimulationDataService
from interfaces.navigation_types import LaneDirection
from warehouse.impl.graph_generator_impl import GraphGeneratorImpl


class GraphPersistenceImpl(IGraphPersistence):
    """Production implementation for persisting navigation graphs into DB."""

    def __init__(self, simulation_data_service: ISimulationDataService) -> None:
        self._sds = simulation_data_service

    def persist_from_csv(self, csv_path: Path, clear_existing: bool = False) -> GraphPersistenceResult:
        generator = GraphGeneratorImpl()
        graph = generator.generate_graph(csv_path)

        boxes_persisted = 0
        nodes_persisted = 0
        edges_persisted = 0

        with self._sds._get_connection() as conn:  # relies on SDS transactional safety
            with conn.cursor() as cur:
                if clear_existing:
                    # Clear existing graph-related data in FK-safe order
                    # 1) Edges then nodes (nodes may be referenced by edges)
                    cur.execute("DELETE FROM navigation_graph_edges")
                    cur.execute("DELETE FROM navigation_graph_nodes")
                    # 2) Dependent conflict box relations (locks/queues) before boxes
                    try:
                        cur.execute("DELETE FROM conflict_box_locks")
                    except Exception:
                        # Table may not exist in some schemas; ignore
                        pass
                    try:
                        cur.execute("DELETE FROM conflict_box_queue")
                    except Exception:
                        # Table may not exist in some schemas; ignore
                        pass
                    # 3) Finally clear conflict boxes
                    cur.execute("DELETE FROM conflict_boxes")

                # Insert conflict boxes first
                for box_id, box in graph.conflict_boxes.items():
                    cur.execute(
                        """
                        INSERT INTO conflict_boxes (box_id, center_x, center_y, size)
                        VALUES (%s, %s, %s, %s)
                        ON CONFLICT (box_id) DO NOTHING
                        """,
                        (str(box_id), float(box.center.x), float(box.center.y), float(box.size))
                    )
                    boxes_persisted += 1

                # Insert nodes
                for node_id, node in graph.nodes.items():
                    directions = [d.value if isinstance(d, LaneDirection) else d for d in node.directions]
                    cur.execute(
                        """
                        INSERT INTO navigation_graph_nodes (
                            node_id, position_x, position_y, directions, is_conflict_box, conflict_box_id
                        ) VALUES (%s,%s,%s,%s::jsonb,%s,%s)
                        ON CONFLICT (node_id) DO NOTHING
                        """,
                        (
                            node_id,
                            float(node.position.x),
                            float(node.position.y),
                            json.dumps(directions),
                            bool(getattr(node, 'is_conflict_box', False)),
                            getattr(node, 'conflict_box_id', None),
                        ),
                    )
                    nodes_persisted += 1

                # Insert edges
                edge_counter = 0
                for src, neighbors in graph.edges.items():
                    for dst in neighbors:
                        cur.execute(
                            """
                            INSERT INTO navigation_graph_edges (edge_id, from_node, to_node, direction, distance)
                            VALUES (%s,%s,%s,%s,%s)
                            ON CONFLICT (edge_id) DO NOTHING
                            """,
                            (f"e_{edge_counter}", src, dst, 'N', 0.5),
                        )
                        edges_persisted += 1
                        edge_counter += 1

                conn.commit()

        return GraphPersistenceResult(
            nodes_persisted=nodes_persisted,
            edges_persisted=edges_persisted,
            boxes_persisted=boxes_persisted,
            cleared_existing=clear_existing,
        )


