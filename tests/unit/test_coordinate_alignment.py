import os
import math
import unittest
from pathlib import Path

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl


class TestCoordinateAlignment(unittest.TestCase):
    def setUp(self) -> None:
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("WAREHOUSE_DB_PASSWORD not set; skipping alignment test")

        self.map = WarehouseMap(csv_file="sample_warehouse.csv")
        self.svc = SimulationDataServiceImpl(
            warehouse_map=self.map,
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD'),
            pool_size=3,
        )

        # Ensure DB reflects CSV
        self.svc.create_shelves_from_map(clear_existing=True)
        self.svc.persist_navigation_graph_from_csv(Path("sample_warehouse.csv"), clear_existing=True)
        self.graph = self.svc.get_navigation_graph()

    def tearDown(self) -> None:
        if hasattr(self, 'svc'):
            self.svc.close()

    def test_nodes_align_with_map_cell_centers(self):
        tol = 1e-9
        for nid, node in self.graph.nodes.items():
            gx, gy = self.map.world_to_grid(node.position.x, node.position.y)
            wx, wy = self.map.grid_to_world(gx, gy)
            self.assertLessEqual(abs(wx - node.position.x), tol, f"x mismatch for {nid}")
            self.assertLessEqual(abs(wy - node.position.y), tol, f"y mismatch for {nid}")

    def test_edge_steps_match_single_cell_size(self):
        cell = self.map.grid_size
        tol = 1e-9
        for src, nbrs in self.graph.edges.items():
            sp = self.graph.nodes[src].position
            for dst in nbrs:
                dp = self.graph.nodes[dst].position
                dx, dy = dp.x - sp.x, dp.y - sp.y
                # Should be a single orthogonal step in N/S/E/W
                self.assertTrue(
                    (abs(dx - cell) <= tol and abs(dy) <= tol) or
                    (abs(dx + cell) <= tol and abs(dy) <= tol) or
                    (abs(dy - cell) <= tol and abs(dx) <= tol) or
                    (abs(dy + cell) <= tol and abs(dx) <= tol),
                    f"Edge {src}->{dst} is not a single-cell orthogonal step (dx={dx}, dy={dy})",
                )


if __name__ == "__main__":
    unittest.main()


