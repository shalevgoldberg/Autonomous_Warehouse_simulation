import os
import unittest
from pathlib import Path

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from pathlib import Path


class TestGraphPersistenceIntegration(unittest.TestCase):
    def setUp(self) -> None:
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            self.skipTest("WAREHOUSE_DB_PASSWORD not set; skipping graph persistence integration test")

        self.map = WarehouseMap(csv_file="sample_warehouse.csv")
        self.svc = SimulationDataServiceImpl(
            warehouse_map=self.map,
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD'),
            pool_size=3,
        )

    def tearDown(self) -> None:
        if hasattr(self, 'svc'):
            self.svc.close()

    def test_persist_graph_from_csv_and_counts(self):
        # Ensure shelves exist as precondition (idempotent)
        self.svc.create_shelves_from_map(clear_existing=True)

        # Persist graph via SimulationDataService interface (preferred)
        result = self.svc.persist_navigation_graph_from_csv(Path("sample_warehouse.csv"), clear_existing=True)

        # Verify counts are non-zero and consistent
        self.assertGreater(result.nodes_persisted, 0)
        self.assertGreater(result.edges_persisted, 0)
        # Query DB to confirm
        with self.svc._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("select count(*) as c from navigation_graph_nodes")
                nodes = cur.fetchone()['c']
                cur.execute("select count(*) as c from navigation_graph_edges")
                edges = cur.fetchone()['c']
                # Idle/charging presence and connectivity
                cur.execute("select count(*) as c from navigation_graph_nodes where node_id like 'idle_%'")
                idle_nodes = cur.fetchone()['c']
                cur.execute("select count(*) as c from navigation_graph_nodes where node_id like 'charge_%'")
                charge_nodes = cur.fetchone()['c']
                # Ensure at least one outgoing edge exists for each idle/charge node
                cur.execute("select count(*) as c from navigation_graph_edges e join navigation_graph_nodes n on e.from_node = n.node_id where n.node_id like 'idle_%'")
                idle_edges = cur.fetchone()['c']
                cur.execute("select count(*) as c from navigation_graph_edges e join navigation_graph_nodes n on e.from_node = n.node_id where n.node_id like 'charge_%'")
                charge_edges = cur.fetchone()['c']
        self.assertEqual(nodes, result.nodes_persisted)
        self.assertEqual(edges, result.edges_persisted)
        self.assertGreater(idle_nodes, 0)
        self.assertGreater(charge_nodes, 0)
        # At least as many edges as nodes (some may be dead-ends but should have a connection)
        self.assertGreaterEqual(idle_edges, idle_nodes)
        self.assertGreaterEqual(charge_edges, charge_nodes)


if __name__ == "__main__":
    unittest.main()


