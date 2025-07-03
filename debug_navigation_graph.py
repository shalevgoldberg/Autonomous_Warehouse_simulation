"""
Debug navigation graph edge issue.
"""
import os
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap

def debug_navigation_graph():
    """Debug navigation graph edge issue."""
    warehouse_map = WarehouseMap(width=20, height=20)
    sim_service = SimulationDataServiceImpl(
        warehouse_map=warehouse_map,
        db_host="localhost",
        db_port=5432,
        db_name="warehouse_sim",
        db_user="postgres",
        db_password=os.getenv('WAREHOUSE_DB_PASSWORD')
    )
    
    try:
        # Check database directly
        with sim_service._get_connection() as conn:
            with conn.cursor() as cur:
                # Check nodes
                cur.execute("SELECT COUNT(*) as node_count FROM navigation_graph_nodes WHERE node_id LIKE 'test_%'")
                node_count = cur.fetchone()['node_count']
                print(f"Test nodes in database: {node_count}")
                
                # Check edges
                cur.execute("SELECT COUNT(*) as edge_count FROM navigation_graph_edges WHERE edge_id LIKE 'test_%'")
                edge_count = cur.fetchone()['edge_count']
                print(f"Test edges in database: {edge_count}")
                
                # List all test edges
                cur.execute("SELECT edge_id, from_node, to_node FROM navigation_graph_edges WHERE edge_id LIKE 'test_%' ORDER BY edge_id")
                edges = cur.fetchall()
                print("Test edges in database:")
                for edge in edges:
                    print(f"  {edge['edge_id']}: {edge['from_node']} -> {edge['to_node']}")
        
        # Check navigation graph
        graph = sim_service.get_navigation_graph()
        test_nodes = [node_id for node_id in graph.nodes.keys() if node_id.startswith('test_')]
        print(f"\nTest nodes in graph: {len(test_nodes)}")
        print(f"Test nodes: {test_nodes}")
        
        # Count edges for test nodes
        test_edges = 0
        print("\nTest edges in graph:")
        for node_id in test_nodes:
            if node_id in graph.edges:
                neighbors = graph.edges[node_id]
                test_edges += len(neighbors)
                print(f"  {node_id}: {neighbors}")
        print(f"Total test edges in graph: {test_edges}")
        
        # Check if there are any non-test edges
        non_test_edges = 0
        for node_id, neighbors in graph.edges.items():
            if not node_id.startswith('test_'):
                non_test_edges += len(neighbors)
        print(f"Non-test edges in graph: {non_test_edges}")
        
    finally:
        sim_service.close()

if __name__ == "__main__":
    debug_navigation_graph() 