"""
Debug script to investigate test failures in database integration tests.
"""
import os
import time
import logging
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def debug_blocked_cells_issue():
    """Debug why blocked cells are not being retrieved."""
    print("=== Debugging Blocked Cells Issue ===")
    
    # Set up database connection
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
        # Test initial state
        blocked_cells = sim_service.get_blocked_cells()
        print(f"Initial blocked cells: {len(blocked_cells)}")
        
        # Report a blocked cell
        current_time = time.time()
        unblock_time = current_time + 60  # Block for 1 minute
        
        print(f"Current time: {current_time}")
        print(f"Unblock time: {unblock_time}")
        print(f"Current time (human): {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))}")
        print(f"Unblock time (human): {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(unblock_time))}")
        
        success = sim_service.report_blocked_cell(
            cell_id='debug_node_1',
            robot_id='test_robot_1',
            unblock_time=unblock_time,
            reason='debug_test'
        )
        print(f"Report blocked cell success: {success}")
        
        # Check database directly
        with sim_service._get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT cell_id, unblock_time, blocked_by_robot, block_reason,
                           EXTRACT(EPOCH FROM unblock_time) as unblock_epoch,
                           EXTRACT(EPOCH FROM CURRENT_TIMESTAMP) as current_epoch
                    FROM blocked_cells
                    WHERE cell_id = 'debug_node_1'
                """)
                result = cur.fetchone()
                if result:
                    print(f"Database record: {result}")
                    print(f"Unblock epoch: {result['unblock_epoch']}")
                    print(f"Current epoch: {result['current_epoch']}")
                    print(f"Is future: {result['unblock_epoch'] > result['current_epoch']}")
                else:
                    print("No record found in database")
        
        # Check blocked cells again
        blocked_cells = sim_service.get_blocked_cells()
        print(f"Blocked cells after report: {len(blocked_cells)}")
        print(f"Blocked cells content: {blocked_cells}")
        
    finally:
        sim_service.close()

def debug_navigation_graph_issue():
    """Debug why navigation graph has fewer edges than expected."""
    print("\n=== Debugging Navigation Graph Issue ===")
    
    # Set up database connection
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
                print("Test edges:")
                for edge in edges:
                    print(f"  {edge['edge_id']}: {edge['from_node']} -> {edge['to_node']}")
        
        # Check navigation graph
        graph = sim_service.get_navigation_graph()
        test_nodes = [node_id for node_id in graph.nodes.keys() if node_id.startswith('test_')]
        print(f"Test nodes in graph: {len(test_nodes)}")
        print(f"Test nodes: {test_nodes}")
        
        # Count edges for test nodes
        test_edges = 0
        for node_id in test_nodes:
            if node_id in graph.edges:
                test_edges += len(graph.edges[node_id])
        print(f"Test edges in graph: {test_edges}")
        
    finally:
        sim_service.close()

if __name__ == "__main__":
    debug_blocked_cells_issue()
    debug_navigation_graph_issue() 