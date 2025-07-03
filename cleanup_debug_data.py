"""
Clean up debug and test data from database.
"""
import os
from simulation.simulation_data_service_impl import SimulationDataServiceImpl
from warehouse.map import WarehouseMap

def cleanup_debug_data():
    """Clean up any debug and test data from database."""
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
        with sim_service._get_connection() as conn:
            with conn.cursor() as cur:
                # Clean up debug and test data
                cur.execute("DELETE FROM blocked_cells WHERE cell_id LIKE 'debug_%' OR cell_id LIKE 'test_%'")
                cur.execute("DELETE FROM navigation_graph_edges WHERE edge_id LIKE 'test_%'")
                cur.execute("DELETE FROM navigation_graph_nodes WHERE node_id LIKE 'test_%'")
                cur.execute("DELETE FROM conflict_boxes WHERE box_id LIKE 'test_%'")
                cur.execute("DELETE FROM lanes WHERE lane_id LIKE 'test_%'")
                deleted = cur.rowcount
                conn.commit()
                print(f"Cleaned up debug and test data from navigation graph tables")
    finally:
        sim_service.close()

if __name__ == "__main__":
    cleanup_debug_data() 