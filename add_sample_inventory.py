#!/usr/bin/env python3
"""
Add sample inventory for the items in sample_orders.json
"""
import os
from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl

def add_sample_inventory():
    """Add inventory for sample orders."""
    print("📦 Adding sample inventory for sample_orders.json items...")
    
    # Items from sample_orders.json
    sample_items = [
        {'item_id': 'book_001', 'name': 'Sample Book', 'shelf_id': 'shelf_10', 'quantity': 10},
        {'item_id': 'phone_001', 'name': 'Sample Phone', 'shelf_id': 'shelf_11', 'quantity': 5},
        {'item_id': 'laptop_001', 'name': 'Sample Laptop', 'shelf_id': 'shelf_12', 'quantity': 3},
        {'item_id': 'tablet_001', 'name': 'Sample Tablet', 'shelf_id': 'shelf_13', 'quantity': 4},
        {'item_id': 'audio_001', 'name': 'Sample Audio Device', 'shelf_id': 'shelf_14', 'quantity': 6},
    ]
    
    try:
        # Create simulation data service
        warehouse_map = WarehouseMap(width=20, height=15)
        simulation_data_service = SimulationDataServiceImpl(
            warehouse_map=warehouse_map,
            db_host="localhost",
            db_port=5432,
            db_name="warehouse_sim",
            db_user="postgres",
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD')
        )
        
        # Add inventory
        created = simulation_data_service.populate_inventory(sample_items)
        print(f"✅ Added {created} inventory entries")
        
        # Verify
        stats = simulation_data_service.get_inventory_statistics()
        print(f"✅ Total items: {stats['total_items']}")
        print(f"✅ Total quantity: {stats['total_quantity']}")
        
        # Check specific items
        for item in sample_items:
            location = simulation_data_service.get_item_location(item['item_id'])
            if location:
                print(f"✅ {item['item_id']} found on {location}")
            else:
                print(f"❌ {item['item_id']} not found")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to add inventory: {e}")
        return False

if __name__ == "__main__":
    add_sample_inventory() 