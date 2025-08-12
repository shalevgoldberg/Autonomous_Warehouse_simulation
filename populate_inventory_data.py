"""
Inventory Data Population Script

This script populates the warehouse database with:
1. Shelves created from the warehouse map
2. Sample items with realistic names and descriptions
3. Inventory assignments (one item per shelf as per current requirements)

Usage:
    $env:WAREHOUSE_DB_PASSWORD="renaspolter"; python populate_inventory_data.py

Future Enhancements:
- Multiple items per shelf support
- Multiple shelves per item support
- Dynamic inventory allocation strategies
- Inventory forecasting and replenishment

Architecture Note:
- All inventory operations go through SimulationDataService (unified database interface)
- This follows the Facade Pattern and Single Responsibility Principle
- Components interact with inventory through SimulationDataService, not directly
"""
import os
import sys
import logging
from typing import List, Dict, Any

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from warehouse.map import WarehouseMap
from simulation.simulation_data_service_impl import SimulationDataServiceImpl, SimulationDataServiceError


def setup_logging():
    """Set up logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def create_sample_items() -> List[Dict[str, Any]]:
    """
    Create sample items for the warehouse.
    
    Returns:
        List[Dict[str, Any]]: List of item data dictionaries
    """
    return [
        {
            'item_id': 'book-fantasy',
            'name': 'Fantasy Novel Collection',
            'description': 'Epic fantasy novels with magical worlds and heroic adventures',
            'category': 'books'
        },
        {
            'item_id': 'book-scifi',
            'name': 'Science Fiction Series',
            'description': 'Futuristic science fiction books exploring space and technology',
            'category': 'books'
        },
        {
            'item_id': 'book-mystery',
            'name': 'Mystery Thriller Pack',
            'description': 'Suspenseful mystery novels with intriguing plot twists',
            'category': 'books'
        },
        {
            'item_id': 'electronics-phone',
            'name': 'Smartphone Pro',
            'description': 'Latest smartphone with advanced features and high performance',
            'category': 'electronics'
        },
        {
            'item_id': 'electronics-laptop',
            'name': 'Gaming Laptop',
            'description': 'High-performance gaming laptop with dedicated graphics',
            'category': 'electronics'
        },
        {
            'item_id': 'electronics-tablet',
            'name': 'Tablet Ultra',
            'description': 'Premium tablet with large display and long battery life',
            'category': 'electronics'
        },
        {
            'item_id': 'clothing-tshirt',
            'name': 'Premium Cotton T-Shirt',
            'description': 'Comfortable cotton t-shirt in various sizes and colors',
            'category': 'clothing'
        },
        {
            'item_id': 'clothing-jeans',
            'name': 'Classic Denim Jeans',
            'description': 'High-quality denim jeans with perfect fit',
            'category': 'clothing'
        },
        {
            'item_id': 'clothing-jacket',
            'name': 'Weatherproof Jacket',
            'description': 'Durable jacket suitable for all weather conditions',
            'category': 'clothing'
        },
        {
            'item_id': 'item_A',
            'name': 'Product Alpha',
            'description': 'High-demand product for testing purposes',
            'category': 'test'
        },
        {
            'item_id': 'item_B',
            'name': 'Product Beta',
            'description': 'Standard product for testing purposes',
            'category': 'test'
        },
        {
            'item_id': 'item_C',
            'name': 'Product Gamma',
            'description': 'Low-priority product for testing purposes',
            'category': 'test'
        }
    ]


def create_inventory_assignments(shelf_ids: List[str], items: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Create inventory assignments (one item per shelf).
    
    Args:
        shelf_ids: List of available shelf IDs
        items: List of item data dictionaries
        
    Returns:
        List[Dict[str, Any]]: List of inventory assignment dictionaries
    """
    inventory_assignments = []
    
    # Assign items to shelves (one item per shelf)
    for i, item in enumerate(items):
        if i < len(shelf_ids):
            # Assign realistic quantities based on item type
            if item['category'] == 'books':
                quantity = 25  # Books can be stacked
            elif item['category'] == 'electronics':
                quantity = 8   # Electronics are more limited
            elif item['category'] == 'clothing':
                quantity = 15  # Clothing items
            else:
                quantity = 10  # Default quantity
            
            inventory_assignments.append({
                'shelf_id': shelf_ids[i],
                'item_id': item['item_id'],
                'name': item['name'],
                'quantity': quantity
            })
    
    return inventory_assignments


def main():
    """Main function to populate inventory data."""
    setup_logging()
    logger = logging.getLogger(__name__)
    
    try:
        logger.info("Starting inventory data population...")
        
        # Check database password
        if not os.getenv('WAREHOUSE_DB_PASSWORD'):
            logger.error("WAREHOUSE_DB_PASSWORD environment variable not set")
            logger.info("Please set it with: $env:WAREHOUSE_DB_PASSWORD=\"renaspolter\"")
            return 1
        
        # Create warehouse map
        logger.info("Creating warehouse map...")
        warehouse_map = WarehouseMap(width=20, height=15)
        logger.info(f"Warehouse map created with {len(warehouse_map.shelves)} shelves")
        
        # Create simulation data service (unified database interface)
        logger.info("Initializing simulation data service...")
        simulation_data_service = SimulationDataServiceImpl(
            warehouse_map=warehouse_map,
            db_host="localhost",
            db_port=5432,
            db_name="warehouse_sim",
            db_user="postgres",
            db_password=os.getenv('WAREHOUSE_DB_PASSWORD')
        )
        
        # Create shelves from map
        logger.info("Creating shelves from warehouse map...")
        shelves_created = simulation_data_service.create_shelves_from_map(clear_existing=True)
        logger.info(f"Created {shelves_created} shelves")
        
        # Get shelf IDs for assignment
        shelf_ids = list(warehouse_map.shelves.keys())
        logger.info(f"Available shelf IDs: {shelf_ids[:5]}... (showing first 5)")
        
        # Create sample items
        logger.info("Creating sample items...")
        items = create_sample_items()
        logger.info(f"Created {len(items)} sample items")
        
        # Create inventory assignments
        logger.info("Creating inventory assignments...")
        inventory_assignments = create_inventory_assignments(shelf_ids, items)
        logger.info(f"Created {len(inventory_assignments)} inventory assignments")
        
        # Populate inventory
        logger.info("Populating inventory in database...")
        inventory_created = simulation_data_service.populate_inventory(inventory_assignments)
        logger.info(f"Populated {inventory_created} inventory entries")
        
        # Get inventory statistics
        logger.info("Getting inventory statistics...")
        stats = simulation_data_service.get_inventory_statistics()
        logger.info("Inventory Statistics:")
        logger.info(f"  Total shelves: {stats['total_shelves']}")
        logger.info(f"  Total items: {stats['total_items']}")
        logger.info(f"  Total quantity: {stats['total_quantity']}")
        logger.info(f"  Low stock items: {len(stats['low_stock_items'])}")
        
        # Show some sample assignments
        logger.info("Sample inventory assignments:")
        for i, assignment in enumerate(inventory_assignments[:5]):
            logger.info(f"  {assignment['shelf_id']}: {assignment['quantity']}x {assignment['item_id']}")
        
        if len(inventory_assignments) > 5:
            logger.info(f"  ... and {len(inventory_assignments) - 5} more assignments")
        
        # Test item location lookup
        logger.info("Testing item location lookup...")
        test_items = ['item_A', 'book-fantasy', 'electronics-phone']
        for item_id in test_items:
            shelf_id = simulation_data_service.get_item_location(item_id)
            if shelf_id:
                logger.info(f"  {item_id} -> {shelf_id}")
            else:
                logger.warning(f"  {item_id} not found in inventory")
        
        logger.info("Inventory data population completed successfully!")
        logger.info("")
        logger.info("Next steps:")
        logger.info("1. Run the complete task flow test")
        logger.info("2. Verify orders can be processed")
        logger.info("3. Check robot task execution")
        
        return 0
        
    except SimulationDataServiceError as e:
        logger.error(f"Simulation data service error: {e}")
        return 1
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        return 1
    finally:
        if 'simulation_data_service' in locals():
            simulation_data_service.close()


if __name__ == "__main__":
    sys.exit(main()) 