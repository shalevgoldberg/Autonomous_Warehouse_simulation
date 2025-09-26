"""
Inventory Operations CLI (JSON-only)

Operations:
  - add-sample: add curated sample inventory (random shelf assignment always)
  - add: add inventory from a JSON file (supports explicit shelf_id or strategy-based assignment)
  - delete-all: delete ALL inventory rows (shelf_inventory only)
  - delete-items: delete inventory for specific item_ids (optional purge of orphaned items)
  - export: export inventory status CSV to a file
  - move: move quantity of an item from one shelf to another

Rules:
  - Never mutate shelves. If DB has no shelves, print error and exit.
  - No mocks. Use real DB via SimulationDataServiceImpl.
  - JSON input only (no CSV).
  - Sample inventory is always assigned randomly to shelves.
  - Log clearly on any fallback or validation failure.

Architecture Note:
  - All DB access flows through SimulationDataServiceImpl
  - This script is a thin CLI; it validates inputs and delegates to the service
"""
import os
import sys
import logging
import argparse
import json
from typing import List, Dict, Any

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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


def _load_json_file(path: str) -> List[Dict[str, Any]]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
        if not isinstance(data, list):
            raise ValueError("Input JSON must be a list of inventory records")
        return data


def main():
    setup_logging()
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser(description="Inventory Operations CLI (JSON-only)")
    sub = parser.add_subparsers(dest="command", required=True)

    # add-sample
    p_add_sample = sub.add_parser("add-sample", help="Add curated sample inventory (random assignment)")
    p_add_sample.add_argument("--strategy", default="random",
                              choices=["random", "load_balanced", "category_based", "proximity_based"],
                              help="Assignment strategy for items without shelf_id (default: random)")

    # add
    p_add = sub.add_parser("add", help="Add inventory from a JSON file")
    p_add.add_argument("--file", required=True, help="Path to JSON file with inventory records")
    p_add.add_argument("--strategy", default="random",
                       choices=["random", "load_balanced", "category_based", "proximity_based"],
                       help="Assignment strategy for items without shelf_id (default: random)")

    # delete-all
    sub.add_parser("delete-all", help="Delete ALL inventory from shelf_inventory")

    # delete-items
    p_del_items = sub.add_parser("delete-items", help="Delete inventory for specific item IDs")
    p_del_items.add_argument("--item-ids", required=True,
                             help="Comma-separated list of item IDs to delete")
    p_del_items.add_argument("--purge-items", action="store_true",
                             help="Also delete items with no remaining inventory")

    # export
    p_export = sub.add_parser("export", help="Export inventory status CSV")
    p_export.add_argument("--file", default="inventory_status.csv",
                          help="Output CSV filename (default: inventory_status.csv)")

    # move
    p_move = sub.add_parser("move", help="Move quantity of an item between shelves")
    p_move.add_argument("--item-id", required=True)
    p_move.add_argument("--from-shelf", required=True)
    p_move.add_argument("--to-shelf", required=True)
    p_move.add_argument("--quantity", type=int, required=True)

    args = parser.parse_args()

    # Check DB configuration
    if not os.getenv('DATABASE_URL') and not os.getenv('WAREHOUSE_DB_PASSWORD'):
        logger.error("Database configuration not found")
        logger.info("Set WAREHOUSE_DB_PASSWORD or DATABASE_URL before running the CLI")
        return 1

    # Set CLI mode to disable KPI recorder for faster shutdown
    os.environ['CLI_MODE'] = '1'

    # Initialize service with a minimal map (never write shelves here)
    # Note: SimulationDataServiceImpl requires a WarehouseMap instance, but we do NOT mutate shelves.
    from warehouse.map import WarehouseMap  # deferred import to keep CLI light
    # Use minimal safe dimensions to satisfy WarehouseMap internals; never used to write shelves
    warehouse_map = WarehouseMap(width=3, height=3)  # placeholder; not used for shelf creation
    svc = SimulationDataServiceImpl(warehouse_map=warehouse_map)

    # Some commands require shelves to exist (add-sample, add, move). Others don't (export, delete-*).
    def _require_shelves() -> bool:
        try:
            with svc._get_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("SELECT COUNT(*) AS cnt FROM shelves")
                    shelf_count = cur.fetchone()['cnt']
                    if shelf_count == 0:
                        logger.error("No shelves found in DB. This CLI only manages inventory on pre-existing shelves.")
                        return False
                    return True
        except Exception as e:
            logger.error(f"Failed to validate shelves in DB: {e}")
            return False

    try:
        if args.command == "add-sample":
            if not _require_shelves():
                return 1
            items = create_sample_items()
            # Assign realistic default quantities when not provided by sample
            payload: List[Dict[str, Any]] = []
            for it in items:
                qty = 25 if it.get('category') == 'books' else (8 if it.get('category') == 'electronics' else (15 if it.get('category') == 'clothing' else 10))
                payload.append({
                    'item_id': it['item_id'],
                    'name': it['name'],
                    'quantity': qty,
                    # No shelf_id → always strategy-based; default random per requirement
                    'category': it.get('category')
                })
            created = svc.populate_inventory(payload, assignment_strategy=args.strategy)
            logger.info(f"Added sample inventory entries: {created}")

        elif args.command == "add":
            if not _require_shelves():
                return 1
            if not os.path.exists(args.file):
                logger.error(f"Input JSON file not found: {args.file}")
                logger.info("Provide a JSON file with a list of records: [{ 'item_id': '...', 'name': '...', 'quantity': N, 'shelf_id': 'optional' }]")
                return 1
            data = _load_json_file(args.file)
            # Validate and coerce
            valid: List[Dict[str, Any]] = []
            for rec in data:
                if not isinstance(rec, dict):
                    logger.warning(f"Skipping non-object record: {rec}")
                    continue
                missing = [k for k in ("item_id", "name", "quantity") if k not in rec]
                if missing:
                    logger.warning(f"Skipping record missing required fields {missing}: {rec}")
                    continue
                if not isinstance(rec['quantity'], int) or rec['quantity'] <= 0:
                    logger.warning(f"Skipping record with non-positive quantity: {rec}")
                    continue
                valid.append(rec)
            if not valid:
                logger.warning("No valid inventory records found; aborting add")
                return 1
            created = svc.populate_inventory(valid, assignment_strategy=args.strategy)
            logger.info(f"Added inventory entries: {created}")

        elif args.command == "delete-all":
            deleted = svc.clear_all_inventory()
            logger.info(f"Deleted all inventory rows: {deleted}")

        elif args.command == "delete-items":
            ids = [s for s in (args.item_ids or "").split(',') if s]
            if not ids:
                logger.warning("No item IDs provided to delete-items")
                return 1
            deleted = svc.delete_inventory_for_items(ids, purge_items=args.purge_items)
            logger.info(f"Deleted inventory rows for items {ids}: {deleted}")

        elif args.command == "export":
            ok = svc.export_inventory_status_csv(args.file)
            if ok:
                logger.info(f"Exported inventory CSV: {args.file}")
            else:
                logger.warning(f"Export failed for file: {args.file}")

        elif args.command == "move":
            if not _require_shelves():
                return 1
            if args.quantity <= 0:
                logger.warning("Quantity must be > 0 for move")
                return 1
            moved = svc.move_inventory(args.item_id, args.from_shelf, args.to_shelf, args.quantity)
            if moved:
                logger.info(f"Moved {args.quantity} of {args.item_id} from {args.from_shelf} to {args.to_shelf}")
            else:
                logger.warning(f"Move failed for {args.item_id} from {args.from_shelf} to {args.to_shelf}")

        else:
            logger.error(f"Unknown command: {args.command}")
            return 1

        return 0

    except SimulationDataServiceError as e:
        logger.error(f"Simulation data service error: {e}")
        return 1
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        return 1
    finally:
        try:
            svc.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main()) 