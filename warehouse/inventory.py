from typing import Dict, List, Tuple, Optional, Any, Set
import uuid
from warehouse.item import Item
from warehouse.shelf import Shelf, ShelfManager

class InventoryManager:
    """Manages the global inventory across all shelves in the warehouse."""
    
    def __init__(self, shelf_manager: ShelfManager, sim_data_service):
        if sim_data_service is None:
            raise ValueError("sim_data_service must be provided for database operations.")
        self.shelf_manager = shelf_manager
        self.sim_data_service = sim_data_service
        # self.items: Dict[str, Item] = {}  # item_id -> Item (removed, now in DB)
    
    @property
    def db_cursor(self):
        return self.sim_data_service.get_db_cursor()
    
    def add_item_definition(self, item: Item) -> str:
        """
        Add a new item definition to the inventory catalog.
        Returns the item_id.
        """
        with self.db_cursor as cur:
            cur.execute("""
                INSERT INTO items (item_id, name)
                VALUES (%s, %s)
                ON CONFLICT (item_id) DO UPDATE SET name = EXCLUDED.name
            """, (item.id, item.name))
        return item.id
    
    def add_item_to_shelf(self, shelf_id: str, item: Item, quantity: int = 1) -> bool:
        """
        Add items to a specific shelf.
        """
        with self.db_cursor as cur:
            # Upsert shelf inventory
            cur.execute("""
                INSERT INTO shelf_inventory (shelf_id, item_id, quantity)
                VALUES (%s, %s, %s)
                ON CONFLICT (shelf_id, item_id)
                DO UPDATE SET quantity = shelf_inventory.quantity + EXCLUDED.quantity
            """, (shelf_id, item.id, quantity))
        return True
    
    def remove_item_from_shelf(self, shelf_id: str, item: Item, quantity: int = 1) -> bool:
        """
        Remove items from a specific shelf.
        """
        with self.db_cursor as cur:
            # Check current quantity
            cur.execute("SELECT quantity FROM shelf_inventory WHERE shelf_id=%s AND item_id=%s", (shelf_id, item.id))
            row = cur.fetchone()
            if not row or row[0] < quantity:
                return False
            new_quantity = row[0] - quantity
            if new_quantity > 0:
                cur.execute("UPDATE shelf_inventory SET quantity=%s WHERE shelf_id=%s AND item_id=%s", (new_quantity, shelf_id, item.id))
            else:
                cur.execute("DELETE FROM shelf_inventory WHERE shelf_id=%s AND item_id=%s", (shelf_id, item.id))
        return True
    
    def transfer_items(self, from_shelf_id: str, to_shelf_id: str, item: Item, quantity: int = 1) -> bool:
        """
        Transfer items between shelves.
        """
        if self.remove_item_from_shelf(from_shelf_id, item, quantity):
            return self.add_item_to_shelf(to_shelf_id, item, quantity)
        return False
    
    def find_item_locations(self, item: Item) -> List[Tuple[str, int]]:
        """
        Find all shelves containing a specific item and their quantities.
        """
        with self.db_cursor as cur:
            cur.execute("SELECT shelf_id, quantity FROM shelf_inventory WHERE item_id=%s AND quantity > 0", (item.id,))
            return [(row[0], row[1]) for row in cur.fetchall()]
    
    def get_all_items(self) -> List[Item]:
        """Get all item definitions in the inventory."""
        with self.db_cursor as cur:
            cur.execute("SELECT item_id, name FROM items")
            return [Item(item_id=row[0], name=row[1]) for row in cur.fetchall()]
    
    def get_item_by_id(self, item_id: str) -> Optional[Item]:
        """Get item definition by ID."""
        with self.db_cursor as cur:
            cur.execute("SELECT item_id, name FROM items WHERE item_id=%s", (item_id,))
            row = cur.fetchone()
            if row:
                return Item(item_id=row[0], name=row[1])
            return None
    
    def get_total_quantity(self, item: Item) -> int:
        """Get the total quantity of an item across all shelves."""
        with self.db_cursor as cur:
            cur.execute("SELECT SUM(quantity) FROM shelf_inventory WHERE item_id=%s", (item.id,))
            row = cur.fetchone()
            return row[0] if row and row[0] is not None else 0
    
    def get_inventory_summary(self) -> Dict[str, Dict[str, Any]]:
        """
        Get a summary of the entire inventory.
        """
        summary = {}
        with self.db_cursor as cur:
            cur.execute("""
                SELECT i.item_id, i.name, COALESCE(SUM(si.quantity),0) as total_quantity
                FROM items i
                LEFT JOIN shelf_inventory si ON i.item_id = si.item_id
                GROUP BY i.item_id, i.name
            """)
            for row in cur.fetchall():
                summary[row[0]] = {
                    'name': row[1],
                    'total_quantity': row[2],
                    'locations': self.find_item_locations(Item(item_id=row[0], name=row[1]))
                }
        return summary 