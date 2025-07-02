from typing import Dict, List, Tuple, Optional, Any
import uuid
from warehouse.item import Item

class Shelf:
    """Represents a shelf in the warehouse."""
    
    def __init__(self, shelf_id: str, position: Tuple[float, float]):
        self.id = shelf_id
        self.position = position
        self.items = {}  # {Item: quantity}
        self.is_locked = False
        self.locked_by = None
        self.is_being_carried = False
        self.carried_by = None
        
    def lock(self, robot_id: str) -> bool:
        """Lock the shelf for a specific robot."""
        if not self.is_locked:
            self.is_locked = True
            self.locked_by = robot_id
            return True
        return False
    
    def unlock(self, robot_id: str) -> bool:
        """Unlock the shelf if locked by the specified robot."""
        if self.is_locked and self.locked_by == robot_id:
            self.is_locked = False
            self.locked_by = None
            return True
        return False
    
    def pickup(self, robot_id: str) -> bool:
        """Mark shelf as being carried by a robot."""
        if not self.is_being_carried and self.is_locked and self.locked_by == robot_id:
            self.is_being_carried = True
            self.carried_by = robot_id
            return True
        return False
    
    def drop(self, robot_id: str) -> bool:
        """Mark shelf as no longer being carried."""
        if self.is_being_carried and self.carried_by == robot_id:
            self.is_being_carried = False
            self.carried_by = None
            return True
        return False
    
    def add_item(self, item, quantity=1):
        if item in self.items:
            self.items[item] += quantity
        else:
            self.items[item] = quantity
    
    def remove_item(self, item, quantity=1):
        if item in self.items and self.items[item] >= quantity:
            self.items[item] -= quantity
            if self.items[item] == 0:
                del self.items[item]
            return True
        return False
    
    def get_inventory(self) -> Dict[Item, int]:
        """Get current inventory."""
        return self.items.copy()
    
    def has_item(self, item, quantity=1):
        """Check if shelf has at least the specified quantity of an item."""
        return self.items.get(item, 0) >= quantity
    
    def get_total_items(self):
        """Get total number of items on the shelf (across all item types)."""
        return sum(self.items.values())
    
    def get_item_quantity(self, item):
        """Get quantity of a specific item on the shelf."""
        return self.items.get(item, 0)
    
    def get_item_types(self) -> List[Item]:
        """Get list of unique item types on the shelf."""
        return list(self.items.keys())
    
    def clear_inventory(self):
        """Remove all items from the shelf."""
        self.items = {}

class ShelfManager:
    """Manages all shelves in the warehouse. Shelves are static and created only by the map."""
    
    def __init__(self):
        self.shelves: Dict[str, Shelf] = {}
        
    def add_shelf(self, position: Tuple[float, float]) -> str:
        """Adding shelves after map creation is not allowed. Shelves are static and created only by the map."""
        raise NotImplementedError("Shelves are static and can only be created by the WarehouseMap during initialization.")
    
    def get_shelf(self, shelf_id: str) -> Optional[Shelf]:
        """Get a shelf by ID."""
        return self.shelves.get(shelf_id)
    
    def get_shelf_at_position(self, position: Tuple[float, float]) -> Optional[Shelf]:
        """Get shelf at a specific position."""
        for shelf in self.shelves.values():
            if shelf.position == position:
                return shelf
        return None
    
    def get_available_shelves(self) -> List[Shelf]:
        """Get list of shelves that are not locked or being carried."""
        return [shelf for shelf in self.shelves.values() 
                if not shelf.is_locked and not shelf.is_being_carried]
    
    def get_shelves_with_item(self, item: Item) -> List[Shelf]:
        """Get shelves that contain a specific item."""
        return [shelf for shelf in self.shelves.values() 
                if item in shelf.items and shelf.items[item] > 0]
    
    def get_shelves_with_items(self, items: List[Item]) -> List[Tuple[Shelf, List[Item]]]:
        """Get shelves containing any of the specified items, with list of matched items."""
        result = []
        for shelf in self.shelves.values():
            matched_items = [item for item in items if item in shelf.items and shelf.items[item] > 0]
            if matched_items:
                result.append((shelf, matched_items))
        return result
    
    def lock_shelf(self, shelf_id: str, robot_id: str) -> bool:
        """Lock a shelf for a robot."""
        shelf = self.get_shelf(shelf_id)
        if shelf:
            return shelf.lock(robot_id)
        return False
    
    def unlock_shelf(self, shelf_id: str, robot_id: str) -> bool:
        """Unlock a shelf."""
        shelf = self.get_shelf(shelf_id)
        if shelf:
            return shelf.unlock(robot_id)
        return False
    
    def pickup_shelf(self, shelf_id: str, robot_id: str) -> bool:
        """Pickup a shelf."""
        shelf = self.get_shelf(shelf_id)
        if shelf:
            return shelf.pickup(robot_id)
        return False
    
    def drop_shelf(self, shelf_id: str, robot_id: str) -> bool:
        """Drop a shelf."""
        shelf = self.get_shelf(shelf_id)
        if shelf:
            return shelf.drop(robot_id)
        return False
    
    def get_total_inventory(self) -> Dict[Item, int]:
        """Get total inventory across all shelves."""
        total = {}
        for shelf in self.shelves.values():
            for item, quantity in shelf.items.items():
                total[item] = total.get(item, 0) + quantity
        return total 