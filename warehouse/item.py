from typing import Dict, Any, Optional
import uuid

class Item:
    """Represents an inventory item."""
    
    def __init__(self, item_id: str, name: str):
        self.id = item_id
        self.name = name

    def __repr__(self):
        return f"Item(id={self.id!r}, name={self.name!r})"

    def __eq__(self, other):
        if not isinstance(other, Item):
            return False
        return self.id == other.id and self.name == other.name

    def __hash__(self):
        return hash((self.id, self.name)) 