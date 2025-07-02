"""
JSON File Order Source Implementation.

Loads orders from JSON files and converts them to Order objects.
Designed for Phase 2 development and testing with sample_orders.json.

Features:
- File watching for automatic updates
- Efficient polling with change detection
- Order status persistence
- Comprehensive error handling
"""
import json
import os
import threading
import time
import logging
from typing import List, Optional, Dict, Any
from datetime import datetime, timedelta
from pathlib import Path
from dataclasses import asdict

from interfaces.order_source_interface import (
    IOrderSource, OrderSourceError, OrderSourceType, OrderSourceStatus
)
from interfaces.jobs_processor_interface import (
    Order, OrderItem, OrderStatus, Priority
)


class JsonOrderSource(IOrderSource):
    """
    JSON file-based order source implementation.
    
    Features:
    - Loads orders from JSON files in standard Order format
    - Tracks processed orders to avoid duplicates
    - File modification detection for efficient polling
    - Thread-safe operations with proper locking
    
    JSON Format Support:
    - Full format: [{"order_id": "...", "items": [...], "scheduled_time": "...", ...}]
    """
    
    def __init__(self, file_path: str, order_id_prefix: str = "auto"):
        """
        Initialize JSON order source.
        
        Args:
            file_path: Path to JSON file containing orders
            order_id_prefix: Prefix for auto-generated order IDs
        """
        self.file_path = Path(file_path)
        self.order_id_prefix = order_id_prefix
        self.logger = logging.getLogger(f"JsonOrderSource.{self.file_path.name}")
        
        # Thread safety
        self._lock = threading.RLock()
        
        # Connection state
        self._connected = False
        self._status = OrderSourceStatus.DISCONNECTED
        
        # Order tracking
        self._orders: Dict[str, Order] = {}
        self._processed_order_ids: set = set()
        self._last_file_mtime: Optional[float] = None
        self._last_poll_time: Optional[datetime] = None
        
        # Statistics
        self._stats = {
            'total_orders_loaded': 0,
            'total_polls': 0,
            'file_reload_count': 0,
            'error_count': 0,
            'last_error': None
        }
    
    def get_due_orders(self, limit: Optional[int] = None) -> List[Order]:
        """
        Get orders that are due for processing.
        Efficiently handles file changes and avoids duplicate processing.
        """
        with self._lock:
            self._stats['total_polls'] += 1
            self._last_poll_time = datetime.now()
            
            if not self._connected:
                raise OrderSourceError("Order source not connected")
            
            try:
                # Check if file has changed
                self._reload_if_changed()
                
                # Filter orders due for processing
                now = datetime.now()
                due_orders = []
                
                for order in self._orders.values():
                    # Only return orders that are:
                    # 1. Due for processing (scheduled_time <= now)
                    # 2. Still pending
                    # 3. Haven't been returned before
                    if (order.scheduled_time <= now and 
                        order.status == OrderStatus.PENDING and
                        order.order_id not in self._processed_order_ids):
                        due_orders.append(order)
                
                # Sort by priority (URGENT first) then by scheduled time
                due_orders.sort(key=lambda o: (
                    self._priority_sort_key(o.priority),
                    o.scheduled_time
                ))
                
                # Apply limit
                if limit is not None:
                    due_orders = due_orders[:limit]
                
                # Mark as returned to avoid duplicates
                for order in due_orders:
                    self._processed_order_ids.add(order.order_id)
                
                return due_orders
                
            except Exception as e:
                self._stats['error_count'] += 1
                self._stats['last_error'] = str(e)
                raise OrderSourceError(f"Failed to get due orders: {e}")
    
    def get_all_orders(self, status_filter: Optional[OrderStatus] = None) -> List[Order]:
        """Get all orders, optionally filtered by status."""
        with self._lock:
            if not self._connected:
                raise OrderSourceError("Order source not connected")
            
            try:
                self._reload_if_changed()
                
                if status_filter is None:
                    return list(self._orders.values())
                else:
                    return [order for order in self._orders.values() 
                           if order.status == status_filter]
                           
            except Exception as e:
                self._stats['error_count'] += 1
                self._stats['last_error'] = str(e)
                raise OrderSourceError(f"Failed to get all orders: {e}")
    
    def get_order_by_id(self, order_id: str) -> Optional[Order]:
        """Get a specific order by ID."""
        with self._lock:
            if not self._connected:
                raise OrderSourceError("Order source not connected")
            
            try:
                self._reload_if_changed()
                return self._orders.get(order_id)
                
            except Exception as e:
                self._stats['error_count'] += 1
                self._stats['last_error'] = str(e)
                raise OrderSourceError(f"Failed to get order {order_id}: {e}")
    
    def update_order_status(self, order_id: str, status: OrderStatus, 
                           metadata: Optional[Dict[str, Any]] = None) -> bool:
        """Update order status in memory (JSON files are read-only for status)."""
        with self._lock:
            if order_id not in self._orders:
                return False
            
            # Update in-memory order
            order = self._orders[order_id]
            order.status = status
            
            if metadata:
                if order.metadata is None:
                    order.metadata = {}
                order.metadata.update(metadata)
            
            # For JSON sources, we track status in memory only
            # Production systems would persist this back to the source
            return True
    
    def add_order(self, order: Order) -> bool:
        """Add order to in-memory collection (JSON files are typically read-only)."""
        with self._lock:
            if order.order_id in self._orders:
                return False  # Duplicate order ID
            
            self._orders[order.order_id] = order
            return True
    
    def remove_order(self, order_id: str) -> bool:
        """Remove order from in-memory collection."""
        with self._lock:
            if order_id in self._orders:
                del self._orders[order_id]
                self._processed_order_ids.discard(order_id)
                return True
            return False
    
    def connect(self, config: Optional[Dict[str, Any]] = None) -> None:
        """Connect to the JSON file source."""
        with self._lock:
            try:
                self._status = OrderSourceStatus.INITIALIZING
                
                # Validate file exists
                if not self.file_path.exists():
                    raise OrderSourceError(f"Order file not found: {self.file_path}")
                
                # Load initial orders
                self._load_orders_from_file()
                
                self._connected = True
                self._status = OrderSourceStatus.CONNECTED
                self.logger.info(f"Connected to order source: {self.file_path}")
                
            except Exception as e:
                self._status = OrderSourceStatus.ERROR
                self._stats['error_count'] += 1
                self._stats['last_error'] = str(e)
                raise OrderSourceError(f"Failed to connect to {self.file_path}: {e}")
    
    def disconnect(self) -> None:
        """Disconnect from the order source."""
        with self._lock:
            self._connected = False
            self._status = OrderSourceStatus.DISCONNECTED
            self.logger.info("Disconnected from order source")
    
    def is_connected(self) -> bool:
        """Check if connected."""
        with self._lock:
            return self._connected
    
    def get_source_info(self) -> Dict[str, Any]:
        """Get source information."""
        with self._lock:
            return {
                'type': OrderSourceType.JSON_FILE.value,
                'file_path': str(self.file_path),
                'file_size': self.file_path.stat().st_size if self.file_path.exists() else 0,
                'last_modified': datetime.fromtimestamp(
                    self.file_path.stat().st_mtime
                ) if self.file_path.exists() else None,
                'status': self._status.value,
                'order_count': len(self._orders)
            }
    
    def get_source_stats(self) -> Dict[str, Any]:
        """Get usage statistics."""
        with self._lock:
            stats = self._stats.copy()
            stats.update({
                'pending_orders': len([o for o in self._orders.values() 
                                     if o.status == OrderStatus.PENDING]),
                'processed_orders': len(self._processed_order_ids),
                'last_poll_time': self._last_poll_time,
                'connected': self._connected
            })
            return stats
    
    def refresh(self) -> None:
        """Force reload from file."""
        with self._lock:
            if not self._connected:
                raise OrderSourceError("Order source not connected")
            
            try:
                self._load_orders_from_file()
                self.logger.info("Orders refreshed from file")
                
            except Exception as e:
                self._stats['error_count'] += 1
                self._stats['last_error'] = str(e)
                raise OrderSourceError(f"Failed to refresh orders: {e}")
    
    def validate_order_format(self, raw_data: Dict[str, Any]) -> bool:
        """Validate order data format."""
        try:
            # Check for required fields in standard Order format
            required_fields = ['order_id', 'items', 'scheduled_time']
            return all(field in raw_data for field in required_fields)
                
        except Exception:
            return False
    
    def get_polling_interval(self) -> float:
        """Get recommended polling interval for JSON files."""
        return 2.0  # 2 seconds - reasonable for file-based sources
    
    # Private methods
    
    def _reload_if_changed(self) -> None:
        """Reload orders if file has been modified."""
        if not self.file_path.exists():
            return
        
        current_mtime = self.file_path.stat().st_mtime
        
        if (self._last_file_mtime is None or 
            current_mtime > self._last_file_mtime):
            self._load_orders_from_file()
    
    def _load_orders_from_file(self) -> None:
        """Load orders from JSON file."""
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                raw_data = json.load(f)
            
            if not isinstance(raw_data, list):
                raise ValueError("JSON file must contain a list of orders")
            
            # Clear existing orders (full reload)
            old_order_count = len(self._orders)
            self._orders.clear()
            
            # Convert raw data to Order objects
            for i, item_data in enumerate(raw_data):
                if not self.validate_order_format(item_data):
                    self.logger.warning(f"Invalid order format at index {i}: {item_data}")
                    continue
                
                try:
                    order = self._convert_to_order(item_data, i)
                    self._orders[order.order_id] = order
                except Exception as e:
                    self.logger.error(f"Failed to convert order at index {i}: {e}")
            
            # Update tracking
            self._last_file_mtime = time.time()
            self._stats['total_orders_loaded'] = len(self._orders)
            self._stats['file_reload_count'] += 1
            
            self.logger.info(
                f"Loaded {len(self._orders)} orders from {self.file_path} "
                f"(was {old_order_count})"
            )
            
        except Exception as e:
            raise OrderSourceError(f"Failed to load orders from {self.file_path}: {e}")
    
    def _convert_to_order(self, raw_data: Dict[str, Any], index: int) -> Order:
        """Convert raw JSON data to Order object."""
        return self._convert_format(raw_data)
    

    
    def _convert_format(self, raw_data: Dict[str, Any]) -> Order:
        """Convert order format to Order object."""
        # Convert items
        items = []
        for item_data in raw_data['items']:
            items.append(OrderItem(
                item_id=item_data['item_id'],
                quantity=item_data['quantity']
            ))
        
        # Parse scheduled_time
        if isinstance(raw_data['scheduled_time'], str):
            scheduled_time = datetime.fromisoformat(raw_data['scheduled_time'])
        else:
            scheduled_time = datetime.fromtimestamp(raw_data['scheduled_time'])
        
        # Parse priority
        priority = Priority(raw_data.get('priority', 'normal'))
        
        return Order(
            order_id=raw_data['order_id'],
            items=items,
            scheduled_time=scheduled_time,
            priority=priority,
            customer_id=raw_data.get('customer_id'),
            deadline=datetime.fromisoformat(raw_data['deadline']) if raw_data.get('deadline') else None,
            status=OrderStatus.PENDING,
            metadata=raw_data.get('metadata')
        )
    
    def _priority_sort_key(self, priority: Priority) -> int:
        """Get sort key for priority (lower number = higher priority)."""
        priority_order = {
            Priority.URGENT: 0,
            Priority.HIGH: 1,
            Priority.NORMAL: 2,
            Priority.LOW: 3
        }
        return priority_order.get(priority, 2) 