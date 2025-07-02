"""
Interface for Order Sources - abstracts external order loading mechanisms.

The Order Source is responsible for:
1. Loading orders from external systems (files, APIs, databases)
2. Converting raw data into Order objects
3. Managing order scheduling and availability
4. Providing efficient polling mechanisms for JobsProcessor

Design Principles:
- **Abstraction**: Hide implementation details of different order sources
- **Flexibility**: Support multiple order source types (JSON, API, DB, etc.)
- **Efficiency**: Minimize unnecessary polling and data transfer
- **Thread Safety**: Safe for concurrent access from processor threads
"""
from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any, Iterator
from datetime import datetime
from enum import Enum
from .jobs_processor_interface import Order, OrderStatus


class OrderSourceType(Enum):
    """Types of order sources."""
    JSON_FILE = "json_file"
    REST_API = "rest_api"
    DATABASE = "database"
    MESSAGE_QUEUE = "message_queue"
    CSV_FILE = "csv_file"


class OrderSourceStatus(Enum):
    """Order source connection status."""
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    ERROR = "error"
    INITIALIZING = "initializing"


class OrderSourceError(Exception):
    """Raised when order source operations fail."""
    pass


class IOrderSource(ABC):
    """
    Interface for order source functionality.
    
    Responsibilities:
    - Load orders from external systems
    - Convert raw data into standardized Order objects
    - Manage order scheduling and filtering
    - Provide efficient polling for new orders
    - Handle connection management and error recovery
    
    **Thread Safety**: All methods are thread-safe for concurrent access.
    **Threading Model**:
    - get_due_orders(): PROCESSOR THREAD (continuous polling)
    - get_*() status methods: ANY THREAD (monitoring)
    - connect/disconnect(): MAIN THREAD (lifecycle management)
    
    **Polling Efficiency**: 
    - Implementations should minimize unnecessary work on repeated calls
    - Use timestamps, cursors, or change detection to avoid re-processing
    - Cache results when appropriate
    """
    
    @abstractmethod
    def get_due_orders(self, limit: Optional[int] = None) -> List[Order]:
        """
        Get orders that are due for processing (scheduled_time <= now).
        This is the main method called by JobsProcessor for continuous polling.
        
        **Efficiency**: Should only return orders that haven't been returned before,
        or orders whose status has changed. Use internal tracking to avoid duplicates.
        
        Args:
            limit: Maximum number of orders to return (None = no limit)
            
        Returns:
            List[Order]: Orders ready for processing, sorted by priority then scheduled_time
            
        Raises:
            OrderSourceError: If order loading fails
        """
        pass
    
    @abstractmethod
    def get_all_orders(self, status_filter: Optional[OrderStatus] = None) -> List[Order]:
        """
        Get all orders from the source, optionally filtered by status.
        Used for monitoring, debugging, and administrative operations.
        
        Args:
            status_filter: Only return orders with this status (None = all orders)
            
        Returns:
            List[Order]: All orders matching the filter
            
        Raises:
            OrderSourceError: If order loading fails
        """
        pass
    
    @abstractmethod
    def get_order_by_id(self, order_id: str) -> Optional[Order]:
        """
        Get a specific order by ID.
        
        Args:
            order_id: Unique identifier for the order
            
        Returns:
            Optional[Order]: Order if found, None otherwise
            
        Raises:
            OrderSourceError: If order loading fails
        """
        pass
    
    @abstractmethod
    def update_order_status(self, order_id: str, status: OrderStatus, 
                           metadata: Optional[Dict[str, Any]] = None) -> bool:
        """
        Update the status of an order in the source system.
        Critical for tracking order lifecycle and preventing duplicate processing.
        
        Args:
            order_id: Order to update
            status: New status
            metadata: Additional status information (error messages, processing time, etc.)
            
        Returns:
            bool: True if update succeeded, False if order not found
            
        Raises:
            OrderSourceError: If update operation fails
        """
        pass
    
    @abstractmethod
    def add_order(self, order: Order) -> bool:
        """
        Add a new order to the source.
        Used for API endpoints, manual order injection, or testing.
        
        Args:
            order: Order to add
            
        Returns:
            bool: True if order was added successfully
            
        Raises:
            OrderSourceError: If order cannot be added (duplicate ID, validation failure, etc.)
        """
        pass
    
    @abstractmethod
    def remove_order(self, order_id: str) -> bool:
        """
        Remove an order from the source.
        Used for order cancellation or cleanup operations.
        
        Args:
            order_id: Order to remove
            
        Returns:
            bool: True if order was removed, False if not found
            
        Raises:
            OrderSourceError: If removal operation fails
        """
        pass
    
    @abstractmethod
    def connect(self, config: Optional[Dict[str, Any]] = None) -> None:
        """
        Establish connection to the order source.
        
        Args:
            config: Source-specific configuration (file paths, API keys, DB connections, etc.)
            
        Raises:
            OrderSourceError: If connection fails
        """
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """
        Close connection to the order source and cleanup resources.
        """
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if the order source is currently connected and operational.
        
        Returns:
            bool: True if connected and ready, False otherwise
        """
        pass
    
    @abstractmethod
    def get_source_info(self) -> Dict[str, Any]:
        """
        Get information about the order source.
        
        Returns:
            Dict[str, Any]: Source metadata (type, location, last_updated, etc.)
        """
        pass
    
    @abstractmethod
    def get_source_stats(self) -> Dict[str, Any]:
        """
        Get statistics about order source usage.
        
        Returns:
            Dict[str, Any]: Usage statistics (total_orders, pending_orders, 
                           last_poll_time, error_count, etc.)
        """
        pass
    
    @abstractmethod
    def refresh(self) -> None:
        """
        Force refresh of order data from the source.
        Used to manually trigger data reload when needed.
        
        Raises:
            OrderSourceError: If refresh operation fails
        """
        pass
    
    @abstractmethod
    def validate_order_format(self, raw_data: Dict[str, Any]) -> bool:
        """
        Validate that raw order data matches expected format.
        Used for data validation before creating Order objects.
        
        Args:
            raw_data: Raw order data from external source
            
        Returns:
            bool: True if format is valid, False otherwise
        """
        pass
    
    @abstractmethod
    def get_polling_interval(self) -> float:
        """
        Get recommended polling interval for this source type.
        Different sources have different optimal polling frequencies.
        
        Returns:
            float: Recommended polling interval in seconds
        """
        pass 