"""
Interface for Visualization - handles rendering and display.
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional


class VisualizationError(Exception):
    """Raised when visualization operations fail."""
    pass


class IVisualization(ABC):
    """
    Interface for visualization functionality.
    
    Responsibilities:
    - Render warehouse state and robot positions
    - Display live dashboard with KPI data
    - Handle visualization updates and rendering
    - Completely decoupled from core simulation logic
    
    **Thread Safety**: All methods are thread-safe internally.
    **Threading Model**:
    - visualize(): VISUALIZATION THREAD ONLY (20-30 FPS)
    - initialize(), shutdown(): MAIN THREAD (application lifecycle)
    - is_active(): ANY THREAD (status check)
    """
    
    @abstractmethod
    def visualize(self) -> None:
        """
        Main visualization method - renders current state.
        Should be called at visualization frequency (20-30 FPS).
        
        Raises:
            VisualizationError: If visualization fails
        """
        pass
    
    @abstractmethod
    def initialize(self, config: Optional[Dict[str, Any]] = None) -> None:
        """
        Initialize the visualization system.
        
        Args:
            config: Optional configuration parameters
            
        Raises:
            VisualizationError: If initialization fails
        """
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """
        Shutdown the visualization system and cleanup resources.
        """
        pass
    
    @abstractmethod
    def is_active(self) -> bool:
        """
        Check if visualization is currently active.
        
        Returns:
            bool: True if active, False otherwise
        """
        pass 