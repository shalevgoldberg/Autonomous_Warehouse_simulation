"""
Interface for Graph Generator - builds navigation graphs from warehouse CSV data.

This interface is responsible for processing manual CSV files containing warehouse
layout information and generating a connected graph that can be used for navigation.
"""
from abc import ABC, abstractmethod
from typing import Dict, Set, List, Tuple, Optional
from dataclasses import dataclass
from pathlib import Path

from .navigation_types import Point, LaneDirection


@dataclass
class GraphNode:
    """Represents a node in the navigation graph."""
    node_id: str
    position: Point
    directions: List[LaneDirection]  # Available exit directions
    is_conflict_box: bool = False
    conflict_box_id: Optional[str] = None


@dataclass
class GraphEdge:
    """Represents an edge in the navigation graph."""
    from_node: str
    to_node: str
    direction: LaneDirection
    distance: float


@dataclass
class ConflictBox:
    """Represents a conflict box with participating cells."""
    box_id: str
    position: Point
    width: float
    height: float
    participating_nodes: Set[str]
    directions: List[LaneDirection]


@dataclass
class NavigationGraph:
    """Complete navigation graph for the warehouse."""
    nodes: Dict[str, GraphNode]  # node_id -> GraphNode
    edges: Dict[str, Set[str]]  # node_id -> set of connected node_ids
    conflict_boxes: Dict[str, ConflictBox]  # box_id -> ConflictBox
    position_to_node: Dict[Tuple[float, float], str]  # (x, y) -> node_id


class IGraphGenerator(ABC):
    """
    Interface for generating navigation graphs from warehouse CSV data.
    
    Responsibilities:
    - Parse manual CSV files containing warehouse layout
    - Build connected graph of drivable cells
    - Identify and process conflict boxes
    - Generate navigation graph for path planning
    
    **Thread Safety**: All methods are thread-safe.
    **Usage**: Called once during system initialization.
    """
    
    @abstractmethod
    def generate_graph(self, lanes_csv: Path, boxes_csv: Path) -> NavigationGraph:
        """
        Generate navigation graph from CSV files.
        
        Args:
            lanes_csv: Path to lanes CSV file
            boxes_csv: Path to conflict boxes CSV file
            
        Returns:
            NavigationGraph: Complete navigation graph
            
        Raises:
            ValueError: If CSV files are invalid or incomplete
            FileNotFoundError: If CSV files don't exist
        """
        pass
    
    @abstractmethod
    def validate_connectivity(self, graph: NavigationGraph) -> List[str]:
        """
        Validate that the graph is properly connected.
        
        Args:
            graph: Navigation graph to validate
            
        Returns:
            List of validation warnings/errors
        """
        pass
    
    @abstractmethod
    def get_graph_statistics(self, graph: NavigationGraph) -> Dict[str, int]:
        """
        Get statistics about the generated graph.
        
        Args:
            graph: Navigation graph to analyze
            
        Returns:
            Dictionary with graph statistics (nodes, edges, conflict_boxes, etc.)
        """
        pass 