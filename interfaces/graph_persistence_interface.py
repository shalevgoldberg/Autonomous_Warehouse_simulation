"""
Interface for persisting a navigation graph into the database.

Responsibilities:
- Generate or accept a navigation graph
- Persist conflict boxes, nodes, and edges in the correct order
- Report counts and timing

Thread safety:
- Intended for one-time initialization (not performance critical)
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path


@dataclass
class GraphPersistenceResult:
    """Result of a graph persistence operation."""
    nodes_persisted: int
    edges_persisted: int
    boxes_persisted: int
    cleared_existing: bool


class IGraphPersistence(ABC):
    """Abstraction for persisting a navigation graph into the database."""

    @abstractmethod
    def persist_from_csv(self, csv_path: Path, clear_existing: bool = False) -> GraphPersistenceResult:
        """
        Generate a navigation graph from a CSV file and persist it in the database.

        Args:
            csv_path: Path to the warehouse CSV file
            clear_existing: If True, clear existing graph tables first

        Returns:
            GraphPersistenceResult with counts of persisted records
        """
        raise NotImplementedError


