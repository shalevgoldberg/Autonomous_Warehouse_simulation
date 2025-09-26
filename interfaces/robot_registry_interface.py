"""
Robot Registry Interface

Defines a clean abstraction for registering robots and persisting their
transient runtime state (position and battery) without leaking database
details to callers. This supports SOLID by isolating responsibilities and
keeping higher layers independent of storage specifics.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class RobotIdentity:
    robot_id: str
    name: Optional[str] = None


@dataclass(frozen=True)
class RobotRuntimeState:
    robot_id: str
    position: Optional[Tuple[float, float, float]]  # (x, y, theta) or None when unplaced
    battery_level: float  # 0.0 - 1.0


class IRobotRegistry(ABC):
    """Abstraction for robot identity and runtime state persistence."""

    @abstractmethod
    def list_registered_robots(self) -> List[RobotIdentity]:
        """
        List all registered robots (deterministic order by name, then id).
        """

    @abstractmethod
    def get_robot_state(self, robot_id: str) -> Optional[RobotRuntimeState]:
        """Get current runtime state; returns None if not present."""

    @abstractmethod
    def upsert_robot_state(self, robot_id: str, position: Optional[Tuple[float, float, float]],
                           battery_level: float) -> None:
        """
        Create or update runtime state for robot. Position may be None to clear placement.
        """

    @abstractmethod
    def clear_robot_state(self, robot_id: str) -> None:
        """Clear position for robot, keeping battery as-is (position becomes NULL)."""

    @abstractmethod
    def clear_all_positions(self) -> int:
        """Nullify position for all robots. Returns affected row count."""

    @abstractmethod
    def charge_all_to_full(self) -> int:
        """Set battery_level = 1.0 for all robots. Returns affected row count."""

    @abstractmethod
    def register_robot(self, robot_id: str, name: Optional[str] = None) -> bool:
        """
        Register a new robot. Returns True if created, False if already exists.
        If name is None, generates a default name.
        """

    @abstractmethod
    def delete_robot(self, robot_id: str) -> bool:
        """
        Delete a robot and its runtime state. Returns True if deleted, False if missing.
        """

    @abstractmethod
    def delete_all_robots(self) -> int:
        """Delete all robots and their runtime state. Returns number of robots deleted."""

    @abstractmethod
    def register_robots(self, count: int, name_prefix: str = "Robot") -> List[RobotIdentity]:
        """
        Register 'count' robots with unique IDs and sensible default names.
        Returns the list of created identities.
        """


