from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Any, Optional


@dataclass(frozen=True)
class KpiEvent:
    """
    Represents a single KPI event emitted by the system.

    Only numeric values are persisted to the simulation_metrics table for
    backward compatibility. Non-numeric values should be converted to suitable
    numeric codes upstream if needed.
    """
    event_type: str
    robot_id: str
    event_data: Dict[str, Any]
    simulation_run_id: Optional[str] = None


class IKpiRecorder(ABC):
    """
    Interface for recording KPI events and periodic snapshots.

    Implementations must be thread-safe and must not block callers from
    control/motion/physics threads. Persisting work should be offloaded to a
    background worker where applicable.
    """

    @abstractmethod
    def record_event(self, event: KpiEvent) -> None:
        """
        Enqueue a KPI event for persistence.

        Implementations must not raise to callers on persistence errors; they
        must log failures and return without blocking.
        """
        pass

    @abstractmethod
    def flush(self) -> None:
        """
        Best-effort flush of any buffered KPI events.

        This method may block briefly but should not be called from high-
        frequency loops. It's intended for shutdown hooks or test scenarios.
        """
        pass


