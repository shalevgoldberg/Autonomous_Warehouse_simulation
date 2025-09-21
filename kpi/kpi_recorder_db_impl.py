from __future__ import annotations

import logging
import queue
import threading
import time
import uuid
from typing import Optional, List

import psycopg2

from kpi.kpi_recorder_interface import IKpiRecorder, KpiEvent


class KpiRecorderDbImpl(IKpiRecorder):
    """
    Thread-safe, non-blocking KPI recorder that batches inserts to the
    simulation_metrics table on a background worker.

    Safety rules:
    - Never block callers from control/physics/motion threads.
    - Log every fallback or persistence failure explicitly.
    - Best-effort delivery; lost metrics must be logged as warnings.
    """

    def __init__(
        self,
        connection_getter,
        flush_interval_sec: float = 60.0,
        max_batch_size: int = 256,
        queue_capacity: int = 8192,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._get_connection = connection_getter
        self._flush_interval_sec = max(0.1, float(flush_interval_sec))
        self._max_batch_size = max(1, int(max_batch_size))
        self._queue: "queue.Queue[KpiEvent]" = queue.Queue(maxsize=max(128, int(queue_capacity)))
        self._logger = logger or logging.getLogger(__name__)

        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, name="KpiRecorderWorker", daemon=True)
        self._worker.start()

    def record_event(self, event: KpiEvent) -> None:
        try:
            self._queue.put_nowait(event)
        except queue.Full:
            # Explicitly log dropped events; do not block caller
            self._logger.warning(
                f"[KPI] Event queue full, dropping event: type={event.event_type} robot={event.robot_id}"
            )

    def flush(self) -> None:
        # Drain the queue synchronously (best-effort) and persist
        self._drain_and_persist(blocking=True)

    def stop(self) -> None:
        self._stop_event.set()
        self._worker.join(timeout=self._flush_interval_sec * 2)
        # Final flush after worker
        try:
            self.flush()
        except Exception:
            # Log but swallow to avoid shutdown exceptions
            self._logger.error("[KPI] Final flush failed during shutdown", exc_info=True)

    # --- Internal ---
    def _worker_loop(self) -> None:
        next_flush = time.monotonic() + self._flush_interval_sec
        while not self._stop_event.is_set():
            try:
                # Wait up to the flush interval for at least one item
                timeout = max(0.01, next_flush - time.monotonic())
                try:
                    event = self._queue.get(timeout=timeout)
                except queue.Empty:
                    event = None

                if event is not None:
                    self._queue.task_done()
                    self._persist_events([event])

                # Time-based flush to catch any remaining items in queue
                now = time.monotonic()
                if now >= next_flush:
                    self._drain_and_persist(blocking=False)
                    next_flush = now + self._flush_interval_sec

            except Exception:
                # Never let the worker die silently
                self._logger.error("[KPI] Worker loop error", exc_info=True)
                # Sleep briefly to avoid tight error loops
                time.sleep(0.1)

    def _drain_and_persist(self, blocking: bool) -> None:
        to_persist: List[KpiEvent] = []
        while len(to_persist) < self._max_batch_size:
            try:
                item = self._queue.get(block=blocking, timeout=0.01 if blocking else 0.0)
            except queue.Empty:
                break
            try:
                to_persist.append(item)
            finally:
                self._queue.task_done()

        if to_persist:
            self._persist_events(to_persist)

    def _persist_events(self, events: List[KpiEvent]) -> None:
        if not events:
            return
        try:
            with self._get_connection() as conn:
                with conn.cursor() as cur:
                    for event in events:
                        # Assign a per-event run id if not provided
                        event_id = event.simulation_run_id or str(uuid.uuid4())

                        # Main event counter row
                        try:
                            cur.execute(
                                """
                                INSERT INTO simulation_metrics (metric_name, metric_value, simulation_run_id)
                                VALUES (%s, %s, %s)
                                """,
                                (f"{event.event_type}_{event.robot_id}", 1.0, event_id),
                            )
                        except Exception:
                            self._logger.error(
                                f"[KPI] Failed to persist main event: type={event.event_type} robot={event.robot_id}",
                                exc_info=True,
                            )
                            continue  # Skip details for this event

                        # Details: numeric keys only, mirroring existing behavior
                        for key, value in event.event_data.items():
                            if isinstance(value, (int, float)):
                                # Global detail (legacy behavior)
                                try:
                                    cur.execute(
                                        """
                                        INSERT INTO simulation_metrics (metric_name, metric_value, simulation_run_id)
                                        VALUES (%s, %s, %s)
                                        """,
                                        (f"{event.event_type}_{key}", float(value), event_id),
                                    )
                                except Exception:
                                    self._logger.error(
                                        f"[KPI] Failed to persist event detail: type={event.event_type} key={key}",
                                        exc_info=True,
                                    )
                                # Per-robot detail (enables per-robot analytics)
                                try:
                                    cur.execute(
                                        """
                                        INSERT INTO simulation_metrics (metric_name, metric_value, simulation_run_id)
                                        VALUES (%s, %s, %s)
                                        """,
                                        (f"{event.event_type}_{key}_{event.robot_id}", float(value), event_id),
                                    )
                                except Exception:
                                    self._logger.error(
                                        f"[KPI] Failed to persist per-robot event detail: type={event.event_type} key={key} robot={event.robot_id}",
                                        exc_info=True,
                                    )

                conn.commit()
        except psycopg2.Error:
            # Database-level failure; log and drop this batch
            self._logger.error("[KPI] Database error while persisting KPI events", exc_info=True)
        except Exception:
            self._logger.error("[KPI] Unexpected error while persisting KPI events", exc_info=True)


