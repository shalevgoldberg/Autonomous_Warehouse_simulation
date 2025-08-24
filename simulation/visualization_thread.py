"""
Visualization thread manager.

Runs a provided `IVisualization` implementation at a target frame rate in its
own daemon thread, decoupled from physics and control threads.
"""
from typing import Optional
import threading
import time

from interfaces.visualization_interface import IVisualization, VisualizationError


class VisualizationThread:
    """Manages a visualization loop at a target FPS in a daemon thread."""
    def __init__(self, visualization: IVisualization, fps: float = 30.0):
        self._visualization = visualization
        self._fps = fps
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="VisualizationThread", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def is_running(self) -> bool:
        return self._running

    def _loop(self) -> None:
        period = 1.0 / self._fps if self._fps > 0 else 0.033
        next_step = time.perf_counter() + period
        while self._running:
            try:
                self._visualization.visualize()
            except VisualizationError as e:
                # Log sporadically to avoid I/O overhead
                # In production, replace with structured logging
                pass
            next_step += period
            sleep_time = next_step - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_step = time.perf_counter() + period



