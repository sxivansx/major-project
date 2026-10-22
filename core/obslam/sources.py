"""ScanSource: anything that produces one ScanFrame per lidar rotation on a background thread."""

from __future__ import annotations

import logging
import threading
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Callable

import numpy as np

log = logging.getLogger(__name__)


@dataclass
class ScanFrame:
    t: float  # unix seconds at grab
    angle: np.ndarray  # float32 deg, lidar native (clockwise)
    range: np.ndarray  # float32 m
    quality: np.ndarray  # uint8

    @property
    def n(self) -> int:
        return int(self.angle.shape[0])


class ScanStats:
    """Scan rate, points per revolution and range extents over the last few frames."""

    def __init__(self, window: int = 20):
        self._t: deque[float] = deque(maxlen=window)
        self.pts_per_rev = 0
        self.range_min = None
        self.range_max = None

    def update(self, f: ScanFrame) -> None:
        self._t.append(f.t)
        self.pts_per_rev = f.n
        if f.n:
            r = f.range[f.range > 0]
            if r.size:
                self.range_min = float(r.min())
                self.range_max = float(r.max())

    @property
    def hz(self) -> float | None:
        if len(self._t) < 2:
            return None
        span = self._t[-1] - self._t[0]
        return (len(self._t) - 1) / span if span > 0 else None

    def reset(self) -> None:
        self._t.clear()
        self.pts_per_rev = 0
        self.range_min = self.range_max = None


class ScanSource(ABC):
    name = "abstract"

    def __init__(self):
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._cb: Callable[[ScanFrame], None] | None = None
        self._on_exit: Callable[[str | None], None] | None = None
        self.error: str | None = None

    @abstractmethod
    def info(self) -> dict:
        """Device identity; called before start(). May raise RuntimeError."""

    @abstractmethod
    def _next_frame(self) -> ScanFrame | None:
        """Block until the next full rotation. None = source ended."""

    def _close(self) -> None:
        pass

    def start(self, on_frame: Callable[[ScanFrame], None], on_exit: Callable[[str | None], None] | None = None):
        self._cb = on_frame
        self._on_exit = on_exit
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"scan-{self.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._close()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=3)

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                f = self._next_frame()
                if f is None:
                    break
                if self._cb:
                    self._cb(f)
        except Exception as e:  # noqa: BLE001
            if not self._stop.is_set():
                self.error = str(e)
                log.exception("scan source %s died", self.name)
        finally:
            self._close()
            if self._on_exit and not self._stop.is_set():
                self._on_exit(self.error)


def now() -> float:
    return time.time()
