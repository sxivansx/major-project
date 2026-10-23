"""The room-mapping session as the server sees it: one RoomMapper fed from the live LiDAR on its own
thread (docs/api.md §3.5). The LiDAR thread only drops the newest rotation into a one-slot mailbox;
if mapping falls behind, older rotations are skipped, never queued."""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from . import wire
from .slam2d import RoomMapper, scan_to_points
from .sources import ScanFrame

log = logging.getLogger(__name__)

STATE_PERIOD_S = 0.2
GRID_PERIOD_S = 0.5


class RoomSession:
    def __init__(self, publish: Callable[[str | bytes], None], publish_json: Callable[[dict], None], log_fn: Callable[[str, str], None]):
        self._publish = publish
        self._publish_json = publish_json
        self._log = log_fn
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self.mapper: RoomMapper | None = None
        self.state = "idle"
        self.note: str | None = None
        self._slot: tuple[ScanFrame, float] | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._was_lost = False
        self._t_state = self._t_grid = 0.0

    # ---- control ---------------------------------------------------------------------------------
    def start(self, cell_m: float = 0.05) -> dict:
        self.stop()
        with self._lock:
            self.mapper = RoomMapper(cell=cell_m)
            self.state = "mapping"
            self.note = None
            self._slot = None
            self._was_lost = False
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="room-map", daemon=True)
            self._thread.start()
        self._log("info", f"room mapping started ({cell_m * 100:.0f} cm cells)")
        self._emit(force=True)
        return self.snapshot()

    def stop(self) -> dict:
        t = self._thread
        if t is not None:
            self._stop.set()
            with self._cv:
                self._cv.notify_all()
            t.join(timeout=2)
        with self._lock:
            self._thread = None
            was = self.state
            self.state = "idle"
        if was == "mapping" and self.mapper is not None:
            self._log("info", f"room mapping stopped: {self.mapper.area_m2():.1f} m² mapped, {self.mapper.keyframes} keyframes")
            self._emit(force=True)
        return self.snapshot()

    # ---- lidar thread ----------------------------------------------------------------------------
    def feed(self, f: ScanFrame, phi_deg: float) -> None:
        if self.state != "mapping":
            return
        with self._cv:
            self._slot = (f, phi_deg)
            self._cv.notify()

    # ---- worker ----------------------------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            with self._cv:
                while self._slot is None and not self._stop.is_set():
                    self._cv.wait(timeout=0.5)
                item, self._slot = self._slot, None
            if item is None:
                continue
            f, phi = item
            m = self.mapper
            try:
                m.add_scan(scan_to_points(f.angle, f.range, f.quality))
            except Exception:  # noqa: BLE001  (a bad rotation must not end the session)
                log.exception("room mapping: scan failed")
                continue
            self.note = f"the LiDAR is tilted {phi:+.0f}°: room mapping needs it level (φ 0°)" if abs(phi) > 2 else None
            if m.lost != self._was_lost:
                self._was_lost = m.lost
                if m.lost:
                    self._log("warn", "room mapping lost track: move back to an area already on the map, slowly")
                else:
                    self._log("info", "room mapping back on track")
            self._emit()

    # ---- output ----------------------------------------------------------------------------------
    def _emit(self, force: bool = False) -> None:
        now = time.monotonic()
        if self.mapper is None:
            return
        if force or now - self._t_grid >= GRID_PERIOD_S:
            self._t_grid = now
            self._publish(self.grid_frame())
        if force or now - self._t_state >= STATE_PERIOD_S:
            self._t_state = now
            self._publish_json({"type": "room", **self.snapshot()})

    def grid_frame(self) -> bytes:
        m = self.mapper or RoomMapper(size_m=1.0)
        cls, x0, y0 = m.classes()
        return wire.encode_room(cls, m.cell, x0, y0)

    def snapshot(self) -> dict:
        m = self.mapper
        if m is None:
            return {"state": self.state, "pose": None, "path": [], "keyframes": 0, "scans": 0, "area_m2": 0.0, "match": None, "lost": False, "note": None}
        return {"state": self.state, **m.state(), "note": self.note}

    def info(self) -> dict:
        out = self.snapshot()
        if self.mapper is not None:
            cls, x0, y0 = self.mapper.classes()
            out["grid"] = {"w": int(cls.shape[1]), "h": int(cls.shape[0]), "cell": self.mapper.cell, "x0": x0, "y0": y0}
        else:
            out["grid"] = None
        return out
