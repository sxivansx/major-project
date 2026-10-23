"""The room-mapping session as the server sees it: one RoomMapper fed from the live LiDAR on its own
thread (docs/api.md §3.5). The LiDAR thread only drops the newest rotation into a one-slot mailbox;
if mapping falls behind, older rotations are skipped, never queued."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Callable

import numpy as np

from . import wire
from .slam2d import RoomMapper, scan_to_points
from .sources import ScanFrame

log = logging.getLogger(__name__)

STATE_PERIOD_S = 0.2
GRID_PERIOD_S = 0.5
STATION_ROTATIONS = 5  # a still station is the median of this many rotations (0.5 s at 10 Hz)
STATION_WAIT_S = 3.0


class RoomError(Exception):
    pass


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
        self.held = False  # a tilt sweep is running: its tilted rotations must not reach the matcher
        self.mode = "walk"  # walk: every rotation is tracked; stations: only still captures update the map
        self.planned = 6
        self._recent: deque[tuple[ScanFrame, float]] = deque(maxlen=12)
        self._station_lock = threading.Lock()

    # ---- control ---------------------------------------------------------------------------------
    def start(self, cell_m: float = 0.05, mode: str = "walk", planned: int = 6) -> dict:
        self.stop()
        with self._lock:
            self.mapper = RoomMapper(cell=cell_m)
            self.state = "mapping"
            self.mode = mode
            self.planned = planned
            self.note = None
            self._slot = None
            self._recent.clear()
            self._was_lost = False
            self._stop.clear()
            if mode == "walk":
                self._thread = threading.Thread(target=self._run, name="room-map", daemon=True)
                self._thread.start()
        how = "walk mode" if mode == "walk" else f"station mode, {planned} stations planned"
        self._log("info", f"room mapping started ({how}, {cell_m * 100:.0f} cm cells)")
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
        if self.state != "mapping" or self.held:
            return
        if self.mode == "stations":
            self._recent.append((f, phi_deg))
            return
        with self._cv:
            self._slot = (f, phi_deg)
            self._cv.notify()

    def add_station(self) -> dict:
        """Station mode: median of the next few rotations, located on the map and added to it."""
        if self.state != "mapping" or self.mode != "stations" or self.mapper is None:
            raise RoomError("start mapping in station mode first")
        if not self._station_lock.acquire(blocking=False):
            raise RoomError("a station is already being added")
        try:
            t0 = time.time()
            deadline = time.monotonic() + STATION_WAIT_S
            while time.monotonic() < deadline:
                fresh = [(f, phi) for f, phi in list(self._recent) if f.t >= t0]
                if len(fresh) >= STATION_ROTATIONS:
                    break
                time.sleep(0.05)
            else:
                raise RoomError("no LiDAR rotations arriving: is the LiDAR connected and spinning?")
            if any(abs(phi) > 2 for _, phi in fresh):
                raise RoomError("the LiDAR is tilted: stations need it level (φ 0°)")
            fs = [f for f, _ in fresh[:STATION_ROTATIONS]]
            pts = scan_to_points(np.concatenate([f.angle for f in fs]), np.concatenate([f.range for f in fs]), np.concatenate([f.quality for f in fs]))
            t1 = time.perf_counter()
            res = self.mapper.add_station(pts)
            res["search_ms"] = round((time.perf_counter() - t1) * 1000)
        finally:
            self._station_lock.release()
        if res["ok"]:
            self._log("info", f"station {res['n']} of {self.planned} added at x {res['x']:.2f} m, y {res['y']:.2f} m ({res['inliers'] * 100:.0f}% matched)")
        else:
            self._log("warn", f"station not added: {res['reason']}")
        self._emit(force=True)
        return res

    def station_pose(self) -> tuple[float, float, float] | None:
        """Where the scanner stands in the room map, if mapping and on track: a sweep taken now is
        placed there, so sweeps from several stations join into one cloud."""
        m = self.mapper
        if self.state != "mapping" or m is None or m.lost or m.key_pose is None:
            return None
        return m.pose

    def hold(self, on: bool) -> None:
        self.held = on
        if not on:
            with self._lock:
                self._slot = None

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
            return {"state": self.state, "mode": self.mode, "planned": self.planned, "pose": None, "path": [], "keyframes": 0, "scans": 0, "area_m2": 0.0, "match": None, "lost": False, "note": None, "stations": [], "next": None}
        return {"state": self.state, "mode": self.mode, "planned": self.planned, **m.state(), "note": self.note}

    def info(self) -> dict:
        out = self.snapshot()
        if self.mapper is not None:
            cls, x0, y0 = self.mapper.classes()
            out["grid"] = {"w": int(cls.shape[1]), "h": int(cls.shape[0]), "cell": self.mapper.cell, "x0": x0, "y0": y0}
        else:
            out["grid"] = None
        return out
