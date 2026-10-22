"""Simulated rig: a ray-cast room with rubble boxes, using the same mount geometry as the real transform."""

from __future__ import annotations

import time
from typing import Callable

import numpy as np

from .config import MountConfig
from .geometry import lidar_rays_base
from .sources import ScanFrame, ScanSource

# Room 6 m × 4 m, 3 m ceiling, lidar at the centre. Boxes are (x0, y0, z0, x1, y1, z1) in base frame.
ROOM = {
    "floor_z": 0.0,
    "ceil_z": 3.0,
    "boxes": [
        (-3.1, -2.1, 0.0, -3.0, 2.1, 3.0),  # wall x = -3
        (3.0, -2.1, 0.0, 3.1, 2.1, 3.0),  # wall x = +3
        (-3.1, -2.1, 0.0, 3.1, -2.0, 3.0),  # wall y = -2
        (-3.1, 2.0, 0.0, 3.1, 2.1, 3.0),  # wall y = +2
        (0.8, 0.9, 0.0, 1.6, 1.5, 0.5),  # rubble
        (-1.5, -1.8, 0.0, -0.9, -1.0, 0.9),
        (1.9, -1.2, 0.0, 2.4, -0.4, 1.2),
        (-2.2, 0.6, 0.0, -1.7, 1.4, 0.3),
    ],
}


def raycast(origin: np.ndarray, dirs: np.ndarray, room: dict = ROOM) -> np.ndarray:
    """Distance along each unit ray to the first surface, inf if nothing."""
    n = dirs.shape[0]
    best = np.full(n, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for pz in (room["floor_z"], room["ceil_z"]):
            t = (pz - origin[2]) / dirs[:, 2]
            ok = np.isfinite(t) & (t > 1e-6) & (t < best)
            best = np.where(ok, t, best)
        for box in room["boxes"]:
            lo = np.asarray(box[:3]) - origin
            hi = np.asarray(box[3:]) - origin
            t1 = lo / dirs
            t2 = hi / dirs
            tmin = np.minimum(t1, t2).max(axis=1)
            tmax = np.maximum(t1, t2).min(axis=1)
            hit = (tmax >= tmin) & (tmax > 1e-6)
            t = np.where(tmin > 1e-6, tmin, tmax)
            ok = hit & (t < best)
            best = np.where(ok, t, best)
    return best


class SimScanSource(ScanSource):
    """Produces S2-like frames (3,200 points at 10 Hz) of ROOM as seen at the current tilt."""

    name = "sim"

    def __init__(
        self,
        phi_provider: Callable[[], float],
        mount: MountConfig,
        hz: float = 10.0,
        n_points: int = 3200,
        max_range: float = 30.0,
        noise_m: float = 0.012,
        dropout: float = 0.015,
        seed: int = 0,
    ):
        super().__init__()
        self.phi_provider = phi_provider
        self.mount = mount
        self.hz = hz
        self.max_range = max_range
        self.noise_m = noise_m
        self.dropout = dropout
        self.angles = (np.arange(n_points) * (360.0 / n_points)).astype(np.float32)
        self.rng = np.random.default_rng(seed)
        self._next_t = None

    def info(self) -> dict:
        return {"model": "SIM", "model_id": 0, "firmware": "0.0", "hardware": 0, "serial": "SIMULATED", "health": "good"}

    def scan_at(self, phi_deg: float) -> ScanFrame:
        origin, dirs = lidar_rays_base(self.angles, phi_deg, self.mount)
        r = raycast(origin, dirs)
        r = r + self.rng.normal(0.0, self.noise_m, r.shape[0])
        miss = ~np.isfinite(r) | (r > self.max_range) | (r <= 0.05)
        miss |= self.rng.random(r.shape[0]) < self.dropout
        q = self.rng.integers(40, 64, r.shape[0]).astype(np.uint8)
        keep = ~miss
        return ScanFrame(time.time(), self.angles[keep], r[keep].astype(np.float32), q[keep])

    def _next_frame(self) -> ScanFrame | None:
        period = 1.0 / self.hz
        if self._next_t is None:
            self._next_t = time.monotonic()
        delay = self._next_t - time.monotonic()
        if delay > 0:
            if self._stop.wait(delay):
                return None
        self._next_t += period
        if self._next_t < time.monotonic() - period:
            self._next_t = time.monotonic()  # fell behind, don't burst
        return self.scan_at(float(self.phi_provider()))
