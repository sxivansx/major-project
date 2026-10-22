"""The bench rig: one lidar source, one tilt servo, the live point cloud and the step-and-scan job.

Thread model: scan sources and the job run on their own threads; `publish` must be thread-safe
(the server hands in a loop.call_soon_threadsafe wrapper). All rig state is guarded by one RLock.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Callable

import numpy as np
from pydantic import BaseModel, Field

from . import wire
from .config import Config
from .geometry import lidar_to_base
from .grid import Grid, build_grid
from .servo import SerialTilt, SimTilt, TiltError, TiltSource
from .session import SessionStore
from .sim import SimScanSource
from .sources import ScanFrame, ScanSource, ScanStats

log = logging.getLogger(__name__)

POINTS_CHUNK = 50_000


class RigError(Exception):
    pass


class JobParams(BaseModel):
    name: str = "scan"
    phi_start: float = -60.0
    phi_end: float = 60.0
    phi_step: float = Field(2.0, gt=0)
    rotations: int = Field(1, ge=1)
    settle_ms: int = Field(300, ge=0)

    def phis(self) -> np.ndarray:
        if abs(self.phi_end - self.phi_start) < 1e-9:
            return np.array([self.phi_start])
        step = self.phi_step if self.phi_end > self.phi_start else -self.phi_step
        return np.arange(self.phi_start, self.phi_end + step / 2, step)


class Rig:
    def __init__(
        self,
        config: Config,
        sessions: SessionStore,
        publish: Callable[[str | bytes], None],
        bridge_path: Path | None = None,
        sim_hz: float = 10.0,
    ):
        self.config = config
        self.sessions = sessions
        self._publish = publish
        self.bridge_path = bridge_path
        self.sim_hz = sim_hz
        self._lock = threading.RLock()
        self._cv = threading.Condition()

        self.scan: ScanSource | None = None
        self.scan_source: str | None = None
        self.scan_port: str | None = None
        self.scan_baud: int | None = None
        self.scan_info: dict = {}
        self.scan_error: str | None = None
        self.stats = ScanStats()
        self.latest: ScanFrame | None = None
        self.seq = 0
        self._last_scan_pub = 0.0

        self.tilt: TiltSource | None = None

        self.job: dict | None = None
        self._job_thread: threading.Thread | None = None
        self._job_stop = threading.Event()
        self._job_counter = 0

        self._live: list[np.ndarray] = []
        self._live_cat: np.ndarray | None = None
        self.session_loaded: str | None = None

    # ---- publishing -------------------------------------------------------------------------
    def publish_json(self, obj: dict) -> None:
        self._publish(json.dumps(obj))

    def publish_status(self) -> None:
        self.publish_json({"type": "status", **self.status()})

    def log(self, level: str, msg: str) -> None:
        getattr(log, level if level != "warn" else "warning")(msg)
        self.publish_json({"type": "log", "level": level, "msg": msg, "t": time.time()})

    # ---- status ------------------------------------------------------------------------------
    def phi_now(self) -> float:
        with self._lock:
            if self.tilt is None:
                return 0.0
            return self.config.mount.servo_to_phi(self.tilt.servo_deg)

    def live_points(self) -> np.ndarray:
        with self._lock:
            if self._live_cat is None:
                self._live_cat = np.concatenate(self._live) if self._live else np.zeros((0, 3), np.float32)
            return self._live_cat

    def status(self) -> dict:
        with self._lock:
            lidar = {
                "connected": self.scan is not None,
                "source": self.scan_source,
                "port": self.scan_port,
                "baud": self.scan_baud,
                "model": self.scan_info.get("model"),
                "serial": self.scan_info.get("serial"),
                "firmware": self.scan_info.get("firmware"),
                "hardware": self.scan_info.get("hardware"),
                "health": self.scan_info.get("health"),
                "scan_hz": self.stats.hz,
                "pts_per_rev": self.stats.pts_per_rev,
                "range_min_m": self.stats.range_min,
                "range_max_m": self.stats.range_max,
                "error": self.scan_error,
            }
            t = self.tilt
            servo = {
                "connected": t is not None,
                "port": t.port if t else None,
                "servo_deg": t.servo_deg if t else None,
                "phi_deg": self.config.mount.servo_to_phi(t.servo_deg) if t else None,
                "min_deg": t.min_deg if t else None,
                "max_deg": t.max_deg if t else None,
                "range": t.range if t else None,
            }
            return {
                "lidar": lidar,
                "servo": servo,
                "job": dict(self.job) if self.job else None,
                "live_points": int(self.live_points().shape[0]),
                "session_loaded": self.session_loaded,
            }

    # ---- lidar -------------------------------------------------------------------------------
    def connect_lidar(self, source: str, port: str | None = None, baud: int = 1_000_000) -> None:
        with self._lock:
            if self.scan is not None:
                raise RigError("lidar already connected")
            if source == "sim":
                src: ScanSource = SimScanSource(self.phi_now, self.config.mount, hz=self.sim_hz)
            elif source == "s2":
                if not port:
                    raise RigError("port required for s2")
                from .s2 import S2ScanSource

                src = S2ScanSource(port, baud, self.bridge_path)
            else:
                raise RigError(f"unknown source {source!r}")
            try:
                info = src.info()
            except RuntimeError as e:
                raise RigError(str(e)) from e
            self.scan, self.scan_source, self.scan_port, self.scan_baud = src, source, port, baud
            self.scan_info, self.scan_error = info, None
            self.stats.reset()
            self.latest = None
            src.start(self._on_frame, self._on_scan_exit)
        self.log("info", f"lidar connected: {info.get('model')} on {port or 'sim'}")
        self.publish_status()

    def disconnect_lidar(self) -> None:
        with self._lock:
            src, self.scan = self.scan, None
            self.scan_source = self.scan_port = self.scan_baud = None
            self.scan_info = {}
            self.stats.reset()
        if src:
            src.stop()
            self.log("info", "lidar disconnected")
        self.publish_status()

    def _on_frame(self, f: ScanFrame) -> None:
        with self._lock:
            self.stats.update(f)
            self.latest = f
            self.seq += 1
            seq = self.seq
            phi = self.phi_now()
        with self._cv:
            self._cv.notify_all()
        now = time.monotonic()
        if now - self._last_scan_pub >= 0.08:
            self._last_scan_pub = now
            self._publish(wire.encode_scan(seq, f.t, phi, f.angle, f.range, f.quality))

    def _on_scan_exit(self, error: str | None) -> None:
        with self._lock:
            self.scan = None
            self.scan_error = error or "lidar stream ended"
        self.log("error", f"lidar stream ended: {error or 'no error reported'}")
        self.publish_status()

    def wait_frames(self, n: int, after: float, timeout: float = 5.0) -> list[ScanFrame]:
        """Collect n frames grabbed after `after` (unix s). Raises RigError on timeout."""
        got: list[ScanFrame] = []
        seen = self.seq
        deadline = time.monotonic() + timeout
        while len(got) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RigError("timed out waiting for lidar frames")
            with self._cv:
                if self.seq == seen:
                    self._cv.wait(remaining)
            with self._lock:
                if self.scan is None:
                    raise RigError("lidar disconnected")
                if self.seq != seen:
                    seen = self.seq
                    if self.latest is not None and self.latest.t >= after:
                        got.append(self.latest)
        return got

    # ---- servo -------------------------------------------------------------------------------
    def connect_servo(self, port: str) -> None:
        with self._lock:
            if self.tilt is not None:
                raise RigError("servo already connected")
            t: TiltSource = SimTilt() if port == "sim" else SerialTilt(port)
            try:
                t.connect()
            except TiltError as e:
                raise RigError(str(e)) from e
            self.tilt = t
        self.log("info", f"servo connected on {port}")
        self.publish_status()

    def disconnect_servo(self) -> None:
        with self._lock:
            t, self.tilt = self.tilt, None
        if t:
            t.disconnect()
            self.log("info", "servo disconnected")
        self.publish_status()

    def _tilt(self) -> TiltSource:
        if self.tilt is None:
            raise RigError("servo not connected")
        return self.tilt

    def move_servo(self, servo_deg: float | None = None, phi_deg: float | None = None) -> float:
        with self._lock:
            t = self._tilt()
            if servo_deg is None:
                if phi_deg is None:
                    raise RigError("servo_deg or phi_deg required")
                servo_deg = self.config.mount.phi_to_servo(phi_deg)
            if not (t.min_deg <= servo_deg <= t.max_deg):
                raise RigError(f"servo {servo_deg:.1f}° outside limits [{t.min_deg}, {t.max_deg}]")
            try:
                deg = t.move(servo_deg)
            except TiltError as e:
                raise RigError(str(e)) from e
        self.publish_status()
        return deg

    def home_servo(self) -> float:
        with self._lock:
            try:
                deg = self._tilt().home()
            except TiltError as e:
                raise RigError(str(e)) from e
        self.publish_status()
        return deg

    def servo_raw(self, line: str) -> str:
        with self._lock:
            t = self._tilt()
            try:
                reply = t.raw(line)
                if line.strip().split()[:1] in (["C"], ["L"], ["?"]):
                    t.refresh()
            except TiltError as e:
                raise RigError(str(e)) from e
        self.publish_status()
        return reply

    # ---- job ---------------------------------------------------------------------------------
    def start_job(self, params: JobParams) -> int:
        with self._lock:
            if self.job and self.job["state"] == "running":
                raise RigError("a job is already running")
            if self.scan is None:
                raise RigError("lidar not connected")
            self._tilt()
            phis = params.phis()
            for phi in (float(phis[0]), float(phis[-1])):
                s = self.config.mount.phi_to_servo(phi)
                if not (self.tilt.min_deg <= s <= self.tilt.max_deg):
                    raise RigError(f"phi {phi:.1f}° needs servo {s:.1f}°, outside limits")
            self._job_counter += 1
            job_id = self._job_counter
            self.job = {
                "job_id": job_id,
                "name": params.name,
                "state": "running",
                "step": 0,
                "n_steps": int(phis.shape[0]),
                "phi_deg": float(phis[0]),
                "points": 0,
                "started": time.time(),
                "eta_s": None,
                "error": None,
            }
            self._live.clear()
            self._live_cat = None
            self.session_loaded = None
            self._job_stop.clear()
            self._job_thread = threading.Thread(
                target=self._run_job, args=(params, job_id, phis), name=f"job-{job_id}", daemon=True
            )
            self._job_thread.start()
        self.log("info", f"job {job_id} '{params.name}' started: {len(phis)} steps")
        self.publish_status()
        return job_id

    def stop_job(self) -> None:
        self._job_stop.set()
        t = self._job_thread
        if t and t is not threading.current_thread():
            t.join(timeout=10)

    def _publish_job(self) -> None:
        with self._lock:
            if self.job:
                self.publish_json({"type": "job", **self.job})

    def _run_job(self, params: JobParams, job_id: int, phis: np.ndarray) -> None:
        cfg = self.config
        cols = {k: [] for k in ("step", "phi", "theta", "range", "quality", "t")}
        last_grid = 0.0
        t0 = time.monotonic()
        state, error = "done", None
        try:
            for i, phi in enumerate(phis):
                phi = float(phi)
                if self._job_stop.is_set():
                    state = "stopped"
                    break
                self.move_servo(phi_deg=phi)
                if params.settle_ms:
                    if self._job_stop.wait(params.settle_ms / 1000):
                        state = "stopped"
                        break
                after = time.time()
                frames = self.wait_frames(params.rotations + 1, after)[1:]  # drop the first (stale rotation)
                ths, rs, qs, ts = [], [], [], []
                for f in frames:
                    keep = (f.quality >= cfg.lidar.min_quality) & (f.range >= cfg.lidar.min_range_m)
                    keep &= f.range <= cfg.lidar.max_range_m
                    ths.append(f.angle[keep])
                    rs.append(f.range[keep])
                    qs.append(f.quality[keep])
                    ts.append(np.full(int(keep.sum()), f.t))
                theta = np.concatenate(ths) if ths else np.zeros(0, np.float32)
                rng = np.concatenate(rs) if rs else np.zeros(0, np.float32)
                xyz = lidar_to_base(theta, rng, phi, cfg.mount)
                cols["step"].append(np.full(theta.shape[0], i, np.uint16))
                cols["phi"].append(np.full(theta.shape[0], phi, np.float32))
                cols["theta"].append(theta.astype(np.float32))
                cols["range"].append(rng.astype(np.float32))
                cols["quality"].append(np.concatenate(qs) if qs else np.zeros(0, np.uint8))
                cols["t"].append(np.concatenate(ts) if ts else np.zeros(0, np.float64))
                with self._lock:
                    self._live.append(xyz)
                    self._live_cat = None
                    total = sum(a.shape[0] for a in self._live)
                    per_step = (time.monotonic() - t0) / (i + 1)
                    self.job.update(step=i + 1, phi_deg=phi, points=total, eta_s=per_step * (len(phis) - i - 1))
                self._publish(wire.encode_points(job_id, i, phi, xyz))
                self._publish_job()
                if time.monotonic() - last_grid >= 1.0:
                    last_grid = time.monotonic()
                    self._publish(wire.encode_grid(self.grid()))
        except RigError as e:
            state, error = "error", str(e)
        except Exception as e:  # noqa: BLE001
            log.exception("job %d crashed", job_id)
            state, error = "error", f"{type(e).__name__}: {e}"

        sid = None
        n_total = sum(a.shape[0] for a in cols["theta"])
        if n_total:
            arrays = {k: np.concatenate(v) for k, v in cols.items()}
            sid = self.sessions.new_id(params.name)
            meta = {
                "id": sid,
                "name": params.name,
                "created": time.time(),
                "n_points": int(n_total),
                "n_steps": int(len(phis)),
                "phi_start": params.phi_start,
                "phi_end": params.phi_end,
                "phi_step": params.phi_step,
                "rotations": params.rotations,
                "settle_ms": params.settle_ms,
                "state": state,
                "lidar": {k: self.scan_info.get(k) for k in ("model", "serial", "firmware")},
                "mount": cfg.mount.model_dump(),
            }
            self.sessions.save(meta, arrays)
        with self._lock:
            self.job.update(state=state, error=error, eta_s=0.0, session_id=sid)
            self.session_loaded = sid
        self._publish(wire.encode_grid(self.grid()))
        self._publish_job()
        if state == "error":
            self.log("error", f"job {job_id} failed: {error}")
        else:
            self.log("info", f"job {job_id} {state}: {n_total} points" + (f", saved {sid}" if sid else ""))
        self.publish_status()

    # ---- grid & sessions ---------------------------------------------------------------------
    def grid(self, cell: float | None = None, layer: str | None = None, z_min=None, z_max=None) -> Grid:
        g = self.config.grid
        return build_grid(
            self.live_points(),
            cell or g.cell_m,
            g.z_min_m if z_min is None else z_min,
            g.z_max_m if z_max is None else z_max,
            layer or g.layer,
        )

    def load_session(self, sid: str) -> None:
        if self.job and self.job["state"] == "running":
            raise RigError("stop the running job first")
        try:
            xyz = self.sessions.points(sid)
        except KeyError:
            raise RigError(f"no session {sid}") from None
        with self._lock:
            self._live = [xyz]
            self._live_cat = None
            self.session_loaded = sid
        for i in range(0, xyz.shape[0], POINTS_CHUNK):
            self._publish(wire.encode_points(0, i // POINTS_CHUNK, 0.0, xyz[i : i + POINTS_CHUNK]))
        self._publish(wire.encode_grid(self.grid()))
        self.log("info", f"session {sid} loaded: {xyz.shape[0]} points")
        self.publish_status()

    def clear_live(self) -> None:
        with self._lock:
            self._live.clear()
            self._live_cat = None
            self.session_loaded = None
        self.publish_status()

    def shutdown(self) -> None:
        self.stop_job()
        self.disconnect_lidar()
        self.disconnect_servo()
