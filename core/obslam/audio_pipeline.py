"""The acoustic layer as the server sees it: one audio source, the spike detector, direction
finding, the event ring and the kind-5 waveform preview (docs/api.md §5.3, §5.4).

Thread model: frames arrive on the source's thread and are processed there (≈0.1 ms per 10 ms
frame, a few ms per spike), so the rig's lock is never taken on that path. Findings for the
candidate lists come from a cache refreshed on a separate thread whenever the map changes,
because one analysis of a full sweep costs ~80 ms (more on big scenes) and must not stall audio.
Lock order: the rig may call into this object while holding its own lock; this object never
calls the rig while holding `_lock`.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Callable

import numpy as np

from . import acoustic
from .acoustic import Detection, Detector, dbfs
from .audio_source import AudioSource, SimAudioSource, TcpAudioSource, parse_tcp
from .audio_wire import AudioFrame, encode_preview
from .config import Config

log = logging.getLogger(__name__)

PREVIEW_POINTS = 100  # per channel: 100 ms at 1 kHz
PREVIEW_PERIOD_S = 0.05  # ≤ 20 Hz
CLIP_LEVEL = 32767
CLIP_HOLD_S = 2.0
LOSS_WINDOW_S = 10.0
LOSS_ALARM_FRAMES = 5  # in LOSS_WINDOW_S; one lost 10 ms frame inside a spike window spoils its bearing
MIN_CONFIDENCE_FOR_CANDIDATES = 0.1
MAP_MIN_INTERVAL_S = 2.0  # during a sweep the cloud changes every step; re-analyse at most this often


class AudioError(Exception):
    status_code = 400


class AudioConflict(AudioError):
    """The request is fine but the current source cannot serve it (sim/emit on a real stream)."""

    status_code = 409


class AudioPipeline:
    def __init__(
        self,
        config: Config,
        publish: Callable[[str | bytes], None],
        log_fn: Callable[[str, str], None],
        findings_fn: Callable[[], dict | None],
        publish_json: Callable[[dict], None] | None = None,
    ):
        self.config = config
        self._publish = publish
        self._publish_json = publish_json
        self._log = log_fn
        self._findings_fn = findings_fn
        self._lock = threading.RLock()

        self.source: AudioSource | None = None
        self.source_kind: str | None = None
        self.port: str | None = None
        self.error: str | None = None
        self._connected_at = 0.0
        self.detector: Detector | None = None
        self.rate: int | None = None
        self.channels: int | None = None
        self.shift: int | None = None
        self._preview = np.zeros((0, 4), np.int16)
        self._preview_seq = 0
        self._last_preview = 0.0
        self._levels: list[float] | None = None
        self._clip_t = [0.0] * 4
        self._loss_hist: deque[tuple[float, int, int]] = deque()

        self._events: deque[dict] = deque(maxlen=200)
        self._event_id = 0
        self._event_cv = threading.Condition(self._lock)
        self._last_event_mono = 0.0
        self.cpu = {"frames": 0, "frame_s": 0.0, "events": 0, "event_s": 0.0}

        self._map: dict | None = None
        self._map_t: float | None = None
        self._map_cost_s: float | None = None
        self._map_dirty = True
        self._map_last_run = 0.0
        self._map_thread: threading.Thread | None = None
        self._map_lock = threading.Lock()
        self._map_stop = threading.Event()

    # ---- connection ----------------------------------------------------------------------------
    def connect(self, source: str, port: str | None = None) -> None:
        cfg = self.config.audio
        with self._lock:
            if self.source is not None:
                raise AudioError("audio already connected")
            if source == "sim":
                src: AudioSource = SimAudioSource(cfg.mics_mm, cfg.array_yaw_deg, c=cfg.speed_of_sound)
                port = "sim"
            elif source == "tcp":
                port = port or cfg.port
                try:
                    host, p = parse_tcp(port)
                except ValueError:
                    raise AudioError(f"bad audio port {port!r}: expected tcp://host:8901") from None
                if not host or not 0 < p < 65536:
                    raise AudioError(f"bad audio port {port!r}: expected tcp://host:8901")
                src = TcpAudioSource(port)
            else:
                raise AudioError(f"unknown audio source {source!r}")
            self.source, self.source_kind, self.port, self.error = src, source, port, None
            self._connected_at = time.time()
            self.detector = None
            self.rate = self.channels = self.shift = None
            self._preview = np.zeros((0, 4), np.int16)
            self._levels = None
            self._clip_t = [0.0] * 4
            self._loss_hist.clear()
            self.cpu = {"frames": 0, "frame_s": 0.0, "events": 0, "event_s": 0.0}
            src.start(self._on_frame, self._on_exit)
        self._map_stop.clear()
        self._kick_map()
        self._log("info", f"audio connected: {source} {port}")

    def disconnect(self) -> None:
        with self._lock:
            src, self.source = self.source, None
            self.source_kind = self.port = None
            self.detector = None
            self._levels = None
        self._map_stop.set()
        if src:
            src.stop()
            self._log("info", "audio disconnected")

    def _on_exit(self, error: str | None) -> None:
        with self._lock:
            self.source = None
            self.error = error or "audio stream ended"
        self._log("error", f"audio stream ended: {error or 'no error reported'}")

    def config_changed(self) -> None:
        cfg = self.config.audio
        with self._lock:
            if self.detector:
                self.detector.configure(cfg.trigger_db, cfg.min_gap_ms)
            if isinstance(self.source, SimAudioSource):
                # the simulated array *is* whatever the config says (a perfectly measured board)
                self.source.mics_mm = np.asarray(cfg.mics_mm, float)
                self.source.array_yaw_deg = cfg.array_yaw_deg
                self.source.c = cfg.speed_of_sound

    # ---- frames ----------------------------------------------------------------------------------
    def _on_frame(self, f: AudioFrame) -> None:
        t0 = time.perf_counter()
        cfg = self.config.audio
        preview = None
        with self._lock:
            if self.source is None:
                return  # a frame that raced a disconnect
            if self.detector is None or f.rate != self.rate or f.channels != self.channels:
                self.rate, self.channels = f.rate, f.channels
                self.detector = Detector(f.rate, f.channels, cfg.trigger_db, cfg.min_gap_ms)
                self._preview = np.zeros((0, f.channels), np.int16)
            self.shift = f.shift
            if f.lost_before:
                # the samples on either side of a gap are not contiguous: no window may span it
                self.detector.reset_stream()
            x = f.samples
            peak = np.max(np.abs(x.astype(np.int32)), axis=0)
            for c in np.nonzero(peak >= CLIP_LEVEL)[0][:4]:
                self._clip_t[int(c)] = f.t
            span = max(1, f.rate // 10)
            self._preview = np.concatenate([self._preview, x])[-span:]
            dets = self.detector.feed(x, f.t_us, f.t)
            now = time.monotonic()
            if self._preview.shape[0] >= span and now - self._last_preview >= PREVIEW_PERIOD_S:
                self._last_preview = now
                preview = self._encode_preview(f.t)
        if preview is not None:
            self._publish(preview)
        for d in dets:
            self._handle(d)
        with self._lock:
            self.cpu["frames"] += 1
            self.cpu["frame_s"] += time.perf_counter() - t0

    def _encode_preview(self, t: float) -> bytes:
        p = self._preview
        ch = p.shape[1]
        per = p.shape[0] // PREVIEW_POINTS
        blk = p[: per * PREVIEW_POINTS].reshape(PREVIEW_POINTS, per, ch)
        mins = blk.min(axis=1).T
        maxs = blk.max(axis=1).T
        rms = np.sqrt(np.mean(p.astype(np.float64) ** 2, axis=0))
        levels = dbfs(rms)
        self._levels = [round(float(v), 1) for v in levels]
        self._preview_seq += 1
        return encode_preview(self._preview_seq, t, levels, mins, maxs)

    def _handle(self, d: Detection) -> None:
        t0 = time.perf_counter()
        cfg = self.config.audio
        loc = acoustic.locate(
            d.window, d.rate, cfg.mics_mm, cfg.speed_of_sound, cfg.array_yaw_deg, cfg.calibrated, d.snr_db,
            offsets_us=cfg.channel_offsets_us,
        )
        reasons = list(loc["reasons"])
        findings, map_t = self._cached_map()
        cands: list[dict] = []
        if loc["bearing_deg"] is not None:
            if loc["confidence"] < MIN_CONFIDENCE_FOR_CANDIDATES:
                reasons.append("confidence too low to point at the map")
            elif findings is None:
                reasons.append("no analysed map yet: run a sweep or load a session")
            else:
                cands = acoustic.candidates(loc["bearing_deg"], loc["bearing_sigma_deg"], findings)
        with self._lock:
            self._event_id += 1
            ev = {
                "id": self._event_id,
                "t": round(d.t, 3),
                "t_us": d.t_us,
                "peak_dbfs": d.peak_dbfs,
                "snr_db": d.snr_db,
                "bearing_deg": loc["bearing_deg"],
                "bearing_sigma_deg": loc["bearing_sigma_deg"],
                "elevation_deg": loc["elevation_deg"],
                "confidence": loc["confidence"],
                "tdoa_us": loc["tdoa_us"],
                "candidates": cands,
                "label": "sustained" if d.sustained else "impulse",
                "calibrated": bool(cfg.calibrated),
                "reasons": reasons,
                "reason": "; ".join(reasons) if reasons else None,
                "coherence": loc.get("coherence"),
                "residual_us": loc.get("residual_us"),
                "map_age_s": None if map_t is None else round(time.time() - map_t, 1),
            }
            self._events.append(ev)
            self._last_event_mono = time.monotonic()
            self.cpu["events"] += 1
            self.cpu["event_s"] += time.perf_counter() - t0
            self._event_cv.notify_all()
        msg = {"type": "audio_event", **ev}
        if self._publish_json:
            self._publish_json(msg)
        else:
            import json

            self._publish(json.dumps(msg))

    # ---- events ----------------------------------------------------------------------------------
    def events(self, limit: int = 50) -> list[dict]:
        with self._lock:
            return list(reversed(self._events))[: max(0, limit)]

    def clear(self) -> None:
        with self._lock:
            self._events.clear()

    def emit_sim(self, bearing_deg: float, elevation_deg: float = 0.0, snr_db: float = 20.0, kind: str = "knock", timeout: float = 2.0) -> dict:
        """Inject a source into the simulator and wait for the detector's verdict: the AudioEvent
        it produced, or {"detected": false, "reason": …} if it stayed under the trigger."""
        with self._lock:
            src = self.source
            last_id = self._event_id
        if not isinstance(src, SimAudioSource):
            raise AudioConflict("sim/emit needs the simulated audio source (POST /api/audio/connect {\"source\":\"sim\"})")
        if not -90.0 <= elevation_deg <= 90.0:
            raise AudioError("elevation_deg must be within ±90")
        # right after connect the detector is still learning the floor (0.5 s); a test knock
        # injected then would be silently missed, so wait for it rather than report a false miss
        warm_deadline = time.monotonic() + 2.0
        while time.monotonic() < warm_deadline:
            with self._lock:
                if self.source is not src or (self.detector is not None and self.detector.warmed_up):
                    break
            time.sleep(0.02)
        # land after the previous event's dead time (its onset was ~45 ms before it was handled),
        # or back-to-back test knocks would be swallowed by design
        since_ms = (time.monotonic() - self._last_event_mono) * 1000 + 45
        delay_ms = max(20.0, self.config.audio.min_gap_ms + 30 - since_ms)
        try:
            truth = src.emit(bearing_deg, elevation_deg, snr_db, kind, delay_ms=delay_ms)
        except ValueError as e:
            raise AudioError(str(e)) from None
        # the event's onset is the first trigger block at or after the injected onset; allow one
        # block plus the pre-trigger before it, in case the onset straddles a block boundary
        earliest = truth["t_us"] - 10_000
        deadline = time.monotonic() + timeout + delay_ms / 1000
        with self._event_cv:
            while True:
                for ev in reversed(self._events):
                    if ev["id"] <= last_id:
                        break
                    if ev["t_us"] >= earliest:
                        return ev
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self.source is not src:
                    break
                self._event_cv.wait(remaining)
        why = "stayed under the trigger" if self.detector is not None and self.detector.warmed_up else "detector still learning the noise floor"
        return {"detected": False, "reason": f"{kind} at {snr_db:.0f} dB SNR {why} (trigger {self.config.audio.trigger_db:.0f} dB over the floor)"}

    # ---- map cache -------------------------------------------------------------------------------
    def map_changed(self) -> None:
        """The live cloud changed (sweep step, sweep end, session loaded, cleared)."""
        self._map_dirty = True
        if self.source is not None:
            self._kick_map()

    def _kick_map(self) -> None:
        with self._map_lock:
            if self._map_thread is not None and self._map_thread.is_alive():
                return  # the running refresher sees the dirty flag and goes again
            self._map_thread = threading.Thread(target=self._map_loop, name="audio-map", daemon=True)
            self._map_thread.start()

    def _map_loop(self) -> None:
        while self._map_dirty and not self._map_stop.is_set():
            wait = MAP_MIN_INTERVAL_S - (time.monotonic() - self._map_last_run)
            if wait > 0 and self._map_stop.wait(wait):
                return
            self._map_dirty = False
            self._map_last_run = time.monotonic()
            t0 = time.perf_counter()
            try:
                f = self._findings_fn()
            except Exception:  # noqa: BLE001  (a bad cloud must not take the audio layer down)
                log.exception("findings for audio candidates failed")
                continue
            with self._map_lock:
                self._map, self._map_t, self._map_cost_s = f, time.time(), time.perf_counter() - t0

    def _cached_map(self) -> tuple[dict | None, float | None]:
        with self._map_lock:
            return self._map, (self._map_t if self._map is not None else None)

    def refresh_map_now(self) -> None:
        """Synchronous refresh, for tests and for callers that already run off the audio path."""
        t0 = time.perf_counter()
        f = self._findings_fn()
        with self._map_lock:
            self._map, self._map_t, self._map_cost_s = f, time.time(), time.perf_counter() - t0
            self._map_dirty = False

    # ---- status & alarms ---------------------------------------------------------------------------
    def status(self) -> dict:
        cfg = self.config.audio
        with self._lock:
            src = self.source
            if src is None:
                return {"connected": False}
            floor = self.detector.floor if self.detector else None
            now = time.time()
            ch = self.channels or 4
            c = self.cpu
            return {
                "connected": src is not None,
                "source": self.source_kind,
                "port": self.port,
                "link": "up" if src.link_up else "connecting",
                "rate": self.rate,
                "channels": self.channels,
                "shift": self.shift,
                "frames": src.frames,
                "lost_frames": src.lost_frames,
                "i2s_timeouts": src.i2s_timeouts,
                "levels_dbfs": self._levels,
                "noise_dbfs": None if floor is None else [round(float(v), 1) for v in dbfs(np.sqrt(floor))],
                "clipping": [bool(now - t < CLIP_HOLD_S) for t in self._clip_t[:ch]],
                "calibrated": bool(cfg.calibrated),
                "array_yaw_deg": cfg.array_yaw_deg,
                "note": None if cfg.calibrated else "uncalibrated: channel order, array yaw to the LiDAR 0° mark and capture offsets not yet verified by a click test",
                "events": len(self._events),
                "error": src.error,
                "map_age_s": None if self._map_t is None else round(now - self._map_t, 1),
                "map_cost_ms": None if self._map_cost_s is None else round(self._map_cost_s * 1000, 1),
                "cpu_us_per_frame": round(c["frame_s"] / c["frames"] * 1e6, 1) if c["frames"] else None,
                "cpu_ms_per_event": round(c["event_s"] / c["events"] * 1e3, 2) if c["events"] else None,
            }

    def alarms(self) -> dict[str, str]:
        """Keyed like Rig.check_alarms: the key is the kind, the text may change."""
        out: dict[str, str] = {}
        now = time.time()
        with self._lock:
            src = self.source
            if src is None:
                self._loss_hist.clear()
                return out
            grace = now - self._connected_at > 5.0
            if not src.link_up:
                if grace:
                    out["audio_link"] = f"audio stream not connected ({self.port}): {src.error or 'connecting'}"
            elif grace:
                age = now - (src.last_frame_t or self._connected_at)
                if age > 2.0:
                    out["audio_stalled"] = f"no audio frames for {age:.0f} s"
            self._loss_hist.append((now, src.lost_frames, src.i2s_timeouts + src.dropped_flags))
            while self._loss_hist and now - self._loss_hist[0][0] > LOSS_WINDOW_S:
                self._loss_hist.popleft()
            _, lost0, fw0 = self._loss_hist[0]
            lost, fw = src.lost_frames - lost0, src.i2s_timeouts + src.dropped_flags - fw0
            if lost >= LOSS_ALARM_FRAMES:
                out["audio_lost"] = f"audio: {lost} frames lost on the link in {LOSS_WINDOW_S:.0f} s"
            if fw >= LOSS_ALARM_FRAMES:
                out["audio_i2s"] = f"audio: ESP32 reported {fw} I2S timeouts/drops in {LOSS_WINDOW_S:.0f} s"
            clipped = [i + 1 for i, t in enumerate(self._clip_t) if now - t < CLIP_HOLD_S]
            if clipped:
                hint = f" (A shift {self.shift + 1})" if self.shift is not None and self.shift < 16 else ""
                out["audio_clip"] = f"audio clipping on mic {', '.join(map(str, clipped))}: lower the gain{hint}"
        return out

