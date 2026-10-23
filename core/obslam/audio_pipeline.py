"""The acoustic layer as the server sees it: one audio source, the spike detector, direction
finding, the event ring, sound sources (events grouped by direction), channel health and the
kind-5 waveform preview (docs/api.md §5.3, §5.4, §5.6).

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
from .acoustic import Detection, Detector, SourceTracker, dbfs
from .audio_source import AudioSource, SimAudioSource, TcpAudioSource, UdpAudioSource, parse_hostport
from .audio_wire import FLAG_DROPPED, FLAG_I2S_TIMEOUT, AudioFrame, encode_preview
from .config import Config

log = logging.getLogger(__name__)

PREVIEW_POINTS = 100  # per channel: 100 ms at 1 kHz
PREVIEW_PERIOD_S = 0.05  # ≤ 20 Hz
CLIP_LEVEL = 32767
DC_ALPHA = 0.02  # per 10 ms frame: DC estimate settles in ~0.5 s
CLIP_HOLD_S = 2.0
LOSS_WINDOW_S = 10.0
LOSS_ALARM_FRAMES = 5  # in LOSS_WINDOW_S; one lost 10 ms frame inside a spike window spoils its bearing
MIN_CONFIDENCE_FOR_CANDIDATES = 0.1
MAP_MIN_INTERVAL_S = 2.0  # during a sweep the cloud changes every step; re-analyse at most this often
EXCLUDE_DB = 12.0  # a mic whose floor is this far above the median of the others is left out of DOA
INCLUDE_DB = 9.0  # …and taken back below this (hysteresis: a floor hovering at 12 dB must not flap)
HEALTH_PERIOD_S = 1.0
PRE_CUT_CONFIDENCE = 0.7  # onset right after a gap: the direct sound may be missing from the window


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

        self.tracker = SourceTracker()
        self._excluded: dict[int, str] = {}  # 0-based mic → why it is left out of direction finding
        self._health_t = 0.0
        self.gaps = 0

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
            elif source in ("tcp", "udp"):
                port = self._port_for(source, port, cfg.port)
                src = TcpAudioSource(port) if source == "tcp" else UdpAudioSource(port)
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
            self._excluded = {}
            self.gaps = 0
            self.cpu = {"frames": 0, "frame_s": 0.0, "events": 0, "event_s": 0.0}
            src.start(self._on_frame, self._on_exit)
        self._map_stop.clear()
        self._kick_map()
        self._log("info", f"audio connected: {source} {port}")

    @staticmethod
    def _port_for(source: str, port: str | None, configured: str) -> str:
        """The explicit port, else the configured one; a configured address of the other
        transport lends its host (config says udp://ip:8902, the operator picks tcp → ip:8901)."""
        default = 8901 if source == "tcp" else 8902
        want = f"{source}://"
        if not port:
            if configured.startswith(want):
                port = configured
            else:
                other = "udp" if source == "tcp" else "tcp"
                try:
                    host, _ = parse_hostport(configured, other, 0)
                except ValueError:
                    host = ""
                port = f"{want}{host}:{default}"
        elif "://" in port and not port.startswith(want):
            raise AudioError(f"source {source!r} needs a {want}host:{default} port, got {port!r}")
        try:
            host, p = parse_hostport(port, source, default)
        except ValueError:
            raise AudioError(f"bad audio port {port!r}: expected {want}host:{default}") from None
        if not host or not 0 < p < 65536:
            raise AudioError(f"bad audio port {port!r}: expected {want}host:{default}")
        return port if port.startswith(want) else f"{want}{host}:{p}"

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
                self._dc = None
            self.shift = f.shift
            # the samples on either side of a gap are not contiguous: no window may span it. A
            # frame with an I2S read timeout may itself be partly padding, so it is not analysed.
            i2s_timeout = bool(f.flags & FLAG_I2S_TIMEOUT)
            if f.lost_before or f.flags & FLAG_DROPPED or i2s_timeout:
                self.detector.reset_stream(f.lost_before * f.n)
                self.gaps += 1
            raw = f.samples
            peak = np.max(np.abs(raw.astype(np.int32)), axis=0)
            for c in np.nonzero(peak >= CLIP_LEVEL)[0][:4]:
                self._clip_t[int(c)] = f.t  # clipping is judged on the raw stream
            # INMP441 boards can sit on a large constant offset (one real mic read -17 dBFS of pure
            # DC); remove it before levels, preview and detection, or the offset reads as loudness
            # and raises the noise floor the trigger is measured against. ~0.5 s time constant.
            m = raw.mean(axis=0, dtype=np.float64)
            dc = getattr(self, "_dc", None)
            self._dc = m if dc is None else dc + DC_ALPHA * (m - dc)
            x = np.clip(np.rint(raw - self._dc), -32768, 32767).astype(np.int16)
            span = max(1, f.rate // 10)
            self._preview = np.concatenate([self._preview, x])[-span:]
            if i2s_timeout:
                self.detector.reset_stream(f.n)
                dets = []
            else:
                dets = self.detector.feed(x, f.t_us, f.t)
            if f.t - self._health_t >= HEALTH_PERIOD_S:
                self._health_t = f.t
                self._check_channels()
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

    def _check_channels(self) -> None:
        """Noisy-mic exclusion (docs/api.md §5.6). Called under `_lock`, about once a second."""
        det = self.detector
        if det is None or not det.warmed_up or det.channels != 4:
            return
        fl = np.asarray(dbfs(np.sqrt(det.floor)), float)
        new: dict[int, str] = {}
        excess = {}
        for i in range(4):
            others = float(np.median(np.delete(fl, i)))
            excess[i] = (fl[i] - others, others)
        # exclude the worst offender only: three mics are the minimum for a 2-D bearing
        worst = max(range(4), key=lambda i: excess[i][0])
        e, others = excess[worst]
        limit = INCLUDE_DB if worst in self._excluded else EXCLUDE_DB
        if e > limit:
            new[worst] = f"noise floor {fl[worst]:.1f} dBFS, {e:.1f} dB above the others' median ({others:.1f})"
        if set(new) != set(self._excluded):
            for i in set(new) - set(self._excluded):
                self._log("warn", f"audio: mic {i + 1} excluded from direction finding: {new[i]}")
            for i in set(self._excluded) - set(new):
                self._log("info", f"audio: mic {i + 1} back in direction finding")
            mask = np.ones(4, bool)
            mask[list(new)] = False
            det.set_sum_mask(mask)
        self._excluded = new

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
        with self._lock:
            use = [m for m in range(d.window.shape[1]) if m not in self._excluded]
        loc = acoustic.locate(
            d.window, d.rate, cfg.mics_mm, cfg.speed_of_sound, cfg.array_yaw_deg, cfg.calibrated, d.snr_db,
            offsets_us=cfg.channel_offsets_us, use_mics=use,
        )
        reasons = list(loc["reasons"])
        if d.cut_ms > 0:
            reasons.append(f"window cut short by a frame gap ({d.cut_ms:.0f} ms missing)")
        if d.pre_cut and loc["bearing_deg"] is not None:
            loc["confidence"] = round(loc["confidence"] * PRE_CUT_CONFIDENCE, 3)
            reasons.append("onset right after a frame gap: the start of the sound may be missing")
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
                "mics_used": loc.get("mics_used"),
            }
            src = self.tracker.add(ev)
            ev["noise"] = src is None
            ev["source_id"] = None if src is None else src.id
            self._events.append(ev)
            self._last_event_mono = time.monotonic()
            self.cpu["events"] += 1
            self.cpu["event_s"] += time.perf_counter() - t0
            self._event_cv.notify_all()
        self._send({"type": "audio_event", **ev})
        if not ev["noise"]:
            self._publish_sources()

    def _send(self, msg: dict) -> None:
        if self._publish_json:
            self._publish_json(msg)
        else:
            import json

            self._publish(json.dumps(msg))

    # ---- sources -------------------------------------------------------------------------------
    def sources(self) -> dict:
        findings, _ = self._cached_map()
        with self._lock:
            return self.tracker.snapshot(findings)

    def _publish_sources(self) -> None:
        self._send({"type": "audio_sources", "active": self.sources()["active"]})

    def tick(self, now: float | None = None) -> None:
        """About once a second (the rig's alarm ticker): retire quiet sources."""
        with self._lock:
            changed = self.tracker.expire(time.time() if now is None else now)
        if changed:
            self._publish_sources()

    # ---- events ----------------------------------------------------------------------------------
    def events(self, limit: int = 50) -> list[dict]:
        with self._lock:
            return list(reversed(self._events))[: max(0, limit)]

    def clear(self) -> None:
        with self._lock:
            self._events.clear()
            self.tracker.clear()
        self._publish_sources()

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
                # TCP is capped by the ESP32's 5,744 B send window per round trip; UDP is not
                "transport_note": "TCP over Wi-Fi is window-limited and drops frames on a slow link: udp://<scanner>:8902 is recommended" if self.source_kind == "tcp" else None,
                "events": len(self._events),
                "sources_active": len(self.tracker.active),
                "excluded_mics": [{"mic": i + 1, "reason": r} for i, r in sorted(self._excluded.items())],
                "gaps": self.gaps,
                "suppressed": None if self.detector is None else {
                    "echoes": self.detector.suppressed_echoes,
                    "duplicates": self.detector.duplicates,
                    "cut_by_gap": self.detector.dropped_by_gap,
                },
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

