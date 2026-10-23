"""AudioSource: anything that produces 4-channel audio frames on a background thread.

TcpAudioSource talks to the ESP32-S3 on port 8901 (docs/api.md §5.2); SimAudioSource renders the
same frames from a model of the array, with sources injected on demand.
"""

from __future__ import annotations

import logging
import math
import socket
import threading
import time
from abc import ABC
from typing import Callable

import numpy as np

from .acoustic import PAIRS, PAIR_KEYS, bearing_to_array
from .audio_wire import FLAG_DROPPED, FLAG_I2S_TIMEOUT, AudioFrame, FrameReader, SeqTracker

log = logging.getLogger(__name__)


class AudioSource(ABC):
    name = "abstract"

    def __init__(self):
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._cb: Callable[[AudioFrame], None] | None = None
        self._on_exit: Callable[[str | None], None] | None = None
        self.error: str | None = None
        self.link_up = False
        self.frames = 0
        self.lost_frames = 0
        self.i2s_timeouts = 0
        self.dropped_flags = 0
        self.connects = 0
        self.last_frame_t: float | None = None

    def start(self, on_frame: Callable[[AudioFrame], None], on_exit: Callable[[str | None], None] | None = None) -> None:
        self._cb = on_frame
        self._on_exit = on_exit
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"audio-{self.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._close()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=3)

    def _close(self) -> None:
        pass

    def _next_frame(self) -> AudioFrame | None:
        raise NotImplementedError

    def _deliver(self, f: AudioFrame) -> None:
        self.frames += 1
        self.lost_frames += f.lost_before
        if f.flags & FLAG_I2S_TIMEOUT:
            self.i2s_timeouts += 1
        if f.flags & FLAG_DROPPED:
            self.dropped_flags += 1
        self.last_frame_t = f.t
        if self._cb:
            self._cb(f)

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                f = self._next_frame()
                if f is None:
                    break
                self._deliver(f)
        except Exception as e:  # noqa: BLE001
            if not self._stop.is_set():
                self.error = str(e)
                log.exception("audio source %s died", self.name)
        finally:
            self._close()
            if self._on_exit and not self._stop.is_set():
                self._on_exit(self.error)


def parse_tcp(port: str, default_port: int = 8901) -> tuple[str, int]:
    hp = port[len("tcp://") :] if port.startswith("tcp://") else port
    host, _, p = hp.rpartition(":")
    if not host:
        return hp, default_port
    return host, int(p)


class TcpAudioSource(AudioSource):
    """Client for the ESP32's audio stream. Never gives up: Wi-Fi drops, ESP reboots and another
    client taking the stream all look the same from here (EOF or silence), and the answer is
    always to reconnect with a growing back-off. The rig never waits on any of it."""

    name = "tcp"

    def __init__(self, port: str, connect_timeout: float = 3.0, stall_s: float = 2.0, max_backoff_s: float = 5.0):
        super().__init__()
        self.host, self.port = parse_tcp(port)
        self.connect_timeout = connect_timeout
        self.stall_s = stall_s
        self.max_backoff_s = max_backoff_s
        self.tracker = SeqTracker()
        self.reader = FrameReader()
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    def _close(self) -> None:
        with self._lock:
            s, self._sock = self._sock, None
        if s is not None:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            s.close()
        self.link_up = False

    def _run(self) -> None:
        backoff = 0.25
        while not self._stop.is_set():
            try:
                s = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
                s.settimeout(0.5)  # short, so stop() and stall detection stay responsive
                with self._lock:
                    if self._stop.is_set():
                        s.close()
                        return
                    self._sock = s
                self.link_up, self.error = True, None
                self.connects += 1
                # a new connection may restart the firmware's counter; the gap across a reconnect
                # is not "lost frames" in the sense the alarm means (it is downtime)
                self.tracker.reset()
                self.reader = FrameReader()
                log.info("audio stream connected to %s:%d", self.host, self.port)
                last_data = time.monotonic()
                while not self._stop.is_set():
                    try:
                        data = s.recv(65536)
                    except socket.timeout:
                        if time.monotonic() - last_data > self.stall_s:
                            raise OSError(f"no audio for {self.stall_s:.0f} s") from None
                        continue
                    if not data:
                        raise OSError("ESP32 closed the audio stream")
                    last_data = time.monotonic()
                    backoff = 0.25
                    now = time.time()
                    for f in self.reader.feed(data):
                        f.t = now
                        f.lost_before = self.tracker.update(f.seq)
                        self._deliver(f)
            except OSError as e:
                if self._stop.is_set():
                    return
                self.error = str(e) or type(e).__name__
                log.warning("audio link %s:%d: %s; retrying in %.1f s", self.host, self.port, self.error, backoff)
            except Exception as e:  # noqa: BLE001  (a bug in a callback must not kill the link forever)
                if self._stop.is_set():
                    return
                self.error = f"{type(e).__name__}: {e}"
                log.exception("audio link crashed; reconnecting")
            finally:
                self._close()
            if self._stop.wait(backoff):
                return
            backoff = min(backoff * 2, self.max_backoff_s)


# ---- simulator --------------------------------------------------------------------------------
def _knock(rate: int, rng: np.random.Generator) -> np.ndarray:
    """A knock on rubble: a short broadband click exciting a few damped resonances."""
    n = int(0.06 * rate)
    t = np.arange(n) / rate
    s = rng.normal(0, 1, n) * np.exp(-t / 0.0006)  # the contact click, ~1 ms
    for lo, hi, amp in ((250, 600, 1.0), (700, 1500, 0.8), (1700, 3200, 0.6), (3500, 6500, 0.35)):
        f = rng.uniform(lo, hi)
        tau = rng.uniform(0.003, 0.012)
        s += amp * np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi)) * np.exp(-t / tau)
    s *= 1 - np.exp(-t / 0.0002)  # no step at sample 0
    return s


def _voice(rate: int, rng: np.random.Generator) -> np.ndarray:
    """A shout: a glottal harmonic series with vibrato through three formants, plus breath."""
    dur = 0.45
    n = int(dur * rate)
    t = np.arange(n) / rate
    f0 = rng.uniform(150, 250) * (1 + 0.03 * np.sin(2 * np.pi * 5.5 * t))
    ph = 2 * np.pi * np.cumsum(f0) / rate
    s = np.zeros(n)
    formants = ((rng.uniform(600, 800), 120), (rng.uniform(1100, 1400), 150), (rng.uniform(2400, 2900), 250))
    for h in range(1, 40):
        fh = h * float(np.mean(f0))
        if fh > 6000:
            break
        g = sum(math.exp(-0.5 * ((fh - fc) / bw) ** 2) for fc, bw in formants) + 0.15 / h
        s += g * np.sin(h * ph + rng.uniform(0, 2 * np.pi))
    s += 0.1 * rng.normal(0, 1, n)
    env = np.minimum(1, t / 0.025) * np.minimum(1, np.maximum(0, (dur - t) / 0.06))
    return s * env


def _delayed(S: np.ndarray, f: np.ndarray, delay_s: float, nfft: int) -> np.ndarray:
    """Exact fractional delay by a linear phase in the frequency domain."""
    return np.fft.irfft(S * np.exp(-2j * np.pi * f * delay_s), nfft)


def plane_wave_delays(mics_m: np.ndarray, azimuth_deg: float, elevation_deg: float, c: float) -> np.ndarray:
    """Arrival time of a far-field wave at each mic, relative to the array origin (s)."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    u = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    return -(np.asarray(mics_m, float) @ u) / c


def synth_event(
    mics_mm,
    bearing_deg: float,
    elevation_deg: float = 0.0,
    snr_db: float = 20.0,
    kind: str = "knock",
    rate: int = 48000,
    noise_rms: float = 10.4,
    array_yaw_deg: float = 0.0,
    c: float = 343.0,
    reverb: bool = True,
    rng: np.random.Generator | None = None,
    gain_mismatch_db: float = 0.5,
    channel_delays_us=None,
) -> tuple[np.ndarray, dict]:
    """The source's contribution at each mic (float, LSB; no background noise), and the truth.

    `bearing_deg` is a base bearing (clockwise from base +x). Direct path: the source waveform
    delayed per mic by the exact far-field geometry. With `reverb`, six early reflections arrive
    from other directions 1.5–15 ms later, each with its own exact per-mic delays, followed by a
    diffuse tail that is independent at every mic, so GCC sees competing coherent peaks and a
    decorrelated floor rather than one clean delay. `channel_delays_us` adds a fixed capture
    delay per channel (an I2S slot skew), which the truth's tdoas do *not* include. SNR is the loudest 2.5 ms block of the
    direct path over the noise power, the definition the Detector reports."""
    rng = rng or np.random.default_rng()
    mics_m = np.asarray(mics_mm, float) / 1000.0
    src = _knock(rate, rng) if kind == "knock" else _voice(rate, rng)
    blk = max(1, int(round(rate * 0.0025)))
    nb = src.shape[0] // blk
    e_max = float(np.max(np.mean(src[: nb * blk].reshape(nb, blk) ** 2, axis=1)))
    src *= math.sqrt(noise_rms**2 * 10 ** (snr_db / 10) / e_max)

    az = bearing_to_array(bearing_deg, array_yaw_deg)
    d = plane_wave_delays(mics_m, az, elevation_deg, c)
    cap = np.zeros(4) if channel_delays_us is None else np.asarray(channel_delays_us, float) * 1e-6
    lead = 0.001  # every mic's delay is positive, and the onset is not at sample 0
    tail_s = 0.25 if reverb else 0.0
    nfft = 1 << int(math.ceil(math.log2(src.shape[0] + int((lead + 0.02 + tail_s) * rate))))
    f = np.fft.rfftfreq(nfft, 1.0 / rate)
    S = np.fft.rfft(src, nfft)
    gains = 10 ** (rng.normal(0, gain_mismatch_db, 4) / 20)
    out = np.zeros((nfft, 4))
    for m in range(4):
        out[:, m] = _delayed(S, f, lead + d[m] - d.min() + cap[m], nfft)
    if reverb:
        lp = 1.0 / (1.0 + (f / 3000.0) ** 2)  # surfaces absorb highs
        for r in range(6):
            raz = rng.uniform(0, 360)
            rel = rng.uniform(-30, 60)
            extra = rng.uniform(0.0015, 0.015)
            g = rng.uniform(0.15, 0.45) * (0.0015 / extra) ** 0.3
            dr = plane_wave_delays(mics_m, raz, rel, c)
            for m in range(4):
                out[:, m] += g * _delayed(S * lp, f, lead + extra + dr[m] - d.min() + cap[m], nfft)
        nt = int(tail_s * rate)
        tt = np.arange(nt) / rate
        env = np.exp(-tt / 0.035) * (tt > 0.003)
        for m in range(4):
            h = rng.normal(0, 1, nt) * env
            h *= 0.35 / (np.linalg.norm(h) or 1.0)
            out[:, m] += _delayed(S * lp * np.fft.rfft(h, nfft), f, cap[m], nfft)
    out *= gains[None, :]
    end = src.shape[0] + int((lead + 0.002 + tail_s + cap.max()) * rate)
    t_rel = d - d.min()
    truth = {
        "bearing_deg": bearing_deg % 360.0,
        "array_azimuth_deg": az,
        "elevation_deg": elevation_deg,
        "onset_s": lead,
        "tdoa_us": {k: (t_rel[i] - t_rel[j]) * 1e6 for k, (i, j) in zip(PAIR_KEYS, PAIRS)},
    }
    return out[: min(end, nfft)], truth


class SimAudioSource(AudioSource):
    """Four simulated INMP441s: white self-noise at `noise_dbfs`, 10 ms frames like the firmware,
    and `emit()` to inject a far-field source. `true_mics_mm` lets a test give the simulator a
    different geometry from the one the estimator believes (a calibration error)."""

    name = "sim"

    def __init__(
        self,
        mics_mm,
        array_yaw_deg: float = 0.0,
        rate: int = 48000,
        frame_n: int = 480,
        noise_dbfs: float = -70.0,
        c: float = 343.0,
        true_mics_mm=None,
        reverb: bool = True,
        seed: int | None = None,
        shift: int = 14,
        channel_delays_us=None,
    ):
        super().__init__()
        self.mics_mm = np.asarray(true_mics_mm if true_mics_mm is not None else mics_mm, float)
        self.array_yaw_deg = array_yaw_deg
        self.rate = rate
        self.frame_n = frame_n
        self.noise_rms = 32768.0 * 10 ** (noise_dbfs / 20)
        self.c = c
        self.reverb = reverb
        self.shift = shift
        self.channel_delays_us = channel_delays_us
        self.rng = np.random.default_rng(seed)
        self._idx = 0
        self._seq = 0
        self._pending: list[tuple[int, np.ndarray]] = []
        self._plock = threading.Lock()
        self._next_t: float | None = None
        self._t0_us = 1_000_000
        self.link_up = True

    def emit(self, bearing_deg: float, elevation_deg: float = 0.0, snr_db: float = 20.0, kind: str = "knock", reverb: bool | None = None, delay_ms: float = 20.0) -> dict:
        if kind not in ("knock", "voice"):
            raise ValueError("kind must be knock or voice")
        sig, truth = synth_event(
            self.mics_mm, bearing_deg, elevation_deg, snr_db, kind, self.rate, self.noise_rms,
            self.array_yaw_deg, self.c, self.reverb if reverb is None else reverb, self.rng,
            channel_delays_us=self.channel_delays_us,
        )
        with self._plock:
            # land at a random point inside a frame, so onsets are never frame-aligned
            at = self._idx + int(delay_ms * self.rate / 1000) + int(self.rng.integers(0, self.frame_n))
            self._pending.append((at, sig))
        truth["onset_sample"] = at + int(truth["onset_s"] * self.rate)
        truth["t_us"] = self._t0_us + int(truth["onset_sample"] * 1e6 / self.rate)
        return truth

    def next_block(self) -> AudioFrame:
        """The next frame, without pacing: tests pull these synchronously."""
        n = self.frame_n
        x = self.rng.normal(0.0, self.noise_rms, (n, 4))
        with self._plock:
            keep = []
            for at, sig in self._pending:
                lo, hi = max(at, self._idx), min(at + sig.shape[0], self._idx + n)
                if hi > lo:
                    x[lo - self._idx : hi - self._idx] += sig[lo - at : hi - at]
                if at + sig.shape[0] > self._idx + n:
                    keep.append((at, sig))
            self._pending = keep
        # INMP441 → 32-bit I2S word → >> shift → i16: the firmware saturates rather than wraps
        s = np.clip(np.round(x), -32768, 32767).astype(np.int16)
        f = AudioFrame(self._seq, self._t0_us + int(self._idx * 1e6 / self.rate), self.rate, self.shift, 0, s, time.time())
        self._idx += n
        self._seq += 1
        return f

    def _next_frame(self) -> AudioFrame | None:
        period = self.frame_n / self.rate
        if self._next_t is None:
            self._next_t = time.monotonic()
        delay = self._next_t - time.monotonic()
        if delay > 0 and self._stop.wait(delay):
            return None
        self._next_t += period
        if self._next_t < time.monotonic() - 0.2:
            self._next_t = time.monotonic()  # fell far behind (debugger, suspended laptop): don't burst
        return self.next_block()
