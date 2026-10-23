"""Acoustic direction finding for the four-microphone array (docs/api.md §5). Pure numpy.

What this array can and cannot do: the mics are 2–4 cm apart, so the largest time difference
between any two of them is ~120 µs. That is enough to tell the *direction* a plane wave came
from, and nothing about range (the wavefront curvature across 4 cm is unmeasurable for sources
further than a few decimetres). A position needs bearings from two scanner stations. Nothing in
here ever says "person": a spike is a sound, its bearing is a direction, and the candidates are
the mapped places along that direction where a sound source could be.

Pipeline per spike: Detector (energy over a running noise floor) → a window around the onset →
GCC-PHAT on all 6 mic pairs (sub-sample peak by zero-padded inverse FFT + parabola) → far-field
least squares for the in-plane slowness → bearing, |elevation|, 1σ, confidence → findings along
that bearing.

Sign conventions:
  tdoa_ij = t_i − t_j: positive when mic i hears the sound *later* than mic j.
  Array frame: x/y from `audio.mics_mm` (right-handed, z up), azimuth counter-clockwise from
  the array +x, as atan2 gives it.
  Base bearing: clockwise from base +x like every bearing in findings (analysis.bearing_deg).
  `array_yaw_deg` is the bearing of the array's +x axis, also clockwise from base +x.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import NamedTuple

import numpy as np

FULL_SCALE = 32767.0  # ±32767 = 0 dBFS (docs/api.md §5.4)
PAIRS = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
PAIR_KEYS = [f"{i + 1}{j + 1}" for i, j in PAIRS]


def dbfs(rms: float | np.ndarray) -> np.ndarray | float:
    """RMS in LSB → dBFS, within [−120, 0]: silence never becomes −inf in JSON, and a stream
    pinned at −32768 does not read as louder than full scale."""
    return np.clip(20.0 * np.log10(np.maximum(rms, 1e-9) / FULL_SCALE), -120.0, 0.0)


# ---- detection -------------------------------------------------------------------------------
@dataclass
class Detection:
    onset: int  # absolute sample index (start of the block that crossed the threshold)
    t_us: int  # ESP32 clock at the onset
    t: float  # unix seconds at the onset (receive clock, so it includes network latency)
    window: np.ndarray  # float32 (L, channels), starts `pre` samples before the onset
    pre: int
    rate: int
    snr_db: float
    peak_dbfs: float
    noise_power: np.ndarray  # per-channel floor, mean square in LSB²
    sustained: bool  # still loud at the end of the window: a voice or a scrape, not a knock


class Detector:
    """Spike detector on a 4-channel stream.

    Short blocks (2.5 ms) are compared against a running noise floor: the median of block
    energies over the last few seconds. The median, not the mean, so a knock or a shout does
    not drag the floor up with it; a noise that persists for more than half the window (a
    generator starting) does become the new floor, which is what an operator wants.

    The trigger uses the energy of the channel *sum*: at low frequencies the four mics are in
    phase, so the sum gains ~6 dB of SNR over any single mic for the cost of one addition.
    """

    def __init__(
        self,
        rate: int,
        channels: int = 4,
        trigger_db: float = 12.0,
        min_gap_ms: float = 300.0,
        pre_ms: float = 5.0,
        post_ms: float = 40.0,
        block_ms: float = 2.5,
        floor_s: float = 3.0,
        warmup_s: float = 0.5,
    ):
        self.rate = int(rate)
        self.channels = int(channels)
        self.trigger_db = float(trigger_db)
        self.min_gap = int(round(rate * min_gap_ms / 1000))
        self.pre = int(round(rate * pre_ms / 1000))
        self.post = int(round(rate * post_ms / 1000))
        self.block = max(1, int(round(rate * block_ms / 1000)))
        self.floor_blocks = max(8, int(round(floor_s * rate / self.block)))
        self.warmup_blocks = max(4, int(round(warmup_s * rate / self.block)))
        self._hist = np.zeros((self.floor_blocks, channels + 1))  # per-channel + sum energies
        self._hist_n = 0
        self._hist_i = 0
        self._floor: np.ndarray | None = None
        self._buf = np.zeros((0, channels), np.float32)
        self._buf_start = 0  # absolute index of _buf[0]
        self._idx = 0  # absolute index of the next sample
        self._acc = 0  # absolute index of the first sample not yet cut into blocks
        self._dead_until = -1
        self._armed = True
        self._pending: list[tuple[int, np.ndarray]] = []  # (onset, floor at trigger time)
        self._times: deque[tuple[int, int, int, float]] = deque(maxlen=64)  # (start, n, t_us, t)
        self.triggers = 0

    def configure(self, trigger_db: float, min_gap_ms: float) -> None:
        """Live retune from the config, keeping the learned floor."""
        self.trigger_db = float(trigger_db)
        self.min_gap = int(round(self.rate * min_gap_ms / 1000))

    @property
    def floor(self) -> np.ndarray | None:
        """Per-channel noise floor (mean square, LSB²). Provisional during warm-up (status can show
        it), None before the first block. Triggering waits for the full warm-up regardless."""
        if self._floor is not None:
            return self._floor[: self.channels]
        if self._hist_n:
            return np.median(self._hist[: self._hist_n], axis=0)[: self.channels]
        return None

    @property
    def warmed_up(self) -> bool:
        return self._floor is not None

    def reset_stream(self) -> None:
        """Samples were lost: what is buffered no longer joins up with what comes next. The
        floor survives (it is statistics, not samples); a half-collected spike is dropped."""
        self._buf = np.zeros((0, self.channels), np.float32)
        self._buf_start = self._acc = self._idx
        self._pending.clear()

    def feed(self, samples: np.ndarray, t_us: int = 0, t: float = 0.0) -> list[Detection]:
        x = np.asarray(samples, np.float32).reshape(-1, self.channels)
        n = x.shape[0]
        self._times.append((self._idx, n, int(t_us), float(t)))
        self._buf = np.concatenate([self._buf, x]) if self._buf.size else x.copy()
        self._idx += n

        # cut whole blocks out of the not-yet-analysed tail
        a0 = self._acc - self._buf_start
        nb = (self._buf.shape[0] - a0) // self.block
        if nb:
            blk = self._buf[a0 : a0 + nb * self.block].reshape(nb, self.block, self.channels)
            e = np.empty((nb, self.channels + 1))
            e[:, : self.channels] = np.mean(blk.astype(np.float64) ** 2, axis=1)
            e[:, self.channels] = np.mean(blk.sum(axis=2, dtype=np.float64) ** 2, axis=1)
            # the floor used for these blocks comes from *before* them, so a spike cannot raise
            # the threshold it is compared against
            if self._hist_n >= self.warmup_blocks:
                self._floor = np.median(self._hist[: self._hist_n], axis=0)
                thr = self._floor[self.channels] * 10 ** (self.trigger_db / 10)
                rearm = thr * 10 ** (-3.0 / 10)
                for j in range(nb):
                    start = self._acc + j * self.block
                    level = e[j, self.channels]
                    # one event per sound: after a trigger the detector re-arms only once the level
                    # has dropped back (3 dB hysteresis) *and* the dead time is over, so a 2 s shout
                    # is one event, not one every min_gap
                    if not self._armed:
                        if level < rearm and start >= self._dead_until:
                            self._armed = True
                        continue
                    if level > thr and start >= self._dead_until:
                        self._armed = False
                        self._dead_until = start + self.min_gap
                        self._pending.append((start, self._floor.copy()))
                        self.triggers += 1
            for row in e:
                self._hist[self._hist_i] = row
                self._hist_i = (self._hist_i + 1) % self.floor_blocks
                self._hist_n = min(self._hist_n + 1, self.floor_blocks)
            self._acc += nb * self.block

        out: list[Detection] = []
        still: list[tuple[int, np.ndarray]] = []
        for onset, fl in self._pending:
            if self._idx >= onset + self.post:
                out.append(self._extract(onset, fl))
            else:
                still.append((onset, fl))
        self._pending = still

        # keep enough history for the pre-trigger part of the next spike
        keep_from = min([self._acc, self._idx - self.pre - self.post - self.block] + [o - self.pre for o, _ in self._pending])
        drop = keep_from - self._buf_start
        if drop > 0:
            self._buf = self._buf[drop:]
            self._buf_start += drop
        return out

    def _time_at(self, idx: int) -> tuple[int, float]:
        for start, n, t_us, t in reversed(self._times):
            if start <= idx:
                dt = (idx - start) / self.rate
                return int(t_us + dt * 1e6), t + dt
        return 0, 0.0

    def _extract(self, onset: int, fl: np.ndarray) -> Detection:
        lo = max(onset - self.pre, self._buf_start)  # after a reset the pre-trigger may be short
        w = self._buf[lo - self._buf_start : onset + self.post - self._buf_start]
        pre = onset - lo
        noise = fl[: self.channels]
        noise_mean = float(np.mean(noise)) or 1e-12
        post = w[pre:]
        nb = post.shape[0] // self.block
        e = np.mean(post[: nb * self.block].reshape(nb, self.block, self.channels).astype(np.float64) ** 2, axis=(1, 2))
        # SNR = the loudest 2.5 ms block, mean over mics, against the floor: the same definition
        # the simulator uses to scale its sources, so "20 dB" means the same thing on both sides
        snr = 10 * math.log10(max(float(e.max()) / noise_mean - 1.0, 1e-2)) if nb else 0.0
        tail = e[-max(1, nb // 4) :].mean() if nb else 0.0
        sustained = bool(tail > noise_mean * 10 ** (self.trigger_db / 10))
        peak = float(np.abs(w).max()) if w.size else 0.0
        t_us, t = self._time_at(onset)
        return Detection(onset, t_us, t, w.copy(), pre, self.rate, round(snr, 1), round(float(dbfs(peak)), 1), noise.copy(), sustained)


# ---- GCC-PHAT ----------------------------------------------------------------------------------
class Gcc(NamedTuple):
    tau: float  # seconds, t_a − t_b
    peak: float  # PHAT coherence at the peak: 1 = every frequency agrees on this delay, ~0 = noise
    psr: float  # peak over the RMS of the correlation across all lags
    edge: bool  # the peak sits on the search limit: the true delay may lie outside it


def _nfft(n: int) -> int:
    return 1 << int(math.ceil(math.log2(max(2, 2 * n))))


def _band_mask(nfft: int, fs: float, band: tuple[float, float] | None) -> np.ndarray:
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    if band is None:
        m = np.ones_like(f)
        m[0] = 0.0
        return m
    return ((f >= band[0]) & (f <= band[1])).astype(np.float64)


def _gcc_from_spectra(A: np.ndarray, B: np.ndarray, mask: np.ndarray, nfft: int, fs: float, max_tau: float, interp: int) -> Gcc:
    R = A * np.conj(B)
    mag = np.abs(R)
    # PHAT: keep only the phase, so a loud low-frequency thump does not swamp the fine timing
    # information carried by the (weaker) high frequencies
    G = np.where(mask > 0, R / np.maximum(mag, 1e-12 * (mag.max() or 1.0)), 0.0)
    n2 = nfft * interp
    # irfft into n2 points zero-pads the spectrum = exact band-limited interpolation by `interp`;
    # the scale makes a perfectly coherent delay peak at exactly 1
    denom = 2.0 * mask.sum() - mask[0] - (mask[-1] if nfft % 2 == 0 else 0.0)
    cc = np.fft.irfft(G, n2) * (n2 / max(denom, 1.0))
    max_shift = max(1, min(int(math.ceil(max_tau * fs * interp)), n2 // 2 - 1))
    seg = np.concatenate([cc[-max_shift:], cc[: max_shift + 1]])
    k = int(np.argmax(seg))
    delta, peak = 0.0, float(seg[k])
    if 0 < k < seg.shape[0] - 1:
        y0, y1, y2 = float(seg[k - 1]), float(seg[k]), float(seg[k + 1])
        d = y0 - 2 * y1 + y2
        if d < 0:
            delta = 0.5 * (y0 - y2) / d
            peak = y1 - 0.25 * (y0 - y2) * delta
    edge = k == 0 or k == seg.shape[0] - 1
    coarse = cc[::interp]
    rms = float(np.sqrt(np.mean(coarse**2))) or 1e-12
    return Gcc((k - max_shift + delta) / (fs * interp), peak, peak / rms, edge)


def gcc_phat(a, b, fs: float, max_tau: float, interp: int = 16, band: tuple[float, float] | None = (150.0, 12000.0)) -> Gcc:
    """Time difference t_a − t_b between two channels, searched only within |tau| ≤ max_tau.

    48 kHz is 20.8 µs (7 mm of path) per sample; the interpolated peak resolves ~1 µs."""
    a = np.asarray(a, np.float64)
    b = np.asarray(b, np.float64)
    nfft = _nfft(max(a.shape[0], b.shape[0]))
    mask = _band_mask(nfft, fs, band)
    return _gcc_from_spectra(np.fft.rfft(a, nfft), np.fft.rfft(b, nfft), mask, nfft, fs, max_tau, interp)


def tukey(n: int, alpha: float = 0.2) -> np.ndarray:
    """Tapered window: the cut edges of a block otherwise look like a click to GCC."""
    if n <= 1 or alpha <= 0:
        return np.ones(n)
    w = np.ones(n)
    m = int(alpha * (n - 1) / 2)
    if m:
        r = 0.5 * (1 + np.cos(np.pi * (np.arange(m) / m - 1)))
        w[:m] = r
        w[-m:] = r[::-1]
    return w


def pair_tdoas(window: np.ndarray, fs: float, mics_m: np.ndarray, c: float, interp: int = 16, band=(150.0, 12000.0), margin: float = 0.15, offsets_s=None) -> list[Gcc]:
    """GCC-PHAT on all 6 pairs of a (L, 4) window. One FFT per channel, six cross-spectra.
    Each pair searches |tau| ≤ (1 + margin)·d/c + 20 µs: wide enough for a caliper error,
    narrow enough that a reflection's peak outside the physical range cannot win."""
    x = np.asarray(window, np.float64)
    x = (x - x.mean(axis=0)) * tukey(x.shape[0])[:, None]
    nfft = _nfft(x.shape[0])
    mask = _band_mask(nfft, fs, band)
    X = np.fft.rfft(x, nfft, axis=0)
    out = []
    for i, j in PAIRS:
        d = float(np.linalg.norm(mics_m[i] - mics_m[j]))
        skew = 0.0 if offsets_s is None else abs(offsets_s[i] - offsets_s[j])
        max_tau = (1.0 + margin) * d / c + 20e-6 + skew
        out.append(_gcc_from_spectra(X[:, i], X[:, j], mask, nfft, fs, max_tau, interp))
    return out


# ---- direction -------------------------------------------------------------------------------
@dataclass
class Doa:
    azimuth_deg: float | None  # array frame, counter-clockwise from array +x, toward the source
    elevation_deg: float | None  # |elevation|: a planar array cannot tell above from below
    sigma_deg: float | None  # 1σ of the azimuth
    k_norm: float | None  # |in-plane slowness| · c; 1 for a source in the array plane
    residual_us: float | None  # RMS misfit of the 6 TDOAs to the best plane wave
    confidence: float
    reasons: list[str] = field(default_factory=list)
    used_pairs: list[str] = field(default_factory=list)


def geometry_problem(mics_m: np.ndarray) -> str | None:
    """Why this mic layout cannot give a bearing, or None if it can."""
    p = np.asarray(mics_m, float)
    if p.shape != (4, 3) or not np.all(np.isfinite(p)):
        return "mic positions must be four [x, y, z] points"
    xy = p[:, :2] - p[:, :2].mean(axis=0)
    s = np.linalg.svd(xy, compute_uv=False)
    if s[0] < 1e-3:  # < 1 mm of aperture: the placeholder zeros, or never measured
        return "mic positions not set (audio.mics_mm)"
    if s[1] < 0.05 * s[0]:
        return "mic positions are collinear: a line array cannot tell its two sides apart"
    return None


def estimate_doa(
    tdoas_s,
    mics_m,
    c: float = 343.0,
    peaks=None,
    snr_db: float | None = None,
    calibrated: bool = True,
    position_sigma_mm: float = 0.5,
    multipath_us: float = 3.5,
) -> Doa:
    """Far-field least squares over the 6 pairs.

    A plane wave from unit direction u reaches mic i at t0 − p_i·u / c, so
    tdoa_ij = (p_j − p_i)·u / c. The array is planar, so only the in-plane part k of u is
    observable: solve A k = tdoa (A rows = (p_j − p_i)_xy / c) for k, azimuth = atan2(k_y, k_x),
    |elevation| = acos(min(1, |k|)). Six equations, two unknowns: the misfit says how well the
    pairs agree with one plane wave and the stated geometry, and propagated through (AᵀWA)⁻¹
    it gives the bearing σ.
    """
    p = np.asarray(mics_m, float)
    reasons: list[str] = []
    problem = geometry_problem(p)
    if problem:
        return Doa(None, None, None, None, None, 0.0, [problem])
    if np.ptp(p[:, 2]) > 1e-3:
        reasons.append("mics are not coplanar; only their x, y are used")
    tau = np.asarray(tdoas_s, float)
    A = np.array([(p[j, :2] - p[i, :2]) / c for i, j in PAIRS])
    w = np.ones(len(PAIRS)) if peaks is None else np.clip(np.asarray(peaks, float), 0.02, 1.0) ** 2
    keep = np.ones(len(PAIRS), bool)
    # timing floor: however clean the signal, a TDOA is no better than the caliper error on the
    # positions (0.5 mm per mic → ~2 µs per pair) plus the bias early reflections put on the GCC
    # peak. Reflections bias every pair coherently, so they do not show in the misfit; without
    # this floor σ came out ~0.8° while the reverberant simulator missed by up to 2.7°.
    floor_s = math.hypot(position_sigma_mm * math.sqrt(2) / 1000 / c, multipath_us * 1e-6)

    def solve(mask):
        Aw = A[mask] * w[mask, None]
        M = A[mask].T @ Aw
        k = np.linalg.solve(M, Aw.T @ tau[mask])
        r = tau[mask] - A[mask] @ k
        return k, r, M

    k, r, M = solve(keep)
    rms = float(np.sqrt(np.mean(r**2)))
    # one pair locked onto a reflection drags the whole solution; with six pairs for two
    # unknowns it shows up as a misfit that vanishes when that pair is left out
    if rms > 4 * floor_s:
        best = None
        for drop in range(len(PAIRS)):
            m = keep.copy()
            m[drop] = False
            k2, r2, M2 = solve(m)
            rms2 = float(np.sqrt(np.mean(r2**2)))
            if best is None or rms2 < best[0]:
                best = (rms2, drop, k2, r2, M2, m)
        if best and best[0] < 0.35 * rms:
            rms, drop, k, r, M, keep = best
            reasons.append(f"pair {PAIR_KEYS[drop]} disagreed with the others and was left out")

    m_used = int(keep.sum())
    wk = w[keep]
    s2 = max(float(np.sum(wk * r**2)) / max(m_used - 2, 1), floor_s**2 * float(wk.mean()))
    cov = s2 * np.linalg.inv(M)
    kn = float(np.hypot(k[0], k[1]))
    az = math.degrees(math.atan2(k[1], k[0]))
    if kn > 1e-6:
        g = np.array([-k[1], k[0]]) / kn**2
        sigma = math.degrees(math.sqrt(max(float(g @ cov @ g), 0.0)))
    else:
        sigma = 180.0
    sigma = min(sigma, 180.0)
    elev = math.degrees(math.acos(min(1.0, kn)))

    # confidence: each factor is 1 when that aspect is as good as this hardware can do
    mean_peak = float(np.mean(np.asarray(peaks, float)[keep])) if peaks is not None else 0.6
    # (calibrated on the simulator: coherence ≈ 0.45 at 20 dB, 0.18 at 10 dB, 0.13 at 6 dB)
    f_peak = float(np.clip((mean_peak - 0.05) / (0.40 - 0.05), 0.0, 1.0))
    f_fit = math.exp(-0.5 * (rms / (2.5 * floor_s)) ** 2)
    f_k = 1.0 if kn <= 1.15 else math.exp(-(((kn - 1.15) / 0.2) ** 2))
    f_snr = 1.0 if snr_db is None else 1.0 / (1.0 + math.exp(-(snr_db - 6.0) / 2.0))
    f_sigma = 1.0 / (1.0 + (sigma / 20.0) ** 2)
    conf = f_peak * f_fit * f_k * f_snr * f_sigma

    if f_peak < 0.5:
        reasons.append(f"weak correlation peaks (mean {mean_peak:.2f})")
    if f_fit < 0.5:
        reasons.append(f"pairs disagree: {rms * 1e6:.1f} µs misfit to a plane wave")
    if kn > 1.15:
        reasons.append(f"|k| = {kn:.2f} > 1: geometry, mic order or speed of sound is off")
    if snr_db is not None and f_snr < 0.5:
        reasons.append(f"low SNR ({snr_db:.0f} dB)")
    if kn < 0.35:
        reasons.append("source is nearly overhead or below: azimuth poorly defined")
    if not calibrated:
        conf = min(conf, 0.5)
        reasons.append("array not calibrated (channel order, yaw to the LiDAR 0° mark and capture offsets unverified): bearing is indicative only")
    return Doa(az, elev, sigma, kn, rms * 1e6, round(conf, 3), reasons, [PAIR_KEYS[i] for i in np.nonzero(keep)[0]])


def array_to_bearing(azimuth_deg: float, array_yaw_deg: float) -> float:
    """Array azimuth (CCW from array +x) → base bearing (clockwise from base +x). The array +x
    is at bearing yaw; turning CCW by az within the array frame is turning clockwise by −az."""
    return (array_yaw_deg - azimuth_deg) % 360.0


def bearing_to_array(bearing_deg: float, array_yaw_deg: float) -> float:
    return (array_yaw_deg - bearing_deg) % 360.0


def locate(
    window: np.ndarray,
    fs: float,
    mics_mm,
    c: float = 343.0,
    array_yaw_deg: float = 0.0,
    calibrated: bool = True,
    snr_db: float | None = None,
    interp: int = 16,
    band=(150.0, 12000.0),
    offsets_us=None,
) -> dict:
    """Window → bearing fields of an AudioEvent (docs/api.md §5.3).

    `tdoa_us` in the result is what the hardware measured (raw GCC, before the per-channel
    capture offsets are removed), so stored events can be re-used by `calibrate`."""
    mics_m = np.asarray(mics_mm, float) / 1000.0
    problem = geometry_problem(mics_m)
    if problem or window.shape[1] != 4:
        return {
            "bearing_deg": None,
            "bearing_sigma_deg": None,
            "elevation_deg": None,
            "confidence": 0.0,
            "tdoa_us": {},
            "reasons": [problem or f"{window.shape[1]} channels, DOA needs 4"],
            "reason": problem or f"{window.shape[1]} channels, DOA needs 4",
        }
    off = np.zeros(4) if offsets_us is None else np.asarray(offsets_us, float) * 1e-6
    g = pair_tdoas(window, fs, mics_m, c, interp, band, offsets_s=off)
    taus = [x.tau for x in g]
    # a channel captured δ late shifts every tdoa it is in: t_i − t_j measured = true + δ_i − δ_j
    corrected = [t - (off[i] - off[j]) for t, (i, j) in zip(taus, PAIRS)]
    doa = estimate_doa(corrected, mics_m, c, peaks=[x.peak for x in g], snr_db=snr_db, calibrated=calibrated)
    reasons = list(doa.reasons)
    edges = [PAIR_KEYS[i] for i, x in enumerate(g) if x.edge]
    if edges:
        reasons.append(f"pair {', '.join(edges)} peaked at the search limit: check mic spacing")
    bearing = None if doa.azimuth_deg is None else round(array_to_bearing(doa.azimuth_deg, array_yaw_deg), 1)
    resolved = bearing is not None
    return {
        "bearing_deg": bearing,
        "bearing_sigma_deg": round(doa.sigma_deg, 1) if resolved else None,
        "elevation_deg": round(doa.elevation_deg, 1) if resolved else None,
        "confidence": doa.confidence,
        "tdoa_us": {k: round(t * 1e6, 1) for k, t in zip(PAIR_KEYS, taus)},
        "coherence": [round(x.peak, 3) for x in g],
        "residual_us": None if doa.residual_us is None else round(doa.residual_us, 2),
        "reasons": reasons,
        "reason": "; ".join(reasons) if reasons else None,
    }


# ---- click calibration ---------------------------------------------------------------------------
def _pair_matrix() -> np.ndarray:
    D = np.zeros((len(PAIRS), 4))
    for r, (i, j) in enumerate(PAIRS):
        D[r, i], D[r, j] = 1.0, -1.0
    return D


def _tdoa_vector(t) -> np.ndarray:
    if isinstance(t, dict):
        return np.array([float(t[k]) for k in PAIR_KEYS])
    return np.asarray(t, float).reshape(len(PAIRS))


def _fit_yaw_offsets(P: np.ndarray, bearings: np.ndarray, elev: np.ndarray, tau: np.ndarray, c: float):
    """Best array yaw and per-channel offsets (mic1 = 0) for tdoas `tau` (n, 6) in seconds."""
    n = tau.shape[0]
    D = np.tile(_pair_matrix()[:, 1:], (n, 1))  # δ1 ≡ 0: only differences are observable
    Dp = np.linalg.pinv(D)
    B = np.array([(P[j] - P[i]) / c for i, j in PAIRS])  # (6, 2)
    ce = np.cos(np.radians(elev))
    y = tau.reshape(-1)

    def rss(yaw):
        a = np.radians(yaw - bearings)  # bearing_to_array
        u = np.stack([ce * np.cos(a), ce * np.sin(a)], axis=1)  # (n, 2) in the array frame
        e = y - (u @ B.T).reshape(-1)
        d = Dp @ e
        r = e - D @ d
        return float(r @ r), d, r

    grid = np.arange(0.0, 360.0, 0.5)
    vals = [rss(v)[0] for v in grid]
    best = float(grid[int(np.argmin(vals))])
    lo, hi = best - 0.5, best + 0.5
    for _ in range(40):  # golden section on the bracket around the grid minimum
        m1, m2 = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        if rss(m1)[0] < rss(m2)[0]:
            hi = m2
        else:
            lo = m1
    yaw = 0.5 * (lo + hi)
    v, d, r = rss(yaw)
    return yaw % 360.0, np.r_[0.0, d], r.reshape(n, len(PAIRS))


def _congruent(P: np.ndarray, Q: np.ndarray, tol_m: float = 1e-3) -> bool:
    """Same per-channel layout up to a rotation and translation (which the yaw fit absorbs)?"""
    a = P - P.mean(axis=0)
    b = Q - Q.mean(axis=0)
    u, _, vt = np.linalg.svd(b.T @ a)
    R = u @ vt
    if np.linalg.det(R) < 0:  # proper rotations only: a reflection is a different layout
        u[:, -1] *= -1
        R = u @ vt
    return float(np.max(np.linalg.norm(a @ R.T - b, axis=1))) < tol_m


def _refine_positions(P: np.ndarray, bearings, elev, tau: np.ndarray, c: float, yaw: float, prior_mm: float = 5.0, tau_sigma_s: float = 3e-6):
    """With the yaw fixed the model is linear in the positions *and* the offsets:
    tdoa = (p_j − p_i)·u_n / c + δ_i − δ_j. Mic 1 stays put (a translation is unobservable,
    and so is a rotation, which the yaw already holds); the other three move under a weak
    prior towards the caliper values so a few noisy claps cannot fold the array."""
    n = tau.shape[0]
    a = np.radians(yaw - np.asarray(bearings))
    ce = np.cos(np.radians(np.asarray(elev)))
    U = np.stack([ce * np.cos(a), ce * np.sin(a)], axis=1)  # (n, 2)
    # unknowns: x2 y2 x3 y3 x4 y4 (corrections, m), δ2 δ3 δ4 (s)
    rows, rhs = [], []
    for k in range(n):
        for r, (i, j) in enumerate(PAIRS):
            row = np.zeros(9)
            pred = (P[j] - P[i]) @ U[k] / c
            for m, sign in ((j, 1.0), (i, -1.0)):
                if m > 0:
                    row[2 * (m - 1) : 2 * (m - 1) + 2] += sign * U[k] / c
            if i > 0:
                row[6 + i - 1] += 1.0
            if j > 0:
                row[6 + j - 1] -= 1.0
            rows.append(row / tau_sigma_s)
            rhs.append((tau[k, r] - pred) / tau_sigma_s)
    for q in range(6):
        row = np.zeros(9)
        row[q] = 1.0 / (prior_mm / 1000.0)
        rows.append(row)
        rhs.append(0.0)
    sol, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    P2 = P.copy()
    P2[1:] += sol[:6].reshape(3, 2)
    off = np.r_[0.0, sol[6:]]
    B = np.array([(P2[j] - P2[i]) / c for i, j in PAIRS])
    r = tau - (U @ B.T) - (_pair_matrix() @ off)[None, :]
    return P2, off, r


def calibrate(clicks: list[dict], mics_mm, c: float = 343.0, rate: int = 48000, check_order: bool = True, refine_positions: bool = False) -> dict:
    """Fit the array yaw and fixed per-channel capture offsets from claps at known bearings.

    Each click: {"bearing_deg": base bearing of the source, "tdoa_us": raw tdoas (dict "12".."34"
    or a list of 6), "elevation_deg": optional, default 0}. Clap at the array's height (or give
    the elevation): the in-plane delays shrink with cos(elevation), and an unaccounted 30° looks
    like a 13 % smaller array. Needs ≥ 4 claps spread around the array (≥ 6 to refine positions).

    Model per clap n and pair ij: tdoa = (p_j − p_i)·u_n / c + δ_i − δ_j with u_n the direction
    at array azimuth yaw − bearing_n. For a fixed yaw the offsets are linear, so yaw is a 1-D
    search (0.5° grid, then golden section) with δ solved in closed form.

    The fit is repeated for every assignment of the four positions to the four channels (and
    for the board mirrored, i.e. measured from the wrong side), because a swapped I2S slot fits
    *something*, just worse than the right order does. Assignments that are the same layout up
    to a rotation (possible on a symmetric board) predict identical delays and are skipped."""
    if len(clicks) < 4:
        raise ValueError("calibration needs at least 4 claps from known bearings")
    P = np.asarray(mics_mm, float)[:, :2] / 1000.0
    problem = geometry_problem(np.asarray(mics_mm, float) / 1000.0)
    if problem:
        raise ValueError(problem)
    bearings = np.array([float(k["bearing_deg"]) for k in clicks])
    elev = np.array([float(k.get("elevation_deg") or 0.0) for k in clicks])
    tau = np.stack([_tdoa_vector(k["tdoa_us"]) for k in clicks]) * 1e-6
    spread = max(abs(angle_diff(a, b)) for a in bearings for b in bearings)
    if spread < 60:
        raise ValueError("claps must come from bearings at least 60° apart (ideally all around)")

    yaw, off, r = _fit_yaw_offsets(P, bearings, elev, tau, c)
    rms = float(np.sqrt(np.mean(r**2)))
    out: dict = {"n_clicks": len(clicks)}
    P_used = P
    if refine_positions:
        if len(clicks) < 6:
            raise ValueError("refining positions needs at least 6 claps")
        P_used, off, r = _refine_positions(P, bearings, elev, tau, c, yaw)
        rms_refined = float(np.sqrt(np.mean(r**2)))
        out["refined"] = {
            "mics_mm": [[round(float(x) * 1000, 2), round(float(y) * 1000, 2), 0.0] for x, y in P_used],
            "moved_mm": [round(float(v) * 1000, 2) for v in np.linalg.norm(P_used - P, axis=1)],
            "residual_before_us": round(rms * 1e6, 2),
        }
        rms = rms_refined
    per_click = []
    for n, k in enumerate(clicks):
        corr = tau[n] - _pair_matrix() @ off
        d = estimate_doa(corr, np.c_[P_used, np.zeros(4)], c)
        b = None if d.azimuth_deg is None else array_to_bearing(d.azimuth_deg, yaw)
        per_click.append(
            {
                "bearing_deg": round(float(bearings[n]), 1),
                "fitted_bearing_deg": None if b is None else round(b, 1),
                "error_deg": None if b is None else round(angle_diff(b, float(bearings[n])), 1),
                "residual_us": round(float(np.sqrt(np.mean(r[n] ** 2))) * 1e6, 2),
            }
        )
    # offsets relative to mic 1, not mean-removed: the suspected failure is "the I2S1 pair
    # (ch3, ch4) is one sample late", and relative to ch1 that reads directly as +20.8 µs
    out.update(
        {
            "array_yaw_deg": round(yaw, 2),
            "channel_offsets_us": [round(float(v) * 1e6, 2) for v in off],
            "channel_offsets_samples": [round(float(v) * rate, 2) for v in off],
            "residual_us": round(rms * 1e6, 2),
            "per_click": per_click,
        }
    )
    if check_order:
        import itertools

        base_rms = float(np.sqrt(np.mean(_fit_yaw_offsets(P, bearings, elev, tau, c)[2] ** 2)))
        rivals = []
        seen = [P]
        # unmirrored first: on a symmetric board "ch2 and ch3 swapped" and "measured from the
        # wrong side" are the same layout, and the swap is the one an operator can act on
        for mirrored in (False, True):
            Pm = P * np.array([1.0, -1.0]) if mirrored else P
            for perm in itertools.permutations(range(4)):
                Q = Pm[list(perm)]
                if any(_congruent(S, Q) for S in seen):
                    continue
                seen.append(Q)
                _, _, rp = _fit_yaw_offsets(Q, bearings, elev, tau, c)
                rivals.append((float(np.sqrt(np.mean(rp**2))), perm, mirrored))
        rivals.sort(key=lambda x: x[0])
        best = rivals[0]
        identity_best = base_rms <= best[0]
        # a rival within 1.5× is no confirmation (claps from too few bearings)
        confirmed = identity_best and best[0] > 1.5 * base_rms
        if identity_best:
            verdict = "channel order confirmed" if confirmed else "channel order plausible but not decisive: add claps from more bearings"
        else:
            names = ", ".join(f"ch{ch + 1} = position of mic {m + 1}" for ch, m in enumerate(best[1]))
            verdict = f"channel order looks wrong: best fit with {names}" + (" on a mirrored board" if best[2] else "")
        out["order"] = {
            "identity_residual_us": round(base_rms * 1e6, 2),
            "best_rival_residual_us": round(best[0] * 1e6, 2),
            "best_rival_channel_to_mic": [m + 1 for m in best[1]],
            "best_rival_mirrored": best[2],
            "confirmed": bool(confirmed),
            "verdict": verdict,
        }
    out["ok"] = bool(rms < 6e-6 and out.get("order", {}).get("confirmed", True))
    return out


# ---- map candidates ---------------------------------------------------------------------------
# docs/api.md §5.3: void, shadow, unknown, obstacle; depressions are not candidates. Findings
# carry no list of unknown regions today (only a cell count), so "unknown" is read if present.
_KIND_ORDER = [("voids", "void"), ("shadows", "shadow"), ("unknowns", "unknown"), ("obstacles", "obstacle")]


def angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a − b in degrees, in (−180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def candidates(bearing_deg: float | None, sigma_deg: float | None, findings: dict | None, n_sigma: float = 2.0, min_half_deg: float = 3.0) -> list[dict]:
    """Mapped regions along a bearing where a sound source could be: voids first (covered space
    a source could be in), then shadows (never seen), unknown regions, then obstacles (their far
    sides). Within each kind nearest first. A region counts when its centroid lies within
    bearing ± n·σ (at least ± min_half_deg); a region wide enough to straddle the bearing counts
    too, since its centroid can sit off-axis while the region itself is right on it."""
    if bearing_deg is None or not findings:
        return []
    half = min(180.0, max(n_sigma * (sigma_deg or 0.0), min_half_deg))
    out = []
    for rank, (key, kind) in enumerate(_KIND_ORDER):
        for f in findings.get(key) or []:
            b = f.get("bearing_deg")
            if b is None:
                continue
            delta = angle_diff(float(b), bearing_deg)
            extent = 0.0
            bbox = f.get("bbox_m")
            dist = float(f.get("distance_m") or 0.0)
            if bbox and dist > 0:
                # angular half-size of the region as seen from the scanner
                extent = math.degrees(math.atan2(0.5 * math.hypot(bbox[2] - bbox[0], bbox[3] - bbox[1]), dist))
            if abs(delta) <= half or abs(delta) <= extent:
                out.append(
                    {
                        "kind": kind,
                        "id": f.get("id"),
                        "distance_m": f.get("distance_m"),
                        "bearing_deg": b,
                        "off_axis_deg": round(delta, 1),
                        "_rank": rank,
                    }
                )
    out.sort(key=lambda c: (c["_rank"], c["distance_m"] if c["distance_m"] is not None else 1e9))
    for c in out:
        c.pop("_rank")
    return out
