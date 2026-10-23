"""Audio framing: the ESP32's TCP 8901 stream (docs/api.md §5.2) and the kind-5 preview (§5.4)."""

from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

MAGIC = 0x31445541  # 'AUD1' little-endian
KIND_AUDIO_PREVIEW = 5

FLAG_I2S_TIMEOUT = 0x1
FLAG_DROPPED = 0x2

_HDR = struct.Struct("<IIQIHHHH")  # magic, seq, t_us, rate, channels, n, shift, flags → 28 bytes
HEADER_SIZE = _HDR.size
_MAGIC_BYTES = struct.pack("<I", MAGIC)
# A frame bigger than this is a desync, not audio: 4 ch × 4800 samples is 100 ms, ten times the
# firmware's 10 ms frames. Without a cap one corrupt `n` would make the reader wait for megabytes.
MAX_SAMPLES = 4800
MAX_CHANNELS = 8

_PREVIEW_HDR = struct.Struct("<BBHId")  # kind, channels, n, seq, t_unix → 16 bytes


@dataclass
class AudioFrame:
    seq: int
    t_us: int  # ESP32 esp_timer at the first sample
    rate: int
    shift: int
    flags: int
    samples: np.ndarray  # int16 (n, channels), mic order 1..4
    t: float = 0.0  # unix seconds when the core received it
    lost_before: int = 0  # frames missing between the previous frame and this one (seq gap)

    @property
    def n(self) -> int:
        return int(self.samples.shape[0])

    @property
    def channels(self) -> int:
        return int(self.samples.shape[1])


def encode_frame(seq: int, t_us: int, rate: int, samples, shift: int = 14, flags: int = 0) -> bytes:
    s = np.ascontiguousarray(samples, dtype="<i2")
    if s.ndim != 2:
        raise ValueError("samples must be (n, channels)")
    n, ch = s.shape
    return _HDR.pack(MAGIC, seq & 0xFFFFFFFF, t_us, rate, ch, n, shift, flags) + s.tobytes()


def decode_frame(buf: bytes | memoryview) -> AudioFrame:
    magic, seq, t_us, rate, ch, n, shift, flags = _HDR.unpack_from(buf)
    if magic != MAGIC:
        raise ValueError(f"bad audio magic {magic:#x}")
    need = HEADER_SIZE + n * ch * 2
    if len(buf) < need:
        raise ValueError(f"short audio frame: {len(buf)} < {need}")
    s = np.frombuffer(buf, "<i2", n * ch, HEADER_SIZE).reshape(n, ch).copy()
    return AudioFrame(seq, t_us, rate, shift, flags, s)


class FrameReader:
    """Incremental parser for a TCP byte stream. Bytes arrive in arbitrary chunks; a corrupt header
    (bad magic or absurd sizes) skips forward to the next magic instead of killing the link."""

    def __init__(self):
        self._buf = bytearray()
        self.resyncs = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list[AudioFrame]:
        self._buf += data
        out: list[AudioFrame] = []
        while True:
            if len(self._buf) < HEADER_SIZE:
                break
            magic, _, _, _, ch, n, _, _ = _HDR.unpack_from(self._buf)
            if magic != MAGIC or not (1 <= ch <= MAX_CHANNELS) or not (1 <= n <= MAX_SAMPLES):
                self._resync()
                continue
            size = HEADER_SIZE + n * ch * 2
            if len(self._buf) < size:
                break
            out.append(decode_frame(memoryview(self._buf)[:size]))
            del self._buf[:size]
        return out

    def _resync(self) -> None:
        self.resyncs += 1
        k = self._buf.find(_MAGIC_BYTES, 1)
        if k < 0:
            # keep the last 3 bytes: they may be the start of a magic split across chunks
            drop = max(0, len(self._buf) - 3)
        else:
            drop = k
        self.skipped_bytes += drop
        del self._buf[:drop]


@dataclass
class SeqTracker:
    """Counts frames lost from gaps in `seq`. A backwards jump means the firmware restarted (or a new
    connection restarted its counter): that resets the baseline instead of counting 4 billion losses."""

    last: int | None = None
    lost: int = 0
    restarts: int = 0
    max_gap: int = 100_000  # bigger forward jumps are also treated as a restart

    def update(self, seq: int) -> int:
        if self.last is None:
            self.last = seq
            return 0
        gap = (seq - self.last - 1) & 0xFFFFFFFF
        self.last = seq
        if gap == 0:
            return 0
        if gap > self.max_gap:
            self.restarts += 1
            return 0
        self.lost += gap
        return gap

    def reset(self) -> None:
        self.last = None


def encode_preview(seq: int, t_unix: float, levels_dbfs, mins, maxs) -> bytes:
    """kind 5: per channel min/max envelopes. mins/maxs are (channels, n) int16."""
    mn = np.asarray(mins, dtype="<i2")
    mx = np.asarray(maxs, dtype="<i2")
    ch, n = mn.shape
    env = np.empty((ch, n, 2), "<i2")
    env[:, :, 0] = mn
    env[:, :, 1] = mx
    lv = np.asarray(levels_dbfs, dtype="<f4").reshape(ch)
    return _PREVIEW_HDR.pack(KIND_AUDIO_PREVIEW, ch, n, seq & 0xFFFFFFFF, t_unix) + lv.tobytes() + env.tobytes()


def decode_preview(buf: bytes) -> dict:
    kind, ch, n, seq, t = _PREVIEW_HDR.unpack_from(buf)
    assert kind == KIND_AUDIO_PREVIEW
    o = _PREVIEW_HDR.size
    lv = np.frombuffer(buf, "<f4", ch, o)
    env = np.frombuffer(buf, "<i2", ch * n * 2, o + 4 * ch).reshape(ch, n, 2)
    return {"seq": seq, "t": t, "levels_dbfs": lv, "min": env[:, :, 0], "max": env[:, :, 1]}


# ---- UDP 8902: 'AUD2' datagrams (docs/api.md §5.2) ---------------------------------------------------
MAGIC2 = 0x32445541  # 'AUD2'
_HDR2 = struct.Struct("<IIQIHHHHBBHH")  # magic seq t_us rate channels n shift flags part parts first count → 34 B
HEADER2_SIZE = _HDR2.size
MAX_PART_SAMPLES = 175  # 1,400 B of samples per datagram at 4 channels


def encode_aud2(seq: int, t_us: int, rate: int, samples, shift: int = 14, flags: int = 0, max_count: int = MAX_PART_SAMPLES) -> list[bytes]:
    """One frame as datagrams, split evenly like the firmware (480 → 3 × 160, 240 → 2 × 120)."""
    s = np.ascontiguousarray(samples, dtype="<i2")
    n, ch = s.shape
    parts = max(1, -(-n // max_count))
    bounds = [round(k * n / parts) for k in range(parts + 1)]
    out = []
    for k in range(parts):
        a, b = bounds[k], bounds[k + 1]
        hdr = _HDR2.pack(MAGIC2, seq & 0xFFFFFFFF, t_us, rate, ch, n, shift, flags, k, parts, a, b - a)
        out.append(hdr + s[a:b].tobytes())
    return out


@dataclass
class _Partial:
    t_us: int
    rate: int
    shift: int
    n: int
    channels: int
    parts: int
    flags: int
    samples: np.ndarray
    got: set
    arrived: float


class Aud2Reassembler:
    """Datagrams → whole frames, in seq order.

    Parts are placed at their `first` sample; a frame is complete when all `parts` arrived.
    Frames are released in seq order; a frame still incomplete once the stream is `reorder`
    frames past it (or `max_wait_s` after a later frame completed) is lost, and the next
    released frame carries the gap in `lost_before`. Garbage, duplicates and parts that
    disagree with their frame are counted and ignored, never raised."""

    def __init__(self, reorder: int = 5, max_wait_s: float = 0.05):
        self.reorder = reorder
        self.max_wait_s = max_wait_s
        self.tracker = SeqTracker()
        self.reset()
        self.garbage = 0
        self.late = 0
        self.duplicates = 0
        self.partial_lost = 0  # frames dropped because a part never came

    def reset(self) -> None:
        self._next: int | None = None
        self._newest: int | None = None
        self._partial: dict[int, _Partial] = {}
        self._done: dict[int, tuple[AudioFrame, float]] = {}
        self.tracker.reset()

    @staticmethod
    def _diff(a: int, b: int) -> int:
        """a − b on the 32-bit wrapping counter, as a signed number."""
        d = (a - b) & 0xFFFFFFFF
        return d - (1 << 32) if d >= 1 << 31 else d

    def feed(self, data: bytes, now: float = 0.0) -> list[AudioFrame]:
        if len(data) < HEADER2_SIZE:
            self.garbage += 1
            return self.poll(now)
        magic, seq, t_us, rate, ch, n, shift, flags, part, parts, first, count = _HDR2.unpack_from(data)
        ok = (
            magic == MAGIC2 and 1 <= ch <= MAX_CHANNELS and 1 <= n <= MAX_SAMPLES and 1 <= parts <= 64
            and part < parts and count >= 1 and first + count <= n and len(data) == HEADER2_SIZE + count * ch * 2
        )
        if not ok:
            self.garbage += 1
            return self.poll(now)
        if self._next is not None:
            d = self._diff(seq, self._next)
            if d < -1000 or d > 100_000:  # the firmware restarted (or a stale stream): start over
                self.reset()
            elif d < 0:
                self.late += 1
                return self.poll(now)
        if self._next is None:
            self._next = seq
        if self._newest is None or self._diff(seq, self._newest) > 0:
            self._newest = seq
        if seq in self._done:
            self.duplicates += 1
            return self.poll(now)
        p = self._partial.get(seq)
        if p is None:
            p = _Partial(t_us, rate, shift, n, ch, parts, 0, np.zeros((n, ch), np.int16), set(), now)
            self._partial[seq] = p
        elif (p.n, p.channels, p.parts, p.rate) != (n, ch, parts, rate):
            self.garbage += 1
            return self.poll(now)
        if part in p.got:
            self.duplicates += 1
            return self.poll(now)
        p.got.add(part)
        p.flags |= flags
        p.samples[first : first + count] = np.frombuffer(data, "<i2", count * ch, HEADER2_SIZE).reshape(count, ch)
        if len(p.got) == p.parts:
            del self._partial[seq]
            self._done[seq] = (AudioFrame(seq, p.t_us, p.rate, p.shift, p.flags, p.samples, now), now)
        return self.poll(now)

    def poll(self, now: float = 0.0, flush: bool = False) -> list[AudioFrame]:
        """Release what can be released. `flush` gives up on every gap (the stream stopped)."""
        out: list[AudioFrame] = []
        while self._next is not None:
            if self._next in self._done:
                f, _ = self._done.pop(self._next)
                f.lost_before = self.tracker.update(f.seq)
                out.append(f)
                self._next = (self._next + 1) & 0xFFFFFFFF
                continue
            if not self._done and not self._partial:
                break
            behind = self._newest is not None and self._diff(self._newest, self._next) >= self.reorder
            waited = bool(self._done) and now - min(t for _, t in self._done.values()) > self.max_wait_s
            if not (flush or behind or waited):
                break
            # give up on this seq: it never completed in time
            if self._partial.pop(self._next, None) is not None:
                self.partial_lost += 1
            if not self._done and not self._partial:
                self._next = (self._next + 1) & 0xFFFFFFFF
                break
            self._next = (self._next + 1) & 0xFFFFFFFF
        # partials that fell behind the release point are lost for good
        for s in [s for s in self._partial if self._next is not None and self._diff(s, self._next) < 0]:
            del self._partial[s]
            self.partial_lost += 1
        return out
