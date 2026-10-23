"""Audio framing (docs/api.md §5.2, §5.4) and the TCP client's reconnect / loss accounting."""

import socket
import threading
import time

import numpy as np
import pytest

from obslam.audio_source import TcpAudioSource
from obslam.audio_wire import (
    FLAG_I2S_TIMEOUT,
    HEADER_SIZE,
    KIND_AUDIO_PREVIEW,
    FrameReader,
    SeqTracker,
    decode_frame,
    decode_preview,
    encode_frame,
    encode_preview,
)


def frame(seq, n=480, flags=0, seed=0):
    rng = np.random.default_rng(seed + seq)
    s = rng.integers(-32768, 32768, (n, 4)).astype(np.int16)
    return s, encode_frame(seq, 1_000_000 + seq * 10_000, 48000, s, 14, flags)


def test_frame_round_trip_and_layout():
    s, buf = frame(7, flags=FLAG_I2S_TIMEOUT)
    assert HEADER_SIZE == 28 and len(buf) == 28 + 480 * 4 * 2
    assert buf[:4] == b"AUD1"
    f = decode_frame(buf)
    assert (f.seq, f.t_us, f.rate, f.channels, f.n, f.shift, f.flags) == (7, 1_070_000, 48000, 4, 480, 14, FLAG_I2S_TIMEOUT)
    np.testing.assert_array_equal(f.samples, s)
    # interleaved s0m1 s0m2 s0m3 s0m4 s1m1 …
    raw = np.frombuffer(buf, "<i2", 8, 28)
    assert list(raw) == list(s[0]) + list(s[1])
    with pytest.raises(ValueError):
        decode_frame(buf[:-2])


def test_reader_handles_arbitrary_chunks_and_garbage():
    frames = [frame(i) for i in range(6)]
    stream = b"".join(b for _, b in frames[:3]) + b"\x00garbage\xff" * 3 + b"".join(b for _, b in frames[3:])
    rng = np.random.default_rng(1)
    r = FrameReader()
    got = []
    i = 0
    while i < len(stream):
        k = int(rng.integers(1, 3000))
        got += r.feed(stream[i : i + k])
        i += k
    assert [f.seq for f in got] == list(range(6))
    for f, (s, _) in zip(got, frames):
        np.testing.assert_array_equal(f.samples, s)
    assert r.resyncs >= 1 and r.skipped_bytes == len(b"\x00garbage\xff" * 3)


def test_reader_rejects_absurd_sizes_instead_of_waiting_forever():
    _, good = frame(1)
    bad = bytearray(frame(0)[1])
    bad[22:24] = (60000).to_bytes(2, "little")  # n = 60000: a desync, not a frame
    r = FrameReader()
    got = r.feed(bytes(bad) + good)
    assert [f.seq for f in got] == [1]


def test_seq_tracker_gaps_wrap_and_restart():
    t = SeqTracker()
    assert [t.update(s) for s in (10, 11, 14, 15)] == [0, 0, 2, 0]
    assert t.lost == 2
    t2 = SeqTracker()
    assert [t2.update(s) for s in (0xFFFFFFFE, 0xFFFFFFFF, 0, 2)] == [0, 0, 0, 1]
    assert t2.lost == 1
    # the firmware restarted: counting back to 4 billion would be nonsense
    assert t2.update(0) == 0 and t2.restarts == 1 and t2.lost == 1


def test_preview_round_trip_channel_major():
    n = 3
    mins = np.array([[-100 - c, -32768, -1] for c in range(4)], np.int16)
    maxs = np.array([[100 + c, 32767, 0] for c in range(4)], np.int16)
    buf = encode_preview(9, 1761250000.25, [-40, -41.5, -39, -60], mins, maxs)
    assert buf[0] == KIND_AUDIO_PREVIEW and len(buf) == 16 + 4 * 4 + 4 * n * 4
    # channel-major: all of mic1's (min, max) pairs, then mic2's …
    body = np.frombuffer(buf, "<i2", 4 * n * 2, 32).reshape(4, n, 2)
    assert body[0, 1, 0] == -32768 and body[1, 0, 1] == 101
    d = decode_preview(buf)
    assert d["seq"] == 9 and d["t"] == 1761250000.25
    np.testing.assert_allclose(d["levels_dbfs"], [-40, -41.5, -39, -60])
    np.testing.assert_array_equal(d["min"], mins)
    np.testing.assert_array_equal(d["max"], maxs)


class FakeEsp(threading.Thread):
    """Streams frames like the ESP32 on 8901; `script` is a list of seq lists, one per connection.
    After each list it drops the client, like a reboot."""

    def __init__(self, script):
        super().__init__(daemon=True)
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.script = script
        self.accepted = 0

    def run(self):
        for seqs in self.script:
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            self.accepted += 1
            for s in seqs:
                conn.sendall(frame(s)[1])
            time.sleep(0.1)
            conn.close()
        self.srv.close()


def test_tcp_source_counts_gaps_and_reconnects():
    esp = FakeEsp([[0, 1, 2, 5, 6], [100, 101, 103]])
    esp.start()
    src = TcpAudioSource(f"tcp://127.0.0.1:{esp.port}")
    got = []
    src.start(got.append)
    deadline = time.time() + 5
    while len(got) < 8 and time.time() < deadline:
        time.sleep(0.02)
    src.stop()
    assert [f.seq for f in got] == [0, 1, 2, 5, 6, 100, 101, 103]
    # 3, 4 lost on the first link, 102 on the second; the jump across the reconnect is downtime
    assert src.lost_frames == 3 and got[3].lost_before == 2 and got[5].lost_before == 0
    assert src.connects == 2 and esp.accepted == 2
    assert all(f.t > 0 for f in got)


def test_tcp_source_never_blocks_on_a_dead_esp():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()  # nothing listens here
    src = TcpAudioSource(f"tcp://127.0.0.1:{port}", connect_timeout=0.5)
    t0 = time.monotonic()
    src.start(lambda f: None)
    assert time.monotonic() - t0 < 0.1
    time.sleep(0.4)
    assert src.link_up is False and src.error
    t0 = time.monotonic()
    src.stop()
    assert time.monotonic() - t0 < 1.5
