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


# ---- UDP 'AUD2' -------------------------------------------------------------------------------------
from obslam.audio_source import UdpAudioSource  # noqa: E402
from obslam.audio_wire import HEADER2_SIZE, Aud2Reassembler, encode_aud2  # noqa: E402


def frame2(seq, rate=48000, flags=0):
    n = rate // 100
    s = np.random.default_rng(seq).integers(-32768, 32768, (n, 4)).astype(np.int16)
    return s, encode_aud2(seq, 1_000_000 + seq * 10_000, rate, s, 14, flags)


def test_aud2_split_matches_the_firmware_table():
    assert HEADER2_SIZE == 34
    for rate, parts, size in ((16000, 1, 1314), (24000, 2, 994), (48000, 3, 1314)):
        _, dg = frame2(0, rate)
        assert len(dg) == parts and all(len(d) == size for d in dg)
        assert all(d[:4] == b"AUD2" for d in dg)


def test_reassembler_in_order_reordered_and_lost_part():
    r = Aud2Reassembler(reorder=5)
    truth = {}
    dgs = []
    for seq in range(12):
        s, dg = frame2(seq)
        truth[seq] = s
        if seq == 4:
            dg = dg[:1] + dg[2:]  # part 1 of frame 4 never arrives
        if seq == 7:
            dg = dg[::-1]  # parts out of order
        dgs.append(dg)
    dgs[8], dgs[9] = dgs[9], dgs[8]  # frames 8 and 9 swapped on the air
    out = []
    for dg in dgs:
        for d in dg:
            out += r.feed(d, 0.0)
    out += r.poll(1.0, flush=True)
    assert [f.seq for f in out] == [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11]
    for f in out:
        np.testing.assert_array_equal(f.samples, truth[f.seq])
        assert f.t_us == 1_000_000 + f.seq * 10_000 and f.rate == 48000 and f.n == 480
    assert [f.lost_before for f in out] == [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    assert r.partial_lost == 1


def test_reassembler_ignores_garbage_duplicates_and_late_frames():
    r = Aud2Reassembler(reorder=3)
    _, d0 = frame2(0)
    out = []
    for d in d0[:1] + d0:  # one part twice, before the frame completes
        out += r.feed(d, 0.0)
    out += r.feed(b"AUD2 but not really", 0.0)
    bad = bytearray(d0[0])
    bad[-1:] = b""  # length disagrees with count
    out += r.feed(bytes(bad), 0.0)
    out += r.feed(b"AUDS 1234", 0.0)
    for seq in range(1, 6):
        for d in frame2(seq)[1]:
            out += r.feed(d, 0.0)
    out += r.feed(frame2(0)[1][0], 0.0)  # frame 0 again, long after it was released
    assert [f.seq for f in out] == [0, 1, 2, 3, 4, 5]
    assert r.duplicates == 1 and r.garbage == 3 and r.late == 1


def test_reassembler_releases_after_a_short_wait_and_flags_carry():
    r = Aud2Reassembler(reorder=5, max_wait_s=0.05)
    _, d0 = frame2(0)
    _, d1 = frame2(1, flags=0x2)
    out = r.feed(d0[0], 0.0)  # frame 0 incomplete…
    for d in d1:
        out += r.feed(d, 0.0)  # …frame 1 complete but waits behind it
    assert out == []
    out = r.poll(0.1)  # the stream went quiet: give up on 0 after 50 ms
    assert [f.seq for f in out] == [1] and out[0].flags == 0x2 and out[0].lost_before == 0
    # the first frame we ever see sets the baseline, so the skipped 0 is not a counted gap; a
    # later gap is
    for d in frame2(3)[1]:
        out = r.feed(d, 0.2)
    out += r.poll(0.3)
    assert [f.seq for f in out] == [3] and out[0].lost_before == 1


def test_reassembler_restart_resets():
    r = Aud2Reassembler()
    for seq in range(5000, 5003):
        for d in frame2(seq)[1]:
            r.feed(d, 0.0)
    out = []
    for seq in range(0, 3):  # the ESP rebooted
        for d in frame2(seq)[1]:
            out += r.feed(d, 0.0)
    assert [f.seq for f in out] == [0, 1, 2] and all(f.lost_before == 0 for f in out)


class FakeEspUdp(threading.Thread):
    """The scanner's UDP 8902: remembers the newest `AUDS <port>` subscriber, forgets it 5 s after
    the last AUDS or on `AUDX`, and streams from a separate ephemeral socket like the firmware."""

    def __init__(self):
        super().__init__(daemon=True)
        self.ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctl.bind(("127.0.0.1", 0))
        self.ctl.settimeout(0.05)
        self.port = self.ctl.getsockname()[1]
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out.bind(("127.0.0.1", 0))
        self.sub = None
        self.sub_t = 0.0
        self.auds: list[float] = []
        self.audx = 0
        self.running = True
        self.lock = threading.Lock()

    def run(self):
        while self.running:
            try:
                data, addr = self.ctl.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            with self.lock:
                if data.startswith(b"AUDS "):
                    self.sub = (addr[0], int(data[5:]))
                    self.sub_t = time.time()
                    self.auds.append(self.sub_t)
                elif data == b"AUDX":
                    self.sub = None
                    self.audx += 1

    def subscriber(self):
        with self.lock:
            if self.sub and time.time() - self.sub_t > 5.0:
                self.sub = None
            return self.sub

    def send(self, datagrams):
        sub = self.subscriber()
        if sub:
            for d in datagrams:
                self.out.sendto(d, sub)

    def wait_sub(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not self.subscriber():
            time.sleep(0.01)
        return self.subscriber()

    def close(self):
        self.running = False
        self.ctl.close()
        self.out.close()


def wait_until(pred, timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_udp_source_subscribes_reassembles_and_counts_loss():
    esp = FakeEspUdp()
    esp.start()
    src = UdpAudioSource(f"udp://127.0.0.1:{esp.port}")
    got = []
    src.start(got.append)
    try:
        assert esp.wait_sub() and esp.subscriber()[1] == src.local_port
        stray = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for seq in range(20):
            _, dg = frame2(seq)
            if seq == 5:
                dg = dg[:2]  # a part lost on the air
            if seq == 9:
                esp.send(frame2(10)[1])  # 10 overtakes 9
            if seq == 10:
                continue
            if seq == 12:
                esp.send([b"\x00garbage"])
                stray.sendto(frame2(12)[1][0], ("127.0.0.1", src.local_port))  # right port, wrong sender? same IP here
            esp.send(dg)
            time.sleep(0.002)
        stray.close()
        assert wait_until(lambda: len(got) >= 19)
        seqs = [f.seq for f in got]
        assert seqs == [s for s in range(20) if s != 5]
        assert src.lost_frames == 1 and got[5].lost_before == 1
        assert src.link_up and src.reasm.garbage >= 1
    finally:
        src.stop()
        esp.close()


def test_udp_source_resubscribes_reports_stall_and_says_goodbye():
    esp = FakeEspUdp()
    esp.start()
    src = UdpAudioSource(f"udp://127.0.0.1:{esp.port}", resubscribe_s=0.2, stall_s=0.5)
    got = []
    src.start(got.append)
    try:
        assert esp.wait_sub()
        for seq in range(5):
            esp.send(frame2(seq)[1])
        assert wait_until(lambda: len(got) == 5 and src.link_up)
        n0 = len(esp.auds)
        time.sleep(0.9)  # the scanner goes quiet (Wi-Fi drop, reboot)
        assert not src.link_up and "no audio datagrams" in src.error
        assert len(esp.auds) - n0 >= 3  # kept subscribing all along
        for seq in range(500, 505):  # back, far ahead: downtime, not 495 lost frames
            esp.send(frame2(seq)[1])
        assert wait_until(lambda: len(got) == 10 and src.link_up)
        assert src.lost_frames == 0 and src.connects == 2
    finally:
        src.stop()
    assert wait_until(lambda: esp.audx == 1)
    esp.close()


def test_udp_source_never_blocks_without_a_scanner():
    t0 = time.monotonic()
    src = UdpAudioSource("udp://127.0.0.1:9", stall_s=0.3)  # discard port: nobody answers
    src.start(lambda f: None)
    assert time.monotonic() - t0 < 0.1
    time.sleep(0.3)
    assert src.link_up is False and src.subscriptions >= 1
    t0 = time.monotonic()
    src.stop()
    assert time.monotonic() - t0 < 1.0
