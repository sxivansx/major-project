#!/usr/bin/env python3
"""audio_check.py: validate the On-board SLAM four-microphone stream.

Two transports, both per docs/api.md section 5.2:

  TCP 8901 (default): 'AUD1' frames on a byte stream.
  UDP (--udp):        'AUD2' datagrams. The checker subscribes by sending
                      `AUDS <port>` to the scanner's UDP 8902 every second,
                      reassembles each frame's parts, and sends `AUDX` on exit.

Every frame is checked (magic, header fields, sizes, sequence gaps, flags);
once a second it prints per-channel RMS and peak level in dBFS, frames lost
and how far behind real time the stream is (lag). Ctrl-C prints totals.
`--wav` saves the four channels (lost frames written as silence so the
timeline stays true).

Standard library only. Examples:

    python3 firmware/tools/audio_check.py 192.168.4.1
    python3 firmware/tools/audio_check.py 192.168.4.1 --udp
    python3 firmware/tools/audio_check.py 192.168.1.49 --udp --wav mics.wav --seconds 20
"""

import argparse
import array
import math
import socket
import struct
import sys
import time
import wave

MAGIC1 = 0x31445541  # 'AUD1'
MAGIC2 = 0x32445541  # 'AUD2'
HDR1 = struct.Struct("<IIQIHHHH")      # 28 B: magic seq t_us rate channels n shift flags
HDR2 = struct.Struct("<IIQIHHHHBBHH")  # 34 B: ... + part parts first count
MAX_N = 48000
REORDER_FRAMES = 5  # UDP: wait this many newer frames for a missing part (50 ms)


def dbfs(x):
    return 20.0 * math.log10(x / 32768.0) if x > 0 else float("-inf")


def fmt_db(x):
    return "  -inf" if x == float("-inf") else f"{x:6.1f}"


class Sink:
    """Per-second levels, totals and the WAV file, fed one complete frame at a time."""

    def __init__(self, wav_path):
        self.wav_path = wav_path
        self.wav = None
        self.rate = None
        self.shift = None
        self.tot = dict(frames=0, samples=0, lost=0, gaps=0, incomplete=0, resyncs=0, skipped=0,
                        bad=0, timeout_flags=0, drop_flags=0, rate_changes=0, clipped=[0, 0, 0, 0])
        self.first_t_us = None
        self.last_t_us = None
        self.first_frame_wall = None
        self.lag_ms = 0.0
        self.start = time.monotonic()
        self._reset_second()
        self.tick = time.monotonic()

    def _reset_second(self):
        self.acc_sq = [0.0] * 4
        self.acc_pk = [0] * 4
        self.acc_n = 0
        self.acc_frames = 0
        self.acc_lost = 0
        self.acc_flags = 0

    def lost(self, frames, n=None):
        """Frames that never arrived (seq gap) or arrived incomplete (UDP)."""
        self.tot["lost"] += frames
        self.acc_lost += frames
        if self.wav is not None and n:
            self.wav.writeframes(bytes(frames * n * 8))

    def frame(self, seq, t_us, rate, n, shift, flags, payload):
        if self.rate is None:
            self.rate = rate
        elif rate != self.rate:
            self.tot["rate_changes"] += 1  # `A rate` while streaming: legal; the WAV keeps the first rate
            print(f"sample rate changed {self.rate} -> {rate} at seq {seq}", file=sys.stderr)
            self.rate = rate
        self.shift = shift
        if flags & 1:
            self.tot["timeout_flags"] += 1
        if flags & 2:
            self.tot["drop_flags"] += 1
        self.acc_flags |= flags
        now = time.monotonic()
        if self.first_t_us is None:
            self.first_t_us = t_us
            self.first_frame_wall = now
        self.last_t_us = t_us
        self.lag_ms = ((now - self.first_frame_wall) - (t_us - self.first_t_us) / 1e6) * 1000

        pcm = array.array("h")
        pcm.frombytes(payload)
        if sys.byteorder != "little":
            pcm.byteswap()
        for c in range(4):
            chan = pcm[c::4]
            self.acc_sq[c] += sum(v * v for v in chan)
            pk = max(max(chan), -min(chan))
            if pk > self.acc_pk[c]:
                self.acc_pk[c] = pk
            self.tot["clipped"][c] += sum(1 for v in chan if v >= 32767 or v <= -32768)
        self.acc_n += n
        self.acc_frames += 1
        self.tot["frames"] += 1
        self.tot["samples"] += n

        if self.wav_path:
            if self.wav is None:
                self.wav = wave.open(self.wav_path, "wb")
                self.wav.setnchannels(4)
                self.wav.setsampwidth(2)
                self.wav.setframerate(rate)
            self.wav.writeframes(payload)

    def maybe_print(self):
        now = time.monotonic()
        if now - self.tick < 1.0:
            return
        rms = [dbfs(math.sqrt(self.acc_sq[c] / self.acc_n)) if self.acc_n else float("-inf") for c in range(4)]
        pk = [dbfs(self.acc_pk[c]) for c in range(4)]
        cols = "  ".join(f"m{c + 1} {fmt_db(rms[c])}/{fmt_db(pk[c])}" for c in range(4))
        total = self.acc_frames + self.acc_lost
        loss = f"lost {self.acc_lost:3d} ({100.0 * self.acc_lost / total:4.1f}%)" if total else "lost   0 ( 0.0%)"
        extra = []
        if self.acc_flags & 1:
            extra.append("I2S-TIMEOUT")
        if self.acc_flags & 2:
            extra.append("DROPPED")
        rate = f"{self.rate // 1000}k" if self.rate else "--"
        print(f"{self.acc_frames:4d} fr  {rate}  shift={self.shift}  {loss}  lag={self.lag_ms:5.0f}ms  {cols}  "
              f"{' '.join(extra)}", flush=True)
        self._reset_second()
        self.tick = now

    def close(self, transport):
        if self.wav is not None:
            self.wav.close()
        t = self.tot
        wall = time.monotonic() - self.start
        print(f"\n--- totals ({transport}) ---")
        print(f"frames            {t['frames']}  ({t['samples']} samples per channel)")
        lost_pct = 100.0 * t["lost"] / max(1, t["frames"] + t["lost"])
        line = f"lost frames       {t['lost']} ({lost_pct:.1f}%) in {t['gaps']} seq gaps"
        if transport == "udp":
            line += f", {t['incomplete']} of them incomplete (missing parts)"
        print(line)
        print(f"flags             i2s-timeout {t['timeout_flags']}, dropped-before {t['drop_flags']}")
        print(f"bad headers       {t['bad']}, magic resyncs {t['resyncs']} ({t['skipped']} bytes skipped)"
              f", rate changes {t['rate_changes']}")
        print("clipped samples   " + "  ".join(f"m{c + 1} {t['clipped'][c]}" for c in range(4)))
        if self.rate and wall > 0:
            print(f"wall time         {wall:.1f} s, received {t['samples'] / wall:.0f} samples/s/ch "
                  f"(nominal {self.rate}), last lag {self.lag_ms:.0f} ms")
        if self.first_t_us is not None and t["frames"] > 1:
            span = (self.last_t_us - self.first_t_us) / 1e6
            print(f"device t_us span  {span:.3f} s")
        if self.wav_path and self.wav is not None:
            print(f"wav               {self.wav_path} (4 ch, 16-bit, {self.rate} Hz; lost frames as silence)")
        return t["frames"] > 0 and t["bad"] == 0 and t["resyncs"] == 0


# ------------------------------------------------------------------ TCP AUD1 --

class Reader:
    """Buffered exact-size reads from a socket."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = bytearray()

    def read(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(max(65536, n - len(self.buf)))
            if not chunk:
                raise ConnectionError("connection closed by the scanner")
            self.buf += chunk
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def resync(self):
        """Drop bytes until the buffer starts with the magic. Returns bytes skipped."""
        skipped = 0
        magic = struct.pack("<I", MAGIC1)
        while True:
            i = self.buf.find(magic)
            if i >= 0:
                del self.buf[:i]
                return skipped + i
            keep = len(magic) - 1
            skipped += max(0, len(self.buf) - keep)
            del self.buf[: max(0, len(self.buf) - keep)]
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("connection closed by the scanner")
            self.buf += chunk


def run_tcp(host, port, sink, seconds):
    print(f"connecting to tcp {host}:{port} ...", flush=True)
    sock = socket.create_connection((host, port), timeout=5)
    sock.settimeout(3)
    rd = Reader(sock)
    print("connected; per second: frames, loss, lag, RMS/peak dBFS for mic1..mic4", flush=True)
    last_seq = None
    try:
        while True:
            hdr = rd.read(HDR1.size)
            magic, seq, t_us, rate, ch, n, shift, flags = HDR1.unpack(hdr)
            if magic != MAGIC1:
                sink.tot["resyncs"] += 1
                rd.buf[:0] = hdr[1:]
                sink.tot["skipped"] += 1 + rd.resync()
                last_seq = None
                continue
            if ch != 4 or n == 0 or n > MAX_N or rate == 0:
                sink.tot["bad"] += 1
                print(f"bad header: seq={seq} channels={ch} n={n} rate={rate}", file=sys.stderr)
                rd.resync()
                last_seq = None
                continue
            payload = rd.read(n * ch * 2)
            if last_seq is not None:
                gap = (seq - last_seq - 1) & 0xFFFFFFFF
                if gap:
                    sink.tot["gaps"] += 1
                    sink.lost(gap, n)
            last_seq = seq
            sink.frame(seq, t_us, rate, n, shift, flags, payload)
            sink.maybe_print()
            if seconds and time.monotonic() - sink.start >= seconds:
                break
    except KeyboardInterrupt:
        pass
    except (ConnectionError, socket.timeout, OSError) as e:
        print(f"stream ended: {e}", file=sys.stderr)
    finally:
        sock.close()


# ------------------------------------------------------------------ UDP AUD2 --

class Pending:
    __slots__ = ("t_us", "rate", "n", "shift", "flags", "parts", "got", "buf")

    def __init__(self, t_us, rate, n, shift, flags, parts):
        self.t_us, self.rate, self.n, self.shift, self.flags, self.parts = t_us, rate, n, shift, flags, parts
        self.got = set()
        self.buf = bytearray(n * 8)


def run_udp(host, port, sink, seconds, listen_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("", listen_port))
    my_port = sock.getsockname()[1]
    sock.settimeout(0.2)
    target = (socket.gethostbyname(host), port)
    sub = f"AUDS {my_port}".encode()
    print(f"udp: subscribing to {target[0]}:{target[1]} with '{sub.decode()}', every 1 s", flush=True)
    print("per second: frames, loss, lag, RMS/peak dBFS for mic1..mic4", flush=True)

    pending = {}
    next_seq = None   # oldest frame not yet emitted
    newest = None
    last_sub = 0.0
    last_rx = time.monotonic()

    in_gap = False

    def emit_ready(force=False):
        """Emit frames in seq order: complete ones at once; a missing or partial
        frame once REORDER_FRAMES newer frames have arrived (or at exit)."""
        nonlocal next_seq, in_gap
        while next_seq is not None:
            ahead = (newest - next_seq) & 0xFFFFFFFF
            if ahead & 0x80000000:
                break  # caught up past the newest frame
            p = pending.get(next_seq)
            if p is not None and len(p.got) == p.parts:
                del pending[next_seq]
                sink.frame(next_seq, p.t_us, p.rate, p.n, p.shift, p.flags, bytes(p.buf))
                in_gap = False
            elif force or ahead > REORDER_FRAMES:
                if not in_gap:
                    sink.tot["gaps"] += 1
                    in_gap = True
                if p is not None:
                    del pending[next_seq]
                    sink.tot["incomplete"] += 1
                    sink.lost(1, p.n)
                else:
                    sink.lost(1, (sink.rate or 16000) // 100)
            else:
                break
            next_seq = (next_seq + 1) & 0xFFFFFFFF

    try:
        while True:
            now = time.monotonic()
            if now - last_sub >= 1.0:
                sock.sendto(sub, target)
                last_sub = now
            if seconds and now - sink.start >= seconds:
                break
            if now - last_rx > 5:
                print("no datagrams for 5 s (is the scanner up, and audio=on?)", file=sys.stderr)
                last_rx = now
            sink.maybe_print()
            try:
                dg, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            last_rx = time.monotonic()
            if len(dg) < HDR2.size:
                sink.tot["bad"] += 1
                continue
            magic, seq, t_us, rate, ch, n, shift, flags, part, parts, first, count = HDR2.unpack_from(dg)
            if (magic != MAGIC2 or ch != 4 or n == 0 or n > MAX_N or rate == 0 or parts == 0
                    or part >= parts or first + count > n or len(dg) != HDR2.size + count * 8):
                sink.tot["bad"] += 1
                print(f"bad datagram: len={len(dg)} magic={magic:08x} seq={seq} n={n} part={part}/{parts} "
                      f"first={first} count={count}", file=sys.stderr)
                continue
            if next_seq is None:
                next_seq = seq
                newest = seq
            if (seq - next_seq) & 0x80000000:
                continue  # older than what was already emitted: late duplicate/straggler
            if (seq - newest) & 0x80000000 == 0:
                newest = seq
            p = pending.get(seq)
            if p is None:
                p = pending[seq] = Pending(t_us, rate, n, shift, flags, parts)
            if part not in p.got:
                p.got.add(part)
                p.buf[first * 8:(first + count) * 8] = dg[HDR2.size:]
            emit_ready()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.sendto(b"AUDX", target)
        except OSError:
            pass
        emit_ready(force=True)
        sock.close()


def main():
    ap = argparse.ArgumentParser(description="Validate the On-board SLAM microphone stream (TCP AUD1 or UDP AUD2).")
    ap.add_argument("host", help="scanner address, e.g. 192.168.4.1 or obslam-tilt.local[:port]")
    ap.add_argument("--udp", action="store_true", help="subscribe to the UDP stream (AUDS to port 8902)")
    ap.add_argument("--port", type=int, help="scanner port (default 8901 for TCP, 8902 for --udp)")
    ap.add_argument("--listen-port", type=int, default=0, help="--udp: local port to receive on (default: any)")
    ap.add_argument("--wav", help="save the 4 channels as a 16-bit WAV file")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this many seconds (0 = until Ctrl-C)")
    args = ap.parse_args()

    host, port = args.host, args.port
    if ":" in host and not host.startswith("["):
        host, p = host.rsplit(":", 1)
        port = port or int(p)
    port = port or (8902 if args.udp else 8901)

    sink = Sink(args.wav)
    if args.udp:
        run_udp(host, port, sink, args.seconds, args.listen_port)
    else:
        run_tcp(host, port, sink, args.seconds)
    ok = sink.close("udp" if args.udp else "tcp")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
