#!/usr/bin/env python3
"""audio_check.py: validate the OBSLAM four-microphone stream (TCP 8901, 'AUD1').

Connects to the scanner, checks every frame against docs/api.md section 5.2
(magic, header fields, payload size, sequence gaps, flags), prints per-channel
RMS and peak level in dBFS once a second, and prints totals on Ctrl-C.
Optionally saves the four channels to a WAV file for listening / plotting.

Standard library only. Examples:

    python3 firmware/tools/audio_check.py 192.168.4.1
    python3 firmware/tools/audio_check.py obslam-tilt.local:8901 --wav out.wav
    python3 firmware/tools/audio_check.py 192.168.4.1 --seconds 10 --wav knock.wav
"""

import argparse
import array
import math
import socket
import struct
import sys
import time
import wave

MAGIC = 0x31445541  # 'AUD1' little-endian
HDR = struct.Struct("<IIQIHHHH")  # magic seq t_us rate channels n shift flags = 28 bytes
MAX_N = 48000  # sanity bound on samples per channel per frame


def dbfs(x):
    return 20.0 * math.log10(x / 32768.0) if x > 0 else float("-inf")


def fmt_db(x):
    return "  -inf" if x == float("-inf") else f"{x:6.1f}"


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
        magic = struct.pack("<I", MAGIC)
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


def parse_target(target, port):
    if ":" in target and not target.startswith("["):
        host, p = target.rsplit(":", 1)
        return host, int(p)
    return target, port


def main():
    ap = argparse.ArgumentParser(description="Validate the AUD1 microphone stream on TCP 8901.")
    ap.add_argument("host", help="scanner address, e.g. 192.168.4.1 or obslam-tilt.local[:port]")
    ap.add_argument("--port", type=int, default=8901, help="TCP port (default 8901)")
    ap.add_argument("--wav", help="save the 4 channels as a 16-bit WAV file")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this many seconds (0 = until Ctrl-C)")
    args = ap.parse_args()

    host, port = parse_target(args.host, args.port)
    print(f"connecting to {host}:{port} ...", flush=True)
    sock = socket.create_connection((host, port), timeout=5)
    sock.settimeout(3)
    rd = Reader(sock)
    print("connected; levels per second (RMS / peak, dBFS), mic1..mic4", flush=True)

    wav = None
    tot = dict(frames=0, samples=0, lost=0, gaps=0, resyncs=0, skipped=0, bad=0,
               timeout_flags=0, drop_flags=0, rate_changes=0, clipped=[0, 0, 0, 0])
    first_t_us = last_t_us = None
    first_frame_wall = None
    lag_ms = 0.0
    first_wall = time.monotonic()
    last_seq = None
    rate = None

    # Per-second accumulators.
    acc_sq = [0.0] * 4
    acc_pk = [0] * 4
    acc_n = 0
    acc_frames = 0
    acc_lost = 0
    acc_flags = 0
    tick = time.monotonic()

    try:
        while True:
            hdr = rd.read(HDR.size)
            magic, seq, t_us, srate, ch, n, shift, flags = HDR.unpack(hdr)
            if magic != MAGIC:
                tot["resyncs"] += 1
                rd.buf[:0] = hdr[1:]  # re-scan from the byte after this bad header start
                tot["skipped"] += 1 + rd.resync()
                last_seq = None
                continue
            if ch != 4 or n == 0 or n > MAX_N or srate == 0:
                tot["bad"] += 1
                print(f"bad header: seq={seq} channels={ch} n={n} rate={srate}", file=sys.stderr)
                rd.resync()
                last_seq = None
                continue
            payload = rd.read(n * ch * 2)

            if rate is None:
                rate = srate
            elif srate != rate:
                tot["rate_changes"] += 1  # `A rate` while streaming: legal, but the WAV keeps the first rate
                print(f"sample rate changed {rate} -> {srate} at seq {seq}", file=sys.stderr)
                rate = srate

            if last_seq is not None:
                gap = (seq - last_seq - 1) & 0xFFFFFFFF
                if gap:
                    tot["gaps"] += 1
                    tot["lost"] += gap
                    acc_lost += gap
            last_seq = seq
            if flags & 1:
                tot["timeout_flags"] += 1
            if flags & 2:
                tot["drop_flags"] += 1
            acc_flags |= flags
            if first_t_us is None:
                first_t_us = t_us
                first_frame_wall = time.monotonic()
            last_t_us = t_us
            # How far this frame is behind real time, relative to the first one:
            # grows without bound if the scanner sends slower than it captures.
            lag_ms = ((time.monotonic() - first_frame_wall) - (t_us - first_t_us) / 1e6) * 1000

            pcm = array.array("h")
            pcm.frombytes(payload)
            if sys.byteorder != "little":
                pcm.byteswap()
            for c in range(4):
                chan = pcm[c::4]
                acc_sq[c] += sum(v * v for v in chan)
                pk = max(max(chan), -min(chan))
                if pk > acc_pk[c]:
                    acc_pk[c] = pk
                tot["clipped"][c] += sum(1 for v in chan if v >= 32767 or v <= -32768)
            acc_n += n
            acc_frames += 1
            tot["frames"] += 1
            tot["samples"] += n

            if args.wav:
                if wav is None:
                    wav = wave.open(args.wav, "wb")
                    wav.setnchannels(4)
                    wav.setsampwidth(2)
                    wav.setframerate(rate)
                wav.writeframes(payload)

            now = time.monotonic()
            if now - tick >= 1.0:
                rms = [dbfs(math.sqrt(acc_sq[c] / acc_n)) if acc_n else float("-inf") for c in range(4)]
                pk = [dbfs(acc_pk[c]) for c in range(4)]
                cols = "  ".join(f"m{c + 1} {fmt_db(rms[c])}/{fmt_db(pk[c])}" for c in range(4))
                extra = []
                if acc_lost:
                    extra.append(f"LOST {acc_lost}")
                if acc_flags & 1:
                    extra.append("I2S-TIMEOUT")
                if acc_flags & 2:
                    extra.append("DROPPED")
                print(f"{acc_frames:4d} fr  {rate // 1000}k  shift={shift}  lag={lag_ms:5.0f}ms  {cols}  {' '.join(extra)}",
                      flush=True)
                acc_sq = [0.0] * 4
                acc_pk = [0] * 4
                acc_n = acc_frames = acc_lost = acc_flags = 0
                tick = now

            if args.seconds and now - first_wall >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    except (ConnectionError, socket.timeout, OSError) as e:
        print(f"stream ended: {e}", file=sys.stderr)
    finally:
        sock.close()
        if wav is not None:
            wav.close()

    wall = time.monotonic() - first_wall
    print("\n--- totals ---")
    print(f"frames            {tot['frames']}  ({tot['samples']} samples per channel)")
    print(f"lost frames       {tot['lost']} in {tot['gaps']} seq gaps")
    print(f"flags             i2s-timeout {tot['timeout_flags']}, dropped-before {tot['drop_flags']}")
    print(f"bad headers       {tot['bad']}, magic resyncs {tot['resyncs']} ({tot['skipped']} bytes skipped)"
          f", rate changes {tot['rate_changes']}")
    print(f"clipped samples   " + "  ".join(f"m{c + 1} {tot['clipped'][c]}" for c in range(4)))
    if rate and wall > 0:
        print(f"wall time         {wall:.1f} s, received {tot['samples'] / wall:.0f} samples/s/ch "
              f"(nominal {rate}, {(tot['samples'] + tot['lost'] * (tot['samples'] / max(1, tot['frames']))) / wall:.0f} incl. lost)")
    if first_t_us is not None and last_t_us is not None and tot["frames"] > 1:
        span = (last_t_us - first_t_us) / 1e6
        print(f"device t_us span  {span:.3f} s over {tot['frames'] + tot['lost'] - 1} frame intervals")
    if args.wav and wav is not None:
        print(f"wav               {args.wav} (4 ch, 16-bit, {rate} Hz)")
    ok = tot["frames"] > 0 and tot["bad"] == 0 and tot["resyncs"] == 0
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
