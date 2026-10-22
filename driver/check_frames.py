#!/usr/bin/env python3
"""Validate s2bridge frames on stdin and print per-frame stats. Stdlib only.

    ./build/s2bridge --serial /dev/cu.usbserial-0001 | python3 check_frames.py
    python3 check_frames.py --fake 5 | python3 check_frames.py    # no hardware

Frame format: docs/api.md section 1. Exits 1 on the first malformed frame.
"""

import math
import random
import struct
import sys
import time

MAGIC = 0x52463253
HEADER = struct.Struct("<IId")  # magic, n, t_unix
POINT = struct.Struct("<ffB")   # angle_deg, range_m, quality
assert HEADER.size == 16 and POINT.size == 9
MAX_POINTS = 100_000            # sanity bound; the S2 gives ~3,200 per rotation


def fail(msg):
    print(f"check_frames: {msg}", file=sys.stderr)
    sys.exit(1)


def read_exact(stream, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            break
        buf += chunk
    return bytes(buf)


def check(stream):
    frames = 0
    points = 0
    t_prev = None
    hz_sum = 0.0
    hz_n = 0
    try:
        while True:
            hdr = read_exact(stream, HEADER.size)
            if not hdr:
                break  # clean EOF on a frame boundary
            if len(hdr) < HEADER.size:
                fail(f"frame {frames}: truncated header ({len(hdr)} of {HEADER.size} bytes)")
            magic, n, t = HEADER.unpack(hdr)
            if magic != MAGIC:
                fail(f"frame {frames}: bad magic 0x{magic:08x}, expected 0x{MAGIC:08x}")
            if n > MAX_POINTS:
                fail(f"frame {frames}: implausible n={n}")
            body = read_exact(stream, n * POINT.size)
            if len(body) < n * POINT.size:
                fail(f"frame {frames}: truncated body ({len(body)} of {n * POINT.size} bytes)")

            rmin = math.inf
            rmax = 0.0
            bad = 0
            for angle, rng, _quality in POINT.iter_unpack(body):
                if not (0.0 <= angle <= 360.0) or not (rng > 0.0) or math.isnan(rng):
                    bad += 1
                    continue
                rmin = min(rmin, rng)
                rmax = max(rmax, rng)
            if n == 0:
                rmin = 0.0

            hz_text = "    -"
            if t_prev is not None and t > t_prev:
                hz = 1.0 / (t - t_prev)
                hz_sum += hz
                hz_n += 1
                hz_text = f"{hz:5.1f}"
            t_prev = t
            frames += 1
            points += n
            print(f"frame {frames:5d}  n={n:5d}  range {rmin:6.2f}..{rmax:6.2f} m"
                  f"  hz={hz_text}  bad={bad}", flush=True)
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        return
    mean_hz = hz_sum / hz_n if hz_n else 0.0
    print(f"ok: {frames} frames, {points} points, mean {mean_hz:.1f} Hz", flush=True)


def fake(count, out):
    """Write `count` synthetic frames (3,200 points, 10 Hz timestamps)."""
    rng = random.Random(1)
    t = time.time()
    n = 3200
    for _ in range(count):
        out.write(HEADER.pack(MAGIC, n, t))
        body = bytearray()
        for i in range(n):
            angle = 360.0 * i / n
            body += POINT.pack(angle, rng.uniform(0.2, 12.0), rng.randrange(0, 64))
        out.write(body)
        t += 0.1
    out.flush()


def main(argv):
    if len(argv) >= 2 and argv[1] == "--fake":
        count = int(argv[2]) if len(argv) > 2 else 5
        fake(count, sys.stdout.buffer)
        return
    if len(argv) > 1:
        fail(f"unknown argument {argv[1]!r}; usage: check_frames.py [--fake N]")
    check(sys.stdin.buffer)


if __name__ == "__main__":
    main(sys.argv)
