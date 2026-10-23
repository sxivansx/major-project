"""Microphone array geometry check: what a layout can resolve, and the `audio.mics_mm` block for it.

    uv run python scripts/mic_geometry.py --xy -35 0  -70 -19.5  0 -19.5  -35 -80
    uv run python scripts/mic_geometry.py --dist 40.1 40.1 80 70 69.9 69.9      # d12 d13 d14 d23 d24 d34
    uv run python scripts/mic_geometry.py --json board.json                      # {"mics_mm": [[x,y,z],…]}
    uv run python scripts/mic_geometry.py --xy … --write                         # into core/config.json

Positions are the sound ports in mm, mic1..mic4 in firmware channel order, viewed from above
with the ports facing up (x right, y away from you). From six distances alone a layout is only
known up to a mirror image: the script puts mic1 at the origin, mic2 on +x and mic3 on the +y
side; if mic3 is actually to the right of the mic1→mic2 line seen from above, pass --mirror.
A mirrored layout gives mirrored bearings and looks perfectly self-consistent, so check it.

Writing the block resets `calibrated` to false: new positions need a new clap calibration.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from obslam.acoustic import PAIR_KEYS, PAIRS, geometry_problem

ROOT = Path(__file__).resolve().parents[1]


def from_distances(d: list[float], mirror: bool) -> tuple[np.ndarray, float]:
    """Trilaterate: mic1 at the origin, mic2 on +x, mic3 on +y (or −y with mirror); mic4 takes
    whichever side of the 1–2 line matches d34 better. Returns positions and the d34 misfit."""
    d12, d13, d14, d23, d24, d34 = d
    p = np.zeros((4, 2))
    p[1] = (d12, 0.0)

    def tri(ra: float, rb: float) -> tuple[float, float]:
        x = (ra**2 - rb**2 + d12**2) / (2 * d12)
        y2 = ra**2 - x**2
        if y2 < -1.0:  # a few tenths of a mm of caliper slop is fine; centimetres are not
            raise ValueError(f"distances do not close a triangle (off by {math.sqrt(-y2):.1f} mm)")
        return x, math.sqrt(max(y2, 0.0))

    x3, y3 = tri(d13, d23)
    p[2] = (x3, -y3 if mirror else y3)
    x4, y4 = tri(d14, d24)
    cands = [np.array([x4, y4]), np.array([x4, -y4])]
    errs = [abs(float(np.linalg.norm(c - p[2])) - d34) for c in cands]
    p[3] = cands[int(np.argmin(errs))]
    return p, min(errs)


def resolution_table(p_m: np.ndarray, c: float, rate: float, interp: int, step_deg: float = 15.0) -> list[tuple]:
    """Per in-plane direction: the 1σ bearing error from TDOA quantisation alone (uniform, δτ/√12
    per pair, through the least-squares covariance) and the bearing change that moves the most
    sensitive pair by one interpolated step δτ."""
    dtau = 1.0 / (rate * interp)
    A = np.array([(p_m[j] - p_m[i]) / c for i, j in PAIRS])
    cov = (dtau**2 / 12.0) * np.linalg.inv(A.T @ A)
    rows = []
    for az in np.arange(0.0, 360.0, step_deg):
        a = math.radians(az)
        k = np.array([math.cos(a), math.sin(a)])
        g = np.array([-k[1], k[0]])  # d k / d az
        sigma = math.degrees(math.sqrt(float(g @ cov @ g)))
        dtau_daz = np.abs(A @ g)  # s per rad, each pair
        step = math.degrees(dtau / float(dtau_daz.max()))
        rows.append((az, sigma, step))
    return rows


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--xy", nargs=8, type=float, metavar="MM", help="x1 y1 x2 y2 x3 y3 x4 y4 in mm")
    src.add_argument("--dist", nargs=6, type=float, metavar="MM", help="d12 d13 d14 d23 d24 d34 in mm")
    src.add_argument("--json", type=Path, help='file with "mics_mm" (or a whole config with audio.mics_mm)')
    ap.add_argument("--mirror", action="store_true", help="with --dist: mic3 is on the -y side of mic1→mic2")
    ap.add_argument("--c", type=float, default=343.0, help="speed of sound, m/s (331 + 0.6·°C)")
    ap.add_argument("--rate", type=float, default=48000.0)
    ap.add_argument("--interp", type=int, default=16, help="GCC interpolation factor")
    ap.add_argument("--write", nargs="?", const=str(ROOT / "config.json"), metavar="CONFIG", help="merge audio.mics_mm into this config (default core/config.json)")
    args = ap.parse_args(argv)

    if args.xy:
        p = np.array(args.xy, float).reshape(4, 2)
        print("positions from x, y")
    elif args.dist:
        try:
            p, misfit = from_distances(args.dist, args.mirror)
        except ValueError as e:
            print(f"error: {e}", file=sys.stderr)
            return 1
        print(f"positions from distances (mic1 at origin, mic2 on +x, mic3 on {'-' if args.mirror else '+'}y); d34 misfit {misfit:.2f} mm")
        if misfit > 1.0:
            print(f"  WARNING: the six distances disagree by {misfit:.1f} mm: re-measure (or the board is not flat)")
        print("  CHECK the mirror: seen from above with the ports up, is mic3 left of the mic1→mic2 line? If not, rerun with --mirror")
    else:
        if args.json:
            data = json.loads(args.json.read_text())
            mics = data.get("mics_mm") or data.get("audio", {}).get("mics_mm")
        else:
            from obslam.config import AudioConfig

            mics = AudioConfig().mics_mm
            print("(no input: showing the default layout from obslam.config)")
        if not mics:
            print("error: no mics_mm found", file=sys.stderr)
            return 1
        p = np.array(mics, float)[:, :2]

    p_m = p / 1000.0
    problem = geometry_problem(np.c_[p_m, np.zeros(4)])
    print("\nmic   x mm     y mm")
    for i, (x, y) in enumerate(p, 1):
        print(f"  {i}  {x:8.2f} {y:8.2f}")
    if problem:
        print(f"\nunusable: {problem}")
        return 1

    print(f"\npair  dist mm  max TDOA µs  samples @{args.rate / 1000:.0f}k  aliasing above")
    for key, (i, j) in zip(PAIR_KEYS, PAIRS):
        d = float(np.linalg.norm(p_m[i] - p_m[j]))
        tmax = d / args.c
        print(f"  {key}  {d * 1000:7.2f}  {tmax * 1e6:10.1f}  {tmax * args.rate:12.2f}  {args.c / (2 * d) / 1000:6.2f} kHz")
    span = max(float(np.linalg.norm(p_m[i] - p_m[j])) for i, j in PAIRS)
    print(f"\naperture {span * 1000:.1f} mm. Direction only: at this size the wavefront curvature of a source 1 m away is")
    print(f"{(span**2 / 8 / 1.0) / args.c * 1e6:.2f} µs across the array, below what GCC can measure, so there is no range.")

    dtau = 1e6 / (args.rate * args.interp)
    print(f"\nangular resolution at {args.rate / 1000:.0f} kHz × {args.interp} interpolation (δτ = {dtau:.2f} µs), source in the array plane")
    print("  array az°  σ from quantisation°  one δτ step°")
    rows = resolution_table(p_m, args.c, args.rate, args.interp)
    for az, sigma, step in rows:
        print(f"  {az:8.0f}  {sigma:20.2f}  {step:12.2f}")
    sig = [r[1] for r in rows]
    print(f"  worst σ {max(sig):.2f}°, best {min(sig):.2f}° (anisotropy {max(sig) / min(sig):.2f}×).")
    print("  Real-world error is set by SNR, reverberation and calibration (typically 1–5°), not by this floor.")

    block = {"audio": {"mics_mm": [[round(float(x), 2), round(float(y), 2), 0.0] for x, y in p], "calibrated": False}}
    print("\n" + json.dumps(block))
    if args.write:
        path = Path(args.write)
        cfg = json.loads(path.read_text()) if path.exists() else {}
        cfg.setdefault("audio", {}).update(block["audio"])
        path.write_text(json.dumps(cfg, indent=2) + "\n")
        print(f"written to {path} (calibrated=false: run the clap calibration next)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
