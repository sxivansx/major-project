"""Handheld 2D room mapping from the level LiDAR (docs/api.md §3.5): carry the scanner around, each
rotation is matched to the map built so far, and the room's floor plan fills in.

Map: log-odds occupancy grid in the frame of the first scan (+x = LiDAR 0° mark at the start, +y to
its left, metres). Matching: each scan is scored against a distance field (distance from every cell
to the nearest occupied cell, clipped): a brute-force search over a small pose window on a coarse
field, then Gauss-Newton on the fine one, so a room-sized map needs no KD-tree or scipy. The map is
only updated from keyframes (moved ≥ 10 cm or turned ≥ 4°) and never while the match is poor.

LiDAR angles are clockwise, so a return at (angle a, range r) sits at (r cos a, −r sin a) in the
scanner frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

L_HIT, L_MISS, L_MIN, L_MAX = 0.9, -0.4, -4.0, 4.0
L_OCC = 0.6  # log-odds above this counts as a wall for matching and display
L_FREE = -0.2
FINE_CLIP_M = 0.30
COARSE_CLIP_M = 0.60
KEY_DIST_M = 0.10
KEY_TURN_DEG = 4.0
INLIER_M = 0.10
MIN_INLIERS = 0.35  # below this fraction of points within INLIER_M of a wall, the scan is not trusted
RANGE_MIN_M, RANGE_MAX_M = 0.15, 12.0
MIN_FRONTIER_M = 0.40  # an open edge narrower than this is a gap between chair legs, not somewhere to go
STAND_CLEAR_M = 0.25  # a suggested station keeps this far from walls
STAND_SEARCH_M = 0.8  # …and lies within this of the open edge it is meant to see past
NEAR_M = 0.4  # an edge closer than this to the scanner is one it already stands at
STATION_SPAN_M = 3.0  # a new station is searched for this far around the last one…
STATION_STEP_M = 0.2
STATION_STEP_DEG = 4.0  # …at any heading
STATION_MIN_INLIERS = 0.55  # a station sees new room, so overlap is lower than for a walking scan…
STATION_MAX_RMS_M = 0.040  # …but what does overlap must sit tight (real still scans: 15–25 mm; a wrong fit ≈ 50)
STATION_MAX_CONFLICT = 0.2  # walls seen where the map knows open floor: more than this and the placement is wrong
STATION_PRIOR = 0.02  # inlier fraction given up per metre from the last station (breaks ties in symmetric rooms)
BIN_DEG = 0.5


def scan_to_points(angle_deg: np.ndarray, rng: np.ndarray, quality: np.ndarray | None = None, bin_deg: float = BIN_DEG) -> np.ndarray:
    """One rotation → (N, 2) scanner-frame points, one per `bin_deg` sector (median range)."""
    a = np.asarray(angle_deg, np.float64) % 360.0
    r = np.asarray(rng, np.float64)
    ok = (r > RANGE_MIN_M) & (r < RANGE_MAX_M) & np.isfinite(r)
    if quality is not None:
        ok &= np.asarray(quality) > 0
    a, r = a[ok], r[ok]
    if a.size == 0:
        return np.zeros((0, 2))
    b = (a / bin_deg).astype(np.int64)
    order = np.lexsort((r, b))
    b, a, r = b[order], a[order], r[order]
    starts = np.r_[0, np.nonzero(np.diff(b))[0] + 1]
    ends = np.r_[starts[1:], b.size]
    mid = (starts + ends - 1) // 2  # median within each sector (ranges sorted per sector)
    rad = np.radians(a[mid])
    return np.column_stack([r[mid] * np.cos(rad), -r[mid] * np.sin(rad)])


def transform(pts: np.ndarray, pose) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([c * pts[:, 0] - s * pts[:, 1] + x, s * pts[:, 0] + c * pts[:, 1] + y])


def distance_field(occ: np.ndarray, cell: float, clip_m: float) -> np.ndarray:
    """Chamfer distance (m) to the nearest True cell, clipped at clip_m. 8-neighbour relaxation."""
    big = np.float32(clip_m)
    d = np.where(occ, np.float32(0.0), big).astype(np.float32)
    steps = int(math.ceil(clip_m / cell))
    diag = np.float32(cell * math.sqrt(2.0))
    for _ in range(steps):
        p = np.pad(d, 1, constant_values=big)
        n = np.minimum.reduce([p[1:-1, :-2], p[1:-1, 2:], p[:-2, 1:-1], p[2:, 1:-1]]) + np.float32(cell)
        g = np.minimum.reduce([p[:-2, :-2], p[:-2, 2:], p[2:, :-2], p[2:, 2:]]) + diag
        nd = np.minimum(d, np.minimum(n, g))
        if np.array_equal(nd, d):
            break
        d = nd
    return np.minimum(d, big)


def place_at(xyz: np.ndarray, pose) -> np.ndarray:
    """Scanner-frame 3D points → room-map frame: turn about z by theta, shift by (x, y)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = np.empty_like(xyz)
    out[:, 0] = c * xyz[:, 0] - s * xyz[:, 1] + x
    out[:, 1] = s * xyz[:, 0] + c * xyz[:, 1] + y
    out[:, 2] = xyz[:, 2]
    return out


@dataclass
class Match:
    pose: tuple[float, float, float]
    rms_m: float
    inliers: float
    ok: bool


@dataclass
class RoomMapper:
    cell: float = 0.05
    size_m: float = 24.0  # square map centred on the start; a house floor fits
    L: np.ndarray = field(init=False)
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, theta (rad, CCW)

    def __post_init__(self):
        self.n = int(round(self.size_m / self.cell))
        self.x0 = self.y0 = -self.n * self.cell / 2.0
        self.L = np.zeros((self.n, self.n), np.float32)
        self.key_pose: tuple[float, float, float] | None = None
        self.keyframes = 0
        self.scans = 0
        self.lost = False
        self.last: Match | None = None
        self.path: list[tuple[float, float]] = []
        self._velocity = (0.0, 0.0, 0.0)
        self._fine: np.ndarray | None = None
        self._next: dict | None = None
        self._next_key: tuple | None = None
        self._frontier: np.ndarray | None = None
        self._vcoarse: np.ndarray | None = None
        self.stations: list[dict] = []
        self._coarse: np.ndarray | None = None

    # ---- grid helpers ------------------------------------------------------------------------------
    def _idx(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ix = np.floor((xy[:, 0] - self.x0) / self.cell).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.y0) / self.cell).astype(np.int64)
        return ix, iy

    def _fields(self) -> None:
        occ = self.L > L_OCC
        self._fine = distance_field(occ, self.cell, FINE_CLIP_M)
        # coarse: half resolution, wider basin for the brute-force search
        h = self.n // 2 * 2
        occ2 = occ[:h, :h].reshape(h // 2, 2, h // 2, 2).any(axis=(1, 3))
        self._coarse = distance_field(occ2, self.cell * 2, COARSE_CLIP_M)
        self._vcoarse = None  # built on demand for station searches

    @staticmethod
    def _lookup(field_: np.ndarray, cell: float, x0: float, y0: float, xy: np.ndarray, clip: float) -> np.ndarray:
        ix = np.floor((xy[..., 0] - x0) / cell).astype(np.int64)
        iy = np.floor((xy[..., 1] - y0) / cell).astype(np.int64)
        h, w = field_.shape
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        out = np.full(ix.shape, clip, np.float32)
        out[ok] = field_[iy[ok], ix[ok]]
        return out

    def _bilinear(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Fine field value and gradient (d/dx, d/dy) at world points."""
        f = self._fine
        u = (xy[:, 0] - self.x0) / self.cell - 0.5
        v = (xy[:, 1] - self.y0) / self.cell - 0.5
        i0 = np.clip(np.floor(u).astype(np.int64), 0, self.n - 2)
        j0 = np.clip(np.floor(v).astype(np.int64), 0, self.n - 2)
        fu = np.clip(u - i0, 0.0, 1.0)
        fv = np.clip(v - j0, 0.0, 1.0)
        f00, f10 = f[j0, i0], f[j0, i0 + 1]
        f01, f11 = f[j0 + 1, i0], f[j0 + 1, i0 + 1]
        val = (1 - fv) * ((1 - fu) * f00 + fu * f10) + fv * ((1 - fu) * f01 + fu * f11)
        gx = ((1 - fv) * (f10 - f00) + fv * (f11 - f01)) / self.cell
        gy = ((1 - fu) * (f01 - f00) + fu * (f11 - f10)) / self.cell
        return val, gx, gy

    # ---- matching --------------------------------------------------------------------------------
    def _search(self, pts: np.ndarray, guess, span_m: float, step_m: float, span_deg: float, step_deg: float):
        """Brute force on the coarse field: the pose in the window with the lowest summed clipped distance²."""
        c2 = self.cell * 2
        best, best_cost = guess, np.inf
        offs = np.arange(-span_m, span_m + 1e-9, step_m)
        dx, dy = np.meshgrid(offs, offs, indexing="ij")
        dx, dy = dx.ravel(), dy.ravel()
        for dth in np.radians(np.arange(-span_deg, span_deg + 1e-9, step_deg)):
            th = guess[2] + dth
            w = transform(pts, (guess[0], guess[1], th))  # (N, 2)
            xy = w[None, :, :] + np.stack([dx, dy], axis=1)[:, None, :]  # (T, N, 2)
            d = self._lookup(self._coarse, c2, self.x0, self.y0, xy, COARSE_CLIP_M)
            cost = (d * d).sum(axis=1)
            k = int(np.argmin(cost))
            if cost[k] < best_cost:
                best_cost, best = cost[k], (guess[0] + dx[k], guess[1] + dy[k], th)
        return best

    def _refine(self, pts: np.ndarray, pose, iters: int = 12):
        x, y, th = pose
        for _ in range(iters):
            w = transform(pts, (x, y, th))
            r, gx, gy = self._bilinear(w)
            use = r < FINE_CLIP_M * 0.95  # points with no wall near them say nothing about the pose
            if use.sum() < 10:
                break
            c, s = math.cos(th), math.sin(th)
            px, py = pts[use, 0], pts[use, 1]
            dth = gx[use] * (-s * px - c * py) + gy[use] * (c * px - s * py)
            J = np.column_stack([gx[use], gy[use], dth])
            ru = r[use]
            k = 0.05  # Huber
            wgt = np.where(ru <= k, 1.0, k / np.maximum(ru, 1e-9))
            H = J.T @ (J * wgt[:, None]) + np.diag([1e-4, 1e-4, 1e-4])
            g = J.T @ (wgt * ru)
            step = -np.linalg.solve(H, g)
            x, y, th = x + step[0], y + step[1], th + step[2]
            if abs(step[0]) + abs(step[1]) < 1e-4 and abs(step[2]) < 1e-4:
                break
        return (x, y, th)

    def _quality(self, pts: np.ndarray, pose) -> tuple[float, float]:
        d = self._lookup(self._fine, self.cell, self.x0, self.y0, transform(pts, pose), FINE_CLIP_M)
        near = d < INLIER_M
        rms = float(np.sqrt(np.mean(d[near] ** 2))) if near.any() else float("nan")
        return rms, float(near.mean())

    def match(self, pts: np.ndarray) -> Match:
        vx, vy, vth = self._velocity
        guess = (self.pose[0] + vx, self.pose[1] + vy, self.pose[2] + vth)
        if self.lost:
            p = self._search(pts, self.pose, 0.6, 0.06, 30.0, 2.0)
        else:
            p = self._search(pts, guess, 0.2, 0.04, 8.0, 1.0)
        p = self._refine(pts, p)
        rms, inl = self._quality(pts, p)
        return Match(p, rms, inl, inl >= MIN_INLIERS)

    # ---- update ----------------------------------------------------------------------------------
    def integrate(self, pts: np.ndarray, pose) -> None:
        """Log-odds update: free along each beam, hit at its end."""
        x, y, th = pose
        w = transform(pts, pose)
        vec = w - np.array([x, y])
        dist = np.hypot(vec[:, 0], vec[:, 1])
        step = self.cell * 0.5
        ns = np.maximum((dist - self.cell) / step, 0).astype(np.int64)
        total = int(ns.sum())
        if total:
            beam = np.repeat(np.arange(pts.shape[0]), ns)
            k = np.arange(total) - np.repeat(np.cumsum(ns) - ns, ns)
            t = (k * step) / np.repeat(dist, ns)
            free = np.column_stack([x + vec[beam, 0] * t, y + vec[beam, 1] * t])
            fx, fy = self._idx(free)
            ok = (fx >= 0) & (fx < self.n) & (fy >= 0) & (fy < self.n)
            flat = np.unique(fy[ok] * self.n + fx[ok])
            self.L.flat[flat] += L_MISS
        hx, hy = self._idx(w)
        ok = (hx >= 0) & (hx < self.n) & (hy >= 0) & (hy < self.n)
        flat = np.unique(hy[ok] * self.n + hx[ok])
        self.L.flat[flat] += L_HIT  # the free samples stop a cell short, so a hit cell is not also freed
        np.clip(self.L, L_MIN, L_MAX, out=self.L)
        self._fields()

    def add_scan(self, pts: np.ndarray) -> Match | None:
        """One rotation (scanner-frame points). Returns the match, or None for the very first scan."""
        self.scans += 1
        if pts.shape[0] < 50:
            return self.last
        if self.key_pose is None:
            self.integrate(pts, self.pose)
            self.key_pose = self.pose
            self.keyframes = 1
            self.path.append((0.0, 0.0))
            return None
        m = self.match(pts)
        self.last = m
        if not m.ok:
            self.lost = True
            self._velocity = (0.0, 0.0, 0.0)
            return m
        self.lost = False
        prev = self.pose
        self.pose = m.pose
        self._velocity = tuple(0.5 * (a - b) for a, b in zip(m.pose, prev))
        if math.hypot(m.pose[0] - self.path[-1][0], m.pose[1] - self.path[-1][1]) > 0.02:
            self.path.append((m.pose[0], m.pose[1]))
        kx, ky, kth = self.key_pose
        moved = math.hypot(m.pose[0] - kx, m.pose[1] - ky)
        turned = abs(math.degrees((m.pose[2] - kth + math.pi) % (2 * math.pi) - math.pi))
        if moved >= KEY_DIST_M or turned >= KEY_TURN_DEG:
            self.integrate(pts, m.pose)
            self.key_pose = m.pose
            self.keyframes += 1
        return m

    # ---- stations (scanner set down still, carried in between) ---------------------------------------
    def _global(self, pts: np.ndarray, center, span_m: float = STATION_SPAN_M, keep: int = 6) -> list[tuple[float, float, float]]:
        """Brute force over ±span_m and all headings on a 4× coarse field; the best `keep` poses."""
        c4 = self.cell * 4
        if self._vcoarse is None:
            occ = self.L > L_OCC
            h = self.n // 4 * 4
            occ4 = occ[:h, :h].reshape(h // 4, 4, h // 4, 4).any(axis=(1, 3))
            self._vcoarse = distance_field(occ4, c4, 1.0)
        sub = pts[:: max(1, pts.shape[0] // 200)]
        offs = np.arange(-span_m, span_m + 1e-9, STATION_STEP_M)
        dx, dy = np.meshgrid(offs, offs, indexing="ij")
        T = np.stack([dx.ravel() + center[0], dy.ravel() + center[1]], axis=1)
        cands: list[tuple[float, float, float, float]] = []
        for th in np.radians(np.arange(0.0, 360.0, STATION_STEP_DEG)):
            w = transform(sub, (0.0, 0.0, th))
            d = self._lookup(self._vcoarse, c4, self.x0, self.y0, w[None, :, :] + T[:, None, :], 1.0)
            cost = (d * d).mean(axis=1)
            for k in np.argsort(cost)[:2]:
                cands.append((float(cost[k]), float(T[k, 0]), float(T[k, 1]), float(th)))
        cands.sort()
        out: list[tuple[float, float, float]] = []
        for _, x, y, th in cands:  # distinct poses only
            if all(math.hypot(x - a, y - b) > 0.4 or abs(math.remainder(th - t, 2 * math.pi)) > math.radians(15) for a, b, t in out):
                out.append((x, y, th))
            if len(out) >= keep:
                break
        return out

    def _conflict(self, pts: np.ndarray, pose) -> float:
        """Fraction of beams that contradict the map: the return lands on floor known to be open (far
        from any wall), or the beam passes through a known wall on its way (a placement behind a wall)."""
        sub = pts[:: max(1, pts.shape[0] // 360)]
        w = transform(sub, pose)
        ix, iy = self._idx(w)
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        bad = np.zeros(w.shape[0], bool)
        bad[ok] = (self.L[iy[ok], ix[ok]] < L_FREE) & (self._fine[iy[ok], ix[ok]] > INLIER_M)
        x, y = pose[0], pose[1]
        vec = w - np.array([x, y])
        dist = np.hypot(vec[:, 0], vec[:, 1])
        step = self.cell
        ns = np.maximum((dist - 3 * self.cell) / step, 0).astype(np.int64)  # stop short of the return's own wall
        total = int(ns.sum())
        if total:
            beam = np.repeat(np.arange(w.shape[0]), ns)
            k = np.arange(total) - np.repeat(np.cumsum(ns) - ns, ns)
            t = (k * step) / np.repeat(dist, ns)
            sx, sy = self._idx(np.column_stack([x + vec[beam, 0] * t, y + vec[beam, 1] * t]))
            inside = (sx >= 0) & (sx < self.n) & (sy >= 0) & (sy < self.n)
            hit = np.zeros(total, bool)
            hit[inside] = self.L[sy[inside], sx[inside]] > L_OCC
            crossed = np.zeros(w.shape[0], bool)
            crossed[beam[hit]] = True
            bad |= crossed
        return float(bad.mean())

    def add_station(self, pts: np.ndarray, guess=None) -> dict:
        """A still scan from a new spot. The first one starts the map; later ones are located with a wide
        search around `guess` (default: the last station) and integrated only if they overlap enough."""
        if pts.shape[0] < 50:
            return {"ok": False, "reason": "too few LiDAR points: is it running and unobstructed?"}
        n = len(self.stations) + 1
        if self.key_pose is None:
            self.integrate(pts, self.pose)
            self.key_pose = self.pose
            self.keyframes = 1
            self.path.append((self.pose[0], self.pose[1]))
            st = {"n": n, "x": 0.0, "y": 0.0, "theta_deg": 0.0, "inliers": 1.0, "rms_mm": None}
            self.stations.append(st)
            return {"ok": True, **st}
        last = self.stations[-1] if self.stations else {"x": self.pose[0], "y": self.pose[1]}
        center = guess if guess is not None else (last["x"], last["y"])
        best, best_score = None, -1.0
        for c in self._global(pts, center):
            p = self._search(pts, c, 0.2, 0.04, 4.0, 1.0)
            p = self._refine(pts, p)
            rms, inl = self._quality(pts, p)
            bad = self._conflict(pts, p)
            score = inl - bad - STATION_PRIOR * math.hypot(p[0] - center[0], p[1] - center[1])
            if score > best_score:
                best_score, best = score, (p, rms, inl, bad)
        p, rms, inl, bad = best
        ok = inl >= STATION_MIN_INLIERS and bad <= STATION_MAX_CONFLICT and (rms == rms and rms <= STATION_MAX_RMS_M)
        self.last = Match(p, rms, inl, ok)
        if not ok:
            return {
                "ok": False,
                "inliers": round(inl, 3),
                "conflict": round(bad, 3),
                "reason": f"only {inl * 100:.0f}% of this view matches the map: put the scanner where it can see some walls already mapped, within 3 m of the last station",
            }
        self.pose = p
        self.lost = False
        self.integrate(pts, p)
        self.key_pose = p
        self.keyframes += 1
        self.path.append((p[0], p[1]))
        st = {
            "n": n, "x": round(p[0], 3), "y": round(p[1], 3), "theta_deg": round(math.degrees(p[2]), 1),
            "inliers": round(inl, 3), "rms_mm": None if rms != rms else round(rms * 1000, 1),
        }
        self.stations.append(st)
        return {"ok": True, **st}

    # ---- output ----------------------------------------------------------------------------------
    def classes(self) -> tuple[np.ndarray, float, float]:
        """u8 grid cropped to what has been seen (+0.5 m): 0 unknown, 1 free, 2 occupied, 3 open edge; and its x0, y0."""
        c = np.zeros(self.L.shape, np.uint8)
        c[self.L < L_FREE] = 1
        c[self.L > L_OCC] = 2
        if self._frontier is not None and self._frontier.shape == c.shape:
            c[self._frontier & (c == 1)] = 3  # open edge (as of the last next_station)
        ys, xs = np.nonzero(c)
        if ys.size == 0:
            return c[:1, :1], self.x0, self.y0
        m = int(0.5 / self.cell)
        y_lo, y_hi = max(0, ys.min() - m), min(self.n, ys.max() + m + 1)
        x_lo, x_hi = max(0, xs.min() - m), min(self.n, xs.max() + m + 1)
        return c[y_lo:y_hi, x_lo:x_hi], self.x0 + x_lo * self.cell, self.y0 + y_lo * self.cell

    # ---- where to go next ------------------------------------------------------------------------
    def frontier_mask(self) -> np.ndarray:
        """Free cells next to unknown ones: the open edges of the map."""
        free = self.L < L_FREE
        unknown = (self.L >= L_FREE) & (self.L <= L_OCC)
        u = np.pad(unknown, 1)
        near_unknown = u[:-2, 1:-1] | u[2:, 1:-1] | u[1:-1, :-2] | u[1:-1, 2:]
        return free & near_unknown

    def next_station(self) -> dict:
        """The open edge most worth seeing past (nearest, then widest), and a clear spot near it to scan from.
        Recomputed only when the map changed (keyframe) or the scanner moved 0.2 m."""
        key = (self.keyframes, round(self.pose[0] / 0.2), round(self.pose[1] / 0.2))
        if key == self._next_key and self._next is not None:
            return self._next
        from .analysis import label_regions

        self._next_key = key
        fr = self.frontier_mask()
        self._frontier = fr
        labels, n = label_regions(fr, 8)
        px, py, th = self.pose
        best, best_score, edges = None, -1.0, 0
        if n and self._fine is not None:
            sizes = np.bincount(labels.ravel(), minlength=n + 1)
            free_clear = (self.L < L_FREE) & (self._fine >= STAND_CLEAR_M)
            fy_all, fx_all = np.nonzero(free_clear)
            fxw = self.x0 + (fx_all + 0.5) * self.cell
            fyw = self.y0 + (fy_all + 0.5) * self.cell
            for k in range(1, n + 1):
                width = sizes[k] * self.cell
                if width < MIN_FRONTIER_M:
                    continue
                ys, xs = np.nonzero(labels == k)
                cxw = self.x0 + (xs + 0.5) * self.cell
                cyw = self.y0 + (ys + 0.5) * self.cell
                d_me = np.hypot(cxw - px, cyw - py)
                far = d_me >= NEAR_M
                if not far.any():
                    continue
                edges += 1
                # a long edge (the whole rim of what one spot can see) is approached at its nearest
                # point, so each suggestion is a short step rather than its far-away middle
                j = int(np.argmin(np.where(far, d_me, np.inf)))
                ex, ey = float(cxw[j]), float(cyw[j])
                d_edge = np.hypot(fxw - ex, fyw - ey)
                ok = d_edge <= STAND_SEARCH_M
                if not ok.any():
                    continue
                i = int(np.argmin(np.where(ok, d_edge, np.inf)))
                sx, sy = float(fxw[i]), float(fyw[i])
                dist = math.hypot(sx - px, sy - py)
                score = min(width, 2.0) / (1.0 + dist) ** 2  # near beats wide
                if score > best_score:
                    best_score = score
                    rel = math.degrees(th - math.atan2(sy - py, sx - px)) % 360.0  # LiDAR convention: clockwise from its 0° mark
                    best = {
                        "x": round(sx, 3), "y": round(sy, 3),
                        "distance_m": round(dist, 2), "bearing_deg": round(rel, 1),
                        "edge": {"x": round(float(ex), 3), "y": round(float(ey), 3), "width_m": round(float(width), 2)},
                    }
        self._next = {"station": best, "open_edges": edges, "complete": edges == 0 and self.keyframes > 1}
        return self._next

    def area_m2(self) -> float:
        return float((self.L < L_FREE).sum()) * self.cell * self.cell

    def state(self) -> dict:
        path = self.path
        if len(path) > 500:
            path = path[:: math.ceil(len(path) / 500)] + [path[-1]]
        m = self.last
        return {
            "pose": {"x": round(self.pose[0], 3), "y": round(self.pose[1], 3), "theta_deg": round(math.degrees(self.pose[2]), 1)},
            "path": [[round(x, 3), round(y, 3)] for x, y in path],
            "keyframes": self.keyframes,
            "scans": self.scans,
            "area_m2": round(self.area_m2(), 2),
            "match": None if m is None else {
                "rms_mm": None if m.rms_m != m.rms_m else round(m.rms_m * 1000, 1),
                "inliers": round(m.inliers, 3),
                "ok": m.ok,
            },
            "lost": self.lost,
            "stations": self.stations,
            "next": self.next_station() if self.key_pose is not None else None,
        }
