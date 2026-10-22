"""Operator-level products derived from the height grid: floor, traversability, obstacles, voids,
occlusion shadows, the blind stripe along the tilt axle, and a route planner. Everything here is
deterministic numpy; the AI layer only consumes the findings dict this module returns.

Conventions: base frame, scanner at the origin (x forward = lidar 0° mark, z up). Bearings are
clockwise from +x in degrees, like the lidar's native angle, so an operator can relate them to
the scanner's cable mark.

Cell classes (one byte per cell in the layers frame):
  UNKNOWN   never sampled and no reason known (a sweep gap too wide to bridge, or past the last return)
  PASSABLE / MARGINAL / BLOCKED   from local step and slope on the (gap-bridged) surface
  SHADOW    occluded by a nearer tall object: scan from another position
  BLIND     the stripe along the tilt axle that no scan plane can reach: rotate the scanner 90°
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from .grid import Grid, build_grid

UNKNOWN, PASSABLE, MARGINAL, BLOCKED, SHADOW, BLIND = 0, 1, 2, 3, 4, 5
CLASS_NAMES = {UNKNOWN: "unknown", PASSABLE: "passable", MARGINAL: "marginal", BLOCKED: "blocked", SHADOW: "shadow", BLIND: "blind"}


@dataclass
class OpsParams:
    cell_m: float = 0.10
    max_step_m: float = 0.15  # step a person or rover can climb
    max_slope_deg: float = 25.0
    marginal_factor: float = 0.6  # amber above this fraction of a limit
    obstacle_min_h_m: float = 0.25  # above the floor
    void_clearance_m: float = 0.40  # open vertical span under cover that a person could fit in
    void_floor_tol_m: float = 0.15  # lowest return must be this close to the floor
    depression_m: float = 0.30  # below the floor
    min_region_cells: int = 4
    region_max_jump_m: float = 0.5  # neighbours differing by more than this are separate objects
    floor_band_m: float = 0.15
    sector_deg: float = 2.0  # bearing sectors for the occlusion analysis
    inpaint_cells: int = 3  # bridge sweep-line gaps up to this many cells wide
    inpaint_tol_m: float = 0.30  # only when the surrounding known cells agree within this
    snap_m: float = 1.0  # route start may snap this far to the nearest known cell
    snap_goal_m: float = 0.6  # route goal may snap this far (routing "to" an object means to its edge)
    wall_min_h_m: float = 1.8  # an obstacle this tall, or larger than wall_min_area_m2, is flagged as wall-like
    wall_min_area_m2: float = 1.5
    z_min_m: float = -1.0
    z_max_m: float = 3.0


@dataclass
class OpsResult:
    grid: Grid  # the max-height grid (defines w, h, x0, y0, cell)
    rel_max: np.ndarray  # height above the fitted floor, gaps bridged, NaN where not known
    rel_min: np.ndarray
    classes: np.ndarray  # uint8 (h, w)
    findings: dict = field(default_factory=dict)


# ---- floor ---------------------------------------------------------------------------------------
def fit_floor(xyz: np.ndarray, band_m: float) -> tuple[np.ndarray | None, dict]:
    """Plane z = a x + b y + c through the lowest band of points. None if too few points."""
    if xyz.shape[0] < 50:
        return None, {"points": int(xyz.shape[0]), "fitted": False}
    z = xyz[:, 2]
    z0 = float(np.percentile(z, 10))
    sel = xyz[np.abs(z - z0) <= band_m]
    if sel.shape[0] < 50:
        return None, {"points": int(sel.shape[0]), "fitted": False}
    a = np.c_[sel[:, 0], sel[:, 1], np.ones(sel.shape[0])].astype(np.float64)
    coef, *_ = np.linalg.lstsq(a, sel[:, 2].astype(np.float64), rcond=None)
    resid = sel[:, 2] - a @ coef
    return coef, {
        "points": int(sel.shape[0]),
        "fitted": True,
        "z_at_origin_m": round(float(coef[2]), 3),
        "tilt_deg": round(float(np.degrees(np.arctan(np.hypot(coef[0], coef[1])))), 2),
        "rms_mm": round(float(np.sqrt(np.mean(resid**2)) * 1000), 1),
    }


def _plane_on_grid(coef: np.ndarray | None, g: Grid) -> np.ndarray:
    if coef is None:
        return np.zeros((g.h, g.w), np.float32)
    xs = g.x0 + (np.arange(g.w) + 0.5) * g.cell
    ys = g.y0 + (np.arange(g.h) + 0.5) * g.cell
    return (coef[0] * xs[None, :] + coef[1] * ys[:, None] + coef[2]).astype(np.float32)


# ---- local geometry ------------------------------------------------------------------------------
_N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
_N4 = [(-1, 0), (0, -1), (0, 1), (1, 0)]


def _shifted(h: np.ndarray, dy: int, dx: int) -> np.ndarray:
    """h shifted by (dy, dx) with NaN padding."""
    out = np.full_like(h, np.nan)
    H, W = h.shape
    ys, yd = (slice(dy, H), slice(0, H - dy)) if dy >= 0 else (slice(0, H + dy), slice(-dy, H))
    xs, xd = (slice(dx, W), slice(0, W - dx)) if dx >= 0 else (slice(0, W + dx), slice(-dx, W))
    out[yd, xd] = h[ys, xs]
    return out


def inpaint(h: np.ndarray, iterations: int, allowed: np.ndarray | None = None, tol: float = 0.3) -> tuple[np.ndarray, np.ndarray]:
    """Bridge small gaps: an empty cell takes the mean of its known 8-neighbours when those neighbours
    agree within `tol` (so a gap between two floor lines closes, a gap between floor and wall does
    not). Repeated `iterations` times ≈ radius in cells. Returns (filled, mask of interpolated cells)."""
    out = h.astype(np.float32).copy()
    known0 = np.isfinite(out)
    for _ in range(iterations):
        nan = ~np.isfinite(out)
        if allowed is not None:
            nan &= allowed
        if not nan.any():
            break
        acc = np.zeros_like(out)
        cnt = np.zeros_like(out)
        lo = np.full_like(out, np.inf)
        hi = np.full_like(out, -np.inf)
        for dy, dx in _N8:
            s = _shifted(out, dy, dx)
            ok = np.isfinite(s)
            acc[ok] += s[ok]
            cnt[ok] += 1
            lo[ok] = np.minimum(lo[ok], s[ok])
            hi[ok] = np.maximum(hi[ok], s[ok])
        fill = nan & (cnt > 0) & ((hi - lo) <= tol)
        out[fill] = acc[fill] / cnt[fill]
    return out, np.isfinite(out) & ~known0


def slope_and_step(h: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Per cell: slope (deg) from the steepest 4-neighbour gradient, step (m) = largest 8-neighbour drop/rise."""
    diffs8, grads4 = [], []
    for dy, dx in _N8:
        d = np.abs(h - _shifted(h, dy, dx))
        diffs8.append(d)
        if dy == 0 or dx == 0:
            grads4.append(d / cell)
    with np.errstate(invalid="ignore", all="ignore"):
        import warnings

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            step = np.nanmax(np.stack(diffs8), axis=0)
            slope = np.degrees(np.arctan(np.nanmax(np.stack(grads4), axis=0)))
    return slope.astype(np.float32), step.astype(np.float32)


def classify(h: np.ndarray, cell: float, p: OpsParams) -> np.ndarray:
    slope, step = slope_and_step(h, cell)
    cls = np.full(h.shape, UNKNOWN, np.uint8)
    known = np.isfinite(h)
    cls[known] = PASSABLE
    with np.errstate(invalid="ignore"):
        marginal = known & ((step > p.max_step_m * p.marginal_factor) | (slope > p.max_slope_deg * p.marginal_factor))
        blocked = known & ((step > p.max_step_m) | (slope > p.max_slope_deg))
    cls[marginal] = MARGINAL
    cls[blocked] = BLOCKED
    return cls


# ---- regions -------------------------------------------------------------------------------------
def label_regions(mask: np.ndarray, connectivity: int = 8, values: np.ndarray | None = None, max_jump: float | None = None) -> tuple[np.ndarray, int]:
    """Connected components of a boolean mask. With `values` and `max_jump`, two neighbours join
    only if their values differ by at most max_jump, so a 0.9 m box touching a 3 m wall stays separate."""
    H, W = mask.shape
    labels = np.zeros((H, W), np.int32)
    n = 0
    nbrs = _N8 if connectivity == 8 else _N4
    ys, xs = np.nonzero(mask)
    for y, x in zip(ys.tolist(), xs.tolist()):
        if labels[y, x]:
            continue
        n += 1
        labels[y, x] = n
        stack = [(y, x)]
        while stack:
            cy, cx = stack.pop()
            for dy, dx in nbrs:
                ny, nx = cy + dy, cx + dx
                if not (0 <= ny < H and 0 <= nx < W) or not mask[ny, nx] or labels[ny, nx]:
                    continue
                if values is not None and max_jump is not None:
                    a, b = float(values[cy, cx]), float(values[ny, nx])
                    if np.isfinite(a) and np.isfinite(b) and abs(a - b) > max_jump:
                        continue
                labels[ny, nx] = n
                stack.append((ny, nx))
    return labels, n


def bearing_deg(x: float, y: float) -> float:
    return float(math.degrees(math.atan2(-y, x)) % 360.0)


def _regions(mask, g: Grid, value, min_cells: int, prefix: str, value_name: str, connectivity: int = 4, max_jump: float | None = None) -> list[dict]:
    labels, n = label_regions(mask, connectivity, value if max_jump is not None else None, max_jump)
    out = []
    for i in range(1, n + 1):
        cells = labels == i
        cnt = int(cells.sum())
        if cnt < min_cells:
            continue
        iy, ix = np.nonzero(cells)
        cx = g.x0 + (float(ix.mean()) + 0.5) * g.cell
        cy = g.y0 + (float(iy.mean()) + 0.5) * g.cell
        v = value[cells]
        v = v[np.isfinite(v)]
        out.append(
            {
                "id": "",
                "cells": cnt,
                "area_m2": round(cnt * g.cell * g.cell, 2),
                value_name: round(float(v.max()) if v.size else float("nan"), 2),
                "x_m": round(cx, 2),
                "y_m": round(cy, 2),
                "distance_m": round(math.hypot(cx, cy), 2),
                "bearing_deg": round(bearing_deg(cx, cy), 0),
                "bbox_m": [
                    round(g.x0 + float(ix.min()) * g.cell, 2),
                    round(g.y0 + float(iy.min()) * g.cell, 2),
                    round(g.x0 + (float(ix.max()) + 1) * g.cell, 2),
                    round(g.y0 + (float(iy.max()) + 1) * g.cell, 2),
                ],
            }
        )
    out.sort(key=lambda r: -r["area_m2"])
    for k, r in enumerate(out, 1):
        r["id"] = f"{prefix}{k}"
    return out


# ---- covered voids -------------------------------------------------------------------------------
def vertical_gap_grid(xyz: np.ndarray, g: Grid, plane: np.ndarray, floor_tol: float, min_gap: float, z_min: float, z_max: float) -> np.ndarray:
    """Per cell: the largest vertical gap between returns whose lower edge sits within floor_tol of
    the floor, if that gap is at least min_gap. NaN otherwise. A wall returns at every height and
    has no gap; a slab over open space returns at the top and on the floor with nothing between."""
    out = np.full(g.h * g.w, np.nan, np.float32)
    if xyz.shape[0] == 0:
        return out.reshape(g.h, g.w)
    p = xyz[(xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)]
    ix = np.floor((p[:, 0] - g.x0) / g.cell).astype(np.int64)
    iy = np.floor((p[:, 1] - g.y0) / g.cell).astype(np.int64)
    inside = (ix >= 0) & (ix < g.w) & (iy >= 0) & (iy < g.h)
    ix, iy, pz = ix[inside], iy[inside], p[inside, 2].astype(np.float64)
    if pz.size < 2:
        return out.reshape(g.h, g.w)
    rel = pz - plane[iy, ix]
    flat = iy * g.w + ix
    order = np.lexsort((rel, flat))
    fs, zs = flat[order], rel[order]
    same = fs[1:] == fs[:-1]
    gaps = zs[1:] - zs[:-1]
    cand = same & (gaps >= min_gap) & (zs[:-1] <= floor_tol)
    acc = np.zeros(g.h * g.w)
    np.maximum.at(acc, fs[1:][cand], gaps[cand])
    out[acc > 0] = acc[acc > 0]
    return out.reshape(g.h, g.w)


# ---- visibility ----------------------------------------------------------------------------------
def visibility(filled: np.ndarray, tall: np.ndarray, rel: np.ndarray, rng: np.ndarray, sec: np.ndarray, nsec: int, h_scan: float | None) -> tuple[np.ndarray, np.ndarray]:
    """Per bearing sector: `shadow` = unfilled cells behind a tall cell within the shadow it casts
    (length d·h/(h−h_obj) from a scanner h above the floor, infinite if the object is taller), and
    `beyond` = unfilled cells farther than the last return in their sector (never observed)."""
    unfilled = ~filled
    shadow = np.zeros(filled.shape, bool)
    maxr = np.zeros(nsec)
    np.maximum.at(maxr, sec[filled], rng[filled])
    beyond = unfilled & (rng >= maxr[sec])
    for s in range(nsec):
        in_s = sec == s
        t = tall & in_s
        if not t.any():
            continue
        cells = unfilled & in_s & ~beyond
        if not cells.any():
            continue
        d = rng[t]
        hb = rel[t]
        if h_scan is None:
            far = np.full_like(d, np.inf)
        else:
            far = np.where(hb < h_scan, d * h_scan / np.maximum(h_scan - hb, 1e-6), np.inf)
        r = rng[cells]
        occ = ((r[:, None] > d[None, :]) & (r[:, None] < far[None, :])).any(axis=1)
        iy, ix = np.nonzero(cells)
        shadow[iy[occ], ix[occ]] = True
    return shadow, beyond


# ---- route ---------------------------------------------------------------------------------------
def plan_route(cls: np.ndarray, g: Grid, start_xy, goal_xy, rel: np.ndarray | None = None, snap_m: float = 1.0, snap_goal_m: float = 0.0) -> dict | None:
    """A* over the class grid, 8-connected. Marginal cells cost 3x, everything else impassable.
    The start snaps to the nearest passable cell within snap_m (the scanner's own footprint is never
    scanned); the goal snaps within snap_goal_m (routing to an object means to its edge)."""

    def to_cell(x, y):
        return int(math.floor((x - g.x0) / g.cell)), int(math.floor((y - g.y0) / g.cell))

    H, W = cls.shape
    passable = (cls == PASSABLE) | (cls == MARGINAL)
    ys_all, xs_all = np.nonzero(passable)
    if xs_all.size == 0:
        return None
    cxs = (xs_all + 0.5) * g.cell + g.x0
    cys = (ys_all + 0.5) * g.cell + g.y0

    def snap(xy, limit):
        x, y = to_cell(*xy)
        if 0 <= x < W and 0 <= y < H and passable[y, x]:
            return x, y, 0.0
        if limit <= 0:
            return None
        d = np.hypot(cxs - xy[0], cys - xy[1])
        k = int(np.argmin(d))
        if d[k] > limit:
            return None
        return int(xs_all[k]), int(ys_all[k]), float(d[k])

    s = snap(start_xy, snap_m)
    if s is None:
        return None
    sx, sy, snapped = s

    # everything reachable from the start with the planner's own moves; the goal must lie in it
    # (a flat object top is "passable" but ringed by blocked cells: an island, never a destination)
    reach = np.zeros((H, W), bool)
    reach[sy, sx] = True
    stack = [(sx, sy)]
    while stack:
        cx, cy = stack.pop()
        for dy, dx in _N8:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < W and 0 <= ny < H) or not passable[ny, nx] or reach[ny, nx]:
                continue
            if dx and dy and not (passable[cy, nx] and passable[ny, cx]):
                continue
            reach[ny, nx] = True
            stack.append((nx, ny))
    ys_r, xs_r = np.nonzero(reach)
    rxs = (xs_r + 0.5) * g.cell + g.x0
    rys = (ys_r + 0.5) * g.cell + g.y0
    gx, gy = to_cell(*goal_xy)
    goal_snapped = 0.0
    if not (0 <= gx < W and 0 <= gy < H and reach[gy, gx]):
        if snap_goal_m <= 0 or xs_r.size == 0:
            return None
        d = np.hypot(rxs - goal_xy[0], rys - goal_xy[1])
        k = int(np.argmin(d))
        if d[k] > snap_goal_m:
            return None
        gx, gy, goal_snapped = int(xs_r[k]), int(ys_r[k]), float(d[k])
    cost = np.where(cls == MARGINAL, 3.0, 1.0)
    open_heap = [(0.0, 0.0, (sx, sy))]
    came: dict[tuple[int, int], tuple[int, int] | None] = {(sx, sy): None}
    best = {(sx, sy): 0.0}
    found = False
    while open_heap:
        _, gcost, (cx, cy) = heapq.heappop(open_heap)
        if (cx, cy) == (gx, gy):
            found = True
            break
        if gcost > best.get((cx, cy), math.inf):
            continue
        for dy, dx in _N8:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < W and 0 <= ny < H) or not passable[ny, nx]:
                continue
            if dx and dy and not (passable[cy, nx] and passable[ny, cx]):
                continue  # no corner cutting past blocked cells
            ng = gcost + math.hypot(dx, dy) * cost[ny, nx]
            if ng < best.get((nx, ny), math.inf):
                best[(nx, ny)] = ng
                came[(nx, ny)] = (cx, cy)
                heapq.heappush(open_heap, (ng + math.hypot(gx - nx, gy - ny), ng, (nx, ny)))
    if not found:
        return None
    path = []
    node: tuple[int, int] | None = (gx, gy)
    while node is not None:
        path.append(node)
        node = came[node]
    path.reverse()
    pts = [[round(g.x0 + (x + 0.5) * g.cell, 3), round(g.y0 + (y + 0.5) * g.cell, 3)] for x, y in path]
    length = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) for i in range(1, len(pts)))
    n_marginal = sum(1 for x, y in path if cls[y, x] == MARGINAL)
    heights = [float(rel[y, x]) for x, y in path if rel is not None and np.isfinite(rel[y, x])] if rel is not None else []
    max_step = max((abs(heights[i] - heights[i - 1]) for i in range(1, len(heights))), default=0.0)
    return {
        "found": True,
        "start_snapped_m": round(snapped, 2),
        "goal_snapped_m": round(goal_snapped, 2),
        "length_m": round(length, 2),
        "cells": len(path),
        "marginal_cells": n_marginal,
        "verdict": "passable" if n_marginal == 0 else "marginal",
        "max_step_m": round(max_step, 3),
        "path": pts,
    }


# ---- the whole thing -----------------------------------------------------------------------------
def analyze(xyz: np.ndarray, p: OpsParams | None = None, scanner_z_m: float | None = None, phi_max_deg: float = 60.0) -> OpsResult:
    """scanner_z_m: height of the scan-plane centre in the base frame (axis_z + h). With it, shadow
    lengths are geometric and the stripe along the tilt axle (|y| < height / tan φmax) is classed BLIND."""
    p = p or OpsParams()
    xyz = np.asarray(xyz, np.float32).reshape(-1, 3)
    gmax = build_grid(xyz, p.cell_m, p.z_min_m, p.z_max_m, "max")
    bounds = (gmax.x0, gmax.y0, gmax.x0 + gmax.w * gmax.cell, gmax.y0 + gmax.h * gmax.cell)
    gmin = build_grid(xyz, gmax.cell, p.z_min_m, p.z_max_m, "min", bounds=bounds)
    coef, floor = fit_floor(xyz, p.floor_band_m)
    plane = _plane_on_grid(coef, gmax)
    rel_max = (gmax.data - plane).astype(np.float32)
    rel_min = (gmin.data - plane).astype(np.float32)
    filled = np.isfinite(rel_max)

    xs = gmax.x0 + (np.arange(gmax.w) + 0.5) * gmax.cell
    ys = gmax.y0 + (np.arange(gmax.h) + 0.5) * gmax.cell
    X, Y = np.meshgrid(xs, ys)
    rng = np.hypot(X, Y)
    nsec = int(round(360.0 / p.sector_deg))
    sec = np.minimum(((np.degrees(np.arctan2(-Y, X)) % 360.0) / p.sector_deg).astype(np.int64), nsec - 1)

    h_scan = None
    blind_half = None
    if scanner_z_m is not None and coef is not None:
        h_scan = max(0.0, scanner_z_m - float(coef[2]))
        if 0 < phi_max_deg < 90:
            blind_half = h_scan / math.tan(math.radians(phi_max_deg))

    with np.errstate(invalid="ignore"):
        tall = filled & (rel_max >= p.obstacle_min_h_m)
    shadow, beyond = visibility(filled, tall, rel_max, rng, sec, nsec, h_scan)
    blind = np.zeros_like(filled)
    if blind_half is not None:
        blind = (~filled) & (np.abs(Y) < blind_half) & (np.abs(X) > gmax.cell) & ~beyond
    shadow &= ~blind

    # bridge only genuine sweep gaps: not occluded, not past the last return, not in the blind stripe
    surface, interpolated = inpaint(rel_max, p.inpaint_cells, allowed=~(shadow | beyond | blind), tol=p.inpaint_tol_m)
    cls = classify(surface, gmax.cell, p)
    cls[blind & (cls == UNKNOWN)] = BLIND
    cls[shadow & (cls == UNKNOWN)] = SHADOW

    gap = vertical_gap_grid(xyz, gmax, plane, p.void_floor_tol_m, p.void_clearance_m, p.z_min_m, p.z_max_m)
    with np.errstate(invalid="ignore"):
        obstacle_mask = tall
        void_mask = np.isfinite(gap)
        depression_mask = filled & (rel_min <= -p.depression_m)
    obstacles = _regions(obstacle_mask, gmax, rel_max, p.min_region_cells, "O", "height_m", max_jump=p.region_max_jump_m)
    for o in obstacles:
        o["wall"] = bool(o["height_m"] >= p.wall_min_h_m or o["area_m2"] >= p.wall_min_area_m2)
    voids = _regions(void_mask, gmax, gap, p.min_region_cells, "V", "clearance_m", max_jump=p.region_max_jump_m)
    depressions = _regions(depression_mask, gmax, -rel_min, p.min_region_cells, "D", "depth_m", max_jump=p.region_max_jump_m)
    shadow_regions = _regions(shadow, gmax, np.zeros_like(rel_max), max(p.min_region_cells, 6), "S", "_", connectivity=8)
    for s in shadow_regions:
        s.pop("_", None)

    # how far out the floor itself was sampled, ignoring the bases of walls and objects
    near_tall = tall.copy()
    for dy, dx in _N8:
        near_tall |= _shifted(tall.astype(np.float32), dy, dx) == 1.0
    with np.errstate(invalid="ignore"):
        floor_cells = filled & (np.abs(rel_max) < p.floor_band_m) & ~near_tall
    floor_reach = float(rng[floor_cells].max()) if floor_cells.any() else 0.0

    n_filled, n_interp = int(filled.sum()), int(interpolated.sum())
    n_shadow, n_blind, n_beyond = int(shadow.sum()), int(blind.sum()), int(beyond.sum())
    n_unknown = int(((cls == UNKNOWN) & ~beyond).sum())
    observable = gmax.w * gmax.h - n_beyond
    counts = {CLASS_NAMES[k]: int((cls == k).sum()) for k in (PASSABLE, MARGINAL, BLOCKED, SHADOW, BLIND)}
    next_station = None
    if shadow_regions:
        s = shadow_regions[0]
        next_station = {
            "bearing_deg": s["bearing_deg"],
            "distance_m": round(max(0.5, s["distance_m"] * 0.5), 1),
            "reason": f"largest occluded region {s['id']}: {s['area_m2']} m² at {s['distance_m']} m",
        }
    elif blind_half and n_blind > 20:
        next_station = {"bearing_deg": None, "distance_m": 0.0, "reason": "rotate the scanner 90° in place to cover the blind stripe"}
    findings = {
        "scanner": {"x_m": 0.0, "y_m": 0.0, "height_above_floor_m": None if h_scan is None else round(h_scan, 3), "note": "base-frame origin, bearings clockwise from +x (lidar 0° mark)"},
        "points": int(xyz.shape[0]),
        "grid": {"cell_m": gmax.cell, "w": gmax.w, "h": gmax.h, "x0_m": gmax.x0, "y0_m": gmax.y0},
        "floor": floor,
        "coverage": {
            "filled_cells": n_filled,
            "interpolated_cells": n_interp,
            "shadow_cells": n_shadow,
            "blind_cells": n_blind,
            "unknown_cells": n_unknown,
            "observable_cells": int(observable),
            "area_seen_m2": round((n_filled + n_interp) * gmax.cell**2, 2),
            "coverage_pct": round(100.0 * (n_filled + n_interp) / observable, 1) if observable else 0.0,
            "floor_reach_m": round(floor_reach, 2),
            "blind_stripe": None
            if blind_half is None
            else {"half_width_m": round(blind_half, 2), "cells": n_blind, "note": f"floor within {blind_half:.2f} m of the tilt axle is unreachable at ±{phi_max_deg:.0f}°"},
        },
        "traversability": {**counts, "params": {"max_step_m": p.max_step_m, "max_slope_deg": p.max_slope_deg}},
        "obstacles": obstacles[:20],
        "voids": voids[:20],
        "depressions": depressions[:20],
        "shadows": shadow_regions[:10],
        "next_station": next_station,
    }
    return OpsResult(gmax, surface, rel_min, cls, findings)
