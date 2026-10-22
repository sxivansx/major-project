// Operator products for the mock: a TypeScript port of core/obslam/analysis.py
// (floor fit, traversability classes, obstacles, covered voids, occlusion
// shadows, the blind stripe along the tilt axle, A* route) plus a canned
// assistant layer that only ever reads the findings. Same conventions as the
// core: base frame, scanner at the origin, bearings clockwise from +x.

import type { AiAsk, AiSitrep, AiTriage, AiTriageItem, Finding, Findings, RouteResult } from "../src/api/types";
import { computeGrid } from "./world";

export const UNKNOWN = 0;
export const PASSABLE = 1;
export const MARGINAL = 2;
export const BLOCKED = 3;
export const SHADOW = 4;
export const BLIND = 5;

export interface OpsParams {
  cell: number;
  maxStep: number;
  maxSlope: number;
}
export const OPS_DEFAULTS: OpsParams = { cell: 0.1, maxStep: 0.15, maxSlope: 25 };

const P = {
  marginalFactor: 0.6,
  obstacleMinH: 0.25,
  voidClearance: 0.4,
  voidFloorTol: 0.15,
  depression: 0.3,
  minRegionCells: 4,
  regionMaxJump: 0.5,
  floorBand: 0.15,
  sectorDeg: 2,
  inpaintCells: 3,
  inpaintTol: 0.3,
  snap: 1.0,
  zMin: -1,
  zMax: 3,
};

export interface OpsResult {
  w: number;
  h: number;
  cell: number;
  x0: number;
  y0: number;
  classes: Uint8Array;
  rel: Float32Array; // height above the floor, gaps bridged, NaN unknown
  findings: Findings;
}

const N8: [number, number][] = [
  [-1, -1],
  [-1, 0],
  [-1, 1],
  [0, -1],
  [0, 1],
  [1, -1],
  [1, 0],
  [1, 1],
];
const N4: [number, number][] = [
  [-1, 0],
  [0, -1],
  [0, 1],
  [1, 0],
];
const DEG = Math.PI / 180;
const r = (v: number, d: number) => Math.round(v * 10 ** d) / 10 ** d;
export const bearingDeg = (x: number, y: number) => ((Math.atan2(-y, x) / DEG) % 360 + 360) % 360;

// ---- floor ----

function fitFloor(xyz: Float32Array, n: number): { coef: [number, number, number] | null; floor: Findings["floor"] } {
  if (n < 50) return { coef: null, floor: { fitted: false, points: n } };
  const zs = new Float32Array(n);
  for (let i = 0; i < n; i++) zs[i] = xyz[i * 3 + 2];
  zs.sort();
  const z0 = zs[Math.floor(0.1 * (n - 1))];
  let sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, s1 = 0, sxz = 0, syz = 0, sz = 0;
  for (let i = 0; i < n; i++) {
    const z = xyz[i * 3 + 2];
    if (Math.abs(z - z0) > P.floorBand) continue;
    const x = xyz[i * 3];
    const y = xyz[i * 3 + 1];
    sxx += x * x; sxy += x * y; sx += x; syy += y * y; sy += y; s1++; sxz += x * z; syz += y * z; sz += z;
  }
  if (s1 < 50) return { coef: null, floor: { fitted: false, points: s1 } };
  // solve [sxx sxy sx; sxy syy sy; sx sy s1] [a b c] = [sxz syz sz]
  const det = sxx * (syy * s1 - sy * sy) - sxy * (sxy * s1 - sy * sx) + sx * (sxy * sy - syy * sx);
  if (Math.abs(det) < 1e-12) return { coef: null, floor: { fitted: false, points: s1 } };
  const a = (sxz * (syy * s1 - sy * sy) - sxy * (syz * s1 - sy * sz) + sx * (syz * sy - syy * sz)) / det;
  const b = (sxx * (syz * s1 - sy * sz) - sxz * (sxy * s1 - sy * sx) + sx * (sxy * sz - syz * sx)) / det;
  const c = (sxx * (syy * sz - syz * sy) - sxy * (sxy * sz - syz * sx) + sxz * (sxy * sy - syy * sx)) / det;
  let ss = 0;
  for (let i = 0; i < n; i++) {
    const z = xyz[i * 3 + 2];
    if (Math.abs(z - z0) > P.floorBand) continue;
    const d = z - (a * xyz[i * 3] + b * xyz[i * 3 + 1] + c);
    ss += d * d;
  }
  return {
    coef: [a, b, c],
    floor: {
      fitted: true,
      points: s1,
      z_at_origin_m: r(c, 3),
      tilt_deg: r(Math.atan(Math.hypot(a, b)) / DEG, 2),
      rms_mm: r(Math.sqrt(ss / s1) * 1000, 1),
    },
  };
}

// ---- regions ----

function labelRegions(mask: Uint8Array, w: number, h: number, conn: 4 | 8, values: Float32Array | null, maxJump: number | null) {
  const labels = new Int32Array(w * h);
  const nb = conn === 8 ? N8 : N4;
  let n = 0;
  const stack: number[] = [];
  for (let s = 0; s < w * h; s++) {
    if (!mask[s] || labels[s]) continue;
    n++;
    labels[s] = n;
    stack.push(s);
    while (stack.length) {
      const k = stack.pop()!;
      const ky = Math.floor(k / w);
      const kx = k - ky * w;
      for (const [dy, dx] of nb) {
        const ny = ky + dy;
        const nx = kx + dx;
        if (ny < 0 || nx < 0 || ny >= h || nx >= w) continue;
        const q = ny * w + nx;
        if (!mask[q] || labels[q]) continue;
        if (values && maxJump !== null) {
          const a = values[k];
          const b = values[q];
          if (Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) > maxJump) continue;
        }
        labels[q] = n;
        stack.push(q);
      }
    }
  }
  return { labels, n };
}

function regions(
  mask: Uint8Array,
  g: { w: number; h: number; cell: number; x0: number; y0: number },
  values: Float32Array,
  minCells: number,
  prefix: string,
  valueName: "height_m" | "clearance_m" | "depth_m" | null,
  conn: 4 | 8,
  maxJump: number | null,
): Finding[] {
  const { labels, n } = labelRegions(mask, g.w, g.h, conn, maxJump !== null ? values : null, maxJump);
  const out: Finding[] = [];
  for (let i = 1; i <= n; i++) {
    let cnt = 0, sx = 0, sy = 0, xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity, vmax = -Infinity;
    for (let k = 0; k < g.w * g.h; k++) {
      if (labels[k] !== i) continue;
      const iy = Math.floor(k / g.w);
      const ix = k - iy * g.w;
      cnt++;
      sx += ix;
      sy += iy;
      if (ix < xmin) xmin = ix;
      if (ix > xmax) xmax = ix;
      if (iy < ymin) ymin = iy;
      if (iy > ymax) ymax = iy;
      const v = values[k];
      if (Number.isFinite(v) && v > vmax) vmax = v;
    }
    if (cnt < minCells) continue;
    const cx = g.x0 + (sx / cnt + 0.5) * g.cell;
    const cy = g.y0 + (sy / cnt + 0.5) * g.cell;
    const f: Finding = {
      id: "",
      cells: cnt,
      area_m2: r(cnt * g.cell * g.cell, 2),
      x_m: r(cx, 2),
      y_m: r(cy, 2),
      distance_m: r(Math.hypot(cx, cy), 2),
      bearing_deg: r(bearingDeg(cx, cy), 0),
      bbox_m: [r(g.x0 + xmin * g.cell, 2), r(g.y0 + ymin * g.cell, 2), r(g.x0 + (xmax + 1) * g.cell, 2), r(g.y0 + (ymax + 1) * g.cell, 2)],
    };
    if (valueName) f[valueName] = r(Number.isFinite(vmax) ? vmax : NaN, 2);
    out.push(f);
  }
  out.sort((a, b) => b.area_m2 - a.area_m2);
  out.forEach((f, k) => (f.id = `${prefix}${k + 1}`));
  return out;
}

// ---- the whole thing ----

function emptyFindings(points: number, p: OpsParams): Findings {
  return {
    scanner: { x_m: 0, y_m: 0, height_above_floor_m: null, note: "base-frame origin, bearings clockwise from +x (lidar 0° mark)" },
    points,
    grid: { cell_m: p.cell, w: 0, h: 0, x0_m: 0, y0_m: 0 },
    floor: { fitted: false, points: 0 },
    coverage: {
      filled_cells: 0, interpolated_cells: 0, shadow_cells: 0, blind_cells: 0, unknown_cells: 0, observable_cells: 0,
      area_seen_m2: 0, coverage_pct: 0, floor_reach_m: 0, blind_stripe: null,
    },
    traversability: { passable: 0, marginal: 0, blocked: 0, shadow: 0, blind: 0, params: { max_step_m: p.maxStep, max_slope_deg: p.maxSlope } },
    obstacles: [], voids: [], depressions: [], shadows: [],
    next_station: null,
  };
}

export function analyze(xyz: Float32Array, n: number, p: OpsParams, scannerZ: number, phiMaxDeg: number): OpsResult {
  const gmax = computeGrid(xyz, n, p.cell, "max", P.zMin, P.zMax);
  const gmin = computeGrid(xyz, n, p.cell, "min", P.zMin, P.zMax);
  const { w, h } = gmax;
  const cell = gmax.cell_m;
  const x0 = gmax.x0_m;
  const y0 = gmax.y0_m;
  const N = w * h;
  if (N === 0) {
    return { w: 0, h: 0, cell, x0: 0, y0: 0, classes: new Uint8Array(0), rel: new Float32Array(0), findings: emptyFindings(n, p) };
  }
  const g = { w, h, cell, x0, y0 };
  const { coef, floor } = fitFloor(xyz, n);

  const relMax = new Float32Array(N);
  const relMin = new Float32Array(N);
  const X = new Float32Array(N);
  const Y = new Float32Array(N);
  const rng = new Float32Array(N);
  const sec = new Int32Array(N);
  const nsec = Math.round(360 / P.sectorDeg);
  const filled = new Uint8Array(N);
  for (let k = 0; k < N; k++) {
    const iy = Math.floor(k / w);
    const ix = k - iy * w;
    const xc = x0 + (ix + 0.5) * cell;
    const yc = y0 + (iy + 0.5) * cell;
    const plane = coef ? coef[0] * xc + coef[1] * yc + coef[2] : 0;
    relMax[k] = gmax.data[k] - plane;
    relMin[k] = gmin.data[k] - plane;
    X[k] = xc;
    Y[k] = yc;
    rng[k] = Math.hypot(xc, yc);
    sec[k] = Math.min(nsec - 1, Math.floor(bearingDeg(xc, yc) / P.sectorDeg));
    filled[k] = Number.isFinite(relMax[k]) ? 1 : 0;
  }

  let hScan: number | null = null;
  let blindHalf: number | null = null;
  if (coef) {
    hScan = Math.max(0, scannerZ - coef[2]);
    if (phiMaxDeg > 0 && phiMaxDeg < 90) blindHalf = hScan / Math.tan(phiMaxDeg * DEG);
  }

  const tall = new Uint8Array(N);
  for (let k = 0; k < N; k++) tall[k] = filled[k] && relMax[k] >= P.obstacleMinH ? 1 : 0;

  // visibility: shadow behind tall cells per bearing sector, beyond the last return never seen
  const maxr = new Float64Array(nsec);
  for (let k = 0; k < N; k++) if (filled[k] && rng[k] > maxr[sec[k]]) maxr[sec[k]] = rng[k];
  const beyond = new Uint8Array(N);
  for (let k = 0; k < N; k++) beyond[k] = !filled[k] && rng[k] >= maxr[sec[k]] ? 1 : 0;
  const tallBySec: number[][] = Array.from({ length: nsec }, () => []);
  for (let k = 0; k < N; k++) if (tall[k]) tallBySec[sec[k]].push(k);
  const shadow = new Uint8Array(N);
  for (let k = 0; k < N; k++) {
    if (filled[k] || beyond[k]) continue;
    for (const t of tallBySec[sec[k]]) {
      const d = rng[t];
      const hb = relMax[t];
      const far = hScan === null ? Infinity : hb < hScan ? (d * hScan) / Math.max(hScan - hb, 1e-6) : Infinity;
      if (rng[k] > d && rng[k] < far) {
        shadow[k] = 1;
        break;
      }
    }
  }
  const blind = new Uint8Array(N);
  if (blindHalf !== null) {
    for (let k = 0; k < N; k++) {
      blind[k] = !filled[k] && Math.abs(Y[k]) < blindHalf && Math.abs(X[k]) > cell && !beyond[k] ? 1 : 0;
      if (blind[k]) shadow[k] = 0;
    }
  }

  // bridge small sweep gaps where the neighbours agree
  const surface = relMax.slice();
  const known0 = filled;
  for (let it = 0; it < P.inpaintCells; it++) {
    const fills: [number, number][] = [];
    for (let k = 0; k < N; k++) {
      if (Number.isFinite(surface[k]) || shadow[k] || beyond[k] || blind[k]) continue;
      const iy = Math.floor(k / w);
      const ix = k - iy * w;
      let cnt = 0, acc = 0, lo = Infinity, hi = -Infinity;
      for (const [dy, dx] of N8) {
        const ny = iy + dy;
        const nx = ix + dx;
        if (ny < 0 || nx < 0 || ny >= h || nx >= w) continue;
        const v = surface[ny * w + nx];
        if (!Number.isFinite(v)) continue;
        cnt++;
        acc += v;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
      if (cnt > 0 && hi - lo <= P.inpaintTol) fills.push([k, acc / cnt]);
    }
    if (!fills.length) break;
    for (const [k, v] of fills) surface[k] = v;
  }
  const interpolated = new Uint8Array(N);
  for (let k = 0; k < N; k++) interpolated[k] = Number.isFinite(surface[k]) && !known0[k] ? 1 : 0;

  // classes from local step (8-neighbour) and slope (4-neighbour)
  const cls = new Uint8Array(N);
  for (let k = 0; k < N; k++) {
    const v = surface[k];
    if (!Number.isFinite(v)) continue;
    const iy = Math.floor(k / w);
    const ix = k - iy * w;
    let step = -Infinity;
    let grad = -Infinity;
    for (const [dy, dx] of N8) {
      const ny = iy + dy;
      const nx = ix + dx;
      if (ny < 0 || nx < 0 || ny >= h || nx >= w) continue;
      const u = surface[ny * w + nx];
      if (!Number.isFinite(u)) continue;
      const d = Math.abs(v - u);
      if (d > step) step = d;
      if ((dy === 0 || dx === 0) && d / cell > grad) grad = d / cell;
    }
    const slope = grad > -Infinity ? Math.atan(grad) / DEG : NaN;
    let c = PASSABLE;
    if (step > p.maxStep * P.marginalFactor || slope > p.maxSlope * P.marginalFactor) c = MARGINAL;
    if (step > p.maxStep || slope > p.maxSlope) c = BLOCKED;
    cls[k] = c;
  }
  for (let k = 0; k < N; k++) {
    if (cls[k] === UNKNOWN && blind[k]) cls[k] = BLIND;
    else if (cls[k] === UNKNOWN && shadow[k]) cls[k] = SHADOW;
  }

  // covered voids: a vertical gap between returns starting near the floor
  const gap = new Float32Array(N).fill(NaN);
  {
    const counts = new Uint32Array(N + 1);
    const flat = new Int32Array(n);
    const relz = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const z = xyz[i * 3 + 2];
      flat[i] = -1;
      if (z < P.zMin || z > P.zMax) continue;
      const ix = Math.floor((xyz[i * 3] - x0) / cell);
      const iy = Math.floor((xyz[i * 3 + 1] - y0) / cell);
      if (ix < 0 || iy < 0 || ix >= w || iy >= h) continue;
      const k = iy * w + ix;
      flat[i] = k;
      relz[i] = z - (coef ? coef[0] * X[k] + coef[1] * Y[k] + coef[2] : 0);
      counts[k + 1]++;
    }
    for (let k = 0; k < N; k++) counts[k + 1] += counts[k];
    const fill = counts.slice(0, N);
    const sorted = new Float32Array(counts[N]);
    for (let i = 0; i < n; i++) if (flat[i] >= 0) sorted[fill[flat[i]]++] = relz[i];
    for (let k = 0; k < N; k++) {
      const a = counts[k];
      const b = counts[k + 1];
      if (b - a < 2) continue;
      const zs = sorted.subarray(a, b).sort();
      let acc = 0;
      for (let i = 0; i + 1 < zs.length; i++) {
        const gv = zs[i + 1] - zs[i];
        if (gv >= P.voidClearance && zs[i] <= P.voidFloorTol && gv > acc) acc = gv;
      }
      if (acc > 0) gap[k] = acc;
    }
  }

  const voidMask = new Uint8Array(N);
  const depMask = new Uint8Array(N);
  const negRelMin = new Float32Array(N);
  for (let k = 0; k < N; k++) {
    voidMask[k] = Number.isFinite(gap[k]) ? 1 : 0;
    depMask[k] = filled[k] && relMin[k] <= -P.depression ? 1 : 0;
    negRelMin[k] = -relMin[k];
  }
  const obstacles = regions(tall, g, relMax, P.minRegionCells, "O", "height_m", 4, P.regionMaxJump);
  const voids = regions(voidMask, g, gap, P.minRegionCells, "V", "clearance_m", 4, P.regionMaxJump);
  const depressions = regions(depMask, g, negRelMin, P.minRegionCells, "D", "depth_m", 4, P.regionMaxJump);
  const shadows = regions(shadow, g, new Float32Array(N), Math.max(P.minRegionCells, 6), "S", null, 8, null);

  // how far out the floor itself was sampled
  let floorReach = 0;
  for (let k = 0; k < N; k++) {
    if (!filled[k] || Math.abs(relMax[k]) >= P.floorBand) continue;
    const iy = Math.floor(k / w);
    const ix = k - iy * w;
    let near = tall[k] === 1;
    for (const [dy, dx] of N8) {
      const ny = iy + dy;
      const nx = ix + dx;
      if (ny >= 0 && nx >= 0 && ny < h && nx < w && tall[ny * w + nx]) near = true;
    }
    if (!near && rng[k] > floorReach) floorReach = rng[k];
  }

  let nFilled = 0, nInterp = 0, nShadow = 0, nBlind = 0, nBeyond = 0, nUnknown = 0;
  const counts = [0, 0, 0, 0, 0, 0];
  for (let k = 0; k < N; k++) {
    nFilled += filled[k];
    nInterp += interpolated[k];
    nShadow += shadow[k];
    nBlind += blind[k];
    nBeyond += beyond[k];
    if (cls[k] === UNKNOWN && !beyond[k]) nUnknown++;
    counts[cls[k]]++;
  }
  const observable = N - nBeyond;

  let next_station: Findings["next_station"] = null;
  if (shadows.length) {
    const s = shadows[0];
    next_station = {
      bearing_deg: s.bearing_deg,
      distance_m: r(Math.max(0.5, s.distance_m * 0.5), 1),
      reason: `largest occluded region ${s.id}: ${s.area_m2} m² at ${s.distance_m} m`,
    };
  } else if (blindHalf && nBlind > 20) {
    next_station = { bearing_deg: null, distance_m: 0, reason: "rotate the scanner 90° in place to cover the blind stripe" };
  }

  const findings: Findings = {
    scanner: {
      x_m: 0,
      y_m: 0,
      height_above_floor_m: hScan === null ? null : r(hScan, 3),
      note: "base-frame origin, bearings clockwise from +x (lidar 0° mark)",
    },
    points: n,
    grid: { cell_m: cell, w, h, x0_m: x0, y0_m: y0 },
    floor,
    coverage: {
      filled_cells: nFilled,
      interpolated_cells: nInterp,
      shadow_cells: nShadow,
      blind_cells: nBlind,
      unknown_cells: nUnknown,
      observable_cells: observable,
      area_seen_m2: r((nFilled + nInterp) * cell * cell, 2),
      coverage_pct: observable ? r((100 * (nFilled + nInterp)) / observable, 1) : 0,
      floor_reach_m: r(floorReach, 2),
      blind_stripe:
        blindHalf === null
          ? null
          : { half_width_m: r(blindHalf, 2), cells: nBlind, note: `floor within ${blindHalf.toFixed(2)} m of the tilt axle is unreachable at ±${phiMaxDeg.toFixed(0)}°` },
    },
    traversability: {
      passable: counts[PASSABLE],
      marginal: counts[MARGINAL],
      blocked: counts[BLOCKED],
      shadow: counts[SHADOW],
      blind: counts[BLIND],
      params: { max_step_m: p.maxStep, max_slope_deg: p.maxSlope },
    },
    obstacles: obstacles.slice(0, 20),
    voids: voids.slice(0, 20),
    depressions: depressions.slice(0, 20),
    shadows: shadows.slice(0, 10),
    next_station,
  };
  return { w, h, cell, x0, y0, classes: cls, rel: surface, findings };
}

// ---- route: A* over the class grid, 8-connected, marginal costs 3x ----

export function planRoute(res: OpsResult, start: [number, number], goal: [number, number]): RouteResult {
  const { w, h, cell, x0, y0, classes: cls, rel } = res;
  const fail = (reason: string): RouteResult => ({ found: false, reason });
  if (w === 0 || h === 0) return fail("no analysed grid: scan or load a session first");
  const toCell = (x: number, y: number) => [Math.floor((x - x0) / cell), Math.floor((y - y0) / cell)];
  let [sx, sy] = toCell(start[0], start[1]);
  const [gx, gy] = toCell(goal[0], goal[1]);
  if (!(gx >= 0 && gx < w && gy >= 0 && gy < h)) return fail("goal is outside the analysed grid");
  const passable = (x: number, y: number) => x >= 0 && y >= 0 && x < w && y < h && (cls[y * w + x] === PASSABLE || cls[y * w + x] === MARGINAL);
  if (!passable(gx, gy)) return fail(`goal cell is ${["unknown", "passable", "marginal", "blocked", "shadow", "blind"][cls[gy * w + gx]]}, not passable`);
  let snapped = 0;
  if (!passable(sx, sy)) {
    let best = Infinity;
    let bx = -1, by = -1;
    for (let k = 0; k < w * h; k++) {
      if (cls[k] !== PASSABLE && cls[k] !== MARGINAL) continue;
      const iy = Math.floor(k / w);
      const ix = k - iy * w;
      const d = Math.hypot(x0 + (ix + 0.5) * cell - start[0], y0 + (iy + 0.5) * cell - start[1]);
      if (d < best) {
        best = d;
        bx = ix;
        by = iy;
      }
    }
    if (bx < 0 || best > P.snap) return fail(`no passable cell within ${P.snap} m of the start`);
    sx = bx;
    sy = by;
    snapped = best;
  }
  const key = (x: number, y: number) => y * w + x;
  const best = new Float64Array(w * h).fill(Infinity);
  const came = new Int32Array(w * h).fill(-1);
  const open: [number, number, number][] = [[0, 0, key(sx, sy)]]; // f, g, cell
  best[key(sx, sy)] = 0;
  let found = false;
  const pop = () => {
    let bi = 0;
    for (let i = 1; i < open.length; i++) if (open[i][0] < open[bi][0]) bi = i;
    const v = open[bi];
    open[bi] = open[open.length - 1];
    open.pop();
    return v;
  };
  while (open.length) {
    const [, gc, k] = pop();
    if (gc > best[k]) continue;
    const cy = Math.floor(k / w);
    const cx = k - cy * w;
    if (cx === gx && cy === gy) {
      found = true;
      break;
    }
    for (const [dy, dx] of N8) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (!passable(nx, ny)) continue;
      if (dx && dy && !(passable(cx + dx, cy) && passable(cx, cy + dy))) continue;
      const nk = key(nx, ny);
      const ng = gc + Math.hypot(dx, dy) * (cls[nk] === MARGINAL ? 3 : 1);
      if (ng < best[nk]) {
        best[nk] = ng;
        came[nk] = k;
        open.push([ng + Math.hypot(gx - nx, gy - ny), ng, nk]);
      }
    }
  }
  if (!found) return fail("no passable route (goal blocked, unknown, or unreachable)");
  const cells: number[] = [];
  for (let k = key(gx, gy); k >= 0; k = came[k]) cells.push(k);
  cells.reverse();
  const path: [number, number][] = cells.map((k) => {
    const iy = Math.floor(k / w);
    const ix = k - iy * w;
    return [r(x0 + (ix + 0.5) * cell, 3), r(y0 + (iy + 0.5) * cell, 3)];
  });
  let length = 0;
  for (let i = 1; i < path.length; i++) length += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
  const heights = cells.map((k) => rel[k]).filter((v) => Number.isFinite(v));
  let maxStep = 0;
  for (let i = 1; i < heights.length; i++) maxStep = Math.max(maxStep, Math.abs(heights[i] - heights[i - 1]));
  const nMarginal = cells.filter((k) => cls[k] === MARGINAL).length;
  return {
    found: true,
    start_snapped_m: r(snapped, 2),
    length_m: r(length, 2),
    cells: cells.length,
    marginal_cells: nMarginal,
    verdict: nMarginal === 0 ? "passable" : "marginal",
    max_step_m: r(maxStep, 3),
    path,
  };
}

// Passable or marginal cell centres within `within` metres of (x, y), nearest first.
export function nearestPassable(res: OpsResult, x: number, y: number, within = 1.0): [number, number][] {
  const out: { xy: [number, number]; d: number }[] = [];
  for (let k = 0; k < res.w * res.h; k++) {
    if (res.classes[k] !== PASSABLE && res.classes[k] !== MARGINAL) continue;
    const iy = Math.floor(k / res.w);
    const ix = k - iy * res.w;
    const cx = res.x0 + (ix + 0.5) * res.cell;
    const cy = res.y0 + (iy + 0.5) * res.cell;
    const d = Math.hypot(cx - x, cy - y);
    if (d <= within) out.push({ xy: [cx, cy], d });
  }
  out.sort((a, b) => a.d - b.d);
  return out.map((o) => o.xy);
}

// ---- assistant layer (canned: reads the findings, never the points) ----

export const AI_MODELS = { decisions: "typesafe/jev-1.14", prose: "google/gemini-2.5-flash" };

const clamp01 = (v: number) => Math.max(0, Math.min(0.99, v));

export function aiTriage(f: Findings): AiTriage {
  const items: AiTriageItem[] = [];
  for (const v of f.voids)
    items.push({
      id: v.id,
      kind: "void",
      attention: clamp01(0.6 + Math.min(0.3, v.area_m2 * 0.2) + (v.distance_m < 2 ? 0.05 : 0)),
      confidence: r(0.72 - Math.min(0.2, v.distance_m * 0.03), 2),
      why: `${v.clearance_m} m clear under cover, ${v.distance_m} m away at ${v.bearing_deg}°: nearest covered space worth checking`,
    });
  for (const s of f.shadows)
    items.push({
      id: s.id,
      kind: "shadow",
      attention: clamp01(0.35 + Math.min(0.25, s.area_m2 * 0.15)),
      confidence: 0.65,
      why: `${s.area_m2} m² not seen behind a nearer object; a station at ${s.bearing_deg}° would cover it`,
    });
  for (const d of f.depressions)
    items.push({
      id: d.id,
      kind: "depression",
      attention: clamp01(0.4 + Math.min(0.2, (d.depth_m ?? 0) * 0.3)),
      confidence: 0.6,
      why: `${d.depth_m} m below the floor over ${d.area_m2} m² at ${d.distance_m} m`,
    });
  for (const o of f.obstacles)
    items.push({
      id: o.id,
      kind: "obstacle",
      attention: clamp01(0.15 + Math.min(0.2, (o.height_m ?? 0) * 0.2)),
      confidence: 0.8,
      why: `${o.height_m} m high at ${o.distance_m} m, ${o.bearing_deg}°: blocks the direct line`,
    });
  items.forEach((i) => (i.attention = r(i.attention, 2)));
  items.sort((a, b) => b.attention - a.attention);
  const c = f.coverage;
  const shadowFrac = c.observable_cells ? c.shadow_cells / c.observable_cells : 0;
  const p = r(Math.max(0.05, Math.min(0.95, 1 - c.coverage_pct / 100 + shadowFrac)), 2);
  return {
    items,
    rescan: { recommended: c.coverage_pct < 85 || shadowFrac > 0.1, p, confidence: 0.8 },
    model: AI_MODELS.decisions,
    cost_usd: 0.0003,
  };
}

export function aiSitrep(f: Findings): AiSitrep {
  const c = f.coverage;
  const lines: string[] = [];
  lines.push(
    `Scanner ${f.scanner.height_above_floor_m ?? "?"} m above a floor fitted to ${f.floor.rms_mm ?? "?"} mm RMS; ` +
      `coverage ${c.coverage_pct}% of the observable area (${c.area_seen_m2} m² seen, floor sampled to ${c.floor_reach_m} m).`,
  );
  lines.push(
    f.voids.length
      ? `Possible voids: ${f.voids.map((v) => `${v.id} ${v.clearance_m} m clearance at ${v.distance_m} m bearing ${v.bearing_deg}° (${v.area_m2} m²)`).join("; ")}.`
      : "No covered voids found in the scanned area.",
  );
  lines.push(
    f.obstacles.length
      ? `Obstacles: ${f.obstacles.slice(0, 4).map((o) => `${o.id} ${o.height_m} m high at ${o.distance_m} m bearing ${o.bearing_deg}°`).join("; ")}${f.obstacles.length > 4 ? ` and ${f.obstacles.length - 4} more` : ""}.`
      : "No obstacles above 0.25 m.",
  );
  lines.push(
    (f.shadows.length ? `Occluded: ${f.shadows.slice(0, 3).map((s) => `${s.id} ${s.area_m2} m² at ${s.distance_m} m bearing ${s.bearing_deg}°`).join("; ")}. ` : "No occluded regions. ") +
      (c.blind_stripe ? `Blind stripe ±${c.blind_stripe.half_width_m} m along the tilt axle (${c.blind_stripe.cells} cells).` : ""),
  );
  lines.push(
    f.next_station
      ? f.next_station.bearing_deg === null
        ? `Next: ${f.next_station.reason}.`
        : `Next: move ${f.next_station.distance_m} m on bearing ${f.next_station.bearing_deg}° (${f.next_station.reason}).`
      : "Next: coverage is complete from this station.",
  );
  return { text: lines.join("\n"), facts: f, model: AI_MODELS.prose, cost_usd: 0.0012 };
}

export function aiAsk(
  question: string,
  f: Findings,
  route: (from: [number, number], to: [number, number]) => RouteResult,
  snapGoal: (x: number, y: number) => [number, number][] = () => [],
): AiAsk {
  const q = question.toLowerCase();
  const idm = question.match(/\b([VOSD]\d+)\b/i);
  const all = [...f.voids, ...f.obstacles, ...f.depressions, ...f.shadows];
  const byId = idm ? all.find((x) => x.id.toLowerCase() === idm[1].toLowerCase()) : undefined;
  const cost_usd = 0.0004;

  if (/\b(route|path|get to|reach|way to|go to)\b/.test(q)) {
    const xy = q.match(/(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)/);
    let target: Finding | undefined = byId;
    let to: [number, number] | null = null;
    let name = "";
    if (target) {
      to = [target.x_m, target.y_m];
      name = target.id;
    } else if (xy) {
      to = [Number(xy[1]), Number(xy[2])];
      name = `(${to[0]}, ${to[1]})`;
    } else if (f.voids.length) {
      target = f.voids[0];
      to = [target.x_m, target.y_m];
      name = `${target.id} (nearest void)`;
    }
    if (!to) return { intent: "route", confidence: 0.6, tool: null, answer: "No target: name a finding (V1, O2) or give x, y in metres.", cost_usd };
    // a finding's own cells are rarely passable (a slab top, a box): aim for the nearest
    // passable cell beside it, skipping islands (a patch under the slab nothing connects to)
    let res = route([0, 0], to);
    if (target && !res.found) {
      for (const cand of snapGoal(to[0], to[1]).slice(0, 40)) {
        res = route([0, 0], cand);
        if (res.found) break;
      }
    }
    const answer = res.found
      ? `Route to ${name}: ${res.length_m} m, ${res.verdict}, max step ${res.max_step_m} m${res.start_snapped_m > 0 ? `, start snapped ${res.start_snapped_m} m` : ""}.`
      : `No route to ${name}: ${res.reason}.`;
    return { intent: "route", confidence: 0.9, tool: res, answer, cost_usd };
  }
  if (/\b(how far|distance|bearing|where is|measure)\b/.test(q)) {
    if (!byId) return { intent: "measure", confidence: 0.5, tool: null, answer: "Name a finding to measure to, for example O1.", cost_usd };
    const t = { id: byId.id, distance_m: byId.distance_m, bearing_deg: byId.bearing_deg, x_m: byId.x_m, y_m: byId.y_m };
    return { intent: "measure", confidence: 0.9, tool: t, answer: `${byId.id} is ${byId.distance_m} m away at bearing ${byId.bearing_deg}° (x ${byId.x_m}, y ${byId.y_m}).`, cost_usd };
  }
  if (/\b(report|situation|summary|sitrep|status)\b/.test(q)) {
    const s = aiSitrep(f);
    return { intent: "sitrep", confidence: 0.85, tool: { text: s.text }, answer: s.text, cost_usd: 0.0012 };
  }
  if (/\b(void|voids|cover|covered|gap|under)\b/.test(q)) {
    const answer = f.voids.length
      ? `${f.voids.length} possible void${f.voids.length > 1 ? "s" : ""}: ${f.voids.map((v) => `${v.id} ${v.clearance_m} m clearance, ${v.distance_m} m at ${v.bearing_deg}°`).join("; ")}.`
      : "No covered voids in the scanned area.";
    return { intent: "list_voids", confidence: 0.9, tool: { voids: f.voids }, answer, cost_usd };
  }
  if (/\b(obstacle|obstacles|block|blocked|box|wall)\b/.test(q)) {
    const answer = f.obstacles.length
      ? `${f.obstacles.length} obstacle${f.obstacles.length > 1 ? "s" : ""}: ${f.obstacles.map((o) => `${o.id} ${o.height_m} m, ${o.distance_m} m at ${o.bearing_deg}°`).join("; ")}.`
      : "No obstacles above 0.25 m.";
    return { intent: "list_obstacles", confidence: 0.9, tool: { obstacles: f.obstacles }, answer, cost_usd };
  }
  if (/\b(coverage|seen|unseen|blind|shadow|unknown|missing)\b/.test(q)) {
    const c = f.coverage;
    const answer = `Coverage ${c.coverage_pct}%: ${c.area_seen_m2} m² seen, ${c.shadow_cells} shadow cells, ${c.blind_cells} blind cells, ${c.unknown_cells} unknown.`;
    return { intent: "coverage", confidence: 0.85, tool: c, answer, cost_usd };
  }
  return {
    intent: "other",
    confidence: 0.3,
    tool: null,
    answer: "I can route to a finding, measure to it, list voids or obstacles, give coverage, or write a situation report.",
    cost_usd,
  };
}
