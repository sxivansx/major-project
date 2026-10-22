// Fake bench room for the mock server: an axis-aligned box room with a few
// boxes in it, ray-cast from the lidar at a given tilt. Base frame: origin on
// the base plate under the tilt axis, z up, x along the tilt axis.

export interface Aabb {
  min: [number, number, number];
  max: [number, number, number];
}

// The bench top is the base plate (z = 0). Floor 0.75 m below, ceiling 4.25 m
// above the floor (above the 3 m the operator analysis looks at, so the max-height
// surface is the floor and the boxes, as in a rubble field). The lidar sits a
// little off-centre in the room.
export const ROOM: Aabb = { min: [-2.0, -1.5, -0.75], max: [4.0, 2.5, 3.5] };

// Laid out so the operator analysis has something to say from a single station:
// a low crate to route around, a tall box that casts a shadow to the far wall,
// a slab with open space underneath (a covered void) against the far wall so
// its own shadow lands on the wall, and a box on the bench behind the scanner
// inside the blind stripe, where the horizontal live scan still sees it.
export const BOXES: Aabb[] = [
  { min: [1.0, 1.0, -0.75], max: [1.6, 1.6, -0.45] }, // crate on the floor, 0.3 m: routes go around it
  { min: [2.6, 0.8, -0.75], max: [3.4, 1.6, 0.35] }, // taller box, above the scanner: shadow to the far wall
  { min: [-1.2, 1.4, -0.75], max: [-0.4, 2.2, -0.45] }, // low box
  { min: [-0.9, -0.15, 0.0], max: [-0.6, 0.15, 0.25] }, // small box on the bench, behind the scanner
  { min: [-1.9, -1.4, -0.75], max: [-1.3, -0.2, 0.05] }, // cabinet in the corner
  { min: [3.0, -1.4, -0.35], max: [4.0, -0.5, -0.25] }, // slab 0.4 m over the floor against the far wall: a covered void
];

export interface Mount {
  axis_z_mm: number;
  h_mm: number;
  lidar_yaw_deg: number;
  center_deg: number;
  invert: boolean;
}

const DEG = Math.PI / 180;

// Distance along the ray (o, d) to the exit of an AABB we are inside, or
// Infinity if the ray misses / we are outside.
function exitDistance(o: number[], d: number[], b: Aabb): number {
  let tmax = Infinity;
  for (let i = 0; i < 3; i++) {
    if (Math.abs(d[i]) < 1e-9) {
      if (o[i] < b.min[i] || o[i] > b.max[i]) return Infinity;
      continue;
    }
    const inv = 1 / d[i];
    let t1 = (b.min[i] - o[i]) * inv;
    let t2 = (b.max[i] - o[i]) * inv;
    if (t1 > t2) [t1, t2] = [t2, t1];
    if (t2 < tmax) tmax = t2;
  }
  return tmax > 0 ? tmax : Infinity;
}

// Distance along the ray to the entry of an AABB, or Infinity.
function entryDistance(o: number[], d: number[], b: Aabb): number {
  let tmin = -Infinity;
  let tmax = Infinity;
  for (let i = 0; i < 3; i++) {
    if (Math.abs(d[i]) < 1e-9) {
      if (o[i] < b.min[i] || o[i] > b.max[i]) return Infinity;
      continue;
    }
    const inv = 1 / d[i];
    let t1 = (b.min[i] - o[i]) * inv;
    let t2 = (b.max[i] - o[i]) * inv;
    if (t1 > t2) [t1, t2] = [t2, t1];
    if (t1 > tmin) tmin = t1;
    if (t2 < tmax) tmax = t2;
    if (tmin > tmax) return Infinity;
  }
  if (tmax < 0) return Infinity;
  return tmin > 0 ? tmin : Infinity;
}

export function castRay(o: number[], d: number[]): number {
  let t = exitDistance(o, d, ROOM);
  for (const b of BOXES) {
    const tb = entryDistance(o, d, b);
    if (tb < t) t = tb;
  }
  return t;
}

// Deterministic-ish gaussian noise
function gauss(): number {
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export interface FakeScan {
  n: number;
  angle: Float32Array; // native, clockwise, degrees
  range: Float32Array;
  quality: Uint8Array;
}

// Pose of the lidar in the base frame for tilt φ (deg): origin and the
// rotation that maps a lidar-frame direction into the base frame.
export function lidarPose(mount: Mount, phiDeg: number) {
  const phi = phiDeg * DEG;
  const yaw = mount.lidar_yaw_deg * DEG;
  const h = mount.h_mm / 1000;
  const axisZ = mount.axis_z_mm / 1000;
  const cp = Math.cos(phi);
  const sp = Math.sin(phi);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  // origin: Rx(phi) * (0, 0, h) + (0, 0, axis_z)
  const origin = [0, -sp * h, cp * h + axisZ];
  // direction for lidar-frame unit vector (ux, uy, 0): Rx(phi) * Rz(yaw) * u
  const dir = (ux: number, uy: number): number[] => {
    const x = cy * ux - sy * uy;
    const y = sy * ux + cy * uy;
    return [x, cp * y, sp * y];
  };
  return { origin, dir };
}

// One full rotation at tilt φ. Points with no return are dropped (range 0
// nodes are dropped by s2bridge too).
export function fakeScan(mount: Mount, phiDeg: number, nNominal = 3200, maxRange = 30): FakeScan {
  const { origin, dir } = lidarPose(mount, phiDeg);
  const angle = new Float32Array(nNominal);
  const range = new Float32Array(nNominal);
  const quality = new Uint8Array(nNominal);
  let n = 0;
  const step = 360 / nNominal;
  for (let i = 0; i < nNominal; i++) {
    // non-negative jitter keeps angles ascending and inside [0, 360)
    const a = i * step + Math.random() * step * 0.3;
    if (Math.random() < 0.015) continue; // dropped node
    const theta = -a * DEG; // native angle is clockwise, lidar frame is ccw
    const d = dir(Math.cos(theta), Math.sin(theta));
    let r = castRay(origin, d);
    if (!Number.isFinite(r) || r > maxRange) continue;
    r += gauss() * 0.01; // S2 precision is about a centimetre; accuracy (±30 mm) is systematic
    let q = Math.max(10, Math.min(63, Math.round(58 - r * 1.2 + gauss() * 4)));
    if (Math.random() < 0.003) {
      r *= 0.5 + Math.random(); // the odd outlier, flagged by a low quality like the real unit
      q = Math.floor(Math.random() * 8);
    }
    if (r < 0.1) continue;
    angle[n] = ((a % 360) + 360) % 360;
    range[n] = r;
    quality[n] = q;
    n++;
  }
  return { n, angle: angle.subarray(0, n), range: range.subarray(0, n), quality: quality.subarray(0, n) };
}

// Drop nodes below a quality threshold (what the core does at ingest with lidar.min_quality).
export function filterQuality(s: FakeScan, minQuality: number): FakeScan {
  let n = 0;
  for (let i = 0; i < s.n; i++) if (s.quality[i] >= minQuality) n++;
  if (n === s.n) return s;
  const angle = new Float32Array(n);
  const range = new Float32Array(n);
  const quality = new Uint8Array(n);
  let o = 0;
  for (let i = 0; i < s.n; i++) {
    if (s.quality[i] < minQuality) continue;
    angle[o] = s.angle[i];
    range[o] = s.range[i];
    quality[o] = s.quality[i];
    o++;
  }
  return { n, angle, range, quality };
}

// Transform a scan at tilt φ into base-frame xyz.
export function scanToPoints(mount: Mount, phiDeg: number, s: FakeScan): Float32Array {
  const { origin, dir } = lidarPose(mount, phiDeg);
  const xyz = new Float32Array(s.n * 3);
  for (let i = 0; i < s.n; i++) {
    const theta = -s.angle[i] * DEG;
    const d = dir(Math.cos(theta), Math.sin(theta));
    const r = s.range[i];
    xyz[i * 3] = origin[0] + d[0] * r;
    xyz[i * 3 + 1] = origin[1] + d[1] * r;
    xyz[i * 3 + 2] = origin[2] + d[2] * r;
  }
  return xyz;
}

export interface GridOut {
  layer: "max" | "min" | "mean" | "count";
  w: number;
  h: number;
  cell_m: number;
  x0_m: number;
  y0_m: number;
  z_min_m: number;
  z_max_m: number;
  data: Float32Array;
}

const MAX_CELLS = 4_000_000;

export function computeGrid(
  xyz: Float32Array,
  nPoints: number,
  cell: number,
  layer: GridOut["layer"],
  zMin: number,
  zMax: number,
): GridOut {
  let minx = Infinity;
  let miny = Infinity;
  let maxx = -Infinity;
  let maxy = -Infinity;
  for (let i = 0; i < nPoints; i++) {
    const z = xyz[i * 3 + 2];
    if (z < zMin || z > zMax) continue;
    const x = xyz[i * 3];
    const y = xyz[i * 3 + 1];
    if (x < minx) minx = x;
    if (x > maxx) maxx = x;
    if (y < miny) miny = y;
    if (y > maxy) maxy = y;
  }
  if (!Number.isFinite(minx)) {
    return { layer, w: 0, h: 0, cell_m: cell, x0_m: 0, y0_m: 0, z_min_m: zMin, z_max_m: zMax, data: new Float32Array(0) };
  }
  const x0 = Math.floor(minx / cell) * cell;
  const y0 = Math.floor(miny / cell) * cell;
  let w = Math.floor((maxx - x0) / cell) + 1;
  let h = Math.floor((maxy - y0) / cell) + 1;
  if (w * h > MAX_CELLS) {
    const k = Math.sqrt(MAX_CELLS / (w * h));
    w = Math.max(1, Math.floor(w * k));
    h = Math.max(1, Math.floor(h * k));
  }
  const acc = new Float32Array(w * h).fill(layer === "min" ? Infinity : layer === "max" ? -Infinity : 0);
  const cnt = new Uint32Array(w * h);
  for (let i = 0; i < nPoints; i++) {
    const z = xyz[i * 3 + 2];
    if (z < zMin || z > zMax) continue;
    const ix = Math.floor((xyz[i * 3] - x0) / cell);
    const iy = Math.floor((xyz[i * 3 + 1] - y0) / cell);
    if (ix < 0 || iy < 0 || ix >= w || iy >= h) continue;
    const k = iy * w + ix;
    cnt[k]++;
    if (layer === "max") {
      if (z > acc[k]) acc[k] = z;
    } else if (layer === "min") {
      if (z < acc[k]) acc[k] = z;
    } else if (layer === "mean") acc[k] += z;
  }
  const data = new Float32Array(w * h);
  for (let k = 0; k < w * h; k++) {
    if (cnt[k] === 0) data[k] = NaN;
    else if (layer === "count") data[k] = cnt[k];
    else if (layer === "mean") data[k] = acc[k] / cnt[k];
    else data[k] = acc[k];
  }
  return { layer, w, h, cell_m: cell, x0_m: x0, y0_m: y0, z_min_m: zMin, z_max_m: zMax, data };
}
