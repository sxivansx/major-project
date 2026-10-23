// Handheld 2D room mapping for the mock core: someone carries the scanner
// around the mock room along a loop; every tick the pose advances and a 2D
// slice (the room walls and the furniture footprints) is ray-cast into an
// occupancy grid. Map frame = where mapping started (+x = start heading,
// +y to its left), so here the map frame is the room frame shifted to the start.

import { encodeRoom } from "../src/api/frames";
import type { RoomState } from "../src/api/types";
import { BOXES, ROOM } from "./world";

const TICK_MS = 200; // "room" messages at 5 Hz
const GRID_EVERY = 3; // kind-6 frames at ~1.7 Hz
const SPEED = 0.35; // m/s
const RAYS = 240;
const MAX_RANGE = 8;

// a loop through the room that stays clear of the furniture
const WAYPOINTS: [number, number][] = [
  [0, 0],
  [2.2, 0],
  [2.2, 2.1],
  [0.2, 2.1],
  [0.2, 0.6],
  [-1.0, 0.6],
  [-1.0, -1.0],
  [2.5, -1.0],
  [2.5, 0],
  [0, 0],
];
const START: [number, number] = WAYPOINTS[0];

function cast(ox: number, oy: number, dx: number, dy: number): number {
  // exit of the room box
  let t = MAX_RANGE;
  const ex = (b0: number, b1: number, o: number, d: number) => (Math.abs(d) < 1e-9 ? Infinity : d > 0 ? (b1 - o) / d : (b0 - o) / d);
  t = Math.min(t, ex(ROOM.min[0], ROOM.max[0], ox, dx), ex(ROOM.min[1], ROOM.max[1], oy, dy));
  // entry into each furniture footprint
  for (const b of BOXES) {
    let t0 = -Infinity;
    let t1 = Infinity;
    for (const [o, d, lo, hi] of [
      [ox, dx, b.min[0], b.max[0]],
      [oy, dy, b.min[1], b.max[1]],
    ] as [number, number, number, number][]) {
      if (Math.abs(d) < 1e-9) {
        if (o < lo || o > hi) {
          t0 = Infinity;
          break;
        }
        continue;
      }
      let a = (lo - o) / d;
      let c = (hi - o) / d;
      if (a > c) [a, c] = [c, a];
      t0 = Math.max(t0, a);
      t1 = Math.min(t1, c);
    }
    if (t0 <= t1 && t0 > 0) t = Math.min(t, t0);
  }
  return t;
}

export class RoomSim {
  private state: "idle" | "mapping" = "idle";
  private cell = 0.05;
  private w = 0;
  private h = 0;
  private x0 = 0;
  private y0 = 0;
  private cells = new Uint8Array(0);
  private hits = new Int16Array(0); // evidence per cell: + occupied, − free
  private pose = { x: 0, y: 0, theta_deg: 0 };
  private path: [number, number][] = [];
  private keyframes = 0;
  private scans = 0;
  private lastKey = { x: 0, y: 0, th: 0 };
  private seg = 0;
  private along = 0;
  private match = { rms_mm: null as number | null, inliers: null as number | null, ok: false };
  private timer: ReturnType<typeof setInterval> | null = null;
  private tick = 0;

  constructor(
    private sendState: (r: RoomState) => void,
    private sendGrid: (buf: ArrayBuffer) => void,
  ) {}

  snapshot(withGrid = false): RoomState {
    let free = 0;
    for (let i = 0; i < this.cells.length; i++) if (this.cells[i] === 1) free++;
    const r: RoomState = {
      state: this.state,
      pose: this.w ? { x: +this.pose.x.toFixed(3), y: +this.pose.y.toFixed(3), theta_deg: +this.pose.theta_deg.toFixed(1) } : null,
      path: this.path.slice(-500),
      keyframes: this.keyframes,
      scans: this.scans,
      area_m2: +(free * this.cell * this.cell).toFixed(2),
      match: this.w ? { ...this.match } : null,
      lost: false,
      note: this.state === "mapping" && this.scans < 5 ? "hold still for a moment while the first scans settle" : null,
    };
    if (withGrid) r.grid = { w: this.w, h: this.h, cell: this.cell, x0: this.x0, y0: this.y0 };
    return r;
  }

  gridFrame(): ArrayBuffer {
    return encodeRoom({ w: this.w, h: this.h, cell_m: this.cell, x0_m: this.x0, y0_m: this.y0, cells: this.cells });
  }

  start(cell?: number): void {
    this.stopTimer();
    this.cell = cell && cell >= 0.02 && cell <= 0.5 ? cell : 0.05;
    // grid covering the room in map coordinates (room shifted to the start), with a margin
    this.x0 = Math.floor((ROOM.min[0] - START[0] - 0.3) / this.cell) * this.cell;
    this.y0 = Math.floor((ROOM.min[1] - START[1] - 0.3) / this.cell) * this.cell;
    this.w = Math.ceil((ROOM.max[0] - ROOM.min[0] + 0.6) / this.cell);
    this.h = Math.ceil((ROOM.max[1] - ROOM.min[1] + 0.6) / this.cell);
    this.cells = new Uint8Array(this.w * this.h);
    this.hits = new Int16Array(this.w * this.h);
    this.pose = { x: 0, y: 0, theta_deg: 0 };
    this.path = [[0, 0]];
    this.keyframes = 1;
    this.scans = 0;
    this.seg = 0;
    this.along = 0;
    this.lastKey = { x: 0, y: 0, th: 0 };
    this.state = "mapping";
    this.tick = 0;
    this.scan();
    this.sendState(this.snapshot());
    this.sendGrid(this.gridFrame());
    this.timer = setInterval(() => this.step(), TICK_MS);
  }

  stop(): void {
    this.stopTimer();
    if (this.state !== "idle") {
      this.state = "idle";
      this.sendState(this.snapshot());
    }
  }

  private stopTimer() {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  }

  private step() {
    // advance along the loop
    let d = SPEED * (TICK_MS / 1000);
    while (d > 0) {
      const a = WAYPOINTS[this.seg % (WAYPOINTS.length - 1)];
      const b = WAYPOINTS[(this.seg % (WAYPOINTS.length - 1)) + 1];
      const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
      const left = len - this.along;
      if (d < left) {
        this.along += d;
        d = 0;
      } else {
        d -= left;
        this.along = 0;
        this.seg++;
      }
    }
    const a = WAYPOINTS[this.seg % (WAYPOINTS.length - 1)];
    const b = WAYPOINTS[(this.seg % (WAYPOINTS.length - 1)) + 1];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
    const f = this.along / len;
    const wx = a[0] + (b[0] - a[0]) * f;
    const wy = a[1] + (b[1] - a[1]) * f;
    const want = (Math.atan2(b[1] - a[1], b[0] - a[0]) * 180) / Math.PI;
    // turn gently toward the direction of travel
    let dth = ((want - this.pose.theta_deg + 540) % 360) - 180;
    dth = Math.max(-25, Math.min(25, dth));
    this.pose = { x: wx - START[0], y: wy - START[1], theta_deg: (((this.pose.theta_deg + dth + 180) % 360) + 360) % 360 - 180 };
    const last = this.path[this.path.length - 1];
    if (Math.hypot(this.pose.x - last[0], this.pose.y - last[1]) > 0.05) this.path.push([+this.pose.x.toFixed(3), +this.pose.y.toFixed(3)]);
    if (this.path.length > 500) this.path.splice(0, this.path.length - 500);
    if (Math.hypot(this.pose.x - this.lastKey.x, this.pose.y - this.lastKey.y) > 0.5 || Math.abs(((this.pose.theta_deg - this.lastKey.th + 540) % 360) - 180) > 20) {
      this.keyframes++;
      this.lastKey = { x: this.pose.x, y: this.pose.y, th: this.pose.theta_deg };
    }
    this.scan();
    this.match = { rms_mm: +(12 + Math.random() * 10).toFixed(1), inliers: +(0.84 + Math.random() * 0.12).toFixed(2), ok: true };
    this.tick++;
    this.sendState(this.snapshot());
    if (this.tick % GRID_EVERY === 0) this.sendGrid(this.gridFrame());
  }

  // one 2D scan from the current pose into the grid
  private scan() {
    this.scans++;
    const ox = this.pose.x + START[0];
    const oy = this.pose.y + START[1];
    const c = this.cell;
    for (let k = 0; k < RAYS; k++) {
      const ang = (k / RAYS) * 2 * Math.PI;
      const dx = Math.cos(ang);
      const dy = Math.sin(ang);
      const t = cast(ox, oy, dx, dy) + (Math.random() - 0.5) * 0.02;
      const hit = t < MAX_RANGE;
      for (let s = 0; s < t; s += c * 0.5) {
        const mx = ox + dx * s - START[0];
        const my = oy + dy * s - START[1];
        const ix = Math.floor((mx - this.x0) / c);
        const iy = Math.floor((my - this.y0) / c);
        if (ix < 0 || iy < 0 || ix >= this.w || iy >= this.h) break;
        const i = iy * this.w + ix;
        if (this.hits[i] > -20) this.hits[i]--;
      }
      if (hit) {
        const mx = ox + dx * t - START[0];
        const my = oy + dy * t - START[1];
        const ix = Math.floor((mx - this.x0) / c);
        const iy = Math.floor((my - this.y0) / c);
        if (ix >= 0 && iy >= 0 && ix < this.w && iy < this.h) {
          const i = iy * this.w + ix;
          this.hits[i] = Math.min(40, this.hits[i] + 4);
        }
      }
    }
    for (let i = 0; i < this.cells.length; i++) this.cells[i] = this.hits[i] > 0 ? 2 : this.hits[i] < 0 ? 1 : 0;
  }
}
