// Types for the core <-> web contract (docs/api.md, section 3).

export type LidarSource = "s2" | "sim" | "replay";

export interface LidarStatus {
  connected: boolean;
  source?: LidarSource | null;
  port?: string | null;
  baud?: number | null;
  model?: string | null;
  serial?: string | null;
  firmware?: string | null;
  hardware?: number | null;
  health?: string | null;
  scan_hz?: number | null;
  pts_per_rev?: number | null;
  range_min_m?: number | null;
  range_max_m?: number | null;
}

export interface ServoStatus {
  connected: boolean;
  port?: string | null;
  servo_deg?: number | null;
  phi_deg?: number | null;
  min_deg?: number | null;
  max_deg?: number | null;
  range?: number | null;
  meas_deg?: number | null;
  phi_meas_deg?: number | null;
  encoder?: string | null;
  settled?: boolean | null;
}

export type JobState = "running" | "done" | "stopped" | "error";

export interface Job {
  job_id: number;
  name: string;
  state: JobState;
  step: number;
  n_steps: number;
  phi_deg: number;
  points: number;
  started: number;
  eta_s: number | null;
  error: string | null;
}

export interface Status {
  lidar: LidarStatus;
  servo: ServoStatus;
  job: Job | null;
  live_points: number;
  session_loaded: string | null;
  alarms?: string[];
}

export interface Metrics {
  points: number;
  steps: number | null;
  points_per_step: number | null;
  duration_s: number | null;
  grid: { cell_m: number; cells: number; filled: number; coverage_pct: number; points_per_filled_cell: number | null };
  floor: { points: number; rms_mm: number; tilt_deg: number; offset_mm: number } | null;
}

export interface PortInfo {
  path: string;
  description: string;
}

export type GridLayer = "max" | "min" | "mean" | "count";
export const GRID_LAYERS: GridLayer[] = ["max", "min", "mean", "count"];

export interface Config {
  mount: {
    axis_z_mm: number;
    h_mm: number;
    lidar_yaw_deg: number;
    center_deg: number;
    invert: boolean;
    enc_center_deg: number;
    enc_invert: boolean;
  };
  grid: {
    cell_m: number;
    z_min_m: number;
    z_max_m: number;
    layer: GridLayer;
  };
  lidar: {
    min_quality: number;
    min_range_m: number;
    max_range_m: number;
  };
}

export interface SessionMeta {
  id: string;
  name: string;
  created: number;
  n_points: number;
  n_steps: number;
  phi_start: number;
  phi_end: number;
  phi_step: number;
  rotations: number;
  lidar?: { model?: string; serial?: string };
  mount?: Partial<Config["mount"]>;
}

export interface JobRequest {
  name: string;
  phi_start: number;
  phi_end: number;
  phi_step: number;
  rotations: number;
  settle_ms: number;
}

export type LogLevel = "info" | "warn" | "error";

export interface LogMessage {
  type: "log";
  level: LogLevel;
  msg: string;
  t: number;
}

export type WsTextMessage =
  | ({ type: "status" } & Status)
  | ({ type: "job" } & Job)
  | LogMessage
  | { type: string; [k: string]: unknown };

// Decoded binary frames. Typed arrays are views over the received buffer
// (or copies when the platform is big-endian).

export interface ScanFrame {
  kind: 1;
  seq: number;
  t_unix: number;
  phi_deg: number;
  n: number;
  angle: Float32Array;
  range: Float32Array;
  quality: Uint8Array;
}

export interface PointsFrame {
  kind: 2;
  job_id: number;
  step: number;
  phi_deg: number;
  n: number;
  xyz: Float32Array; // n * 3, base frame, metres
}

export interface GridFrame {
  kind: 3;
  layer: GridLayer;
  w: number;
  h: number;
  cell_m: number;
  x0_m: number;
  y0_m: number;
  z_min_m: number;
  z_max_m: number;
  data: Float32Array; // h * w, row-major, row = y, NaN = empty
}

export type BinaryFrame = ScanFrame | PointsFrame | GridFrame | OpsLayersFrame;

// ---- operator layer: /api/ops/* and /api/ai/* (docs/api.md, section 3) ----

// Cell classes in the kind-4 layers frame.
export const CLASS_NAMES = ["unknown", "passable", "marginal", "blocked", "shadow", "blind"] as const;
export type ClassName = (typeof CLASS_NAMES)[number];

export interface OpsLayersFrame {
  kind: 4;
  w: number;
  h: number;
  cell_m: number;
  x0_m: number;
  y0_m: number;
  classes: Uint8Array; // h * w, row-major, row = y
  height: Float32Array; // h * w, metres above the fitted floor, NaN = not known
}

export interface OpsQuery {
  cell?: number;
  max_step?: number;
  max_slope?: number;
}

// One region in the findings lists. height_m for obstacles, clearance_m for
// voids, depth_m for depressions; shadows carry none of the three.
export interface Finding {
  id: string;
  cells: number;
  area_m2: number;
  x_m: number;
  y_m: number;
  distance_m: number;
  bearing_deg: number; // clockwise from +x
  bbox_m: [number, number, number, number]; // x min, y min, x max, y max
  height_m?: number;
  clearance_m?: number;
  depth_m?: number;
}

export type FindingKind = "voids" | "obstacles" | "depressions" | "shadows";
export const FINDING_KINDS: FindingKind[] = ["voids", "obstacles", "depressions", "shadows"];

export interface Findings {
  scanner: { x_m: number; y_m: number; height_above_floor_m: number | null; note?: string };
  points: number;
  grid: { cell_m: number; w: number; h: number; x0_m: number; y0_m: number };
  floor: { fitted: boolean; points: number; z_at_origin_m?: number; tilt_deg?: number; rms_mm?: number };
  coverage: {
    filled_cells: number;
    interpolated_cells: number;
    shadow_cells: number;
    blind_cells: number;
    unknown_cells: number;
    observable_cells: number;
    area_seen_m2: number;
    coverage_pct: number;
    floor_reach_m: number;
    blind_stripe: { half_width_m: number; cells: number; note: string } | null;
  };
  traversability: {
    passable: number;
    marginal: number;
    blocked: number;
    shadow: number;
    blind: number;
    params: { max_step_m: number; max_slope_deg: number };
  };
  obstacles: Finding[];
  voids: Finding[];
  depressions: Finding[];
  shadows: Finding[];
  // bearing_deg is null when the advice is "rotate in place" (blind stripe).
  next_station: { bearing_deg: number | null; distance_m: number; reason: string } | null;
  // newer cores: whether the live cloud is still being swept (the analysis is partial then)
  scan?: { in_progress: boolean; step?: number | null; n_steps?: number | null; session?: string | null };
}

export interface RouteRequest {
  from_xy: [number, number];
  to_xy: [number, number];
  cell?: number;
  max_step?: number;
  max_slope?: number;
}

export type RouteResult =
  | {
      found: true;
      start_snapped_m: number;
      length_m: number;
      cells: number;
      marginal_cells: number;
      verdict: "passable" | "marginal";
      max_step_m: number;
      path: [number, number][];
    }
  | { found: false; reason: string };

export interface AiStatus {
  available: boolean;
  reason?: string;
  models?: { decisions?: string; prose?: string };
  // newer cores may report the two models separately
  decisions?: { available?: boolean; reason?: string };
  prose?: { available?: boolean; reason?: string };
}

export interface AiTriageItem {
  id: string;
  kind: string;
  attention: number; // 0..1
  confidence: number; // 0..1
  why: string;
}

export interface AiTriage {
  items: AiTriageItem[];
  rescan: { recommended: boolean; p: number; confidence: number };
  model?: string;
  cost_usd?: number;
}

export interface AiSitrep {
  text: string;
  facts?: unknown;
  model?: string; // "template" when written without a model (no API key)
  cost_usd?: number;
}

export type AiIntent = "route" | "measure" | "list_voids" | "list_obstacles" | "coverage" | "sitrep" | "other";

export interface AiAsk {
  intent: AiIntent | string;
  confidence: number;
  tool?: unknown; // shape depends on intent; a route intent carries a RouteResult
  answer: string;
  model?: string;
  cost_usd?: number;
}
