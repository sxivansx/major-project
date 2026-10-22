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

export type BinaryFrame = ScanFrame | PointsFrame | GridFrame;
