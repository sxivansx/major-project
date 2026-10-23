// Binary frame codec for the /ws stream and the binary REST endpoints.
// Everything is little-endian (docs/api.md). Shared by the app, the mock
// server and the headless check script, so keep this file dependency-free.

import type { AudioFrame, BinaryFrame, GridFrame, GridLayer, OpsLayersFrame, PointsFrame, ScanFrame } from "./types";

const LAYER_BY_CODE: GridLayer[] = ["max", "min", "mean", "count"];
export const LAYER_CODE: Record<GridLayer, number> = { max: 0, min: 1, mean: 2, count: 3 };

export const SCAN_HEADER = 24; // u8 kind, u8[3] pad, u32 seq, f64 t, f32 phi, u32 n
export const POINTS_HEADER = 20; // u8 kind, u8[3] pad, u32 job_id, u32 step, f32 phi, u32 n
export const GRID_HEADER = 32; // u8 kind, u8 layer, u16 pad, u32 w, u32 h, 5 x f32
export const OPS_HEADER = 24; // u8 kind=4, u8 pad, u16 pad, u32 w, u32 h, f32 cell, f32 x0, f32 y0

// Typed-array views assume the platform is little-endian (every browser and
// Bun target we care about). Fall back to a DataView copy otherwise.
const PLATFORM_LE = new Uint8Array(new Uint32Array([1]).buffer)[0] === 1;

function f32View(buf: ArrayBuffer, byteOffset: number, n: number): Float32Array {
  if (n === 0) return new Float32Array(0);
  if (PLATFORM_LE && byteOffset % 4 === 0) return new Float32Array(buf, byteOffset, n);
  const dv = new DataView(buf, byteOffset, n * 4);
  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) out[i] = dv.getFloat32(i * 4, true);
  return out;
}

export class FrameError extends Error {}

function need(buf: ArrayBuffer, bytes: number, what: string) {
  if (buf.byteLength < bytes) {
    throw new FrameError(`${what}: need ${bytes} bytes, got ${buf.byteLength}`);
  }
}

export function parseScan(buf: ArrayBuffer): ScanFrame {
  need(buf, SCAN_HEADER, "scan header");
  const dv = new DataView(buf);
  const seq = dv.getUint32(4, true);
  const t_unix = dv.getFloat64(8, true);
  const phi_deg = dv.getFloat32(16, true);
  const n = dv.getUint32(20, true);
  need(buf, SCAN_HEADER + n * 9, "scan body");
  const angle = f32View(buf, SCAN_HEADER, n);
  const range = f32View(buf, SCAN_HEADER + n * 4, n);
  const quality = new Uint8Array(buf, SCAN_HEADER + n * 8, n);
  return { kind: 1, seq, t_unix, phi_deg, n, angle, range, quality };
}

export function parsePoints(buf: ArrayBuffer): PointsFrame {
  need(buf, POINTS_HEADER, "points header");
  const dv = new DataView(buf);
  const job_id = dv.getUint32(4, true);
  const step = dv.getUint32(8, true);
  const phi_deg = dv.getFloat32(12, true);
  const n = dv.getUint32(16, true);
  need(buf, POINTS_HEADER + n * 12, "points body");
  const xyz = f32View(buf, POINTS_HEADER, n * 3);
  return { kind: 2, job_id, step, phi_deg, n, xyz };
}

export function parseGrid(buf: ArrayBuffer): GridFrame {
  need(buf, GRID_HEADER, "grid header");
  const dv = new DataView(buf);
  const layerCode = dv.getUint8(1);
  const w = dv.getUint32(4, true);
  const h = dv.getUint32(8, true);
  const cell_m = dv.getFloat32(12, true);
  const x0_m = dv.getFloat32(16, true);
  const y0_m = dv.getFloat32(20, true);
  const z_min_m = dv.getFloat32(24, true);
  const z_max_m = dv.getFloat32(28, true);
  need(buf, GRID_HEADER + w * h * 4, "grid body");
  const data = f32View(buf, GRID_HEADER, w * h);
  return {
    kind: 3,
    layer: LAYER_BY_CODE[layerCode] ?? "max",
    w,
    h,
    cell_m,
    x0_m,
    y0_m,
    z_min_m,
    z_max_m,
    data,
  };
}

// Kind 4, operator layers (GET /api/ops/layers only). After the 24-byte
// header come w*h u8 classes, then w*h f32 heights above the fitted floor.
// The f32 block is 4-byte aligned only when w*h is a multiple of 4, so
// f32View falls back to a DataView copy for the other cases.
export function parseOpsLayers(buf: ArrayBuffer): OpsLayersFrame {
  need(buf, OPS_HEADER, "ops layers header");
  const dv = new DataView(buf);
  const kind = dv.getUint8(0);
  if (kind !== 4) throw new FrameError(`ops layers: kind ${kind}, expected 4`);
  const w = dv.getUint32(4, true);
  const h = dv.getUint32(8, true);
  const cell_m = dv.getFloat32(12, true);
  const x0_m = dv.getFloat32(16, true);
  const y0_m = dv.getFloat32(20, true);
  const n = w * h;
  need(buf, OPS_HEADER + n * 5, "ops layers body");
  const classes = new Uint8Array(buf, OPS_HEADER, n);
  const height = f32View(buf, OPS_HEADER + n, n);
  return { kind: 4, w, h, cell_m, x0_m, y0_m, classes, height };
}

// Kind 5, audio preview (docs/api.md §5.4):
//   u8 kind=5, u8 channels, u16 n, u32 seq, f64 t_unix,      16 bytes
//   channels × f32 level_dbfs,
//   channels × n × (i16 min, i16 max), channel-major
// Read through a DataView so no alignment is assumed anywhere.
export const AUDIO_HEADER = 16;

export function parseAudio(buf: ArrayBuffer): AudioFrame {
  need(buf, AUDIO_HEADER, "audio header");
  const dv = new DataView(buf);
  const kind = dv.getUint8(0);
  if (kind !== 5) throw new FrameError(`audio: kind ${kind}, expected 5`);
  const channels = dv.getUint8(1);
  const n = dv.getUint16(2, true);
  const seq = dv.getUint32(4, true);
  const t_unix = dv.getFloat64(8, true);
  if (channels === 0 || channels > 16) throw new FrameError(`audio: ${channels} channels`);
  const bodyAt = AUDIO_HEADER + channels * 4;
  need(buf, bodyAt + channels * n * 4, "audio body");
  const levels_dbfs = new Float32Array(channels);
  for (let c = 0; c < channels; c++) levels_dbfs[c] = dv.getFloat32(AUDIO_HEADER + c * 4, true);
  const mins: Int16Array[] = [];
  const maxs: Int16Array[] = [];
  let o = bodyAt;
  for (let c = 0; c < channels; c++) {
    const mn = new Int16Array(n);
    const mx = new Int16Array(n);
    for (let i = 0; i < n; i++, o += 4) {
      mn[i] = dv.getInt16(o, true);
      mx[i] = dv.getInt16(o + 2, true);
    }
    mins.push(mn);
    maxs.push(mx);
  }
  return { kind: 5, channels, n, seq, t_unix, levels_dbfs, mins, maxs };
}

export function encodeAudio(f: Omit<AudioFrame, "kind">): ArrayBuffer {
  const { channels, n } = f;
  const buf = new ArrayBuffer(AUDIO_HEADER + channels * 4 + channels * n * 4);
  const dv = new DataView(buf);
  dv.setUint8(0, 5);
  dv.setUint8(1, channels);
  dv.setUint16(2, n, true);
  dv.setUint32(4, f.seq >>> 0, true);
  dv.setFloat64(8, f.t_unix, true);
  for (let c = 0; c < channels; c++) dv.setFloat32(AUDIO_HEADER + c * 4, f.levels_dbfs[c], true);
  let o = AUDIO_HEADER + channels * 4;
  for (let c = 0; c < channels; c++) {
    for (let i = 0; i < n; i++, o += 4) {
      dv.setInt16(o, f.mins[c][i], true);
      dv.setInt16(o + 2, f.maxs[c][i], true);
    }
  }
  return buf;
}

export function parseFrame(buf: ArrayBuffer): BinaryFrame {
  if (buf.byteLength < 1) throw new FrameError("empty binary frame");
  const kind = new Uint8Array(buf, 0, 1)[0];
  switch (kind) {
    case 1:
      return parseScan(buf);
    case 2:
      return parsePoints(buf);
    case 3:
      return parseGrid(buf);
    case 4:
      return parseOpsLayers(buf);
    case 5:
      return parseAudio(buf);
    default:
      throw new FrameError(`unknown frame kind ${kind}`);
  }
}

// GET /api/sessions/{id}/points: u32 n, n x (f32 x, y, z)
export function parseSessionPoints(buf: ArrayBuffer): Float32Array {
  need(buf, 4, "session points header");
  const n = new DataView(buf).getUint32(0, true);
  need(buf, 4 + n * 12, "session points body");
  return f32View(buf, 4, n * 3);
}

// ---- encoders (used by the mock server and tests) ----

export function encodeScan(f: Omit<ScanFrame, "kind">): ArrayBuffer {
  const n = f.n;
  const buf = new ArrayBuffer(SCAN_HEADER + n * 9);
  const dv = new DataView(buf);
  dv.setUint8(0, 1);
  dv.setUint32(4, f.seq >>> 0, true);
  dv.setFloat64(8, f.t_unix, true);
  dv.setFloat32(16, f.phi_deg, true);
  dv.setUint32(20, n, true);
  let o = SCAN_HEADER;
  for (let i = 0; i < n; i++, o += 4) dv.setFloat32(o, f.angle[i], true);
  for (let i = 0; i < n; i++, o += 4) dv.setFloat32(o, f.range[i], true);
  new Uint8Array(buf, o, n).set(f.quality.subarray(0, n));
  return buf;
}

export function encodePoints(f: Omit<PointsFrame, "kind">): ArrayBuffer {
  const n = f.n;
  const buf = new ArrayBuffer(POINTS_HEADER + n * 12);
  const dv = new DataView(buf);
  dv.setUint8(0, 2);
  dv.setUint32(4, f.job_id >>> 0, true);
  dv.setUint32(8, f.step >>> 0, true);
  dv.setFloat32(12, f.phi_deg, true);
  dv.setUint32(16, n, true);
  let o = POINTS_HEADER;
  for (let i = 0; i < n * 3; i++, o += 4) dv.setFloat32(o, f.xyz[i], true);
  return buf;
}

export function encodeGrid(f: Omit<GridFrame, "kind">): ArrayBuffer {
  const count = f.w * f.h;
  const buf = new ArrayBuffer(GRID_HEADER + count * 4);
  const dv = new DataView(buf);
  dv.setUint8(0, 3);
  dv.setUint8(1, LAYER_CODE[f.layer]);
  dv.setUint16(2, 0, true);
  dv.setUint32(4, f.w, true);
  dv.setUint32(8, f.h, true);
  dv.setFloat32(12, f.cell_m, true);
  dv.setFloat32(16, f.x0_m, true);
  dv.setFloat32(20, f.y0_m, true);
  dv.setFloat32(24, f.z_min_m, true);
  dv.setFloat32(28, f.z_max_m, true);
  let o = GRID_HEADER;
  for (let i = 0; i < count; i++, o += 4) dv.setFloat32(o, f.data[i], true);
  return buf;
}

export function encodeOpsLayers(f: Omit<OpsLayersFrame, "kind">): ArrayBuffer {
  const n = f.w * f.h;
  const buf = new ArrayBuffer(OPS_HEADER + n * 5);
  const dv = new DataView(buf);
  dv.setUint8(0, 4);
  dv.setUint8(1, 0);
  dv.setUint16(2, 0, true);
  dv.setUint32(4, f.w, true);
  dv.setUint32(8, f.h, true);
  dv.setFloat32(12, f.cell_m, true);
  dv.setFloat32(16, f.x0_m, true);
  dv.setFloat32(20, f.y0_m, true);
  new Uint8Array(buf, OPS_HEADER, n).set(f.classes.subarray(0, n));
  let o = OPS_HEADER + n;
  for (let i = 0; i < n; i++, o += 4) dv.setFloat32(o, f.height[i], true);
  return buf;
}

export function encodeSessionPoints(xyz: Float32Array): ArrayBuffer {
  const n = Math.floor(xyz.length / 3);
  const buf = new ArrayBuffer(4 + n * 12);
  const dv = new DataView(buf);
  dv.setUint32(0, n, true);
  let o = 4;
  for (let i = 0; i < n * 3; i++, o += 4) dv.setFloat32(o, xyz[i], true);
  return buf;
}
