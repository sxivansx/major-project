// Thin fetch wrappers for /api. Errors throw ApiError with the status and
// whatever the server said, so panels can show the message inline.

import type {
  AiAsk,
  AiSitrep,
  AiStatus,
  AiTriage,
  AudioEvent,
  Config,
  Findings,
  GridFrame,
  GridLayer,
  JobRequest,
  Metrics,
  OpsLayersFrame,
  OpsQuery,
  PortInfo,
  RouteRequest,
  RouteResult,
  SessionMeta,
  Status,
} from "./types";
import { parseGrid, parseOpsLayers, parseSessionPoints } from "./frames";

export class ApiError extends Error {
  constructor(
    public status: number,
    message: string,
    public path: string,
  ) {
    super(message);
  }
}

async function readError(res: Response, path: string): Promise<ApiError> {
  let text = "";
  try {
    text = await res.text();
    try {
      const j = JSON.parse(text);
      text = j.detail ?? j.error ?? j.message ?? text;
      if (typeof text !== "string") text = JSON.stringify(text);
    } catch {
      // plain text body
    }
  } catch {
    // no body
  }
  return new ApiError(res.status, text || `${res.status} ${res.statusText}`, path);
}

async function request<T>(method: string, path: string, body?: unknown): Promise<T> {
  const url = "/api" + path;
  const res = await fetch(url, {
    method,
    headers: body !== undefined ? { "content-type": "application/json" } : undefined,
    body: body !== undefined ? JSON.stringify(body) : undefined,
  });
  if (!res.ok) throw await readError(res, path);
  if (res.status === 204) return undefined as T;
  const ct = res.headers.get("content-type") ?? "";
  if (ct.includes("application/json")) return (await res.json()) as T;
  const text = await res.text();
  return (text ? JSON.parse(text) : undefined) as T;
}

async function requestBinary(path: string): Promise<ArrayBuffer> {
  const res = await fetch("/api" + path);
  if (!res.ok) throw await readError(res, path);
  return res.arrayBuffer();
}

function gridQuery(q: GridQuery): string {
  const p = new URLSearchParams();
  if (q.cell !== undefined) p.set("cell", String(q.cell));
  if (q.layer !== undefined) p.set("layer", q.layer);
  if (q.z_min !== undefined && q.z_min !== null) p.set("z_min", String(q.z_min));
  if (q.z_max !== undefined && q.z_max !== null) p.set("z_max", String(q.z_max));
  const s = p.toString();
  return s ? "?" + s : "";
}

function opsQuery(q: OpsQuery): string {
  const p = new URLSearchParams();
  if (q.cell !== undefined) p.set("cell", String(q.cell));
  if (q.max_step !== undefined) p.set("max_step", String(q.max_step));
  if (q.max_slope !== undefined) p.set("max_slope", String(q.max_slope));
  const s = p.toString();
  return s ? "?" + s : "";
}

export interface GridQuery {
  cell?: number;
  layer?: GridLayer;
  z_min?: number | null;
  z_max?: number | null;
}

export const api = {
  status: () => request<Status>("GET", "/status"),
  ports: () => request<PortInfo[]>("GET", "/ports"),

  lidarConnect: (body: { source: "s2" | "sim"; port: string; baud: number }) =>
    request<Status>("POST", "/lidar/connect", body),
  lidarDisconnect: () => request<Status>("POST", "/lidar/disconnect"),

  servoConnect: (port: string) => request<Status>("POST", "/servo/connect", { port }),
  servoDisconnect: () => request<Status>("POST", "/servo/disconnect"),
  servoMove: (body: { servo_deg: number } | { phi_deg: number }) =>
    request<Status>("POST", "/servo/move", body),
  servoHome: () => request<Status>("POST", "/servo/home"),
  servoRaw: (line: string) => request<{ reply: string }>("POST", "/servo/raw", { line }),

  jobStart: (body: JobRequest) => request<{ job_id: number }>("POST", "/job/start", body),
  jobStop: () => request<Status>("POST", "/job/stop"),

  grid: async (q: GridQuery): Promise<GridFrame> => parseGrid(await requestBinary("/grid" + gridQuery(q))),

  sessions: () => request<SessionMeta[]>("GET", "/sessions"),
  session: (id: string) => request<SessionMeta>("GET", `/sessions/${encodeURIComponent(id)}`),
  sessionPoints: async (id: string) =>
    parseSessionPoints(await requestBinary(`/sessions/${encodeURIComponent(id)}/points`)),
  sessionGrid: async (id: string, q: GridQuery) =>
    parseGrid(await requestBinary(`/sessions/${encodeURIComponent(id)}/grid` + gridQuery(q))),
  sessionLoad: (id: string) => request<Status>("POST", `/sessions/${encodeURIComponent(id)}/load`),
  sessionDelete: (id: string) => request<void>("DELETE", `/sessions/${encodeURIComponent(id)}`),
  sessionExportUrl: (id: string, fmt: "ply" | "csv") => `/api/sessions/${encodeURIComponent(id)}/export.${fmt}`,

  config: () => request<Config>("GET", "/config"),
  putConfig: (c: Config) => request<Config>("PUT", "/config", c),

  metrics: (q: GridQuery) => request<Metrics>("GET", "/metrics" + gridQuery(q)),
  snapshotUrl: (rotations: number) => `/api/scan/snapshot.csv?rotations=${rotations}`,

  // operator products (deterministic) and the assistant layer
  opsFindings: (q: OpsQuery) => request<Findings>("GET", "/ops/findings" + opsQuery(q)),
  opsLayers: async (q: OpsQuery): Promise<OpsLayersFrame> => parseOpsLayers(await requestBinary("/ops/layers" + opsQuery(q))),
  opsRoute: (body: RouteRequest) => request<RouteResult>("POST", "/ops/route", body),
  aiStatus: () => request<AiStatus>("GET", "/ai/status"),
  aiTriage: (body: { cell?: number }) => request<AiTriage>("POST", "/ai/triage", body),
  aiSitrep: (body: { cell?: number }) => request<AiSitrep>("POST", "/ai/sitrep", body),
  aiAsk: (question: string) => request<AiAsk>("POST", "/ai/ask", { question }),

  // acoustic layer (docs/api.md §5)
  audioConnect: (body: { source: "tcp" | "sim"; port: string }) => request<Status>("POST", "/audio/connect", body),
  audioDisconnect: () => request<Status>("POST", "/audio/disconnect"),
  audioEvents: (limit = 50) => request<AudioEvent[]>("GET", `/audio/events?limit=${limit}`),
  audioClear: () => request<void>("POST", "/audio/clear"),
  audioEmit: (body: { bearing_deg: number; elevation_deg?: number; snr_db?: number }) => request<unknown>("POST", "/audio/sim/emit", body),
};

export function errorText(e: unknown): string {
  if (e instanceof ApiError) return `${e.status}: ${e.message}`;
  if (e instanceof Error) return e.message;
  return String(e);
}
