import { create } from "zustand";
import type {
  Config,
  GridFrame,
  GridLayer,
  Job,
  JobRequest,
  LogLevel,
  Metrics,
  PortInfo,
  ScanFrame,
  SessionMeta,
  Status,
} from "../api/types";

export type WsState = "connecting" | "open" | "closed";
export type Tab = "ops" | "live" | "sound" | "cloud" | "room" | "sessions" | "log";

export interface LogEntry {
  id: number;
  level: LogLevel;
  msg: string;
  t: number; // unix seconds
  src: "server" | "client";
}

export const LOG_MAX = 200;

// View settings; every tab's Advanced drawer edits a slice of these.
export interface UiState {
  showSurface: boolean;
  showHelpers: boolean;
  pointSize: number;
  colorZ: [number, number]; // z range for the colour ramp in 3D
  surfaceMaxStep: number; // metres; quads with a larger z spread are not drawn
  gridForm: { cell: string; layer: GridLayer; zMin: string; zMax: string }; // height map query (strings: form inputs)
  fitColours: boolean; // height map colour range from the data instead of z min/max
  cloudResetNonce: number; // bump to put the 3D camera back to the default view
}

export interface State extends UiState {
  // websocket
  wsState: WsState;
  wsAttempt: number;
  wsRetryAt: number | null; // performance.now() ms when the next attempt fires
  wsFrames: number; // binary frames received (debug counter)

  // server state
  status: Status | null;
  config: Config | null;
  ports: PortInfo[];
  sessions: SessionMeta[];
  metrics: Metrics | null; // GET /api/metrics for the current grid
  metricsError: string | null;

  // live data (throttled)
  scan: ScanFrame | null;
  scanHzMeasured: number;
  grid: GridFrame | null;
  cloudCount: number;
  cloudVersion: number;

  log: LogEntry[];

  // scan job form (shared by the Scan job panel and the action bars)
  jobForm: JobRequest;

  tab: Tab;

  // actions
  setWs: (p: Partial<Pick<State, "wsState" | "wsAttempt" | "wsRetryAt">>) => void;
  setStatus: (s: Status) => void;
  setJob: (j: Job) => void;
  setConfig: (c: Config) => void;
  setPorts: (p: PortInfo[]) => void;
  setSessions: (s: SessionMeta[]) => void;
  setMetrics: (m: Metrics | null, err?: string | null) => void;
  setScan: (f: ScanFrame, hz: number) => void;
  setGrid: (g: GridFrame | null) => void;
  setCloud: (count: number, version: number) => void;
  addLog: (level: LogLevel, msg: string, t?: number, src?: "server" | "client") => void;
  clearLog: () => void;
  setJobForm: (p: Partial<JobRequest>) => void;
  setTab: (t: Tab) => void;
  setUi: (p: Partial<UiState>) => void;
}

let logId = 0;

const TAB_IDS: Tab[] = ["ops", "live", "sound", "cloud", "room", "sessions", "log"];
function initialTab(): Tab {
  const h = typeof location !== "undefined" ? location.hash.replace(/^#/, "") : "";
  return (TAB_IDS as string[]).includes(h) ? (h as Tab) : "ops";
}

export const JOB_DEFAULTS: JobRequest = {
  name: "bench-1",
  phi_start: -60,
  phi_end: 60,
  phi_step: 2,
  rotations: 1,
  settle_ms: 300,
};

export const useStore = create<State>((set) => ({
  wsState: "closed",
  wsAttempt: 0,
  wsRetryAt: null,
  wsFrames: 0,

  status: null,
  config: null,
  ports: [],
  sessions: [],
  metrics: null,
  metricsError: null,

  scan: null,
  scanHzMeasured: 0,
  grid: null,
  cloudCount: 0,
  cloudVersion: 0,

  log: [],

  jobForm: JOB_DEFAULTS,

  tab: initialTab(),
  showSurface: true,
  showHelpers: true,
  pointSize: 2,
  colorZ: [-0.5, 2.5],
  surfaceMaxStep: 1.0,
  gridForm: { cell: "0.05", layer: "max", zMin: "-0.5", zMax: "2.5" },
  fitColours: false,
  cloudResetNonce: 0,

  setWs: (p) => set(p),
  setStatus: (status) => set({ status }),
  setJob: (job) =>
    set((s) => (s.status ? { status: { ...s.status, job } } : {})),
  setConfig: (config) => set({ config }),
  setPorts: (ports) => set({ ports }),
  setSessions: (sessions) => set({ sessions }),
  setMetrics: (metrics, metricsError = null) => set({ metrics, metricsError }),
  setScan: (scan, hz) => set((s) => ({ scan, scanHzMeasured: hz, wsFrames: s.wsFrames + 1 })),
  setGrid: (grid) => set({ grid }),
  setCloud: (cloudCount, cloudVersion) => set({ cloudCount, cloudVersion }),
  addLog: (level, msg, t, src = "server") =>
    set((s) => {
      const entry: LogEntry = { id: ++logId, level, msg, t: t ?? Date.now() / 1000, src };
      const log = s.log.length >= LOG_MAX ? s.log.slice(s.log.length - LOG_MAX + 1) : s.log.slice();
      log.push(entry);
      return { log };
    }),
  clearLog: () => set({ log: [] }),
  setJobForm: (p) => set((s) => ({ jobForm: { ...s.jobForm, ...p } })),
  setTab: (tab) => {
    if (typeof history !== "undefined") history.replaceState(null, "", "#" + tab);
    set({ tab });
  },
  setUi: (p) => set(p),
}));

export const clientLog = (level: LogLevel, msg: string) =>
  useStore.getState().addLog(level, msg, undefined, "client");
