import { create } from "zustand";
import type {
  Config,
  GridFrame,
  Job,
  JobRequest,
  LogLevel,
  PortInfo,
  ScanFrame,
  SessionMeta,
  Status,
} from "../api/types";

export type WsState = "connecting" | "open" | "closed";
export type Tab = "ops" | "live" | "cloud" | "sessions" | "log";

export interface LogEntry {
  id: number;
  level: LogLevel;
  msg: string;
  t: number; // unix seconds
  src: "server" | "client";
}

export const LOG_MAX = 200;

export interface State {
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

  // live data (throttled)
  scan: ScanFrame | null;
  scanHzMeasured: number;
  grid: GridFrame | null;
  cloudCount: number;
  cloudVersion: number;

  log: LogEntry[];

  // scan job form (shared by the Scan job panel and the operator action bar)
  jobForm: JobRequest;

  // ui
  tab: Tab;
  showSurface: boolean;
  showHelpers: boolean;
  pointSize: number;
  colorZ: [number, number]; // z range for the colour ramp in 3D
  surfaceMaxStep: number; // metres; quads with a larger z spread are not drawn

  // actions
  setWs: (p: Partial<Pick<State, "wsState" | "wsAttempt" | "wsRetryAt">>) => void;
  setStatus: (s: Status) => void;
  setJob: (j: Job) => void;
  setConfig: (c: Config) => void;
  setPorts: (p: PortInfo[]) => void;
  setSessions: (s: SessionMeta[]) => void;
  setScan: (f: ScanFrame, hz: number) => void;
  setGrid: (g: GridFrame | null) => void;
  setCloud: (count: number, version: number) => void;
  addLog: (level: LogLevel, msg: string, t?: number, src?: "server" | "client") => void;
  clearLog: () => void;
  setJobForm: (p: Partial<JobRequest>) => void;
  setTab: (t: Tab) => void;
  setUi: (p: Partial<Pick<State, "showSurface" | "showHelpers" | "pointSize" | "colorZ" | "surfaceMaxStep">>) => void;
}

let logId = 0;

const TAB_IDS: Tab[] = ["ops", "live", "cloud", "sessions", "log"];
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

  setWs: (p) => set(p),
  setStatus: (status) => set({ status }),
  setJob: (job) =>
    set((s) => (s.status ? { status: { ...s.status, job } } : {})),
  setConfig: (config) => set({ config }),
  setPorts: (ports) => set({ ports }),
  setSessions: (sessions) => set({ sessions }),
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
