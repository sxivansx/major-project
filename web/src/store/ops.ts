// State for the Operator tab: analysis parameters, the findings JSON and the
// kind-4 layers frame from /api/ops/*, the situation report, the current
// route, the selection and view requests for the map, and the Advanced
// disclosure. Kept apart from the main store because nothing else needs it
// and the frames are large.

import { create } from "zustand";
import { api, errorText } from "../api/rest";
import type { AiSitrep, AiStatus, Finding, FindingKind, Findings, OpsLayersFrame, RouteResult, Status } from "../api/types";
import { FINDING_KINDS } from "../api/types";
import { clientLog, useStore } from "./store";

export interface LayerToggles {
  terrain: boolean;
  traversability: boolean;
  shadow: boolean;
  blind: boolean;
  findings: boolean;
  station: boolean;
}

export type XY = [number, number];

const ADVANCED_KEY = "obslam.ops.advanced";
function loadAdvanced(): boolean {
  try {
    return localStorage.getItem(ADVANCED_KEY) === "1";
  } catch {
    return false;
  }
}
function saveAdvanced(v: boolean): void {
  try {
    localStorage.setItem(ADVANCED_KEY, v ? "1" : "0");
  } catch {
    // private window or storage blocked: the setting just does not persist
  }
}

export interface OpsState {
  // analysis parameters sent to both /ops endpoints
  cell: number;
  maxStep: number;
  maxSlope: number;

  findings: Findings | null;
  layers: OpsLayersFrame | null;
  loading: boolean;
  error: string | null;
  updatedAt: number | null; // unix seconds of the last successful refresh

  // situation report (POST /api/ai/sitrep), refreshed after every analysis
  sitrep: AiSitrep | null;
  sitrepBusy: boolean;
  sitrepError: string | null;

  // GET /api/ai/status, fetched once per tab mount
  aiStatus: AiStatus | null;
  aiStatusError: string | null;

  // route
  routeMode: boolean; // next map click sets the start, then the goal
  routeStart: XY | null;
  routeEnds: { from: XY; to: XY } | null;
  route: RouteResult | null;
  routeError: string | null;
  routeBusy: boolean;
  routeSource: "manual" | "assistant" | null;

  // map
  selected: string | null; // finding id
  focus: { x: number; y: number; n: number } | null; // pan request, n makes repeats distinct
  fitNonce: number;
  toggles: LayerToggles; // Advanced only; the simple view has one fixed rendering

  advanced: boolean;

  setParams: (p: Partial<Pick<OpsState, "cell" | "maxStep" | "maxSlope">>) => void;
  setToggle: (k: keyof LayerToggles, v: boolean) => void;
  select: (id: string | null) => void;
  focusOn: (x: number, y: number) => void;
  fit: () => void;
  setRouteMode: (on: boolean) => void;
  setRouteStart: (p: XY | null) => void;
  setRoute: (r: RouteResult | null, source: "manual" | "assistant", ends?: { from: XY; to: XY }) => void;
  clearRoute: () => void;
  setAdvanced: (v: boolean) => void;
}

export const useOps = create<OpsState>((set) => ({
  cell: 0.1,
  maxStep: 0.15,
  maxSlope: 25,

  findings: null,
  layers: null,
  loading: false,
  error: null,
  updatedAt: null,

  sitrep: null,
  sitrepBusy: false,
  sitrepError: null,

  aiStatus: null,
  aiStatusError: null,

  routeMode: false,
  routeStart: null,
  routeEnds: null,
  route: null,
  routeError: null,
  routeBusy: false,
  routeSource: null,

  selected: null,
  focus: null,
  fitNonce: 0,
  toggles: { terrain: false, traversability: true, shadow: true, blind: true, findings: true, station: true },

  advanced: loadAdvanced(),

  setParams: (p) => set(p),
  setToggle: (k, v) => set((s) => ({ toggles: { ...s.toggles, [k]: v } })),
  select: (selected) => set({ selected }),
  focusOn: (x, y) => set((s) => ({ focus: { x, y, n: (s.focus?.n ?? 0) + 1 } })),
  fit: () => set((s) => ({ fitNonce: s.fitNonce + 1 })),
  setRouteMode: (routeMode) => set({ routeMode, routeStart: null }),
  setRouteStart: (routeStart) => set({ routeStart }),
  setRoute: (route, routeSource, routeEnds) =>
    set({ route, routeSource, routeError: null, routeEnds: routeEnds ?? null, routeBusy: false }),
  clearRoute: () =>
    set({ route: null, routeError: null, routeEnds: null, routeStart: null, routeMode: false, routeSource: null }),
  setAdvanced: (advanced) => {
    saveAdvanced(advanced);
    set({ advanced });
  },
}));

// ---- refresh (findings + layers, then the situation report) ----

// The grid frame the current analysis was requested for; a different grid in
// the main store means the analysis is stale.
let analysedGrid: unknown = undefined;
let inFlight = false;
let runAgain = false;

export function opsStale(): boolean {
  return useOps.getState().findings === null || useStore.getState().grid !== analysedGrid;
}

export async function refreshOps(): Promise<void> {
  if (inFlight) {
    runAgain = true;
    return;
  }
  inFlight = true;
  const s = useOps.getState();
  useOps.setState({ loading: true, error: null });
  analysedGrid = useStore.getState().grid;
  const q = { cell: s.cell, max_step: s.maxStep, max_slope: s.maxSlope };
  let ok = false;
  try {
    const [findings, layers] = await Promise.all([api.opsFindings(q), api.opsLayers(q)]);
    const sel = useOps.getState().selected;
    useOps.setState({
      findings,
      layers,
      loading: false,
      updatedAt: Date.now() / 1000,
      selected: sel && findingById(findings, sel) ? sel : null,
    });
    ok = true;
  } catch (e) {
    useOps.setState({ loading: false, error: errorText(e) });
    clientLog("error", `ops: ${errorText(e)}`);
  } finally {
    inFlight = false;
  }
  if (runAgain) {
    runAgain = false;
    await refreshOps();
    return;
  }
  if (ok) void refreshSitrep();
}

let sitrepSeq = 0;
export async function refreshSitrep(): Promise<void> {
  const seq = ++sitrepSeq;
  const s = useOps.getState();
  if (!s.findings || s.findings.grid.w === 0) {
    useOps.setState({ sitrep: null, sitrepBusy: false, sitrepError: null });
    return;
  }
  useOps.setState({ sitrepBusy: true, sitrepError: null });
  try {
    const r = await api.aiSitrep({ cell: s.cell });
    if (seq !== sitrepSeq) return; // a newer request is on its way
    useOps.setState({ sitrep: r, sitrepBusy: false });
  } catch (e) {
    if (seq !== sitrepSeq) return;
    useOps.setState({ sitrepBusy: false, sitrepError: errorText(e) });
  }
}

export async function loadAiStatus(): Promise<void> {
  try {
    useOps.setState({ aiStatus: await api.aiStatus(), aiStatusError: null });
  } catch (e) {
    useOps.setState({ aiStatus: null, aiStatusError: errorText(e) });
  }
}

// At most one refresh per REFRESH_MIN_MS, trailing: a burst of grid frames
// becomes one request every 2 s plus one at the end.
const REFRESH_MIN_MS = 2000;
let refreshTimer: number | undefined;
let lastScheduled = 0;

export function scheduleOpsRefresh(): void {
  if (refreshTimer !== undefined) return;
  const wait = Math.max(0, REFRESH_MIN_MS - (performance.now() - lastScheduled));
  refreshTimer = window.setTimeout(() => {
    refreshTimer = undefined;
    lastScheduled = performance.now();
    void refreshOps();
  }, wait);
}

export function jobRunning(status: Status | null | undefined): boolean {
  return status?.job?.state === "running";
}

// Analysis follows the live cloud but never while a sweep is running (the
// floor fit on a partial sweep is nonsense): refresh on mount when stale, on
// every kind-3 grid frame that arrives with no job running (sweep end, session
// load), and when a job stops after a grid frame arrived during it (the mock
// sends the final grid before the job's "done"). Returns the unsubscribe.
export function startOpsAutoRefresh(): () => void {
  let pending = false;
  const st = useStore.getState();
  if (!jobRunning(st.status) && opsStale()) void refreshOps();
  return useStore.subscribe((s, prev) => {
    const wasRunning = jobRunning(prev.status);
    const isRunning = jobRunning(s.status);
    if (s.grid !== prev.grid && s.grid) {
      if (isRunning) pending = true;
      else {
        pending = false;
        scheduleOpsRefresh();
      }
    } else if (wasRunning && !isRunning && pending) {
      pending = false;
      scheduleOpsRefresh();
    }
  });
}

// What the top of the tab should say about scanning: the backend's
// findings.scan when it is there, otherwise status.job.
export function scanState(findings: Findings | null, status: Status | null): { scanning: boolean; step: number; n_steps: number } {
  const job = status?.job;
  if (job && job.state === "running") return { scanning: true, step: job.step, n_steps: job.n_steps };
  const sc = findings?.scan;
  if (sc && sc.in_progress) return { scanning: true, step: sc.step ?? 0, n_steps: sc.n_steps ?? 0 };
  return { scanning: false, step: 0, n_steps: 0 };
}

// ---- route ----

export async function planRoute(from: XY, to: XY, source: "manual" | "assistant" = "manual"): Promise<void> {
  const s = useOps.getState();
  useOps.setState({ routeBusy: true, routeError: null, route: null, routeEnds: { from, to }, routeSource: source });
  try {
    const r = await api.opsRoute({ from_xy: from, to_xy: to, cell: s.cell, max_step: s.maxStep, max_slope: s.maxSlope });
    useOps.setState({ route: r, routeBusy: false });
  } catch (e) {
    useOps.setState({ routeBusy: false, routeError: errorText(e) });
    clientLog("error", `ops/route: ${errorText(e)}`);
  }
}

// ---- helpers ----

export function findingById(f: Findings, id: string): { kind: FindingKind; item: Finding } | null {
  for (const kind of FINDING_KINDS) {
    const item = f[kind].find((x) => x.id === id);
    if (item) return { kind, item };
  }
  return null;
}

export function isRouteResult(v: unknown): v is RouteResult {
  return !!v && typeof v === "object" && typeof (v as { found?: unknown }).found === "boolean";
}

// Whether the decisions model (ask, triage) can be used. The status object may
// carry per-model availability; otherwise the top-level flag decides.
export function decisionsAvailable(s: AiStatus | null): boolean {
  if (!s) return false;
  if (s.decisions && typeof s.decisions.available === "boolean") return s.decisions.available;
  return s.available;
}

export function nextStationText(f: Findings | null): string {
  const ns = f?.next_station;
  if (!f || f.grid.w === 0 || !ns) return "–";
  if (ns.bearing_deg === null || ns.bearing_deg === undefined) return "rotate scanner 90°";
  return `${ns.distance_m.toFixed(1)} m toward ${Math.round(ns.bearing_deg)}°`;
}
