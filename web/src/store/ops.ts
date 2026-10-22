// State for the Operator tab: analysis parameters, the findings JSON and the
// kind-4 layers frame from /api/ops/*, the current route, the selection and
// view requests for the map. Kept apart from the main store because nothing
// else needs it and the frames are large.

import { create } from "zustand";
import { api, errorText } from "../api/rest";
import type { Finding, FindingKind, Findings, OpsLayersFrame, RouteResult } from "../api/types";
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
  toggles: LayerToggles;

  setParams: (p: Partial<Pick<OpsState, "cell" | "maxStep" | "maxSlope">>) => void;
  setToggle: (k: keyof LayerToggles, v: boolean) => void;
  select: (id: string | null) => void;
  focusOn: (x: number, y: number) => void;
  fit: () => void;
  setRouteMode: (on: boolean) => void;
  setRouteStart: (p: XY | null) => void;
  setRoute: (r: RouteResult | null, source: "manual" | "assistant", ends?: { from: XY; to: XY }) => void;
  clearRoute: () => void;
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
  toggles: { terrain: true, traversability: true, shadow: true, blind: true, findings: true, station: true },

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
}));

// ---- refresh (findings + layers) ----

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
  } catch (e) {
    useOps.setState({ loading: false, error: errorText(e) });
    clientLog("error", `ops: ${errorText(e)}`);
  } finally {
    inFlight = false;
  }
  if (runAgain) {
    runAgain = false;
    await refreshOps();
  }
}

// At most one refresh per REFRESH_MIN_MS, trailing: a burst of grid frames
// during a sweep becomes one request every 2 s plus one at the end.
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
