// Actions shared by more than one place in the UI (the action bars, the side
// columns and the Advanced drawers all reach the same server calls). Each one
// updates the store, logs failures and rethrows so the caller can show the
// message inline.

import { api, errorText } from "../api/rest";
import type { JobRequest } from "../api/types";
import { cloudClear } from "./cloud";
import { clientLog, useStore } from "./store";

async function statusCall(name: string, fn: () => Promise<import("../api/types").Status>): Promise<void> {
  try {
    useStore.getState().setStatus(await fn());
  } catch (e) {
    clientLog("error", `${name}: ${errorText(e)}`);
    throw e;
  }
}

// ---- sweeps ----

export function sweepSteps(f: JobRequest): number {
  if (!(f.phi_step > 0)) return 0;
  return Math.floor(Math.abs(f.phi_end - f.phi_start) / f.phi_step + 1e-9) + 1;
}

// Starts a sweep with the current job form. Throws on failure after logging.
export async function startSweep(): Promise<number> {
  const st = useStore.getState();
  const form = st.jobForm;
  try {
    cloudClear();
    st.setCloud(0, 0);
    const r = await api.jobStart(form);
    clientLog("info", `job ${r.job_id} "${form.name}" started`);
    return r.job_id;
  } catch (e) {
    clientLog("error", `job/start: ${errorText(e)}`);
    throw e;
  }
}

export const stopSweep = () => statusCall("job/stop", () => api.jobStop());

// ---- connections ----

export const connectLidar = (body: { source: "s2" | "sim"; port: string; baud: number }) =>
  statusCall("lidar/connect", () => api.lidarConnect(body));
export const disconnectLidar = () => statusCall("lidar/disconnect", () => api.lidarDisconnect());
export const connectServo = (port: string) => statusCall("servo/connect", () => api.servoConnect(port));
export const disconnectServo = () => statusCall("servo/disconnect", () => api.servoDisconnect());

export async function refreshPorts(): Promise<void> {
  try {
    useStore.getState().setPorts(await api.ports());
  } catch (e) {
    clientLog("error", `GET /api/ports: ${errorText(e)}`);
    throw e;
  }
}

// ---- servo ----

export const servoMove = (body: { servo_deg: number } | { phi_deg: number }) =>
  statusCall("servo/move", () => api.servoMove(body));
export const servoHome = () => statusCall("servo/home", () => api.servoHome());

// ---- height map ----

// PUT the grid part of the config from the form, then GET the grid with it.
export async function applyGridForm(): Promise<void> {
  const st = useStore.getState();
  const f = st.gridForm;
  const q = {
    cell: Number(f.cell),
    layer: f.layer,
    z_min: f.zMin === "" ? null : Number(f.zMin),
    z_max: f.zMax === "" ? null : Number(f.zMax),
  };
  try {
    if (st.config) {
      const next = {
        ...st.config,
        grid: {
          cell_m: q.cell,
          layer: f.layer,
          z_min_m: q.z_min ?? st.config.grid.z_min_m,
          z_max_m: q.z_max ?? st.config.grid.z_max_m,
        },
      };
      st.setConfig(await api.putConfig(next));
      st.setUi({ colorZ: [next.grid.z_min_m, next.grid.z_max_m] });
    }
    st.setGrid(await api.grid(q));
  } catch (e) {
    clientLog("error", `grid: ${errorText(e)}`);
    throw e;
  }
}

export function clearCloud(): void {
  cloudClear();
  useStore.getState().setCloud(0, 0);
}
