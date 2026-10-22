// Actions shared by more than one panel. The Scan job panel and the operator
// action bar both start and stop sweeps from the same form in the store.

import { api, errorText } from "../api/rest";
import type { JobRequest } from "../api/types";
import { cloudClear } from "./cloud";
import { clientLog, useStore } from "./store";

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

export async function stopSweep(): Promise<void> {
  useStore.getState().setStatus(await api.jobStop());
}
