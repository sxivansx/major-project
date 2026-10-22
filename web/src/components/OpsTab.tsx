import { useEffect, useRef, useState } from "react";
import { errorText } from "../api/rest";
import { rampCssGradient } from "../lib/colormap";
import { fmt, fmtDuration, fmtTime } from "../lib/format";
import { startSweep, stopSweep } from "../store/actions";
import { opsStale, refreshOps, scheduleOpsRefresh, useOps, type LayerToggles } from "../store/ops";
import { useStore } from "../store/store";
import { HATCH_COLOUR, OpsMap, TRAV_RGB, heightRange, type OpsMapHandle } from "./OpsMap";
import { OpsFindings } from "./OpsFindings";

// The Operator tab: what is out there, where could someone be, can we get
// there, what have we not seen. Map left, findings right, actions under the map.

const TOGGLES: { k: keyof LayerToggles; label: string }[] = [
  { k: "terrain", label: "terrain" },
  { k: "traversability", label: "traversability" },
  { k: "shadow", label: "shadow" },
  { k: "blind", label: "blind" },
  { k: "findings", label: "findings" },
  { k: "station", label: "next station" },
];

function download(blob: Blob, name: string) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

function stamp(): string {
  return new Date().toISOString().replace(/[-:]/g, "").slice(0, 15).replace("T", "-");
}

function rgb(c: [number, number, number]): string {
  return `rgba(${c[0]},${c[1]},${c[2]},0.7)`;
}

export function OpsTab() {
  const status = useStore((s) => s.status);
  const findings = useOps((s) => s.findings);
  const layers = useOps((s) => s.layers);
  const loading = useOps((s) => s.loading);
  const updatedAt = useOps((s) => s.updatedAt);
  const route = useOps((s) => s.route);
  const routeError = useOps((s) => s.routeError);
  const routeBusy = useOps((s) => s.routeBusy);
  const routeMode = useOps((s) => s.routeMode);
  const routeStart = useOps((s) => s.routeStart);
  const routeSource = useOps((s) => s.routeSource);
  const toggles = useOps((s) => s.toggles);
  const setToggle = useOps((s) => s.setToggle);
  const setRouteMode = useOps((s) => s.setRouteMode);
  const clearRoute = useOps((s) => s.clearRoute);
  const cell = useOps((s) => s.cell);
  const maxStep = useOps((s) => s.maxStep);
  const maxSlope = useOps((s) => s.maxSlope);

  const mapRef = useRef<OpsMapHandle>(null);
  const [scanErr, setScanErr] = useState<string | null>(null);
  const [scanBusy, setScanBusy] = useState(false);

  // analysis follows the live grid: refresh on mount when stale, then on
  // every kind-3 frame, at most once per 2 s
  useEffect(() => {
    if (opsStale()) void refreshOps();
    return useStore.subscribe((s, prev) => {
      if (s.grid !== prev.grid && s.grid) scheduleOpsRefresh();
    });
  }, []);

  const alarms = status?.alarms ?? [];
  const job = status?.job ?? null;
  const running = job?.state === "running";
  const ready = !!status?.lidar.connected && !!status?.servo.connected;
  const pct = job && job.n_steps > 0 ? Math.min(100, (100 * job.step) / job.n_steps) : 0;

  const scan = async () => {
    setScanErr(null);
    setScanBusy(true);
    try {
      if (running) await stopSweep();
      else await startSweep();
    } catch (e) {
      setScanErr(errorText(e));
    } finally {
      setScanBusy(false);
    }
  };

  const report = async () => {
    if (!findings) return;
    const t = stamp();
    const png = await mapRef.current?.toBlob();
    if (png) download(png, `operator-map-${t}.png`);
    const json = JSON.stringify(
      { generated: new Date().toISOString(), params: { cell_m: cell, max_step_m: maxStep, max_slope_deg: maxSlope }, findings, route },
      null,
      2,
    );
    download(new Blob([json], { type: "application/json" }), `findings-${t}.json`);
  };

  let routeText: React.ReactNode = null;
  if (routeMode) routeText = <span className="dim">{routeStart ? "click the goal on the map" : "click the start on the map"}</span>;
  else if (routeBusy) routeText = <span className="dim">planning route</span>;
  else if (routeError) routeText = <span className="err">route: {routeError}</span>;
  else if (route) {
    if (route.found)
      routeText = (
        <span className={route.verdict === "passable" ? "ok" : "warn"}>
          route {fmt(route.length_m, 1)} m · {route.verdict} · max step {fmt(route.max_step_m, 2)} m
          {route.start_snapped_m > 0 ? ` · start snapped ${fmt(route.start_snapped_m, 1)} m` : ""}
          {routeSource === "assistant" ? " · from the assistant" : ""}
        </span>
      );
    else routeText = <span className="err">no route: {route.reason}</span>;
  }

  const range = layers ? heightRange(layers) : null;

  return (
    <div className="ops-grid">
      <section className="panel ops-main">
        <div className="panel-head">
          <span>Operator map</span>
          <span className="sub">
            {findings
              ? `${findings.grid.w}×${findings.grid.h} @ ${+findings.grid.cell_m.toFixed(3)} m` + (updatedAt ? ` · ${fmtTime(updatedAt)}` : "")
              : "no analysis"}
            {loading ? " · analysing" : ""}
          </span>
        </div>
        {alarms.length > 0 && <div className="ops-alarm">{alarms.join(" · ")}</div>}
        <div className="canvas-toolbar">
          {TOGGLES.map((t) => (
            <label key={t.k} className="inline-check">
              <input type="checkbox" checked={toggles[t.k]} onChange={(e) => setToggle(t.k, e.target.checked)} />
              {t.label}
            </label>
          ))}
          <span className="spacer" />
          <button className="small" onClick={() => mapRef.current?.fit()} title="Fit the analysed area in the view">
            Fit
          </button>
        </div>
        <OpsMap ref={mapRef} />
        <div className="legend ops-legend">
          <span className="sw" style={{ background: rgb(TRAV_RGB[1]) }} /> passable
          <span className="sw" style={{ background: rgb(TRAV_RGB[2]) }} /> marginal
          <span className="sw" style={{ background: rgb(TRAV_RGB[3]) }} /> blocked
          <span className="sw hatch-diag" style={{ color: HATCH_COLOUR.shadow }} /> shadow
          <span className="sw hatch-horiz" style={{ color: HATCH_COLOUR.blind }} /> blind
          <span className="dim">unknown dark</span>
          <span className="spacer" />
          <span>{range ? fmt(range[0], 2) : "–"}</span>
          <div className="bar" style={{ background: rampCssGradient(), maxWidth: 120 }} />
          <span>{range ? fmt(range[1], 2) : "–"}</span>
          <span className="dim">height above floor (m)</span>
        </div>
        <div className="ops-actions">
          <button
            className={running ? "danger" : "primary"}
            disabled={scanBusy || (!running && !ready)}
            onClick={scan}
            title={ready ? "Start a sweep with the Scan job form in the sidebar" : "Connect lidar and servo first"}
          >
            {running ? "Stop" : "Scan"}
          </button>
          {job && running && (
            <>
              <div className="progress" title={`${fmt(pct, 0)}%`}>
                <div style={{ width: `${pct}%` }} />
              </div>
              <span className="mono dim">
                step {job.step} of {job.n_steps} · φ {fmt(job.phi_deg, 0)}° · ETA {fmtDuration(job.eta_s)}
              </span>
            </>
          )}
          <button className={routeMode ? "toggled" : ""} onClick={() => setRouteMode(!routeMode)} disabled={!layers}>
            Route
          </button>
          <button onClick={clearRoute} disabled={!route && !routeStart && !routeError}>
            Clear route
          </button>
          <button onClick={() => void refreshOps()} disabled={loading}>
            Refresh
          </button>
          <button onClick={() => void report()} disabled={!findings} title="Download the map as PNG and the findings as JSON">
            Report
          </button>
          <span className="spacer" />
          <span className="mono ops-route-text">{routeText}</span>
          {scanErr && <span className="err mono">{scanErr}</span>}
        </div>
      </section>
      <div className="ops-right">
        <OpsFindings />
      </div>
    </div>
  );
}
