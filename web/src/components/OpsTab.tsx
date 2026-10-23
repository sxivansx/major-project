import { useEffect, useRef, useState } from "react";
import { errorText } from "../api/rest";
import { fmt } from "../lib/format";
import { startSweep } from "../store/actions";
import { loadAiStatus, nextStationText, scanState, startOpsAutoRefresh, useOps } from "../store/ops";
import { useStore } from "../store/store";
import { OpsAdvanced } from "./OpsAdvanced";
import { OpsMap, type OpsMapHandle } from "./OpsMap";
import { OpsSummary } from "./OpsSummary";
import { Shell, type BigNumber } from "./Shell";
import { LOW_CONFIDENCE, RECENT_MAP_S, bearingText, coreNowSec, useAudio, useCoreNow } from "../store/audio";

// The Operator tab, simple view: three numbers, one map, what we have, three
// buttons. Everything else sits behind Advanced.

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

// "no passable route: goal blocked" rather than the reason repeating the prefix
function routeFailText(reason: string): string {
  const r = reason.replace(/^no passable route\s*/i, "").replace(/^\((.*)\)$/, "$1").trim();
  return "no passable route" + (r ? ": " + r : "");
}

export function OpsTab() {
  const status = useStore((s) => s.status);
  const findings = useOps((s) => s.findings);
  const layers = useOps((s) => s.layers);
  const loading = useOps((s) => s.loading);
  const error = useOps((s) => s.error);
  const route = useOps((s) => s.route);
  const routeError = useOps((s) => s.routeError);
  const routeBusy = useOps((s) => s.routeBusy);
  const routeMode = useOps((s) => s.routeMode);
  const routeStart = useOps((s) => s.routeStart);
  const advanced = useOps((s) => s.advanced);
  const setAdvanced = useOps((s) => s.setAdvanced);
  const setRouteMode = useOps((s) => s.setRouteMode);
  const clearRoute = useOps((s) => s.clearRoute);
  const cell = useOps((s) => s.cell);
  const maxStep = useOps((s) => s.maxStep);
  const maxSlope = useOps((s) => s.maxSlope);

  const soundEvents = useAudio((s) => s.events);
  const latestSound = soundEvents[0] ?? null;
  const soundNow = useCoreNow(!!latestSound && coreNowSec() - latestSound.t < RECENT_MAP_S + 2);
  const soundFresh = !!latestSound && soundNow - latestSound.t < RECENT_MAP_S;
  const mapRef = useRef<OpsMapHandle>(null);
  const [scanErr, setScanErr] = useState<string | null>(null);
  const [scanBusy, setScanBusy] = useState(false);

  useEffect(() => {
    void loadAiStatus();
    return startOpsAutoRefresh();
  }, []);

  const alarms = status?.alarms ?? [];
  const connected = !!status?.lidar.connected && !!status?.servo.connected;
  const haveSession = !!status?.session_loaded;
  const scan = scanState(findings, status);
  const scanningText = scan.scanning ? `scanning… step ${scan.step} of ${scan.n_steps}` : null;
  const haveAnalysis = !!findings && findings.grid.w > 0;

  const doScan = async () => {
    setScanErr(null);
    setScanBusy(true);
    try {
      await startSweep();
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

  // Route button: Route -> Cancel while picking -> Clear route once there is a result
  const routeLabel = route || routeError ? "Clear route" : routeMode ? "Cancel route" : "Route";
  const onRoute = () => {
    if (route || routeError) clearRoute();
    else setRouteMode(!routeMode);
  };
  let routeLine: React.ReactNode = null;
  if (routeMode) routeLine = <span className="dim">{routeStart ? "now click the goal" : "click the start on the map"}</span>;
  else if (routeBusy) routeLine = <span className="dim">planning</span>;
  else if (routeError) routeLine = <span className="err">route: {routeError}</span>;
  else if (route) {
    routeLine = route.found ? (
      <span className={route.verdict === "passable" ? "ok" : "warn"}>
        {fmt(route.length_m, 1)} m, {route.verdict}
      </span>
    ) : (
      <span className="err">{routeFailText(route.reason)}</span>
    );
  }

  const numbers: BigNumber[] = [
    {
      label: "Seen",
      value: haveAnalysis ? (
        <>
          {fmt(findings!.coverage.coverage_pct, 0)}
          <small>%</small>
        </>
      ) : (
        "–"
      ),
    },
    { label: "Possible voids", value: haveAnalysis ? findings!.voids.length : "–" },
    { label: "Next", value: nextStationText(findings), className: "ops-next" },
    {
      label: "Sound",
      value: latestSound ? (latestSound.bearing_deg === null ? "unresolved" : bearingText(latestSound)) : "–",
      // grey when the direction is weak or older than the wedges on the map
      className: "ops-next" + (latestSound && (latestSound.confidence < LOW_CONFIDENCE || !soundFresh || latestSound.bearing_deg === null) ? " dim" : ""),
    },
  ];

  return (
    <Shell
      numbers={numbers}
      numbersDim={scan.scanning}
      main={
        <section className="panel ops-mappanel">
          {alarms.length > 0 && <div className="ops-alarm">{alarms.join(" · ")}</div>}
          {error && <div className="ops-alarm">analysis: {error}</div>}
          {connected || haveSession ? (
            <>
              <OpsMap ref={mapRef} scanningText={scanningText} />
              <div className="legend ops-legend">
                {advanced ? (
                  <>
                    <span className="sw sw-passable" /> passable
                    <span className="sw sw-marginal" /> marginal
                    <span className="sw sw-blocked" /> blocked
                    <span className="sw hatch-diag" /> shadow
                    <span className="sw hatch-horiz" /> blind
                    <span className="dim">unknown dark</span>
                  </>
                ) : (
                  <>
                    <span className="sw sw-passable" /> can walk
                    <span className="sw sw-blocked" /> blocked
                    <span className="sw hatch-diag" /> not seen yet
                    <span className="sw sw-void" /> V1 possible void
                    <span className="dim">arrow: next station</span>
                    {soundFresh && <span className="dim">blue wedge: sound direction</span>}
                  </>
                )}
                <span className="spacer" />
                <button className="small" onClick={() => mapRef.current?.fit()} title="Fit the scanned area in the view">
                  Fit
                </button>
              </div>
            </>
          ) : (
            <div className="empty ops-connect">Connect the LiDAR and servo on the Live scan tab</div>
          )}
        </section>
      }
      side={<OpsSummary />}
      actions={
        <>
          <button className="primary" disabled={!connected || scan.scanning || scanBusy} onClick={() => void doScan()} title="Sweep with the Scan job settings">
            Scan
          </button>
          {scanningText && <span className="mono dim">{scanningText}</span>}
          {scanErr && <span className="err mono">{scanErr}</span>}
          <button className={routeMode ? "toggled" : ""} disabled={!layers || layers.w === 0} onClick={onRoute}>
            {routeLabel}
          </button>
          <span className="mono ops-route-text">{routeLine}</span>
          <button disabled={!haveAnalysis} onClick={() => void report()} title="Download the map as PNG and the findings as JSON">
            Report
          </button>
          {loading && <span className="dim">analysing</span>}
        </>
      }
      advanced={<OpsAdvanced />}
      advancedOpen={advanced}
      onAdvanced={setAdvanced}
    />
  );
}
