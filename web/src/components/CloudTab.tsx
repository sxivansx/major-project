import { useState } from "react";
import { api, errorText } from "../api/rest";
import { cloud } from "../store/cloud";
import { fmt, fmtInt } from "../lib/format";
import { clearCloud, startSweep } from "../store/actions";
import { useStore } from "../store/store";
import { CloudView } from "./CloudView";
import { HeightMapControls, HeightMapPanel, downloadHeightMapPng } from "./HeightMapPanel";
import { JobPanel } from "./JobPanel";
import { MountPanel } from "./MountPanel";
import { Shell, useAdvancedFlag, type BigNumber } from "./Shell";
import { useRoom } from "../store/room";
import { useRoomFloorZ } from "./RoomLayer";

// Point cloud and height map: the 3D view left, the 2.5D map right, the
// numbers from /api/metrics on top.

function CloudControls() {
  const showHelpers = useStore((s) => s.showHelpers);
  const pointSize = useStore((s) => s.pointSize);
  const colorZ = useStore((s) => s.colorZ);
  const surfaceMaxStep = useStore((s) => s.surfaceMaxStep);
  const cloudCount = useStore((s) => s.cloudCount);
  const setUi = useStore((s) => s.setUi);
  const setZ = (i: 0 | 1, v: string) => {
    const n = Number(v);
    if (!Number.isFinite(n)) return;
    const next: [number, number] = [colorZ[0], colorZ[1]];
    next[i] = n;
    setUi({ colorZ: next });
  };
  return (
    <section className="panel">
      <div className="panel-head">
        <span>3D view</span>
      </div>
      <div className="panel-body">
        <div className="field">
          <label>Point px</label>
          <input
            type="number"
            min={1}
            max={8}
            step={1}
            value={pointSize}
            onChange={(e) => setUi({ pointSize: Math.max(1, Math.min(8, Number(e.target.value) || 1)) })}
          />
        </div>
        <div className="field">
          <label>Colour z</label>
          <div className="row">
            <input type="number" step={0.1} value={colorZ[0]} onChange={(e) => setZ(0, e.target.value)} />
            <input type="number" step={0.1} value={colorZ[1]} onChange={(e) => setZ(1, e.target.value)} />
            <button
              disabled={cloudCount === 0}
              title="Set the colour range to the z extent of the cloud"
              onClick={() => {
                if (Number.isFinite(cloud.zMin) && cloud.zMax > cloud.zMin) setUi({ colorZ: [+cloud.zMin.toFixed(2), +cloud.zMax.toFixed(2)] });
              }}
            >
              Fit colours
            </button>
          </div>
        </div>
        <div className="field" title="Surface quads with a larger z spread are skipped (0 = draw all)">
          <label>Max step m</label>
          <input type="number" min={0} step={0.1} value={surfaceMaxStep} onChange={(e) => setUi({ surfaceMaxStep: Math.max(0, Number(e.target.value) || 0) })} />
        </div>
        <label className="inline-check">
          <input type="checkbox" checked={showHelpers} onChange={(e) => setUi({ showHelpers: e.target.checked })} />
          grid and axes
        </label>
        <RoomHeightField />
      </div>
    </section>
  );
}

// Scanner height above the floor for the room layer (blank = automatic).
function RoomHeightField() {
  const manual = useStore((s) => s.roomScannerH);
  const setUi = useStore((s) => s.setUi);
  const { source } = useRoomFloorZ();
  return (
    <div className="field" style={{ marginTop: 6 }} title="Where the room map's floor is drawn: blank uses the fitted floor or the findings, else 1 m">
      <label>Scanner above floor m</label>
      <input
        type="number"
        min={0.1}
        step={0.05}
        placeholder="auto"
        value={manual ?? ""}
        onChange={(e) => {
          const v = e.target.value === "" ? null : Number(e.target.value);
          setUi({ roomScannerH: v !== null && v > 0 ? v : null });
        }}
      />
      <span />
      <span className="hint" style={{ marginTop: 0 }}>
        room floor from: {source}
      </span>
    </div>
  );
}

export function CloudTab() {
  const [advanced, setAdvanced] = useAdvancedFlag("cloud");
  const status = useStore((s) => s.status);
  const sessions = useStore((s) => s.sessions);
  const cloudCount = useStore((s) => s.cloudCount);
  const metrics = useStore((s) => s.metrics);
  const grid = useStore((s) => s.grid);
  const showSurface = useStore((s) => s.showSurface);
  const cloudResetNonce = useStore((s) => s.cloudResetNonce);
  const showRoom = useStore((s) => s.showRoom);
  const wallH = useStore((s) => s.roomWallH);
  const roomGrid = useRoom((s) => s.grid);
  const haveRoom = !!roomGrid && roomGrid.w > 0 && roomGrid.h > 0;
  const [wallText, setWallText] = useState(String(wallH));
  const setUi = useStore((s) => s.setUi);
  const [scanErr, setScanErr] = useState<string | null>(null);
  const [scanBusy, setScanBusy] = useState(false);

  const job = status?.job ?? null;
  const running = job?.state === "running";
  const ready = !!status?.lidar.connected && !!status?.servo.connected;
  const sessionId = status?.session_loaded ?? null;
  const sessionName = sessionId ? (sessions.find((s) => s.id === sessionId)?.name ?? sessionId) : null;

  const numbers: BigNumber[] = [
    { label: "Points", value: fmtInt(cloudCount) },
    {
      label: "Coverage",
      value: metrics ? (
        <>
          {fmt(metrics.grid.coverage_pct, 0)}
          <small>% · {fmtInt(metrics.grid.filled)} cells</small>
        </>
      ) : (
        "–"
      ),
    },
    {
      label: "Floor RMS",
      value: metrics?.floor ? (
        <>
          {fmt(metrics.floor.rms_mm, 0)}
          <small>mm</small>
        </>
      ) : (
        "–"
      ),
    },
    { label: "Session", value: <span title={sessionId ?? ""}>{sessionName ?? "live"}</span> },
  ];

  const scan = async () => {
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

  return (
    <Shell
      numbers={numbers}
      numbersDim={running}
      main={
        <section className="panel ops-mappanel">
          <div className="panel-head">
            <span>3D point cloud</span>
            <span className="sub">
              {fmtInt(cloudCount)} pts
              {sessionId ? ` · session ${sessionId}` : ""}
              {cloud.dropped > 0 ? ` · ${fmtInt(cloud.dropped)} dropped (buffer full)` : ""}
            </span>
          </div>
          <div className="scene-wrap">
            <CloudView resetNonce={cloudResetNonce} />
            <div className="scene-overlay">drag: orbit · right-drag: pan · wheel: zoom · x red, y green, z blue · blue dot: tilt axis</div>
          </div>
          {showRoom && haveRoom && (
            <div className="hint room-3d-hint">Walls are the room map raised to an assumed height. Real heights need a tilt sweep: the servo must move.</div>
          )}
        </section>
      }
      side={<HeightMapPanel />}
      actions={
        <>
          <button className="primary" disabled={!ready || running || scanBusy} onClick={() => void scan()} title="Sweep with the Scan job settings">
            Scan
          </button>
          {running && job && (
            <span className="mono dim">
              scanning… step {job.step} of {job.n_steps}
            </span>
          )}
          {scanErr && <span className="err mono">{scanErr}</span>}
          <button
            onClick={() => setUi({ cloudResetNonce: cloudResetNonce + 1 })}
            title={cloudCount === 0 && showRoom && haveRoom ? "Frame the room map" : "Camera back to the default view"}
          >
            Fit
          </button>
          <button className={showSurface ? "toggled" : ""} onClick={() => setUi({ showSurface: !showSurface })}>
            Surface {showSurface ? "on" : "off"}
          </button>
          <button
            className={showRoom && haveRoom ? "toggled" : ""}
            disabled={!haveRoom}
            title={haveRoom ? "The handheld room map (Room map tab) in 3D" : "No room map yet: map one on the Room map tab"}
            onClick={() => setUi({ showRoom: !showRoom })}
          >
            Room {showRoom && haveRoom ? "on" : "off"}
          </button>
          {showRoom && haveRoom && (
            <label className="action-field" title="The room map has no heights: walls are drawn this tall">
              Wall height (assumed)
              <input
                type="number"
                min={0.2}
                max={10}
                step={0.1}
                value={wallText}
                onChange={(e) => {
                  setWallText(e.target.value);
                  const v = Number(e.target.value);
                  if (v > 0 && v <= 10) setUi({ roomWallH: v });
                }}
              />
              <span className="dim">m</span>
            </label>
          )}
          <button disabled={!grid} onClick={() => downloadHeightMapPng()} title="Download the height map as a PNG">
            PNG
          </button>
          {sessionId ? (
            <a href={api.sessionExportUrl(sessionId, "ply")} download>
              <button title={`Export session ${sessionId} as ASCII PLY`}>PLY</button>
            </a>
          ) : (
            <button disabled title="Load a session first">
              PLY
            </button>
          )}
          <button disabled={cloudCount === 0 || running} onClick={clearCloud}>
            Clear
          </button>
        </>
      }
      advanced={
        <div className="ops-advanced-grid cols-3">
          <div className="ops-advanced-col">
            <CloudControls />
            <HeightMapControls />
          </div>
          <div className="ops-advanced-col">
            <JobPanel />
          </div>
          <div className="ops-advanced-col">
            <MountPanel />
          </div>
        </div>
      }
      advancedOpen={advanced}
      onAdvanced={setAdvanced}
    />
  );
}
