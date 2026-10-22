import { cloud, cloudClear } from "../store/cloud";
import { useStore } from "../store/store";
import { fmtInt } from "../lib/format";
import { CloudView } from "./CloudView";
import { HeightMapPanel } from "./HeightMapPanel";

export function CloudTab() {
  const cloudCount = useStore((s) => s.cloudCount);
  const setCloud = useStore((s) => s.setCloud);
  const showSurface = useStore((s) => s.showSurface);
  const showHelpers = useStore((s) => s.showHelpers);
  const pointSize = useStore((s) => s.pointSize);
  const colorZ = useStore((s) => s.colorZ);
  const surfaceMaxStep = useStore((s) => s.surfaceMaxStep);
  const setUi = useStore((s) => s.setUi);
  const sessionLoaded = useStore((s) => s.status?.session_loaded ?? null);

  const setZ = (i: 0 | 1, v: string) => {
    const n = Number(v);
    if (!Number.isFinite(n)) return;
    const next: [number, number] = [colorZ[0], colorZ[1]];
    next[i] = n;
    setUi({ colorZ: next });
  };

  return (
    <div className="cloud-grid">
      <section className="panel">
        <div className="panel-head">
          <span>3D point cloud</span>
          <span className="sub">
            {fmtInt(cloudCount)} pts
            {sessionLoaded ? ` · session ${sessionLoaded}` : ""}
            {cloud.dropped > 0 ? ` · ${fmtInt(cloud.dropped)} dropped (buffer full)` : ""}
          </span>
        </div>
        <div className="canvas-toolbar">
          <label>
            Point px
            <input
              type="number"
              min={1}
              max={8}
              step={1}
              value={pointSize}
              onChange={(e) => setUi({ pointSize: Math.max(1, Math.min(8, Number(e.target.value) || 1)) })}
            />
          </label>
          <label>
            Colour z
            <input type="number" step={0.1} value={colorZ[0]} onChange={(e) => setZ(0, e.target.value)} />
            to
            <input type="number" step={0.1} value={colorZ[1]} onChange={(e) => setZ(1, e.target.value)} />
          </label>
          <button
            className="small"
            disabled={cloudCount === 0}
            title="Set the colour range to the z extent of the cloud"
            onClick={() => {
              if (Number.isFinite(cloud.zMin) && cloud.zMax > cloud.zMin)
                setUi({ colorZ: [+cloud.zMin.toFixed(2), +cloud.zMax.toFixed(2)] });
            }}
          >
            Fit
          </button>
          <label className="inline-check">
            <input type="checkbox" checked={showHelpers} onChange={(e) => setUi({ showHelpers: e.target.checked })} />
            grid and axes
          </label>
          <label className="inline-check">
            <input type="checkbox" checked={showSurface} onChange={(e) => setUi({ showSurface: e.target.checked })} />
            surface
          </label>
          <label title="Surface quads with a larger z spread are skipped (0 = draw all)">
            max step m
            <input
              type="number"
              min={0}
              step={0.1}
              value={surfaceMaxStep}
              disabled={!showSurface}
              onChange={(e) => setUi({ surfaceMaxStep: Math.max(0, Number(e.target.value) || 0) })}
            />
          </label>
          <span className="spacer" />
          <button
            className="small"
            disabled={cloudCount === 0}
            onClick={() => {
              cloudClear();
              setCloud(0, cloud.version);
            }}
          >
            Clear
          </button>
        </div>
        <div className="scene-wrap">
          <CloudView />
          <div className="scene-overlay">drag: orbit · right-drag: pan · wheel: zoom · x red, y green, z blue · blue dot: tilt axis</div>
        </div>
      </section>
      <HeightMapPanel />
    </div>
  );
}
