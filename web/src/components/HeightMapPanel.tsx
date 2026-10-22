import { useEffect, useMemo, useRef, useState } from "react";
import { errorText } from "../api/rest";
import { GRID_LAYERS, type GridFrame, type GridLayer } from "../api/types";
import { applyGridForm } from "../store/actions";
import { useStore } from "../store/store";
import { rampCssGradient, rampRgb } from "../lib/colormap";
import { fmt, fmtInt } from "../lib/format";
import { MetricsLine } from "./MetricsLine";

// Top-down 2.5D height map on a canvas. Row index = y, so grid row 0 is the
// lowest y and is drawn at the bottom (+y up on screen, +x right). The query
// controls (cell, layer, z range, Apply) are a separate component for the
// Advanced drawer; the PNG download is an action-bar function.

function colourRange(g: GridFrame, fitToData: boolean): [number, number] {
  if (g.layer === "count" || fitToData || !(g.z_max_m > g.z_min_m)) {
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < g.data.length; i++) {
      const v = g.data[i];
      if (Number.isFinite(v)) {
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    }
    if (!Number.isFinite(lo)) return [0, 1];
    if (g.layer === "count") lo = 0;
    if (hi <= lo) hi = lo + 1;
    return [lo, hi];
  }
  return [g.z_min_m, g.z_max_m];
}

function renderGrid(g: GridFrame, range: [number, number]): HTMLCanvasElement {
  const off = document.createElement("canvas");
  off.width = g.w;
  off.height = g.h;
  const ctx = off.getContext("2d")!;
  const img = ctx.createImageData(g.w, g.h);
  const px = img.data;
  const [lo, hi] = range;
  const inv = 1 / (hi - lo);
  const rgb: [number, number, number] = [0, 0, 0];
  for (let iy = 0; iy < g.h; iy++) {
    const row = g.h - 1 - iy; // flip so +y is up
    for (let ix = 0; ix < g.w; ix++) {
      const v = g.data[iy * g.w + ix];
      const o = (row * g.w + ix) * 4;
      if (!Number.isFinite(v)) {
        px[o + 3] = 0; // NaN = transparent
        continue;
      }
      rampRgb((v - lo) * inv, rgb);
      px[o] = rgb[0] * 255;
      px[o + 1] = rgb[1] * 255;
      px[o + 2] = rgb[2] * 255;
      px[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  return off;
}

interface Fit {
  ox: number;
  oy: number;
  scale: number; // css px per metre
}

function fitGrid(g: GridFrame, cssW: number, cssH: number): Fit {
  const gw = g.w * g.cell_m;
  const gh = g.h * g.cell_m;
  const pad = 24;
  const scale = Math.max(1e-6, Math.min((cssW - pad * 2) / gw, (cssH - pad * 2) / gh));
  const ox = (cssW - gw * scale) / 2;
  const oy = (cssH - gh * scale) / 2;
  return { ox, oy, scale };
}

// The mounted height map canvas, for the PNG action.
const canvasRegistry: { el: HTMLCanvasElement | null } = { el: null };

export function downloadHeightMapPng(): boolean {
  const c = canvasRegistry.el;
  const grid = useStore.getState().grid;
  if (!c || !grid) return false;
  c.toBlob((blob) => {
    if (!blob) return;
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `heightmap-${grid.layer}-${grid.cell_m}m.png`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  });
  return true;
}

export function HeightMapPanel() {
  const grid = useStore((s) => s.grid);
  const fitToData = useStore((s) => s.fitColours);
  const [hover, setHover] = useState<{ x: number; y: number; v: number; ix: number; iy: number } | null>(null);

  const range = useMemo(() => (grid ? colourRange(grid, fitToData) : ([0, 1] as [number, number])), [grid, fitToData]);
  const image = useMemo(() => (grid ? renderGrid(grid, range) : null), [grid, range]);

  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const fitRef = useRef<Fit | null>(null);

  useEffect(() => {
    canvasRegistry.el = canvasRef.current;
    return () => {
      if (canvasRegistry.el === canvasRef.current) canvasRegistry.el = null;
    };
  }, []);

  useEffect(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const paint = () => {
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      const cssW = wrap.clientWidth;
      const cssH = wrap.clientHeight;
      canvas.width = Math.max(1, Math.round(cssW * dpr));
      canvas.height = Math.max(1, Math.round(cssH * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = "#1c1c1c";
      ctx.fillRect(0, 0, cssW, cssH);
      ctx.font = "11px ui-monospace, Menlo, monospace";
      ctx.fillStyle = "#8c8c8c";
      if (!grid || !image) {
        fitRef.current = null;
        ctx.textAlign = "center";
        ctx.fillText("no grid yet: run a sweep or load a session", cssW / 2, cssH / 2);
        return;
      }
      const fit = fitGrid(grid, cssW, cssH);
      fitRef.current = fit;
      const gw = grid.w * grid.cell_m * fit.scale;
      const gh = grid.h * grid.cell_m * fit.scale;
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(image, fit.ox, fit.oy, gw, gh);
      ctx.strokeStyle = "#454545";
      ctx.strokeRect(fit.ox + 0.5, fit.oy + 0.5, gw, gh);
      // 1 m ticks along the edges
      ctx.strokeStyle = "#383838";
      ctx.textAlign = "center";
      const x1 = grid.x0_m + grid.w * grid.cell_m;
      const y1 = grid.y0_m + grid.h * grid.cell_m;
      for (let x = Math.ceil(grid.x0_m); x <= x1; x++) {
        const sx = fit.ox + (x - grid.x0_m) * fit.scale;
        ctx.beginPath();
        ctx.moveTo(sx, fit.oy + gh);
        ctx.lineTo(sx, fit.oy + gh + 4);
        ctx.stroke();
        ctx.fillText(`${x}`, sx, fit.oy + gh + 12);
      }
      ctx.textAlign = "right";
      for (let y = Math.ceil(grid.y0_m); y <= y1; y++) {
        const sy = fit.oy + gh - (y - grid.y0_m) * fit.scale;
        ctx.beginPath();
        ctx.moveTo(fit.ox - 4, sy);
        ctx.lineTo(fit.ox, sy);
        ctx.stroke();
        ctx.fillText(`${y}`, fit.ox - 6, sy + 4);
      }
      // origin marker
      const ox0 = fit.ox + (0 - grid.x0_m) * fit.scale;
      const oy0 = fit.oy + gh - (0 - grid.y0_m) * fit.scale;
      if (ox0 >= fit.ox && ox0 <= fit.ox + gw && oy0 >= fit.oy && oy0 <= fit.oy + gh) {
        ctx.strokeStyle = "#4d9de0";
        ctx.beginPath();
        ctx.moveTo(ox0 - 5, oy0);
        ctx.lineTo(ox0 + 5, oy0);
        ctx.moveTo(ox0, oy0 - 5);
        ctx.lineTo(ox0, oy0 + 5);
        ctx.stroke();
      }
      if (hover) {
        const hx = fit.ox + (hover.ix * grid.cell_m) * fit.scale;
        const hy = fit.oy + gh - ((hover.iy + 1) * grid.cell_m) * fit.scale;
        const cs = Math.max(2, grid.cell_m * fit.scale);
        ctx.strokeStyle = "#ffffff";
        ctx.strokeRect(hx + 0.5, hy + 0.5, cs, cs);
      }
    };
    paint();
    const ro = new ResizeObserver(paint);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [grid, image, hover]);

  const onMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const fit = fitRef.current;
    if (!grid || !fit) return setHover(null);
    const rect = e.currentTarget.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const gh = grid.h * grid.cell_m * fit.scale;
    const ix = Math.floor((sx - fit.ox) / fit.scale / grid.cell_m);
    const iy = Math.floor((fit.oy + gh - sy) / fit.scale / grid.cell_m);
    if (ix < 0 || iy < 0 || ix >= grid.w || iy >= grid.h) return setHover(null);
    const v = grid.data[iy * grid.w + ix];
    setHover({
      ix,
      iy,
      x: grid.x0_m + (ix + 0.5) * grid.cell_m,
      y: grid.y0_m + (iy + 0.5) * grid.cell_m,
      v,
    });
  };

  let filled = 0;
  if (grid) for (let i = 0; i < grid.data.length; i++) if (Number.isFinite(grid.data[i])) filled++;

  return (
    <section className="panel ops-right">
      <div className="panel-head">
        <span>2.5D height map</span>
        <span className="sub">
          {grid ? `${grid.w}×${grid.h} @ ${+grid.cell_m.toFixed(3)} m · ${grid.layer} · ${fmtInt(filled)} cells` : "no grid"}
        </span>
      </div>
      <div className="heightmap-wrap" ref={wrapRef}>
        <canvas ref={canvasRef} onMouseMove={onMove} onMouseLeave={() => setHover(null)} />
        {hover && (
          <div className="heightmap-readout">
            x {fmt(hover.x, 3)} y {fmt(hover.y, 3)}{" "}
            {grid?.layer === "count" ? `count ${fmtInt(hover.v)}` : `z ${Number.isFinite(hover.v) ? fmt(hover.v, 3) : "empty"}`}
          </div>
        )}
      </div>
      <div className="legend">
        <span>{fmt(range[0], grid?.layer === "count" ? 0 : 2)}</span>
        <div className="bar" style={{ background: rampCssGradient() }} />
        <span>{fmt(range[1], grid?.layer === "count" ? 0 : 2)}</span>
        <span className="dim">{grid?.layer === "count" ? "points per cell" : "z (m)"}</span>
      </div>
      <MetricsLine />
    </section>
  );
}

// Cell size, layer, z range and Apply (PUT /api/config then GET /api/grid),
// plus the colour options. Advanced drawer of the point cloud tab.
export function HeightMapControls() {
  const form = useStore((s) => s.gridForm);
  const fitColours = useStore((s) => s.fitColours);
  const showSurface = useStore((s) => s.showSurface);
  const setUi = useStore((s) => s.setUi);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const set = (p: Partial<typeof form>) => setUi({ gridForm: { ...form, ...p } });
  const apply = async () => {
    setErr(null);
    setBusy(true);
    try {
      await applyGridForm();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <section className="panel">
      <div className="panel-head">
        <span>Height map</span>
      </div>
      <div className="panel-body">
        <div className="field">
          <label>Cell m</label>
          <input type="number" step={0.01} min={0.01} value={form.cell} onChange={(e) => set({ cell: e.target.value })} />
        </div>
        <div className="field">
          <label>Layer</label>
          <select value={form.layer} onChange={(e) => set({ layer: e.target.value as GridLayer })}>
            {GRID_LAYERS.map((l) => (
              <option key={l} value={l}>
                {l}
              </option>
            ))}
          </select>
        </div>
        <div className="field">
          <label>z min / max</label>
          <div className="row">
            <input type="number" step={0.1} value={form.zMin} onChange={(e) => set({ zMin: e.target.value })} />
            <input type="number" step={0.1} value={form.zMax} onChange={(e) => set({ zMax: e.target.value })} />
          </div>
        </div>
        <div className="btn-row">
          <button className="primary" disabled={busy} onClick={() => void apply()} title="PUT /api/config then GET /api/grid">
            Apply
          </button>
        </div>
        {err && <div className="error">{err}</div>}
        <label className="inline-check" style={{ marginTop: 8 }} title="Colour range from the data instead of z min/max">
          <input type="checkbox" checked={fitColours} onChange={(e) => setUi({ fitColours: e.target.checked })} />
          fit colours to the data
        </label>
        <label className="inline-check" style={{ marginTop: 4 }}>
          <input type="checkbox" checked={showSurface} onChange={(e) => setUi({ showSurface: e.target.checked })} />
          surface in 3D
        </label>
      </div>
    </section>
  );
}
