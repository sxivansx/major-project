import { forwardRef, useEffect, useImperativeHandle, useMemo, useRef, useState } from "react";
import type { RoomFrame, RoomState } from "../api/types";
import { fmt } from "../lib/format";

// Handheld 2D room map: occupancy grid (kind 6) + walked path + pose.
// Map frame: origin where mapping started, +x = the LiDAR cable mark at the
// start, +y to its left; +y is drawn up. theta_deg is counter-clockwise from +x.
// Colours come from the theme variables, so a light theme inverts them.

export interface RoomMapHandle {
  toBlob(): Promise<Blob | null>;
  fit(): void;
}

interface View {
  cx: number;
  cy: number;
  scale: number; // css px per metre
}

const MIN_SCALE = 4;
const MAX_SCALE = 2000;
const DRAG_PX = 3;
const CELL_WORDS = ["unknown", "free", "occupied"];

function cssVar(name: string, fallback: string): string {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
}

function hexRgb(c: string): [number, number, number] {
  const m = c.match(/^#([0-9a-f]{6})$/i);
  if (!m) return [128, 128, 128];
  const n = parseInt(m[1], 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

function renderGrid(g: RoomFrame): HTMLCanvasElement | null {
  if (g.w === 0 || g.h === 0) return null;
  const free = hexRgb(cssVar("--bg-hover", "#333333"));
  const occ = hexRgb(cssVar("--text", "#d9d9d9"));
  const edge = hexRgb(cssVar("--warn", "#e0a040"));
  const off = document.createElement("canvas");
  off.width = g.w;
  off.height = g.h;
  const ctx = off.getContext("2d")!;
  const img = ctx.createImageData(g.w, g.h);
  const px = img.data;
  for (let iy = 0; iy < g.h; iy++) {
    const row = g.h - 1 - iy; // +y up
    for (let ix = 0; ix < g.w; ix++) {
      const v = g.cells[iy * g.w + ix];
      if (v === 0) continue; // unknown = background
      const c = v === 2 ? occ : v === 3 ? edge : free;
      const o = (row * g.w + ix) * 4;
      px[o] = c[0];
      px[o + 1] = c[1];
      px[o + 2] = c[2];
      px[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  return off;
}

type Bounds = [number, number, number, number];

function boundsOf(g: RoomFrame | null, room: RoomState | null): Bounds {
  if (g && g.w > 0) return [g.x0_m, g.y0_m, g.x0_m + g.w * g.cell_m, g.y0_m + g.h * g.cell_m];
  const p = room?.pose;
  return p ? [p.x - 3, p.y - 3, p.x + 3, p.y + 3] : [-3, -3, 3, 3];
}

function fitView(b: Bounds, W: number, H: number): View {
  const pad = 28;
  const gw = Math.max(0.5, b[2] - b[0]);
  const gh = Math.max(0.5, b[3] - b[1]);
  const scale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, Math.min((W - pad * 2) / gw, (H - pad * 2) / gh)));
  return { cx: (b[0] + b[2]) / 2, cy: (b[1] + b[3]) / 2, scale };
}

function gridStep(scale: number): number {
  if (scale >= 60) return 1;
  if (scale >= 24) return 2;
  if (scale >= 10) return 5;
  return 10;
}

export const RoomMap = forwardRef<RoomMapHandle, { grid: RoomFrame | null; room: RoomState | null }>(function RoomMap({ grid, room }, ref) {
  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [view, setView] = useState<View>({ cx: 0, cy: 0, scale: 0 });
  const viewRef = useRef(view);
  viewRef.current = view;
  const [hover, setHover] = useState<{ x: number; y: number; v: number | null } | null>(null);
  const dragRef = useRef<{ x: number; y: number; cx: number; cy: number; moved: boolean } | null>(null);
  const fitted = useRef("");
  const userMoved = useRef(false);

  const image = useMemo(() => (grid ? renderGrid(grid) : null), [grid]);

  const doFit = () => {
    const wrap = wrapRef.current;
    if (!wrap) return;
    const b = boundsOf(grid, room);
    setView(fitView(b, wrap.clientWidth, wrap.clientHeight));
    fitted.current = b.join(",");
    userMoved.current = false;
  };

  useImperativeHandle(ref, () => ({
    toBlob: () =>
      new Promise<Blob | null>((resolve) => {
        const c = canvasRef.current;
        if (!c) return resolve(null);
        c.toBlob((b) => resolve(b), "image/png");
      }),
    fit: doFit,
  }));

  // follow the map as it grows until the user pans or zooms
  useEffect(() => {
    const key = boundsOf(grid, room).join(",");
    if (fitted.current === "" || (!userMoved.current && key !== fitted.current)) doFit();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [grid]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      userMoved.current = true;
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      setView((v) => {
        const s0 = v.scale > 0 ? v.scale : 60;
        const s = Math.max(MIN_SCALE, Math.min(MAX_SCALE, s0 * Math.exp(-e.deltaY * 0.0015)));
        const wx = v.cx + (mx - rect.width / 2) / s0;
        const wy = v.cy - (my - rect.height / 2) / s0;
        return { cx: wx - (mx - rect.width / 2) / s, cy: wy + (my - rect.height / 2) / s, scale: s };
      });
    };
    canvas.addEventListener("wheel", onWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", onWheel);
  }, []);

  useEffect(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const paint = () => {
      const bg = cssVar("--bg", "#1c1c1c");
      const gridC = cssVar("--border", "#383838");
      const dim = cssVar("--text-dim", "#8c8c8c");
      const accent = cssVar("--accent", "#4d9de0");
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      const W = wrap.clientWidth;
      const H = wrap.clientHeight;
      canvas.width = Math.max(1, Math.round(W * dpr));
      canvas.height = Math.max(1, Math.round(H * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = bg;
      ctx.fillRect(0, 0, W, H);
      const v = view.scale > 0 ? view : fitView(boundsOf(grid, room), W, H);
      const sx = (x: number) => W / 2 + (x - v.cx) * v.scale;
      const sy = (y: number) => H / 2 - (y - v.cy) * v.scale;

      if (image && grid) {
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(image, sx(grid.x0_m), sy(grid.y0_m + grid.h * grid.cell_m), grid.w * grid.cell_m * v.scale, grid.h * grid.cell_m * v.scale);
      }

      // 1 m grid on top, faint, with labels
      const step = gridStep(v.scale);
      ctx.font = "11px ui-monospace, Menlo, monospace";
      ctx.lineWidth = 1;
      ctx.globalAlpha = 0.45;
      for (let x = Math.ceil((v.cx - W / 2 / v.scale) / step) * step; x <= v.cx + W / 2 / v.scale; x += step) {
        const px = Math.round(sx(x)) + 0.5;
        ctx.strokeStyle = gridC;
        ctx.beginPath();
        ctx.moveTo(px, 0);
        ctx.lineTo(px, H);
        ctx.stroke();
      }
      for (let y = Math.ceil((v.cy - H / 2 / v.scale) / step) * step; y <= v.cy + H / 2 / v.scale; y += step) {
        const py = Math.round(sy(y)) + 0.5;
        ctx.strokeStyle = gridC;
        ctx.beginPath();
        ctx.moveTo(0, py);
        ctx.lineTo(W, py);
        ctx.stroke();
      }
      ctx.globalAlpha = 1;
      ctx.fillStyle = dim;
      ctx.textAlign = "center";
      for (let x = Math.ceil((v.cx - W / 2 / v.scale) / step) * step; x <= v.cx + W / 2 / v.scale; x += step) ctx.fillText(`${x}`, sx(x), H - 4);
      ctx.textAlign = "left";
      for (let y = Math.ceil((v.cy - H / 2 / v.scale) / step) * step; y <= v.cy + H / 2 / v.scale; y += step) ctx.fillText(`${y}`, 4, sy(y) - 3);

      // start point
      ctx.strokeStyle = dim;
      ctx.beginPath();
      ctx.moveTo(sx(0) - 6, sy(0));
      ctx.lineTo(sx(0) + 6, sy(0));
      ctx.moveTo(sx(0), sy(0) - 6);
      ctx.lineTo(sx(0), sy(0) + 6);
      ctx.stroke();
      ctx.fillStyle = dim;
      ctx.fillText("start", sx(0) + 8, sy(0) + 14);

      // walked path
      const path = room?.path ?? [];
      if (path.length > 1) {
        ctx.strokeStyle = accent;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        path.forEach(([x, y], i) => (i ? ctx.lineTo(sx(x), sy(y)) : ctx.moveTo(sx(x), sy(y))));
        ctx.stroke();
        ctx.lineWidth = 1;
      }

      // next station: dashed line from the scanner to a ring where the next scan should be taken
      const st = room?.state === "mapping" ? room?.next?.station : null;
      if (st && room?.pose) {
        const warn = cssVar("--warn", "#e0a040");
        ctx.strokeStyle = warn;
        ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.moveTo(sx(room.pose.x), sy(room.pose.y));
        ctx.lineTo(sx(st.x), sy(st.y));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(sx(st.x), sy(st.y), 10, 0, Math.PI * 2);
        ctx.stroke();
        ctx.lineWidth = 1;
        ctx.fillStyle = warn;
        ctx.font = "12px ui-monospace, Menlo, monospace";
        ctx.textAlign = "left";
        ctx.fillText(`next scan here · ${st.distance_m.toFixed(1)} m`, sx(st.x) + 14, sy(st.y) + 4);
      }

      // pose: a triangle pointing along theta (CCW from +x, +y drawn up)
      const p = room?.pose;
      if (p) {
        const a = (-p.theta_deg * Math.PI) / 180; // screen angle (y down)
        const x0 = sx(p.x);
        const y0 = sy(p.y);
        const L = 13;
        const Wd = 7;
        ctx.fillStyle = room?.lost ? cssVar("--err", "#e06060") : accent;
        ctx.beginPath();
        ctx.moveTo(x0 + Math.cos(a) * L, y0 + Math.sin(a) * L);
        ctx.lineTo(x0 + Math.cos(a + 2.5) * Wd, y0 + Math.sin(a + 2.5) * Wd);
        ctx.lineTo(x0 + Math.cos(a - 2.5) * Wd, y0 + Math.sin(a - 2.5) * Wd);
        ctx.closePath();
        ctx.fill();
      }

      if (!grid || grid.w === 0) {
        ctx.fillStyle = dim;
        ctx.font = "13px system-ui, sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(room?.state === "mapping" ? "waiting for the first map" : "no map yet: Start mapping", W / 2, H / 2 + 28);
      }
    };
    paint();
    const ro = new ResizeObserver(paint);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [view, grid, image, room]);

  const toWorld = (e: React.MouseEvent<HTMLCanvasElement>): [number, number] => {
    const rect = e.currentTarget.getBoundingClientRect();
    const v = viewRef.current.scale > 0 ? viewRef.current : fitView(boundsOf(grid, room), rect.width, rect.height);
    return [v.cx + (e.clientX - rect.left - rect.width / 2) / v.scale, v.cy - (e.clientY - rect.top - rect.height / 2) / v.scale];
  };

  return (
    <div className="ops-map-wrap" ref={wrapRef}>
      <canvas
        ref={canvasRef}
        onMouseDown={(e) => {
          if (e.button !== 0) return;
          const v = viewRef.current;
          dragRef.current = { x: e.clientX, y: e.clientY, cx: v.cx, cy: v.cy, moved: false };
        }}
        onMouseMove={(e) => {
          const d = dragRef.current;
          if (d && (e.buttons & 1) === 1) {
            const dx = e.clientX - d.x;
            const dy = e.clientY - d.y;
            if (!d.moved && Math.hypot(dx, dy) > DRAG_PX) {
              d.moved = true;
              userMoved.current = true;
            }
            if (d.moved) {
              const s = viewRef.current.scale || 60;
              setView((v) => ({ ...v, cx: d.cx - dx / s, cy: d.cy + dy / s }));
              return;
            }
          }
          const [x, y] = toWorld(e);
          let val: number | null = null;
          if (grid && grid.w > 0) {
            const ix = Math.floor((x - grid.x0_m) / grid.cell_m);
            const iy = Math.floor((y - grid.y0_m) / grid.cell_m);
            if (ix >= 0 && iy >= 0 && ix < grid.w && iy < grid.h) val = grid.cells[iy * grid.w + ix];
          }
          setHover({ x, y, v: val });
        }}
        onMouseUp={() => {
          dragRef.current = null;
        }}
        onMouseLeave={() => {
          dragRef.current = null;
          setHover(null);
        }}
      />
      {hover && (
        <div className="heightmap-readout">
          x {fmt(hover.x, 2)} y {fmt(hover.y, 2)} m{hover.v !== null ? ` · ${CELL_WORDS[hover.v] ?? hover.v}` : ""}
        </div>
      )}
      <div className="scene-overlay">drag: pan · wheel: zoom</div>
    </div>
  );
});
