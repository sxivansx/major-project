import { forwardRef, useEffect, useImperativeHandle, useMemo, useRef, useState } from "react";
import { CLASS_NAMES, FINDING_KINDS, type Finding, type FindingKind, type Findings, type OpsLayersFrame } from "../api/types";
import { rampRgb } from "../lib/colormap";
import { fmt } from "../lib/format";
import { findingById, planRoute, useOps } from "../store/ops";
import { LOW_CONFIDENCE, RECENT_MAP_S, bearingText, coreNowSec, useAudio, useCoreNow } from "../store/audio";

// Top-down operator map on a plain canvas. World frame is the base frame:
// scanner at the origin, +x the lidar's 0° (cable) mark, +y up on screen.
// Bearings in the findings are clockwise from +x, so a bearing b points
// along (cos b, -sin b) in world coordinates.
//
// Two renderings. Simple (default): one green for walkable, one red for
// blocked, one hatch for everything not seen (shadow, blind, unknown), labels
// only on voids, plus the next-station arrow. Advanced: the layer toggles
// (terrain ramp, traversability, shadow hatch, blind hatch) and every finding
// outlined and labelled.

export interface OpsMapHandle {
  toBlob(): Promise<Blob | null>;
  fit(): void;
}

interface Props {
  scanningText?: string | null; // shown top-left while a sweep runs
}

interface View {
  cx: number; // world x at the canvas centre
  cy: number;
  scale: number; // css px per metre
}

const DEG = Math.PI / 180;
const HATCH_PX = 8; // offscreen pixels per cell for the hatch layers
const MIN_SCALE = 4;
const MAX_SCALE = 1500;
const DRAG_PX = 3;

const C = {
  bg: "#1c1c1c",
  grid: "#2a2a2a",
  gridMajor: "#333333",
  gridText: "#6f6f6f",
  text: "#d9d9d9",
  dim: "#8c8c8c",
  accent: "#4d9de0",
  ok: "#6fbf73",
  warn: "#e0a040",
  err: "#e06060",
  mono: "11px ui-monospace, Menlo, monospace",
};

// Desaturated traversability fills, drawn at 50 % alpha over the terrain.
export const TRAV_RGB: Record<number, [number, number, number]> = {
  1: [95, 143, 100],
  2: [176, 138, 69],
  3: [168, 85, 85],
};
// Simple view: walkable green (marginal = same green, lighter), blocked red.
export const SIMPLE_RGBA: Record<number, [number, number, number, number]> = {
  1: [95, 143, 100, 150],
  2: [95, 143, 100, 70],
  3: [168, 85, 85, 150],
};
export const HATCH_COLOUR = { shadow: "#9a9a9a", blind: "#7a7a7a", notSeen: "#7c7c7c" };

export const FINDING_STYLE: Record<FindingKind, { colour: string; width: number; dash: number[]; label: string }> = {
  voids: { colour: C.accent, width: 2, dash: [], label: "void" },
  obstacles: { colour: C.text, width: 1, dash: [], label: "obstacle" },
  depressions: { colour: "#b8b8b8", width: 1, dash: [2, 3], label: "depression" },
  shadows: { colour: C.dim, width: 1, dash: [6, 4], label: "shadow" },
};

// Plain words for the hover readout in the simple view.
export function classWords(cls: number): string {
  switch (cls) {
    case 1:
      return "can walk";
    case 2:
      return "can walk, rough";
    case 3:
      return "blocked";
    default:
      return "not seen";
  }
}

// ---- layer images (world-aligned, row 0 at the top = max y) ----

export function heightRange(l: OpsLayersFrame): [number, number] {
  let lo = Infinity;
  let hi = -Infinity;
  for (let i = 0; i < l.height.length; i++) {
    const v = l.height[i];
    if (Number.isFinite(v)) {
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
  }
  if (!Number.isFinite(lo)) return [0, 1];
  lo = Math.max(-1, lo);
  hi = Math.min(3, hi);
  if (!(hi > lo + 0.05)) hi = lo + 0.5;
  return [lo, hi];
}

function renderTerrain(l: OpsLayersFrame, range: [number, number]): HTMLCanvasElement | null {
  if (l.w === 0 || l.h === 0) return null;
  const off = document.createElement("canvas");
  off.width = l.w;
  off.height = l.h;
  const ctx = off.getContext("2d")!;
  const img = ctx.createImageData(l.w, l.h);
  const px = img.data;
  const inv = 1 / (range[1] - range[0]);
  const rgb: [number, number, number] = [0, 0, 0];
  for (let iy = 0; iy < l.h; iy++) {
    const row = l.h - 1 - iy;
    for (let ix = 0; ix < l.w; ix++) {
      const v = l.height[iy * l.w + ix];
      const o = (row * l.w + ix) * 4;
      if (!Number.isFinite(v)) continue; // transparent
      rampRgb((v - range[0]) * inv, rgb);
      px[o] = rgb[0] * 255;
      px[o + 1] = rgb[1] * 255;
      px[o + 2] = rgb[2] * 255;
      px[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  return off;
}

function renderClasses(l: OpsLayersFrame, colours: Record<number, [number, number, number, number]>): HTMLCanvasElement | null {
  if (l.w === 0 || l.h === 0) return null;
  const off = document.createElement("canvas");
  off.width = l.w;
  off.height = l.h;
  const ctx = off.getContext("2d")!;
  const img = ctx.createImageData(l.w, l.h);
  const px = img.data;
  for (let iy = 0; iy < l.h; iy++) {
    const row = l.h - 1 - iy;
    for (let ix = 0; ix < l.w; ix++) {
      const c = colours[l.classes[iy * l.w + ix]];
      if (!c) continue;
      const o = (row * l.w + ix) * 4;
      px[o] = c[0];
      px[o + 1] = c[1];
      px[o + 2] = c[2];
      px[o + 3] = c[3];
    }
  }
  ctx.putImageData(img, 0, 0);
  return off;
}

const TRAV_RGBA: Record<number, [number, number, number, number]> = {
  1: [...TRAV_RGB[1], 128],
  2: [...TRAV_RGB[2], 128],
  3: [...TRAV_RGB[3], 128],
};

function hatchTile(kind: "diag" | "horiz", colour: string): HTMLCanvasElement {
  const t = document.createElement("canvas");
  t.width = HATCH_PX;
  t.height = HATCH_PX;
  const ctx = t.getContext("2d")!;
  ctx.strokeStyle = colour;
  ctx.lineWidth = 1;
  ctx.beginPath();
  if (kind === "diag") {
    ctx.moveTo(0, HATCH_PX);
    ctx.lineTo(HATCH_PX, 0);
    ctx.moveTo(-1, 1);
    ctx.lineTo(1, -1);
    ctx.moveTo(HATCH_PX - 1, HATCH_PX + 1);
    ctx.lineTo(HATCH_PX + 1, HATCH_PX - 1);
  } else {
    ctx.moveTo(0, HATCH_PX / 2 + 0.5);
    ctx.lineTo(HATCH_PX, HATCH_PX / 2 + 0.5);
  }
  ctx.stroke();
  return t;
}

// Cells of the given classes filled with a repeating hatch, at HATCH_PX per
// cell so the lines stay continuous across neighbouring cells.
function renderHatch(l: OpsLayersFrame, classes: number[], kind: "diag" | "horiz", colour: string): HTMLCanvasElement | null {
  if (l.w === 0 || l.h === 0) return null;
  const off = document.createElement("canvas");
  off.width = l.w * HATCH_PX;
  off.height = l.h * HATCH_PX;
  const ctx = off.getContext("2d")!;
  const pat = ctx.createPattern(hatchTile(kind, colour), "repeat");
  if (!pat) return null;
  ctx.fillStyle = pat;
  const want = new Uint8Array(256);
  for (const c of classes) want[c] = 1;
  let any = false;
  for (let iy = 0; iy < l.h; iy++) {
    const row = l.h - 1 - iy;
    let ix = 0;
    while (ix < l.w) {
      if (!want[l.classes[iy * l.w + ix]]) {
        ix++;
        continue;
      }
      let run = ix + 1;
      while (run < l.w && want[l.classes[iy * l.w + run]]) run++;
      ctx.fillRect(ix * HATCH_PX, row * HATCH_PX, (run - ix) * HATCH_PX, HATCH_PX);
      any = true;
      ix = run;
    }
  }
  return any ? off : null;
}

// ---- geometry ----

type Bounds = [number, number, number, number]; // x0, y0, x1, y1

function boundsOf(layers: OpsLayersFrame | null, findings: Findings | null): Bounds {
  const g = layers && layers.w > 0 ? layers : findings && findings.grid.w > 0 ? findings.grid : null;
  if (!g) return [-3, -3, 3, 3];
  return [g.x0_m, g.y0_m, g.x0_m + g.w * g.cell_m, g.y0_m + g.h * g.cell_m];
}

function fitView(b: Bounds, cssW: number, cssH: number): View {
  const pad = 28;
  const gw = Math.max(0.5, b[2] - b[0]);
  const gh = Math.max(0.5, b[3] - b[1]);
  const scale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, Math.min((cssW - pad * 2) / gw, (cssH - pad * 2) / gh)));
  return { cx: (b[0] + b[2]) / 2, cy: (b[1] + b[3]) / 2, scale };
}

function gridStep(scale: number): number {
  if (scale >= 60) return 1;
  if (scale >= 24) return 2;
  if (scale >= 10) return 5;
  return 10;
}

function arrowHead(ctx: CanvasRenderingContext2D, x0: number, y0: number, x1: number, y1: number, size: number) {
  const a = Math.atan2(y1 - y0, x1 - x0);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - size * Math.cos(a - 0.45), y1 - size * Math.sin(a - 0.45));
  ctx.lineTo(x1 - size * Math.cos(a + 0.45), y1 - size * Math.sin(a + 0.45));
  ctx.closePath();
  ctx.fill();
}

function label(ctx: CanvasRenderingContext2D, text: string, x: number, y: number, colour: string) {
  ctx.font = C.mono;
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  const w = ctx.measureText(text).width;
  ctx.fillStyle = "rgba(28,28,28,0.82)";
  ctx.fillRect(x - 2, y - 10, w + 4, 13);
  ctx.fillStyle = colour;
  ctx.fillText(text, x, y);
}

interface Hover {
  x: number;
  y: number;
  ix: number;
  iy: number;
  h: number;
  cls: number;
}

export const OpsMap = forwardRef<OpsMapHandle, Props>(function OpsMap({ scanningText }, ref) {
  const layers = useOps((s) => s.layers);
  const findings = useOps((s) => s.findings);
  const route = useOps((s) => s.route);
  const routeEnds = useOps((s) => s.routeEnds);
  const routeMode = useOps((s) => s.routeMode);
  const routeStart = useOps((s) => s.routeStart);
  const selected = useOps((s) => s.selected);
  const focus = useOps((s) => s.focus);
  const fitNonce = useOps((s) => s.fitNonce);
  const toggles = useOps((s) => s.toggles);
  const advanced = useOps((s) => s.advanced);
  const select = useOps((s) => s.select);
  const setRouteStart = useOps((s) => s.setRouteStart);
  const setRouteMode = useOps((s) => s.setRouteMode);
  const simple = !advanced;
  const soundEvents = useAudio((s) => s.events);
  const soundActive = soundEvents.some((e) => e.bearing_deg !== null && coreNowSec() - e.t < RECENT_MAP_S);
  const now = useCoreNow(soundActive);
  const recentSound = useMemo(
    () => soundEvents.filter((e) => e.bearing_deg !== null && now - e.t < RECENT_MAP_S && now - e.t > -5),
    [soundEvents, now],
  );

  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [view, setView] = useState<View>({ cx: 0, cy: 0, scale: 0 }); // scale 0 = not fitted yet
  const viewRef = useRef(view);
  viewRef.current = view;
  const [hover, setHover] = useState<Hover | null>(null);
  const dragRef = useRef<{ x: number; y: number; cx: number; cy: number; moved: boolean } | null>(null);

  const range = useMemo(() => (layers ? heightRange(layers) : ([0, 1] as [number, number])), [layers]);
  const terrainImg = useMemo(() => (layers && advanced && toggles.terrain ? renderTerrain(layers, range) : null), [layers, range, advanced, toggles.terrain]);
  const travImg = useMemo(() => (layers ? renderClasses(layers, simple ? SIMPLE_RGBA : TRAV_RGBA) : null), [layers, simple]);
  const notSeenImg = useMemo(() => (layers && simple ? renderHatch(layers, [0, 4, 5], "diag", HATCH_COLOUR.notSeen) : null), [layers, simple]);
  const shadowImg = useMemo(() => (layers && advanced ? renderHatch(layers, [4], "diag", HATCH_COLOUR.shadow) : null), [layers, advanced]);
  const blindImg = useMemo(() => (layers && advanced ? renderHatch(layers, [5], "horiz", HATCH_COLOUR.blind) : null), [layers, advanced]);

  // The view fits the analysed area on the first non-empty layers and follows
  // it as the area grows, until the user pans or zooms; Fit starts that over.
  const fittedBoundsRef = useRef("");
  const userMovedRef = useRef(false);
  const doFit = () => {
    const wrap = wrapRef.current;
    if (!wrap) return;
    const b = boundsOf(useOps.getState().layers, useOps.getState().findings);
    setView(fitView(b, wrap.clientWidth, wrap.clientHeight));
    fittedBoundsRef.current = b.join(",");
    userMovedRef.current = false;
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

  useEffect(() => {
    if (!layers || layers.w === 0 || layers.h === 0) return;
    const key = boundsOf(layers, null).join(",");
    if (fittedBoundsRef.current === "" || (!userMovedRef.current && key !== fittedBoundsRef.current)) doFit();
  }, [layers]);
  useEffect(() => {
    if (fitNonce > 0) doFit();
  }, [fitNonce]);
  useEffect(() => {
    if (focus) setView((v) => ({ ...v, cx: focus.x, cy: focus.y }));
  }, [focus]);

  // wheel zoom around the cursor (non-passive so the page does not scroll)
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      userMovedRef.current = true;
      const rect = canvas.getBoundingClientRect();
      const mx = e.clientX - rect.left;
      const my = e.clientY - rect.top;
      setView((v) => {
        const scale0 = v.scale > 0 ? v.scale : 60;
        const scale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, scale0 * Math.exp(-e.deltaY * 0.0015)));
        const W = rect.width;
        const H = rect.height;
        const wx = v.cx + (mx - W / 2) / scale0;
        const wy = v.cy - (my - H / 2) / scale0;
        return { cx: wx - (mx - W / 2) / scale, cy: wy + (my - H / 2) / scale, scale };
      });
    };
    canvas.addEventListener("wheel", onWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", onWheel);
  }, []);

  // paint
  useEffect(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const paint = () => {
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      const W = wrap.clientWidth;
      const H = wrap.clientHeight;
      canvas.width = Math.max(1, Math.round(W * dpr));
      canvas.height = Math.max(1, Math.round(H * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = C.bg;
      ctx.fillRect(0, 0, W, H);

      const v = view.scale > 0 ? view : fitView(boundsOf(layers, findings), W, H);
      const scale = v.scale;
      const sx = (x: number) => W / 2 + (x - v.cx) * scale;
      const sy = (y: number) => H / 2 - (y - v.cy) * scale;

      // 1 m grid (coarser when zoomed out), labels along the bottom and left
      const step = gridStep(scale);
      const xmin = v.cx - W / 2 / scale;
      const xmax = v.cx + W / 2 / scale;
      const ymin = v.cy - H / 2 / scale;
      const ymax = v.cy + H / 2 / scale;
      ctx.lineWidth = 1;
      ctx.font = C.mono;
      ctx.textBaseline = "alphabetic";
      for (let x = Math.ceil(xmin / step) * step; x <= xmax; x += step) {
        const px = Math.round(sx(x)) + 0.5;
        ctx.strokeStyle = x === 0 ? C.gridMajor : C.grid;
        ctx.beginPath();
        ctx.moveTo(px, 0);
        ctx.lineTo(px, H);
        ctx.stroke();
        ctx.fillStyle = C.gridText;
        ctx.textAlign = "center";
        ctx.fillText(`${x}`, px, H - 4);
      }
      for (let y = Math.ceil(ymin / step) * step; y <= ymax; y += step) {
        const py = Math.round(sy(y)) + 0.5;
        ctx.strokeStyle = y === 0 ? C.gridMajor : C.grid;
        ctx.beginPath();
        ctx.moveTo(0, py);
        ctx.lineTo(W, py);
        ctx.stroke();
        ctx.fillStyle = C.gridText;
        ctx.textAlign = "left";
        ctx.fillText(`${y}`, 4, py - 3);
      }

      // layers
      if (layers && layers.w > 0) {
        const gx = sx(layers.x0_m);
        const gy = sy(layers.y0_m + layers.h * layers.cell_m);
        const gw = layers.w * layers.cell_m * scale;
        const gh = layers.h * layers.cell_m * scale;
        ctx.imageSmoothingEnabled = false;
        if (simple) {
          if (travImg) ctx.drawImage(travImg, gx, gy, gw, gh);
          ctx.imageSmoothingEnabled = true;
          if (notSeenImg) ctx.drawImage(notSeenImg, gx, gy, gw, gh);
        } else {
          if (toggles.terrain && terrainImg) ctx.drawImage(terrainImg, gx, gy, gw, gh);
          if (toggles.traversability && travImg) ctx.drawImage(travImg, gx, gy, gw, gh);
          ctx.imageSmoothingEnabled = true;
          if (toggles.shadow && shadowImg) ctx.drawImage(shadowImg, gx, gy, gw, gh);
          if (toggles.blind && blindImg) ctx.drawImage(blindImg, gx, gy, gw, gh);
        }
        ctx.strokeStyle = "#454545";
        ctx.strokeRect(Math.round(gx) + 0.5, Math.round(gy) + 0.5, Math.round(gw), Math.round(gh));
      }

      // sound directions: wedges bearing ± 2σ from the scanner (a direction,
      // never a distance), fading out over RECENT_MAP_S; newest on top and
      // strongest; the candidate regions along them outlined in the accent
      if (recentSound.length) {
        const x0 = sx(0);
        const y0 = sy(0);
        const b = boundsOf(layers, findings);
        const reach = Math.max(3, Math.hypot(b[0], b[1]), Math.hypot(b[0], b[3]), Math.hypot(b[2], b[1]), Math.hypot(b[2], b[3]));
        const rad = reach * scale;
        for (let i = recentSound.length - 1; i >= 0; i--) {
          const ev = recentSound[i];
          const fade = Math.max(0, Math.min(1, 1 - (now - ev.t) / RECENT_MAP_S));
          const newest = i === 0;
          const low = ev.confidence < LOW_CONFIDENCE;
          const a = ev.bearing_deg! * DEG;
          const half = Math.min(180, 2 * (ev.bearing_sigma_deg ?? 10)) * DEG;
          ctx.beginPath();
          ctx.moveTo(x0, y0);
          ctx.arc(x0, y0, rad, a - half, a + half);
          ctx.closePath();
          ctx.fillStyle = low ? `rgba(140,140,140,${0.14 * fade})` : `rgba(77,157,224,${(newest ? 0.22 : 0.1) * fade})`;
          ctx.fill();
          ctx.strokeStyle = low ? `rgba(140,140,140,${0.8 * fade})` : `rgba(77,157,224,${(newest ? 0.95 : 0.5) * fade})`;
          ctx.lineWidth = newest ? 2 : 1;
          ctx.beginPath();
          ctx.moveTo(x0, y0);
          ctx.lineTo(x0 + Math.cos(a) * rad, y0 + Math.sin(a) * rad);
          ctx.stroke();
          ctx.lineWidth = 1;
          if (newest) {
            const lr = Math.min(rad * 0.55, Math.max(60, Math.min(W, H) * 0.3));
            label(ctx, `sound ${bearingText(ev)}${low ? " (low confidence)" : ""}`, x0 + Math.cos(a) * lr + 6, y0 + Math.sin(a) * lr - 6, low ? C.dim : C.accent);
          }
        }
        if (findings) {
          const done = new Set<string>();
          for (const ev of recentSound) {
            const fade = Math.max(0.25, Math.min(1, 1 - (now - ev.t) / RECENT_MAP_S));
            for (const cand of ev.candidates) {
              if (done.has(cand.id)) continue;
              done.add(cand.id);
              const hit = findingById(findings, cand.id);
              if (!hit) continue;
              const fb = hit.item.bbox_m;
              ctx.globalAlpha = fade;
              ctx.strokeStyle = C.accent;
              ctx.lineWidth = 2;
              ctx.strokeRect(Math.round(sx(fb[0])) + 0.5, Math.round(sy(fb[3])) + 0.5, Math.max(2, Math.round((fb[2] - fb[0]) * scale)), Math.max(2, Math.round((fb[3] - fb[1]) * scale)));
              ctx.lineWidth = 1;
              if (hit.kind !== "voids") label(ctx, cand.id, Math.round(sx(fb[0])) + 3, Math.round(sy(fb[3])) - 3, C.accent);
              ctx.globalAlpha = 1;
            }
          }
        }
      }

      // findings outlines: voids always, the rest only in Advanced
      if (findings && (simple || toggles.findings)) {
        const kinds: FindingKind[] = simple ? ["voids"] : FINDING_KINDS;
        for (const kind of kinds) {
          const st = FINDING_STYLE[kind];
          for (const f of findings[kind]) {
            const b = f.bbox_m;
            const x = Math.round(sx(b[0])) + 0.5;
            const y = Math.round(sy(b[3])) + 0.5;
            const w = Math.max(2, Math.round((b[2] - b[0]) * scale));
            const h = Math.max(2, Math.round((b[3] - b[1]) * scale));
            const sel = f.id === selected;
            if (sel) {
              ctx.fillStyle = "rgba(255,255,255,0.10)";
              ctx.fillRect(x, y, w, h);
            }
            ctx.strokeStyle = sel ? "#ffffff" : st.colour;
            ctx.lineWidth = st.width + (sel ? 1 : 0);
            ctx.setLineDash(st.dash);
            ctx.strokeRect(x, y, w, h);
            ctx.setLineDash([]);
            label(ctx, f.id, x + 3, y - 3, sel ? "#ffffff" : st.colour);
          }
        }
        ctx.lineWidth = 1;
      }

      // next station: arrow from the scanner at the bearing (clockwise from +x)
      const ns = findings?.next_station;
      if ((simple || toggles.station) && ns && ns.bearing_deg !== null && ns.bearing_deg !== undefined) {
        const len = Math.max(0.5, ns.distance_m);
        const dx = Math.cos(ns.bearing_deg * DEG);
        const dy = -Math.sin(ns.bearing_deg * DEG);
        const x0 = sx(0);
        const y0 = sy(0);
        const x1 = sx(dx * len);
        const y1 = sy(dy * len);
        ctx.strokeStyle = C.text;
        ctx.fillStyle = C.text;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
        arrowHead(ctx, x0, y0, x1, y1, 9);
        ctx.lineWidth = 1;
        label(ctx, simple ? "next station" : `next station ${fmt(ns.bearing_deg, 0)}° ${fmt(ns.distance_m, 1)} m`, x1 + 6, y1 + 4, C.text);
      }

      // route
      const ends = routeEnds;
      if (route && route.found && route.path.length > 0) {
        ctx.strokeStyle = route.verdict === "passable" ? C.ok : C.warn;
        ctx.lineWidth = 2;
        ctx.beginPath();
        route.path.forEach(([x, y], i) => (i ? ctx.lineTo(sx(x), sy(y)) : ctx.moveTo(sx(x), sy(y))));
        ctx.stroke();
        ctx.lineWidth = 1;
      }
      if (ends) {
        const colour = route ? (route.found ? (route.verdict === "passable" ? C.ok : C.warn) : C.err) : C.dim;
        ctx.strokeStyle = colour;
        ctx.fillStyle = colour;
        ctx.beginPath();
        ctx.arc(sx(ends.from[0]), sy(ends.from[1]), 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeRect(Math.round(sx(ends.to[0])) - 4.5, Math.round(sy(ends.to[1])) - 4.5, 9, 9);
        label(ctx, "start", sx(ends.from[0]) + 7, sy(ends.from[1]) + 4, colour);
        label(ctx, "goal", sx(ends.to[0]) + 7, sy(ends.to[1]) + 4, colour);
      } else if (routeStart) {
        ctx.fillStyle = C.dim;
        ctx.beginPath();
        ctx.arc(sx(routeStart[0]), sy(routeStart[1]), 4, 0, Math.PI * 2);
        ctx.fill();
        label(ctx, "start: now click the goal", sx(routeStart[0]) + 7, sy(routeStart[1]) + 4, C.dim);
      }

      // scanner at the origin with the +x (cable mark) direction
      {
        const x0 = sx(0);
        const y0 = sy(0);
        ctx.strokeStyle = C.accent;
        ctx.fillStyle = C.accent;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(x0, y0, 4, 0, Math.PI * 2);
        ctx.fill();
        ctx.beginPath();
        ctx.arc(x0, y0, 8, 0, Math.PI * 2);
        ctx.stroke();
        const ax = x0 + 36;
        ctx.beginPath();
        ctx.moveTo(x0 + 8, y0);
        ctx.lineTo(ax, y0);
        ctx.stroke();
        arrowHead(ctx, x0, y0, ax, y0, 7);
        ctx.lineWidth = 1;
        label(ctx, simple ? "scanner" : "+x cable mark", ax + 4, y0 + 4, C.accent);
      }

      // hovered cell
      if (hover && layers) {
        const hx = sx(layers.x0_m + hover.ix * layers.cell_m);
        const hy = sy(layers.y0_m + (hover.iy + 1) * layers.cell_m);
        const cs = Math.max(2, layers.cell_m * scale);
        ctx.strokeStyle = "#ffffff";
        ctx.strokeRect(Math.round(hx) + 0.5, Math.round(hy) + 0.5, Math.round(cs), Math.round(cs));
      }

      if (!layers || layers.w === 0) {
        ctx.fillStyle = C.dim;
        ctx.font = "13px system-ui, sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(scanningText ? "first results after this scan" : "nothing scanned yet", W / 2, H / 2 + 24);
      }
    };
    paint();
    const ro = new ResizeObserver(paint);
    ro.observe(wrap);
    return () => ro.disconnect();
  }, [view, layers, findings, route, routeEnds, routeStart, selected, hover, toggles, simple, terrainImg, travImg, notSeenImg, shadowImg, blindImg, scanningText, recentSound, now]);

  // ---- mouse ----

  const toWorld = (e: React.MouseEvent<HTMLCanvasElement>): [number, number] => {
    const rect = e.currentTarget.getBoundingClientRect();
    const v = viewRef.current.scale > 0 ? viewRef.current : fitView(boundsOf(layers, findings), rect.width, rect.height);
    return [v.cx + (e.clientX - rect.left - rect.width / 2) / v.scale, v.cy - (e.clientY - rect.top - rect.height / 2) / v.scale];
  };

  const onDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (e.button !== 0) return;
    const v = viewRef.current;
    dragRef.current = { x: e.clientX, y: e.clientY, cx: v.cx, cy: v.cy, moved: false };
  };

  const onMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const d = dragRef.current;
    if (d && (e.buttons & 1) === 1) {
      const dx = e.clientX - d.x;
      const dy = e.clientY - d.y;
      if (!d.moved && Math.hypot(dx, dy) > DRAG_PX) {
        d.moved = true;
        userMovedRef.current = true;
      }
      if (d.moved) {
        const scale = viewRef.current.scale || 60;
        setView((v) => ({ ...v, cx: d.cx - dx / scale, cy: d.cy + dy / scale }));
        return;
      }
    } else if (d) {
      dragRef.current = null;
    }
    if (!layers || layers.w === 0) return setHover(null);
    const [wx, wy] = toWorld(e);
    const ix = Math.floor((wx - layers.x0_m) / layers.cell_m);
    const iy = Math.floor((wy - layers.y0_m) / layers.cell_m);
    if (ix < 0 || iy < 0 || ix >= layers.w || iy >= layers.h) return setHover(null);
    const k = iy * layers.w + ix;
    setHover({ x: wx, y: wy, ix, iy, h: layers.height[k], cls: layers.classes[k] });
  };

  const onUp = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const d = dragRef.current;
    dragRef.current = null;
    if (!d || d.moved) return;
    const p = toWorld(e);
    if (routeMode) {
      if (!routeStart) setRouteStart(p);
      else {
        void planRoute(routeStart, p, "manual");
        setRouteMode(false);
      }
      return;
    }
    // select the smallest finding whose bbox contains the click (voids only in the simple view)
    let best: { f: Finding; area: number } | null = null;
    if (findings) {
      const kinds: FindingKind[] = simple ? ["voids"] : FINDING_KINDS;
      for (const kind of kinds) {
        for (const f of findings[kind]) {
          const b = f.bbox_m;
          if (p[0] >= b[0] && p[0] <= b[2] && p[1] >= b[1] && p[1] <= b[3]) {
            const area = (b[2] - b[0]) * (b[3] - b[1]);
            if (!best || area < best.area) best = { f, area };
          }
        }
      }
    }
    select(best ? (best.f.id === selected ? null : best.f.id) : null);
  };

  return (
    <div className={"ops-map-wrap" + (routeMode ? " route" : "")} ref={wrapRef}>
      <canvas
        ref={canvasRef}
        onMouseDown={onDown}
        onMouseMove={onMove}
        onMouseUp={onUp}
        onMouseLeave={() => {
          dragRef.current = null;
          setHover(null);
        }}
      />
      {scanningText && <div className="ops-scanning">{scanningText}</div>}
      {hover && (
        <div className="heightmap-readout">
          x {fmt(hover.x, 2)} y {fmt(hover.y, 2)} ·{" "}
          {simple
            ? classWords(hover.cls)
            : `h ${Number.isFinite(hover.h) ? fmt(hover.h, 2) + " m" : "–"} · ${CLASS_NAMES[hover.cls] ?? `class ${hover.cls}`}`}
        </div>
      )}
      <div className="scene-overlay">drag: pan · wheel: zoom{routeMode ? " · route: click start, then goal" : ""}</div>
    </div>
  );
});
