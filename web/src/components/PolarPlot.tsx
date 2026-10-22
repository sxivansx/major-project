import { useEffect, useRef, useState } from "react";
import type { ScanFrame } from "../api/types";
import { useStore } from "../store/store";

// Live 2D polar plot on a plain canvas. Subscribes to the store outside
// React and redraws in requestAnimationFrame when a new scan arrives.
//
// Native lidar angle is clockwise viewed from above: 0° (cable mark, +x) is
// drawn upwards, 90° to the right, 180° down, 270° left. The lidar frame's
// +y (90° counter-clockwise) therefore points left on screen.

const NICE_RANGES = [1, 2, 3, 5, 8, 12, 16, 20, 30];
const RANGE_OPTIONS: { v: number; label: string }[] = [
  { v: 0, label: "auto" },
  { v: 2, label: "2 m" },
  { v: 5, label: "5 m" },
  { v: 8, label: "8 m" },
  { v: 12, label: "12 m" },
  { v: 20, label: "20 m" },
  { v: 30, label: "30 m" },
];
const QBINS = 8;

const BG = "#1c1c1c";
const RING = "#333333";
const RING_MAJOR = "#454545";
const TEXT = "#8c8c8c";
const CENTRE = "#4d9de0";

function binColour(i: number): string {
  // dim grey -> accent blue as quality rises
  const t = i / (QBINS - 1);
  const r = Math.round(80 + (77 - 80) * t);
  const g = Math.round(80 + (157 - 80) * t);
  const b = Math.round(80 + (224 - 80) * t);
  return `rgb(${r},${g},${b})`;
}
const BIN_COLOURS = Array.from({ length: QBINS }, (_, i) => binColour(i));

function ringStep(maxR: number): number {
  if (maxR <= 2) return 0.5;
  if (maxR <= 5) return 1;
  if (maxR <= 12) return 2;
  return 5;
}

interface DrawState {
  autoR: number;
}

function draw(
  ctx: CanvasRenderingContext2D,
  cssW: number,
  cssH: number,
  scan: ScanFrame | null,
  fixedR: number,
  ds: DrawState,
) {
  ctx.fillStyle = BG;
  ctx.fillRect(0, 0, cssW, cssH);

  const cx = cssW / 2;
  const cy = cssH / 2;
  const R = Math.max(10, Math.min(cssW, cssH) / 2 - 22);

  // pick the plot range
  let maxR = fixedR;
  if (maxR <= 0) {
    let need = 0;
    if (scan) for (let i = 0; i < scan.n; i++) if (scan.range[i] > need) need = scan.range[i];
    let pick = NICE_RANGES[NICE_RANGES.length - 1];
    for (const r of NICE_RANGES) {
      if (need <= r) {
        pick = r;
        break;
      }
    }
    // hysteresis: only shrink when the data is well inside the smaller range
    if (pick < ds.autoR && need > ds.autoR * 0.6) pick = ds.autoR;
    ds.autoR = pick;
    maxR = pick;
  }
  const scale = R / maxR;

  // rings
  const step = ringStep(maxR);
  ctx.lineWidth = 1;
  ctx.font = "11px ui-monospace, Menlo, monospace";
  ctx.textBaseline = "middle";
  for (let r = step, k = 1; r <= maxR + 1e-6; r += step, k++) {
    ctx.strokeStyle = k % 2 === 0 ? RING_MAJOR : RING;
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = TEXT;
    ctx.textAlign = "left";
    ctx.fillText(`${+r.toFixed(2)} m`, cx + 3, cy - r * scale + 7);
  }

  // angle spokes every 30°, labels clockwise from the top
  for (let a = 0; a < 360; a += 30) {
    const rad = (a * Math.PI) / 180;
    const dx = Math.sin(rad);
    const dy = -Math.cos(rad);
    ctx.strokeStyle = a % 90 === 0 ? RING_MAJOR : RING;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + dx * R, cy + dy * R);
    ctx.stroke();
    ctx.fillStyle = TEXT;
    ctx.textAlign = "center";
    ctx.fillText(`${a}°`, cx + dx * (R + 12), cy + dy * (R + 12));
  }

  // frame hints
  ctx.fillStyle = TEXT;
  ctx.textAlign = "left";
  ctx.fillText("+x (cable mark)", cx + 6, cy - R + 14);
  ctx.textAlign = "right";
  ctx.fillText("+y", cx - R + 24, cy - 8);

  // centre marker
  ctx.fillStyle = CENTRE;
  ctx.beginPath();
  ctx.arc(cx, cy, 3, 0, Math.PI * 2);
  ctx.fill();

  if (!scan) {
    ctx.fillStyle = TEXT;
    ctx.textAlign = "center";
    ctx.font = "13px system-ui, sans-serif";
    ctx.fillText("no scan yet", cx, cy + 20);
    return;
  }

  // points, bucketed by quality to limit fillStyle changes
  const n = scan.n;
  let qmax = 1;
  for (let i = 0; i < n; i++) if (scan.quality[i] > qmax) qmax = scan.quality[i];
  const bins: number[][] = Array.from({ length: QBINS }, () => []);
  for (let i = 0; i < n; i++) {
    const r = scan.range[i];
    if (!(r > 0) || r > maxR) continue;
    const b = Math.min(QBINS - 1, Math.floor((scan.quality[i] / qmax) * QBINS));
    bins[b].push(i);
  }
  const DEG = Math.PI / 180;
  for (let b = 0; b < QBINS; b++) {
    const idx = bins[b];
    if (!idx.length) continue;
    ctx.fillStyle = BIN_COLOURS[b];
    for (let k = 0; k < idx.length; k++) {
      const i = idx[k];
      const a = scan.angle[i] * DEG;
      const r = scan.range[i] * scale;
      const x = cx + Math.sin(a) * r;
      const y = cy - Math.cos(a) * r;
      ctx.fillRect(x - 1, y - 1, 2, 2);
    }
  }
}

export function PolarPlot() {
  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [fixedR, setFixedR] = useState(0);
  const fixedRRef = useRef(fixedR);
  const dirtyRef = useRef(true);
  const [info, setInfo] = useState<{ seq: number; n: number; phi: number; t: number } | null>(null);

  useEffect(() => {
    fixedRRef.current = fixedR;
    dirtyRef.current = true;
  }, [fixedR]);

  useEffect(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    let cssW = 0;
    let cssH = 0;
    let dpr = 1;
    const resize = () => {
      dpr = Math.min(2, window.devicePixelRatio || 1);
      cssW = wrap.clientWidth;
      cssH = wrap.clientHeight;
      canvas.width = Math.max(1, Math.round(cssW * dpr));
      canvas.height = Math.max(1, Math.round(cssH * dpr));
      dirtyRef.current = true;
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(wrap);

    let latest: ScanFrame | null = useStore.getState().scan;
    let lastInfoAt = 0;
    const unsub = useStore.subscribe((s) => {
      if (s.scan !== latest) {
        latest = s.scan;
        dirtyRef.current = true;
      }
    });

    const ds: DrawState = { autoR: 5 };
    let raf = 0;
    const loop = () => {
      raf = requestAnimationFrame(loop);
      if (!dirtyRef.current) return;
      dirtyRef.current = false;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw(ctx, cssW, cssH, latest, fixedRRef.current, ds);
      const now = performance.now();
      if (latest && now - lastInfoAt > 250) {
        lastInfoAt = now;
        setInfo({ seq: latest.seq, n: latest.n, phi: latest.phi_deg, t: latest.t_unix });
      }
    };
    raf = requestAnimationFrame(loop);

    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      unsub();
    };
  }, []);

  return (
    <>
      <div className="canvas-toolbar">
        <label>
          Range
          <select value={fixedR} onChange={(e) => setFixedR(Number(e.target.value))}>
            {RANGE_OPTIONS.map((o) => (
              <option key={o.v} value={o.v}>
                {o.label}
              </option>
            ))}
          </select>
        </label>
        <span className="spacer" />
        <span className="mono dim">
          {info
            ? `seq ${info.seq} · ${info.n} pts · φ ${info.phi.toFixed(1)}°`
            : "waiting for scan frames"}
        </span>
      </div>
      <div className="polar-wrap" ref={wrapRef}>
        <canvas ref={canvasRef} />
      </div>
    </>
  );
}
