export function fmt(v: number | null | undefined, digits = 2, unit = ""): string {
  if (v === null || v === undefined || Number.isNaN(v)) return "–";
  return v.toFixed(digits) + unit;
}

export function fmtInt(v: number | null | undefined): string {
  if (v === null || v === undefined || Number.isNaN(v)) return "–";
  return Math.round(v).toLocaleString("en-US");
}

export function fmtTime(unixSeconds: number): string {
  const d = new Date(unixSeconds * 1000);
  return d.toLocaleTimeString("en-GB", { hour12: false });
}

export function fmtDateTime(unixSeconds: number): string {
  const d = new Date(unixSeconds * 1000);
  return d.toLocaleString("en-GB", { hour12: false });
}

export function fmtDuration(s: number | null | undefined): string {
  if (s === null || s === undefined || !Number.isFinite(s)) return "–";
  s = Math.max(0, Math.round(s));
  const m = Math.floor(s / 60);
  const r = s % 60;
  return m > 0 ? `${m}m ${r.toString().padStart(2, "0")}s` : `${r}s`;
}

export function text(v: string | number | null | undefined): string {
  if (v === null || v === undefined || v === "") return "–";
  return String(v);
}

// Assistant call cost: $0.005, $0.0003, $0.0012 (up to 4 decimals, no trailing zeros)
export function fmtUsd(v: number | null | undefined): string {
  if (v === null || v === undefined || !Number.isFinite(v)) return "–";
  let s = v.toFixed(4).replace(/0+$/, "");
  if (s.endsWith(".")) s += "00";
  return "$" + s;
}
