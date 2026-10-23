// Acoustic layer state (docs/api.md §5). Two parts:
//  - `wave`: the kind-5 envelopes in a 1 kHz ring buffer per channel, kept
//    outside React (like the point cloud); the waveform canvas reads it in
//    requestAnimationFrame.
//  - `useAudio`: events (newest first), the selection, and throttled levels.
// Time on the waveform and the age of events are on the core's clock: the
// offset between the core and this browser is estimated from the frames.

import { useEffect, useState } from "react";
import { create } from "zustand";
import { api, errorText } from "../api/rest";
import type { AudioEvent, AudioFrame, AudioSource, Status } from "../api/types";
import { clientLog, useStore } from "./store";

export const RING_MS = 6000;
export const MAX_CH = 8;
export const CLIP = 32700; // |sample| at or above this reads as clipping

export const wave = {
  channels: 4,
  mins: Array.from({ length: MAX_CH }, () => new Int16Array(RING_MS)),
  maxs: Array.from({ length: MAX_CH }, () => new Int16Array(RING_MS)),
  stamp: new Float64Array(RING_MS).fill(-1), // ms (core clock) held by each slot
  levels: new Float32Array(MAX_CH).fill(NaN), // dBFS of the newest frame
  latestMs: 0, // end of the newest frame, core clock
  version: 0,
  lastSeq: -1,
  gaps: 0, // frames missing between received sequence numbers
};

// core clock = local clock - offset
let clockOffsetMs = 0;
let haveOffset = false;
export function coreNowSec(): number {
  return (Date.now() - clockOffsetMs) / 1000;
}

const MAX_EVENTS = 200;
const LEVEL_PUSH_MS = 250;
let lastLevelPush = 0;

export interface AudioState {
  events: AudioEvent[]; // newest first
  sources: AudioSource[]; // active sources (§5.6)
  sourcesSupported: boolean | null; // null until GET /audio/sources answered
  highlightSourceId: number | null; // "Show on map" from the Sound tab
  selectedId: number | null;
  eventsError: string | null;
  levels: number[] | null; // dBFS per channel from the latest frame (throttled)
  framesSeen: number;
  setSelected: (id: number | null) => void;
}

export const useAudio = create<AudioState>((set) => ({
  events: [],
  sources: [],
  sourcesSupported: null,
  highlightSourceId: null,
  selectedId: null,
  eventsError: null,
  levels: null,
  framesSeen: 0,
  setSelected: (selectedId) => set({ selectedId }),
}));

export function audioPushFrame(f: AudioFrame): void {
  const ch = Math.min(f.channels, MAX_CH);
  wave.channels = ch;
  // t_unix is taken as the time of the last envelope point of the block
  const endMs = Math.round(f.t_unix * 1000);
  for (let i = 0; i < f.n; i++) {
    const ms = endMs - (f.n - 1 - i);
    const k = ((ms % RING_MS) + RING_MS) % RING_MS;
    wave.stamp[k] = ms;
    for (let c = 0; c < ch; c++) {
      wave.mins[c][k] = f.mins[c][i];
      wave.maxs[c][k] = f.maxs[c][i];
    }
  }
  if (endMs > wave.latestMs) wave.latestMs = endMs;
  for (let c = 0; c < ch; c++) wave.levels[c] = f.levels_dbfs[c];
  if (wave.lastSeq >= 0 && f.seq > wave.lastSeq + 1) wave.gaps += f.seq - wave.lastSeq - 1;
  wave.lastSeq = f.seq;
  wave.version++;
  const off = Date.now() - endMs;
  clockOffsetMs = haveOffset ? clockOffsetMs * 0.9 + off * 0.1 : off;
  haveOffset = true;
  const now = performance.now();
  if (now - lastLevelPush >= LEVEL_PUSH_MS) {
    lastLevelPush = now;
    useAudio.setState((s) => ({ levels: Array.from(f.levels_dbfs.subarray(0, ch)), framesSeen: s.framesSeen + 1 }));
  }
}

export function audioAddEvent(ev: AudioEvent): void {
  useAudio.setState((s) => {
    const rest = s.events.filter((e) => e.id !== ev.id);
    return { events: [ev, ...rest].slice(0, MAX_EVENTS) };
  });
}

export function audioSetSources(active: AudioSource[]): void {
  useAudio.setState({ sources: active, sourcesSupported: true });
}

export async function loadAudioSources(): Promise<void> {
  try {
    const r = await api.audioSources();
    useAudio.setState({ sources: r.active ?? [], sourcesSupported: true });
  } catch {
    // a core without §5.6 has no sources: the views fall back to raw events
    useAudio.setState({ sourcesSupported: false });
  }
}

export async function loadAudioEvents(): Promise<void> {
  try {
    const evs = await api.audioEvents(50);
    const sorted = [...evs].sort((a, b) => b.t - a.t);
    useAudio.setState((s) => {
      const seen = new Set(sorted.map((e) => e.id));
      const merged = [...sorted, ...s.events.filter((e) => !seen.has(e.id))].sort((a, b) => b.t - a.t);
      return { events: merged.slice(0, MAX_EVENTS), eventsError: null };
    });
  } catch (e) {
    useAudio.setState({ eventsError: errorText(e) });
  }
}

export async function clearAudioEvents(): Promise<void> {
  await api.audioClear();
  useAudio.setState({ events: [], selectedId: null });
  await loadAudioSources();
}

export function clearWave(): void {
  wave.stamp.fill(-1);
  wave.latestMs = 0;
  wave.lastSeq = -1;
  wave.gaps = 0;
  wave.version++;
}

export async function audioConnect(source: "tcp" | "sim", port: string): Promise<void> {
  try {
    useStore.getState().setStatus(await api.audioConnect({ source, port }));
  } catch (e) {
    clientLog("error", `audio/connect: ${errorText(e)}`);
    throw e;
  }
}

export async function audioDisconnect(): Promise<void> {
  try {
    useStore.getState().setStatus(await api.audioDisconnect());
  } catch (e) {
    clientLog("error", `audio/disconnect: ${errorText(e)}`);
    throw e;
  }
}

export function audioConnected(s: Status | null | undefined): boolean {
  return !!s?.audio?.connected;
}

// ---- helpers shared by the Sound and Operator tabs ----

export const RECENT_MAP_S = 60; // wedges fade out over this
export const RECENT_TEXT_S = 300; // events described in "What we have"
export const LOW_CONFIDENCE = 0.4;

// Current core time in seconds, re-rendering every `ms` while `active`.
export function useCoreNow(active: boolean, ms = 1000): number {
  const [now, setNow] = useState(coreNowSec);
  useEffect(() => {
    setNow(coreNowSec());
    if (!active) return;
    const id = window.setInterval(() => setNow(coreNowSec()), ms);
    return () => window.clearInterval(id);
  }, [active, ms]);
  return now;
}

export function eventWord(ev: AudioEvent): string {
  const l = (ev.label ?? "").toLowerCase();
  if (l === "impulse" || l === "knock" || l === "") return "Knock";
  return l.charAt(0).toUpperCase() + l.slice(1);
}

export function bearingText(ev: AudioEvent): string {
  if (ev.bearing_deg === null || ev.bearing_deg === undefined) return "–";
  const sig = ev.bearing_sigma_deg;
  return `${Math.round(ev.bearing_deg)}°${sig !== null && sig !== undefined ? ` ± ${Math.round(sig)}°` : ""}`;
}

export function candidateWord(kind: string): string {
  switch (kind) {
    case "void":
      return "void";
    case "shadow":
    case "unknown":
    case "blind":
      return "unseen region";
    case "obstacle":
      return "behind obstacle";
    case "depression":
      return "depression";
    default:
      return kind;
  }
}

// TDOA (µs) a source at bearing / elevation would give for each mic pair,
// with the "ij" = t_i − t_j convention: positive when mic i hears it later.
export function predictTdoa(
  mics: [number, number, number][],
  yawDeg: number,
  bearingDeg: number,
  elevationDeg: number,
  c: number,
): Record<string, number> {
  const b = (bearingDeg * Math.PI) / 180;
  const e = (elevationDeg * Math.PI) / 180;
  // base frame: bearing clockwise from +x
  const ux = Math.cos(b) * Math.cos(e);
  const uy = -Math.sin(b) * Math.cos(e);
  const uz = Math.sin(e);
  // into the array frame (array x is yaw CCW from base x)
  const y = (yawDeg * Math.PI) / 180;
  const ax = Math.cos(y) * ux + Math.sin(y) * uy;
  const ay = -Math.sin(y) * ux + Math.cos(y) * uy;
  const arrival = mics.map((p) => -(p[0] * ax + p[1] * ay + p[2] * uz) / 1000 / c); // s, relative
  const out: Record<string, number> = {};
  for (let i = 0; i < mics.length; i++)
    for (let j = i + 1; j < mics.length; j++) out[`${i + 1}${j + 1}`] = (arrival[i] - arrival[j]) * 1e6;
  return out;
}

// ---- sources (§5.6) ----

// Half-width of a source's wedge: the larger of its spread and its sigma.
export function sourceHalfWidth(s: AudioSource): number {
  return Math.max(s.spread_deg || 0, s.sigma_deg || 0, 1);
}

export function sourceBearingText(s: AudioSource): string {
  return `${Math.round(s.bearing_deg)}° ± ${Math.round(sourceHalfWidth(s))}°`;
}

export function sourceKindWord(s: AudioSource): string {
  return s.kind === "tapping" ? "Tapping" : "Sound";
}

export function knocksText(n: number, kind: string): string {
  const unit = kind === "tapping" ? "knock" : "sound";
  return `${n} ${unit}${n === 1 ? "" : "s"}`;
}

export function agoText(seconds: number): string {
  const s = Math.max(0, Math.round(seconds));
  if (s < 60) return `${s} s ago`;
  const m = Math.floor(s / 60);
  return `${m} min ${s % 60 < 10 ? "0" : ""}${s % 60} s ago`;
}

// Operator order: repeated tapping first (the signal that matters), then by
// confidence, newest on a tie.
export function sourceRank(a: AudioSource, b: AudioSource): number {
  const ta = a.kind === "tapping" ? 1 : 0;
  const tb = b.kind === "tapping" ? 1 : 0;
  if (ta !== tb) return tb - ta;
  if (a.confidence !== b.confidence) return b.confidence - a.confidence;
  return b.last_t - a.last_t;
}

export function strongestSource(sources: AudioSource[]): AudioSource | null {
  return sources.length ? [...sources].sort(sourceRank)[0] : null;
}

// excluded_mics → [{mic (1-based), reason}]
export function excludedList(st: Status | null | undefined): { mic: number; reason: string }[] {
  const raw = st?.audio?.excluded_mics ?? [];
  return raw
    .map((x) => (typeof x === "number" ? { mic: x, reason: "" } : { mic: x.mic ?? x.channel ?? 0, reason: x.reason ?? "" }))
    .filter((x) => x.mic > 0);
}
