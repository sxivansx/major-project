// WebSocket client for /ws. Reconnects with exponential backoff, parses
// binary frames off the React render path and pushes into the store with
// throttling: scans at most every SCAN_MIN_MS (latest wins), cloud count at
// most every CLOUD_MIN_MS. Status/job/log text messages go straight in.

import { FrameError, parseFrame } from "./frames";
import type { Job, ScanFrame, Status, WsTextMessage } from "./types";
import { cloud, cloudAppend } from "../store/cloud";
import { clientLog, useStore } from "../store/store";
import { audioAddEvent, audioPushFrame, audioSetSources } from "../store/audio";
import type { AudioEvent, AudioSource } from "./types";

const SCAN_MIN_MS = 100;
const CLOUD_MIN_MS = 200;
const BACKOFF_BASE_MS = 500;
const BACKOFF_MAX_MS = 10_000;
const PING_MS = 20_000;

export function wsUrl(): string {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws`;
}

class WsClient {
  private sock: WebSocket | null = null;
  private stopped = true;
  private attempt = 0;
  private retryTimer: number | undefined;
  private pingTimer: number | undefined;

  // scan throttle
  private lastScanPush = 0;
  private pendingScan: ScanFrame | null = null;
  private scanTimer: number | undefined;
  private scanTimes: number[] = [];

  // cloud throttle
  private lastCloudPush = 0;
  private cloudTimer: number | undefined;

  start(): void {
    if (!this.stopped) return;
    this.stopped = false;
    this.attempt = 0;
    this.connect();
  }

  stop(): void {
    this.stopped = true;
    window.clearTimeout(this.retryTimer);
    window.clearInterval(this.pingTimer);
    window.clearTimeout(this.scanTimer);
    window.clearTimeout(this.cloudTimer);
    if (this.sock) {
      this.sock.onclose = null;
      this.sock.close();
      this.sock = null;
    }
    useStore.getState().setWs({ wsState: "closed", wsRetryAt: null });
  }

  // Force an immediate reconnect attempt (e.g. a "reconnect now" button).
  reconnectNow(): void {
    if (this.stopped) return this.start();
    window.clearTimeout(this.retryTimer);
    if (this.sock && this.sock.readyState === WebSocket.OPEN) return;
    this.attempt = 0;
    this.connect();
  }

  private connect(): void {
    if (this.stopped) return;
    const url = wsUrl();
    useStore.getState().setWs({ wsState: "connecting", wsAttempt: this.attempt, wsRetryAt: null });
    let sock: WebSocket;
    try {
      sock = new WebSocket(url);
    } catch (e) {
      clientLog("error", `ws: ${(e as Error).message}`);
      this.scheduleRetry();
      return;
    }
    sock.binaryType = "arraybuffer";
    this.sock = sock;

    sock.onopen = () => {
      this.attempt = 0;
      useStore.getState().setWs({ wsState: "open", wsAttempt: 0, wsRetryAt: null });
      clientLog("info", `ws: connected to ${url}`);
      window.clearInterval(this.pingTimer);
      this.pingTimer = window.setInterval(() => {
        if (sock.readyState === WebSocket.OPEN) sock.send(JSON.stringify({ type: "ping" }));
      }, PING_MS);
    };

    sock.onmessage = (ev: MessageEvent) => {
      if (typeof ev.data === "string") this.onText(ev.data);
      else if (ev.data instanceof ArrayBuffer) this.onBinary(ev.data);
    };

    sock.onerror = () => {
      // onclose follows; nothing useful in the event itself
    };

    sock.onclose = (ev) => {
      window.clearInterval(this.pingTimer);
      if (this.sock === sock) this.sock = null;
      if (this.stopped) return;
      clientLog("warn", `ws: closed (code ${ev.code})${ev.reason ? " " + ev.reason : ""}`);
      this.scheduleRetry();
    };
  }

  private scheduleRetry(): void {
    if (this.stopped) return;
    const base = Math.min(BACKOFF_MAX_MS, BACKOFF_BASE_MS * 2 ** this.attempt);
    const delay = Math.round(base * (0.8 + Math.random() * 0.4));
    this.attempt++;
    useStore.getState().setWs({
      wsState: "closed",
      wsAttempt: this.attempt,
      wsRetryAt: performance.now() + delay,
    });
    window.clearTimeout(this.retryTimer);
    this.retryTimer = window.setTimeout(() => this.connect(), delay);
  }

  private onText(data: string): void {
    let msg: WsTextMessage;
    try {
      msg = JSON.parse(data);
    } catch {
      clientLog("warn", `ws: unparseable text frame (${data.length} chars)`);
      return;
    }
    const st = useStore.getState();
    switch (msg.type) {
      case "status": {
        const { type: _t, ...status } = msg as { type: "status" } & Status;
        st.setStatus(status);
        break;
      }
      case "job": {
        const { type: _t, ...job } = msg as { type: "job" } & Job;
        st.setJob(job);
        break;
      }
      case "log": {
        const m = msg as { level?: string; msg?: string; t?: number };
        const level = m.level === "warn" || m.level === "error" ? m.level : "info";
        st.addLog(level, String(m.msg ?? ""), typeof m.t === "number" ? m.t : undefined, "server");
        break;
      }
      case "audio_event": {
        const { type: _t, ...ev } = msg as unknown as { type: "audio_event" } & AudioEvent;
        audioAddEvent(ev as AudioEvent);
        break;
      }
      case "audio_sources": {
        const m = msg as unknown as { active?: AudioSource[] };
        audioSetSources(Array.isArray(m.active) ? m.active : []);
        break;
      }
      case "pong":
        break;
      default:
        clientLog("warn", `ws: unknown message type "${msg.type}"`);
    }
  }

  private onBinary(buf: ArrayBuffer): void {
    let frame;
    try {
      frame = parseFrame(buf);
    } catch (e) {
      if (e instanceof FrameError) clientLog("error", `ws: bad binary frame: ${e.message}`);
      else throw e;
      return;
    }
    switch (frame.kind) {
      case 1:
        this.pushScan(frame);
        break;
      case 2:
        cloudAppend(frame.xyz, frame.n, frame.job_id);
        this.pushCloud();
        break;
      case 3:
        useStore.getState().setGrid(frame);
        break;
      case 5:
        audioPushFrame(frame);
        break;
    }
  }

  private pushScan(f: ScanFrame): void {
    const now = performance.now();
    const since = now - this.lastScanPush;
    if (since >= SCAN_MIN_MS) {
      this.lastScanPush = now;
      this.pendingScan = null;
      this.commitScan(f);
    } else {
      this.pendingScan = f;
      if (this.scanTimer === undefined) {
        this.scanTimer = window.setTimeout(() => {
          this.scanTimer = undefined;
          if (this.pendingScan) {
            this.lastScanPush = performance.now();
            const p = this.pendingScan;
            this.pendingScan = null;
            this.commitScan(p);
          }
        }, SCAN_MIN_MS - since);
      }
    }
  }

  private commitScan(f: ScanFrame): void {
    const times = this.scanTimes;
    times.push(f.t_unix);
    if (times.length > 20) times.shift();
    let hz = 0;
    if (times.length >= 2) {
      const span = times[times.length - 1] - times[0];
      if (span > 0) hz = (times.length - 1) / span;
    }
    useStore.getState().setScan(f, hz);
  }

  private pushCloud(): void {
    const now = performance.now();
    const since = now - this.lastCloudPush;
    const commit = () => {
      this.lastCloudPush = performance.now();
      useStore.getState().setCloud(cloud.count, cloud.version);
    };
    if (since >= CLOUD_MIN_MS) commit();
    else if (this.cloudTimer === undefined) {
      this.cloudTimer = window.setTimeout(() => {
        this.cloudTimer = undefined;
        commit();
      }, CLOUD_MIN_MS - since);
    }
  }
}

export const ws = new WsClient();
