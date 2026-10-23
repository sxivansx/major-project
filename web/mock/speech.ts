// Spoken calls for help for the mock core (docs/api.md §5.7): while audio is
// connected and speech is enabled, a scripted spoken segment every 15–25 s,
// some of them calls for help (exact words, a sound-alike, a high distress
// probability), with a direction from the array. A flagged one raises the
// "speech_help" alarm for 30 s.

import type { SpeechEvent, SpeechStatus } from "../src/api/types";

const SCRIPT: Omit<SpeechEvent, "id" | "t" | "bearing_deg" | "bearing_sigma_deg" | "confidence" | "calibrated" | "stt_ms" | "jev_ms" | "cost_usd">[] = [
  { duration_s: 1.4, transcript: "bachao! koi hai?", language: "hi-IN", matched: ["bachao"], sounds_like: [], distress_p: 0.93, flagged: true, reasons: ["exact help word: bachao"], jev_error: null },
  { duration_s: 1.1, transcript: "is anyone there", language: "en-IN", matched: [], sounds_like: [], distress_p: 0.35, flagged: false, reasons: [], jev_error: null },
  {
    duration_s: 0.9,
    transcript: "safe me please",
    language: "en-IN",
    matched: [],
    sounds_like: [{ word: "save me", heard: "safe me" }],
    distress_p: 0.71,
    flagged: true,
    reasons: ["sounds like a help word: save me"],
    jev_error: null,
  },
  { duration_s: 1.6, transcript: "testing one two three", language: "en-IN", matched: [], sounds_like: [], distress_p: 0.04, flagged: false, reasons: [], jev_error: null },
  { duration_s: 1.2, transcript: "madad karo", language: "hi-IN", matched: ["madad"], sounds_like: [], distress_p: 0.88, flagged: true, reasons: ["exact help word: madad"], jev_error: null },
];

const r = (v: number, d: number) => Math.round(v * 10 ** d) / 10 ** d;

export class SpeechSim {
  enabled: boolean;
  readonly key = process.env.MOCK_SPEECH_KEY !== "off";
  events: SpeechEvent[] = []; // newest first
  private nextId = 1;
  private step = 0;
  private timer: ReturnType<typeof setTimeout> | null = null;
  private flagUntil = 0;
  private flagText = "";
  private stats = { segments: 0, transcribed: 0, dropped: 0, rate_limited: 0, errors: 0 };
  private running = false;

  constructor(
    private bearingHint: () => number, // where the "person" is
    private calibrated: () => boolean,
    private send: (ev: SpeechEvent) => void,
    private log: (level: "info" | "warn", msg: string) => void,
  ) {
    this.enabled = this.key;
  }

  status(): SpeechStatus {
    return {
      enabled: this.enabled,
      key: this.key,
      model: "saaras:v4",
      judge: "typesafe/jev-1.14",
      ...this.stats,
      queued: 0,
      last_error: null,
    };
  }

  alarm(): string | null {
    return Date.now() < this.flagUntil ? this.flagText : null;
  }

  // audio connected / disconnected
  start(): void {
    this.running = true;
    this.schedule(2000);
  }

  stop(): void {
    this.running = false;
    if (this.timer) clearTimeout(this.timer);
    this.timer = null;
  }

  setEnabled(on: boolean): void {
    this.enabled = on && this.key;
    if (this.running) this.schedule(this.enabled ? 1500 : -1);
  }

  clear(): void {
    this.events = [];
  }

  private schedule(ms: number) {
    if (this.timer) clearTimeout(this.timer);
    this.timer = null;
    if (ms < 0 || !this.running) return;
    this.timer = setTimeout(() => {
      this.timer = null;
      if (this.running && this.enabled) this.speak();
      this.schedule(15000 + Math.random() * 10000);
    }, ms);
  }

  private speak() {
    const s = SCRIPT[this.step++ % SCRIPT.length];
    this.stats.segments++;
    this.stats.transcribed++;
    const sigma = 5 + Math.random() * 4;
    const bearing = s.flagged ? (this.bearingHint() + (Math.random() * 2 - 1) * sigma + 360) % 360 : Math.random() * 360;
    const ev: SpeechEvent = {
      ...s,
      id: this.nextId++,
      t: Date.now() / 1000,
      bearing_deg: r(bearing, 1),
      bearing_sigma_deg: r(sigma, 1),
      confidence: r(0.6 + Math.random() * 0.25, 2),
      calibrated: this.calibrated(),
      stt_ms: Math.round(600 + Math.random() * 300),
      jev_ms: Math.round(300 + Math.random() * 200),
      cost_usd: 0.0004,
    };
    this.events.unshift(ev);
    if (this.events.length > 100) this.events.length = 100;
    if (ev.flagged) {
      this.flagUntil = Date.now() + 30000;
      this.flagText = `call for help: "${ev.transcript}" from ${Math.round(bearing)}° ± ${Math.round(sigma)}°`;
      this.log("warn", this.flagText);
    }
    this.send(ev);
  }
}
