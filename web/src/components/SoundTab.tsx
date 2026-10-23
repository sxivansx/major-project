import { useEffect, useState } from "react";
import { api, errorText } from "../api/rest";
import { fmt, fmtInt } from "../lib/format";
import {
  RECENT_TEXT_S,
  audioConnect,
  audioDisconnect,
  clearAudioEvents,
  clearWave,
  loadAudioEvents,
  useAudio,
  useCoreNow,
} from "../store/audio";
import { useStore } from "../store/store";
import { Compass, EventList, GeometryForm, TdoaTable, TriggerForm, Waveforms } from "./AudioViews";
import { Shell, useAdvancedFlag, type BigNumber } from "./Shell";

// Sound: four microphone waveforms, the detected spikes, and the direction
// each came from. The array gives a direction only, never a distance.

const PORT_KEY = "obslam.port.audio";
function loadPort(): string {
  try {
    return localStorage.getItem(PORT_KEY) ?? "";
  } catch {
    return "";
  }
}

function AudioConnectBar() {
  const audio = useStore((s) => s.status?.audio);
  const [source, setSource] = useState<"tcp" | "sim">("tcp");
  const [port, setPortState] = useState(loadPort);
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const setPort = (v: string) => {
    setPortState(v);
    try {
      localStorage.setItem(PORT_KEY, v);
    } catch {
      // storage blocked: not remembered
    }
  };
  const run = async (key: string, fn: () => Promise<unknown>) => {
    setErr(null);
    setBusy(key);
    try {
      await fn();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };
  const connected = !!audio?.connected;

  return (
    <>
      {connected ? (
        <>
          <button disabled={busy !== null} onClick={() => void run("off", audioDisconnect)}>
            Disconnect audio
          </button>
          <span className="mono dim">{audio?.source === "sim" ? "simulator" : audio?.port}</span>
        </>
      ) : (
        <>
          <select value={source} onChange={(e) => setSource(e.target.value as "tcp" | "sim")} title="Source">
            <option value="tcp">tcp (ESP32 array)</option>
            <option value="sim">sim (simulator)</option>
          </select>
          <input
            className="mono audio-port"
            value={port}
            disabled={source === "sim"}
            placeholder="tcp://192.168.4.7:8901"
            spellCheck={false}
            onChange={(e) => setPort(e.target.value)}
          />
          <button
            className="primary"
            disabled={busy !== null || (source === "tcp" && !port.trim())}
            onClick={() =>
              void run("on", async () => {
                clearWave();
                await audioConnect(source, source === "sim" ? "sim" : port.trim());
                await loadAudioEvents();
              })
            }
          >
            Connect audio
          </button>
        </>
      )}
      <button
        disabled={!connected || audio?.source !== "sim" || busy !== null}
        title="Simulator only: inject a knock from a random direction"
        onClick={() =>
          void run("knock", () =>
            api.audioEmit({ bearing_deg: Math.round(Math.random() * 360), elevation_deg: Math.round(Math.random() * 30), snr_db: 14 + Math.round(Math.random() * 16) }),
          )
        }
      >
        Test knock
      </button>
      <button disabled={busy !== null} onClick={() => void run("clear", clearAudioEvents)}>
        Clear events
      </button>
      {err && <span className="err mono">{err}</span>}
    </>
  );
}

export function SoundTab() {
  const [advanced, setAdvanced] = useAdvancedFlag("sound");
  const status = useStore((s) => s.status);
  const config = useStore((s) => s.config);
  const events = useAudio((s) => s.events);
  const levels = useAudio((s) => s.levels);
  const selectedId = useAudio((s) => s.selectedId);
  const eventsError = useAudio((s) => s.eventsError);
  const audio = status?.audio;
  const supported = !!audio || !!config?.audio;
  const connected = !!audio?.connected;
  const now = useCoreNow(events.length > 0);

  useEffect(() => {
    if (supported) void loadAudioEvents();
  }, [supported]);

  const lv = connected ? (levels ?? audio?.levels_dbfs ?? null) : null;
  const loudest = lv && lv.length ? Math.max(...lv) : null;
  const noise = connected && audio?.noise_dbfs?.length ? audio.noise_dbfs.reduce((a, b) => a + b, 0) / audio.noise_dbfs.length : null;
  const clipping = connected && !!audio?.clipping?.some(Boolean);
  const calibrated = config?.audio?.calibrated ?? audio?.calibrated ?? null;

  const numbers: BigNumber[] = [
    {
      label: "Level",
      value:
        loudest !== null ? (
          <>
            {fmt(loudest, 0)}
            <small>dBFS{clipping ? " · clipping" : ""}</small>
          </>
        ) : (
          "–"
        ),
      className: clipping ? "err" : "",
    },
    {
      label: "Noise floor",
      value:
        noise !== null ? (
          <>
            {fmt(noise, 0)}
            <small>dBFS</small>
          </>
        ) : (
          "–"
        ),
    },
    { label: "Events", value: fmtInt(events.length) },
    { label: "Calibrated", value: calibrated === null ? "–" : calibrated ? "yes" : "no", className: calibrated === false ? "warn" : "" },
  ];

  const recent = events.filter((e) => now - e.t < RECENT_TEXT_S);
  const selected = events.find((e) => e.id === selectedId) ?? events[0] ?? null;

  return (
    <Shell
      numbers={numbers}
      main={
        <section className="panel ops-mappanel">
          <div className="panel-head">
            <span>Microphones</span>
            <span className="sub">
              {connected
                ? `${audio?.rate ? `${(audio.rate / 1000).toFixed(0)} kHz` : ""} · ${fmtInt(audio?.frames)} frames${audio?.lost_frames ? ` · ${fmtInt(audio.lost_frames)} lost` : ""}`
                : "not connected"}
            </span>
          </div>
          {!supported ? <div className="empty ops-connect">This core has no audio layer yet.</div> : <Waveforms />}
        </section>
      }
      side={
        <section className="panel ops-right">
          <div className="panel-head">
            <span>Direction</span>
            <span className="sub">0° = lidar cable mark, clockwise</span>
          </div>
          <div className="panel-body ops-summary">
            <div className="compass-wrap">
              <Compass events={events} now={now} />
            </div>
            <div className="hint">
              Direction only: the array cannot tell how far away a sound is. Wedges are ± 2σ; recent ones last {RECENT_TEXT_S / 60} min.
            </div>
            {eventsError && supported && <div className="error">events: {eventsError}</div>}
            <EventList events={recent.length ? recent : events.slice(0, 20)} />
          </div>
        </section>
      }
      actions={<AudioConnectBar />}
      advanced={
        <div className="ops-advanced-grid cols-3">
          <div className="ops-advanced-col">
            <section className="panel">
              <div className="panel-head">
                <span>Time differences</span>
                <span className="sub">{selected ? `event #${selected.id}` : ""}</span>
              </div>
              <div className="panel-body">
                <TdoaTable ev={selected} />
              </div>
            </section>
          </div>
          <div className="ops-advanced-col">
            <section className="panel">
              <div className="panel-head">
                <span>Trigger</span>
              </div>
              <div className="panel-body">
                <TriggerForm />
              </div>
            </section>
          </div>
          <div className="ops-advanced-col">
            <section className="panel">
              <div className="panel-head">
                <span>Mic geometry</span>
                <span className="sub">{config?.audio?.calibrated ? "calibrated" : "not calibrated"}</span>
              </div>
              <div className="panel-body">
                <GeometryForm />
              </div>
            </section>
          </div>
        </div>
      }
      advancedOpen={advanced}
      onAdvanced={setAdvanced}
    />
  );
}
