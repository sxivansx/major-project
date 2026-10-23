import { useEffect } from "react";
import { api, errorText } from "./api/rest";
import { ws } from "./api/ws";
import { startMetricsFollow } from "./store/metrics";
import { clientLog, useStore, type Tab } from "./store/store";
import { OpsTab } from "./components/OpsTab";
import { LiveTab } from "./components/LiveTab";
import { SoundTab } from "./components/SoundTab";
import { loadAudioEvents, loadAudioSources } from "./store/audio";
import { CloudTab } from "./components/CloudTab";
import { SessionsPanel } from "./components/SessionsPanel";
import { LogPanel } from "./components/LogPanel";
import { StatusBar } from "./components/StatusBar";

// Every tab is one shell: numbers on top, a main view left, a plain-language
// column right, an action bar, and an Advanced drawer with the engineering
// panels (connection, servo, mount, scan job). No sidebar.

const TABS: { id: Tab; label: string }[] = [
  { id: "ops", label: "Operator" },
  { id: "live", label: "Live scan" },
  { id: "sound", label: "Sound" },
  { id: "cloud", label: "Point cloud and height map" },
  { id: "sessions", label: "Sessions" },
  { id: "log", label: "Log" },
];

async function bootstrap() {
  const st = useStore.getState();
  const results = await Promise.allSettled([api.status(), api.config(), api.ports(), api.sessions()]);
  const [status, config, ports, sessions] = results;
  if (status.status === "fulfilled") st.setStatus(status.value);
  else clientLog("error", `GET /api/status failed: ${errorText(status.reason)}`);
  if (config.status === "fulfilled") {
    const c = config.value;
    st.setConfig(c);
    st.setUi({
      colorZ: [c.grid.z_min_m, c.grid.z_max_m],
      gridForm: { cell: String(c.grid.cell_m), layer: c.grid.layer, zMin: String(c.grid.z_min_m), zMax: String(c.grid.z_max_m) },
    });
  } else clientLog("error", `GET /api/config failed: ${errorText(config.reason)}`);
  if (ports.status === "fulfilled") st.setPorts(ports.value);
  else clientLog("error", `GET /api/ports failed: ${errorText(ports.reason)}`);
  if (sessions.status === "fulfilled") st.setSessions(sessions.value);
  else clientLog("error", `GET /api/sessions failed: ${errorText(sessions.reason)}`);
  // older cores have no audio layer: loadAudioEvents keeps the error to itself
  if (status.status === "fulfilled" && status.value.audio) {
    void loadAudioEvents();
    void loadAudioSources();
  }
}

export default function App() {
  const tab = useStore((s) => s.tab);
  const setTab = useStore((s) => s.setTab);

  useEffect(() => {
    ws.start();
    void bootstrap();
    const stopMetrics = startMetricsFollow();
    return () => {
      stopMetrics();
      ws.stop();
    };
  }, []);

  return (
    <div className="app">
      <main className="main">
        <nav className="tabs">
          {TABS.map((t) => (
            <button key={t.id} className={"tab" + (tab === t.id ? " active" : "")} onClick={() => setTab(t.id)}>
              {t.label}
            </button>
          ))}
        </nav>
        <div className="tab-body">
          {tab === "ops" && <OpsTab />}
          {tab === "live" && <LiveTab />}
          {tab === "sound" && <SoundTab />}
          {tab === "cloud" && <CloudTab />}
          {tab === "sessions" && <SessionsPanel />}
          {tab === "log" && <LogPanel />}
        </div>
      </main>
      <StatusBar />
    </div>
  );
}
