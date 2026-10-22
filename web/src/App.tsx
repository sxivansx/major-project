import { useEffect } from "react";
import { api, errorText } from "./api/rest";
import { ws } from "./api/ws";
import { clientLog, useStore, type Tab } from "./store/store";
import { ConnectionPanel } from "./components/ConnectionPanel";
import { ServoPanel } from "./components/ServoPanel";
import { MountPanel } from "./components/MountPanel";
import { JobPanel } from "./components/JobPanel";
import { LiveTab } from "./components/LiveTab";
import { CloudTab } from "./components/CloudTab";
import { SessionsPanel } from "./components/SessionsPanel";
import { LogPanel } from "./components/LogPanel";
import { StatusBar } from "./components/StatusBar";

const TABS: { id: Tab; label: string }[] = [
  { id: "live", label: "Live scan" },
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
    st.setConfig(config.value);
    st.setUi({ colorZ: [config.value.grid.z_min_m, config.value.grid.z_max_m] });
  } else clientLog("error", `GET /api/config failed: ${errorText(config.reason)}`);
  if (ports.status === "fulfilled") st.setPorts(ports.value);
  else clientLog("error", `GET /api/ports failed: ${errorText(ports.reason)}`);
  if (sessions.status === "fulfilled") st.setSessions(sessions.value);
  else clientLog("error", `GET /api/sessions failed: ${errorText(sessions.reason)}`);
}

export default function App() {
  const tab = useStore((s) => s.tab);
  const setTab = useStore((s) => s.setTab);

  useEffect(() => {
    ws.start();
    void bootstrap();
    return () => ws.stop();
  }, []);

  return (
    <div className="app">
      <aside className="sidebar">
        <ConnectionPanel />
        <ServoPanel />
        <JobPanel />
        <MountPanel />
      </aside>
      <main className="main">
        <nav className="tabs">
          {TABS.map((t) => (
            <button
              key={t.id}
              className={"tab" + (tab === t.id ? " active" : "")}
              onClick={() => setTab(t.id)}
            >
              {t.label}
            </button>
          ))}
        </nav>
        <div className="tab-body">
          {tab === "live" && <LiveTab />}
          {tab === "cloud" && <CloudTab />}
          {tab === "sessions" && <SessionsPanel />}
          {tab === "log" && <LogPanel />}
        </div>
      </main>
      <StatusBar />
    </div>
  );
}
