import { useEffect, useState } from "react";
import { api, errorText } from "../api/rest";
import { cloudClear } from "../store/cloud";
import { clientLog, useStore } from "../store/store";
import { fmtDateTime, fmtInt } from "../lib/format";

export function SessionsPanel() {
  const sessions = useStore((s) => s.sessions);
  const setSessions = useStore((s) => s.setSessions);
  const setStatus = useStore((s) => s.setStatus);
  const setGrid = useStore((s) => s.setGrid);
  const setCloud = useStore((s) => s.setCloud);
  const setTab = useStore((s) => s.setTab);
  const config = useStore((s) => s.config);
  const loaded = useStore((s) => s.status?.session_loaded ?? null);
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);

  const refresh = async () => {
    setBusy("refresh");
    setErr(null);
    try {
      setSessions(await api.sessions());
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };

  useEffect(() => {
    void refresh();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const load = async (id: string) => {
    setBusy(id);
    setErr(null);
    try {
      cloudClear();
      setCloud(0, 0);
      setStatus(await api.sessionLoad(id));
      clientLog("info", `session ${id} loaded into the live view`);
      setTab("cloud");
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };

  const grid = async (id: string) => {
    setBusy(id);
    setErr(null);
    try {
      setGrid(
        await api.sessionGrid(id, {
          cell: config?.grid.cell_m,
          layer: config?.grid.layer,
          z_min: config?.grid.z_min_m,
          z_max: config?.grid.z_max_m,
        }),
      );
      setTab("cloud");
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };

  const del = async (id: string) => {
    if (!window.confirm(`Delete session ${id}? This removes meta.json and sweeps.npz from disk.`)) return;
    setBusy(id);
    setErr(null);
    try {
      await api.sessionDelete(id);
      clientLog("info", `session ${id} deleted`);
      await refresh();
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setBusy(null);
    }
  };

  return (
    <section className="panel full-panel">
      <div className="panel-head">
        <span>Sessions</span>
        <span className="row" style={{ display: "flex", gap: 6, alignItems: "center" }}>
          <span className="sub">{loaded ? `loaded: ${loaded}` : ""}</span>
          <button className="small" disabled={busy !== null} onClick={refresh}>
            Refresh
          </button>
        </span>
      </div>
      <div className="panel-body" style={{ padding: 0 }}>
        {err && (
          <div className="error" style={{ padding: 8 }}>
            {err}
          </div>
        )}
        {sessions.length === 0 ? (
          <div className="empty">No sessions yet. Run a sweep to create one.</div>
        ) : (
          <table className="list">
            <thead>
              <tr>
                <th>Id</th>
                <th>Name</th>
                <th>Created</th>
                <th style={{ textAlign: "right" }}>Points</th>
                <th style={{ textAlign: "right" }}>Steps</th>
                <th>φ range</th>
                <th>Lidar</th>
                <th>Actions</th>
              </tr>
            </thead>
            <tbody>
              {sessions.map((s) => (
                <tr key={s.id}>
                  <td className="mono">{s.id}</td>
                  <td>{s.name}</td>
                  <td className="mono">{fmtDateTime(s.created)}</td>
                  <td className="num">{fmtInt(s.n_points)}</td>
                  <td className="num">{s.n_steps}</td>
                  <td className="mono">
                    {s.phi_start}° to {s.phi_end}° by {s.phi_step}° × {s.rotations}
                  </td>
                  <td className="mono">{s.lidar?.model ?? "–"}</td>
                  <td className="actions">
                    <button className="small" disabled={busy !== null} onClick={() => load(s.id)}>
                      Load
                    </button>
                    <button className="small" disabled={busy !== null} onClick={() => grid(s.id)}>
                      Grid
                    </button>
                    <a className="small" href={api.sessionExportUrl(s.id, "ply")} download>
                      <button className="small">PLY</button>
                    </a>
                    <a className="small" href={api.sessionExportUrl(s.id, "csv")} download>
                      <button className="small">CSV</button>
                    </a>
                    <button className="small danger" disabled={busy !== null} onClick={() => del(s.id)}>
                      Delete
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </div>
    </section>
  );
}
