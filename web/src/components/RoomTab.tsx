import { useEffect, useRef, useState } from "react";
import { errorText } from "../api/rest";
import { fmt, fmtInt } from "../lib/format";
import { loadRoom, roomStart, roomStop, useRoom } from "../store/room";
import { useStore } from "../store/store";
import { RoomMap, type RoomMapHandle } from "./RoomMap";
import { Shell, type BigNumber } from "./Shell";

// Room map: carry the scanner (level, slowly) and the core builds a 2D map of
// the room from the LiDAR, with the path walked and where the scanner is now.

function stamp(): string {
  return new Date().toISOString().replace(/[-:]/g, "").slice(0, 15).replace("T", "-");
}

export function RoomTab() {
  const room = useRoom((s) => s.room);
  const grid = useRoom((s) => s.grid);
  const supported = useRoom((s) => s.supported);
  const error = useRoom((s) => s.error);
  const lidarOn = useStore((s) => !!s.status?.lidar.connected);
  const mapRef = useRef<RoomMapHandle>(null);
  const [busy, setBusy] = useState<string | null>(null);
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    void loadRoom();
  }, []);

  const mapping = room?.state === "mapping";
  const m = room?.match;
  const numbers: BigNumber[] = [
    {
      label: "Area mapped",
      value: room ? (
        <>
          {fmt(room.area_m2, 1)}
          <small>m²</small>
        </>
      ) : (
        "–"
      ),
    },
    { label: "Keyframes", value: room ? fmtInt(room.keyframes) : "–" },
    {
      label: "Match",
      value:
        m && m.rms_mm !== null ? (
          <>
            {fmt(m.rms_mm, 0)}
            <small>mm · {m.inliers !== null ? `${fmt(m.inliers * 100, 0)}% inliers` : "–"}</small>
          </>
        ) : (
          "–"
        ),
      className: m && !m.ok ? "warn" : "",
    },
    {
      label: "State",
      value: room ? (room.lost ? "lost" : room.state) : "–",
      className: room?.lost ? "err" : mapping ? "ok" : "",
    },
  ];

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

  const png = async () => {
    const b = await mapRef.current?.toBlob();
    if (!b) return;
    const a = document.createElement("a");
    a.href = URL.createObjectURL(b);
    a.download = `room-map-${stamp()}.png`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  };

  const pose = room?.pose;
  return (
    <Shell
      numbers={numbers}
      numbersDim={!mapping}
      main={
        <section className="panel ops-mappanel">
          {room?.lost && <div className="ops-alarm">Lost track: move back to an area already on the map, slowly</div>}
          {supported === false ? (
            <div className="empty ops-connect">This core has no room mapping yet.</div>
          ) : (
            <>
              <RoomMap ref={mapRef} grid={grid} room={room} />
              <div className="legend ops-legend">
                <span className="sw sw-room-free" /> free
                <span className="sw sw-room-occ" /> wall or object
                <span className="dim">dark: not seen</span>
                <span className="sw sw-room-edge" /> open edge, not seen past yet
                <span className="sw sw-room-path" /> path walked
                <span className="dim">triangle: scanner now</span>
                <span className="spacer" />
                <button className="small" onClick={() => mapRef.current?.fit()}>
                  Fit
                </button>
              </div>
            </>
          )}
        </section>
      }
      side={
        <section className="panel ops-right">
          <div className="panel-head">
            <span>Mapping</span>
            <span className="sub">{grid ? `${grid.w}×${grid.h} @ ${+grid.cell_m.toFixed(3)} m` : ""}</span>
          </div>
          <div className="panel-body ops-summary">
            <div className="sensor-text">
              {!room
                ? "No mapping state yet."
                : mapping
                  ? room.lost
                    ? "Mapping, but the scan no longer matches the map. Walk back slowly to a place already mapped."
                    : `Mapping: ${fmt(room.area_m2, 1)} m² seen from ${fmtInt(room.scans)} scans.`
                  : grid && grid.w > 0
                    ? `Stopped. The map (${fmt(room.area_m2, 1)} m²) is kept until the next Start.`
                    : "Not mapping."}
            </div>
            {pose && (
              <div className="sensor-text">
                Scanner at x {fmt(pose.x, 2)} m, y {fmt(pose.y, 2)} m from the start, heading {fmt(pose.theta_deg, 0)}° (anticlockwise from the start direction).
              </div>
            )}
            {m && (
              <div className="sensor-text">
                Scan match: {m.rms_mm !== null ? `${fmt(m.rms_mm, 0)} mm RMS` : "–"}, {m.inliers !== null ? `${fmt(m.inliers * 100, 0)}% of points agree` : "–"}
                {m.ok ? "." : ": poor, slow down."}
              </div>
            )}
            {mapping && room?.next && (
              <div className="sensor-text room-next">
                {room.next.station
                  ? `Next scan: carry the scanner ${fmt(room.next.station.distance_m, 1)} m toward ${fmt(room.next.station.bearing_deg, 0)}° (cable mark 0°, clockwise) to the ring on the map, then hold it still for 2 s. ${room.next.open_edges} open edge${room.next.open_edges === 1 ? "" : "s"} left.`
                  : room.next.complete
                    ? "No open edges left: the room is closed. Press Stop and export the PNG."
                    : `${room.next.open_edges} open edge${room.next.open_edges === 1 ? "" : "s"} left, none with a clear spot to scan from yet: turn slowly where you stand.`}
              </div>
            )}
            {room?.note && <div className="hint">{room.note}</div>}
            {!lidarOn && <div className="hint">The LiDAR is not connected (Live scan tab).</div>}
            {(err || error) && <div className="error">{err ?? error}</div>}
            <div className="hint">
              The map starts where Start was pressed: +x is the LiDAR cable mark at that moment, +y to its left. It is a 2D slice at the height the scanner is
              carried.
            </div>
          </div>
        </section>
      }
      actions={
        <>
          <button
            className="primary"
            disabled={busy !== null || supported === false}
            title={mapping ? "Start again: clears the current map" : "Clears any previous map"}
            onClick={() => void run("start", () => roomStart())}
          >
            Start mapping
          </button>
          <button disabled={busy !== null || !mapping} onClick={() => void run("stop", roomStop)}>
            Stop
          </button>
          <button disabled={!grid || grid.w === 0} onClick={() => void png()}>
            PNG
          </button>
        </>
      }
      actionsNote="Keep the scanner level, walk slowly, turn gently; start where it can see several walls."
    />
  );
}
