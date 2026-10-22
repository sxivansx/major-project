import { useState } from "react";
import { api } from "../api/rest";
import { useStore } from "../store/store";

// Download the next N raw rotations at the current tilt as CSV (GET /api/scan/snapshot.csv).
// For range-bias measurements: target at a known distance, grab, compare.
export function SnapshotButton() {
  const connected = useStore((s) => !!s.status?.lidar.connected);
  const [rot, setRot] = useState(1);
  return (
    <>
      <select value={rot} onChange={(e) => setRot(Number(e.target.value))} title="rotations to capture" disabled={!connected}>
        {[1, 2, 5, 10].map((n) => (
          <option key={n} value={n}>
            {n} rot
          </option>
        ))}
      </select>
      <a
        className={"button small" + (connected ? "" : " disabled")}
        href={connected ? api.snapshotUrl(rot) : undefined}
        download
        title="Save the next rotations as CSV: rotation,t,phi,angle,range,quality"
        onClick={(e) => {
          if (!connected) e.preventDefault();
        }}
      >
        Snapshot CSV
      </a>
    </>
  );
}
