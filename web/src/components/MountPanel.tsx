import { useEffect, useRef, useState } from "react";
import { api, errorText } from "../api/rest";
import type { Config } from "../api/types";
import { clientLog, useStore } from "../store/store";
import { fmt } from "../lib/format";

// Mount geometry (docs/api.md §3 "Frames of reference") and the lidar point filters.
// Saved config applies to the next sweep; loaded sessions re-derive from the mount stored in their meta.

type Form = {
  axis_z_mm: string;
  h_mm: string;
  lidar_yaw_deg: string;
  center_deg: string;
  invert: boolean;
  enc_center_deg: string;
  enc_invert: boolean;
  min_quality: string;
  min_range_m: string;
  max_range_m: string;
};

function fromConfig(c: Config): Form {
  return {
    axis_z_mm: String(c.mount.axis_z_mm),
    h_mm: String(c.mount.h_mm),
    lidar_yaw_deg: String(c.mount.lidar_yaw_deg),
    center_deg: String(c.mount.center_deg),
    invert: c.mount.invert,
    enc_center_deg: String(c.mount.enc_center_deg ?? c.mount.center_deg),
    enc_invert: c.mount.enc_invert ?? false,
    min_quality: String(c.lidar.min_quality),
    min_range_m: String(c.lidar.min_range_m),
    max_range_m: String(c.lidar.max_range_m),
  };
}

export function MountPanel() {
  const config = useStore((s) => s.config);
  const setConfig = useStore((s) => s.setConfig);
  const servo = useStore((s) => s.status?.servo);

  const [form, setForm] = useState<Form | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const seeded = useRef(false);

  useEffect(() => {
    if (config && !seeded.current) {
      seeded.current = true;
      setForm(fromConfig(config));
    }
  }, [config]);

  if (!config || !form) {
    return (
      <section className="panel">
        <div className="panel-head">
          <span>Mount and filters</span>
        </div>
        <div className="panel-body">
          <div className="hint">waiting for config</div>
        </div>
      </section>
    );
  }

  const num = (k: keyof Form) => Number(form[k]);
  const valid =
    (["axis_z_mm", "h_mm", "lidar_yaw_deg", "center_deg", "enc_center_deg", "min_quality", "min_range_m", "max_range_m"] as const).every(
      (k) => form[k] !== "" && Number.isFinite(num(k)),
    ) && num("min_range_m") < num("max_range_m");
  const dirty = JSON.stringify(form) !== JSON.stringify(fromConfig(config));

  const set = (k: keyof Form, v: string | boolean) => setForm((f) => (f ? { ...f, [k]: v } : f));

  const save = async () => {
    if (!valid) return;
    setErr(null);
    setBusy(true);
    const next: Config = {
      ...config,
      mount: {
        axis_z_mm: num("axis_z_mm"),
        h_mm: num("h_mm"),
        lidar_yaw_deg: num("lidar_yaw_deg"),
        center_deg: num("center_deg"),
        invert: form.invert,
        enc_center_deg: num("enc_center_deg"),
        enc_invert: form.enc_invert,
      },
      lidar: {
        min_quality: Math.round(num("min_quality")),
        min_range_m: num("min_range_m"),
        max_range_m: num("max_range_m"),
      },
    };
    try {
      const saved = await api.putConfig(next);
      setConfig(saved);
      setForm(fromConfig(saved));
      clientLog("info", `config saved: centre ${saved.mount.center_deg}°${saved.mount.invert ? " inverted" : ""}, h ${saved.mount.h_mm} mm`);
    } catch (e) {
      setErr(errorText(e));
      clientLog("error", `config: ${errorText(e)}`);
    } finally {
      setBusy(false);
    }
  };

  const servoDeg = servo?.connected ? servo.servo_deg ?? null : null;
  const measDeg = servo?.connected ? servo.meas_deg ?? null : null;

  const numField = (label: string, k: keyof Form, step: number, title?: string) => (
    <div className="field" title={title}>
      <label>{label}</label>
      <div className="row">
        <input type="number" step={step} value={String(form[k])} onChange={(e) => set(k, e.target.value)} />
      </div>
    </div>
  );

  return (
    <section className="panel">
      <div className="panel-head">
        <span>Mount and filters</span>
        <span className="sub">{dirty ? "unsaved" : ""}</span>
      </div>
      <div className="panel-body">
        <div className="section-title">Servo to tilt</div>
        <div className="field" title="Servo degrees at which the scan plane is horizontal">
          <label>Centre (servo °)</label>
          <div className="row">
            <input type="number" step={0.5} value={form.center_deg} onChange={(e) => set("center_deg", e.target.value)} />
            <button
              disabled={servoDeg === null}
              title={servoDeg === null ? "connect the servo" : `set centre to the current servo angle (${fmt(servoDeg, 1)}°)`}
              onClick={() => servoDeg !== null && set("center_deg", String(servoDeg))}
            >
              Use current
            </button>
          </div>
        </div>
        <div className="field" title="Tick if +servo tilts the plane the wrong way (φ must be a right-hand rotation about +x)">
          <label>Invert</label>
          <div className="row">
            <input type="checkbox" checked={form.invert} onChange={(e) => set("invert", e.target.checked)} />
            <span className="hint">φ = {form.invert ? "−" : ""}(servo − {form.center_deg || "centre"})</span>
          </div>
        </div>

        <div className="section-title">Encoder to tilt (AS5600)</div>
        <div className="field" title="Encoder reading (servo-degree scale) at which the scan plane is horizontal">
          <label>Centre (meas °)</label>
          <div className="row">
            <input type="number" step={0.1} value={form.enc_center_deg} onChange={(e) => set("enc_center_deg", e.target.value)} />
            <button
              disabled={measDeg === null}
              title={measDeg === null ? "no encoder reading" : `set to the current reading (${fmt(measDeg, 1)}°)`}
              onClick={() => measDeg !== null && set("enc_center_deg", String(measDeg))}
            >
              Use measured
            </button>
          </div>
        </div>
        <div className="field" title="Tick if the encoder counts the opposite way to φ">
          <label>Invert</label>
          <div className="row">
            <input type="checkbox" checked={form.enc_invert} onChange={(e) => set("enc_invert", e.target.checked)} />
            <span className="hint">
              {servo?.encoder && servo.encoder !== "none" ? `encoder ${servo.encoder}` : "no encoder: commanded angle is used"}
            </span>
          </div>
        </div>

        <div className="section-title">Geometry (mm, deg)</div>
        {numField("Axis height", "axis_z_mm", 1, "Tilt axis above the base plate; 80 from the CAD")}
        {numField("h: plane − axis", "h_mm", 0.5, "Scan plane height relative to the tilt axis, + if above. Measure on the rig.")}
        {numField("Lidar yaw", "lidar_yaw_deg", 1, "Angle from the lidar's 0° mark to the tilt axis direction; 0 if the cable points along the axle")}

        <div className="section-title">Point filters (next sweep)</div>
        {numField("Min quality", "min_quality", 1)}
        {numField("Min range m", "min_range_m", 0.05)}
        {numField("Max range m", "max_range_m", 0.5)}

        <div className="row" style={{ display: "flex", gap: 4 }}>
          <button disabled={!dirty || !valid || busy} onClick={() => void save()} style={{ flex: 1 }}>
            Save config
          </button>
          <button disabled={!dirty || busy} onClick={() => setForm(fromConfig(config))}>
            Reset
          </button>
        </div>
        {!valid && <div className="error">check the numbers: all required, min range below max</div>}
        {err && <div className="error">{err}</div>}
        <div className="hint">
          Applies to the next sweep. Saved sessions keep their own mount and are re-derived on load.
        </div>
      </div>
    </section>
  );
}
