# Bring-up runbook (first night with the rig)

Do these in order. Each step has a pass condition; don't move on until it passes.

## 0. Before plugging anything in

```bash
cd obslam
make setup          # builds driver/build/s2bridge, python venv, web deps
make sim            # open http://localhost:5173, confirm the dashboard works on the simulator
```
Pass: live 2D scan moving, a sim sweep (-60..60, step 2) finishes with a height map.

## 1. Servo board

1. Wiring: servo (GS2010MG) red → separate 5–6 V supply (≥3 A). Black → that supply's GND **and** an ESP32 GND pin. Signal → GPIO 13. Never the ESP32 5 V pin. AS5600: 3.3 V, GND, SDA/SCL → GPIO 8/9 on an S3 (21/22 on a plain ESP32), magnet centred on the axle end, 0.5–3 mm gap.
2. Plug the ESP32 in over USB, find it: `ls /dev/cu.*` (usually `/dev/cu.usbmodem*` or `/dev/cu.usbserial-*`).
3. Flash: `make flash PORT=/dev/cu.usbmodemXXXX` (add `FQBN=esp32:esp32:esp32s3` for an S3).
4. Dashboard → Servo → pick the port → connect. Opening the port resets the board and the cradle snaps to centre (90°), so keep hands clear. In the raw console: `?` then `T 90`.
5. Encoder: send `E`. Adjust the magnet gap until `status=ok` (not `nomag`/`weak`/`strong`). Then `T 60`, `T 120`: `meas` must follow `angle` in the same direction; if it runs backwards, `E dir`.

Pass: `STATUS angle=90 meas=… enc=ok …` and the cradle moves to roughly level.

## 2. LiDAR

1. Plug the S2 in through its USB adapter. `ls /dev/cu.*` again: new entry `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`. The motor should spin up on its own.
2. Quick check without the dashboard:
   ```bash
   ./driver/build/s2bridge --serial /dev/cu.usbserial-XXXX --info
   ./driver/build/s2bridge --serial /dev/cu.usbserial-XXXX | python3 driver/check_frames.py
   ```
   Expect model S2, health good, ~10 Hz, ~3,000 points per frame.
3. Dashboard → Connection → source `s2`, port, baud 1000000 → connect.

Pass: model/serial/health filled in, scan Hz ≈ 10, raw table and polar plot live. Hold a hand at 0.5 m and see it.

If `--info` fails: nothing else may hold the port (RoboStudio, `screen`). Unplug/replug once. If the port is missing entirely, check the adapter cable; the CP210x driver is built into macOS.

## 3. Calibrate the mount (config panel)

1. **Servo centre:** with the LiDAR connected and the servo at 90°, look at the 2D scan of a flat wall straight ahead. Move the servo until the wall range is minimal and symmetric (plane horizontal). In "Mount and filters" click **Use current** next to Centre (servo °), then **Use measured** next to Centre (meas °) so the encoder scale is zeroed at the same position. (Alternatively send `E zero` in the raw console, which makes `meas` equal the servo angle there.)
2. **Direction:** move the servo +20°. If the scan plane went *down* at the front of the LiDAR (the side opposite the cable) set `invert: true`; if the encoder's φ in the servo panel moved the opposite way to the commanded φ, set the encoder Invert too. Convention: positive φ is a right-hand rotation about +x (the tilt axis).
3. **`h_mm`:** vertical distance from the tilt axis to the S2's optical window centre, measured with a ruler on the rig. Sign: positive if the window is above the axis.
4. **`lidar_yaw_deg`:** angle between the S2's 0° mark and the tilt axis direction, measured on the cradle. 0 if the cable points along the axle.
5. Save. Sessions store raw sweeps, so all of this can be re-done later without re-scanning.

Pass: in a sweep, floor points land near z = 0 and a vertical wall is vertical in the point cloud.

## 4. First sweep

Scan job: name `bench-1`, φ −45 → 45, step 2, rotations 1, settle 300 ms. ~46 steps × ~0.5 s ≈ 25 s.

Pass: point cloud fills, height map shows the floor flat and objects as bumps. Export PLY for the report.

## 5. If it misbehaves

| Symptom | Likely cause |
|---|---|
| Servo twitches / ESP32 reboots during a sweep | servo on USB power; use the external supply |
| `lidar stream ended` mid-sweep | USB adapter dropped at 1 Mbaud; try a shorter cable, different USB port, no hub |
| Floor slopes in the height map | `center_deg` off, or `invert` wrong |
| Objects doubled / smeared | settle too short, or `h_mm` wrong |
| Point cloud rotated 90° | `lidar_yaw_deg` |
