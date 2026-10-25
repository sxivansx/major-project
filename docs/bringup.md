# Bring-up runbook: new hardware, step by step

Use this the first time you wire a rig, or after changing its hardware. For day-to-day use of a rig that already works, [SETUP.md](../SETUP.md) is enough.

Do the steps in order. Each one ends with a **Pass** check; don't move on until it passes. Pins are for the ESP32-S3 carrier board, with a plain ESP32 dev board in brackets. The full wiring is in [firmware/README.md](../firmware/README.md#wiring).

## 0. Software first (no hardware)

```bash
cd obslam
make setup          # builds driver/build/s2bridge, the Python venv and the web dependencies
make sim            # open http://localhost:5173 and check the dashboard works on the simulator
```

**Pass:** the Live scan tab moves, and a simulated sweep (Scan on the Operator tab) finishes with a height map.

## 1. Servo and encoder

1. **Wire the servo** (GS2010MG, 180°). Red goes to a separate 5–6 V supply rated 3 A or more. Black goes to that supply's GND **and** an ESP32 GND pin. Signal goes to **GPIO 17** (GPIO 13). Never power the servo from the ESP32's 5 V pin.
2. **Wire the encoder** (AS5600, optional). 3.3 V, GND, SDA to **GPIO 11** (21), SCL to **GPIO 12** (22). Glue the magnet centred on the axle end with a 0.5–3 mm gap to the chip.
3. **Flash the firmware** with the ESP32 on USB and the power bank disconnected: `make flash PORT=/dev/cu.usbmodemXXXX FQBN=esp32:esp32:esp32s3:PSRAM=opi`. Find the port with `ls /dev/cu.*`. Toolchain setup is in [SETUP.md](../SETUP.md#flash-the-firmware-not-needed-for-the-demo).
4. **Connect** from the dashboard's Live scan tab: servo port `/dev/cu.usbserial-*` (the board's UART socket) over USB, or `tcp://192.168.4.1:8900` over Wi-Fi. Opening the USB port resets the board and the cradle snaps to level (90°), so keep your hands clear.
5. **Open the travel.** A fresh board only allows 81–99° so a new servo cannot hit its stops. In the servo console send `T 85` and `T 95` and check the cradle moves freely. Then send `L 0 180`.
6. **Check the encoder** (if fitted). Send `E` and adjust the magnet gap until `status=ok` (not `nomag`, `weak` or `strong`). Then send `T 60` and `T 120`: `meas` must follow `angle` in the same direction. If it runs backwards, send `E dir`.

**Pass:** `?` answers `STATUS angle=90 … min=0 max=180 …` (with `enc=ok` if the encoder is fitted) and the cradle sits roughly level.

## 2. LiDAR

The LiDAR reaches the laptop either through the rig (normal use) or through Slamtec's USB adapter (handy for testing the LiDAR alone).

**Through the rig (Wi-Fi):** wire LiDAR TX to **GPIO 13** and LiDAR RX to **GPIO 14**, with a common ground. Give the LiDAR its own 5 V supply, not the ESP32's 5 V pin. Power the rig, join the `SLAM-Test` Wi-Fi network, then:

```bash
./driver/build/s2bridge --tcp 192.168.4.1:8899 --info
./driver/build/s2bridge --tcp 192.168.4.1:8899 | python3 driver/check_frames.py
```

**Through the USB adapter:** plug it in, find the new `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`, then use `--serial <port>` instead of `--tcp …` in the commands above.

Expect: model S3 (or S2), health good, about 10 Hz and about 2,860 points per frame. The motor spins up on its own.

Then connect it in the dashboard (Live scan tab: source `s2`, port `tcp://192.168.4.1:8899` or the serial port).

**Pass:** model, serial and health are filled in, the scan rate is about 10 Hz, and the polar plot is live. Hold a hand 0.5 m away and see it.

If `--info` fails, something else may be holding the LiDAR: only one program can use it at a time (RoboStudio, `screen`, or a running dashboard). Close it and try again. Over USB, unplug and replug once. Over Wi-Fi, check `ping 192.168.4.1`.

## 3. Calibrate the mount

All of this is in the dashboard: Live scan tab, Advanced, **Mount and filters**.

1. **Servo centre.** With the servo at 90°, look at the 2D scan of a flat wall straight ahead. Nudge the servo until the wall distance is smallest and symmetric, which means the scan plane is level. Click **Use current** next to Centre (servo °), then **Use measured** next to Centre (meas °) so the encoder is zeroed at the same place. (`E zero` in the console does the same for the encoder.)
2. **Direction.** Move the servo +20°. If the scan plane went *down* at the front of the LiDAR (the side opposite the cable), set `invert`. If the encoder's φ moved the opposite way to the commanded φ, set the encoder Invert too. Convention: positive φ is a right-hand rotation about +x, the tilt axis.
3. **`h_mm`.** Measure with a ruler the vertical distance from the tilt axis to the centre of the LiDAR's window. Positive if the window is above the axis.
4. **`lidar_yaw_deg`.** The angle between the LiDAR's 0° mark and the tilt axis, measured on the cradle. 0 if the cable points along the axle.
5. **Save.** Sessions keep the raw data, so you can redo the calibration later without scanning again.

**Pass:** in a sweep, floor points land near z = 0 and a vertical wall is vertical in the point cloud.

## 4. First sweep

On the Point cloud tab, open Advanced and fill in the Scan job form: name `bench-1`, φ −45 to 45, step 2, 1 rotation, settle 300 ms. That is 46 steps at about 0.5 s each, so roughly 25 seconds.

**Pass:** the point cloud fills in, and the height map shows a flat floor with objects as bumps. Export a PLY file for the report.

## 5. Microphones

Wire the four INMP441 mics as in [firmware/README.md](../firmware/README.md#microphones-esp32-s3), then connect audio on the Sound tab (`tcp://192.168.4.1:8901`).

**Pass:** all four waveforms move when you speak, and a knock shows up as a detection. Then run the **clap calibration** in the Sound tab's Advanced drawer to fix the channel order and direction offsets.

## If something misbehaves

| Symptom | Likely cause |
|---|---|
| Servo twitches, or the ESP32 reboots during a sweep | the servo is on USB or power-bank power; give it its own supply |
| The cradle barely moves | the soft limits are still at 81–99°; send `L 0 180` |
| `lidar stream ended` mid-sweep | over USB: the adapter dropped at 1 Mbaud, so try a shorter cable and no hub. Over Wi-Fi: weak link, so move closer |
| The floor slopes in the height map | `center_deg` is off, or `invert` is wrong |
| Objects look doubled or smeared | settle time too short, or `h_mm` wrong |
| The point cloud is rotated 90° | `lidar_yaw_deg` |
