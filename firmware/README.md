# firmware: tilt servo controller

One Arduino sketch, `servo_tilt/servo_tilt.ino`, for an ESP32 that drives the DS3218 servo tilting the LiDAR cradle. The Mac talks to it over USB serial with the line protocol in `docs/api.md` section 2 (that file is the contract; this README only covers wiring, building and a hand test).

Works on either board, pick one and use its FQBN below:

| Board | FQBN |
|---|---|
| ESP32 DevKit (plain ESP32, CP2102/CH340 USB-UART) | `esp32:esp32:esp32` |
| ESP32-S3 DevKitC-1 | `esp32:esp32:esp32s3` |

## Wiring

The DS3218 draws up to ~3 A when it stalls or starts a move. It gets its own supply.

| DS3218 wire | Goes to |
|---|---|
| red (V+) | **separate 5–6 V supply**, rated 3 A or more (bench PSU, 2S LiPo through a UBEC, or a 6 V wall adapter) |
| black/brown (GND) | that supply's GND **and** an ESP32 GND pin (the grounds must be shared or the signal has no reference) |
| yellow/white (signal) | ESP32 **GPIO 13** |

Rules:

- **Never power the servo from the ESP32's 5 V / VIN pin.** The USB port cannot source 3 A; the board browns out and resets mid-move, and the current spike can damage the USB-UART bridge.
- Connect the grounds first, then the servo supply, then plug in USB.
- Signal is 3.3 V from the ESP32; the DS3218 accepts it directly, no level shifter.
- To use another pin, change `#define SERVO_PIN 13` at the top of the sketch. Avoid strapping pins (0, 2, 5, 12, 15 on the plain ESP32; 0, 3, 45, 46 on the S3).

```
   5–6 V / 3 A supply            ESP32
   +---------------+          +-----------+
   |  V+        GND|----+-----|GND        |
   +---|-----------+    |     |           |
       |                |     |  GPIO 13  |----> servo signal (yellow/white)
       +--> servo red   +---> servo black
```

## Toolchain

```sh
brew install arduino-cli
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32          # ~300 MB, takes a few minutes
arduino-cli lib install ESP32Servo
```

## Compile and flash

Run from the repo root.

Plain ESP32 DevKit:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/servo_tilt
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 firmware/servo_tilt
```

ESP32-S3 DevKitC-1 (plug the USB cable into the port labelled **UART**, not **USB**):

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/servo_tilt
arduino-cli upload  --fqbn esp32:esp32:esp32s3 -p /dev/cu.usbserial-0001 firmware/servo_tilt
```

If you would rather talk to the S3 over its native **USB** port instead, build with the CDC option and the serial protocol then appears on that port as `/dev/cu.usbmodem*`:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc firmware/servo_tilt
arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p /dev/cu.usbmodem1101 firmware/servo_tilt
```

If the upload sits at `Connecting........` hold the **BOOT** button, tap **RST**/**EN**, release BOOT, and retry.

### Finding the port on macOS

```sh
ls /dev/cu.*
```

Unplug and replug the board and see which entry appears. Typical names:

- `/dev/cu.usbserial-0001`, `/dev/cu.usbserial-XXXX` (CP2102)
- `/dev/cu.wchusbserialXXXX` (CH340)
- `/dev/cu.SLAB_USBtoUART` (older CP210x driver)
- `/dev/cu.usbmodemXXXX` (ESP32-S3 native USB)

`arduino-cli board list` shows the same ports with the detected board where it can.

## Quick manual test

With `screen` (press Enter after each command; `screen` sends `\r`, which the firmware accepts):

```sh
screen /dev/cu.usbserial-0001 115200
```

Or with arduino-cli:

```sh
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=115200
```

Opening the port resets the board (the USB-UART bridge pulls DTR/RTS), so you first see the ROM boot chatter, then `OBSLAM-TILT v1`, and the servo goes to the middle of its range. Then type:

```
I
OBSLAM-TILT v1
?
STATUS angle=135 min=0 max=270 range=270 us_min=500 us_max=2500
T 135
OK 135
S 90 1000
OK 90
H
OK 135
L 10 260
OK
C 500 2500 270
OK
x
ERR unknown cmd
T abc
ERR bad args
```

Leave `screen` with `Ctrl-A` then `k`, then `y`. Leave the arduino-cli monitor with `Ctrl-C`.

## Behaviour notes

- Every target is clamped twice: to `[0, range]` and then to the soft limits from `L`. The reply echoes the clamped angle, so `T 400` on a 270 servo answers `OK 270`.
- `S <deg> <ms>` moves in 20 ms steps (one 50 Hz servo frame) and blocks until done; `ms` must be in `0..60000`. The reply arrives after the move.
- `L` and `C` are stored in NVS (`Preferences`, namespace `tilt`) and survive reboots and reflashes. Defaults are `L 0 270` and `C 500 2500 270`; send those to reset. `C` refuses pulses outside 500–2500 µs (the ESP32Servo library hard-clips there) and ranges other than 180 or 270. If `C` shrinks the range, the soft limits are pulled into the new range.
- Changing `L` does not move the servo; changing `C` re-issues the pulse for the current angle under the new mapping, so the cradle may move slightly.
- On boot the servo goes to `range/2` (clamped to the soft limits) so `?` reports a real angle; the firmware has no position feedback, so the angle it reports is always the last one commanded.
- Pulse resolution is 0.3 µs on the plain ESP32 (16-bit LEDC timer) and 1.2 µs on the S3 (14-bit), so the smallest step is well under 0.2°. ESP32Servo's default is 10 bits (~2.6° steps); the sketch overrides it after `attach()`.
- Command letters are accepted in either case. Lines are trimmed; blank lines get no reply; lines over 63 characters get `ERR line too long`.
- The core should wait about 1.5 s after opening the port and discard any pending input before sending `I`, since the auto-reset puts boot output on the line.
