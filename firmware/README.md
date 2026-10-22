# firmware: tilt servo controller

One Arduino sketch, `servo_tilt/servo_tilt.ino`, for an ESP32 that drives the GS2010MG servo tilting the LiDAR cradle and reads the cradle angle back from an AS5600 magnetic encoder on the tilt axle. The Mac talks to it over USB serial (or TCP over Wi-Fi) with the line protocol in `docs/api.md` section 2 (that file is the contract; this README only covers wiring, building and a hand test).

On the ESP32-S3 it also relays the RPLIDAR: the LiDAR's 1 Mbaud UART is wired to the S3, and its bytes are passed through unchanged, both ways, to the Mac, either over the board's second USB socket or over Wi-Fi (TCP 8899). Slamtec's SDK in `s2bridge` decodes them on the Mac exactly as if the LiDAR were plugged in directly.

The encoder is optional: the sketch probes for it at boot and a board without one behaves exactly like the earlier servo-only firmware (`enc=none`, no `meas` values).

Works on either board, pick one and use its FQBN below:

| Board | FQBN |
|---|---|
| ESP32-S3 DevKitC-1 (the target board) | `esp32:esp32:esp32s3:CDCOnBoot=cdc` |
| ESP32 DevKit (plain ESP32, CP2102/CH340 USB-UART) | `esp32:esp32:esp32` |

What each build has:

| Feature | ESP32-S3 | plain ESP32 |
|---|---|---|
| Tilt protocol over USB | native **USB** socket (`/dev/cu.usbmodem*`) | the one USB socket |
| LiDAR USB pass-through | **UART** socket (`/dev/cu.usbserial-*`), 1,000,000 baud | not built (one USB socket); boot prints a `NOTE` |
| Wi-Fi, mDNS, TCP 8899 / 8900 | yes | not built; `N` answers `ERR no wifi`, `?` reports `ip=none` |

`CDCOnBoot=cdc` matters on the S3: it puts `Serial` (the tilt protocol) on the native USB socket and frees UART0 (`Serial0`, the CP2102N "UART" socket) for the LiDAR bytes. Built without it, `Serial` and `Serial0` are the same object; the sketch then swaps the sockets (tilt on UART, LiDAR on native USB) so the two still never share a link, but the documented setup is `CDCOnBoot=cdc`.

## Wiring

### Servo (GS2010MG)

A high-torque metal-gear servo draws a few amps when it stalls or starts a move. It gets its own supply.

| Servo wire | Goes to |
|---|---|
| red (V+) | **separate 5–6 V supply**, rated 3 A or more (bench PSU, 2S LiPo through a UBEC, or a 6 V wall adapter) |
| black/brown (GND) | that supply's GND **and** an ESP32 GND pin (the grounds must be shared or the signal has no reference) |
| yellow/white (signal) | ESP32 **GPIO 13** |

Rules:

- **Never power the servo from the ESP32's 5 V / VIN pin.** The USB port cannot source 3 A; the board browns out and resets mid-move, and the current spike can damage the USB-UART bridge.
- Connect the grounds first, then the servo supply, then plug in USB.
- Signal is 3.3 V from the ESP32; the GS2010MG accepts it directly, no level shifter.
- To use another pin, change `#define SERVO_PIN 13` at the top of the sketch. Avoid strapping pins (0, 2, 5, 12, 15 on the plain ESP32; 0, 3, 45, 46 on the S3).
- The GS2010MG is a **180°** servo, so the firmware defaults are `C 500 2500 180` and home (`H`) is **90°**. Its real pulse window has not been measured yet: if 0° or 180° hits the mechanical stop early or late, measure the pulses that reach the true end points and set them with `C <us_min> <us_max> 180`. Until then, the soft limits (`L`) are the safe way to keep it off the stops.

### Encoder (AS5600)

The AS5600 sits on the tilt axle and reads a small magnet glued to the end of the axle. It needs four wires and the magnet mounted right.

| AS5600 pin | Goes to |
|---|---|
| VCC | ESP32 **3.3 V** (not 5 V: the ESP32 pins are not 5 V tolerant and the chip runs fine at 3.3 V) |
| GND | ESP32 GND |
| SDA | **GPIO 8** on the ESP32-S3, **GPIO 21** on the plain ESP32 |
| SCL | **GPIO 9** on the ESP32-S3, **GPIO 22** on the plain ESP32 |
| DIR, OUT, PGO | leave unconnected (the sketch handles direction in software) |

Rules:

- Most AS5600 breakout boards have 4.7 kΩ pull-ups on SDA and SCL. If yours does not (check for two resistors next to the chip, or measure SDA to VCC), add **4.7 kΩ from SDA to 3.3 V and from SCL to 3.3 V**. Without pull-ups the probe fails and the board reports `enc=none`.
- Magnet: a **6 mm diametrically magnetised** disc (poles across the diameter, not through the thickness; the kind sold with AS5600 breakouts). An axially magnetised magnet reads `nomag` no matter the spacing.
- Mount the magnet **centred on the axle**, its face parallel to the chip, with a **0.5–3 mm air gap** to the chip package. The chip must be on the axle's axis, so fix the breakout to the frame, not to the cradle.
- Keep steel and other magnets a few cm away. Use `E` to check the spacing (see "Encoder setup" below).
- The pins follow the board's default I2C bus. To move them, change `AS5600_SDA` / `AS5600_SCL` in the sketch (there is one pair for the S3 and one for the plain ESP32). On the plain ESP32, GPIO 8 and 9 are flash pins: never use them.

```
   5–6 V / 3 A supply            ESP32 (S3 pins; plain ESP32 in brackets)
   +---------------+          +----------------+
   |  V+        GND|----+-----|GND             |
   +---|-----------+    |     |                |       AS5600
       |                |     |  GPIO 13       |----> servo signal (yellow/white)
       +--> servo red   +---> servo black      |
                              |  3V3           |----> VCC
                              |  GND           |----> GND
                              |  GPIO 8  (21)  |<---> SDA   (4.7 kΩ to 3V3 if the breakout has none)
                              |  GPIO 9  (22)  |<---> SCL   (4.7 kΩ to 3V3 if the breakout has none)
                              +----------------+
```

### LiDAR (RPLIDAR S3, ESP32-S3 only)

The RPLIDAR's UART is 3.3 V logic at 1,000,000 baud and connects straight to the S3, no level shifter.

| RPLIDAR wire | Goes to |
|---|---|
| TX (data out of the LiDAR) | ESP32-S3 **GPIO 18** (`LIDAR_RX_PIN`) |
| RX (data into the LiDAR) | ESP32-S3 **GPIO 17** (`LIDAR_TX_PIN`) |
| GND | ESP32 GND, **and** the LiDAR supply's GND (common ground) |
| 5 V | **its own 5 V supply** (≥1.5 A), or the power side of the Slamtec USB adapter. **Not** the ESP32's 5 V pin |

Rules:

- The LiDAR pulls about **1.2 A while the motor spins up** (~450 mA running). Through the ESP32's 5 V pin that comes from the USB port and browns the board out, so it needs a separate 5 V source with its ground tied to the ESP32's.
- TX goes to RX: LiDAR **TX → GPIO 18**, LiDAR **RX ← GPIO 17**. Swapped wires give `lidar_rx=0` in `?` while the motor spins.
- To use other pins, change `#define LIDAR_RX_PIN 18` / `#define LIDAR_TX_PIN 17` in the sketch (any free GPIO works, UART1 is routed through the GPIO matrix). Avoid the S3 strapping pins (0, 3, 45, 46), the flash/PSRAM pins (26–37 on modules with octal PSRAM), USB D−/D+ (19, 20), UART0 (43, 44), and the AS5600 pins (8, 9).

### The two USB cables (ESP32-S3 DevKitC-1)

| Socket on the board | macOS port | Carries |
|---|---|---|
| **USB** (native, ESP32-S3 USB-Serial/JTAG) | `/dev/cu.usbmodemXXXX` | the tilt protocol (`I`, `?`, `T`, …) at any baud; also used for flashing |
| **UART** (CP2102N bridge) | `/dev/cu.usbserial-XXXX` (or `/dev/cu.SLAB_USBtoUART`) | the raw LiDAR byte stream, 1,000,000 baud, for `s2bridge --serial` |

Plug both into the Mac. Over Wi-Fi neither cable needs to carry data (power only).

## Toolchain

```sh
brew install arduino-cli
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32          # ~300 MB, takes a few minutes
arduino-cli lib install ESP32Servo
```

The AS5600 driver is a few register reads on `Wire` (part of the ESP32 core); Wi-Fi, mDNS and the TCP servers use the core's `WiFi`, `ESPmDNS` and `Network` libraries. No extra library.

## Compile and flash

Run from the repo root.

Plain ESP32 DevKit:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/servo_tilt
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 firmware/servo_tilt
```

ESP32-S3 DevKitC-1 (the target; always with `CDCOnBoot=cdc`, see the board table above). Flash over either socket; the native **USB** one is shown:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc firmware/servo_tilt
arduino-cli upload  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc -p /dev/cu.usbmodem1101 firmware/servo_tilt
```

Over the **UART** socket instead: same FQBN, `-p /dev/cu.usbserial-XXXX`. With Wi-Fi in, the S3 image is about 980 KB (74% of the default 1.25 MB app partition).

If the upload sits at `Connecting........` hold the **BOOT** button, tap **RST**/**EN**, release BOOT, and retry.

Compiler warnings from inside `ESP32Servo` (unused `TAG`, `val`, `ret`; the deprecated MCPWM header on the S3) are known noise from the library, not from the sketch.

Keep the board option `DebugLevel` at its default (`none`). With it raised, the ESP32 core's `Wire` prints its own error lines on the same serial port whenever an I2C read fails (missing encoder at boot, cable pulled), which breaks the one-reply-per-command rule.

### Finding the port on macOS

```sh
ls /dev/cu.*
```

Unplug and replug the board and see which entry appears. Typical names:

- `/dev/cu.usbserial-0001`, `/dev/cu.usbserial-XXXX` (CP2102)
- `/dev/cu.wchusbserialXXXX` (CH340)
- `/dev/cu.SLAB_USBtoUART` (older CP210x driver)
- `/dev/cu.usbmodemXXXX` (ESP32-S3 native USB: the tilt protocol)

`arduino-cli board list` shows the same ports with the detected board where it can.

## Quick manual test

With `screen` (press Enter after each command; `screen` sends `\r`, which the firmware accepts):

```sh
screen /dev/cu.usbmodem1101 115200        # ESP32-S3: the native USB socket
screen /dev/cu.usbserial-0001 115200      # plain ESP32
```

Or with arduino-cli:

```sh
arduino-cli monitor -p /dev/cu.usbmodem1101 -c baudrate=115200
```

On the plain ESP32, opening the port resets the board (the USB-UART bridge pulls DTR/RTS), so you first see the ROM boot chatter, then the `NOTE` lines and `OBSLAM-TILT v1`, and the servo goes to the middle of its range. The S3's native USB port does not reset on open; press **RST** to see the boot lines. Then type (replies shown for a board with the encoder fitted and zeroed; without one, drop the trailing `meas` token on `OK` lines and expect `meas=nan enc=none settled=0` in `STATUS`):

```
I
OBSLAM-TILT v1
?
STATUS angle=90 meas=90.09 min=0 max=180 range=180 us_min=500 us_max=2500 enc=ok settled=1 lidar_rx=0 lidar_tx=0 lidar_drop=0 lidar_ovf=0 ip=192.168.1.42 tcp_rx=0 tcp_tx=0
T 120
OK 120 90.09
W
OK 119.97 settled=1
S 60 1000
OK 60 60.12
H
OK 90 60.12
W 500
OK 89.91 settled=1
E
ENC raw=2051 deg=180.26 zero=90.35 dir=1 status=ok
L 10 170
OK
C 500 2500 180
OK
x
ERR unknown cmd
T abc
ERR bad args
```

Leave `screen` with `Ctrl-A` then `k`, then `y`. Leave the arduino-cli monitor with `Ctrl-C`.

## LiDAR pass-through over USB (ESP32-S3)

With the LiDAR wired and powered and both cables in, run against the **UART** socket's port:

```sh
./driver/build/s2bridge --serial /dev/cu.usbserial-XXXX --baud 1000000 --info
```

It should print the `{"model":...,"health":"good",...}` line. Then send `?` on the tilt port: `lidar_tx` counts the SDK's command bytes that reached the LiDAR and `lidar_rx` its reply and scan bytes (about 100,000 a second while scanning).

The counters in `?`, all since boot, from the ESP's point of view:

| Field | Meaning |
|---|---|
| `lidar_rx` | bytes read from the LiDAR and handed to a host link (USB or TCP 8899) |
| `lidar_tx` | bytes written to the LiDAR (from USB or TCP 8899) |
| `lidar_drop` | bytes the firmware discarded because the host link could not keep up (UART ring at its high-water mark, or a TCP client left with a chunk in flight) |
| `lidar_ovf` | UART overflow events from the driver itself (bytes lost before the firmware saw them; the count is events, not bytes). Should stay 0 |
| `tcp_rx` / `tcp_tx` | bytes received from / sent to the TCP 8899 client |

A rising `lidar_drop` means the Mac is not reading fast enough; a rising `lidar_ovf` means the loop was held up (for example a `N` that re-joined Wi-Fi, which can block up to 1 s). On the plain ESP32 all of these stay 0.

## Wi-Fi (ESP32-S3)

### Set the network

Once, over the USB tilt port (stored in NVS, survives reboots and reflashes):

```
N myhotspot mypassword
OK
NET ip=192.168.1.42
```

`OK` comes straight away; the `NET ip=…` line follows when the S3 has joined and got an address (also after every reconnect). For an open network send `N myhotspot` with no password. The SSID and password cannot contain spaces (the parser splits on them); a WPA2 password must be 8–63 characters, an SSID 1–32. Sending `N` with new values replaces the old ones and re-joins.

The firmware re-joins by itself when the link drops (the Wi-Fi driver's auto-reconnect, plus a forced reconnect every 15 s while it stays down). Modem power-save is off, so latency stays flat.

### Find the IP

Any of:

- `?` on the USB tilt port: `ip=192.168.1.42` (`ip=none` while not joined)
- `N` alone:

  ```
  NET ssid=myhotspot ip=192.168.1.42 rssi=-58 clients_lidar=0 clients_tilt=0
  ```

  `rssi` is in dBm (`none` while not joined); `clients_lidar` / `clients_tilt` are 1 while a client holds TCP 8899 / 8900. `ssid=none` means no network is stored.
- mDNS: `obslam-tilt.local` (`ping obslam-tilt.local` on the Mac).

### Ports

| Port | What |
|---|---|
| **8899** | raw LiDAR bridge: bytes in go to the LiDAR, LiDAR bytes come out. Same bytes as the USB pass-through. While a client is connected the USB pass-through's LiDAR → Mac direction is paused (one consumer at a time) and resumes when it leaves; USB → LiDAR keeps working, so do not run both at once. |
| **8900** | the tilt line protocol, same commands and replies as on USB. Each reply goes back to the link its command came from. |

One client per port; a new connection replaces the old one (the old socket is closed).

### Test from the Mac

```sh
ping -c1 obslam-tilt.local                                # prints the IP, if you don't have it from `?`
./driver/build/s2bridge --tcp 192.168.x.y:8899 --info       # numeric IP: the SDK's TCP channel parses an address
nc 192.168.x.y 8900
?
STATUS angle=90 ... ip=192.168.x.y tcp_rx=... tcp_tx=...
```

(`nc` sends `\n` after each line; quit with `Ctrl-C`.)

## Encoder setup

`meas` is the encoder angle reported **on the servo scale** (0..range, same units as `T`), so after alignment `meas` tracks `angle`. The scale is set once and stored in NVS (`enc_zero`, `enc_dir`); it survives reboots and reflashes.

### 1. Magnet spacing with `E`

`E` alone prints the raw chip state:

```
ENC raw=<0..4095> deg=<deg> zero=<deg> dir=<1|-1> status=<ok|nomag|weak|strong|err|none>
```

- `raw` and `deg` are the chip's own 12-bit reading (0..4095, 0..360°), before zero and direction are applied. `zero` is on that same scale.
- `status` is what to watch while adjusting the magnet:
  - `nomag`: no magnet seen. Wrong magnet type, too far away, or off-axis.
  - `weak`: magnet detected but the field is too weak. Move it closer (smaller air gap).
  - `strong`: field too strong. Move it further away.
  - `ok`: in range. Aim for the middle of the gap window that reads `ok`, usually 1–2 mm.
  - `err`: the chip stopped answering (wiring, pull-ups, cable pulled). The last good `raw` is kept.
  - `none`: the boot probe found no chip; fix the wiring and reset the board (the probe only runs at boot).

Repeat `E` while you move the breakout or shim the magnet; then turn the axle by hand and check `raw` changes smoothly over the full tilt travel and never jumps.

### 2. Align the scale

With the magnet reading `ok`:

```
H              servo to 90 (home)
W              let it settle
E zero         meas now reads 90
T 120
W              read meas: ~120 means the direction is right
```

If `W` after `T 120` reports about **60** instead of 120, the encoder counts the other way. Send `E dir`: it flips the direction and re-aligns at the current pose in one go, so `meas` reads 120 there and 90 back at home. No second `E zero` is needed, but `?` after `H` is a cheap check.

`E zero` can be done at any commanded angle, not just home: it makes `meas` equal whatever `angle` is at that moment. The mechanical mapping servo-deg → tilt φ (centre, sign) still lives in the core config, exactly as before; the encoder only adds a measured value in servo degrees next to the commanded one.

### 3. Waiting for the move with `W`

`W [ms]` blocks until the encoder has been stable, meaning it stayed within **0.15° peak-to-peak for 150 ms**, or until `ms` elapses (default 1500, max 10000), then replies:

```
OK <meas> settled=<0|1>
```

`settled=0` means the timeout hit first: the servo is still moving, hunting, or the magnet reads `nomag`/`err`. Every new pulse (`T`, `S`, `H`, and `C` when it changes the pulse) clears the stability window, so a `W` sent straight after a move always waits at least 150 ms before it can say `settled=1`; a `T` to the angle the servo is already at does not clear it.

The same detector drives `settled=` in `STATUS`, evaluated continuously from the 100 Hz encoder poll.

**Without an encoder** `W` waits a fixed **300 ms** and replies `OK <angle> settled=1`, so a step-and-scan loop written against `W` runs unchanged on an encoder-less board (`?` on such a board reports `settled=0`, since there is nothing to measure).

## Behaviour notes

- Every target is clamped twice: to `[0, range]` and then to the soft limits from `L`. The reply echoes the clamped angle, so `T 400` on a 180 servo answers `OK 180`.
- `T`, `S` and `H` reply `OK <deg> <meas>` when an encoder is fitted and `OK <deg>` without one. The second token is always the commanded angle, so a parser that reads only that still works. `meas` on a `T`/`H` reply is the reading at the moment the pulse was set (the cradle has not moved yet); use `W` or `?` for the reading after the move.
- `S <deg> <ms>` moves in 20 ms steps (one 50 Hz servo frame) and blocks until done; `ms` must be in `0..60000`. The reply arrives after the move. The encoder keeps being polled during `S` and `W`.
- `L` and `C` are stored in NVS (`Preferences`, namespace `tilt`) and survive reboots and reflashes. Defaults are `L 0 180` and `C 500 2500 180`; send those to reset. `C` refuses pulses outside 500–2500 µs (the ESP32Servo library hard-clips there) and ranges other than 180 or 270. If `C` shrinks the range, the soft limits are pulled into the new range. A board flashed with the old firmware keeps its stored `C 500 2500 270` until you send `C 500 2500 180`.
- `E zero` and `E dir` are stored in the same namespace (`enc_zero`, `enc_dir`). Both refuse (`ERR no encoder`, `ERR enc=nomag`, `ERR enc=err`) unless there is a valid reading with a magnet behind it; `weak`/`strong` are accepted.
- Changing `L` does not move the servo; changing `C` re-issues the pulse for the current angle under the new mapping, so the cradle may move slightly.
- On boot the servo goes to `range/2` (90° by default, clamped to the soft limits) so `?` reports a real angle. `angle` is always the last commanded angle; `meas` is the only measured value.
- The encoder is polled at about 100 Hz from the main loop (non-blocking, two short I2C reads at 400 kHz: STATUS, then RAW ANGLE 0x0C/0x0D). The unscaled RAW ANGLE register is used on purpose, so nothing programmed into the chip's own ZPOS/MPOS registers can shift the reading; zero and direction are applied in the firmware. If a read fails the status goes to `err`, the last good value is kept, and polling backs off to 10 Hz until the chip answers again. If the chip is missing at boot (three probes, 10 ms apart), the I2C bus is released and never touched again.
- `meas` is wrapped into a 360° window centred on `range/2` (so −90..270 for a 180 servo), which keeps it continuous a little past either end of the travel instead of jumping to 359.
- Pulse resolution is 0.3 µs on the plain ESP32 (16-bit LEDC timer) and 1.2 µs on the S3 (14-bit), so the smallest step is well under 0.2°. ESP32Servo's default is 10 bits (~1.8° steps on a 180 servo); the sketch overrides it after `attach()`. The AS5600 resolves 0.088° (12 bits).
- Command letters are accepted in either case (`e zero` works too). Lines are trimmed; blank lines get no reply; lines over 127 characters get `ERR line too long`.
- The core should wait about 1.5 s after opening the port and discard any pending input before sending `I`, since the auto-reset puts boot output on the line. Boot output now includes `NOTE …` lines before the banner.
- Two kinds of unsolicited line can appear on the USB tilt port: `NOTE …` at boot and `NET ip=…` whenever Wi-Fi gets an address. Both are printed between commands, never inside a reply, but a parser waiting for a reply should skip lines starting with `NOTE ` or `NET ip=`. Neither is sent on TCP 8900.
- The LiDAR bytes are copied in chunks (up to 512 bytes LiDAR → Mac, 64 bytes Mac → LiDAR) with zero-timeout reads and writes capped to the receiver's free space, from the main loop and from inside every blocking command (`S`, `W`), so scan data keeps flowing during moves. The UART1 receive ring is 16 KB (~160 ms of scan data).
- Wi-Fi credentials are stored as `wifi_ssid` / `wifi_pass` in the `tilt` NVS namespace; `N` never echoes the password.
