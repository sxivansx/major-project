# firmware: tilt servo controller

One Arduino sketch, `servo_tilt/servo_tilt.ino`, for an ESP32 that drives the GS2010MG servo tilting the LiDAR cradle and reads the cradle angle back from an AS5600 magnetic encoder on the tilt axle. The Mac talks to it over USB serial (or TCP over Wi-Fi) with the line protocol in `docs/api.md` section 2 (that file is the contract; this README only covers wiring, building and a hand test).

Pins on the ESP32-S3 follow the hardware team's `pin_connection.md` (ESP32-S3 WROOM-1 N16R8): servo GPIO 17, LiDAR UART RX 13 / TX 14, I2C 11/12, microphones on GPIO 4–7, GPIO 8/9 left unconnected.

On the ESP32-S3 it also relays the RPLIDAR: the LiDAR's 1 Mbaud UART is wired to the S3, and its bytes are passed through unchanged, both ways, to the Mac, either over the board's second USB socket or over Wi-Fi (TCP 8899). Slamtec's SDK in `s2bridge` decodes them on the Mac exactly as if the LiDAR were plugged in directly.

On the ESP32-S3 it also captures four INMP441 microphones simultaneously (24 kHz by default) and streams them on TCP 8901 (see "Microphones" below).

The encoder is optional: the sketch probes for it at boot and a board without one behaves exactly like the earlier servo-only firmware (`enc=none`, no `meas` values).

Works on either board, pick one and use its FQBN below:

| Board | FQBN |
|---|---|
| ESP32-S3 N16R8 (the target board) | `esp32:esp32:esp32s3:PSRAM=opi` |
| ESP32 DevKit (plain ESP32, CP2102/CH340 USB-UART) | `esp32:esp32:esp32` |

What each build has:

| Feature | ESP32-S3 | plain ESP32 |
|---|---|---|
| Tilt protocol over USB | **UART** socket (`/dev/cu.usbserial-*`, 115200) | the one USB socket |
| LiDAR USB pass-through | native **USB** socket (`/dev/cu.usbmodem*`) | not built (one USB socket); boot prints a `NOTE` |
| Wi-Fi, mDNS, TCP 8899 / 8900 | yes | not built; `N` answers `ERR no wifi`, `?` reports `ip=none` |
| Microphones, TCP 8901 | yes | not built; `A` answers `ERR no audio`, `?` reports `audio=off` |

`PSRAM=opi` turns on the N16R8's 8 MB octal PSRAM, which holds the 1 MB LiDAR TCP ring and the 2 s microphone ring. The S3 build is made **without** `CDCOnBoot`, so `Serial` (the tilt protocol) is UART0 on the **UART** socket and the LiDAR bytes go out of the native **USB** socket. (With `CDCOnBoot=cdc` the sketch swaps the two; either way the two streams never share a link.)

## Wiring

### Servo (GS2010MG)

A high-torque metal-gear servo draws a few amps when it stalls or starts a move. It gets its own supply.

| Servo wire | Goes to |
|---|---|
| red (V+) | **separate 5–6 V supply**, rated 3 A or more (bench PSU, 2S LiPo through a UBEC, or a 6 V wall adapter) |
| black/brown (GND) | that supply's GND **and** an ESP32 GND pin (the grounds must be shared or the signal has no reference) |
| yellow/white (signal) | ESP32-S3 **GPIO 17** (plain ESP32: GPIO 13) |

Rules:

- **Never power the servo from the ESP32's 5 V / VIN pin.** The USB port cannot source 3 A; the board browns out and resets mid-move, and the current spike can damage the USB-UART bridge.
- Connect the grounds first, then the servo supply, then plug in USB.
- Signal is 3.3 V from the ESP32; the GS2010MG accepts it directly, no level shifter.
- To use another pin, change `SERVO_PIN` (17 on the S3, 13 on the plain ESP32) at the top of the sketch. Avoid strapping pins (0, 2, 5, 12, 15 on the plain ESP32; 0, 3, 45, 46 on the S3).
- The GS2010MG is a **180°** servo, so the firmware defaults are `C 500 2500 180` and home (`H`) is **90°**. Its real pulse window has not been measured yet: if 0° or 180° hits the mechanical stop early or late, measure the pulses that reach the true end points and set them with `C <us_min> <us_max> 180`. Until then, the soft limits (`L`) are the safe way to keep it off the stops.

### Encoder (AS5600)

The AS5600 sits on the tilt axle and reads a small magnet glued to the end of the axle. It needs four wires and the magnet mounted right.

| AS5600 pin | Goes to |
|---|---|
| VCC | ESP32 **3.3 V** (not 5 V: the ESP32 pins are not 5 V tolerant and the chip runs fine at 3.3 V) |
| GND | ESP32 GND |
| SDA | **GPIO 11** on the ESP32-S3 (shared with the IMU), **GPIO 21** on the plain ESP32 |
| SCL | **GPIO 12** on the ESP32-S3 (shared with the IMU), **GPIO 22** on the plain ESP32 |
| DIR, OUT, PGO | leave unconnected (the sketch handles direction in software) |

Rules:

- Most AS5600 breakout boards have 4.7 kΩ pull-ups on SDA and SCL. If yours does not (check for two resistors next to the chip, or measure SDA to VCC), add **4.7 kΩ from SDA to 3.3 V and from SCL to 3.3 V**. Without pull-ups the probe fails and the board reports `enc=none`.
- Magnet: a **6 mm diametrically magnetised** disc (poles across the diameter, not through the thickness; the kind sold with AS5600 breakouts). An axially magnetised magnet reads `nomag` no matter the spacing.
- Mount the magnet **centred on the axle**, its face parallel to the chip, with a **0.5–3 mm air gap** to the chip package. The chip must be on the axle's axis, so fix the breakout to the frame, not to the cradle.
- Keep steel and other magnets a few cm away. Use `E` to check the spacing (see "Encoder setup" below).
- To move the I2C pins, change `AS5600_SDA` / `AS5600_SCL` in the sketch (there is one pair for the S3 and one for the plain ESP32). Do not use GPIO 8/9 on the S3: the wiring doc keeps them unconnected. On the plain ESP32, GPIO 8 and 9 are flash pins: never use them.

```
   5–6 V / 3 A supply            ESP32 (S3 pins; plain ESP32 in brackets)
   +---------------+          +----------------+
   |  V+        GND|----+-----|GND             |
   +---|-----------+    |     |                |       AS5600
       |                |     |  GPIO 17 (13)  |----> servo signal (yellow/white)
       +--> servo red   +---> servo black      |
                              |  3V3           |----> VCC
                              |  GND           |----> GND
                              |  GPIO 11 (21)  |<---> SDA   (4.7 kΩ to 3V3 if the breakout has none)
                              |  GPIO 12 (22)  |<---> SCL   (4.7 kΩ to 3V3 if the breakout has none)
                              +----------------+
```

### LiDAR (RPLIDAR S3, ESP32-S3 only)

The RPLIDAR's UART is 3.3 V logic at 1,000,000 baud and connects straight to the S3, no level shifter.

| RPLIDAR wire | Goes to |
|---|---|
| TX (data out of the LiDAR, yellow on this cable) | ESP32-S3 **GPIO 13** (`LIDAR_RX_PIN`) |
| RX (data into the LiDAR, green on this cable) | ESP32-S3 **GPIO 14** (`LIDAR_TX_PIN`) |
| GND | ESP32 GND, **and** the LiDAR supply's GND (common ground) |
| 5 V | **its own 5 V supply** (≥1.5 A), or the power side of the Slamtec USB adapter. **Not** the ESP32's 5 V pin |

Rules:

- The LiDAR pulls about **1.2 A while the motor spins up** (~450 mA running). Through the ESP32's 5 V pin that comes from the USB port and browns the board out, so it needs a separate 5 V source with its ground tied to the ESP32's.
- TX goes to RX: LiDAR **TX → GPIO 13**, LiDAR **RX ← GPIO 14**. Swapped wires give `lidar_rx=0` in `?` while the motor spins.
- To use other pins, change `#define LIDAR_RX_PIN 13` / `#define LIDAR_TX_PIN 14` in the sketch (any free GPIO works, UART1 is routed through the GPIO matrix). Avoid the S3 strapping pins (0, 3, 45, 46), the flash/PSRAM pins (26–37 on modules with octal PSRAM), USB D−/D+ (19, 20), UART0 (43, 44), I2C (11, 12), the microphones (4–7), the servo (17) and the unconnected GPIO 8/9. GPIO 15 is the legacy motor-control assignment in the wiring doc: leave it alone.

### The two USB cables (ESP32-S3)

| Socket on the board | macOS port | Carries |
|---|---|---|
| **UART** (CP2102N bridge) | `/dev/cu.usbserial-XXXX` (or `/dev/cu.SLAB_USBtoUART`) | the tilt protocol (`I`, `?`, `T`, …), 115200 |
| **USB** (native, ESP32-S3 USB-Serial/JTAG) | `/dev/cu.usbmodemXXXX` | the raw LiDAR byte stream for `s2bridge --serial` (any baud); also used for flashing |

Plug both into the Mac. Over Wi-Fi neither cable needs to carry data. The wiring doc's rule applies: no powered laptop USB while the powerbank also feeds the board's 5 V header, unless the carrier's isolation has been checked.

## Toolchain

```sh
brew install arduino-cli
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32          # ~300 MB, takes a few minutes
arduino-cli lib install ESP32Servo
```

The AS5600 driver is a few register reads on `Wire` (part of the ESP32 core); Wi-Fi, mDNS and the TCP servers use the core's `WiFi`, `ESPmDNS` and `Network` libraries; the microphones use ESP-IDF's `driver/i2s_std.h` directly (the core ships IDF 5.5). No extra library.

## Compile and flash

Run from the repo root.

Plain ESP32 DevKit:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/servo_tilt
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 firmware/servo_tilt
```

ESP32-S3 N16R8 (the target; always with `PSRAM=opi`, see the board table above). Flash over either socket; the native **USB** one is shown:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi firmware/servo_tilt
arduino-cli upload  --fqbn esp32:esp32:esp32s3:PSRAM=opi -p /dev/cu.usbmodem1101 firmware/servo_tilt
```

Over the **UART** socket instead: same FQBN, `-p /dev/cu.usbserial-XXXX`. With Wi-Fi and audio in, the S3 image is about 1.0 MB (76% of the default 1.25 MB app partition).

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
- `/dev/cu.usbmodemXXXX` (ESP32-S3 native USB: the LiDAR bytes)

`arduino-cli board list` shows the same ports with the detected board where it can.

## Quick manual test

With `screen` (press Enter after each command; `screen` sends `\r`, which the firmware accepts):

```sh
screen /dev/cu.usbserial-XXXX 115200      # ESP32-S3: the UART socket
screen /dev/cu.usbserial-0001 115200      # plain ESP32
```

Or with arduino-cli:

```sh
arduino-cli monitor -p /dev/cu.usbserial-XXXX -c baudrate=115200
```

Opening the port resets the board (the USB-UART bridge pulls DTR/RTS), so you first see the ROM boot chatter, then the `NOTE` lines and `OBSLAM-TILT v1`, and the servo goes to the middle of its range. Then type (replies shown for a board with the encoder fitted and zeroed; without one, drop the trailing `meas` token on `OK` lines and expect `meas=nan enc=none settled=0` in `STATUS`):

```
I
OBSLAM-TILT v1
?
STATUS angle=90 meas=90.09 min=0 max=180 range=180 us_min=500 us_max=2500 enc=ok settled=1 lidar_rx=0 lidar_tx=0 lidar_drop=0 lidar_ovf=0 ip=192.168.1.42 tcp_rx=0 tcp_tx=0 buf=0/1048576 audio=on
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

With the LiDAR wired and powered and both cables in, run against the native **USB** socket's port:

```sh
./driver/build/s2bridge --serial /dev/cu.usbmodemXXXX --baud 1000000 --info
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
STATUS angle=90 ... ip=192.168.x.y tcp_rx=... tcp_tx=... buf=.../1048576 audio=on
```

(`nc` sends `\n` after each line; quit with `Ctrl-C`.)

## Microphones (ESP32-S3)

Four INMP441 MEMS microphones, captured **simultaneously** at 24 kHz (default; 16 or 48 kHz with `A rate`) and streamed on TCP **8901** in the `AUD1` frame format of `docs/api.md` §5.2. The array gives direction only; the core turns the stream into bearings.

### Wiring recap (pin_connection.md §3)

| Mic pin | Mic 1 (top) | Mic 2 (left) | Mic 3 (right) | Mic 4 (bottom) |
|---|---|---|---|---|
| VDD | 3V3 | 3V3 | 3V3 | 3V3 |
| GND | GND | GND | GND | GND |
| SCK / BCLK | GPIO 4 | GPIO 4 | GPIO 4 | GPIO 4 |
| WS | GPIO 5 | GPIO 5 | GPIO 5 | GPIO 5 |
| SD | GPIO 7 | GPIO 7 | GPIO 6 | GPIO 6 |
| L/R | GND (left slot) | 3V3 (right slot) | GND (left slot) | 3V3 (right slot) |

- Never join GPIO 6 and GPIO 7: each pair shares its SD wire through the opposite L/R straps.
- **Do not fit the GPIO 4→8 / 5→9 jumpers.** GPIO 8 and 9 stay unconnected; the firmware sets them as plain inputs.
- The firmware turns on the internal pull-down on both SD inputs (GPIO 6, 7), as the INMP441 datasheet asks for on SD. A mic that is not driving its slot then reads exactly 0 instead of floating.
- Mic power comes from the board's 3V3, never 5 V. Keep the mic wires away from the servo and LiDAR motor leads.

### How all four are sampled together

- **I2S0** is the master receiver: it drives BCLK (GPIO 4, 64 × the sample rate: 1.536 MHz at 24 kHz) and WS (GPIO 5, the sample rate) for all four mics and reads mics 1/2 on GPIO 7.
- **I2S1** is a slave receiver for mics 3/4 on GPIO 6. It must see the *same* BCLK/WS, but the wiring doc rules out the jumper wires that would bring them to I2S1's own pins. So the firmware routes the pads internally: after I2S0 has made GPIO 4/5 its outputs, it also turns on their input buffers (`gpio_input_enable`, the output stays routed) and connects them to I2S1's slave clock inputs through the GPIO matrix:

  ```c
  esp_rom_gpio_connect_in_signal(4, I2S1I_BCK_IN_IDX, false);   // signal 31
  esp_rom_gpio_connect_in_signal(5, I2S1I_WS_IN_IDX,  false);   // signal 32
  ```

  I2S1 itself is created with no clock pins (`I2S_GPIO_UNUSED`). If a driver version refuses that, it is created on the unconnected GPIO 8/9 instead (they stay inputs) and the matrix routing above replaces them as I2S1's clock source.
- I2S1 is **enabled before** I2S0, so the slave is already waiting when the master's first WS edge arrives and both units start on the same frame. Both then run off the same physical clock, so they cannot drift. If a read ever times out, both units are stopped and restarted together, which re-aligns them.
- The fallback, if the routing ever turns out not to work on the hardware: build with `-DAUDIO_CLOCK_JUMPERS=1` (or edit the `#define`), which clocks I2S1 from GPIO 8/9. That needs the 4→8 / 5→9 jumpers, so it is only for a bench test after the hardware team agrees. The boot `NOTE audio:` line says which mode is running.

What has **not** been verified yet (no hardware on the bench when this was written): that I2S1 really starts on the same frame as I2S0. A fixed one-sample offset between the pairs (mics 1/2 vs 3/4) is the plausible failure mode; check it once with a single sharp tap equidistant from two mics of different pairs (their cross-correlation peak should be at 0 samples).

### Data path

- A capture task on **core 0** (with Wi-Fi; `loop()` keeps core 1 to itself) reads 10 ms blocks (n = rate/100 frames of 32-bit slots: 240 at 24 kHz) from both units, converts to `int16` with the right shift (default 14, saturating), interleaves mic1..mic4, stamps the frame and puts it in a 200-frame ring (~774 KB) in PSRAM. The DMA holds 60 ms per unit, so short Wi-Fi bursts on core 0 lose nothing.
- `t_us` is `esp_timer_get_time()` at the frame's first sample. It comes from a sample-counted timeline (10 000 µs per frame) anchored to `esp_timer` by the minimum over each second of (read time − elapsed audio), so it is jitter-free and follows any slow rate offset.
- `loop()` copies the ring to the client with non-blocking `send()`, at most two frames per pass, exactly like the LiDAR path; servo, encoder and LiDAR work never wait on audio.
- Frames only go into the ring while a client is connected. A new client starts on fresh audio (anything queued is skipped).
- **Latency cap.** If the link is slower than the stream, the backlog can never drain, and sending it oldest-first only hides the loss behind a growing delay (that is what the first hardware test showed: 39 % of real time received with no gaps, because the drops were all queued behind the 6 s capture window). So when more than 30 frames (300 ms) are waiting, the sender skips to the newest 5. The skipped samples are added to `dropped` in `A`, `seq` jumps, and the next frame sent has flag bit 1 set. The checker's `lag=` column shows how far behind real time the stream is; it should stay under ~300 ms.
- **Bandwidth.** Each frame is 28 + n × 8 bytes: **128 KB/s at 16 kHz, 193 KB/s at 24 kHz, 384 KB/s at 48 kHz**. TCP on the ESP32 can only have lwIP's 5,744-byte send buffer in flight, so the ceiling is about 5,744 B ÷ round-trip time: ~570 KB/s at 10 ms, ~190 KB/s at 30 ms, ~150 KB/s at 38 ms (what the first test on a home router implies). So 24 kHz needs a round trip under ~30 ms. The board's own `SLAM-Test` AP is usually fine; on a slow router use `A rate 16000`. The buffer size is compiled into the Arduino core's prebuilt lwIP and cannot be raised from the sketch.
- **Not built yet: a UDP path.** It would remove that window ceiling: the same `AUD1` frames, split into 5 ms datagrams under the 1,472-byte MTU payload, sent with non-blocking `sendto()` to an address the Mac subscribes with (`A udp <port>` on 8900, renewed every few seconds or it stops). No acknowledgements means no RTT limit (only Wi-Fi airtime, several MB/s); a lost datagram is just a `seq` gap, which the core already handles.

### Commands (USB or TCP 8900)

```
A
AUDIO rate=24000 shift=14 clients=0 frames=1234 dropped=0 i2s_timeouts=0
A shift 12
OK
A rate 16000
OK
A raw
RAW rate=16000 n=160 fmt=philips data_bits=32 slot_bits=32 mask=both ws_width=32 ... | i2s0.L(mic1) first=... min=... max=... nz=... ones=... toggle=... | ...
```

- `frames`: frames captured since boot (100 per second at any rate, whether or not a client is connected).
- `dropped`: samples per channel not delivered because the client fell behind (n per skipped frame).
- `A rate <16000|24000|48000>` sets the sample rate and saves it in NVS (`aud_rate`). The capture task switches both units within ~20 ms; the new rate and n appear in the next frames' headers. All three keep the INMP441 inside its 0.5–3.2 MHz SCK range.
- `i2s_timeouts`: reads that timed out (should stay 0; non-zero means a unit lost its clock).
- `A shift <8..16>` sets the gain and is saved in NVS (`aud_shift`). The INMP441's 24-bit sample sits at the top of the 32-bit slot, so `shift 16` keeps exactly its top 16 bits (0 dBFS = the mic's full scale); each step lower doubles the level (+6 dB) and clips that much sooner. The default 14 is +12 dB, which suits a quiet room. The value applies from the next frame and is reported in every frame header.
- `?` ends with `audio=on` when both units and the capture task are running, `audio=off` otherwise.

### `A raw`: what the lines are really doing

`A raw` grabs the next raw block (before the shift) and replies with one long line: the slot/clock configuration actually passed to the driver, a pad probe, and per unit and slot the first 8 raw 32-bit words in hex, `min`/`max`, `nz` (non-zero words), `ones` (words equal to 0xFFFFFFFF) and `toggle` (every bit that ever differed from the first word).

| What you see | Meaning |
|---|---|
| `toggle=00000000`, `ones=n` | the data line sat high the whole block: nothing is driving it (before the pull-down fix this gave −1 after the shift = the −90.3 dBFS seen in the first test) |
| `toggle=00000000`, `nz=0` | line held low: with the pull-down on, the mic is not driving its slot |
| top bits toggling, low byte `00` | a working INMP441 (24 bits left-justified, low 8 bits zero) |
| only one slot of a pair alive | wrong or floating L/R strap, or one dead mic |
| `pads_hi` for bclk4/ws5 far from half the samples | the clock pad is not toggling at all (wiring, pin, or I2S0 not started) |
| `pads_hi` for din7/din6 at 0 or 4000 | the data pad never changes during the probe |

Expected configuration for the INMP441: `fmt=philips data_bits=32 slot_bits=32 mask=both ws_width=32 ws_pol=0 bit_shift=1 big_endian=0 lsb_first=0` (64 BCLK per frame, MSB one BCLK after the WS edge, left slot while WS is low).

### Checking the stream from the Mac

`firmware/tools/audio_check.py` (Python 3, standard library only) connects to port 8901, validates every frame (magic, header fields, payload size, `seq` gaps, flags), and prints each channel's RMS and peak level in dBFS once a second:

```sh
python3 firmware/tools/audio_check.py 192.168.4.1                    # on the SLAM-Test AP
python3 firmware/tools/audio_check.py 192.168.1.42 --wav mics.wav    # also save 4-channel 16-bit WAV
python3 firmware/tools/audio_check.py 192.168.4.1:8901 --seconds 10
```

```
 100 fr  24k  shift=14  lag=   41ms  m1  -52.3/ -38.0  m2  -51.8/ -37.1  m3  -52.9/ -39.4  m4  -52.0/ -36.6
```

Ctrl-C prints totals (frames, lost frames and gaps, flag counts, resyncs, clipped samples per mic, received rate). Exit status is 0 only if frames arrived and no header was malformed. A tap near one mic should raise that channel most; a channel stuck at `-inf` is a mic that is not driving its slot (dead, unpowered, or a wrong L/R strap putting two mics on one slot). Each line also shows the rate and `lag=` (how far behind real time the stream is).

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
