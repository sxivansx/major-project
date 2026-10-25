# driver: s2bridge

`s2bridge` is a small C++ program that reads an RPLIDAR S-series scanner (S2 or S3) and writes one binary frame per rotation to stdout. The Python core starts it as a subprocess and reads those frames.

Why a separate program: the S-series sends scans in a dense packet format that Slamtec does not document and that no Python LiDAR library decodes. Slamtec's own SDK does, so the bridge is a thin wrapper around it. The SDK lives in `rplidar_sdk/` as a git submodule and is never edited.

## Build

```sh
make -C driver            # builds the SDK if needed, then build/s2bridge
make -C driver clean
```

Needs `clang++` (Xcode command line tools) and `make`. `make setup` at the repo root does this for you.

## Use

The LiDAR can reach the laptop two ways: over the rig's Wi-Fi (the ESP32-S3 relays the LiDAR's bytes on TCP 8899), or plugged straight in through Slamtec's USB adapter.

```sh
# over the rig's Wi-Fi
./build/s2bridge --tcp 192.168.4.1:8899 --info                # one JSON line with model, firmware, health; exit 0
./build/s2bridge --tcp 192.168.4.1:8899 | python3 check_frames.py

# through the USB adapter
./build/s2bridge --serial /dev/cu.usbserial-0001 --info
./build/s2bridge --serial /dev/cu.usbserial-0001 | python3 check_frames.py
```

| Flag | Meaning |
|---|---|
| `--tcp <host:port>` | read from the ESP32's LiDAR relay |
| `--serial <port>` | read from a serial port (use the `/dev/cu.*` node on macOS, not `tty.*`) |
| `--baud <n>` | serial only; default 1000000, the S2/S3 rate |
| `--info` | print device info as one JSON line and exit |

Errors are one JSON line `{"error":"..."}` on stderr with exit code 1. Logs go to stderr; stdout carries frames only. The bridge stops cleanly on SIGINT or SIGTERM, when the reader closes stdout, or when its stdin closes.

## Frame format

Each frame is one 360° rotation:

- a 16-byte header: `u32 magic 0x52463253`, `u32 n`, `f64 t_unix`
- then `n` packed 9-byte points: `f32 angle_deg`, `f32 range_m`, `u8 quality`

Everything is little-endian, and points with zero range are dropped. The full contract is in [../docs/api.md](../docs/api.md) section 1.

`check_frames.py` (standard library only) checks the magic and sizes and prints each frame's point count, range and rate. Without hardware, `python3 check_frames.py --fake 5` makes synthetic frames you can pipe back into it.

## macOS notes

- Slamtec's USB adapter uses a Silicon Labs CP210x chip, which macOS supports without a driver. It shows up as `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`.
- 1 Mbaud is not a standard rate. On macOS the SDK sets it with the `IOSSIOSPEED` ioctl after opening the port.
- The LiDAR motor spins up on power; the bridge does not control it.
- Only one program can hold the LiDAR at a time, whether that's the serial port or TCP 8899 on the rig.
