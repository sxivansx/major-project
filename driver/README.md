# driver: s2bridge

C++ bridge from an RPLIDAR S2 to binary scan frames on stdout. The Python core
server spawns it as a subprocess; Python lidar libraries do not understand the
S2 packet format, so Slamtec's SDK (vendored in `rplidar_sdk/`, a git
submodule, never edited) does the serial work.

## Build

```sh
make -C driver            # builds rplidar_sdk if its .a is missing, then build/s2bridge
make -C driver clean
```

Needs `clang++` (Xcode command line tools) and `make`.

## Usage

```sh
./build/s2bridge --serial /dev/cu.usbserial-0001 --info     # one JSON line, exit 0
./build/s2bridge --serial /dev/cu.usbserial-0001            # frames to stdout until SIGTERM
./build/s2bridge --serial /dev/cu.usbserial-0001 | python3 check_frames.py
```

`--baud` defaults to 1000000 (the S2 rate). Errors are one JSON line
`{"error":"..."}` on stderr with exit 1. Logs go to stderr only; stdout is
frames only. The bridge stops cleanly on SIGINT/SIGTERM, when stdout is closed
by the reader (EPIPE) or when its stdin pipe hangs up.

Each frame is one 360° rotation: a 16-byte header (`u32 magic 0x52463253`,
`u32 n`, `f64 t_unix`) followed by `n` packed 9-byte points (`f32 angle_deg`,
`f32 range_m`, `u8 quality`), little-endian, zero-range points dropped; the
contract lives in `docs/api.md` section 1.

`check_frames.py` (stdlib only) validates magic and sizes and prints per-frame
`n`, min/max range and Hz. Without hardware, `python3 check_frames.py --fake 5`
emits synthetic frames you can pipe back into it.

## macOS notes

- The S2 kit's USB adapter is a Silicon Labs CP210x, which macOS supports
  natively (no driver install). The port shows up as `/dev/cu.usbserial-*` or
  `/dev/cu.SLAB_USBtoUART`. Use the `cu.` node, not `tty.`.
- 1 Mbaud is a non-standard rate; on Darwin the SDK sets it with the
  `IOSSIOSPEED` ioctl after opening the port (`sdk/src/arch/macOS/net_serial.cpp`).
- The S2 motor spins up on power and is not controlled by the bridge.
