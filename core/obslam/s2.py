"""RPLIDAR S2 via the C++ bridge subprocess (driver/build/s2bridge). Frame format: docs/api.md §1."""

from __future__ import annotations

import json
import logging
import struct
import subprocess
from pathlib import Path

import numpy as np

from .sources import ScanFrame, ScanSource

log = logging.getLogger(__name__)

MAGIC = 0x52463253
_HDR = struct.Struct("<IId")  # magic, n, t_unix → 16 bytes
POINT_DTYPE = np.dtype([("angle", "<f4"), ("range", "<f4"), ("quality", "u1")])  # packed, 9 bytes
assert POINT_DTYPE.itemsize == 9

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BRIDGE = ROOT / "driver" / "build" / "s2bridge"


class S2ScanSource(ScanSource):
    name = "s2"

    def __init__(self, port: str, baud: int = 1_000_000, bridge: Path | None = None):
        super().__init__()
        self.port = port
        self.baud = int(baud)
        self.bridge = Path(bridge or DEFAULT_BRIDGE)
        self._proc: subprocess.Popen | None = None

    def _argv(self, *extra: str) -> list[str]:
        if not self.bridge.exists():
            raise RuntimeError(f"bridge not built: {self.bridge} (run `make -C driver`)")
        if self.port.startswith("tcp://"):
            # the ESP32 relays the LiDAR's UART over Wi-Fi: tcp://192.168.x.y:8899
            return [str(self.bridge), "--tcp", self.port[len("tcp://") :], *extra]
        return [str(self.bridge), "--serial", self.port, "--baud", str(self.baud), *extra]

    def info(self) -> dict:
        r = subprocess.run(self._argv("--info"), capture_output=True, text=True, timeout=15)
        if r.returncode != 0:
            msg = r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "bridge failed"
            try:
                msg = json.loads(msg).get("error", msg)
            except ValueError:
                pass
            raise RuntimeError(msg)
        return json.loads(r.stdout.strip().splitlines()[-1])

    def start(self, on_frame, on_exit=None):
        self._proc = subprocess.Popen(
            self._argv(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL, bufsize=0
        )
        super().start(on_frame, on_exit)

    def _read_exact(self, n: int) -> bytes | None:
        assert self._proc and self._proc.stdout
        buf = bytearray()
        while len(buf) < n:
            chunk = self._proc.stdout.read(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return bytes(buf)

    def _next_frame(self) -> ScanFrame | None:
        hdr = self._read_exact(_HDR.size)
        if hdr is None:
            self._capture_stderr()
            return None
        magic, n, t = _HDR.unpack(hdr)
        if magic != MAGIC:
            raise RuntimeError(f"bad frame magic {magic:#x}, bridge stream desynced")
        body = self._read_exact(n * POINT_DTYPE.itemsize)
        if body is None:
            self._capture_stderr()
            return None
        pts = np.frombuffer(body, POINT_DTYPE, n)
        return ScanFrame(t, pts["angle"].copy(), pts["range"].copy(), pts["quality"].copy())

    def _capture_stderr(self) -> None:
        """After stdout EOF: a non-zero exit is an error (last stderr line, JSON-decoded if the bridge
        emitted one); a clean exit is just the end of the stream."""
        p = self._proc
        if p is None or self._stop.is_set():
            return
        try:
            p.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass
        last = ""
        if p.stderr:
            try:
                err = p.stderr.read().decode(errors="replace").strip()
            except ValueError:
                err = ""
            last = err.splitlines()[-1] if err else ""
        if last.startswith("{"):
            try:
                last = json.loads(last).get("error", last)
            except ValueError:
                pass
        if p.returncode not in (0, None):
            self.error = last or f"bridge exited with code {p.returncode}"

    def _close(self) -> None:
        p, self._proc = self._proc, None
        if p is None:
            return
        try:
            p.terminate()
            p.wait(timeout=2)
        except Exception:  # noqa: BLE001
            p.kill()
        for stream in (p.stdout, p.stderr):
            try:
                stream and stream.close()
            except Exception:  # noqa: BLE001
                pass
