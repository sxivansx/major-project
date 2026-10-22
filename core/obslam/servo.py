"""Tilt servo over the ESP32 line protocol (docs/api.md §2), plus a simulated one."""

from __future__ import annotations

import logging
import re
import threading
import time

import serial

log = logging.getLogger(__name__)

_STATUS_RE = re.compile(r"(\w+)=([-\d.]+)")


class TiltError(Exception):
    pass


class TiltSource:
    port: str
    servo_deg: float
    min_deg: float
    max_deg: float
    range: int

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def move(self, servo_deg: float) -> float: ...
    def home(self) -> float: ...
    def raw(self, line: str) -> str: ...

    def refresh(self) -> dict:
        return {"servo_deg": self.servo_deg, "min_deg": self.min_deg, "max_deg": self.max_deg, "range": self.range}


class SimTilt(TiltSource):
    port = "sim"

    def __init__(self, range_deg: int = 270):
        self.range = range_deg
        self.min_deg, self.max_deg = 0.0, float(range_deg)
        self.servo_deg = range_deg / 2.0

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def move(self, servo_deg: float) -> float:
        self.servo_deg = float(min(max(servo_deg, self.min_deg), self.max_deg))
        return self.servo_deg

    def home(self) -> float:
        return self.move(self.range / 2.0)

    def raw(self, line: str) -> str:
        line = line.strip()
        if line == "I":
            return "OBSLAM-TILT v1 (sim)"
        if line == "?":
            return f"STATUS angle={self.servo_deg:.1f} min={self.min_deg:.1f} max={self.max_deg:.1f} range={self.range} us_min=500 us_max=2500"
        if line.startswith("T "):
            return f"OK {self.move(float(line[2:])):.1f}"
        if line == "H":
            return f"OK {self.home():.1f}"
        return "ERR unknown command"


class SerialTilt(TiltSource):
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()
        self.servo_deg = 0.0
        self.min_deg, self.max_deg, self.range = 0.0, 270.0, 270

    def connect(self) -> None:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=2)
        except serial.SerialException as e:
            raise TiltError(str(e)) from e
        # Opening the port resets most ESP32 dev boards; wait for the boot banner, then identify.
        deadline = time.time() + 4.0
        ident = None
        while time.time() < deadline and ident is None:
            self._ser.reset_input_buffer()
            self._ser.write(b"I\n")
            line = self._readline()
            if line.startswith("OBSLAM-TILT"):
                ident = line
            else:
                time.sleep(0.3)
        if ident is None:
            self.disconnect()
            raise TiltError(f"no OBSLAM-TILT firmware answering on {self.port}")
        self.refresh()

    def disconnect(self) -> None:
        s, self._ser = self._ser, None
        if s:
            try:
                s.close()
            except Exception:  # noqa: BLE001
                pass

    def _readline(self) -> str:
        assert self._ser
        return self._ser.readline().decode(errors="replace").strip()

    def raw(self, line: str) -> str:
        if not self._ser:
            raise TiltError("servo not connected")
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write((line.strip() + "\n").encode())
            reply = self._readline()
            # The firmware prints its banner once at boot; skip it if it arrives late.
            while reply.startswith("OBSLAM-TILT") and line.strip() != "I":
                reply = self._readline()
        if not reply:
            raise TiltError(f"no reply to {line.strip()!r}")
        return reply

    def _ok(self, reply: str) -> float:
        if not reply.startswith("OK"):
            raise TiltError(reply)
        parts = reply.split()
        if len(parts) > 1:
            self.servo_deg = float(parts[1])
        return self.servo_deg

    def move(self, servo_deg: float) -> float:
        return self._ok(self.raw(f"T {servo_deg:.2f}"))

    def home(self) -> float:
        return self._ok(self.raw("H"))

    def refresh(self) -> dict:
        reply = self.raw("?")
        if not reply.startswith("STATUS"):
            raise TiltError(reply)
        kv = {k: float(v) for k, v in _STATUS_RE.findall(reply)}
        self.servo_deg = kv.get("angle", self.servo_deg)
        self.min_deg = kv.get("min", self.min_deg)
        self.max_deg = kv.get("max", self.max_deg)
        self.range = int(kv.get("range", self.range))
        return super().refresh()
