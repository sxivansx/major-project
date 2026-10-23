"""Tilt servo over the ESP32 line protocol (docs/api.md §2), plus a simulated one."""

from __future__ import annotations

import logging
import re
import threading
import time

import serial

log = logging.getLogger(__name__)

_KV_RE = re.compile(r"(\w+)=(\S+)")


def _parse_kv(reply: str) -> dict:
    out = {}
    for k, v in _KV_RE.findall(reply):
        try:
            out[k] = float(v)
        except ValueError:
            out[k] = v
    return out


class TiltError(Exception):
    pass


class TiltSource:
    port: str
    servo_deg: float
    min_deg: float
    max_deg: float
    range: int
    meas_deg: float | None = None  # AS5600 reading in servo-degree scale, None without an encoder
    encoder: str = "none"  # ok | nomag | weak | strong | err | none | sim
    settled: bool | None = None

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def move(self, servo_deg: float) -> float: ...
    def home(self) -> float: ...
    def raw(self, line: str) -> str: ...

    def wait_settled(self, timeout_ms: int) -> bool:
        time.sleep(min(timeout_ms, 300) / 1000)
        return True

    def refresh(self) -> dict:
        return {
            "servo_deg": self.servo_deg,
            "min_deg": self.min_deg,
            "max_deg": self.max_deg,
            "range": self.range,
            "meas_deg": self.meas_deg,
            "encoder": self.encoder,
            "settled": self.settled,
        }


class SimTilt(TiltSource):
    port = "sim"
    encoder = "sim"
    settled = True

    def __init__(self, range_deg: int = 180):
        self.range = range_deg
        self.min_deg, self.max_deg = 0.0, float(range_deg)
        self.servo_deg = range_deg / 2.0

    @property
    def meas_deg(self) -> float:  # type: ignore[override]
        return self.servo_deg

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def move(self, servo_deg: float) -> float:
        self.servo_deg = float(min(max(servo_deg, self.min_deg), self.max_deg))
        return self.servo_deg

    def home(self) -> float:
        return self.move(self.range / 2.0)

    def wait_settled(self, timeout_ms: int) -> bool:
        return True

    def raw(self, line: str) -> str:
        line = line.strip()
        if line == "I":
            return "OBSLAM-TILT v1 (sim)"
        if line == "?":
            return (
                f"STATUS angle={self.servo_deg:.1f} meas={self.servo_deg:.1f} min={self.min_deg:.1f} "
                f"max={self.max_deg:.1f} range={self.range} us_min=500 us_max=2500 enc=sim settled=1"
            )
        if line.startswith("T "):
            d = self.move(float(line[2:]))
            return f"OK {d:.1f} {d:.1f}"
        if line == "H":
            d = self.home()
            return f"OK {d:.1f} {d:.1f}"
        if line.startswith("W"):
            return f"OK {self.servo_deg:.1f} settled=1"
        return "ERR unknown command"


class _TcpLine:
    """A serial.Serial look-alike over a TCP socket (the ESP32's tilt service on port 8900)."""

    def __init__(self, host: str, port: int, timeout: float = 2.0):
        import socket

        self._sock = socket.create_connection((host, port), timeout=5)
        self._sock.settimeout(timeout)
        self._buf = b""
        self.timeout = timeout

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)

    def readline(self) -> bytes:
        import socket

        self._sock.settimeout(self.timeout)
        while b"\n" not in self._buf:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                return b""
            if not chunk:
                # end of stream: the ESP32 closed the socket (reboot, or another client took it)
                raise ConnectionResetError("tilt link closed by the ESP32")
            self._buf += chunk
        line, _, self._buf = self._buf.partition(b"\n")
        return line + b"\n"

    def reset_input_buffer(self) -> None:
        self._buf = b""
        self._sock.setblocking(False)
        try:
            while self._sock.recv(65536):
                pass
        except (BlockingIOError, OSError):
            pass
        finally:
            self._sock.setblocking(True)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


def open_link(port: str, baud: int = 115200):
    """`tcp://host[:port]` → socket link; anything else → serial port."""
    if port.startswith("tcp://"):
        hp = port[len("tcp://") :]
        host, _, p = hp.partition(":")
        try:
            return _TcpLine(host, int(p or 8900))
        except OSError as e:
            raise TiltError(f"cannot reach {port}: {e}") from e
    try:
        return serial.Serial(port, baud, timeout=2)
    except serial.SerialException as e:
        raise TiltError(str(e)) from e


class SerialTilt(TiltSource):
    """The ESP32 tilt firmware over USB serial, or over TCP when the port is `tcp://host:8900`."""

    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self._ser = None
        self._lock = threading.Lock()
        self.servo_deg = 0.0
        self.min_deg, self.max_deg, self.range = 0.0, 180.0, 180
        self.meas_deg = None
        self.encoder = "none"
        self.settled = None

    def connect(self) -> None:
        self._ser = open_link(self.port, self.baud)
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
        # the W wait blocks inside the firmware; give the serial read a margin above the longest allowed wait
        self._ser.timeout = 12

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

    def _exchange(self, line: str) -> str:
        self._ser.reset_input_buffer()
        self._ser.write((line.strip() + "\n").encode())
        reply = self._readline()
        # The firmware prints its banner once at boot (and on every new TCP
        # client); skip it, and the NOTE / NET lines, if they arrive first.
        while reply.startswith(("OBSLAM-TILT", "NOTE ", "NET ip=")) and line.strip() != "I":
            reply = self._readline()
        return reply

    def raw(self, line: str) -> str:
        if not self._ser:
            raise TiltError("servo not connected")
        with self._lock:
            try:
                reply = self._exchange(line)
            except (OSError, serial.SerialException) as e:
                # Over Wi-Fi the ESP32 closes our socket when it reboots or when another
                # client takes its single tilt connection. Reopen once and repeat the
                # command; it is the same command the operator just asked for.
                if not self.port.startswith("tcp://"):
                    raise TiltError(f"servo link lost: {e}") from e
                log.warning("tilt link to %s lost (%s), reconnecting", self.port, e)
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = open_link(self.port, self.baud)
                try:
                    self._ser.readline()  # banner
                    reply = self._exchange(line)
                except OSError as e2:
                    raise TiltError(f"servo link lost: {e2}") from e2
        if not reply:
            raise TiltError(f"no reply to {line.strip()!r}")
        return reply

    @staticmethod
    def _num(tok: str) -> float | None:
        try:
            v = float(tok)
        except ValueError:
            return None
        return None if v != v else v  # nan → None

    def _ok(self, reply: str) -> float:
        """`OK <deg> [<meas>]`: commanded angle, and the encoder reading when the firmware has one."""
        if not reply.startswith("OK"):
            raise TiltError(reply)
        parts = reply.split()
        if len(parts) > 1 and (v := self._num(parts[1])) is not None:
            self.servo_deg = v
        if len(parts) > 2 and "=" not in parts[2]:
            self.meas_deg = self._num(parts[2])
        return self.servo_deg

    def move(self, servo_deg: float) -> float:
        return self._ok(self.raw(f"T {servo_deg:.2f}"))

    def home(self) -> float:
        return self._ok(self.raw("H"))

    def wait_settled(self, timeout_ms: int) -> bool:
        """`W <ms>` → `OK <meas> settled=<0|1>`; blocks on the firmware side."""
        reply = self.raw(f"W {int(timeout_ms)}")
        if not reply.startswith("OK"):
            raise TiltError(reply)
        parts = reply.split()
        if len(parts) > 1 and self.encoder not in ("none",):
            self.meas_deg = self._num(parts[1])
        kv = _parse_kv(reply)
        self.settled = bool(kv.get("settled", 1))
        return self.settled

    def refresh(self) -> dict:
        reply = self.raw("?")
        if not reply.startswith("STATUS"):
            raise TiltError(reply)
        kv = _parse_kv(reply)
        num = lambda k, d: kv[k] if isinstance(kv.get(k), float) and kv[k] == kv[k] else d  # noqa: E731
        self.servo_deg = num("angle", self.servo_deg)
        self.min_deg = num("min", self.min_deg)
        self.max_deg = num("max", self.max_deg)
        self.range = int(num("range", self.range))
        self.encoder = str(kv.get("enc", "none"))
        self.meas_deg = None if self.encoder == "none" else num("meas", None)
        self.settled = bool(kv["settled"]) if isinstance(kv.get("settled"), float) else None
        return super().refresh()
