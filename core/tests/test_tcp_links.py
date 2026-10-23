"""The ESP32 over Wi-Fi: tilt protocol on tcp://host:8900, LiDAR bytes via `s2bridge --tcp`."""

import socket
import threading

import pytest

from obslam.s2 import S2ScanSource
from obslam.servo import SerialTilt, TiltError


class FakeTiltServer(threading.Thread):
    """Speaks the firmware's line protocol over TCP, like the ESP32-S3 does on port 8900."""

    def __init__(self):
        super().__init__(daemon=True)
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.angle = 90.0
        self.lines: list[str] = []

    def run(self):
        while True:  # like the ESP32: one client at a time, accepts the next after a drop
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            self.conn = conn
            self.serve(conn)

    def drop(self):
        """The ESP32 rebooting, or another client taking the tilt link."""
        import socket as _s

        self.conn.shutdown(_s.SHUT_RDWR)
        self.conn.close()

    def serve(self, conn):
        conn.sendall(b"OBSLAM-TILT v1\n")  # boot banner, like the serial firmware
        f = conn.makefile("rwb", buffering=0)
        try:
            lines = list(self._lines(f))
        except (OSError, ValueError):
            return

    def _lines(self, f):
        for raw in f:
            line = raw.decode().strip()
            self.lines.append(line)
            if line == "I":
                f.write(b"OBSLAM-TILT v1\n")
            elif line == "?":
                f.write(f"STATUS angle={self.angle:.1f} meas={self.angle + 0.3:.1f} min=0 max=180 range=180 us_min=500 us_max=2500 enc=ok settled=1 ip=192.168.4.7\n".encode())
            elif line.startswith("T "):
                self.angle = float(line[2:])
                f.write(f"OK {self.angle:.1f} {self.angle + 0.3:.1f}\n".encode())
            elif line.startswith("W"):
                f.write(f"OK {self.angle + 0.3:.1f} settled=1\n".encode())
            else:
                f.write(b"ERR unknown cmd\n")
            yield line


def test_tilt_over_tcp():
    srv = FakeTiltServer()
    srv.start()
    t = SerialTilt(f"tcp://127.0.0.1:{srv.port}")
    t.connect()
    assert t.encoder == "ok" and t.meas_deg == pytest.approx(90.3) and t.range == 180
    assert t.move(120.0) == 120.0 and t.meas_deg == pytest.approx(120.3)
    assert t.wait_settled(500) is True
    assert t.raw("?").startswith("STATUS")
    t.disconnect()
    assert "T 120.00" in srv.lines


def test_tilt_tcp_reconnects_after_the_esp_drops_the_socket():
    srv = FakeTiltServer()
    srv.start()
    t = SerialTilt(f"tcp://127.0.0.1:{srv.port}")
    t.connect()
    srv.drop()
    import time

    time.sleep(0.1)
    assert t.move(95.0) == 95.0  # transparently reconnected and repeated
    assert srv.lines.count("T 95.00") == 1


def test_tilt_tcp_unreachable_is_a_tilt_error():
    with pytest.raises(TiltError, match="cannot reach"):
        SerialTilt("tcp://127.0.0.1:1").connect()


def test_s2_source_uses_tcp_flag(tmp_path):
    bridge = tmp_path / "s2bridge"
    bridge.write_text("#!/bin/sh\necho '{\"model\":\"S3\",\"health\":\"good\"}'\n")
    bridge.chmod(0o755)
    src = S2ScanSource("tcp://192.168.4.7:8899", bridge=bridge)
    assert src._argv("--info") == [str(bridge), "--tcp", "192.168.4.7:8899", "--info"]
    assert src.info()["model"] == "S3"
    assert S2ScanSource("/dev/cu.usbserial-1", 1_000_000, bridge=bridge)._argv()[1:3] == ["--serial", "/dev/cu.usbserial-1"]
