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
        conn, _ = self.srv.accept()
        conn.sendall(b"OBSLAM-TILT v1\n")  # boot banner, like the serial firmware
        f = conn.makefile("rwb", buffering=0)
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
