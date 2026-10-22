"""S2ScanSource against a fake bridge: driver/check_frames.py --fake generates contract-exact frames."""

import os
import stat
import sys
import time
from pathlib import Path

import pytest

from obslam.s2 import S2ScanSource

ROOT = Path(__file__).resolve().parents[2]
CHECK = ROOT / "driver" / "check_frames.py"


@pytest.fixture
def fake_bridge(tmp_path):
    if not CHECK.exists():
        pytest.skip("driver/check_frames.py missing")
    script = tmp_path / "fakebridge"
    script.write_text(
        "#!/bin/sh\n"
        'case "$*" in\n'
        '  *--info*) echo \'{"model":"S2M1","model_id":113,"firmware":"1.02","hardware":18,"serial":"FAKE","health":"good","health_code":0}\'; exit 0;;\n'
        f'  *) exec {sys.executable} "{CHECK}" --fake 4;;\n'
        "esac\n"
    )
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    return script


def test_info_and_stream(fake_bridge):
    src = S2ScanSource("/dev/cu.fake", 1_000_000, bridge=fake_bridge)
    info = src.info()
    assert info["model"] == "S2M1" and info["health"] == "good"
    got, exits = [], []
    src.start(got.append, exits.append)
    deadline = time.time() + 5
    while len(exits) == 0 and time.time() < deadline:
        time.sleep(0.05)
    src.stop()
    assert len(got) == 4
    f = got[0]
    assert f.n == 3200 and f.angle.dtype.name == "float32" and f.quality.dtype.name == "uint8"
    assert 0 <= f.angle.min() and f.angle.max() < 360
    assert 0.1 < f.range.min() and f.range.max() <= 30
    assert exits == [None]  # stream ended cleanly, no error


def test_missing_bridge_is_a_clear_error(tmp_path):
    src = S2ScanSource("/dev/cu.fake", bridge=tmp_path / "nope")
    with pytest.raises(RuntimeError, match="bridge not built"):
        src.info()


def test_bridge_error_line_is_surfaced(tmp_path):
    script = tmp_path / "failbridge"
    script.write_text('#!/bin/sh\necho \'{"error":"cannot open serial port"}\' >&2\nexit 1\n')
    os.chmod(script, 0o755)
    src = S2ScanSource("/dev/cu.fake", bridge=script)
    with pytest.raises(RuntimeError, match="cannot open serial port"):
        src.info()
