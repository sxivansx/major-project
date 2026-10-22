"""Captured sweeps on disk: sessions/<id>/meta.json + sweeps.npz. docs/api.md §4."""

from __future__ import annotations

import json
import re
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np

from .config import MountConfig
from .geometry import lidar_to_base

ARRAYS = ("step", "phi", "phi_meas", "theta", "range", "quality", "t")  # phi_meas NaN = no encoder
_ID_RE = re.compile(r"^[A-Za-z0-9._-]{1,120}$")


def valid_id(sid: str) -> bool:
    return bool(_ID_RE.match(sid)) and ".." not in sid


class SessionStore:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def new_id(self, name: str) -> str:
        slug = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-") or "scan"
        return f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{slug[:40]}"

    def _dir(self, sid: str) -> Path:
        if not valid_id(sid):
            raise KeyError(sid)
        return self.root / sid

    def save(self, meta: dict, arrays: dict[str, np.ndarray]) -> str:
        sid = meta["id"]
        d = self._dir(sid)
        d.mkdir(parents=True, exist_ok=True)
        cols = {k: np.asarray(arrays[k]) for k in ARRAYS if k in arrays}
        if "phi_meas" not in cols:
            cols["phi_meas"] = np.full(cols["phi"].shape[0], np.nan, np.float32)
        np.savez_compressed(d / "sweeps.npz", **cols)
        (d / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
        return sid

    def list(self) -> list[dict]:
        out = []
        for d in self.root.iterdir():
            m = d / "meta.json"
            if d.is_dir() and m.exists():
                try:
                    out.append(json.loads(m.read_text()))
                except ValueError:
                    continue
        out.sort(key=lambda m: m.get("created", 0), reverse=True)
        return out

    def get(self, sid: str) -> dict:
        m = self._dir(sid) / "meta.json"
        if not m.exists():
            raise KeyError(sid)
        return json.loads(m.read_text())

    def arrays(self, sid: str) -> dict[str, np.ndarray]:
        f = self._dir(sid) / "sweeps.npz"
        if not f.exists():
            raise KeyError(sid)
        with np.load(f) as z:
            out = {k: z[k] for k in ARRAYS if k in z.files}
        if "phi_meas" not in out:  # sessions recorded before the encoder existed
            out["phi_meas"] = np.full(out["phi"].shape[0], np.nan, np.float32)
        return out

    @staticmethod
    def phi_used(a: dict[str, np.ndarray]) -> np.ndarray:
        """Measured tilt where the encoder had it, commanded otherwise."""
        return np.where(np.isfinite(a["phi_meas"]), a["phi_meas"], a["phi"]).astype(np.float32)

    def delete(self, sid: str) -> None:
        d = self._dir(sid)
        if not d.exists():
            raise KeyError(sid)
        shutil.rmtree(d)

    def points(self, sid: str, mount: MountConfig | None = None) -> np.ndarray:
        meta = self.get(sid)
        a = self.arrays(sid)
        m = mount or MountConfig.model_validate(meta.get("mount", {}))
        return lidar_to_base(a["theta"], a["range"], self.phi_used(a), m)

    @staticmethod
    def ply(xyz: np.ndarray) -> bytes:
        p = np.asarray(xyz, np.float32).reshape(-1, 3)
        head = (
            "ply\nformat ascii 1.0\n"
            f"element vertex {p.shape[0]}\n"
            "property float x\nproperty float y\nproperty float z\nend_header\n"
        )
        body = "\n".join(f"{x:.4f} {y:.4f} {z:.4f}" for x, y, z in p)
        return (head + body + ("\n" if p.shape[0] else "")).encode()

    def csv(self, sid: str) -> bytes:
        a = self.arrays(sid)
        xyz = self.points(sid)
        rows = ["phi,phi_meas,theta,range,quality,x,y,z"]
        for i in range(xyz.shape[0]):
            pm = a["phi_meas"][i]
            rows.append(
                f"{a['phi'][i]:.3f},{pm:.3f},{a['theta'][i]:.4f},{a['range'][i]:.4f},{int(a['quality'][i])},"
                f"{xyz[i, 0]:.4f},{xyz[i, 1]:.4f},{xyz[i, 2]:.4f}"
            )
        return ("\n".join(rows) + "\n").encode()
