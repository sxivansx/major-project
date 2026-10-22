"""Lidar frame → base frame. Frames are defined in docs/api.md §3.

Lidar frame: origin at the optical centre, x = the 0° mark, z up. The S2 reports angles
clockwise viewed from above, so the counter-clockwise angle is -angle_deg.

Base frame: origin on the base plate under the tilt axis, x along the tilt axis, z up.

    p = Rx(phi) · (Rz(yaw) · (r cosθ, r sinθ, 0) + (0, 0, h)) + (0, 0, axis_z)
"""

from __future__ import annotations

import numpy as np

from .config import MountConfig


def _tilt(x: np.ndarray, y: np.ndarray, z: np.ndarray, phi_deg, mount: MountConfig):
    ph = np.deg2rad(np.asarray(phi_deg, dtype=np.float64))
    c, s = np.cos(ph), np.sin(ph)
    y2 = y * c - z * s
    z2 = y * s + z * c + mount.axis_z_mm / 1000.0
    return x, y2, z2


def lidar_to_base(theta_deg, range_m, phi_deg, mount: MountConfig) -> np.ndarray:
    """Points (N,3) float32 in the base frame. phi_deg may be a scalar or per-point array."""
    th = np.deg2rad(-np.asarray(theta_deg, dtype=np.float64) + mount.lidar_yaw_deg)
    r = np.asarray(range_m, dtype=np.float64)
    x = r * np.cos(th)
    y = r * np.sin(th)
    z = np.full_like(x, mount.h_mm / 1000.0)
    x, y, z = _tilt(x, y, z, phi_deg, mount)
    return np.stack([x, y, z], axis=1).astype(np.float32)


def lidar_rays_base(theta_deg, phi_deg: float, mount: MountConfig) -> tuple[np.ndarray, np.ndarray]:
    """Ray origin (3,) and unit directions (N,3) in the base frame, for the simulator."""
    th = np.deg2rad(-np.asarray(theta_deg, dtype=np.float64) + mount.lidar_yaw_deg)
    dx, dy, dz = np.cos(th), np.sin(th), np.zeros_like(th)
    ph = np.deg2rad(phi_deg)
    c, s = np.cos(ph), np.sin(ph)
    dirs = np.stack([dx, dy * c - dz * s, dy * s + dz * c], axis=1)
    h = mount.h_mm / 1000.0
    origin = np.array([0.0, -h * s, h * c + mount.axis_z_mm / 1000.0])
    return origin, dirs
