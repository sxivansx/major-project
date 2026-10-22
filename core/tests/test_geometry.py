import numpy as np

from obslam.config import MountConfig
from obslam.geometry import lidar_rays_base, lidar_to_base


def test_level_scan_lands_at_axis_height():
    m = MountConfig(axis_z_mm=80, h_mm=20)
    p = lidar_to_base([0.0, 90.0, 180.0], [1.0, 2.0, 3.0], 0.0, m)
    np.testing.assert_allclose(p[0], [1.0, 0.0, 0.10], atol=1e-6)
    np.testing.assert_allclose(p[1], [0.0, -2.0, 0.10], atol=1e-6)  # native angle is clockwise
    np.testing.assert_allclose(p[2], [-3.0, 0.0, 0.10], atol=1e-6)


def test_tilt_about_x_keeps_x_and_rotates_yz():
    m = MountConfig(axis_z_mm=80, h_mm=0)
    # theta=270 native → ccw 90 → +y direction; tilt +90° lifts +y into +z
    p = lidar_to_base([270.0], [1.0], 90.0, m)
    np.testing.assert_allclose(p[0], [0.0, 0.0, 1.08], atol=1e-6)
    # a point along the axis is unaffected by tilt
    p = lidar_to_base([0.0], [1.5], 45.0, m)
    np.testing.assert_allclose(p[0], [1.5, 0.0, 0.08], atol=1e-6)


def test_per_point_phi_and_yaw():
    m = MountConfig(axis_z_mm=0, h_mm=0, lidar_yaw_deg=90)
    p = lidar_to_base([0.0, 0.0], [1.0, 1.0], [0.0, 90.0], m)
    np.testing.assert_allclose(p[0], [0.0, 1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(p[1], [0.0, 0.0, 1.0], atol=1e-6)


def test_rays_match_points():
    m = MountConfig(axis_z_mm=80, h_mm=15, lidar_yaw_deg=30)
    th = np.array([10.0, 200.0, 300.0])
    r = np.array([2.0, 0.5, 4.0])
    o, d = lidar_rays_base(th, 25.0, m)
    np.testing.assert_allclose(np.linalg.norm(d, axis=1), 1.0)
    np.testing.assert_allclose(o + d * r[:, None], lidar_to_base(th, r, 25.0, m), atol=1e-5)
