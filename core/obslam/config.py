"""Persistent config: mount geometry, grid defaults, lidar filters. See docs/api.md §3."""

from __future__ import annotations

import json
from pathlib import Path

from pydantic import BaseModel, Field


class MountConfig(BaseModel):
    axis_z_mm: float = 80.0  # tilt axis height above the base plate (from CAD)
    h_mm: float = 0.0  # scan-plane offset from the tilt axis, measured on the rig
    lidar_yaw_deg: float = 0.0  # rotation of the lidar's 0° mark about its own z, in the cradle
    center_deg: float = 90.0  # servo degrees at which the scan plane is horizontal (GS2010MG 180°)
    invert: bool = False  # servo direction vs. phi sign
    enc_center_deg: float = 90.0  # AS5600 reading (servo-degree scale) at which the plane is horizontal
    enc_invert: bool = False  # encoder direction vs. phi sign

    def servo_to_phi(self, servo_deg: float) -> float:
        sign = -1.0 if self.invert else 1.0
        return sign * (servo_deg - self.center_deg)

    def phi_to_servo(self, phi_deg: float) -> float:
        sign = -1.0 if self.invert else 1.0
        return self.center_deg + sign * phi_deg

    def enc_to_phi(self, meas_deg: float) -> float:
        sign = -1.0 if self.enc_invert else 1.0
        return sign * (meas_deg - self.enc_center_deg)


class GridConfig(BaseModel):
    cell_m: float = Field(0.05, gt=0)
    z_min_m: float = -0.5
    z_max_m: float = 2.5
    layer: str = "max"


class LidarConfig(BaseModel):
    min_quality: int = 10
    min_range_m: float = 0.1
    max_range_m: float = 40.0  # RPLIDAR S3: 40 m on 70% reflectivity, 15 m on 10%


class Config(BaseModel):
    mount: MountConfig = Field(default_factory=MountConfig)
    grid: GridConfig = Field(default_factory=GridConfig)
    lidar: LidarConfig = Field(default_factory=LidarConfig)


def load_config(path: Path) -> Config:
    if path.exists():
        return Config.model_validate(json.loads(path.read_text()))
    return Config()


def save_config(cfg: Config, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(cfg.model_dump(), indent=2) + "\n")
