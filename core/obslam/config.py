"""Persistent config: mount geometry, grid defaults, lidar filters, microphone array. See docs/api.md §3, §5.3."""

from __future__ import annotations

import json
from pathlib import Path

from pydantic import BaseModel, Field, field_validator


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


# Working layout (24 Oct 2026), in firmware channel order: ch1 top, ch2 left, ch3 right,
# ch4 bottom (hardware team's labels and the I2S wiring: I2S0 = ch1/ch2, I2S1 = ch3/ch4).
# Reliable anchors: left↔right 70 mm, top↔bottom 80 mm; top and bottom centred between the
# sides, the sides ~24 % of the way down (from a top-down photo). Board axes, origin at the LiDAR
# centre projected onto the board. Still calibrated = false: the clap calibration
# (POST /api/audio/calibrate) fits the per-channel capture offsets and the array yaw, can refine
# these positions, and is what earns calibrated = true. Until then bearings are capped at 0.5.
MEASURED_MICS_MM = [[-35.0, 0.0, 0.0], [-70.0, -19.5, 0.0], [0.0, -19.5, 0.0], [-35.0, -80.0, 0.0]]


class AudioConfig(BaseModel):
    port: str = "udp://192.168.1.49:8902"  # udp://ip:8902 (AUD2, recommended) or tcp://ip:8901 (AUD1)
    mics_mm: list[list[float]] = Field(default_factory=lambda: [list(m) for m in MEASURED_MICS_MM])  # sound ports, array frame
    array_yaw_deg: float = 0.0  # bearing of the array +x axis, clockwise from base +x (lidar 0° mark); 0 is an assumption until calibrated
    calibrated: bool = False
    # Not in the original §5.3 block: fixed per-channel capture delays (µs, mic1..4) fitted by the
    # click calibration. A one-sample skew between the I2S0 pair (mics 1, 2) and the I2S1 pair
    # (mics 3, 4) is 20.8 µs, which alone swings bearings by up to ~7° on the 40–80 mm board.
    channel_offsets_us: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    trigger_db: float = Field(12.0, gt=0)  # spike = short-term energy this many dB above the running floor
    min_gap_ms: float = Field(300.0, ge=0)  # dead time between events
    speed_of_sound: float = Field(343.0, gt=0)  # m/s; 331 + 0.6·T°C

    @field_validator("mics_mm")
    @classmethod
    def _four_xyz(cls, v: list[list[float]]) -> list[list[float]]:
        if len(v) != 4 or any(len(m) != 3 for m in v):
            raise ValueError("mics_mm must be four [x, y, z] points in mm (mic1..mic4)")
        return v

    @field_validator("channel_offsets_us")
    @classmethod
    def _four_offsets(cls, v: list[float]) -> list[float]:
        if len(v) != 4:
            raise ValueError("channel_offsets_us must have four values (mic1..mic4)")
        return v


class Config(BaseModel):
    mount: MountConfig = Field(default_factory=MountConfig)
    grid: GridConfig = Field(default_factory=GridConfig)
    lidar: LidarConfig = Field(default_factory=LidarConfig)
    audio: AudioConfig = Field(default_factory=AudioConfig)


def load_config(path: Path) -> Config:
    if path.exists():
        return Config.model_validate(json.loads(path.read_text()))
    return Config()


def save_config(cfg: Config, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(cfg.model_dump(), indent=2) + "\n")
