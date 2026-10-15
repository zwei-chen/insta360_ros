"""Native measurements and decoded video frames."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class ImuSample:
    device_ns: int
    gyro: tuple[float, float, float]
    accel: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class Exposure:
    device_ns: int
    duration_ns: int


@dataclass(frozen=True, slots=True)
class RecordingMetadata:
    camera: str
    firmware: str
    first_frame_device_ns: int
    gyro_offset_ns: int
    imu: tuple[ImuSample, ...]
    exposures: tuple[Exposure, ...]
    raw_imu: bool
    accel_range_g: int | None
    gyro_range_deg_s: int | None
    local_creation_date: str | None
    rolling_shutter_ms: float
    imu_payload_sha256: str
    trailer_version: int


@dataclass(frozen=True, slots=True)
class VideoInfo:
    source: Path
    width: int
    height: int
    frame_count: int
    fps: float
    creation_time: str | None


@dataclass(frozen=True, slots=True)
class VideoFrame:
    index: int
    pts_ns: int
    image: Any
