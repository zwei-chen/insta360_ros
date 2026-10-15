"""Exact UTC epoch arithmetic and explicit frame/exposure matching."""

import re
from bisect import bisect_left
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta

from insta360_ros.models import Exposure

EPOCH = datetime(1970, 1, 1, tzinfo=UTC)
ISO_TIME = re.compile(
    r"^(\d{4}-\d{2}-\d{2})[T ](\d{2}:\d{2}:\d{2})(?:\.(\d{1,9}))?(Z|[+-]\d{2}:\d{2})$"
)


def parse_utc_ns(value: str) -> int:
    """Parse a timezone-aware date without float or microsecond truncation."""
    match = ISO_TIME.fullmatch(value)
    if not match:
        raise ValueError(
            "UTC start must be ISO 8601 with an explicit timezone and <=9 fractional digits"
        )
    date, clock, fraction, zone = match.groups()
    if zone != "Z" and (int(zone[1:3]) > 23 or int(zone[4:6]) > 59):
        raise ValueError("Invalid timezone offset; hours must be <24 and minutes <60")
    dt = datetime.fromisoformat(f"{date}T{clock}{'+00:00' if zone == 'Z' else zone}")
    elapsed = dt.astimezone(UTC) - EPOCH
    return (elapsed.days * 86400 + elapsed.seconds) * 1_000_000_000 + int(
        (fraction or "").ljust(9, "0")
    )


def format_utc_ns(value: int) -> str:
    seconds, fraction = divmod(value, 1_000_000_000)
    return (EPOCH + timedelta(seconds=seconds)).strftime("%Y-%m-%dT%H:%M:%S") + f".{fraction:09d}Z"


@dataclass(frozen=True)
class TimeBase:
    utc_start_ns: int
    first_frame_device_ns: int
    gyro_offset_ns: int

    def imu_ns(self, device_ns: int) -> int:
        return self.utc_start_ns + device_ns - self.first_frame_device_ns - self.gyro_offset_ns

    def image_ns(self, device_ns: int) -> int:
        return self.utc_start_ns + device_ns - self.first_frame_device_ns


class ExposureMatcher:
    """Match real PTS to unique exposure records; never silently shift frame indices."""

    def __init__(
        self,
        exposures: tuple[Exposure, ...] | list[Exposure],
        first_frame_device_ns: int,
        tolerance_ns: int,
    ):
        if not exposures:
            raise ValueError("No embedded exposure timestamps; use explicit PTS mode if intended")
        self.exposures = exposures
        self.stamps = [e.device_ns for e in exposures]
        if any(b <= a for a, b in zip(self.stamps, self.stamps[1:])):
            raise ValueError("Embedded exposure timestamps must be strictly increasing")
        self.origin = first_frame_device_ns
        self.tolerance = tolerance_ns
        self.last = -1
        self.max_error_ns = 0

    def match(self, relative_pts_ns: int) -> Exposure:
        target = self.origin + relative_pts_ns
        insertion = bisect_left(self.stamps, target)
        candidates = [i for i in (insertion - 1, insertion) if 0 <= i < len(self.stamps)]
        if not candidates:
            raise ValueError(f"No unused exposure timestamp for frame at {relative_pts_ns} ns")
        index = min(candidates, key=lambda i: abs(self.stamps[i] - target))
        error = abs(self.stamps[index] - target)
        if sum(abs(self.stamps[i] - target) == error for i in candidates) > 1:
            raise ValueError(f"Ambiguous exposure timestamp for frame at {relative_pts_ns} ns")
        if index <= self.last:
            raise ValueError(f"Nearest exposure already used for frame at {relative_pts_ns} ns")
        if error > self.tolerance:
            raise ValueError(
                f"No matching exposure timestamp: {error} ns error exceeds {self.tolerance} ns"
            )
        self.last = index
        self.max_error_ns = max(self.max_error_ns, error)
        return self.exposures[index]
