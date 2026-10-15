"""Decode Insta360 trailer records, independently of the MP4 video stream.

Format references: AdrianEddy/telemetry-parser, src/insta360. Metadata is
protobuf; sensor records are binary. Raw IMU ticks are microseconds, while
the older processed-float representation uses millisecond ticks.
"""

import hashlib
import math
import struct
from pathlib import Path

from insta360_ros.models import Exposure, ImuSample, RecordingMetadata

MAGIC = b"8db42d694ccc418790edff439fe026bf"
FOOTER_SIZE = 72


class MetadataError(ValueError):
    """Missing, unsupported, or inconsistent native telemetry."""


def _varint(data: bytes, pos: int) -> tuple[int, int]:
    value = 0
    for shift in range(0, 70, 7):
        if pos >= len(data):
            raise MetadataError("Truncated protobuf varint")
        byte = data[pos]
        pos += 1
        if shift == 63 and byte > 1:
            raise MetadataError("Protobuf uint64 overflow")
        value |= (byte & 127) << shift
        if not byte & 128:
            return value, pos
    raise MetadataError("Invalid protobuf varint")


def _fields(data: bytes, expected_wires: dict[int, int]) -> dict[int, object]:
    result = {}
    pos = 0
    while pos < len(data):
        key, pos = _varint(data, pos)
        tag, wire = key >> 3, key & 7
        if tag == 0:
            raise MetadataError("Invalid protobuf field zero")
        if tag in expected_wires and wire != expected_wires[tag]:
            raise MetadataError(f"Invalid protobuf wire type {wire} for field {tag}")
        if wire == 0:
            value, pos = _varint(data, pos)
        elif wire in (1, 5):
            size = 8 if wire == 1 else 4
            if pos + size > len(data):
                raise MetadataError("Truncated protobuf numeric field")
            value = struct.unpack_from("<d" if wire == 1 else "<f", data, pos)[0]
            pos += size
        elif wire == 2:
            size, pos = _varint(data, pos)
            if size > len(data) - pos:
                raise MetadataError("Truncated protobuf byte field")
            value = data[pos : pos + size]
            pos += size
        else:
            raise MetadataError(f"Unsupported protobuf wire type {wire}")
        result[tag] = value
    return result


def _records(path: Path) -> tuple[dict[int, bytes], int]:
    total = path.stat().st_size
    if total < FOOTER_SIZE + 6:
        raise MetadataError("No original Insta360 telemetry trailer found")
    with path.open("rb") as stream:
        stream.seek(-FOOTER_SIZE, 2)
        footer = stream.read(FOOTER_SIZE)
        if footer[-32:] != MAGIC:
            raise MetadataError(
                "No original Insta360 telemetry trailer found; use an original camera file"
            )
        trailer_size, version = struct.unpack_from("<II", footer, 32)
        if not FOOTER_SIZE + 6 <= trailer_size <= total:
            raise MetadataError("Insta360 trailer size is out of bounds")
        start, end = total - trailer_size, total - FOOTER_SIZE

        def read_at(offset, size):
            if offset < start or size < 0 or offset + size > end:
                raise MetadataError("Insta360 record is out of bounds")
            stream.seek(offset)
            data = stream.read(size)
            if len(data) != size:
                raise MetadataError("Truncated Insta360 record")
            return data

        descriptor = end - 6
        fmt, rid, size = struct.unpack("<BBI", read_at(descriptor, 6))
        entries = {}
        if rid == 0 and size:
            if size % 10:
                raise MetadataError("Invalid Insta360 index length")
            index = read_at(descriptor - size, size)
            for rid, fmt, size, offset in struct.iter_unpack("<BBII", index):
                if rid == 0:
                    continue
                if start + offset + size > end or offset < 0:
                    raise MetadataError("Insta360 index is out of bounds")
                if rid in entries:
                    raise MetadataError(f"Duplicate Insta360 record {rid}")
                entries[rid] = (fmt, start + offset, size)
        else:
            while descriptor >= start:
                fmt, rid, size = struct.unpack("<BBI", read_at(descriptor, 6))
                offset = descriptor - size
                if offset < start:
                    raise MetadataError("Insta360 descriptor is out of bounds")
                if rid in entries:
                    raise MetadataError(f"Duplicate Insta360 record {rid}")
                entries[rid] = (fmt, offset, size)
                descriptor = offset - 6
            if descriptor != start - 6:
                raise MetadataError("Invalid Insta360 trailer boundary")
        for required in (1, 3):
            if required not in entries:
                raise MetadataError(f"Insta360 trailer has no required record {required}")
        if entries[1][0] != 1 or entries[3][0] != 0:
            raise MetadataError("Unsupported Insta360 metadata or IMU encoding")
        records = {
            rid: read_at(offset, size)
            for rid, (_, offset, size) in entries.items()
            if rid in (1, 3, 4)
        }
    return records, version


def read_metadata(path: Path) -> RecordingMetadata:
    """Read native axes and timestamps; do not infer UTC from local camera dates."""
    records, version = _records(Path(path))
    meta = _fields(records[1], {2: 2, 3: 2, 7: 0, 24: 0, 25: 1, 28: 1, 29: 0, 62: 0, 65: 2})
    if 24 not in meta:
        raise MetadataError("Missing first-frame device timestamp")
    raw = bool(meta.get(62, 0))
    tick_ns = 1000 if raw else 1_000_000
    first_frame = int(meta[24])
    if first_frame >= 2**63:
        first_frame -= 2**64
    offset_ms = float(meta.get(28, 0)) if meta.get(29, 0) else 0.0
    if not math.isfinite(offset_ms):
        raise MetadataError("Invalid embedded gyro timestamp offset")
    config = _fields(meta[65], {1: 0, 2: 0}) if 65 in meta else {}
    if any(config[tag] >= 2**32 for tag in (1, 2) if tag in config):
        raise MetadataError("Sensor range exceeds protobuf uint32 width")
    acc_range = int(config[1]) if 1 in config else None
    gyro_range = int(config[2]) if 2 in config else None
    if raw and (not acc_range or not gyro_range):
        raise MetadataError("Raw IMU requires explicit accelerometer and gyroscope ranges")
    item = struct.Struct("<Q6H" if raw else "<Q6d")
    payload = records[3]
    if not payload or len(payload) % item.size:
        raise MetadataError("Invalid or truncated Insta360 IMU record length")
    imu = []
    for row in item.iter_unpack(payload):
        stamp, values = int(row[0]) * tick_ns, row[1:]
        if raw:
            accel = tuple((v - 32768) * acc_range / 32768 * 9.80665 for v in values[:3])
            gyro = tuple((v - 32768) * gyro_range / 32768 * math.pi / 180 for v in values[3:])
        else:
            accel = tuple(v * 9.80665 for v in values[:3])
            gyro = tuple(values[3:])
        if not all(math.isfinite(v) for v in (*accel, *gyro)):
            raise MetadataError("Non-finite IMU sample")
        if imu and stamp <= imu[-1].device_ns:
            raise MetadataError("IMU device timestamps must be strictly increasing")
        imu.append(ImuSample(stamp, gyro, accel))
    exposure_data = records.get(4, b"")
    if len(exposure_data) % 16:
        raise MetadataError("Truncated exposure record")
    exposures = []
    for stamp, duration in struct.iter_unpack("<Qd", exposure_data):
        if not math.isfinite(duration) or duration <= 0:
            raise MetadataError("Invalid exposure duration")
        device_ns = stamp * tick_ns
        if exposures and device_ns <= exposures[-1].device_ns:
            raise MetadataError("Exposure timestamps must be strictly increasing")
        exposures.append(Exposure(device_ns, round(duration * 1e9)))
    return RecordingMetadata(
        camera=meta.get(2, b"Insta360").decode("utf-8"),
        firmware=meta.get(3, b"").decode("utf-8"),
        first_frame_device_ns=first_frame * tick_ns,
        gyro_offset_ns=round(offset_ms * 1_000_000),
        imu=tuple(imu),
        exposures=tuple(exposures),
        raw_imu=raw,
        accel_range_g=acc_range,
        gyro_range_deg_s=gyro_range,
        local_creation_date=str(meta[7]) if meta.get(7) else None,
        rolling_shutter_ms=float(meta.get(25, 0)),
        imu_payload_sha256=hashlib.sha256(payload).hexdigest(),
        trailer_version=version,
    )
