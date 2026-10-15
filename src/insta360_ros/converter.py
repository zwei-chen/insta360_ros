"""Bind native telemetry to frames and export through one chronological pipeline."""

import csv
import json
import math
import shutil
import tempfile
from contextlib import ExitStack
from pathlib import Path

from insta360_ros import __version__
from insta360_ros.core.metadata import read_metadata
from insta360_ros.core.timing import (
    ExposureMatcher,
    TimeBase,
    format_utc_ns,
    parse_utc_ns,
)
from insta360_ros.core.video import decode_frames, inspect_video


def _writer(root: Path, output_format: str):
    if output_format == "euroc":
        from insta360_ros.writers.euroc import EurocWriter

        return EurocWriter(root)
    from insta360_ros.writers.ros import RosWriter

    return RosWriter(root, output_format)


def convert(
    input_path: Path,
    output_path: Path,
    output_format: str,
    *,
    scale: float = 1.0,
    grayscale: bool = False,
    utc_start: str | None = None,
    frame_time_source: str = "exposure",
    progress=None,
) -> dict:
    """Create a new output directory; publish it only after a successful export."""
    source, output = Path(input_path).resolve(), Path(output_path).absolute()
    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")
    if output_format not in ("ros1", "ros2", "euroc"):
        raise ValueError("Output format must be ros1, ros2, or euroc (EuRoC / TUM-VI)")
    if frame_time_source not in ("exposure", "pts"):
        raise ValueError("Frame time source must be exposure or pts")
    if not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError("Scale must be finite and in (0, 1]")
    metadata = read_metadata(source)
    video = inspect_video(source)
    date = utc_start or video.creation_time
    if not date:
        raise ValueError("No timezone-aware UTC creation_time; provide --utc-start explicitly")
    clock = TimeBase(parse_utc_ns(date), metadata.first_frame_device_ns, metadata.gyro_offset_ns)
    if clock.imu_ns(metadata.imu[0].device_ns) < 0:
        raise ValueError("UTC anchor would produce negative IMU timestamps")
    matcher = None
    if frame_time_source == "exposure":
        matcher = ExposureMatcher(
            metadata.exposures,
            metadata.first_frame_device_ns,
            round(0.45 * 1_000_000_000 / video.fps),
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.partial-", dir=output.parent))
    try:
        with ExitStack() as stack:
            writer = stack.enter_context(_writer(staging, output_format))
            frame_file = stack.enter_context(
                (staging / "frame_timestamps.csv").open("w", newline="")
            )
            imu_file = stack.enter_context((staging / "imu_device.csv").open("w", newline=""))
            frame_csv, imu_csv = csv.writer(frame_file), csv.writer(imu_file)
            frame_csv.writerow(
                [
                    "frame_index",
                    "timestamp_ns",
                    "timestamp_device_ns",
                    "relative_pts_ns",
                    "exposure_duration_ns",
                    "exposure_pts_error_ns",
                ]
            )
            imu_csv.writerow(
                [
                    "timestamp_ns",
                    "timestamp_device_ns",
                    "omega_x_rad_s",
                    "omega_y_rad_s",
                    "omega_z_rad_s",
                    "accel_x_m_s2",
                    "accel_y_m_s2",
                    "accel_z_m_s2",
                ]
            )
            counts = {"images": 0, "imu": 0}
            imu_index, first_pts, previous_image_ns = 0, None, None
            output_size = None

            def write_imu(sample):
                timestamp = clock.imu_ns(sample.device_ns)
                writer.write_imu(sample, timestamp)
                imu_csv.writerow([timestamp, sample.device_ns, *sample.gyro, *sample.accel])
                counts["imu"] += 1

            for frame in decode_frames(source, scale, grayscale):
                if first_pts is None:
                    first_pts = frame.pts_ns
                relative_pts = frame.pts_ns - first_pts
                exposure = matcher.match(relative_pts) if matcher else None
                device_ns = (
                    exposure.device_ns
                    if exposure
                    else metadata.first_frame_device_ns + relative_pts
                )
                stamp_ns = clock.image_ns(device_ns)
                if previous_image_ns is not None and stamp_ns <= previous_image_ns:
                    raise ValueError("Mapped image times must be strictly increasing")
                if not (
                    clock.imu_ns(metadata.imu[0].device_ns)
                    <= stamp_ns
                    < clock.imu_ns(metadata.imu[-1].device_ns)
                ):
                    raise ValueError("IMU samples do not bracket every image time")
                previous_image_ns = stamp_ns
                while (
                    imu_index < len(metadata.imu)
                    and clock.imu_ns(metadata.imu[imu_index].device_ns) <= stamp_ns
                ):
                    write_imu(metadata.imu[imu_index])
                    imu_index += 1
                writer.write_image(frame.image, stamp_ns)
                frame_csv.writerow(
                    [
                        frame.index,
                        stamp_ns,
                        device_ns,
                        relative_pts,
                        exposure.duration_ns if exposure else "",
                        device_ns - metadata.first_frame_device_ns - relative_pts
                        if exposure
                        else "",
                    ]
                )
                counts["images"] += 1
                output_size = [int(frame.image.shape[1]), int(frame.image.shape[0])]
                if progress:
                    progress(counts["images"], video.frame_count)
            if not counts["images"]:
                raise ValueError("No video frames were decoded")
            if video.frame_count and counts["images"] != video.frame_count:
                raise ValueError(
                    f"Decoded frame count {counts['images']} differs from container count {video.frame_count}"
                )
            for sample in metadata.imu[imu_index:]:
                write_imu(sample)
        report = {
            "converter_version": __version__,
            "format": output_format,
            "source": str(source),
            "source_size_bytes": source.stat().st_size,
            "counts": counts,
            "camera": metadata.camera,
            "firmware": metadata.firmware,
            "image": {
                "source_size": [video.width, video.height],
                "output_size": output_size,
                "encoding": "mono8" if grayscale else "rgb8",
                "scale": scale,
                "fps": video.fps,
            },
            "timing": {
                "utc_start": format_utc_ns(clock.utc_start_ns),
                "utc_start_ns": clock.utc_start_ns,
                "utc_source": "override" if utc_start else "mp4_creation_time",
                "original_mp4_creation_time": video.creation_time,
                "camera_local_creation_date_without_timezone": metadata.local_creation_date,
                "first_frame_device_ns": metadata.first_frame_device_ns,
                "embedded_gyro_offset_ns": metadata.gyro_offset_ns,
                "frame_time_source": frame_time_source,
                "max_exposure_pts_difference_ns": matcher.max_error_ns if matcher else None,
                "first_imu_utc": format_utc_ns(clock.imu_ns(metadata.imu[0].device_ns)),
                "last_imu_utc": format_utc_ns(clock.imu_ns(metadata.imu[-1].device_ns)),
                "imu_formula": "UTC_start_ns + device_ns - first_frame_device_ns - gyro_offset_ns",
                "image_formula": (
                    "UTC_start_ns + exposure_device_ns - first_frame_device_ns"
                    if matcher
                    else "UTC_start_ns + relative_pts_ns"
                ),
                "utc_anchor_assumption": "MP4 creation_time is anchored to the embedded first-frame device timestamp",
            },
            "imu": {
                "axes": "native Insta360 record axes; no rotation applied",
                "gyro_unit": "rad/s",
                "accel_unit": "m/s^2",
                "raw_record_variant": metadata.raw_imu,
                "accel_range_g": metadata.accel_range_g,
                "gyro_range_deg_s": metadata.gyro_range_deg_s,
                "payload_sha256": metadata.imu_payload_sha256,
                "bias_corrected": False,
                "gravity_removed": False,
                "preroll_and_postroll_preserved": True,
            },
            "rolling_shutter_ms": metadata.rolling_shutter_ms,
            "limitations": [
                "UTC accuracy depends on the camera clock and creation_time precision.",
                "Exposure timestamp start/center convention and residual synchronization delay are not calibrated.",
                "Camera intrinsics, camera-IMU extrinsics and IMU noise parameters must be calibrated separately.",
            ],
        }
        (staging / "conversion.json").write_text(json.dumps(report, indent=2) + "\n")
        if output.exists():
            raise FileExistsError(f"Output appeared during conversion: {output}")
        staging.rename(output)
        return report
    except BaseException:
        shutil.rmtree(staging)
        raise
