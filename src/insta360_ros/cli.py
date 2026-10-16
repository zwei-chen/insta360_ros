"""Command-line inspection and export."""

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

import av

from insta360_ros import __version__
from insta360_ros.converter import convert
from insta360_ros.core.metadata import read_metadata
from insta360_ros.core.video import inspect_video


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert original Insta360 video/IMU using UTC timestamps."
    )
    parser.add_argument("input", type=Path, help="Original camera MP4/MOV/INSV file")
    parser.add_argument(
        "--format",
        choices=("ros1", "ros2", "euroc"),
        default="euroc",
        help="Output container (default: euroc, EuRoC / TUM-VI monocular layout)",
    )
    parser.add_argument(
        "--output", type=Path, help="New output directory; existing paths are never overwritten"
    )
    parser.add_argument(
        "--compression",
        choices=("none", "zstd"),
        default="none",
        help="ROS2 bag compression (zstd is lossless; default: none)",
    )
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="Print video and telemetry metadata without exporting images",
    )
    parser.add_argument(
        "--utc-start", help="Override the first-frame UTC anchor, e.g. 2025-11-02T03:06:05Z"
    )
    parser.add_argument(
        "--frame-time-source",
        choices=("exposure", "pts"),
        default="exposure",
        help="Use embedded exposure timestamps (default), or explicitly choose video PTS",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Image scale in (0, 1], default: original resolution",
    )
    parser.add_argument("--grayscale", action="store_true", help="Export mono8 instead of RGB8")
    parser.add_argument("--quiet", action="store_true", help="Suppress progress on stderr")
    parser.add_argument("--version", action="version", version=__version__)
    args = parser.parse_args(argv)
    if not args.inspect and args.output is None:
        parser.error("--output is required for conversion")
    try:
        if args.inspect:
            metadata = read_metadata(args.input)
            video = asdict(inspect_video(args.input))
            video["source"] = str(video["source"])
            result = {
                "camera": metadata.camera,
                "firmware": metadata.firmware,
                "video": video,
                "imu_samples": len(metadata.imu),
                "exposure_records": len(metadata.exposures),
                "first_frame_device_ns": metadata.first_frame_device_ns,
                "embedded_gyro_offset_ns": metadata.gyro_offset_ns,
                "camera_local_creation_date_without_timezone": metadata.local_creation_date,
                "native_imu_axes_preserved": True,
            }
        else:

            def progress(count, total):
                if not args.quiet and (count == 1 or count % 100 == 0 or count == total):
                    print(f"Images: {count}/{total or '?'}", file=sys.stderr)

            result = convert(
                args.input,
                args.output,
                args.format,
                scale=args.scale,
                grayscale=args.grayscale,
                utc_start=args.utc_start,
                frame_time_source=args.frame_time_source,
                compression=args.compression,
                progress=progress,
            )
        print(json.dumps(result, indent=2))
        return 0
    except (OSError, ValueError, av.error.FFmpegError) as error:
        print(f"{parser.prog}: {error}", file=sys.stderr)
        return 1
