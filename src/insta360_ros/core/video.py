"""Stream images in presentation order while preserving their exact PTS."""

import math
from fractions import Fraction
from pathlib import Path

import av

from insta360_ros.models import VideoFrame, VideoInfo


def inspect_video(path: Path) -> VideoInfo:
    with av.open(str(path)) as container:
        if not container.streams.video:
            raise ValueError("Input has no video stream")
        stream = container.streams.video[0]
        rate = stream.average_rate or stream.guessed_rate
        if not rate or rate <= 0:
            raise ValueError("Cannot determine video frame rate")
        return VideoInfo(
            source=Path(path),
            width=stream.width,
            height=stream.height,
            frame_count=stream.frames,
            fps=float(rate),
            creation_time=container.metadata.get("creation_time")
            or stream.metadata.get("creation_time"),
        )


def decode_frames(path: Path, scale: float = 1.0, grayscale: bool = False):
    """Yield RGB8 or mono8 images without automatic camera-axis rotation."""
    if not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError("Scale must be finite and in (0, 1]")
    with av.open(str(path)) as container:
        stream = container.streams.video[0]
        stream.thread_type = "AUTO"
        previous = None
        for index, frame in enumerate(container.decode(stream)):
            if frame.pts is None or frame.time_base is None:
                raise ValueError("Decoded video frame has no PTS")
            pts_ns = round(Fraction(frame.pts) * frame.time_base * 1_000_000_000)
            if previous is not None and pts_ns <= previous:
                raise ValueError("Decoded video PTS must be strictly increasing")
            previous = pts_ns
            width, height = max(1, round(frame.width * scale)), max(1, round(frame.height * scale))
            image = frame.reformat(
                width=width, height=height, format="gray" if grayscale else "rgb24"
            ).to_ndarray()
            yield VideoFrame(index, pts_ns, image)
