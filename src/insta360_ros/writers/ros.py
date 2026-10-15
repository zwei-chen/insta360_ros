"""Standard ROS bag messages without a ROS runtime dependency."""

from contextlib import ExitStack
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer as Ros1Writer
from rosbags.rosbag2 import Writer as Ros2Writer
from rosbags.typesys import Stores, get_typestore

from insta360_ros.models import ImuSample


class RosWriter:
    def __init__(self, root: Path, output_format: str):
        self.ros1 = output_format == "ros1"
        self.types = get_typestore(Stores.ROS1_NOETIC if self.ros1 else Stores.ROS2_HUMBLE)
        self.stack = ExitStack()
        self.sequence = {"image": 0, "imu": 0}
        self.last_stamp = None
        try:
            self.writer = self.stack.enter_context(
                Ros1Writer(root / "data.bag") if self.ros1 else Ros2Writer(root / "bag", version=9)
            )
            self.connections = {
                "image": self.writer.add_connection(
                    "/insta360/image_raw", "sensor_msgs/msg/Image", typestore=self.types
                ),
                "imu": self.writer.add_connection(
                    "/insta360/imu", "sensor_msgs/msg/Imu", typestore=self.types
                ),
            }
        except BaseException:
            self.stack.close()
            raise

    def _type(self, name):
        return self.types.types[name]

    def _header(self, stamp_ns: int, kind: str):
        seconds, fraction = divmod(stamp_ns, 1_000_000_000)
        limit = 2**32 if self.ros1 else 2**31
        if not 0 <= seconds < limit:
            raise ValueError("UTC timestamp exceeds the ROS message time range")
        time = self._type("builtin_interfaces/msg/Time")(seconds, fraction)
        args = {"stamp": time, "frame_id": f"insta360_{'camera' if kind == 'image' else 'imu'}"}
        if self.ros1:
            args["seq"] = self.sequence[kind]
        self.sequence[kind] += 1
        return self._type("std_msgs/msg/Header")(**args)

    def _write(self, kind: str, message, stamp_ns: int):
        if self.last_stamp is not None and stamp_ns < self.last_stamp:
            raise ValueError("Bag events must be written in chronological order")
        connection = self.connections[kind]
        serialize = self.types.serialize_ros1 if self.ros1 else self.types.serialize_cdr
        self.writer.write(connection, stamp_ns, serialize(message, connection.msgtype))
        self.last_stamp = stamp_ns

    def write_image(self, image, stamp_ns: int):
        image = np.ascontiguousarray(image, dtype=np.uint8)
        grayscale = image.ndim == 2
        if not grayscale and (image.ndim != 3 or image.shape[2] != 3):
            raise ValueError("Image must be mono8 or RGB8")
        height, width = image.shape[:2]
        message = self._type("sensor_msgs/msg/Image")(
            header=self._header(stamp_ns, "image"),
            height=height,
            width=width,
            encoding="mono8" if grayscale else "rgb8",
            is_bigendian=0,
            step=width * (1 if grayscale else 3),
            data=image.reshape(-1),
        )
        self._write("image", message, stamp_ns)

    def write_imu(self, sample: ImuSample, stamp_ns: int):
        vector = self._type("geometry_msgs/msg/Vector3")
        orientation_covariance = np.zeros(9, dtype=np.float64)
        orientation_covariance[0] = -1
        message = self._type("sensor_msgs/msg/Imu")(
            header=self._header(stamp_ns, "imu"),
            orientation=self._type("geometry_msgs/msg/Quaternion")(0.0, 0.0, 0.0, 0.0),
            orientation_covariance=orientation_covariance,
            angular_velocity=vector(*sample.gyro),
            angular_velocity_covariance=np.zeros(9),
            linear_acceleration=vector(*sample.accel),
            linear_acceleration_covariance=np.zeros(9),
        )
        self._write("imu", message, stamp_ns)

    def close(self):
        self.stack.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
