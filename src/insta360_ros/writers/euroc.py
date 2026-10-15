"""Monocular EuRoC / TUM-VI output, including ORB-SLAM3's time list."""

import csv
from contextlib import ExitStack
from pathlib import Path

from PIL import Image

from insta360_ros.models import ImuSample


class EurocWriter:
    def __init__(self, root: Path):
        self.stack = ExitStack()
        self.data = root / "mav0/cam0/data"
        self.data.mkdir(parents=True)
        (root / "mav0/imu0").mkdir(parents=True)
        try:
            image_file = self.stack.enter_context(
                (root / "mav0/cam0/data.csv").open("w", newline="")
            )
            imu_file = self.stack.enter_context((root / "mav0/imu0/data.csv").open("w", newline=""))
            self.times = self.stack.enter_context((root / "timestamps.txt").open("w"))
            self.images, self.imu = csv.writer(image_file), csv.writer(imu_file)
            self.images.writerow(["#timestamp [ns]", "filename"])
            self.imu.writerow(
                [
                    "#timestamp [ns]",
                    "w_RS_S_x [rad s^-1]",
                    "w_RS_S_y [rad s^-1]",
                    "w_RS_S_z [rad s^-1]",
                    "a_RS_S_x [m s^-2]",
                    "a_RS_S_y [m s^-2]",
                    "a_RS_S_z [m s^-2]",
                ]
            )
        except BaseException:
            self.stack.close()
            raise

    def write_image(self, image, stamp_ns: int):
        filename = f"{stamp_ns}.png"
        Image.fromarray(image).save(self.data / filename)
        self.images.writerow([stamp_ns, filename])
        self.times.write(f"{stamp_ns}\n")

    def write_imu(self, sample: ImuSample, stamp_ns: int):
        self.imu.writerow([stamp_ns, *sample.gyro, *sample.accel])

    def close(self):
        self.stack.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
