# insta360_ros

Extract synchronized images and IMU measurements from original Insta360 videos
and export a **ROS 1 bag**, a **ROS 2 bag**, or a **EuRoC / TUM-VI dataset**.

Tested with Insta360 Ace Pro 2 recordings.

## Installation

```bash
git clone https://github.com/zwei-chen/insta360_ros.git
cd insta360_ros
conda env create -f environment.yml
conda activate insta360_ros
```

## Usage

### ROS 1

```bash
python convert.py /path/to/video.mp4 --format ros1 --output runs/recording_ros1
```

### ROS 2

```bash
python convert.py /path/to/video.mp4 --format ros2 --output runs/recording_ros2
```

### EuRoC / TUM-VI

```bash
python convert.py /path/to/video.mp4 --format euroc --output runs/recording_euroc
```

Inspect a recording:

```bash
python convert.py /path/to/video.mp4 --inspect
```

Use a new output directory for each conversion. Images retain their original
resolution and RGB color by default.

## Output

| Format | Files |
|---|---|
| ROS 1 | `data.bag` |
| ROS 2 | `bag/metadata.yaml`, `bag/*.db3` |
| EuRoC / TUM-VI | `mav0/cam0/data/*.png`, `mav0/cam0/data.csv`, `mav0/imu0/data.csv`, `timestamps.txt` |

ROS topics:

| Topic | Type | Frame |
|---|---|---|
| `/insta360/image_raw` | `sensor_msgs/Image` (`rgb8` or `mono8`) | `insta360_camera` |
| `/insta360/imu` | `sensor_msgs/Imu` | `insta360_imu` |

The EuRoC / TUM-VI export contains monocular images and a seven-column IMU CSV:

```text
timestamp_ns,omega_x,omega_y,omega_z,accel_x,accel_y,accel_z
```

Gyroscope values use rad/s and acceleration uses m/s². Native IMU axes are
preserved. `timestamps.txt` lists one image timestamp in nanoseconds per line.

All formats include `conversion.json`, `frame_timestamps.csv` and `imu_device.csv`
with conversion settings and source timestamps.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `--format` | `euroc` | `ros1`, `ros2`, or `euroc` |
| `--output` | required for conversion | Output directory |
| `--inspect` | off | Print recording metadata |
| `--utc-start` | MP4 `creation_time` | First-frame UTC time, with an explicit timezone |
| `--frame-time-source` | `exposure` | Image timing from `exposure` or video `pts` |
| `--scale` | `1.0` | Image scale in `(0, 1]` |
| `--grayscale` | off | Export grayscale images |
| `--quiet` | off | Suppress progress output |

## References

- [gopro_ros](https://github.com/AutonomousFieldRoboticsLab/gopro_ros)
- [telemetry-parser](https://github.com/AdrianEddy/telemetry-parser)
