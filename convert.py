"""Convert original Insta360 recordings to ROS bags or EuRoC / TUM-VI data."""

import sys
from pathlib import Path

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
    from insta360_ros.cli import main

    raise SystemExit(main())
