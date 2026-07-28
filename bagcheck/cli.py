"""`deepen-bag-check` — the console entrypoint. A thin wrapper: all decision logic
lives in `bagcheck.engine`/`bagcheck.checks`/`bagcheck.coverage`, so the exact same
code is reusable as a library (see `bagcheck.run_checks`) wherever validation needs
to run, not just from the command line."""

from __future__ import annotations

import argparse
import json
import sys

from bagcheck.checks import DEFAULT_MIN_DURATION_S
from bagcheck.containers import BagCheckError
from bagcheck.engine import run_checks
from bagcheck.model import BAG_CHECK_VERSION, CalibrationType
from bagcheck.report import render_human_readable

_FOR_CHOICES = {
    "lidar-camera": CalibrationType.LIDAR_CAMERA,
    "multi-lidar": CalibrationType.MULTI_LIDAR,
    "lidar-vehicle": CalibrationType.LIDAR_VEHICLE,
    "lidar-imu": CalibrationType.LIDAR_IMU,
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="deepen-bag-check",
        description="Validate a rosbag before uploading it for calibration.",
    )
    parser.add_argument("bag_path", help="Path to a .bag file, .db3 file, .mcap file, or ROS2 bag directory")
    parser.add_argument(
        "--for",
        dest="calibration_type",
        choices=sorted(_FOR_CHOICES),
        default=None,
        help="Check minimum sensor coverage for this calibration type; omit to only report what's eligible.",
    )
    parser.add_argument("--json", action="store_true", help="Print the machine-readable report instead of the human one.")
    parser.add_argument(
        "--min-duration-s",
        type=float,
        default=DEFAULT_MIN_DURATION_S,
        help=f"Minimum bag duration to pass (default: {DEFAULT_MIN_DURATION_S}s).",
    )
    parser.add_argument("--version", action="version", version=f"deepen-bag-check {BAG_CHECK_VERSION}")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    calibration_type = _FOR_CHOICES[args.calibration_type] if args.calibration_type else None

    try:
        report = run_checks(args.bag_path, calibration_type=calibration_type, min_duration_s=args.min_duration_s)
    except BagCheckError as exc:
        if args.json:
            print(json.dumps({"error": str(exc)}), file=sys.stdout)
        else:
            print(f"deepen-bag-check: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report.to_dict(), indent=2))
    else:
        print(render_human_readable(report, args.bag_path))

    return report.exit_code()


if __name__ == "__main__":
    sys.exit(main())
