"""Per-calibration-type minimum sensor coverage.

Minimum sensor sets for each `--for` calibration type:

- `lidar_camera`: >=1 camera + >=1 lidar topic.
- `multi_lidar`: >=2 lidar topics.
- `lidar_vehicle`: >=1 lidar topic. Calibrating a lidar against the vehicle frame
  doesn't require a camera or IMU topic in the bag — a missing CAN/odometry signal is
  surfaced as a warning rather than an eligibility blocker, since motion can also be
  inferred from lidar odometry.
- `lidar_imu`: >=1 lidar + >=1 IMU topic. Reliable lidar-IMU calibration also benefits
  from a GNSS reference for gauge-fixing, which most AV/AMR rigs don't carry — this
  tool treats that as a separate warning (missing `sensor_msgs/NavSatFix`) rather than
  an eligibility blocker, since the minimum topic set is still enough to attempt
  calibration. See the README's "Known limitations" for more on this tradeoff.

"Raw-packet lanes": a `TopicRole.LIDAR_RAW` topic counts toward lidar coverage for all
four types above only when it's a recognized Hesai `pandar_msgs/PandarScan` stream —
the one raw-packet vendor Deepen's calibration engine decodes natively (see
classify.py). Other raw-packet vendors (Velodyne, Ouster) are recognized in the topic
inventory but don't count toward coverage, since nothing in this pipeline can turn
their packets into points yet (bagcheck/checks.py flags them with a WARN instead).
"""

from __future__ import annotations

from dataclasses import dataclass

from bagcheck.classify import LIDAR_RAW_ENGINE_VENDOR
from bagcheck.model import CalibrationType, TopicRole, TopicSummary


@dataclass(frozen=True)
class CoverageResult:
    eligible: bool
    reason: str | None = None
    warning: str | None = None


def _count(topics: list[TopicSummary], role: TopicRole) -> int:
    return sum(1 for t in topics if t.role is role)


def _count_lidar(topics: list[TopicSummary]) -> int:
    """PointCloud2 lidars plus engine-ingestible raw-packet lidars."""
    raw_engine_lidars = sum(
        1
        for t in topics
        if t.role is TopicRole.LIDAR_RAW and t.vendor_signature == LIDAR_RAW_ENGINE_VENDOR
    )
    return _count(topics, TopicRole.LIDAR) + raw_engine_lidars


def _has_gnss(topics: list[TopicSummary]) -> bool:
    return any(t.msgtype.rstrip().endswith("NavSatFix") for t in topics)


def check_coverage(topics: list[TopicSummary], calibration_type: CalibrationType) -> CoverageResult:
    camera_count = _count(topics, TopicRole.CAMERA_RAW) + _count(topics, TopicRole.CAMERA_COMPRESSED)
    lidar_count = _count_lidar(topics)
    imu_count = _count(topics, TopicRole.IMU)

    if calibration_type is CalibrationType.LIDAR_CAMERA:
        if camera_count < 1:
            return CoverageResult(False, "no camera topic found (sensor_msgs/Image or CompressedImage)")
        if lidar_count < 1:
            return CoverageResult(False, "no lidar topic found (sensor_msgs/PointCloud2)")
        return CoverageResult(True)

    if calibration_type is CalibrationType.MULTI_LIDAR:
        if lidar_count < 2:
            return CoverageResult(
                False, f"multi_lidar needs >=2 lidar topics, found {lidar_count}"
            )
        return CoverageResult(True)

    if calibration_type is CalibrationType.LIDAR_VEHICLE:
        if lidar_count < 1:
            return CoverageResult(False, "no lidar topic found (sensor_msgs/PointCloud2)")
        return CoverageResult(True)

    if calibration_type is CalibrationType.LIDAR_IMU:
        if lidar_count < 1:
            return CoverageResult(False, "no lidar topic found (sensor_msgs/PointCloud2)")
        if imu_count < 1:
            return CoverageResult(False, "no IMU topic found (sensor_msgs/Imu)")
        if not _has_gnss(topics):
            return CoverageResult(
                True,
                warning=(
                    "no GNSS (sensor_msgs/NavSatFix) topic found — lidar-IMU calibration "
                    "typically needs GNSS as a gauge-fixing reference; this bag may not "
                    "calibrate even though the minimum topic set is present"
                ),
            )
        return CoverageResult(True)

    raise ValueError(f"unknown calibration type: {calibration_type!r}")
