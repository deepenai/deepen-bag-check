"""Shared data model: the versioned report shape emitted by both `run_checks()` and
the CLI that wraps it, so any consumer (human, script, or another service) reads the
exact same fields regardless of which output format it asked for."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any

SCHEMA_VERSION = "1.2"
BAG_CHECK_VERSION = "1.0.1"


class TopicRole(str, Enum):
    CAMERA_RAW = "camera_raw"
    CAMERA_COMPRESSED = "camera_compressed"
    CAMERA_H264_UNSUPPORTED = "camera_h264_unsupported"
    LIDAR = "lidar"
    LIDAR_RAW = "lidar_raw"
    # v1.2: a PointCloud2 topic whose field schema and/or point density confidently
    # indicates a radar sensor, not a spinning/solid-state lidar (bagcheck/classify.py's
    # `classify_pointcloud_role`). Radar and lidar share `sensor_msgs/PointCloud2` as
    # their wire type, so this can only be decided after inspecting a sample message —
    # never at connection-level classify_topic() time. Deliberately excluded from every
    # lidar coverage count (bagcheck/coverage.py) so a bag with one lidar + one radar
    # can't be misreported as `multi_lidar`-eligible.
    RADAR = "radar"
    # v1.2: a PointCloud2 topic bagcheck could decode but could not confidently place in
    # either LIDAR or RADAR — the schema and density signals disagree, or neither fired.
    # Conservative by design: excluded from lidar coverage (same as RADAR) rather than
    # silently guessing, and always paired with a WARN check naming the ambiguity.
    LIDAR_AMBIGUOUS = "lidar_ambiguous"
    IMU = "imu"
    CAMERA_INFO = "camera_info"
    TF = "tf"
    TF_STATIC = "tf_static"
    CUSTOM_UNSUPPORTED = "custom_unsupported"
    UNKNOWN = "unknown"


CAMERA_ROLES = (TopicRole.CAMERA_RAW, TopicRole.CAMERA_COMPRESSED)


class CalibrationType(str, Enum):
    LIDAR_CAMERA = "lidar_camera"
    MULTI_LIDAR = "multi_lidar"
    LIDAR_VEHICLE = "lidar_vehicle"
    LIDAR_IMU = "lidar_imu"


class CheckStatus(str, Enum):
    PASS = "pass"
    WARN = "warn"
    FAIL = "fail"


class ReportStatus(str, Enum):
    PASSED = "passed"
    WARNINGS = "warnings"
    FAILED = "failed"


# Exit codes per the spec: 0 pass, 1 pass-with-warnings, 2 fail.
EXIT_CODE_BY_STATUS: dict[ReportStatus, int] = {
    ReportStatus.PASSED: 0,
    ReportStatus.WARNINGS: 1,
    ReportStatus.FAILED: 2,
}


@dataclass
class TopicSummary:
    topic: str
    msgtype: str
    role: TopicRole
    message_count: int
    hz: float | None = None
    vendor_signature: str | None = None
    has_per_point_time: bool | None = None
    encoding: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "topic": self.topic,
            "type": self.msgtype,
            "role": self.role.value,
            "message_count": self.message_count,
            "hz": round(self.hz, 2) if self.hz is not None else None,
            "vendor_signature": self.vendor_signature,
            "has_per_point_time": self.has_per_point_time,
            "encoding": self.encoding,
        }


@dataclass
class CheckResult:
    id: str
    status: CheckStatus
    message: str
    topic: str | None = None

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {"id": self.id, "status": self.status.value, "message": self.message}
        if self.topic is not None:
            d["topic"] = self.topic
        return d


@dataclass
class IneligibleType:
    type: CalibrationType
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {"type": self.type.value, "reason": self.reason}


@dataclass
class ValidationReport:
    status: ReportStatus
    container_format: str
    topics: list[TopicSummary]
    checks: list[CheckResult]
    eligible_calibration_types: list[CalibrationType]
    ineligible_calibration_types: list[IneligibleType]
    schema_version: str = SCHEMA_VERSION
    bag_check_version: str = BAG_CHECK_VERSION
    requested_calibration_type: CalibrationType | None = None

    def exit_code(self) -> int:
        return EXIT_CODE_BY_STATUS[self.status]

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "bag_check_version": self.bag_check_version,
            "status": self.status.value,
            "container_format": self.container_format,
            "requested_calibration_type": (
                self.requested_calibration_type.value
                if self.requested_calibration_type
                else None
            ),
            "topics": [t.to_dict() for t in self.topics],
            "checks": [c.to_dict() for c in self.checks],
            "eligible_calibration_types": [t.value for t in self.eligible_calibration_types],
            "ineligible_calibration_types": [
                t.to_dict() for t in self.ineligible_calibration_types
            ],
        }


def worst_status(statuses: list[CheckStatus]) -> ReportStatus:
    """Roll individual check statuses up into the report-level status."""
    if any(s is CheckStatus.FAIL for s in statuses):
        return ReportStatus.FAILED
    if any(s is CheckStatus.WARN for s in statuses):
        return ReportStatus.WARNINGS
    return ReportStatus.PASSED


__all__ = [
    "BAG_CHECK_VERSION",
    "CAMERA_ROLES",
    "SCHEMA_VERSION",
    "CalibrationType",
    "CheckResult",
    "CheckStatus",
    "IneligibleType",
    "ReportStatus",
    "TopicRole",
    "TopicSummary",
    "ValidationReport",
    "worst_status",
]
