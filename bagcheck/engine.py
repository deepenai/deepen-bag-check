"""The library entrypoint: `run_checks()`. This is the code the CLI wraps, and it's
designed to be reused as-is by anything else that needs to validate a bag — a bag
that passes locally should never fail differently somewhere downstream because that
caller reimplemented the checks instead of calling this function directly.
"""

from __future__ import annotations

import math
import statistics
import struct
from pathlib import Path
from typing import Any

from bagcheck import checks as _checks
from bagcheck.classify import classify_pointcloud_role, classify_topic, raw_lidar_vendor
from bagcheck.containers import BagCheckError, UnsupportedContainerError, detect_container
from bagcheck.coverage import check_coverage
from bagcheck.model import (
    CAMERA_ROLES,
    CalibrationType,
    CheckResult,
    CheckStatus,
    IneligibleType,
    TopicRole,
    TopicSummary,
    ValidationReport,
    worst_status,
)
from bagcheck.pointcloud import POINTFIELD_DATATYPE_NAMES, RING_ALIASES, normalize_fields
from bagcheck.readers import BagReader, open_reader

DEFAULT_MIN_DURATION_S = _checks.DEFAULT_MIN_DURATION_S

_TF_ROLES = (TopicRole.TF, TopicRole.TF_STATIC)
_SENSOR_ROLES = (
    *CAMERA_ROLES,
    TopicRole.LIDAR,
    TopicRole.LIDAR_RAW,
    TopicRole.RADAR,
    TopicRole.IMU,
)


def run_checks(
    path: str | Path,
    calibration_type: CalibrationType | None = None,
    min_duration_s: float = DEFAULT_MIN_DURATION_S,
) -> ValidationReport:
    """Run the full v1 bag-doctor check list against `path` and return a report.

    `min_duration_s` is the hard floor (FAIL below it). Independently, a bag shorter than
    the recording guide recommends — 30 s, or 60 s when every lidar has 32 beams or fewer —
    gets a `duration_recommended` WARN. The CLI and the server use the same defaults.

    Raises `UnsupportedContainerError` (a `BagCheckError`) for a file that isn't a
    recognizable ROS1 .bag / ROS2 .db3 / ROS2 .mcap container, or that fails to read
    as one — the "clear error for unsupported/corrupt files" requirement.
    """
    path = Path(path)
    detected = detect_container(path)  # raises UnsupportedContainerError on its own

    try:
        with open_reader(detected) as reader:
            return _scan(reader, detected.format.value, calibration_type, min_duration_s)
    except BagCheckError:
        raise
    except Exception as exc:
        # Any failure while actually reading a container we detected as this format
        # (truncated file, unreadable index, etc.) is still "corrupt/unsupported" from
        # the customer's point of view — surfaced as one clear error type.
        raise UnsupportedContainerError(
            f"{path}: could not read as a valid container ({exc})"
        ) from exc


def _scan(
    reader: BagReader,
    container_format: str,
    calibration_type: CalibrationType | None,
    min_duration_s: float,
) -> ValidationReport:
    connections = reader.connections()
    topics = [
        TopicSummary(
            topic=c.topic,
            msgtype=c.msgtype,
            role=classify_topic(c.topic, c.msgtype),
            message_count=c.message_count,
        )
        for c in connections
    ]
    role_by_topic = {t.topic: t.role for t in topics}

    checks: list[CheckResult] = list(_checks.check_schema_types(connections))
    scan = _stream_messages(reader, role_by_topic)

    reader_start, reader_end = reader.start_ns, reader.end_ns
    duration_s = (
        (reader_end - reader_start) / 1e9
        if reader_start is not None and reader_end is not None
        else 0.0
    )
    checks.extend(_enrich_topic_summaries(topics, scan))

    checks.append(_checks.check_duration(duration_s, min_duration_s))
    recommended = _checks.check_recommended_duration(duration_s, _lidar_beams(topics, scan))
    if recommended is not None:
        checks.append(recommended)
    checks.extend(_checks.check_lidar_raw_packets(topics))
    checks.extend(_checks.check_ros1_chunk_compression(reader.ros1_chunk_compression, topics))
    checks.extend(_pointcloud_checks(topics, scan.pointcloud_fields))
    checks.extend(_checks.check_duplicate_cameras(topics))
    checks.extend(
        _checks.check_camera_info(
            list(dict.fromkeys(t.topic for t in topics if t.role in CAMERA_ROLES)),
            [t.topic for t in topics if t.role is TopicRole.CAMERA_INFO],
            scan.camera_info_k,
        )
    )
    checks.extend(
        _checks.check_tf_completeness(
            scan.tf_edges,
            scan.sensor_frame_ids,
            tf_available=bool(scan.tf_edges) or scan.tf_topic_present,
        )
    )
    checks.append(_motion_check(scan.imu_samples))
    # Translation matters specifically to lidar-camera's frame selection, so (matching
    # the coverage-check noise philosophy above) the check only appears when that type
    # was requested or when no type was requested at all.
    if calibration_type is None or calibration_type is CalibrationType.LIDAR_CAMERA:
        translation = _checks.check_translation_excitation(scan.lidar_range_medians)
        if translation is not None:
            checks.append(translation)
    checks.extend(_sync_checks(topics, scan.topic_timestamps))

    eligible, ineligible, coverage_checks = _coverage_checks(topics, calibration_type)
    checks.extend(coverage_checks)
    if calibration_type is not None:
        checks.append(_requested_type_check(calibration_type, eligible, ineligible))

    return ValidationReport(
        status=worst_status([c.status for c in checks]),
        container_format=container_format,
        topics=topics,
        checks=checks,
        eligible_calibration_types=eligible,
        ineligible_calibration_types=ineligible,
        requested_calibration_type=calibration_type,
    )


class _ScanResult:
    """Everything collected in the single pass over the bag's raw messages."""

    def __init__(self) -> None:
        self.topic_timestamps: dict[str, list[int]] = {}
        self.imu_samples: list[tuple[int, float]] = []
        self.pointcloud_fields: dict[str, dict[str, str]] = {}
        self.pointcloud_point_counts: dict[str, int] = {}
        # Beam count read off each lidar topic's first PointCloud2 (max ring + 1, or the
        # height of an organized cloud); None when the cloud doesn't say.
        self.pointcloud_beams: dict[str, int | None] = {}
        self.camera_info_k: dict[str, list[float]] = {}
        self.camera_encoding: dict[str, str] = {}
        self.sensor_frame_ids: dict[str, str] = {}
        self.tf_edges: list[tuple[str, str]] = []
        self.tf_topic_present = False
        # Time-ordered (timestamp_ns, median_range_m) from subsampled scans of the
        # first lidar topic — feeds check_translation_excitation.
        self.lidar_range_medians: list[tuple[int, float]] = []


# Range-sampling budget for the translation check: decode at most one scan every
# _RANGE_SAMPLE_MIN_GAP_NS from the primary lidar topic, capped at _RANGE_SAMPLE_MAX
# scans total, and stride each scan down to at most _RANGE_SAMPLE_POINTS points. Keeps
# the extra decode cost bounded on arbitrarily long/dense bags.
_RANGE_SAMPLE_MIN_GAP_NS = 500_000_000
_RANGE_SAMPLE_MAX = 40
_RANGE_SAMPLE_POINTS = 192


def _median_range(decoded: Any) -> float | None:
    """Median Euclidean range of a strided sample of a PointCloud2's points, or None
    when the cloud has no parseable float32 x/y/z triplet (the pointcloud field-schema
    check reports that separately)."""
    offsets: dict[str, int] = {}
    for f in decoded.fields:
        if (
            f.name in ("x", "y", "z")
            and POINTFIELD_DATATYPE_NAMES.get(int(f.datatype)) == "FLOAT32"
        ):
            offsets[f.name] = int(f.offset)
    if len(offsets) != 3:
        return None
    n_points = int(decoded.width) * int(decoded.height)
    step = int(decoded.point_step)
    data = bytes(decoded.data)
    if n_points == 0 or step <= 0 or len(data) < step:
        return None
    n_points = min(n_points, len(data) // step)
    stride = max(1, n_points // _RANGE_SAMPLE_POINTS)
    ranges: list[float] = []
    for i in range(0, n_points, stride):
        base = i * step
        try:
            x = struct.unpack_from("<f", data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", data, base + offsets["z"])[0]
        except struct.error:
            return None
        r = math.sqrt(x * x + y * y + z * z)
        if math.isfinite(r) and r > 0.0:
            ranges.append(r)
    if not ranges:
        return None
    return statistics.median(ranges)


_RING_STRUCT = {"UINT8": "B", "INT8": "b", "UINT16": "<H", "INT16": "<h", "UINT32": "<I", "INT32": "<i"}
_RING_SCAN_MAX_POINTS = 400_000
_ORGANIZED_BEAMS = range(8, 257)  # an organized lidar cloud's height is its beam count


def _cloud_beams(decoded: Any) -> int | None:
    """Beam count of one PointCloud2: max ring index + 1 when a ring-family field is
    present, else the height of an organized cloud. Every point is read (not a stride),
    because ring indices cycle and a fixed stride can land on the same ring every time."""
    ring = next((f for f in decoded.fields if f.name in RING_ALIASES), None)
    fmt = _RING_STRUCT.get(POINTFIELD_DATATYPE_NAMES.get(int(ring.datatype), "")) if ring else None
    if fmt is not None:
        step = int(decoded.point_step)
        data = bytes(decoded.data)
        n_points = min(int(decoded.width) * int(decoded.height), len(data) // step if step > 0 else 0)
        n_points = min(n_points, _RING_SCAN_MAX_POINTS)
        offset = int(ring.offset)
        top = -1
        try:
            for i in range(n_points):
                value = struct.unpack_from(fmt, data, i * step + offset)[0]
                if value > top:
                    top = value
        except struct.error:
            top = -1
        # An all-zero ring column (stripped by a driver or a conversion) says nothing about
        # the sensor; only a plausible beam count is trusted.
        if top + 1 in _ORGANIZED_BEAMS:
            return top + 1
    height = int(decoded.height)
    return height if height in _ORGANIZED_BEAMS else None


def _lidar_beams(topics: list[TopicSummary], scan: _ScanResult) -> dict[str, int | None]:
    """Beam count per lidar topic (PointCloud2 or raw packets), from the cloud itself
    and, failing that, from a model name in the topic (`checks.lidar_beams_from_name`)."""
    beams: dict[str, int | None] = {}
    for t in topics:
        if t.role not in (TopicRole.LIDAR, TopicRole.LIDAR_RAW):
            continue
        from_cloud = scan.pointcloud_beams.get(t.topic)
        beams[t.topic] = from_cloud if from_cloud is not None else _checks.lidar_beams_from_name(t.topic)
    return beams


def _stream_messages(reader: BagReader, role_by_topic: dict[str, TopicRole]) -> _ScanResult:
    """One pass over every raw message. Only IMU topics are fully deserialized (every
    sample is needed for the motion check); lidar/camera/camera_info topics are
    deserialized once (first sample) purely to inspect their schema; tf is small and
    fully deserialized. Timestamps for every topic are free — the underlying readers
    yield them without decoding message bodies."""
    scan = _ScanResult()
    sampled: set[str] = set()
    range_topic: str | None = None  # first lidar topic seen — the one we range-sample
    last_range_ns: int | None = None

    for msg in reader.messages():
        scan.topic_timestamps.setdefault(msg.topic, []).append(msg.timestamp_ns)
        role = role_by_topic.get(msg.topic)

        if role is TopicRole.IMU:
            decoded = reader.decode(msg.msgtype, msg.rawdata)
            if decoded is not None:
                scan.imu_samples.append((msg.timestamp_ns, float(decoded.angular_velocity.z)))
                if msg.topic not in sampled:
                    scan.sensor_frame_ids[msg.topic] = decoded.header.frame_id
                    sampled.add(msg.topic)

        elif role is TopicRole.LIDAR:
            if range_topic is None:
                range_topic = msg.topic
            want_schema = msg.topic not in sampled
            want_range = (
                msg.topic == range_topic
                and len(scan.lidar_range_medians) < _RANGE_SAMPLE_MAX
                and (
                    last_range_ns is None
                    or msg.timestamp_ns - last_range_ns >= _RANGE_SAMPLE_MIN_GAP_NS
                )
            )
            if want_schema or want_range:
                decoded = reader.decode(msg.msgtype, msg.rawdata)
                if decoded is not None:
                    if want_schema:
                        sampled.add(msg.topic)
                        scan.pointcloud_fields[msg.topic] = {
                            f.name: POINTFIELD_DATATYPE_NAMES.get(int(f.datatype), "?")
                            for f in decoded.fields
                        }
                        # width * height per PointCloud2's own definition (matches the real
                        # Foxglove bag numbers cited in classify.py: radar ~20-30, lidar ~40,000)
                        # — the density signal `classify_pointcloud_role` needs for the topics
                        # whose field schema alone doesn't say radar or lidar.
                        scan.pointcloud_point_counts[msg.topic] = int(decoded.width) * int(
                            decoded.height
                        )
                        scan.pointcloud_beams[msg.topic] = _cloud_beams(decoded)
                        scan.sensor_frame_ids[msg.topic] = decoded.header.frame_id
                    if want_range:
                        median = _median_range(decoded)
                        if median is not None:
                            scan.lidar_range_medians.append((msg.timestamp_ns, median))
                            last_range_ns = msg.timestamp_ns

        elif role is TopicRole.CAMERA_INFO and msg.topic not in sampled:
            sampled.add(msg.topic)
            decoded = reader.decode(msg.msgtype, msg.rawdata)
            if decoded is not None:
                # ROS1's CameraInfo.K became ROS2's CameraInfo.k — a real ROS1 bag decodes
                # through the ROS1-shaped typestore (readers.py), so both spellings occur.
                k_field = decoded.k if hasattr(decoded, "k") else decoded.K
                scan.camera_info_k[msg.topic] = [float(v) for v in k_field]

        elif role in CAMERA_ROLES and msg.topic not in sampled:
            sampled.add(msg.topic)
            decoded = reader.decode(msg.msgtype, msg.rawdata)
            if decoded is not None:
                scan.sensor_frame_ids[msg.topic] = decoded.header.frame_id
                scan.camera_encoding[msg.topic] = (
                    decoded.encoding if role is TopicRole.CAMERA_RAW else decoded.format
                )

        elif role in _TF_ROLES:
            scan.tf_topic_present = True
            decoded = reader.decode(msg.msgtype, msg.rawdata)
            if decoded is not None:
                scan.tf_edges.extend(
                    (t.header.frame_id, t.child_frame_id) for t in decoded.transforms
                )

    return scan


def _enrich_topic_summaries(topics: list[TopicSummary], scan: _ScanResult) -> list[CheckResult]:
    """Mutates each `TopicSummary` in place (hz, vendor_signature, encoding, ...) and,
    for `TopicRole.LIDAR` topics, runs the radar/lidar discriminator on the decoded
    sample (`classify.classify_pointcloud_role`) — reassigning `t.role` to `RADAR` or
    `LIDAR_AMBIGUOUS` when the schema/density signals say so. Returns the check(s) that
    reclassification produces (a WARN for the ambiguous case, an informational note for
    a confident radar call) — `_pointcloud_checks`, called after this, only evaluates
    topics still classified `TopicRole.LIDAR`, so a reclassified topic is automatically
    excluded from lidar-specific field-schema checks too."""
    role_checks: list[CheckResult] = []
    for t in topics:
        stamps = scan.topic_timestamps.get(t.topic, [])
        if len(stamps) >= 2:
            span_s = (max(stamps) - min(stamps)) / 1e9
            t.hz = (len(stamps) - 1) / span_s if span_s > 0 else None
        if t.role is TopicRole.LIDAR:
            fields = scan.pointcloud_fields.get(t.topic)
            if fields:
                decision = classify_pointcloud_role(
                    t.topic, set(fields), scan.pointcloud_point_counts.get(t.topic)
                )
                if decision.role is TopicRole.LIDAR:
                    roles = normalize_fields(fields)
                    t.vendor_signature = roles.vendor_signature
                    t.has_per_point_time = roles.has_per_point_time
                else:
                    t.role = decision.role
                    role_checks.append(
                        CheckResult(
                            id="sensor_role_ambiguous"
                            if decision.role is TopicRole.LIDAR_AMBIGUOUS
                            else "sensor_role_radar",
                            status=CheckStatus.WARN
                            if decision.role is TopicRole.LIDAR_AMBIGUOUS
                            else CheckStatus.PASS,
                            topic=t.topic,
                            message=decision.reason or "",
                        )
                    )
        if t.role is TopicRole.LIDAR_RAW:
            t.vendor_signature = raw_lidar_vendor(t.msgtype)
        if t.role in CAMERA_ROLES:
            t.encoding = scan.camera_encoding.get(t.topic)
    return role_checks


def _pointcloud_checks(
    topics: list[TopicSummary], pointcloud_fields: dict[str, dict[str, str]]
) -> list[CheckResult]:
    results: list[CheckResult] = []
    for t in topics:
        if t.role is not TopicRole.LIDAR:
            continue
        fields = pointcloud_fields.get(t.topic)
        if fields:
            results.extend(_checks.check_pointcloud_topic(t.topic, fields))
        else:
            results.append(
                CheckResult(
                    id="pointcloud_field_schema",
                    status=CheckStatus.WARN,
                    topic=t.topic,
                    message=f"{t.topic}: could not decode a PointCloud2 sample to inspect its field schema.",
                )
            )
    return results


def _motion_check(imu_samples: list[tuple[int, float]]) -> CheckResult:
    if not imu_samples:
        return CheckResult(
            id="motion_excitation",
            status=CheckStatus.WARN,
            message="no IMU topic found — cannot estimate motion excitation from IMU data.",
        )
    return _checks.check_motion_excitation(sorted(imu_samples, key=lambda s: s[0]))


def _sync_checks(
    topics: list[TopicSummary], topic_timestamps: dict[str, list[int]]
) -> list[CheckResult]:
    results: list[CheckResult] = []
    windows: dict[str, tuple[int, int]] = {}
    for t in topics:
        if t.role not in _SENSOR_ROLES:
            continue
        stamps = topic_timestamps.get(t.topic, [])
        if stamps:
            windows[t.topic] = (min(stamps), max(stamps))
        gap_result = _checks.check_topic_gaps(t.topic, sorted(stamps))
        if gap_result:
            results.append(gap_result)
    results.extend(_checks.check_time_sync(windows))
    return results


def _coverage_checks(
    topics: list[TopicSummary], requested_type: CalibrationType | None
) -> tuple[list[CalibrationType], list[IneligibleType], list[CheckResult]]:
    """`eligible_calibration_types`/`ineligible_calibration_types` always cover all four
    types, but a coverage *warning* only becomes a top-level check when it's about the
    type actually being checked (`requested_type`) or when no type was requested at all
    (informational survey mode). A caveat about lidar-IMU's missing GNSS topic shouldn't
    turn a lidar-camera check into "warnings" — that's noise the customer didn't ask
    about, and every check this tool reports should be exact and actionable.
    """
    eligible: list[CalibrationType] = []
    ineligible: list[IneligibleType] = []
    checks: list[CheckResult] = []
    for ctype in CalibrationType:
        result = check_coverage(topics, ctype)
        if result.eligible:
            eligible.append(ctype)
            if result.warning and (requested_type is None or requested_type is ctype):
                checks.append(
                    CheckResult(
                        id=f"coverage_{ctype.value}",
                        status=CheckStatus.WARN,
                        message=result.warning,
                    )
                )
        else:
            ineligible.append(IneligibleType(ctype, result.reason or "requirements not met"))
    return eligible, ineligible, checks


def _requested_type_check(
    calibration_type: CalibrationType,
    eligible: list[CalibrationType],
    ineligible: list[IneligibleType],
) -> CheckResult:
    if calibration_type in eligible:
        return CheckResult(
            id="requested_calibration_coverage",
            status=CheckStatus.PASS,
            message=f"bag meets minimum sensor coverage for --for {calibration_type.value}.",
        )
    reason = next(
        (i.reason for i in ineligible if i.type is calibration_type), "requirements not met"
    )
    return CheckResult(
        id="requested_calibration_coverage",
        status=CheckStatus.FAIL,
        message=f"bag does not meet minimum coverage for --for {calibration_type.value}: {reason}",
    )


__all__ = ["run_checks"]
