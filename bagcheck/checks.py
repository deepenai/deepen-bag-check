"""Sync/quality checks: schema flags, PointCloud2 field mapping, CameraInfo pairing,
tf-tree completeness, duration, motion excitation, and cross-topic time sync/gaps.
Pure functions over plain data — no reader/IO here — so each check is unit-testable
without building a bag.
"""

from __future__ import annotations

import math

from bagcheck.classify import LIDAR_RAW_ENGINE_VENDOR, fixit_for_custom_type
from bagcheck.model import CheckResult, CheckStatus, TopicRole, TopicSummary
from bagcheck.pointcloud import normalize_fields
from bagcheck.readers import ConnectionSummary

DEFAULT_MIN_DURATION_S = 5.0
DEFAULT_MIN_CUMULATIVE_YAW_DEG = 5.0
DEFAULT_MIN_OVERLAP_FRACTION = 0.5
DEFAULT_GAP_FACTOR = 5.0
DEFAULT_MIN_GAP_S = 1.0


def check_schema_types(connections: list[ConnectionSummary]) -> list[CheckResult]:
    """Flag connections whose message type is a known custom/unsupported sensor
    format — e.g. Livox `CustomMsg`, ffmpeg H.264 packets."""
    results = []
    for conn in connections:
        fixit = fixit_for_custom_type(conn.msgtype)
        if fixit:
            results.append(
                CheckResult(
                    id="custom_message_type",
                    status=CheckStatus.WARN,
                    topic=conn.topic,
                    message=f"{conn.topic} publishes {conn.msgtype} — {fixit}",
                )
            )
    return results


def check_lidar_raw_packets(topics: list[TopicSummary]) -> list[CheckResult]:
    """Flag every vendor raw-packet lidar topic ("raw-packet lanes" — see the README).
    Hesai `pandar_msgs/PandarScan` is decoded natively by Deepen's calibration engine,
    so it's informational (PASS); other recognized raw-packet vendors (Velodyne,
    Ouster) aren't decoded by anything in this pipeline, so they're a WARN pointing at
    the vendor driver decode — generic `sensor_msgs/PointCloud2` is the preferred lane
    either way."""
    results: list[CheckResult] = []
    for t in topics:
        if t.role is not TopicRole.LIDAR_RAW:
            continue
        if t.vendor_signature == LIDAR_RAW_ENGINE_VENDOR:
            results.append(
                CheckResult(
                    id="lidar_raw_packets",
                    status=CheckStatus.PASS,
                    topic=t.topic,
                    message=(
                        f"{t.topic}: raw Hesai packets — decoded natively by the calibration "
                        "engine. Generic sensor_msgs/PointCloud2 is still the preferred lane "
                        "where your driver can provide it."
                    ),
                )
            )
        else:
            results.append(
                CheckResult(
                    id="lidar_raw_packets",
                    status=CheckStatus.WARN,
                    topic=t.topic,
                    message=(
                        f"{t.topic}: raw packets require the vendor driver decode or engine "
                        "support — verify. Generic sensor_msgs/PointCloud2 is the preferred lane."
                    ),
                )
            )
    return results


def check_pointcloud_topic(topic: str, field_dtypes: dict[str, str]) -> list[CheckResult]:
    """Vendor field-alias normalization for one lidar topic."""
    roles = normalize_fields(field_dtypes)
    results: list[CheckResult] = []

    geo_missing = [m for m in roles.missing if m in ("x", "y", "z")]
    if geo_missing:
        results.append(
            CheckResult(
                id="pointcloud_field_schema",
                status=CheckStatus.FAIL,
                topic=topic,
                message=(
                    f"{topic}: PointCloud2 is missing required field(s) "
                    f"{', '.join(geo_missing)} — cannot recover point geometry."
                ),
            )
        )
        return results  # no geometry, no point analyzing intensity/ring/time either

    other_missing = [m for m in roles.missing if m not in ("x", "y", "z")]
    if other_missing:
        results.append(
            CheckResult(
                id="pointcloud_field_schema",
                status=CheckStatus.WARN,
                topic=topic,
                message=(
                    f"{topic}: PointCloud2 field mapping incomplete — missing "
                    f"{', '.join(other_missing)} (vendor_signature="
                    f"{roles.vendor_signature or 'unrecognized'})."
                ),
            )
        )
    else:
        results.append(
            CheckResult(
                id="pointcloud_field_schema",
                status=CheckStatus.PASS,
                topic=topic,
                message=(
                    f"{topic}: fields map cleanly to x,y,z,intensity,ring "
                    f"(vendor_signature={roles.vendor_signature or 'unrecognized'})."
                ),
            )
        )

    if roles.has_per_point_time:
        results.append(
            CheckResult(
                id="pointcloud_per_point_time",
                status=CheckStatus.PASS,
                topic=topic,
                message=f"{topic}: per-point time field '{roles.time}' present.",
            )
        )
    else:
        results.append(
            CheckResult(
                id="pointcloud_per_point_time",
                status=CheckStatus.WARN,
                topic=topic,
                message=(
                    f"{topic}: lidar point cloud has no per-point timestamp field — "
                    "targetless motion deskew will be degraded."
                ),
            )
        )
    return results


def _camera_namespace(topic: str) -> str:
    """The path prefix a CameraInfo topic is expected to share with its image topic."""
    t = topic.rstrip("/")
    if t.endswith("/compressed"):
        t = t.rsplit("/", 1)[0]
    return t.rsplit("/", 1)[0] if "/" in t else t


def check_camera_info(
    camera_topics: list[str],
    camera_info_topics: list[str],
    camera_info_k: dict[str, list[float]],
) -> list[CheckResult]:
    """Per camera topic: is there a paired CameraInfo stream, and is K non-zero
    (`K[0] == 0.0` is the ROS convention for "uncalibrated")."""
    info_by_namespace = {_camera_namespace(t): t for t in camera_info_topics}
    results = []
    for cam_topic in camera_topics:
        info_topic = info_by_namespace.get(_camera_namespace(cam_topic))
        if info_topic is None:
            results.append(
                CheckResult(
                    id="camera_info_present",
                    status=CheckStatus.WARN,
                    topic=cam_topic,
                    message=(
                        f"{cam_topic} has no paired CameraInfo topic — supply an "
                        "intrinsics.json sidecar or run intrinsic pre-calibration."
                    ),
                )
            )
            continue
        k = camera_info_k.get(info_topic)
        if k is not None and all(abs(v) < 1e-12 for v in k):
            results.append(
                CheckResult(
                    id="camera_info_present",
                    status=CheckStatus.WARN,
                    topic=cam_topic,
                    message=(
                        f"{info_topic} is present but K is all-zero (uncalibrated, per ROS "
                        "convention) — supply intrinsics.json or run intrinsic pre-calibration."
                    ),
                )
            )
        else:
            results.append(
                CheckResult(
                    id="camera_info_present",
                    status=CheckStatus.PASS,
                    topic=cam_topic,
                    message=f"{cam_topic} paired with {info_topic}, K is non-zero.",
                )
            )
    return results


def check_tf_completeness(
    tf_edges: list[tuple[str, str]],
    sensor_frame_ids: dict[str, str],
    tf_available: bool,
) -> list[CheckResult]:
    """Walk /tf_static (+ /tf) for a path to every sensor frame_id (REP-105 naming).
    `tf_edges` is (parent_frame, child_frame) pairs."""
    if not tf_available:
        return [
            CheckResult(
                id="tf_completeness",
                status=CheckStatus.WARN,
                message=(
                    "no /tf_static topic found — no bag-derived initial extrinsics guess "
                    "is available for any sensor; extrinsics must be supplied manually."
                ),
            )
        ]
    frames_in_graph = {frame for edge in tf_edges for frame in edge}
    results = []
    for topic, frame_id in sensor_frame_ids.items():
        if not frame_id:
            continue
        if frame_id in frames_in_graph:
            results.append(
                CheckResult(
                    id="tf_completeness",
                    status=CheckStatus.PASS,
                    topic=topic,
                    message=f"{topic}: frame '{frame_id}' found in the tf tree.",
                )
            )
        else:
            results.append(
                CheckResult(
                    id="tf_completeness",
                    status=CheckStatus.WARN,
                    topic=topic,
                    message=(
                        f"{topic}: frame '{frame_id}' has no entry in /tf_static — cannot "
                        "bootstrap an initial extrinsics guess for this sensor."
                    ),
                )
            )
    return results


def check_duration(duration_s: float, min_duration_s: float = DEFAULT_MIN_DURATION_S) -> CheckResult:
    if duration_s < min_duration_s:
        return CheckResult(
            id="duration",
            status=CheckStatus.FAIL,
            message=(
                f"bag duration {duration_s:.1f}s is below the {min_duration_s:.1f}s minimum "
                "for reliable calibration."
            ),
        )
    return CheckResult(id="duration", status=CheckStatus.PASS, message=f"bag duration {duration_s:.1f}s.")


def check_motion_excitation(
    imu_samples: list[tuple[int, float]],
    min_cumulative_yaw_deg: float = DEFAULT_MIN_CUMULATIVE_YAW_DEG,
) -> CheckResult:
    """`imu_samples`: time-ordered `(timestamp_ns, angular_velocity_z_rad_s)`. Cumulative
    |yaw rate| integrated over time is a cheap proxy for "did this rig actually turn" —
    trapezoidal integration of gyro z."""
    if len(imu_samples) < 2:
        return CheckResult(
            id="motion_excitation",
            status=CheckStatus.WARN,
            message="not enough IMU samples to estimate rotational excitation.",
        )
    cumulative_rad = 0.0
    for (t0, w0), (t1, w1) in zip(imu_samples, imu_samples[1:], strict=False):
        dt = (t1 - t0) / 1e9
        if dt > 0:
            cumulative_rad += abs((w0 + w1) / 2.0) * dt
    cumulative_deg = math.degrees(cumulative_rad)
    duration_s = (imu_samples[-1][0] - imu_samples[0][0]) / 1e9

    if cumulative_deg < min_cumulative_yaw_deg:
        return CheckResult(
            id="motion_excitation",
            status=CheckStatus.WARN,
            message=(
                f"cumulative yaw {cumulative_deg:.1f}° over {duration_s:.1f}s — "
                f"insufficient rotational excitation for reliable extrinsic calibration "
                f"(need >= {min_cumulative_yaw_deg:.0f}°)."
            ),
        )
    return CheckResult(
        id="motion_excitation",
        status=CheckStatus.PASS,
        message=(
            f"cumulative yaw {cumulative_deg:.1f}° over {duration_s:.1f}s — "
            "sufficient rotational excitation."
        ),
    )


def check_time_sync(
    topic_windows: dict[str, tuple[int, int]],
    min_overlap_fraction: float = DEFAULT_MIN_OVERLAP_FRACTION,
) -> list[CheckResult]:
    """`topic_windows`: {topic: (start_ns, end_ns)} for sensor-role topics. Compares the
    overlap of all windows against their union."""
    if len(topic_windows) < 2:
        return []
    starts = [w[0] for w in topic_windows.values()]
    ends = [w[1] for w in topic_windows.values()]
    overlap_span = max(0, min(ends) - max(starts))
    union_span = max(ends) - min(starts)
    fraction = overlap_span / union_span if union_span > 0 else 1.0

    if fraction < min_overlap_fraction:
        return [
            CheckResult(
                id="cross_topic_overlap",
                status=CheckStatus.WARN,
                message=(
                    f"sensor topics overlap only {fraction * 100:.0f}% of the combined "
                    "recording window — some sensors may lack data for parts of the run."
                ),
            )
        ]
    return [
        CheckResult(
            id="cross_topic_overlap",
            status=CheckStatus.PASS,
            message=f"sensor topics overlap {fraction * 100:.0f}% of the combined recording window.",
        )
    ]


def check_topic_gaps(
    topic: str,
    sorted_timestamps_ns: list[int],
    gap_factor: float = DEFAULT_GAP_FACTOR,
    min_gap_s: float = DEFAULT_MIN_GAP_S,
) -> CheckResult | None:
    """Flag a topic whose largest inter-message gap is well beyond its own median
    period — a likely dropped-frames window rather than a genuinely low rate."""
    if len(sorted_timestamps_ns) < 3:
        return None
    deltas = sorted(b - a for a, b in zip(sorted_timestamps_ns, sorted_timestamps_ns[1:], strict=False))
    median_s = deltas[len(deltas) // 2] / 1e9
    max_gap_s = deltas[-1] / 1e9
    if median_s > 0 and max_gap_s > max(gap_factor * median_s, min_gap_s):
        return CheckResult(
            id="topic_gap",
            status=CheckStatus.WARN,
            topic=topic,
            message=(
                f"{topic}: largest inter-message gap is {max_gap_s:.2f}s vs a "
                f"{median_s:.3f}s median period — possible dropped messages."
            ),
        )
    return None
