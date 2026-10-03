"""Sync/quality checks: schema flags, PointCloud2 field mapping, CameraInfo pairing,
tf-tree completeness, duration, motion excitation, and cross-topic time sync/gaps.
Pure functions over plain data — no reader/IO here — so each check is unit-testable
without building a bag.
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping

from bagcheck.classify import LIDAR_RAW_ENGINE_VENDOR, fixit_for_custom_type
from bagcheck.model import CheckResult, CheckStatus, TopicRole, TopicSummary
from bagcheck.pointcloud import normalize_fields
from bagcheck.readers import ConnectionSummary

# Hard floor: below this a bag FAILS. The CLI and the server's upload validation call the
# same `run_checks` with the same defaults, so a bag that passes here passes on upload.
DEFAULT_MIN_DURATION_S = 5.0
# The recording guide's recommendation (WARN, not FAIL): 30 s of continuous driving, 60 s
# when every lidar has 32 beams or fewer. Calibration can still work on shorter bags.
RECOMMENDED_DURATION_S = 30.0
SPARSE_LIDAR_RECOMMENDED_DURATION_S = 60.0
SPARSE_LIDAR_MAX_BEAMS = 32
DEFAULT_MIN_CUMULATIVE_YAW_DEG = 5.0
DEFAULT_MIN_TRANSLATION_M = 1.0
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
    Ouster, RoboSense) aren't decoded by anything in this pipeline, so they're a WARN pointing at
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


def check_ros1_chunk_compression(
    compression: str | None, topics: list[TopicSummary]
) -> list[CheckResult]:
    """Flag a ROS1 `.bag` whose chunks are lz4/bz2-compressed. `rosbags` (what this package
    and Deepen's calibration pipeline both read with) decompresses chunks transparently, so
    compression alone never breaks *validation* — the risk is downstream, at calibration time:
    a bag whose lidar publishes raw Hesai packets is read by decoding those packets directly
    off the bag, which cannot decompress a chunk on the way in (this is exactly why `/validate`
    used to report an identical result for a compressed and decompressed copy of the same bag,
    then let a compressed one fail later). Every other lidar lane reads generic
    `sensor_msgs/PointCloud2`, decompressed the same way `bagcheck` itself already read it to
    run this check — no extra action needed for those."""
    if compression is None:
        return []
    has_raw_hesai_lidar = any(
        t.role is TopicRole.LIDAR_RAW and t.vendor_signature == LIDAR_RAW_ENGINE_VENDOR
        for t in topics
    )
    if has_raw_hesai_lidar:
        return [
            CheckResult(
                id="ros1_chunk_compression",
                status=CheckStatus.WARN,
                message=(
                    f"this bag's chunks are {compression}-compressed. Its raw lidar packets are "
                    "read directly off the bag and cannot be decompressed on the way in — "
                    "re-export this bag uncompressed before running a calibration on it, or "
                    "contact support."
                ),
            )
        ]
    return [
        CheckResult(
            id="ros1_chunk_compression",
            status=CheckStatus.PASS,
            message=(
                f"this bag's chunks are {compression}-compressed — its point-cloud topics "
                "decompress and read normally, no action needed."
            ),
        )
    ]


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


def check_duplicate_cameras(topics: list[TopicSummary]) -> list[CheckResult]:
    """Two ways one recording can hold what looks like two cameras but isn't:
    - one topic on several channels (possible in bare MCAP/db3; bags merge by topic). The
      pipeline keys cameras by topic, so every channel's frames land in one camera.
    - a raw `Image` and a `CompressedImage` in the same camera namespace (the usual
      image_transport pair). Both would be calibrated as separate cameras."""
    results: list[CheckResult] = []
    cams = [t for t in topics if t.role in (TopicRole.CAMERA_RAW, TopicRole.CAMERA_COMPRESSED)]

    channels: dict[str, int] = {}
    for t in cams:
        channels[t.topic] = channels.get(t.topic, 0) + 1
    for topic, n in channels.items():
        if n > 1:
            results.append(
                CheckResult(
                    id="camera_channels_repeated",
                    status=CheckStatus.WARN,
                    topic=topic,
                    message=(
                        f"{topic} is recorded on {n} channels; they will be treated as one "
                        "camera. If they are different cameras, record them on separate topics."
                    ),
                )
            )

    by_namespace: dict[str, dict[TopicRole, list[str]]] = {}
    for t in cams:
        roles = by_namespace.setdefault(_camera_namespace(t.topic), {})
        if t.topic not in roles.setdefault(t.role, []):
            roles[t.role].append(t.topic)
    for roles in by_namespace.values():
        raw = roles.get(TopicRole.CAMERA_RAW, [])
        compressed = roles.get(TopicRole.CAMERA_COMPRESSED, [])
        if raw and compressed:
            a, b = raw[0], compressed[0]
            results.append(
                CheckResult(
                    id="camera_raw_and_compressed",
                    status=CheckStatus.WARN,
                    topic=b,
                    message=(
                        f"{a} and {b} look like the same camera (raw and compressed). "
                        "Untick one in the sensor step to avoid calibrating it twice."
                    ),
                )
            )
    return results


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


def check_duration(
    duration_s: float, min_duration_s: float = DEFAULT_MIN_DURATION_S
) -> CheckResult:
    if duration_s < min_duration_s:
        return CheckResult(
            id="duration",
            status=CheckStatus.FAIL,
            message=(
                f"bag duration {duration_s:.1f}s is below the {min_duration_s:.1f}s minimum "
                "for reliable calibration."
            ),
        )
    return CheckResult(
        id="duration", status=CheckStatus.PASS, message=f"bag duration {duration_s:.1f}s."
    )


# Beam count from a model name in the topic, for lidars whose messages don't say (raw
# packets, or a PointCloud2 without a ring field). Only ever used to pick the duration
# minimum — never to decide a topic's role.
_BEAMS_BY_NAME: tuple[tuple[re.Pattern[str], int | None], ...] = tuple(
    (re.compile(r"(?<![a-z0-9])" + pattern + r"(?![0-9])"), beams)
    for pattern, beams in (
        (r"vlp[-_]?(16|32)", None),
        (r"puck", 16),
        (r"hdl[-_]?(32|64)", None),
        (r"vls[-_]?(128)", None),
        (r"xt[-_]?(16|32)", None),
        (r"pandar[-_]?(40|64|128)", None),
        (r"(?:qt|ot|at)[-_]?(64|128)", None),
        (r"os[-_]?[012d][-_]?(32|64|128)", None),
        (r"rs[-_]?(?:lidar[-_]?)?(16|32|80|128)", None),
        (r"rslidar[-_]?(16|32|80|128)", None),
        (r"helios[-_]?(16|32)", None),
        (r"bpearl", 32),
        (r"ruby", 128),
    )
)


def lidar_beams_from_name(topic: str) -> int | None:
    """The beam count named in a lidar topic (e.g. `/lidar/pandar_xt32/packets` -> 32),
    or None when the topic names no recognisable model."""
    name = topic.lower()
    for pattern, beams in _BEAMS_BY_NAME:
        match = pattern.search(name)
        if match:
            return beams if beams is not None else int(match.group(1))
    return None


def check_recommended_duration(
    duration_s: float, lidar_beams: Mapping[str, int | None]
) -> CheckResult | None:
    """WARN when the bag is shorter than the recording guide recommends: 60 s when every
    lidar is known to have 32 beams or fewer, otherwise 30 s (and, when a lidar's beam
    count is unknown, the message says sparse lidar wants 60 s). None when long enough."""
    sparse = {t: b for t, b in lidar_beams.items() if b is not None and b <= SPARSE_LIDAR_MAX_BEAMS}
    unknown = sorted(t for t, b in lidar_beams.items() if b is None)
    if lidar_beams and len(sparse) == len(lidar_beams):
        recommended = SPARSE_LIDAR_RECOMMENDED_DURATION_S
        named = ", ".join(f"{t}: {b} beams" for t, b in sorted(sparse.items()))
        why = f" for lidar with {SPARSE_LIDAR_MAX_BEAMS} beams or fewer ({named})"
    else:
        recommended = RECOMMENDED_DURATION_S
        why = ""
        if unknown:
            why = (
                f" ({SPARSE_LIDAR_RECOMMENDED_DURATION_S:.0f}s if {', '.join(unknown)} has "
                f"{SPARSE_LIDAR_MAX_BEAMS} beams or fewer — its beam count could not be read)"
            )
    if duration_s >= recommended:
        return None
    return CheckResult(
        id="duration_recommended",
        status=CheckStatus.WARN,
        message=(
            f"bag duration {duration_s:.1f}s is shorter than the recommended "
            f"{recommended:.0f}s{why}. Calibration may still work; record at least "
            f"{recommended:.0f}s of continuous driving for the most reliable result."
        ),
    )


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


def check_translation_excitation(
    range_medians: list[tuple[int, float]],
    min_translation_m: float = DEFAULT_MIN_TRANSLATION_M,
) -> CheckResult | None:
    """`range_medians`: time-ordered `(timestamp_ns, median_range_m)` from subsampled
    lidar scans. Lidar-camera calibration selects camera frames by travelled distance
    (~1m and ~15° between kept frames) and then reconstructs camera motion from their
    overlap — a rig that rotates in place or stands still yields one selected frame and
    the reconstruction fails after the customer has already paid. Pure rotation of a
    spinning lidar leaves the scene's range distribution essentially unchanged, while
    translation shifts it, so the spread of per-scan median ranges is a cheap,
    rotation-insensitive proxy for travelled distance. It is a proxy, not odometry:
    limited-FOV lidars and highly dynamic scenes can move the median without rig
    translation, so an insufficient spread is a WARN (with the consequence spelled
    out), never an eligibility gate. Returns None when fewer than two scans were
    rangeable — missing lidar is already flagged by schema/coverage checks."""
    if len(range_medians) < 2:
        return None
    medians = [m for _, m in range_medians]
    spread_m = max(medians) - min(medians)
    duration_s = (range_medians[-1][0] - range_medians[0][0]) / 1e9

    if spread_m < min_translation_m:
        return CheckResult(
            id="translation_excitation",
            status=CheckStatus.WARN,
            message=(
                f"estimated scene-distance change {spread_m:.2f}m over {duration_s:.1f}s — "
                "the rig may not travel far enough for lidar-camera calibration "
                f"(frame selection needs ~{min_translation_m:.0f}m+ of travel between kept "
                "frames; recordings that rotate in place or stand still fail at "
                "reconstruction). This is an estimate from lidar range drift — if the rig "
                "genuinely moved several meters, you can proceed."
            ),
        )
    return CheckResult(
        id="translation_excitation",
        status=CheckStatus.PASS,
        message=(
            f"estimated scene-distance change {spread_m:.2f}m over {duration_s:.1f}s — "
            "sufficient translation for lidar-camera frame selection."
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
    deltas = sorted(
        b - a for a, b in zip(sorted_timestamps_ns, sorted_timestamps_ns[1:], strict=False)
    )
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
