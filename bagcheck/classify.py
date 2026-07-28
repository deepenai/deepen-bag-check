"""Topic-role classification by message type — never by topic name.

Autoware, Nav2, and raw vendor drivers all disagree on topic *names*, so name-based
discovery is ruled out entirely. Role is decided purely from the connection's message
type string; `/tf` vs `/tf_static` is the one legitimate exception, since both publish
the same `tf2_msgs/TFMessage` type and only the topic name distinguishes "the tree
changes over time" from "static".
"""

from __future__ import annotations

from dataclasses import dataclass

from bagcheck.model import TopicRole
from bagcheck.pointcloud import RADAR_ONLY_FIELD_NAMES, RING_ALIASES

_STANDARD_ROLE_BY_TYPE: dict[str, TopicRole] = {
    "sensor_msgs/Image": TopicRole.CAMERA_RAW,
    "sensor_msgs/CompressedImage": TopicRole.CAMERA_COMPRESSED,
    "sensor_msgs/PointCloud2": TopicRole.LIDAR,
    "sensor_msgs/Imu": TopicRole.IMU,
    "sensor_msgs/CameraInfo": TopicRole.CAMERA_INFO,
    "tf2_msgs/TFMessage": TopicRole.TF,
}

# Vendor raw-UDP-packet lidar message types ("raw-packet lanes" — see the README).
# Classification only — nothing here decodes a packet. Hesai's `pandar_msgs/PandarScan`
# is the one vendor Deepen's calibration engine ingests natively, decoding raw Hesai
# UDP packets directly. Velodyne/Ouster raw packets are recognized so they're not
# misclassified as [unknown], but nothing in this pipeline decodes them — generic
# `sensor_msgs/PointCloud2` remains the preferred lane.
LIDAR_RAW_ENGINE_VENDOR = "hesai"

_LIDAR_RAW_VENDOR_BY_TYPE: dict[str, str] = {
    "pandar_msgs/PandarScan": LIDAR_RAW_ENGINE_VENDOR,
    "velodyne_msgs/VelodyneScan": "velodyne",
    "ouster_ros/PacketMsg": "ouster",
    "ouster_sensor_msgs/PacketMsg": "ouster",
}

# Non-standard message types known to carry sensor data on some vendor rigs, each with
# a targeted fix-it pointing at the documented conversion.
KNOWN_CUSTOM_SENSOR_TYPES: dict[str, str] = {
    "livox_ros_driver/CustomMsg": (
        "Livox CustomMsg is not ingested directly — convert with livox_to_pointcloud2 "
        "(https://github.com/porizou/livox_to_pointcloud2) before upload."
    ),
    "livox_ros_driver2/CustomMsg": (
        "Livox CustomMsg is not ingested directly — convert with livox_to_pointcloud2 "
        "(https://github.com/porizou/livox_to_pointcloud2) before upload."
    ),
    "ffmpeg_image_transport_msgs/FFMPEGPacket": (
        "H.264/ffmpeg-transport images are not decoded in v1 — re-record with the "
        "'raw' or 'compressed' (JPEG) image_transport instead, or convert first with "
        "ffmpeg_image_transport_tools' uncompress_bag "
        "(https://github.com/ros-misc-utilities/ffmpeg_image_transport_tools)."
    ),
}


def normalize_msgtype(msgtype: str) -> str:
    """Collapse ROS1 (`pkg/Msg`) and ROS2 (`pkg/msg/Msg`) spellings to one key."""
    parts = msgtype.split("/")
    if len(parts) == 3 and parts[1] == "msg":
        return f"{parts[0]}/{parts[2]}"
    return msgtype


def classify_topic(topic: str, msgtype: str) -> TopicRole:
    """Classify a connection's role from its message type (and, for tf only, topic name)."""
    norm = normalize_msgtype(msgtype)

    if norm in KNOWN_CUSTOM_SENSOR_TYPES:
        return TopicRole.CAMERA_H264_UNSUPPORTED if "FFMPEG" in norm else TopicRole.CUSTOM_UNSUPPORTED

    if norm in _LIDAR_RAW_VENDOR_BY_TYPE:
        return TopicRole.LIDAR_RAW

    role = _STANDARD_ROLE_BY_TYPE.get(norm)
    if role is TopicRole.TF:
        return TopicRole.TF_STATIC if topic.rstrip("/").endswith("tf_static") else TopicRole.TF
    return role if role is not None else TopicRole.UNKNOWN


def fixit_for_custom_type(msgtype: str) -> str | None:
    """The targeted conversion instruction for a known custom sensor type, if any."""
    return KNOWN_CUSTOM_SENSOR_TYPES.get(normalize_msgtype(msgtype))


def raw_lidar_vendor(msgtype: str) -> str | None:
    """The recognized vendor for a `TopicRole.LIDAR_RAW` topic's message type, if any."""
    return _LIDAR_RAW_VENDOR_BY_TYPE.get(normalize_msgtype(msgtype))


# --- radar vs. lidar discrimination for TopicRole.LIDAR (v1.2) --------------------
#
# `classify_topic()` above only ever sees a connection's message type — by design, per
# this module's docstring, since topic names disagree across stacks. But radar and
# lidar both publish `sensor_msgs/PointCloud2`, so type alone cannot tell them apart;
# doing so requires inspecting a decoded sample message's field schema and point
# density, which isn't available until the engine streams messages (bagcheck/engine.py
# calls `classify_pointcloud_role` there, after `classify_topic` has already produced a
# tentative TopicRole.LIDAR). See bagcheck/pointcloud.py's docstring for the vendor
# field-name research this leans on, and the real Foxglove demo bag numbers
# (`/radar/points`: bare x,y,z, 20-30 points/message; `/velodyne_points`: x,y,z,
# intensity,ring, ~40,000 points/message) that ground the density thresholds below.

# "Radar-sparse": real radar returns per the vendor drivers researched (smartmicro
# UMRR, TI mmWave) and the live Foxglove bag are tens to low hundreds of points/frame.
RADAR_MAX_POINTS_PER_MESSAGE = 1_000
# "Lidar-dense": a real spinning/solid-state lidar frame (Velodyne VLP-16 and up,
# Ouster, Hesai, RoboSense) is tens of thousands of points/frame.
LIDAR_MIN_POINTS_PER_MESSAGE = 10_000

_RADAR_NAME_TOKENS = ("radar",)
_LIDAR_NAME_TOKENS = ("lidar", "velodyne", "ouster", "hesai", "robosense", "livox", "pandar")

_RING_ALIAS_SET = frozenset(RING_ALIASES)


@dataclass(frozen=True)
class PointcloudRoleDecision:
    """The outcome of `classify_pointcloud_role`. `role` is `TopicRole.LIDAR`,
    `TopicRole.RADAR`, or `TopicRole.LIDAR_AMBIGUOUS`; `reason` is a human-readable
    explanation the caller can attach as a check message (a WARN for the ambiguous
    case, an informational note for a confident radar call)."""

    role: TopicRole
    reason: str | None = None


def _topic_name_hint(topic: str) -> TopicRole | None:
    """Topic name as a TIEBREAKER ONLY (see module docstring's name-vs-type rule) —
    consulted by `classify_pointcloud_role` solely when the schema and density signals
    are both silent, never to override or outvote either one."""
    low = topic.lower()
    is_radar = any(tok in low for tok in _RADAR_NAME_TOKENS)
    is_lidar = any(tok in low for tok in _LIDAR_NAME_TOKENS)
    if is_radar and not is_lidar:
        return TopicRole.RADAR
    if is_lidar and not is_radar:
        return TopicRole.LIDAR
    return None


def classify_pointcloud_role(
    topic: str, field_names: set[str] | frozenset[str], point_count: int | None
) -> PointcloudRoleDecision:
    """Disambiguate a `sensor_msgs/PointCloud2` topic already tentatively classified
    `TopicRole.LIDAR` by `classify_topic()` into `LIDAR`, `RADAR`, or `LIDAR_AMBIGUOUS`.

    `field_names`: the decoded sample message's PointField names. `point_count`:
    `width * height` from that same sample (`None` if unavailable).

    Priority, combining signals rather than trusting any one (see the module
    docstring above for the fix's rationale):
      1. Schema conflict — both a ring-family field *and* a radar-only field present —
         is a genuine disagreement, never resolved by density or name: WARN, ambiguous.
      2. Schema decisive: a ring/channel/laser_id field is the strongest single signal
         this codebase has for lidar (no radar driver researched publishes one) — LIDAR,
         regardless of point count. This also keeps every existing small-`n_points` test
         fixture (built for speed, not realism) classifying correctly.
      3. Schema decisive: a recognized radar-only field with no ring field — RADAR.
      4. Schema silent (e.g. a bare x,y,z or x,y,z,intensity cloud, as the real
         Foxglove `/radar/points` topic is): fall back to point density. Below
         `RADAR_MAX_POINTS_PER_MESSAGE` -> RADAR; at/above
         `LIDAR_MIN_POINTS_PER_MESSAGE` -> LIDAR.
      5. Both schema and density silent (unknown or mid-range point count): topic name
         is a last-resort tiebreaker. If that's silent too, WARN, ambiguous — never
         silently default to lidar, which is exactly the bug this function exists to
         close.
    """
    matched_radar_fields = sorted(RADAR_ONLY_FIELD_NAMES & set(field_names))
    has_ring = bool(_RING_ALIAS_SET & set(field_names))

    if has_ring and matched_radar_fields:
        return PointcloudRoleDecision(
            TopicRole.LIDAR_AMBIGUOUS,
            reason=(
                f"{topic}: has both a ring/channel field (a lidar signal) and radar-shaped "
                f"field(s) {matched_radar_fields} (a radar signal) — cannot confidently "
                "classify as lidar or radar."
            ),
        )
    if has_ring:
        return PointcloudRoleDecision(TopicRole.LIDAR)
    if matched_radar_fields:
        return PointcloudRoleDecision(
            TopicRole.RADAR,
            reason=(
                f"{topic}: radar-shaped PointCloud2 field(s) {matched_radar_fields} present, "
                "no ring/channel/laser_id field — classified as radar, excluded from lidar "
                "coverage."
            ),
        )

    if point_count is not None:
        if point_count < RADAR_MAX_POINTS_PER_MESSAGE:
            return PointcloudRoleDecision(
                TopicRole.RADAR,
                reason=(
                    f"{topic}: {point_count} points/message (radar-sparse, below "
                    f"{RADAR_MAX_POINTS_PER_MESSAGE}) with no lidar/radar field signal — "
                    "classified as radar by point density, excluded from lidar coverage."
                ),
            )
        if point_count >= LIDAR_MIN_POINTS_PER_MESSAGE:
            return PointcloudRoleDecision(TopicRole.LIDAR)

    name_hint = _topic_name_hint(topic)
    if name_hint is TopicRole.RADAR:
        return PointcloudRoleDecision(
            TopicRole.RADAR,
            reason=(
                f"{topic}: no decisive field-schema or point-density signal — classified as "
                "radar from the topic name only (tiebreaker), excluded from lidar coverage."
            ),
        )
    if name_hint is TopicRole.LIDAR:
        return PointcloudRoleDecision(TopicRole.LIDAR)

    density_desc = "unknown" if point_count is None else f"{point_count}"
    return PointcloudRoleDecision(
        TopicRole.LIDAR_AMBIGUOUS,
        reason=(
            f"{topic}: PointCloud2 has no ring/channel field, no recognized radar field, "
            f"{density_desc} points/message is inconclusive, and the topic name gives no "
            "hint — cannot confidently classify as lidar or radar."
        ),
    )
