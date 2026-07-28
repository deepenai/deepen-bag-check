"""Topic-role classification by message type — never by topic name.

Autoware, Nav2, and raw vendor drivers all disagree on topic *names*, so name-based
discovery is ruled out entirely. Role is decided purely from the connection's message
type string; `/tf` vs `/tf_static` is the one legitimate exception, since both publish
the same `tf2_msgs/TFMessage` type and only the topic name distinguishes "the tree
changes over time" from "static".
"""

from __future__ import annotations

from bagcheck.model import TopicRole

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
