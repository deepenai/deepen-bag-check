from bagcheck.classify import (
    LIDAR_MIN_POINTS_PER_MESSAGE,
    RADAR_MAX_POINTS_PER_MESSAGE,
    classify_pointcloud_role,
    classify_topic,
    fixit_for_custom_type,
    normalize_msgtype,
    raw_lidar_vendor,
)
from bagcheck.model import TopicRole


def test_normalizes_ros2_style_to_ros1_style() -> None:
    assert normalize_msgtype("sensor_msgs/msg/Imu") == "sensor_msgs/Imu"


def test_leaves_ros1_style_unchanged() -> None:
    assert normalize_msgtype("sensor_msgs/Imu") == "sensor_msgs/Imu"


def test_classifies_by_type_not_name() -> None:
    # A topic named nothing like "imu" still classifies correctly by message type.
    assert classify_topic("/weird/topic/name", "sensor_msgs/msg/Imu") is TopicRole.IMU
    assert classify_topic("/whatever", "sensor_msgs/msg/PointCloud2") is TopicRole.LIDAR
    assert classify_topic("/x", "sensor_msgs/msg/CameraInfo") is TopicRole.CAMERA_INFO


def test_classifies_raw_and_compressed_image() -> None:
    assert classify_topic("/cam/image_raw", "sensor_msgs/msg/Image") is TopicRole.CAMERA_RAW
    assert classify_topic("/cam/image/compressed", "sensor_msgs/msg/CompressedImage") is TopicRole.CAMERA_COMPRESSED


def test_tf_vs_tf_static_distinguished_by_topic_name() -> None:
    assert classify_topic("/tf", "tf2_msgs/msg/TFMessage") is TopicRole.TF
    assert classify_topic("/tf_static", "tf2_msgs/msg/TFMessage") is TopicRole.TF_STATIC


def test_flags_livox_custom_msg_with_targeted_fixit() -> None:
    role = classify_topic("/livox/lidar", "livox_ros_driver/msg/CustomMsg")
    assert role is TopicRole.CUSTOM_UNSUPPORTED
    fixit = fixit_for_custom_type("livox_ros_driver/msg/CustomMsg")
    assert fixit is not None
    assert "livox_to_pointcloud2" in fixit


def test_flags_ffmpeg_h264_transport() -> None:
    role = classify_topic("/cam/image_raw/ffmpeg", "ffmpeg_image_transport_msgs/msg/FFMPEGPacket")
    assert role is TopicRole.CAMERA_H264_UNSUPPORTED
    fixit = fixit_for_custom_type("ffmpeg_image_transport_msgs/msg/FFMPEGPacket")
    assert fixit is not None
    assert "image_transport" in fixit


def test_unknown_type_falls_back_to_unknown_role() -> None:
    assert classify_topic("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray") is TopicRole.UNKNOWN
    assert fixit_for_custom_type("diagnostic_msgs/msg/DiagnosticArray") is None


def test_classifies_hesai_raw_pandar_scan_as_lidar_raw() -> None:
    # Raw vendor packet lidars are recognized (never decoded) rather than falling
    # through to [unknown], regardless of topic name.
    role = classify_topic("/lidar/lidar_1/pandar_xt32/pandar_packets", "pandar_msgs/msg/PandarScan")
    assert role is TopicRole.LIDAR_RAW


def test_classifies_velodyne_and_ouster_raw_packets_as_lidar_raw() -> None:
    assert classify_topic("/velodyne_packets", "velodyne_msgs/msg/VelodyneScan") is TopicRole.LIDAR_RAW
    assert classify_topic("/ouster/lidar_packets", "ouster_ros/msg/PacketMsg") is TopicRole.LIDAR_RAW
    assert classify_topic("/ouster/lidar_packets", "ouster_sensor_msgs/msg/PacketMsg") is TopicRole.LIDAR_RAW


def test_raw_lidar_vendor_identifies_known_vendors() -> None:
    assert raw_lidar_vendor("pandar_msgs/msg/PandarScan") == "hesai"
    assert raw_lidar_vendor("velodyne_msgs/msg/VelodyneScan") == "velodyne"
    assert raw_lidar_vendor("ouster_ros/msg/PacketMsg") == "ouster"
    assert raw_lidar_vendor("sensor_msgs/msg/PointCloud2") is None


# --- classify_pointcloud_role (v1.2 radar/lidar discrimination) -------------------


def test_ring_field_is_decisive_lidar_signal_regardless_of_density() -> None:
    # A ring/channel field is treated as lidar-decisive on its own — a genuinely tiny
    # point count (e.g. a cheap synthetic test fixture) must not flip it to radar.
    decision = classify_pointcloud_role(
        "/lidar/points", {"x", "y", "z", "intensity", "ring"}, point_count=5
    )
    assert decision.role is TopicRole.LIDAR


def test_radar_only_fields_present_is_radar_regardless_of_density() -> None:
    # smartmicro UMRR's real PointCloud2 field names (bagcheck/pointcloud.py docstring).
    decision = classify_pointcloud_role(
        "/radar/points",
        {"x", "y", "z", "radial_speed", "power", "rcs", "snr"},
        point_count=15_000,  # deliberately dense — schema wins over density here
    )
    assert decision.role is TopicRole.RADAR
    assert decision.reason and "radar-shaped" in decision.reason


def test_conflicting_ring_and_radar_fields_is_ambiguous_warn() -> None:
    decision = classify_pointcloud_role(
        "/weird/points", {"x", "y", "z", "ring", "rcs"}, point_count=5000
    )
    assert decision.role is TopicRole.LIDAR_AMBIGUOUS
    assert decision.reason and "ring" in decision.reason and "rcs" in decision.reason


def test_bare_xyz_sparse_cloud_is_radar_by_density() -> None:
    # Reproduces the real Foxglove demo bag's /radar/points topic: no vendor field
    # names survive the radar-to-PointCloud2 conversion, just x,y,z. A neutral topic
    # name proves this is decided by density, not the name tiebreaker.
    decision = classify_pointcloud_role(
        "/sensor_3/points", {"x", "y", "z"}, point_count=RADAR_MAX_POINTS_PER_MESSAGE - 1
    )
    assert decision.role is TopicRole.RADAR
    assert decision.reason and "points/message" in decision.reason


def test_bare_xyz_dense_cloud_is_lidar_by_density() -> None:
    decision = classify_pointcloud_role(
        "/sensor_3/points", {"x", "y", "z"}, point_count=LIDAR_MIN_POINTS_PER_MESSAGE
    )
    assert decision.role is TopicRole.LIDAR


def test_mid_range_density_with_no_name_hint_is_ambiguous_warn() -> None:
    # Schema silent (bare xyz), density between the radar-sparse and lidar-dense
    # thresholds, topic name gives no hint either — must not silently guess.
    mid = (RADAR_MAX_POINTS_PER_MESSAGE + LIDAR_MIN_POINTS_PER_MESSAGE) // 2
    decision = classify_pointcloud_role("/sensor_3/points", {"x", "y", "z"}, point_count=mid)
    assert decision.role is TopicRole.LIDAR_AMBIGUOUS
    assert decision.reason and "inconclusive" in decision.reason


def test_unknown_point_count_with_no_name_hint_is_ambiguous_warn() -> None:
    decision = classify_pointcloud_role("/sensor_3/points", {"x", "y", "z"}, point_count=None)
    assert decision.role is TopicRole.LIDAR_AMBIGUOUS


def test_topic_name_is_tiebreaker_only_when_schema_and_density_are_silent() -> None:
    mid = (RADAR_MAX_POINTS_PER_MESSAGE + LIDAR_MIN_POINTS_PER_MESSAGE) // 2
    radar_named = classify_pointcloud_role("/radar/points", {"x", "y", "z"}, point_count=mid)
    assert radar_named.role is TopicRole.RADAR

    lidar_named = classify_pointcloud_role("/velodyne_points", {"x", "y", "z"}, point_count=mid)
    assert lidar_named.role is TopicRole.LIDAR


def test_topic_name_never_overrides_a_real_density_signal() -> None:
    # A topic literally named "lidar" with a radar-sparse point count and no lidar
    # fields is still radar — name is a tiebreaker, never primary.
    decision = classify_pointcloud_role(
        "/lidar_or_is_it/points", {"x", "y", "z"}, point_count=10
    )
    assert decision.role is TopicRole.RADAR
