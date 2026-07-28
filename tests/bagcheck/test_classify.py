from bagcheck.classify import (
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
