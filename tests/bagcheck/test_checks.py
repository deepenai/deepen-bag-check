from bagcheck import checks
from bagcheck.model import CheckStatus, TopicRole, TopicSummary
from bagcheck.readers import ConnectionSummary


def test_check_schema_types_flags_livox_only() -> None:
    connections = [
        ConnectionSummary("/livox/lidar", "livox_ros_driver/msg/CustomMsg", 10),
        ConnectionSummary("/imu", "sensor_msgs/msg/Imu", 10),
    ]
    results = checks.check_schema_types(connections)
    assert len(results) == 1
    assert results[0].topic == "/livox/lidar"
    assert results[0].status is CheckStatus.WARN


def test_check_lidar_raw_packets_hesai_is_informational_pass() -> None:
    topics = [
        TopicSummary("/lidar", "pandar_msgs/msg/PandarScan", TopicRole.LIDAR_RAW, 10, vendor_signature="hesai")
    ]
    results = checks.check_lidar_raw_packets(topics)
    assert len(results) == 1
    assert results[0].status is CheckStatus.PASS
    assert "decoded natively" in results[0].message


def test_check_lidar_raw_packets_non_hesai_is_warn() -> None:
    topics = [
        TopicSummary(
            "/lidar", "velodyne_msgs/msg/VelodyneScan", TopicRole.LIDAR_RAW, 10, vendor_signature="velodyne"
        )
    ]
    results = checks.check_lidar_raw_packets(topics)
    assert len(results) == 1
    assert results[0].status is CheckStatus.WARN
    assert "vendor driver decode" in results[0].message


def test_check_lidar_raw_packets_ignores_non_raw_topics() -> None:
    topics = [TopicSummary("/lidar", "sensor_msgs/msg/PointCloud2", TopicRole.LIDAR, 10)]
    assert checks.check_lidar_raw_packets(topics) == []


def test_check_pointcloud_topic_fails_on_missing_xyz() -> None:
    results = checks.check_pointcloud_topic("/lidar", {"x": "FLOAT32", "intensity": "FLOAT32"})
    assert results[0].status is CheckStatus.FAIL
    assert "y" in results[0].message and "z" in results[0].message


def test_check_pointcloud_topic_passes_with_full_schema() -> None:
    fields = {"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "FLOAT32", "ring": "UINT16", "time": "FLOAT32"}
    results = checks.check_pointcloud_topic("/lidar", fields)
    statuses = {r.id: r.status for r in results}
    assert statuses["pointcloud_field_schema"] is CheckStatus.PASS
    assert statuses["pointcloud_per_point_time"] is CheckStatus.PASS


def test_check_pointcloud_topic_warns_on_missing_per_point_time() -> None:
    fields = {"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "FLOAT32", "ring": "UINT16"}
    results = checks.check_pointcloud_topic("/lidar", fields)
    time_result = next(r for r in results if r.id == "pointcloud_per_point_time")
    assert time_result.status is CheckStatus.WARN
    assert "deskew" in time_result.message


def test_check_camera_info_warns_when_absent() -> None:
    results = checks.check_camera_info(["/cam/image_raw"], [], {})
    assert results[0].status is CheckStatus.WARN
    assert "no paired CameraInfo" in results[0].message


def test_check_camera_info_warns_on_zeroed_k() -> None:
    results = checks.check_camera_info(["/cam/image_raw"], ["/cam/camera_info"], {"/cam/camera_info": [0.0] * 9})
    assert results[0].status is CheckStatus.WARN
    assert "all-zero" in results[0].message


def test_check_camera_info_passes_with_nonzero_k() -> None:
    k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    results = checks.check_camera_info(["/cam/image_raw"], ["/cam/camera_info"], {"/cam/camera_info": k})
    assert results[0].status is CheckStatus.PASS


def test_check_camera_info_matches_compressed_image_namespace() -> None:
    results = checks.check_camera_info(
        ["/sensor/camera/front/image/compressed"], ["/sensor/camera/front/camera_info"], {}
    )
    # no K sample provided, but the namespace match should still find the pairing
    # rather than reporting "no paired CameraInfo".
    assert "no paired CameraInfo" not in results[0].message


def test_tf_completeness_warns_when_absent() -> None:
    results = checks.check_tf_completeness([], {"/imu": "imu_link"}, tf_available=False)
    assert results[0].status is CheckStatus.WARN
    assert "no /tf_static" in results[0].message


def test_tf_completeness_flags_unreachable_frame() -> None:
    edges = [("base_link", "lidar_link")]
    results = checks.check_tf_completeness(edges, {"/imu": "imu_link"}, tf_available=True)
    assert results[0].status is CheckStatus.WARN
    assert "imu_link" in results[0].message


def test_tf_completeness_passes_for_connected_frame() -> None:
    edges = [("base_link", "imu_link")]
    results = checks.check_tf_completeness(edges, {"/imu": "imu_link"}, tf_available=True)
    assert results[0].status is CheckStatus.PASS


def test_duration_fails_below_minimum() -> None:
    result = checks.check_duration(2.0, min_duration_s=5.0)
    assert result.status is CheckStatus.FAIL


def test_duration_passes_at_or_above_minimum() -> None:
    result = checks.check_duration(5.0, min_duration_s=5.0)
    assert result.status is CheckStatus.PASS


def test_motion_excitation_warns_when_stationary() -> None:
    # Zero angular velocity for 10s — no-motion IMU case.
    samples = [(i * 100_000_000, 0.0) for i in range(100)]
    result = checks.check_motion_excitation(samples)
    assert result.status is CheckStatus.WARN
    assert "insufficient" in result.message


def test_motion_excitation_passes_with_real_rotation() -> None:
    # A steady 0.5 rad/s turn for 3s is well above the 5-degree default threshold.
    samples = [(i * 100_000_000, 0.5) for i in range(30)]
    result = checks.check_motion_excitation(samples)
    assert result.status is CheckStatus.PASS


def test_time_sync_warns_on_low_overlap() -> None:
    windows = {"/a": (0, 10_000_000_000), "/b": (9_000_000_000, 20_000_000_000)}
    results = checks.check_time_sync(windows, min_overlap_fraction=0.5)
    assert results[0].status is CheckStatus.WARN


def test_time_sync_passes_on_full_overlap() -> None:
    windows = {"/a": (0, 10_000_000_000), "/b": (0, 10_000_000_000)}
    results = checks.check_time_sync(windows)
    assert results[0].status is CheckStatus.PASS


def test_topic_gaps_flags_large_gap() -> None:
    stamps = [0, 100_000_000, 200_000_000, 5_200_000_000]  # 5s gap after 0.1s cadence
    result = checks.check_topic_gaps("/lidar", stamps)
    assert result is not None
    assert result.status is CheckStatus.WARN


def test_topic_gaps_none_for_regular_cadence() -> None:
    stamps = [i * 100_000_000 for i in range(10)]
    assert checks.check_topic_gaps("/lidar", stamps) is None
