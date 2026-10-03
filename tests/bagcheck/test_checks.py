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
        TopicSummary(
            "/lidar",
            "pandar_msgs/msg/PandarScan",
            TopicRole.LIDAR_RAW,
            10,
            vendor_signature="hesai",
        )
    ]
    results = checks.check_lidar_raw_packets(topics)
    assert len(results) == 1
    assert results[0].status is CheckStatus.PASS
    assert "decoded natively" in results[0].message


def test_check_lidar_raw_packets_non_hesai_is_warn() -> None:
    topics = [
        TopicSummary(
            "/lidar",
            "velodyne_msgs/msg/VelodyneScan",
            TopicRole.LIDAR_RAW,
            10,
            vendor_signature="velodyne",
        )
    ]
    results = checks.check_lidar_raw_packets(topics)
    assert len(results) == 1
    assert results[0].status is CheckStatus.WARN
    assert "vendor driver decode" in results[0].message


def test_check_lidar_raw_packets_ignores_non_raw_topics() -> None:
    topics = [TopicSummary("/lidar", "sensor_msgs/msg/PointCloud2", TopicRole.LIDAR, 10)]
    assert checks.check_lidar_raw_packets(topics) == []


def test_check_ros1_chunk_compression_no_op_when_uncompressed() -> None:
    topics = [
        TopicSummary(
            "/lidar",
            "pandar_msgs/msg/PandarScan",
            TopicRole.LIDAR_RAW,
            10,
            vendor_signature="hesai",
        )
    ]
    assert checks.check_ros1_chunk_compression(None, topics) == []


def test_check_ros1_chunk_compression_warns_when_raw_hesai_lidar_is_compressed() -> None:
    topics = [
        TopicSummary(
            "/lidar",
            "pandar_msgs/msg/PandarScan",
            TopicRole.LIDAR_RAW,
            10,
            vendor_signature="hesai",
        )
    ]
    results = checks.check_ros1_chunk_compression("lz4", topics)
    assert len(results) == 1
    assert results[0].status is CheckStatus.WARN
    assert results[0].id == "ros1_chunk_compression"
    assert "lz4" in results[0].message


def test_check_ros1_chunk_compression_is_informational_for_generic_pointcloud_lidar() -> None:
    """No raw-packet lidar on this bag — everything reads through the generic
    `sensor_msgs/PointCloud2` path, which already decompresses transparently."""
    topics = [TopicSummary("/lidar", "sensor_msgs/msg/PointCloud2", TopicRole.LIDAR, 10)]
    results = checks.check_ros1_chunk_compression("bz2", topics)
    assert len(results) == 1
    assert results[0].status is CheckStatus.PASS
    assert "no action needed" in results[0].message


def test_check_ros1_chunk_compression_warns_even_alongside_a_non_hesai_raw_lidar() -> None:
    """A non-Hesai raw-packet lidar (velodyne_msgs, say) is already a separate WARN via
    `check_lidar_raw_packets` — only a *Hesai* raw lane changes this check's own verdict."""
    topics = [
        TopicSummary(
            "/lidar",
            "velodyne_msgs/msg/VelodyneScan",
            TopicRole.LIDAR_RAW,
            10,
            vendor_signature="velodyne",
        )
    ]
    results = checks.check_ros1_chunk_compression("lz4", topics)
    assert results[0].status is CheckStatus.PASS


def test_check_pointcloud_topic_fails_on_missing_xyz() -> None:
    results = checks.check_pointcloud_topic("/lidar", {"x": "FLOAT32", "intensity": "FLOAT32"})
    assert results[0].status is CheckStatus.FAIL
    assert "y" in results[0].message and "z" in results[0].message


def test_check_pointcloud_topic_passes_with_full_schema() -> None:
    fields = {
        "x": "FLOAT32",
        "y": "FLOAT32",
        "z": "FLOAT32",
        "intensity": "FLOAT32",
        "ring": "UINT16",
        "time": "FLOAT32",
    }
    results = checks.check_pointcloud_topic("/lidar", fields)
    statuses = {r.id: r.status for r in results}
    assert statuses["pointcloud_field_schema"] is CheckStatus.PASS
    assert statuses["pointcloud_per_point_time"] is CheckStatus.PASS


def test_check_pointcloud_topic_warns_on_missing_per_point_time() -> None:
    fields = {
        "x": "FLOAT32",
        "y": "FLOAT32",
        "z": "FLOAT32",
        "intensity": "FLOAT32",
        "ring": "UINT16",
    }
    results = checks.check_pointcloud_topic("/lidar", fields)
    time_result = next(r for r in results if r.id == "pointcloud_per_point_time")
    assert time_result.status is CheckStatus.WARN
    assert "deskew" in time_result.message


def test_check_camera_info_warns_when_absent() -> None:
    results = checks.check_camera_info(["/cam/image_raw"], [], {})
    assert results[0].status is CheckStatus.WARN
    assert "no paired CameraInfo" in results[0].message


def test_check_camera_info_warns_on_zeroed_k() -> None:
    results = checks.check_camera_info(
        ["/cam/image_raw"], ["/cam/camera_info"], {"/cam/camera_info": [0.0] * 9}
    )
    assert results[0].status is CheckStatus.WARN
    assert "all-zero" in results[0].message


def test_check_camera_info_passes_with_nonzero_k() -> None:
    k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    results = checks.check_camera_info(
        ["/cam/image_raw"], ["/cam/camera_info"], {"/cam/camera_info": k}
    )
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


def test_translation_excitation_warns_on_static_ranges() -> None:
    # Rotate-in-place / stationary: per-scan median ranges barely move (Z-F2's 7/7
    # reproduced engine failure — frame selection keeps 1 frame, SfM dies at stage 8).
    medians = [(i * 500_000_000, 10.0 + 0.01 * (i % 2)) for i in range(20)]
    result = checks.check_translation_excitation(medians)
    assert result is not None
    assert result.status is CheckStatus.WARN
    assert "rotate in place" in result.message


def test_translation_excitation_passes_on_drifting_ranges() -> None:
    # A rig driving through the scene: median scene range drifts by several meters.
    medians = [(i * 500_000_000, 10.0 + 0.4 * i) for i in range(20)]
    result = checks.check_translation_excitation(medians)
    assert result is not None
    assert result.status is CheckStatus.PASS


def test_translation_excitation_skips_without_samples() -> None:
    # <2 rangeable scans (no lidar, unparseable clouds): no check emitted — missing
    # lidar is already flagged by schema/coverage, a WARN here would be noise.
    assert checks.check_translation_excitation([]) is None
    assert checks.check_translation_excitation([(0, 5.0)]) is None


def _cam(topic: str, role: TopicRole) -> TopicSummary:
    return TopicSummary(topic, "sensor_msgs/msg/Image", role, 10)


def test_duplicate_cameras_warns_on_a_topic_recorded_on_several_channels() -> None:
    results = checks.check_duplicate_cameras(
        [_cam("/cam/front/image", TopicRole.CAMERA_RAW), _cam("/cam/front/image", TopicRole.CAMERA_RAW)]
    )
    assert [(r.id, r.status, r.topic) for r in results] == [
        ("camera_channels_repeated", CheckStatus.WARN, "/cam/front/image")
    ]
    assert "2 channels" in results[0].message


def test_duplicate_cameras_warns_on_a_raw_and_compressed_pair() -> None:
    results = checks.check_duplicate_cameras(
        [
            _cam("/cam/front/image_raw", TopicRole.CAMERA_RAW),
            _cam("/cam/front/image_raw/compressed", TopicRole.CAMERA_COMPRESSED),
        ]
    )
    assert [(r.id, r.status) for r in results] == [("camera_raw_and_compressed", CheckStatus.WARN)]
    assert "/cam/front/image_raw and /cam/front/image_raw/compressed" in results[0].message


def test_duplicate_cameras_is_silent_for_distinct_cameras() -> None:
    assert checks.check_duplicate_cameras(
        [
            _cam("/cam/front/image/compressed", TopicRole.CAMERA_COMPRESSED),
            _cam("/cam/rear/image/compressed", TopicRole.CAMERA_COMPRESSED),
            _cam("/cam/left/image_raw", TopicRole.CAMERA_RAW),
        ]
    ) == []


def test_check_lidar_raw_packets_robosense_is_warn() -> None:
    topics = [
        TopicSummary(
            "/rslidar_packets",
            "rslidar_msg/msg/RslidarPacket",
            TopicRole.LIDAR_RAW,
            message_count=10,
            vendor_signature="robosense",
        )
    ]
    results = checks.check_lidar_raw_packets(topics)
    assert [r.status for r in results] == [CheckStatus.WARN]


def test_hard_floor_is_unchanged_and_30_60s_is_recommended() -> None:
    assert checks.DEFAULT_MIN_DURATION_S == 5.0
    assert checks.RECOMMENDED_DURATION_S == 30.0
    assert checks.SPARSE_LIDAR_RECOMMENDED_DURATION_S == 60.0


def test_recommended_duration_dense_lidar_over_30s_is_quiet() -> None:
    assert checks.check_recommended_duration(31.0, {"/lidar": 64}) is None


def test_recommended_duration_dense_lidar_under_30s_warns() -> None:
    result = checks.check_recommended_duration(20.0, {"/lidar": 64})
    assert result is not None and result.status is CheckStatus.WARN
    assert "recommended 30s" in result.message and "may still work" in result.message


def test_recommended_duration_sparse_lidar_wants_60s() -> None:
    result = checks.check_recommended_duration(45.0, {"/lidar": 32})
    assert result is not None and "recommended 60s" in result.message and "32 beams" in result.message
    assert checks.check_recommended_duration(60.0, {"/lidar": 32}) is None


def test_recommended_duration_needs_every_lidar_sparse_for_60s() -> None:
    assert checks.check_recommended_duration(45.0, {"/front": 16, "/roof": 128}) is None


def test_recommended_duration_unknown_beams_recommends_30s_and_mentions_60s() -> None:
    result = checks.check_recommended_duration(20.0, {"/lidar": None})
    assert result is not None and "recommended 30s" in result.message and "60s if /lidar" in result.message
    assert checks.check_recommended_duration(31.0, {"/lidar": None}) is None


def test_recommended_duration_without_lidar_is_30s() -> None:
    result = checks.check_recommended_duration(10.0, {})
    assert result is not None and "recommended 30s" in result.message


def test_lidar_beams_from_name_reads_common_models() -> None:
    assert checks.lidar_beams_from_name("/lidar/lidar_1/pandar_xt32/pandar_packets") == 32
    assert checks.lidar_beams_from_name("/velodyne_vlp16/points") == 16
    assert checks.lidar_beams_from_name("/ouster/os1_64/points") == 64
    assert checks.lidar_beams_from_name("/rslidar_32/points") == 32
    assert checks.lidar_beams_from_name("/rs16/points") == 16
    assert checks.lidar_beams_from_name("/sensor/lidar/roof/points") is None
    assert checks.lidar_beams_from_name("/lidar_top") is None
