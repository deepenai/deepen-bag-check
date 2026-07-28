from bagcheck.coverage import check_coverage
from bagcheck.model import CalibrationType, TopicRole, TopicSummary


def _topic(
    role: TopicRole, topic: str = "/t", msgtype: str = "x", vendor_signature: str | None = None
) -> TopicSummary:
    return TopicSummary(topic=topic, msgtype=msgtype, role=role, message_count=10, vendor_signature=vendor_signature)


def test_lidar_camera_needs_both() -> None:
    only_camera = [_topic(TopicRole.CAMERA_RAW)]
    result = check_coverage(only_camera, CalibrationType.LIDAR_CAMERA)
    assert not result.eligible
    assert "lidar" in result.reason

    both = [_topic(TopicRole.CAMERA_RAW, "/cam"), _topic(TopicRole.LIDAR, "/lidar")]
    result = check_coverage(both, CalibrationType.LIDAR_CAMERA)
    assert result.eligible


def test_multi_lidar_needs_two_lidar_topics() -> None:
    one = [_topic(TopicRole.LIDAR, "/lidar1")]
    assert not check_coverage(one, CalibrationType.MULTI_LIDAR).eligible

    two = [_topic(TopicRole.LIDAR, "/lidar1"), _topic(TopicRole.LIDAR, "/lidar2")]
    assert check_coverage(two, CalibrationType.MULTI_LIDAR).eligible


def test_lidar_vehicle_needs_only_one_lidar() -> None:
    none = []
    assert not check_coverage(none, CalibrationType.LIDAR_VEHICLE).eligible
    one = [_topic(TopicRole.LIDAR)]
    assert check_coverage(one, CalibrationType.LIDAR_VEHICLE).eligible


def test_lidar_imu_needs_lidar_and_imu_and_warns_without_gnss() -> None:
    lidar_only = [_topic(TopicRole.LIDAR)]
    assert not check_coverage(lidar_only, CalibrationType.LIDAR_IMU).eligible

    lidar_and_imu = [_topic(TopicRole.LIDAR, "/lidar"), _topic(TopicRole.IMU, "/imu")]
    result = check_coverage(lidar_and_imu, CalibrationType.LIDAR_IMU)
    assert result.eligible
    assert result.warning is not None
    assert "GNSS" in result.warning

    with_gnss = [*lidar_and_imu, _topic(TopicRole.UNKNOWN, "/gnss", msgtype="sensor_msgs/msg/NavSatFix")]
    result = check_coverage(with_gnss, CalibrationType.LIDAR_IMU)
    assert result.eligible
    assert result.warning is None


def test_hesai_raw_lidar_counts_toward_coverage() -> None:
    # Raw Hesai PandarScan topics are engine-ingestible, so they count toward lidar
    # coverage just like sensor_msgs/PointCloud2.
    two_hesai_raw = [
        _topic(TopicRole.LIDAR_RAW, "/lidar1", vendor_signature="hesai"),
        _topic(TopicRole.LIDAR_RAW, "/lidar2", vendor_signature="hesai"),
    ]
    assert check_coverage(two_hesai_raw, CalibrationType.MULTI_LIDAR).eligible


def test_pointcloud2_lidar_and_hesai_raw_lidar_both_count() -> None:
    mixed = [_topic(TopicRole.LIDAR, "/lidar1"), _topic(TopicRole.LIDAR_RAW, "/lidar2", vendor_signature="hesai")]
    assert check_coverage(mixed, CalibrationType.MULTI_LIDAR).eligible


def test_non_hesai_raw_lidar_does_not_count_toward_coverage() -> None:
    # Velodyne/Ouster raw packets are recognized (not [unknown]) but nothing decodes
    # them, so they must not silently satisfy the minimum lidar coverage.
    two_velodyne_raw = [
        _topic(TopicRole.LIDAR_RAW, "/lidar1", vendor_signature="velodyne"),
        _topic(TopicRole.LIDAR_RAW, "/lidar2", vendor_signature="velodyne"),
    ]
    result = check_coverage(two_velodyne_raw, CalibrationType.MULTI_LIDAR)
    assert not result.eligible
