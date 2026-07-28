import json
from pathlib import Path

import pytest

from bagcheck.engine import run_checks
from bagcheck.model import (
    CalibrationType,
    CheckResult,
    CheckStatus,
    IneligibleType,
    ReportStatus,
    TopicRole,
    TopicSummary,
    ValidationReport,
)
from tests.bagcheck.conftest import (
    VENDOR_FIELD_LAYOUTS,
    camera_info_spec,
    compressed_image_spec,
    imu_spec,
    pointcloud2_spec,
    tf_static_spec,
    write_bare_db3,
    write_custom_type_ros1_bag,
    write_mcap,
    write_ros1_bag,
    write_ros2_bag_dir,
    write_unknown_type_ros1_bag,
)

HZ_IMU = 100
IMU_DT_NS = 1_000_000_000 // HZ_IMU
LIDAR_DT_NS = 100_000_000  # 10 Hz
DURATION_S = 8


def _well_formed_bag_specs():
    """A bag with camera+lidar+imu+camera_info+tf_static, enough duration, and real
    rotational motion — should pass cleanly."""
    specs = []
    n_imu = DURATION_S * HZ_IMU
    for i in range(n_imu):
        t_ns = i * IMU_DT_NS
        wz = 0.6 if (i // HZ_IMU) % 2 == 0 else -0.6  # alternate turning, always moving
        specs.append(imu_spec("/imu", t_ns, frame_id="imu_link", wz=wz))

    n_lidar = DURATION_S * 10
    for i in range(n_lidar):
        specs.append(
            pointcloud2_spec("/lidar/points", i * LIDAR_DT_NS, VENDOR_FIELD_LAYOUTS["hesai"], frame_id="lidar_link")
        )

    n_cam = DURATION_S * 10
    for i in range(n_cam):
        specs.append(compressed_image_spec("/cam/front/image/compressed", i * LIDAR_DT_NS, frame_id="cam_front_link"))
    specs.append(camera_info_spec("/cam/front/camera_info", 0, k=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]))

    specs.append(tf_static_spec("/tf_static", 0, "base_link", "imu_link"))
    specs.append(tf_static_spec("/tf_static", 0, "base_link", "lidar_link"))
    specs.append(tf_static_spec("/tf_static", 0, "base_link", "cam_front_link"))
    return specs


@pytest.mark.parametrize(
    "writer",
    [write_ros1_bag, write_ros2_bag_dir, write_bare_db3, write_mcap],
    ids=["ros1_bag", "ros2_db3_dir", "bare_db3", "mcap"],
)
def test_well_formed_bag_passes_on_every_container(tmp_path: Path, writer) -> None:
    path = writer(tmp_path, _well_formed_bag_specs())
    report = run_checks(path, calibration_type=CalibrationType.LIDAR_CAMERA)

    assert report.status is ReportStatus.PASSED, [c.to_dict() for c in report.checks if c.status is not CheckStatus.PASS]
    assert report.exit_code() == 0
    assert CalibrationType.LIDAR_CAMERA in report.eligible_calibration_types
    lidar_topic = next(t for t in report.topics if t.role.value == "lidar")
    assert lidar_topic.vendor_signature == "hesai"
    assert lidar_topic.has_per_point_time


def test_missing_camera_info_produces_warning_status(tmp_path: Path) -> None:
    specs = [s for s in _well_formed_bag_specs() if s.topic != "/cam/front/camera_info"]
    path = write_mcap(tmp_path, specs)
    report = run_checks(path)

    assert report.status is ReportStatus.WARNINGS
    assert report.exit_code() == 1
    camera_info_checks = [c for c in report.checks if c.id == "camera_info_present"]
    assert camera_info_checks and camera_info_checks[0].status is CheckStatus.WARN
    assert "no paired CameraInfo" in camera_info_checks[0].message


def test_custom_type_only_lidar_fails_lidar_camera_request(tmp_path: Path) -> None:
    # Bag has a camera but its only "lidar-shaped" topic is Livox's raw CustomMsg,
    # which bagcheck refuses to classify as a lidar role — coverage must reflect that.
    path = write_custom_type_ros1_bag(tmp_path, "/livox/lidar", "livox_ros_driver/CustomMsg")
    report = run_checks(path, calibration_type=CalibrationType.LIDAR_CAMERA, min_duration_s=0.0)

    assert report.status is ReportStatus.FAILED
    assert report.exit_code() == 2
    schema_warns = [c for c in report.checks if c.id == "custom_message_type"]
    assert schema_warns and "livox_to_pointcloud2" in schema_warns[0].message
    coverage_check = next(c for c in report.checks if c.id == "requested_calibration_coverage")
    assert coverage_check.status is CheckStatus.FAIL
    assert CalibrationType.LIDAR_CAMERA not in report.eligible_calibration_types


def test_insufficient_duration_fails(tmp_path: Path) -> None:
    # Only 2s of data against the default 5s minimum.
    specs = [imu_spec("/imu", i * IMU_DT_NS, wz=0.5) for i in range(2 * HZ_IMU)]
    specs += [pointcloud2_spec("/lidar/points", i * LIDAR_DT_NS, VENDOR_FIELD_LAYOUTS["velodyne"]) for i in range(20)]
    path = write_mcap(tmp_path, specs)
    report = run_checks(path)

    assert report.status is ReportStatus.FAILED
    assert report.exit_code() == 2
    duration_check = next(c for c in report.checks if c.id == "duration")
    assert duration_check.status is CheckStatus.FAIL


def test_no_motion_imu_warns(tmp_path: Path) -> None:
    # Stationary rig for the full duration — passes duration, fails motion excitation.
    specs = [imu_spec("/imu", i * IMU_DT_NS, wz=0.0) for i in range(DURATION_S * HZ_IMU)]
    specs += [
        pointcloud2_spec("/lidar/points", i * LIDAR_DT_NS, VENDOR_FIELD_LAYOUTS["robosense"])
        for i in range(DURATION_S * 10)
    ]
    path = write_mcap(tmp_path, specs)
    report = run_checks(path)

    assert report.status is ReportStatus.WARNINGS
    assert report.exit_code() == 1
    motion_check = next(c for c in report.checks if c.id == "motion_excitation")
    assert motion_check.status is CheckStatus.WARN
    assert "insufficient" in motion_check.message


def test_ros1_bag_imu_decodes_for_motion_excitation(tmp_path: Path) -> None:
    # Regression: a real-world ROS1 `.bag` IMU stream was classified correctly as [imu]
    # from its message-type string (topic inventory), yet motion_excitation reported "no
    # IMU topic found" — decode() was silently misparsing genuine ROS1 wire bytes (which
    # carry a `seq` field on std_msgs/Header) through a ROS2-shaped typestore (which
    # doesn't have one), so zero IMU samples ever reached the check. See the ROS1/ROS2
    # typestore split in bagcheck/readers.py.
    specs = [imu_spec("/imu/data_raw", i * IMU_DT_NS, wz=0.6) for i in range(DURATION_S * HZ_IMU)]
    path = write_ros1_bag(tmp_path, specs)
    report = run_checks(path)

    imu_topic = next(t for t in report.topics if t.topic == "/imu/data_raw")
    assert imu_topic.role is TopicRole.IMU

    motion_check = next(c for c in report.checks if c.id == "motion_excitation")
    assert motion_check.status is CheckStatus.PASS
    assert "no IMU topic found" not in motion_check.message


def test_raw_hesai_lidar_topics_make_multi_lidar_and_lidar_imu_eligible(tmp_path: Path) -> None:
    # Regression for the "raw-packet lanes" fix, modeled on a real bag that motivated
    # it: 5 raw Hesai PandarScan lidars and no sensor_msgs/PointCloud2 anywhere.
    # Before this fix every lidar topic here fell through to [unknown] and
    # multi_lidar/lidar_imu were reported ineligible despite Deepen's calibration
    # engine decoding these packets natively.
    imu_specs = [imu_spec("/imu/data_raw", i * IMU_DT_NS, wz=0.6) for i in range(DURATION_S * HZ_IMU)]
    lidar_topics = {f"/lidar/lidar_{i}/pandar_packets": "pandar_msgs/PandarScan" for i in range(1, 6)}
    path = write_unknown_type_ros1_bag(tmp_path, lidar_topics, extra_specs=imu_specs)
    report = run_checks(path)

    lidar_raw_topics = [t for t in report.topics if t.role is TopicRole.LIDAR_RAW]
    assert len(lidar_raw_topics) == 5
    assert all(t.vendor_signature == "hesai" for t in lidar_raw_topics)

    assert CalibrationType.MULTI_LIDAR in report.eligible_calibration_types
    assert CalibrationType.LIDAR_IMU in report.eligible_calibration_types
    assert CalibrationType.LIDAR_CAMERA not in report.eligible_calibration_types

    raw_packet_checks = [c for c in report.checks if c.id == "lidar_raw_packets"]
    assert len(raw_packet_checks) == 5
    assert all(c.status is CheckStatus.PASS for c in raw_packet_checks)


def test_json_report_round_trips_through_dict() -> None:
    # Smoke-check the JSON shape without needing a full bag — schema stability matters
    # since downstream consumers (scripts, CI, other services) rely on this exact shape.
    report = ValidationReport(
        status=ReportStatus.WARNINGS,
        container_format="ros2_mcap",
        topics=[TopicSummary("/lidar", "sensor_msgs/msg/PointCloud2", TopicRole.LIDAR, 10, hz=10.0)],
        checks=[CheckResult("duration", CheckStatus.PASS, "bag duration 10.0s.")],
        eligible_calibration_types=[CalibrationType.LIDAR_CAMERA],
        ineligible_calibration_types=[IneligibleType(CalibrationType.MULTI_LIDAR, "needs >=2 lidar topics")],
    )
    encoded = json.dumps(report.to_dict())
    decoded = json.loads(encoded)
    assert decoded["schema_version"] == "1.1"
    assert decoded["status"] == "warnings"
    assert decoded["topics"][0]["role"] == "lidar"
    assert decoded["eligible_calibration_types"] == ["lidar_camera"]
