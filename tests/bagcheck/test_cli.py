import json
from pathlib import Path

from bagcheck.cli import main
from tests.bagcheck.conftest import imu_spec, write_ros1_bag


def test_cli_json_output_and_exit_code(tmp_path: Path, capsys) -> None:
    # Short/no-motion bag -> guaranteed non-zero exit without depending on other checks.
    path = write_ros1_bag(tmp_path, [imu_spec("/imu", i * 10_000_000, wz=0.0) for i in range(50)])
    exit_code = main([str(path), "--json"])
    captured = capsys.readouterr()
    report = json.loads(captured.out)

    expected_exit = {"passed": 0, "warnings": 1, "failed": 2}[report["status"]]
    assert exit_code == expected_exit
    assert report["schema_version"] == "1.2"
    assert report["container_format"] == "ros1_bag"


def test_cli_human_output_contains_status_line(tmp_path: Path, capsys) -> None:
    path = write_ros1_bag(tmp_path, [imu_spec("/imu", i * 10_000_000, wz=0.0) for i in range(50)])
    exit_code = main([str(path)])
    captured = capsys.readouterr()

    assert "deepen-bag-check" in captured.out
    assert "status:" in captured.out
    assert exit_code in (0, 1, 2)


def test_cli_reports_clear_error_for_corrupt_file(tmp_path: Path, capsys) -> None:
    junk = tmp_path / "junk.bag"
    junk.write_bytes(b"not a bag")
    exit_code = main([str(junk)])
    captured = capsys.readouterr()

    assert exit_code == 2
    assert "unrecognized container" in captured.err


def test_cli_for_flag_gates_on_calibration_type(tmp_path: Path, capsys) -> None:
    path = write_ros1_bag(tmp_path, [imu_spec("/imu", i * 10_000_000, wz=0.0) for i in range(50)])
    exit_code = main([str(path), "--for", "lidar-camera", "--json"])
    captured = capsys.readouterr()
    report = json.loads(captured.out)

    assert report["requested_calibration_type"] == "lidar_camera"
    assert exit_code == 2  # no camera or lidar topic at all
