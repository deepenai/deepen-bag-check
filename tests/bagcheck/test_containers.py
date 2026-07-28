from pathlib import Path

import pytest

from bagcheck.containers import ContainerFormat, UnsupportedContainerError, detect_container
from tests.bagcheck.conftest import (
    imu_spec,
    write_bare_db3,
    write_mcap,
    write_ros1_bag,
    write_ros2_bag_dir,
)


def test_detects_ros1_bag(tmp_path: Path) -> None:
    path = write_ros1_bag(tmp_path, [imu_spec("/imu", 0)])
    detected = detect_container(path)
    assert detected.format is ContainerFormat.ROS1_BAG
    assert detected.is_bare_file


def test_detects_ros2_bag_directory_sqlite(tmp_path: Path) -> None:
    path = write_ros2_bag_dir(tmp_path, [imu_spec("/imu", 0)])
    detected = detect_container(path)
    assert detected.format is ContainerFormat.ROS2_DB3
    assert not detected.is_bare_file


def test_detects_bare_db3(tmp_path: Path) -> None:
    path = write_bare_db3(tmp_path, [imu_spec("/imu", 0)])
    detected = detect_container(path)
    assert detected.format is ContainerFormat.ROS2_DB3
    assert detected.is_bare_file


def test_detects_bare_mcap(tmp_path: Path) -> None:
    path = write_mcap(tmp_path, [imu_spec("/imu", 0)])
    detected = detect_container(path)
    assert detected.format is ContainerFormat.ROS2_MCAP
    assert detected.is_bare_file


def test_rejects_missing_path(tmp_path: Path) -> None:
    with pytest.raises(UnsupportedContainerError, match="no such file"):
        detect_container(tmp_path / "does_not_exist.bag")


def test_rejects_corrupt_file(tmp_path: Path) -> None:
    junk = tmp_path / "junk.bag"
    junk.write_bytes(b"not a real bag file at all")
    with pytest.raises(UnsupportedContainerError, match="unrecognized container"):
        detect_container(junk)


def test_rejects_directory_without_metadata(tmp_path: Path) -> None:
    empty_dir = tmp_path / "not_a_bag"
    empty_dir.mkdir()
    with pytest.raises(UnsupportedContainerError, match="metadata.yaml"):
        detect_container(empty_dir)
