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


def test_rejects_empty_file_with_its_own_message(tmp_path: Path) -> None:
    """B7: a 0-byte file and a corrupt-but-nonempty file must not read as the same
    problem — "empty" and "not a recognizable container" call for different fixes."""
    empty = tmp_path / "empty.bag"
    empty.write_bytes(b"")
    with pytest.raises(UnsupportedContainerError, match="file is empty") as excinfo:
        detect_container(empty)
    assert "unrecognized container" not in str(excinfo.value)


def test_rejects_directory_without_metadata(tmp_path: Path) -> None:
    empty_dir = tmp_path / "not_a_bag"
    empty_dir.mkdir()
    with pytest.raises(UnsupportedContainerError, match="metadata.yaml"):
        detect_container(empty_dir)
