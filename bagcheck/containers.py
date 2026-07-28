"""Container-format detection for rosbag-family files.

Covers the three containers this tool supports — ROS1 `.bag`, ROS2 `.db3`, ROS2
`.mcap` — and nothing else. Detection reads magic bytes / bag-directory metadata
rather than trusting the file extension, so a renamed or extension-less file still
gets a clear, specific error instead of failing deep inside a reader.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import yaml

ROS1_BAG_MAGIC = b"#ROSBAG V2.0\n"
SQLITE_MAGIC = b"SQLite format 3\x00"
MCAP_MAGIC = b"\x89MCAP0\r\n"


class ContainerFormat(str, Enum):
    ROS1_BAG = "ros1_bag"
    ROS2_DB3 = "ros2_db3"
    ROS2_MCAP = "ros2_mcap"


class BagCheckError(Exception):
    """Base class for all deepen-bag-check errors."""


class UnsupportedContainerError(BagCheckError):
    """Raised when a path isn't a recognizable or readable v1 container."""


@dataclass(frozen=True)
class DetectedContainer:
    format: ContainerFormat
    path: Path
    # True for a standalone .db3/.mcap file with no sibling metadata.yaml — these
    # need the bare-file reader path instead of the bag-directory one.
    is_bare_file: bool


def detect_container(path: Path) -> DetectedContainer:
    """Identify the container format of `path`, or raise UnsupportedContainerError."""
    path = Path(path)
    if not path.exists():
        raise UnsupportedContainerError(f"{path}: no such file or directory")
    if path.is_dir():
        return _detect_bag_directory(path)
    return _detect_bag_file(path)


def _detect_bag_directory(path: Path) -> DetectedContainer:
    metadata_path = path / "metadata.yaml"
    if not metadata_path.exists():
        raise UnsupportedContainerError(
            f"{path}: directory has no metadata.yaml — not a recognizable ROS2 bag. "
            "If you have a standalone .db3 or .mcap file (no bag directory), point "
            "deepen-bag-check at that file directly."
        )
    try:
        metadata = yaml.safe_load(metadata_path.read_text())
    except yaml.YAMLError as exc:
        raise UnsupportedContainerError(f"{path}/metadata.yaml: not valid YAML ({exc})") from exc

    try:
        storage_id = metadata["rosbag2_bagfile_information"]["storage_identifier"]
    except (KeyError, TypeError) as exc:
        raise UnsupportedContainerError(
            f"{path}/metadata.yaml: missing rosbag2_bagfile_information.storage_identifier"
        ) from exc

    if storage_id == "sqlite3":
        return DetectedContainer(ContainerFormat.ROS2_DB3, path, is_bare_file=False)
    if storage_id == "mcap":
        return DetectedContainer(ContainerFormat.ROS2_MCAP, path, is_bare_file=False)
    raise UnsupportedContainerError(
        f"{path}/metadata.yaml: unsupported storage_identifier {storage_id!r} — "
        "deepen-bag-check reads 'sqlite3' and 'mcap' rosbag2 storage plugins only"
    )


def _detect_bag_file(path: Path) -> DetectedContainer:
    try:
        with path.open("rb") as f:
            header = f.read(16)
    except OSError as exc:
        raise UnsupportedContainerError(f"{path}: cannot read file ({exc})") from exc

    if header.startswith(ROS1_BAG_MAGIC):
        return DetectedContainer(ContainerFormat.ROS1_BAG, path, is_bare_file=True)
    if header.startswith(SQLITE_MAGIC):
        return DetectedContainer(ContainerFormat.ROS2_DB3, path, is_bare_file=True)
    if header.startswith(MCAP_MAGIC):
        return DetectedContainer(ContainerFormat.ROS2_MCAP, path, is_bare_file=True)

    raise UnsupportedContainerError(
        f"{path}: unrecognized container — checked magic bytes, not just the extension, "
        "and this is not a ROS1 .bag, ROS2 .db3 (sqlite3), or ROS2 .mcap file. "
        "deepen-bag-check v1 supports these three containers only (not raw non-ROS MCAP "
        "or PX4 .ulg — see the README for conversion paths)."
    )
