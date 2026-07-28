"""Unified message reader over the three v1 containers.

Every reader implementation yields the same `RawMessage` shape and exposes the same
`decode(msgtype, rawdata)` call, so the rest of the package (classify/checks/engine)
never has to know whether it's looking at a ROS1 bag, a ROS2 sqlite3/mcap bag
directory, or a bare standalone `.db3`/`.mcap` file.

This package uses `rosbags` for ROS1 `.bag` and ROS2 `.db3`, and `mcap` +
`mcap-ros2-support` for `.mcap`. `rosbags.rosbag2.Reader` can also open an mcap-storage
bag *directory* (it ships an mcap storage plugin), so that combination is handled
there rather than duplicated — the bare/standalone `.mcap` file case is the one that
needs the `mcap` package directly.
"""

from __future__ import annotations

import sqlite3
from abc import ABC, abstractmethod
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory as _Ros2DecoderFactory
from rosbags.rosbag1 import Reader as _Ros1Reader
from rosbags.rosbag2 import Reader as _Ros2Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from bagcheck.containers import ContainerFormat, DetectedContainer, UnsupportedContainerError

# ROS2 `.db3`/`.mcap` containers are read as CDR, whose wire shape matches `Stores.LATEST`
# (a ROS2 store) exactly, so one shared typestore is correct there.
TYPESTORE = get_typestore(Stores.LATEST)

# ROS1 `.bag` containers are NOT CDR: real ROS1 wire format carries a `seq` field on every
# `std_msgs/Header` that ROS2 dropped. Deserializing ROS1 bytes with a ROS2-shaped store
# (as a single shared `TYPESTORE` would) silently misaligns every field after the header in
# any message that embeds one (Imu, PointCloud2, Image, CameraInfo, tf) — decode() then
# raises and is swallowed, so a topic classified correctly from its message-type string
# still produces zero decoded samples downstream (e.g. motion_excitation reporting "no IMU
# topic found" for a topic the topic inventory correctly lists as [imu]). `Stores.ROS1_NOETIC`
# matches the real ROS1 layout; `tf2_msgs/msg/TFMessage` is the one common type missing from
# its bundled catalog, so it's registered by hand from its single-field definition.
_ROS1_TYPESTORE = get_typestore(Stores.ROS1_NOETIC)
_ROS1_TYPESTORE.register(
    get_types_from_msg("geometry_msgs/TransformStamped[] transforms", "tf2_msgs/msg/TFMessage")
)


@dataclass(frozen=True)
class RawMessage:
    topic: str
    msgtype: str
    timestamp_ns: int
    rawdata: bytes


@dataclass(frozen=True)
class ConnectionSummary:
    topic: str
    msgtype: str
    message_count: int


class BagReader(ABC):
    """Context-managed, format-agnostic view over one bag/mcap file."""

    container_format: ContainerFormat

    @abstractmethod
    def __enter__(self) -> BagReader: ...

    @abstractmethod
    def __exit__(self, *exc_info: object) -> None: ...

    @abstractmethod
    def connections(self) -> list[ConnectionSummary]:
        """Per-topic type + message count, from the container's own index (no scan)."""

    @abstractmethod
    def messages(self) -> Iterator[RawMessage]:
        """Every message, raw (undecoded) — cheap even on large bags."""

    @abstractmethod
    def decode(self, msgtype: str, rawdata: bytes) -> Any | None:
        """Deserialize one message. Returns None for a type this reader can't decode
        (unknown/custom types — the schema/type check flags those separately)."""

    @property
    @abstractmethod
    def start_ns(self) -> int | None: ...

    @property
    @abstractmethod
    def end_ns(self) -> int | None: ...


class _RosbagsReader(BagReader):
    """Shared implementation for ROS1 `.bag` and ROS2 bag-directory (sqlite3 or mcap
    storage plugin) — both go through the `rosbags` package's own reader/typestore."""

    def __init__(self, path: Path, container_format: ContainerFormat, reader: Any, deserialize: Any):
        self.container_format = container_format
        self._reader = reader
        self._deserialize = deserialize

    def __enter__(self) -> _RosbagsReader:
        self._reader.__enter__()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self._reader.__exit__(*exc_info)

    def connections(self) -> list[ConnectionSummary]:
        return [
            ConnectionSummary(topic=topic, msgtype=info.msgtype, message_count=info.msgcount)
            for topic, info in self._reader.topics.items()
        ]

    def messages(self) -> Iterator[RawMessage]:
        for connection, timestamp, rawdata in self._reader.messages():
            yield RawMessage(
                topic=connection.topic,
                msgtype=connection.msgtype,
                timestamp_ns=timestamp,
                rawdata=bytes(rawdata),
            )

    def decode(self, msgtype: str, rawdata: bytes) -> Any | None:
        try:
            return self._deserialize(rawdata, msgtype)
        except Exception:
            return None

    @property
    def start_ns(self) -> int | None:
        return self._reader.start_time

    @property
    def end_ns(self) -> int | None:
        return self._reader.end_time


class _BareDb3Reader(BagReader):
    """Standalone ROS2 `.db3` file with no sibling `metadata.yaml`. Reads the
    rosbag2 sqlite3 schema directly —
    `topics(id, name, type, ...)` / `messages(id, topic_id, timestamp, data)` — since
    `rosbags.rosbag2.Reader` expects the metadata-described bag-directory layout."""

    def __init__(self, path: Path):
        self.container_format = ContainerFormat.ROS2_DB3
        self._path = path
        self._conn: sqlite3.Connection | None = None
        self._topic_by_id: dict[int, tuple[str, str]] = {}

    def __enter__(self) -> _BareDb3Reader:
        self._conn = sqlite3.connect(str(self._path))
        rows = self._conn.execute("SELECT id, name, type FROM topics").fetchall()
        self._topic_by_id = {row[0]: (row[1], row[2]) for row in rows}
        return self

    def __exit__(self, *exc_info: object) -> None:
        if self._conn is not None:
            self._conn.close()

    def connections(self) -> list[ConnectionSummary]:
        assert self._conn is not None
        counts = dict(
            self._conn.execute("SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id").fetchall()
        )
        return [
            ConnectionSummary(topic=name, msgtype=msgtype, message_count=counts.get(topic_id, 0))
            for topic_id, (name, msgtype) in self._topic_by_id.items()
        ]

    def messages(self) -> Iterator[RawMessage]:
        assert self._conn is not None
        cursor = self._conn.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
        )
        for topic_id, timestamp, data in cursor:
            name, msgtype = self._topic_by_id[topic_id]
            yield RawMessage(topic=name, msgtype=msgtype, timestamp_ns=timestamp, rawdata=bytes(data))

    def decode(self, msgtype: str, rawdata: bytes) -> Any | None:
        try:
            return TYPESTORE.deserialize_cdr(rawdata, msgtype)
        except Exception:
            return None

    @property
    def start_ns(self) -> int | None:
        assert self._conn is not None
        row = self._conn.execute("SELECT MIN(timestamp) FROM messages").fetchone()
        return row[0] if row and row[0] is not None else None

    @property
    def end_ns(self) -> int | None:
        assert self._conn is not None
        row = self._conn.execute("SELECT MAX(timestamp) FROM messages").fetchone()
        return row[0] if row and row[0] is not None else None


class _McapReader(BagReader):
    """Standalone `.mcap` file, read directly with `mcap` + `mcap-ros2-support`
    rather than via a rosbag2 bag directory."""

    def __init__(self, path: Path):
        self.container_format = ContainerFormat.ROS2_MCAP
        self._path = path
        self._file: Any = None
        self._reader: Any = None
        self._summary: Any = None
        self._schema_by_name: dict[str, Any] = {}
        self._decoders: dict[str, Any] = {}
        self._decoder_factory = _Ros2DecoderFactory()

    def __enter__(self) -> _McapReader:
        self._file = self._path.open("rb")
        self._reader = make_reader(self._file)
        summary = self._reader.get_summary()
        if summary is None:
            self._file.close()
            raise UnsupportedContainerError(
                f"{self._path}: mcap file has no summary index — likely truncated/corrupt "
                "(deepen-bag-check requires an indexed, non-streaming mcap file)"
            )
        self._summary = summary
        self._schema_by_name = {schema.name: schema for schema in summary.schemas.values()}
        return self

    def __exit__(self, *exc_info: object) -> None:
        if self._file is not None:
            self._file.close()

    def connections(self) -> list[ConnectionSummary]:
        counts = self._summary.statistics.channel_message_counts if self._summary.statistics else {}
        result = []
        for channel in self._summary.channels.values():
            schema = self._summary.schemas.get(channel.schema_id)
            msgtype = schema.name if schema else channel.message_encoding
            result.append(
                ConnectionSummary(
                    topic=channel.topic, msgtype=msgtype, message_count=counts.get(channel.id, 0)
                )
            )
        return result

    def messages(self) -> Iterator[RawMessage]:
        for schema, channel, message in self._reader.iter_messages():
            msgtype = schema.name if schema else channel.message_encoding
            yield RawMessage(
                topic=channel.topic,
                msgtype=msgtype,
                timestamp_ns=message.log_time,
                rawdata=message.data,
            )

    def decode(self, msgtype: str, rawdata: bytes) -> Any | None:
        if msgtype not in self._decoders:
            schema = self._schema_by_name.get(msgtype)
            try:
                self._decoders[msgtype] = (
                    self._decoder_factory.decoder_for("cdr", schema) if schema else None
                )
            except Exception:
                self._decoders[msgtype] = None
        decoder = self._decoders[msgtype]
        if decoder is None:
            return None
        try:
            return decoder(rawdata)
        except Exception:
            return None

    @property
    def start_ns(self) -> int | None:
        stats = self._summary.statistics
        return stats.message_start_time if stats else None

    @property
    def end_ns(self) -> int | None:
        stats = self._summary.statistics
        return stats.message_end_time if stats else None


def open_reader(detected: DetectedContainer) -> BagReader:
    """Build the right reader for a `detect_container()` result. Does not open the
    file yet — use as `with open_reader(detected) as reader:`."""
    if detected.format is ContainerFormat.ROS1_BAG:
        return _RosbagsReader(
            detected.path, detected.format, _Ros1Reader(detected.path), _ROS1_TYPESTORE.deserialize_ros1
        )
    if detected.format is ContainerFormat.ROS2_DB3:
        if detected.is_bare_file:
            return _BareDb3Reader(detected.path)
        return _RosbagsReader(
            detected.path, detected.format, _Ros2Reader(detected.path), TYPESTORE.deserialize_cdr
        )
    if detected.format is ContainerFormat.ROS2_MCAP:
        if detected.is_bare_file:
            return _McapReader(detected.path)
        return _RosbagsReader(
            detected.path, detected.format, _Ros2Reader(detected.path), TYPESTORE.deserialize_cdr
        )
    raise UnsupportedContainerError(f"no reader for container format {detected.format!r}")
