"""Synthetic bag builders shared across the bagcheck test suite.

No network, no committed fixtures — every bag is built in `tmp_path` at test time,
using `rosbags` (for `.bag`/`.db3`) and `mcap`/`mcap-ros2-support` (for `.mcap`), the
same libraries `bagcheck` itself reads with. A `MessageSpec` carries both a rosbags
typestore object (for the `.bag`/`.db3` writers) and an equivalent plain-dict message
(for the mcap writer), so one call site can describe a message once and write it to
any container.
"""

from __future__ import annotations

import dataclasses
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.rosbag1 import Writer as Ros1Writer
from rosbags.rosbag2 import Writer as Ros2Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

TS = get_typestore(Stores.LATEST)

# `write_ros1_bag` needs genuinely ROS1-shaped bytes (real ROS1 wire format keeps a `seq`
# field on `std_msgs/Header` that ROS2 dropped — see bagcheck/readers.py's typestore split),
# not the ROS2-shaped bytes `TS.serialize_ros1` would produce. `TS1_ROS1` + `_to_ros1` below
# rebuild each `TS`-typed spec object into its real-ROS1 equivalent at write time so every
# `.bag` fixture round-trips through the same reader path production bags do.
TS1_ROS1 = get_typestore(Stores.ROS1_NOETIC)
TS1_ROS1.register(
    get_types_from_msg("geometry_msgs/TransformStamped[] transforms", "tf2_msgs/msg/TFMessage")
)


# ROS2 lower-cased CameraInfo's distortion/intrinsics fields; ROS1 kept them upper-case.
_ROS1_FIELD_RENAMES: dict[str, dict[str, str]] = {
    "sensor_msgs/msg/CameraInfo": {"d": "D", "k": "K", "r": "R", "p": "P"},
}


def _to_ros1(obj: Any) -> Any:
    """Rebuild a `Stores.LATEST`-typed message object as its `Stores.ROS1_NOETIC` equivalent,
    filling in `seq=0` on every `std_msgs/Header` (the field ROS1 carries and ROS2 dropped)
    and renaming the handful of fields ROS2 spells differently (CameraInfo's D/K/R/P)."""
    if not dataclasses.is_dataclass(obj):
        return obj
    msgtype = obj.__msgtype__
    cls = TS1_ROS1.types[msgtype]
    renames = _ROS1_FIELD_RENAMES.get(msgtype, {})
    kwargs = {}
    for f in dataclasses.fields(obj):
        if f.name == "__msgtype__":
            continue
        value = getattr(obj, f.name)
        value = [_to_ros1(v) for v in value] if isinstance(value, list) else _to_ros1(value)
        kwargs[renames.get(f.name, f.name)] = value
    if msgtype == "std_msgs/msg/Header":
        kwargs["seq"] = 0
    return cls(**kwargs)

Header = TS.types["std_msgs/msg/Header"]
Time = TS.types["builtin_interfaces/msg/Time"]
Imu = TS.types["sensor_msgs/msg/Imu"]
Vector3 = TS.types["geometry_msgs/msg/Vector3"]
Quaternion = TS.types["geometry_msgs/msg/Quaternion"]
PointCloud2 = TS.types["sensor_msgs/msg/PointCloud2"]
PointField = TS.types["sensor_msgs/msg/PointField"]
CameraInfo = TS.types["sensor_msgs/msg/CameraInfo"]
RegionOfInterest = TS.types["sensor_msgs/msg/RegionOfInterest"]
Image = TS.types["sensor_msgs/msg/Image"]
CompressedImage = TS.types["sensor_msgs/msg/CompressedImage"]
TFMessage = TS.types["tf2_msgs/msg/TFMessage"]
TransformStamped = TS.types["geometry_msgs/msg/TransformStamped"]
Transform = TS.types["geometry_msgs/msg/Transform"]

# sensor_msgs/PointField datatype constants.
FLOAT32, FLOAT64, UINT8, UINT16, UINT32 = 7, 8, 2, 4, 6
_NUMPY_DTYPE_BY_PF = {FLOAT32: "<f4", FLOAT64: "<f8", UINT8: "u1", UINT16: "<u2", UINT32: "<u4"}


@dataclass
class MessageSpec:
    topic: str
    msgtype: str
    timestamp_ns: int
    obj: Any  # rosbags typestore object, for .bag / .db3
    mcap_dict: Any  # plain dict, for .mcap


def _header(ts_ns: int, frame_id: str) -> Any:
    return Header(stamp=Time(sec=ts_ns // 10**9, nanosec=ts_ns % 10**9), frame_id=frame_id)


def _header_dict(ts_ns: int, frame_id: str) -> dict:
    return {"stamp": {"sec": ts_ns // 10**9, "nanosec": ts_ns % 10**9}, "frame_id": frame_id}


def imu_spec(topic: str, ts_ns: int, frame_id: str = "imu_link", wz: float = 0.0) -> MessageSpec:
    obj = Imu(
        header=_header(ts_ns, frame_id),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=np.zeros(9),
        angular_velocity=Vector3(x=0.0, y=0.0, z=wz),
        angular_velocity_covariance=np.zeros(9),
        linear_acceleration=Vector3(x=0.0, y=0.0, z=9.8),
        linear_acceleration_covariance=np.zeros(9),
    )
    mcap_dict = {
        "header": _header_dict(ts_ns, frame_id),
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "orientation_covariance": [0.0] * 9,
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": wz},
        "angular_velocity_covariance": [0.0] * 9,
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.8},
        "linear_acceleration_covariance": [0.0] * 9,
    }
    return MessageSpec(topic, Imu.__msgtype__, ts_ns, obj, mcap_dict)


# Per-vendor PointCloud2 field layouts, dtypes confirmed against each vendor's own ROS
# driver source (see bagcheck/pointcloud.py docstring for citations).
VENDOR_FIELD_LAYOUTS: dict[str, list[tuple[str, int]]] = {
    "velodyne": [("x", FLOAT32), ("y", FLOAT32), ("z", FLOAT32), ("intensity", FLOAT32), ("ring", UINT16), ("time", FLOAT32)],
    "ouster": [("x", FLOAT32), ("y", FLOAT32), ("z", FLOAT32), ("intensity", FLOAT32), ("t", UINT32), ("reflectivity", UINT16), ("ring", UINT16)],
    "hesai": [("x", FLOAT32), ("y", FLOAT32), ("z", FLOAT32), ("intensity", FLOAT32), ("ring", UINT16), ("timestamp", FLOAT64)],
    "robosense": [("x", FLOAT32), ("y", FLOAT32), ("z", FLOAT32), ("intensity", UINT8), ("ring", UINT16), ("timestamp", FLOAT64)],
}


def pointcloud2_spec(
    topic: str,
    ts_ns: int,
    field_layout: list[tuple[str, int]],
    n_points: int = 5,
    frame_id: str = "lidar_link",
) -> MessageSpec:
    """Build a PointCloud2 with an arbitrary (name, PointField-datatype) field layout,
    tightly packed in the order given — this is how `normalize_fields()` is exercised
    against each vendor's real field names/dtypes."""
    np_dtype = np.dtype([(name, _NUMPY_DTYPE_BY_PF[dt]) for name, dt in field_layout])
    points = np.zeros(n_points, dtype=np_dtype)
    if "x" in np_dtype.names:
        points["x"] = np.arange(n_points, dtype=np_dtype["x"])
    data = points.tobytes()

    fields = []
    offset = 0
    for name, dt in field_layout:
        fields.append(PointField(name=name, offset=offset, datatype=dt, count=1))
        offset += np.dtype(_NUMPY_DTYPE_BY_PF[dt]).itemsize

    obj = PointCloud2(
        header=_header(ts_ns, frame_id),
        height=1,
        width=n_points,
        fields=fields,
        is_bigendian=False,
        point_step=np_dtype.itemsize,
        row_step=np_dtype.itemsize * n_points,
        data=np.frombuffer(data, dtype=np.uint8),
        is_dense=True,
    )
    mcap_dict = {
        "header": _header_dict(ts_ns, frame_id),
        "height": 1,
        "width": n_points,
        "fields": [{"name": n, "offset": o.offset, "datatype": o.datatype, "count": 1} for n, o in zip(
            [f[0] for f in field_layout], fields, strict=True
        )],
        "is_bigendian": False,
        "point_step": np_dtype.itemsize,
        "row_step": np_dtype.itemsize * n_points,
        "data": list(data),
        "is_dense": True,
    }
    return MessageSpec(topic, PointCloud2.__msgtype__, ts_ns, obj, mcap_dict)


def camera_info_spec(topic: str, ts_ns: int, k: list[float], frame_id: str = "cam_link") -> MessageSpec:
    obj = CameraInfo(
        header=_header(ts_ns, frame_id),
        height=480,
        width=640,
        distortion_model="plumb_bob",
        d=np.zeros(5),
        k=np.array(k, dtype=np.float64),
        r=np.eye(3).flatten(),
        p=np.zeros(12),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )
    mcap_dict = {
        "header": _header_dict(ts_ns, frame_id),
        "height": 480,
        "width": 640,
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": list(k),
        "r": list(np.eye(3).flatten()),
        "p": [0.0] * 12,
        "binning_x": 0,
        "binning_y": 0,
        "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
    }
    return MessageSpec(topic, CameraInfo.__msgtype__, ts_ns, obj, mcap_dict)


def compressed_image_spec(topic: str, ts_ns: int, fmt: str = "jpeg", frame_id: str = "cam_link") -> MessageSpec:
    payload = b"\xff\xd8\xff\xd9"  # minimal (fake) JPEG-ish payload, content is never decoded
    obj = CompressedImage(header=_header(ts_ns, frame_id), format=fmt, data=np.frombuffer(payload, dtype=np.uint8))
    mcap_dict = {"header": _header_dict(ts_ns, frame_id), "format": fmt, "data": list(payload)}
    return MessageSpec(topic, CompressedImage.__msgtype__, ts_ns, obj, mcap_dict)


def image_spec(topic: str, ts_ns: int, encoding: str = "bgr8", frame_id: str = "cam_link") -> MessageSpec:
    payload = bytes(12)
    obj = Image(header=_header(ts_ns, frame_id), height=1, width=4, encoding=encoding, is_bigendian=0, step=12, data=np.frombuffer(payload, dtype=np.uint8))
    mcap_dict = {"header": _header_dict(ts_ns, frame_id), "height": 1, "width": 4, "encoding": encoding, "is_bigendian": 0, "step": 12, "data": list(payload)}
    return MessageSpec(topic, Image.__msgtype__, ts_ns, obj, mcap_dict)


def tf_static_spec(topic: str, ts_ns: int, parent: str, child: str) -> MessageSpec:
    transform = Transform(translation=Vector3(x=0.0, y=0.0, z=0.0), rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    obj = TFMessage(transforms=[TransformStamped(header=_header(ts_ns, parent), child_frame_id=child, transform=transform)])
    mcap_dict = {
        "transforms": [
            {
                "header": _header_dict(ts_ns, parent),
                "child_frame_id": child,
                "transform": {"translation": {"x": 0.0, "y": 0.0, "z": 0.0}, "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
            }
        ]
    }
    return MessageSpec(topic, TFMessage.__msgtype__, ts_ns, obj, mcap_dict)


def write_ros1_bag(tmp_path: Path, specs: list[MessageSpec], name: str = "bag.bag") -> Path:
    path = tmp_path / name
    with Ros1Writer(path) as writer:
        connections = {}
        for spec in specs:
            if spec.topic not in connections:
                connections[spec.topic] = writer.add_connection(spec.topic, spec.msgtype, typestore=TS1_ROS1)
            conn = connections[spec.topic]
            raw = TS1_ROS1.serialize_ros1(_to_ros1(spec.obj), spec.msgtype) if spec.obj is not None else b"\x00"
            writer.write(conn, spec.timestamp_ns, bytes(raw))
    return path


def write_custom_type_ros1_bag(tmp_path: Path, topic: str, msgtype: str, name: str = "bag.bag") -> Path:
    """A minimal bag with one connection of a message type unknown to any typestore —
    exercises schema/custom-type flagging. `rosbags` requires the
    3-part `pkg/msg/Msg` spelling on write even for a ROS1 bag; `bagcheck.classify`
    normalizes both spellings before matching, so this still round-trips correctly."""
    return write_unknown_type_ros1_bag(tmp_path, {topic: msgtype}, name=name)


def write_unknown_type_ros1_bag(
    tmp_path: Path,
    topic_msgtypes: dict[str, str],
    extra_specs: list[MessageSpec] | None = None,
    name: str = "bag.bag",
) -> Path:
    """A bag with one or more connections of a message type unknown to any typestore —
    e.g. real vendor raw-packet lidar types like `pandar_msgs/PandarScan`, which no
    typestore decodes — plus optional normal (`MessageSpec`-built) topics like IMU.
    Exercises schema/custom-type flagging and, for recognized raw-packet lidar types,
    the `TopicRole.LIDAR_RAW` classification and coverage path, all without needing a
    real vendor SDK to decode anything."""
    path = tmp_path / name
    with Ros1Writer(path) as writer:
        for topic, msgtype in topic_msgtypes.items():
            pkg, _, name_part = msgtype.partition("/")
            three_part = f"{pkg}/msg/{name_part}"
            conn = writer.add_connection(topic, three_part, msgdef="uint8 dummy\n", md5sum="0" * 32)
            writer.write(conn, 0, b"\x00")
        connections: dict[str, Any] = {}
        for spec in extra_specs or []:
            if spec.topic not in connections:
                connections[spec.topic] = writer.add_connection(spec.topic, spec.msgtype, typestore=TS1_ROS1)
            conn = connections[spec.topic]
            raw = TS1_ROS1.serialize_ros1(_to_ros1(spec.obj), spec.msgtype)
            writer.write(conn, spec.timestamp_ns, bytes(raw))
    return path


def write_ros2_bag_dir(tmp_path: Path, specs: list[MessageSpec], name: str = "ros2_bag") -> Path:
    path = tmp_path / name
    with Ros2Writer(path) as writer:
        connections = {}
        for spec in specs:
            if spec.topic not in connections:
                connections[spec.topic] = writer.add_connection(spec.topic, spec.msgtype, typestore=TS)
            conn = connections[spec.topic]
            raw = TS.serialize_cdr(spec.obj, spec.msgtype) if spec.obj is not None else b"\x00"
            writer.write(conn, spec.timestamp_ns, bytes(raw))
    return path


def write_bare_db3(tmp_path: Path, specs: list[MessageSpec], name: str = "bare.db3") -> Path:
    """A standalone .db3 with no sibling metadata.yaml — built by writing a normal
    ROS2 bag directory and lifting the .db3 file out on its own."""
    bag_dir = write_ros2_bag_dir(tmp_path, specs, name="_tmp_ros2_bag")
    (db3_file,) = bag_dir.glob("*.db3")
    bare_path = tmp_path / name
    shutil.move(str(db3_file), str(bare_path))
    shutil.rmtree(bag_dir)
    return bare_path


def write_mcap(tmp_path: Path, specs: list[MessageSpec], name: str = "bag.mcap") -> Path:
    from mcap_ros2.writer import Writer as McapRos2Writer

    path = tmp_path / name
    with path.open("wb") as f:
        writer = McapRos2Writer(f)
        schemas = {}
        for spec in specs:
            if spec.msgtype not in schemas:
                msgdef, _ = TS.generate_msgdef(spec.msgtype, ros_version=2)
                schemas[spec.msgtype] = writer.register_msgdef(spec.msgtype, msgdef)
            writer.write_message(spec.topic, schemas[spec.msgtype], spec.mcap_dict, log_time=spec.timestamp_ns)
        writer.finish()
    return path
