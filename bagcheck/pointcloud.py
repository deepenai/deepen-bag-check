"""PointCloud2 vendor field-alias normalization.

Field *names* and *dtypes* for x/y/z/intensity/ring/per-point-time are not
standardized across lidar vendors. The dtypes below are taken from each vendor's own
ROS driver source, not guessed:

- Velodyne: x,y,z FLOAT32 · intensity FLOAT32 · ring UINT16 · time FLOAT32.
- Ouster: x,y,z FLOAT32 · intensity FLOAT32 · t UINT32 · reflectivity UINT16 ·
  ring UINT16 (ouster-ros `os_point.h`: `_Point` struct — field set varies by driver
  profile, but `t`/`intensity`/`ring` are the three this checker maps).
- Hesai: x,y,z FLOAT32 · intensity FLOAT32 · ring UINT16 · timestamp FLOAT64
  (HesaiLidar_ROS_2.0 `source_driver_ros1.hpp`, `addPointField` calls — verbatim).
- RoboSense: x,y,z FLOAT32 · intensity UINT8 · ring UINT16 · timestamp FLOAT64
  (`PointXYZIRT`, RoboSense's own field-layout doc).

Vendor *identification* (`vendor_signature`) is a nice-to-have for the report; the
role mapping (x,y,z,intensity,ring,time) is what actually matters and is computed
from the alias tables regardless of whether a vendor signature matches.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Alternate field names seen for the same role across vendors/tools: ring/channel/
# laser_id, and t/time/timestamp.
RING_ALIASES: tuple[str, ...] = ("ring", "channel", "laser_id")
TIME_ALIASES: tuple[str, ...] = ("time", "t", "timestamp", "time_stamp")

REQUIRED_ROLES = ("x", "y", "z", "intensity", "ring")


@dataclass(frozen=True)
class VendorSignature:
    name: str
    intensity_dtype: str
    time_field: str
    time_dtype: str


VENDOR_SIGNATURES: tuple[VendorSignature, ...] = (
    VendorSignature("velodyne", intensity_dtype="FLOAT32", time_field="time", time_dtype="FLOAT32"),
    VendorSignature("ouster", intensity_dtype="FLOAT32", time_field="t", time_dtype="UINT32"),
    VendorSignature("hesai", intensity_dtype="FLOAT32", time_field="timestamp", time_dtype="FLOAT64"),
    VendorSignature("robosense", intensity_dtype="UINT8", time_field="timestamp", time_dtype="FLOAT64"),
)

# sensor_msgs/PointField datatype constants (ROS1 and ROS2 agree on these values).
POINTFIELD_DATATYPE_NAMES: dict[int, str] = {
    1: "INT8",
    2: "UINT8",
    3: "INT16",
    4: "UINT16",
    5: "INT32",
    6: "UINT32",
    7: "FLOAT32",
    8: "FLOAT64",
}


@dataclass
class FieldRoleMap:
    x: str | None = None
    y: str | None = None
    z: str | None = None
    intensity: str | None = None
    ring: str | None = None
    time: str | None = None
    vendor_signature: str | None = None
    missing: list[str] = field(default_factory=list)

    @property
    def has_xyz(self) -> bool:
        return self.x is not None and self.y is not None and self.z is not None

    @property
    def has_per_point_time(self) -> bool:
        return self.time is not None


def normalize_fields(field_dtypes: dict[str, str]) -> FieldRoleMap:
    """Map a PointCloud2's raw field names to canonical roles.

    `field_dtypes`: {field_name: PointField datatype name, e.g. "x": "FLOAT32"}.
    """
    names = set(field_dtypes)
    result = FieldRoleMap()
    missing: list[str] = []

    for axis in ("x", "y", "z"):
        if axis in names:
            setattr(result, axis, axis)
        else:
            missing.append(axis)

    if "intensity" in names:
        result.intensity = "intensity"
    else:
        missing.append("intensity")

    result.ring = next((alias for alias in RING_ALIASES if alias in names), None)
    if result.ring is None:
        missing.append("ring")

    # Per-point time absence is a degraded-quality warning, not a hard failure —
    # deliberately not added to `missing`.
    result.time = next((alias for alias in TIME_ALIASES if alias in names), None)

    result.missing = missing
    result.vendor_signature = _match_vendor_signature(field_dtypes, result)
    return result


def _match_vendor_signature(field_dtypes: dict[str, str], roles: FieldRoleMap) -> str | None:
    if not (roles.intensity and roles.time):
        return None
    intensity_dtype = field_dtypes.get(roles.intensity)
    time_dtype = field_dtypes.get(roles.time)
    for sig in VENDOR_SIGNATURES:
        if (
            sig.time_field == roles.time
            and sig.intensity_dtype == intensity_dtype
            and sig.time_dtype == time_dtype
        ):
            return sig.name
    return None
