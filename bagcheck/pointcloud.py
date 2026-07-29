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

`RADAR_ONLY_FIELD_NAMES` (v1.2, `classify.classify_pointcloud_role`): automotive/
robotics radar drivers publish `sensor_msgs/PointCloud2` with the same wire type as
lidar but a different field vocabulary — range/doppler/RCS-style returns instead of
lidar's ring/per-point-time geometry. Field names below are verified against real
driver source, not guessed:

- smartmicro UMRR (`smartmicro/smartmicro_ros2_radars`, `umrr_ros2_driver/src/
  smartmicro_radar_node.cpp`, `struct RadarPoint`) publishes `sensor_msgs/PointCloud2`
  directly with fields `x, y, z, radial_speed, power, rcs, noise, snr, azimuth_angle,
  elevation_angle, range` — the single strongest citation here, since it's the exact
  wire-level PointCloud2 field list, not a custom message.
- Delphi/Aptiv ESR (`astuff/astuff_sensor_msgs`, `delphi_esr_msgs/msg/EsrTrack.msg`):
  `range_rate` (plus `range`, `angle`, `width`).
- ROS 2's standard radar message package (`ros-perception/radar_msgs`, `msg/
  RadarReturn.msg`): `range, azimuth, elevation, doppler_velocity, amplitude` — the
  field vocabulary most PointCloud2-emitting radar converters mirror.
- AinsteinAI (`AinsteinAI/ainstein_radar`, `ainstein_radar_msgs/msg/RadarTarget.msg`):
  `target_id, snr, range, speed, azimuth, elevation`.
- Continental ARS408 (`tier4/ars408_driver`, `src/ars408_driver.cpp`): CAN-decodes an
  `rcs` field (`current_object.rcs = (in_can_data[7] * 0.5) - 64.0`).

Two important asymmetries, confirmed against a real public bag (Foxglove's demo.bag,
`/radar/points`, a genuine radar sibling of `/radar/range` + `/radar/tracks`): some
radar-to-PointCloud2 converters strip every vendor field and publish bare `x,y,z` —
zero overlap with `RADAR_ONLY_FIELD_NAMES` and zero overlap with lidar's `ring`. TI
mmWave (`radar-lab/ti_mmwave_rospkg`, `src/DataHandlerClass.cpp`) does the same:
`pcl::PointXYZI`, `intensity` populated from SNR, no `ring`. Field-schema matching
alone cannot catch either case — that's why `classify_pointcloud_role` also weighs
point density (radar returns are sparse; the real bag's `/radar/points` carries
20-30 points/message against `/velodyne_points`' ~40,000) and never relies on a
single signal.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Alternate field names seen for the same role across vendors/tools: ring/channel/
# laser_id, and t/time/timestamp.
RING_ALIASES: tuple[str, ...] = ("ring", "channel", "laser_id")
TIME_ALIASES: tuple[str, ...] = ("time", "t", "timestamp", "time_stamp")

REQUIRED_ROLES = ("x", "y", "z", "intensity", "ring")

# PointCloud2 field names that indicate a radar return, not a lidar point (see this
# module's docstring for per-name source citations). Every real driver checked this
# session publishes at most a handful of these, never `ring`/`channel`/`laser_id` —
# `classify.classify_pointcloud_role` treats ring-family presence as the stronger,
# decisive lidar signal and only reaches for this set when ring is absent.
RADAR_ONLY_FIELD_NAMES: frozenset[str] = frozenset(
    {
        "rcs",  # Continental ARS408 (CAN decode); smartmicro UMRR PointCloud2
        "radial_speed",  # smartmicro UMRR PointCloud2 (direct field name)
        "doppler_velocity",  # ros-perception/radar_msgs RadarReturn.msg (ROS 2 standard)
        "doppler",  # common alias for the same quantity — not independently verified
        # against a PointCloud2 wire format this session, included per the ROS radar
        # driver convention doppler_velocity/radial_speed both shorten to.
        "range_rate",  # Delphi/Aptiv ESR (delphi_esr_msgs/msg/EsrTrack.msg)
        "snr",  # smartmicro UMRR PointCloud2; TI mmWave (intensity derived from SNR);
        # Ainstein RadarTarget.msg
        "power",  # smartmicro UMRR PointCloud2 (direct field name)
        "noise",  # smartmicro UMRR PointCloud2 (direct field name)
        "amplitude",  # ros-perception/radar_msgs RadarReturn.msg
        "azimuth_angle",  # smartmicro UMRR PointCloud2 (direct field name)
        "elevation_angle",  # smartmicro UMRR PointCloud2 (direct field name)
        "azimuth",  # Ainstein RadarTarget.msg; ros-perception/radar_msgs RadarReturn.msg
        "elevation",  # Ainstein RadarTarget.msg; ros-perception/radar_msgs RadarReturn.msg
        "speed",  # Ainstein RadarTarget.msg — generic enough to be a weaker signal on
        # its own, only ever consulted alongside the rest of this set, never alone.
    }
)


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

# Tie-break order used only when a real recording's per-point time field is entirely
# absent (see `_match_vendor_signature`) and intensity dtype alone leaves more than one
# vendor standing. Velodyne is listed first: its ROS1 `velodyne_pointcloud` driver is
# the one most commonly seen, in real recordings, publishing `PointXYZIR` (x,y,z,
# intensity,ring) with no per-point time at all — this is the exact shape of the real
# `/velodyne_points` topic in Foxglove's public `demo.bag` (verified live: `point_step`
# 32, fields `x@0,y@4,z@8,intensity@16,ring@20` — a PCL/Eigen 16-byte alignment gap
# between `z` and `intensity` where a `time` field would otherwise sit, not a name or
# dtype difference). Ouster and Hesai are listed after; their own driver sources
# (docstring above) don't rule out a similarly time-stripped recording, so this is a
# documented default, not a claim of certainty — `vendor_signature` has always been a
# cosmetic, best-effort label (see this module's docstring), never load-bearing for the
# coverage/role mapping above it.
_NO_TIME_FALLBACK_PRIORITY: tuple[str, ...] = ("velodyne", "ouster", "hesai", "robosense")

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
    """Identify the lidar vendor from field NAMES + PointField datatypes alone — never
    from byte offsets or `point_step`. Real-world recordings routinely carry PCL/Eigen
    memory-alignment padding (a byte gap between two declared fields' offsets) and/or
    have their per-point time field dropped entirely by the driver or a bag-conversion
    step; neither should defeat fingerprinting, since both just mean "some bytes aren't
    a named field", not "the named fields present mean something different".

    `ring` is required (every vendor above declares one) so this never fires on a bare
    x,y,z,intensity cloud that happens to share an intensity dtype with a real vendor.
    """
    if not (roles.intensity and roles.ring):
        return None
    intensity_dtype = field_dtypes.get(roles.intensity)
    candidates = [sig for sig in VENDOR_SIGNATURES if sig.intensity_dtype == intensity_dtype]
    if not candidates:
        return None

    if roles.time:
        # Per-point time is present and decoded — the strong, unambiguous signal.
        # Unchanged from prior behavior: a time field that matches no candidate's
        # (name, dtype) pair is "unrecognized", not a guess.
        time_dtype = field_dtypes.get(roles.time)
        for sig in candidates:
            if sig.time_field == roles.time and sig.time_dtype == time_dtype:
                return sig.name
        return None

    # No per-point time field at all. If intensity dtype alone already narrowed to one
    # vendor (RoboSense's UINT8 is unique among the four researched), that's still a
    # confident match. Otherwise (Velodyne/Ouster/Hesai all use FLOAT32 intensity),
    # fall back to the documented priority order above rather than leaving a real,
    # well-formed vendor cloud unrecognized just because its time field was stripped.
    for name in _NO_TIME_FALLBACK_PRIORITY:
        if any(sig.name == name for sig in candidates):
            return name
    return None
