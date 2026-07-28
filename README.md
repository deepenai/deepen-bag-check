# deepen-bag-check

A pre-upload validator for rosbags used in sensor calibration. It reads your bag,
tells you exactly what it found, and flags anything that would block a calibration
run — in seconds, on your own machine, before you upload anything over any network.

No ROS installation is required. `deepen-bag-check` reads the raw bag/mcap formats
directly in pure Python.

## Why

Multi-sensor calibration jobs can take a long time to run. Finding out afterward that
your bag was missing a CameraInfo topic, had an uncalibrated intrinsics matrix, or
never actually turned the rig enough to excite the extrinsics is an expensive way to
learn that. `deepen-bag-check` runs the same checks up front, in seconds, so you catch
that class of problem before you wait on (or pay for) a run that was never going to
succeed.

## Install

```
pip install deepen-bag-check
```

```
pipx install deepen-bag-check
```

```
uv tool install deepen-bag-check
```

(Or, from a clone of this repo: `uv sync` — the console script `deepen-bag-check` is
registered as part of the package, or run it directly with `uv run deepen-bag-check`.)

## Usage

```
deepen-bag-check my_drive.bag
deepen-bag-check my_drive.mcap --for lidar-camera
deepen-bag-check /path/to/ros2_bag_dir --json > report.json
```

```
$ deepen-bag-check my_drive.mcap --for lidar-camera
deepen-bag-check 1.0.0 — my_drive.mcap
container: ros2_mcap
status: WARNINGS (exit code 1)

Topics:
  /sensor/camera/front_wide/image/compressed  [camera_compressed]  sensor_msgs/msg/CompressedImage  20.0 Hz  6023 msgs  encoding=jpeg
  /sensor/lidar/roof/points  [lidar]  sensor_msgs/msg/PointCloud2  10.1 Hz  3012 msgs  vendor=hesai  per_point_time=yes
  /sensor/imu  [imu]  sensor_msgs/msg/Imu  100.0 Hz  31000 msgs

Checks:
  [WARN] camera_info_present: /sensor/camera/front_wide/image/compressed has no paired CameraInfo topic — supply an intrinsics.json sidecar or run intrinsic pre-calibration.
  [PASS] pointcloud_field_schema: /sensor/lidar/roof/points: fields map cleanly to x,y,z,intensity,ring (vendor_signature=hesai).
  [PASS] motion_excitation: cumulative yaw 94.2° over 31.4s — sufficient rotational excitation.
  [PASS] requested_calibration_coverage: bag meets minimum sensor coverage for --for lidar_camera.

Eligible calibration types: lidar_camera, multi_lidar
Ineligible:
  - lidar_vehicle: no CAN/vehicle-speed topic found
  - lidar_imu: no IMU topic found (sensor_msgs/Imu)
```

### Exit codes

| Code | Meaning |
|---|---|
| `0` | Passed — no issues found. |
| `1` | Passed with warnings — usable, but quality may be degraded (see the fix-it list). |
| `2` | Failed — the bag is not usable for the requested calibration type. |

### Flags

| Flag | Meaning |
|---|---|
| `--for {lidar-camera,multi-lidar,lidar-vehicle,lidar-imu}` | Gate the exit code on whether the bag has the minimum sensor set for this calibration type. Omit to only report what the bag is eligible for. |
| `--json` | Print the machine-readable report (see below) instead of the human-readable one. |
| `--min-duration-s SECONDS` | Override the minimum bag duration (default: 5s). |
| `--version` | Print the tool version. |

## What it checks

1. **Container detection** — ROS1 `.bag`, ROS2 `.db3` (bag directory or a standalone
   file), ROS2 `.mcap` (bag directory or a standalone file). Anything else — including
   a file with the right extension but a corrupt/truncated body — gets a specific
   error instead of a confusing crash downstream.
2. **Schema introspection** — every topic's message type is enumerated and checked
   against a table of known custom/vendor types. A Livox `CustomMsg` topic, for
   example, is flagged with the exact conversion command to run first.
3. **Topic classification by message type, never by name** — a camera is anything
   publishing `sensor_msgs/Image` or `CompressedImage`, a lidar is anything publishing
   `sensor_msgs/PointCloud2`, regardless of what the topic happens to be called.
   H.264/`ffmpeg_image_transport` camera topics are flagged separately, since they
   need a re-export before anything downstream can read them.
4. **PointCloud2 field-schema check** — point cloud field names and types vary by
   lidar vendor. This tool normalizes the common variants (Velodyne, Ouster, Hesai,
   RoboSense) to canonical roles (`x`, `y`, `z`, `intensity`, `ring`, per-point `time`)
   and reports exactly which field is missing or unmappable if normalization fails.
5. **Vendor raw-packet lidar recognition** — Hesai `pandar_msgs/PandarScan`, Velodyne
   `velodyne_msgs/VelodyneScan`, and Ouster raw-packet topics are classified as
   `lidar_raw` — recognized, never decoded. Raw Hesai packets count toward lidar
   coverage since Deepen's calibration engine decodes them natively; other raw-packet
   vendors are flagged with a warning, since generic `PointCloud2` remains the
   preferred lane (see "Vendor raw-packet lidar lanes" below).
6. **Per-calibration-type coverage** — `--for lidar-camera` (etc.) checks whether the
   bag has the minimum sensor set for that calibration mode.
7. **Sync and quality checks** — cross-topic time overlap and gaps, per-topic message
   rates, duration, a motion-excitation estimate from IMU angular velocity, CameraInfo
   presence and plausibility (a zeroed `K` matrix means "uncalibrated" per ROS
   convention), and tf-tree completeness for every sensor frame referenced in the bag.
8. **Two output formats from one report** — the human-readable text above, or
   `--json` for scripting/CI. Both come from the exact same underlying check results,
   so nothing is inconsistent between them.

## The JSON report

`--json` prints a versioned report (`schema_version`, currently `"1.1"`):

```json
{
  "schema_version": "1.1",
  "bag_check_version": "1.0.0",
  "status": "warnings",
  "container_format": "ros2_mcap",
  "requested_calibration_type": "lidar_camera",
  "topics": [
    {
      "topic": "/sensor/lidar/roof/points",
      "type": "sensor_msgs/msg/PointCloud2",
      "role": "lidar",
      "message_count": 3012,
      "hz": 10.1,
      "vendor_signature": "hesai",
      "has_per_point_time": true,
      "encoding": null
    }
  ],
  "checks": [
    {
      "id": "camera_info_present",
      "status": "warn",
      "message": "...",
      "topic": "/sensor/camera/front_wide/image/compressed"
    }
  ],
  "eligible_calibration_types": ["lidar_camera", "multi_lidar"],
  "ineligible_calibration_types": [
    { "type": "lidar_vehicle", "reason": "no CAN/vehicle-speed topic found" }
  ]
}
```

## Using it as a library

The CLI is a thin wrapper. Everything is available directly for programmatic use:

```python
from bagcheck import run_checks, CalibrationType

report = run_checks("my_drive.mcap", calibration_type=CalibrationType.LIDAR_CAMERA)
print(report.status, report.exit_code())
for check in report.checks:
    print(check.status, check.message)
```

## Vendor field reference

Field name and datatype aliases this tool recognizes when normalizing PointCloud2
schemas (dtype taken from each vendor's own ROS driver source):

| Vendor | x/y/z | intensity | ring | per-point time |
|---|---|---|---|---|
| Velodyne | `x,y,z` float32 | `intensity` float32 | `ring` uint16 | `time` float32 |
| Ouster | `x,y,z` float32 | `intensity` float32 | `ring` uint16 | `t` uint32 |
| Hesai | `x,y,z` float32 | `intensity` float32 | `ring` uint16 | `timestamp` float64 |
| RoboSense | `x,y,z` float32 | `intensity` **uint8** | `ring` uint16 | `timestamp` float64 |

`ring` is also recognized under the aliases `channel` and `laser_id`; per-point time
is also recognized under `time_stamp`.

## Vendor raw-packet lidar lanes

Some lidar drivers publish raw vendor UDP packets instead of `sensor_msgs/PointCloud2`
— these are classified as `lidar_raw`, recognized only (never decoded here):

| Vendor | Message type | Coverage | Note |
|---|---|---|---|
| Hesai | `pandar_msgs/PandarScan` | Counts as lidar coverage | Deepen's calibration engine decodes these packets natively — no conversion needed. |
| Velodyne | `velodyne_msgs/VelodyneScan` | Does not count | Convert to `PointCloud2` with the vendor driver, or verify engine support first. |
| Ouster | `ouster_ros/PacketMsg` (also `ouster_sensor_msgs/PacketMsg`) | Does not count | Convert to `PointCloud2` with the vendor driver, or verify engine support first. |

Generic `sensor_msgs/PointCloud2` is still the preferred lane for every vendor,
including Hesai — raw packets are a recognized fallback, not a replacement.

## Known limitations

- **Non-ROS-native MCAP** (Foxglove/Protobuf/JSON-schema recordings without a ROS
  layer) and **PX4 `.ulg`** logs are not supported. Convert with `pyulog`/`ulog2rosbag`
  first for PX4 data.
- **H.264/`ffmpeg_image_transport` images** are flagged, not decoded. Re-record with
  the `raw` or `compressed` (JPEG) transport, or convert first with
  [`ffmpeg_image_transport_tools`](https://github.com/ros-misc-utilities/ffmpeg_image_transport_tools).
- **Livox `CustomMsg`** is flagged, not converted automatically. Run
  [`livox_to_pointcloud2`](https://github.com/porizou/livox_to_pointcloud2) first.
- **`--for lidar-imu`** checks for a lidar and an IMU topic, but does not currently
  require GNSS. Lidar-IMU calibration may also require a GNSS topic as a fixed
  reference for a rig with little motion diversity — a bag without one will still be
  reported as "eligible" here, with a warning, since the minimum topic-level
  requirement is met even if the calibration itself needs more.
- **CameraInfo pairing** with its image topic is done by shared topic namespace
  (stripping the last path segment, and `/compressed` if present). A rig whose
  CameraInfo topic doesn't share a namespace with its image topic won't be paired
  automatically.
- **Non-Hesai raw-packet lidars** (Velodyne, Ouster) are recognized but not decoded,
  and don't count toward calibration-type coverage — see "Vendor raw-packet lidar
  lanes" above. Only Hesai `PandarScan` is engine-ingestible as raw packets today.

## Development

```
uv sync
uv run pytest
uv run ruff check .
```

Tests build small synthetic bags/mcap files on the fly (see `tests/bagcheck/conftest.py`)
— no committed fixtures, no network access required. See `CONTRIBUTING.md` for more.

## Hosted calibration

`deepen-bag-check` is free and open source, and works standalone — nothing here talks
to a network. If you'd rather not run the calibration pipeline itself, Deepen AI also
offers hosted and in-VPC multi-sensor calibration: https://deepen.ai

## License

Apache License, Version 2.0. See `LICENSE`.
