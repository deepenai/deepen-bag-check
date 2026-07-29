import pytest

from bagcheck.pointcloud import normalize_fields
from tests.bagcheck.conftest import VENDOR_FIELD_LAYOUTS


def _dtypes(vendor: str) -> dict[str, str]:
    from bagcheck.pointcloud import POINTFIELD_DATATYPE_NAMES

    return {name: POINTFIELD_DATATYPE_NAMES[dt] for name, dt in VENDOR_FIELD_LAYOUTS[vendor]}


@pytest.mark.parametrize(
    ("vendor", "expected_time_field"),
    [
        ("velodyne", "time"),
        ("ouster", "t"),
        ("hesai", "timestamp"),
        ("robosense", "timestamp"),
    ],
)
def test_vendor_field_variants_normalize_correctly(vendor: str, expected_time_field: str) -> None:
    roles = normalize_fields(_dtypes(vendor))
    assert roles.has_xyz
    assert roles.intensity == "intensity"
    assert roles.ring == "ring"
    assert roles.time == expected_time_field
    assert roles.has_per_point_time
    assert roles.missing == []
    assert roles.vendor_signature == vendor


def test_ring_alias_channel_is_recognized() -> None:
    roles = normalize_fields({"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "FLOAT32", "channel": "UINT16"})
    assert roles.ring == "channel"
    assert roles.missing == []


def test_missing_xyz_is_reported() -> None:
    roles = normalize_fields({"x": "FLOAT32", "intensity": "FLOAT32", "ring": "UINT16"})
    assert not roles.has_xyz
    assert "y" in roles.missing
    assert "z" in roles.missing


def test_missing_ring_and_time_reported_but_xyz_present() -> None:
    roles = normalize_fields({"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "FLOAT32"})
    assert roles.has_xyz
    assert roles.missing == ["ring"]
    assert not roles.has_per_point_time


def test_no_matching_vendor_signature_when_dtypes_dont_match_any_known_vendor() -> None:
    roles = normalize_fields(
        {"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "FLOAT64", "ring": "UINT16", "timestamp": "FLOAT32"}
    )
    assert roles.vendor_signature is None


# --- vendor fingerprint tolerance for PCL/Eigen alignment padding ------------------


def test_velodyne_recognized_when_time_field_is_absent() -> None:
    """A real Velodyne recording (Foxglove's public `demo.bag`, `/velodyne_points`) has
    a PCL/Eigen alignment gap where a `time` field would sit and publishes no per-point
    time at all — `x,y,z,intensity,ring` only. `normalize_fields` only ever sees names
    + dtypes (never byte offsets), so the padding itself is invisible here; what matters
    is that the vendor match no longer hard-requires `time` to be present."""
    roles = normalize_fields({"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "FLOAT32", "ring": "UINT16"})
    assert roles.vendor_signature == "velodyne"
    assert not roles.has_per_point_time
    assert roles.missing == []


def test_robosense_recognized_when_time_field_is_absent() -> None:
    """RoboSense's UINT8 intensity is unique among the four vendors researched, so it
    still resolves confidently without a time field — no reliance on the
    Velodyne-first fallback order at all."""
    roles = normalize_fields({"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "UINT8", "ring": "UINT16"})
    assert roles.vendor_signature == "robosense"


def test_ouster_not_collided_with_velodyne_fallback_when_time_present() -> None:
    """Collision guard: Ouster and Velodyne both use FLOAT32 intensity — exactly the
    pair the no-time fallback above could conflate. When Ouster's own time field ('t',
    UINT32) is present, it must still be identified as "ouster", never "velodyne"."""
    roles = normalize_fields(_dtypes("ouster"))
    assert roles.vendor_signature == "ouster"


def test_bare_xyz_intensity_with_no_ring_is_not_guessed_as_a_vendor() -> None:
    """A cloud missing `ring` entirely isn't a degraded real vendor cloud — every known
    vendor signature includes `ring` — so it must stay unrecognized, not fall into the
    no-time fallback."""
    roles = normalize_fields({"x": "FLOAT32", "y": "FLOAT32", "z": "FLOAT32", "intensity": "FLOAT32"})
    assert roles.vendor_signature is None
