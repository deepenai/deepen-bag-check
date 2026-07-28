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
