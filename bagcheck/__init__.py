"""deepen-bag-check: pre-upload validation for rosbag calibration inputs.

Reads ROS1 `.bag`, ROS2 `.db3`, and ROS2 `.mcap` recordings (no ROS installation
required) and reports whether they contain what a targetless calibration run needs —
before any data leaves the machine. See `bagcheck.engine.run_checks` for the library
entrypoint and `bagcheck.cli` for the command-line wrapper around it.
"""

from bagcheck.engine import run_checks
from bagcheck.model import (
    BAG_CHECK_VERSION,
    SCHEMA_VERSION,
    CalibrationType,
    CheckResult,
    CheckStatus,
    ReportStatus,
    TopicRole,
    TopicSummary,
    ValidationReport,
)

__version__ = BAG_CHECK_VERSION

__all__ = [
    "BAG_CHECK_VERSION",
    "SCHEMA_VERSION",
    "CalibrationType",
    "CheckResult",
    "CheckStatus",
    "ReportStatus",
    "TopicRole",
    "TopicSummary",
    "ValidationReport",
    "run_checks",
]
