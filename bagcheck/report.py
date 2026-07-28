"""Human-readable rendering of a `ValidationReport` — the fix-it list a customer
reads before ever touching the JSON. Every line aims to be exact and actionable."""

from __future__ import annotations

from bagcheck.model import CheckStatus, ValidationReport

_STATUS_LABEL = {
    CheckStatus.PASS: "PASS",
    CheckStatus.WARN: "WARN",
    CheckStatus.FAIL: "FAIL",
}


def render_human_readable(report: ValidationReport, source: str) -> str:
    lines: list[str] = []
    lines.append(f"deepen-bag-check {report.bag_check_version} — {source}")
    lines.append(f"container: {report.container_format}")
    lines.append(f"status: {report.status.value.upper()} (exit code {report.exit_code()})")
    lines.append("")

    lines.append("Topics:")
    if report.topics:
        for t in sorted(report.topics, key=lambda t: t.topic):
            bits = [f"{t.topic}", f"[{t.role.value}]", t.msgtype]
            if t.hz is not None:
                bits.append(f"{t.hz:.1f} Hz")
            bits.append(f"{t.message_count} msgs")
            if t.vendor_signature:
                bits.append(f"vendor={t.vendor_signature}")
            if t.has_per_point_time is not None:
                bits.append(f"per_point_time={'yes' if t.has_per_point_time else 'no'}")
            if t.encoding:
                bits.append(f"encoding={t.encoding}")
            lines.append("  " + "  ".join(bits))
    else:
        lines.append("  (no topics found)")
    lines.append("")

    lines.append("Checks:")
    if report.checks:
        for c in report.checks:
            prefix = f"[{_STATUS_LABEL[c.status]}] {c.id}"
            lines.append(f"  {prefix}: {c.message}")
    else:
        lines.append("  (no checks ran)")
    lines.append("")

    lines.append(
        "Eligible calibration types: "
        + (", ".join(t.value for t in report.eligible_calibration_types) or "(none)")
    )
    if report.ineligible_calibration_types:
        lines.append("Ineligible:")
        for i in report.ineligible_calibration_types:
            lines.append(f"  - {i.type.value}: {i.reason}")

    return "\n".join(lines)
