# Contributing

Thanks for considering a contribution to `deepen-bag-check`.

## Development setup

```
uv sync
uv run pytest
uv run ruff check .
```

Tests build small synthetic bags/mcap files on the fly (see `tests/bagcheck/conftest.py`)
— there are no committed fixtures and no network access is required to run the suite.

## Before opening a PR

- Add or update tests for any behavioral change. `run_checks()` and the individual
  check functions in `bagcheck/checks.py` are pure functions over plain data, so most
  new logic can be unit-tested without building a full bag.
- Run `uv run pytest` and `uv run ruff check .` locally; CI runs the same two commands
  on Python 3.11 and 3.12.
- Keep the human-readable and JSON report outputs in sync — both are rendered from the
  same `ValidationReport`, so a new check should show up correctly in both.

## Reporting a vendor/format gap

If your rig uses a lidar, camera, or message type this tool doesn't recognize yet,
please open an issue with:

- The message type string(s) involved (e.g. `sensor_msgs/msg/PointCloud2`).
- For point clouds, the field names and datatypes (`ros2 bag info` or
  `ros2 topic echo --field fields <topic>` output is enough).
- Whether you're able to share a small (few-second) sample bag.

## License

By contributing, you agree that your contributions will be licensed under the
Apache License, Version 2.0 (see `LICENSE`).
