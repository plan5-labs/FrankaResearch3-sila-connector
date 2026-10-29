# Contributing

Use Python 3.12 and uv. From the repository root:

```bash
uv sync --all-extras
uv run pytest -q
uv run ruff format --check src/
uv run ruff check src/
```

CI runs the same offline suite on Ubuntu and macOS. It never connects to a physical robot. Tests put an offline franky stand-in (`tests/fake_franky/`) first on the import path. On Linux x86_64, where the real `franky-control` wheel installs, one extra test checks that every franky attribute the backend uses still exists.

Preserve the io/features split. `io/` owns robot communication, limits, kinematics and simulation, while feature methods delegate and declare SiLA metadata. Put every safety rule in `Fr3Protocol` so that simulation and hardware enforce the same rules.

Default to simulation, loopback endpoints and disabled hardware motion. Do not add automatic retries or resume-on-startup for motions with unknown outcomes. Keep site IP addresses, credentials and raw robot logs out of commits.

For hardware-related changes, report offline tests, simulation and physical observations separately. A passing test suite does not establish collision safety.
