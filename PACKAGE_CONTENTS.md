# Package Contents

- `tools/`: MORAI suite preparation, environment checks, launchers, and the PDM runner.
- `tools/pdm_morai/`: runtime modules and tests.
- `pdm_lite/team_code/`: PDM Lite controller dependencies.
- `configs/pdm_morai.example.json`: compact runtime configuration example.
- `requirements-pdm-morai.txt`: Python runtime dependencies.

No scenario suite is bundled. Users must open a suite in MORAI SIM or supply
their own `.msuite` path.

Excluded: MGeo map data, DCAS scenario suites, other generated suites, previous
dist packages, runtime logs, traces, and temporary validation data.
