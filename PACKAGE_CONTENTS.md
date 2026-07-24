# Package Contents

- `tools/`: MORAI suite preparation, environment check, and PDM runner.
- `tools/pdm_morai/`: runtime modules and tests from the 26-r2 source tree.
- `pdm_lite/team_code/`: PDM Lite controller dependencies.
- `data/DCAS_Scenario_Suites_Set_02/`: packaged DCAS scenario suites 01 through 10.
- `data/morai/mgeo/KATRI/`: runtime MGeo dependency for route fallback / attach-only route generation.
- `run_dcas_variable_udp.ps1`: GUI attach / variable UDP baseline launcher.

Included suites are limited to `DCAS_Scenario_Suites_Set_02`.

Excluded: other generated suites, previous dist packages, runtime logs, and unrelated coverage data.
