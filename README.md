# MORAI PDM Coverage Package 2026.06.26-r2

This package contains the 26-r2 MORAI PDM runner, `DCAS_Scenario_Suites_Set_02`, and the KATRI MGeo runtime dependency.

Default runner behavior is safe:

- GUI/headless is execution mode only.
- Runtime behavior is selected by control mode.
- Suite load, scenario play, scenario stop, and control send are not automatic unless explicitly requested.

Example GUI plan/check:

```powershell
.\run_dcas_variable_udp.ps1 `
  --plan-only `
  --check-suite `
  --check-udp
```

Example variable UDP run after the suite/scenario is open in MORAI GUI:

```powershell
.\run_dcas_variable_udp.ps1 `
  --scenario-name "4.2.5.2.1. Stationary Vehicle Ahead on a Straight Section of Road_08" `
  --scenario-play `
  --send-control
```

Direct runner example:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --execution-mode gui `
  --control-mode variable-udp `
  --suite ".\data\DCAS_Scenario_Suites_Set_02\DCAS_Scenario_01\DCAS_Scenario_01.msuite" `
  --tcp-port 20000 `
  --scenario-play `
  --send-control
```
