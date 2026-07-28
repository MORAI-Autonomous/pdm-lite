# MORAI PDM Coverage Package 2026.07.24-r1

This repository contains the MORAI PDM runner based on the 0724 fixed-lockstep
implementation. It also includes the later `fixed-auto-udp` control mode.

Scenario suites are not bundled. Open a suite in MORAI SIM yourself for GUI
attach mode, or provide your own `.msuite` path when suite metadata or headless
loading is required.

## Runtime modes

| Control mode | SIM time mode | Clock owner | Control transport |
|---|---|---|---|
| `variable-udp` | Variable | SIM | UDP |
| `fixed-auto-udp` | Fixed | SIM (`user_control=0`) | UDP |
| `fixed-udp` | Fixed | PDM runner | UDP + TCP `FixedStep(1)` |
| `fixed-tcp-lockstep` | Fixed | PDM runner | TCP control + `FixedStep(1)` |

GUI/headless execution mode is independent from the control mode.

- `gui`: attaches to the suite already opened by the user.
- `headless`: can load a suite only when `--load-suite` is explicitly supplied.
- Suite load and scenario play/stop are never implied by GUI attach mode.

## Installation

```powershell
py -3 -m pip install -r .\requirements-pdm-morai.txt
```

## GUI attach examples

Variable UDP:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --execution-mode gui `
  --control-mode variable-udp `
  --tcp-port 20000
```

Fixed time with a SIM-owned clock:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --execution-mode gui `
  --control-mode fixed-auto-udp `
  --tcp-port 20000 `
  --fixed-target-fps 50 `
  --physics-delta-time 20
```

SUT-owned TCP lockstep:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --execution-mode gui `
  --control-mode fixed-tcp-lockstep `
  --tcp-port 20000 `
  --pdm-step-delta-time 20 `
  --physics-delta-time 20 `
  --lockstep-state-timeout 20 `
  --save-data-each-step
```

To use suite metadata without loading the suite, add your own path:

```powershell
  --suite "C:\path\to\scenario.msuite"
```

Do not add `--load-suite` in GUI attach mode unless changing the active suite is
intentional.

## Headless load example

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --execution-mode headless `
  --control-mode fixed-tcp-lockstep `
  --suite "C:\path\to\scenario.msuite" `
  --load-suite `
  --scenario-play `
  --tcp-port 20000 `
  --pdm-step-delta-time 20 `
  --physics-delta-time 20
```

## 0724 lockstep behavior

Each `fixed-tcp-lockstep` iteration uses the latest Ego state, sends
`ManualControlById`, validates its ACK, sends one `FixedStep(1)`, and then waits
for the next UDP Ego sample. A sample is new only when the complete
`(stamp_seconds, stamp_nanos)` tuple advances.

The runner also detects SIM-owned active-scenario changes, discards cached UDP
state from the previous scenario, and performs a bootstrap `FixedStep(1)` for
the replacement scenario. Ego-only VehicleInfo packets are supported.

## MGeo

The repository currently retains `data/morai/mgeo/KATRI` as the default
attach-only route fallback. A different map can be supplied with:

```powershell
  --mgeo-dir "C:\path\to\MGeo"
```

No DCAS or other scenario suite is included.
