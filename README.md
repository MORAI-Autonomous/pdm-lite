# MORAI PDM Coverage Package 2026.07.30-r1

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

Run once with `--config`. The runner creates `configs\pdm_morai.local.json`
with built-in defaults. The default control mode is `variable-udp`. Edit
`mgeo_dir` if the authorized MGeo is stored elsewhere; the directory must
contain `link_set.json`.

Check the runtime and external map:

```powershell
.\tools\run_pdm_morai_env_check.ps1 `
  --mgeo-dir "C:\MORAI_DATA\MGeo\KATRI"
```

## Config-based execution

Run with `--config` alone to use `configs\pdm_morai.local.json`. On first use,
the `configs` directory and local file are created automatically from built-in
defaults. The whole directory is ignored by Git so machine-specific paths can
be edited safely.

```powershell
.\tools\run_pdm_morai_control.ps1 --config
```

An explicit config path is still supported:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --config ".\configs\pdm_morai.local.json"
```

Command-line arguments remain available and override config values. For
example, switch from the default `variable-udp` mode to Fixed time with a
SIM-owned clock:

```powershell
.\tools\run_pdm_morai_control.ps1 `
  --config ".\configs\pdm_morai.local.json" `
  --control-mode fixed-auto-udp
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
  --config ".\configs\pdm_morai.local.json" `
  --execution-mode headless `
  --suite "C:\path\to\scenario.msuite" `
  --load-suite `
  --scenario-play
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

MGeo map data is not included in this repository or distribution package.
Global-route calculation requires an authorized local dataset supplied with
`mgeo_dir` in the JSON config or with:

```powershell
  --mgeo-dir "C:\path\to\MGeo"
```

The path location does not change the route algorithm. Benchmark it on the
target storage with:

```powershell
py -3 .\tools\benchmark_mgeo_route.py `
  --mgeo-dir "C:\path\to\MGeo" `
  --repeats 7
```

No DCAS or other scenario suite is included.

## License

MORAI-authored portions of this repository are provided under the
[MIT License](LICENSE).

Third-party components remain under their respective licenses. In particular,
the bundled PDM-Lite source retains its
[Apache License 2.0](pdm_lite/LICENSE). Files adapted from other projects and
their associated assets are not relicensed by the repository-level MIT
License; review their original notices and license terms before redistribution
or commercial use.
