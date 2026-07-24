# TESTER_CONTROL_REPLAY_BEGIN
# Temporary deterministic-input tester. Delete this file after validation.
param(
  [string]$Suite = "data\generated-suite\cto-1467-0\cto-1467-0.msuite",
  [string]$ControlCsv = "logs\deterministic-0622.csv",
  [double]$PdmStepDeltaTime = 10,
  [int]$PhysicsDeltaTime = 10,
  [int]$TcpPort = 20000
)

$ErrorActionPreference = "Stop"
$ControlScript = Join-Path $PSScriptRoot "run_pdm_morai_control.ps1"

& $ControlScript `
  --suite $Suite `
  --tester-control-csv $ControlCsv `
  --load-suite `
  --scenario-play `
  --fixed-step `
  --tcp-port $TcpPort `
  --pdm-step-delta-time $PdmStepDeltaTime `
  --physics-delta-time $PhysicsDeltaTime `
  --save-data-each-step

exit $LASTEXITCODE
# TESTER_CONTROL_REPLAY_END
