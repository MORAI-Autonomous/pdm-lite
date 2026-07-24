$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$Suite = Join-Path $Root "data\DCAS_Scenario_Suites_Set_02\DCAS_Scenario_01\DCAS_Scenario_01.msuite"
$Trace = Join-Path $Root "logs\dcas_variable_udp_trace.csv"

New-Item -ItemType Directory -Force (Split-Path -Parent $Trace) | Out-Null

& (Join-Path $Root "tools\run_pdm_morai_control.ps1") `
  --execution-mode gui `
  --control-mode variable-udp `
  --suite $Suite `
  --tcp-port 20000 `
  --trace-log $Trace `
  @args

exit $LASTEXITCODE
