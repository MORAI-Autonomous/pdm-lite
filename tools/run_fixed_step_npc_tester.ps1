# TESTER_NPC_FIXED_STEP_BEGIN
# Temporary TCP-only tester. Delete after NPC deterministic validation.
param(
  [string]$Suite = "data\generated-suite\cto-1467-0\cto-1467-0.msuite",
  [int]$Steps = 1500,
  [int]$TargetFps = 50,
  [int]$PhysicsDeltaTime = 10,
  [int]$TcpPort = 20000
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Script = Join-Path $Root "tools\fixed_step_npc_tester.py"

if (Get-Command py -ErrorAction SilentlyContinue) {
  & py -3 $Script `
    --suite $Suite `
    --steps $Steps `
    --target-fps $TargetFps `
    --physics-delta-time $PhysicsDeltaTime `
    --tcp-port $TcpPort
} else {
  & python $Script `
    --suite $Suite `
    --steps $Steps `
    --target-fps $TargetFps `
    --physics-delta-time $PhysicsDeltaTime `
    --tcp-port $TcpPort
}
exit $LASTEXITCODE
# TESTER_NPC_FIXED_STEP_END
