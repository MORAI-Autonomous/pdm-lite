param(
  [int]$Iterations = 5,
  [string]$Suite = "data\generated-suite\cto-1467-ego-only\cto-1467-ego-only.msuite",
  [int]$ControlSteps = 1499,
  [int]$StatusHz = 2,
  [string]$LogDir = "logs",
  [string]$SimulationLogRoot = "C:\MoraiSaveData\Clients\1.0.0.1.1.104\Windows\SimulationLogs\Hwaseong-si"
)

$ErrorActionPreference = "Continue"

# run_pdm_with_logs.ps1 이 tools 폴더 안에 있으므로,
# 같은 폴더의 run_pdm_morai_control.ps1을 절대경로로 잡음
$ControlScript = Join-Path $PSScriptRoot "run_pdm_morai_control.ps1"
$CompareScript = Join-Path $PSScriptRoot "compare_ego_statelogs.py"

# 프로젝트 루트: v1
$ProjectRoot = Split-Path $PSScriptRoot -Parent
Set-Location $ProjectRoot

if (-not (Test-Path $ControlScript)) {
  Write-Error "Control script not found: $ControlScript"
  exit 1
}

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

$BatchStamp = Get-Date -Format "yyyyMMdd_HHmmss"
$BatchLog = Join-Path $LogDir "pdm_morai_batch_${BatchStamp}.log"
$StateLogs = @()

"=== PDM-MORAI 5-run batch start ===" | Tee-Object -FilePath $BatchLog
"StartTime    : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" | Tee-Object -FilePath $BatchLog -Append
"ProjectRoot  : $ProjectRoot" | Tee-Object -FilePath $BatchLog -Append
"ControlScript: $ControlScript" | Tee-Object -FilePath $BatchLog -Append
"Suite        : $Suite" | Tee-Object -FilePath $BatchLog -Append
"Iterations   : $Iterations" | Tee-Object -FilePath $BatchLog -Append
"ControlSteps : $ControlSteps" | Tee-Object -FilePath $BatchLog -Append
"StatusHz     : $StatusHz" | Tee-Object -FilePath $BatchLog -Append
"" | Tee-Object -FilePath $BatchLog -Append

for ($i = 1; $i -le $Iterations; $i++) {
  $IterStamp = Get-Date -Format "yyyyMMdd_HHmmss"
  $IterLog = Join-Path $LogDir "pdm_morai_${IterStamp}_iter${i}.csv"
  $TraceLog = Join-Path $LogDir "pdm_morai_trace_${IterStamp}_iter${i}.csv"

  "" | Tee-Object -FilePath $BatchLog -Append
  "=== iteration $i / $Iterations start ===" | Tee-Object -FilePath $BatchLog -Append
  "IterStart : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" | Tee-Object -FilePath $BatchLog -Append
  "IterLog   : $IterLog" | Tee-Object -FilePath $BatchLog -Append

  $Start = Get-Date

  & $ControlScript `
    --suite $Suite `
    --load-suite `
    --scenario-play `
    --fixed-step `
    --tcp-port 20000 `
    --fixed-target-fps 50 `
    --pdm-step-delta-time 20 `
    --physics-delta-time 20 `
    --fixed-control-transport tcp `
    --lockstep-state-timeout 5 `
    --max-control-steps $ControlSteps `
    --trace-log $TraceLog `
    --status-hz $StatusHz `
    --save-data-each-step *>&1 |
    Tee-Object -FilePath $IterLog

  $ExitCode = $LASTEXITCODE
  $End = Get-Date
  $Elapsed = New-TimeSpan -Start $Start -End $End
  $StateLog = Get-ChildItem -LiteralPath $SimulationLogRoot -Recurse -Filter "*_statelog.csv" -File -ErrorAction SilentlyContinue |
    Where-Object { $_.LastWriteTime -ge $Start } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1
  if ($StateLog) {
    $StateLogs += $StateLog.FullName
  }

  "IterEnd   : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" | Tee-Object -FilePath $BatchLog -Append
  "Elapsed   : $($Elapsed.ToString())" | Tee-Object -FilePath $BatchLog -Append
  "ExitCode  : $ExitCode" | Tee-Object -FilePath $BatchLog -Append
  "TraceLog  : $TraceLog" | Tee-Object -FilePath $BatchLog -Append
  "StateLog  : $(if ($StateLog) { $StateLog.FullName } else { '-' })" | Tee-Object -FilePath $BatchLog -Append
  "=== iteration $i / $Iterations end ===" | Tee-Object -FilePath $BatchLog -Append

  Start-Sleep -Seconds 1
}

"" | Tee-Object -FilePath $BatchLog -Append
"EndTime : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" | Tee-Object -FilePath $BatchLog -Append
if ($StateLogs.Count -ge 2) {
  "=== statelog comparison ===" | Tee-Object -FilePath $BatchLog -Append
  if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 $CompareScript @StateLogs *>&1 | Tee-Object -FilePath $BatchLog -Append
  } else {
    & python $CompareScript @StateLogs *>&1 | Tee-Object -FilePath $BatchLog -Append
  }
}
"=== PDM-MORAI 5-run batch end ===" | Tee-Object -FilePath $BatchLog -Append
