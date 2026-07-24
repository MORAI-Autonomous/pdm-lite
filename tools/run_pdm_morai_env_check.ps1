# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Script = Join-Path $Root "tools\pdm_morai_env_check.py"

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 -B $Script @args
} else {
    & python $Script @args
}
exit $LASTEXITCODE
