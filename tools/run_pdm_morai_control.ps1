# Copyright Scenario Validation Team, Morai Inc. All Rights Reserved.

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$Script = Join-Path $Root "tools\pdm_morai_control.py"

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 $Script @args
} else {
    & python $Script @args
}
exit $LASTEXITCODE
