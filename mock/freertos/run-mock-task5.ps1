# Plan-A Task5 mock runner (host gcc, no hardware required).
# Compiles the UNMODIFIED production source (BSP\MQ2\mq2.c) against the
# stub/ headers and runs the Task5 MQ2 error-semantics test suites:
#   three input classes (normal / ADC timeout / RS<=0) + integration + legacy.
# L2 evidence: log saved to docs/design-evidence/logs/.
param(
    [string]$Gcc = 'D:\QT\Tools\mingw730_64\bin\gcc.exe',
    [string]$OutLog = '..\..\docs\design-evidence\logs\task5-mq2-mock-run.log'
)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$root = Resolve-Path '..\..'

& $Gcc -Wall -Wextra -I'stub' -I"$root\BSP\MQ2" `
    -o "$env:TEMP\task5-mock.exe" `
    '.\test_mq2.c' "$root\BSP\MQ2\mq2.c" -lm
if ($LASTEXITCODE -ne 0) { throw "mock compile failed: $LASTEXITCODE" }
Write-Output "gcc exit: $LASTEXITCODE"

$outDir = Split-Path $OutLog -Parent
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
& "$env:TEMP\task5-mock.exe" | Tee-Object -FilePath $OutLog
exit $LASTEXITCODE
