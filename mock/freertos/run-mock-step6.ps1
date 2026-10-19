# Plan-A Step6 mock runner (host gcc, no hardware required).
# Compiles the UNMODIFIED production sources (iap_meta.c, crc32.c)
# against the stub/ headers and runs the Step6 test suites.
# L2 evidence: logs saved to docs/design-evidence/logs/.
param(
    [string]$Gcc = 'D:\QT\Tools\mingw730_64\bin\gcc.exe',
    [string]$OutLog = '..\..\docs\design-evidence\logs\step6-mock-run.log'
)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$root = Resolve-Path '..\..'

& $Gcc -Wall -Wextra -I'stub' -I"$root\BSP\IAP" -I"$root\BSP\CRC32" -I"$root\CONFIG" `
    -o "$env:TEMP\step6-mock.exe" `
    '.\test_iap_meta.c' "$root\BSP\IAP\iap_meta.c" "$root\BSP\CRC32\crc32.c" '.\fake_flash.c'
if ($LASTEXITCODE -ne 0) { throw "mock compile failed: $LASTEXITCODE" }
Write-Output "gcc exit: $LASTEXITCODE"

& "$env:TEMP\step6-mock.exe" | Tee-Object -FilePath $OutLog
exit $LASTEXITCODE
