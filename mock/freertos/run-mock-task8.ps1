# Plan-A P3 Task8 mock runner (host gcc, no hardware required).
# Compiles the UNMODIFIED production source (APP_TASK\fault_decision.c) against
# the stub/ headers (only sys.h basic types) and runs the Task8 fault-injection
# test suites:
#   stall detection (8 suites) + sensor health (7) + queue-full policy (6)
#   + 3 end-to-end sequences.
# L2 evidence: log saved to docs/design-evidence/logs/.
param(
    [string]$Gcc = 'D:\QT\Tools\mingw730_64\bin\gcc.exe',
    [string]$OutLog = '..\..\docs\design-evidence\logs\task8-fault-injection-mock-run.log'
)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$root = Resolve-Path '..\..'

# fault_decision.c only depends on sys.h basic types (u8/u16/u32/float);
# no FreeRTOS/SPL, so stub/sys.h is the only stub needed.
& $Gcc -Wall -Wextra -std=c99 -I'stub' -I"$root\APP_TASK" `
    -o "$env:TEMP\task8-mock.exe" `
    '.\test_fault_decision.c' "$root\APP_TASK\fault_decision.c"
if ($LASTEXITCODE -ne 0) { throw "mock compile failed: $LASTEXITCODE" }
Write-Output "gcc exit: $LASTEXITCODE"

$outDir = Split-Path $OutLog -Parent
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
& "$env:TEMP\task8-mock.exe" | Tee-Object -FilePath $OutLog
exit $LASTEXITCODE
