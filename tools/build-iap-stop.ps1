# build-iap-stop.ps1 -- Task 16 全量复测 / CB 布局冻结验证 / .iap round-trip 流水线
#
# IAP+STOP，见本仓库 docs/design-evidence/））
# 一次执行覆盖计划 Steps 全部验证点：
#   Stage 0  Python 工具单测（check_memory_layout / iap_tools，TDD 回归）
#   Stage 1  全量 host mock 回归（13 套件）+ 合并日志 iap-stop-mock.log
#   Stage 2  FINAL_AUDIT：清空重建 Boot_SizeAudit / App_SizeAudit（0E0W）
#            + 尺寸门禁（map/bin 同源、RAM 预算、半字对齐、冻结公式、硬余量）
#   Stage 3  主工程 project.uvprojx 重建（0E0W，三 APP 工程证据）
#   Stage 4  最终布局构建 pass1：Boot_CB_Provisional / App_CB_Provisional 重建
#            + 同一套门禁（PROV_PASS1）
#   Stage 5  收敛验证 pass2：再次重建 + 门禁（PROV_PASS2），
#            连续两次 LAYOUT_COMPUTED 逐字相同才判定收敛
#   Stage 6  check_memory_layout.py --final 全量交叉校验 -> iap-stop-layout.log
#   Stage 7  旧绝对地址散落扫描（stale-scan）
#   Stage 8  .iap 打包 + 解包回 .bin 字节级 round-trip
#
# 产物：
#   docs/design-evidence/logs/iap-stop-build.log   本流水线日志
#   docs/design-evidence/logs/iap-stop-mock.log    mock 合并日志
#   docs/design-evidence/logs/iap-stop-layout.log  布局校验日志
#   OBJ_APP_PROV/range_hood_app.iap                                   升级包（gitignored）
#
# 用法（仓库根目录执行）：
#   powershell -ExecutionPolicy Bypass -File tools\build-iap-stop.ps1
#   powershell -ExecutionPolicy Bypass -File tools\build-iap-stop.ps1 -SkipMock
#
# 退出码：0 全部通过；1 任一阶段失败。

param(
    [string]$Uv4Path = 'D:\Keilv5\UV4\UV4.exe',
    [string]$PythonPath = 'python',
    [string]$Gcc = 'D:\QT\Tools\mingw730_64\bin\gcc.exe',
    [switch]$SkipMock
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$logsRel = 'docs\design-evidence\logs'
$buildLog = Join-Path $root (Join-Path $logsRel 'iap-stop-build.log')

# ---------------------------------------------------------------------------
# 输出：同时写控制台与 iap-stop-build.log
# ---------------------------------------------------------------------------

function Emit {
    param([string]$msg)
    Write-Host $msg
    Add-Content -Path $buildLog -Value $msg -Encoding UTF8
}

function Emit-Raw {
    param([object[]]$lines)
    foreach ($l in $lines) { Emit ("{0}" -f $l) }
}

function Invoke-Py {
    param([string[]]$PyArgs)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out = & $PythonPath @PyArgs 2>&1
        $rc = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }
    foreach ($l in $out) { Emit ("{0}" -f $l) }
    return @($rc, $out)
}

# ---------------------------------------------------------------------------
# UV4 重建（清空输出目录 + Rebuild + 0E0W 校验）
# ---------------------------------------------------------------------------

function Invoke-KeilBuild {
    param([string]$ProjectFile, [string]$Target, [string]$ObjDir)

    $objAbs = Join-Path $root $ObjDir
    if (Test-Path $objAbs) {
        Remove-Item -Path $objAbs -Recurse -Force
    }
    New-Item -ItemType Directory -Path $objAbs | Out-Null

    $projAbs = Join-Path $root $ProjectFile
    if (-not (Test-Path $projAbs)) {
        Emit "FAIL: project file not found: $projAbs"
        return $false
    }

    $logAbs = Join-Path $objAbs 'build.log'
    Emit "==> Rebuild '$Target' in $ProjectFile (log: $ObjDir\build.log)"
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & $Uv4Path -b $projAbs -t $Target -j0 -o $logAbs | Out-Null
        $rc = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }

    if (-not (Test-Path $logAbs)) {
        Emit "FAIL: build log not created: $logAbs"
        return $false
    }
    $logText = Get-Content $logAbs -Raw
    if ($logText -match '(\d+) Error\(s\), (\d+) Warning\(s\)') {
        $errCount = $Matches[1]; $warnCount = $Matches[2]
    } else {
        Emit "FAIL: no error/warning summary found in $logAbs"
        Emit-Raw (Get-Content $logAbs)
        return $false
    }

    if ($rc -ne 0 -or $errCount -ne '0' -or $warnCount -ne '0') {
        Emit "FAIL: target '$Target' rc=$rc, $errCount Error(s), $warnCount Warning(s)"
        Emit-Raw (Get-Content $logAbs)
        return $false
    }
    Emit "PASS: target '$Target' 0 Error(s), 0 Warning(s) (rc=$rc)"
    return $true
}

# ---------------------------------------------------------------------------
# mock 合并日志（13 套件 -> iap-stop-mock.log）
# ---------------------------------------------------------------------------

$mockSuites = @(
    @{ name = 'crc32';            log = 'iap-stop-crc32-mock.log' },
    @{ name = 'meta';             log = 'iap-stop-meta-mock.log' },
    @{ name = 'protocol';         log = 'iap-stop-protocol-mock.log' },
    @{ name = 'flash';            log = 'iap-stop-flash-mock.log' },
    @{ name = 'image';            log = 'iap-stop-image-mock.log' },
    @{ name = 'lowpower-math';    log = 'iap-stop-lowpower-math-mock.log' },
    @{ name = 'lowpower-policy';  log = 'iap-stop-lowpower-policy-mock.log' },
    @{ name = 'rtc-stub';         log = 'iap-stop-rtc-stub-mock.log' },
    @{ name = 'rtc-lse';          log = 'iap-stop-rtc-lse-mock.log' },
    @{ name = 'exti-stop';        log = 'iap-stop-exti-stop-mock.log' },
    @{ name = 'key-wakeup';       log = 'iap-stop-key-wakeup-mock.log' },
    @{ name = 'task-health';      log = 'iap-stop-task-health-mock.log' },
    @{ name = 'standby-blocking'; log = 'iap-stop-standby-blocking-mock.log' }
)

function Merge-MockLogs {
    $merged = New-Object System.Collections.Generic.List[string]
    $stamp = Get-Date -Format 'yyyy-MM-dd HH:mm'
    $merged.Add("# IAP+STOP 全量 host mock 回归（plan Task 16，$stamp）")
    $merged.Add('# 命令：powershell -ExecutionPolicy Bypass -File mock\freertos\run-mock-iap-stop.ps1 -Suite all')
    $merged.Add('# 13 套件全绿：CRC / 元数据断电注入 / 协议 / Flash / 镜像 / 低功耗换算 /')
    $merged.Add('# 低功耗决策 / RTC stub+LSE / EXTI STOP 后端 / 按键唤醒 / 任务健康 / 待机阻塞')
    $merged.Add('')

    $total = 0; $totalFailed = 0; $passCount = 0
    foreach ($s in $mockSuites) {
        $logPath = Join-Path (Join-Path $root $logsRel) $s.log
        if (-not (Test-Path $logPath)) {
            Emit "FAIL: mock log missing: $logPath"
            return $false
        }
        $raw = [IO.File]::ReadAllText($logPath)
        $count = -1; $failed = -1
        $m = [regex]::Matches($raw, '(?m)(\d+) checks, (\d+) failed')
        if ($m.Count -gt 0) {
            $count = [int]$m[$m.Count - 1].Groups[1].Value
            $failed = [int]$m[$m.Count - 1].Groups[2].Value
        } else {
            $m2 = [regex]::Matches($raw, '(?m)(\d+)/(\d+) cases passed')
            if ($m2.Count -gt 0) {
                $count = [int]$m2[$m2.Count - 1].Groups[2].Value
                $failed = $count - [int]$m2[$m2.Count - 1].Groups[1].Value
            }
        }
        if ($count -lt 0) {
            Emit "FAIL: cannot extract check count from $($s.log)"
            return $false
        }
        $total += $count; $totalFailed += $failed
        if ($failed -eq 0) { $passCount++ }
        $merged.Add("===== suite: $($s.name) ($count checks) =====")
        $merged.Add('')
        $merged.AddRange([string[]](Get-Content $logPath))
        $merged.Add('')
    }
    $merged.Add("TOTAL: $total checks, $totalFailed failed ($passCount/13 suites PASS)")

    $mergedPath = Join-Path (Join-Path $root $logsRel) 'iap-stop-mock.log'
    [IO.File]::WriteAllLines($mergedPath, $merged, [Text.UTF8Encoding]::new($false))
    Emit "mock merged log: $logsRel\iap-stop-mock.log"
    Emit "mock: TOTAL $total checks, $totalFailed failed ($passCount/13 suites PASS)"
    return ($totalFailed -eq 0 -and $passCount -eq 13)
}

# ---------------------------------------------------------------------------
# 流水线
# ---------------------------------------------------------------------------

$layoutH = 'CONFIG\memory_layout.h'
$summary = New-Object System.Collections.Generic.List[string]

Push-Location $root
try {
    New-Item -ItemType Directory -Force -Path (Join-Path $root $logsRel) | Out-Null
    Set-Content -Path $buildLog -Value "# Task 16 pipeline $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -Encoding UTF8

    if (-not (Test-Path $Uv4Path)) { Emit "FAIL: UV4 not found: $Uv4Path"; exit 1 }
    if (-not (Test-Path $Gcc))      { Emit "FAIL: gcc not found: $Gcc"; exit 1 }

    # ---- Stage 0: Python 工具单测 ----
    Emit ''
    Emit '==== Stage 0: python tool unit tests ===='
    $r = Invoke-Py @('tools\test_check_memory_layout.py')
    if ($r[0] -ne 0) { Emit 'FAIL: test_check_memory_layout.py'; exit 1 }
    $r = Invoke-Py @('-m', 'unittest', 'tools.test_iap_tools')
    if ($r[0] -ne 0) { Emit 'FAIL: tools.test_iap_tools'; exit 1 }
    Emit 'Stage 0: PASS (77 + 26 unit tests)'
    $summary.Add('tool unit tests: PASS')

    # ---- Stage 1: 全量 mock 回归 ----
    Emit ''
    Emit '==== Stage 1: host mock full regression (13 suites) ===='
    if (-not $SkipMock) {
        $mockScript = Join-Path $root 'mock\freertos\run-mock-iap-stop.ps1'
        $prev = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        try {
            $mockOut = & powershell -NoProfile -ExecutionPolicy Bypass -File $mockScript -Suite all -Gcc $Gcc 2>&1
            $mockRc = $LASTEXITCODE
        } finally {
            $ErrorActionPreference = $prev
        }
        foreach ($l in $mockOut) { Emit ("{0}" -f $l) }
        if ($mockRc -ne 0) { Emit "FAIL: mock regression rc=$mockRc"; exit 1 }
    } else {
        Emit '(skipped by -SkipMock: reuse existing suite logs)'
    }
    if (-not (Merge-MockLogs)) { Emit 'FAIL: mock merge/gate'; exit 1 }
    $summary.Add('mock: PASS')

    # ---- Stage 2: FINAL_AUDIT 重建 + 门禁 ----
    Emit ''
    Emit '==== Stage 2: FINAL_AUDIT rebuild (Boot_SizeAudit / App_SizeAudit) ===='
    if (-not (Invoke-KeilBuild 'USER\boot_audit.uvprojx' 'Boot_SizeAudit' 'OBJ_BOOT_AUDIT')) { exit 1 }
    if (-not (Invoke-KeilBuild 'USER\app_audit.uvprojx'  'App_SizeAudit'  'OBJ_APP_AUDIT'))  { exit 1 }
    $summary.Add('Boot_SizeAudit: 0 Error(s), 0 Warning(s)')
    $summary.Add('App_SizeAudit: 0 Error(s), 0 Warning(s)')

    $r = Invoke-Py @('tools\iap_stop_gates.py', 'audit-gates',
        '--layout', $layoutH,
        '--boot-map', 'OBJ_BOOT_AUDIT\boot.map',
        '--boot-bin',  'OBJ_BOOT_AUDIT\boot.bin',
        '--app-map',  'OBJ_APP_AUDIT\range_hood_app.map',
        '--app-bin',  'OBJ_APP_AUDIT\range_hood_app.bin',
        '--label', 'FINAL_AUDIT')
    if ($r[0] -ne 0) { Emit 'FAIL: FINAL_AUDIT gates'; exit 1 }
    $summary.Add('FINAL_AUDIT gates: PASS')

    # ---- Stage 3: 主工程重建（三 APP 工程之一） ----
    Emit ''
    Emit '==== Stage 3: main project rebuild (project.uvprojx) ===='
    if (-not (Invoke-KeilBuild 'USER\project.uvprojx' 'project' 'OBJ')) { exit 1 }
    $summary.Add('project (main): 0 Error(s), 0 Warning(s)')

    # ---- Stage 4/5: 最终布局构建两遍（收敛） ----
    Emit ''
    Emit '==== Stage 4: provisional rebuild pass 1 (final layout) ===='
    if (-not (Invoke-KeilBuild 'USER\boot_cb_provisional.uvprojx' 'Boot_CB_Provisional' 'OBJ_BOOT_PROV')) { exit 1 }
    if (-not (Invoke-KeilBuild 'USER\app_cb_provisional.uvprojx'  'App_CB_Provisional'  'OBJ_APP_PROV'))  { exit 1 }
    $r1 = Invoke-Py @('tools\iap_stop_gates.py', 'audit-gates',
        '--layout', $layoutH,
        '--boot-map', 'OBJ_BOOT_PROV\boot.map',
        '--boot-bin',  'OBJ_BOOT_PROV\boot.bin',
        '--app-map',  'OBJ_APP_PROV\range_hood_app.map',
        '--app-bin',  'OBJ_APP_PROV\range_hood_app.bin',
        '--label', 'PROV_PASS1')
    if ($r1[0] -ne 0) { Emit 'FAIL: PROV_PASS1 gates'; exit 1 }
    $c1 = (@($r1[1]) | Where-Object { ("{0}" -f $_) -match '^LAYOUT_COMPUTED' } | Select-Object -Last 1)
    Emit "convergence pass1: $c1"
    $summary.Add('Boot_CB_Provisional: 0 Error(s), 0 Warning(s)')
    $summary.Add('App_CB_Provisional: 0 Error(s), 0 Warning(s)')

    Emit ''
    Emit '==== Stage 5: provisional rebuild pass 2 (convergence) ===='
    if (-not (Invoke-KeilBuild 'USER\boot_cb_provisional.uvprojx' 'Boot_CB_Provisional' 'OBJ_BOOT_PROV')) { exit 1 }
    if (-not (Invoke-KeilBuild 'USER\app_cb_provisional.uvprojx'  'App_CB_Provisional'  'OBJ_APP_PROV'))  { exit 1 }
    $r2 = Invoke-Py @('tools\iap_stop_gates.py', 'audit-gates',
        '--layout', $layoutH,
        '--boot-map', 'OBJ_BOOT_PROV\boot.map',
        '--boot-bin',  'OBJ_BOOT_PROV\boot.bin',
        '--app-map',  'OBJ_APP_PROV\range_hood_app.map',
        '--app-bin',  'OBJ_APP_PROV\range_hood_app.bin',
        '--label', 'PROV_PASS2')
    if ($r2[0] -ne 0) { Emit 'FAIL: PROV_PASS2 gates'; exit 1 }
    $c2 = (@($r2[1]) | Where-Object { ("{0}" -f $_) -match '^LAYOUT_COMPUTED' } | Select-Object -Last 1)
    Emit "convergence pass2: $c2"
    if ("$c1" -ne "$c2") {
        Emit 'FAIL: layout not converged (two passes differ)'
        exit 1
    }
    Emit 'layout_converged: PASS (two consecutive builds identical)'
    $summary.Add('layout_converged: PASS')

    # ---- Stage 6: check_memory_layout --final ----
    Emit ''
    Emit '==== Stage 6: check_memory_layout --final ===='
    $r = Invoke-Py @('tools\check_memory_layout.py',
        '--layout', $layoutH,
        '--boot-proj', 'USER\boot_cb_provisional.uvprojx',
        '--boot-map', 'OBJ_BOOT_PROV\boot.map',
        '--boot-bin',  'OBJ_BOOT_PROV\boot.bin',
        '--app-proj', 'USER\app_cb_provisional.uvprojx',
        '--app-map',  'OBJ_APP_PROV\range_hood_app.map',
        '--app-bin',  'OBJ_APP_PROV\range_hood_app.bin',
        '--app-audit-bin', 'OBJ_APP_AUDIT\range_hood_app.bin',
        '--final')
    if ($r[0] -ne 0) { Emit 'FAIL: layout check --final'; exit 1 }
    $layoutLog = Join-Path (Join-Path $root $logsRel) 'iap-stop-layout.log'
    $layoutLines = @($r[1]) | ForEach-Object { "{0}" -f $_ }
    [IO.File]::WriteAllLines($layoutLog, [string[]]$layoutLines, [Text.UTF8Encoding]::new($false))
    Emit "layout log: $logsRel\iap-stop-layout.log"
    $summary.Add('layout --final: PASS')
    $summary.Add('layout_provisional: 0')

    # ---- Stage 7: 旧地址扫描 ----
    Emit ''
    Emit '==== Stage 7: stale address scan ===='
    $r = Invoke-Py @('tools\iap_stop_gates.py', 'stale-scan',
        'BSP', 'APP_TASK', 'SYSTEM', 'USER', 'CONFIG', 'BOOT')
    if ($r[0] -ne 0) { Emit 'FAIL: stale-scan'; exit 1 }
    $summary.Add('stale-scan: PASS')

    # ---- Stage 8: .iap 打包 + round-trip ----
    Emit ''
    Emit '==== Stage 8: .iap pack + round-trip ===='
    $r = Invoke-Py @('tools\iap_pack.py',
        '--input', 'OBJ_APP_PROV\range_hood_app.bin',
        '--output', 'OBJ_APP_PROV\range_hood_app.iap',
        '--layout', $layoutH)
    if ($r[0] -ne 0) { Emit 'FAIL: iap_pack'; exit 1 }
    $r = Invoke-Py @('tools\iap_stop_gates.py', 'roundtrip',
        '--iap', 'OBJ_APP_PROV\range_hood_app.iap',
        '--bin', 'OBJ_APP_PROV\range_hood_app.bin',
        '--layout', $layoutH)
    if ($r[0] -ne 0) { Emit 'FAIL: iap round-trip'; exit 1 }
    $summary.Add('package round-trip: PASS')

    # ---- 汇总 ----
    Emit ''
    Emit '==== Task 16 pipeline summary ===='
    foreach ($s in $summary) { Emit $s }
    Emit 'overall: PASS'
    exit 0
}
finally {
    Pop-Location
}
