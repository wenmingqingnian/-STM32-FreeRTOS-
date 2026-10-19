# build-size-audit.ps1 -- Boot/App 尺寸审计构建入口
#
# 背景：UV4 5.39 命令行在单个 uvprojx 内构建多 Target 时，会把所有 Target 的
#       Groups 合并进每个 Target 的编译列表（同路径文件重复编译为 *_1.o 并在
#       链接时报符号重复）。因此 Boot/App 审计拆为两个独立工程文件（与 Keil
#       官方 RTX_Lib_CM 库工程的做法一致），每个工程只含一个 Target：
#         - USER\boot_audit.uvprojx -> Boot_SizeAudit -> OBJ_BOOT_AUDIT\boot.{axf,map,bin}
#         - USER\app_audit.uvprojx  -> App_SizeAudit  -> OBJ_APP_AUDIT\range_hood_app.{axf,map,bin}
#       日常主工程 USER\project.uvprojx 保持单 Target（project）不受影响。
#
# 功能：
#   1. 清空 OBJ_BOOT_AUDIT / OBJ_APP_AUDIT 输出目录（防止上一次构建的残留混入）；
#   2. 用 UV4 命令行 Rebuild 两个审计 Target：
#        - Boot_SizeAudit -> OBJ_BOOT_AUDIT\boot.{axf,map,bin}
#        - App_SizeAudit  -> OBJ_APP_AUDIT\range_hood_app.{axf,map,bin}
#   3. 校验每个 Target 都是 "0 Error(s), 0 Warning(s)"；
#   4. 调用 extract_keil_size.py 提取 load_region_size / bin_size / total_rw，
#      输出标记为 PRELIMINARY（分区布局未冻结）。
#
# 用法（在仓库根目录执行）：
#   powershell -ExecutionPolicy Bypass -File tools\build-size-audit.ps1
#
# 退出码：0 全部通过；1 任一构建失败 / 有错误或警告 / 尺寸提取失败。

param(
    [string]$Uv4Path = 'D:\Keilv5\UV4\UV4.exe',
    [string]$PythonPath = 'python'
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

function Invoke-KeilBuild {
    param([string]$ProjectFile, [string]$Target, [string]$ObjDir)

    $objAbs = Join-Path $root $ObjDir
    if (Test-Path $objAbs) {
        Remove-Item -Path $objAbs -Recurse -Force
    }
    New-Item -ItemType Directory -Path $objAbs | Out-Null

    $projAbs = Join-Path $root $ProjectFile
    if (-not (Test-Path $projAbs)) {
        Write-Host "FAIL: project file not found: $projAbs"
        return $false
    }

    $logAbs = Join-Path $objAbs 'build.log'
    Write-Host "==> Build '$Target' in $ProjectFile (log: $logAbs)"
    & $Uv4Path -b $projAbs -t $Target -j0 -o $logAbs | Out-Null
    $rc = $LASTEXITCODE

    if (-not (Test-Path $logAbs)) {
        Write-Host "FAIL: build log not created: $logAbs"
        return $false
    }
    $logText = Get-Content $logAbs -Raw
    if ($logText -match '(\d+) Error\(s\), (\d+) Warning\(s\)') {
        $errCount = $Matches[1]; $warnCount = $Matches[2]
    } else {
        Write-Host "FAIL: no error/warning summary found in $logAbs"
        Write-Host $logText
        return $false
    }

    if ($rc -ne 0 -or $errCount -ne '0' -or $warnCount -ne '0') {
        Write-Host "FAIL: target '$Target' rc=$rc, $errCount Error(s), $warnCount Warning(s)"
        Write-Host $logText
        return $false
    }
    Write-Host "PASS: target '$Target' 0 Error(s), 0 Warning(s) (rc=$rc)"
    return $true
}

# UV4 工作目录为仓库根目录（工程路径与日志路径均相对根目录解析）
Push-Location $root
try {
    if (-not (Test-Path $Uv4Path)) {
        Write-Host "FAIL: UV4 not found: $Uv4Path"
        exit 1
    }

    $okBoot = Invoke-KeilBuild -ProjectFile 'USER\boot_audit.uvprojx' -Target 'Boot_SizeAudit' -ObjDir 'OBJ_BOOT_AUDIT'
    $okApp  = Invoke-KeilBuild -ProjectFile 'USER\app_audit.uvprojx'  -Target 'App_SizeAudit'  -ObjDir 'OBJ_APP_AUDIT'

    if (-not ($okBoot -and $okApp)) {
        exit 1
    }

    Write-Host ''
    Write-Host '==> Boot size audit'
    & $PythonPath (Join-Path $root 'tools\extract_keil_size.py') `
        --map 'OBJ_BOOT_AUDIT\boot.map' --bin 'OBJ_BOOT_AUDIT\boot.bin' --label 'boot'
    if ($LASTEXITCODE -ne 0) { exit 1 }

    Write-Host ''
    Write-Host '==> App size audit'
    & $PythonPath (Join-Path $root 'tools\extract_keil_size.py') `
        --map 'OBJ_APP_AUDIT\range_hood_app.map' --bin 'OBJ_APP_AUDIT\range_hood_app.bin' --label 'range_hood_app'
    if ($LASTEXITCODE -ne 0) { exit 1 }

    Write-Host ''
    Write-Host 'All size-audit targets built clean (0 Error(s), 0 Warning(s)).'
    Write-Host 'Numbers above are PRELIMINARY: final partition layout is frozen in later tasks.'
    exit 0
}
finally {
    Pop-Location
}
