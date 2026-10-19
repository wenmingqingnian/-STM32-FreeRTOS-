# IAP+STOP 计划 mock 统一入口（Task 4 起，host gcc，无需硬件）。
# 按 -Suite 选择测试集；后续 Task 的 mock 套件（低功耗等）挂到本入口。
#   crc32    : Task 4 流式 CRC32（编译未修改的 BSP\CRC32\crc32.c）
#   meta     : Task 5 双页元数据 + fake_flash 断电注入
#（编译未修改的 BSP\IAP\iap_meta.c + BSP\CRC32\crc32.c）
#   protocol : Task 6 帧协议解析 + 升级会话状态机
#（编译未修改的 BOOT\boot_protocol.c + BSP\CRC32\crc32.c，
#               不链接任何 Flash 后端——"不访问 Flash"由链接集本身证明）
#   flash    : Task 7 Boot 专用 Flash 写入器
#（编译未修改的 BOOT\boot_flash.c + BSP\CRC32\crc32.c，
#               Flash 物理行为经 fake_flash.c 注入：页擦除/1->0 编程/断电）
#   uartdma  : Task 8 Boot UART DMA 环形接收（本套件为该模块**唯一**单测）
#（编译未修改的 BOOT\boot_uart_dma.c；不链接任何 FreeRTOS/SPL 源，
#               "ISR 不调 FreeRTOS API"由链接集证明；DMA/USART 寄存器由
#               stub\stm32f10x.h 建模、测试体提供 SPL 函数体。
#               含"逐页轮询不变量"断言——锁住 boot_update.c 的逐页 Poll 不可删）
#   image    : Task 9 镜像校验链（状态/尺寸/基址/MSP/PC/整镜像 CRC）
#（编译未修改的 BOOT\boot_image.c + BSP\CRC32\crc32.c，
#               只读后端接 fake_flash.c——校验只读不写的契约由链接集证明）
#   lowpower : Task 11 板型感知 STOP 决策 + 1000Hz<->1024Hz tick 换算
#（编译未修改的 BSP\LOWPOWER\lowpower_policy.c /
#               lowpower_math.c——纯函数只读快照、不读寄存器/全局，
#               "不访问硬件"由链接集证明；NO_LSE 不走 RTC 换算
#               由决策面测试 + 本链接集无 LSI 换算路径共同证明）
#   rtc      : Task 12 RTC 时间基准（同一测试源编译两次，L2 双模式）
#              rtc-lse : -I board_lse 影子板型 -> 完整 IO 注入实现
#（首启序列/快速路径/LSE 超时/稳定读/闹钟顺序）
#              rtc-stub: -I board_no_lse 影子板型 -> 零硬件 stub
#（所有 API NOT_SUPPORTED 且 IO 计数==0：
#                         非 LSE 板型不触碰 PC14/PC15 的链接集+运行时双证明）
#   exti     : Task 12 EXTI-only STOP 后端 + 公共时钟恢复
#（编译未修改的 BSP\LOWPOWER\exti_stop_backend.c +
#               system_clock_resume.c——不链接 rtc_timebase.c 与任何
#               FreeRTOS 源："不设 RTC Alarm、不调 vTaskStepTick"
#               由接口形状与链接集双证明；vTaskStepTick 桩计数==0）
#   key      : Task 13 按键 EXTI 唤醒（编译未修改的 BSP\KEY\key_exti.c +
#              key.c——不链接任何 FreeRTOS 源："ISR 只通知、不延时、不
#              printf、不判按键"由 IO 形状 + 链接集双证明；抖动合并/
#              睡眠边沿不丢/扫描阻塞策略/优先级契约为运行时证明）
#   health   : Plan-A Step5 心跳监督 + Task 14 required mask 切换/等待策略
#（编译未修改的 APP_TASK\task_health.c + BSP\RESET\reset_cause.c）
#   standby  : Task 14 待机阻塞决策 + IWDG budget + EXTI-only 拒绝
#（编译未修改的 APP_TASK\standby_policy.c + task_health.c +
#               BSP\LOWPOWER\lowpower_policy.c——不链接任何 FreeRTOS 源，
#               待机阻塞语义由纯函数决策表 + 事件位常量单一来源锁定）
#   all      : 当前已实现的所有套件
# L2 evidence: logs saved to docs/design-evidence/logs/.
param(
    [string]$Gcc = 'D:\QT\Tools\mingw730_64\bin\gcc.exe',
    [ValidateSet('crc32', 'meta', 'protocol', 'flash', 'uartdma', 'image', 'lowpower', 'rtc', 'exti', 'key', 'health', 'standby', 'all')]
    [string]$Suite = 'all',
    [string]$OutLog = ''
)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$root = Resolve-Path '..\..'

if (-not (Test-Path $Gcc)) {
    throw "gcc not found: $Gcc"
}

function Invoke-Suite {
    param([string]$Name, [string[]]$Sources, [string[]]$Includes,
          [string]$Exe, [string]$Log, [string[]]$ExtraArgs = @())

    $incArgs = @()
    foreach ($i in $Includes) { $incArgs += "-I$i" }
    & $Gcc -Wall -Wextra -std=c99 @incArgs @ExtraArgs -o $Exe @Sources
    if ($LASTEXITCODE -ne 0) { throw "suite '$Name' compile failed: $LASTEXITCODE" }

    $outDir = Split-Path $Log -Parent
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
    Write-Output "==> suite $Name (log: $Log)"
    & $Exe | Tee-Object -FilePath $Log
    if ($LASTEXITCODE -ne 0) {
        throw "suite '$Name' FAILED"
    }
}

if ($Suite -eq 'crc32' -or $Suite -eq 'all') {
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-crc32-mock.log' }
    Invoke-Suite -Name 'crc32' -Exe "$env:TEMP\iap-stop-crc32-mock.exe" -Log $log `
        -Sources @('.\test_crc32_stream.c', "$root\BSP\CRC32\crc32.c") `
        -Includes @('stub', "$root\BSP\CRC32")
}

if ($Suite -eq 'meta' -or $Suite -eq 'all') {
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-meta-mock.log' }
    Invoke-Suite -Name 'meta' -Exe "$env:TEMP\iap-stop-meta-mock.exe" -Log $log `
        -Sources @('.\test_iap_meta.c', '.\fake_flash.c',
                   "$root\BSP\IAP\iap_meta.c", "$root\BSP\CRC32\crc32.c") `
        -Includes @('stub', "$root\BSP\IAP", "$root\BSP\CRC32", "$root\CONFIG")
}

if ($Suite -eq 'protocol' -or $Suite -eq 'all') {
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-protocol-mock.log' }
    Invoke-Suite -Name 'boot-protocol' -Exe "$env:TEMP\iap-stop-protocol-mock.exe" -Log $log `
        -Sources @('.\test_boot_protocol.c', "$root\BOOT\boot_protocol.c",
                   "$root\BSP\CRC32\crc32.c") `
        -Includes @('stub', "$root\BOOT", "$root\BSP\CRC32", "$root\CONFIG")
}

if ($Suite -eq 'flash' -or $Suite -eq 'all') {
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-flash-mock.log' }
    Invoke-Suite -Name 'boot-flash' -Exe "$env:TEMP\iap-stop-flash-mock.exe" -Log $log `
        -Sources @('.\test_boot_flash.c', '.\fake_flash.c',
                   "$root\BOOT\boot_flash.c", "$root\BSP\CRC32\crc32.c") `
        -Includes @('stub', "$root\BOOT", "$root\BSP\CRC32", "$root\CONFIG")
}

if ($Suite -eq 'uartdma' -or $Suite -eq 'all') {
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-uartdma-mock.log' }
    # -Wno-pointer-to-int-cast：生产代码把指针存进 u32 的 DMA_InitTypeDef 字段
    # ((uint32_t)s_ring)，在 64 位宿主上必然告警；这是裸机目标的固有写法，
    # 不是缺陷。测试体因此改用 Pending() 空环返回值取环基址（见测试文件注释）。
    Invoke-Suite -Name 'boot-uart-dma' -Exe "$env:TEMP\iap-stop-uartdma-mock.exe" -Log $log `
        -Sources @('.\test_boot_uart_dma.c', "$root\BOOT\boot_uart_dma.c") `
        -Includes @('stub', "$root\BOOT", "$root\CONFIG") `
        -ExtraArgs @('-Wno-pointer-to-int-cast')
}

if ($Suite -eq 'image' -or $Suite -eq 'all') {
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-image-mock.log' }
    Invoke-Suite -Name 'boot-image' -Exe "$env:TEMP\iap-stop-image-mock.exe" -Log $log `
        -Sources @('.\test_boot_image.c', '.\fake_flash.c',
                   "$root\BOOT\boot_image.c", "$root\BSP\CRC32\crc32.c") `
        -Includes @('stub', "$root\BOOT", "$root\BSP\IAP", "$root\BSP\CRC32", "$root\CONFIG")
}

if ($Suite -eq 'lowpower' -or $Suite -eq 'all') {
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-lowpower-policy-mock.log' }
    Invoke-Suite -Name 'lowpower-policy' -Exe "$env:TEMP\iap-stop-lowpower-policy-mock.exe" -Log $log `
        -Sources @('.\test_lowpower_policy.c', "$root\BSP\LOWPOWER\lowpower_policy.c") `
        -Includes @('stub', "$root\BSP\LOWPOWER")

    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-lowpower-math-mock.log' }
    Invoke-Suite -Name 'lowpower-math' -Exe "$env:TEMP\iap-stop-lowpower-math-mock.exe" -Log $log `
        -Sources @('.\test_lowpower_math.c', "$root\BSP\LOWPOWER\lowpower_math.c") `
        -Includes @('stub', "$root\BSP\LOWPOWER")
}

if ($Suite -eq 'rtc' -or $Suite -eq 'all') {
    # 双模式：同一测试源 + 未修改的 rtc_timebase.c，仅板型头文件解析不同。
    # 注意 rtc-lse 不含 CONFIG 路径：board_profile.h 必须来自 board_lse 影子。
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-rtc-lse-mock.log' }
    Invoke-Suite -Name 'rtc-lse' -Exe "$env:TEMP\iap-stop-rtc-lse-mock.exe" -Log $log `
        -Sources @('.\test_rtc_timebase.c', "$root\BSP\LOWPOWER\rtc_timebase.c") `
        -Includes @('stub', 'board_lse', "$root\BSP\LOWPOWER")

    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-rtc-stub-mock.log' }
    Invoke-Suite -Name 'rtc-stub' -Exe "$env:TEMP\iap-stop-rtc-stub-mock.exe" -Log $log `
        -Sources @('.\test_rtc_timebase.c', "$root\BSP\LOWPOWER\rtc_timebase.c") `
        -Includes @('stub', 'board_no_lse', "$root\BSP\LOWPOWER")
}

if ($Suite -eq 'exti' -or $Suite -eq 'all') {
    # 链接集证明：不链接 rtc_timebase.c（不设 RTC Alarm）、不链接任何
    # FreeRTOS 源（tick 冻结不补偿）；test_exti_stop_backend.c 的
    # vTaskStepTick 桩计数==0 提供运行时第二重证明。
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-exti-stop-mock.log' }
    Invoke-Suite -Name 'exti-stop' -Exe "$env:TEMP\iap-stop-exti-stop-mock.exe" -Log $log `
        -Sources @('.\test_exti_stop_backend.c', "$root\BSP\LOWPOWER\exti_stop_backend.c",
                   "$root\BSP\LOWPOWER\system_clock_resume.c") `
        -Includes @('stub', "$root\BSP\LOWPOWER")
}

if ($Suite -eq 'health' -or $Suite -eq 'all') {
    # Plan-A Step5 遗留套件挂入统一入口（原 test_task_health.c 无 runner）：
    # 编译未修改的 task_health.c + reset_cause.c；Task 14 扩展 standby mask。
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-task-health-mock.log' }
    Invoke-Suite -Name 'task-health' -Exe "$env:TEMP\iap-stop-task-health-mock.exe" -Log $log `
        -Sources @('.\test_task_health.c', "$root\APP_TASK\task_health.c",
                   "$root\BSP\RESET\reset_cause.c") `
        -Includes @('stub', "$root\APP_TASK", "$root\BSP\RESET", "$root\BSP\WATCHDOG")
}

if ($Suite -eq 'standby' -or $Suite -eq 'all') {
    # 链接集证明：不链接任何 FreeRTOS 源（等待策略为纯函数决策表）；
    # standby_policy.c / task_health.c / lowpower_policy.c 均未修改编译。
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-standby-blocking-mock.log' }
    Invoke-Suite -Name 'standby-blocking' -Exe "$env:TEMP\iap-stop-standby-blocking-mock.exe" -Log $log `
        -Sources @('.\test_standby_blocking.c', "$root\APP_TASK\standby_policy.c",
                   "$root\APP_TASK\task_health.c", "$root\BSP\LOWPOWER\lowpower_policy.c") `
        -Includes @('stub', "$root\APP_TASK", "$root\BSP\WATCHDOG", "$root\BSP\LOWPOWER")
}

if ($Suite -eq 'key' -or $Suite -eq 'all') {
    # 链接集证明：不链接任何 FreeRTOS 源（通知经 IO 回调注入）；
    # key.c 消抖状态机 + key_exti.c ISR 体均未修改编译。
    $log = if ($OutLog) { $OutLog }
           else { '..\..\docs\design-evidence\logs\iap-stop-key-wakeup-mock.log' }
    Invoke-Suite -Name 'key-wakeup' -Exe "$env:TEMP\iap-stop-key-wakeup-mock.exe" -Log $log `
        -Sources @('.\test_key_wakeup.c', "$root\BSP\KEY\key_exti.c",
                   "$root\BSP\KEY\key.c") `
        -Includes @('stub', "$root\BSP\KEY", "$root\CONFIG")
}

Write-Output "mock suites [$Suite]: all passed"
exit 0
