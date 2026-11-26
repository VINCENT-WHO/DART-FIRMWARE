# 四路舵机安装零位工具：只在调用函数后，才会通过串口控制硬件。
# 四路均输出1500微秒，便于把舵片安装到机械0度位置。
# 不需要重新编译或烧录；复用已有标定工具的握手、续期和退出清理。
function Start-AllServoNeutral {
    [CmdletBinding()]
    param(
        [string]$Port = 'COM6',
        # 保持期间再安装、对齐舵片；默认60秒，可按需要缩短。
        [ValidateRange(1, 60)] [int]$HoldSeconds = 60,
        # 模拟模式只打印计划，不打开串口，不让舵机动作。
        [switch]$Simulate
    )

    $ErrorActionPreference = 'Stop'
    $calibrationToolPath = 'C:\RMDART\dart_firmware\tools\servo_calibrate.ps1'
    . ([scriptblock]::Create([IO.File]::ReadAllText($calibrationToolPath, [Text.Encoding]::UTF8)))

    Write-Host "四路目标：1=1500、2=1500、3=1500、4=1500微秒；保持$HoldSeconds 秒。" -ForegroundColor Cyan
    Write-Host '先等待舵机停稳，再对齐舵片；按Ctrl+C可提前结束。'

    # 通过四路参数明确给出每个通道的目标，复用标定工具的通信流程。
    # 标定工具每500毫秒发送续期命令，正常结束或中断时尝试关闭PWM。
    Start-ServoCalibration -PulsesUs 1500,1500,1500,1500 -HoldSeconds $HoldSeconds -Port $Port -Simulate:$Simulate
}
