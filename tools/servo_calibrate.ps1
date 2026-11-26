# 舵偏角标定辅助工具：在电脑上运行，不编译、不烧录、不修改固件。
# 指定通道和脉宽后保持指定秒数，期间续期，最后关闭四路 PWM。
# 在VSCode运行任务“准备舵机标定（仅载入，不动作）”，把函数载入终端。
# 然后调用：Start-ServoCalibration -Channel 1 -PulseUs 1520 -HoldSeconds 30
# 四路分别指定：Start-ServoCalibration -PulsesUs 1500,1520,1480,1550 -HoldSeconds 5
# -Simulate 只打印计划，不打开串口，不会驱动硬件。
# -CheckOnly 只查询连接和固件状态，不设置脉宽、不关闭或开启PWM。
function Start-ServoCalibration {
[CmdletBinding(DefaultParameterSetName = 'Single')]
param(
    [Parameter(Mandatory = $true, ParameterSetName = 'Single')]
    [ValidateRange(1, 4)] [int]$Channel,
    # 按用户指定开放1000至2000微秒，与板端手动PWM命令范围一致。
    # 这是指令范围，装配后的实际可用行程由逐点测量确认。
    [Parameter(ParameterSetName = 'Single')]
    [ValidateRange(1000, 2000)] [int]$PulseUs = 1500,
    # 四个数依次对应1、2、3、4号通道；四路模式不再使用Channel或NeutralUs。
    [Parameter(Mandatory = $true, ParameterSetName = 'All')]
    [ValidateCount(4, 4)]
    [ValidateRange(1000, 2000)] [int[]]$PulsesUs,
    [ValidateRange(1, 60)] [int]$HoldSeconds = 30,
    [string]$Port = 'COM6',
    # 未选通道使用各自已确认的保持值；未标定时仍按原先1500µs。
    [Parameter(ParameterSetName = 'Single')]
    [ValidateCount(4, 4)]
    [ValidateRange(1000, 2000)] [int[]]$NeutralUs = @(1500, 1500, 1500, 1500),
    [switch]$CheckOnly,
    [switch]$Simulate
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$servoConnection = $null
$pwmCleanupNeeded = $false

# 在打开串口前确定四路完整目标；两种用法互斥，避免参数含义不清。
if ($PSCmdlet.ParameterSetName -eq 'All') {
    $targetPulses = [int[]]$PulsesUs.Clone()
}
else {
    $targetPulses = [int[]]$NeutralUs.Clone()
    $targetPulses[$Channel - 1] = $PulseUs
}

function Send-CalibrationCommand([string]$Command, [string]$Expected) {
    if ($Simulate) {
        if ($Command -eq 'STATUS') {
            return 'DART F103 v=2.0 active=0 pwm=0'
        }
        if ($Command -eq 'H7 STATUS') {
            return 'H7: DART H750 v=2.0 state=DISARMED sd=1 frames=0 imu=0'
        }
        if ($Command -ne 'PWM KEEP') { Write-Host "[模拟，不发送] $Command" }
        return $Expected
    }

    # 逐字发送，避免当前板端在连续接收时丢字。ASCII命令以LF结束。
    foreach ($character in ($Command + "`n").ToCharArray()) {
        $servoConnection.Write([string]$character)
        Start-Sleep -Milliseconds 5
    }
    $replyTimer = [Diagnostics.Stopwatch]::StartNew()
    while ($replyTimer.Elapsed.TotalSeconds -lt 2) {
        try { $reply = $servoConnection.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }

        if (-not $reply) { continue }
        if ($Command -ne 'PWM KEEP') { Write-Host "$Command -> $reply" }
        if ($reply.StartsWith($Expected, [StringComparison]::Ordinal)) { return $reply }
        if ($reply.StartsWith('ERR ', [StringComparison]::Ordinal)) {
            throw "控制板拒绝命令：$reply"
        }
    }
    throw "命令未收到正确确认：$Command。停止测试。"
}

try {
    if (-not $Simulate) {
        $servoConnection = [System.IO.Ports.SerialPort]::new($Port, 115200)
        $servoConnection.NewLine = "`n"
        $servoConnection.ReadTimeout = 200
        $servoConnection.WriteTimeout = 1000
        $servoConnection.Open()
        $servoConnection.WriteLine('')
        Start-Sleep -Milliseconds 200
        $servoConnection.DiscardInBuffer()
    }

    $hello = 'CAL' + [Guid]::NewGuid().ToString('N').Substring(0, 6)
    $null = Send-CalibrationCommand "PING $hello" "PONG $hello"
    $status = Send-CalibrationCommand 'STATUS' 'DART F103'
    if ($status -notmatch 'v=2\.0(?:\s|$)') {
        throw '不是本工具对应的 F103 v2.0 固件，停止。'
    }

    if ($CheckOnly) {
        Write-Host "F103状态：$status"
        $h7Status = Send-CalibrationCommand 'H7 STATUS' 'H7: DART H750'
        Write-Host "H750状态：$h7Status"
        Write-Host '连接检查完成：没有发送PWM或启停采集命令，没有让舵机动作。' -ForegroundColor Green
        return
    }

    # 握手和型号确认完成后，才允许改变PWM；预检或连接失败不发送清理命令。
    $pwmCleanupNeeded = $true
    $null = Send-CalibrationCommand 'PWM OFF' 'OK PWM OFF'
    $null = Send-CalibrationCommand 'H7 AUTO OFF' 'OK H7 TX'
    Start-Sleep -Milliseconds 800
    $status = Send-CalibrationCommand 'STATUS' 'DART F103'
    if ($status -notmatch '(?:^|\s)active=0(?:\s|$)' -or
        $status -notmatch '(?:^|\s)pwm=0(?:\s|$)') {
        throw 'IMU采集或PWM尚未停止，请等待当前记录结束后再试。'
    }

    # 当前固件PWM ON会同时开启四路。先在输出关闭时逐路预装目标，
    # 收到四路设置确认后再共同开启，避免某路先回1500再跳到测量点。
    foreach ($index in 1..4) {
        $requested = $targetPulses[$index - 1]
        $null = Send-CalibrationCommand "PWM $index $requested" 'OK PWM SET'
    }
    $null = Send-CalibrationCommand 'PWM ON' 'OK PWM ON'
    Write-Host "正在保持：1=$($targetPulses[0])、2=$($targetPulses[1])、3=$($targetPulses[2])、4=$($targetPulses[3]) 微秒，共$HoldSeconds 秒。" -ForegroundColor Cyan
    if ($PSCmdlet.ParameterSetName -eq 'Single') {
        Write-Host "本次测量通道：$Channel；其他通道按NeutralUs给定值输出。"
    }
    Write-Host '读数需由量角器/照片得到，本工具不能读取真实舵角。'

    # 固件3秒没有新命令会关闭输出；每500ms续期，不改变目标脉宽。
    $holdTimer = [Diagnostics.Stopwatch]::StartNew()
    $nextKeepMs = 0
    while ($holdTimer.Elapsed.TotalSeconds -lt $HoldSeconds) {
        if ($holdTimer.ElapsedMilliseconds -ge $nextKeepMs) {
            $null = Send-CalibrationCommand 'PWM KEEP' 'OK KEEP'
            $nextKeepMs = $holdTimer.ElapsedMilliseconds + 500
        }
        Start-Sleep -Milliseconds 50
    }
}
finally {
    # 正常结束、超时或脚本异常均尝试关闭输出；进程意外退出仍有板端超时。
    if ($Simulate -and $pwmCleanupNeeded) {
        $null = Send-CalibrationCommand 'PWM OFF' 'OK PWM OFF'
    }
    elseif ($null -ne $servoConnection) {
        try {
            if ($servoConnection.IsOpen -and $pwmCleanupNeeded) {
                try { $null = Send-CalibrationCommand 'PWM OFF' 'OK PWM OFF' }
                catch { Write-Warning "未收到关闭确认，请检查连接；停止续期后固件应在3秒内关闭PWM。$($_.Exception.Message)" }
            }
        }
        finally {
            $servoConnection.Close()
            $servoConnection.Dispose()
        }
    }
}

Write-Host '本次测点结束。正常收到OK PWM OFF才表示已确认关闭输出；它不等于舵面回到0度。'
}
