param([string]$PortName = 'COM6', [int]$Seconds = 8)

# 只接收串口数据，不发送控制指令；需使用 Windows PowerShell 运行。
$port = New-Object System.IO.Ports.SerialPort $PortName,9600,'None',8,'One'
$port.ReadTimeout = 500
$invalidSeen = $false
$validSeen = $false
try {
    $port.Open()
    $until = [DateTime]::UtcNow.AddSeconds($Seconds)
    # 在限定时间内收集完整行，以真实固件输出判断角速度是否有效。
    while ([DateTime]::UtcNow -lt $until) {
        try {
            $line = $port.ReadLine().Trim()
            Write-Output $line
            # FireWater 双通道帧：左轮、右轮计数率，单位 counts/s。
            if ($line -match '^encoder:-?\d+,-?\d+$') { $validSeen = $true }
            elseif ($line.Length -gt 0) { $invalidSeen = $true }
        } catch [System.TimeoutException] {
            # 无新行时继续接收，等待板端下一次周期消息。
        }
    }
} finally {
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
if ($invalidSeen) { throw '收到不符合 FireWater 编码器双通道格式的串口消息。' }
if (-not $validSeen) { throw '未收到有效编码器数据，不能判定通过。' }
Write-Output 'PASS: 收到有效编码器数据，采集期间未出现格式错误。'
