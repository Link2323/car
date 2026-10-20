param([string]$PortName = 'COM3', [int]$Seconds = 15)

# 仅接收测距调试帧，不向小车发送命令；兼容 Windows PowerShell 5.1。
$port = New-Object System.IO.Ports.SerialPort $PortName,9600,'None',8,'One'
$port.ReadTimeout = 500
$validCount = 0
$faultCount = 0
$malformedCount = 0
try {
    $port.Open()
    $until = [DateTime]::UtcNow.AddSeconds($Seconds)
    # 第一行可能从帧中间开始；先丢弃至首个换行，之后检查完整消息。
    $synchronized = $false
    while ([DateTime]::UtcNow -lt $until) {
        try {
            $line = $port.ReadLine().Trim()
            if (-not $synchronized) { $synchronized = $true; continue }
            Write-Output $line
            if ($line -match '^tof:(-?\d+),([01]),([0-5]),(\d+)$') {
                $distance = [int]$Matches[1]
                $valid = [int]$Matches[2]
                $state = [int]$Matches[3]
                $raw = [int]$Matches[4]
                # 有效帧必须同时满足距离范围、诊断状态与器件内部有效码。
                if ($valid -eq 1 -and $state -eq 0 -and $raw -eq 11 -and
                    $distance -ge 1 -and $distance -le 2000) {
                    $validCount++
                } elseif ($valid -eq 0 -and $state -ne 0 -and $distance -eq -1 -and $raw -le 255) {
                    $faultCount++
                } else {
                    $malformedCount++
                }
            } elseif ($line.Length -gt 0) {
                $malformedCount++
            }
        } catch [System.TimeoutException] {
            # 暂无完整行时继续等下一帧，截止后统一判断是否收到有效测距。
        }
    }
} finally {
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
Write-Output "TOF frames: valid=$validCount invalid=$faultCount malformed=$malformedCount"
if ($malformedCount -gt 0) { throw 'Malformed or inconsistent TOF frames received.' }
if ($validCount -eq 0) { throw 'No valid front distance received; inspect TOF state and wiring.' }
# 无目标时的无效回波也属于正常协议帧，但必须单独计数，不能掩盖故障。
Write-Output 'PASS: valid front distance received; inspect invalid frame count separately.'
