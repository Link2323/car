param([string]$PortName = 'COM3', [int]$Seconds = 15)

# 仅接收测距调试帧，不向小车发送命令；兼容 Windows PowerShell 5.1。
$port = New-Object System.IO.Ports.SerialPort $PortName,9600,'None',8,'One'
$port.ReadTimeout = 500
$directions = @('front', 'left', 'right')
$validCount = @(0, 0, 0)
$faultCount = @(0, 0, 0)
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
            if ($line -match '^tof:(-?\d+,\d+,\d+,\d+,){2}-?\d+,\d+,\d+,\d+$') {
                $values = $line.Substring(4).Split(',')
                # 前、左、右各四个通道；分别统计，避免某一路正常掩盖另外两路故障。
                for ($sensor = 0; $sensor -lt 3; $sensor++) {
                    $offset = $sensor * 4
                    $distance = [long]$values[$offset]
                    $valid = [long]$values[$offset + 1]
                    $state = [long]$values[$offset + 2]
                    $raw = [long]$values[$offset + 3]
                    if ($valid -eq 1 -and $state -eq 0 -and $raw -eq 11 -and
                        $distance -ge 1 -and $distance -le 2000) {
                        $validCount[$sensor]++
                    } elseif ($valid -eq 0 -and $state -ge 1 -and $state -le 5 -and
                              $distance -eq -1 -and $raw -le 255) {
                        $faultCount[$sensor]++
                    } else {
                        $malformedCount++
                    }
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
for ($sensor = 0; $sensor -lt 3; $sensor++) {
    Write-Output ("TOF {0}: valid={1} invalid={2}" -f $directions[$sensor], $validCount[$sensor], $faultCount[$sensor])
}
if ($malformedCount -gt 0) { throw "Malformed or inconsistent TOF channels: $malformedCount" }
# 实物验收时三个方向都应放置量程内目标；某一路一直无效则明确报出方向。
for ($sensor = 0; $sensor -lt 3; $sensor++) {
    if ($validCount[$sensor] -eq 0) {
        throw ("No valid {0} distance received; inspect TOF state, target and wiring." -f $directions[$sensor])
    }
}
Write-Output 'PASS: valid front/left/right distances received; inspect invalid counts separately.'
