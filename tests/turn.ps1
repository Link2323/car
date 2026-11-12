# 转向调试脚本：单条命令让小车执行左转90/右转90/掉头180或急停。
# 用法：
#   pwsh tests/turn.ps1 left90            # 左转 90 度（默认 COM14）
#   pwsh tests/turn.ps1 right90           # 右转 90 度
#   pwsh tests/turn.ps1 uturn             # 掉头（左旋 180 度）
#   pwsh tests/turn.ps1 stop              # 紧急停止
#   pwsh tests/turn.ps1 left90 -Port COM6 # 指定其他蓝牙串口
# 转向为相对角度（以当前朝向为基准），自动到位停机，无需心跳。
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [ValidateSet('left90', 'right90', 'uturn', 'stop')]
    [string]$Action,
    [string]$Port = 'COM14'
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.Encoding]::UTF8

# 动作到转向角的映射：正角左转（俯视逆时针），掉头固定左旋 180。
$angleByAction = @{
    left90  = 90
    right90 = -90
    uturn   = 180
    stop    = 0
}

# ---- 串口命令辅助 ----
function Wait-Ack($p, $id, $op, $timeoutMs) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalMilliseconds -lt $timeoutMs) {
        try {
            $l = $p.ReadLine().Trim("`r").Trim()
            if ($l -like "ack:$id,$op,*") { return $l }
        } catch {}
    }
    return $null
}

# 发送单条命令并等待回执；失败返回 $null。
function Send-Command($p, [string]$text, [int]$id, [int]$op, [int]$timeoutMs = 1500) {
    $p.Write("tune:$id,$op$text`n")
    return Wait-Ack $p $id $op $timeoutMs
}

$p = New-Object System.IO.Ports.SerialPort($Port, 9600, 'None', 8, 'One')
$p.ReadTimeout = 600
$p.Open()
try {
    # 链路确认：连续收到 5 帧遥测才继续，避免命令打进断链窗口丢失。
    $alive = 0
    $wait = [Diagnostics.Stopwatch]::StartNew()
    while ($alive -lt 5 -and $wait.Elapsed.TotalSeconds -lt 10.0) {
        try { $l = $p.ReadLine().Trim("`r").Trim(); if ($l) { $alive++ } } catch { $alive = 0 }
    }
    if ($alive -lt 5) { Write-Host "LINK UNSTABLE on $Port - abort"; exit 1 }

    # 事务号取当前秒数，每次运行天然不同，避免与上次执行撞号。
    $id = [int]([DateTimeOffset]::UtcNow.ToUnixTimeSeconds() % 900000000) + 100

    # 先停止清障：上次转向若超时锁存 FAULT，不清则新命令会被拒绝。
    $stopAck = Send-Command $p '' ($id + 1) 3
    if ($Action -eq 'stop') {
        Write-Host "stop ack: $stopAck"
        return
    }

    $angle = $angleByAction[$Action]
    $turnAck = Send-Command $p ",$angle" ($id + 2) 10
    Write-Host "$Action ($angle deg) ack: $turnAck"
    if (-not $turnAck -or $turnAck -notlike '*,0') {
        Write-Host 'TURN COMMAND REJECTED (1=error 2=busy 3=timeout)'
        exit 1
    }

    # 记录 z 轨迹直到转向完成：正常到位或超时（3s + 30ms/度 + 余量）。
    $timeoutS = 3.0 + [math]::Abs($angle) * 0.03 + 1.5
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $samples = New-Object System.Collections.Generic.List[double]
    $lastMove = 0.0
    $prev = $null
    while ($sw.Elapsed.TotalSeconds -lt $timeoutS) {
        try {
            $l = $p.ReadLine().Trim("`r").Trim()
            if ($l -like 'head:*') {
                $f = $l.Substring(5) -split ','
                if ($f.Count -eq 3 -and $f[0] -eq '1') {
                    $z = [double]$f[2]
                    if ($null -ne $prev -and [math]::Abs($z - $prev) -ge 0.15) {
                        $lastMove = $sw.Elapsed.TotalSeconds
                    }
                    $prev = $z
                    $samples.Add($z)
                }
            }
        } catch {}
    }
    if ($samples.Count -ge 2) {
        $start = $samples[0]
        $end = $samples[$samples.Count - 1]
        $delta = $end - $start
        $verdict = if ($lastMove -lt ($timeoutS - 2.0)) { 'REACHED' } else { 'TIMEOUT' }
        Write-Host ("turned {0:F2} deg (error {1:F2}), z {2:F2} -> {3:F2}, {4}" -f `
            $delta, ($delta - $angle), $start, $end, $verdict)
    }
    else { Write-Host 'no telemetry during turn' }
}
finally {
    $p.Close()
}
