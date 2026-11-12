$ErrorActionPreference = 'Stop'
$carRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $carRoot
try {
    # 链接完整调参路径，仅由测试替代 HAL 和硬件寄存器。
    $includeArgs = @(
        '-DSTM32F103xB', '-ICore/Inc',
        '-IDrivers/STM32F1xx_HAL_Driver/Inc',
        '-IDrivers/CMSIS/Device/ST/STM32F1xx/Include',
        '-IDrivers/CMSIS/Include'
    )
    # 航向积分为纯计算模块，直接验证积分、回绕与异常间隔保护。
    $ErrorActionPreference = 'Continue'
    & gcc -std=c99 @includeArgs tests/test_heading.c Core/Src/heading.c -o tests/test_heading.exe 2> tests/heading_build.log
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($compileExit -ne 0) {
        Get-Content tests/heading_build.log
        throw '航向积分测试编译失败。'
    }
    & ./tests/test_heading.exe
    if ($LASTEXITCODE -ne 0) { throw '航向积分测试失败。' }
    # 陀螺仪闭环转向：真实转向模块 + 桩速度环，验证差速方向与全部保护。
    $ErrorActionPreference = 'Continue'
    & gcc -std=c99 @includeArgs tests/test_turn_control.c Core/Src/turn_control.c Core/Src/heading.c -o tests/test_turn_control.exe 2> tests/turn_build.log
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($compileExit -ne 0) {
        Get-Content tests/turn_build.log
        throw '转向控制测试编译失败。'
    }
    & ./tests/test_turn_control.exe
    if ($LASTEXITCODE -ne 0) { throw '转向控制测试失败。' }
    # 左手法则迷宫状态机：真实状态机 + 桩测距与速度环。
    $ErrorActionPreference = 'Continue'
    & gcc -std=c99 @includeArgs tests/test_maze.c Core/Src/maze.c Core/Src/turn_control.c Core/Src/heading.c -o tests/test_maze.exe 2> tests/maze_build.log
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($compileExit -ne 0) {
        Get-Content tests/maze_build.log
        throw '迷宫状态机测试编译失败。'
    }
    & ./tests/test_maze.exe
    if ($LASTEXITCODE -ne 0) { throw '迷宫状态机测试失败。' }
    # 链接完整调参路径，仅由测试替代 HAL 和硬件寄存器。
    $ErrorActionPreference = 'Continue'
    & gcc -std=c99 @includeArgs tests/test_bluetooth_tune.c Core/Src/bluetooth_tune.c Core/Src/hc05.c Core/Src/speed_control.c Core/Src/motor.c Core/Src/encoder.c Core/Src/heading.c Core/Src/turn_control.c Core/Src/maze.c -o tests/test_bluetooth_tune.exe 2> tests/tune_build.log
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($compileExit -ne 0) {
        Get-Content tests/tune_build.log
        throw '蓝牙调参集成测试编译失败。'
    }
    & ./tests/test_bluetooth_tune.exe
    if ($LASTEXITCODE -ne 0) { throw '蓝牙调参集成测试失败。' }
    # RX 中断组帧与发送共用 UART，必须同时验证忙态、长行及丢字节恢复。
    $ErrorActionPreference = 'Continue'
    & gcc -std=c99 @includeArgs tests/test_hc05.c Core/Src/hc05.c -o tests/test_hc05.exe 2> tests/hc05_build.log
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($compileExit -ne 0) {
        Get-Content tests/hc05_build.log
        throw 'HC05 收发测试编译失败。'
    }
    & ./tests/test_hc05.exe
    if ($LASTEXITCODE -ne 0) { throw 'HC05 收发测试失败。' }
    & ./tests/run_speed_control.ps1
}
finally {
    Pop-Location
}
