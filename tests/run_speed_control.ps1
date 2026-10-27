$ErrorActionPreference = 'Stop'
$carRoot = Split-Path -Parent $PSScriptRoot
Push-Location -LiteralPath $carRoot
try {
    # 编译真实控制器、编码器与电机模块，测试代码仅替代 HAL 与寄存器。
    $includeArgs = @(
        '-DSTM32F103xB', '-ICore/Inc',
        '-IDrivers/STM32F1xx_HAL_Driver/Inc',
        '-IDrivers/CMSIS/Device/ST/STM32F1xx/Include',
        '-IDrivers/CMSIS/Include'
    )
    $buildLog = Join-Path $PSScriptRoot 'speed_build.log'
    # CMSIS 的主机地址宽度警告写入日志，编译失败时完整输出。
    # PowerShell 5 将原生 stderr 包装成错误记录，编译阶段改用 Continue 后检查退出码。
    $ErrorActionPreference = 'Continue'
    & gcc -std=c99 @includeArgs tests/test_speed_control.c Core/Src/speed_control.c Core/Src/motor.c Core/Src/encoder.c -o tests/test_speed_control.exe 2> $buildLog
    $compileExit = $LASTEXITCODE
    $ErrorActionPreference = 'Stop'
    if ($compileExit -ne 0) {
        Get-Content -LiteralPath $buildLog
        throw '速度控制主机测试编译失败。'
    }
    # 两次独立进程分别验证 PI 集成保护和测量微分，防止静态状态相互影响。
    & ./tests/test_speed_control.exe
    if ($LASTEXITCODE -ne 0) { throw '速度控制集成测试失败。' }
    & ./tests/test_speed_control.exe derivative
    if ($LASTEXITCODE -ne 0) { throw '速度控制测量微分测试失败。' }
}
finally {
    Pop-Location
}
