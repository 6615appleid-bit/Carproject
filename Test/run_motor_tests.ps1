$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
New-Item -ItemType Directory -Force -Path (Join-Path $projectRoot 'Build') | Out-Null
Push-Location $projectRoot
try {
    foreach ($polarity in @(@(1, 1), @(-1, 1), @(1, -1), @(-1, -1))) {
        $leftPolarity = $polarity[0]
        $rightPolarity = $polarity[1]
        & gcc -std=c99 -Wall -Wextra -Werror -pedantic `
            -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER `
            "-DMOTOR_LEFT_POLARITY=$leftPolarity" "-DMOTOR_RIGHT_POLARITY=$rightPolarity" `
            -ITest/MotorHardware -IDrivers -IUser -ILibraries/CMSIS `
            -ILibraries/STM32F10x_StdPeriph_Driver/inc `
            Hardware/Motor.c Test/MotorHardware/test_motor.c -o Build/test_motor.exe
        if ($LASTEXITCODE -ne 0) { throw 'Motor test compilation failed.' }
        & ./Build/test_motor.exe
        if ($LASTEXITCODE -ne 0) { throw 'Motor driver tests failed.' }
    }
} finally { Pop-Location }
