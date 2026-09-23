$ErrorActionPreference = 'Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    New-Item -ItemType Directory -Force Build | Out-Null
    & gcc -std=c99 -Wall -Wextra -Werror -pedantic -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER `
        -ITest/ModuleHardware -IModules -IPlatform -IUser -ILibraries/CMSIS `
        -ILibraries/STM32F10x_StdPeriph_Driver/inc `
        Hardware/Ultrasound.c Test/ModuleHardware/test_ultrasound.c -o Build/test_ultrasound.exe
    if ($LASTEXITCODE -ne 0) { throw 'Ultrasound test compilation failed.' }
    & ./Build/test_ultrasound.exe
    if ($LASTEXITCODE -ne 0) { throw 'Ultrasound tests failed.' }
} finally { Pop-Location }
