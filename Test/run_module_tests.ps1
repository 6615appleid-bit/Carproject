$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force Build | Out-Null
    $common = @('-std=c99','-Wall','-Wextra','-Werror','-pedantic',
        '-DSTM32F10X_MD','-DUSE_STDPERIPH_DRIVER','-ITest/ModuleHardware','-ITest',
        '-IDrivers','-IModules','-IApp','-IPlatform','-IUser','-ILibraries/CMSIS',
        '-ILibraries/STM32F10x_StdPeriph_Driver/inc',
        'Hardware/track.c','Hardware/Avoid.c','App/CarControl.c',
        'Test/ModuleHardware/test_modules.c','Test/Heading_Mock.c','-o','Build/test_modules.exe')
    foreach ($reverse in @(0,1)) {
        foreach ($level in @(0,1)) {
            & gcc @common "-DTRACK_BLACK_LEVEL=$level" "-DTRACK_REVERSE_ORDER=$reverse"
            if ($LASTEXITCODE -ne 0) { throw 'Module test compilation failed.' }
            & ./Build/test_modules.exe
            if ($LASTEXITCODE -ne 0) { throw 'Module tests failed.' }
        }
    }
} finally { Pop-Location }
