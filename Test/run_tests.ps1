# 用途：在电脑上编译并运行CarControl模拟测试，不生成单片机固件。
# 编译失败或断言失败会报错；Build目录保存测试程序。
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $projectRoot 'Build'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
Push-Location $projectRoot
try {
    & gcc -std=c99 -Wall -Wextra -Werror -pedantic -IApp -IModules -IDrivers -IPlatform -ITest App/CarControl.c Test/Track_Mock.c Test/Avoid_Mock.c Test/Motor_Mock.c Test/test_car_control.c -o Build/test_car_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
    & ./Build/test_car_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'State-machine tests failed.' }
} finally { Pop-Location }
