$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Force Build | Out-Null
    & gcc -std=c99 -Wall -Wextra -Werror -pedantic `
        -IDrivers -IApp -IPlatform -IHardware `
        Hardware/Heading.c Test/HeadingHardware/test_heading.c -o Build/test_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading test compilation failed.' }
    & ./Build/test_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading tests failed.' }
} finally { Pop-Location }
