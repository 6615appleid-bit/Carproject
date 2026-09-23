param([string]$ToolchainBin = 'D:/LeStoreDownload/Keil 5/ARM/ARMCC/bin')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
[xml]$project = Get-Content -Raw (Join-Path $PSScriptRoot 'CarProject.uvprojx')
$target = $project.Project.Targets.Target | Where-Object TargetName -eq 'CarControl_Hardware_STM32F103C8'
if (-not $target) { throw 'Hardware target not found.' }
$includes = @($target.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath.Split(';') | ForEach-Object {
    '-I' + [IO.Path]::GetFullPath((Join-Path $PSScriptRoot $_))
})
$defines = @($target.TargetOption.TargetArmAds.Cads.VariousControls.Define.Split(',') | ForEach-Object { '-D' + $_ })
$outDir = Join-Path $root 'Build/HardwareCLI'
New-Item -ItemType Directory -Force $outDir | Out-Null
$objects = @()
foreach ($file in $target.Groups.Group.Files.File) {
    if ($file.FileType -notin @('1','2')) { continue }
    $inputFile = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot $file.FilePath))
    $object = Join-Path $outDir ([IO.Path]::GetFileNameWithoutExtension($file.FileName) + '.o')
    if ($objects -contains $object) { throw "Duplicate object basename: $object" }
    if ($file.FileType -eq '1') {
        & (Join-Path $ToolchainBin 'armcc.exe') --cpu Cortex-M3 --c99 --apcs=interwork -O0 -g @defines @includes -c $inputFile -o $object
    } else {
        & (Join-Path $ToolchainBin 'armasm.exe') --cpu Cortex-M3 --apcs=interwork -g $inputFile -o $object
    }
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $inputFile" }
    $objects += $object
}
$scatter = Join-Path $outDir 'CarControl_Hardware.sct'
@'
LR_IROM1 0x08000000 0x00010000 {
  ER_IROM1 0x08000000 0x00010000 {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00005000 {
    .ANY (+RW +ZI)
  }
}
'@ | Set-Content -Encoding ASCII $scatter
$image = Join-Path $outDir 'CarControl_Hardware.axf'
$map = Join-Path $outDir 'CarControl_Hardware.map'
& (Join-Path $ToolchainBin 'armlink.exe') --cpu Cortex-M3 --strict --scatter $scatter @objects --info sizes --map --symbols --list $map -o $image
if ($LASTEXITCODE -ne 0) { throw 'ARM link failed.' }
& (Join-Path $ToolchainBin 'fromelf.exe') --i32combined --output (Join-Path $outDir 'CarControl_Hardware.hex') $image
if ($LASTEXITCODE -ne 0) { throw 'HEX conversion failed.' }
Write-Output "ARM build/link passed: $image"
