param(
    [string]$ToolchainBin = ''
)
$ErrorActionPreference = 'Stop'
$sourceRoot = $PSScriptRoot
if (-not $ToolchainBin) {
    $candidate = Get-ChildItem -Path 'C:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*\tools\bin\arm-none-eabi-gcc.exe' -ErrorAction SilentlyContinue |
        Sort-Object FullName | Select-Object -Last 1
    if (-not $candidate) { throw 'GNU Arm compiler not found; pass -ToolchainBin.' }
    $ToolchainBin = $candidate.Directory.FullName
}
$cc = Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe'
$size = Join-Path $ToolchainBin 'arm-none-eabi-size.exe'
$nm = Join-Path $ToolchainBin 'arm-none-eabi-nm.exe'
foreach ($tool in @($cc,$objcopy,$size,$nm)) {
    if (-not (Test-Path -LiteralPath $tool -PathType Leaf)) { throw "Missing tool: $tool" }
}
$outputDir = Join-Path $sourceRoot 'build'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$common = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard',
    '-std=c11','-O2','-g3','-ffreestanding','-fno-builtin','-fno-common',
    '-ffunction-sections','-fdata-sections','-fno-unwind-tables','-fno-asynchronous-unwind-tables',
    '-Wall','-Wextra','-Werror',
    '-I',$sourceRoot)
$sources = @('startup_stm32f407.S','platform.c','main.c','dr16.c','chassis_control.c','can_policy.c')
$objects = @()
foreach ($source in $sources) {
    $sourcePath = Join-Path $sourceRoot $source
    if (-not (Test-Path -LiteralPath $sourcePath -PathType Leaf)) { throw "Missing source: $sourcePath" }
    $object = Join-Path $outputDir ([IO.Path]::GetFileNameWithoutExtension($source) + '.o')
    & $cc @common '-c' $sourcePath '-o' $object
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $source" }
    $objects += $object
}
$elf = Join-Path $outputDir 'type_c_dr16.elf'
$map = Join-Path $outputDir 'type_c_dr16.map'
$linker = Join-Path $sourceRoot 'stm32f407ig.ld'
$linkArgs = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-nostdlib',
    '-T',$linker,"-Wl,-Map=$map",'-Wl,--gc-sections','-Wl,--build-id=none',
    '-Wl,-z,noexecstack','-Wl,--fatal-warnings')
& $cc @linkArgs @objects '-lgcc' '-o' $elf
if ($LASTEXITCODE -ne 0) { throw 'Link failed.' }
& $objcopy '-O' 'binary' $elf (Join-Path $outputDir 'type_c_dr16.bin')
if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed.' }
& $objcopy '-O' 'ihex' $elf (Join-Path $outputDir 'type_c_dr16.hex')
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed.' }
& $size $elf
if ($LASTEXITCODE -ne 0) { throw 'Size inspection failed.' }
$symbols = & $nm '-n' '-S' '--defined-only' $elf
if ($LASTEXITCODE -ne 0) { throw 'Symbol inspection failed.' }
$symbols | Set-Content -LiteralPath (Join-Path $outputDir 'symbols.txt') -Encoding utf8
$compilerVersion = (& $cc '--version' | Select-Object -First 1)
$metadata = [ordered]@{
    built_at = (Get-Date).ToString('o')
    compiler = $compilerVersion
    startup = 'zero output; DR16 S1 DOWN plus neutral dwell, then UP while neutral'
    target = 'RoboMaster Type C STM32F407IG, 12MHz HSE, 168MHz SYSCLK, CAN1 PD0/PD1 1Mbps'
    remote = 'USART3 RX PC11 AF7; 100000 baud 8E1; DMA1 Stream1 Channel4; IDLE framed'
    controlled_ids = @(1,2,3,4)
    excluded_outputs = 'ID7 feeder and all ID5..8 current slots always zero; no CAN2/PWM/laser'
    hardware_operations = 'none; this script only compiles and inspects files'
    outputs = @(Get-ChildItem -LiteralPath $outputDir -File | Where-Object { $_.Extension -in '.elf','.bin','.hex' } | ForEach-Object {
        [ordered]@{ name=$_.Name; bytes=$_.Length; sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
    })
}
$metadata | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $outputDir 'build.json') -Encoding utf8
Write-Output "Built: $elf"
