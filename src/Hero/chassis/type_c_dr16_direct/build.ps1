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
$sources = @('startup_stm32f407.S','platform.c','main.c','dr16.c','chassis_control.c')
$objects = @()
foreach ($source in $sources) {
    $sourcePath = Join-Path $sourceRoot $source
    if (-not (Test-Path -LiteralPath $sourcePath -PathType Leaf)) { throw "Missing source: $sourcePath" }
    $object = Join-Path $outputDir ([IO.Path]::GetFileNameWithoutExtension($source) + '.o')
    & $cc @common '-c' $sourcePath '-o' $object
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $source" }
    $objects += $object
}
$elf = Join-Path $outputDir 'type_c_dr16_direct.elf'
$map = Join-Path $outputDir 'type_c_dr16_direct.map'
$linker = Join-Path $sourceRoot 'stm32f407ig.ld'
$linkArgs = @('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-nostdlib',
    '-T',$linker,"-Wl,-Map=$map",'-Wl,--gc-sections','-Wl,--build-id=none',
    '-Wl,-z,noexecstack','-Wl,--fatal-warnings')
& $cc @linkArgs @objects '-lgcc' '-o' $elf
if ($LASTEXITCODE -ne 0) { throw 'Link failed.' }
& $objcopy '-O' 'binary' $elf (Join-Path $outputDir 'type_c_dr16_direct.bin')
if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed.' }
& $objcopy '-O' 'ihex' $elf (Join-Path $outputDir 'type_c_dr16_direct.hex')
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
    startup = 'zero until first valid remote packet; S1 UP drives immediately, other positions command zero'
    variant = 'direct development control; no runtime protection or fault latch; invalid RC zeros output until next valid frame; no silence timeout'
    target = 'RoboMaster Type C STM32F407IG, 12MHz HSE, 168MHz SYSCLK, CAN1 PD0/PD1 1Mbps'
    remote = 'USART3 RX PC11 AF7; 100000 baud 8E1; DMA1 Stream1 Channel4; IDLE framed'
    controlled_ids = @(1,2,3,4)
    excluded_outputs = 'ID7 feeder and all ID5..8 current slots always zero; no CAN2/PWM/laser'
    hardware_operations = 'none; this script only compiles and inspects files'
    source_sha256 = @(($sources + @('platform.h','chassis_control.h','wheel_config.h','dr16.h','stm32f407ig.ld')) | ForEach-Object {
        [ordered]@{ name=$_; sha256=(Get-FileHash -LiteralPath (Join-Path $sourceRoot $_) -Algorithm SHA256).Hash }
    })
    outputs = @(Get-ChildItem -LiteralPath $outputDir -File | Where-Object { $_.Extension -in '.elf','.bin','.hex' } | ForEach-Object {
        [ordered]@{ name=$_.Name; bytes=$_.Length; sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
    })
}
$metadata | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $outputDir 'build.json') -Encoding utf8
Write-Output "Built: $elf"
