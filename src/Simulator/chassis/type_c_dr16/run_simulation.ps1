param(
    [Parameter(Mandatory=$true)][string]$OutputDir,
    [Parameter(Mandatory=$true)][ValidatePattern('^[A-Za-z0-9_.-]{1,96}$')][string]$JobId,
    [Parameter(Mandatory=$true)][ValidatePattern('^[A-Za-z0-9_.-]{1,96}$')][string]$Nonce,
    [string]$SourceDir = (Join-Path $PSScriptRoot '../../../Hero/chassis/type_c_dr16'),
    [string]$CompilerPath,
    [string]$PythonPath,
    [ValidateSet('record-only','ignore','require-neutral')][string]$LeftVerticalPolicy = 'record-only'
)
$ErrorActionPreference = 'Stop'
if (-not [IO.Path]::IsPathRooted($OutputDir)) { throw 'OutputDir must be an absolute path' }
$outputRoot = [IO.Path]::GetFullPath($OutputDir)
$sourceRoot = (Resolve-Path -LiteralPath $SourceDir).Path
$simRoot = (Resolve-Path -LiteralPath $PSScriptRoot).Path
if ($outputRoot.TrimEnd('\','/') -eq $sourceRoot.TrimEnd('\','/') -or
    $outputRoot.TrimEnd('\','/') -eq $simRoot.TrimEnd('\','/')) {
    throw 'Use a dedicated output directory; never overwrite the input directory'
}
if (-not $CompilerPath) {
    $CompilerPath = (Get-Command zig.exe -ErrorAction Stop).Source
}
$compiler = (Resolve-Path -LiteralPath $CompilerPath).Path
if (-not $PythonPath) { $PythonPath = (Get-Command python.exe -ErrorAction Stop).Source }
$python = (Resolve-Path -LiteralPath $PythonPath).Path
$sourceFiles = @('dr16.c','dr16.h','chassis_control.c','chassis_control.h','can_policy.c','can_policy.h') | ForEach-Object { Join-Path $sourceRoot $_ }
$inputs = @($sourceFiles) + @((Join-Path $simRoot 'model_runner.c'),(Join-Path $simRoot 'report.py'),$PSCommandPath)
foreach ($path in $inputs) { if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing input $path" } }
$reserved = @('raw.csv','runner_summary.json','summary.json','report.html','model_runner.exe','build_manifest.json')
foreach ($name in $reserved) {
    if (Test-Path -LiteralPath (Join-Path $outputRoot $name)) { throw "Refusing to reuse old output: $name. Choose a new nonce/output directory." }
}
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
$hashes = @($inputs | ForEach-Object { $h=Get-FileHash -Algorithm SHA256 -LiteralPath $_; [ordered]@{path=$_.ToString();sha256=$h.Hash.ToLowerInvariant()} })
$priorLocalCache=$env:ZIG_LOCAL_CACHE_DIR; $priorGlobalCache=$env:ZIG_GLOBAL_CACHE_DIR
$priorNoBytecode=$env:PYTHONDONTWRITEBYTECODE
$env:ZIG_LOCAL_CACHE_DIR=Join-Path $outputRoot 'zig-local-cache'
$env:ZIG_GLOBAL_CACHE_DIR=Join-Path $outputRoot 'zig-global-cache'
$env:PYTHONDONTWRITEBYTECODE='1'
Push-Location -LiteralPath $outputRoot
try {
    $compilerVersion=(& $compiler version | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) { throw 'Unable to query Zig version' }
    $manifest=[ordered]@{
        schema_version=1; job_id=$JobId; nonce=$Nonce
        started_at=[DateTimeOffset]::Now.ToString('o'); software_only=$true
        source_dir=$sourceRoot; simulation_dir=$simRoot; output_dir=$outputRoot
        compiler=$compiler; compiler_version="Zig $compilerVersion / Clang C99 -O2"
        python=$python; inputs=$hashes; inputs_unchanged=$false
        model_scope='Assumed first-order rotor and injected CAN observations; production C parser/control/CAN policy; no BSP or hardware'
        left_vertical_policy=$LeftVerticalPolicy
    }
    $manifest | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath 'build_manifest.json' -Encoding UTF8
    $arguments=@('cc','-std=c99','-Wall','-Wextra','-Werror','-O2','-I',$sourceRoot,
                 (Join-Path $sourceRoot 'dr16.c'),(Join-Path $sourceRoot 'chassis_control.c'),(Join-Path $sourceRoot 'can_policy.c'),
                 (Join-Path $simRoot 'model_runner.c'),'-o',(Join-Path $outputRoot 'model_runner.exe'))
    & $compiler @arguments *> 'compile.log'
    if ($LASTEXITCODE -ne 0) { throw "Simulation compilation failed; see $outputRoot\compile.log" }
    $manifest.runner_sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath 'model_runner.exe').Hash.ToLowerInvariant()
    & (Join-Path $outputRoot 'model_runner.exe') --job-id $JobId --nonce $Nonce --left-vertical-policy $LeftVerticalPolicy *> 'runner.log'
    $runnerExit=$LASTEXITCODE
    $manifest.runner_exit=$runnerExit
    $manifest.inputs_unchanged=$true
    foreach ($entry in $hashes) {
        if ((Get-FileHash -Algorithm SHA256 -LiteralPath $entry.path).Hash.ToLowerInvariant() -ne $entry.sha256) {
            $manifest.inputs_unchanged=$false
        }
    }
    $manifest.finished_at=[DateTimeOffset]::Now.ToString('o')
    $manifest | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath 'build_manifest.json' -Encoding UTF8
    if (-not (Test-Path -LiteralPath 'runner_summary.json')) { throw "Runner failed before producing summary; exit $runnerExit" }
    & $python -B (Join-Path $simRoot 'report.py') --output-dir $outputRoot --job-id $JobId --nonce $Nonce --runner-exit $runnerExit *> 'report.log'
    $reportExit=$LASTEXITCODE
    Get-Content -LiteralPath 'runner.log'
    Get-Content -LiteralPath 'report.log'
    if ($runnerExit -ne 0 -or $reportExit -ne 0 -or -not $manifest.inputs_unchanged) { throw 'Simulation or artifact verification failed' }
} finally {
    Pop-Location
    $env:ZIG_LOCAL_CACHE_DIR=$priorLocalCache; $env:ZIG_GLOBAL_CACHE_DIR=$priorGlobalCache
    $env:PYTHONDONTWRITEBYTECODE=$priorNoBytecode
}
