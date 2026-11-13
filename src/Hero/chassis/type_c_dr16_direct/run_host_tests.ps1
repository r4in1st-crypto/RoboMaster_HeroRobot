param([string]$Zig = 'zig.exe', [string]$OutputDir = (Join-Path $PSScriptRoot 'build_host'))

$ErrorActionPreference = 'Stop'
$compiler = (Get-Command $Zig -CommandType Application -ErrorAction Stop).Source
$dir = New-Item -ItemType Directory -Force -Path $OutputDir
$exe = Join-Path $dir.FullName 'chassis_host_test.exe'
$sources = 'dr16.c', 'chassis_control.c', 'chassis_host_test.c' | ForEach-Object { Join-Path $PSScriptRoot $_ }
& $compiler cc -std=c99 -Wall -Wextra -Werror -O2 @sources -o $exe
if ($LASTEXITCODE) { throw 'Direct chassis host compilation failed' }
& $exe | Tee-Object -FilePath (Join-Path $dir.FullName 'host_test_result.txt')
if ($LASTEXITCODE) { throw 'Direct chassis host tests failed' }
