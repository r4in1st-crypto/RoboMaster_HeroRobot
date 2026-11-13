param([string]$Zig = 'zig.exe')
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$zig = (Get-Command $Zig -ErrorAction Stop).Source
Push-Location -LiteralPath $taskRoot
try {
    & $zig cc -std=c99 -Wall -Wextra -Werror -O2 dr16.c dr16_host_test.c -o dr16_host_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'DR16 parser compilation failed' }
    & '.\dr16_host_test.exe' | Tee-Object -FilePath 'dr16_host_test_result.txt'
    if ($LASTEXITCODE -ne 0) { throw 'DR16 parser tests failed' }
    & $zig cc -std=c99 -Wall -Wextra -Werror -O2 dr16.c chassis_control.c chassis_host_test.c -o chassis_host_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Chassis controller compilation failed' }
    & '.\chassis_host_test.exe' | Tee-Object -FilePath 'chassis_host_test_result.txt'
    if ($LASTEXITCODE -ne 0) { throw 'Chassis controller tests failed' }
    & $zig cc -std=c99 -Wall -Wextra -Werror -O2 can_policy.c can_policy_host_test.c -o can_policy_host_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'CAN policy compilation failed' }
    & '.\can_policy_host_test.exe' | Tee-Object -FilePath 'can_policy_host_test_result.txt'
    if ($LASTEXITCODE -ne 0) { throw 'CAN policy tests failed' }
} finally {
    Pop-Location
}
