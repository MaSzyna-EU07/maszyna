<#
.SYNOPSIS
Runs the game in a folder made by New-TestGame.ps1, with a limit on time and memory, and keeps its logs.

.DESCRIPTION
eu07.ini of the run is eu07.base.ini with the lines of -IniLines after it. APPDATA points at the appdata folder of
the test game, so the settings of the player's profile aren't read and nothing is written into it. With -Selftest the
scenery opens in the editor (-edit) and the script runs, as EU07_EDITOR_SELFTEST; the game has to be built with
-DWITH_EDITOR_SELFTESTS=ON for that.

The run is ended when it takes longer than -TimeoutMinutes or its working set grows over -MemoryLimitGB, so a script
gone wrong can't fill the memory of the machine. The logs, errors.txt and the screenshots go to -Output, and the
lines of the self-test (SELFTEST ...) are printed at the end.

.EXAMPLE
.\Start-TestGame.ps1 -Game C:\games\test-foliage -Exe C:\src\maszyna\build\bin\RelWithDebInfo\eu07.exe `
    -Scenery foliage_test.scn -Selftest C:\games\foliage.selftest -IniLines 'vsync no' -Output C:\games\out\run1
#>
param(
    [Parameter(Mandatory = $true)] [string] $Game,
    [Parameter(Mandatory = $true)] [string] $Exe,
    [Parameter(Mandatory = $true)] [string] $Scenery,
    [string] $Selftest = '',
    [string[]] $IniLines = @(),
    [int] $TimeoutMinutes = 30,
    [double] $MemoryLimitGB = 16,
    [string] $Output = ''
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path (Join-Path $Game 'eu07.base.ini'))) { throw "$Game wasn't made by New-TestGame.ps1 (no eu07.base.ini)" }
if (-not (Test-Path $Exe)) { throw "no game binary at $Exe" }
if ($Output -eq '') { $Output = Join-Path $Game ('runs\' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
New-Item -ItemType Directory -Force $Output | Out-Null

$ini = [System.IO.File]::ReadAllText((Join-Path $Game 'eu07.base.ini'))
foreach ($line in $IniLines) { $ini += "$line`r`n" }
[System.IO.File]::WriteAllText((Join-Path $Game 'eu07.ini'), $ini)

# what an earlier run left
foreach ($name in 'log.txt', 'errors.txt') { Remove-Item (Join-Path $Game $name) -ErrorAction SilentlyContinue }
foreach ($name in 'logs', 'screenshots') {
    $folder = Join-Path $Game $name
    if (Test-Path $folder) { Get-ChildItem $folder -Force | Remove-Item -Recurse -Force }
}

$arguments = '-s ' + $Scenery
if ($Selftest -ne '') { $arguments = '-edit ' + $Scenery }
$start = New-Object System.Diagnostics.ProcessStartInfo $Exe, $arguments
$start.WorkingDirectory = $Game
$start.UseShellExecute = $false
$start.EnvironmentVariables['APPDATA'] = (Join-Path $Game 'appdata')
if ($Selftest -ne '') { $start.EnvironmentVariables['EU07_EDITOR_SELFTEST'] = (Resolve-Path $Selftest).Path }
$process = [System.Diagnostics.Process]::Start($start)

$deadline = (Get-Date).AddMinutes($TimeoutMinutes)
$limit = [long]($MemoryLimitGB * 1GB)
[long] $peak = 0
$ended = 'exited'
while (-not $process.HasExited) {
    Start-Sleep -Seconds 2
    try {
        $process.Refresh()
        if ($process.WorkingSet64 -gt $peak) { $peak = $process.WorkingSet64 }
    }
    catch { }
    if ($peak -gt $limit) { $process.Kill(); $ended = "ended: working set over $MemoryLimitGB GB"; break }
    if ((Get-Date) -gt $deadline) { $process.Kill(); $ended = "ended: over $TimeoutMinutes minutes"; break }
}
$process.WaitForExit()
'{0}, exit code {1}, peak working set {2:N1} GB' -f $ended, $process.ExitCode, ($peak / 1GB)

foreach ($file in Get-ChildItem $Game -File -Filter '*.txt') { Copy-Item $file.FullName $Output }
foreach ($name in 'logs', 'screenshots') {
    $folder = Join-Path $Game $name
    if (Test-Path $folder) { Copy-Item (Join-Path $folder '*') $Output -Recurse -ErrorAction SilentlyContinue }
}
"logs in $Output"
Get-ChildItem $Output -Recurse -File -Filter '*.txt' |
    Select-String -Pattern 'SELFTEST' |
    ForEach-Object { $_.Line -replace '\x1b\[[0-9;]*m', '' }
