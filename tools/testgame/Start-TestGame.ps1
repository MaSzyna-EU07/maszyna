<#
.SYNOPSIS
Runs the game in a folder made by New-TestGame.ps1, with a limit on time and memory, keeps its logs and checks that
the game data wasn't changed.

.DESCRIPTION
eu07.ini of the run is eu07.base.ini with the lines of -IniLines after it. APPDATA and LOCALAPPDATA point at the
appdata and localappdata folders of the test game, so the settings of the player's profile aren't read and nothing is
written into the profile or its caches. PYTHONDONTWRITEBYTECODE keeps Python from writing .pyc files next to the
scripts of the data. With -Selftest the scenery opens in the editor (-edit) and the script runs, as
EU07_EDITOR_SELFTEST; the game has to be built with -DWITH_EDITOR_SELFTESTS=ON for that.

The run is ended when it takes longer than -TimeoutMinutes or the memory it has committed (its private bytes, the
working set as well as what went to the page file) grows over -MemoryLimitGB, so a script gone wrong can't fill the
memory of the machine. The logs, errors.txt and the screenshots go to -Output, and the lines of the self-test
(SELFTEST ...) are printed at the end.

After the run the data and the shaders the junctions lead to are walked (without going into links) for files and
folders changed since the start. Any there are listed and the script ends with exit code 1. A change made by
something else at the same time, such as an SVN update, shows up the same way. A file deleted from a folder shows as
a change of that folder.

The run is in real time with the renderer on (-s, or -edit), which is what a measurement needs. A check that is to
give the same result every time runs in --test mode (a fixed step and seed) through tests\scenarios\run.py.

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
$Game = (Resolve-Path $Game).Path
if ($Output -eq '') { $Output = Join-Path $Game ('runs\' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
New-Item -ItemType Directory -Force $Output | Out-Null

# where the junctions lead, as New-TestGame.ps1 recorded it; a folder made by an earlier version has no record
$watched = @()
$mark = Join-Path $Game '.testgame'
if (Test-Path $mark) {
    foreach ($line in [System.IO.File]::ReadAllLines($mark)) {
        if ($line -match '^(data|shaders)=(.+)$') { $watched += $Matches[2] }
    }
}
if ($watched.Count -eq 0) { Write-Warning "$Game has no .testgame record, so the data won't be checked after the run" }

$ini = [System.IO.File]::ReadAllText((Join-Path $Game 'eu07.base.ini'))
foreach ($line in $IniLines) { $ini += "$line`r`n" }
[System.IO.File]::WriteAllText((Join-Path $Game 'eu07.ini'), $ini)

# what an earlier run left
foreach ($name in 'log.txt', 'errors.txt') { Remove-Item (Join-Path $Game $name) -ErrorAction SilentlyContinue }
foreach ($name in 'logs', 'screenshots') {
    $folder = Join-Path $Game $name
    if (Test-Path $folder) { Get-ChildItem $folder -Force | Remove-Item -Recurse -Force }
}
foreach ($name in 'appdata', 'localappdata') { New-Item -ItemType Directory -Force (Join-Path $Game $name) | Out-Null }

$arguments = '-s "' + $Scenery + '"'
if ($Selftest -ne '') { $arguments = '-edit "' + $Scenery + '"' }
$start = New-Object System.Diagnostics.ProcessStartInfo $Exe, $arguments
$start.WorkingDirectory = $Game
$start.UseShellExecute = $false
$start.EnvironmentVariables['APPDATA'] = (Join-Path $Game 'appdata')
$start.EnvironmentVariables['LOCALAPPDATA'] = (Join-Path $Game 'localappdata')
$start.EnvironmentVariables['PYTHONDONTWRITEBYTECODE'] = '1'
if ($Selftest -ne '') { $start.EnvironmentVariables['EU07_EDITOR_SELFTEST'] = (Resolve-Path $Selftest).Path }
# a little earlier than the start, for the coarser time stamps of some file systems
$since = (Get-Date).ToUniversalTime().AddSeconds(-2)
$process = [System.Diagnostics.Process]::Start($start)

$deadline = (Get-Date).AddMinutes($TimeoutMinutes)
$limit = [long]($MemoryLimitGB * 1GB)
[long] $peak = 0
[long] $peakWorkingSet = 0
$ended = 'exited'
while (-not $process.HasExited) {
    Start-Sleep -Seconds 2
    try {
        $process.Refresh()
        if ($process.PrivateMemorySize64 -gt $peak) { $peak = $process.PrivateMemorySize64 }
        if ($process.WorkingSet64 -gt $peakWorkingSet) { $peakWorkingSet = $process.WorkingSet64 }
    }
    catch { }
    if ($peak -gt $limit) { $process.Kill(); $ended = "ended: committed memory over $MemoryLimitGB GB"; break }
    if ((Get-Date) -gt $deadline) { $process.Kill(); $ended = "ended: over $TimeoutMinutes minutes"; break }
}
$process.WaitForExit()
'{0}, exit code {1}, peak committed memory {2:N1} GB, peak working set {3:N1} GB' -f $ended, $process.ExitCode, ($peak / 1GB), ($peakWorkingSet / 1GB)

foreach ($file in Get-ChildItem $Game -File -Filter '*.txt') { Copy-Item $file.FullName $Output }
foreach ($name in 'logs', 'screenshots') {
    $folder = Join-Path $Game $name
    if (Test-Path $folder) { Copy-Item (Join-Path $folder '*') $Output -Recurse -ErrorAction SilentlyContinue }
}
"logs in $Output"
Get-ChildItem $Output -Recurse -File -Filter '*.txt' |
    Select-String -Pattern 'SELFTEST' |
    ForEach-Object { $_.Line -replace '\x1b\[[0-9;]*m', '' }

# the files and folders under a folder changed since the start, found without going into links
function Find-Changed([System.IO.DirectoryInfo] $Folder) {
    foreach ($entry in $Folder.GetFileSystemInfos()) {
        if ($entry.Name -eq '.svn') { continue }
        if ($entry.LastWriteTimeUtc -ge $since -or $entry.CreationTimeUtc -ge $since) { $entry.FullName }
        $link = ($entry.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0
        if ($entry -is [System.IO.DirectoryInfo] -and -not $link) { Find-Changed $entry }
    }
}
if ($watched.Count -gt 0) {
    $changed = @(foreach ($folder in $watched) { Find-Changed (New-Object System.IO.DirectoryInfo $folder) })
    if ($changed.Count -gt 0) {
        Write-Warning "$($changed.Count) files or folders of the data changed during the run:"
        $changed | Select-Object -First 50 | ForEach-Object { "  $_" }
        exit 1
    }
    "the data is unchanged ($($watched -join ', '))"
}
