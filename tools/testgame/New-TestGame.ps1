<#
.SYNOPSIS
Makes a game folder for a test run that can't change the game data it runs on.

.DESCRIPTION
The folders of the data (models, sounds, ...) become junctions to the game data, such as an SVN working copy, so they
take no space. Whatever a run may write into goes into real folders of its own:

- scenery: its root files are copied (without the .sbt caches, which the run neither reads nor writes), the subfolders
  named by -CopyScenery are copied, the rest are junctions; an editor self-test saves only into the copied ones;
- textures, terrain, dynamic: the same, with -CopyTextures, -CopyTerrain and -CopyDynamic. The editor saves a
  heightmap terrain into terrain\<name>, and tests\scenarios\run.py writes its vehicle variants into
  dynamic\<vehicle dir>, so those are to be copied for such a test;
- appdata and localappdata: the profile and the caches of the run (Start-TestGame.ps1 points APPDATA and LOCALAPPDATA
  at them), so the player's own %APPDATA%\MaSzyna\eu07.ini is never read and nothing is written into the player's
  profile;
- eu07.base.ini: the eu07.ini of the data with the settings that keep a run from writing through the junctions
  (convertmodels 0, file.binary.terrain no) and the lines of -IniLines after it.

python64 (the Python of the game, which the build puts next to the binary) is a junction to the one in -Binaries;
without it the Python screens don't start. .testgame records where the data is, for the check of Start-TestGame.ps1.

A junction is followed by whatever writes into it, so remove the folder with Remove-TestGame.ps1 only, which takes
the junctions off before it deletes anything. When the making fails half way, what was made is removed that way.

Junctions need no administrator rights and no Developer Mode, unlike symbolic links, but they work on local NTFS
volumes only: the test folder can't be on FAT or exFAT, and the data can't be on a network share.

.EXAMPLE
.\New-TestGame.ps1 -Data C:\games\pctga -Destination C:\games\test-foliage -Shaders C:\src\maszyna\shaders `
    -CopyScenery linia053_wrzosy -CopyTextures l61_plants -Binaries C:\src\maszyna\build\bin\RelWithDebInfo `
    -IniLines 'vsync no', 'fullscreen no'
#>
param(
    [Parameter(Mandatory = $true)] [string] $Data,
    [Parameter(Mandatory = $true)] [string] $Destination,
    [string] $Shaders = '',
    [string[]] $CopyScenery = @(),
    [string[]] $CopyTextures = @(),
    [string[]] $CopyTerrain = @(),
    [string[]] $CopyDynamic = @(),
    [string] $Binaries = '',
    [string[]] $IniLines = @()
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path (Join-Path $Data 'eu07.ini'))) { throw "no eu07.ini in $Data, it doesn't look like game data" }
if (Test-Path $Destination) { throw "$Destination exists already; pick a new folder or remove it with Remove-TestGame.ps1" }
if ($Shaders -eq '') { $Shaders = Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'shaders' }
if (-not (Test-Path $Shaders)) { throw "no shaders in $Shaders" }
if ($Binaries -ne '' -and -not (Test-Path $Binaries)) { throw "no folder $Binaries" }
$Data = (Resolve-Path $Data).Path
$Shaders = (Resolve-Path $Shaders).Path

# the mark goes in before the first junction, so Remove-TestGame.ps1 takes a half made folder off as well
New-Item -ItemType Directory $Destination | Out-Null
$Destination = (Resolve-Path $Destination).Path
[System.IO.File]::WriteAllText((Join-Path $Destination '.testgame'), "data=$Data`r`nshaders=$Shaders`r`n")

# folders filled the way scenery is, with the subfolders to copy and the root files to leave out
$split = [ordered]@{
    'scenery'  = @{ Copied = $CopyScenery; Excluded = @('*.sbt') }
    'textures' = @{ Copied = $CopyTextures; Excluded = @() }
    'terrain'  = @{ Copied = $CopyTerrain; Excluded = @() }
    'dynamic'  = @{ Copied = $CopyDynamic; Excluded = @() }
}
# folders of a run's own output, and the Python of the build
$skip = @('shaders', 'logs', 'screenshots', 'crashdumps', 'physicslog', 'python64', '.svn')

# a real folder: the root files and the named subfolders copied, the other subfolders junctions
function New-SplitFolder([string] $Name, [string[]] $Copied, [string[]] $Excluded) {
    $source = Join-Path $Data $Name
    $target = Join-Path $Destination $Name
    New-Item -ItemType Directory $target | Out-Null
    if (-not (Test-Path $source)) { return }
    foreach ($file in Get-ChildItem $source -File -Force) {
        $left = $false
        foreach ($pattern in $Excluded) { if ($file.Name -like $pattern) { $left = $true } }
        if (-not $left) { Copy-Item $file.FullName $target }
    }
    foreach ($folder in Get-ChildItem $source -Directory -Force) {
        if ($Copied -contains $folder.Name) {
            Copy-Item $folder.FullName (Join-Path $target $folder.Name) -Recurse
        }
        else {
            New-Item -ItemType Junction -Path (Join-Path $target $folder.Name) -Target $folder.FullName | Out-Null
        }
    }
    foreach ($copy in $Copied) {
        if (-not (Test-Path (Join-Path $source $copy))) { Write-Warning "$Name\$copy isn't in the data, nothing copied" }
    }
}

try {
    foreach ($folder in Get-ChildItem $Data -Directory -Force) {
        if ($split.Contains($folder.Name) -or $skip -contains $folder.Name) { continue }
        New-Item -ItemType Junction -Path (Join-Path $Destination $folder.Name) -Target $folder.FullName | Out-Null
    }
    New-Item -ItemType Junction -Path (Join-Path $Destination 'shaders') -Target $Shaders | Out-Null

    foreach ($name in $split.Keys) { New-SplitFolder $name $split[$name].Copied $split[$name].Excluded }

    New-Item -ItemType Directory (Join-Path $Destination 'appdata') | Out-Null
    New-Item -ItemType Directory (Join-Path $Destination 'localappdata') | Out-Null

    foreach ($file in Get-ChildItem $Data -File -Filter 'eu07_input-*.ini') { Copy-Item $file.FullName $Destination }
    # the libraries the game loads from its folder, when they aren't next to the binary, and its Python
    $python = ''
    if ($Binaries -ne '') {
        foreach ($file in Get-ChildItem $Binaries -File -Filter '*.dll') { Copy-Item $file.FullName $Destination }
        $python = Join-Path $Binaries 'python64'
    }
    if ($python -ne '' -and (Test-Path $python)) {
        New-Item -ItemType Junction -Path (Join-Path $Destination 'python64') -Target (Resolve-Path $python).Path | Out-Null
    }
    else {
        Write-Warning "no python64 in -Binaries, so the Python screens won't start in the runs"
    }

    # later keys win, so the safety settings and the lines of the test come after the shipped ones
    $ini = [System.IO.File]::ReadAllText((Join-Path $Data 'eu07.ini'))
    $ini += "`r`n// test game: nothing written through the junctions into the data`r`nconvertmodels 0`r`nfile.binary.terrain no`r`n"
    foreach ($line in $IniLines) { $ini += "$line`r`n" }
    [System.IO.File]::WriteAllText((Join-Path $Destination 'eu07.base.ini'), $ini)
}
catch {
    $failure = $_
    Write-Warning "making $Destination failed, what was made is taken off: $($failure.Exception.Message)"
    try { & (Join-Path $PSScriptRoot 'Remove-TestGame.ps1') -Game $Destination | Out-Null }
    catch { Write-Warning "and it couldn't be removed: $($_.Exception.Message); remove it with Remove-TestGame.ps1" }
    throw $failure
}

"test game made: $Destination"
$copied = foreach ($name in $split.Keys) { "$name [$($split[$name].Copied -join ', ')]" }
"copied: $($copied -join ', ')"
