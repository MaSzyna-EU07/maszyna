<#
.SYNOPSIS
Makes a game folder for a test run that can't change the game data it runs on.

.DESCRIPTION
The folders of the data (dynamic, models, sounds, ...) become junctions to a copy of the game data, such as an SVN
working copy, so they take no space. Whatever a run may write into goes into real folders of its own:

- scenery: its root files are copied, the subfolders named by -CopyScenery are copied, the rest are junctions;
- textures: the same, with -CopyTextures, so the materials of a test can be changed in the copy;
- appdata: the profile of the run (Start-TestGame.ps1 points APPDATA at it), so the player's own
  %APPDATA%\MaSzyna\eu07.ini is never read;
- eu07.base.ini: the eu07.ini of the data with the settings that keep a run from writing through the junctions
  (convertmodels 0, file.binary.terrain no) and the lines of -IniLines after it.

A junction is followed by whatever writes into it, so remove the folder with Remove-TestGame.ps1 only, which takes
the junctions off before it deletes anything.

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
    [string] $Binaries = '',
    [string[]] $IniLines = @()
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path (Join-Path $Data 'eu07.ini'))) { throw "no eu07.ini in $Data, it doesn't look like game data" }
if (Test-Path $Destination) { throw "$Destination exists already; pick a new folder or remove it with Remove-TestGame.ps1" }
if ($Shaders -eq '') { $Shaders = Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'shaders' }
if (-not (Test-Path $Shaders)) { throw "no shaders in $Shaders" }

New-Item -ItemType Directory $Destination | Out-Null

# folders filled the way scenery and textures are, and folders of a run's own output
$split = @('scenery', 'textures')
$skip = @('shaders', 'logs', 'screenshots', 'crashdumps', 'physicslog', '.svn')

foreach ($folder in Get-ChildItem $Data -Directory -Force) {
    if ($split -contains $folder.Name -or $skip -contains $folder.Name) { continue }
    New-Item -ItemType Junction -Path (Join-Path $Destination $folder.Name) -Target $folder.FullName | Out-Null
}
New-Item -ItemType Junction -Path (Join-Path $Destination 'shaders') -Target (Resolve-Path $Shaders).Path | Out-Null

# a real folder: the root files and the named subfolders copied, the other subfolders junctions
function New-SplitFolder([string] $Name, [string[]] $Copied) {
    $source = Join-Path $Data $Name
    $target = Join-Path $Destination $Name
    New-Item -ItemType Directory $target | Out-Null
    if (-not (Test-Path $source)) { return }
    Get-ChildItem $source -File -Force | Copy-Item -Destination $target
    foreach ($folder in Get-ChildItem $source -Directory -Force) {
        if ($Copied -contains $folder.Name) {
            Copy-Item $folder.FullName (Join-Path $target $folder.Name) -Recurse
        }
        else {
            New-Item -ItemType Junction -Path (Join-Path $target $folder.Name) -Target $folder.FullName | Out-Null
        }
    }
    foreach ($name in $Copied) {
        if (-not (Test-Path (Join-Path $source $name))) { Write-Warning "$Name\$name isn't in the data, nothing copied" }
    }
}
New-SplitFolder 'scenery' $CopyScenery
New-SplitFolder 'textures' $CopyTextures

New-Item -ItemType Directory (Join-Path $Destination 'appdata') | Out-Null

foreach ($file in Get-ChildItem $Data -File -Filter 'eu07_input-*.ini') { Copy-Item $file.FullName $Destination }
# the libraries the game loads from its folder, when they aren't next to the binary
if ($Binaries -ne '') {
    foreach ($file in Get-ChildItem $Binaries -File -Filter '*.dll') { Copy-Item $file.FullName $Destination }
}

# later keys win, so the safety settings and the lines of the test come after the shipped ones
$ini = [System.IO.File]::ReadAllText((Join-Path $Data 'eu07.ini'))
$ini += "`r`n// test game: nothing written through the junctions into the data`r`nconvertmodels 0`r`nfile.binary.terrain no`r`n"
foreach ($line in $IniLines) { $ini += "$line`r`n" }
[System.IO.File]::WriteAllText((Join-Path $Destination 'eu07.base.ini'), $ini)

"test game made: $Destination"
"copied: scenery [$($CopyScenery -join ', ')], textures [$($CopyTextures -join ', ')]"
