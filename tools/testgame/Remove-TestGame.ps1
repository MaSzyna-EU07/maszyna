<#
.SYNOPSIS
Removes a folder made by New-TestGame.ps1 without touching the game data its junctions lead to.

.DESCRIPTION
New-TestGame.ps1 puts junctions only in the folder itself and in its scenery and textures folders. Those three
places are listed without recursion (a recursive listing could go through a junction into the data) and every link
in them is taken off as a link alone (Directory.Delete without recursion doesn't go into it). Only when none is left
is the rest of the folder deleted. A folder without eu07.base.ini isn't touched.
#>
param(
    [Parameter(Mandatory = $true)] [string] $Game
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path $Game)) { "nothing at $Game"; return }
if (-not (Test-Path (Join-Path $Game 'eu07.base.ini'))) { throw "$Game wasn't made by New-TestGame.ps1 (no eu07.base.ini), not removed" }

function Get-Links {
    $places = @($Game, (Join-Path $Game 'scenery'), (Join-Path $Game 'textures'))
    foreach ($place in $places) {
        if (-not (Test-Path $place)) { continue }
        Get-ChildItem $place -Force -Attributes ReparsePoint -ErrorAction SilentlyContinue
    }
}

$links = @(Get-Links)
foreach ($link in $links) {
    [System.IO.Directory]::Delete($link.FullName, $false)
}
$left = @(Get-Links)
if ($left.Count -gt 0) { throw "$($left.Count) links are still in $Game, nothing else removed: $($left[0].FullName)" }

Remove-Item $Game -Recurse -Force
"removed $Game ($($links.Count) junctions taken off first)"
