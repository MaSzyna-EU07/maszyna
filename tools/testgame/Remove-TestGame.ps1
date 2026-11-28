<#
.SYNOPSIS
Removes a folder made by New-TestGame.ps1 without touching the game data its junctions lead to.

.DESCRIPTION
The whole folder is walked without going into any link (a junction, a symbolic link), and every link found is taken
off as a link alone (Directory.Delete without recursion doesn't go into it). The folder is walked once more, and only
when no link is left anywhere in it is the rest deleted. A listing that fails stops the script before anything else
is deleted, so a folder that can't be read whole is left as it is.

A folder without the .testgame mark of New-TestGame.ps1 (or the eu07.base.ini of a folder made by an earlier version)
isn't touched. The mark is written before the first junction, so a folder whose making failed half way can be removed
too.
#>
param(
    [Parameter(Mandatory = $true)] [string] $Game
)
$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $Game)) { "nothing at $Game"; return }
$Game = (Resolve-Path -LiteralPath $Game).Path
$marked = (Test-Path -LiteralPath (Join-Path $Game '.testgame')) -or (Test-Path -LiteralPath (Join-Path $Game 'eu07.base.ini'))
if (-not $marked) { throw "$Game wasn't made by New-TestGame.ps1 (no .testgame, no eu07.base.ini), not removed" }

# every link in the folder, found without going into one; a folder that can't be listed stops the script
function Find-Links([System.IO.DirectoryInfo] $Folder) {
    try { $entries = $Folder.GetFileSystemInfos() }
    catch { throw "can't list $($Folder.FullName), nothing removed: $($_.Exception.Message)" }
    foreach ($entry in $entries) {
        if (($entry.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) { $entry; continue }
        if ($entry -is [System.IO.DirectoryInfo]) { Find-Links $entry }
    }
}

$root = New-Object System.IO.DirectoryInfo $Game
if (($root.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) { throw "$Game is a link itself, not removed" }

$links = @(Find-Links $root)
foreach ($link in $links) {
    if ($link -is [System.IO.DirectoryInfo]) { [System.IO.Directory]::Delete($link.FullName, $false) }
    else { [System.IO.File]::Delete($link.FullName) }
}
$left = @(Find-Links $root)
if ($left.Count -gt 0) { throw "$($left.Count) links are still in $Game, nothing else removed: $($left[0].FullName)" }

# no link is left anywhere in the folder, so the recursive delete can't reach outside it
Remove-Item -LiteralPath $Game -Recurse -Force
"removed $Game ($($links.Count) links taken off first)"
