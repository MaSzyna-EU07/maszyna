# Test game folders

Scripts for running the game or an editor self-test on real game data without changing it: for a measurement, a
check of a change on a scenery, or a test the unit tests can't do. They follow the rules of `AGENTS.md`: the run works
on a copy of the scenery, writes nothing through links into the data, reads none of the player's own settings and
writes nothing into the player's profile, has a limit on time and memory, and the data is checked after every run.

| Script | What it does |
| --- | --- |
| `New-TestGame.ps1` | makes the folder: junctions to the data, copies of the scenery, texture, terrain and vehicle folders a test may change, a profile and caches of its own, `eu07.base.ini` with `convertmodels 0` and `file.binary.terrain no` |
| `Start-TestGame.ps1` | runs the game or a self-test in it, ends it past `-TimeoutMinutes` or `-MemoryLimitGB` of committed memory, keeps the logs and the screenshots, prints the `SELFTEST` lines, fails when a file of the data changed during the run |
| `Remove-TestGame.ps1` | takes every link in the folder off without going into it, then deletes the folder |

Remove a test folder with `Remove-TestGame.ps1` only. A junction is followed by whatever deletes or writes into it,
so `Remove-Item -Recurse` on the folder would delete the game data the junctions lead to. When `New-TestGame.ps1`
fails half way, it removes what it made itself.

## What a run can write, and where

- The game folder of the run: the logs, the screenshots, `physicslog` and `eu07.ini` are real folders and files of
  the test folder.
- `scenery`, `textures`, `terrain` and `dynamic`: only the subfolders named by `-CopyScenery`, `-CopyTextures`,
  `-CopyTerrain` and `-CopyDynamic` are copies; the others are junctions into the data. Name every folder a test
  saves into: the scenery files of an editor self-test that saves, the `terrain\<name>` of a heightmap terrain it
  saves, the vehicle folders `tests\scenarios\run.py` writes its variants into.
- `%APPDATA%\MaSzyna` and `%LOCALAPPDATA%\MaSzyna`: the run's `appdata` and `localappdata` folders.
- Python: `PYTHONDONTWRITEBYTECODE` keeps it from writing `__pycache__` next to the scripts of the data.

Whatever else writes into the data is caught by the check at the end of `Start-TestGame.ps1`: the files and folders
of the data and of the shaders changed since the start are listed, and the script ends with exit code 1.

## Before the first run

- `-Data` is best an SVN working copy. The `.sbt` caches in the root of `scenery` of an installed game (gigabytes on
  a Steam install) aren't copied: with `file.binary.terrain no` a run neither reads nor writes them.
- Junctions need no administrator rights and no Developer Mode, unlike symbolic links, so don't change them to
  symbolic links. They work on local NTFS volumes only: neither the test folder nor the data on FAT, exFAT or a network
  share. A test folder on another local drive than the data is fine.
- A self-test needs a build with `-DWITH_EDITOR_SELFTESTS=ON`. Its commands are in `application/editormodeselftest*.cpp`;
  `perf <label>` logs the frames since the last `perf`, the time per frame and the timings and counts of the renderer.
- Pass the folder of the build as `-Binaries`. The game loads `lua51.dll`, `libpng16.dll`, `libsndfile-1.dll` and
  `libserialport-0.dll`, which the build doesn't put next to it, so the `.dll` files of `-Binaries` are copied into
  the test folder (or pass a folder that has them); a run without them ends at once with exit code `-1073741515` (`0xC0000135`, a library not found). Its `python64`
  folder becomes a junction; without it the Python screens don't start.
- Turn vsync off for a measurement (`-IniLines 'vsync no'`), or the frame time is the refresh rate of the screen.
- Big sceneries take a long time and much memory to load in the editor: the whole of Wrzosy took about 13 minutes
  (RelWithDebInfo) and went over 20 GB before the load ended. For a measurement of one part, make a small scenery in
  the copy that includes only that part (the sky, the time and the files you need).

## Measurements and checks

`Start-TestGame.ps1` runs the game in real time with the renderer on (`-s`, or `-edit` for a self-test), which is
what a measurement needs. A check that is to give the same result every time runs in `--test` mode, with a fixed step
and seed, through `tests\scenarios\run.py`. That one can run on a test folder too (`--game-dir`), with the vehicle
folders of its cases named in `-CopyDynamic`, so it doesn't touch an installed game.

## Example

```powershell
.\New-TestGame.ps1 -Data C:\games\pctga -Destination C:\games\test-foliage `
    -CopyScenery linia053_wrzosy -CopyTextures l61_plants -Binaries C:\src\maszyna\build\bin\RelWithDebInfo `
    -IniLines 'vsync no', 'fullscreen no'

# a small scenery in the copy: the terrain and the plants of Wrzosy only
Set-Content C:\games\test-foliage\scenery\foliage_test.scn -Encoding ascii -Value @(
    'sky skydome_clouds.t3d endsky',
    'atmo 0 0 0 8500 10200 0.737 0.812 0.812 0.37 endatmo',
    'time 09:24 05:48 19:23 endtime',
    'include linia053_wrzosy/scm_wrzosy/6-teren.scm end',
    'include linia053_wrzosy/scm_wrzosy/9-flora.scm end')

# the script: a view, a warm-up, the measurement, a screenshot
Set-Content C:\games\foliage.selftest -Encoding ascii -Value @(
    'camera -560 25 -14400 -610 5 -14500',
    'wait 180', 'perf warmup', 'wait 300', 'perf near',
    'screenshot', 'wait 15', 'quit')

.\Start-TestGame.ps1 -Game C:\games\test-foliage -Exe C:\src\maszyna\build\bin\RelWithDebInfo\eu07.exe `
    -Scenery foliage_test.scn -Selftest C:\games\foliage.selftest -MemoryLimitGB 16 -Output C:\games\out\near

.\Remove-TestGame.ps1 -Game C:\games\test-foliage
```

`wait` counts frames. Leave about 15 frames after `screenshot` before the next command: the picture is written later,
on another thread.
