# Test game folders

Scripts for running the game or an editor self-test on real game data without changing it: for a measurement, a
check of a change on a scenery, or a test the unit tests can't do. They follow the rules of `AGENTS.md`: the run works
on a copy of the scenery, writes nothing through links into the data, reads none of the player's own settings, and
has a limit on time and memory.

| Script | What it does |
| --- | --- |
| `New-TestGame.ps1` | makes the folder: junctions to the data, copies of the scenery and texture folders a test may change, a profile of its own, `eu07.base.ini` with `convertmodels 0` and `file.binary.terrain no` |
| `Start-TestGame.ps1` | runs the game or a self-test in it, ends it past `-TimeoutMinutes` or `-MemoryLimitGB`, keeps the logs and the screenshots, prints the `SELFTEST` lines |
| `Remove-TestGame.ps1` | takes the junctions off without going into them, then deletes the folder |

Remove a test folder with `Remove-TestGame.ps1` only. A junction is followed by whatever deletes or writes into it,
so `Remove-Item -Recurse` on the folder would delete the game data the junctions lead to.

## Before the first run

- A self-test needs a build with `-DWITH_EDITOR_SELFTESTS=ON`. Its commands are in `application/editormodeselftest*.cpp`;
  `perf <label>` logs the frames since the last `perf`, the time per frame and the timings and counts of the renderer.
- The game binary loads `lua51.dll`, `libpng16.dll`, `libsndfile-1.dll` and `libserialport-0.dll`, which the build
  doesn't put next to it. Pass a folder that has them as `-Binaries`, or copy them into the test folder; a run without
  them ends at once with exit code `-1073741515` (`0xC0000135`, a library not found).
- Turn vsync off for a measurement (`-IniLines 'vsync no'`), or the frame time is the refresh rate of the screen.
- Big sceneries take a long time and much memory to load in the editor: the whole of Wrzosy took about 13 minutes
  (RelWithDebInfo) and went over 20 GB before the load ended. For a measurement of one part, make a small scenery in
  the copy that includes only that part (the sky, the time and the files you need).

## Example

```powershell
.\New-TestGame.ps1 -Data C:\games\pctga -Destination C:\games\test-foliage `
    -CopyScenery linia053_wrzosy -CopyTextures l61_plants -Binaries C:\games\dlls -IniLines 'vsync no', 'fullscreen no'

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
