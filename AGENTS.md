# MaSzyna: notes for coding agents

These are the instructions for every coding agent that works in this repository: Claude Code, OpenAI Codex, Gemini CLI and any
other tool that reads `AGENTS.md`. `CLAUDE.md` and `GEMINI.md` only point to this file, so a rule is added or changed here, never
there.

## The code builds and works with GCC, not only MSVC

The Linux CI (`.github/workflows/build.yml`) builds with GCC 13 on ubuntu-24.04, through the Sonar build-wrapper, and `sonar.yml`
analyses that build once it has passed. A change that compiles with MSVC only fails there, and a failed GCC build also leaves Sonar
without an analysis of the PR. So every change has to compile with GCC, and behave the same with it, as well as with MSVC. The CI
builds with `-DWITH_BETTER_RENDERER=OFF` and Sonar excludes `betterRenderer/`, so that part is neither built on CI nor analysed.

Where no GCC is at hand, read the new code for what MSVC lets through and GCC doesn't:

- `constexpr` initialised from a floating or class-type constant declared only `const`: make the source `constexpr` too.
- Standard headers MSVC pulls in through others: include them yourself. For example, `<ranges>` for `std::views`, `<format>`,
  `<numbers>`, `<span>`, `<cstring>`, `<cfloat>` and `<bit>`.
- `typename` and `this->` where a template needs them.
- A temporary bound to a non-const reference.
- Explicit specialisations inside a class.
- A name a class uses and then declares again in its own body (GCC: "changes meaning").
- A nested type with default member initialisers used as a default argument inside the class that encloses it.
- A declaration changed in a header (`const`, parameters) without its definition, or a new member function defined but not declared.
- `std::ifstream` / `std::ofstream` opened with a `std::wstring`.
- MSVC-only functions: `_stricmp`, `sprintf_s`, `fopen_s`, `localtime_s` and the like.
- Designated initialisers out of declaration order.
- `emplace_back(a, b, c)` with parenthesised aggregate initialisation is fine on GCC 13, but not for `std::array`. There is no brace
  elision in parentheses, so build a named local instead.

The same behaviour needs portable code. Don't rely on the order arguments are evaluated in, on the signedness of `char`, on the size of
`wchar_t`, or on `long` being 32 bits.

## SonarCloud: the PR passes the quality gate

SonarCloud (`sonar.yml`, `sonar-project.properties`) analyses each PR. A PR has to pass Sonar's quality gate. The gate rates the new
code: reliability, security and maintainability have to be A, duplication is at most 3% and the security hotspots are reviewed.
Any bug or vulnerability on the new code lowers its rating below A, so each one gets fixed (or gets a NOSONAR when it is a false
positive). Code smells count only together, through the maintainability rating, so a single one rarely fails the gate, but many in a
large PR can. A hotspot is reviewed by marking it in SonarCloud as safe or fixed, which is the user's step there, not a reason to
rewrite the code. A remark that doesn't fail the gate doesn't have to be fixed. Leave large restructurings that only answer such a
remark (splitting a class or a file, rewriting a working algorithm) out of the PR. The analysis runs on the GCC build, so a PR that
fails to build with GCC gets no analysis at all, and so it doesn't pass the gate.

Fixing a remark must not change what the code does:

- Keep the order of side effects, the conditions and the floating-point results as they were.
- Text in `STR(...)`, `STR_C(...)` and `_(...)` is looked up in the translations (`lang/*.po`), so it stays byte for byte the same.
  The same goes for what is written to the log.
- `// NOSONAR: <reason>` is only for a real false positive, or where a fix would change behaviour. Use it rarely, always with the
  reason.

To keep the gate passing, write new code so that it doesn't raise the rules that keep coming up:

- **S134** (nesting deeper than 3 `if` / `for` / `while` / `switch` / `try`) and **S3776** (cognitive complexity over 25):
  - use early `continue` / `return`;
  - move loop bodies and branches into named helpers, preferably free functions in the anonymous namespace;
  - group the state of a long algorithm in a small struct with methods.

  Every `else if`, every `&&` / `||` sequence and every nested lambda adds to the complexity.
- **S1188**: a lambda over 20 lines becomes a named function.
- **S107**: a function with more than 7 parameters takes a struct.
- **S3608**: lambdas capture explicitly (`[this, &points]`, not `[&]` or `[=]`). The only exception is `[&]` on a lambda passed
  straight to a standard algorithm; `[=]` is flagged even there, and so is `[&]` passed to one of our own functions.
- **S6197** / **S5566**: use the `std::ranges::` algorithm that takes the container rather than a begin/end pair or a hand-written
  loop.
- **S6165**: use `std::erase_if` rather than erase-remove.
- **S6004**: a variable used only by the condition is declared in the `if (init; condition)`.
- **S3358**: no nested `?:`; use an `if` or a small function.
- **S1121**: no assignment inside an expression, chained ones included (`a = b = c`).
- **S1117**: no declaration shadowing a local, a member or an enumerator.
- **S6003**: `v.emplace_back(a, b, c)` rather than `v.push_back({a, b, c})`. For a `std::array` element, push a named local instead
  (see the GCC notes above).
- **S6012**: let class template argument deduction work: `std::vector values{1.0, 2.0}`.
- **S5945**: `std::array` / `std::vector` / `std::string`, not C arrays.
- **S1659**: one declaration per statement.
- **S6009**: `std::string_view` rather than `std::string const &` for a string parameter that is only read.
- **S6185** / **S6494**: `std::format` rather than concatenation or `snprintf`.
- **S924**: at most one `break` per loop.
- **S886**: a `for` whose counter changes in its body becomes a `while`.
- **S5997**: `std::scoped_lock` rather than `std::lock_guard`.
- **S6168**: `std::jthread` rather than `std::thread`. A `std::jthread` joins in its destructor where a `std::thread` would
  terminate, so in existing code switch only where the thread is always joined or detached before it is destroyed.
- **S6177**: `using enum` where a function names many enumerators.
- **S5350** / **S995**: `const` on references and pointers that aren't written through.
- **S5817**: `const` on member functions that change nothing.
- **S2193**: no `float` loop counters.
- **S5421**: no non-const globals.
- **S6179**: `std::lerp` rather than a hand-written interpolation, in new code. In existing code only where it gives the very same
  result.
- **S6164**: `std::numbers::pi` rather than a literal, in new code. In existing code only where it gives the very same result.

The project is C++20 (`CMAKE_CXX_STANDARD 20`), so no C++23 library addition is available on any compiler. That rules out
`std::ranges::to`, `views::zip`, `views::enumerate`, `ranges::contains`, `std::print` and `std::expected`.

## Renderers: the move to OpenGL 4.6 and Vulkan

The renderers on `master` are to be replaced. `feat/gl46-migration` (PR #216, a draft with milestone 27.04, to be merged after the
26.11 release) replaces the OpenGL 3.3 renderer with an OpenGL 4.6 core one and adds a Vulkan 1.4 renderer beside it. It also
removes the legacy OpenGL 2.1, the OpenGL ES / ANGLE and the NVRHI (`betterRenderer/`) paths.

- `betterRenderer/` will be removed. Don't put work into it.
- On that branch, of the `rendering/opengl*` files only `openglcamera.*` and `openglmatrixstack.*` stay. The whole
  `rendering/opengl33*` family and the rest of the legacy `rendering/opengl*` go, and the new renderers are in
  `rendering/opengl46/`, `rendering/vulkan14/` and `rendering/common/`. The shared `rendering/renderer.*`, `geometrybank.*`,
  `particles.*` and the like stay. The shaders move from `shaders/` to `shaders/ogl46/` and `shaders/vk14/`.
- So a change on `master` to the code that goes away, or to the shaders, may be pointless. Keep it to fixes and to what a feature
  on `master` needs in order to be drawn at all. For such a change, say in the PR where `feat/gl46-migration` has the counterpart,
  or that the branch already carries the change.

## Building, testing and running

- The unit tests (`tests/unit`) run at the end of the build, which prints `Unit tests: N/N passed` and fails on any failed case.
  `-DWITH_TESTS=OFF` builds the game without them.
- A new feature goes on its own branch from `master`, not on whatever branch is checked out. Say which branch the work lands on
  before the first edit. A fresh worktree has empty submodules, so it doesn't build until they are filled.
- The editor runs a script when `EU07_EDITOR_SELFTEST=<file>` is set: one command per line, `#` for comments. The commands are in
  `application/editormodeselftest*.cpp` (`camera`, `wait`, `select`, `lay`, `screenshot`, `save`, `quit` and many more).
  - Run it on a copy of the scenery, never on the original, and delete the copy afterwards.
  - Give every run a timeout and keep an eye on its memory. A loop gone wrong can fill the RAM of the whole machine.
  - `screenshot` only queues the grab and waits 3 frames; the image is written later on another thread. A click right after it
    has been seen to lose the selection, so leave a `wait` of about 15 frames after a `screenshot` before clicking.
- At run time the editor's texts load from `lang/editor_<language>.po` in the game folder (`utilities/translation.cpp`), not from
  the build. New strings of a branch show in English until that file has them. Without the `fonts` folder Polish letters show as `?`.
- When a build runs on a copy of the game data made of links to the real folders, turn off whatever writes into the data
  (`convertmodels 0` in `eu07.ini`), or the run changes the real game folder through the links.
- A test of code that moves, migrates or deletes files in the user's profile (`%APPDATA%`, `%LOCALAPPDATA%`, caches) gets its own
  test folder. Never point it at the real profile.
- A GPU hang (Windows TDR) can freeze the whole machine, not just the game. Don't test a hang hypothesis by running it. Guard the
  code against what causes it (degenerate geometry, NaN in the camera or in transforms) and read the `LiveKernelEvent` / watchdog
  dumps instead.
- PowerShell variable names are case-insensitive: `$Track` and `$track` are the same variable.

## Game data and backward compatibility

The code and the game data live apart. The assets (sceneries, vehicles with their `.fiz` and `.mmd`, models, textures, sounds,
timetables, most of `lang` and the shipped `eu07.ini`) are in a separate SVN repository, `https://svn.eu07.pl/pctga`. It needs an
account; there is no anonymous checkout. Of `lang/`, only `editor_pl.po` is in this repository. Players install and
launch the game through Steam, GOG and the like, and run new builds on the data they already have.

The official starter is StarterNG (`https://github.com/MaSzyna-EU07/StarterNG`), a separate repository; the older
`MaSzyna-EU07/Starter` is not the one in use. `launcher/` in this repository is the simulator's built-in scenario and vehicle
picker (`launcher_mode`), not StarterNG.

- Every existing file must still be read the way it was. Don't rename or drop a keyword, change the meaning or the unit of a value,
  or make an optional entry required. This holds for every format the game reads: for example `.scn` and its includes, `.fiz`,
  `.mmd`, `.e3d` / `.t3d`, `.mat`, the `.txt` timetables and track profiles, `eu07.ini` and the Python screens.
- A bug fix may change what an existing file leads to, such as how a vehicle from its `.fiz` behaves, as long as the PR says so.
- Something new in a format is optional, and without it the code does exactly what it did before. A file that loaded correctly
  before doesn't get new warnings or errors in the log. A real fault that is newly detected may be logged.
- A code change must not need a change in the SVN to work. Where it uses new or changed assets, it falls back to the old behaviour
  without them, and the PR says what is to be added to the SVN and in what order the two go in.
- A change that uses new or changed assets (and falls back without them, see above) says so plainly in its commit message: which
  files, and what is to change in them. It is never left implied. Before such a commit, either make the asset change right away in
  the user's SVN working copy, or get the user's agreement to leave it for later. Ask the user where their SVN working copy is; the
  installed game folder is not one. Pushing to git and committing to the SVN both need the user's consent.
- A scenery saved by the editor without using a new feature holds nothing the current release can't read. The entries of a new
  feature (such as a new kind of node) appear only where that feature was used, and the PR says that the release doesn't read
  them.
- Saving patches the scenery files by byte spans: unchanged definitions stay byte for byte, and an edited node is written anew by
  its `export_as_text`. A change to saving must not widen what gets rewritten, nor make `export_as_text` drop more of an edited
  node (comments, unknown tokens) than it did before.
- A new feature that changes what the simulator does or shows with existing sceneries and vehicles can be turned off in
  `eu07.ini`, and turned off, the simulator behaves as it did before the feature. Editor tools and new optional entries of a
  format are covered by the points above. The key is parsed in `utilities/Globals.cpp` (`global_settings::ConfigParse*`) and
  written by `global_settings::export_as_text`, which puts the settings in the log at start. `eu07.ini` is read from the user's
  profile (`user_config_path`), and from the game folder only when the profile has none; test the switch in the file that is
  actually read.
- A feature doesn't make existing sceneries and vehicles look suddenly different after an update. A change to how things look is
  something the player turns on, or comes in gradually.
- When a format really has to change, read both the old and the new form, and say in the PR what changes for the data and for
  the people who make it.
