# OpenGL 4.6 renderer

The default renderer (`gfxrenderer default`, the `modern` backend, `rendering/opengl46/`) runs on an
**OpenGL 4.6 core profile** context. There is no fallback to an older version, and the OpenGL 2.1 (legacy)
renderer and the OpenGL ES / ANGLE path no longer exist.

For how the lights are handled, see [clustered-lighting.md](clustered-lighting.md).

## Requirements

- A graphics card and driver that provide **OpenGL 4.6** (core profile). What the driver offers is written to
  the log at start: `Renderer:`, `Vendor:` and `GL version:`.
- A CPU with AVX2, as before.
- Windows and Linux. macOS provides OpenGL only up to 4.1, so the renderer does not run there.
- The experimental Better Renderer (`gfxrenderer experimental`) is a separate renderer and is not affected.

On Linux, `glxinfo -B` should report an `OpenGL core profile version string` of 4.6 or newer.

### If the simulator does not start

When the context cannot be created, the log (`log.txt`) contains a line like

```
Bad init: failed to create OpenGL 4.6 core profile context (<GLFW error>); the graphics card/driver provides: OpenGL X.Y ..., <renderer> (<vendor>)
```

On Windows the same text is shown in a message box. Updating the graphics driver is the usual fix; a card
which does not support OpenGL 4.6 cannot run this renderer. The process ends with a non-zero exit code.

## eu07.ini

### Removed options

Old `eu07.ini` files still load. These entries are read and ignored, with a line in the log:

| Entry | What happens now |
|---|---|
| `gfxrenderer legacy`, `simple` or any other value except `default`, `full` and `experimental` | the default renderer is used (`eu07.ini: gfxrenderer "X" is no longer supported, using "default"`) |
| `usevbo` | ignored (`eu07.ini: "usevbo" is no longer supported and was ignored`) |
| `gfx.usegles` | ignored, same message |
| `gfx.angleplatform` | ignored, same message |

`gfxrenderer full` is still taken for `default`.

### New options

| Entry | Default | Meaning |
|---|---|---|
| `gfx.lights.clustered yes/no` | `yes` | `yes`: every lit vehicle headlight within 1 km lights the surroundings (see below). `no`: the shading goes through the first `dynamiclights` headlights only, as the old renderer did; useful to compare images or to work around a problem |
| `gfx.lights.clusterheatmap yes/no` | `no` | debug view: the number of lights in each cluster, drawn over the image |

### Changed option

`dynamiclights` (0..7) used to limit the number of headlight sets that light the surroundings. It is still
read, but it only takes effect with `gfx.lights.clustered no`.

## What looks different from the previous renderer

- **No limit of 7 lit headlights.** All lit headlights within 1 km take part in the lighting (each lit end
  of a vehicle counts separately), up to 255 in the buffer and 64 reaching any one part of the view. Beyond
  these capacities the farthest and dimmest lights are dropped, and the statistics show how many.
- **Outside the beam texture a headlight adds nothing.** Before, the edge of the texture was stretched
  beyond the beam, which could leave a faint glow around vehicles. This also applies with
  `gfx.lights.clustered no`.
- **Reflections (environment map)** are lit with the sun as seen from the reflection itself, and vehicle
  headlights no longer light the scene in them.
- **Motion blur and SSAO** read the real depth of the scene. Before, the depth texture was read in a way the
  specification leaves undefined, and the result depended on the driver.
- **Lines are 1 pixel wide**, as they always were on this kind of context. The widths the code asked for
  (catenary wires, map tracks) never took effect.

The editor (F11, orthographic view), the reflection pass and the model previews do not use clusters. The
editor goes through the first `dynamiclights` headlights, the reflections through the sun only.

## Statistics and debugging

- **Shift+F12**, Gfx Renderer: the `lights:` row gives the number of lit headlight sets, how many did not fit,
  and `clustered` or `first N`. With clusters on, `clusters:` (rebuilding of the grid, which happens when the
  projection or the draw range change) and `cull:` (assigning lights to clusters, every frame) show the GPU
  time, and `cluster lists:` the size of the lists and the number of dropped (cluster, light) pairs
  (`overflow`, with its peak). A non-zero overflow means some cluster held more than 64 lights.
- **Light clusters** window (in debug mode, menu *Windows*, item *Headlight config* opens the renderer debug
  windows): the switch for clustered lights, the heat map, the grid state and the same counters.
- `gfx.gldebug yes` turns on the OpenGL debug output (`KHR_debug`); messages go to the log.

## Building

`-DWITH_OPENGL_MODERN=ON` (default) builds this renderer; `WITH_OPENGL_LEGACY` and the GLES / ANGLE options
are gone. Linux builds still use `-DWITH_BETTER_RENDERER=OFF`.

All shaders in `shaders/` can be checked without a graphics card against GLSL 4.60, compute stage included:

```
ci_shadervalidator/build.sh
cd ci_shadervalidator && ./validateshaders
```

It needs `g++` with C++20, `glslangValidator` (Debian/Ubuntu: `glslang-tools`) and the `ref/glm` submodule.
The same check runs in CI for changes to shaders and the `gl/` wrappers.
