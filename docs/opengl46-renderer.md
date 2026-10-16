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

### Sky light and exposure

All of these are optional; a missing entry takes the default. The switches can also be changed while the simulator
runs, in the **Sky and exposure** debug window. With every switch set to `no` the image is as before these options
existed.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.ambient.sh yes/no` | `yes` | the ambient light depends on where a surface faces: more from the bright part of the sky, less from the ground below. The average stays the ambient light the simulation works out |
| `gfx.ambient.sh.strength` | `1.0` | 0..2: how much the ambient light differs between directions (0 = uniform, as before) |
| `gfx.ambient.groundalbedo` | `0.1` | 0..1: how much light the ground below the horizon gives back (to the directional ambient light and to the atmosphere) |
| `gfx.envmap.ggx yes/no` | `yes` | rough reflections read the environment map prefiltered with the GGX lobe (made in a compute pass after each update of the map) instead of its plain mipmaps |
| `gfx.atmosphere yes/no` | `yes` | the sky, the colour of the sunlight, the ambient light and the fog colour come from a physical model of the atmosphere (Hillaire 2020) during the day; the old sky takes over between 10 degrees of sun elevation and 6 degrees below the horizon, and at night. Render only: the simulation keeps its own values |
| `gfx.atmosphere.sky` | `1.0` | brightness of the drawn sky and of the fog colour (1 = calibrated against the old sky) |
| `gfx.atmosphere.ambient` | `1.0` | brightness of the ambient light from the atmosphere |
| `gfx.atmosphere.sun` | `1.0` | brightness of the sunlight from the atmosphere |
| `gfx.atmosphere.haze` | `1.0` | amount of aerosols on a clear day (whiter, hazier sky); the weather adds to it with the overcast |
| `gfx.atmosphere.altitude` | `200` | height of the scenery above the sea level, metres |
| `gfx.postfx.autoexposure yes/no` | `yes` | the exposure follows the average brightness of the image (luminance histogram, compute pass) |
| `gfx.postfx.autoexposure.key` | `0.3` | average luminance of the image the exposure is set for |
| `gfx.postfx.autoexposure.min` | `-1.0` | the most the exposure darkens a bright image, EV |
| `gfx.postfx.autoexposure.max` | `0.5` | the most the exposure brightens a dark image, EV. Kept low on purpose: tunnels and the night are not meant to be brightened much |
| `gfx.postfx.autoexposure.speed.dark` | `1.0` | rate of adaptation to a darker image, 1/s |
| `gfx.postfx.autoexposure.speed.bright` | `3.0` | rate of adaptation to a brighter image, 1/s |
| `gfx.postfx.autoexposure.compensation` | `0.0` | fixed exposure correction, EV; applies also with the automatic exposure off |
| `gfx.postfx.bloom yes/no` | `yes` | glow around bright parts of the image |
| `gfx.postfx.bloom.intensity` | `0.04` | share of the glow in the image |
| `gfx.postfx.bloom.threshold` | `0.0` | luminance below which nothing glows; 0 = everything contributes a little (physically based) |

The automatic exposure and the bloom need the post processing pipeline (`gfx.skippipeline no`, the default).
Model previews of the editor use none of these.

### Ambient occlusion

Darkening of creases, corners and the ground under objects, worked out from the depth of the image (screen space)
and multiplied into the image before the motion blur and the tonemapping. It works with and without motion blur.
It darkens all light alike, the sun and the headlights included; limiting it to the ambient light needs the depth
before the shading (a depth prepass), which is planned later.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.postfx.ao gtao/ssao/no` | `gtao` | `gtao` (or `yes`): ground truth ambient occlusion (Jimenez et al. 2016), at half the resolution with a depth aware smoothing and upsampling. `ssao`: the older, experimental ssao. `no`: none, and no targets are made for it; the mode can then not be turned on in the debug window |
| `gfx.postfx.ao.radius` | `1.5` | 0.1..10: reach of the occluders, metres |
| `gfx.postfx.ao.strength` | `1.0` | 0..4: power the occlusion is raised to; 1 = as computed, 0 = none |
| `gfx.postfx.ao.bounce` | `0.3` | 0..1: albedo of the surfaces assumed for the light that comes back after more bounces (brightens the occlusion a little) |
| `gfx.postfx.ao.quality slices steps` | `3 4` | directions searched around each pixel (1..8) and samples on each side of a direction (1..16) |

### Lamps

With the clustered lights on, lamps light their surroundings, besides the sun and the vehicle headlights. Without the
clustered lights (`gfx.lights.clustered no`) none of this happens and the cab is lit as before. The switches and
scales can also be changed in the **Light clusters** debug window.

- **Light points of models** (`freespotlight` submodels: street and platform lamps, signals, lamps of vehicles) are taken
  the way the experimental renderer takes them. A point which is visible, has self-illumination (`SelfIllum` `true`, or a
  threshold the light level is below) shines as a spot light along the -z axis of its submodel, in the cone of its glare
  (`FalloffAngle`, full brightness within `HotspotAngle`), in its colour; `hotspotpower` scales it. Lamps of scenery
  models switched with `Light_On` / `Light_Off` (`lights` of the model node, events) turn their light points on and off
  with them. Nothing in the models has to change; a light point without self-illumination stays a glare only.
- **Lit compartments of vehicles** (the `cab`, `corridor`/`korytarz` and `compartment`/`przedzial` sections of the low
  poly interior) give a point light in the middle of the section, in the colour of the interior light, unless the
  section holds light points of its own (these light it then, through its self-illumination).
- **The cab lamp** of the occupied cab is a point light under the top of the cab model. Before, it added to the ambient
  light of the whole cab and weakened the shadows of the sun in it; now a share of that ambient light is kept for the light
  which bounces around the cab (`gfx.lights.cab.bounce`), and the sun shadows stay.

The lamps of models and compartments fade in from the light level at which scenery lamps switch on (`ls_Dark`) to full
darkness; in daylight they are not gathered at all. Of these lights only the cab lamp casts shadows (see below); the others
also light what is behind walls within their reach, which is kept short for compartments.

**Shadows of the cab lamp** (`gfx.lights.shadows`): the cab casts shadows in the light of its lamp, the desk, the seats,
the levers and the walls (light leaves the cab only through the windows, glass doesn't stop it). They come from a cube
shadow map around the lamp, drawn with the opaque parts of the cab model only. The map is kept in the space of the cab, so
it stays right as the vehicle moves; it is drawn anew every few frames, for the moving parts of the cab, and at once when
the lamp turns on or the cab changes. All six faces are drawn in one go, so drawing it costs about as much as drawing the
cab once more. Nothing is drawn with the lamp off (in the daylight it is). The level:

| Level | Map | Drawn | Edges |
|---|---|---|---|
| `no` (0) | none | never | lamp without shadows, as before |
| `low` (1) | 512 x 512 per face, 6 MB | every 4th frame | hard (2x2 filter) |
| `medium` (2, default) | 1024 x 1024 per face, 24 MB | every 2nd frame | soft, 4 samples |
| `high` (3) | 1024 x 1024 per face, 24 MB | every frame | softer, 8 samples |

| Entry | Default | Meaning |
|---|---|---|
| `gfx.lights.scenery yes/no` | `yes` | light points of scenery models light their surroundings |
| `gfx.lights.vehicles yes/no` | `yes` | light points of vehicles and their lit compartments light their surroundings |
| `gfx.lights.cab yes/no` | `yes` | the cab lamp is a point light; `no`: it adds to the ambient light of the cab, as before |
| `gfx.lights.freespot.intensity` | `10` | brightness of a light point (the experimental renderer uses the same default) |
| `gfx.lights.compartment.intensity` | `1.0` | brightness of the light of a lit compartment |
| `gfx.lights.cab.intensity` | `1.5` | brightness of the cab lamp |
| `gfx.lights.cab.bounce` | `0.3` | 0..1: share of the former ambient light of the cab lamp kept with the lamp on |
| `gfx.lights.range` | `300` | distance from the camera the lamps of models and vehicles are gathered from, metres |
| `gfx.lights.shadows` | `medium` | `no`/`low`/`medium`/`high` (or 0..3): shadows of the cab lamp, see above |

### Changed option

`dynamiclights` (0..7) used to limit the number of headlight sets that light the surroundings. It is still
read, but it only takes effect with `gfx.lights.clustered no`.

## What looks different from the previous renderer

- **No limit of 7 lit headlights.** All lit headlights within 1 km take part in the lighting (each lit end
  of a vehicle counts separately), up to 1023 lights in the buffer (headlights, lamps and the cab lamp) and 64 reaching
  any one part of the view. Beyond
  these capacities the farthest and dimmest lights are dropped, and the statistics show how many.
- **Outside the beam texture a headlight adds nothing.** Before, the edge of the texture was stretched
  beyond the beam, which could leave a faint glow around vehicles. This also applies with
  `gfx.lights.clustered no`.
- **Reflections (environment map)** are lit with the sun as seen from the reflection itself, and vehicle
  headlights no longer light the scene in them.
- **Motion blur and ambient occlusion** read the real depth of the scene. Before, the depth texture was read in a way the
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
  windows): the switch for clustered lights, the heat map, the grid state and the same counters, and the switches and
  scales of the lamps with their counts. Shift+F12 shows the lamps in the `local lights:` row: how many were gathered (from
  scenery models, vehicles and compartments, and the cab lamp), how many were placed in the light buffer of the main view
  (those that may light something in view) and did not fit, and the cpu time of gathering them. The `lamp shadows:` row
  gives the size of the cube map, whether it holds the cab lamp, the draw calls and cpu time of its last drawing, and the
  timings the GPU time of drawing it (`lamp shadows:`); the level can be changed in the window.
- **Sky and exposure** window (next to the Light clusters window): the switches and scales above, the share of the
  atmosphere, the colours of the sun, ambient light and fog in use, and the measured luminance and exposure.
  Shift+F12 shows the GPU time of `atmosphere:` (look-up tables, only when the sun moved or the weather changed),
  `envmap ggx:` (prefiltering, with each update of the environment map), `exposure:` and `bloom:`. The same window holds
  the ambient occlusion mode and its settings; Shift+F12 shows its time as `ambient occlusion:` (gtao only).
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
