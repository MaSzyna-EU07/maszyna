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
| `gfx.postfx.autoexposure.max.cab` | `2.0` | the same seen from the cab by day: the eye takes to the light of the cab, which the bright windows would leave dark. It goes down to `gfx.postfx.autoexposure.max` with the daylight around the cab (the light level of the simulation, the occlusion of the sky): at night and in tunnels it's that one |
| `gfx.postfx.autoexposure.speed.dark` | `1.0` | rate of adaptation to a darker image, 1/s |
| `gfx.postfx.autoexposure.speed.bright` | `3.0` | rate of adaptation to a brighter image, 1/s |
| `gfx.postfx.autoexposure.compensation` | `0.0` | fixed exposure correction, EV; applies also with the automatic exposure off |
| `gfx.postfx.bloom yes/no` | `yes` | glow around bright parts of the image |
| `gfx.postfx.bloom.intensity` | `0.04` | share of the glow in the image |
| `gfx.postfx.bloom.threshold` | `0.0` | luminance below which nothing glows; 0 = everything contributes a little (physically based) |
| `gfx.postfx.whitebalance yes/no` | `yes` | white balance of the camera, applied to the image before the tonemapping (chromatic adaptation in the Bradford cone space) |
| `gfx.postfx.whitebalance.temperature` | `7500` | 4000..25000 K: colour temperature of the light the camera takes as white. 6500 = no change; more makes the image warmer, less makes it cooler (as in a camera or a photo editor) |
| `gfx.postfx.whitebalance.tint` | `0.0` | -1..1: shift off the daylight colours; + makes the image more magenta, - more green |

The automatic exposure and the bloom need the post processing pipeline (`gfx.skippipeline no`, the default).
Model previews of the editor use none of these.

### Anti-aliasing

| Entry | Default | Meaning |
|---|---|---|
| `gfx.antialiasing off/msaax2/msaax4/msaax8/taa` | `msaax4` | `msaax2`..`msaax8`: multisampling of the colour pass. `taa`: temporal anti-aliasing of the main view: one sample per pixel, the projection moved by a part of a pixel every frame (8 offsets) and the frame blended into the history of the last frames along the velocities of the pixels. Also in the Vulkan renderer. Read at the start; the older `multisampling 0..4` still works, the entry later in the file wins |

TAA needs the post processing pipeline (`gfx.skippipeline no`); extra viewports and the eyes of the VR get no
anti-aliasing with it. The velocities are drawn with it whether the motion blur is on or not; with both on the
motion blur smears the image after the TAA, and it also blurs the turning of the camera. Thin lines across the
pixels (rails and wires far away) are locked in the history for 8 frames on the world standing still farther
than 20..40 m, so the frames whose offset misses them don't make them flicker; on near or fast moving pixels the
usual check of the history against the colours around the pixel applies. Further anti-aliasing modes (FSR) are meant to be added to
the same entry.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.antialiasing dlss/dlaa` | | NVIDIA DLSS of the Vulkan renderer: the scene of the main window drawn at `gfx.buffer.percent` of the window with one sample, the jitter of `taa` (more offsets the smaller the image: 8 for each pixel of the window) and anti-aliased and upscaled to the window by DLSS. `dlaa` is DLSS at the full size of the window whatever `gfx.buffer.percent` says. Needs a build with `WITH_DLSS`, a GeForce RTX and its driver; otherwise, and with OpenGL, `taa` is taken at the start (the log says why). Should DLSS fail later, the scene is stretched to the window without anti-aliasing (the negative bias of the mipmaps stays until the restart) |
| `gfx.buffer.percent` | `1.0` | the size of the image the scene of the main window is drawn at, as a part of the size of the window (`0.5`: half its width and half its height; a value over 4 is taken as percents). With `dlss` it's kept within what the DLSS mode nearest to it takes (about 0.33..1.0) and upscaled by it; with the other modes the image is stretched to the window. Replaces `gfx.framebuffer.width` and `gfx.framebuffer.height`, which aren't read any more. Read at the start |
| `gfx.dlss.preset default/j/k/l/m` | `default` | the preset of the DLSS model (the DLSS SDK 310 has these; `default` lets the library pick per mode) |
| `gfx.dlss.autoexposure yes/no` | `no` | DLSS works the exposure out itself instead of being given the exposure of the tonemapping |
| `gfx.dlss.lodbias` | `0` | added to the bias of the mipmaps of the textures of the scene with DLSS, `log2(drawn / window) - 1` (less negative against moiré, more for sharper textures) |
| `gfx.dlss.developer yes/no` | `no` | the development library of DLSS (`ngx_dev/` next to the program): its indicator and the debug keys of the DLSS programming guide (Ctrl+Alt+F12 debug views, Ctrl+Alt+F9 jitter configurations) |
| `gfx.framegeneration off/x2/x3/x4` | `off` | DLSS Frame Generation of the Vulkan renderer, with `dlss` or `dlaa`: one (`x2`) to three (`x4`) frames made by DLSS between two frames drawn, shown before the frame drawn and spaced over the time of a frame. Needs a GeForce RTX of the 40 series (`x2`) or the 50 series (`x3`, `x4`; a gpu which can't is given the most it can) and `nvngx_dlssg.dll` / `libnvidia-ngx-dlssg.so.<version>` next to the program; otherwise the frames aren't generated (the log says why). Turns on Reflex. The presents are spaced by the render thread (`gfx.vulkan.renderthread`); without it they go at once, which works with vsync. The motion blur is shortened as many times. Read at the start |
| `gfx.reflex off/on/boost` | `off` | NVIDIA Reflex of the Vulkan renderer (`VK_NV_low_latency2`): the driver keeps the cpu from running ahead of the gpu, the main loop waits for it before the input of a frame; `boost` keeps the clocks of the gpu up too. On by itself with `gfx.framegeneration`. Read at the start |

DLSS needs the NVIDIA DLSS SDK at build time, which isn't in the repository (it keeps the binaries of every platform in
git): `cmake -P tools/fetch_dlss_sdk.cmake` takes the headers and the libraries of the platform into `ref/DLSS`
(ignored by git), then configure with `-DWITH_DLSS=ON` (`-DDLSS_SDK_ROOT=<dir>` for another place). The build copies
`nvngx_dlss.dll` (Windows) or `libnvidia-ngx-dlss.so.<version>` (Linux) next to the program, where NGX looks for it,
with `nvngx_dlssg.dll` / `libnvidia-ngx-dlssg.so.<version>` of the frame generation, and the development libraries into
`ngx_dev/`; a program copied elsewhere needs the libraries next to it. The velocities DLSS
is given are those of `taa`, the sky (which has none drawn) gets the motion of the camera; animated parts of models other
than wheels (gauges, levers, wipers) have no velocity of their own yet. The debug window of the Vulkan renderer shows the
mode and the sizes, and can turn the jitter offsets the other way (to check their direction) and drop the history.

The frame generation is given the frame of the main window with the user interface and without it (the "hudless", so it
tells the user interface apart), the depth and the velocities of the scene turned the way up of the window, and the
camera without the jitter. It makes its frames into images of its own, copied to images of the swapchain acquired with
the frame (the swapchain of the main window has one image more for each of them). If it fails, the frames it was to make
are the frame drawn and it stays off. Its debug keys (Ctrl+Shift+V, Ctrl+Shift+T, see the DLSS-FG programming guide) are
in the development library (`gfx.dlss.developer yes`); the debug window can turn the clip space of its matrices over,
should its test of the matrices show the camera motion the wrong way.

### Ambient occlusion

Darkening of creases, corners and the ground under objects, worked out from the depth of the image (screen space).
With `gfx.postfx.ao.lighting yes` (the default, OpenGL and Vulkan) the ground truth occlusion is worked out from the depth
of the whole scene drawn before its colour (it turns that depth pass on, like the contact shadows and the screen space
reflections) and the shading takes it into the ambient light only: the diffuse ambient light, with the light coming back
from more bounces off surfaces of their own albedo, and the reflection of the surroundings (specular occlusion,
Lagarde 2014). The sun, the headlights, the lamps and the reflection of the scene the screen space reflections find
aren't darkened. Surfaces the depth pass doesn't hold (translucent ones, the grass, the cab drawn after rain or snow,
the edges of objects) get no occlusion. With `gfx.postfx.ao.lighting no`, or with `ssao`, the occlusion is multiplied
into the image before the motion blur and the tonemapping, darkening all light alike, as before.

Cost: with `lighting` the occlusion itself costs the same; the depth pass of the scene costs CPU in OpenGL when nothing
else turned it on (`gfx.ssr`, `gfx.contactshadows`, `gfx.depthprepass.scene`; with `gfx.depthprepass.replay yes` it is
the walk of the colour pass recorded, not a second walk), and every shaded pixel reads the occlusion. `gfx.postfx.ao.lighting no`
is the cheaper setting.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.postfx.ao gtao/ssao/no` | `gtao` | `gtao` (or `yes`): ground truth ambient occlusion (Jimenez et al. 2016), at half the resolution with a depth aware smoothing and upsampling. `ssao`: the older, experimental ssao. `no`: none, and no targets are made for it; the mode can then not be turned on in the debug window |
| `gfx.postfx.ao.radius` | `1.5` | 0.1..10: reach of the occluders, metres |
| `gfx.postfx.ao.strength` | `1.0` | 0..4: power the occlusion is raised to; 1 = as computed, 0 = none |
| `gfx.postfx.ao.bounce` | `0.3` | 0..1: albedo of the surfaces assumed for the light that comes back after more bounces (brightens the occlusion a little); over the image only, in the light of the surfaces their own albedo is taken |
| `gfx.postfx.ao.cab` | `0.5` | 0..1: share of the occlusion the parts of the cab and the low poly interiors take (with `lighting`): walls and things are near all around in them and the whole occlusion darkens them too much; the light which bounces inside them comes from the probe of the cab (`gfx.cabprobe.sun`) or `gfx.interior.bounce`. 1: all of it, as before |
| `gfx.postfx.ao.lighting yes/no` | `yes` | with `gtao`: the occlusion darkens the ambient light and the reflection of the surroundings in the shading (needs the depth of the scene before its colour); `no`: it darkens the finished image |
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
  which bounces around the cab (`gfx.lights.cab.bounce`), and the sun shadows stay. With the probe of the cab (below) the
  light which bounces comes from the probe instead and nothing of the lamp is kept in the ambient light.
- **Lights of the cab model.** Every light point (`FreeSpotLight`) of the cab model lights the cab, in place of the lamp
  under the top of the cab, which a cab model gets only when it has no light points at all. They are switched by the
  visibility of the submodels they are children of: under the submodel of `i-cablight:` a light point shines with the
  lamp of the cab, under `i-dashboardlight:` with the dashboard light, and so on (one not under any switched submodel
  shines always). Their colour, power, cone and the optional `Radius:`/`SoftFalloff:`/`CastShadows:` are those of the light
  point (brightness `gfx.lights.freespot.intensity`, the self-illumination threshold doesn't matter), and they fade in the
  daylight as the cab lamp does. The dimming of the cab lamp applies only to the lamp under the top.

The lamps of models and compartments fade in from the light level at which scenery lamps switch on (`ls_Dark`) to full
darkness; in daylight they are not gathered at all. The cab lamp and the lamps of scenery models nearest to the camera cast
shadows (see below); the others (lamps of vehicles, compartments, farther scenery lamps) also light what is behind walls
within their reach, which is kept short for compartments.

**Shadows of the cab lamp** (`gfx.lights.shadows`): the cab casts shadows in the light of its lamp (and of the brightest
`gfx.lights.shadows.cab` lights of the cab model, each with a map of its own, made as they're needed; the other lights of the
cab take the map of the closest light with one; for the light points the near plane is 3 cm, or their `Radius:`, so the desk
right under its lights holds their light back; a few maps are redrawn per frame), the desk, the seats,
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

**Shadows of scenery lamps** (with `gfx.lights.shadows` above `no`): the street, platform and other lamps of scenery models
nearest to the camera (`gfx.lights.shadows.lamps` of them, within `gfx.lights.shadows.lamps.range` of the camera) cast the
shadows of the scenery around them, of the vehicles near the camera and of the occupied cab: light from a lamp outside comes
into the cab only through the windows. Each such lamp has a cube shadow map in world axes around it, in two parts:
- the scenery (terrain, tracks, models; not the model of the lamp itself, which would shadow everything), drawn once when the
  lamp gets its map and again only when the scene changes (events showing or hiding models, the editor). At most one map is
  drawn per frame (two at `high`), so lamps coming into reach get their shadows over a few frames;
- a copy of the scenery with the vehicles within `gfx.lights.shadows.vehicles.range` of the camera (and the cab) added,
  made every few frames (as for the cab lamp) while any of them is within the reach of the lamp. A vehicle farther from
  the camera casts no shadow of the lamps; with no vehicle near a lamp nothing is drawn for it.

Lamps of vehicles and lit compartments cast no shadows. The memory taken is 2 maps per lamp: with the defaults (8 lamps,
512 x 512 per face) about 100 MB of video memory.

**Shadows of headlights** (with `gfx.lights.shadows` above `no`): the headlights of the vehicles cast the shadows of what is
in their beam, other vehicles, signals, poles, platforms, the walls of a cutting. The beam of a set of headlights (one end of
a vehicle) is projected from behind the vehicle, as a narrow cone along it which starts at its end; the shadow map is drawn
with that same projection, so it covers the beam and nothing else, and the shadows fall the way the beam does (nearly along
the vehicle). In the map is what lies within `gfx.lights.shadows.headlights.range` ahead of the vehicle (the terrain,
tracks, models, the other vehicles and the occupied cab, only their back faces, as in the shadows of the sun), not the
vehicle itself; past that reach things cast no shadow of the headlights, but what is behind a nearer one is still in its
shadow. Maps go to the most important headlights (the nearest and brightest, as the light buffer orders them),
`gfx.lights.shadows.headlights` sets of them, of those whose beam is in view within that reach. By day, when the beam isn't
seen, nothing is drawn. A map keeps the place the projector was in when it was drawn, so an older one still holds right for
what doesn't move:

| Level | Map | Drawn, vehicle moving / standing | Edges |
|---|---|---|---|
| `low` (1) | 1024 x 1024, 4 MB | every 2nd / 8th frame | hard (2x2 filter) |
| `medium` (2) | 1024 x 1024, 4 MB | every frame / every 4th | soft, 4 samples |
| `high` (3) | 2048 x 2048, 16 MB | every frame / every 2nd | softer, 8 samples |

Drawing a map is a shadow pass of the scene in the beam (draw calls in proportion to what is in it, on the CPU like the
shadows of the sun, see `headlight shadows:` in Shift+F12); with the vehicle on the move it is drawn every frame from `medium` on.

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
| `gfx.lights.shadows` | `medium` | `no`/`low`/`medium`/`high` (or 0..3): shadows of the cab lamp and the scenery lamps, see above |
| `gfx.lights.shadows.cab` | `8` | 0..16: how many lights of the cab (the brightest; the lamp or the light points of the cab model) have a cube shadow map of their own; the others take the map of the closest light with one (within 1 m); 0: none |
| `gfx.lights.shadows.lamps` | `8` | 0..32: how many scenery lamps nearest to the camera cast shadows; 0: none |
| `gfx.lights.shadows.lamps.size` | `512` | 128..2048: size of a face of the shadow map of a scenery lamp |
| `gfx.lights.shadows.lamps.range` | `100` | distance from the camera within which scenery lamps cast shadows, metres |
| `gfx.lights.shadows.vehicles.range` | `50` | distance from the camera within which vehicles (and the occupied cab) cast the shadows of scenery lamps, metres; 0: only the scenery does |
| `gfx.lights.shadows.headlights` | `2` | 0..4: how many sets of headlights (vehicle ends) cast shadows; 0: none |
| `gfx.lights.shadows.headlights.range` | `250` | 10..1000: distance ahead of the vehicle within which things cast the shadows of its headlights, metres |

**The probe of the cab** (`gfx.cabprobe`, OpenGL and Vulkan): two cube maps drawn from the middle of the box of the cab model,
in the space of the cab (they stay right while the vehicle moves): the light of the cab lamp the cab reflects (with the
shadows of the lamp) and the light of what glows in it (the lights of the desk), and the albedo of the cab. The glasses
aren't drawn, so through them the probe shows nothing. They are drawn every `gfx.cabprobe.interval` frames (for the moving
parts and the lights of the desk) and at once when the cab or the lamp change, the six faces in one pass with the opaque
parts of the cab (as the shadow map of the lamp). The parts of the cab read them:
- the light of the lamp the cab reflects lights the cab again (`gfx.cabprobe.bounce`, from spherical harmonics of the
  probe) in place of `gfx.lights.cab.bounce`, so the light bounces where the lamp actually lights the cab;
- the cab reflects itself (`gfx.cabprobe.reflections`) where the screen space reflections find nothing: the glasses show the
  lit cab at night, the glossy parts reflect the cab instead of the sky. The reflected ray is taken to the box of the cab
  (the walls), the cab shows the light of the lamp it reflects and its albedo lit by the ambient light, through the windows
  the sky is seen as before.

The sun (the moon at night) lights the probe too, where it comes in through the windows (`gfx.cabprobe.sun`): the light the cab
lights itself with again is then also the sunlight bouncing off the desk, the floor and the walls where it falls, and the cab
reflects the sunlit spots. Where the windows are the probe drawn last tells: a small level of its albedo (32 pixels a face at most,
the mask) is copied after each drawing; from a point of the cab the ray towards the sun is taken to the box of the cab and the
mask, seen from the middle of the box, says whether the cab is there or a window (what is in the way inside the cab, the desk or
a seat, holds the sun back only where it hides that point from the middle: a coarse shadow, enough for the light that bounces).
The sun is held back by the scenery around the cab (`gfx.skyvisibility`, or the environment flags with `gfx.tunnelflags yes`)
and by the shadows of the clouds, as outside. A cab drawn for the first time (another vehicle, another cab) has no mask yet,
so its probe is drawn without the sun and again with it in the next frame. The cab gets the sun itself from the shading as
before (the shadow map); the ambient light is left out of the probe, the cab gets it as before. `gfx.cabprobe no` is the old way
(the share of the lamp in the ambient light, the sky in the reflections, `gfx.interior.bounce` for the sun).

The parts of the cab of the `sunless` materials get no light of the sun; where it falls on them the ambient light is raised in the
proportion their shadow tone gives (at most four times), and their shadow (of the walls of the cab, of the clouds) leaves the
ambient light as it is. Before, the shadow darkened it to the shadow tone, so the whole cab went dark as in a tunnel.

**The low poly interiors** of the vehicles (and the cab when it has no probe) get the sun which comes in through their windows
and bounces around them as a share of the light of the sun outside (`gfx.interior.bounce`), held back by the scenery and the
clouds like the sun; the interior isn't known well enough for more. Both kinds of interiors take only a share of the ambient
occlusion of the view (`gfx.postfx.ao.cab`). Cost: one draw of the opaque cab (about the draw calls of
the cab in the colour pass) every `interval` frames, 2 cube maps of the size of the level with their mipmaps and the depth (64: 0.5 MB, 128: 2 MB, 256: 8 MB), two texture reads in
the reflections of the cab.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.cabprobe` | `medium` | `no`/`low`/`medium`/`high` (or 0..3): the probe of the cab, faces of 64/128/256 pixels; `no`: none |
| `gfx.cabprobe.interval` | `4` | 1..120: frames between the updates of the probe |
| `gfx.cabprobe.bounce` | `1.0` | 0..4: share of the light of the lamp the cab reflects which lights it again |
| `gfx.cabprobe.reflections yes/no` | `yes` | the cab reflects itself where the screen space reflections find nothing; `no`: the sky, as before |
| `gfx.cabprobe.sun` | `1.0` | 0..4: the sun lights the probe where it comes in through the windows, so its light bounces around the cab; 0: no sun in the probe, as before |
| `gfx.interior.bounce` | `0.1` | 0..1: share of the light of the sun outside the low poly interiors (and the cab without its probe) get as the light which comes in through their windows and bounces around them; 0: none, as before |

### Tunnels, bridges and roofs: the occlusion of the sky

The light of the sky and of the sun is held back by the geometry around, not by the environment flags of the tracks
(`tunnel`, `canyon`): the tracks and the vehicles on them are drawn alike wherever they are, and a tunnel is dark because the
hill and the vault over it hide the sky. A track with the `tunnel` flag and nothing built over it is lit as in the open; the
flag stays in the scenery files (the sound in the tunnel, the driver turning the lights on). `gfx.tunnelflags yes` brings the
old darkening by the flags back.

The scenery around the camera is laid out seen from above, in tiles, in the background (a thread of its own): for each texel
the height of the highest surface (the top) and of the lowest one facing down (the ceiling: the vault of a tunnel, the deck of a
bridge seen from below, a roof). Taken are the shapes (the terrain, the heightmap terrain, roads, platforms) and the opaque parts
of the models at their nearest level of detail; not the vehicles, the tracks, the catenary, nor what is see-through (trees).
Things narrower than about three texels (poles, masts, fences) are left out. From the heights around, each texel gets:
- the share of the sky seen above the top, and under the ceiling: near it and a metre above the ground around (the ground under
  a high viaduct sees much of the sky on its sides, the vault of a tunnel lets light in only along the tunnel from the portal); a
  point under the ceiling gets a blend of the two by its height;
- the heights between which the sun (the moon at night) comes in: above the top what is higher than the shadow of the hills
  and buildings towards it, under the ceiling what the rays reach under the vault from the portal (a low sun shines a little way
  into a tunnel).

With `gfx.skyvisibility.tunnels yes` (the default) all this is done only around the tracks with the `tunnel` flag (see the table):
the flag tells where a tunnel is, the geometry how dark it is and where it begins.

A point is taken as under the scenery only where a ceiling is over it (a tunnel, a bridge, a roof). Under the top with no
ceiling over it, which a surface that is seen can't really be (the edges of platforms and walls the texels are blended over,
holes cut in the terrain, parts of models sunk in the ground), it's lit as at the top, in the open: no darkness of a tunnel
anywhere but under something.

The shading takes the ambient light, the reflection of the sky and the sun by them, wherever a surface is (the half of a train
in the tunnel is dark, the other half in the light, with the passage over a few metres at the portal). The headlights, the lamps
and the light of the cab are direct light and are not held back; the headlights fade in daylight by the light around the
vehicle, which is read from the same maps, so in a tunnel they light by day as by night. The rain and the snow don't fall under
the roofs, bridges and in tunnels, and the light of the volumetric fog and of the smoke is held back as well. A tile is laid out
once, when it comes into the square around the camera, and again only when its scenery changes (the editor, the streamed
terrain, models shown or hidden by events); the sun is followed every quarter of a degree.

| Level | Square around the camera | Texel | Searched around a texel (sky / sun) |
|---|---|---|---|
| `low` (1) | 256 m (at least 96 m on each side) | 1 m | 32 m in 8 directions / 64 m |
| `medium` (2) | 512 m (at least 192 m) | 1 m | 48 m in 12 directions / 128 m |
| `high` (3) | 512 m (at least 192 m) | 0.5 m | 64 m in 16 directions / 128 m |

Cost: on the main thread, the triangles of a tile gathered when it comes in (one tile a frame, more at the start), a fraction
of a millisecond; in the background, a few to some tens of milliseconds per tile, which doesn't hold the frame up. On the GPU,
two textures (`medium`: 6 MB, `high`: 24 MB) and eight texture gathers per pixel in the shading. With `no` nothing of it is made
and the scenery is lit as in the open everywhere.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.skyvisibility` | `medium` | `no`/`low`/`medium`/`high` (or 0..3), see above |
| `gfx.skyvisibility.sun yes/no` | `yes` | the sun is held back too (beyond the reach of the shadow map, and in the tunnels whatever the shadow map holds); `no`: the sun only by the shadow map |
| `gfx.skyvisibility.fog yes/no` | `yes` | the light of the volumetric fog is held back too |
| `gfx.skyvisibility.tunnels yes/no` | `yes` | the occlusion only around the tracks with the `tunnel` flag: whole within 20 m of them, fading out by 35 m; there the geometry makes the tunnel dark (the portals, the passage at them, the half of a train in the tunnel), elsewhere the scenery is lit as in the open (no darkening between platforms, in cuttings, under shelters). `no`: everywhere, by the geometry alone |
| `gfx.tunnelflags yes/no` | `no` | `yes`: the flags `tunnel`/`canyon` of the tracks darken the tracks and the vehicles on them, as before (on top of the occlusion) |

All of them can be changed while the simulator runs, in the **Sky and exposure** debug window, which also shows views of the
maps around the camera (the share of the sky above the top and under the ceilings, the sun, the heights).

### Contact shadows

Short shadows where things touch or nearly touch: the rails on the sleepers, bolts and edges of panels, a wheel on the rail,
a post in the ground, the knobs on the desk of the cab. The shadow maps are too coarse for them, and a surface right next to
another one is kept from shadowing itself there. A contact shadow is found in the image: from each pixel a short ray goes
towards the light, `gfx.contactshadows.length` long, and where the depth of the scene drawn so far holds a surface in front
of it, the light is held back. So only what is in the picture casts them (nothing out of view or behind something else),
they fade over the end of the ray, and they end at `gfx.contactshadows.range` from the camera. They need the depth of the
whole scene before its colour, so turning them on turns on the depth pass of `gfx.depthprepass.scene` (whatever that is set
to; see its cost below); they are there only in the main view, with the post processing on and not in the orthographic view
of the editor. The things left out of the depth pass (parallax materials, water, glass, faded parts) cast none.

| Level | Rays |
|---|---|
| `low` (1) | towards the sun (or the moon), 8 steps |
| `medium` (2) | towards the sun, 12 steps, and towards 2 lamps or headlights which light the pixel, 8 steps |
| `high` (3) | towards the sun, 16 steps, and towards 4 lamps or headlights, 12 steps |

The rays towards a lamp go at most half the way to it. The lamps of a pixel are the first which light it noticeably, in the
order of the light buffer: the cab lamp, the headlights, then the lamps of models and vehicles, the most important first.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.contactshadows` | `no` | `no`/`low`/`medium`/`high` (or 0..3), see above |
| `gfx.contactshadows.length` | `0.5` | 0.05..5: length of the rays, metres |
| `gfx.contactshadows.range` | `50` | 1..1000: distance from the camera within which there are contact shadows, metres |

All three can be changed while the simulator runs, in the **Light clusters** debug window.

### Reflections

The reflections of the scenery are found in the image (screen space reflections): the reflected ray of each pixel of a
shiny surface is followed through the depth of the scene drawn before its colour, and where it meets a surface, the colour
is taken from the image of the last frame (moved to where that surface is now; the cab is moved with the vehicle). What the
image doesn't hold (what is out of view, behind something else or behind the camera, the backs of things) reflects the
environment map, which now holds the sky and the clouds only, with the ground below the horizon in the colour of
`gfx.ambient.groundalbedo`. The scenery is no longer drawn into the environment map, so its cost is gone. The image of the
last frame is copied after the anti-aliasing and the ambient occlusion and before the motion blur; the first frame, a jump
of the camera of more than 50 metres and a change of the size of the window go without it (the sky only). Like the contact
shadows they need the depth of the whole scene before its colour, so they turn on the depth pass of `gfx.depthprepass.scene`
(see its cost below), and they are there only in the main view, with the post processing on and not in the orthographic
view of the editor. The rough surfaces read a blurrier copy of the image; those rougher than `gfx.ssr.roughness` reflect
the environment map only.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.ssr` | `medium` | `no`/`low`/`medium`/`high` (or 0..3): steps of the rays, 16, 32 or 64. `no`: the environment map only (the sky) |
| `gfx.ssr.range` | `1000` | 10..2000: the longest ray, metres; what is farther from the reflecting surface reflects the sky |
| `gfx.ssr.roughness` | `0.6` | 0..1: the roughest surface (perceptual roughness) that reflects the scene |
| `gfx.raytracing yes/no` | `no` | Vulkan renderer only, on a graphics card with ray queries (`VK_KHR_ray_query`): what the screen space reflections don't find is traced through the scenery around the camera instead of reflecting the sky. Read at the start; with `gfx.ssr no` all the reflections of the scenery are traced |
| `gfx.raytracing.range` | `500` | 10..5000: the scenery within this distance from the camera takes part in the traced reflections, and the rays are this long, metres |

The traced reflections hold what the environment map held before, as `gfx.reflections.fidelity` says: the terrain
and the large shapes of the scenery (0), the tracks, the roads and the models within `gfx.reflections.range_instances` (1)
and the vehicles within `gfx.reflections.range_vehicles` (2, the default now), except the vehicle of the cab. `gfx.reflections.fidelity` and the
two ranges have no other use now; `gfx.reflections.framerate` still says how often the environment map (the sky) is
drawn. The point hit has the colour of the diffuse texture of its material (the heightmap terrain: its layers blended by their
weights, without the normal and specular maps; the holes cut in it are not), and is lit by the sky, by the sun (its shadow map
where the map holds the point, a ray of its own elsewhere) and by its own light; the lamps and headlights don't light it. Each part of a model gets its structure
for the tracing the first time it's met (a few a frame, so a new place fills in over some frames), and loses it when it
isn't met for half a minute.

All the entries can be changed while the simulator runs, in the **Sky and exposure** debug window (the ray tracing can be
turned off and on there once it was started).

### Weather

The clouds, the fog and the stars are drawn from the weather of the simulation (a `weather` section of the scenery, or its
`atmo` section as before, see `weather.md`). The cloud model of the `sky` section and the starry dome (`skydome_stars.t3d`)
are no longer loaded.

- **Clouds** are ray marched through layers of the genera of aviation reports (cumulus, cumulonimbus, stratus, cirrus...),
  each with its own shape, height and depth, moving with the wind. The main view gets them at a fraction of its resolution
  and blended over frames; reflections and the other views read a panorama of the sky with them, renewed in bands over a few
  frames. Low clouds hide the hills and the far scenery they are in. They are lit by the sun through themselves (dark bellies,
  edges glowing against the sun, thin wisps darker with the sun behind), by the sky above them in its colour (with the
  directional ambient light, `gfx.ambient.sh`) and from below by the ground.
- **Cloud shadows**: the clouds cast their shadows on the ground, the models, the vehicles and into the fog, from a map
  around the camera along the sun (or the moon). With them on, the overcast no longer dims the sun and softens its shadows
  as a whole: the sun is dimmed where the clouds are, and shines in the gaps.
- **Fog**: the haze, thinning out with the height, and the ground fog of the weather. `volumetric`: a volume over the view
  (froxels) holds the light the fog scatters, the sun with the shadows of the terrain, models and clouds in it (shafts of
  light), and the lamps and headlights; past its depth the fog is worked out along the ray. `analytic`: the same fog along
  the ray only, without the shafts and the lamps, as the old fog was drawn; it takes the same weather and looks the same
  from afar, so the two can be switched without changing anything else. The colour of the fog comes from the sky around the
  horizon, towards and away from the sun.
- **Stars** come from a catalogue built into the simulator (about 9100 stars down to magnitude 6.5), with their colours,
  turned with the sidereal time and the latitude of the scenery, dimmed near the horizon, by the fog and the clouds.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.fog` | `volumetric` | `volumetric` (`yes`) or `analytic` (`old`, `no`), see above |
| `gfx.fog.quality` | `medium` | `low`/`medium`/`high`/`ultra` (or 1..4): the size of the fog volume, 80x45x48 to 240x135x128 cells |
| `gfx.fog.range` | `1500` | 100..10000: the depth of the fog volume, metres; farther, the fog is worked out along the ray |
| `gfx.fog.lights` | `1.0` | 0..10: the brightness of the lamps and headlights in the fog; 0: none |
| `gfx.fog.shadows yes/no` | `yes` | the sun and the headlights cast their shadows into the fog (shafts of light) |
| `gfx.fog.temporal yes/no` | `yes` | the fog volume is blended over frames (smoother, a little lag in quick turns) |
| `gfx.clouds` | `medium` | `no`, or the quality `low`/`medium`/`high`/`ultra` (or 1..4): the resolution and the number of steps of the clouds. `no`: no clouds are drawn, the overcast dims the sun as before |
| `gfx.clouds.distance` | `40000` | 5000..200000: the farthest the clouds are drawn, metres |
| `gfx.clouds.shadows yes/no` | `yes` | the clouds cast shadows, see above |
| `gfx.clouds.shadows.size` | `512` | 64..2048: the size of the cloud shadow map, texels |
| `gfx.clouds.shadows.range` | `16000` | 1000..100000: the side of the cloud shadow map, metres; farther the clouds cast no shadows |
| `gfx.stars yes/no` | `yes` | the stars of the catalogue |
| `gfx.stars.brightness` | `1.0` | 0..16: their brightness |
| `gfx.sun.disc` | `1.0` | 0..4: brightness of the sun disc against the radiance its sunlight gives it; 1 is physical (held within the range of the image), the glare around it comes from the bloom |
| `gfx.moon.disc` | `1.0` | 0..16: brightness of the moon disc |
| `gfx.moonlight` | `1.0` | 0..10: how much the moon lights the scenery at night (its direct light, the night light that comes with it and the moonlit sky); 0: no moonlight |

### Particles

- **Smoke** of vehicles and scenery models is simulated and drawn on the graphics card. The sources (the `smokesource_`
  points of the models and their `data/*.txt` templates, unchanged) work out on the processor only how many particles they
  give off and what these start with, as before; the particles themselves live in a buffer of the graphics card and are
  moved there once per step of the simulation, by the same rules as on the processor. The particles of a step are spread
  along the way the source went in it, so the smoke of a fast train no longer comes out in clumps. Each view leaves out
  what it doesn't see and lights the rest with the sun (with the shadows of the terrain, models and clouds), the sky and
  the lamps and headlights, as the surfaces are lit: smoke in the shade is darker, smoke against the sun glows, smoke in
  the beam of headlights at night is lit by it.
- **Templates of smoke sources** (`data/smokesource_*.txt`, both renderers) may say how heavy the particles are: with
  `terminal_velocity: v` (m/s, positive up: light, hot smoke rises; negative down: dust, sand, sparks fall) the velocity
  of a particle goes over to `v` (straight up or down, besides the drift with the wind), the faster the larger `drag: d`
  (per second, default 1: about 63% of the way in a second). It takes the place of the old rules of the rise (slower in
  cold and humid air); a template without it behaves as before. The particles don't stop at the ground.
  Two more keys make the smoke come out of the exhaust as a jet instead of puffs swelling above it: `spread: s` — the
  particles grow by `s` metres of size for each metre they go through the air (a jet widens by about 0.3–0.5 per metre),
  on top of `size_change`, so they leave the exhaust as small as `min_size`/`max_size` (the size of the pipe) and widen
  as they rise; `fade_in: t` — they take `t` seconds to show up after they're born, instead of popping up at full opacity.
- **Rain and snow** are particles in three boxes around the camera (10, 24 and 60 metres), more of them the heavier the
  rain or snow (the overcast above 1). The drops and flakes fall, drift with the wind, the flakes flutter, and each is
  drawn as a streak of its motion against the camera: they slant when the train moves. They are lit like the smoke, so the
  rain shows in the headlights and under the lamps, and fogged. The cab still hides them as before. They don't stop under
  roofs or in tunnels yet (neither did the old cylinder).
- **Soft particles**: smoke, rain and snow fade where they near the scenery behind them instead of being cut by it
  (the depth of the view drawn so far is copied for it once per view).

| Entry | Default | Meaning |
|---|---|---|
| `gfx.smoke.gpu yes/no` | `yes` | the smoke on the graphics card, see above; `no`: simulated on the processor and drawn as before |
| `gfx.smoke.lighting yes/no` | `yes` | the smoke of the graphics card lit by the sun, the sky and the lamps; `no`: in the colour the processor gave it, as before |
| `gfx.particles.soft` | `1.0` | 0..4: how far in front of the scenery the particles fade out, as a share of the half size of a particle of smoke, and times 0.25 m for rain and snow; 0: cut, as before |
| `gfx.precipitation.gpu yes/no` | `yes` | rain and snow as particles, see above; `no`: the textured cylinder, as before |
| `gfx.precipitation.particles` | `65536` | 0..1048576: the particles of the heaviest rain or snow; lighter rain has fewer, down to 15% of it |

### Models drawn from the GPU

Many models of the scenery (trees, bushes, poles, fences: models without lights and animations) are drawn in batches, one
batch per model and level of detail. Which of them are drawn, and at which level of detail, can be worked out on the
processor (as before) or on the graphics card. On the graphics card the processor no longer goes through the models one by
one, so with dense vegetation it is much less loaded. The main view, the three cascades of the sun shadows, the shadow maps of headlights and the scenery of the shadow maps of scenery lamps are drawn
this way, and so are the sleepers of the tracks with a `sleepermodel` in the main view; the shadow map of the cab lamp, the
cab and other views go as before.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.gpucull` | `no` | `no`: on the processor, as before. `yes`: on the graphics card, and the models are drawn from its result. `compare`: on the processor, which draws, and on the graphics card too, for the results to be compared (debugging) |
| `gfx.gpucull.occlusion yes/no` | `yes` | with `gfx.gpucull yes`: models hidden whole behind what was drawn before them (the cab, the terrain, buildings and other models drawn on their own, vehicles) are left out |
| `gfx.gpucull.occlusion.instances yes/no` | `yes` | with `gfx.gpucull.occlusion yes`: the models drawn this way hide one another too (see below) |
| `gfx.depthprepass yes/no` | `yes` | with `gfx.gpucull yes`: the depth of the models is drawn first, so the colour is worked out only for what stays in view (dense foliage) |
| `gfx.gpucull.multidraw yes/no` | `yes` | with `gfx.gpucull yes`: parts of models with the same material (and texture variant) in the same model are drawn with one call (multi draw), see below |
| `gfx.gpucull.shadows yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the models of the cascades of the sun shadows too (each cascade with its own camera and the distances from the real camera, as before; models none of whose materials cast shadows are left out at once) |
| `gfx.gpucull.reflections yes/no` | `yes` | no longer used: the environment map holds the sky only (see Reflections) |
| `gfx.gpucull.shadows.headlights yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the models of the shadow maps of headlights too (each map tested against the beam it is drawn with, to its reach, the distances from the real camera, as before) |
| `gfx.gpucull.shadows.lamps yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the models of the scenery part of the shadow maps of scenery lamps too (each map tested against the reach of its lamp, as before); the part with the vehicles has no models of the scenery |
| `gfx.gpucull.sleepers yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the sleepers of the tracks (`sleepermodel`) of the main view too, each one tested against the view and given its level of detail by its distance, as before (the tracks within `sleeperdistance` are chosen as before); they also take the occlusion test, the depth prepass and the two phases of the models |
| `gfx.gpucull.models yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the models drawn on their own too (scenery models with lights, animations, or skins and animations given by events, which the instanced draws leave out), in every pass the instances are culled in: each such model is tested on its own like the others (visibility, the view, its range, the draw range, its size on the screen) and given the level of detail its distance picks from the `mindistance`/`maxdistance` of its submodels; the processor animates and prepares the model only to record what it draws. Models with billboards turned to the camera stay on the processor, and so do vehicles |
| `gfx.gpucull.bucketrange yes/no` | `yes` | the models of one kind in a scene cell lying wholly beyond their range (the largest visibility range of their nodes, and of the submodels of the model) are left out of each pass before the culling, the one on the graphics card included, which would leave out each of them anyway; the picture stays the same. Matters with many small models of a short range (grass): without it they are tested and sent to the graphics card in every pass up to the draw range |
| `gfx.depthprepass.scene yes/no` | `no` | the depth of the whole opaque scene of the main view (terrain, tracks, buildings, models, vehicles, the cab) is drawn before its colour, see below |
| `gfx.depthprepass.replay yes/no` | `yes` | the scene is gone through once for its depth and its colour: what the colour pass draws is recorded and both are drawn from the record, see below. `no`: twice, as before |

With `gfx.gpucull.occlusion.instances yes` the test goes in two phases: the models seen in the previous frame are drawn
first (their depth, or their colour without the depth prepass), the depth they leave is added to what the others are tested
against, and only then are the others tested and drawn; the graphics card also notes which models it saw, for the next frame.
So a tree hidden behind other trees, or behind a hill of models, is left out as well. Whatever is left out is hidden behind
something drawn, so the picture is the same either way; a model coming into view is drawn in the frame it appears in. It
costs the depth pyramid a second time and the processor one more pass over the draws of the models (hundreds of draw calls,
not the scene). With `no` the models do not cover each other: a tree hidden behind another tree is still drawn (its colour
is not worked out with the depth prepass on). Models whose material is not one of the usual ones (parallax, water and the
like) are left out of the depth prepass and drawn as before; with the depth prepass on, they don't hide the others either.
All of these can be changed while the simulator runs, in the **Instance culling** debug window.

The shadows take the same tests as before (which models are in the cascade or the map, their distance
and level of detail), without the occlusion test, the depth prepass and the two phases, which belong to the main view. The
copies of the models are sent to the graphics card once and serve all of these views. In the shadows the parts with the
same texture for the alpha test (or with none) in the same model go in one call with `gfx.gpucull.multidraw yes`, whatever
their material.

With `gfx.gpucull.multidraw yes` the draws of the models drawn from the GPU are put in an order in which the parts with the
same material, texture variant and model (the model keeps its geometry in a buffer of its own) follow one another, and each
such run goes in one call (`glMultiDrawElementsIndirect`); the data of every part (its position in the model, colours,
parameters) is read by the shaders from a buffer, by the number of the part in the call. The parts of one model at one level
of detail keep their order (parts of a model lying on one another are drawn as before), only the parts of different copies
and levels of detail are brought together: a trunk seen in two levels of detail, the six sides of a block in the same
material. The depth prepass joins the parts of a model with the same alpha-tested texture, whatever their material. The
picture is the same either way; `no` draws them one by one, as before. Materials whose shader can't be built in this form
(a message in the log) are drawn one by one.

With `gfx.depthprepass.scene yes` the scene is gone through twice: first only its depth is drawn, then its colour, worked
out once per pixel for the surface which stays in view. That costs the processor a second walk through the scene (fewer
draws than the colour pass: what the depth programs can't draw, like parallax materials and faded parts, is left to the
colour pass) and saves the graphics card the shading of hidden surfaces. It also gives the depth of the scene before its
colour, which the effects that need it build on. The models drawn from the GPU (`gfx.gpucull yes`) are culled in the first
walk and drawn from the same result in the second. The picture is the same either way. In rain and snow (when the cab is
drawn last, after the precipitation) the cab isn't in the depth pass. Can be changed while the simulator runs, in the
**Instance culling** debug window.

With `gfx.depthprepass.replay yes` (the default) the scene isn't gone through twice: the walk of the colour pass is made
once, in the depth pass, and every opaque draw it would make is recorded with what it needs (the data of the model part, the
material and its texture variant, the light of tunnels and of the cab). The depth is drawn from the record in runs of the
same kind (with or without the alpha test, the texture of the test, the model or section the geometry is kept in), each run
with one multi draw, and the colour pass then draws the record in the order of the walk, with the programs of the materials.
The depth is drawn before the depth pyramid of the occlusion test of the models (`gfx.gpucull.occlusion`) and before the
cursor of the editor reads the ground, so they see the same as before. When the depth pass is there only for the contact
shadows (`gfx.depthprepass.scene no`), the parts lying whole farther than the reach of the contact shadows
(`gfx.contactshadows.range` and a little more) are left out of it and only drawn in colour, as they would be without the depth
pass; the copies of models drawn from the GPU always have their depth drawn. With fewer parts in the depth the occlusion test
of the models has less to hide them behind past that reach. In the editor the whole depth is drawn. The picture is the same
either way; `no` walks the scene twice, as before, for comparison.

### Model data of the draws

Before every draw the renderer hands the graphics card the data of the drawn part of the model (its position in the view, its
colours and parameters), and before batches of models the positions of the copies in the batch. With `gfx.bufferstreaming`
on, these go to buffers the processor writes to directly, a new place for every draw, one buffer for each of the last three
frames; before, every draw replaced the data of the one block with a separate copy through the driver. The picture is the
same either way.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.bufferstreaming yes/no` | `yes` | `yes`: the data of the draws is written to persistently mapped buffers. `no`: as before (`glBufferSubData`), for comparison or when a driver misbehaves. Read at the start |

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
- **Reflections** of the scenery come from the image (screen space reflections, see above) instead of the
  environment map, which holds the sky only.
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
  timings the GPU time of drawing it (`lamp shadows:`); the level can be changed in the window. The `scenery lamp shadows:`
  row gives the lamps with maps (of the slots, how many ready, how many with vehicles), how many scenery parts and vehicle
  copies were drawn so far, and the draw calls and cpu time of the last frame which drew any; the timings show their GPU time
  (`scenery lamp shadows:`). The count and both ranges can be changed in the window too. The `headlight shadows:` row gives
  the headlights with maps (of the slots, how many ready), the size of the maps, how many were drawn so far, and the draw
  calls and cpu time of the last frame which drew any, with their GPU time in the timings (`headlight shadows:`); the count
  and the reach can be changed in the window.
- **Sky and exposure** window (next to the Light clusters window): the switches and scales above, the share of the
  atmosphere, the colours of the sun, ambient light and fog in use, and the measured luminance and exposure.
  Shift+F12 shows the GPU time of `atmosphere:` (look-up tables, only when the sun moved or the weather changed),
  `envmap ggx:` (prefiltering, with each update of the environment map), `exposure:` and `bloom:`. The same window holds
  the ambient occlusion mode and its settings; Shift+F12 shows its time as `ambient occlusion:` (gtao only).
- **Weather** window (next to the Sky and exposure window): the switches above, the fog range in use, the share of the sun
  in the light of the fog and how much light the clouds let through on average; below them the weather: a report (METAR)
  of it, which can be edited and applied with a time of change, and the layers of clouds, the fog, the precipitation and
  the wind, which change at once. These changes are of this computer only. Shift+F12 shows the GPU time of `clouds:` (the
  main view), `clouds panorama:` (a band of the panorama), `cloud shadows:` and `fog volume:`.
- **Instance culling** window: the mode of `gfx.gpucull` and the switches above. With `gfx.gpucull yes` Shift+F12 shows
  `gpu cull:` (models drawn, in two phases how many of them first, and hidden by the occlusion test, as counted on the
  graphics card a few frames before, the levels of detail, the draws with those of the depth prepass and the calls which
  made them — fewer than the draws with `gfx.gpucull.multidraw yes` — and the processor time of the whole step) and the timings `instance cull:`, `occlusion depth:` (the depth pyramid of the test) and
  `depth prepass:`; in two phases each of them adds in brackets the time of the second phase, of the second pyramid and of
  the depth of the models drawn first. With `compare` it shows how many models both counted and how many levels of detail
  differ. With `gfx.gpucull.shadows` the same rows (without the occlusion test and the
  prepass) follow for each cascade, `gpu cull shadows 1:` to `3:` (1 the nearest), with the timings
  `instance cull shadows 1:` …. With `gfx.gpucull.shadows.headlights` and
  `gfx.gpucull.shadows.lamps` they follow for each map of headlights drawn so far, `gpu cull headlight shadows 1:` … (by the
  slot of the map), and for the scenery of the maps of lamps, `gpu cull lamp shadows, scenery 1:` and `2:` (the first and
  the second drawn in a frame), each with the numbers of its last drawing and the timings `instance cull headlight shadows 1:`
  … and `instance cull lamp shadows, scenery 1:` …. With `gfx.gpucull.sleepers` the row
  `gpu cull:` is followed by how many sleepers of how many tracks were queued (they are counted among the models). With
  `gfx.gpucull.models` each `gpu cull` row is followed by how many models drawn on their own were queued (also counted
  among the models).
  Each `gpu cull` row is also followed by how many models the pass tested in how many buckets (models of one kind in one
  cell), with `gfx.gpucull.bucketrange` how many buckets and models were left out beyond their range, how many records of
  models were sent to the graphics card, and whether the buffer of the records was started over (all buckets of the pass
  sent anew).
- With `gfx.depthprepass.scene yes` Shift+F12 shows the `scene depth:` row (draws of the depth pass and its processor time)
  and the timing `scene depth:` (its GPU time). With `gfx.depthprepass.replay yes` the row tells how many of the recorded
  parts (of models, of the scenery; neighbouring tracks count one by one) had their depth drawn, in how many calls, how many
  were left beyond the reach of the contact shadows, and the processor time of the depth pass, which holds the walk of the
  colour pass now (the `color:` time is the one to compare).
- With `gfx.contactshadows` on Shift+F12 shows the `contact shadows:` row (the level, the length and reach of the rays, or
  `off in this view`), the `scene depth:` row and timing of the depth pass they turn on, and the timing `contact shadows depth:`
  (copying the depth for them). The rays themselves are part of the shading of the colour pass: their cost shows as the
  difference in the GPU frame time with them on and off.
- Shift+F12 shows the `instance buffer:` row (positions of copies written in the frame and the size of the buffer) and the
  `model blocks:` row: how many blocks of model data the frame wrote, with `gfx.bufferstreaming yes` also how much of the
  buffer of the frame they took, how many times a frame had to wait for the graphics card to finish with a buffer before
  writing to it (`waits`, should stay at 0) and how many times a buffer had to grow (`grown`, a few times after the start).
  With `gfx.gpucull yes` and `gfx.gpucull.multidraw yes` the `multi draw blocks:` row shows the same for the data of the
  parts of models drawn with multi draws (one block per draw, written once a frame).
- Shift+F12 shows below `gpu frame total:` the `gpu frame parts:` of the main view: marks of time put one after another
  through the frame, so each part lasts until the next one begins and the parts add up to the frame with nothing left out
  (`frame setup`, `sun shadows 1` to `3` — 1 the nearest, with the culling of the cascade —, `reflections`, `lamp and
  headlight shadows`, `clear, clusters, lights`, `clouds, fog volume`, `sky`, `scene depth` and `contact shadows depth` when
  drawn, `colour, opaque` — with the culling, the occlusion depth and the depth prepass of the instances —,
  `colour, translucent and cab`, `resolve, ambient occlusion, motion blur` — with the copy of the image for the
  reflections —, `exposure, bloom`, `tonemapping, output`, `ui`,
  and `other viewports` when there are more of them; only those in the context of the main window are counted). The timings
  under `gpu steps:` are measured each from its own pair of marks, and the first mark of a pair can be put while the graphics
  card still works on what came before, so a short step right after a heavy one may show some of that work; the parts don't.
  Like the other timings, they are of a frame a few frames back.
- **Particles** window (next to the Weather window): the switches above. Shift+F12 shows the `smoke of the gpu:` row
  (particles drawn, as counted on the graphics card a few frames before, the places for the particles and the sources they
  are shared by) and, while it rains or snows, `rain of the gpu:` / `snow of the gpu:` (drawn of how many); the timings
  `particles:` give the GPU time of the step of the smoke, of the smoke of the view (culling, light and drawing) and of the
  rain or snow.
- `gfx.gldebug yes` turns on the OpenGL debug output (`KHR_debug`); messages go to the log.

## Building

`-DWITH_OPENGL_MODERN=ON` (default) builds this renderer; `WITH_OPENGL_LEGACY` and the GLES / ANGLE options
are gone.

All shaders in `shaders/` can be checked without a graphics card against GLSL 4.60, compute stage included:

```
ci_shadervalidator/build.sh
cd ci_shadervalidator && ./validateshaders
```

It needs `g++` with C++20, `glslangValidator` (Debian/Ubuntu: `glslang-tools`) and the `ref/glm` submodule.
The same check runs in CI for changes to shaders and the `gl/` wrappers.
