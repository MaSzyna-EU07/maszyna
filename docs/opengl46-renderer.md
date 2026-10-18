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
darkness; in daylight they are not gathered at all. The cab lamp and the lamps of scenery models nearest to the camera cast
shadows (see below); the others (lamps of vehicles, compartments, farther scenery lamps) also light what is behind walls
within their reach, which is kept short for compartments.

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
| `gfx.lights.shadows.lamps` | `8` | 0..32: how many scenery lamps nearest to the camera cast shadows; 0: none |
| `gfx.lights.shadows.lamps.size` | `512` | 128..2048: size of a face of the shadow map of a scenery lamp |
| `gfx.lights.shadows.lamps.range` | `100` | distance from the camera within which scenery lamps cast shadows, metres |
| `gfx.lights.shadows.vehicles.range` | `50` | distance from the camera within which vehicles (and the occupied cab) cast the shadows of scenery lamps, metres; 0: only the scenery does |
| `gfx.lights.shadows.headlights` | `2` | 0..4: how many sets of headlights (vehicle ends) cast shadows; 0: none |
| `gfx.lights.shadows.headlights.range` | `250` | 10..1000: distance ahead of the vehicle within which things cast the shadows of its headlights, metres |

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

### Models drawn from the GPU

Many models of the scenery (trees, bushes, poles, fences: models without lights and animations) are drawn in batches, one
batch per model and level of detail. Which of them are drawn, and at which level of detail, can be worked out on the
processor (as before) or on the graphics card. On the graphics card the processor no longer goes through the models one by
one, so with dense vegetation it is much less loaded. The main view, the three cascades of the sun shadows and the faces of
the environment map (reflections) are drawn this way, and so are the sleepers of the tracks with a `sleepermodel` in the main
view; the shadow maps of lamps and headlights, the cab and other views go as before.

| Entry | Default | Meaning |
|---|---|---|
| `gfx.gpucull` | `no` | `no`: on the processor, as before. `yes`: on the graphics card, and the models are drawn from its result. `compare`: on the processor, which draws, and on the graphics card too, for the results to be compared (debugging) |
| `gfx.gpucull.occlusion yes/no` | `yes` | with `gfx.gpucull yes`: models hidden whole behind what was drawn before them (the cab, the terrain, buildings and other models drawn on their own, vehicles) are left out |
| `gfx.gpucull.occlusion.instances yes/no` | `yes` | with `gfx.gpucull.occlusion yes`: the models drawn this way hide one another too (see below) |
| `gfx.depthprepass yes/no` | `yes` | with `gfx.gpucull yes`: the depth of the models is drawn first, so the colour is worked out only for what stays in view (dense foliage) |
| `gfx.gpucull.multidraw yes/no` | `yes` | with `gfx.gpucull yes`: parts of models with the same material (and texture variant) in the same model are drawn with one call (multi draw), see below |
| `gfx.gpucull.shadows yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the models of the cascades of the sun shadows too (each cascade with its own camera and the distances from the real camera, as before; models none of whose materials cast shadows are left out at once) |
| `gfx.gpucull.reflections yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the models of the faces of the environment map too (its own range of models, as before) |
| `gfx.gpucull.sleepers yes/no` | `yes` | with `gfx.gpucull yes` or `compare`: the sleepers of the tracks (`sleepermodel`) of the main view too, each one tested against the view and given its level of detail by its distance, as before (the tracks within `sleeperdistance` are chosen as before); they also take the occlusion test, the depth prepass and the two phases of the models |
| `gfx.depthprepass.scene yes/no` | `no` | the depth of the whole opaque scene of the main view (terrain, tracks, buildings, models, vehicles, the cab) is drawn before its colour, see below |

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

The shadows and the reflections take the same tests as before (which models are in the cascade or the face, their distance
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
  differ. With `gfx.gpucull.shadows` and `gfx.gpucull.reflections` the same rows (without the occlusion test and the
  prepass) follow for each cascade, `gpu cull shadows 1:` to `3:` (1 the nearest), and for the environment map,
  `gpu cull reflections, last face:` (the last face of the map drawn, as the map is drawn now and then), with the timings
  `instance cull shadows 1:` … and `instance cull reflections, last face:`. With `gfx.gpucull.sleepers` the row
  `gpu cull:` is followed by how many sleepers of how many tracks were queued (they are counted among the models).
- With `gfx.depthprepass.scene yes` Shift+F12 shows the `scene depth:` row (draws of the depth pass and its processor time)
  and the timing `scene depth:` (its GPU time).
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
  `colour, translucent and cab`, `resolve, ambient occlusion, motion blur`, `exposure, bloom`, `tonemapping, output`, `ui`,
  and `other viewports` when there are more of them; only those in the context of the main window are counted). The timings
  under `gpu steps:` are measured each from its own pair of marks, and the first mark of a pair can be put while the graphics
  card still works on what came before, so a short step right after a heavy one may show some of that work; the parts don't.
  Like the other timings, they are of a frame a few frames back.
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
