# Clustered lighting

How the OpenGL 4.6 renderer (`rendering/opengl46/`) gives the shaders the lights of vehicle headlights.
For the settings and what changed for players see [opengl46-renderer.md](opengl46-renderer.md).

## Idea

The view frustum of the main pass is cut into a grid of **clusters**: 16 x 9 screen tiles and 24 depth slices,
3456 in all. A compute shader works out which lights reach each cluster, once per frame. The fragment shader
finds the cluster of its fragment and loops over the lights of that cluster only, so the cost of shading
depends on the lights that are near a fragment and not on the number of lights in the scene. The sun is not
a part of this: it is a single directional light, handled in the shader as before, with the cascaded shadow map.

All of the sizes are constants in `gl/ubo.h`; they reach the shaders as `#define`s written by
`gl/glsl_common.cpp` from the same constants.

## Buffers

| Binding | Kind | Name (GLSL) | Content | Written by |
|---|---|---|---|---|
| SSBO 0 | std430 | `light_ssbo` | header (`ambient`, `fog_color`, `lights_total`, `lights_count`, `lights_clustered`) and `lights[]` (slot 0 is the sun) | CPU, `Update_Lights()` |
| SSBO 1 | std430 | `cluster_aabb_ssbo` | bounding box of every cluster in view space | `cluster_build.comp` |
| SSBO 2 | std430 | `cluster_range_ssbo` | `(offset, count)` of every cluster into the index list | `cluster_cull.comp` |
| SSBO 3 | std430 | `cluster_index_ssbo` | shared list of indices into `lights[]` | `cluster_cull.comp` |
| SSBO 4 | std430 | `cluster_counter_ssbo` | `index_count` (allocator of the list) and `overflow_count` | zeroed by the CPU, `cluster_cull.comp` |
| UBO 2 | std140 | `cluster_ubo` | grid parameters of the pass: projection, near / far, slice scale and bias, target size, `valid` | CPU, `cluster_grid::update()` |

The UBO numbers go together with the scene (0), model (1) and instance (3) blocks; the SSBOs have a binding
space of their own. Layouts of the structures are checked with `static_assert`s on sizes and offsets in
`gl/ubo.h`, and have to stay equal to the blocks in `gl/glsl_common.cpp`.

Sizes: `MAX_LIGHTS` = 256 (the sun and 255 headlights), `MAX_LIGHTS_PER_CLUSTER` = 64. The buffers have a
fixed size, so changing the resolution, the field of view or the draw range only makes the grid parameters
and boxes be recomputed.

`gl::light_data.lights_total` is the number of lights written to `lights[]`, which the culling reads.
`lights_count` is the length of the plain loop over the lights (the sun and the first `dynamiclights` headlights) used
when the lights are not clustered. `lights_clustered` is set per pass and tells the shading which one to use.

## Frame order

In `Render_pass()` for the color pass of a view (`opengl46_renderer.cpp`):

1. `Update_Clusters(vp)`: `cluster_grid::update()` compares the projection and near / far plane with those the
   boxes were built for. When they differ, `cluster_build.comp` recomputes the boxes (one invocation per
   cluster, 54 work groups of 64). This is the "clusters" timer.
2. `Update_Lights(...)`: sorts the lights of the scene (`simulation::Lights`) by distance squared over
   intensity, skips the lights that are off or farther than 1 km, writes each lit headlight to `lights[]`
   in view space, and uploads the header and the part of the array in use.
3. `Cull_Lights(vp)`: the CPU zeroes the counters, then `cluster_cull.comp` runs (one invocation per cluster, 54
   work groups of 64): it tests every light against the box of its cluster and writes the indices of the ones
   that reach it. Then `glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT)`. This is the
   "cull" timer.
4. The scene is drawn. Opaque, transparent and the cab all take the lights from the cluster lists.
5. With `gfx.lights.clusterheatmap`, `Render_cluster_heatmap()` draws the number of lights per cluster over
   the finished image.

Steps 1 and 3 are skipped when neither `gfx.lights.clustered` nor the heat map is on.

The grid and its buffers belong to a **viewport** (`viewport_config::clusters`): the main view, the extra
viewports and the eyes of a VR headset each have their own, because each has its own projection. The
indexed binding points are context state, so `cluster_grid::bind()` is called at the start of every pass that
uses the grid.

Passes without a grid: the orthographic view of the editor (`valid` = 0), the environment map (it gets its own
`Update_Lights(..., false)`: the sun and ambient light, `lights_total` = 1) and the model previews. They set
`lights_clustered` = 0 and use the plain loop.

## The grid

Depth is the clip space `w` of a point, which equals `1 / gl_FragCoord.w` of its fragment. Unlike the view
space `z` it is right for tilted screens (VR, extra viewports), where the screen is not perpendicular to the view axis. The projection is
reverse-Z with the far plane at infinity, so the `znear` / `zfar` of the pass bound the slices.

- Slice: `floor(log(w) * slice_scale + slice_bias)`, with `slice_scale = 24 / ln(zfar / znear)` and
  `slice_bias = -ln(znear) * slice_scale`; exponential spacing from `znear` to `zfar`.
- Tile: `gl_FragCoord.xy` scaled to the 16 x 9 grid; both the tile and the slice are clamped to the grid
  (a coordinate just under the edge rounds up to the next tile in float).
- Index: `x + 16 * (y + 9 * z)`, with `x` and `y` counted from the lower left corner, as `gl_FragCoord`.
- Box: `x`, `y` and `w` of the clip space are affine in the view-space position, so the eight corners of a
  cluster (`ndc.xy` times `w` at the near and far side of the slice) map back to view space through the
  inverse of the corresponding 3x3 part of the projection. The box is the min and max of the corners.

## Culling

`cluster_cull.comp` handles one cluster per invocation and goes through `lights[1 .. lights_total)`.

- A light's reach is the distance where its brightest possible contribution falls under `MIN_CONTRIBUTION`
  (1/1024) of the display range, at most `MAX_RANGE` (1000 m, as the 1 km filter on the CPU). Headlights are tested against
  that sphere and against the frustum of their projection (four side planes and the near plane, with 5 cm of margin);
  point lights against a sphere; spot lights against a sphere and a cone; directional lights reach everything.
- A cluster takes up to 64 lights. The ones that come first in `lights[]` win, which is the order from
  `Update_Lights()`. The pairs that did not fit are counted in `overflow_count`.
- The list is one shared array: each cluster reserves its place with one `atomicAdd` on `index_count`.
- The counters are copied to a ring of four small buffers with fences and read some frames later, so the
  statistics never stall the GPU.

The headlight projection has no far plane (infinite, reverse-Z), so the reach of a headlight comes from
its falloff and the sphere.

## Shading

`shaders/light_common.glsl`:

- `fragment_cluster()` gives the cluster index of the fragment as described above.
- `add_headlights()` is the single loop over headlights, used by `apply_lights()` and by the normal-map
  materials without sun: if `lights_clustered` is set it goes through
  `cluster_light_indices[offset .. offset + count)`, otherwise through `1 .. lights_count - 1` of `lights[]`.
- `calc_headlights()` samples the beam texture only inside the 0..1 window of the projection, with
  `textureGrad` and derivatives taken from the position before the loop: fragments of one quad may have
  different lists at a cluster edge, and derivatives inside divergent control flow are undefined.
- The sun and `calc_shadow()` (cascaded shadow map with `textureGather`) are outside the loop.

`light_ssbo` and the cluster blocks are declared only for the fragment and compute stages; GL 4.6 does not
guarantee any storage blocks in the vertex and geometry stages.

## Changing it

- Grid size, per-cluster capacity, `MAX_LIGHTS`, binding numbers: `gl/ubo.h`, then `gl/glsl_common.cpp` takes them over.
  The work group size in the shaders and the number of groups dispatched both come from `CLUSTER_*_GROUP_SIZE`.
- Check the shaders with `ci_shadervalidator` (see [opengl46-renderer.md](opengl46-renderer.md)); CI does it for
  changes in `shaders/` and `gl/`.
- Only vehicle headlights feed `lights[]` today. The culling already knows `SPOT`, `POINT` and `DIR`, but
  `add_headlights()` runs `calc_headlights()` for every listed light, so a new kind of light also needs
  its own branch there.

## Not covered

- The environment map pass has no clusters of its own; it has the sun only.
- A grid for extra viewports and VR is built, but those paths have not been verified on hardware.
