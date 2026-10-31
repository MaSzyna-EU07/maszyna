# Shaders

| Directory | What is in it |
|---|---|
| `ogl46/` | GLSL sources of the OpenGL 4.6 renderer |
| `ogl46bin/` | SPIR-V binaries of `ogl46/` (optional, made by `compile_spirv.py`) |
| `vk14/` | GLSL sources of the Vulkan 1.4 renderer |
| `vk14bin/` | SPIR-V binaries of `vk14/` (optional, made by `compile_spirv.py`) |

The renderers load the binary of a shader first, when there is one and it was compiled from the sources that are there now
(`manifest.txt` of the directory of the binaries lists the sources of each binary with their hashes). Otherwise the source is
compiled: by the OpenGL driver for `ogl46/`, by glslang inside the simulator for `vk14/` (with a cache of the results in the
directory of the user). `gfx.shaders.binaries no` in `eu07.ini` makes them always use the sources. The sources are needed
even with the binaries: the directives of the materials (`#pragma texture`, `#pragma param`) are read from them.

## Compiling to SPIR-V

    python shaders/compile_spirv.py

compiles every shader of `ogl46/` (with the variant `MODEL_DRAWS` of the vertex and fragment shaders) and of `vk14/` with
`glslc` (Vulkan SDK, `VULKAN_SDK`, or `--glslc <path>`). A single file compiles the same way by hand:

    glslc --target-env=opengl4.5 ogl46/mat_default.frag -o ogl46bin/mat_default.frag.spv
    glslc --target-env=opengl4.5 -DMODEL_DRAWS=1 ogl46/vertex.vert -o ogl46bin/vertex.vert.MODEL_DRAWS.spv
    glslc --target-env=vulkan1.4 vk14/imgui.frag -o vk14bin/imgui.frag.spv

A binary made by hand isn't in the manifest, so it is taken without being compared with its source. After a change of a
source, compile the binaries again (or delete them): a binary older than its source is not used, and the log says so.

## Writing shaders

* Every stage file starts with `#version 460 core` and `#define STAGE_VERTEX 1` (`STAGE_FRAGMENT`, `STAGE_GEOMETRY`,
  `STAGE_COMPUTE`); files included by others (`.glsl`) don't.
* Includes: `#include "file.glsl"`, relative to the directory of the file.
* Every input and output between the stages has an explicit `layout(location = N)`, every sampler, image and block an explicit
  binding, every loose uniform an explicit location: SPIR-V has no names to link by. The locations of the inputs and outputs
  of `ogl46/` are one table for all the shaders (a name has the same location everywhere), see `ogl46/common.glsl`.
* The settings of the game (`SHADOWMAP_ENABLED`, `ENVMAP_ENABLED`, `MOTIONBLUR_ENABLED`, `POSTFX_ENABLED`,
  `EXTRAEFFECTS_ENABLED`, `OUTPUT_GAMMA`) are `const bool` (specialization constants in the binaries, `ogl46/settings.glsl`):
  use them in `if`, not in `#if`.
* The constants of the game (bindings, sizes) are written in `ogl46/common.glsl` and `vk14/common.glsl` for the binaries; the
  game puts its own values before the sources and checks at the start that the files have the same ones (the shader
  validator in `ci_shadervalidator` checks them too).

## vk14

The shaders of `vk14/` are those of `ogl46/` made for the descriptor sets of the Vulkan renderer
(`rendering/vulkan14/vulkan14bindings.h`), drawn with the same matrices and the same conventions as the OpenGL renderer
(nothing is turned upside down but the picture shown in the window, `present.frag`).

* A stage file includes `common.glsl` right after its stage define. The pipeline layout it is drawn with is picked by a
  define before that include: none for the passes of the scene, `LAYOUT_POST` for the post-processing (with `quad.vert`),
  `LAYOUT_CLUSTER` for the compute passes of the light clusters, `LAYOUT_UI` for the user interface.
* The scene: set `PASS_SET` holds the blocks of the scene, of the draws (`model_draws[]`, the one of a draw picked by the push
  constant `pc.draw`), of the instances and of the lights and clusters, and the shadow, environment and headlight maps;
  set `MATERIAL_SET` the textures of the material at `MATERIAL_TEXTURE_FIRST + unit` of their `#pragma texture`. The names of
  the model block (`modelview`, `param`, `opacity` ...) stand for the fields of the draw, as in the `MODEL_DRAWS` variant of
  `ogl46/`.
* The push constants (`pc`, `PUSH_CONSTANTS_SIZE` bytes) carry the draw and the parameters of the special shaders in
  `pc.data[]`; each of those says at the top what it takes from them. In the post-processing `param` stands for `pc.data`.
* The settings are specialization constants with the ids `SPEC_*` (`vk14/settings.glsl`).
* The features of the later stages of the renderer (the shadows of the lamps and the headlights, the contact shadows, the
  weather and its fog, the sky ambient light, the prefiltered environment map, the automatic exposure, soft and gpu
  particles) aren't in `vk14/` yet; the code paths are the ones `ogl46/` takes with them off, the fog is an analytic one
  (`vk14/apply_fog.glsl`).
