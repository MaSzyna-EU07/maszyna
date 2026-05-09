#include "stdafx.h"
#include "glsl_common.h"

std::string gl::glsl_common;

void gl::glsl_common_setup()
{
    glsl_common =
    "#define SHADOWMAP_ENABLED " + std::to_string((int)Global.gfx_shadowmap_enabled) + "\n" +
    "#define ENVMAP_ENABLED " + std::to_string((int)Global.gfx_envmap_enabled) + "\n" +
    "#define MOTIONBLUR_ENABLED " + std::to_string((int)Global.gfx_postfx_motionblur_enabled) + "\n" +
    "#define POSTFX_ENABLED " + std::to_string((int)!Global.gfx_skippipeline) + "\n" +
    "#define EXTRAEFFECTS_ENABLED " + std::to_string((int)Global.gfx_extraeffects) + "\n" +
    "#define MAX_LIGHTS " + std::to_string(MAX_LIGHTS) + "U\n" +
    "#define MAX_CASCADES " + std::to_string(MAX_CASCADES) + "U\n" +
    "#define MAX_PARAMS " + std::to_string(MAX_PARAMS) + "U\n" +
    "#define MAX_INSTANCES_PER_BATCH " + std::to_string(MAX_INSTANCES_PER_BATCH) + "U\n" +
    R"STRING(
    const uint LIGHT_SPOT = 0U;
    const uint LIGHT_POINT = 1U;
    const uint LIGHT_DIR = 2U;
    const uint LIGHT_HEADLIGHTS = 3U;

    struct light_s
    {
            vec3 pos;
            uint type;

            vec3 dir;
            float in_cutoff;

            vec3 color;
            float out_cutoff;

            float linear;
            float quadratic;

	        float intensity;
	        float ambient;

            mat4 headlight_projection;
            vec4 headlight_weights;
    };

    layout(std140) uniform light_ubo
    {
            vec3 ambient;

            vec3 fog_color;
            uint lights_count;

            light_s lights[MAX_LIGHTS];
    };

    layout (std140) uniform model_ubo
    {
            mat4 modelview;
            mat3 modelviewnormal;
            vec4 param[MAX_PARAMS];

            mat4 future;
            float opacity;
            float emission;
            float fog_density;
            float alpha_mult;
            float shadow_tone;
    };

    layout (std140) uniform scene_ubo
    {
        mat4 projection;
        mat4 inv_view;
        mat4 lightview[MAX_CASCADES];
        vec3 cascade_end;
        float time;
		vec4 rain_params;
		vec4 wiper_pos;
		vec4 wiper_timer_out;
		vec4 wiper_timer_return;
    };

    // Per-instance modelview matrices for GPU-instanced draws.
    // The UBO itself is declared in every stage so the binding stays valid,
    // but the helper functions reference gl_InstanceID which is vertex-only,
    // so they are guarded behind STAGE_VERTEX.
    layout (std140) uniform instance_ubo
    {
        mat4 instance_modelview[MAX_INSTANCES_PER_BATCH];
    };

    #ifdef STAGE_VERTEX
    // For non-instanced draws gl_InstanceID == 0 and slot 0 is permanently set
    // to identity, so effective_modelview() reduces to model_ubo.modelview.
    // For instanced draws the C++ side uploads camera-space root transforms for
    // instances 0..N-1 and sets model_ubo.modelview to the submodel-local chain
    // (computed from identity, NOT from the camera/instance), so the product
    // gives the correct final transform per instance per submodel.
    mat4 effective_modelview() {
        return instance_modelview[gl_InstanceID] * modelview;
    }
    mat3 effective_modelviewnormal() {
        // instance_modelview is rotation+translation only (no scale), so its
        // upper-3x3 is its own inverse-transpose (rotation matrix).
        return mat3(instance_modelview[gl_InstanceID]) * modelviewnormal;
    }
    #endif

    )STRING";
}
