#include "stdafx.h"
#include "glsl_common.h"
#include "rendering/common/shadersource.h"

std::string gl::glsl_common;

void gl::glsl_common_setup()
{
    // the values shaders/ogl46/common.glsl takes instead of its own (MASZYNA_CONSTANTS). the settings of the game aren't here,
    // they are specialization constants of the binaries (glsl_settings_defines(), glsl_settings_values())
    glsl_common =
    "#define MASZYNA_CONSTANTS 1\n"
    "#define MAX_LIGHTS " + std::to_string(MAX_LIGHTS) + "U\n" +
    "#define MAX_CASCADES " + std::to_string(MAX_CASCADES) + "U\n" +
    "#define MAX_PARAMS " + std::to_string(MAX_PARAMS) + "U\n" +
    "#define SHADOW_TEX " + std::to_string(SHADOW_TEX) + "\n" +
    "#define ENV_TEX " + std::to_string(ENV_TEX) + "\n" +
    "#define HEADLIGHT_TEX " + std::to_string(HEADLIGHT_TEX) + "\n" +
    "#define ENV_GGX_TEX " + std::to_string(ENV_GGX_TEX) + "\n" +
    "#define SCENE_UBO_BINDING " + std::to_string(SCENE_UBO_BINDING) + "\n" +
    "#define MODEL_UBO_BINDING " + std::to_string(MODEL_UBO_BINDING) + "\n" +
    "#define INSTANCE_SSBO_BINDING " + std::to_string(INSTANCE_SSBO_BINDING) + "\n" +
    "#define INSTANCE_RECORD_SSBO_BINDING " + std::to_string(INSTANCE_RECORD_SSBO_BINDING) + "\n" +
    "#define INSTANCE_JOB_SSBO_BINDING " + std::to_string(INSTANCE_JOB_SSBO_BINDING) + "\n" +
    "#define INSTANCE_GROUP_SSBO_BINDING " + std::to_string(INSTANCE_GROUP_SSBO_BINDING) + "\n" +
    "#define INSTANCE_LOD_SSBO_BINDING " + std::to_string(INSTANCE_LOD_SSBO_BINDING) + "\n" +
    "#define INSTANCE_COUNTER_SSBO_BINDING " + std::to_string(INSTANCE_COUNTER_SSBO_BINDING) + "\n" +
    "#define INSTANCE_OUTPUT_SSBO_BINDING " + std::to_string(INSTANCE_OUTPUT_SSBO_BINDING) + "\n" +
    "#define INSTANCE_COMMAND_SSBO_BINDING " + std::to_string(INSTANCE_COMMAND_SSBO_BINDING) + "\n" +
    "#define MODEL_DRAW_SSBO_BINDING " + std::to_string(MODEL_DRAW_SSBO_BINDING) + "\n" +
    "#define INSTANCE_CULL_GROUP_SIZE " + std::to_string(INSTANCE_CULL_GROUP_SIZE) + "\n" +
    "#define INSTANCE_OFFSETS_GROUP_SIZE " + std::to_string(INSTANCE_OFFSETS_GROUP_SIZE) + "\n" +
    "#define INSTANCE_COMMAND_GROUP_SIZE " + std::to_string(INSTANCE_COMMAND_GROUP_SIZE) + "\n" +
    "#define HIZ_GROUP_SIZE " + std::to_string(HIZ_GROUP_SIZE) + "\n" +
    "#define SHADOW_FIT_GROUP_SIZE " + std::to_string(SHADOW_FIT_GROUP_SIZE) + "\n" +
    "#define SHADOW_FIT_SSBO_BINDING " + std::to_string(SHADOW_FIT_SSBO_BINDING) + "\n" +
    "#define PARTICLE_GROUP_SIZE " + std::to_string(PARTICLE_GROUP_SIZE) + "\n" +
    "#define PARTICLE_SSBO_BINDING " + std::to_string(PARTICLE_SSBO_BINDING) + "\n" +
    "#define PARTICLE_SOURCE_SSBO_BINDING " + std::to_string(PARTICLE_SOURCE_SSBO_BINDING) + "\n" +
    "#define PARTICLE_OWNER_SSBO_BINDING " + std::to_string(PARTICLE_OWNER_SSBO_BINDING) + "\n" +
    "#define PARTICLE_OUTPUT_SSBO_BINDING " + std::to_string(PARTICLE_OUTPUT_SSBO_BINDING) + "\n" +
    "#define PARTICLE_COMMAND_SSBO_BINDING " + std::to_string(PARTICLE_COMMAND_SSBO_BINDING) + "\n" +
    "#define INSTANCE_COUNTER_HEADER " + std::to_string(INSTANCE_COUNTER_HEADER) + "\n" +
    "#define INSTANCE_JOB_SLEEPERS " + std::to_string(INSTANCE_JOB_SLEEPERS) + "U\n" +
    "#define LIGHT_SSBO_BINDING " + std::to_string(LIGHT_SSBO_BINDING) + "\n" +
    "#define CLUSTER_GRID_X " + std::to_string(CLUSTER_GRID_X) + "\n" +
    "#define CLUSTER_GRID_Y " + std::to_string(CLUSTER_GRID_Y) + "\n" +
    "#define CLUSTER_GRID_Z " + std::to_string(CLUSTER_GRID_Z) + "\n" +
    "#define CLUSTER_COUNT " + std::to_string(CLUSTER_COUNT) + "\n" +
    "#define MAX_LIGHTS_PER_CLUSTER " + std::to_string(MAX_LIGHTS_PER_CLUSTER) + "\n" +
    "#define CLUSTER_BUILD_GROUP_SIZE " + std::to_string(CLUSTER_BUILD_GROUP_SIZE) + "\n" +
    "#define CLUSTER_CULL_GROUP_SIZE " + std::to_string(CLUSTER_CULL_GROUP_SIZE) + "\n" +
    "#define CLUSTER_UBO_BINDING " + std::to_string(CLUSTER_UBO_BINDING) + "\n" +
    "#define CLUSTER_AABB_SSBO_BINDING " + std::to_string(CLUSTER_AABB_SSBO_BINDING) + "\n" +
    "#define CLUSTER_RANGE_SSBO_BINDING " + std::to_string(CLUSTER_RANGE_SSBO_BINDING) + "\n" +
    "#define CLUSTER_INDEX_SSBO_BINDING " + std::to_string(CLUSTER_INDEX_SSBO_BINDING) + "\n" +
    "#define CLUSTER_COUNTER_SSBO_BINDING " + std::to_string(CLUSTER_COUNTER_SSBO_BINDING) + "\n" +
    "#define ENVMAP_GGX_LEVELS " + std::to_string(ENVMAP_GGX_LEVELS) + "\n" +
    "#define ENVMAP_GGX_ROUGHNESS_MIN " + std::to_string(ENVMAP_GGX_ROUGHNESS_MIN) + "\n" +
    "#define ENVMAP_GGX_GROUP_SIZE " + std::to_string(ENVMAP_GGX_GROUP_SIZE) + "\n" +
    "#define SKY_SSBO_BINDING " + std::to_string(SKY_SSBO_BINDING) + "\n" +
    "#define EXPOSURE_SSBO_BINDING " + std::to_string(EXPOSURE_SSBO_BINDING) + "\n" +
    "#define EXPOSURE_HISTOGRAM_BINS " + std::to_string(EXPOSURE_HISTOGRAM_BINS) + "\n" +
    "#define EXPOSURE_HISTOGRAM_GROUP_SIZE " + std::to_string(EXPOSURE_HISTOGRAM_GROUP_SIZE) + "\n" +
    "#define ATMOSPHERE_UBO_BINDING " + std::to_string(ATMOSPHERE_UBO_BINDING) + "\n" +
    "#define ATMOSPHERE_RESULT_SSBO_BINDING " + std::to_string(ATMOSPHERE_RESULT_SSBO_BINDING) + "\n" +
    "#define ATMOSPHERE_TRANSMITTANCE_WIDTH " + std::to_string(ATMOSPHERE_TRANSMITTANCE_WIDTH) + "\n" +
    "#define ATMOSPHERE_TRANSMITTANCE_HEIGHT " + std::to_string(ATMOSPHERE_TRANSMITTANCE_HEIGHT) + "\n" +
    "#define ATMOSPHERE_MULTISCATTERING_SIZE " + std::to_string(ATMOSPHERE_MULTISCATTERING_SIZE) + "\n" +
    "#define ATMOSPHERE_SKYVIEW_WIDTH " + std::to_string(ATMOSPHERE_SKYVIEW_WIDTH) + "\n" +
    "#define ATMOSPHERE_SKYVIEW_HEIGHT " + std::to_string(ATMOSPHERE_SKYVIEW_HEIGHT) + "\n" +
    "#define ATMOSPHERE_AMBIENT_GROUP_SIZE " + std::to_string(ATMOSPHERE_AMBIENT_GROUP_SIZE) + "\n" +
    "#define ATMOSPHERE_TRANSMITTANCE_UNIT " + std::to_string(ATMOSPHERE_TRANSMITTANCE_UNIT) + "\n" +
    "#define ATMOSPHERE_MULTISCATTERING_UNIT " + std::to_string(ATMOSPHERE_MULTISCATTERING_UNIT) + "\n" +
    "#define ATMOSPHERE_SKYVIEW_UNIT " + std::to_string(ATMOSPHERE_SKYVIEW_UNIT) + "\n" +
    "#define SKY_VIEW_TEX " + std::to_string(SKY_VIEW_TEX) + "\n" +
    "#define LOCAL_SHADOW_TEX " + std::to_string(LOCAL_SHADOW_TEX) + "\n" +
    "#define LOCAL_SHADOW_LAMPS_TEX " + std::to_string(LOCAL_SHADOW_LAMPS_TEX) + "\n" +
    "#define HEADLIGHT_SHADOW_TEX " + std::to_string(HEADLIGHT_SHADOW_TEX) + "\n" +
    "#define MAX_HEADLIGHT_SHADOWS " + std::to_string(MAX_HEADLIGHT_SHADOWS) + "U\n" +
    "#define FOG_VOLUME_TEX " + std::to_string(FOG_VOLUME_TEX) + "\n" +
    "#define CLOUD_SHADOW_TEX " + std::to_string(CLOUD_SHADOW_TEX) + "\n" +
    "#define CLOUD_TEX " + std::to_string(CLOUD_TEX) + "\n" +
    "#define CLOUD_SHAPE_TEX " + std::to_string(CLOUD_SHAPE_TEX) + "\n" +
    "#define CLOUD_DETAIL_TEX " + std::to_string(CLOUD_DETAIL_TEX) + "\n" +
    "#define CLOUD_COVERAGE_TEX " + std::to_string(CLOUD_COVERAGE_TEX) + "\n" +
    "#define CLOUD_HISTORY_TEX " + std::to_string(CLOUD_HISTORY_TEX) + "\n" +
    "#define SCENE_DEPTH_TEX " + std::to_string(SCENE_DEPTH_TEX) + "\n" +
    "#define WEATHER_UBO_BINDING " + std::to_string(WEATHER_UBO_BINDING) + "\n" +
    "#define WEATHER_SCRATCH_SSBO_BINDING " + std::to_string(WEATHER_SCRATCH_SSBO_BINDING) + "\n" +
    "#define MAX_CLOUD_LAYERS " + std::to_string(MAX_CLOUD_LAYERS) + "\n" +
    "#define FOG_HORIZON_SAMPLES " + std::to_string(FOG_HORIZON_SAMPLES) + "\n" +
    "#define WEATHER_GROUP_SIZE " + std::to_string(WEATHER_GROUP_SIZE) + "\n" +
    "#define FOG_VOLUME_BIAS " + std::to_string(FOG_VOLUME_BIAS) + "\n";
}

std::string gl::glsl_settings_defines()
{
    auto const value = [](bool const Value) { return std::string(Value ? "true" : "false"); };
    return
        "#define SHADOWMAP_ENABLED_VALUE " + value(Global.gfx_shadowmap_enabled) + "\n"
        "#define ENVMAP_ENABLED_VALUE " + value(Global.gfx_envmap_enabled) + "\n"
        "#define MOTIONBLUR_ENABLED_VALUE " + value(Global.gfx_motion_vectors()) + "\n"
        "#define POSTFX_ENABLED_VALUE " + value(!Global.gfx_skippipeline) + "\n"
        "#define EXTRAEFFECTS_ENABLED_VALUE " + value(Global.gfx_extraeffects) + "\n"
        "#define OUTPUT_GAMMA_VALUE " + value(Global.gfx_shadergamma) + "\n";
}

std::array<GLuint, 6> gl::glsl_settings_values()
{
    // in the order of gl::shader_setting
    return {
        static_cast<GLuint>(Global.gfx_shadowmap_enabled),
        static_cast<GLuint>(Global.gfx_envmap_enabled),
        static_cast<GLuint>(Global.gfx_motion_vectors()),
        static_cast<GLuint>(!Global.gfx_skippipeline),
        static_cast<GLuint>(Global.gfx_extraeffects),
        static_cast<GLuint>(Global.gfx_shadergamma) };
}

bool gl::glsl_common_matches(std::string const &Basedir, std::vector<std::string> &Differences)
{
    return gfx::shader_constants_match(gfx::shader_api::opengl46, "common.glsl", glsl_common, Basedir, Differences);
}
