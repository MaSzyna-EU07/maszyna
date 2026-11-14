#version 460 core
#define STAGE_FRAGMENT 1
// Windscreen snow: replaces rain_windscreen when it snows - v3 (OpenGL 4.6 / Vulkan 1.4)
//
// MaSzyna switches the weather itself (simulationenvironment.cpp): with Overcast > 1 it SNOWS at 1 deg C and below, and the .mat
// block "snow:" is picked.
//
// What lies on the glass comes from the simulation (water_snow.comp, water.glsl): the map of the glass keeps the snow, the water,
// how packed the snow is and the shape of its flakes from frame to frame. The flakes land and stay where they landed, pile up,
// the blades wipe it as they wipe the rain (a wet trace stays), it melts at the warm glass, slush slides down. The renderer gives
// the glass its map through the param snow_slot (1 + its slot); a glass without a map has no snow.
//
// The look is only sprites from the atlas, as the drops of rain_windscreen and the splats of bugs_windscreen, laid together into
// a layer of the glass by water_snowlayer.comp (snow_sprites.glsl) when the snow under them changes, so the glass only reads it:
//  - the lumps, in nine grids of ~1.6 cm cells shifted against each other: each cell has its lump fixed once (tile, place,
//    size, the snow it takes); it comes when the snow around reaches that and stays as it is until the snow goes. The grids come
//    one after the other as the snow piles up (all of them icy fleece, more and bigger; a lump of packed snow very rarely), each on the ones before,
//    so the snow accumulates rather than changes, and covers the glass by 2-3 mm, small gaps left (the scene behind them blurred). In a heap they come sooner
//  - dots and small puffs of dry snow between them, and drops where a flake has melted (they bend the picture behind, as rain does)
// A thin flake lets the light through (ice crystals): the picture behind it is blurred, not hidden.
//
// Atlas / normalmap (snieg_atlas_maszyna.py), 4 x 4 tiles of 512 x 512 (2048 x 2048):
//    0 a dot             1 dots           2, 3 small puffs    4, 5 puffs                     6, 7 crumbs
//    8, 9 wet lumps      10, 11 clumps    12, 13 packed snow 14 slush                        15 drop
// An older 2 x 2 atlas (1024 x 1024: fluffy flake, wet lump, crumbs, melted spot) still works, its tiles stand in for these.
//
// .mat:
//
//   snow: {
//   shader: snow_windscreen
//   texture_raindropsatlas: textures/fx/snow-atlas
//   texture_normalmap: textures/fx/snow_normalmap
//   }

layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;
layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;
#include "common.glsl"

#pragma texture(diffuse,         0, sRGB_A)
#pragma texture(raindropsatlas,  1, sRGB_A)
#pragma texture(wipermask,       2, sRGB_A)  // the renderer bakes it into the map of the glass (the blades); not read here
#pragma texture(normalmap,       3, RGBA)    // rg = normal xy, b = height, a = mask; the layout of raindropsatlas (linear)
#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, specular)
#pragma param(reflection, 1, 2, 1, zero)
#pragma param(glossiness, 1, 3, 1, glossiness)
#pragma param(raindrop_grid_size, 2, 0, 1, one)  // not used any more (the map of the glass sets the size of the flakes)
// snow_slot: 1 + the slot of the glass in the simulation of the snow (the renderer sets it), 0: none
#pragma param(snow_slot, 2, 2, 1, zero)

#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"
#include "random.glsl"
#include "water.glsl"

layout(binding = 0) uniform sampler2D diffuse;
layout(binding = 1) uniform sampler2D raindropsatlas;
layout(binding = 2) uniform sampler2D wipermask;
layout(binding = 3) uniform sampler2D normalmap;

#include "snow_sprites.glsl"

void main()
{
    // Derivatives first, in uniform control flow (no discard before them).
    vec3  Ngl     = normalize(f_normal);
    vec3  dp1     = dFdx(f_pos.xyz);
    vec3  dp2     = dFdy(f_pos.xyz);
    vec2  duv1    = dFdx(f_coord);
    vec2  duv2    = dFdy(f_coord);
    vec3  dp2perp = cross(dp2, Ngl);
    vec3  dp1perp = cross(Ngl, dp1);
    vec3  Tgl     = dp2perp * duv1.x + dp1perp * duv2.x;
    vec3  Bgl     = dp2perp * duv1.y + dp1perp * duv2.y;
    float invmax  = inversesqrt(max(max(dot(Tgl, Tgl), dot(Bgl, Bgl)), 1e-20));
    Tgl *= invmax;
    Bgl *= invmax;

    vec4 tex_color = texture(diffuse, f_coord);   // no discard: clean glass between faint dirt must still get snow and blur
    // the snow laid together from the sprites (water_snowlayer.comp, a band of its rows every step and wherever a blade goes)
    vec3  snowRGB = vec3(0.0);
    float snowA   = 0.0;
    vec3  snowN   = vec3(0.0);
    float snowTr  = 0.0;
    float wet = 0.0;
    snSlot = int(param[2].z + 0.5) - 1;
    if ((snSlot >= 0) && (snSlot < WATER_SLOTS) && water_snow_map(uint(snSlot), snMap))
    {
        vec4 layerColour;
        vec4 layerNormal;
        vec4 layerBend;
        water_layer_at(uint(snSlot), f_coord, layerColour, layerNormal, layerBend);
        snowRGB = layerColour.rgb;
        snowA   = layerColour.a;
        snowN   = layerNormal.xyz;
        snowTr  = layerNormal.w;
        snBend  = layerBend.xy;
        snBendA = layerBend.z;
        vec4 q = water_snow_at(uint(snSlot), snMap, f_coord);
        wet = smoothstep(0.05, 0.3, q.y) * (1.0 - smoothstep(0.5, 2.0, q.x));
        // the picture behind thin flakes and a wet film is blurred
        snHaze = max(layerBend.w, wet * snow_wet_blur);
        snHaze = max(snHaze, smoothstep(snow_blur_mm.x, snow_blur_mm.y, q.x));
    }

    float glassA  = tex_color.a * alpha_mult;
    float finalA  = snowA + glassA * (1.0 - snowA);
    vec3  premult = snowRGB + tex_color.rgb * glassA * (1.0 - snowA);
    vec3  albedo  = premult / max(finalA, 1e-4);
    float glassShare = glassA * (1.0 - snowA) / max(finalA, 1e-4);

    // relief: tilt the glass normal by the normals of the sprites (flat where there are none)
    vec3  nDir       = snowN + vec3(0.0, 0.0, 1.0) * (1.0 - snowA);
    vec3  fragnormal = Ngl;
    if (snowA > 0.001)
        fragnormal = normalize(Ngl * max(nDir.z, 0.05) + Tgl * nDir.x + Bgl * nDir.y);

    vec3  fragcolor    = ambient;
    float reflectivity = param[1].z * glassShare;
    float specularity  = (tex_color.r + tex_color.g + tex_color.b) * 0.5 * glassShare + snow_gloss * snowA;
    legacy_gloss();

    // most of the glass is clear: the lighting of the 4.6 renderers is skipped there, it would be multiplied by an alpha of ~0
    if (finalA > 0.002)
        fragcolor = apply_lights(fragcolor, fragnormal, albedo, reflectivity, specularity, shadow_tone);

    // thin snow glows when you look towards the sun / a lamp
    if ((finalA > 0.002) && (snowA > 0.001) && (snow_translucency > 0.0) && (lights_count > 0U))
    {
        vec3  I = normalize(f_pos.xyz);
        float shadowSun = 0.0;
        if (shadow_tone < 1.0)
            shadowSun = (1.0 - shadow_tone) * clamp(calc_shadow(), 0.0, 1.0);
        float towards = max(dot(I, normalize(-lights[0].dir)), 0.0);
        float glow    = pow(towards, 3.0) * (1.0 - shadowSun);
        fragcolor += lights[0].color * lights[0].intensity * glow * snow_translucency * (snowTr / max(snowA, 1e-3)) * albedo * snowA;
    }
    vec3 lit = apply_fog(fragcolor);
    vec4 color = vec4(lit, finalA);   // straight, as rain_windscreen of the 4.6 renderers writes the glass

    out_color = POSTFX_ENABLED ? color : tonemap(color);

    // the scene seen through the snow, drawn by the post-processing (rain_windscreen_haze.glsl), asked for in the velocity target as
    // rain_windscreen asks: the drops bend it (x, y = WS_DROP_BIAS + WS_BEND_ENC * bend), thin snow and a wet film blur it
    // (x = WS_ENC * strength, y = 0); the snow itself stays sharp. alpha 0 elsewhere: the blending keeps the motion of the scene behind
    if (MOTIONBLUR_ENABLED)
    {
        vec4 req = vec4(0.0);
        float clear = 1.0 - smoothstep(0.5, 0.95, glassA);
        if ((snBendA > 0.1) && (glassA < 0.5))
        {
            // the post-processing puts the picture from the bent place in place of this pixel, snow and all (85% of it): through
            // snow it hides, the snow itself would be replaced by the picture from a place up to 96 px away, a gap of the snow
            // there showing here, so an opaque cover would never close. bent only as much as the snow lets the light through
            float through = 1.0 - smoothstep(0.6, 0.98, snowA);
            vec2 bend = clamp(-snBend / max(snBendA, 1e-3) * snow_drop_bend, vec2(-1.0), vec2(1.0)) * smoothstep(0.1, 0.4, snBendA) * through;
            req = vec4(WS_DROP_BIAS + WS_BEND_ENC * bend.x, WS_DROP_BIAS + WS_BEND_ENC * bend.y, 0.0, 1.0);
        }
        else
        {
            float blurMask = clamp(snHaze * snow_haze * (1.0 - smoothstep(0.3, 0.8, snowA)) * clear, 0.0, 1.0);
            if (blurMask >= 0.01)
                req = vec4(WS_ENC * blurMask, 0.0, 0.0, 1.0);
        }
        out_motion = req;
    }
}
