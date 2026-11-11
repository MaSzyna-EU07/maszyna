#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// Windscreen snow: replaces rain_windscreen when it snows - v3 (OpenGL 4.6 / Vulkan 1.4)
//
// MaSzyna switches the weather itself (simulationenvironment.cpp): with Overcast > 1 it SNOWS at 1 deg C and below, and the .mat
// block "snow:" is picked.
//
// What lies on the glass comes from the simulation (water_snow.comp, water.glsl): the map of the glass keeps the snow, the water,
// how packed the snow is and the shape of its flakes from frame to frame. The flakes land and stay where they landed, pile up,
// the blades push the snow into ridges at the ends of their sweeps and leave a wet trace, it melts at the warm glass, slush slides
// down. The renderer gives the glass its map through the param snow_slot (1 + its slot); a glass without a map has no snow.
//
// The look is only sprites from the atlas, as the drops of rain_windscreen and the splats of bugs_windscreen, two layers of them:
//  - the flakes: a sprite on every texel of the map holding snow, the more snow the bigger. Dry: crystals, flakes of a few of
//    them, crumbs; wet: lumps; where only the water of a melted flake is left, a drop (it bends the picture behind, as rain does)
//  - the layer, under the flakes: a bigger sprite every three texels where the snow is thicker, clumps of flakes, then packed
//    snow, overlapping into a whole cover where the snow is thick; grey slush where it's wet
// A thin flake lets the light through (ice crystals): the picture behind it is blurred, not hidden.
//
// Atlas / normalmap (snieg_atlas_maszyna.py), 4 x 4 tiles of 512 x 512 (2048 x 2048):
//    0, 1 star crystals   2 plate          3 needles          4, 5 flakes of a few crystals   6, 7 crumbs
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
#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

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

layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 1) uniform sampler2D raindropsatlas;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 2) uniform sampler2D wipermask;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 3) uniform sampler2D normalmap;

// --- tuning ------------------------------------------------------------------------------------------------------------------
const float snow_flake      = 1.2;   // size of a flake, texels of the map (grows to 1.6x with the snow of its texel)
const float snow_layer      = 0.6;   // mm of snow from which the layer starts (clumps), the cover grows to full by 1.5 mm
const float snow_layer_cell = 3.0;   // a sprite of the layer every this many texels
const float snow_opaque     = 1.8;   // mm of fresh snow letting 1/e of the light through (packed: half of it)
const float snow_thin       = 0.45;  // opacity of a single thin flake (ice crystals let most of the light through)
const float snow_bump       = 1.0;   // strength of the relief from the normal map, 0 = flat
const float snow_gloss      = 0.45;  // glints of the sun / headlights on the snow, 0 = matt
const float snow_translucency = 0.8; // glow of thin snow when you look towards the sun / a lamp, 0 = off
const float snow_haze       = 1.0;   // blur of the scene behind thin snow (frosted glass), 0 = off
const float snow_drop_bend  = 1.0;   // how strongly the drops bend the picture behind them
const float snow_wet_blur   = 0.6;   // blur of the scene behind a wet film

const float WS_REFR_MAXPX = 96.0;    // the largest picture shift the post-processing makes, screen pixels; as rain_windscreen_haze.glsl
const float WS_BEND_ENC   = 60.0;    // the bend request in the velocity target; as rain_windscreen_haze.glsl
const float WS_DROP_BIAS  = 192.0;
const float WS_ENC        = 64.0;    // the haze request in the velocity target; as rain_windscreen_haze.glsl

const float SNOW_FRESH  = 0.1;       // water in a mm of fresh snow, mm (as water_snow.comp)
const float SNOW_PACKED = 0.4;       // and of packed snow

// tiles of the atlas
const int TILE_DROP = 15;

int   snSlot = 0;
uint  snMap  = 0u;
bool  snBig  = true;      // a 4 x 4 atlas (else the older 2 x 2)
float snHaze = 0.0;       // the picture behind is blurred (thin snow, a wet film)
vec2  snBend = vec2(0.0); // the drops bend the picture behind (their normals, weighted)
float snBendA = 0.0;

struct sprite_acc
{
    vec3 rgb;     // premultiplied
    float a;
    vec3 n;       // premultiplied, tangent space
    float tr;     // translucency, premultiplied
};

// the corner of a tile in the atlas; an older 2 x 2 atlas stands its four tiles in for the sixteen
vec2 snTile(int tile)
{
    if (snBig)
        return vec2(float(tile % 4), float(tile / 4)) * 0.25;
    int old = 0;                                  // a fluffy flake
    if ((tile == 6) || (tile == 7))
        old = 2;                                  // crumbs
    else if ((tile >= 8) && (tile != 10) && (tile != 11) && (tile != TILE_DROP))
        old = 1;                                  // a wet lump
    else if (tile == TILE_DROP)
        old = 3;                                  // a melted spot
    return vec2(float(old % 2), float(old / 2)) * 0.5;
}

float snTileSize()
{
    return snBig ? 0.25 : 0.5;
}

// a sprite over the accumulator: Local the place in it (0 .. 1), Rot its rotation (cos, sin), Tone its colour
void snSprite(vec2 local, int tile, float alpha, vec3 tone, vec2 rot, float flip, float lod, float bump, inout sprite_acc acc)
{
    vec2 uv = snTile(tile) + local * snTileSize();
    vec4 t = textureLod(raindropsatlas, uv, lod);
    vec4 n = textureLod(normalmap, uv, lod);
    float a = clamp(t.a * alpha, 0.0, 1.0);
    if (a <= 0.002)
        return;
    vec2 nxy = (n.rg * 2.0 - 1.0) * snow_bump * bump;
    nxy.x *= flip;
    vec2 nuv = vec2(rot.x * nxy.x - rot.y * nxy.y, rot.y * nxy.x + rot.x * nxy.y);
    vec3 nT = vec3(nuv, sqrt(max(1.0 - dot(nuv, nuv), 0.02)));
    acc.rgb = t.rgb * tone * a + acc.rgb * (1.0 - a);
    acc.n   = nT * a + acc.n * (1.0 - a);
    acc.tr  = (1.0 - t.a * 0.55) * a + acc.tr * (1.0 - a);
    acc.a   = a + acc.a * (1.0 - a);
    if (tile == TILE_DROP)
    {
        snBend  = snBend * (1.0 - a) + nuv * a;
        snBendA = snBendA * (1.0 - a) + a;
    }
}

// where in a sprite the pixel is: P the pixel and C the centre of the sprite, in the units of its grid (the size of the sprite
// in them), Aspect the cells as they lie on the glass; false outside the sprite
bool snLocal(vec2 p, vec2 c, float size, vec2 aspect, vec2 rot, float flip, float aniso, out vec2 local)
{
    vec2 d = (p - c) * aspect;
    vec2 r = vec2(rot.x * d.x + rot.y * d.y, -rot.y * d.x + rot.x * d.y);
    r.x *= flip / aniso;
    local = r / size + 0.5;
    return all(greaterThanEqual(local, vec2(0.0))) && all(lessThanEqual(local, vec2(1.0)));
}

float snLod(float pixelUV, float sizeUV)
{
    float tilePx = float(textureSize(raindropsatlas, 0).x) * snTileSize();
    return clamp(log2(max(pixelUV / sizeUV * tilePx, 1.0)), 0.0, 6.0);
}

// how wet the snow of a state of the map is (0 dry .. 1 slush)
float snWetness(vec4 q)
{
    return q.y / (q.y + q.x * mix(SNOW_FRESH, SNOW_PACKED, q.z) + 1e-4);
}

// the tone of the snow: grey and darker when wet (slush), a little grey when packed
vec3 snTone(vec4 q, float random)
{
    vec3 tone = vec3(mix(0.94, 1.04, random));
    tone *= mix(vec3(1.0), vec3(0.95, 0.96, 0.98), q.z);
    return tone * mix(vec3(1.0), vec3(0.78, 0.83, 0.88), smoothstep(0.3, 0.8, snWetness(q)));
}

// the flakes: a sprite on every texel holding snow (a drop where only its water is left)
void snFlakes(vec2 aspect, float pixelUV, inout sprite_acc acc)
{
    vec2 p = f_coord * float(WATER_MAP);
    ivec2 here = ivec2(floor(p));
    for (int oy = -1; oy <= 1; ++oy)
    {
        for (int ox = -1; ox <= 1; ++ox)
        {
            ivec2 texel = here + ivec2(ox, oy);
            vec4 q = water_snow_texel(uint(snSlot), snMap, texel);
            bool drop = (q.x < 0.05) && (q.y > 0.03);
            if ((q.x < 0.05) && (false == drop))
                continue;
            uint seed = Hash(uvec3(uvec2(texel), uint(q.w * 65535.0) + 7u * uint(snSlot)));
            // the water of a texel is a drop only now and then: the more water the more of them
            if (drop && (RandF(seed) > smoothstep(0.03, 0.2, q.y)))
                continue;
            vec2 centre = vec2(texel) + 0.2 + 0.6 * vec2(RandF(seed), RandF(seed));
            float ang = RandF(seed) * 6.2831853;
            vec2 rot = vec2(cos(ang), sin(ang));
            float flip = (RandF(seed) < 0.5) ? -1.0 : 1.0;
            float kind = RandF(seed);
            float random = RandF(seed);
            float size = snow_flake * mix(0.75, 1.0, RandF(seed))
                       * (drop ? mix(0.5, 1.0, smoothstep(0.03, 0.3, q.y)) : mix(0.8, 1.6, smoothstep(0.05, 1.5, q.x)));
            vec2 local;
            if (false == snLocal(p, centre, size, aspect, rot, flip, 1.0, local))
                continue;
            int tile;
            float alpha;
            if (drop)
            {
                tile = TILE_DROP;
                alpha = smoothstep(0.03, 0.1, q.y);
            }
            else
            {
                if (snWetness(q) > 0.35)
                    tile = 8 + int(kind * 2.0);                    // wet lumps
                else if (kind < 0.25)
                    tile = int(kind * 8.0);                        // star crystals (0, 1)
                else if (kind < 0.35)
                    tile = 2;                                      // a plate
                else if (kind < 0.45)
                    tile = 3;                                      // needles
                else if (kind < 0.80)
                    tile = 4 + int((kind - 0.45) / 0.35 * 2.0);    // flakes of a few crystals
                else
                    tile = 6 + int((kind - 0.80) / 0.2 * 2.0);     // crumbs
                tile = min(tile, 9);
                alpha = smoothstep(0.05, 0.25, q.x) * mix(snow_thin, 1.0, smoothstep(0.3, 2.5, q.x));
            }
            snSprite(local, tile, alpha, snTone(q, random), rot, flip, snLod(pixelUV, size / float(WATER_MAP)), 1.0, acc);
        }
    }
}

// the layer: a bigger sprite every few texels where the snow is thicker, clumps and then packed snow, overlapping into a cover
void snLayer(vec2 aspect, float pixelUV, inout sprite_acc acc)
{
    float cells = float(WATER_MAP) / snow_layer_cell;
    vec2 p = f_coord * cells;
    ivec2 here = ivec2(floor(p));
    for (int oy = -1; oy <= 1; ++oy)
    {
        for (int ox = -1; ox <= 1; ++ox)
        {
            ivec2 cell = here + ivec2(ox, oy);
            uint seed = Hash(uvec3(uvec2(cell + 4096), 31u + 7u * uint(snSlot)));
            vec2 centre = vec2(cell) + 0.3 + 0.4 * vec2(RandF(seed), RandF(seed));
            vec4 q = water_snow_at(uint(snSlot), snMap, centre / cells);
            float present = smoothstep(snow_layer, snow_layer * 2.5, q.x);
            if (RandF(seed) >= present)
                continue;
            float ang = RandF(seed) * 6.2831853;
            vec2 rot = vec2(cos(ang), sin(ang));
            float flip = (RandF(seed) < 0.5) ? -1.0 : 1.0;
            float kind = RandF(seed);
            float random = RandF(seed);
            float aniso = mix(0.8, 1.0, RandF(seed));
            float size = mix(1.25, 2.0, smoothstep(snow_layer, 4.0, q.x)) * mix(0.85, 1.0, RandF(seed));
            vec2 local;
            if (false == snLocal(p, centre, size, aspect, rot, flip, aniso, local))
                continue;
            int tile;
            if (snWetness(q) > 0.45)
                tile = 14;                                 // slush
            else if (q.x < 3.0)
                tile = 10 + int(kind * 2.0);               // clumps
            else
                tile = 12 + int(kind * 2.0);               // packed snow
            float thick = 1.0 - exp(-q.x / (snow_opaque * mix(1.0, 0.5, q.z)));
            float alpha = mix(0.6, 1.0, thick);
            // the ridges of the blades: packed and thick, a stronger relief
            float bump = mix(1.0, 1.6, q.z * smoothstep(3.0, 12.0, q.x));
            snSprite(local, min(tile, 14), alpha, snTone(q, random), rot, flip, snLod(pixelUV, size / cells), bump, acc);
        }
    }
}

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
    float pixelUV  = max(length(duv1), length(duv2));
    snBig = (textureSize(raindropsatlas, 0).x >= 2048);

    sprite_acc layer = sprite_acc(vec3(0.0), 0.0, vec3(0.0), 0.0);
    sprite_acc flakes = sprite_acc(vec3(0.0), 0.0, vec3(0.0), 0.0);
    float wet = 0.0;
    snSlot = int(param[2].z + 0.5) - 1;
    if ((snSlot >= 0) && (snSlot < WATER_SLOTS) && water_snow_map(uint(snSlot), snMap))
    {
        // the texels of the map as they lie on the glass (the uv may be stretched)
        ivec2 texel = clamp(ivec2(floor(f_coord * float(WATER_MAP))), ivec2(0), ivec2(WATER_MAP - 1));
        uint axis = (uint(snSlot) * uint(WATER_MAP * WATER_MAP) + uint(texel.y * WATER_MAP + texel.x)) * 2u;
        vec2 span = vec2(length(water_axes[axis].xyz), length(water_axes[axis + 1u].xyz));
        vec2 aspect = (min(span.x, span.y) > 1e-6) ? span / max(span.x, span.y) : vec2(1.0);
        snLayer(aspect, pixelUV, layer);
        snFlakes(aspect, pixelUV, flakes);
        vec4 q = water_snow_at(uint(snSlot), snMap, f_coord);
        wet = smoothstep(0.05, 0.3, q.y) * (1.0 - smoothstep(0.5, 2.0, q.x));
        // the picture behind thin flakes and a wet film is blurred
        snHaze = max(smoothstep(0.0, 0.3, flakes.a) * (1.0 - smoothstep(0.5, 0.9, layer.a)), wet * snow_wet_blur);
    }
    // the flakes over the layer
    vec3  snowRGB = flakes.rgb + layer.rgb * (1.0 - flakes.a);
    float snowA   = flakes.a + layer.a * (1.0 - flakes.a);
    vec3  snowN   = flakes.n + layer.n * (1.0 - flakes.a);
    float snowTr  = flakes.tr + layer.tr * (1.0 - flakes.a);

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
        if ((snBendA > 0.25) && (layer.a < 0.3) && (glassA < 0.5))
        {
            vec2 bend = clamp(-snBend / max(snBendA, 1e-3) * snow_drop_bend, vec2(-1.0), vec2(1.0)) * smoothstep(0.25, 0.6, snBendA);
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
