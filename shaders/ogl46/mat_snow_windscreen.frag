#version 460 core
#define STAGE_FRAGMENT 1
// Windscreen snow: replaces rain_windscreen when it snows - v2 (OpenGL 4.6 / Vulkan 1.4 port of the OpenGL 3.3 v1)
//   v2: the syntax of the 4.6 renderers (explicit locations and bindings, #pragma texture / param), the tuning values as constants
//       (a uniform can't have a default any more); the blur of the scene and the picture bent by thin snow go to the
//       post-processing through the velocity target, as the haze and the drops of rain_windscreen (postfx_taa.frag /
//       postfx_motionblur.frag with rain_windscreen_haze.glsl); the colour is written straight
//       (not premultiplied) as rain_windscreen of the 4.6 renderers writes the glass; the gloss of the glass from legacy_gloss();
//       no lighting where the glass is clear (no snow, a clear texture). the look and the behaviour of the snow are those of v1.
//
// MaSzyna switches the weather itself (simulationenvironment.cpp): with Overcast > 1 it rains when the air is warmer
// than 1 deg C and it SNOWS at 1 deg C and below, and the .mat block "snow:" is picked. rain_params.x is the
// precipitation intensity (saturate(Overcast - 1)), exactly what rain_windscreen uses for the drops, so the snow
// on the glass follows the snowfall: nothing without snow, and the harder it snows the more the glass gets plastered.
//
//  - flakes: every cell of the glass can hold a flake (atlas tiles 0..2: fluffy, wet lump, crumbs). It lands at a random
//    moment, stays white for `snow_hold` seconds and then MELTS for `snow_melt` seconds: the snow fades into a water spot
//    (tile 3). About `snow_slide_share` of the melting flakes also SLIDE DOWN the glass, stretching into a drip.
//    Only the wet, melting spot blurs the scene (water refracts; dry snow does not, see snow_fresh_blur).
//  - clumping: flakes land small and GROW as more snow piles onto them. Where flakes land next to each other they run together
//    into one bigger patch (a crusty white film), and the more it snows the more flakes land side by side, so the bigger
//    the patches get. The patch exists only where flakes really sit close together; when they melt / slide away it breaks
//    up and thins out (snow_clump).
//  - wipers (wipermask, GetMixFactor) clear all of it; it builds up again after `snow_regen` seconds (faster in heavy snow).
// Nothing depends on the time since the game started.
//
// Atlas / normalmap (blender_snow_atlas.py), 2 x 2 tiles:
//   0 dry fluffy flake   1 wet compact lump   2 crumbs   3 melted spot (water + ice remnants)
//
// .mat:
//
//   texture_diffuse: dynamic/pkp/vectron_v1/cab/szyby
//   texture_raindropsatlas: textures/fx/raindrops-atlas
//   texture_normalmap: textures/fx/raindrops_normalmap
//   texture_wipermask: dynamic/pkp/vectron_v1/cab/szyby_wipermask
//   shader: rain_windscreen
//   param_raindrop_grid_size: 150
//
//   snow: {
//   shader: snow_windscreen
//   texture_raindropsatlas: textures/fx/snow-atlas
//   texture_normalmap: textures/fx/snow_normalmap
//   param_raindrop_grid_size: 70
//   }
//
// One cell is about one flake: a grid of ~50-90 gives flakes of ~1-2 cm on a ~1.5 m windscreen.

layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;
layout(location = 4) in vec4 f_clip_pos;
layout(location = 5) in vec4 f_clip_future_pos;
#include "common.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

#pragma texture(diffuse,         0, sRGB_A)
#pragma texture(raindropsatlas,  1, sRGB_A)
#pragma texture(wipermask,       2, sRGB_A)
#pragma texture(normalmap,       3, RGBA)    // flakes: rg = normal xy, b = height, a = mask; same 2x2 layout as raindropsatlas (linear)
#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, specular)
#pragma param(reflection, 1, 2, 1, zero)
#pragma param(glossiness, 1, 3, 1, glossiness)
#pragma param(raindrop_grid_size, 2, 0, 1, one)
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

// --- Tuning -----------------------------------------------------------------
// Intensity = rain_params.x = saturate(Overcast - 1). The slider range 1.00..2.00 (Overcast) maps to intensity 0..1. The share of flake cells is pow(intensity, 0.8) * snow_gain * snow_density,
// so with the defaults every cell carries a flake from Overcast ~ 1.2 and the film closes the glass at about 1.3 (1.1 = light snow, 1.5+ = blizzard).
const float snow_gain = 1.4;    // how fast the glass gets covered when the snowfall gets heavier (higher = already plastered in light snow)
const float snow_density = 0.95;   // upper limit of the share of cells that carry a big flake
const float snow_small = 0.80;   // same for the layer of small flakes (0 = off)
const float snow_pack = 0.90;   // opacity of the patches that form where flakes clump together (0 = no patches)
const float snow_clump = 2.60;   // how readily neighbouring flakes run together into one patch (higher = patches form sooner and get bigger)
const float snow_clump_share = 0.55;   // share of the flakes that run together into patches; the rest stay separate flakes
const float snow_hold = 40.0;   // seconds a flake stays white before it starts to melt (longer in heavy snow)
const float snow_melt = 30.0;   // seconds the melting takes (snow fades into a water spot)
const float snow_slide_share = 0.40;   // share of the melting flakes that run down the glass
const float snow_slide_len = 2.0;    // how far a sliding flake runs, in cells (max 2.4)
const float snow_grow_fast = 5.3;    // seconds a patch needs to grow from a clump to a big sheet in the heaviest snowfall
const float snow_grow_slow = 46.7;  // same in the lightest snowfall (it still gets there, just much later)
const float snow_haze = 1.0;    // blur of the scene behind and around the snow patches (frosted glass), 0 = off, 1 = full (radius: windscreen_view.glsl)
const float snow_distort = 2.0;    // picture distortion seen through thin snow (refraction by its relief), 0 = off
const float snow_flake_scale = 2.5;   // size of the flakes from the atlas relative to one grid cell (2-3 = two to three times bigger than before; the flakes get fewer)
const float snow_regen = 25.0;   // seconds until snow is back on a wiped area (in light snowfall)
const float snow_opacity = 1.0;    // overall opacity
const float snow_uv_aspect = 1.0;    // glass width / height in the world if the UV is stretched
const float snow_bump = 1.0;    // strength of the 3D relief from the normal map, 0 = flat
const float snow_gloss = 0.45;   // glints of the sun / headlights on the snow, 0 = matt
const float snow_translucency = 0.8;   // glow of thin snow when you look towards the sun / a lamp, 0 = off
const float snow_melt_blur = 0.8;    // blur of the scene around the water spots of melting flakes (0 = off, 1 = full)
const float snow_fresh_blur = 0.0;    // blur around fresh dry flakes: physically there is none, a little (0.1-0.2) fakes light scattering

#ifndef SNOW_FLIP_Y
#define SNOW_FLIP_Y 0                    // set to 1 if sliding flakes run UP the glass (the glass UV then has v growing downwards)
#endif
const float SNOW_SIZE_MIN  = 0.70;   // flake size in grid cells (keep max <= 1.8, the search covers 3x3 cells)
const float SNOW_SIZE_MAX  = 1.50;
const float SNOW_SLIDER_MAX = 1.10;  // a flake that slides is smaller than this (it gets stretched on its way down)
const float SNOW_HALO_SCALE = 1.5;   // blur ring radius = flake half-size * this
const float SNOW_LAND_TIME = 1.5;    // seconds a flake takes to land and settle
const float SNOW_REFR_MAXPX = 96.0;  // the largest picture shift of thin snow, screen pixels
const float WS_REFR_MAXPX  = 96.0;    // the largest picture shift the post-processing makes, screen pixels; must match rain_windscreen_haze.glsl
const float WS_BEND_ENC   = 60.0;    // the bend request in the velocity target; must match rain_windscreen_haze.glsl
const float WS_DROP_BIAS  = 192.0;
const float WS_ENC        = 64.0;    // the haze request in the velocity target; must match rain_windscreen_haze.glsl
const float SNOW_REFR_MIX   = 0.85;  // 0 = no refraction, 1 = only the bent picture

#ifndef SNOW_MATCH_RAIN_GLASS
#define SNOW_MATCH_RAIN_GLASS 0   // 4.6: rain_windscreen writes the glass straight, not premultiplied
#endif

float snHaze = 0.0;          // set by snowFilm: 1 inside / next to a snow patch (drives the frosted-glass blur)
float snWipeSince = 1.0e5;   // set by GetMixFactor: seconds since the wiper last cleaned this spot
float GetMixFactor(in vec2 co, out float side);

const float SNOW_MERGE_R = 2.0;      // reach of a flake's 'snow mass' in flake half-sizes (flakes closer than this run together)
const float SNOW_CLUMP_THR = 0.80;   // summed mass above which the glass is covered

float snHash(vec2 p)
{
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

float snNoise(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(snHash(i), snHash(i + vec2(1.0, 0.0)), f.x),
               mix(snHash(i + vec2(0.0, 1.0)), snHash(i + vec2(1.0, 1.0)), f.x), f.y);
}

float snFbm(vec2 p)
{
    float a = 0.5;
    float v = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        v += a * snNoise(p);
        p = p * 2.07 + vec2(17.3, 9.1);
        a *= 0.5;
    }
    return v / 0.9375;
}

// One flake in one cell. `age` = seconds since it landed.
void snowFlake(vec2 ncell, uint seed, float age, float gridSize, float pixelUV, vec2 aspect,
               float sizeMul, float opacityMul, float hold, float meltT, float slideShare,
               inout vec3 accRGB, inout float accA, inout float smear,
               inout vec3 accN, inout float accTr, inout float accM)
{
    float mt    = clamp((age - hold) / meltT, 0.0, 1.0);         // 0 = dry, 1 = melted away
    float melt  = smoothstep(0.0, 1.0, mt);
    float fade  = 1.0 - smoothstep(0.75, 1.0, mt);               // the water spot dries up at the very end
    float land  = smoothstep(0.0, 0.3, age);                     // it lands at once (no ghostly fade-in): it just starts small

    float jx = RandF(seed);
    float jy = RandF(seed);
    vec2 center = (ncell + vec2(jx, jy)) / gridSize;

    float rs      = RandF(seed);
    float ang     = RandF(seed) * 6.2831853;
    float flipX   = (RandF(seed) < 0.5) ? -1.0 : 1.0;
    float kind    = RandF(seed);
    float tone    = mix(0.94, 1.04, RandF(seed));
    bool  slider  = RandF(seed) < slideShare;
    float slideLen = mix(0.8, 1.0, RandF(seed)) * min(snow_slide_len, 2.4) / gridSize;
    bool  clumper  = RandF(seed) < snow_clump_share;            // this flake sticks to its neighbours (else it stays on its own)
    float aniso    = mix(0.62, 1.0, RandF(seed));               // flakes are never round: random squash after rotation

    float szMax  = slider ? SNOW_SLIDER_MAX : SNOW_SIZE_MAX;
    float size   = mix(SNOW_SIZE_MIN, szMax, rs * rs) / gridSize * sizeMul * mix(0.45, 1.0, smoothstep(0.0, max(hold * 0.25, 1.0), age)) * mix(1.0, 0.85, melt);   // grows as snow piles onto it

    // a sliding flake runs down while it melts and stretches into a drip
    float slideP = slider ? smoothstep(0.05, 1.0, mt) : 0.0;
#if SNOW_FLIP_Y
    center.y += slideLen * slideP * slideP;
#else
    center.y -= slideLen * slideP * slideP;
#endif
    float stretchY = 1.0 + 0.9 * slideP;

    vec2 d = (f_coord - center) * aspect;
    d.y /= stretchY;

    // the 'snow mass' of this flake: neighbouring flakes add up, and where the sum is high they have run into one patch
    {
        float kr = min(size * 0.5 * SNOW_MERGE_R, 1.0 / gridSize);       // (1 cell: the search only covers the 3x3 cells around the pixel)
        float m  = 1.0 - smoothstep(0.0, kr, length(d));
        if (clumper)
            accM += m * m * (3.0 - 2.0 * m) * (1.0 - 0.6 * melt) * snow_clump * land;
    }

    // blur only around the water of a melting flake (and, optionally, a little around fresh ones)
    float haloR = size * 0.5 * SNOW_HALO_SCALE;
    float dist  = length(d);
    if (dist < haloR)
    {
        float ring = 1.0 - smoothstep(0.35 * haloR, haloR, dist);
        smear = max(smear, ring * land * (melt * snow_melt_blur + (1.0 - melt) * snow_fresh_blur));
    }

    float c = cos(ang);
    float s = sin(ang);
    vec2 r  = vec2(c * d.x + s * d.y, -s * d.x + c * d.y);
    r.x *= flipX / aniso;

    vec2 local = r / size + 0.5;
    if (any(lessThan(local, vec2(0.0))) || any(greaterThan(local, vec2(1.0))))
        return;

    float lod = clamp(log2(max(pixelUV / size * 0.5 * float(textureSize(raindropsatlas, 0).x), 1.0)), 0.0, 4.0);

    // dry flake (tiles 0..2: fluffy, wet lump, crumbs) and, while melting, its water spot (tile 3)
    vec2  off0 = (kind < 0.40) ? vec2(0.0, 0.5) : ((kind < 0.72) ? vec2(0.5, 0.5) : vec2(0.0, 0.0));
    vec2  off3 = vec2(0.5, 0.0);
    vec4  t0 = textureLod(raindropsatlas, off0 + local * 0.5, lod);
    vec4  n0 = textureLod(normalmap,      off0 + local * 0.5, lod);
    float a0 = t0.a * (1.0 - melt);
    float a3 = 0.0;
    vec4  t3 = vec4(0.0);
    vec4  n3 = vec4(0.5, 0.5, 0.0, 0.0);
    if (melt > 0.001)
    {
        t3 = textureLod(raindropsatlas, off3 + local * 0.5, lod);
        n3 = textureLod(normalmap,      off3 + local * 0.5, lod);
        a3 = t3.a * smoothstep(0.0, 0.5, melt) * fade;
    }

    float aSum = a0 + a3;
    float a = clamp(aSum * land * snow_opacity * opacityMul, 0.0, 1.0);
    if (a <= 0.0)
        return;
    vec3 col = (t0.rgb * a0 + t3.rgb * a3) / max(aSum, 1e-4);
    vec2 nxy = ((n0.rg * 2.0 - 1.0) * a0 + (n3.rg * 2.0 - 1.0) * a3) / max(aSum, 1e-4);
    nxy *= snow_bump * land;
    nxy.x *= flipX;
    vec2 nuv = vec2(c * nxy.x - s * nxy.y, s * nxy.x + c * nxy.y);
    vec3 nT  = vec3(nuv, sqrt(max(1.0 - dot(nuv, nuv), 0.02)));

    // thin snow lets light through: the dry part is translucent, the water spot is not
    float tr = (a0 / max(aSum, 1e-4)) * (1.0 - t0.a * 0.55);

    accRGB = col * tone * a + accRGB * (1.0 - a);
    accN   = nT * a + accN * (1.0 - a);
    accTr  = tr * a + accTr * (1.0 - a);
    accA   = a + accA * (1.0 - a);
}

// Adds one layer of flakes to the premultiplied accumulators. Same scheme as the drops of rain_windscreen:
// the cells around the pixel (a few more above it, for the flakes that slide down), the wipers decide per cell whether it
// is clear, the precipitation intensity decides whether the cell carries a flake right now.
void snowLayer(float gridSize, float density, uint layerSeed, float sizeMul, float opacityMul, float slideShare, int rowsUp,
               float pixelUV, vec2 aspect,
               inout vec3 accRGB, inout float accA, inout float smear,
               inout vec3 accN, inout float accTr, inout float accM)
{
    vec2  cell  = floor(f_coord * gridSize);
    float heavy = clamp(rain_params.x, 0.0, 1.0);
    float hold  = snow_hold * (1.0 + 1.5 * heavy);              // in heavy snow there is no time to melt
    float meltT = max(snow_melt, 1.0);
    float life  = hold + meltT;
    float occupancy = mix(0.70, 0.97, heavy);                   // share of the time a cell holds a flake
    float cycBase   = life / occupancy;
    float share     = clamp(pow(rain_params.x, 0.8) * snow_gain, 0.0, 1.0) * density;

    // flakes slide upwards in UV if SNOW_FLIP_Y: search the rows on the other side
#if SNOW_FLIP_Y
    int oyMin = -1 - rowsUp;
    int oyMax = 1;
#else
    int oyMin = -1;
    int oyMax = 1 + rowsUp;
#endif

    for (int oy = oyMin; oy <= oyMax; oy++) {
        for (int ox = -1; ox <= 1; ox++) {

            vec2 ncell   = cell + vec2(float(ox), float(oy));
            uvec2 ucell  = uvec2(ivec2(ncell));

            float side;
            float mixFactor = GetMixFactor((ncell + 0.5) / gridSize, side);

            // timing of this cell: the same in every round
            uint  tseed = Hash(uvec3(ucell, layerSeed + 31u));
            float cyc   = cycBase * (1.0 + mix(0.45, 0.03, heavy) * RandF(tseed));   // in heavy snow the cells hardly ever stand empty
            float tOff  = RandF(tseed) * cyc;
            float patchMul = mix(0.7, 1.3, RandF(tseed));
            float tl    = time + tOff;
            float rnd   = floor(tl / cyc);
            float age   = tl - rnd * cyc;                        // seconds since this round's flake landed
            if (age > life)
                continue;

            // a new flake (position, shape, size) every round; the wiper stroke is part of the seed, as in the rain
            uint seed = Hash(uvec3(ucell, layerSeed + uint(side) * 7919u + uint(max(rnd, 0.0)) * 2654435u));

            if (mixFactor < RandF(seed))
                continue;                                          // wiped / still regrowing

            float activate = RandF(seed);
            if (activate > share * patchMul)
                continue;

            snowFlake(ncell, seed, age, gridSize, pixelUV, aspect, sizeMul, opacityMul, hold, meltT, slideShare,
                      accRGB, accA, smear, accN, accTr, accM);
        }
    }
}

// Snow patches: a crusty white sheet with the structure of real snow (lumps, grain, thin translucent edges, holes).
// WHERE it grows is decided by the flakes: the 'snow mass' of the flakes that stick together (accM) pushes the sheet up, so a patch starts
// where several flakes landed close to each other, spreads from there in an irregular shape and keeps growing while more flakes land
// next to it; when they melt or slide away it breaks up into holes. It is drawn UNDER the flakes.
// Snow patches built from 'pach pach pach' accretion: every big cell carries a patch that is a heap of 22 small clumps (about flake sized).
// The clumps pop up one after another (the ones near the middle first, so the patch spreads outwards) and each lands at once, as a ragged
// piece that merges with the pieces already there. How fast that goes depends on the snowfall intensity (growT); left alone the pieces
// overlap into a closed sheet. Regions of the glass start at different moments. Returns f > 0 inside the snow (f = depth in sheet cells).
float snowSheetField(float gridSize)
{
    float sg    = gridSize * 0.19;
    vec2  pg    = f_coord * sg;
    vec2  cell  = floor(pg);
    float heavy = clamp(rain_params.x, 0.0, 1.0);
    float growT = mix(snow_grow_slow, snow_grow_fast, pow(heavy, 0.5));
    float hold  = snow_hold * (1.0 + 1.5 * heavy);
    float meltT = max(snow_melt, 1.0);
    float life  = growT + hold + meltT;
    float occupancy = mix(0.80, 0.97, heavy);
    float cycBase   = life / occupancy;
    float share = clamp(heavy * 1.6, 0.0, 1.0) * 0.95 + 0.05 * heavy;    // share of the cells that carry a patch (1.0 = none, ~1.6 = nearly all)

    // wiper: pixel precise. The sheet at a pixel is no older than the last wipe of that pixel.
    float sidePx;
    float mixPx = GetMixFactor(f_coord, sidePx);
    float wipeAgePx = snWipeSince;

    float acc = 0.0;                                             // summed clump kernels (metaballs): touching clumps merge, no scalloped circles
    for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {
            vec2  ncell  = cell + vec2(float(ox), float(oy));
            uvec2 ucell  = uvec2(ivec2(ncell));

            uint  tseed = Hash(uvec3(ucell, 211u));
            float cyc   = cycBase * (1.0 + mix(0.45, 0.03, heavy) * RandF(tseed));
            float tOff  = RandF(tseed) * cyc;
            float tl    = time + tOff;
            float rnd   = floor(tl / cyc);
            float age   = tl - rnd * cyc;
            if (age > life)
                continue;

            uint seed = Hash(uvec3(ucell, 223u + uint(sidePx) * 7919u + uint(max(rnd, 0.0)) * 2654435u));
            if (RandF(seed) > share)
                continue;

            vec2  c  = ncell + vec2(0.1 + 0.8 * RandF(seed), 0.1 + 0.8 * RandF(seed));
            vec2  dc = pg - c;
            if (dot(dc, dc) > 1.2 * 1.2)
                continue;                                         // too far from this patch to be touched by any of its clumps

            age = min(age, wipeAgePx);
            float delay  = snNoise(c * 0.30 + 3.7) * growT * 0.35;   // regions of the glass start at different moments
            float ga     = max(age - delay, 0.0);
            float mt     = clamp((age - growT - hold) / meltT, 0.0, 1.0);
            float shrink = 1.0 - 1.1 * mt * mt;                  // melting: the clumps shrink and holes open

            for (int i = 0; i < 22; i++) {
                uint  bs  = Hash(uvec3(seed, uint(i), 977u));
                float rho = RandF(bs);
                float an  = RandF(bs) * 6.2831853;
                float rad = 0.85 * sqrt(rho);
                vec2  bp  = c + rad * vec2(cos(an), sin(an));
                float rb  = mix(0.17, 0.30, RandF(bs));
                float ti  = growT * (0.03 + 0.97 * pow(rad / 0.85, 0.9) * mix(0.70, 1.0, RandF(bs)));   // inner clumps first
                // each clump arrives in three small portions (pop, pop, pop): 45% -> 72% -> 100% of its size, no fading
                float dT  = growT * 0.10;
                float stg = step(ti, ga) + step(ti + dT, ga) + step(ti + 2.0 * dT, ga);   // 0..3
                if (stg <= 0.0)
                    continue;
                float rr = rb * shrink * (stg < 1.5 ? 0.45 : (stg < 2.5 ? 0.72 : 1.0));
                float kd = max(1.0 - length(pg - bp) / max(rr * 1.35, 1e-3), 0.0);
                acc += kd * kd * (3.0 - 2.0 * kd);
            }
        }
    }
    // torn, crumbly edge: fine noise (features about a flake wide) decides where the summed clumps stop, so the outline is irregular
    // snow and not a ring of circles
    float nz = snFbm(pg * 5.5 + 13.0) * 0.45 + snFbm(pg * 13.0 + 7.0) * 0.35 + snFbm(pg * 31.0 + 3.0) * 0.20 - 0.5;
    float best = (acc - 0.50) * 0.20 + nz * 0.30;
    best -= (1.0 - smoothstep(0.0, 0.12, mixPx)) * 1.0;          // swept (or still regrowing) areas
    return best;
}

void snowFilm(float gridSize, inout vec3 accRGB, inout float accA, inout vec3 accN, inout float accTr)
{
#if SNOW_FLIP_Y
    float dir = -1.0;
#else
    float dir = 1.0;
#endif
    float sf = snowSheetField(gridSize);
    snHaze = max(snHaze, smoothstep(-0.045, 0.0, sf));             // the glass next to a patch is frosted a little as well
    float cover = smoothstep(0.0, 0.03, sf);
    if (cover <= 0.002)
        return;
    float thick = smoothstep(0.0, 0.11, sf);                       // 0 at the thin edge, 1 in the thick middle
    vec2  p   = f_coord * gridSize * 0.17 + vec2(0.0, dir * time * 0.008);
    vec2  q   = f_coord * gridSize * 2.6;
    float h0  = snFbm(p);

    // lumpy surface: big drifts + fine grain
    float f1  = snFbm(q);
    float e   = 0.06;
    float ef  = 0.05;
    vec2  g   = vec2(snFbm(p + vec2(e, 0.0)) - h0, snFbm(p + vec2(0.0, e)) - h0) / e;
    vec2  gf  = vec2(snFbm(q + vec2(ef, 0.0)) - f1, snFbm(q + vec2(0.0, ef)) - f1) / ef;
    vec3  nT  = normalize(vec3(-(g * 0.55 + gf * 0.20) * snow_bump, 1.0));

    vec3  thin = vec3(0.66, 0.72, 0.80);                         // thin snow is greyish-blue and lets the dark background through
    vec3  full = vec3(0.93, 0.95, 0.98);
    vec3  col  = mix(thin, full, thick) * (0.62 + 0.55 * f1) * (0.88 + 0.22 * snNoise(f_coord * gridSize * 9.0)) * (0.85 + 0.3 * h0);
    float a    = clamp(cover * mix(0.50, 0.96, thick) * snow_pack * snow_opacity, 0.0, 0.97);
    float under = 1.0 - accA;
    accRGB = accRGB + col * a * under;
    accN   = accN + nT * a * under;
    accTr  = accTr + mix(0.60, 0.30, thick) * a * under;
    accA   = accA + a * under;
}

// --- The snow of the simulation (water_snow.comp): the map of the glass ---------------------------------------------------
// When the renderer simulates the snow of the glass (the param snow_slot holds 1 + its slot), the snow, the water, how packed the
// snow is and the shape of its flakes come from the map of the glass (water.glsl), which keeps them from frame to frame: the flakes
// land and stay where they landed, pile up into a layer, the blades push it into ridges at the ends of their sweeps and leave a wet
// trace, it melts at the warm glass and slush slides down. Here it's only drawn:
//  - a texel of the map with a little snow is a flake (atlas tiles 0..2: fluffy when dry, a wet lump when wet or packed, crumbs),
//    the more snow the bigger; with mostly water left it's the water spot (tile 3)
//  - where the snow is thicker it's a layer: its edge torn by the grain of the snow, as opaque as it's thick (Beer-Lambert), grey
//    and translucent when wet (slush), smoother and denser when packed; its relief from the slopes of the snow of the map
//  - the water: wet glass bends the picture behind (the post-processing), the thin snow hazes it
// Without the simulation (an older renderer, no compute) the procedural snow below draws the glass as before.
const float snow_map_flake      = 1.15;  // size of a flake relative to a texel of the map (the snow of a texel)
const float snow_map_layer      = 1.0;   // mm of snow above which the flakes run together into a layer
const float snow_map_opaque     = 1.8;   // mm of fresh snow which lets 1/e of the light through (packed: half of it)
const float snow_map_relief     = 0.6;   // how much the slopes of the snow tilt its surface
const float snow_map_grain      = 0.35;  // the fine grain of the layer (its relief)
const float snow_map_wet_bend   = 0.6;   // how strongly a wet glass bends the picture behind it

int   snMapSlot = -1;
uint  snMapIndex = 0u;
float snWet = 0.0;          // set by snowMapped: the film of water (0 .. 1)
vec2  snWetN = vec2(0.0);   // set by snowMapped: the slope of the water, for the bend of the picture

// the snow of the map at a place of the glass
vec4 snMapAt(vec2 uv)
{
    return water_snow_at(uint(snMapSlot), snMapIndex, uv);
}

// a flake of the map drawn over the accumulators: Local the place in its sprite (0 .. 1), Tile the corner of its tile in the atlas
void snowMapSprite(vec2 local, vec2 tile, float alpha, float tone, float flip, float c, float s, float lod,
                   inout vec3 accRGB, inout float accA, inout vec3 accN, inout float accTr)
{
    vec4 t = textureLod(raindropsatlas, tile + local * 0.5, lod);
    vec4 n = textureLod(normalmap,      tile + local * 0.5, lod);
    float a = clamp(t.a * alpha * snow_opacity, 0.0, 1.0);
    if (a <= 0.0)
        return;
    vec2 nxy = (n.rg * 2.0 - 1.0) * snow_bump;
    nxy.x *= flip;
    vec2 nuv = vec2(c * nxy.x - s * nxy.y, s * nxy.x + c * nxy.y);
    vec3 nT  = vec3(nuv, sqrt(max(1.0 - dot(nuv, nuv), 0.02)));
    float tr = (1.0 - t.a * 0.55);
    accRGB = t.rgb * tone * a + accRGB * (1.0 - a);
    accN   = nT * a + accN * (1.0 - a);
    accTr  = tr * a + accTr * (1.0 - a);
    accA   = a + accA * (1.0 - a);
}

// the flakes of the texels around the pixel
void snowMapFlakes(vec2 aspect, float pixelUV, inout vec3 accRGB, inout float accA, inout vec3 accN, inout float accTr)
{
    vec2 p = f_coord * float(WATER_MAP);
    ivec2 here = ivec2(floor(p));
    for (int oy = -1; oy <= 1; ++oy)
    {
        for (int ox = -1; ox <= 1; ++ox)
        {
            ivec2 texel = here + ivec2(ox, oy);
            vec4 q = water_snow_texel(uint(snMapSlot), snMapIndex, texel);
            float density = mix(0.1, 0.4, q.z);
            float wet = q.y / (q.y + q.x * density + 1e-4);
            bool spot = (q.x < 0.05) && (q.y > 0.015);   // a flake melted into its water
            if ((q.x < 0.05) && (false == spot))
                continue;
            uint seed = Hash(uvec3(uvec2(texel), uint(q.w * 65535.0) + 7u * uint(snMapSlot)));
            vec2 center = vec2(texel) + vec2(0.15 + 0.7 * RandF(seed), 0.15 + 0.7 * RandF(seed));
            float ang = RandF(seed) * 6.2831853;
            float flip = (RandF(seed) < 0.5) ? -1.0 : 1.0;
            float kind = RandF(seed);
            float tone = mix(0.94, 1.04, RandF(seed));
            float aniso = mix(0.62, 1.0, RandF(seed));
            // the more snow the texel holds the bigger the flake (a lump of a few flakes)
            float size = snow_map_flake * mix(0.6, 1.0, RandF(seed)) * (spot ? mix(0.5, 1.1, smoothstep(0.015, 0.2, q.y))
                                                                              : mix(0.7, 1.6, smoothstep(0.05, 2.0, q.x)));
            vec2 d = (p - center) * aspect;
            float c = cos(ang);
            float s = sin(ang);
            vec2 r = vec2(c * d.x + s * d.y, -s * d.x + c * d.y);
            r.x *= flip / aniso;
            vec2 local = r / size + 0.5;
            if (any(lessThan(local, vec2(0.0))) || any(greaterThan(local, vec2(1.0))))
                continue;
            float sizeUV = size / float(WATER_MAP);
            float lod = clamp(log2(max(pixelUV / sizeUV * 0.5 * float(textureSize(raindropsatlas, 0).x), 1.0)), 0.0, 4.0);
            vec2 tile;
            if (spot)
                tile = vec2(0.5, 0.0);
            else if ((wet > 0.3) || (q.z > 0.5) || (kind < 0.30))
                tile = vec2(0.5, 0.5);                 // a wet lump
            else if (kind < 0.75)
                tile = vec2(0.0, 0.5);                 // fluffy
            else
                tile = vec2(0.0, 0.0);                 // crumbs
            float alpha = spot ? smoothstep(0.015, 0.08, q.y) : smoothstep(0.05, 0.25, q.x);
            snowMapSprite(local, tile, alpha, tone, flip, c, s, lod, accRGB, accA, accN, accTr);
        }
    }
}

// the snow of the map at the pixel: the flakes, the layer of the snow under them, the water, the haze
void snowMapped(vec2 aspect, float pixelUV, inout vec3 accRGB, inout float accA, inout float smear, inout vec3 accN, inout float accTr)
{
    vec4 q = snMapAt(f_coord);
    float e = 1.0 / float(WATER_MAP);
    vec4 qr = snMapAt(f_coord + vec2(e, 0.0));
    vec4 ql = snMapAt(f_coord - vec2(e, 0.0));
    vec4 qu = snMapAt(f_coord + vec2(0.0, e));
    vec4 qd = snMapAt(f_coord - vec2(0.0, e));
    float around = (q.x + qr.x + ql.x + qu.x + qd.x) * 0.2;

    // the flakes of thin snow and the water spots
    snowMapFlakes(aspect, pixelUV, accRGB, accA, accN, accTr);

    // the water: the film bends the picture behind
    snWet = smoothstep(0.02, 0.25, q.y) * (1.0 - smoothstep(0.5, 2.0, q.x));
    snWetN = vec2(qr.y - ql.y, qu.y - qd.y) * 2.0 + (vec2(snNoise(f_coord * 900.0), snNoise(f_coord * 900.0 + 31.0)) - 0.5) * 0.6 * snWet;
    smear = max(smear, snWet * snow_melt_blur * 0.6);
    // thin snow hazes the picture around it
    snHaze = max(snHaze, smoothstep(0.03, 0.6, around));

    // the layer, where the snow has run together
    vec2  gp    = f_coord * float(WATER_MAP) * aspect;
    float grain = snFbm(gp * 0.9 + 3.0) * 0.55 + snFbm(gp * 2.7 + 11.0) * 0.30 + snFbm(gp * 7.1 + 5.0) * 0.15;
    float edge  = (q.x - snow_map_layer) / max(snow_map_layer, 1e-3) + (grain - 0.5) * 1.6;
    float cover = smoothstep(0.0, 0.35, edge);
    if (cover <= 0.002)
        return;
    float density = mix(0.1, 0.4, q.z);
    float wet     = q.y / (q.y + q.x * density + 1e-4);
    float opaque  = snow_map_opaque * mix(1.0, 0.5, q.z);
    float thick   = 1.0 - exp(-q.x / opaque);
    // the relief: the slopes of the snow of the map (ridges, the edges of the swept arcs) and the grain of the fresh snow
    vec2  slope = vec2(qr.x - ql.x, qu.x - qd.x) * 0.5 * snow_map_relief;
    float f1    = snFbm(gp * 3.3);
    float ef    = 0.05;
    vec2  gf    = vec2(snFbm(gp * 3.3 + vec2(ef, 0.0)) - f1, snFbm(gp * 3.3 + vec2(0.0, ef)) - f1) / ef;
    vec3  nT    = normalize(vec3(-(slope + gf * snow_map_grain * (1.0 - 0.6 * q.z)) * snow_bump, 1.0));

    vec3  fresh  = vec3(0.93, 0.95, 0.98);
    vec3  packed = vec3(0.84, 0.87, 0.91);
    vec3  slush  = vec3(0.60, 0.66, 0.73);
    vec3  col    = mix(mix(fresh, packed, q.z), slush, smoothstep(0.2, 0.7, wet));
    col *= (0.80 + 0.30 * f1 * (1.0 - 0.5 * q.z)) * (0.92 + 0.12 * grain);
    float a      = clamp(cover * mix(0.35, 0.97, thick) * mix(1.0, 0.75, smoothstep(0.3, 0.8, wet)) * snow_pack * snow_opacity, 0.0, 0.97);
    float under  = 1.0 - accA;
    accRGB = accRGB + col * a * under;
    accN   = accN + nT * a * under;
    accTr  = accTr + mix(0.60, 0.25, thick) * a * under;
    accA   = accA + a * under;
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

    float gridSize = max(ceil(param[2].x), 1.0);
    float pixelUV  = max(length(dFdx(f_coord)), length(dFdy(f_coord)));
    snHaze = 0.0;
    vec2  aspect   = (snow_uv_aspect >= 1.0) ? vec2(snow_uv_aspect, 1.0) : vec2(1.0, 1.0 / max(snow_uv_aspect, 0.01));

    vec3  snowRGB = vec3(0.0);   // premultiplied
    float snowA   = 0.0;
    float smear   = 0.0;
    vec3  snowN   = vec3(0.0);
    float snowTr  = 0.0;
    // the snow of the simulation where the glass has a map of it, else made up as before
    snMapSlot = int(param[2].z + 0.5) - 1;
    bool mapped = (snMapSlot >= 0) && (snMapSlot < WATER_SLOTS) && water_snow_map(uint(snMapSlot), snMapIndex);
    if (mapped)
    {
        // the texels of the map as they lie on the glass (the uv may be stretched)
        ivec2 texel = clamp(ivec2(floor(f_coord * float(WATER_MAP))), ivec2(0), ivec2(WATER_MAP - 1));
        uint axis = (uint(snMapSlot) * uint(WATER_MAP * WATER_MAP) + uint(texel.y * WATER_MAP + texel.x)) * 2u;
        vec2 span = vec2(length(water_axes[axis].xyz), length(water_axes[axis + 1u].xyz));
        if (min(span.x, span.y) > 1e-6)
            aspect = span / max(span.x, span.y);
        snowMapped(aspect, pixelUV, snowRGB, snowA, smear, snowN, snowTr);
    }
    else if (rain_params.x > 0.0)
    {
        float mass = 0.0;
        if (snow_small > 0.0)
            snowLayer(gridSize * 2.4 / snow_flake_scale, snow_small, 17u, 0.9, 0.85, 0.0, 0, pixelUV, aspect, snowRGB, snowA, smear, snowN, snowTr, mass);
        snowLayer(gridSize / snow_flake_scale, snow_density, 13u, 1.0, 1.0, snow_slide_share, 4, pixelUV, aspect, snowRGB, snowA, smear, snowN, snowTr, mass);
        if (snow_pack > 0.0)
            snowFilm(gridSize, snowRGB, snowA, snowN, snowTr);              // patches where flakes have run together
    }

    float glassA  = tex_color.a * alpha_mult;
    float finalA  = snowA + glassA * (1.0 - snowA);
    vec3  premult = snowRGB + tex_color.rgb * glassA * (1.0 - snowA);
    vec3  albedo  = premult / max(finalA, 1e-4);
    float glassShare = glassA * (1.0 - snowA) / max(finalA, 1e-4);

    // relief: tilt the glass normal by the slope of the snow (flat where there is none)
    vec3  nDir       = snowN + vec3(0.0, 0.0, 1.0) * (1.0 - snowA);
    vec3  fragnormal = Ngl;
    if (snowA > 0.001)
        fragnormal = normalize(Ngl * max(nDir.z, 0.05) + Tgl * nDir.x + Bgl * nDir.y);

    vec3  fragcolor    = ambient;
    float reflectivity = param[1].z * glassShare;
    float specularity  = (tex_color.r + tex_color.g + tex_color.b) * 0.5 * glassShare + snow_gloss * snowA;
    legacy_gloss();

    // most of the glass is clear (no snow, a clear glass texture): the lighting of the 4.6 renderers (the clusters of lamps, the
    // shadows, the reflections) is skipped there, it would be multiplied by an alpha of ~0
    if (finalA > 0.002)
        fragcolor = apply_lights(fragcolor, fragnormal, albedo, reflectivity, specularity, shadow_tone);

    // thin snow glows when you look towards the sun / a lamp
    if (finalA > 0.002 && snowA > 0.001 && snow_translucency > 0.0 && lights_count > 0U)
    {
        vec3  I = normalize(f_pos.xyz);
        float shadowSun = 0.0;
        if (shadow_tone < 1.0)
            shadowSun = (1.0 - shadow_tone) * clamp(calc_shadow(), 0.0, 1.0);
        float towards = max(dot(I, normalize(-lights[0].dir)), 0.0);
        float glow    = pow(towards, 3.0) * (1.0 - shadowSun);
        fragcolor += lights[0].color * lights[0].intensity * glow * snow_translucency * snowTr * albedo * snowA;
    }
    vec3 lit = apply_fog(fragcolor);

#if SNOW_MATCH_RAIN_GLASS
    vec4 color = vec4(lit * finalA, finalA);
#else
    vec4 color = vec4(lit, finalA);
#endif

    out_color = POSTFX_ENABLED ? color : tonemap(color);

    // the scene seen through the snow, drawn by the post-processing (rain_windscreen_haze.glsl), asked for in the velocity target as
    // rain_windscreen asks: thin snow bends it (its relief refracts; x, y = WS_DROP_BIAS + WS_BEND_ENC * bend), elsewhere it's blurred in
    // and around thin snow and around the melting spots (x = WS_ENC * strength, y = 0); the snow itself stays sharp. alpha 0 elsewhere:
    // the blending keeps the motion of the scene behind
    if (MOTIONBLUR_ENABLED)
    {
        float thin = smoothstep(0.04, 0.20, snowA) * (1.0 - smoothstep(0.55, 0.90, snowA));
        vec4  req  = vec4(0.0);
        if ((snWet > 0.25) && (snowA < 0.35) && (glassA < 0.5))
        {
            // the water on the glass: its film and its spots bend the picture behind
            vec2 bend = clamp(-snWetN * snow_map_wet_bend, vec2(-1.0), vec2(1.0)) * smoothstep(0.25, 0.6, snWet);
            req = vec4(WS_DROP_BIAS + WS_BEND_ENC * bend.x, WS_DROP_BIAS + WS_BEND_ENC * bend.y, 0.0, 1.0);
        }
        else if ((snow_distort > 0.0) && (thin > 0.30) && (glassA < 0.5))
        {
            vec2 slope = snowN.xy / max(snowA, 1e-3);
            vec2 bend  = clamp(-slope * snow_distort, vec2(-1.0), vec2(1.0)) * smoothstep(0.30, 0.60, thin) * (SNOW_REFR_MAXPX / WS_REFR_MAXPX);
            req = vec4(WS_DROP_BIAS + WS_BEND_ENC * bend.x, WS_DROP_BIAS + WS_BEND_ENC * bend.y, 0.0, 1.0);
        }
        else
        {
            float haze     = snHaze * snow_haze * (1.0 - smoothstep(0.30, 0.80, snowA));       // frosted glass in and around thin snow
            float blurMask = clamp(max(smear * (1.0 - snowA), haze) * (1.0 - smoothstep(0.5, 0.95, glassA)), 0.0, 1.0);
            if (blurMask >= 0.01)
                req = vec4(WS_ENC * blurMask, 0.0, 0.0, 1.0);
        }
        out_motion = req;
    }
}

float GetMixFactor(in vec2 co, out float side) {
    vec4 movePhase  = wiper_pos;
    bvec4 is_out = lessThanEqual(movePhase, vec4(1.));
    movePhase = mix(vec4(2.) - movePhase, movePhase, is_out);

    vec4 mask = textureLod(wipermask, co, 0.0);   // called from non-uniform flow: no implicit derivatives
    
    vec4 areaMask = step(.001, mask);

    vec4 maskVal   = mix(1. - mask, mask, is_out);
    vec4 wipeWidth = smoothstep(1., .9, movePhase) * .25;
    vec4 cleaned   = smoothstep(movePhase - wipeWidth, movePhase, maskVal) * areaMask;
    vec4 side_v = step(maskVal, movePhase);
    cleaned *= side_v;
    side_v = mix(side_v, vec4(1.) - side_v, is_out);

    // "regeneration", snow gradually returning after a wiper pass (slower than rain, and faster in heavy snowfall):
    float regenT = max(mix(snow_regen, snow_regen * 0.6, clamp(rain_params.x, 0.0, 1.0)), 0.5);
    vec4 sinceV = vec4(time) - mix(wiper_timer_out, wiper_timer_return, side_v) - vec4(.2);
    vec4 regenPhase = clamp(sinceV / vec4(regenT), vec4(0.), vec4(1.));
    
    side_v = mix(vec4(0.), vec4(1.) - side_v, areaMask);
    
    vec4 factor_v = mix(vec4(1.), regenPhase * (vec4(1.) - cleaned), areaMask);
    
    side = 0.;
    float out_factor = 1.;
    bool  have_side = false;
    snWipeSince = 1.0e5;
    
    // out_factor: the blade that clears the grid cell the most wins (as before).
    // side: which wiper stroke the drops of this cell belong to. It is part of the random
    // seed of every drop, so it must NOT depend on out_factor. It used to: once the drops had
    // fully regrown (out_factor == 1) side fell back to 0 and every drop in the swept area
    // jumped to a new position, about one regrow time after each stroke (most visible in
    // heavy rain, where regrowth takes ~1 s).
    for(int i = 0; i < 4; ++i)
    {
      out_factor = min(out_factor, factor_v[i]);
      if (!have_side && areaMask[i] > 0.)
      {
        side = side_v[i];
        snWipeSince = max(sinceV[i], 0.0);
        have_side = true;
      }
    }
    
    return out_factor;
}
