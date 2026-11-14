#version 460 core
#define STAGE_FRAGMENT 1
// Windscreen bugs (spring / summer) - v3 (OpenGL 4.6 / Vulkan 1.4 port of the OpenGL 3.3 v2)
//   v3: the syntax of the 4.6 renderers (explicit locations and bindings, #pragma texture / param, the settings as constants);
//       the tuning values are constants (a uniform can't have a default any more); the blur ring goes to the post-processing
//       through the velocity target as the haze of rain_windscreen does (postfx_taa.frag / postfx_motionblur.frag with
//       rain_windscreen_haze.glsl); the colour is written straight (not premultiplied), as rain_windscreen of the 4.6 renderers
//       writes the glass; the gloss of the glass from legacy_gloss(); no lighting where the glass is clear (no bug, clear texture)
//   v2: cells without a bug are skipped before the wiper mask is read (the mask used to be read for all
//       18 cells of both layers on every pixel); the fine specks are procedural (no texture reads, only the
//       pixel's own cell); optional vertical bias of the dirt; quick splat instead of a 2 s growth;
//       now and then a really big bug and rarely a huge one (own coarse grids, never cut off by the 3x3 search);
//       smaller blur ring, scaled by the size of the bug on screen.
//
// Insects splat on the glass one by one, only while the vehicle goes faster than 20 km/h and the more the faster, and the glass
// gets dirtier until it is fully dirty after `bugs_fill_time` seconds of driving at 100 km/h (default 3600 = 1 h; at 160 km/h
// about 35 minutes, at 50 km/h 2.7 hours). The engine counts the insects met (rain_wind.w) and keeps the count of the moments
// the blades were at the ends of their sweep (wiper_bugs_out / wiper_bugs_return), next to their times.
// No sliding, no rain: every bug is placed once (hash of its grid cell) and just
// appears at its own random moment. Wipers (wipermask) wipe the bugs off in the area they
// sweep; the glass there then gets dirty again, with new bugs, over the next bugs_fill_time.
//
// .mat (shader name WITHOUT the "mat_" prefix, the engine adds it).
// The sampler is called "raindropsatlas" on purpose, exactly like in rain_windscreen:
// the engine collects ALL texture_* lines of a .mat and checks them against the final
// shader, so a key that only one of the two shaders knows ends in "unknown texture binding".
// With one shared key, a season / weather block just overrides the file:
//
//   texture_diffuse: dynamic/pkp/vectron_v1/cab/szyby
//   texture_wipermask: dynamic/pkp/vectron_v1/cab/szyby_wipermask
//   shader: rain_windscreen
//   texture_raindropsatlas: textures/fx/raindrops-atlas
//   texture_normalmap: textures/fx/raindrops_normalmap
//   param_raindrop_grid_size: 150
//
//   spring: {
//   shader: bugs_windscreen
//   texture_raindropsatlas: textures/fx/bugs-atlas     <- 2x2 atlas (blender_bugs_atlas.py), colour + alpha
//   texture_normalmap: textures/fx/bugs_normalmap      <- same layout: rg = normal xy, b = height, a = mask
//   param_raindrop_grid_size: 60                       <- cells across the glass UV, see below
//   }
//   summer: {
//   shader: bugs_windscreen
//   texture_raindropsatlas: textures/fx/bugs-atlas
//   texture_normalmap: textures/fx/bugs_normalmap
//   param_raindrop_grid_size: 60
//   }
//
//   rain: {                                            <- rain weather beats the season (priority 2 > 1)
//   shader: rain_windscreen
//   texture_raindropsatlas: textures/fx/raindrops-atlas
//   texture_normalmap: textures/fx/raindrops_normalmap
//   param_raindrop_grid_size: 150
//   }
//
// NOTE: the wipers' memory of when the glass was last wiped lives only in the engine's
// wiper timers, and the renderer resets them to -1000 whenever the camera is not in
// a cab (F4 ...). The shader cannot recover what is gone, so after returning to the cab the
// parts of the glass the blades cover are dirty as if never wiped (capped by `bugs_unknown_level`) until they are
// wiped again. A one-line engine fix would keep the memory, see the note in the chat.
//
// One cell is about one bug, so a grid of ~40-80 gives bugs of a few cm on a ~1.5 m
// windscreen (150, as used for rain drops, would make them tiny).

layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;
#include "common.glsl"

layout(location = 0) out vec4 out_color;
layout(location = 1) out vec4 out_motion;

#pragma texture(diffuse,         0, sRGB_A)
#pragma texture(raindropsatlas,  1, sRGB_A)
#pragma texture(wipermask,       2, sRGB_A)
#pragma texture(normalmap,       3, RGBA)    // splats: rg = normal xy, b = height, a = mask; same 2x2 layout as raindropsatlas (linear)
#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, specular)
#pragma param(reflection, 1, 2, 1, zero)
#pragma param(glossiness, 1, 3, 1, glossiness)
#pragma param(raindrop_grid_size, 2, 0, 1, one)

#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"
#include "random.glsl"

layout(binding = 0) uniform sampler2D diffuse;
layout(binding = 1) uniform sampler2D raindropsatlas;
layout(binding = 2) uniform sampler2D wipermask;
layout(binding = 3) uniform sampler2D normalmap;

// --- Tuning -----------------------------------------------------------------
const float bugs_fill_time = 3600.0; // seconds of driving at 100 km/h until the glass is fully dirty (none below 20 km/h)
const float bugs_start_time = 0.0;    // simulation time at which the glass was clean
const float bugs_density = 0.15;   // fraction of grid cells that end up with a big bug (0..1)
const float specks_density = 0.12;   // same for the layer of tiny specks (0 = off)
const float bugs_opacity = 1.0;    // overall opacity of the dirt
const float bugs_uv_aspect = 1.0;    // glass width / height in the world if the UV is stretched (e.g. 2.0 for a 1.4 x 0.7 m pane)
const float bugs_unknown_level = 1.0;  // upper limit of the dirt where the wiper memory was lost: 1 = grows like everywhere else, 0.3 = never more than 30%, 0 = stays clean until a blade passes
const float bugs_wiper_clean = 1.0;    // 1 = wipers wipe the bugs off (uses wipermask), 0 = wipers ignore them
const float bugs_gloss = 0.45;   // shininess of the wet splats (sun / headlight glints on the bumps), 0 = matt
const float bugs_bump = 1.0;    // strength of the 3D relief from the normal map, 0 = flat
const float bugs_halo_blur = 0.8;    // blur of the scene seen through the see-through parts of a bug (its texture half transparent: wings,
                                     // the thin smear of the splat), none around it (0 = off, 1 = full); done in the post-processing
const float bugs_halo_size = 1.15;   // ring radius = bug half-size * this (1 = only under the splat, max 1.6)
const float bugs_halo_px = 60.0;   // a bug this many pixels wide on screen gets the full blur, smaller ones less (the blur radius is fixed in pixels)
const float bugs_vertical = 0.0;    // -1..1: more dirt at the bottom (<0) or at the top (>0) of the pane, 0 = even
const float bugs_giant_density = 0.015; // share of the coarse cells that end up with a really big bug (0 = off)
const float bugs_giant_scale = 4.0;   // how much bigger the big ones are (the coarse grid is this many times coarser)
const float bugs_huge_density = 0.02;  // share of the very coarse cells with a huge bug: about one per pane when fully dirty (0 = off)
const float bugs_huge_scale = 9.0;   // how much bigger the huge ones are than the normal bugs
const float specks_size = 0.30;   // size of the fine specks, in speck cells (0.1 .. 0.45)
const float specks_opacity = 0.55;   // how much a speck covers the glass

const float BUG_SIZE_MIN  = 0.6;    // bug size in grid cells (keep max <= 1.2, the search covers 3x3 cells)
const float BUG_SIZE_MAX  = 1.2;
const float BUG_FADE_TIME = 0.15;   // seconds a splat takes to appear (a quick splat)
const float SPECK_SCALE   = 2.5;    // speck grid = raindrop_grid_size * this
const float WS_ENC        = 64.0;   // must match rain_windscreen_haze.glsl

// Match the way rain_windscreen draws the glass itself (colour premultiplied by
// alpha, then alpha-blended), so the pane looks identical in summer and in rain.
// Set to 1 to see the shader's clock (`time`) as red digits on the glass, see bugs_start_time.
#ifndef BUGS_DEBUG_TIME
#define BUGS_DEBUG_TIME 0
#endif

#ifndef BUGS_MATCH_RAIN_GLASS
#define BUGS_MATCH_RAIN_GLASS 0   // 4.6: rain_windscreen writes the glass straight, not premultiplied
#endif

vec2 tileOffset(float choice)
{
    if      (choice < 0.25) return vec2(0.0, 0.0);
    else if (choice < 0.5)  return vec2(0.5, 0.0);
    else if (choice < 0.75) return vec2(0.0, 0.5);
    return vec2(0.5, 0.5);
}

// Simulation time at which a wiper blade last swept over `co`.
//   WIPER_NONE    no blade covers this spot (it can never be wiped)
//   WIPER_UNKNOWN a blade covers it, but the engine has lost its memory: while the camera is
//                 outside the cab (F4 ...) the renderer resets the stroke timers to -1000
// Same blade/stroke logic as GetMixFactor in rain_windscreen: wipermask channel i is the
// area of blade i, its value says when during the stroke the blade reaches that spot, and
// wiper_pos is 0..1 on the way out and 1..2 on the way back. The timers hold the moment
// each stroke started, which is good enough (a stroke takes about a second).
const float WIPER_NONE    = -1.0e9;
const float WIPER_UNKNOWN = -2.0e9;

float lastCleanTime(vec2 co, out float bugsAt)
{
    vec4  m       = textureLod(wipermask, co, 0.0);
    float best    = WIPER_NONE;
    bugsAt        = 0.0;
    bool  unknown = false;
    for (int i = 0; i < 4; ++i)
    {
        if (m[i] <= 0.001)
            continue;
        float pos     = wiper_pos[i];
        bool  isOut   = pos <= 1.0;
        float phase   = isOut ? pos : 2.0 - pos;
        float maskVal = isOut ? m[i] : 1.0 - m[i];
        bool  passed  = maskVal <= phase;                       // blade is already past this spot
        float sideV   = isOut ? (passed ? 0.0 : 1.0) : (passed ? 1.0 : 0.0);
        float t       = mix(wiper_timer_out[i], wiper_timer_return[i], sideV);
        if (!(t <= time))                                        // garbage or NaN: ignore
            continue;
        float b       = mix(wiper_bugs_out[i], wiper_bugs_return[i], sideV);   // the insects met at that moment
        if ((t < -500.0) || (b < 0.0))                           // the -1000 "memory lost" marker
        {
            unknown = true;
            continue;
        }
        if (t > best)
        {
            best   = t;
            bugsAt = b;
        }
    }
    if (best > WIPER_NONE)
        return best;
    return unknown ? WIPER_UNKNOWN : WIPER_NONE;
}

// Fine specks: tiny dried drops of bug juice. Procedural (a soft irregular blob with a darker rim),
// one per speck cell and always inside it, so only the pixel's own cell is checked: no texture reads
// apart from the wiper state of this pixel, which main() passes in.
void speckLayer(float gridSize, float density, float tcPix, float bugsPix, vec2 aspect, float pixelUV,
                inout vec3 accRGB, inout float accA, inout vec3 accN, inout float accNA)
{
    vec2  g    = f_coord * gridSize;
    vec2  cell = floor(g);
    float tBase, prog;
    float met = max(rain_wind.w, 0.0);                     // the insects met so far
    if (tcPix < -1.5e9)
    {
        tBase = -1000.0;
        prog  = min(met / max(bugs_fill_time, 1.0), bugs_unknown_level);
    }
    else if (tcPix < -0.5e9)
    {
        tBase = bugs_start_time;
        prog  = met / max(bugs_fill_time, 1.0);
    }
    else
    {
        tBase = tcPix;
        prog  = max(met - bugsPix, 0.0) / max(bugs_fill_time, 1.0);
    }
    if (prog <= 0.0)
        return;
    uint  pseed    = Hash(uvec3(uvec2(ivec2(floor(cell / 8.0))), 107u));
    float patchMul = mix(0.4, 1.6, RandF(pseed));
    float vertMul  = clamp(1.0 + bugs_vertical * ((cell.y + 0.5) / gridSize - 0.5) * 2.0, 0.0, 2.0);
    uint  seed     = Hash(uvec3(uvec2(ivec2(cell)), 7u ^ floatBitsToUint(tBase)));
    if (RandF(seed) > density * patchMul * vertMul)
        return;
    float appearAt = RandF(seed);
    if (prog < appearAt)
        return;
    float r   = clamp(specks_size, 0.05, 0.45) * mix(0.5, 1.0, RandF(seed));
    vec2  c   = vec2(r) + vec2(RandF(seed), RandF(seed)) * max(1.0 - 2.0 * r, 0.0);
    float ang = RandF(seed) * 6.2831853;
    float el  = mix(0.6, 1.0, RandF(seed));
    vec2  p   = (g - cell - c) / r;
    p = vec2(cos(ang) * p.x + sin(ang) * p.y, (-sin(ang) * p.x + cos(ang) * p.y) / el);
    // irregular outline: a few lobes
    float th  = atan(p.y, p.x);
    float lob = 1.0 + 0.18 * sin(3.0 * th + RandF(seed) * 6.28) + 0.10 * sin(5.0 * th + RandF(seed) * 6.28);
    float d   = length(p) / lob;
    if (d >= 1.0)
        return;
    float edgeW = clamp(pixelUV * gridSize / r * 1.5, 0.05, 0.6);         // ~1.5 px soft edge
    float m   = 1.0 - smoothstep(1.0 - edgeW, 1.0, d);
    float a   = m * specks_opacity * bugs_opacity * (0.6 + 0.4 * smoothstep(0.5, 0.95, d));   // the dried rim covers more
    vec3  col = vec3(0.42, 0.34, 0.16) * mix(0.8, 1.15, RandF(seed)) * mix(1.0, 0.75, smoothstep(0.6, 0.95, d));
    vec3  nT  = normalize(vec3(p / max(lob, 0.5) * 0.25 * bugs_bump, 1.0));
    accRGB = col * a + accRGB * (1.0 - a);
    accN   = nT * a + accN * (1.0 - a);
    accA   = a + accA * (1.0 - a);
    accNA  = accA;
}

// Adds one layer of bugs to the premultiplied accumulators (accRGB, accA).
void bugLayer(float gridSize, float density, uint layerSeed,
              float pixelUV, vec2 aspect, float haloStrength, float opacityMul, float lightMix,
              inout vec3 accRGB, inout float accA, inout float smear,
              inout vec3 accN, inout float accNA)
{
    vec2  cell    = floor(f_coord * gridSize);
    float fadeSpan = BUG_FADE_TIME / max(bugs_fill_time, 1.0);

    for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {

            vec2 ncell = cell + vec2(float(ox), float(oy));

            // Dirt is patchy: big areas get more bugs than others; optional bias towards the top / bottom
            uint pseed     = Hash(uvec3(uvec2(ivec2(floor(ncell / 8.0))), layerSeed + 100u));
            float patchMul = mix(0.4, 1.6, RandF(pseed));
            float vy       = (ncell.y + 0.5) / gridSize;                 // 0 = bottom of the UV, 1 = top
            float vertMul  = clamp(1.0 + bugs_vertical * (vy - 0.5) * 2.0, 0.0, 2.0);
            float pCell    = clamp(density * patchMul * vertMul, 0.0, 1.0);

            // 1st test, fixed per cell (cheap, no texture): can this cell hold a bug at all?
            // Cells that fail it are skipped before the wiper mask is read.
            float pMax = min(2.0 * pCell, 1.0);
            uint  cseed = Hash(uvec3(uvec2(ivec2(ncell)), layerSeed + 31u));
            if (RandF(cseed) > pMax)
                continue;

            // When the wipers last swept over this cell. Whatever hit the glass before that
            // moment is gone; the cell then gets a brand new set of bugs (the seed depends on
            // that moment) that again take bugs_fill_time to build up.
            float bugsAt = 0.0;
            float tc = (bugs_wiper_clean > 0.5) ? lastCleanTime((ncell + 0.5) / gridSize, bugsAt) : WIPER_NONE;

            // the insects met since the glass here was clean (they come only above 20 km/h, see the head of the file)
            float met = max(rain_wind.w, 0.0);
            float tBase;
            float prog;
            if (tc < -1.5e9)
            {
                // The engine forgot when the wipers last swept here (another vehicle). We cannot know how dirty the glass was,
                // so show the level of the glass never wiped, at most bugs_unknown_level, until a blade passes.
                tBase = -1000.0;
                prog  = min(met / max(bugs_fill_time, 1.0), bugs_unknown_level);
            }
            else if (tc < -0.5e9)
            {
                // no blade reaches here: everything met so far
                tBase = bugs_start_time;
                prog  = met / max(bugs_fill_time, 1.0);
            }
            else
            {
                tBase = tc;
                prog  = max(met - bugsAt, 0.0) / max(bugs_fill_time, 1.0);
            }
            if (prog <= 0.0)
                continue;

            uint seed = Hash(uvec3(uvec2(ivec2(ncell)), layerSeed ^ floatBitsToUint(tBase)));

            // 2nd test, new after every wipe: together with the 1st one the cell holds a bug with
            // probability pCell, and a wiped cell gets a new set of bugs
            float present = RandF(seed);
            if (present * pMax > pCell)
                continue;

            // Moment (0..1 of the fill time) when this bug hits the glass
            float appearAt = RandF(seed);
            if (prog < appearAt)
                continue;
            float appear = smoothstep(appearAt, appearAt + fadeSpan, prog);

            float jx = RandF(seed);
            float jy = RandF(seed);
            vec2 center = (ncell + vec2(jx, jy)) / gridSize;

            float size   = mix(BUG_SIZE_MIN, BUG_SIZE_MAX, RandF(seed)) / gridSize * mix(0.7, 1.0, appear);
            float ang    = RandF(seed) * 6.2831853;
            float flipX  = (RandF(seed) < 0.5) ? -1.0 : 1.0;
            float choice = RandF(seed);
            float tone   = mix(0.8, 1.15, RandF(seed));

            vec2 d = (f_coord - center) * aspect;

            // Blur ring around the splat (greasy smear): strongest next to the bug, fading out
            // (the post pass blurs with a fixed radius in pixels, so a bug that is small on screen gets less of it)

            float c = cos(ang);
            float s = sin(ang);
            vec2 r  = vec2(c * d.x + s * d.y, -s * d.x + c * d.y);
            r.x *= flipX;

            vec2 local = r / size + 0.5;
            if (any(lessThan(local, vec2(0.0))) || any(greaterThan(local, vec2(1.0))))
                continue;

            // mip level from the on-screen size of the bug (derivatives are not
            // allowed inside this non-uniform branch, so pixelUV comes from main)
            float lod = clamp(log2(max(pixelUV / size * 0.5 * float(textureSize(raindropsatlas, 0).x), 1.0)), 0.0, 4.0);
            vec4 t = textureLod(raindropsatlas, tileOffset(choice) + local * 0.5, lod);

            // fine specks: fainter and lighter than the big bugs (the splat bodies are nearly black)
            t.rgb = mix(t.rgb, vec3(0.42, 0.34, 0.16), lightMix);
            float a = clamp(t.a * appear * bugs_opacity * opacityMul, 0.0, 1.0);

            // the scene blurred only where the bug is see-through (half transparent), not around it and not where it's opaque
            // (the post pass blurs with a fixed radius in pixels, so a bug that is small on screen gets less of it)
            float pxScale = clamp(size / max(pixelUV, 1e-6) / max(bugs_halo_px, 1.0), 0.15, 1.0);
            float seeThrough = smoothstep(0.03, 0.3, a) * (1.0 - smoothstep(0.75, 0.95, a));
            smear = max(smear, seeThrough * haloStrength * pxScale);

            // relief: tangent-space normal of the splat -> glass UV frame (undo the mirror, then the rotation)
            vec4 nm = textureLod(normalmap, tileOffset(choice) + local * 0.5, lod);
            vec2 nxy = (nm.rg * 2.0 - 1.0) * bugs_bump * appear;       // a splat that is still settling is flatter
            nxy.x *= flipX;
            vec2 nuv = vec2(c * nxy.x - s * nxy.y, s * nxy.x + c * nxy.y);
            vec3 nT  = vec3(nuv, sqrt(max(1.0 - dot(nuv, nuv), 0.02)));

            // "over" compositing in premultiplied form
            accRGB = t.rgb * tone * a + accRGB * (1.0 - a);
            accN   = nT * a + accN * (1.0 - a);
            accA   = a + accA * (1.0 - a);
            accNA  = accA;
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
    vec3  Tgl     = dp2perp * duv1.x + dp1perp * duv2.x;          // tangent frame of the glass (T along u, B along v)
    vec3  Bgl     = dp2perp * duv1.y + dp1perp * duv2.y;
    float invmax  = inversesqrt(max(max(dot(Tgl, Tgl), dot(Bgl, Bgl)), 1e-20));
    Tgl *= invmax;
    Bgl *= invmax;

    vec4 tex_color = texture(diffuse, f_coord);
    // No discard on a (nearly) clear glass texture: clean glass between patches of faint dirt would
    // otherwise be "bald" - no bugs, no halo.

    float gridSize = max(ceil(param[2].x), 1.0);
    float pixelUV  = max(length(dFdx(f_coord)), length(dFdy(f_coord)));   // uniform control flow here
    vec2  aspect   = (bugs_uv_aspect >= 1.0) ? vec2(bugs_uv_aspect, 1.0) : vec2(1.0, 1.0 / max(bugs_uv_aspect, 0.01));

    // 0 = never dirty yet; the real per-cell progress is computed in bugLayer (wipers reset it)
    float progress = clamp(max(rain_wind.w, 0.0) / max(bugs_fill_time, 1.0), 0.0, 1.0);

    vec3  bugRGB = vec3(0.0);    // premultiplied
    float bugA   = 0.0;
    float smear  = 0.0;          // blur ring around the bugs
    vec3  bugN   = vec3(0.0);    // normals of the splats, premultiplied like the colour
    float bugNA  = 0.0;
    if (progress > 0.0)
    {
        // fine specks underneath (procedural, one wiper read for the pixel), big bugs on top
        if (specks_density > 0.0)
        {
            float bugsPix = 0.0;
            float tcPix = (bugs_wiper_clean > 0.5) ? lastCleanTime(f_coord, bugsPix) : WIPER_NONE;
            speckLayer(gridSize * SPECK_SCALE, specks_density, tcPix, bugsPix, aspect, pixelUV, bugRGB, bugA, bugN, bugNA);
        }
        bugLayer(gridSize, bugs_density, 3u, pixelUV, aspect, 1.0, 1.0, 0.0, bugRGB, bugA, smear, bugN, bugNA);
        // now and then a really big one, on top (its own coarse grid: same 3x3 search, bigger cells)
        if (bugs_giant_density > 0.0)
            bugLayer(gridSize / max(bugs_giant_scale, 1.0), bugs_giant_density, 11u, pixelUV, aspect, 1.0, 1.0, 0.0,
                     bugRGB, bugA, smear, bugN, bugNA);
        // and rarely a huge one (about one per pane when it is fully dirty)
        if (bugs_huge_density > 0.0)
            bugLayer(gridSize / max(bugs_huge_scale, 1.0), bugs_huge_density, 13u, pixelUV, aspect, 1.0, 1.0, 0.0,
                     bugRGB, bugA, smear, bugN, bugNA);
    }

    // Bugs over the glass (premultiplied "over")
    float glassA  = tex_color.a * alpha_mult;
    float finalA  = bugA + glassA * (1.0 - bugA);
    vec3  premult = bugRGB + tex_color.rgb * glassA * (1.0 - bugA);
    vec3  albedo  = premult / max(finalA, 1e-4);

    // Reflections / specular belong to the glass only, not to the bugs
    float glassShare = glassA * (1.0 - bugA) / max(finalA, 1e-4);

    vec3  fragcolor    = ambient;
    // Relief of the splats: tilt the glass normal by the splat's slope (the tilt direction is physical,
    // so it does not matter which way the mesh normal points); flat where there is no bug.
    vec3  bugDir       = bugN + vec3(0.0, 0.0, 1.0) * (1.0 - bugNA);
    vec3  fragnormal   = Ngl;
    if (bugNA > 0.001)
        fragnormal = normalize(Ngl * max(bugDir.z, 0.05) + Tgl * bugDir.x + Bgl * bugDir.y);
    float reflectivity = param[1].z * glassShare;
    float specularity  = (tex_color.r + tex_color.g + tex_color.b) * 0.5 * glassShare
                       + bugs_gloss * bugA;                     // wet splats catch the sun and the headlights
    legacy_gloss();

    // Lit by sun, ambient and headlights like everything else (bugs glow in oncoming beams)
    // most of the glass is clear (no bug, a clear glass texture): the lighting of the 4.6 renderers (the clusters of lamps, the
    // shadows, the reflections) is skipped there, it would be multiplied by an alpha of ~0
    if (finalA > 0.002)
        fragcolor = apply_lights(fragcolor, fragnormal, albedo, reflectivity, specularity, shadow_tone);
    vec3 lit  = apply_fog(fragcolor);

#if BUGS_MATCH_RAIN_GLASS
    vec4 color = vec4(lit * finalA, finalA);
#else
    vec4 color = vec4(lit, finalA);
#endif

    out_color = POSTFX_ENABLED ? color : tonemap(color);

#if BUGS_DEBUG_TIME
    // Calibration: paints floor(time) as 8 red digits across the glass (the shader's own clock, in seconds).
    // Read the number right after the scenario has loaded and put it into bugs_start_time.
    {
        const int SEG[10] = int[10](63, 6, 91, 79, 102, 109, 125, 7, 127, 111);
        float dx  = (f_coord.x - 0.10) / 0.10;
        float dy  = (f_coord.y - 0.42) / 0.08;
        int   idx = int(floor(dx));
        if (idx >= 0 && idx < 8 && dy >= 0.0 && dy <= 2.0)
        {
            float tt  = floor(max(time, 0.0));
            float div = pow(10.0, float(7 - idx));
            int   dig = int(mod(floor(tt / div), 10.0));
            vec2  q   = vec2(fract(dx) / 0.8, dy);
            int   m   = SEG[dig];
            bool on = false;
            if ((m & 1)  != 0 && q.y > 1.86 && q.x > 0.1 && q.x < 0.9) on = true;                 // a
            if ((m & 2)  != 0 && q.x > 0.86 && q.x < 1.0 && q.y > 1.0 && q.y < 1.9) on = true;   // b
            if ((m & 4)  != 0 && q.x > 0.86 && q.x < 1.0 && q.y > 0.1 && q.y < 1.0) on = true;   // c
            if ((m & 8)  != 0 && q.y < 0.14 && q.x > 0.1 && q.x < 0.9) on = true;                // d
            if ((m & 16) != 0 && q.x < 0.14 && q.y > 0.1 && q.y < 1.0) on = true;                // e
            if ((m & 32) != 0 && q.x < 0.14 && q.y > 1.0 && q.y < 1.9) on = true;                // f
            if ((m & 64) != 0 && q.y > 0.93 && q.y < 1.07 && q.x > 0.1 && q.x < 0.9) on = true;  // g
            if (on)
                out_color = vec4(1.0, 0.05, 0.05, 1.0);
        }
    }
#endif

    if (MOTIONBLUR_ENABLED)
    {
        // alpha 0 = blending keeps the scene's own motion vectors
        // only the see-through parts of the bugs (smear); no blur over opaque parts of the pane (1 - glassA)
        float blurMask = clamp(smear * bugs_halo_blur * (1.0 - smoothstep(0.5, 0.95, glassA)), 0.0, 1.0);
        out_motion = (blurMask >= 0.01) ? vec4(WS_ENC * blurMask, 0.0, 0.0, 1.0) : vec4(0.0);
    }
}
