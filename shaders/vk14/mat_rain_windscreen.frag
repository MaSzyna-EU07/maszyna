#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// Windshield rain effects - v3 (rewrite)
// by @lcddisplay; wiper movement @MichauSto; rain v3: drop layers, drop life, rivulets that sweep drops
//
// What is on the glass:
//   1. Micro droplets: a dense grid of tiny, static droplets. Procedural (no texture reads): a spherical cap
//      per droplet. They make the glass look wet between the bigger drops.
//   2. Drops: one per grid cell (raindrop_grid_size), shape from the 2x2 drop tiles of the atlas
//      (normalmap: rg = normal xy, b = height, a = mask). A drop lands (quick pop), sits still for
//      drop_life_min..drop_life_max seconds, then makes room for a new one. Drops do NOT slide: all motion is
//      done by the rivulets (a sliding drop used to leave its 3x3 neighbourhood and got cut off).
//      Sizes are skewed (many medium, few big) and the tiles are randomly mirrored (8 shapes instead of 4).
//      Where drops overlap, the higher one wins (no averaged "ghost" normals).
//   3. Rivulets: a drop runs down in stick-slip fashion and leaves a wet trail (4 sprites in the right half
//      of the atlas, 256 x 1024: head in the bottom STREAK_HEAD_FRAC, trail above). The head sweeps up every
//      drop and droplet on its way: the trail band is clean until the rivulet fades out.
//      The trail right above the head is taken from the sprite part that touches the head, then it blends
//      into the trail pattern that is fixed to the glass (beads stay where they were left).
//
// Speed: the hashes and the life cycles of the cells (which drop a cell holds now, where, how big; where the head of a rivulet is)
// are worked out once a frame for every cell by rain_cells.comp (rain_cells.glsl), not here for the 9 cells around every pixel;
// the wipers are asked only for the drops the pixel lies on. the look is the same.
//
// Shading: the grey atlas (bright rim, dark middle) is read once per pixel, only for the drop that wins the
// pixel (it used to be read for all 9 neighbours). Micro droplets are shaded analytically.
//
// Refraction / blur (OpenGL 4.6, the renderer with the screen space reflections): the glass reads the image of the last
// frame the renderer keeps for the reflections (ssr_history, with mipmaps) and the depth of the scene (scene_depth):
//   - drops act as small lenses: the pixel shows the scene on the opposite side of the drop centre, so every drop holds a
//     small, flipped picture (drop_lens); rivulets bend the view by Snell's law from their normal,
//   - bright lights near a drop (signals, lamps, headlights) show up in it,
//   Works standing still, with the motion blur off and with taa. Needs gfx.ssr (default: medium); without it the drops show
//   the sky / ground only, faintly.
// The haze of the wet glass is blurred by the post-processing, from the picture of this frame: the glass writes a request
// into the velocity target (x = 64 * strength, y = 0) and postfx_taa.frag (with gfx.antialiasing taa: every frame) or
// postfx_motionblur.frag (without it: only while something moves) blur the picture there (rain_windscreen_haze.glsl).
//
// Speed: the air against the vehicle (rain_wind) drives the water: at speed a second layer of rivulets runs along its push
// (up a front glass, back along a side window) and sweeps the drops up on its way, the drops are stretched along the push,
// the haze grows. Standing still the rain is as it always was.
//
// OpenGL 4.6 renderer (feat/gl46-migration): headlights and lamps are clustered, calc_headlights takes the
// screen-space derivatives of f_pos, the sun glint follows the cloud shadow. Lamps (cab lamp, scenery) light up
// the drops too (drop_lamp_scatter).

layout(location = 0) in vec3 f_normal;
layout(location = 2) in vec2 f_coord;
layout(location = 3) in vec4 f_pos;

layout(location = 0) out vec4 out_color;

#ifdef NO_MOTION_OUTPUT
// a pass without the velocity target: the writes go nowhere
vec4 out_motion;
#else
layout(location = 1) out vec4 out_motion;
#endif

#pragma texture(diffuse,         0, sRGB_A)
#pragma texture(raindropsatlas,  1, sRGB_A)
#pragma texture(wipermask,       2, sRGB_A)
#pragma texture(normalmap,        3, RGBA)    // rg = normal xy, b = height, a = mask; 2048 x 1024 (drops left, rivulets right), linear
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
#include "rain_cells.glsl"

layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 1) uniform sampler2D raindropsatlas;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 2) uniform sampler2D wipermask;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 3) uniform sampler2D normalmap;

// --- Haze of the wet glass (blur) ------------------------------------------------
// The haze follows the rain and is wiped away by the wipers, as before (fully blurred from a quarter of the haze on).
const float blur_amount   = 1.0;            // strength of the haze: 0 = off, 1 = full
// after the wiper the haze grows with the rain fallen on the glass since (intensity x seconds, more at speed) from nothing up to
// its full strength, in any rain (a lighter rain takes longer). where no wiper reaches it is full
const float haze_fill  = 12.0;              // rain fallen since the blade to mist the glass up fully: seconds of the heaviest
                                            // rain standing still (2x as long at 1.5; the speed counted in by the game)
const float haze_curve = 1.0;               // > 1: slow at first, then more and more
const float WS_BEND_ENC   = 60.0;           // the drop request in the velocity target; must match rain_windscreen_haze.glsl
const float WS_DROP_BIAS  = 192.0;
const float WS_ENC        = 64.0;           // the haze request in the velocity target; must match rain_windscreen_haze.glsl

// --- How the water gathers on the glass ------------------------------------------------------------------
// the game counts the rain the glass meets (rain_state, gl::scene_ubs; more at speed) and how wet the glass is (0 dry .. 1 as full
// of drops as it gets): the glass fills up to the full in any rain, a lighter one only takes longer, and dries when the rain stops.
// the times are in the game (fill_rate, fill_speed, dry_halflife in the setup of the scene block). where a blade went over the glass
// the drops come back with the rain fallen since

// --- Drops (atlas tiles) ------------------------------------------------------
// (how long they live: rain_cells.glsl; the cells of the drops and of the rivulets are worked out once a frame by rain_cells.comp)

// --- Micro droplets -------------------------------------------------------------
const float micro_density   = 0.45;         // share of micro cells holding a droplet on a full glass, 0 = off
const float micro_scale     = 3.0;          // micro grid = raindrop_grid_size * micro_scale
const float micro_size_min  = 0.12;         // droplet radius in micro cells
const float micro_size_max  = 0.40;
const float micro_bulge     = 0.80;         // slope of the droplet dome at its rim (0..0.95)
const float micro_life      = 60.0;         // average seconds a droplet lives

// --- Look of the drops (grey atlas for drops / rivulets, the same look computed for micro droplets) ---------
const float drop_rim_start  = 0.30;         // micro droplets: dome slope (1 - nz) where the bright rim starts
const float drop_rim_end    = 0.80;         // ... and where it is fully bright
const float drop_light_side = 0.25;         // micro droplets: brighter on the upper-left side
const float drop_center_alpha = 0.25;       // how much the dark middle of a drop covers the scene
const float drop_rim_alpha    = 0.85;       // how much the bright rim covers the scene
const float drop_shade_alpha  = 0.30;       // faint alpha over the whole dome (carries the glints)

// --- Light on the drops --------------------------------------------------------
const float drop_ior               = 1.33;  // refraction index of water
const float drop_refract           = 0.5;   // rivulets: strength of the picture shift (Snell; 1 = physical)
const float drop_lens              = 2.0;   // drops: lens field, the drop shows (1 + drop_lens) x its own size, flipped
const float drop_lens_mix          = 0.7;   // drops: 1 = lens picture, 0 = Snell shift like the rivulets
const float drop_env               = 0.6;   // reflection of the environment on the dome (0 = off)
const float drop_glint_strength    = 5.0;   // sun / moon glint brightness
const float drop_glint_sharpness   = 40.0;  // sun / moon glint size (lower = a wider lobe, so more drops catch it)
const float drop_sheen             = 0.3;   // wide, soft sun sheen on the domes around the sharp glint
const float drop_headlight_scatter = 3.0;   // how much drops light up in headlight beams
const float drop_lamp_scatter      = 1.0;   // GL 4.6: how much drops light up from lamps (station lamps, signals), 0 = off
const float drop_light_near        = 4.0;   // lights nearer than this (metres; the own vehicle) don't light the drops, full from 2x
const float drop_light_max         = 3.0;
const float drop_dark_lum          = 0.25;  // average brightness of the screen (scene units) below which the drops darken with it   // the most the lamps and headlights add to a drop (scene units, before tonemapping)

// --- Rivulets (drops that run down and leave a wet trail) ---------------------------------
// Sizes are in "drop cells" (1 cell = 1 / raindrop_grid_size of the glass UV).
const float streak_density   = 1.00;  // share of cells carrying a rivulet (scaled by the rain intensity), 0 = off
const float streak_cell      = 6.0;   // width of a rivulet cell in drop cells
const float streak_run       = 20.0;  // height of a rivulet cell in drop cells = the longest run of one rivulet
const float streak_head      = 1.7;   // mean width of the drop head, in drop cells (randomised 0.75x .. 1.25x)
const float streak_wiggle    = 0.35;  // how far the path swings sideways, in head widths (0 = straight)
const float streak_wiggle_speed = 0.5; // speed of the little waves that flow down the trail
const float streak_flow      = 0.15;  // ripples flowing down inside the trail (shows in refraction and glints), 0 = off
const float streak_uv_aspect = 1.0;   // glass width / height in UV terms (1 = same stretch as the drops)
const float streak_sweep     = 1.0;   // 1 = the head sweeps up drops on its way (clean band behind it), 0 = off
const float streak_sweep_width = 0.9; // width of the swept band, in head widths
const float streak_link      = 1.5;   // head widths above the head where the trail is taken from the sprite part
                                        // that touches the head (so the trail always leaves the head cleanly)

// --- Speed: the air against the vehicle (rain_wind) drives the water on the glass --------------------
// The force on the water along the glass: its weight (gravity) and the drag of the air going along the glass (and of the
// share of the air meeting it head on which turns up along it), as in the stock shader of the renderer. Standing still
// (or in a light wind) nothing of it shows: the rain lands, grows and runs down as it always did.
const float wind_drag_gain     = 4.0;   // how strongly the air drives the water (1 = the drag of the stock shader: on a
                                        // vertical front glass the water would go up only at ~250 km/h)
const float wind_streaks       = 1.5;   // density of the rivulets driven by the air, at full drag (x streak_density)
const float wind_cell_scale    = 0.5;   // the rivulets of the air: their cells this much narrower ...
const float wind_run_scale     = 1.2;   // ... and their runs this long against those of gravity: few drops with long trails
const float wind_head_scale    = 0.4;   // their heads this much smaller (a small bead at the front of a long trail)
const int   wind_layers        = 4;     // layers of them, each with drops of its own (1 .. 4)
const float wind_long_run      = 3.0;   // the last layer: its trails this much longer than those of gravity (the long ones, a part of all)
const float WS_DRAG_SPEED = 24.0;       // speed of the air along the glass at which its drag equals the weight of a drop, m/s
const float WS_HEAD_ON    = 0.35;       // share of the air meeting the glass head on which turns up along it
const float WS_WIND_STEP  = 0.1309;     // the direction of the rivulets of the air in steps of 7.5 degrees (no seams on a
                                        // curved glass, where every triangle would turn them a little)

// --- The rivulets of the air of different thickness: the width of every one is drawn from this range (x streak_head, x
// wind_head_scale), most of them thin, a few thick (never wider than their cell) -------------------------------------------------
const float streak_thin       = 0.5;   // the thinnest
const float streak_thick      = 2.0;   // the thickest
const float streak_thick_bias = 2.0;   // > 1: more thin ones
// --- What the blades leave: they don't take every drop (a few stay where they were): wiper_miss of rain_cells.glsl -----------------

#ifndef STREAK_FLIP_Y
#define STREAK_FLIP_Y 0                 // set to 1 if the rivulets run UP the glass (head on top)
#endif
const float STREAK_TAIL_SCALE = 1.5;    // the trail pattern of the sprite is drawn this much longer than native
const float STREAK_HEAD_FRAC  = 0.22;   // part of the sprite height used by the drop head (= HEAD_FRAC of the atlas generator)

const float WS_INSIDE         = 0.3;   // metres beyond the glass nearer than which the image is taken for the inside of the cab
                                       // (in the rain the cab isn't in the depth: the nose of the own vehicle is outside)
const float WS_REFR_MAXPX     = 96.0;  // largest picture shift of a drop, screen pixels
const float WS_REFR_MIX       = 0.85;  // 0 = no refraction, 1 = only the refracted picture
const float WS_LIGHT_GAIN     = 1.0;   // brightness of the lights near a drop shown in it (0 = off)
const float WS_LIGHT_CONTRAST = 3.0;   // a light: this many times brighter than the place around it

// measuring what costs what (the time of "colour, translucent and cab" with each): 0 everything (as in the game), 1 the drops not lit
// (no sun glint, lamps, reflection), 2 no rivulets (and nothing swept by them), 3 no micro droplets, 4 no drops of the atlas,
// 5 none of the rain (only the glass and the haze)
#ifndef WS_PROFILE
#define WS_PROFILE 0
#endif

const vec3  DROP_LIGHT = vec3(-0.5527, 0.6532, 0.5226);   // same light as the atlas generator (upper left)

// the surroundings in a view space direction where the image doesn't hold them: the environment map above the horizon,
// the ground below it, blurred by Lod (as in the stock mat_rain_windscreen.frag)
vec3 wsSurroundings(vec3 direction, float lod)
{
    vec3 world = mat3(inv_view) * direction;
    vec3 sky = envmap_sample_rough(normalize(vec3(world.x, max(world.y, 0.0), world.z)), clamp(lod / 8.0, 0.0, 1.0));
    float below = smoothstep(-0.03 - 0.05 * lod, 0.25 + 0.05 * lod, -world.y);
    if ((below <= 0.0) || (false == ENVMAP_ENABLED))
        return sky;
    return mix(sky, reflection_ground(), below);
}

// what is seen through the glass in a view space direction: the image of the last frame where it holds the place, Distance
// away along the direction (0: far), blurred by Lod; Share: how much of it the image gave. Fade: towards the edges of the
// image the surroundings take over. The inside of the cab (near the camera in the depth) is never taken: the surroundings
// stand for it (else the drops would show the dark walls of the cab)
vec3 wsBehind(vec3 direction, float lod, float distance, out float share, bool fade)
{
    share = 0.0;
    vec3 around = wsSurroundings(direction, lod);
    if (ssr_params.x < 1.0)
        return around;
    vec4 previous = ssr_reprojection * ((distance > 0.0) ? vec4(direction * distance, 1.0) : vec4(direction, 0.0));
    if (previous.w <= 1e-5)
        return around;
    vec2 coords = (previous.xy / previous.w) * 0.5 + 0.5;
    vec2 edges = min(coords, 1.0 - coords);
    share = fade ? smoothstep(0.0, 0.04, min(edges.x, edges.y)) : step(-0.02, min(edges.x, edges.y));
    coords = clamp(coords, vec2(0.0), vec2(1.0));
    if (share <= 0.0)
        return around;
    ivec2 size = textureSize(scene_depth, 0);
    float depth = texelFetch(scene_depth, clamp(ivec2(coords * vec2(size)), ivec2(0), size - 1), 0).r;
    float near = projection[3][2];
    if ((depth > 0.0) && (near / depth < -f_pos.z + WS_INSIDE))
    {
        share = 0.0;
        return around;
    }
    vec3 color = textureLod(ssr_history, coords, clamp(lod, 0.0, ssr_params2.x)).rgb;
    if (any(isnan(color)) || any(isinf(color)))
    {
        share = 0.0;
        return around;
    }
    return mix(around, max(color, vec3(0.0)), share);
}

float wsLum(vec3 c)
{
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

// bright lights near a drop (signals, lamps, headlights): a drop is a wide-angle lens and picks them up. A light is where a
// fine mip level of the image (~11 px around) is much brighter than a coarse one (~90 px around)
vec3 wsLightGlow(vec3 direction, float distance)
{
    vec4 previous = ssr_reprojection * ((distance > 0.0) ? vec4(direction * distance, 1.0) : vec4(direction, 0.0));
    if (previous.w <= 1e-5)
        return vec3(0.0);
    vec2 coords = clamp((previous.xy / previous.w) * 0.5 + 0.5, vec2(0.0), vec2(1.0));
    vec3 near = textureLod(ssr_history, coords, min(3.5, ssr_params2.x)).rgb;
    vec3 wide = textureLod(ssr_history, coords, min(6.5, ssr_params2.x)).rgb;
    if (any(isnan(near)) || any(isinf(near)) || any(isnan(wide)) || any(isinf(wide)))
        return vec3(0.0);
    vec3 excess = max(near - wide * WS_LIGHT_CONTRAST, vec3(0.0));
    return min(excess, vec3(30.0)) * (2.0 * WS_LIGHT_GAIN);
}

// a layer Over put over Under, the colours not premultiplied
vec4 wsOver(vec4 over, vec4 under)
{
    float alpha = over.a + under.a * (1.0 - over.a);
    if (alpha <= 1e-5)
        return vec4(0.0);
    return vec4((over.rgb * over.a + under.rgb * under.a * (1.0 - over.a)) / alpha, alpha);
}

// ===========================================================================
float GetMixFactor(in vec2 co, out float side);

// the rain fallen when each blade last went over the place of its sweep given by the wiper mask (-1: not known), see
// gl::scene_ubs::wiper_rain_pass
vec4 wsWipePass(vec4 mask)
{
    ivec4 b = min(ivec4(max(mask, vec4(0.0)) * float(WIPER_PASS_BUCKETS)), ivec4(WIPER_PASS_BUCKETS - 1));
    return vec4(wiper_rain_pass[b.x].x, wiper_rain_pass[b.y].y, wiper_rain_pass[b.z].z, wiper_rain_pass[b.w].w);
}

uvec2 cellKey(vec2 c);

vec2 tileOffset(float choice)
{
    if      (choice < 0.25) return vec2(0.0, 0.0);
    else if (choice < 0.5)  return vec2(0.5, 0.0);
    else if (choice < 0.75) return vec2(0.0, 0.5);
    return vec2(0.5, 0.5);
}

// the 2x2 drop tiles live in the left half of the atlas: a tile is 0.25 wide and 0.5 high in atlas UV
vec2 dropAtlasUV(float choice, vec2 uv)
{
    vec2 off = tileOffset(choice);
    return vec2((off.x + uv.x * 0.5) * 0.5, off.y + uv.y * 0.5);
}

uvec2 cellKey(vec2 c)
{
    return uvec2(ivec2(c) + ivec2(65536));
}

// --- one rivulet cell ------------------------------------------------------------
struct Riv
{
    bool  on;
    float life;     // 0..1 (fade in / out)
    float width;    // sprite width = head width (glass UV)
    float headH;    // head height (glass UV)
    float topY;     // where the trail starts (fixed on the glass)
    float bottom;   // bottom of the head now
    float left;     // left edge of the sprite
    float flip;
    float col;      // which of the 4 sprites
    float wipe;
    float wavK, wavPh, wavSp;
};

// the layer of rivulets being worked on: the one of gravity (the rivulets as they always were, coordinates = glass UV with v
// up) or the one of the air (its own cells and seeds, turned so that its rivulets run along the push of the air)
float wsLayerDensity = 1.0;    // x rain x streak_density: share of the cells holding a rivulet
int   wsLayerIndex   = 0;      // its cells in rain_cells (0 gravity, 1 .. 4 the air)
float wsLayerHead    = 1.0;    // x streak_head
float wsLayerCos     = 1.0;    // the layer is turned by this angle (its "down" is the way its water runs on the glass)
float wsLayerSin     = 0.0;
bool  wsLayerVaried  = false;  // the widths of its rivulets from streak_thin .. streak_thick (the air)

// glass UV (v up) -> coordinates of the layer, where the rivulets run down (-v)
vec2 wsToLayerDir(vec2 v)
{
    return vec2(wsLayerCos * v.x + wsLayerSin * v.y, -wsLayerSin * v.x + wsLayerCos * v.y);
}
vec2 wsToLayer(vec2 g)
{
    return wsToLayerDir(g - 0.5) + 0.5;
}
vec2 wsFromLayerDir(vec2 v)
{
    return vec2(wsLayerCos * v.x - wsLayerSin * v.y, wsLayerSin * v.x + wsLayerCos * v.y);
}
vec2 wsFromLayer(vec2 p)
{
    return wsFromLayerDir(p - 0.5) + 0.5;
}

// the layers: 0 gravity, 1 air. WindShare: how much the air drives the water (0 standing still), WindCos / WindSin: the turn
// of the layer of the air
float wsWindShare = 0.0;
float wsWindCos   = 1.0;
float wsWindSin   = 0.0;
void wsSetLayer(int layer)
{
    if (layer == 0)
    {
        wsLayerDensity = 1.0 - wsWindShare;
        wsLayerIndex   = 0;
        wsLayerHead    = 1.0;
        wsLayerCos     = 1.0;
        wsLayerSin     = 0.0;
        wsLayerVaried  = false;
    }
    else
    {
        wsLayerDensity = wsWindShare * wind_streaks;
        wsLayerIndex   = layer;
        wsLayerHead    = wind_head_scale;
        wsLayerCos     = wsWindCos;
        wsLayerSin     = wsWindSin;
        wsLayerVaried  = true;
    }
}

// the rivulet of cell SCell of the layer set (wsSetLayer): the randoms and the life of the cell from rain_cells (rain_cells.comp
// worked them out as this did once from the hashes of the cell), the sizes and the places from them
Riv getRiv(vec2 sCell, float sCW, float sCH, float gridSize)
{
    Riv r;
    uvec4 d  = rain_cells[rain_riv_index(wsLayerIndex, ivec2(sCell))];
    r.on     = ((float(d.x & 65535u) + 0.5) * (1.0 / 65536.0) < streak_density * wsLayerDensity * rain_state.y);   // as wet as the glass is
    r.life   = float((d.x >> 16) & 255u) * (1.0 / 255.0);
    float wr = rain_byte(d.y, 16u);
    r.width  = streak_head * wsLayerHead / gridSize
             * (wsLayerVaried ? mix(streak_thin, streak_thick, pow(wr, streak_thick_bias)) : mix(0.75, 1.25, wr));
    if (wsLayerVaried)
        r.width = min(r.width, sCW / 2.6);   // the trail with its swing (sMargin) stays in its cell
    r.headH  = r.width * 0.88 * streak_uv_aspect;
    float sSpan = max(sCH * mix(0.35, 0.70, rain_byte(d.y, 24u)), r.headH * 3.0);
    // stick-slip: how far down its run the head is now (rain_cells.comp)
    float sRun  = float(d.y & 65535u) * (1.0 / 65535.0);
    r.topY   = sCell.y * sCH + sSpan + rain_byte(d.z, 0u) * max(sCH - sSpan, 0.0);
    r.bottom = r.topY - r.headH * 1.2 - sRun * (sSpan - r.headH * 1.2);
    const float sMargin = 0.8;
    r.left   = sCell.x * sCW + sMargin * r.width + rain_byte(d.z, 8u) * max(sCW - r.width * (1.0 + 2.0 * sMargin), 0.0);
    r.flip   = (((d.w >> 8) & 1u) != 0u) ? 1.0 : 0.0;
    r.col    = float((d.w >> 9) & 3u);
    r.wipe   = rain_byte(d.x, 24u);
    r.wavK   = 6.2831853 / mix(2.5, 5.0, rain_byte(d.z, 16u));
    r.wavPh  = rain_byte(d.z, 24u) * 6.2831853;
    r.wavSp  = streak_wiggle_speed * mix(0.6, 1.4, rain_byte(d.w, 0u));
    return r;
}

// sideways offset of the path (in head widths) at distance dW (head widths) down from the start of the trail
float rivWob(Riv r, float dW, out float ramp, out float wobD)
{
    float rampT = clamp(dW / 2.5, 0.0, 1.0);
    ramp        = rampT * rampT * (3.0 - 2.0 * rampT);
    float rampD = 3.0 * rampT * (1.0 - rampT) / 2.5;
    float a1    = dW * r.wavK + r.wavPh;
    float a2    = dW * r.wavK * 1.7 - time * r.wavSp * 1.0472 + r.wavPh * 2.0;   // the wave runs DOWN
    float w1    = 0.7 * sin(a1) + 0.3 * sin(a2);
    float w1D   = 0.7 * cos(a1) * r.wavK + 0.3 * cos(a2) * r.wavK * 1.7;
    wobD        = -streak_wiggle * (rampD * w1 + ramp * w1D);
    return streak_wiggle * ramp * w1;
}

// how much a drop centred at p (glass UV, v up) with radius rad has been swept up by rivulet r (0..1)
float rivSwept(Riv r, vec2 p, float rad)
{
    if (!r.on || r.life <= 0.001) return 0.0;
    if (p.y > r.topY || p.y < r.bottom - rad) return 0.0;
    bool  inHead = p.y < r.bottom + r.headH;
    float dW = (r.topY - (inHead ? r.bottom + 0.5 * r.headH : p.y)) / r.width;
    float ramp, wobD;
    float wob  = rivWob(r, dW, ramp, wobD);
    float cx   = r.left + (0.5 + wob) * r.width;
    float halfW = 0.5 * streak_sweep_width * r.width;
    return (1.0 - smoothstep(halfW, halfW + rad, abs(p.x - cx))) * r.life;
}

// candidate on the glass that wins the pixel (the highest dome)
struct Hit
{
    float score;
    float cov;      // coverage of the dome (normal map alpha)
    float colA;     // coverage of the drawn drop (sharper)
    vec3  n;        // dome normal, tangent space of the glass
    vec2  offUV;    // pixel - centre (glass UV), for the lens
    float radUV;    // radius (glass UV)
    float isDrop;   // 1 = drop / droplet (lens), 0 = rivulet (Snell)
    float vis;      // life / sweep visibility
    vec2  auv;      // grey atlas lookup of the winner (drops, rivulets)
    vec2  agx, agy;
    float atlas;    // 1 = shade from the grey atlas, 0 = analytic (micro droplets: shade in colA's place)
    float shade;    // analytic shade (micro droplets)
};

void consider(inout Hit h, float score, float cov, float colA, vec3 n, vec2 offUV, float radUV, float isDrop,
              float vis, vec2 auv, vec2 agx, vec2 agy, float atlas, float shade)
{
    if (score > h.score)
    {
        h.score = score; h.cov = cov; h.colA = colA; h.n = n;
        h.offUV = offUV; h.radUV = radUV; h.isDrop = isDrop;
        h.vis = vis; h.auv = auv; h.agx = agx; h.agy = agy; h.atlas = atlas; h.shade = shade;
    }
}

// the rivulet R of the layer set (wsSetLayer) at the pixel, at SUVr in the coordinates of the layer; Rd1 / Rd2: their change
// per screen pixel
void drawRivulet(inout Hit hit, Riv r, vec2 sUVr, vec2 rd1, vec2 rd2)
{
    if (!(r.on && r.life > 0.001))
        return;
    const float sMargin = 0.8;
    float sLen = r.topY - r.bottom;
    float sx   = (sUVr.x - r.left) / r.width;
    float sy   = sUVr.y - r.bottom;
    if (sx >= -sMargin && sx <= 1.0 + sMargin && sy >= 0.0 && sy <= sLen)
    {
        bool  inHead = sy < r.headH;
        float dW = (r.topY - (inHead ? r.bottom + 0.5 * r.headH : sUVr.y)) / r.width;
        float ramp, wobD;
        float wob = rivWob(r, dW, ramp, wobD);
        if (inHead) wobD = 0.0;
        float sxw = sx - wob;
        if (sxw >= 0.0 && sxw <= 1.0)
        {
            float sSide;
            vec2  sWipeUV = f_coord;
            if (inHead)
            {
                sWipeUV = wsFromLayer(vec2(r.left + 0.5 * r.width, r.bottom + 0.5 * r.headH));
                if (STREAK_FLIP_Y != 0) sWipeUV.y = 1.0 - sWipeUV.y;
            }
            if (GetMixFactor(sWipeUV, sSide) >= mix(0.5, 1.0, r.wipe))
            {
                float sNative = 3.125 * r.width * streak_uv_aspect * STREAK_TAIL_SCALE;
                float sxs = mix(sxw, 1.0 - sxw, r.flip);
                float su  = 0.5 + (r.col + sxs) * 0.125;
                vec2  sgx = vec2(0.125 / r.width * rd1.x, 0.0);
                vec2  sgy = vec2(0.125 / r.width * rd2.x, 0.0);
                vec4  sNm;
                vec2  sAuv;
                float vtSign = 1.0;
                if (inHead)
                {
                    float kv = STREAK_HEAD_FRAC / r.headH;
                    sgx.y = kv * rd1.y; sgy.y = kv * rd2.y;
                    sAuv = vec2(su, sy / r.headH * STREAK_HEAD_FRAC);
                    sNm = textureGrad(normalmap, sAuv, sgx, sgy);
                }
                else
                {
                    float kv = (1.0 - STREAK_HEAD_FRAC) / sNative;
                    sgx.y = kv * rd1.y; sgy.y = kv * rd2.y;
                    // a) trail pattern fixed to the glass, tiled by mirroring (beads stay where they were left)
                    float dd  = max(r.topY - sUVr.y, 0.0) / sNative;
                    float vt  = abs(mod(dd, 2.0) - 1.0);
                    vtSign    = (mod(dd, 2.0) < 1.0) ? 1.0 : -1.0;
                    // b) right above the head: the sprite part that touches the head
                    float above = (sy - r.headH) / r.width;              // head widths above the head
                    float link  = 1.0 - smoothstep(0.5 * streak_link, streak_link, above);
                    vec4  nGlass = vec4(0.0);
                    vec4  nLink  = vec4(0.0);
                    vec2  uvGlass = vec2(su, STREAK_HEAD_FRAC + vt * (1.0 - STREAK_HEAD_FRAC));
                    vec2  uvLink  = vec2(su, STREAK_HEAD_FRAC + min((sy - r.headH) / sNative, 1.0) * (1.0 - STREAK_HEAD_FRAC));
                    if (link < 0.999)
                        nGlass = textureGrad(normalmap, uvGlass, sgx, sgy);
                    if (link > 0.001)
                        nLink = textureGrad(normalmap, uvLink, sgx, sgy);
                    sAuv = (link > 0.5) ? uvLink : uvGlass;
                    sNm    = mix(nGlass, nLink, link);
                    vtSign = mix(vtSign, 1.0, link);
                }

                float sFade = inHead ? 1.0 : smoothstep(0.0, 1.5, dW);   // the start of the trail melts into the glass
                float sVis  = r.life * sFade;
                float sCov  = smoothstep(0.1, 0.6, sNm.a) * sVis;
                if (sCov > 0.001)
                {
                    vec2 nxy = sNm.rg * 2.0 - 1.0;
                    if (r.flip > 0.5) nxy.x = -nxy.x;
                    if (STREAK_FLIP_Y != 0) nxy.y = -nxy.y;
                    float nz = sqrt(1.0 - clamp(dot(nxy, nxy), 0.0, 1.0));
                    vec2 slope = -nxy / max(nz, 0.08);
                    if (!inHead)
                        slope.y = slope.y * vtSign / STREAK_TAIL_SCALE - slope.x * wobD
                                  + streak_flow * ramp * sin(dW * 4.8 - time * 4.0);
                    // the slope of the layer to the one of the glass
                    if (STREAK_FLIP_Y != 0) slope.y = -slope.y;
                    slope = wsFromLayerDir(slope);
                    if (STREAK_FLIP_Y != 0) slope.y = -slope.y;
                    vec3 n = normalize(vec3(-slope, 1.0));
                    float h = sNm.b * r.width * 0.5;
                    consider(hit, sCov * (h + 1e-5), sCov, 0.0, n, vec2(0.0), r.width * 0.5, 0.0,
                             sVis, sAuv, sgx, sgy, 1.0, 0.0);
                }
            }
        }
    }
}

// how much of a drop centred at Cg (glass UV, v up) with radius Rad the rivulets of both layers have swept up (0..1 kept)
float wsKept(vec2 cg, float rad, float sCW, float sCH, float gridSize)
{
    wsSetLayer(0);
    Riv  rv = getRiv(floor(cg / vec2(sCW, sCH)), sCW, sCH, gridSize);
    float kept = 1.0 - streak_sweep * rivSwept(rv, cg, rad);
    if (wsWindShare > 0.001)
    {
        wsSetLayer(1);
        vec2 cw = wsToLayer(cg);
        vec2 wc = vec2(sCW * wind_cell_scale, sCH * wind_run_scale);
        Riv  rw = getRiv(floor(cw / wc), wc.x, wc.y, gridSize);
        kept *= 1.0 - streak_sweep * rivSwept(rw, cw, rad);
        wsSetLayer(0);
    }
    return kept;
}

// the rain fallen (rain_state.x) since a blade last went over this place (texture UV); -1 where no blade goes or not known
float wsWipeRain(vec2 co)
{
    vec4  mask    = textureLod(wipermask, co, 0.0);
    vec4  at      = wsWipePass(mask);
    float fallen  = -1.0;
    for (int i = 0; i < 4; ++i)
        if ((mask[i] > 0.001) && (at[i] >= 0.0))
            fallen = (fallen < 0.0) ? max(rain_state.x - at[i], 0.0) : min(fallen, max(rain_state.x - at[i], 0.0));
    return fallen;
}

// the tangent frame of the glass (the directions of u and v on it, view space) from the screen-space change of the place and the uv
void wsGlassFrame(vec3 Ngl, vec2 duv1, vec2 duv2, out vec3 Tgl, out vec3 Bgl)
{
    vec3  dp1     = dFdx(f_pos.xyz);
    vec3  dp2     = dFdy(f_pos.xyz);
    vec3  dp2perp = cross(dp2, Ngl);
    vec3  dp1perp = cross(Ngl, dp1);
    Tgl = dp2perp * duv1.x + dp1perp * duv2.x;
    Bgl = dp2perp * duv1.y + dp1perp * duv2.y;
    float invmax  = inversesqrt(max(max(dot(Tgl, Tgl), dot(Bgl, Bgl)), 1e-20));
    Tgl *= invmax;
    Bgl *= invmax;
}

// the drop of a cell for one side of the blades (A, B: its half of the cell in rain_cells): where it lies, how big it is and its
// shape (the sag, the stretch by the air along PushT). true when P lies on it
bool wsDropShape(uint a, uint b, vec2 nc, float gridSize, float spreadF, float speed, vec2 pushT, vec2 p,
                 out vec2 center, out float sq, out float sag, out mat2 shapeM, out vec2 dl)
{
    center = (nc + vec2(rain_byte(a, 8u), rain_byte(a, 16u))) / gridSize;
    float size = mix(drop_size_min, drop_size_max, rain_byte(a, 24u)) / gridSize;   // the bias of the sizes: rain_cells.comp
    sag    = 1.0 + drop_sag * rain_byte(b, 0u);
    // at speed the air stretches the drop along its push (1 standing still: the drop as it always was)
    float stretch = 1.0 + drop_wind_stretch * speed * rain_byte(b, 8u);
    shapeM = mat2(1.0, 0.0, 0.0, 1.0 / sag) * (mat2(1.0) + (1.0 / stretch - 1.0) * outerProduct(pushT, pushT));
    sq     = size * spreadF;   // spreads out as it lands
    dl     = shapeM * (p - center);
    return max(abs(dl.x), abs(dl.y)) < sq * 0.5;
}

void main()
{
    // ---- screen-space derivatives (before any non-uniform flow) ----
    // only what the drops, the droplets and the rivulets read is kept through their loops; the frame of the glass, the change of the
    // view and the lens are worked out again for the shading after them (fewer registers in the loops: more pixels at once)
    vec2  duv1    = dFdx(f_coord);
    vec2  duv2    = dFdy(f_coord);
    float uvPerPx = max(length(duv1), length(duv2));
    vec3  Ngl     = normalize(f_normal);

    vec4 tex_color = texture(diffuse, f_coord);
    float glassA   = tex_color.a * alpha_mult;

    float gridSize = ceil(param[2].x);
    vec2  rainCoord = f_coord;

    float ambLum    = clamp(dot(ambient, vec3(0.2126, 0.7152, 0.0722)), 0.0, 1.0);
    float dynBright = pow(clamp(ambLum * 6.0, 0.0, 1.0), 0.5);
    // GL 4.6: the ambient light isn't a level of 0..1 any more (it is in the units of the scene, the light of the sky added), so
    // at night the drops came out as bright as by day. how bright the picture around really is says it: the coarsest level of
    // the image of the last frame, the average of the screen
    float sceneLum = (ssr_params.x >= 1.0) ? wsLum(textureLod(ssr_history, vec2(0.5), ssr_params2.x).rgb) : 1.0;
    float wsNight  = smoothstep(0.0, drop_dark_lum, sceneLum);     // 0 dark night .. 1 daylight
    dynBright *= wsNight;

    Hit hit;
    hit.score = 0.0; hit.cov = 0.0; hit.colA = 0.0; hit.n = vec3(0.0, 0.0, 1.0);
    hit.offUV = vec2(0.0); hit.radUV = 1.0; hit.isDrop = 1.0;
    hit.vis = 0.0; hit.auv = vec2(0.0); hit.agx = vec2(0.0); hit.agy = vec2(0.0); hit.atlas = 0.0; hit.shade = 0.0;

    // ---- the air against the vehicle on the glass (uniform flow: the derivatives of the glass) ----
    vec2  wsPushT;
    float wsSpeed;
    float flipS  = (STREAK_FLIP_Y != 0) ? -1.0 : 1.0;          // texture v -> glass v (up)
    vec3  wsN    = (dot(Ngl, f_pos.xyz) < 0.0) ? -Ngl : Ngl;    // the outside of the glass
    {
    vec3  Tgl, Bgl;
    wsGlassFrame(Ngl, duv1, duv2, Tgl, Bgl);
    vec3  wsUp   = vec3(0.0, 1.0, 0.0) * mat3(inv_view);       // world up, view space
    vec3  wsAir  = rain_wind.xyz * mat3(inv_view);              // the air against the vehicle, view space, m/s
    vec3  wsGrav = -wsUp - wsN * dot(-wsUp, wsN);               // the weight along the glass
    vec3  wsRise = wsUp - wsN * dot(wsUp, wsN);
    wsRise = (length(wsRise) > 0.1) ? normalize(wsRise) : vec3(0.0);
    vec3  wsFlow = wsAir - wsN * dot(wsAir, wsN) + wsRise * (max(-dot(wsAir, wsN), 0.0) * WS_HEAD_ON);
    float wsDrag = dot(wsFlow, wsFlow) / (WS_DRAG_SPEED * WS_DRAG_SPEED) * wind_drag_gain;   // in units of the weight
    vec3  wsFlowDir = (length(wsFlow) > 1e-3) ? normalize(wsFlow) : vec3(0.0);
    vec3  wsPush = wsGrav + wsFlowDir * wsDrag;
    // to the glass UV (v up) through the tangent frame of the texture
    vec2  gravG  = vec2(dot(wsGrav, Tgl), dot(wsGrav, Bgl) * flipS);
    vec2  flowG  = vec2(dot(wsFlowDir, Tgl), dot(wsFlowDir, Bgl) * flipS);
    wsSpeed = smoothstep(0.3, 1.2, wsDrag);               // 0 standing still .. 1 at speed (haze, drops)
    wsWindShare   = smoothstep(0.5, 1.3, wsDrag);               // how much the air drives the rivulets
    if (wsWindShare > 0.0)
    {
        // the rivulets of the air run along the push of the air at twice the weight: the same way at any speed (they don't swing
        // round as the train speeds up), up a front glass, back and down a side window
        vec2 way = gravG + flowG * 2.0;
        if (dot(way, way) > 1e-8)
        {
            way = normalize(way);
            float turn = round(atan(way.x, -way.y) / WS_WIND_STEP) * WS_WIND_STEP;
            wsWindCos = cos(turn);
            wsWindSin = sin(turn);
        }
        else
            wsWindShare = 0.0;
    }
    // the push of the air on the drops, texture UV (for stretching them)
    wsPushT  = vec2(dot(wsPush, Tgl), dot(wsPush, Bgl));
    wsPushT = (dot(wsPushT, wsPushT) > 1e-10) ? normalize(wsPushT) : vec2(0.0, -1.0);
    }

    // the wiper state of this pixel (blur, micro droplets)
    float hazeSide;
    float wipeVis = GetMixFactor(f_coord, hazeSide);
    // the change of the glass UV (v up) per screen pixel, for the rivulets
    vec2  wsRd1 = vec2(duv1.x, flipS * duv1.y);
    vec2  wsRd2 = vec2(duv2.x, flipS * duv2.y);

    // ---- rivulet cell of this pixel (also used to sweep the drops) ----
    wsSetLayer(0);
    float sCW   = streak_cell / gridSize;
    float sCH   = streak_run  / gridSize;
    vec2  sUVg  = vec2(f_coord.x, (STREAK_FLIP_Y != 0) ? 1.0 - f_coord.y : f_coord.y);   // glass UV, v up
    vec2  sCell = floor(sUVg / vec2(sCW, sCH));
    bool  sweepOn = streak_sweep > 0.0 && streak_density > 0.0 && (wind_streaks > 0.0 || wsWindShare < 1.0) && (WS_PROFILE != 2);

    if ((rain_state.y > 0.001) && (WS_PROFILE != 5))
    {
        // ===================================================================
        // 1. drops from the atlas: this cell and its 8 neighbours
        // ===================================================================
        vec2  cellF  = rainCoord * gridSize;
        vec2  cell   = floor(cellF);
#if WS_PROFILE != 4
        // the drops of the 9 cells around which reach the quarter of the cell the pixel lies in (worked out by rain_cells.comp,
        // with the drops there are now): usually 1 .. 3 of them
        uvec2 quarter = uvec2(greaterThanEqual(cellF - cell, vec2(0.5)));
        uint  reach   = rain_cells[rain_mask_index(ivec2(cell))][quarter.y * 2u + quarter.x];
        while (reach != 0u)
        {
            int   bit = findLSB(reach);
            reach &= reach - 1u;
            vec2  nc = cell + vec2(float(bit % 3 - 1), float(bit / 3 - 1));
            uvec4 d  = rain_cells[rain_drop_index(ivec2(nc))];
            // as many drops as the glass is wet (the cells were sorted out with a little more)
            if (rain_byte(d.y, 24u) > min(rain_state.y * drop_density, 0.95))
                continue;
            float vis = float(d.w >> 24) * (1.0 / 255.0);
            float spreadF = mix(0.8, 1.0, float((d.y >> 19) & 31u) * (1.0 / 31.0));
            // a drop the blades miss: the same drop before and after the stroke, left where it is
            bool  missed  = ((d.w >> 19) & 1u) != 0u;
            vec2  center0, center1, dl0, dl1;
            float sq0, sq1, sag0, sag1;
            mat2  shape0, shape1;
            bool  in0 = wsDropShape(d.x, d.y, nc, gridSize, spreadF, wsSpeed, wsPushT, rainCoord, center0, sq0, sag0, shape0, dl0);
            bool  in1 = (!missed) && wsDropShape(d.z, d.w, nc, gridSize, spreadF, wsSpeed, wsPushT, rainCoord, center1, sq1, sag1, shape1, dl1);
            // the wipers asked only for the drops the pixel lies on
            if (!(in0 || in1))
                continue;
            float side = 0.0;
            float mixF = missed ? 1.0 : GetMixFactor((nc + 0.5) / gridSize, side);
            bool  back = (side > 0.5);
            if (!(back ? in1 : in0))
                continue;
            if (mixF < rain_byte(back ? d.z : d.x, 0u))
                continue;
            uint  db      = back ? d.w : d.y;
            vec2  center  = back ? center1 : center0;
            float sq      = back ? sq1 : sq0;
            float sag     = back ? sag1 : sag0;
            mat2  shapeM  = back ? shape1 : shape0;
            vec2  dl      = back ? dl1 : dl0;
            float flipX   = (((db >> 16) & 1u) != 0u) ? 1.0 : 0.0;
            float choice  = (float((db >> 17) & 3u) + 0.5) * 0.25;
            vec2  diff    = rainCoord - center;

            if (sweepOn)
            {
                vec2  cg = vec2(center.x, (STREAK_FLIP_Y != 0) ? 1.0 - center.y : center.y);
                vis *= wsKept(cg, 0.35 * sq, sCW, sCH, gridSize);
            }
            if (vis <= 0.001)
                continue;

            vec2 luv = dl / sq + 0.5;
            if (flipX > 0.5) luv.x = 1.0 - luv.x;
            float atlasTexels = float(textureSize(normalmap, 0).x);
            float quadPx = sq / max(uvPerPx, 1e-6);
            float nlod   = clamp(log2(max(0.25 * atlasTexels / max(quadPx, 1.0), 1.0)), 0.0, 6.0);
            vec4  nm     = textureLod(normalmap, dropAtlasUV(choice, luv), nlod);
            float cov    = smoothstep(0.1, 0.6, nm.a) * vis;
            if (cov <= 0.001)
                continue;
            vec2 nxy = nm.rg * 2.0 - 1.0;
            if (flipX > 0.5) nxy.x = -nxy.x;
            float nz = sqrt(1.0 - clamp(dot(nxy, nxy), 0.0, 1.0));
            vec2 slope = transpose(shapeM) * (-nxy / max(nz, 0.08));
            vec3 n = normalize(vec3(-slope, 1.0));
            // gradients of the tile UV (for the one atlas read after the loop)
            vec2 gx = (shapeM * duv1) / sq * vec2(0.25, 0.5);
            vec2 gy = (shapeM * duv2) / sq * vec2(0.25, 0.5);
            consider(hit, cov * (nm.b * sq * sag + 1e-5), cov, 0.0, n, diff, 0.5 * sq, 1.0,
                     vis, dropAtlasUV(choice, luv), gx, gy, 1.0, 0.0);
        }
#endif

        // ===================================================================
        // 2. micro droplets: procedural, one per micro cell, fully inside it
        // ===================================================================
        if ((micro_density > 0.0) && (WS_PROFILE != 3))
        {
            float mg  = gridSize * micro_scale;
            vec2  mc  = floor(rainCoord * mg);
            uvec2 mk  = cellKey(mc);
            uint  m0  = Hash(uvec3(mk, 51u));
            float mLife = micro_life * mix(0.6, 1.6, RandF(m0));
            float mtt   = time / mLife + RandF(m0);
            float mph   = fract(mtt);
            uint  mep   = uint(mod(floor(mtt), 65521.0));
            uint  m1    = Hash(uvec3(mk, mep * 4u + 1u));
            if (RandF(m1) < min(micro_density * rain_state.y, 0.95))
            {
                uint m2 = Hash(uvec3(mk, mep * 4u + 2u + uint(hazeSide) * 2u));
                if (wipeVis >= RandF(m2))
                {
                    float r   = mix(micro_size_min, micro_size_max, pow(RandF(m2), 1.5));
                    vec2  c   = mc + vec2(r) + vec2(RandF(m2), RandF(m2)) * max(1.0 - 2.0 * r, 0.0);
                    float ell = mix(0.85, 1.15, RandF(m2));
                    vec2  p   = (rainCoord * mg - c) / r;
                    p.x *= ell;
                    float d2  = dot(p, p);
                    if (d2 < 1.0)
                    {
                        float vis = smoothstep(0.0, 0.01, mph) * (1.0 - smoothstep(0.97, 1.0, mph));
                        if (sweepOn)
                        {
                            vec2 cu = c / mg;
                            vec2 cg = vec2(cu.x, (STREAK_FLIP_Y != 0) ? 1.0 - cu.y : cu.y);
                            vis *= wsKept(cg, r / mg, sCW, sCH, gridSize);
                        }
                        float d   = sqrt(d2);
                        float cov = smoothstep(1.0, 0.8, d) * vis;
                        if (cov > 0.001)
                        {
                            vec2  nxy = p * micro_bulge;
                            float nz  = sqrt(1.0 - clamp(dot(nxy, nxy), 0.0, 1.0));
                            vec3  n   = normalize(vec3(nxy, nz));
                            float h   = sqrt(1.0 - d2) * micro_bulge * (r / mg);
                            float rim = smoothstep(drop_rim_start, drop_rim_end, 1.0 - n.z);
                            float lt  = dot(n, DROP_LIGHT) * drop_light_side * 0.5 + (1.0 - drop_light_side * 0.5);
                            consider(hit, cov * (h + 1e-6), cov, smoothstep(1.0, 0.9, d) * vis, n,
                                     rainCoord - c / mg, r / mg, 1.0,
                                     vis, vec2(0.0), vec2(0.0), vec2(0.0), 0.0, clamp(rim * lt, 0.0, 1.0));
                        }
                    }
                }
            }
        }

        // ===================================================================
        // 3. rivulets of this pixel's cells: the layer of gravity and, at speed, the one of the air
        // ===================================================================
#if WS_PROFILE != 2
        wsSetLayer(0);
        drawRivulet(hit, getRiv(sCell, sCW, sCH, gridSize), sUVg, wsRd1, wsRd2);
        if (wsWindShare > 0.001)
        {
            for (int wl = 1; wl <= wind_layers; ++wl)
            {
                vec2 wc = vec2(sCW * wind_cell_scale, sCH * ((wl == wind_layers) && (wl > 1) ? wind_long_run : wind_run_scale));
                wsSetLayer(wl);
                // every further layer: its cells shifted aside, drops of its own
                vec2 sUVw  = wsToLayer(sUVg) + fract(vec2(0.5, 0.618) * float(wl - 1)) * wc;
                Riv  rivW  = getRiv(floor(sUVw / wc), wc.x, wc.y, gridSize);
                drawRivulet(hit, rivW, sUVw, wsToLayerDir(wsRd1), wsToLayerDir(wsRd2));
            }
            wsSetLayer(0);
        }
#endif
    }

    // ---- the frame of the glass, the change of the view and the lens, for the shading (uniform flow again) ----
    vec2  sxy         = f_pos.xy / max(-f_pos.z, 1e-3);
    float pxPerSlope  = 1.0 / max(max(length(dFdx(sxy)), length(dFdy(sxy))), 1e-6);
    vec3  wsView  = normalize(f_pos.xyz);   // the view through the glass and its change per pixel (for the bent view)
    vec3  wsDvx   = dFdx(wsView);
    vec3  wsDvy   = dFdy(wsView);
    vec3  dp1     = dFdx(f_pos.xyz);
    vec3  dp2     = dFdy(f_pos.xyz);
    vec3  Tgl, Bgl;
    wsGlassFrame(Ngl, duv1, duv2, Tgl, Bgl);
    // glass UV offset -> screen pixels (inverse of the UV Jacobian), for the drop lens
    mat2  Juv  = mat2(duv1, duv2);
    float detJ = determinant(Juv);
    mat2  Jinv = (abs(detJ) > 1e-20) ? inverse(Juv) : mat2(0.0);

    // -----------------------------------------------------------------------
    // Shading of the winning dome
    // -----------------------------------------------------------------------
    float coverA = clamp(hit.cov, 0.0, 1.0);
    vec3  dropLayer = vec3(0.0);
    float alphaOut  = 0.0;
    vec3  dropGlint = vec3(0.0);
    vec2  lensBend  = vec2(0.0);
    float lensK     = 0.0;     // drops: how much the lens shrinks the picture
    float lensRadPx = 1.0e4;   // drops: radius on screen, pixels
    if (coverA > 0.001)
    {
        vec3  Nt   = hit.n;
        // look of the drop: grey atlas (bright rim, dark middle) for drops and rivulets, analytic for droplets
        vec3  rgb;
        float colA;
        if (hit.atlas > 0.5)
        {
            vec4 at = textureGrad(raindropsatlas, hit.auv, hit.agx, hit.agy);
            rgb  = at.rgb;
            colA = smoothstep(0.3, 0.9, at.a) * hit.vis;
        }
        else
        {
            rgb  = vec3(hit.shade);
            colA = hit.colA;
        }
        float a = colA * mix(drop_center_alpha, drop_rim_alpha, smoothstep(0.1, 0.3, length(rgb)));
        dropLayer = rgb * dynBright * a;
        alphaOut  = clamp(a, 0.0, 1.0);

        // refraction: Snell for rivulets (and as a fallback), lens picture for drops
        vec3 I  = normalize(f_pos.xyz);
        vec3 No = (dot(Ngl, I) > 0.0) ? Ngl : -Ngl;
        vec3 Nd = normalize(Tgl * Nt.x + Bgl * Nt.y + No * Nt.z);
        vec3 R  = refract(I, -Nd, drop_ior);
        if (dot(R, R) < 1e-4)
            R = refract(I, -Nd, 1.0 + 0.5 * (drop_ior - 1.0));
        vec2 dSlope = vec2(0.0);
        if (dot(R, R) > 1e-4 && R.z < -0.05)
            dSlope = R.xy / (-R.z) - I.xy / (-I.z);
        vec2 snellBend = clamp(-(dSlope * pxPerSlope * drop_refract) / WS_REFR_MAXPX, vec2(-1.0), vec2(1.0));
        lensBend = snellBend;
        if (hit.isDrop > 0.5 && drop_lens_mix > 0.0)
        {
            vec2  offPx = Jinv * hit.offUV;                         // pixel - drop centre, in screen pixels
            float radPx = hit.radUV / max(uvPerPx, 1e-6);
            float k     = clamp(min(drop_lens, 0.95 * WS_REFR_MAXPX / max(radPx, 1.0) - 1.0), 0.0, drop_lens);
            vec2  lb    = clamp((1.0 + k) * offPx / WS_REFR_MAXPX, vec2(-1.0), vec2(1.0));
            lensBend    = mix(snellBend, lb, drop_lens_mix);
            lensK       = k;
            lensRadPx   = radPx;
        }

        vec3 Rd = (dot(R, R) > 1e-4) ? normalize(R) : I;
        vec3 glint = vec3(0.0);
        if (lights_count > 0U)
        {
            float cosS = max(dot(Rd, normalize(-lights[0].dir)), 0.0);
            float lobe = pow(cosS, drop_glint_sharpness) * drop_glint_strength
                       + pow(cosS, 0.2 * drop_glint_sharpness) * drop_sheen;
            // the shadow (shadow map + GL 4.6 contact shadow ray) only where the sun shows in the drop at all
            if (lobe > 1e-4)
            {
                float shadowSun = 0.0;
                if (shadow_tone < 1.0)
                    shadowSun = (1.0 - shadow_tone) * clamp(calc_shadow(), 0.0, 1.0);
                // GL 4.6: the clouds hold the sun back where their shadow falls (as in apply_lights)
                float cloudSun = cloud_shadow(mat3(inv_view) * f_pos.xyz);
                glint += lights[0].color * lights[0].intensity * lobe * (1.0 - shadowSun) * cloudSun * wsNight;
            }
        }
        // GL 4.6: headlights and lamps come from the light cluster of the fragment (lights_clustered), or are the
        // first lights_count - 1 headlights otherwise. calc_headlights takes the screen-space change of f_pos
        // (worked out at the top of main, in uniform flow). The drop scatters light from both sides of the glass,
        // so the glass normal is turned towards the light.
        // only the lights outside the glass and a few metres away light the drops up (signals, lamps of the station, the
        // headlights of other trains): the lamps and the headlights of the own vehicle, a metre or two from the glass, would
        // make every drop glow white
        vec3 lamps = vec3(0.0);
        if (lights_clustered != 0U)
        {
            cluster_range_s range = cluster_ranges[fragment_cluster()];
            for (uint i = 0U; i < range.count; i++)
            {
                light_s lt = lights[cluster_light_indices[range.offset + i]];
                vec3  toL = lt.pos - f_pos.xyz;
                float away = smoothstep(0.0, 0.2, dot(toL, wsN) / max(length(toL), 1e-4))
                           * smoothstep(drop_light_near, 2.0 * drop_light_near, length(toL));
                if (away <= 0.0)
                    continue;
                vec3  nl = (dot(Ngl, toL) > 0.0) ? Ngl : -Ngl;
                if (lt.type == LIGHT_HEADLIGHTS)
                    lamps += lt.color * calc_headlights(lt, nl, dp1, dp2).x * lt.intensity * drop_headlight_scatter * away;
                else if (drop_lamp_scatter > 0.0)
                    lamps += lt.color * calc_local_light(lt, nl).x * lt.intensity * drop_lamp_scatter * away;
            }
        }
        else
        {
            for (uint i = 1U; i < lights_count; i++)
            {
                light_s lt = lights[i];
                vec3  toL = lt.pos - f_pos.xyz;
                float away = smoothstep(0.0, 0.2, dot(toL, wsN) / max(length(toL), 1e-4))
                           * smoothstep(drop_light_near, 2.0 * drop_light_near, length(toL));
                if (away <= 0.0)
                    continue;
                vec3  nl = (dot(Ngl, toL) > 0.0) ? Ngl : -Ngl;
                lamps += lt.color * calc_headlights(lt, nl, dp1, dp2).x * lt.intensity * drop_headlight_scatter * away;
            }
        }
        glint += min(lamps, vec3(drop_light_max));
        // reflection of the surroundings on the dome: straight from the environment map. (It used to go through a
        // second apply_lights, which in GL 4.6 also runs the shadow map, the contact shadows and the whole light
        // cluster again for every drop pixel; the sun and the lamps are done above already.)
        if (drop_env > 0.0)
        {
            vec3  Nv  = -Nd;                                   // dome normal towards the viewer
            float NoV = clamp(dot(Nv, -I), 0.0, 1.0);
            glint += envmap_color_lod(Nv, 0.8) * EnvBRDFApprox(vec3(0.04), 0.1, NoV) * drop_env * wsNight;
        }
#if WS_PROFILE == 1
        glint = vec3(0.0);
#endif
        dropGlint = glint;
        if (!POSTFX_ENABLED)
            dropGlint = min(dropGlint, vec3(1.0));   // this path is not tonemapped
    }

    // -----------------------------------------------------------------------
    // What is seen through the wet glass: the image of the last frame, blurred by the haze, bent by the drops
    // -----------------------------------------------------------------------
    // the haze after the wiper: grows with the rain fallen since, up to the intensity of the rain (as it always was where no wiper reaches)
    float wipeRain = wsWipeRain(f_coord);
    float hazeBack = (wipeRain < 0.0) ? rain_state.y
                   : min(rain_state.y, pow(clamp(wipeRain / haze_fill, 0.0, 1.0), haze_curve));
    // up to the full haze in any rain, as the glass gets wet (a lighter rain only mists it up more slowly), less as it dries
    float hazeWeight = hazeBack * (1.0 - max(alphaOut, coverA));
    float glassClear = 1.0 - smoothstep(0.5, 0.95, glassA);
    bool  image      = ssr_params.x >= 1.0;
    float background = 0.0;                      // distance to the scene behind, along the view (0: far / not known)
    if (image)
    {
        ivec2 size  = textureSize(scene_depth, 0);
        float depth = texelFetch(scene_depth, clamp(ivec2(gl_FragCoord.xy), ivec2(0), size - 1), 0).r;
        if (depth > 0.0)
            background = (projection[3][2] / depth) / max(-wsView.z, 0.05);
    }
    vec4 under = vec4(0.0);                      // the blurred view under the drops (not premultiplied)
    vec4 bent  = vec4(0.0);                      // the view bent by the drops, laid over everything (as the old postfx did)

    // the haze is blurred by the post-processing (postfx_taa.frag / postfx_motionblur.frag, rain_windscreen_haze.glsl): the
    // picture of this frame can't be read here, and one of the last frame laid over it only shows a milky ghost of the scene
    float blurAmt = clamp(hazeWeight * blur_amount, 0.0, 1.0);

    // the drops: the view bent (the picture shift worked out above), and the lights near them. with the motion vectors the
    // post-processing bends the picture of this frame (postfx_taa.frag / postfx_motionblur.frag, rain_windscreen_haze.glsl): it has
    // the depth of the cab and never pulls the dashboard or the roof into a drop. the image of the last frame is only the fallback
    float refrA = smoothstep(0.05, 0.35, coverA) * glassClear * step(glassA, 0.5);
    if ((false == MOTIONBLUR_ENABLED) && (refrA > 0.001))
    {
        vec2  shiftPx = -lensBend * WS_REFR_MAXPX;                         // where the drop looks, screen pixels
        vec3  dir     = normalize(wsView + shiftPx.x * wsDvx + shiftPx.y * wsDvy);
        // a drop shows a minified picture: read it from a coarser level (no grain); tiny drops a little blurrier still
        float lod     = clamp(log2(1.0 + lensK * drop_lens_mix) - 0.5, 0.0, 3.0)
                      + clamp(2.5 - log2(max(lensRadPx, 1.0)), 0.0, 1.5);
        float share;
        vec3  seen    = wsBehind(dir, lod, background, share, true);
        if (image && (WS_LIGHT_GAIN > 0.0))
            seen += wsLightGlow(wsView, background);
        // without the image the surroundings stand in: only faintly, they aren't the scene behind
        bent = vec4(seen, refrA * WS_REFR_MIX * (image ? 1.0 : 0.3));
    }

    // the drop shading over it
    float shadeA = coverA * drop_shade_alpha;
    float outA   = alphaOut + shadeA * (1.0 - alphaOut);
    vec4  drops  = vec4(dropLayer, outA);
    drops.rgb += dropGlint * coverA / max(outA, 0.05);   // the blend multiplies by alpha once more
    out_color = wsOver(drops, under);

    { // windshield texture over the rain
        vec3 fragcolor = ambient;
        // where the glass texture is clear, its lighting is multiplied by ~0 below: skip apply_lights there
        if (glassA > 0.002)
        {
            vec3 fragnormal = normalize(f_normal);
            float reflectivity = param[1].z;
            float specularity = (tex_color.r + tex_color.g + tex_color.b) * 0.5;
            legacy_gloss();
            fragcolor = apply_lights(fragcolor, fragnormal, tex_color.rgb, reflectivity, specularity, shadow_tone);
        }
        vec4 color = vec4(fragcolor, glassA);
        out_color.xyz = apply_fog(mix(out_color.xyz, color.xyz, color.a));
        out_color.a = mix(out_color.a, 1., color.a);
    }
    // the picture in the drops goes over the drop shading and the glass, as the old postfx replaced the pixel with it: the dark
    // middle of the atlas only shows through faintly, the rim and the glints a little more
    out_color = wsOver(bent, out_color);
    if (!POSTFX_ENABLED)
        out_color = tonemap(out_color);

    // the velocity target: on the drops the picture shift (x, y = WS_DROP_BIAS + WS_BEND_ENC * bend), on the hazy glass the request
    // for the haze (x = WS_ENC * strength, y = 0); elsewhere the velocity of the scene behind the glass stays as it is (alpha 0)
    if (MOTIONBLUR_ENABLED)
    {
        vec4 req = vec4(0.0);
        bool drop = (coverA > 0.05 || alphaOut > 0.05) && (glassA < 0.5);
        float blurMask = blurAmt * glassClear;
        if (drop)
        {
            vec2 bend = lensBend * smoothstep(0.05, 0.35, coverA);
            req = vec4(WS_DROP_BIAS + WS_BEND_ENC * bend.x, WS_DROP_BIAS + WS_BEND_ENC * bend.y, 0.0, 1.0);
        }
        else if (blurMask >= 0.01)
            req = vec4(WS_ENC * blurMask, 0.0, 0.0, 1.0);
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

    // "regeneration", raindrops gradually returning after wiper pass:
    // the share of the drops back: the glass fills with the rain fallen since the blade (rain_state), up to as wet as the glass is
    // where no blade goes (the drops of a cell are there only up to that share, so the share of those back is the fill over it)
    vec4 at         = wsWipePass(mask);
    vec4 fallen     = vec4(rain_state.x) - at;
    vec4 known      = step(vec4(0.), at);
    vec4 filled     = mix(vec4(1.), vec4(1.) - exp(-max(fallen, vec4(0.)) * rain_state.z), known);
    vec4 regenPhase = clamp(filled / max(rain_state.y, 1e-4), vec4(0.), vec4(1.));

    side_v = mix(vec4(0.), vec4(1.) - side_v, areaMask);

    vec4 factor_v = mix(vec4(1.), regenPhase * (vec4(1.) - cleaned), areaMask);

    side = 0.;
    float out_factor = 1.;
    bool  have_side = false;
    // out_factor: the blade that clears the cell the most wins. side: which wiper stroke the drops of this
    // cell belong to (part of their random seed, so it must not depend on out_factor).
    for (int i = 0; i < 4; ++i)
    {
        out_factor = min(out_factor, factor_v[i]);
        if (!have_side && areaMask[i] > 0.)
        {
            side = side_v[i];
            have_side = true;
        }
    }
    return out_factor;
}
