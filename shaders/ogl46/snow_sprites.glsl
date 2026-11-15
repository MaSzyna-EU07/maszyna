// the snow on the windscreens laid together from the sprites of the atlas (mat_snow_windscreen.frag, water_snowlayer.comp): the
// tuning of the look of the snow and the laying of the lumps, the dots and the drops. the including file declares the samplers
// raindropsatlas and normalmap (the atlas of the snow and its normal map) and includes random.glsl and water.glsl before

#ifndef SNOW_SPRITES_GLSL
#define SNOW_SPRITES_GLSL

// --- tuning ------------------------------------------------------------------------------------------------------------------
const float snow_flake_cell = 0.007; // a dot, a small puff or a drop at most every this many metres of the glass (its size)
const float snow_crystals   = 0.35;  // share of the cells with a little dry snow showing a dot or a small puff
const float snow_drops      = 0.05;  // share of the texels with the water of a melted flake showing a drop (more with more water)
const float snow_deep       = 2.5;   // mm of snow from which the cover is opaque (no picture behind it)
const float snow_cover      = 0.25;  // the lumps get opaque from three times this many mm of snow
const float snow_lump_cell  = 0.016; // a lump at most every this many metres of the glass (its size: up to two of them)
const vec2  snow_lump_grow  = vec2(0.75, 0.88); // a lump's size as it comes .. on thick snow, of its size (small gaps stay open)
const float snow_heap_cells = 4.0;   // the heaps the lumps come in, cells of the lumps (bigger: bigger heaps)
const float snow_opaque     = 1.8;   // mm of fresh snow letting 1/e of the light through (packed: half of it)
const float snow_ice        = 0.6;   // opacity of a lump until the snow gets thick (icy: the picture shows through, bent)
const vec3  snow_ice_tint   = vec3(0.93, 0.97, 1.0);   // the colour of the icy snow
const float snow_thin       = 0.45;  // opacity of a single thin flake (ice crystals let most of the light through)
const float snow_bump       = 1.0;   // strength of the relief from the normal map, 0 = flat
const float snow_gloss      = 0.8;   // glints of the sun / headlights on the wet snow, 0 = matt
const float snow_ice_bend   = 1.0;   // how strongly the icy snow bends the picture behind it, of the bend of a drop
const float snow_translucency = 0.8; // glow of thin snow when you look towards the sun / a lamp, 0 = off
const float snow_haze       = 1.0;   // blur of the scene behind thin snow (frosted glass), 0 = off
const float snow_drop_bend  = 1.0;   // how strongly the drops bend the picture behind them
const float snow_wet_blur   = 0.6;   // blur of the scene behind a wet film
const vec2  snow_blur_mm    = vec2(0.05, 0.6); // the scene behind the snow blurred from .. fully at this many mm of it (as the rain
                                               // mists the glass up: the gaps between the lumps too, clear again behind a blade)

const float WS_REFR_MAXPX = 96.0;    // the largest picture shift the post-processing makes, screen pixels; as rain_windscreen_haze.glsl
const float WS_BEND_ENC   = 60.0;    // the bend request in the velocity target; as rain_windscreen_haze.glsl
const float WS_DROP_BIAS  = 192.0;
const float WS_ENC        = 64.0;    // the haze request in the velocity target; as rain_windscreen_haze.glsl

const float SNOW_FRESH  = 0.1;       // water in a mm of fresh snow, mm (as water_snow.comp)
const float SNOW_PACKED = 0.4;       // and of packed snow

// tiles of the atlas
const int TILE_DROP = 15;

vec2  snUV   = vec2(0.0); // the place on the glass laid, uv
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
// the rows of the tiles counted from the top of the picture of the atlas (as snieg_atlas_maszyna.py makes it), the v of the
// texture from its bottom: the first row is at the top of the texture (the tiles of the old 2 x 2 atlas the same way)
vec2 snTile(int tile)
{
    if (snBig)
        return vec2(float(tile % 4), float(3 - tile / 4)) * 0.25;
    int old = 0;                                  // a fluffy flake
    if ((tile == 6) || (tile == 7))
        old = 2;                                  // crumbs
    else if ((tile >= 8) && (tile != 10) && (tile != 11) && (tile != TILE_DROP))
        old = 1;                                  // a wet lump
    else if (tile == TILE_DROP)
        old = 3;                                  // a melted spot
    return vec2(float(old % 2), float(1 - old / 2)) * 0.5;
}

float snTileSize()
{
    return snBig ? 0.25 : 0.5;
}

// a sprite over the accumulator: Local the place in it (0 .. 1), Rot its rotation (cos, sin), Tone its colour
void snSprite(vec2 local, int tile, float alpha, vec3 tone, vec2 rot, float flip, float lod, float bend, inout sprite_acc acc)
{
    vec2 uv = snTile(tile) + local * snTileSize();
    vec4 t = textureLod(raindropsatlas, uv, lod);
    vec4 n = textureLod(normalmap, uv, lod);
    float a = clamp(t.a * alpha, 0.0, 1.0);
    if (a <= 0.002)
        return;
    vec2 nxy = (n.rg * 2.0 - 1.0) * snow_bump;
    nxy.x *= flip;
    vec2 nuv = vec2(rot.x * nxy.x - rot.y * nxy.y, rot.y * nxy.x + rot.x * nxy.y);
    vec3 nT = vec3(nuv, sqrt(max(1.0 - dot(nuv, nuv), 0.02)));
    // front to back: the sprites come from the top one down (the same picture as laying them back to front), so the pixel stops
    // looking once the snow over it covers it
    float w = a * (1.0 - acc.a);
    acc.rgb += t.rgb * tone * w;
    acc.n   += nT * w;
    acc.tr  += (1.0 - t.a * 0.55) * w;
    acc.a   += w;
    // all the snow bends the picture behind it (wet icy snow, slush, drops), as the drops of the rain; the drops and the slush
    // the most
    float bends = (((tile == TILE_DROP) || (tile == 8) || (tile == 9) || (tile == 14)) ? 1.0 : snow_ice_bend) * bend;
    float wb = a * (1.0 - snBendA);
    snBend  += nuv * bends * wb;
    snBendA += wb;
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
    vec3 tone = vec3(mix(0.94, 1.04, random)) * snow_ice_tint;
    tone *= mix(vec3(1.0), vec3(0.95, 0.96, 0.98), q.z);
    return tone * mix(vec3(1.0), vec3(0.88, 0.91, 0.94), smoothstep(0.3, 0.8, snWetness(q)));
}

// the dots and the drops, in cells of snow_flake_cell metres of the glass: in many of the cells holding a little dry snow a dot
// or a small puff sits between the lumps, in some of those holding only water a drop of a melted flake
void snFlakes(vec2 span, float pixelUV, inout sprite_acc acc)
{
    vec2 perUV = span / snow_flake_cell;                // cells in a unit of the uv
    vec2 p = snUV * perUV;
    ivec2 here = ivec2(floor(p));
    for (int oy = 1; oy >= -1; --oy)
    {
        for (int ox = 1; ox >= -1; --ox)
        {
            ivec2 cell = here + ivec2(ox, oy);
            uint seed = Hash(uvec3(uvec2(cell + 8192), 17u + 7u * uint(snSlot)));
            vec2 centre = vec2(cell) + 0.15 + 0.7 * vec2(RandF(seed), RandF(seed));
            // every draw of the cell first, in the order they always came, so the flake doesn't change: the map is the costly
            // part, so it is read only for a flake which, as big as it gets, covers the pixel
            float chance = RandF(seed);
            float ang = RandF(seed) * 6.2831853;
            vec2 rot = vec2(cos(ang), sin(ang));
            float flip = (RandF(seed) < 0.5) ? -1.0 : 1.0;
            float kind = RandF(seed);
            float random = RandF(seed);
            float sizeDraw = mix(0.6, 1.0, RandF(seed));
            vec2 local;
            if (false == snLocal(p, centre, sizeDraw * 1.4, vec2(1.0), rot, flip, 1.0, local))
                continue;
            vec4 q = water_snow_at(uint(snSlot), snMap, centre / perUV);
            bool drop = (q.x < 0.05) && (q.y > 0.03);
            bool crystal = (q.x >= 0.05);
            if ((false == drop) && (false == crystal))
                continue;
            if (chance > (drop ? snow_drops * smoothstep(0.03, 0.2, q.y) : snow_crystals))
                continue;
            float size = sizeDraw * (drop ? mix(0.7, 1.4, smoothstep(0.03, 0.3, q.y)) : 1.0);
            if (false == snLocal(p, centre, size, vec2(1.0), rot, flip, 1.0, local))
                continue;
            int tile = TILE_DROP;
            float alpha = smoothstep(0.03, 0.1, q.y);
            if (crystal)
            {
                tile = 2 + min(int(kind * 2.0), 1);                // small icy puffs
                alpha = snow_thin;
            }
            snSprite(local, tile, alpha, snTone(q, random), rot, flip, snLod(pixelUV, size / max(perUV.x, perUV.y)), 1.0, acc);
        }
    }
}

// how much a cell of the lumps lies in a heap: the snow landing side by side runs together, so the lumps come in heaps with
// barer glass between them; a random number for every few cells, blended between them (the same for every pixel of the cell)
float snHeap(ivec2 cell)
{
    vec2 c = (vec2(cell) + 0.5) / snow_heap_cells;
    ivec2 i = ivec2(floor(c - 0.5));
    vec2 f = c - 0.5 - vec2(i);
    f = f * f * (3.0 - 2.0 * f);
    uint s00 = Hash(uvec3(uvec2(i + 8192), 53u));
    uint s10 = Hash(uvec3(uvec2(i + ivec2(1, 0) + 8192), 53u));
    uint s01 = Hash(uvec3(uvec2(i + ivec2(0, 1) + 8192), 53u));
    uint s11 = Hash(uvec3(uvec2(i + ivec2(1, 1) + 8192), 53u));
    return mix(mix(RandF(s00), RandF(s10), f.x), mix(RandF(s01), RandF(s11), f.x), f.y);
}

// the lumps, in grids of cells of snow_lump_cell metres of the glass, shifted against each other. every cell of a grid has its
// lump fixed once (its tile, place, turn, size and the snow it takes to be there): the lump comes when the snow of the map around
// it reaches that, grows a little with more snow, and stays as it is until the snow goes (a blade, the melting); it never turns
// into another lump. the grids come one after the other as the snow piles up: dots and crumbs first, then puffs, clumps, and last
// the big lumps of packed snow which close the glass. in a heap (the snow landing side by side runs together) they come sooner
struct lump_grid
{
    vec2 shift;      // the grid shifted, cells
    uint salt;
    vec2 snow;       // the snow it takes for a lump of the grid to come, mm: from .. to (random per cell)
    vec2 size;       // the size of its lumps, cells: from .. to (random per cell)
    int tiles;       // its tiles: the first one and how many
    int count;
    float share;     // the share of its cells having a lump at all (1: all of them)
};

const float SN_COVERED = 0.998;   // the snow over the pixel covering it: the sprites under it are not looked at

void snLumps(vec2 span, float pixelUV, lump_grid grid, inout sprite_acc acc)
{
    if (acc.a >= SN_COVERED)
        return;   // the grids over this one cover the pixel
    vec2 perUV = span / snow_lump_cell;
    vec2 p = snUV * perUV + grid.shift;
    ivec2 here = ivec2(floor(p));
    for (int oy = 1; oy >= -1; --oy)
    {
        for (int ox = 1; ox >= -1; --ox)
        {
            if (acc.a >= SN_COVERED)
                return;
            ivec2 cell = here + ivec2(ox, oy);
            uint seed = Hash(uvec3(uvec2(cell + 4096), grid.salt + 7u * uint(snSlot)));
            vec2 centre = vec2(cell) + 0.1 + 0.8 * vec2(RandF(seed), RandF(seed));
            // every draw of the cell first, in the order they always came, so the lump doesn't change: the map is the costly
            // part, so it is read only for a lump which, as big as it grows, covers the pixel (most of the nine cells don't)
            float needsDraw = RandF(seed);
            if (RandF(seed) >= grid.share)
                continue;
            float ang = RandF(seed) * 6.2831853;
            vec2 rot = vec2(cos(ang), sin(ang));
            float flip = (RandF(seed) < 0.5) ? -1.0 : 1.0;
            int tile = grid.tiles + min(int(RandF(seed) * float(grid.count)), grid.count - 1);
            float random = RandF(seed);
            float aniso = mix(0.8, 1.0, RandF(seed));
            float sizeDraw = mix(grid.size.x, grid.size.y, RandF(seed));
            vec2 local;
            if (false == snLocal(p, centre, sizeDraw * snow_lump_grow.y, vec2(1.0), rot, flip, aniso, local))
                continue;
            // the snow it takes, sooner in a heap
            float needs = mix(grid.snow.x, grid.snow.y, needsDraw) * mix(1.4, 0.6, snHeap(cell));
            vec4 q = water_snow_at(uint(snSlot), snMap, (centre - grid.shift) / perUV);
            if (q.x <= needs)
                continue;
            float size = sizeDraw * mix(snow_lump_grow.x, snow_lump_grow.y, smoothstep(needs, needs * 2.0 + 0.2, q.x));
            if (false == snLocal(p, centre, size, vec2(1.0), rot, flip, aniso, local))
                continue;
            // fading in over the first snow (no flicker on the edge), opaque when thick; the thick snow bends nothing
            float deep = smoothstep(snow_cover * 3.0, snow_deep, q.x);
            float alpha = smoothstep(needs, needs + 0.04, q.x) * mix(snow_ice, 1.0, deep);
            snSprite(local, tile, alpha, snTone(q, random), rot, flip, snLod(pixelUV, size / max(perUV.x, perUV.y)), 1.0 - deep, acc);
        }
    }
}

// the snow of the glass laid together at UV: the flakes over the lumps (water_snowlayer.comp bakes it into the layer of the glass,
// mat_snow_windscreen.frag reads it). Span: metres of the glass in a unit of its uv; PixelUV: the uv a texel of the layer covers
// (the level of the atlas). snSlot, snMap and snBig set before
struct snow_layer_s
{
    vec3  rgb;    // premultiplied
    float a;
    vec3  n;      // premultiplied, tangent space
    float tr;     // translucency, premultiplied
    vec2  bend;   // the bend of the picture behind, weighted
    float bendA;  // its weight
    float haze;   // the blur of the picture behind the thin snow
};

snow_layer_s snCompose(vec2 uv, vec2 span, float pixelUV)
{
    snUV = uv;
    snBend = vec2(0.0);
    snBendA = 0.0;
    sprite_acc layer = sprite_acc(vec3(0.0), 0.0, vec3(0.0), 0.0);
    sprite_acc flakes = sprite_acc(vec3(0.0), 0.0, vec3(0.0), 0.0);
    // the grids of the lumps, coming one after the other as the snow piles up, each on the ones before: all of them icy
    // fleece (small puffs and icy clumps, tiles 2-5), more and bigger, closing the glass by 2-3 mm; now and then a single
    // lump of packed snow on top. laid front to back: the flakes, the single packed lumps, then the grids from the top one
    // down, so a texel stops looking once the snow over it covers it
    snFlakes(span, pixelUV, flakes);
    snLumps(span, pixelUV, lump_grid(vec2(0.4, 0.6), 271u, vec2(0.6, 2.0), vec2(1.2, 1.6), 12, 2, 0.03), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.75), 251u, vec2(1.4, 2.8), vec2(2.0, 3.0), 4, 2, 1.0), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.0, 0.5), 233u, vec2(1.2, 2.4), vec2(2.0, 3.0), 4, 2, 1.0), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.5, 0.0), 199u, vec2(1.0, 2.0), vec2(2.0, 3.0), 4, 2, 1.0), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.75, 0.25), 151u, vec2(0.7, 1.6), vec2(1.8, 2.8), 4, 2, 1.0), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.25, 0.75), 113u, vec2(0.4, 1.2), vec2(1.8, 2.8), 4, 2, 1.0), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.25), 97u, vec2(0.2, 0.9), vec2(1.4, 2.4), 2, 4, 1.0), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.5), 77u, vec2(0.15, 0.7), vec2(1.4, 2.4), 2, 4, 1.0), layer);
    snLumps(span, pixelUV, lump_grid(vec2(0.0), 31u, vec2(0.02, 0.35), vec2(1.2, 2.0), 2, 4, 1.0), layer);
    snow_layer_s s;
    // the flakes over the layer
    s.rgb = flakes.rgb + layer.rgb * (1.0 - flakes.a);
    s.a   = flakes.a + layer.a * (1.0 - flakes.a);
    s.n   = flakes.n + layer.n * (1.0 - flakes.a);
    s.tr  = flakes.tr + layer.tr * (1.0 - flakes.a);
    s.bend = snBend;
    s.bendA = snBendA;
    // the picture behind thin flakes is blurred
    s.haze = smoothstep(0.0, 0.3, flakes.a + layer.a) * (1.0 - smoothstep(0.5, 0.9, layer.a)) * 0.6;
    return s;
}

#endif
