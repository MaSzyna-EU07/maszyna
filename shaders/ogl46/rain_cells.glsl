// the cells of the glasses with rain (mat_rain_windscreen.frag): the drops of the grid of drops and the rivulets of the layers of
// rivulets, worked out once a frame for every cell by rain_cells.comp (a thread a cell) instead of in every pixel of a glass for
// its 9 neighbouring cells. a cell holds the random values of its drop or rivulet as they are now (the hashes of the cell and of
// its epoch, the life cycle at the time of the frame); the glass takes the sizes and the places from them with its own settings.
// so the glass looks the same as when it worked them out itself (the hashes are the same; the values are kept to 8 or 16 bits)
//
// a cell is one uvec4. the drops: RAIN_DROP_CELLS x RAIN_DROP_CELLS cells, cell c of the grid in element (c.y & 511) * 512 +
// (c.x & 511), whole for the cells -1 .. 510 (grids up to 510 cells; finer ones repeat their drops every 512 cells):
//   x  side 0 (the drop of a cell the blades went over on their way out): bits 0-7 the threshold the wipers have to be past for
//      it, 8-15 / 16-23 the place in the cell (x, y), 24-31 its size (0 .. 1 from drop_size_min to drop_size_max)
//   y  side 0: bits 0-7 the random of its sag, 8-15 of its stretch by the air, 16 mirrored, 17-18 the tile of the atlas,
//      19-23 how far it spread out landing (0 .. 31), 24-31 the random a cell holds a drop by (a drop when below the share)
//   z  side 1 (the blades on their way back): as x
//   w  side 1: as y, bit 19 set when the blades miss the drop, bits 24-31 its visibility in its life (0 .. 255)
// the reach of the drops: for every drop cell (element RAIN_MASK_BASE + the index of the drop cell) the drops of the 9 cells around
// it (bit (dy + 1) * 3 + dx + 1) which reach into each quarter of it (x the lower left, y the lower right, z the upper left, w the
// upper right), with the drops the cells hold now in the rain there is now (a glass only looks at those)
// the rivulets: RAIN_RIV_LAYERS layers (0 gravity, 1 .. 4 the air) of RAIN_RIV_CELLS x RAIN_RIV_CELLS cells, cell c in element
// RAIN_RIV_BASE + layer * 128 * 128 + (c.y & 127) * 128 + (c.x & 127), whole for the cells -32 .. 95:
//   x  bits 0-15 the random the cell holds a rivulet by, 16-23 its life (0 .. 255), 24-31 the random of the wipers it stays under
//   y  bits 0-15 how far down its run the head is (0 .. 65535), 16-23 the random of its width, 24-31 of its span
//   z  the randoms of its top, its left edge, the length and the phase of its waves (a byte each)
//   w  bits 0-7 the random of the speed of its waves, 8 mirrored, 9-10 the sprite of the atlas

#ifndef RAIN_CELLS_GLSL
#define RAIN_CELLS_GLSL

#define RAIN_DROP_CELLS 512
#define RAIN_RIV_CELLS 128
#define RAIN_RIV_LAYERS 5
#define RAIN_RIV_BASE (RAIN_DROP_CELLS * RAIN_DROP_CELLS)
#define RAIN_MASK_BASE (RAIN_RIV_BASE + RAIN_RIV_LAYERS * RAIN_RIV_CELLS * RAIN_RIV_CELLS)
#define RAIN_CELLS_COUNT (RAIN_MASK_BASE + RAIN_DROP_CELLS * RAIN_DROP_CELLS)
#define RAIN_CELLS_GROUP_SIZE 64

#if defined(STAGE_COMPUTE)
layout(std430, binding = RAIN_CELLS_SSBO_BINDING) buffer rain_cells_ssbo
#else
layout(std430, binding = RAIN_CELLS_SSBO_BINDING) readonly buffer rain_cells_ssbo
#endif
{
	uvec4 rain_cells[];
};

// the drops and the rivulets (the settings of mat_rain_windscreen.frag which the cells are worked out with)
const float drop_density       = 0.6;   // share of grid cells holding a drop at full rain (scaled by rain intensity)
const float drop_size_min      = 0.45;  // drop size in grid cells
const float drop_size_max      = 1.15;
const float drop_size_bias     = 1.8;   // >1 = more small/medium drops, fewer big ones
const float drop_sag           = 0.12;  // max vertical stretch of a resting drop (gravity)
const float drop_wind_stretch  = 0.4;   // drops are stretched up to this much along the push of the air (0 = off)
const float drop_life_min      = 14.0;  // seconds a drop sits on the glass
const float drop_life_max      = 45.0;
const float drop_appear        = 0.18;  // seconds of the landing "pop"
const float drop_fade          = 1.5;   // seconds a drop takes to vanish at the end of its life
const float wiper_miss         = 0.0167; // share of the drops a blade goes over without taking them (a third of the 0.05 it was)
const float streak_period      = 40.0;  // seconds a rivulet lives (it runs for ~90% of that and then fades out), average
const float wind_streak_period = 18.0;  // seconds a rivulet driven by the air lives (they run faster than the ones of gravity)

// a random kept to 8 bits, back to 0 .. 1 (the middle of its step)
float rain_byte(uint Value, uint Shift)
{
	return (float((Value >> Shift) & 255u) + 0.5) * (1.0 / 256.0);
}

uint rain_drop_index(ivec2 Cell)
{
	return uint(Cell.y & (RAIN_DROP_CELLS - 1)) * uint(RAIN_DROP_CELLS) + uint(Cell.x & (RAIN_DROP_CELLS - 1));
}

uint rain_mask_index(ivec2 Cell)
{
	return uint(RAIN_MASK_BASE) + rain_drop_index(Cell);
}

uint rain_riv_index(int Layer, ivec2 Cell)
{
	return uint(RAIN_RIV_BASE) + uint(Layer) * uint(RAIN_RIV_CELLS * RAIN_RIV_CELLS) + uint(Cell.y & (RAIN_RIV_CELLS - 1)) * uint(RAIN_RIV_CELLS)
	     + uint(Cell.x & (RAIN_RIV_CELLS - 1));
}

#endif
