#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// Windshield rain and snow
// by @lcddisplay; wiper movement @MichauSto; water on the glass rewritten as a model of drops lying on it
//
// the material: the textures diffuse (the glass) and wipermask (a blade per channel, r g b a: 0 where it doesn't reach, elsewhere
// the place of the blade over its sweep, 0 at rest .. 1 at the far end, at which it goes over the place). raindropsatlas and
// raindrop_grid_size aren't used any more, they stay so the materials which name them still load. rain_scale (1 when not given)
// scales the drops and the flakes. water_slot is given by the renderer (1 + the glass of the simulation, 0 none), not by the
// material. the block of the scene: rain_params (x: how heavy the rain or snow is, z: 1 for snow, w: temperature of the air),
// wiper_pos, wiper_timer_out, wiper_timer_return (the blades of the vehicle of the camera) and rain_wind (the air against it).
//
// the glasses of the cab of the train are simulated (gfx::windscreen_water, water_*.comp, water.glsl): the raindrops land as often
// as the rain hits the glass, splashing; the drops lie held by surface tension until gravity and the drag of the air pull them
// harder, then slide in stops and goes, wetting their paths (the next ones follow them into rivulets) and leaving droplets behind;
// the drops which touch flow together; the blades take the water along, gather it into a bead and leave it at the ends of their
// sweep, where it runs down; the flakes stick and melt. here the drops of the simulation are drawn where the fragment is, and the
// wetted paths as a thin film. elsewhere (the windows of other vehicles, a glass the simulation has no room for) the water is made
// up from the time alone: drops landing, growing and sliding along tracks, the blades pushing a ridge and leaving the water at the
// edge of their sweep.
//
// the water is a field of soft kernels (metaballs): drops coming close flow together, and a drop is a few kernels, so the drops
// are of many shapes. a drop is a lens holding a sharp picture of the place around it turned over (the image of the last frame
// where the screen holds it, the sky and the ground elsewhere), its edge shows the light of the sky reflected inside it; the lamps
// and the headlights light the water up; the film the blades leave breaks up behind them and the lights seen through it are drawn
// into lines towards their pivot; the wet glass is a little milky

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
#pragma param(color, 0, 0, 4, diffuse)
#pragma param(diffuse, 1, 0, 1, diffuse)
#pragma param(specular, 1, 1, 1, specular)
#pragma param(reflection, 1, 2, 1, zero)
#pragma param(glossiness, 1, 3, 1, glossiness)
#pragma param(raindrop_grid_size, 2, 0, 1, one)
#pragma param(rain_scale, 2, 1, 1, one)

#include "light_common.glsl"
#include "apply_fog.glsl"
#include "tonemapping.glsl"
#include "water.glsl"

layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 0) uniform sampler2D diffuse;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 1) uniform sampler2D raindropsatlas;
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 2) uniform sampler2D wipermask;

const float RW_PI = 3.14159265;
// refractive index of water
const float RW_IOR = 1.333;
// the falling particles (particles_precipitation.comp, rendering/common/gpuparticles.cpp): the diameters of the flakes, metres,
// and how fast the rain and the snow fall, m/s
const vec2 RW_SNOW_DIAMETER = vec2(0.004, 0.012);
const float RW_RAIN_FALL = 7.0;
const float RW_SNOW_FALL = 1.1;
// a drop is a lens: it holds a sharp picture of the place around it, turned over through its middle, RW_LENS times as far out
// as the fragment is from the middle (and as much smaller), with this share of the bending of the view by the surface on top
const float RW_LENS = 2.0;
const float RW_LENS_BEND = 0.15;
// the drag of the air going along the glass: the speed at which it equals the weight of a drop, m/s; and the share of the air
// meeting the glass head on which turns up along it. so the water goes up a glass the sooner the more it leans back: on a
// vertical glass at about 250 km/h, leaning 20 degrees at about 130 km/h, 45 degrees at about 80 km/h
const float RW_DRAG_SPEED = 24.0;
const float RW_HEAD_ON = 0.35;
// the water is a field summed from soft kernels (metaballs) and its surface is where the field is RW_SURFACE: drops which come
// close flow together the way surface tension pulls them. a kernel alone reaches the surface at RW_EDGE of its radius
const float RW_SURFACE = 0.25;
const float RW_EDGE = 0.61;

// the glass at the fragment: the view space steps along it for a step of the coordinates of the glass (metres along the axes
// of the model, see main()), and the size of a pixel on it, metres
vec3 rw_tu;
vec3 rw_tv;
float rw_pixel;

// how the water on the glass behaves here, set by main()
float rw_scale;  // size of the drops (param rain_scale)
float rw_since;  // seconds since a blade last cleaned the place
float rw_period; // seconds between the small drops landing on a place of one
float rw_grow;   // seconds the drops need after a wipe to grow big enough to slide
float rw_rate;   // how fast the glass gathers the rain against a standing vertical glass in the heaviest rain
float rw_push;   // the force on a drop along the glass, in units of its weight
vec2 rw_down;    // the way gravity pulls along the glass (coordinates of the glass, unit)
vec2 rw_flow;    // the way the air goes along the glass (unit, 0 in still air)

// the field of the water: the sum of the kernels, its gradient along the glass (view space) and the sums for the mean radius of
// the drops it is made of
float rw_field = 0.0;
vec3 rw_fgrad = vec3(0.0);
float rw_fsize = 0.0;
float rw_fweight = 0.0;

// the water at the fragment: the thickness of the highest water there (metres), its gradient (view space), the radius of the
// curvature of its surface (metres: how much it bends the view across a pixel) and how much of the pixel it covers
float rw_height = 0.0;
vec3 rw_slope = vec3(0.0);
float rw_curve = 1.0;
float rw_cover = 0.0;
// from the middle of the drop which gives the water to the fragment (view space, metres), 0 for spread water
vec3 rw_offset = vec3(0.0);
// the last time the water a blade pushes went over the fragment, taking the drops lying there with it
float rw_cleared = -1.0e9;

uvec4 rw_pcg4d(uvec4 v)
{
	v = v * 1664525u + 1013904223u;
	v.x += v.y * v.w;
	v.y += v.z * v.x;
	v.z += v.x * v.y;
	v.w += v.y * v.z;
	v ^= v >> 16u;
	v.x += v.y * v.w;
	v.y += v.z * v.x;
	v.z += v.x * v.y;
	v.w += v.y * v.z;
	return v;
}

// four numbers 0 .. 1 of a cell of a layer
vec4 rw_random(ivec2 cell, uint layer, uint index)
{
	return vec4(rw_pcg4d(uvec4(uvec2(cell), layer, index))) * (1.0 / 4294967296.0);
}

float rw_noise(float x, uint layer)
{
	float i = floor(x);
	float f = x - i;
	float a = rw_random(ivec2(int(i), 0), layer, 7u).x;
	float b = rw_random(ivec2(int(i) + 1, 0), layer, 7u).x;
	return mix(a, b, f * f * (3.0 - 2.0 * f));
}

// value noise 0 .. 1 (x) with its gradient (yz)
vec3 rw_noise2(vec2 p, uint layer)
{
	vec2 i = floor(p);
	vec2 f = p - i;
	vec2 u = f * f * (3.0 - 2.0 * f);
	vec2 du = 6.0 * f * (1.0 - f);
	ivec2 c = ivec2(i);
	float a = rw_random(c, layer, 8u).x;
	float b = rw_random(c + ivec2(1, 0), layer, 8u).x;
	float d = rw_random(c + ivec2(0, 1), layer, 8u).x;
	float e = rw_random(c + ivec2(1, 1), layer, 8u).x;
	float k1 = b - a;
	float k2 = d - a;
	float k4 = a - b - d + e;
	return vec3(a + k1 * u.x + k2 * u.y + k4 * u.x * u.y, du * vec2(k1 + k4 * u.y, k2 + k4 * u.x));
}

// P turned by A (radians). every set of drops is laid on its own grid, turned and shifted against the others, so the drops line
// up neither with each other nor along the glass
vec2 rw_turn(vec2 p, float a)
{
	float c = cos(a);
	float s = sin(a);
	return vec2(c * p.x - s * p.y, s * p.x + c * p.y);
}

// a gradient given along the coordinates of the glass, view space
vec3 rw_along(vec2 gradient)
{
	return rw_tu * (gradient.x / dot(rw_tu, rw_tu)) + rw_tv * (gradient.y / dot(rw_tv, rw_tv));
}

// the gradient along the glass (view space) of a value of the fragments, from how it changes from pixel to pixel; Dpdx, Dpdy:
// the steps of the position, Normal: of the glass
vec3 rw_gradient(float value, vec3 dpdx, vec3 dpdy, vec3 normal)
{
	vec3 r1 = cross(dpdy, normal);
	vec3 r2 = cross(normal, dpdx);
	float det = dot(dpdx, r1);
	float dx = dFdx(value);
	float dy = dFdy(value);
	return (abs(det) > 1e-14) ? (dx * r1 + dy * r2) / det : vec3(0.0);
}

// a kernel of the water, Offset (view space) from its centre to the fragment: alone it makes water of the radius Visible,
// Weight 1 .. 0 as it goes. water smaller than a pixel would only flicker, it is left to the haze of the wet glass
void rw_kernel(vec3 offset, float visible, float weight)
{
	if ((weight <= 0.0) || (visible < 0.5 * rw_pixel))
		return;
	// a drop a pixel or two across is grain rather than a lens: it fades into the haze of the wet glass
	weight *= smoothstep(0.5 * rw_pixel, 2.0 * rw_pixel, visible);
	float support = visible / RW_EDGE;
	float q = dot(offset, offset) / (support * support);
	if (q >= 1.0)
		return;
	float k = 1.0 - q;
	float k3 = weight * k * k * k;
	rw_field += k3;
	rw_fgrad += offset * (-6.0 * weight * k * k / (support * support));
	rw_fsize += k3 * visible;
	rw_fweight += k3;
}

// a kernel at the Offset in the coordinates of the glass
void rw_blob(vec2 offset, float visible, float weight)
{
	rw_kernel(offset.x * rw_tu + offset.y * rw_tv, visible, weight);
}

// a drop lying on the glass: a few kernels, sagging down the glass and lopsided as Shape (random 0 .. 1) says
void rw_drop(vec2 offset, float visible, float weight, vec4 shape)
{
	rw_blob(offset, visible, weight);
	if (visible < 1.5 * rw_pixel)
		return;
	vec2 side = vec2(-rw_down.y, rw_down.x);
	vec2 sag = (rw_down * (0.2 + 0.35 * shape.x) + side * ((shape.y - 0.5) * 0.7)) * visible;
	rw_blob(offset - sag, visible * mix(0.6, 0.85, shape.z), weight);
	if (shape.w > 0.4)
	{
		float angle = shape.w * 40.0;
		rw_blob(offset - vec2(cos(angle), sin(angle)) * (0.6 * visible), visible * mix(0.4, 0.6, shape.x), weight);
	}
}

// water spread over the glass (a film, a ridge): its thickness, the gradient of it (view space), the radius of the curvature of
// its surface and how much of the pixel it covers
void rw_sheet(float thickness, vec3 slope, float curve, float cover)
{
	if (cover <= 0.0)
		return;
	rw_cover = max(rw_cover, cover);
	if (thickness > rw_height)
	{
		rw_height = thickness;
		rw_slope = slope;
		rw_curve = curve;
		rw_offset = vec3(0.0);
	}
}

// the surface of the water of the field: a drop is a cap of a sphere, flat lens of the contact angle Contact (radians) the water
// makes with the glass. how far across a drop the fragment is follows from the field (inverting the kernel), the height and
// the slope of the cap from that: the slope rises slowly from the middle and steeply only close to the edge, where it reaches
// the contact angle; where drops flowed together the field is above that of a drop alone and the water is flat. the edge is
// antialiased by how fast the field changes across a pixel
void rw_settle(float contact)
{
	if (rw_field <= 0.5 * RW_SURFACE)
		return;
	float gradient = length(rw_fgrad);
	float cover = clamp((rw_field - RW_SURFACE) / max(gradient * rw_pixel, 1e-3) + 0.5, 0.0, 1.0);
	if (cover <= 0.0)
		return;
	float mean = rw_fsize / max(rw_fweight, 1e-12);
	// 0 in the middle of a drop, 1 at its edge
	float across = min(sqrt(max(1.0 - pow(min(rw_field, 1.0), 1.0 / 3.0), 0.0)) / RW_EDGE, 1.0);
	float s = sin(contact);
	float rs = across * s;
	float root = sqrt(max(1.0 - rs * rs, 1e-4));
	float sphere = mean / s;
	vec3 slope = (gradient > 1e-9) ? rw_fgrad * ((rs / root) / gradient) : vec3(0.0);
	float height = max(sphere * (root - cos(contact)), 0.0);
	bool top = height > rw_height;
	rw_sheet(height, slope, sphere, cover);
	// the field rises towards the middle of the drop
	if (top && (gradient > 1e-9))
		rw_offset = rw_fgrad * (-across * mean / gradient);
}

// the blades. a channel of the mask of the wipers is a blade: 0 where it doesn't reach, elsewhere the place of the blade over its
// sweep (0 at rest .. 1 at the far end) at which it goes over the place. wiper_pos: where the blade is (0 .. 1 on its way out,
// 1 + where it is on its way back), wiper_timer_out / _return: the last time it lay at its start / was at its far end
struct rw_wipe_s
{
	float since;   // seconds since a blade last went over the place (large where none reaches)
	float ahead;   // 0 .. 1, how close in front of a moving blade the place is (the water the blade pushes)
	float front;   // how far in front of the nearest moving blade the place is, units of the mask (1 where none)
	float swept;   // 0 .. 1, how much of its way the blade in front did, the water it gathered on the way
	float working; // 0 .. 1, the blades went lately
};

rw_wipe_s rw_wipe(vec4 mask)
{
	rw_wipe_s wipe = rw_wipe_s(1.0e4, 0.0, 1.0, 0.0, 0.0);
	for (int i = 0; i < 4; ++i)
	{
		float m = mask[i];
		if (m < 0.001)
			continue;
		float pos = wiper_pos[i];
		bool back = pos > 1.0;
		float blade = back ? pos - 1.0 : pos;
		float t0 = wiper_timer_out[i];
		float t1 = wiper_timer_return[i];
		// when the blade went over the place last: on this stroke, the blade moving at an even pace from its end, or on the
		// stroke before, which took as long as the time between the ends (if the blade lay at its end longer, two seconds)
		float pass;
		float front; // how far in front of the blade the place is (units of the mask), below 0 behind it
		if (back)
		{
			pass = (m >= blade) ? t1 + (time - t1) * ((1.0 - m) / max(1.0 - blade, 1e-3)) : t0 + m * clamp(t1 - t0, 0.0, 2.0);
			front = blade - m;
		}
		else
		{
			pass = (m <= blade) ? t0 + (time - t0) * (m / max(blade, 1e-3)) : t1 + (1.0 - m) * clamp(t0 - t1, 0.0, 2.0);
			front = m - blade;
		}
		wipe.since = min(wipe.since, max(time - pass, 0.0));
		float moving = smoothstep(0.0, 0.03, blade) * smoothstep(1.0, 0.97, blade);
		float ahead = (front > 0.0) ? (1.0 - smoothstep(0.0, 0.035, front)) * moving : 0.0;
		wipe.ahead = max(wipe.ahead, ahead);
		if ((front > 0.0) && (moving > 0.5))
			wipe.front = min(wipe.front, front);
		wipe.swept = max(wipe.swept, ahead * (back ? 1.0 - blade : blade));
		float working = 1.0 - smoothstep(4.0, 30.0, time - t1);
		wipe.working = max(wipe.working, working);
	}
	return wipe;
}

// a place of the glass a blade reaches
bool rw_reached(vec2 coords)
{
	return any(greaterThanEqual(textureLod(wipermask, clamp(coords, vec2(0.0), vec2(1.0)), 0.0), vec4(0.001)));
}

// the edge of the sweep of the blades, where they push the water out to: 0 where a blade reaches, else the share of the places
// around at Reach (metres along the glass) which a blade reaches. found from where the blades reach and not from the values of
// the mask, which may stay at the end of the sweep over a wide part of the glass. Maskstep: the coordinates of the mask for a
// step along the glass
float rw_rim(vec2 coords, float reach, mat2 maskstep)
{
	if (rw_reached(coords))
		return 0.0;
	float inside = 0.0;
	for (int k = 0; k < 6; ++k)
	{
		float angle = float(k) * (RW_PI / 3.0);
		inside += rw_reached(coords + maskstep * (vec2(cos(angle), sin(angle)) * reach)) ? 1.0 : 0.0;
	}
	return inside / 6.0;
}

// whether the edge of the sweep of the blades is within Reach of the place
bool rw_border(vec2 coords, float reach, mat2 maskstep)
{
	bool here = rw_reached(coords);
	for (int k = 0; k < 6; ++k)
	{
		float angle = float(k) * (RW_PI / 3.0);
		if (rw_reached(coords + maskstep * (vec2(cos(angle), sin(angle)) * reach)) != here)
			return true;
	}
	return false;
}

// how far along its way a sliding drop is at the share T of the time it takes: it stops and goes, the more smoothly the harder
// it is pushed
float rw_stickslip(float t, float seed)
{
	t = clamp(t, 0.0, 1.0);
	float s = t * 3.0;
	float i = floor(s);
	float stuck = (i + smoothstep(0.25 + 0.4 * seed, 1.0, s - i)) / 3.0;
	return mix(stuck, t, smoothstep(0.8, 2.0, rw_push));
}

// the track of the water down a column of the glass (Column: its index, Width: of a column, Y: metres up the way): where it
// is across, metres. the water keeps to the way wetted before, so every drop of a column goes the same way, wavy, with kinks
// where the glass held it
float rw_path(int column, float y, float width, uint layer)
{
	float slow = rw_noise(y / (0.05 * rw_scale) + float(column) * 7.31, layer) - 0.5;
	float k = y / (0.008 * rw_scale);
	float i = floor(k);
	float a = rw_random(ivec2(column, int(i)), layer + 500u, 3u).x;
	float b = rw_random(ivec2(column, int(i) + 1), layer + 500u, 3u).x;
	float kinks = mix(a, b, smoothstep(0.15, 0.85, k - i)) - 0.5;
	return (float(column) + 0.5 + 0.4 * slow + 0.2 * kinks) * width;
}

// the drops which grew big enough to slide, and the streams they leave: Q the place across and up the way they go (Side, Down:
// those directions in the coordinates of the glass), Share of them going this way. a column of the glass carries the water of
// a track; a drop starts somewhere in a cell of the column now and then, slides down with stops, thinning, and leaves a stream
// which drains into beads and dries
void rw_sliders(vec2 q, vec2 side, vec2 down, uint layer, float share)
{
	float width = 0.04 * rw_scale;
	float height = 0.15 * rw_scale;
	int column = int(floor(q.x / width));
	int row = int(floor(q.y / height));
	vec4 a = rw_random(ivec2(column, 7), layer, 0u);
	if (a.x > share * mix(0.35, 0.9, clamp(rw_rate / 3.0, 0.0, 1.0)))
		return;
	float speed = clamp(0.035 + 0.06 * (rw_push - 0.4) + 0.3 * max(rw_push - 1.5, 0.0), 0.01, 0.8);
	float period = clamp(25.0 / rw_rate, 3.0, 60.0);
	float trail = 0.0;
	for (int r = 0; r < 3; ++r)
	{
		ivec2 cell = ivec2(column, row + r);
		vec4 b = rw_random(cell, layer, 1u);
		float span = period * mix(0.7, 1.3, b.x);
		float now = floor(time / span + b.y);
		for (int k = 0; k < 2; ++k)
		{
			float cycle = now - float(k);
			float start = (cycle - b.y) * span;
			float t = time - start;
			// the drop gathered from the drops landed since the last wipe
			if ((t < 0.0) || (rw_since - t < rw_grow))
				continue;
			vec4 c = rw_random(cell, layer + 1000u, uint(int(cycle) + 1000000));
			float top = (float(cell.y) + 0.8 - 0.6 * c.x) * height;
			float travel = mix(0.06, 0.28, c.y) * rw_scale;
			float v = speed * mix(0.7, 1.3, c.z);
			float gather = 0.6;
			float moved = travel * rw_stickslip((t - gather) * v / travel, c.w);
			float y = top - moved;
			float radius = mix(0.003, 0.0042, c.w) * rw_scale * (1.0 - 0.35 * moved / travel) * mix(0.6, 1.0, smoothstep(0.0, gather, t));
			// at the end of its way what is left of it stays and flows out into the stream
			float body = 1.0 - smoothstep(0.0, 1.5, t - gather - travel / v);
			if ((body > 0.0) && (abs(q.y - y) < 3.5 * radius))
			{
				vec2 o = vec2(q.x - rw_path(column, y, width, layer), q.y - y);
				rw_blob(side * o.x - down * o.y, radius, body);
				// its rear up the stream: a corner, drawn out into a cusp the faster it goes (the front stays round)
				float drawn = clamp(v * 5.0, 0.0, 1.5);
				for (int k = 1; k <= 2; ++k)
				{
					float yt = y + radius * (0.45 + 0.5 * drawn) * float(k);
					vec2 ot = vec2(q.x - rw_path(column, yt, width, layer), q.y - yt);
					rw_blob(side * ot.x - down * ot.y, radius * ((k == 1) ? 0.7 : 0.42), body);
				}
			}
			if ((moved > 0.0) && (q.y > y) && (q.y < top))
			{
				float age = t - gather - (top - q.y) / v;
				if ((age >= 0.0) && (age < rw_since))
				{
					// fresh, the stream is whole; draining, it breaks into beads, which dry
					float beads = rw_noise(q.y / (0.004 * rw_scale), layer + 9u);
					float wet = mix(1.15, 0.2 + 1.0 * beads, smoothstep(0.0, 15.0, age)) * (1.0 - smoothstep(30.0, 60.0, age));
					trail = max(trail, wet);
				}
			}
		}
	}
	if (trail > 0.0)
	{
		float across = q.x - rw_path(column, q.y, width, layer);
		float half_width = mix(0.0005, 0.0011, a.y) * rw_scale * (0.6 + 0.8 * rw_noise(q.y / (0.006 * rw_scale), layer + 11u));
		rw_blob(side * across, half_width, trail);
	}
}

// a drop lying on the glass, as rw_drop() with the directions along the glass given in the view space (Down: the way gravity
// pulls along the glass, Side across it)
void rw_drop_view(vec3 offset, float visible, float weight, vec4 shape, vec3 down, vec3 side)
{
	rw_kernel(offset, visible, weight);
	if (visible < 1.5 * rw_pixel)
		return;
	vec3 sag = (down * (0.2 + 0.35 * shape.x) + side * ((shape.y - 0.5) * 0.7)) * visible;
	rw_kernel(offset - sag, visible * mix(0.6, 0.85, shape.z), weight);
	if (shape.w > 0.4)
	{
		float angle = shape.w * 40.0;
		rw_kernel(offset - (cos(angle) * side + sin(angle) * down) * (0.6 * visible), visible * mix(0.4, 0.6, shape.x), weight);
	}
}

// the water the simulation keeps on the glass (gfx::windscreen_water, water.glsl), where the material is a glass of the cab of the
// train: the drops listed by the cell of the grid the fragment is in. Slot: of the glass, Uv: the coordinates of the fragment,
// Puv: the view space step along the glass for a step of the uv; Down, Side: along the glass. a sliding drop is drawn out behind
// itself into a cusp, the faster the longer; one lying is lopsided and sags down the glass; a flake of snow is lace, grey and
// clear as it melts (Snowcover, Wet: as the flakes of rw_snow())
void rw_simulated(uint slot, vec2 uv, mat2x3 puv, vec3 down, vec3 side, inout float snowcover, inout float wet)
{
	uint base = water_grid_base(slot);
	uint cell = water_cell(uv);
	uint count = min(water_cells[base + cell], uint(WATER_CELL_DROPS));
	for (uint k = 0u; k < count; ++k)
	{
		uint place = water_cells[base + uint(WATER_GRID * WATER_GRID) + cell * uint(WATER_CELL_DROPS) + k];
		water_drop_s drop = water_drops[slot * uint(WATER_DROPS) + place];
		float volume = drop.place.z;
		if (volume <= 0.0)
			continue;
		vec3 offset = puv * (uv - drop.place.xy);
		vec4 shape = vec4(rw_pcg4d(uvec4(place, slot, 41u, 0u))) * (1.0 / 4294967296.0);
		if (drop.motion.w < 1.0)
		{
			// a flake: lobed lace, shrinking, grey and clear as it melts
			float m = drop.motion.w;
			vec2 flat2 = vec2(dot(offset, side), dot(offset, down));
			float radius = 0.5 * water_flake(volume);
			float dist = length(flat2);
			if (dist < radius + rw_pixel)
			{
				float angle = atan(flat2.y, flat2.x);
				float lobes = 0.55 + 0.45 * (0.6 * rw_noise(angle * 1.9 + shape.z * 40.0, 13u) + 0.4 * abs(cos(3.0 * angle + shape.w * 6.2831853)));
				float rim = radius * lobes * (1.0 - 0.45 * m);
				float body = smoothstep(rim + rw_pixel, 0.55 * rim - rw_pixel, dist);
				float lace = rw_noise2(flat2 / 0.0007 + shape.zw * 100.0, 14u).x;
				float flake = body * mix(0.55, 1.0, lace) * (1.0 - 0.7 * m * m);
				snowcover = 1.0 - (1.0 - snowcover) * (1.0 - flake);
				wet = max(wet, m * body);
			}
			continue;
		}
		float radius = water_radius(volume);
		vec3 velocity = puv * drop.motion.xy;
		float speed = length(velocity);
		if (speed > 0.002)
		{
			// its rear up the way it came: a corner, drawn out into a cusp the faster it goes (the front stays round)
			vec3 way = velocity / speed;
			rw_kernel(offset, radius, 1.0);
			float drawn = clamp(speed * 5.0, 0.0, 1.5);
			for (int t = 1; t <= 2; ++t)
				rw_kernel(offset + way * (radius * (0.45 + 0.5 * drawn) * float(t)), radius * ((t == 1) ? 0.7 : 0.42), 1.0);
		}
		else
		{
			rw_drop_view(offset, radius, 1.0, shape, down, side);
		}
	}
}

// how fresh the wetted paths of the simulation are at a place of the glass (x) and how that changes along u and v (yz): the
// texels of their map filtered by a cubic b-spline over 4 x 4 of them, so the edges of a path are smooth curves and its slopes
// change smoothly (a bilinear filter would leave steps at every texel in the bending of the view)
vec3 rw_trail(uint slot, vec2 uv, float now)
{
	// a path stays wet (and seen) longer than the drops keep to it
	float lasting = 25.0;
	vec2 p = uv * float(WATER_TRAIL) - 0.5;
	vec2 i = floor(p);
	vec2 f = p - i;
	// the weights of the b-spline and their derivatives, for the texels i - 1 .. i + 2
	vec2 g = 1.0 - f;
	vec4 wx = vec4(g.x * g.x * g.x, 3.0 * f.x * f.x * f.x - 6.0 * f.x * f.x + 4.0, -3.0 * f.x * f.x * f.x + 3.0 * f.x * f.x + 3.0 * f.x + 1.0, f.x * f.x * f.x) / 6.0;
	vec4 wy = vec4(g.y * g.y * g.y, 3.0 * f.y * f.y * f.y - 6.0 * f.y * f.y + 4.0, -3.0 * f.y * f.y * f.y + 3.0 * f.y * f.y + 3.0 * f.y + 1.0, f.y * f.y * f.y) / 6.0;
	vec4 dx = vec4(-g.x * g.x, 3.0 * f.x * f.x - 4.0 * f.x, -3.0 * f.x * f.x + 2.0 * f.x + 1.0, f.x * f.x) * 0.5;
	vec4 dy = vec4(-g.y * g.y, 3.0 * f.y * f.y - 4.0 * f.y, -3.0 * f.y * f.y + 2.0 * f.y + 1.0, f.y * f.y) * 0.5;
	vec3 result = vec3(0.0);
	ivec2 base = ivec2(i) - 1;
	for (int y = 0; y < 4; ++y)
	{
		for (int x = 0; x < 4; ++x)
		{
			float value = water_wetted(slot, base + ivec2(x, y), now, lasting);
			result += value * vec3(wx[x] * wy[y], dx[x] * wy[y], wx[x] * dy[y]);
		}
	}
	result.yz *= float(WATER_TRAIL);
	return result;
}

// the surroundings in a view space direction where the screen doesn't hold them: the environment map above the horizon (the sky
// and the clouds), the ground below it, blurred by Lod
vec3 rw_surroundings(vec3 direction, float lod)
{
	vec3 world = mat3(inv_view) * direction;
	vec3 sky = envmap_sample_rough(normalize(vec3(world.x, max(world.y, 0.0), world.z)), clamp(lod / 8.0, 0.0, 1.0));
	// the horizon as blurred as the view is
	float below = smoothstep(-0.03 - 0.05 * lod, 0.25 + 0.05 * lod, -world.y);
	if ((below <= 0.0) || (false == ENVMAP_ENABLED))
		return sky;
	return mix(sky, reflection_ground(), below);
}

// what is seen through the glass in a view space direction: the image of the last frame (the screen space reflections keep it)
// where it holds the place, Distance away along the direction (0: far), blurred by Lod; Share: how much of it the image gave.
// Fade: towards the edges of the image the surroundings take over (else the edges are stretched)
vec3 rw_behind_edges(vec3 direction, float lod, float distance, out float share, bool fade)
{
	share = 0.0;
	vec3 around = rw_surroundings(direction, lod);
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
	// the view out through the water goes on outside, where the image shows the inside of the cab around the windows the
	// surroundings stand for it (else the drops show the dark walls and roof of the cab)
	ivec2 size = textureSize(scene_depth, 0);
	float depth = texelFetch(scene_depth, clamp(ivec2(coords * vec2(size)), ivec2(0), size - 1), 0).r;
	float near = projection[3][2];
	if ((depth > 0.0) && (near / depth < -f_pos.z + 2.5))
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

vec3 rw_behind(vec3 direction, float lod, float distance, out float share)
{
	return rw_behind_edges(direction, lod, distance, share, true);
}

// a layer Over put over Under, the colours not premultiplied
vec4 rw_over(vec4 over, vec4 under)
{
	float alpha = over.a + under.a * (1.0 - over.a);
	if (alpha <= 1e-5)
		return vec4(0.0);
	return vec4((over.rgb * over.a + under.rgb * under.a * (1.0 - over.a)) / alpha, alpha);
}

void main()
{
	vec4 tex_color = texture(diffuse, f_coord);
	vec4 mask = texture(wipermask, f_coord);

	// the coordinates of the glass: metres along the axes of the model, so the water stays where it lies on the glass whatever the
	// vehicle and the camera do. the axis of the model nearest to the vertical is up, the glass is projected along the
	// horizontal one it faces most (along the vertical one if it lies flat)
	mat3 linear = mat3(modelview);
	mat3 rot = mat3(normalize(linear[0]), normalize(linear[1]), normalize(linear[2]));
	mat3 unrot = transpose(rot);
	vec3 local = unrot * (f_pos.xyz - modelview[3].xyz);
	vec3 viewnormal = normalize(f_normal);
	// the water is on the side away from the camera, the outside of the cab
	vec3 outward = (dot(viewnormal, f_pos.xyz) < 0.0) ? -viewnormal : viewnormal;
	vec3 normal = unrot * outward;
	vec3 up = unrot * (vec3(0.0, 1.0, 0.0) * mat3(inv_view));
	vec3 aup = abs(up);
	int iu = (aup.x > aup.y) ? ((aup.x > aup.z) ? 0 : 2) : ((aup.y > aup.z) ? 1 : 2);
	int ia = (iu + 1) % 3;
	int ib = (iu + 2) % 3;
	vec3 axis_up = vec3(0.0);
	axis_up[iu] = (up[iu] < 0.0) ? -1.0 : 1.0;
	vec3 axis_a = vec3(0.0);
	axis_a[ia] = 1.0;
	vec3 axis_b = vec3(0.0);
	axis_b[ib] = 1.0;
	vec3 an = abs(normal);
	vec3 axis_u, axis_v, axis_n;
	if (an[iu] > max(an[ia], an[ib]))
	{
		axis_n = axis_up;
		axis_u = axis_a;
		axis_v = axis_b;
	}
	else if (an[ia] >= an[ib])
	{
		axis_n = axis_a;
		axis_u = axis_b;
		axis_v = axis_up;
	}
	else
	{
		axis_n = axis_b;
		axis_u = axis_a;
		axis_v = axis_up;
	}
	vec2 glass = vec2(dot(local, axis_u), dot(local, axis_v));
	float facing = dot(normal, axis_n);
	facing = ((facing < 0.0) ? -1.0 : 1.0) * max(abs(facing), 0.35);
	// a step of the coordinates is a step along the glass
	rw_tu = rot * (axis_u - axis_n * (dot(normal, axis_u) / facing));
	rw_tv = rot * (axis_v - axis_n * (dot(normal, axis_v) / facing));

	// the derivatives, before anything is discarded: the size of a pixel on the glass and in the view, and how the coordinates of
	// the mask follow the ones of the glass
	vec3 view = normalize(f_pos.xyz);
	vec3 dpdx = dFdx(f_pos.xyz);
	vec3 dpdy = dFdy(f_pos.xyz);
	rw_pixel = max(max(length(dpdx), length(dpdy)), 1e-6);
	float pixelangle = max(max(length(dFdx(view)), length(dFdy(view))), 1e-6);
	vec2 gdx = dFdx(glass);
	vec2 gdy = dFdy(glass);
	mat2 steps = mat2(gdx, gdy);
	float det = determinant(steps);
	mat2 maskstep = (abs(det) > 1e-14) ? mat2(dFdx(f_coord), dFdy(f_coord)) * inverse(steps) : mat2(0.0);

	// the water the simulation keeps, where the material is a glass of the cab of the train (the param water_slot, given by the
	// renderer: 1 + the glass): the view space step along the glass for a step of the uv, and how fresh the wetted path the water
	// went along is, with how that changes along the glass (before anything is discarded)
	int slot = int(param[2].z + 0.5) - 1;
	mat2 uvsteps = mat2(dFdx(f_coord), dFdy(f_coord));
	float uvdet = determinant(uvsteps);
	bool simulated = (slot >= 0) && (slot < WATER_SLOTS) && (abs(uvdet) > 1e-24);
	simulated = simulated && (water_header[clamp(slot, 0, WATER_SLOTS - 1)].x > 0.5);
	mat2x3 puv = (abs(uvdet) > 1e-24) ? mat2x3(dpdx, dpdy) * inverse(uvsteps) : mat2x3(0.0);
	vec3 trailfield = simulated ? rw_trail(uint(slot), f_coord, water_header[clamp(slot, 0, WATER_SLOTS - 1)].y) : vec3(0.0);
	float trail = trailfield.x;
	// its gradient along the glass, view space: the uv gradient taken through the steps of the uv along the glass
	vec3 trailslope = vec3(0.0);
	{
		mat2 metric = transpose(puv) * puv;
		if (abs(determinant(metric)) > 1e-30)
			trailslope = puv * (inverse(metric) * trailfield.yz);
	}

	if (tex_color.a < 0.01)
		discard;

	vec4 layer = vec4(0.0);
	float heavy = clamp(rain_params.x, 0.0, 1.0);
	bool snow = rain_params.z > 0.5;
	if ((heavy > 0.0) || simulated)
	{
		rw_scale = max(param[2].y, 0.05);

		// what meets the glass. the air against the vehicle (rain_wind) drags the drops along the glass with what of it goes
		// along the glass and with the share of what meets it head on which turns up it: the more the glass leans back the more
		// of the air goes along it, and the less of their weight pulls the drops down it
		vec3 air = unrot * (rain_wind.xyz * mat3(inv_view));
		float fall = snow ? RW_SNOW_FALL : RW_RAIN_FALL;
		float hits = max(-dot(air - up * fall, normal), 0.0);
		rw_rate = heavy * clamp(0.7 + hits / fall, 0.7, 6.0);
		rw_period = clamp((snow ? 10.0 : 5.0) / rw_rate, 0.4, 300.0);
		rw_grow = 2.0 * rw_period;
		vec3 gravity = -up - normal * dot(-up, normal);
		vec3 rise = up - normal * dot(up, normal);
		rise = (length(rise) > 0.1) ? normalize(rise) : vec3(0.0);
		vec3 flow = air - normal * dot(air, normal) + rise * (max(-dot(air, normal), 0.0) * RW_HEAD_ON);
		vec3 push = gravity + flow * (length(flow) / (RW_DRAG_SPEED * RW_DRAG_SPEED));
		vec2 force = vec2(dot(push, axis_u), dot(push, axis_v));
		rw_push = length(force);
		vec2 fallway = vec2(dot(gravity, axis_u), dot(gravity, axis_v));
		rw_down = (length(fallway) > 0.1) ? normalize(fallway) : vec2(0.0, -1.0);
		vec2 flowway = vec2(dot(flow, axis_u), dot(flow, axis_v));
		rw_flow = (length(flowway) > 1.0) ? normalize(flowway) : vec2(0.0);

		rw_wipe_s wipe = rw_wipe(mask);
		rw_since = wipe.since;
		// the blades went lately (any of them, also where none of them reaches)
		float working = 0.0;
		for (int i = 0; i < 4; ++i)
			working = max(working, 1.0 - smoothstep(4.0, 30.0, time - wiper_timer_return[i]));
		// how much of the water the place gathers it has
		float gathered = 1.0 - exp(-rw_since / (6.0 * rw_period));
		// the film a blade leaves is drawn into fine streaks along its arcs, each a thin cylindrical lens which draws a light seen
		// through it into a line across the streaks, towards the pivot of the blade: the way across the streaks on the screen
		vec2 streakway = vec2(0.0);
		if ((working > 0.0) && (false == snow) && (abs(det) > 1e-14))
		{
			int blade = 0;
			for (int i = 1; i < 4; ++i)
				blade = (mask[i] > mask[blade]) ? i : blade;
			if (mask[blade] >= 0.001)
			{
				// the mask grows along the arcs; a step of a centimetre, so the steps of its values don't matter
				float mu = textureLod(wipermask, f_coord + maskstep * vec2(0.01, 0.0), 0.0)[blade];
				float mv = textureLod(wipermask, f_coord + maskstep * vec2(0.0, 0.01), 0.0)[blade];
				vec2 arc = vec2(mu, mv) - mask[blade];
				vec2 onscreen = inverse(steps) * vec2(-arc.y, arc.x);
				if ((length(arc) > 1e-4) && (length(onscreen) > 1e-9))
					streakway = normalize(onscreen);
			}
		}
		vec3 ripple = rw_noise2(glass / (0.006 * rw_scale), 5u);

		// the water a blade pushes: a bead in front of it while it moves, as wide as the water it gathered on its way, with a steep
		// lumpy front (its edge shows dark, the view can't get out there). the drops the bead reaches flow into it
		vec3 lumps = rw_noise2(glass / (0.009 * rw_scale), 7u);
		float gatheredwater = snow ? 0.0 : min(gathered * 1.5, 1.0) * (0.35 + 0.65 * min(wipe.swept * 2.5, 1.0));
		float reach = 0.004 + 0.018 * gatheredwater + 0.008 * (ripple.x - 0.5);
		float ridge = gatheredwater * smoothstep(0.0, 0.003, wipe.front) * (1.0 - smoothstep(reach - 0.004, reach, wipe.front));
		// the smooth shape from how it changes from pixel to pixel, the lumps on it as they are
		float pooledbase = 0.004 * ridge;
		float pooled = pooledbase * (0.75 + 0.5 * lumps.x);
		vec3 pooledslope = rw_gradient(pooledbase, dpdx, dpdy, outward) * (0.75 + 0.5 * lumps.x) + rw_along(lumps.yz * (0.5 * pooledbase / (0.009 * rw_scale)));
		if ((pooled > 0.0) && (false == simulated))
		{
			rw_sheet(pooled, pooledslope, 0.03, smoothstep(0.0, 0.1, ridge));
			if (ridge > 0.05)
				rw_cleared = time;
		}

		float snowcover = 0.0;
		float wet = 0.0;
		if (simulated)
		{
			// the directions along the glass, view space
			vec3 upview = vec3(0.0, 1.0, 0.0) * mat3(inv_view);
			vec3 downview = -(upview - outward * dot(upview, outward));
			downview = (length(downview) > 0.05) ? normalize(downview) : normalize(cross(outward, normalize(rw_tu)));
			vec3 sideview = cross(outward, downview);
			rw_simulated(uint(slot), f_coord, puv, downview, sideview, snowcover, wet);
			// the paths the water went along lately: a rivulet, a band of water rounded across (a cylinder lens: it holds a picture of
			// the place around it turned over across the band, its edges dark as the edges of drops), thicker the fresher. the
			// smoothed freshness falls off from the middle of a path to its edges
			if (trail > 0.08)
			{
				float p = clamp((trail - 0.08) / 0.5, 0.0, 1.0);
				float thickness = 0.0005 * (1.0 - (1.0 - p) * (1.0 - p));
				vec3 rise = trailslope * (0.0005 * 2.0 * (1.0 - p) / 0.5);
				float steep = length(rise);
				if (steep > 0.7)
					rise *= 0.7 / steep;
				bool top = thickness > rw_height;
				rw_sheet(thickness, rise, 0.004, smoothstep(0.0, 0.15, p) * 0.95);
				// from the middle of the band to the fragment
				float across = length(trailslope) / 0.5;
				if (top && (across > 1.0))
					rw_offset = -trailslope * ((1.0 - p) / (across * length(trailslope)));
			}
		}
		else if (false == snow)
		{
			// the drops which grew big enough slide off, with gravity and the air: a set of tracks for each of the directions of the
			// push nearest to the one now, each laid along its own way, so none of them moves when the push turns
			float sliding = smoothstep(0.25, 0.6, rw_push);
			if (sliding > 0.0)
			{
				float bin = atan(force.y, force.x) / (RW_PI / 6.0);
				float lower = floor(bin);
				for (int k = 0; k < 2; ++k)
				{
					float share = ((k == 0) ? 1.0 - (bin - lower) : bin - lower) * sliding;
					if (share < 0.02)
						continue;
					float angle = (lower + float(k)) * (RW_PI / 6.0);
					vec2 down = vec2(cos(angle), sin(angle));
					vec2 side = vec2(-down.y, down.x);
					rw_sliders(vec2(dot(glass, side), -dot(glass, down)), side, down, uint(int(lower) + k + 64) * 2u + 16u, share);
				}
			}

			// the drops as they land, each time somewhere else in its cell, as often as the rain hits the glass: the drops of the
			// rain spread on the glass, thrown with a fine splash which runs back into them
			// each drop anywhere in its cell, the cell as large as a drop with its splash reaches
			float cell = 0.007 * rw_scale;
			vec2 laid = rw_turn(glass, 0.61) + vec2(0.31, 0.17);
			ivec2 home = ivec2(floor(laid / cell));
			float density = mix(0.08, 0.3, heavy);
			for (int y = -1; y <= 1; ++y)
			{
				for (int x = -1; x <= 1; ++x)
				{
					ivec2 c = home + ivec2(x, y);
					vec4 a = rw_random(c, 1u, 0u);
					if (a.x > density)
						continue;
					vec4 b = rw_random(c, 1u, 1u);
					float span = rw_period * mix(0.7, 1.3, a.w);
					float cycle = floor(time / span + b.x);
					float age = time - (cycle - b.x) * span;
					float life = span * mix(0.5, 0.9, b.y);
					if ((age > life) || (age > rw_since) || (time - age < rw_cleared))
						continue;
					vec4 p = rw_random(c, 2u, uint(cycle));
					vec2 offset = rw_turn(laid - (vec2(c) + p.xy) * cell, -0.61);
					float visible = mix(0.0011, 0.0019, b.z) * rw_scale;
					// a raindrop hits the glass far above the speed at which it splashes (Weber number in the hundreds and thousands):
					// it spreads, throws a spray of droplets which stay on the glass around it, and draws back
					float splash = exp(-age / 0.06);
					float fade = 1.0 - smoothstep(life - min(0.4, 0.3 * life), life, age);
					if (visible * 0.3 > 0.5 * rw_pixel)
					{
						int spray = 3 + int(p.z * 4.0);
						for (int s = 0; s < spray; ++s)
						{
							vec4 e = rw_random(c, 3u, uint(cycle) * 8u + uint(s));
							vec2 way = normalize(vec2(cos(e.x * 6.2831853), sin(e.x * 6.2831853)) + rw_flow * 0.9);
							rw_blob(offset - way * (visible * (1.4 + 1.6 * e.y)), visible * mix(0.18, 0.35, e.z), fade * (0.55 + 0.45 * splash));
						}
					}
					rw_drop(offset, visible * (1.0 + 0.3 * splash), fade, p.wzyx);
				}
			}
			// the drops the landing ones gathered into, growing until they are big enough to slide (the air blows them off sooner)
			cell = 0.011 * rw_scale;
			laid = rw_turn(glass, 1.07) + vec2(0.53, 0.29);
			home = ivec2(floor(laid / cell));
			density = mix(0.55, 1.0, heavy);
			for (int y = -1; y <= 1; ++y)
			{
				for (int x = -1; x <= 1; ++x)
				{
					ivec2 c = home + ivec2(x, y);
					vec4 a = rw_random(c, 4u, 0u);
					if (a.x > density)
						continue;
					vec4 b = rw_random(c, 4u, 1u);
					float span = rw_period * mix(2.5, 4.0, a.w);
					float cycle = floor(time / span + b.x);
					float age = time - (cycle - b.x) * span;
					float life = span * mix(0.75, 0.98, b.y) / (1.0 + 2.0 * max(rw_push - 1.0, 0.0));
					if ((age > life) || (age > rw_since) || (time - age < rw_cleared))
						continue;
					vec4 p = rw_random(c, 5u, uint(cycle));
					// the big drops are much alike: they grow as they gather the small ones (the radius in step with the time) up to the
					// size at which they slide off (on a vertical glass a footprint of about 9 mm, larger on a glass leaning back, smaller
					// in a strong air flow), and go
					float slides = min(0.0045 / sqrt(max(rw_push, 0.3)), 0.005);
					float visible = slides * mix(0.6, 1.0, b.z) * rw_scale * mix(0.35, 1.0, clamp(age / (0.6 * life), 0.0, 1.0));
					rw_drop(rw_turn(laid - (vec2(c) + p.xy) * cell, -1.07), visible, 1.0 - smoothstep(life - min(0.5, 0.3 * life), life, age), p.zwxy);
				}
			}

			// the water the blades push out of their sweep: a broken row of drops flowed together along its edge, just outside it
			float leave = working * min(heavy * 1.5, 1.0);
			if ((leave > 0.0) && rw_border(f_coord, 0.009 * rw_scale, maskstep))
			{
				cell = 0.0065 * rw_scale;
				laid = rw_turn(glass, 0.29) + vec2(0.11, 0.43);
				home = ivec2(floor(laid / cell));
				for (int y = -1; y <= 1; ++y)
				{
					for (int x = -1; x <= 1; ++x)
					{
						ivec2 c = home + ivec2(x, y);
						vec4 a = rw_random(c, 20u, 0u);
						if (a.x > 0.85)
							continue;
						// from the fragment to the middle of the drop, along the glass
						vec2 centre = rw_turn((vec2(c) + a.yz) * cell - laid, -0.29);
						float rim = rw_rim(f_coord + maskstep * centre, 0.004 * rw_scale, maskstep) * leave;
						if (rim <= 0.05)
							continue;
						rw_drop(-centre, mix(0.0016, 0.003, a.w) * rw_scale * mix(0.6, 1.0, rim), smoothstep(0.05, 0.35, rim), rw_random(c, 21u, 0u));
					}
				}
			}

			// the streams running down from the water the blades leave at the ends of their sweep: a stream in some of the columns
			// along the way down, where up the glass there is such water, as far down as the water had time to run since the blade
			// went over the place
			if ((working > 0.0) && (length(fallway) > 0.3))
			{
				vec2 side = vec2(-rw_down.y, rw_down.x);
				float width = 0.035 * rw_scale;
				float across = dot(glass, side);
				float along = -dot(glass, rw_down);
				int column = int(floor(across / width));
				vec4 a = rw_random(ivec2(column, 0), 30u, 0u);
				if (a.x < 0.6 * heavy * working)
				{
					float lateral = across - rw_path(column, along, width, 31u);
					float half_width = mix(0.0006, 0.0012, a.z) * rw_scale * (0.6 + 0.8 * rw_noise(along / (0.006 * rw_scale), 32u));
					if (abs(lateral) < 2.0 * half_width + rw_pixel)
					{
						// up the glass, the first place where the reach of the blades begins or ends
						const float reach[6] = float[](0.01, 0.025, 0.05, 0.1, 0.18, 0.3);
						bool here = rw_reached(f_coord);
						float source = -1.0;
						for (int s = 0; s < 6; ++s)
						{
							if (rw_reached(f_coord + maskstep * (-rw_down * reach[s])) != here)
							{
								source = reach[s];
								break;
							}
						}
						if ((source > 0.0) && (source < max(rw_since - 0.1, 0.0) * 0.2))
						{
							float beads = rw_noise(along / (0.004 * rw_scale), 33u);
							float flowing = working * (1.0 - 0.5 * smoothstep(0.15, 0.3, source));
							rw_blob(side * lateral, half_width, flowing * mix(0.6, 1.2, beads));
						}
					}
				}
			}
		}
		else
		{
			// the flakes stick where they land and melt into drops: the glass is warmed by the cab, faster in the air above zero
			float warm = rain_params.w;
			float cell = 0.016 * rw_scale;
			vec2 laid = rw_turn(glass, 0.83) + vec2(0.07, 0.61);
			ivec2 home = ivec2(floor(laid / cell));
			float density = mix(0.5, 0.9, heavy);
			for (int y = -1; y <= 1; ++y)
			{
				for (int x = -1; x <= 1; ++x)
				{
					ivec2 c = home + ivec2(x, y);
					vec4 a = rw_random(c, 11u, 0u);
					if (a.x > density)
						continue;
					vec4 b = rw_random(c, 11u, 1u);
					float span = rw_period * mix(0.7, 1.3, a.w);
					float cycle = floor(time / span + b.x);
					float age = time - (cycle - b.x) * span;
					float life = span * mix(0.6, 0.95, b.y);
					if ((age > life) || (age > rw_since))
						continue;
					vec4 p = rw_random(c, 12u, uint(cycle));
					vec2 offset = rw_turn(laid - (vec2(c) + p.xy) * cell, -0.83);
					float diameter = mix(RW_SNOW_DIAMETER.x, RW_SNOW_DIAMETER.y, b.z) * rw_scale;
					float melt = ((warm > 0.5) ? mix(3.0, 10.0, b.w) / (1.0 + 0.25 * warm) : mix(15.0, 45.0, b.w));
					if (age < melt)
					{
						float m = age / melt;
						float dist = length(offset.x * rw_tu + offset.y * rw_tv);
						float radius = 0.5 * diameter;
						if (dist < radius + rw_pixel)
						{
							float angle = atan(offset.y, offset.x);
							float lobes = 0.55 + 0.45 * (0.6 * rw_noise(angle * 1.9 + p.z * 40.0, 13u) + 0.4 * abs(cos(3.0 * angle + p.w * 6.2831853)));
							float rim = radius * lobes * (1.0 - 0.45 * m);
							float body = smoothstep(rim + rw_pixel, 0.55 * rim - rw_pixel, dist);
							float lace = rw_noise2(glass / (0.0007 * rw_scale) + p.zw * 100.0, 14u).x;
							float flake = body * mix(0.55, 1.0, lace) * (1.0 - 0.7 * m * m);
							snowcover = 1.0 - (1.0 - snowcover) * (1.0 - flake);
							wet = max(wet, m * body);
						}
					}
					else
					{
						// the water of the flake
						rw_drop(offset, 0.0018 * pow(diameter / 0.008, 0.667) * rw_scale, 1.0 - smoothstep(life - min(0.5, 0.3 * life), life, age), p.wzyx);
					}
				}
			}
			// the snow the blades push in front of them and leave at the ends of their sweep
			float pile = max(wipe.ahead * min(gathered * 2.0, 1.0) * (0.4 + 0.6 * min(wipe.swept * 2.5, 1.0)), rw_rim(f_coord, 0.006 * rw_scale, maskstep) * working * heavy);
			float packed = rw_noise2(glass / (0.0015 * rw_scale), 15u).x;
			snowcover = max(snowcover, smoothstep(0.15, 0.5, pile * 1.2 + 0.4 * (ripple.x - 0.5)) * mix(0.7, 1.0, packed));
		}
		// untreated glass, weathered and a little dirty: flat drops (a rain repellent coating would give beads above 90 degrees)
		rw_settle(radians(mix(30.0, 48.0, rw_noise2(glass / (0.005 * rw_scale), 16u).x)));

		// the lamps (of the cab, of the scenery) and the headlights light the water up, from both sides of the glass (worked out
		// here, where the fragments of a quad go together)
		vec3 lamps = vec3(0.0);
		vec3 lampgloss = vec3(0.0);
		add_local_lights(outward, lamps, lampgloss);
		add_local_lights(-outward, lamps, lampgloss);

		// the film just behind a blade, breaking up
		float film = snow ? 0.0 : heavy * exp(-rw_since / 0.8) * smoothstep(0.02, 0.08, rw_since);
		if (film > 0.01)
		{
			vec3 fine = rw_noise2(glass / (0.0021 * rw_scale), 6u);
			vec2 tilt = fine.yz * (0.004 * film);
			rw_sheet(0.00005 * film, rw_along(tilt), 0.3, 0.8 * film);
		}

		// what the water shows: the view goes through the glass (a plate: only the way from the air into the water counts) and out
		// through the surface of the water, bent as much as it is curved. it is the world turned upside down in a drop, and the
		// edges of a drop, where the view can't get out (total internal reflection), show the inside of the cab
		float background = 0.0;
		bool image = ssr_params.x >= 1.0;
		if (image)
		{
			ivec2 size = textureSize(scene_depth, 0);
			float depth = texelFetch(scene_depth, clamp(ivec2(gl_FragCoord.xy), ivec2(0), size - 1), 0).r;
			if (depth > 0.0)
				background = (projection[3][2] / depth) / max(-view.z, 0.05);
		}
		vec3 inside = ambient * (AMBIENT_SCALE * 0.5);
		vec4 water = vec4(0.0);
		if (rw_cover > 0.0)
		{
			vec3 surface = normalize(outward - rw_slope);
			vec3 inwater = refract(view, -outward, 1.0 / RW_IOR);
			vec3 through = refract(inwater, -surface, RW_IOR);
			// what the surface reflects back inside the water: the light which came in at the other side of the drop, from the sky,
			// and the inside of the cab. where the view can't get out (the edge of a drop) that is all it shows: a bright edge
			vec3 bounced = reflect(inwater, surface);
			if (dot(bounced, outward) < 0.0)
				bounced = reflect(bounced, outward);
			vec3 inner = mix(inside, rw_surroundings(normalize(bounced), 5.0), 0.3);
			vec3 color = inner;
			if (dot(through, through) > 0.5)
			{
				float fresnel = 0.02 + 0.98 * pow(1.0 - abs(dot(through, surface)), 5.0);
				// the view through the water spreads over this much of the world across a pixel (radians)
				float spread = (RW_IOR - 1.0) * rw_pixel / max(rw_curve, 1e-4);
				// the picture the drop holds as a lens, and a share of the bending by the surface
				vec3 lens = normalize(f_pos.xyz - (1.0 + RW_LENS) * rw_offset) + (through - view) * RW_LENS_BEND;
				float lod = mix(log2(1.0 + RW_LENS) - 0.5, log2(max(spread / pixelangle, 1.0)), RW_LENS_BEND);
				// a drop a few pixels across would hold a picture of single pixels, a grain: blurred more the smaller it looks
				lod += clamp(2.5 - log2(max(rw_curve / rw_pixel, 1.0)), 0.0, 1.5);
				float share;
				vec3 seen = rw_behind(normalize(lens), lod, background, share);
				// the sun through the drop, where the image doesn't hold it: as bright as the drop gathers its light. in heavier rain
				// the sun is behind the clouds, its disc is not there to be focused
				float sunshare = (1.0 - heavy) * (1.0 - heavy);
				if ((lights_count > 0U) && (share < 1.0) && (sunshare > 0.01))
				{
					vec3 sun = normalize(-lights[0].dir);
					float width = max(spread, 0.03);
					float off = acos(clamp(dot(through, sun), -1.0, 1.0));
					if (off < 4.0 * width)
					{
						float lit = 1.0 - clamp(calc_shadow(), 0.0, 1.0);
						vec3 sunlight = lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * cloud_shadow(mat3(inv_view) * f_pos.xyz));
						// the light of the sun over the spread of the view: a pixel shows pi * irradiance * the share of a steradian
						seen += sunlight * (sunshare * lit * (1.0 - share) * exp(-0.5 * off * off / (width * width)) / (2.0 * width * width));
					}
				}
				color = mix(seen, inner, fresnel);
			}
			color += lamps * 0.6;
			// without the image of the last frame the surroundings stand for what the water bends the view to; nearly flat water
			// would show them in place of the scene behind it, so it is left mostly clear
			float opaque = image ? 1.0 : mix(0.3, 1.0, smoothstep(0.02, 0.15, 1.0 - dot(through, view)));
			water = vec4(color, rw_cover * opaque);
		}
		// the wet glass where the rain gathers is a little hazy: the tiniest drops spread the light
		if (image && (false == snow))
		{
			float mist = heavy * gathered * (simulated ? 0.18 : 0.3);
			if (mist > 0.01)
			{
				float share;
				vec3 hazy = rw_behind_edges(view, 3.5, background, share, false);
				// the tiniest drops spread the light of the sky coming through the glass
				hazy = mix(hazy, rw_surroundings(outward, 7.0), 0.1);
				layer = vec4(hazy, mist * max(share, 0.5));
			}
		}
		layer = rw_over(water, layer);
		// the lights seen through the streaks of the film the blades leave, drawn into lines towards their pivot
		if (image && (length(streakway) > 0.5))
		{
			vec4 previous = ssr_reprojection * ((background > 0.0) ? vec4(view * background, 1.0) : vec4(view, 0.0));
			if (previous.w > 1e-5)
			{
				vec2 coords = (previous.xy / previous.w) * 0.5 + 0.5;
				vec2 texel = 1.0 / vec2(textureSize(ssr_history, 0));
				ivec2 size = textureSize(scene_depth, 0);
				float near = projection[3][2];
				// a light: much brighter than the place around it (the bright sky is not)
				vec3 around = textureLod(ssr_history, clamp(coords, vec2(0.0), vec2(1.0)), 6.0).rgb;
				float threshold = max(1.0, 3.0 * max(around.r, max(around.g, around.b)));
				vec3 glare = vec3(0.0);
				float total = 0.0;
				for (int k = -8; k <= 8; ++k)
				{
					if (k == 0)
						continue;
					float w = 1.0 - abs(float(k)) / 9.0;
					total += w;
					vec2 tap = coords + streakway * texel * (float(k) * 5.0);
					if (any(lessThan(tap, vec2(0.0))) || any(greaterThan(tap, vec2(1.0))))
						continue;
					// only what is outside: the lamps of the cab are not behind the glass
					float depth = texelFetch(scene_depth, clamp(ivec2(tap * vec2(size)), ivec2(0), size - 1), 0).r;
					if ((depth > 0.0) && (near / depth < -f_pos.z + 2.5))
						continue;
					vec3 c = textureLod(ssr_history, tap, 1.5).rgb;
					if (any(isnan(c)) || any(isinf(c)))
						continue;
					glare += max(c - vec3(threshold), vec3(0.0)) * w;
				}
				glare *= 0.6 * heavy * working / total;
				if (any(greaterThan(glare, vec3(1e-4))))
				{
					// the scene itself under the water carries the light added (it changes nothing seen)
					vec3 scene = textureLod(ssr_history, clamp(coords, vec2(0.0), vec2(1.0)), 0.0).rgb;
					layer = rw_over(layer, vec4(max(scene, vec3(0.0)), 0.2));
					layer.rgb += glare / layer.a;
				}
			}
		}
		if (snowcover > 0.0)
		{
			// the flakes are lit through from the outside, by the sky and the sun, and from the inside of the cab; melting they turn
			// grey and clear
			vec3 outside = rw_surroundings(outward, 6.0);
			if (lights_count > 0U)
			{
				vec3 sun = normalize(-lights[0].dir);
				outside += lights[0].color * (SUN_DIFFUSE_SCALE * lights[0].intensity * max(dot(outward, sun), 0.0) * (1.0 - clamp(calc_shadow(), 0.0, 1.0)));
			}
			vec3 flake = mix(vec3(0.92), vec3(0.62, 0.66, 0.7), wet);
			vec3 color = flake * (outside * (0.35 * (1.0 - 0.5 * wet)) + ambient * (AMBIENT_SCALE * 0.7));
			layer = rw_over(vec4(color, snowcover * mix(0.97, 0.6, wet)), layer);
		}
	}
	out_color = layer;

	{ // Overlay windshield texture with alpha
	  vec3 fragcolor = ambient;
	  vec3 fragnormal = normalize(f_normal);
	  float reflectivity = param[1].z;
	  float specularity = (tex_color.r + tex_color.g + tex_color.b) * 0.5;
	  legacy_gloss();

	  fragcolor = apply_lights(fragcolor, fragnormal, tex_color.rgb, reflectivity, specularity, shadow_tone);
	  vec4 color = vec4(fragcolor, tex_color.a * alpha_mult);

	  out_color.xyz = apply_fog(mix(out_color.xyz, color.xyz, color.a));
	  out_color.a = mix(out_color.a, 1., color.a);
	}
	if (false == POSTFX_ENABLED)
	{
		out_color = tonemap(out_color);
	}

	if (MOTIONBLUR_ENABLED)
	{
		out_motion = vec4(0.0);
	}
}
