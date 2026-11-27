// the temporal anti-aliasing (gfx.antialiasing taa, gfx::temporal_aa): the frame, drawn with the projection moved by a part of a
// pixel, blended into the history of the last frames. the same in ogl46/ and vk14/; the stage file gives the images and
//   TAA_HISTORY_VALID  the history holds the last frame (false: the frame alone is taken)
//   TAA_REPROJECTION   the clip space of this frame (with the depth) to the one of the last frame, for the world standing still
//   TAA_NEAR           the near plane of the projection: a pixel is that much over its depth away (reverse z)
// the history: the colour (linear, hdr) and in alpha the lock of the thin things.
//
// the thin things (rails and wires far away, thinner than a pixel) are hit by some of the offsets only; a pixel of a frame which
// misses them has the colours of the ground or the sky all around, so the usual check of the history against the colours around
// the pixel would wipe them out of the history and they would flicker. a pixel where a frame shows a line across (brighter or
// darker than the neighbours on both sides, and not along) is locked: for a few frames its history is kept as it is, as long as the
// pixel belongs to the world standing still (where the place in the last frame is sure), moves slowly over the screen and is far
// enough not to be a part of the cab (the needles of the gauges, the screens) or of things passing close by

// the share of the frame in the result
const float TAA_CURRENT_SHARE = 0.1;
// the history kept within this many standard deviations of the colours around the pixel: wide for a pixel standing still on the
// screen, narrow for one moving across it this many pixels a frame
const float TAA_VARIANCE_WIDE = 1.5;
const float TAA_VARIANCE_NARROW = 1.0;
const float TAA_SPEED_NARROW = 8.0;
// the velocity drawn agrees with the world standing still within this many pixels (the velocity is a half float)
const float TAA_STILL_PIXELS = 0.25;
// the lock of a thin thing lasts as many frames as the offsets go round (gfx::temporal_aa), and holds while the pixel moves slower
// than this many pixels a frame
const float TAA_LOCK_DECAY = 1.0 / 8.0;
const float TAA_LOCK_SPEED_MIN = 2.0;
const float TAA_LOCK_SPEED_MAX = 6.0;
const float TAA_LOCK_DISTANCE_MIN = 20.0;
const float TAA_LOCK_DISTANCE_FULL = 40.0;
// a line across: brighter or darker than both neighbours by this share of the brightest of the three, and the neighbours along
// it closer to the pixel than half of that
const float TAA_THIN_CONTRAST = 0.1;
const float TAA_THIN_MINIMUM = 0.01;

// the largest value of a half float: the colour drawn kept within it, without the nan and the infinity which would stay in the
// history for good
const float TAA_HALF_MAX = 65504.0;

vec3 taa_clean(vec3 c)
{
	return (any(isnan(c)) || any(isinf(c))) ? vec3(0.0) : clamp(c, vec3(0.0), vec3(TAA_HALF_MAX));
}

// the colour squeezed into 0..1 by its largest channel (a weight against the bright pixels), and back
vec3 taa_compress(vec3 c)
{
	return c / (1.0 + max(c.r, max(c.g, c.b)));
}

vec3 taa_decompress(vec3 c)
{
	return c / max(1.0 - max(c.r, max(c.g, c.b)), 1.0 / TAA_HALF_MAX);
}

vec3 taa_ycocg(vec3 c)
{
	return vec3(0.25 * c.r + 0.5 * c.g + 0.25 * c.b, 0.5 * c.r - 0.5 * c.b, -0.25 * c.r + 0.5 * c.g - 0.25 * c.b);
}

vec3 taa_rgb(vec3 c)
{
	return vec3(c.x + c.y - c.z, c.x + c.z, c.x - c.y - c.z);
}

// the history at the place, sharp: a catmull-rom spline through the texels, in five bilinear lookups (the corners left out)
vec3 taa_history_colour(vec2 uv, vec2 size)
{
	vec2 position = uv * size;
	vec2 texel1 = floor(position - 0.5) + 0.5;
	vec2 f = position - texel1;
	vec2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
	vec2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
	vec2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
	vec2 w3 = f * f * (-0.5 + 0.5 * f);
	vec2 w12 = w1 + w2;
	vec2 uv0 = (texel1 - 1.0) / size;
	vec2 uv3 = (texel1 + 2.0) / size;
	vec2 uv12 = (texel1 + w2 / w12) / size;
	vec3 result = textureLod(history_tex, vec2(uv12.x, uv0.y), 0.0).rgb * (w12.x * w0.y);
	result += textureLod(history_tex, vec2(uv0.x, uv12.y), 0.0).rgb * (w0.x * w12.y);
	result += textureLod(history_tex, uv12, 0.0).rgb * (w12.x * w12.y);
	result += textureLod(history_tex, vec2(uv3.x, uv12.y), 0.0).rgb * (w3.x * w12.y);
	result += textureLod(history_tex, vec2(uv12.x, uv3.y), 0.0).rgb * (w12.x * w3.y);
	float weight = w12.x * w0.y + w0.x * w12.y + w12.x * w12.y + w3.x * w12.y + w12.x * w3.y;
	return taa_clean(result / weight);
}

// the history brought towards the middle of the box of the colours around the pixel until it's inside it
vec3 taa_clip(vec3 history, vec3 centre, vec3 extent)
{
	vec3 offset = history - centre;
	vec3 units = abs(offset / max(extent, vec3(1.0 / 65504.0)));
	float largest = max(units.x, max(units.y, units.z));
	return (largest > 1.0) ? centre + offset / largest : history;
}

// the pixel against its neighbours on both sides (A, B) and along (C, D): a line across it
bool taa_line(float centre, float a, float b, float c, float d)
{
	float da = a - centre;
	float db = b - centre;
	float across = min(abs(da), abs(db));
	float contrast = max(TAA_THIN_MINIMUM, TAA_THIN_CONTRAST * max(centre, max(a, b)));
	return (da * db > 0.0) && (across > contrast) && (max(abs(c - centre), abs(d - centre)) < 0.5 * across);
}

void main()
{
	vec2 size = vec2(textureSize(color_tex, 0));
	ivec2 pixel = ivec2(f_coords * size);
	ivec2 last = ivec2(size) - 1;

	// the colours around the pixel (squeezed, ycocg): their moments and brightness, and the nearest depth among them
	vec3 m1 = vec3(0.0);
	vec3 m2 = vec3(0.0);
	float luma[9];
	float nearest = -1.0;
	ivec2 nearestpixel = pixel;
	for (int y = -1; y <= 1; ++y)
	{
		for (int x = -1; x <= 1; ++x)
		{
			ivec2 p = clamp(pixel + ivec2(x, y), ivec2(0), last);
			vec3 c = taa_ycocg(taa_compress(taa_clean(texelFetch(color_tex, p, 0).rgb)));
			m1 += c;
			m2 += c * c;
			luma[(y + 1) * 3 + (x + 1)] = c.x;
			// reverse z: the nearer, the greater
			float d = texelFetch(depth_tex, p, 0).r;
			if (d > nearest)
			{
				nearest = d;
				nearestpixel = p;
			}
		}
	}
	vec3 frame = taa_clean(texelFetch(color_tex, pixel, 0).rgb);
	if (false == TAA_HISTORY_VALID)
	{
		out_color = vec4(frame, 0.0);
		return;
	}
	vec3 current = taa_ycocg(taa_compress(frame));

	// the place of the pixel in the last frame: by the velocity drawn (of the nearest pixel around, so the edges go with what's in
	// front), or for the world standing still by the depth. the sky has no velocity drawn and stands still
	float depth = texelFetch(depth_tex, pixel, 0).r;
	vec2 moved = f_coords + texelFetch(velocity_tex, nearestpixel, 0).xy;
	vec4 world = TAA_REPROJECTION * vec4(f_coords * 2.0 - 1.0, depth, 1.0);
	vec2 worlduv = (world.w > 0.0) ? world.xy / world.w * 0.5 + 0.5 : vec2(-1.0);
	float difference = length((moved - worlduv) * size);
	bool still = (world.w > 0.0) && ((depth <= 0.0) || (difference < max(TAA_STILL_PIXELS, 0.01 * length((moved - f_coords) * size))));
	vec2 uv = still ? worlduv : moved;
	if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))))
	{
		// out of the last frame: the frame alone
		out_color = vec4(frame, 0.0);
		return;
	}
	float speed = length((uv - f_coords) * size);

	vec3 history = taa_ycocg(taa_compress(taa_history_colour(uv, size)));
	float lock = textureLod(history_tex, uv, 0.0).a;
	lock = isnan(lock) ? 0.0 : clamp(lock, 0.0, 1.0);

	// a line across the pixel in this frame locks it; the lock wears off, and holds only where the place in the last frame is sure,
	// the pixel moves slowly and is far away
	bool line = taa_line(luma[4], luma[3], luma[5], luma[1], luma[7]) || taa_line(luma[4], luma[1], luma[7], luma[3], luma[5]) ||
	            taa_line(luma[4], luma[0], luma[8], luma[2], luma[6]) || taa_line(luma[4], luma[2], luma[6], luma[0], luma[8]);
	lock = max(lock - TAA_LOCK_DECAY, line ? 1.0 : 0.0);
	float distance = (depth > 0.0) ? TAA_NEAR / depth : TAA_LOCK_DISTANCE_FULL;
	lock *= (still ? 1.0 : 0.0) * (1.0 - smoothstep(TAA_LOCK_SPEED_MIN, TAA_LOCK_SPEED_MAX, speed)) *
	        smoothstep(TAA_LOCK_DISTANCE_MIN, TAA_LOCK_DISTANCE_FULL, distance);

	// the history within the colours around the pixel, but where it's locked
	vec3 mean = m1 / 9.0;
	vec3 deviation = sqrt(max(m2 / 9.0 - mean * mean, vec3(0.0)));
	float gamma = mix(TAA_VARIANCE_WIDE, TAA_VARIANCE_NARROW, clamp(speed / TAA_SPEED_NARROW, 0.0, 1.0));
	history = mix(taa_clip(history, mean, gamma * deviation), history, lock);

	vec3 result = mix(history, current, TAA_CURRENT_SHARE);
	out_color = vec4(taa_clean(taa_decompress(clamp(taa_rgb(result), vec3(0.0), vec3(1.0 - 1.0 / TAA_HALF_MAX)))), lock);
}
