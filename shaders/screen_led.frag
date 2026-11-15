// image:led_post of the script screens on the gpu (rendering/opengl33screenpost.cpp), the pixels of led_post in
// scripting/scriptscreenkernels.cpp (led_util.render_post): each diode becomes 2x2 pixels with a darker grid between
// them, glowing into its neighbours, in a 1 pixel black frame

uniform sampler2D source; // the image, from the top row down
uniform ivec2 source_size;
uniform ivec2 target_size;
uniform int flip; // 1: the rows are written bottom row first, as the screen's texture takes them
uniform int mode; // 1: with the glow

layout(location = 0) out vec4 out_color;

const int glow_radius = 6;
const float glow_factor = 0.08;

ivec3 diode(ivec2 at)
{
	return ivec3(round(texelFetch(source, at, 0).rgb * 255.0));
}

// the light gathered from the diodes around, weighted 1 / (0.08 + distance squared)
ivec3 glow(ivec2 at)
{
	vec3 sum = vec3(0.0);
	int left = max(at.x - glow_radius, 0);
	int right = min(at.x + glow_radius + 1, source_size.x);
	int top = max(at.y - glow_radius, 0);
	int bottom = min(at.y + glow_radius + 1, source_size.y);
	for (int i = left; i < right; ++i)
	{
		float m = glow_factor + float((i - at.x) * (i - at.x));
		if (m > 20.0)
			continue;
		for (int j = top; j < bottom; ++j)
		{
			float s = m + float((j - at.y) * (j - at.y));
			if (s > 20.0)
				continue;
			sum += vec3(diode(ivec2(i, j))) / s;
		}
	}
	return ivec3(sum * glow_factor);
}

// the light through the grid, rounded down as led_util gets it in double precision. there 1 - 0.15 gives the exact
// result, but 1 - 2 * 0.15 is a little below 0.7, which takes these lights one lower (above 360 the result is clamped
// to 255 anyway)
int shaded(int light, int grid)
{
	if (grid == 0)
		return light;
	if (grid == 1)
		return light * 85 / 100;
	int result = light * 70 / 100;
	if (light == 90 || light == 170 || light == 180 || (light >= 330 && light <= 360 && light % 10 == 0))
		result -= 1;
	return result;
}

void main()
{
	ivec2 at = ivec2(gl_FragCoord.xy);
	if (flip != 0)
		at.y = target_size.y - 1 - at.y;
	if (at.x == 0 || at.y == 0 || at.x == target_size.x - 1 || at.y == target_size.y - 1)
	{
		out_color = vec4(0.0, 0.0, 0.0, 1.0);
		return;
	}
	ivec2 pixel = at - 1;
	ivec2 lit = pixel / 2;
	ivec3 light = (mode != 0) ? glow(lit) : diode(lit);
	int grid = pixel.x % 2 + pixel.y % 2;
	ivec3 result = ivec3(shaded(light.r, grid), shaded(light.g, grid), shaded(light.b, grid));
	out_color = vec4(vec3(clamp(result, 0, 255)) / 255.0, 1.0);
}
