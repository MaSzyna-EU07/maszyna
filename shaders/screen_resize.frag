// image:resize of the script screens on the gpu (rendering/opengl33screenpost.cpp): one pass of Pillow's resize with
// the Lanczos filter, along the rows (mode 0) or along the columns (mode 1), rounded to 8 bits, as resize_lanczos in
// scripting/scriptscreenkernels.cpp does it

uniform sampler2D source; // the image, from the top row down
uniform ivec2 source_size;
uniform ivec2 target_size;
uniform int flip; // 1: the rows are written bottom row first, as the screen's texture takes them
uniform int mode;

layout(location = 0) out vec4 out_color;

const float pi = 3.14159265358979;

float sinc(float x)
{
	if (x == 0.0)
		return 1.0;
	x *= pi;
	return sin(x) / x;
}

float lanczos(float x)
{
	if (x >= -3.0 && x < 3.0)
		return sinc(x) * sinc(x / 3.0);
	return 0.0;
}

void main()
{
	ivec2 at = ivec2(gl_FragCoord.xy);
	if (flip != 0)
		at.y = target_size.y - 1 - at.y;
	bool vertical = mode != 0;
	int index = vertical ? at.y : at.x;
	int from_size = vertical ? source_size.y : source_size.x;
	int to_size = vertical ? target_size.y : target_size.x;

	float scale = float(from_size) / float(to_size);
	float filterscale = max(scale, 1.0);
	float support = 3.0 * filterscale;
	float center = (float(index) + 0.5) * scale;
	float ss = 1.0 / filterscale;
	int first = max(int(center - support + 0.5), 0);
	int count = min(int(center + support + 0.5), from_size) - first;

	float total = 0.0;
	for (int x = 0; x < count; ++x)
		total += lanczos((float(x + first) - center + 0.5) * ss);

	vec3 sum = vec3(0.0);
	for (int x = 0; x < count; ++x)
	{
		float weight = lanczos((float(x + first) - center + 0.5) * ss);
		if (total != 0.0)
			weight /= total;
		ivec2 tap = vertical ? ivec2(at.x, first + x) : ivec2(first + x, at.y);
		sum += round(texelFetch(source, tap, 0).rgb * 255.0) * weight;
	}
	out_color = vec4(clamp(floor(sum + 0.5), 0.0, 255.0) / 255.0, 1.0);
}
