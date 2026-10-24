// one step up the bloom chain: the level below (half as large, with all the levels under it already added) blurred up with
// a 3x3 tent filter and added to this level of the way down

in vec2 f_coords;
layout(location = 0) out vec4 out_color;

#texture (lower_tex, 0, RGB)
uniform sampler2D lower_tex;

#texture (current_tex, 1, RGB)
uniform sampler2D current_tex;

void main()
{
	vec2 texel = 1.0 / vec2(textureSize(lower_tex, 0));
	vec2 uv = f_coords;

	vec3 sum = texture(lower_tex, uv).rgb * 4.0;
	sum += (texture(lower_tex, uv + texel * vec2(0.0, 1.0)).rgb
	      + texture(lower_tex, uv + texel * vec2(-1.0, 0.0)).rgb
	      + texture(lower_tex, uv + texel * vec2(1.0, 0.0)).rgb
	      + texture(lower_tex, uv + texel * vec2(0.0, -1.0)).rgb) * 2.0;
	sum += texture(lower_tex, uv + texel * vec2(-1.0, 1.0)).rgb
	     + texture(lower_tex, uv + texel * vec2(1.0, 1.0)).rgb
	     + texture(lower_tex, uv + texel * vec2(-1.0, -1.0)).rgb
	     + texture(lower_tex, uv + texel * vec2(1.0, -1.0)).rgb;

	out_color = vec4(texture(current_tex, uv).rgb + sum / 16.0, 1.0);
}
