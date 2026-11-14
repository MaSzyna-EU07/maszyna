#version 460 core
#define STAGE_VERTEX 1
// an animation baked into textures (vertex animation texture, VAT): people walking, standing, waiting. the vertices are moved
// here by the frames of the animation the textures hold, the material is drawn by mat_vat.frag (as mat_default). made by
// vat_bake.py from an animated mesh in blender; the model (t3d) carries in the userdata of every vertex (x) its column of the
// textures. the same shader draws the sun shadows (MATERIAL_SHADOW) and the multi draws (MODEL_DRAWS, opengl46_renderer)
//
// the textures (linear, never compressed or downscaled: at most gfx maxtexturesize, 2048 by default):
//   vatpos_hi, vatpos_lo: the position of a vertex in a frame, 16 bits per axis over the box of the animation (rgb: the high and
//                         the low bytes), the normal in vatnormal (rgb, 0.5 + 0.5 n)
//   the frames lie one under another, each in RowsPerFrame rows of Width vertices; the first row of vatpos_hi holds the header,
//   the last one the same header with another mark (so the shader sees whether the image came upside down):
//     pixel 0: 'V' 'A' 'T' 1 in the first row, 'V' 'A' 'T' 2 in the last
//     pixel 1: min x, min y (16 bits each: hi, lo; millimetres + 32768)    pixel 2: min z, size x (size: millimetres)
//     pixel 3: size y, size z                                             pixel 4: vertices, frames
//     pixel 5: rows per frame, frames a second x 100
//     pixel 6: clips (0 or 1: one animation); with more the frames are that many animations of equal length one after another,
//              and every one of the models plays one of them, picked as its moment is; then flags: 1 the frames packed one
//              right after another (vertex Frame * Vertices + Vertex of the whole image, row by row), else each in its own rows
// the textures go with the trait r (texture_vatpos_hi: name:r), which keeps them as they are (never scaled down nor compressed)
// param vat_speed (1 if not given): how fast the animation plays
// param vat_random (1 if not given): 1 every model its own moment from its place; 0 the moment of the clock alone, for a model the game
// moves (its place changes the whole time, the moment taken from it would jump); -1 the game picks the clip and the moment, by the
// fraction of ten times the turn of the model about the vertical in degrees: (clip + 0.25 + 0.5 * moment) / clips (world/deer.cpp)
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;
layout(location = 3) in vec4 v_tangent;
layout(location = 4) in vec4 v_userdata;

#include "common.glsl"

#pragma texture(vatpos_hi, 1, RGBA)
#pragma texture(vatpos_lo, 2, RGBA)
#pragma texture(vatnormal, 3, RGBA)
#pragma param(vat_speed, 2, 0, 1, one)
#pragma param(vat_random, 2, 1, 1, one)
layout(binding = 1) uniform sampler2D vatpos_hi;
layout(binding = 2) uniform sampler2D vatpos_lo;
layout(binding = 3) uniform sampler2D vatnormal;

#ifdef MATERIAL_SHADOW
layout(location = 2) out vec2 f_coord;
#else
invariant gl_Position;

layout(location = 0) out vec3 f_normal;
layout(location = 1) flat out vec3 f_normal_raw;
layout(location = 2) out vec2 f_coord;
layout(location = 3) out vec4 f_pos;
layout(location = 8) out mat3 f_tbn;

layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;

layout(location = 6) out vec3 TangentFragPos;
layout(location = 7) out vec4 UserData;
#endif

// the header: Flipped when the image came upside down
struct vat_header
{
	bool valid;
	bool flipped;
	int height;
	vec3 lower;
	vec3 size;
	int vertices;
	int frames;
	int rows;
	float fps;
	int clips;
	bool dense;
};

ivec4 vat_bytes(ivec2 texel)
{
	return ivec4(round(texelFetch(vatpos_hi, texel, 0) * 255.0));
}

vat_header vat_read_header()
{
	vat_header h;
	ivec2 size = textureSize(vatpos_hi, 0);
	h.height = size.y;
	ivec4 mark = vat_bytes(ivec2(0, 0));
	h.valid = (mark.x == 86) && (mark.y == 65) && (mark.z == 84) && ((mark.w == 1) || (mark.w == 2));
	h.flipped = (mark.w == 2);
	int row = 0;
	ivec4 p1 = vat_bytes(ivec2(1, row));
	ivec4 p2 = vat_bytes(ivec2(2, row));
	ivec4 p3 = vat_bytes(ivec2(3, row));
	ivec4 p4 = vat_bytes(ivec2(4, row));
	ivec4 p5 = vat_bytes(ivec2(5, row));
	ivec4 p6 = vat_bytes(ivec2(6, row));
	h.lower = (vec3(p1.x * 256 + p1.y, p1.z * 256 + p1.w, p2.x * 256 + p2.y) - 32768.0) * 0.001;
	h.size = vec3(p2.z * 256 + p2.w, p3.x * 256 + p3.y, p3.z * 256 + p3.w) * 0.001;
	h.vertices = p4.x * 256 + p4.y;
	h.frames = max(p4.z * 256 + p4.w, 1);
	h.rows = max(p5.x * 256 + p5.y, 1);
	h.fps = float(p5.z * 256 + p5.w) * 0.01;
	h.clips = clamp(p6.x * 256 + p6.y, 1, h.frames);
	h.dense = (p6.z & 1) != 0;
	return h;
}

// the texel of vertex Vertex in frame Frame
ivec2 vat_texel(vat_header h, int vertex, int frame, int width)
{
	int ix = h.dense ? frame * h.vertices + vertex : frame * h.rows * width + vertex;
	int row = 1 + ix / width;
	if (h.flipped)
		row = h.height - 1 - row;
	return ivec2(ix % width, row);
}

vec3 vat_position(vat_header h, ivec2 texel)
{
	vec3 high = round(texelFetch(vatpos_hi, texel, 0).rgb * 255.0);
	vec3 low = round(texelFetch(vatpos_lo, texel, 0).rgb * 255.0);
	return h.lower + h.size * ((high * 256.0 + low) / 65535.0);
}

vec3 vat_normal(ivec2 texel)
{
	return texelFetch(vatnormal, texel, 0).rgb * 2.0 - 1.0;
}

// the place and the normal of the vertex now (the rest pose of the model where the textures don't fit it)
void vat_animate(mat4 mv, out vec3 position, out vec3 normal)
{
	position = v_vert;
	normal = v_normal;
	vat_header h = vat_read_header();
	int vertex = int(v_userdata.x + 0.5);
	if ((false == h.valid) || (vertex < 0) || (vertex >= h.vertices))
		return;
	int width = textureSize(vatpos_hi, 0).x;
	// every one of the models a moment (and a clip) of its own: from its place in the world, whole metres, and its turn about the
	// vertical, whole 2 degrees. the place is the one of the camera of the pass (camera_hi exact, camera_lo the rest) and the model
	// relative to it (mv, turned back to the world with inv_view), so it's the same in every pass and with every move of the camera;
	// the edges of the cells and of the bins away from the round values models are set at
	mat3 world = mat3(inv_view) * mat3(mv);
	vec3 rel = camera_lo.xyz + mat3(inv_view) * mv[3].xyz;
	ivec3 cell = ivec3(camera_hi.xyz) + ivec3(floor(rel + 0.37));
	float yaw = degrees(atan(world[0].z, world[0].x));
	int bin = int(mod(floor(yaw * 0.5 + 0.315), 180.0));
	uint seed = uint(cell.x) * 73856093u ^ uint(cell.y) * 19349663u ^ uint(cell.z) * 83492791u ^ uint(bin) * 2654435761u;
	seed ^= seed >> 16; seed *= 0x7feb352du; seed ^= seed >> 15; seed *= 0x846ca68bu; seed ^= seed >> 16;
	float random = clamp(param[2].y, 0.0, 1.0);
	float phase = random * float(seed & 0xffffu) / 65536.0;
	float pick = random * float(seed >> 16) / 65536.0;
	if (param[2].y < -0.5)
	{
		// picked by the game: the fraction of the turn in tenths of a degree (the turn is -yaw), a quarter of a clip's span kept
		// clear of its edges
		float spin = fract(-yaw * 10.0) * float(h.clips);
		float slot = floor(spin);
		pick = (slot + 0.5) / float(h.clips);
		phase = clamp((spin - slot - 0.25) * 2.0, 0.0, 1.0);
	}
	int clipframes = max(h.frames / h.clips, 1);
	int clip = min(int(pick * float(h.clips)), h.clips - 1);
	float t = (time * h.fps * max(param[2].x, 0.0)) + phase * float(clipframes);
	float f = mod(t, float(clipframes));
	int f0 = clip * clipframes + int(floor(f)) % clipframes;
	int f1 = clip * clipframes + (int(floor(f)) + 1) % clipframes;
	float a = fract(f);
	ivec2 t0 = vat_texel(h, vertex, f0, width);
	ivec2 t1 = vat_texel(h, vertex, f1, width);
	position = mix(vat_position(h, t0), vat_position(h, t1), a);
	vec3 n = mix(vat_normal(t0), vat_normal(t1), a);
	normal = (dot(n, n) > 1e-6) ? normalize(n) : v_normal;
}

void main()
{
	mat4 mv = effective_modelview();
	vec3 position;
	vec3 normal;
	vat_animate(mv, position, normal);
#ifdef MATERIAL_SHADOW
	gl_Position = (projection * mv) * vec4(position, 1.0);
	f_coord = v_coord;
#else
	mat3 mvn = effective_modelviewnormal();

	f_normal = normalize(mvn * normal);
	f_normal_raw = normal;
	f_coord = v_coord;
	f_pos = mv * vec4(position, 1.0);
	f_clip_pos = projection * f_pos;
	f_clip_future_pos = (projection * future) * f_pos;

	gl_Position = f_clip_pos;
	gl_PointSize = param[1].x;

	vec3 N = f_normal;
	vec3 T = mvn * v_tangent.xyz;
	T = normalize(T - N * dot(N, T));
	vec3 B = cross(N, T) * (v_tangent.w < 0.0 ? -1.0 : 1.0);
	f_tbn = mat3(T, B, N);
	TangentFragPos = transpose(f_tbn) * f_pos.xyz;
	UserData = v_userdata;
#endif
#ifdef MODEL_DRAWS
	f_model_draw = uint(gl_DrawID);
#endif
}
