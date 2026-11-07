#version 460 core
#define STAGE_VERTEX 1
#include "common.glsl"
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;
layout(location = 3) in vec4 v_tangent;
layout(location = 4) in vec4 v_userdata;


// the depth prepass (depthprepass.vert) works the position out the same way, and has to get the same depth
invariant gl_Position;

layout(location = 0) out vec3 f_normal;
layout(location = 1) flat out vec3 f_normal_raw;
layout(location = 2) out vec2 f_coord;
layout(location = 3) out vec4 f_pos;
layout(location = 8) out mat3 f_tbn;

layout(location = 4) out vec4 f_clip_pos;
layout(location = 5) out vec4 f_clip_future_pos;

//out vec3 TangentLightPos;
//out vec3 TangentViewPos;
layout(location = 6) out vec3 TangentFragPos;
layout(location = 7) out vec4 UserData;

void main()
{
	mat4 mv = effective_modelview();
	mat3 mvn = effective_modelviewnormal();

	f_normal = normalize(mvn * v_normal);
	f_normal_raw = v_normal;
	f_coord = v_coord;
	f_pos = mv * vec4(v_vert, 1.0);
	f_clip_pos = projection * f_pos;
	f_clip_future_pos = (projection * future) * f_pos;

	gl_Position = f_clip_pos;
	gl_PointSize = param[1].x;

	vec3 N = f_normal;
	vec3 T = mvn * v_tangent.xyz;
	T = normalize(T - N * dot(N, T));
	// w is -1 where the texture is mirrored (calculate_tangents()), the bitangent turns with it
	vec3 B = cross(N, T) * (v_tangent.w < 0.0 ? -1.0 : 1.0);
	f_tbn = mat3(T, B, N);

	mat3 TBN = transpose(f_tbn);
//	TangentLightPos = TBN * f_light_pos.xyz;
//	TangentViewPos = TBN * vec3(0.0, 0.0, 0.0);
	TangentFragPos = TBN * f_pos.xyz;
	UserData = v_userdata;
#ifdef MODEL_DRAWS
	// which block of the multi draw the fragments read (common.glsl)
	f_model_draw = MODEL_DRAW;
#endif
}
