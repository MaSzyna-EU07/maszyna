layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;
layout(location = 3) in vec4 v_tangent;
layout(location = 4) in vec4 v_userdata;

#include <common>

// the depth prepass (depthprepass.vert) works the position out the same way, and has to get the same depth
invariant gl_Position;

out vec3 f_normal;
flat out vec3 f_normal_raw;
out vec2 f_coord;
out vec4 f_pos;
out mat3 f_tbn;

out vec4 f_clip_pos;
out vec4 f_clip_future_pos;

//out vec3 TangentLightPos;
//out vec3 TangentViewPos;
out vec3 TangentFragPos;
out vec4 UserData;

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
	// which model block of the multi draw the fragments read (glsl_common)
	f_model_draw = uint(gl_DrawID);
#endif
}
