layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;
layout(location = 3) in vec4 v_tangent;
layout(location = 4) in vec4 v_userdata;

#include <common>

out vec3 f_normal;
flat out vec3 f_normal_raw;
out vec2 f_coord;
out vec4 f_pos;
out mat3 f_tbn;
out vec4 f_light_pos[MAX_CASCADES];

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
	for (uint idx = 0U ; idx < MAX_CASCADES ; ++idx) {
		f_light_pos[idx] = lightview[idx] * f_pos;
	}
	f_clip_pos = projection * f_pos;
	f_clip_future_pos = (projection * future) * f_pos;

	gl_Position = f_clip_pos;
	gl_PointSize = param[1].x;

	vec3 T = normalize(mvn * v_tangent.xyz);
	vec3 N = f_normal;
	vec3 B = normalize(cross(N, T));
	f_tbn = mat3(T, B, N);

	mat3 TBN = transpose(f_tbn);
//	TangentLightPos = TBN * f_light_pos.xyz;
//	TangentViewPos = TBN * vec3(0.0, 0.0, 0.0);
	TangentFragPos = TBN * f_pos.xyz;
	UserData = v_userdata;
}
