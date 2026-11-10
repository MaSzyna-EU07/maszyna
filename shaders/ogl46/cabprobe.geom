#version 460 core
#define STAGE_GEOMETRY 1
// draws the cab into the six faces of its probe at once, one invocation per face (as shadowcube.geom draws the cube shadow maps of
// the lamps). the input is in the space of the probe; the faces go in the order of the faces of a cube map (+x, -x, +y, -y, +z,
// -z), each looks the way glm::lookAt() with the usual up vectors of cube maps sets it, with a 90 degree projection of infinite
// reach and reversed depth like the one of the scene: the depth is near / d, d the distance along the axis of the face
layout(triangles, invocations = 6) in;
layout(triangle_strip, max_vertices = 3) out;

layout(location = 20) in vec3 g_pos[];
layout(location = 21) in vec3 g_normal[];
layout(location = 18) in vec2 g_coord[];
layout(location = 0) out vec3 f_normal;
layout(location = 2) out vec2 f_coord;
layout(location = 3) out vec4 f_pos;

// near plane of the probe, metres
const float PROBE_NEAR = 0.05;

// right, up and forward axes of the faces
const vec3 face_right[6] = vec3[](vec3(0.0, 0.0, -1.0), vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0));
const vec3 face_up[6] = vec3[](vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0), vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0));
const vec3 face_forward[6] = vec3[](vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0));

void main()
{
	int face = gl_InvocationID;
	vec4 clip[3];
	for (int i = 0; i < 3; i++)
		clip[i] = vec4(dot(g_pos[i], face_right[face]), dot(g_pos[i], face_up[face]), PROBE_NEAR, dot(g_pos[i], face_forward[face]));
	// triangles wholly outside of the face are not sent further
	if ((clip[0].w < PROBE_NEAR) && (clip[1].w < PROBE_NEAR) && (clip[2].w < PROBE_NEAR))
		return;
	if ((clip[0].x > clip[0].w) && (clip[1].x > clip[1].w) && (clip[2].x > clip[2].w))
		return;
	if ((clip[0].x < -clip[0].w) && (clip[1].x < -clip[1].w) && (clip[2].x < -clip[2].w))
		return;
	if ((clip[0].y > clip[0].w) && (clip[1].y > clip[1].w) && (clip[2].y > clip[2].w))
		return;
	if ((clip[0].y < -clip[0].w) && (clip[1].y < -clip[1].w) && (clip[2].y < -clip[2].w))
		return;
	for (int i = 0; i < 3; i++)
	{
		gl_Layer = face;
		gl_Position = clip[i];
		f_normal = g_normal[i];
		f_coord = g_coord[i];
		f_pos = vec4(g_pos[i], 1.0);
		EmitVertex();
	}
	EndPrimitive();
}
