#version 460 core
#define STAGE_GEOMETRY 1
// draws the geometry into the six faces of a cube shadow map at once, one invocation per face. the input is in the space of the
// map, with the lamp at the origin. the faces go in the order of the faces of a cube map (+x, -x, +y, -y, +z, -z), each looks the
// way glm::lookAt() with the usual up vectors of cube maps sets it, with a 90 degree projection of infinite reach and reversed
// depth like the one of the scene: the depth is near / d, d the distance along the axis of the face
layout(triangles, invocations = 6) in;
layout(triangle_strip, max_vertices = 3) out;

layout(location = 20) in vec3 g_pos[];
layout(location = 18) in vec2 g_coord[];
layout(location = 2) out vec2 f_coord;

// near plane of the map, metres: closer surfaces (the lamp itself) cast no shadows
layout(location = 0) uniform float cube_near;
// layer of the cube map array the first face of the map is in
layout(location = 1) uniform int cube_layer;

// right, up and forward axes of the faces
const vec3 face_right[6] = vec3[](vec3(0.0, 0.0, -1.0), vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0));
const vec3 face_up[6] = vec3[](vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0), vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0));
const vec3 face_forward[6] = vec3[](vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0));

void main()
{
	int face = gl_InvocationID;
	vec4 clip[3];
	for (int i = 0; i < 3; i++)
		clip[i] = vec4(dot(g_pos[i], face_right[face]), dot(g_pos[i], face_up[face]), cube_near, dot(g_pos[i], face_forward[face]));
	// triangles wholly outside of the face are not sent further
	if ((clip[0].w < cube_near) && (clip[1].w < cube_near) && (clip[2].w < cube_near))
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
		gl_Layer = cube_layer + face;
		gl_Position = clip[i];
		f_coord = g_coord[i];
		EmitVertex();
	}
	EndPrimitive();
}
