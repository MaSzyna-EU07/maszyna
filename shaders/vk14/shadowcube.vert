#version 460 core
#extension GL_EXT_multiview : require
#define STAGE_VERTEX 1
// draws the geometry into the six faces of a cube shadow map of a lamp at once, a view of the multiview rendering per face (the
// geometry shader shadowcube.geom of the opengl renderer). the modelview takes the geometry to the space of the map, with the lamp at
// the origin. the faces go in the order of the faces of a cube map (+x, -x, +y, -y, +z, -z), each looks the way glm::lookAt() with
// the usual up vectors of cube maps sets it, with a 90 degree projection of infinite reach and reversed depth like the one of the
// scene: the depth is near / d, d the distance along the axis of the face. the near plane of the map, LOCAL_SHADOW_NEAR: closer
// surfaces (the lamp itself) cast no shadows
#include "common.glsl"
layout(location = 0) in vec3 v_vert;
layout(location = 2) in vec2 v_coord;

layout(location = 2) out vec2 f_coord;

// right, up and forward axes of the faces
const vec3 face_right[6] = vec3[](vec3(0.0, 0.0, -1.0), vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0));
const vec3 face_up[6] = vec3[](vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0), vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0));
const vec3 face_forward[6] = vec3[](vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0));

void main()
{
	vec3 position = (effective_modelview() * vec4(v_vert, 1.0f)).xyz;
	int face = int(gl_ViewIndex);
	gl_Position = vec4(dot(position, face_right[face]), dot(position, face_up[face]), LOCAL_SHADOW_NEAR, dot(position, face_forward[face]));
	f_coord = v_coord;
#ifdef MODEL_DRAWS
	// which block of the multi draw the fragments read (common.glsl)
	f_model_draw = MODEL_DRAW;
#endif
}
