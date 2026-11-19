#version 460 core
// the shadow of a span of the catenary in the map of a lamp: shadowcube.vert with the wires blown aside by the wind as they are drawn (wire_sway.glsl)
#extension GL_EXT_multiview : require
#define STAGE_VERTEX 1
// draws the geometry into the six faces of a cube shadow map of a lamp at once, a view of the multiview rendering per face (the
// geometry shader shadowcube.geom of the opengl renderer). the modelview takes the geometry to the space of the map, with the lamp at
// the origin. the faces go in the order of the faces of a cube map (+x, -x, +y, -y, +z, -z), each looks the way glm::lookAt() with
// the usual up vectors of cube maps sets it, with a 90 degree projection of infinite reach and reversed depth like the one of the
// scene: the depth is near / d, d the distance along the axis of the face. the near plane of the map, shadow_fit.y of the scene block
// for the map of a scenery lamp (past the radius of its light), LOCAL_SHADOW_NEAR otherwise: closer surfaces (the lamp itself) cast no
// shadows
#include "common.glsl"
layout(location = 0) in vec3 v_vert;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_coord;

layout(location = 2) out vec2 f_coord;

// right, up and forward axes of the faces
const vec3 face_right[6] = vec3[](vec3(0.0, 0.0, -1.0), vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0));
const vec3 face_up[6] = vec3[](vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0), vec3(0.0, -1.0, 0.0), vec3(0.0, -1.0, 0.0));
const vec3 face_forward[6] = vec3[](vec3(1.0, 0.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, -1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, -1.0));

#include "wire_sway.glsl"

void main()
{
	vec3 position = (effective_modelview() * vec4(wire_sway(v_vert, v_normal, v_coord), 1.0f)).xyz;
	int face = int(gl_ViewIndex);
	float nearplane = (shadow_fit.y > 0.0) ? shadow_fit.y : LOCAL_SHADOW_NEAR;
	gl_Position = vec4(dot(position, face_right[face]), dot(position, face_up[face]), nearplane, dot(position, face_forward[face]));
	f_coord = v_coord;
#ifdef MODEL_DRAWS
	// which block of the multi draw the fragments read (common.glsl)
	f_model_draw = MODEL_DRAW;
#endif
}
