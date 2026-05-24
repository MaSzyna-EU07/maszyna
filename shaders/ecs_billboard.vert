// ECS billboard instanced vertex shader.
// Per-instance: camera-relative position and size.
// UVs are generated per-corner from gl_VertexID.
layout(location = 0) in vec3  i_position;
layout(location = 1) in float i_size;

out vec2 f_uv;
out vec4 f_pos;

#include <common>

const vec2 corners[6] = vec2[6](
    vec2(-0.5, -0.5), vec2( 0.5, -0.5), vec2( 0.5,  0.5),
    vec2(-0.5, -0.5), vec2( 0.5,  0.5), vec2(-0.5,  0.5)
);
const vec2 uvs[6] = vec2[6](
    vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
    vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0)
);

void main()
{
    int idx = gl_VertexID;
    vec3 right = vec3(inv_view[0]);
    vec3 up    = vec3(inv_view[1]);
    vec3 worldPos = i_position + (right * corners[idx].x + up * corners[idx].y) * i_size;
    f_pos       = modelview * vec4(worldPos, 1.0);
    gl_Position = projection * f_pos;
    f_uv        = uvs[idx];
}
