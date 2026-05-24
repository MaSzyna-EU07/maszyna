// ECS particle instanced billboard vertex shader.
// Per-instance data: camera-relative world position, size, color.
// Billboard corners are generated via gl_VertexID (no geometry shader needed).
layout(location = 0) in vec3  i_position;  // camera-relative world position
layout(location = 1) in float i_size;
layout(location = 2) in vec4  i_color;

out vec4 f_color;
out vec4 f_pos;

#include <common>

// Two-triangle billboard quad (CCW winding)
const vec2 corners[6] = vec2[6](
    vec2(-0.5, -0.5), vec2( 0.5, -0.5), vec2( 0.5,  0.5),
    vec2(-0.5, -0.5), vec2( 0.5,  0.5), vec2(-0.5,  0.5)
);

void main()
{
    vec2 corner = corners[gl_VertexID];

    // inv_view is the inverse of the rotation-only view matrix.
    // Its columns are the camera's right, up, forward vectors in world space.
    vec3 right = vec3(inv_view[0]);
    vec3 up    = vec3(inv_view[1]);

    // Expand billboard in world space around the camera-relative centre.
    vec3 worldPos = i_position + (right * corner.x + up * corner.y) * i_size;

    f_pos       = modelview * vec4(worldPos, 1.0);
    gl_Position = projection * f_pos;
    f_color     = i_color;
}
