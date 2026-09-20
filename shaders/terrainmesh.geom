// Terrain mesh, geometry stage.
//
// It is here only to tell the fragment stage which three materials the triangle it is in has at its
// corners, and how near that fragment is to each of them. A vertex carries one material; a triangle
// crossing a boundary between two kinds of ground has different ones at its corners, and what makes
// that read as a transition rather than as a line along the triangle's edge is blending the three by
// their barycentric weight.
//
// Nothing else needs a whole stage: the weights cannot be worked out in the fragment stage on its own,
// since an indexed mesh shares a vertex between triangles and a vertex cannot say which corner of which
// triangle it is. Desktop GL 4.6 has gl_BaryCoordEXT for exactly this; 3.3 does not.

#include <common>

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in vec4 v_pos[];
in vec3 v_normal[];
in vec2 v_ground[];
in uint v_material[];

out vec4 f_pos;
out vec3 f_normal;
out vec2 f_ground;
out vec3 f_weights;
flat out uvec3 f_materials;

void main()
{
	uvec3 materials = uvec3( v_material[ 0 ], v_material[ 1 ], v_material[ 2 ] );
	for( int corner = 0; corner < 3; ++corner ) {
		f_pos = v_pos[ corner ];
		f_normal = v_normal[ corner ];
		f_ground = v_ground[ corner ];
		f_weights = vec3( corner == 0 ? 1.0 : 0.0, corner == 1 ? 1.0 : 0.0, corner == 2 ? 1.0 : 0.0 );
		f_materials = materials;
		gl_Position = gl_in[ corner ].gl_Position;
		EmitVertex();
	}
	EndPrimitive();
}
