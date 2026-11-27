#version 460 core
#define STAGE_FRAGMENT 1
#include "common.glsl"
// heightmap terrain with holes cut in it, for the portals of tunnels and the like (opengl46_terrain_materials): the cells cut
// out entirely are left out of the mesh, the rest of the openings is cut here. the mask of the holes covers the chunk, 255
// where the ground is cut; the fragments where the value led between its samples is over the half are discarded
#define TERRAIN_HOLES 1

#pragma texture(holes, 7, R)
layout(set = MATERIAL_SET, binding = MATERIAL_TEXTURE_FIRST + 7) uniform sampler2D holes;

#include "mat_terrain.glsl"
