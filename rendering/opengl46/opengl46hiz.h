/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <memory>
#include <string>

#include "gl/shader.h"
#include "gl/query.h"

// depth pyramid of the occlusion test of the instance culling (instance_cull.comp). made in the middle of the colour
// pass of the main view, from the depth drawn up to then (cab, terrain, shapes, tracks, vehicles and the models drawn on
// their own), so it needs no reprojection: the instances are tested against what is already in the image.
// the multisampled depth is resolved by a blit, then each level of the pyramid (hiz_build.comp) keeps the farthest
// depth of the 2x2 texels below it; the first level is half the size of the view
class opengl46_hiz
{
  public:
	opengl46_hiz() = default;
	~opengl46_hiz();
	opengl46_hiz(const opengl46_hiz &) = delete;
	opengl46_hiz &operator=(const opengl46_hiz &) = delete;

	// creates the program. throws gl::shader_exception
	void init();
	// makes the pyramid from the depth attachment of Source, a framebuffer of Width x Height pixels.
	// leaves the framebuffer bindings alone, texture unit 0 and image unit 0 are used (the unit cache is reset)
	void build(GLuint const Source, int const Width, int const Height);
	// the pyramid, its first level size and level count; valid after build()
	GLuint texture() const
	{
		return m_pyramid;
	}
	glm::ivec2 size() const
	{
		return m_size;
	}
	int levels() const
	{
		return m_levels;
	}
	glm::ivec2 source_size() const
	{
		return m_sourcesize;
	}
	std::string info_times();

  private:
	// (re)creates the textures for a view of the given size
	void allocate(int const Width, int const Height);
	void release();

	std::unique_ptr<gl::program> m_program;
	GLuint m_depth{0}; // resolved depth, the format of the scene depth
	GLuint m_depthfb{0};
	GLuint m_pyramid{0}; // r32f, m_levels levels
	glm::ivec2 m_sourcesize{0};
	glm::ivec2 m_size{0};
	int m_levels{0};
	gl::gpu_timer m_timer;
};
