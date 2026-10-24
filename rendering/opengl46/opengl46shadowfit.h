/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "gl/ubo.h"
#include "gl/ssbo.h"
#include "gl/shader.h"
#include "gl/query.h"
#include "gl/readback.h"

// the cascades of the sun shadows fitted to the depth of the main view (sample distribution shadow maps, gfx.shadow.sdsm).
// at the end of the colour pass of the main view two compute passes go over its resolved depth: the nearest and the farthest
// distance of the pixels which aren't the sky (shadow_fit_depth.comp), then the distances split between the cascades and,
// for each cascade, the box of the pixels it takes in the space of the light (shadow_fit_bounds.comp). the result comes back
// to the cpu a frame or two later without waiting (gl::readback), with the camera and the light it was measured with, and
// the renderer makes the cascades of the frame cover the boxes (opengl46_renderer::Shadow_Fit()) instead of the whole slices
// of the view between the fixed distances of shadowtune
class opengl46_shadow_fit
{
  public:
	// the view and the light a measurement was made with
	struct view
	{
		glm::dvec3 camera; // position of the camera
		glm::mat4 projection; // projection of the view
		glm::mat3 rotation; // world to view space, rotation only
		glm::mat3 light; // world to the space of the light, rotation only
		float range{0.f}; // the farthest distance the cascades may reach
	};
	// a measurement which came back
	struct result
	{
		gl::shadow_fit_data data;
		view source;
		unsigned int frame{0}; // frame of the renderer it was made in
	};

	opengl46_shadow_fit() = default;
	~opengl46_shadow_fit();
	opengl46_shadow_fit(const opengl46_shadow_fit &) = delete;
	opengl46_shadow_fit &operator=(const opengl46_shadow_fit &) = delete;

	// creates the programs and the buffer. throws gl::shader_exception
	void init();
	// measures the depth of the view: Depth, its resolved depth texture of Width x Height pixels, or 0 to resolve the depth
	// attachment of Source, the multisampled framebuffer, into a texture of its own. Frame: the frame of the renderer.
	// texture unit 0 is used (the unit cache is reset), framebuffer bindings are left alone
	void measure(GLuint const Depth, GLuint const Source, int const Width, int const Height, view const &View, unsigned int const Frame);
	// takes in the newest measurement the gpu has finished, if any, and returns the newest one taken in so far
	result const *latest();
	// the time of the passes, for the debug panel
	std::string info_times();

	// the ends of the bounds, as the passes store them (shadow_fit_order() in shadow_fit_bounds.comp)
	static float unorder(std::uint32_t const Value);

  private:
	// the texture the depth is resolved to when there is no resolved depth to measure
	void allocate(int const Width, int const Height);
	void release();

	// a measurement on its way back, kept by its number
	struct pending
	{
		std::uint32_t measurement{0}; // 0: none
		view source;
		unsigned int frame{0};
	};
	static constexpr std::size_t PENDING = 8; // more than the copies gl::readback keeps in flight

	std::unique_ptr<gl::program> m_depthprogram;
	std::unique_ptr<gl::program> m_boundsprogram;
	std::unique_ptr<gl::ssbo> m_buffer;
	std::unique_ptr<gl::readback> m_readback;
	std::array<pending, PENDING> m_pending;
	std::uint32_t m_measurement{0}; // number of the last measurement
	std::optional<result> m_latest;
	GLuint m_depth{0};
	GLuint m_depthfb{0};
	glm::ivec2 m_depthsize{0};
	gl::gpu_timer m_timer;
};
