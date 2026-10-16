/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46localshadows.h"

#include "gl/framebuffer.h"
#include "gl/ubo.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

namespace
{

// near plane of the maps, metres. surfaces closer to the lamp (its own fitting) cast no shadow
float const SHADOW_NEAR{0.15f};
// uniform locations of shadowcube.geom
GLint const NEAR_LOCATION{0};
GLint const LAYER_LOCATION{1};
// layer of the map of the cab lamp in the array
int const CAB_SLOT{0};

} // namespace

opengl46_local_shadows::~opengl46_local_shadows()
{
	release();
	if (m_empty != 0)
		glDeleteTextures(1, &m_empty);
}

void opengl46_local_shadows::init()
{
	gl::shader vertex("shadowcube.vert");
	gl::shader geometry("shadowcube.geom");
	{
		gl::shader fragment("shadowmap.frag");
		m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, geometry, fragment});
	}
	{
		gl::shader fragment("alphashadowmap.frag");
		m_alpha_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, geometry, fragment});
	}
	glProgramUniform1f(*m_program, NEAR_LOCATION, SHADOW_NEAR);
	glProgramUniform1f(*m_alpha_program, NEAR_LOCATION, SHADOW_NEAR);
	glProgramUniform1i(*m_program, LAYER_LOCATION, CAB_SLOT * 6);
	glProgramUniform1i(*m_alpha_program, LAYER_LOCATION, CAB_SLOT * 6);

	// a shadow sampler with no depth texture to read from is undefined, also where the shading doesn't read it
	glCreateTextures(GL_TEXTURE_CUBE_MAP_ARRAY, 1, &m_empty);
	glTextureStorage3D(m_empty, 1, GL_DEPTH_COMPONENT32F, 1, 1, 6);
	glTextureParameteri(m_empty, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(m_empty, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTextureParameteri(m_empty, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
	glTextureParameteri(m_empty, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	float const farthest{0.f};
	glClearTexImage(m_empty, 0, GL_DEPTH_COMPONENT, GL_FLOAT, &farthest);
}

opengl46_local_shadows::level_setup opengl46_local_shadows::setup(int const Level)
{
	switch (Level)
	{
	case 1:
		return {512, 4, 0.f};
	case 2:
		return {1024, 2, 1.f};
	case 3:
		return {1024, 1, 2.f};
	default:
		return {0, 0, 0.f};
	}
}

void opengl46_local_shadows::allocate(int const Size)
{
	release();
	if (Size <= 0)
		return;
	glCreateTextures(GL_TEXTURE_CUBE_MAP_ARRAY, 1, &m_texture);
	glTextureStorage3D(m_texture, 1, GL_DEPTH_COMPONENT32F, Size, Size, 6 * (CAB_SLOT + 1));
	// one level: the lookups need no derivatives, also in the loops over the lights the fragments of a quad don't go through together
	glTextureParameteri(m_texture, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTextureParameteri(m_texture, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureParameteri(m_texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(m_texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTextureParameteri(m_texture, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
	// 1 (shadow) where the map holds n / d at least as large as the reference: a surface at most as far from the lamp
	glTextureParameteri(m_texture, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);

	glCreateFramebuffers(1, &m_framebuffer);
	// all layers: the geometry shader picks the face
	glNamedFramebufferTexture(m_framebuffer, GL_DEPTH_ATTACHMENT, m_texture, 0);
	glNamedFramebufferDrawBuffer(m_framebuffer, GL_NONE);
	glNamedFramebufferReadBuffer(m_framebuffer, GL_NONE);
	if (glCheckNamedFramebufferStatus(m_framebuffer, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		ErrorLog("lamp shadow map framebuffer setup failed, the lamps have no shadows");
		release();
		Global.gfx_lights_shadows = 0;
		return;
	}
	m_size = Size;
	WriteLog("lamp shadow maps: " + std::to_string(Size) + " x " + std::to_string(Size) + " cube");
}

void opengl46_local_shadows::release()
{
	if (m_framebuffer != 0)
		glDeleteFramebuffers(1, &m_framebuffer);
	if (m_texture != 0)
		glDeleteTextures(1, &m_texture);
	m_framebuffer = 0;
	m_texture = 0;
	m_size = 0;
	m_valid = false;
}

void opengl46_local_shadows::bind()
{
	glBindTextureUnit(gl::LOCAL_SHADOW_TEX, (m_texture != 0) ? m_texture : m_empty);
}

bool opengl46_local_shadows::update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Lamp, unsigned int const Framestamp)
{
	auto const level{std::clamp(Global.gfx_lights_shadows, 0, 3)};
	if (level != m_level)
	{
		m_level = level;
		auto const size{setup(level).size};
		if (size != m_size)
			allocate(size);
		m_valid = false;
	}
	if ((m_size == 0) || (Vehicle == nullptr) || (Cab == nullptr))
	{
		// with the lamp off nothing reads the map; once it is on again the cab may have changed
		m_valid = false;
		return false;
	}
	if ((false == m_valid) || (Vehicle != m_vehicle) || (Cab != m_cab) || (Lamp != m_lamp))
	{
		m_valid = false;
	}
	// the moving parts of the cab are followed every few frames
	if (m_valid && (Framestamp - m_drawnframe < setup(level).interval))
	{
		return false;
	}
	m_vehicle = Vehicle;
	m_cab = Cab;
	m_lamp = Lamp;
	m_drawnframe = Framestamp;
	return true;
}

void opengl46_local_shadows::begin()
{
	m_cpustart = std::chrono::steady_clock::now();
	m_timer.begin();
	// the framebuffer is bound past the cache of gl::framebuffer, which end() resets
	gl::framebuffer::unbind();
	glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
	glViewport(0, 0, m_size, m_size);
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	// one sided walls of the cab face the lamp, they are to cast shadows as well
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	float const farthest{0.f};
	glClearNamedFramebufferfv(m_framebuffer, GL_DEPTH, 0, &farthest);
}

void opengl46_local_shadows::end(std::uint32_t const Drawcalls)
{
	glEnable(GL_CULL_FACE);
	gl::framebuffer::unbind();
	m_timer.end();
	m_valid = true;
	m_drawcalls = Drawcalls;
	++m_updatecount;
	auto const elapsed{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - m_cpustart).count()};
	m_cputime = (m_cputime == 0.0 ? elapsed : m_cputime * 0.9 + elapsed * 0.1);
}

glm::vec4 opengl46_local_shadows::weights() const
{
	// a face spans twice the distance from the lamp
	return glm::vec4{static_cast<float>(CAB_SLOT), SHADOW_NEAR, 2.f / static_cast<float>(std::max(m_size, 1)), setup(m_level).radius};
}

std::string opengl46_local_shadows::info_times()
{
	std::string text;
	if (auto const time = m_timer.result())
		text += " lamp shadows: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}

std::string opengl46_local_shadows::info() const
{
	if (m_size == 0)
		return "";
	return "\nlamp shadows: " + std::to_string(m_size) + " cube, " + (m_valid ? "cab lamp" : "none") + ", " + std::to_string(m_drawcalls) + " drawcalls, cpu " + to_string(m_cputime, 2) +
	       " ms";
}

void opengl46_local_shadows::draw_debug_ui()
{
	char const *levels[]{"no", "low (512, every 4th frame)", "medium (1024, every 2nd frame)", "high (1024, every frame, softer)"};
	ImGui::Combo("lamp shadows (gfx.lights.shadows)", &Global.gfx_lights_shadows, levels, IM_ARRAYSIZE(levels));
	if (m_size == 0)
		return;
	ImGui::Text("cube %d x %d, %s, drawn %u times, %u drawcalls, %.2f ms (cpu)", m_size, m_size, m_valid ? "cab lamp" : "no lamp", m_updatecount, m_drawcalls, m_cputime);
}
