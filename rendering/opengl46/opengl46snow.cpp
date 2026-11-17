/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46snow.h"

#include <format>

#include "gl/ubo.h"
#include "rendering/opengl46/opengl46skyocclusion.h"
#include "environment/weather.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace
{

GLuint groups(int const Size)
{
	return static_cast<GLuint>((Size + static_cast<int>(gl::SNOW_GROUP_SIZE) - 1) / static_cast<int>(gl::SNOW_GROUP_SIZE));
}

double wrap(double const Value, double const Size)
{
	auto const result{std::fmod(Value, Size)};
	return (result < 0.0) ? result + Size : result;
}

// the snow is there to be drawn at all
bool snowing_scenery()
{
	auto const &snow{simulation::Environment.weather().snow()};
	return (Global.gfx_snow > 0) && ((snow.depth() > 0.0005f) || (snow.plaster() > 0.01f));
}

} // namespace

opengl46_snow::~opengl46_snow()
{
	release();
	if (m_standin != 0)
		glDeleteBuffers(1, &m_standin);
}

void opengl46_snow::init()
{
	if (m_state != 0)
		return;
	// a buffer with nothing to read from is undefined, also where the shading doesn't read it
	glCreateBuffers(1, &m_standin);
	glm::vec4 const none{-1.0e4f, 0.f, 1.0e4f, 0.f};
	glNamedBufferStorage(m_standin, sizeof(glm::vec4), &none, 0);
	try
	{
		gl::shader const drift("snow_drift.comp");
		m_drift = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{drift});
		gl::shader const settle("snow_settle.comp");
		m_settle = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{settle});
		gl::shader const vertex("snow_surface.vert");
		gl::shader const geometry("snow_surface.geom");
		gl::shader const fragment("snow_surface.frag");
		m_surface = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, geometry, fragment});
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("snow: the shaders can't be used, the snow lies evenly and without drifts (" + std::string(e.what()) + ")");
		m_state = -1;
		bind();
		return;
	}
	m_vao = std::make_unique<gl::vao>();
	m_state = 1;
	bind();
}

void opengl46_snow::release()
{
	if (m_cells != 0)
		glDeleteBuffers(1, &m_cells);
	if (m_drifted != 0)
		glDeleteTextures(1, &m_drifted);
	m_cells = 0;
	m_drifted = 0;
	m_side = 0;
	m_active = false;
}

bool opengl46_snow::allocate(int const Side)
{
	release();
	auto const cells{static_cast<GLsizeiptr>(Side) * Side * static_cast<GLsizeiptr>(sizeof(glm::vec4))};
	glCreateBuffers(1, &m_cells);
	glNamedBufferStorage(m_cells, cells, nullptr, 0);
	glCreateTextures(GL_TEXTURE_2D, 1, &m_drifted);
	glTextureStorage2D(m_drifted, 1, GL_RG32F, Side, Side);
	glTextureParameteri(m_drifted, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(m_drifted, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	if ((m_cells == 0) || (m_drifted == 0))
	{
		release();
		return false;
	}
	m_side = Side;
	WriteLog(std::format("snow: the drifts worked out over {0}x{0} cells", Side));
	return true;
}

void opengl46_snow::update(opengl46_sky_occlusion const &Maps, glm::dvec3 const &Camera)
{
	m_active = false;
	if ((m_state != 1) || (false == snowing_scenery()) || (false == Maps.active()) || (Maps.surface_texture() == 0))
	{
		bind();
		return;
	}
	auto const side{Maps.side()};
	if ((side != m_side) && (false == allocate(side)))
	{
		bind();
		return;
	}
	// the texel of the camera in the toroidal maps, as gfx::sky_occlusion::setup_scene() places it
	auto const texel{static_cast<double>(Maps.map_texel())};
	auto const size{static_cast<double>(Maps.map_size())};
	glm::ivec2 const camera{static_cast<int>(std::floor(wrap(Camera.x, size) / texel)), static_cast<int>(std::floor(wrap(Camera.z, size) / texel))};
	auto const reach{Maps.map_reach()};

	// the depths worked by the wind
	m_drift->bind();
	glBindTextureUnit(0, Maps.surface_texture());
	glBindImageTexture(0, m_drifted, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RG32F);
	glProgramUniform4i(*m_drift, 0, side, camera.x, camera.y, 0);
	glProgramUniform4f(*m_drift, 1, static_cast<float>(texel), reach * 0.6f, reach * 0.3f, 0.f);
	m_drift->dispatch(groups(side), groups(side), 1);
	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
	glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RG32F);

	// smoothed into the cells
	m_settle->bind();
	glBindTextureUnit(0, Maps.surface_texture());
	glBindTextureUnit(1, m_drifted);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, gl::SNOW_SSBO_BINDING, m_cells);
	glProgramUniform4i(*m_settle, 0, side, 0, 0, 0);
	m_settle->dispatch(groups(side), groups(side), 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_active = true;
	// the grid of the surface covers the filled part of the maps
	m_across = std::min(side, static_cast<int>(2.f * reach / Maps.map_texel()) + 1);
	bind();
}

void opengl46_snow::bind() const
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, gl::SNOW_SSBO_BINDING, m_active ? m_cells : m_standin);
}

bool opengl46_snow::drawn() const
{
	return m_active && (Global.gfx_snow >= 2) && (m_across > 1) && (simulation::Environment.weather().snow().depth() > 0.02f);
}

std::size_t opengl46_snow::render(glm::mat4 const &View, std::function<void(glm::mat4 const &)> const &Modelview)
{
	if (false == drawn())
		return 0;
	Modelview(View);
	m_surface->bind();
	m_vao->bind();
	glProgramUniform4i(*m_surface, 0, m_across, 1, 0, 0);
	glDisable(GL_CULL_FACE);
	auto const quads{static_cast<std::size_t>(m_across - 1) * static_cast<std::size_t>(m_across - 1)};
	glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(quads * 6));
	glEnable(GL_CULL_FACE);
	gl::vao::unbind();
	return quads * 2;
}

std::string opengl46_snow::info() const
{
	auto const &snow{simulation::Environment.weather().snow()};
	return std::format("snow: {:.2f} m, drifts {:.0f}%, loose {:.0f}%, wet {:.0f}%, blowing {:.0f}%, wire ice {:.1f} mm{}", snow.depth(), snow.drift() * 100.f, snow.fresh() * 100.f, snow.wetness() * 100.f,
	                   snow.blowing() * 100.f, snow.wire_ice() * 1000.f, m_active ? "" : " (no drifts)");
}
