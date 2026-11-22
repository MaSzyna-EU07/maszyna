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

#include <algorithm>
#include <array>
#include <cstdlib>
#include <format>

#include "gl/ubo.h"
#include "model/Texture.h"
#include "rendering/opengl46/opengl46skyocclusion.h"
#include "environment/weather.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace
{

// the drifts worked out again a strip of the maps a frame, all of them in so many frames
int const SNOW_STRIPS{4};
// texels the camera goes before the drifts around it are worked out again (they fade out towards the edge of the maps by it)
int const SNOW_MOVE_TEXELS{4};
// what the snow changes by before the drifts are worked out again: m of depth, the share drifted, m/s of the drifting wind
float const SNOW_DEPTH_STEP{0.002f};
float const SNOW_DRIFT_STEP{0.01f};
float const SNOW_WIND_STEP{0.25f};
// quads across a block of the grid of the surface (snow_surface.vert), the part of it left out together when out of the view
int const SNOW_BLOCK{16};
// m up and down from the camera a block of the surface is taken to reach: the heights are on the gpu only
float const SNOW_HEIGHT_REACH{5000.f};

GLuint groups(int const Size)
{
	return static_cast<GLuint>((Size + static_cast<int>(gl::SNOW_GROUP_SIZE) - 1) / static_cast<int>(gl::SNOW_GROUP_SIZE));
}

double wrap(double const Value, double const Size)
{
	auto const result{std::fmod(Value, Size)};
	return (result < 0.0) ? result + Size : result;
}

// the step from From to To in the toroidal maps of Side texels, the shorter way round
int toroidal_step(int const From, int const To, int const Side)
{
	auto const half{Side / 2};
	return (((To - From + half) % Side) + Side) % Side - half;
}

// the snow is there to be drawn at all
bool snowing_scenery()
{
	auto const &snow{simulation::Environment.weather().snow()};
	return (Global.gfx_snow > 0) && ((snow.depth() > 0.0005f) || (snow.plaster() > 0.01f));
}

// blocks across the grid of the surface of Across vertices
int blocks_across(int const Across)
{
	return (Across - 1 + SNOW_BLOCK - 1) / SNOW_BLOCK;
}

// the planes of the left, right, bottom and top of the view of Viewprojection (the clip space -w..w); a point is inside them where
// dot(xyz, plane.xyz) + plane.w >= 0. the near and the far planes are left out: the depth is reversed and the far one infinite
std::array<glm::vec4, 4> side_planes(glm::mat4 const &Viewprojection)
{
	auto const row = [&Viewprojection](int const Row) { return glm::vec4{Viewprojection[0][Row], Viewprojection[1][Row], Viewprojection[2][Row], Viewprojection[3][Row]}; };
	auto const w{row(3)};
	return {w + row(0), w - row(0), w + row(1), w - row(1)};
}

// a box Low..High is at least partly inside all of the Planes
bool box_seen(std::array<glm::vec4, 4> const &Planes, glm::vec3 const &Low, glm::vec3 const &High)
{
	return std::ranges::all_of(Planes, [&Low, &High](glm::vec4 const &Plane) {
		glm::vec3 const farthest{Plane.x >= 0.f ? High.x : Low.x, Plane.y >= 0.f ? High.y : Low.y, Plane.z >= 0.f ? High.z : Low.z};
		return glm::dot(glm::vec3{Plane}, farthest) + Plane.w >= 0.f;
	});
}

// the blocks of the grid of the surface (Across vertices across, a vertex a texel of Texel m, the camera in the middle, as
// snow_surface.vert lays it) the view of Viewprojection may see, as the ranges of the vertices for glMultiDrawArrays(); the
// neighbouring ones in one range
void visible_blocks(int const Across, float const Texel, glm::mat4 const &Viewprojection, std::vector<GLint> &Firsts, std::vector<GLsizei> &Counts)
{
	Firsts.clear();
	Counts.clear();
	auto const planes{side_planes(Viewprojection)};
	auto const blocks{blocks_across(Across)};
	auto const vertices{SNOW_BLOCK * SNOW_BLOCK * 6};
	auto const half{Across / 2};
	for (int by = 0; by < blocks; ++by)
	{
		for (int bx = 0; bx < blocks; ++bx)
		{
			// a vertex lies within half a texel of its place relative to the camera; a texel of margin
			glm::vec3 const low{static_cast<float>(bx * SNOW_BLOCK - half - 1) * Texel, -SNOW_HEIGHT_REACH, static_cast<float>(by * SNOW_BLOCK - half - 1) * Texel};
			glm::vec3 const high{static_cast<float>((bx + 1) * SNOW_BLOCK - half + 1) * Texel, SNOW_HEIGHT_REACH, static_cast<float>((by + 1) * SNOW_BLOCK - half + 1) * Texel};
			if (false == box_seen(planes, low, high))
				continue;
			auto const first{(by * blocks + bx) * vertices};
			if ((false == Counts.empty()) && (Firsts.back() + Counts.back() == first))
				Counts.back() += vertices;
			else
			{
				Firsts.emplace_back(first);
				Counts.emplace_back(vertices);
			}
		}
	}
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
		gl::shader const fragment("snow_surface.frag");
		m_surface = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, fragment});
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
	m_valid = false;
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

bool opengl46_snow::changed(std::uint32_t const Revision, glm::ivec2 const &Camera, snow_key const &Key) const
{
	if (Revision != m_revision)
		return true;
	auto const moved{std::max(std::abs(toroidal_step(m_camera.x, Camera.x, m_side)), std::abs(toroidal_step(m_camera.y, Camera.y, m_side)))};
	if (moved >= SNOW_MOVE_TEXELS)
		return true;
	return (std::abs(Key.depth - m_key.depth) > SNOW_DEPTH_STEP) || (std::abs(Key.drift - m_key.drift) > SNOW_DRIFT_STEP) || (glm::length(Key.wind - m_key.wind) > SNOW_WIND_STEP);
}

void opengl46_snow::work_out(opengl46_sky_occlusion const &Maps, glm::ivec2 const &Camera, int const First, int const Rows)
{
	auto const reach{Maps.map_reach()};
	m_worktimer.begin();
	// the depths worked by the wind
	m_drift->bind();
	glBindTextureUnit(0, Maps.surface_texture());
	glBindImageTexture(0, m_drifted, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RG32F);
	glProgramUniform4i(*m_drift, 0, m_side, Camera.x, Camera.y, First);
	glProgramUniform4f(*m_drift, 1, m_texel, reach * 0.6f, reach * 0.3f, 0.f);
	m_drift->dispatch(groups(m_side), groups(Rows), 1);
	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
	glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RG32F);

	// smoothed into the cells; on the edge of a strip with the drifts of the strip next to it as they were worked out last
	m_settle->bind();
	glBindTextureUnit(0, Maps.surface_texture());
	glBindTextureUnit(1, m_drifted);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, gl::SNOW_SSBO_BINDING, m_cells);
	glProgramUniform4i(*m_settle, 0, m_side, First, 0, 0);
	m_settle->dispatch(groups(m_side), groups(Rows), 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	// the units of the textures were taken behind the back of the texture cache
	opengl_texture::reset_unit_cache();
	m_worktimer.end();
}

void opengl46_snow::update(opengl46_sky_occlusion const &Maps, glm::dvec3 const &Camera)
{
	if ((m_state != 1) || (false == snowing_scenery()) || (false == Maps.active()) || (Maps.surface_texture() == 0))
	{
		// worked out all over again when the snow comes back
		m_active = false;
		m_valid = false;
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
	m_texel = Maps.map_texel();
	auto const texel{static_cast<double>(m_texel)};
	auto const size{static_cast<double>(Maps.map_size())};
	glm::ivec2 const camera{static_cast<int>(std::floor(wrap(Camera.x, size) / texel)), static_cast<int>(std::floor(wrap(Camera.z, size) / texel))};
	auto const &snow{simulation::Environment.weather().snow()};
	snow_key const key{snow.depth(), snow.drift(), snow.drift_wind()};
	auto const revision{Maps.revision()};
	// the first drifts, and the ones of a place the camera jumped to, all at once
	auto const jumped{m_valid && (std::max(std::abs(toroidal_step(m_camera.x, camera.x, side)), std::abs(toroidal_step(m_camera.y, camera.y, side))) > side / 4)};
	if ((false == m_valid) || jumped)
	{
		work_out(Maps, camera, 0, side);
		m_valid = true;
		m_strips_left = 0;
		m_revision = revision;
		m_camera = camera;
		m_key = key;
	}
	else
	{
		// a change starts the strips over, from the one next in turn
		if (changed(revision, camera, key))
		{
			m_strips_left = SNOW_STRIPS;
			m_revision = revision;
			m_camera = camera;
			m_key = key;
		}
		if (m_strips_left > 0)
		{
			auto const rows{(side + SNOW_STRIPS - 1) / SNOW_STRIPS};
			auto const first{m_strip * rows};
			work_out(Maps, camera, first, std::min(rows, side - first));
			m_strip = (m_strip + 1) % SNOW_STRIPS;
			--m_strips_left;
		}
	}
	m_active = true;
	// the grid of the surface covers the filled part of the maps
	m_across = std::min(side, static_cast<int>(2.f * Maps.map_reach() / m_texel) + 1);
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

std::size_t opengl46_snow::render(glm::mat4 const &View, glm::mat4 const &Projection, std::function<void(glm::mat4 const &)> const &Modelview)
{
	if (false == drawn())
		return 0;
	visible_blocks(m_across, m_texel, Projection * View, m_firsts, m_counts);
	if (m_firsts.empty())
		return 0;
	Modelview(View);
	m_drawtimer.begin();
	m_surface->bind();
	m_vao->bind();
	glProgramUniform4i(*m_surface, 0, m_across, 1, SNOW_BLOCK, blocks_across(m_across));
	// the culling as the pass had it is given back afterwards
	auto const culling{glIsEnabled(GL_CULL_FACE) == GL_TRUE};
	if (culling)
		glDisable(GL_CULL_FACE);
	glMultiDrawArrays(GL_TRIANGLES, m_firsts.data(), m_counts.data(), static_cast<GLsizei>(m_firsts.size()));
	if (culling)
		glEnable(GL_CULL_FACE);
	gl::vao::unbind();
	m_drawtimer.end();
	std::size_t vertices{0};
	for (auto const count : m_counts)
		vertices += static_cast<std::size_t>(count);
	return vertices / 3;
}

std::string opengl46_snow::info()
{
	auto const &snow{simulation::Environment.weather().snow()};
	auto text{std::format("snow: {:.2f} m, drifts {:.0f}%, loose {:.0f}%, wet {:.0f}%, blowing {:.0f}%, wire ice {:.1f} mm{}", snow.depth(), snow.drift() * 100.f, snow.fresh() * 100.f,
	                      snow.wetness() * 100.f, snow.blowing() * 100.f, snow.wire_ice() * 1000.f, m_active ? "" : " (no drifts)")};
	// the gpu times of the last strip worked out and of the last surface drawn
	if (auto const time = m_worktimer.result())
		text += std::format(", drifts {:.3f} ms", static_cast<double>(*time) / 1000000.0);
	if (auto const time = m_drawtimer.result())
		text += std::format(", surface {:.3f} ms", static_cast<double>(*time) / 1000000.0);
	return text;
}
