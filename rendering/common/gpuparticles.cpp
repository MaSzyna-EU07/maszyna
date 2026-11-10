/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/gpuparticles.h"

#include "rendering/particles.h"
#include "simulation/simulationenvironment.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"

namespace gfx
{

namespace
{

// places of the smoke at most, all sources together (48 bytes each, as many again for the draw); the sources past it get none
std::uint32_t constexpr MAX_SMOKE_PLACES{1u << 19};
// the origin of the smoke moves by whole steps when the camera gets this far from it
double constexpr ORIGIN_STEP{256.0};
double constexpr ORIGIN_REACH{1024.0};
// how much the smoke scatters the light onwards (Henyey-Greenstein)
float constexpr SMOKE_FORWARD{0.3f};

// the boxes of the rain and snow around the camera: their sizes (metres) and the shares of the particles in each
std::array<float, 3> constexpr BOX_SIZE{10.f, 24.f, 60.f};
std::array<float, 3> constexpr BOX_SHARE{0.2f, 0.35f, 0.45f};

// what the rain and the snow are like: the speed of the fall (m/s), how long a streak lasts (s, the eye's or the camera's
// exposure), opacity, how much a drop or a flake scatters the light onwards and how bright it is against a white surface
struct precipitation_kind
{
	float fall;
	float exposure;
	float opacity;
	float forward;
	float brightness;
};
precipitation_kind constexpr RAIN{7.f, 0.025f, 0.5f, 0.6f, 0.6f};
precipitation_kind constexpr SNOW{1.1f, 0.01f, 0.85f, 0.25f, 0.9f};
// the share of the wind of the environment the drops and flakes drift with (as the old cylinder leant with it)
double constexpr WIND_SHARE{0.5};
// the camera faster than this from a frame to the next was moved, not driven or flown
double constexpr CAMERA_JUMP{120.0};

} // namespace

bool gpu_particles::init()
{
	if (m_state != 0)
		return m_state > 0;
	m_state = -1;
	if (false == gpu_init())
		return false;
	m_state = 1;
	return true;
}

bool gpu_particles::smoke_available()
{
	return init();
}

bool gpu_particles::precipitation_available()
{
	return init();
}

void gpu_particles::clear()
{
	m_pool.clear();
	m_requested.clear();
	m_poolfull = false;
	m_sourcerecords.clear();
	m_places = 0;
	m_storage = false;
	gpu_clear();
	m_precipitationcapacity = 0;
	m_originset = false;
	m_lastupdate = 0;
}

void gpu_particles::layout(particle_manager &Particles)
{
	auto const &sources{Particles.sequence()};
	// the sources already laid out which asked for the same places since
	std::size_t kept{0};
	while ((kept < m_requested.size()) && (kept < sources.size()) && (m_requested[kept] == sources[kept].max_particles()))
		++kept;
	if ((kept == m_requested.size()) && (kept == sources.size()) && m_storage)
		return;
	if (kept < m_requested.size())
	{
		// a source went away or changed: all anew
		kept = 0;
		m_pool.clear();
		m_requested.clear();
		m_places = 0;
		m_poolfull = false;
	}
	auto const oldplaces{m_places};

	auto first{m_places};
	m_pool.resize(sources.size());
	m_requested.resize(sources.size());
	for (std::size_t i = kept; i < sources.size(); ++i)
	{
		m_requested[i] = sources[i].max_particles();
		auto const size{static_cast<std::uint32_t>(std::min<std::size_t>(m_requested[i], MAX_SMOKE_PLACES))};
		m_pool[i] = pool_source{};
		if (first + size > MAX_SMOKE_PLACES)
		{
			if (false == m_poolfull)
				WriteLog("particles: the smoke of some sources left out, over " + std::to_string(MAX_SMOKE_PLACES) + " places of particles");
			m_poolfull = true;
			continue;
		}
		m_pool[i].first = first;
		m_pool[i].size = size;
		first += size;
	}
	m_places = first;

	std::vector<std::uint32_t> owners(std::max<std::uint32_t>(m_places, 1), 0);
	for (std::size_t i = 0; i < m_pool.size(); ++i)
		std::fill_n(owners.begin() + m_pool[i].first, m_pool[i].size, static_cast<std::uint32_t>(i));

	// the particles of the places kept go over to the new storage, the new places are dead
	auto const places{std::max<std::uint32_t>(m_places, 1)};
	gpu_layout(places, (m_storage && (oldplaces > 0) && (kept > 0)) ? oldplaces : 0, owners, sources.size());
	m_storage = true;
	m_sourcerecords.resize(sources.size(), gl::particle_smoke_source{});
}

void gpu_particles::simulate_smoke(particle_manager &Particles, glm::dvec3 const &Camera)
{
	if (false == init())
		return;
	layout(Particles);
	if (Particles.updates() == m_lastupdate)
		return;
	m_lastupdate = Particles.updates();
	if (m_places == 0)
		return;

	// the origin of the positions near the camera; when it moves, the particles move by as much the other way
	glm::vec3 shift{0.f};
	auto const snapped{glm::floor(Camera / ORIGIN_STEP) * ORIGIN_STEP};
	if (false == m_originset)
	{
		m_origin = snapped;
		m_originset = true;
	}
	else if (glm::length(Camera - m_origin) > ORIGIN_REACH)
	{
		shift = glm::vec3{m_origin - snapped};
		m_origin = snapped;
	}

	// the light of the smoke as the cpu works it out, for gfx.smoke.lighting off
	auto const oldlight{(glm::vec3{Global.DayLight.ambient} + 0.35f * glm::vec3{Global.DayLight.diffuse}) * simulation::Environment.light_intensity()};
	auto const &sources{Particles.sequence()};
	m_smokealive = 0;
	for (std::size_t i = 0; i < sources.size(); ++i)
	{
		auto const &state{sources[i].gpu()};
		auto &pool{m_pool[i]};
		auto &record{m_sourcerecords[i]};
		auto const spawns{(pool.size > 0) ? std::min(state.spawns, pool.size) : 0u};
		record.pool = glm::uvec4{pool.first, pool.size, pool.head, spawns};
		record.location = glm::vec4{glm::vec3{state.location - m_origin}, 0.f};
		record.previous = glm::vec4{glm::vec3{state.previous - m_origin}, 0.f};
		record.launch = glm::vec4{state.inclination[0], state.inclination[1], state.velocity[0], state.velocity[1]};
		record.initial = glm::vec4{state.size[0], state.size[1], state.opacity[0], state.opacity[1]};
		record.sizechange = glm::vec4{state.sizechange[0], state.sizechange[1], state.sizechange[2], state.spread};
		record.opacitychange = glm::vec4{state.opacitychange[0], state.opacitychange[1], state.opacitychange[2], 0.f};
		record.motion = glm::vec4{state.rise, state.damping, state.risemin, 0.f};
		record.settling = glm::vec4{state.settling, state.drag, state.settles ? 1.f : 0.f, state.fadein};
		record.color = glm::vec4{glm::clamp(state.color, glm::vec3{0.f}, glm::vec3{1.f}), 0.f};
		record.oldcolor = glm::vec4{glm::clamp(state.color * oldlight, glm::vec3{0.f}, glm::vec3{1.f}), 0.f};
		if (pool.size > 0)
		{
			pool.head = (pool.head + spawns) % pool.size;
			if (spawns > 0)
				m_smokealive += pool.size;
		}
	}

	smoke_simulation simulation;
	simulation.places = m_places;
	simulation.step = glm::vec4{static_cast<float>(Particles.timedelta()), shift.x, shift.y, shift.z};
	auto const wind{glm::vec3{simulation::Environment.wind()}};
	simulation.wind = glm::vec4{wind.x, wind.y, wind.z, 0.f};
	simulation.seed = static_cast<std::uint32_t>(m_lastupdate);
	gpu_simulate(simulation, m_sourcerecords);
}

std::array<glm::vec4, 5> gpu_particles::planes(view const &View)
{
	// the sides and the near side of the clip space, w >= 0 (the far plane is at infinity)
	auto const matrix{glm::transpose(View.projection * View.modelview)};
	std::array<glm::vec4, 5> result{matrix[3] + matrix[0], matrix[3] - matrix[0], matrix[3] + matrix[1], matrix[3] - matrix[1], matrix[3]};
	for (auto &plane : result)
	{
		auto const length{glm::length(glm::vec3{plane})};
		if (length > 0.f)
			plane /= length;
	}
	return result;
}

std::size_t gpu_particles::render_smoke(view const &View)
{
	if ((false == init()) || (m_places == 0))
		return 0;
	smoke_draw draw;
	draw.places = m_places;
	draw.origin = glm::vec3{m_origin - View.camera};
	draw.view = View.modelview;
	draw.options = glm::vec4{Global.gfx_smoke_lighting ? 1.f : 0.f, SMOKE_FORWARD, static_cast<float>(m_places), 0.f};
	draw.planes = planes(View);
	draw.soft = glm::vec4{View.soft ? Global.gfx_particles_soft : 0.f, 0.f, 0.f, 0.f};
	gpu_render_smoke(draw, m_smokedrawn);
	return m_smokedrawn;
}

std::size_t gpu_particles::render_precipitation(view const &View, bool const Snow, float const Intensity, unsigned int const Frame)
{
	if (false == init())
		return 0;
	gpu_collect_precipitation(m_precipitationdrawn);

	auto const &kind{Snow ? SNOW : RAIN};
	// once per frame: the time moves on, the particles drift with the wind and fall, the camera has its velocity
	if ((false == m_framestarted) || (Frame != m_frame))
	{
		m_framestarted = true;
		m_frame = Frame;
		auto const seconds{Timer::GetDeltaTime()};
		if (seconds > 0.0)
		{
			auto const wind{glm::dvec3{simulation::Environment.wind()} * WIND_SHARE};
			for (std::size_t box = 0; box < BOX_SIZE.size(); ++box)
			{
				auto const size{static_cast<double>(BOX_SIZE[box])};
				m_drift[box] = glm::mod(m_drift[box] + wind * seconds, size);
				m_fall[box] = std::fmod(m_fall[box] + kind.fall * seconds, 16.0 * size);
			}
			m_time = std::fmod(m_time + seconds, 3600.0);
			if (m_camerastarted)
			{
				auto const velocity{(View.camera - m_lastcamera) / seconds};
				if (glm::length(velocity) < CAMERA_JUMP)
					m_cameravelocity = glm::mix(m_cameravelocity, velocity, 1.0 - std::exp(-10.0 * seconds));
			}
		}
		m_lastcamera = View.camera;
		m_camerastarted = true;
	}

	// the particles, more of them in heavier rain or snow
	auto const most{static_cast<std::uint32_t>(std::max(Global.gfx_precipitation_particles, 0))};
	auto const count{static_cast<std::uint32_t>(most * std::clamp(0.15f + 0.85f * Intensity, 0.f, 1.f))};
	m_precipitationcount = count;
	m_precipitationsnow = Snow;
	if (count == 0)
		return 0;
	if (m_precipitationcapacity < count)
	{
		m_precipitationcapacity = std::max(count, most);
		gpu_precipitation_storage(m_precipitationcapacity);
	}

	precipitation_draw draw;
	draw.count = count;
	auto const second{static_cast<std::uint32_t>(count * BOX_SHARE[0])};
	auto const third{static_cast<std::uint32_t>(count * (BOX_SHARE[0] + BOX_SHARE[1]))};
	draw.counts = glm::uvec4{second, third, count, Snow ? 1u : 0u};
	for (std::size_t box = 0; box < BOX_SIZE.size(); ++box)
	{
		auto const size{static_cast<double>(BOX_SIZE[box])};
		draw.offsets[box] = glm::vec4{glm::vec3{glm::mod(m_drift[box] - glm::mod(View.camera, size), size)}, static_cast<float>(m_fall[box])};
	}
	draw.sizes = glm::vec4{BOX_SIZE[0], BOX_SIZE[1], BOX_SIZE[2], 0.f};
	auto const velocity{glm::vec3{glm::dvec3{simulation::Environment.wind()} * WIND_SHARE - m_cameravelocity}};
	draw.velocity = glm::vec4{velocity.x, velocity.y, velocity.z, kind.fall};
	draw.timing = glm::vec4{static_cast<float>(m_time), kind.exposure, kind.opacity, static_cast<float>(m_precipitationcapacity)};
	draw.view = View.modelview;
	draw.options = glm::vec4{1.f, kind.forward, kind.brightness, 0.f};
	draw.planes = planes(View);
	// the width of a pixel a metre in front of the camera, for the least width of a streak
	auto const pixel{(View.height > 0) && (View.projection[1][1] != 0.f) ? 2.f / (std::abs(View.projection[1][1]) * static_cast<float>(View.height)) : 0.f};
	draw.soft = glm::vec4{View.soft ? 0.25f * Global.gfx_particles_soft : 0.f, 0.f, 0.f, 0.f};
	draw.pixel = glm::vec4{pixel, 0.f, 0.f, 0.f};
	gpu_render_precipitation(draw);
	return m_precipitationdrawn;
}

std::string gpu_particles::info() const
{
	if (m_state <= 0)
		return {};
	std::string text;
	if (m_places > 0)
		text += " smoke of the gpu: " + std::to_string(m_smokedrawn) + " drawn, " + std::to_string(m_places) + " places of " + std::to_string(m_pool.size()) + " sources ("
		        + std::to_string(m_smokealive) + " of the ones which bore lately)" + (m_poolfull ? ", some sources left out" : "") + "\n";
	if (m_precipitationcount > 0)
		text += std::string(" ") + (m_precipitationsnow ? "snow" : "rain") + " of the gpu: " + std::to_string(m_precipitationdrawn) + " of " + std::to_string(m_precipitationcount)
		        + " drawn\n";
	return text;
}

} // namespace gfx
