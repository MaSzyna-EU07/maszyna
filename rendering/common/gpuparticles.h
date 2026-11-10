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
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "gl/ubo.h"

class particle_manager;

namespace gfx
{

// particles of the gpu, whichever api draws them: the smoke and the rain or snow.
//
// smoke: the sources (particle_manager in its gpu mode) work out on the cpu only how many particles they bear in an update of
// the simulation and what these start from; the particles are kept in a storage buffer, a ring of places for each source, and
// simulated by a compute pass once per update (particles_smoke_simulate.comp). in every colour pass which draws them another
// compute pass (particles_smoke_prepare.comp) leaves out the dead ones and the ones out of the view, lights the rest with the
// lights of the pass and adds them to an indirect draw of billboards (particles_smoke.vert/.frag).
//
// rain and snow: particles in three boxes around the camera, each repeated along the world axes, which keep no state: their
// place follows from their number, how far the wind took them and how far they fell (particles_precipitation.comp), drawn as
// streaks of their motion against the camera (particles_precipitation.vert/.frag).
//
// the positions of the smoke are kept from an origin near the camera, moved by whole steps as the camera goes, so they keep
// their precision far from the centre of the scenery. what the passes are given is worked out here; the buffers, the compute
// passes and the draws are the renderer's (the gpu_ functions)
class gpu_particles
{
  public:
	gpu_particles() = default;
	virtual ~gpu_particles() = default;
	gpu_particles(gpu_particles const &) = delete;
	gpu_particles &operator=(gpu_particles const &) = delete;

	// what a colour pass draws with
	struct view
	{
		glm::mat4 modelview; // world axes around the camera to the view space of the pass
		glm::mat4 projection;
		glm::dvec3 camera;
		int height{0}; // of the target, pixels
		bool soft{false}; // the resolved depth of the scene drawn so far is bound to the depth of the scene of the pass
	};

	// whether the programs could be made; without them the smoke stays on the cpu and the rain is the old cylinder
	bool smoke_available();
	bool precipitation_available();
	// once per update of the particle manager (in its gpu mode): the births and the change of the smoke. Camera: of the view
	void simulate_smoke(particle_manager &Particles, glm::dvec3 const &Camera);
	// draws the smoke the view sees, with the texture bound to unit 0. returns: particles drawn, as read back a few frames later
	std::size_t render_smoke(view const &View);
	// draws the rain (Snow false) or the snow of the intensity 0-1. Frame: number of the frame, the time moves on once per frame.
	// returns: particles drawn, read back
	std::size_t render_precipitation(view const &View, bool Snow, float Intensity, unsigned int Frame);
	// drops what is on the gpu, e.g. when the context goes
	void clear();

	std::string info() const;

  protected:
	// the uniforms of particles_smoke_simulate.comp
	struct smoke_simulation
	{
		std::uint32_t places{0};
		glm::vec4 step{0.f}; // x: seconds of the update, yzw: added to the positions, as the origin moved
		glm::vec4 wind{0.f};
		std::uint32_t seed{0};
	};
	// the uniforms of particles_smoke_prepare.comp and of the draw
	struct smoke_draw
	{
		std::uint32_t places{0};
		glm::vec3 origin{0.f}; // of the particles less the camera
		glm::mat4 view{1.f};
		glm::vec4 options{0.f}; // x: lit by the lights of the pass, y: scattering onwards, z: places for the draws
		std::array<glm::vec4, 5> planes{};
		glm::vec4 soft{0.f}; // of particles_smoke.vert
	};
	// the uniforms of particles_precipitation.comp and of the draw
	struct precipitation_draw
	{
		glm::uvec4 counts{0u};
		std::array<glm::vec4, 3> offsets{};
		glm::vec4 sizes{0.f};
		glm::vec4 velocity{0.f};
		glm::vec4 timing{0.f};
		glm::mat4 view{1.f};
		glm::vec4 options{0.f};
		std::array<glm::vec4, 5> planes{};
		glm::vec4 soft{0.f}; // of particles_precipitation.vert
		glm::vec4 pixel{0.f};
		std::uint32_t count{0}; // particles, the threads of the compute pass
	};

	// the programs; false: the particles of the gpu can't be used
	virtual bool gpu_init() = 0;
	// new storage of the smoke: Places places (at least 1) of particles, all dead but the first Kept of the storage before, which
	// keep what they had; the storage of the draws for as many particles, the source of every place (Owners) and room for the
	// records of Sources sources
	virtual void gpu_layout(std::uint32_t const Places, std::uint32_t const Kept, std::vector<std::uint32_t> const &Owners, std::size_t const Sources) = 0;
	// the records of the sources (when there are any) and the compute pass of the simulation
	virtual void gpu_simulate(smoke_simulation const &Simulation, std::vector<gl::particle_smoke_source> const &Records) = 0;
	// the compute pass and the draw of the smoke the view sees; Drawn: the particles drawn, as the last read back gives them (left
	// as it is when nothing came back yet)
	virtual void gpu_render_smoke(smoke_draw const &Draw, std::size_t &Drawn) = 0;
	// the particles of the rain or snow drawn, as the last read back gives them
	virtual void gpu_collect_precipitation(std::size_t &Drawn) = 0;
	// room for the draws of Capacity particles of the rain or snow
	virtual void gpu_precipitation_storage(std::uint32_t const Capacity) = 0;
	// the compute pass and the draw of the rain or snow
	virtual void gpu_render_precipitation(precipitation_draw const &Draw) = 0;
	// the storage goes (the draw commands and the read backs stay)
	virtual void gpu_clear() = 0;

	std::uint32_t places() const
	{
		return m_places;
	}
	std::uint32_t precipitation_count() const
	{
		return m_precipitationcount;
	}
	bool precipitation_snow() const
	{
		return m_precipitationsnow;
	}

  private:
	struct pool_source
	{
		std::uint32_t first{0};
		std::uint32_t size{0};
		std::uint32_t head{0};
	};

	bool init();
	// the places of the sources in the storage: sources added after the others get theirs after the places already given, and
	// the particles stay; otherwise all is laid out anew
	void layout(particle_manager &Particles);
	// the planes of the view of the pass, for the culling of the compute passes
	static std::array<glm::vec4, 5> planes(view const &View);

	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used

	// smoke
	std::vector<pool_source> m_pool;
	std::vector<std::size_t> m_requested; // the sizes the sources asked for
	std::vector<gl::particle_smoke_source> m_sourcerecords;
	std::uint32_t m_places{0};
	bool m_poolfull{false}; // some sources got no places
	bool m_storage{false}; // the storage of the smoke was made
	std::uint64_t m_lastupdate{0};
	glm::dvec3 m_origin{0.0};
	bool m_originset{false};
	std::size_t m_smokedrawn{0};
	std::size_t m_smokealive{0}; // places of the sources which bore particles lately

	// rain and snow
	std::uint32_t m_precipitationcapacity{0};
	unsigned int m_frame{0};
	bool m_framestarted{false};
	std::array<glm::dvec3, 3> m_drift{}; // how far the wind took the particles of each box, within the box
	std::array<double, 3> m_fall{}; // how far they fell, within 16 sizes of the box
	double m_time{0.0}; // seconds, for the flutter of the flakes
	glm::dvec3 m_lastcamera{0.0};
	glm::dvec3 m_cameravelocity{0.0};
	bool m_camerastarted{false};
	std::size_t m_precipitationdrawn{0};
	std::uint32_t m_precipitationcount{0};
	bool m_precipitationsnow{false};
};

} // namespace gfx
