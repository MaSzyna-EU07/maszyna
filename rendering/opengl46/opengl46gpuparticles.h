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
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "gl/shader.h"
#include "gl/ssbo.h"
#include "gl/ubo.h"
#include "gl/query.h"
#include "gl/readback.h"

class particle_manager;

// particles of the gpu for the opengl 4.6 renderer: the smoke and the rain or snow.
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
// their precision far from the centre of the scenery
class opengl46_gpu_particles
{
  public:
	opengl46_gpu_particles() = default;
	~opengl46_gpu_particles();
	opengl46_gpu_particles(opengl46_gpu_particles const &) = delete;
	opengl46_gpu_particles &operator=(opengl46_gpu_particles const &) = delete;

	// what a colour pass draws with
	struct view
	{
		glm::mat4 modelview; // world axes around the camera to the view space of the pass
		glm::mat4 projection;
		glm::dvec3 camera;
		int height{0}; // of the target, pixels
		bool soft{false}; // the resolved depth of the scene drawn so far is bound to gl::SCENE_DEPTH_TEX
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
	std::string info_times();

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
	// reads back the instances counted by a pass; Drawn is left as it was when nothing came back yet
	static void collect(gl::readback &Readback, std::size_t &Drawn);
	// the planes of the view of the pass, for the culling of the compute passes
	static std::array<glm::vec4, 5> planes(view const &View);
	void draw(GLuint Output, GLuint Command, gl::program &Program);

	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	std::unique_ptr<gl::program> m_smokesimulate;
	std::unique_ptr<gl::program> m_smokeprepare;
	std::unique_ptr<gl::program> m_smokedraw;
	std::unique_ptr<gl::program> m_precipitationprepare;
	std::unique_ptr<gl::program> m_precipitationdraw;
	GLuint m_vao{0}; // the attributes of the instances, from the buffer of the draw bound to it before the draw

	// smoke
	std::vector<pool_source> m_pool;
	std::vector<std::size_t> m_requested; // the sizes the sources asked for
	std::vector<gl::particle_smoke_source> m_sourcerecords;
	std::uint32_t m_places{0};
	bool m_poolfull{false}; // some sources got no places
	std::unique_ptr<gl::ssbo> m_particles;
	std::unique_ptr<gl::ssbo> m_sources;
	std::unique_ptr<gl::ssbo> m_owners;
	std::unique_ptr<gl::ssbo> m_smokeoutput;
	std::unique_ptr<gl::ssbo> m_smokecommand;
	std::unique_ptr<gl::readback> m_smokereadback;
	std::uint64_t m_lastupdate{0};
	glm::dvec3 m_origin{0.0};
	bool m_originset{false};
	std::size_t m_smokedrawn{0};
	std::size_t m_smokealive{0}; // places of the sources which bore particles lately
	gl::gpu_timer m_simulatetimer;
	gl::gpu_timer m_smoketimer;

	// rain and snow
	std::unique_ptr<gl::ssbo> m_precipitationoutput;
	std::unique_ptr<gl::ssbo> m_precipitationcommand;
	std::unique_ptr<gl::readback> m_precipitationreadback;
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
	gl::gpu_timer m_precipitationtimer;
};
