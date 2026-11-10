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

#include "rendering/common/gpuparticles.h"
#include "gl/shader.h"
#include "gl/ssbo.h"
#include "gl/query.h"
#include "gl/readback.h"

// particles of the gpu for the opengl 4.6 renderer (gfx::gpu_particles): the storage buffers, the compute programs and the
// indirect draws
class opengl46_gpu_particles : public gfx::gpu_particles
{
  public:
	using view = gfx::gpu_particles::view;

	opengl46_gpu_particles() = default;
	~opengl46_gpu_particles() override;

	std::string info_times();

  protected:
	bool gpu_init() override;
	void gpu_layout(std::uint32_t const Places, std::uint32_t const Kept, std::vector<std::uint32_t> const &Owners, std::size_t const Sources) override;
	void gpu_simulate(smoke_simulation const &Simulation, std::vector<gl::particle_smoke_source> const &Records) override;
	void gpu_render_smoke(smoke_draw const &Draw, std::size_t &Drawn) override;
	void gpu_collect_precipitation(std::size_t &Drawn) override;
	void gpu_precipitation_storage(std::uint32_t const Capacity) override;
	void gpu_render_precipitation(precipitation_draw const &Draw) override;
	void gpu_clear() override;

  private:
	// reads back the instances counted by a pass; Drawn is left as it was when nothing came back yet
	static void collect(gl::readback &Readback, std::size_t &Drawn);
	void draw(GLuint Output, GLuint Command, gl::program &Program);

	std::unique_ptr<gl::program> m_smokesimulate;
	std::unique_ptr<gl::program> m_smokeprepare;
	std::unique_ptr<gl::program> m_smokedraw;
	std::unique_ptr<gl::program> m_precipitationprepare;
	std::unique_ptr<gl::program> m_precipitationdraw;
	GLuint m_vao{0}; // the attributes of the instances, from the buffer of the draw bound to it before the draw

	// smoke
	std::unique_ptr<gl::ssbo> m_particles;
	std::unique_ptr<gl::ssbo> m_sources;
	std::unique_ptr<gl::ssbo> m_owners;
	std::unique_ptr<gl::ssbo> m_smokeoutput;
	std::unique_ptr<gl::ssbo> m_smokecommand;
	std::unique_ptr<gl::readback> m_smokereadback;
	gl::gpu_timer m_simulatetimer;
	gl::gpu_timer m_smoketimer;

	// rain and snow
	std::unique_ptr<gl::ssbo> m_precipitationoutput;
	std::unique_ptr<gl::ssbo> m_precipitationcommand;
	std::unique_ptr<gl::readback> m_precipitationreadback;
	gl::gpu_timer m_precipitationtimer;
};
