/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gl/shader.h"
#include "gl/ssbo.h"
#include "gl/readback.h"
#include "gl/query.h"
#include "gl/ubo.h"
#include "scene/scene.h"
#include "rendering/common/instancecull.h"

// culling of the model instances of a render pass in compute passes (instance_cull.comp, instance_cull_offsets.comp,
// instance_cull_commands.comp): the buffers and the passes of gfx::instance_cull, which tells what they do. with
// Global.gfx_gpucull 1 the renderer draws the result with opengl46_renderer::Render_Instances_Indirect(); the occlusion test
// takes the depth pyramid of opengl46_hiz
class opengl46_instance_cull : public gfx::instance_cull
{
  public:
	// occlusion test of a pass
	struct occlusion_setup
	{
		GLuint pyramid{0}; // texture of the depth pyramid (opengl46_hiz)
		glm::ivec2 size{0}; // of its first level
		int levels{0};
		glm::vec2 screen{0.f}; // size of the depth it was made from, pixels
		glm::mat4 projection{1.f}; // of the pass
	};

	opengl46_instance_cull() = default;

	// creates the buffers, and the programs and the record buffer, or takes the ones of Shared. throws gl::shader_exception
	void init(opengl46_instance_cull const *Shared = nullptr);
	// runs the culling passes for the buckets added. View: rotation of the view of the pass. Occlusion: the depth pyramid
	// to test the instances against, or null. Twophase: with the occlusion test, only the instances seen in the previous
	// frame pass for now, dispatch_second() takes the others
	void dispatch(glm::mat4 const &View, occlusion_setup const *Occlusion = nullptr, bool const Twophase = false);
	// the second phase, after the instances which passed dispatch() were drawn and the pyramid was made again with them:
	// the others are tested against it, then the commands of all instances and of the ones of this phase are filled in.
	// needs fill_commands() before it
	void dispatch_second(occlusion_setup const &Occlusion);
	// sends the indirect draw commands made for the entries of the pass and fills in their instances (of the instances drawn
	// first, in two phases). after this the commands are in the buffer bound by bind_draw(), the modelview matrices of the
	// instances in the output
	void fill_commands(std::vector<gl::instance_draw_command> const &Commands);
	// binds the commands to GL_DRAW_INDIRECT_BUFFER and the output in place of the instance buffer of the frame
	void bind_draw();
	std::string info_times(std::string const &Label = "instance cull");

  protected:
	std::int64_t record_buffer_size() const override;
	void resize_record_buffer(std::int64_t const Size) override;
	void send_records(std::uint32_t const First, scene::basic_cell::instance_record const *Records, std::uint32_t const Count) override;
	void push_counts() override;
	bool pop_counts(std::uint32_t *Counts) override;

  private:
	// what the culls of all passes share
	struct shared_data
	{
		std::unique_ptr<gl::program> cull_program;
		std::unique_ptr<gl::program> offsets_program;
		std::unique_ptr<gl::program> commands_program;
		std::unique_ptr<gl::ssbo> records;
	};

	// binds the buffers of the culling passes
	void bind_buffers();

	std::shared_ptr<shared_data> m_shared;
	std::unique_ptr<gl::ssbo> m_jobs;
	std::unique_ptr<gl::ssbo> m_groups;
	std::unique_ptr<gl::ssbo> m_lods;
	std::unique_ptr<gl::ssbo> m_counters;
	std::unique_ptr<gl::ssbo> m_output;
	std::unique_ptr<gl::ssbo> m_commands;
	std::unique_ptr<gl::readback> m_readback;
	gl::gpu_timer m_timer;
	gl::gpu_timer m_secondtimer;
};
