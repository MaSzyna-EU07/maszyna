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

class cFrustum;
class TAnimModel;

// culling of the model instances of a render pass in compute passes (instance_cull.comp, instance_cull_offsets.comp),
// the gpu side of opengl46_renderer::Queue_Instances() and Render_Instanced():
// - the records of the instances (scene::basic_cell::instance_record) are kept in one buffer, each bucket of a scene cell
//   in its own place, and sent again only when the bucket made them anew
// - every bucket queued in the pass becomes a job; the buckets of one model and one set of skins make a group, whose
//   lod bands are its entries
// - the passes count the instances which pass per entry, turn the counts into places in the output and write the
//   modelview matrices of the instances there
// with Global.gfx_gpucull 1 the instances are drawn from the result: the renderer turns each lod band of a group into a
// list of indirect draw commands (one per submodel drawn in the band), whose instances a fourth pass fills in from the
// counts (instance_cull_commands.comp), and draws them with the output bound as the instance buffer
// (opengl46_renderer::Render_Instances_Indirect()). an occlusion test against a depth pyramid of what the pass drew before
// (opengl46_hiz) can leave out more instances.
// with Global.gfx_gpucull 2 the cpu still draws, and the result is only compared with its own: the counts per entry are
// read back a few frames later and set against the counts Render_Instanced() reported for the same frame
class opengl46_instance_cull
{
  public:
	// what a pass needs for the tests, the way Render_Instanced() takes it
	struct pass_setup
	{
		unsigned int frame{0};
		glm::dvec3 camera{0.0}; // camera of the pass: frustum and placement
		glm::dvec3 distancecamera{0.0}; // the distance is measured from it
		cFrustum const *frustum{nullptr}; // of the pass camera, in world coordinates
		float distancescale{1.f}; // of the squared distance
		float distanceoffset{0.f}; // added to the scaled squared distance past the range test
		float distancerange{0.f}; // squared range of the scaled distance before the offset
		float drawrange{0.f}; // squared draw range
		float zoom{1.f};
		bool drawing{false}; // the instances are drawn from the result (Global.gfx_gpucull 1), not compared
	};
	// occlusion test of a pass
	struct occlusion_setup
	{
		GLuint pyramid{0}; // texture of the depth pyramid (opengl46_hiz)
		glm::ivec2 size{0}; // of its first level
		int levels{0};
		glm::vec2 screen{0.f}; // size of the depth it was made from, pixels
		glm::mat4 projection{1.f}; // of the pass
	};
	// a group of instances (model and skins) of the pass, for the renderer to make the draw commands of
	struct draw_group
	{
		scene::basic_cell::instance_bucket_key const *key;
		TAnimModel *instance; // one of the group, prepares the model and gives the material
		std::uint32_t lodfirst; // in lod_bounds()
		std::uint32_t lodcount;
		std::uint32_t entryfirst;
		float distancemin; // squared distances (scaled as the culling does) the instances of the group can be at
		float distancemax;
	};

	opengl46_instance_cull() = default;
	opengl46_instance_cull(const opengl46_instance_cull &) = delete;
	opengl46_instance_cull &operator=(const opengl46_instance_cull &) = delete;

	// creates the programs and buffers. throws gl::shader_exception
	void init();
	// starts collecting the buckets of a pass
	void begin_pass(pass_setup const &Setup);
	bool in_pass() const
	{
		return m_inpass;
	}
	bool drawing() const
	{
		return m_inpass && m_setup.drawing;
	}
	// adds the records of a bucket (made already) to the pass, sending them to the gpu when they changed.
	// Area: bounds of the cell holding the bucket, which hold the locations of all its instances
	void add_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area);
	// runs the culling passes for the buckets added. View: rotation of the view of the pass. Occlusion: the depth pyramid
	// to test the instances against, or null
	void dispatch(glm::mat4 const &View, occlusion_setup const *Occlusion = nullptr);
	// groups of the pass and the lod bounds they refer to, valid from dispatch() to end_pass()
	std::vector<draw_group> const &draw_groups() const
	{
		return m_drawgroups;
	}
	std::vector<float> const &lod_bounds() const
	{
		return m_lodlist;
	}
	// sends the indirect draw commands made for the entries of the pass and fills in their instances. after this the
	// commands are in the buffer bound by bind_draw(), the modelview matrices of the instances in output()
	void fill_commands(std::vector<gl::instance_draw_command> const &Commands);
	// binds the commands to GL_DRAW_INDIRECT_BUFFER and the output in place of the instance buffer of the frame
	void bind_draw();
	// counts of the last result which came back from the gpu, with the draws of the pass, for the debug panel
	void draw_result(std::uint32_t const Draws, std::uint32_t const Prepassdraws, double const Cputime);
	// what draw_result() was given last
	std::uint32_t prepass_draws() const
	{
		return m_prepassdraws;
	}
	double cpu_time() const
	{
		return m_cputime;
	}
	// instances which passed in the last result which came back from the gpu, a few frames old
	std::uint32_t last_count() const
	{
		return static_cast<std::uint32_t>(m_last.gputotal);
	}
	// counts per lod band of the instances Render_Instanced() drew for a model and its skins in this pass
	void cpu_result(scene::basic_cell::instance_bucket_key const &Key, std::vector<std::uint32_t> const &Bandcounts);
	// ends the pass: its cpu counts are kept for the comparison
	void end_pass();
	// lines for the debug panel
	std::string info();
	std::string info_times();

  private:
	struct group_data
	{
		std::uint32_t index;
		std::uint32_t entryfirst;
		std::uint32_t entrycount;
		float distancemin;
		float distancemax;
	};
	// counts of the cpu of a pass, waiting for those of the gpu
	struct cpu_pass
	{
		unsigned int frame;
		std::vector<std::uint32_t> counts;
		std::uint64_t total;
	};
	struct comparison
	{
		unsigned int frame{0};
		std::uint64_t gputotal{0};
		std::uint64_t cputotal{0};
		std::uint32_t entries{0};
		std::uint32_t compared{0}; // entries compared, at most READBACK_ENTRIES
		std::uint32_t mismatched{0};
		std::uint32_t largestdifference{0};
		std::uint32_t occluded{0}; // left out by the occlusion test
	};

	// finds a place for the records of a bucket in the record buffer and sends them
	void place(scene::basic_cell::instance_bucket const &Bucket);
	// takes the newest counts of the gpu which arrived and compares them with the ones of the cpu
	void collect();

	// entries read back for the comparison; with more of them only the totals and these are compared
	static constexpr std::uint32_t READBACK_ENTRIES{16384};

	std::unique_ptr<gl::program> m_cull_program;
	std::unique_ptr<gl::program> m_offsets_program;
	std::unique_ptr<gl::program> m_commands_program;
	std::unique_ptr<gl::ssbo> m_records;
	std::unique_ptr<gl::ssbo> m_jobs;
	std::unique_ptr<gl::ssbo> m_groups;
	std::unique_ptr<gl::ssbo> m_lods;
	std::unique_ptr<gl::ssbo> m_counters;
	std::unique_ptr<gl::ssbo> m_output;
	std::unique_ptr<gl::ssbo> m_commands;
	std::unique_ptr<gl::readback> m_readback;
	gl::gpu_timer m_timer;

	// record buffer: records placed so far, the generation which tells current places from the ones of a replaced buffer
	std::uint32_t m_recordfill{0};
	unsigned int m_recordgeneration{1};

	bool m_inpass{false};
	pass_setup m_setup;
	std::vector<scene::basic_cell::instance_bucket const *> m_buckets; // of the jobs, in their order
	std::vector<gl::instance_cull_job> m_joblist;
	std::vector<gl::instance_cull_group> m_grouplist;
	std::vector<draw_group> m_drawgroups;
	std::vector<float> m_lodlist;
	std::unordered_map<scene::basic_cell::instance_bucket_key, group_data, scene::basic_cell::instance_bucket_key_hash> m_groupmap;
	std::vector<TSubModel const *> m_pending; // walk of the submodel tree
	std::uint32_t m_candidates{0};
	std::uint32_t m_entries{0};
	bool m_dispatched{false};

	std::vector<std::uint32_t> m_cpucounts; // of the current pass, by entry
	std::deque<cpu_pass> m_cpupasses;
	std::vector<std::uint32_t> m_readbackdata;
	comparison m_last;
	std::uint64_t m_comparedframes{0};
	std::uint64_t m_mismatchedframes{0};
	// draws of the last pass drawn from the gpu
	std::uint32_t m_draws{0};
	std::uint32_t m_prepassdraws{0};
	std::uint32_t m_commandcount{0};
	double m_cputime{0.0};
	bool m_occlusion{false}; // the last dispatch tested occlusion
};
