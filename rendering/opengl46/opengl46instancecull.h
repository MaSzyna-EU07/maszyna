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

// culling of the model instances of a render pass in compute passes (instance_cull.comp, instance_cull_offsets.comp),
// the gpu side of opengl46_renderer::Queue_Instances() and Render_Instanced():
// - the records of the instances (scene::basic_cell::instance_record) are kept in one buffer, each bucket of a scene cell
//   in its own place, and sent again only when the bucket made them anew
// - every bucket queued in the pass becomes a job; the buckets of one model and one set of skins make a group, whose
//   lod bands are its entries
// - the passes count the instances which pass per entry, turn the counts into places in the output and write the
//   modelview matrices of the instances there
// so far the result is only compared with the one of the cpu, which still draws: the counts per entry are read back a few
// frames later and set against the counts Render_Instanced() reported for the same frame (Global.gfx_gpucull)
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
	// adds the records of a bucket (made already) to the pass, sending them to the gpu when they changed
	void add_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket);
	// runs the culling passes for the buckets added. View: rotation of the view of the pass
	void dispatch(glm::mat4 const &View);
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
	};

	// finds a place for the records of a bucket in the record buffer and sends them
	void place(scene::basic_cell::instance_bucket const &Bucket);
	// takes the newest counts of the gpu which arrived and compares them with the ones of the cpu
	void collect();

	// entries read back for the comparison; with more of them only the totals and these are compared
	static constexpr std::uint32_t READBACK_ENTRIES{16384};

	std::unique_ptr<gl::program> m_cull_program;
	std::unique_ptr<gl::program> m_offsets_program;
	std::unique_ptr<gl::ssbo> m_records;
	std::unique_ptr<gl::ssbo> m_jobs;
	std::unique_ptr<gl::ssbo> m_groups;
	std::unique_ptr<gl::ssbo> m_lods;
	std::unique_ptr<gl::ssbo> m_counters;
	std::unique_ptr<gl::ssbo> m_output;
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
};
