/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gl/ubo.h"
#include "scene/scene.h"

class cFrustum;
class TAnimModel;

namespace gfx
{

// the cpu side of the culling of the model instances of a render pass in compute passes, shared by the renderers (stage N6c of
// claude/vulkan14-renderer-plan.md); the backend (opengl46_instance_cull, vk14 instance_cull) keeps the buffers and runs the passes:
// - the records of the instances (scene::basic_cell::instance_record) are kept in one buffer, each bucket of a scene cell
//   in its own place, and sent again only when the bucket made them anew
// - every bucket queued in the pass becomes a job; the buckets of one model and one set of skins make a group, whose
//   lod bands are its entries
// - the passes count the instances which pass per entry, turn the counts into places in the output and write the
//   modelview matrices of the instances there
// with Global.gfx_gpucull 1 the instances are drawn from the result: the renderer turns each lod band of a group into a
// list of indirect draw commands (one per submodel drawn in the band), whose instances a fourth pass fills in from the
// counts (instance_cull_commands.comp), and draws them with the output bound as the instance buffer. an occlusion test
// against a depth pyramid of what the pass drew before can leave out more instances. in two phases the instances seen in the
// previous frame are culled and drawn first, then the pyramid is made again with them in it and the others are tested
// against it, so the instances hide one another too. the command buffer holds the commands three times over: the instances
// drawn first, all of them, and the ones drawn second, each copy command_count() commands long (draw_command_offset())
// with Global.gfx_gpucull 2 the cpu still draws, and the result is only compared with its own: the counts per entry are
// read back a few frames later and set against the counts Render_Instanced() reported for the same frame
// each render pass culled on the gpu (the main view, the cascades of the sun shadows, the faces of the environment map, the
// shadow maps of headlights and the scenery parts of those of lamps) has a culling of its own, with its own buffers, while the
// records and the programs are shared by all of them: the records of a bucket go to the gpu once and serve every pass which
// queues the bucket
// the sleepers of tracks (TTrack::m_sleeper_local_transforms) are culled the same way, each track with them a bucket of its
// own, and drawn together with the instances; their groups are kept apart from those of the model instances
// so are the model instances drawn on their own (not instanceable): each one a bucket and a group of its own (the instance in
// its key), whose submodel tree the renderer animates and walks for it alone
class instance_cull
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
	// a group of instances (model and skins) of the pass, for the renderer to make the draw commands of
	struct draw_group
	{
		scene::basic_cell::instance_bucket_key const *key;
		TAnimModel *instance; // one of the group, prepares the model and gives the material. none for sleepers
		bool sleepers; // the skins come from the key, as Render_Sleepers() sets them
		std::uint32_t lodfirst; // in lod_bounds()
		std::uint32_t lodcount;
		std::uint32_t entryfirst;
		float distancemin; // squared distances (scaled as the culling does) the instances of the group can be at
		float distancemax;
	};
	// copies of the commands in the command buffer
	enum class command_set : std::uint32_t
	{
		first = 0, // instances which passed the first dispatch: in one phase all of them
		all = 1, // after the second phase
		second = 2 // the ones which passed the second phase
	};

	instance_cull();
	instance_cull(const instance_cull &) = delete;
	instance_cull &operator=(const instance_cull &) = delete;
	virtual ~instance_cull() = default;

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
	// Area: bounds which hold the locations of all its visible instances: their own (scene_walk::Bucket_Area()) or of the cell
	// holding the bucket
	void add_bucket(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area);
	// adds the sleepers of a track, their records made already as a bucket with no instances (only the records and the place
	// in the record buffer count). Area: bounds of the locations of the sleepers. the range of the whole track is tested by
	// the caller, the culling tests each sleeper against the frustum and gives it the lod band of its plain distance from the
	// camera of the pass (Render_Sleepers())
	void add_sleepers(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area);
	// a bucket the renderer left out of the pass, as none of its instances can pass the culling (Global.gfx_gpucull_bucketrange): only counted,
	// for the panel
	void skip_bucket(scene::basic_cell::instance_bucket const &Bucket);
	// the last dispatch works in two phases
	bool two_phase() const
	{
		return m_twophase;
	}
	// groups of the pass and the lod bounds they refer to, valid from the dispatch to end_pass()
	std::vector<draw_group> const &draw_groups() const
	{
		return m_drawgroups;
	}
	std::vector<float> const &lod_bounds() const
	{
		return m_lodlist;
	}
	// offset of command Index of a copy in the command buffer
	std::intptr_t draw_command_offset(command_set const Set, std::size_t const Index) const
	{
		return static_cast<std::intptr_t>((static_cast<std::size_t>(Set) * m_commandcount + Index) * sizeof(gl::instance_draw_command));
	}
	// counts of the last result which came back from the gpu, with the draws of the pass and the calls which made them (fewer
	// with multi draws), for the debug panel
	void draw_result(std::uint32_t const Draws, std::uint32_t const Prepassdraws, std::uint32_t const Calls, std::uint32_t const Prepasscalls, double const Cputime);
	// what draw_result() was given last
	std::uint32_t prepass_draws() const
	{
		return m_prepassdraws;
	}
	std::uint32_t prepass_calls() const
	{
		return m_prepasscalls;
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
	// counts per lod band of the instances Render_Instanced() drew for a model and its skins in this pass, or of the sleepers
	// Render_Sleepers() drew. needs the dispatch before it
	void cpu_result(scene::basic_cell::instance_bucket_key const &Key, std::vector<std::uint32_t> const &Bandcounts, bool const Sleepers = false);
	// the cpu drew the instance of a group of its own (Key with the instance) at a squared distance scaled as the culling
	// scales it: counted in the lod band the distance falls in. needs the dispatch before it
	void cpu_instance(scene::basic_cell::instance_bucket_key const &Key, float const Distancesquared);
	// ends the pass: its cpu counts are kept for the comparison
	void end_pass();
	// takes the newest counts of the gpu which arrived and compares them with the ones of the cpu. done when a pass begins;
	// for the panel also by the culls of passes which aren't made every frame
	void collect();
	// a pass was culled since the start
	bool ran() const
	{
		return m_ran;
	}
	// lines for the debug panel, Label naming the pass. Occlusion: the pass may test occlusion (the main view)
	std::string info(std::string const &Label = "gpu cull", bool const Occlusion = true);

  protected:
	// what the cullings of all passes share of the record buffer: records placed so far, the generation which tells current places
	// from the ones of a replaced buffer, the frame of the last time the buffer was started over, and whether it was
	struct record_state
	{
		std::uint32_t recordfill{0};
		unsigned int recordgeneration{1};
		unsigned int restartframe{0};
		bool restarted{false};
	};
	// entries read back for the comparison; with more of them only the totals and these are compared
	static constexpr std::uint32_t READBACK_ENTRIES{16384};

	// what the backend does with the buffers
	// size of the record buffer, bytes, and a new one of the given size (the contents are dropped)
	virtual std::int64_t record_buffer_size() const = 0;
	virtual void resize_record_buffer(std::int64_t const Size) = 0;
	// sends records to their place in the record buffer
	virtual void send_records(std::uint32_t const First, scene::basic_cell::instance_record const *Records, std::uint32_t const Count) = 0;
	// the buffer is started over without growing: the records go to the places of the buffer from its start again
	virtual void restart_records() {}
	// the counters of the pass (INSTANCE_COUNTER_HEADER + READBACK_ENTRIES words) go back to the cpu, and the oldest of them which
	// arrived: false when none did
	virtual void push_counts() = 0;
	virtual bool pop_counts(std::uint32_t *Counts) = 0;

	// the start of the dispatch of the first phase: false when there's nothing to cull
	bool dispatch_begin();
	// planes of the frustum of the pass moved to its camera, in double precision: the cpu tests world coordinates
	void frustum_planes(glm::vec4 (&Planes)[6]) const;
	// the end of the dispatch of the first phase
	void dispatch_end();
	// with a culling made for another pass: shares the records with it
	void share_records(instance_cull const &Shared)
	{
		m_records = Shared.m_records;
	}

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
		std::uint32_t first{0}; // drawn first, seen in the previous frame (two phases)
	};

	using group_map = std::unordered_map<scene::basic_cell::instance_bucket_key, group_data, scene::basic_cell::instance_bucket_key_hash>;

	// add_bucket() and add_sleepers()
	void add(scene::basic_cell::instance_bucket_key const &Key, scene::basic_cell::instance_bucket const &Bucket, scene::bounding_area const &Area, bool const Sleepers);
	// finds a place for the records of a bucket in the record buffer and sends them
	void place(scene::basic_cell::instance_bucket const &Bucket);

	std::shared_ptr<record_state> m_records;

	bool m_inpass{false};
	bool m_ran{false};
	pass_setup m_setup;
	std::vector<scene::basic_cell::instance_bucket const *> m_buckets; // of the jobs, in their order
	std::vector<gl::instance_cull_job> m_joblist;
	std::vector<gl::instance_cull_group> m_grouplist;
	std::vector<draw_group> m_drawgroups;
	std::vector<float> m_lodlist;
	group_map m_groupmap;
	group_map m_sleepergroupmap; // groups of the sleepers
	std::uint32_t m_candidates{0};
	std::uint32_t m_entries{0};
	// tracks with sleepers and their sleepers queued in the pass, and in the last pass, for the panel
	std::uint32_t m_sleepertracks{0};
	std::uint32_t m_sleepers{0};
	std::uint32_t m_lastsleepertracks{0};
	std::uint32_t m_lastsleepers{0};
	// model instances drawn on their own queued in the pass, and in the last pass, for the panel
	std::uint32_t m_models{0};
	std::uint32_t m_lastmodels{0};
	// what the pass held and sent, and the same of the last pass, for the panel: jobs (buckets queued), buckets left out beyond
	// their range and their instances, records sent to the gpu, the record buffer started over
	struct pass_counts
	{
		std::uint32_t jobs{0};
		std::uint32_t candidates{0};
		std::uint32_t skippedbuckets{0};
		std::uint64_t skippedinstances{0};
		std::uint64_t sentrecords{0};
		bool restarted{false};
	};
	pass_counts m_counts;
	pass_counts m_lastcounts;
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
	std::uint32_t m_calls{0};
	std::uint32_t m_prepasscalls{0};
	std::uint32_t m_commandcount{0};
	double m_cputime{0.0};
	bool m_occlusion{false}; // the last dispatch tested occlusion
	bool m_twophase{false}; // in two phases
};

} // namespace gfx
