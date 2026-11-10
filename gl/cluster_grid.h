#pragma once

#include <array>
#include <memory>
#include <optional>

#include <glm/glm.hpp>

#include "fence.h"
#include "ssbo.h"
#include "ubo.h"
#include "shader.h"

namespace gl
{
	// GPU resources of the light cluster grid of one viewport. the grid cuts the view frustum of a pass into
	// CLUSTER_GRID_X x CLUSTER_GRID_Y screen tiles and CLUSTER_GRID_Z slices of depth, spaced exponentially
	// between znear and zfar (layout and binding points: ubo.h). depth is the clip space w of a point, which is
	// also 1.0 / gl_FragCoord.w of its fragment and, unlike the view space z, is right for tilted screens.
	// the buffers have a fixed size, so the grid survives changes of the resolution, fov and draw range: those
	// only make its parameters and boxes be recomputed.
	// bindings of the buffers are context state and the buffers are per viewport, so bind() has to be done in
	// the context and before every pass which uses the grid
	class cluster_grid
	{
	  public:
		// what the culling pass reports about a frame
		struct cull_stats
		{
			// entries of the shared index list in use
			uint32_t index_count = 0;
			// (cluster, light) pairs dropped because the cluster was full
			uint32_t overflow_count = 0;
			// highest overflow_count seen since the grid was created
			uint32_t overflow_peak = 0;
		};

		cluster_grid();
		~cluster_grid();
		cluster_grid(const cluster_grid &) = delete;
		cluster_grid &operator=(const cluster_grid &) = delete;

		void bind();

		// refreshes the parameters for a pass: its projection, depth range and the size of the render target.
		// returns true if the boxes of the clusters no longer match them and build() has to run
		bool update(const glm::mat4 &projection, float znear, float zfar, glm::vec2 viewport_size);
		// recomputes the boxes of the clusters in view space with the program made from cluster_build.comp.
		// binds the grid, and leaves the boxes ready to be read by the compute passes which follow
		void build(program &build_program);
		// assigns the lights of light_ssbo to the clusters with the program made from cluster_cull.comp: zeroes the
		// counters, fills the (offset, count) of every cluster and the index list, and leaves them ready to be
		// read by the shading and the debug view which follow. the boxes have to be current (update(), build()) and the
		// grid valid. also queues the counters for reading back, see stats()
		void cull(program &cull_program);

		// false if the last update found no grid possible for the pass (orthographic projection)
		bool valid() const
		{
			return m_params.valid != 0;
		}
		const cluster_ubs &params() const
		{
			return m_params;
		}
		// counters of the most recent culling pass whose result has reached the cpu, which takes a few frames
		// (read back without waiting for the gpu); empty until there is one
		std::optional<cull_stats> stats() const
		{
			return m_stats;
		}

	  private:
		// cluster_aabb per cluster
		ssbo m_aabbs;
		// cluster_range per cluster
		ssbo m_ranges;
		// light indices of all clusters, allocated from m_counters.index_count
		ssbo m_indices;
		// cluster_counters
		ssbo m_counters;
		ubo m_params_buffer;

		cluster_ubs m_params{};
		bool m_boxes_stale = true;

		// counters copied for reading back: a queue of buffers, each with the fence of the copy into it
		static constexpr size_t READBACK_SLOTS = 4;
		struct readback_slot
		{
			GLuint buffer = 0;
			std::unique_ptr<fence> copied;
		};
		void collect_stats();
		std::array<readback_slot, READBACK_SLOTS> m_readback;
		// oldest slot in use and the number of slots in use
		size_t m_readback_first = 0;
		size_t m_readback_used = 0;
		std::optional<cull_stats> m_stats;
	};
} // namespace gl
