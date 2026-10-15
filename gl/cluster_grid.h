#pragma once

#include <glm/glm.hpp>

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
		cluster_grid();

		void bind();

		// refreshes the parameters for a pass: its projection, depth range and the size of the render target.
		// returns true if the boxes of the clusters no longer match them and build() has to run
		bool update(const glm::mat4 &projection, float znear, float zfar, glm::vec2 viewport_size);
		// recomputes the boxes of the clusters in view space with the program made from cluster_build.comp.
		// binds the grid, and leaves the boxes ready to be read by the compute passes which follow
		void build(program &build_program);

		// false if the last update found no grid possible for the pass (orthographic projection)
		bool valid() const
		{
			return m_params.valid != 0;
		}
		const cluster_ubs &params() const
		{
			return m_params;
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
	};
} // namespace gl
