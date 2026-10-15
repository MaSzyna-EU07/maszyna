#include "stdafx.h"
#include "cluster_grid.h"

#include <cstring>

gl::cluster_grid::cluster_grid()
    : m_aabbs(sizeof(cluster_aabb) * CLUSTER_COUNT, CLUSTER_AABB_SSBO_BINDING)
    , m_ranges(sizeof(cluster_range) * CLUSTER_COUNT, CLUSTER_RANGE_SSBO_BINDING)
    , m_indices(sizeof(uint32_t) * CLUSTER_COUNT * MAX_LIGHTS_PER_CLUSTER, CLUSTER_INDEX_SSBO_BINDING)
    , m_counters(sizeof(cluster_counters), CLUSTER_COUNTER_SSBO_BINDING)
    , m_params_buffer(sizeof(cluster_ubs), CLUSTER_UBO_BINDING)
{
	// no light is assigned to any cluster until the culling pass runs
	std::vector<uint8_t> const empty(sizeof(cluster_range) * CLUSTER_COUNT, 0);
	m_ranges.update(empty.data(), 0, static_cast<GLsizeiptr>(empty.size()));
	m_counters.update(cluster_counters{});
	m_params_buffer.update(m_params);
}

void gl::cluster_grid::bind()
{
	m_aabbs.bind_base();
	m_ranges.bind_base();
	m_indices.bind_base();
	m_counters.bind_base();
	m_params_buffer.bind_uniform();
}

bool gl::cluster_grid::update(const glm::mat4 &projection, float znear, float zfar, glm::vec2 viewport_size)
{
	cluster_ubs params = m_params;
	params.viewport_width = viewport_size.x;
	params.viewport_height = viewport_size.y;

	// clip space w is a function of the position in a perspective projection, and a constant in an orthographic one
	bool const perspective{glm::length(glm::vec3{projection[0][3], projection[1][3], projection[2][3]}) > 1e-4f};
	bool const usable{perspective && znear > 0.f && zfar > znear};
	params.valid = usable ? 1 : 0;

	if (usable && (projection != m_params.projection || znear != m_params.znear || zfar != m_params.zfar))
	{
		params.projection = projection;
		params.znear = znear;
		params.zfar = zfar;
		params.slice_scale = static_cast<float>(CLUSTER_GRID_Z) / std::log(zfar / znear);
		params.slice_bias = -std::log(znear) * params.slice_scale;
		m_boxes_stale = true;
	}

	if (std::memcmp(&params, &m_params, sizeof(params)) != 0)
	{
		m_params = params;
		m_params_buffer.update(m_params);
	}

	return usable && m_boxes_stale;
}

void gl::cluster_grid::build(program &build_program)
{
	bind();
	build_program.dispatch((CLUSTER_COUNT + CLUSTER_BUILD_GROUP_SIZE - 1) / CLUSTER_BUILD_GROUP_SIZE);
	// the boxes are read by compute passes which follow, from the same shader storage buffer
	memory_barrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_boxes_stale = false;
}
