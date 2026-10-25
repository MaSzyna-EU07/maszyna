/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14cull.h"
#include "rendering/vulkan14/vulkan14commands.h"

#include "rendering/frustum.h"
#include "utilities/Globals.h"

namespace vk14
{

namespace
{

constexpr VkDeviceSize RECORD_SIZE{sizeof(scene::basic_cell::instance_record)};
// the counters read back: the header and the entries compared
constexpr VkDeviceSize READBACK_SIZE{(gl::INSTANCE_COUNTER_HEADER + 16384) * sizeof(std::uint32_t)};

// the parameters of instance_cull.comp, its kernel_block (std140)
struct cull_block
{
	glm::mat4 view;
	glm::vec4 planes[6];
	glm::vec4 distance_params;
	glm::vec4 size_params;
	glm::mat4 hiz_projection;
	glm::ivec4 hiz_params;
	glm::vec4 hiz_screen;
	glm::uvec4 phases;
};
static_assert(sizeof(cull_block) == 304, "the block of the culling doesn't match the shader");

// the parameters of hiz_build.comp
struct hiz_constants
{
	glm::ivec4 sizes;
	glm::ivec4 params;
};

// what read and wrote the buffers of the culling before (the draws of the passes before, the copies for the cpu, the kernels) is
// done before they're written again
void before_writes(command_stream *Commands)
{
	memory_barrier(Commands,
	               VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT,
	               VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT | VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT,
	               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT,
	               VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
}

} // namespace

instance_cull::~instance_cull() = default;

bool instance_cull::init(cull_environment const &Environment, VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout, instance_cull const *Shared)
{
	if (Shared != nullptr)
	{
		m_shared = Shared->m_shared;
		share_records(*Shared);
	}
	else
	{
		auto shared{std::make_shared<shared_data>()};
		shared->environment = Environment;
		try
		{
			if ((false == shared->cull.create(Device, Pipelines, Shaders.get("instance_cull.comp"), Passlayout, {}, "instance cull")) ||
			    (false == shared->offsets.create(Device, Pipelines, Shaders.get("instance_cull_offsets.comp"), Passlayout, {}, "instance cull offsets")) ||
			    (false == shared->commands.create(Device, Pipelines, Shaders.get("instance_cull_commands.comp"), Passlayout, {}, "instance cull commands")))
			{
				return false;
			}
		}
		catch (gfx::shader_exception const &Exception)
		{
			log_error(Exception.what());
			return false;
		}
		shared->records = std::make_shared<buffer>();
		if (false == shared->records->create(*Environment.memory, 65536 * RECORD_SIZE, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, memory_kind::device, "instance records"))
		{
			return false;
		}
		m_shared = std::move(shared);
	}
	auto const &environment{m_shared->environment};
	return reserve(m_counters, READBACK_SIZE, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, "instance counters") &&
	       reserve(m_output, 65536 * sizeof(glm::mat4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, "instance output") &&
	       reserve(m_commands, 3 * 4096 * sizeof(gl::instance_draw_command), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, "instance commands") &&
	       m_readback.create(*environment.memory, READBACK_SIZE, environment.frames, "instance counters readback");
}

void instance_cull::destroy(VkDevice Device)
{
	if (m_shared && (m_shared.use_count() == 1))
	{
		m_shared->cull.destroy(Device);
		m_shared->offsets.destroy(Device);
		m_shared->commands.destroy(Device);
	}
	m_shared.reset();
	m_counters.reset();
	m_output.reset();
	m_commands.reset();
	m_readback.destroy();
	if (set_pool != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorPool(Device, set_pool, nullptr);
		set_pool = VK_NULL_HANDLE;
	}
	sets = {};
}

void instance_cull::begin_frame(std::uint32_t const Slot, std::uint64_t const Value, std::uint64_t const Completed)
{
	if (!m_shared)
	{
		return;
	}
	auto &environment{m_shared->environment};
	environment.slot = Slot;
	environment.value = Value;
	environment.completed = Completed;
}

bool instance_cull::reserve(std::shared_ptr<buffer> &Buffer, VkDeviceSize const Size, VkBufferUsageFlags const Usage, char const *Name)
{
	if (Buffer && (Size <= Buffer->size()))
	{
		return true;
	}
	auto const &environment{m_shared->environment};
	auto replacement{std::make_shared<buffer>()};
	if (false == replacement->create(*environment.memory, std::max<VkDeviceSize>(Size, Buffer ? 2 * Buffer->size() : 0), Usage, memory_kind::device, Name))
	{
		log_error(std::string("no memory for the buffer of the culling of the instances: ") + Name);
		return false;
	}
	if (Buffer)
	{
		// the frames in flight may still read it
		environment.deletions->push(environment.uploads->current_value(), [old = std::move(Buffer)]() {});
	}
	Buffer = std::move(replacement);
	return true;
}

std::int64_t instance_cull::record_buffer_size() const
{
	return (m_shared && m_shared->records) ? static_cast<std::int64_t>(m_shared->records->size()) : 0;
}

void instance_cull::resize_record_buffer(std::int64_t const Size)
{
	auto &shared{*m_shared};
	auto const &environment{shared.environment};
	auto replacement{std::make_shared<buffer>()};
	if (false == replacement->create(*environment.memory, static_cast<VkDeviceSize>(Size), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, memory_kind::device, "instance records"))
	{
		log_error("no memory for the records of the instances");
		return;
	}
	// the passes culled before in the frame keep the old buffer, with its records
	environment.deletions->push(environment.uploads->current_value(), [old = std::move(shared.records)]() {});
	shared.records = std::move(replacement);
	shared.pending.clear();
	shared.parts.clear();
}

void instance_cull::restart_records()
{
	// the records of the passes culled before in the frame go to the gpu before the frame (the uploads), where the records sent from
	// the start of the buffer again would take their places
	resize_record_buffer(record_buffer_size());
}

void instance_cull::send_records(std::uint32_t const First, scene::basic_cell::instance_record const *Records, std::uint32_t const Count)
{
	if ((Count == 0) || !m_shared)
	{
		return;
	}
	auto &shared{*m_shared};
	auto const *data{reinterpret_cast<std::uint8_t const *>(Records)};
	shared.pending.insert(shared.pending.end(), data, data + Count * RECORD_SIZE);
	shared.parts.emplace_back(First * RECORD_SIZE, Count * RECORD_SIZE);
}

void instance_cull::flush_records()
{
	auto &shared{*m_shared};
	if (shared.pending.empty())
	{
		return;
	}
	if (false == shared.environment.uploads->upload_buffer(shared.records, shared.pending.data(), shared.pending.size(), shared.parts))
	{
		log_error("no staging memory for the records of the instances");
	}
	shared.pending.clear();
	shared.parts.clear();
}

void instance_cull::push_counts()
{
	// copied with the last stage of the culling (read_counters())
}

bool instance_cull::pop_counts(std::uint32_t *Counts)
{
	if (!m_shared)
	{
		return false;
	}
	return m_readback.pop(m_shared->environment.completed, Counts, READBACK_SIZE);
}

void instance_cull::read_counters(command_stream *Commands)
{
	auto const &environment{m_shared->environment};
	m_readback.push(Commands, environment.slot, environment.value, m_counters->handle(), 0, READBACK_SIZE);
}

std::vector<descriptor> instance_cull::cull_descriptors(cull_occlusion const *Occlusion) const
{
	auto const &environment{m_shared->environment};
	auto const slice = [](ring_slice const &Slice) { return buffer_descriptor(Slice.buffer, Slice.offset, Slice.size); };
	return {buffer_descriptor(m_shared->records->handle(), 0, VK_WHOLE_SIZE),
	        slice(m_jobs),
	        slice(m_groups),
	        slice(m_lods),
	        buffer_descriptor(m_counters->handle(), 0, VK_WHOLE_SIZE),
	        buffer_descriptor(m_output->handle(), 0, VK_WHOLE_SIZE),
	        ((Occlusion != nullptr) && (Occlusion->pyramid != VK_NULL_HANDLE)) ? image_descriptor(Occlusion->sampler, Occlusion->pyramid, VK_IMAGE_LAYOUT_GENERAL) : environment.no_pyramid,
	        slice(m_block)};
}

void instance_cull::dispatch(command_stream *Commands, glm::mat4 const &View, cull_occlusion const *Occlusion, bool const Twophase)
{
	if (!m_shared || (false == dispatch_begin()))
	{
		return;
	}
	auto &shared{*m_shared};
	auto const &environment{shared.environment};
	flush_records();

	m_countersize = (gl::INSTANCE_COUNTER_HEADER + 3 * m_entries) * sizeof(std::uint32_t);
	if ((false == reserve(m_counters, std::max(m_countersize, READBACK_SIZE), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, "instance counters")) ||
	    (false == reserve(m_output, static_cast<VkDeviceSize>(std::max<std::uint32_t>(m_candidates, 1)) * sizeof(glm::mat4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, "instance output")))
	{
		return;
	}
	// the data of the pass in the ring of the frame
	auto const place = [&](ring_slice &Slice, void const *Data, std::size_t const Size, VkDeviceSize const Alignment) {
		Slice = environment.ring->allocate(std::max<VkDeviceSize>(Size, 16), Alignment);
		if (Slice && (Size > 0))
		{
			std::memcpy(Slice.data, Data, Size);
		}
		return bool(Slice);
	};
	cull_block block{};
	block.view = View;
	frustum_planes(block.planes);
	block.distance_params = glm::vec4{m_setup.distancescale, m_setup.distanceoffset, m_setup.distancerange, m_setup.drawrange};
	block.size_params = glm::vec4{m_setup.zoom, 0.f, 0.f, 0.f};
	// the occlusion test only when the instances are drawn from the result, the cpu doesn't make it
	m_occlusion = (m_setup.drawing && (Occlusion != nullptr) && (Occlusion->pyramid != VK_NULL_HANDLE));
	if (m_occlusion)
	{
		block.hiz_projection = Occlusion->projection;
		block.hiz_params = glm::ivec4{1, Occlusion->levels, Occlusion->size.x, Occlusion->size.y};
		block.hiz_screen = glm::vec4{Occlusion->screen, 0.f, 0.f};
	}
	m_twophase = (m_occlusion && Twophase);
	block.phases = glm::uvec4{m_twophase ? 1u : 0u, 0u, 0u, 0u};
	if ((false == place(m_jobs, m_joblist.data(), m_joblist.size() * sizeof(gl::instance_cull_job), environment.storage_alignment)) ||
	    (false == place(m_groups, m_grouplist.data(), m_grouplist.size() * sizeof(gl::instance_cull_group), environment.storage_alignment)) ||
	    (false == place(m_lods, m_lodlist.data(), m_lodlist.size() * sizeof(float), environment.storage_alignment)) || (false == place(m_block, &block, sizeof(block), environment.uniform_alignment)))
	{
		log_error("the ring of the frame is full, the instances aren't culled");
		m_occlusion = false;
		m_twophase = false;
		return;
	}

	// counts and cursors start at zero
	before_writes(Commands);
	std::uint32_t const header[gl::INSTANCE_COUNTER_HEADER]{m_setup.frame, 0, m_entries, 0, 0, 0, 0, 0};
	Commands->update_buffer(m_counters->handle(), 0, sizeof(header), header);
	Commands->fill_buffer(m_counters->handle(), sizeof(header), m_countersize - sizeof(header), 0);
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_CLEAR_BIT | VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
	               VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

	// one invocation per record; past the count limit of one dimension the groups go into rows
	auto const groups{(m_candidates + gl::INSTANCE_CULL_GROUP_SIZE - 1) / gl::INSTANCE_CULL_GROUP_SIZE};
	auto const groups_x{std::min<std::uint32_t>(groups, 65535)};
	auto const groups_y{(groups + groups_x - 1) / groups_x};
	auto const descriptors{cull_descriptors(Occlusion)};
	auto const jobs{static_cast<std::uint32_t>(m_joblist.size())};
	shared.cull.dispatch(Commands, descriptors, glm::uvec4{jobs, m_candidates, m_entries, 0u}, groups_x, groups_y);
	compute_barrier(Commands);
	shared.offsets.dispatch(Commands, {buffer_descriptor(m_counters->handle(), 0, VK_WHOLE_SIZE)}, glm::uvec4{m_entries, 0u, 0u, 0u}, 1);
	compute_barrier(Commands);
	shared.cull.dispatch(Commands, descriptors, glm::uvec4{jobs, m_candidates, m_entries, 1u}, groups_x, groups_y);
	// the output and the counts go to draws, indirect draws and the copy for the cpu
	compute_barrier(Commands);
	if (false == m_twophase)
	{
		read_counters(Commands);
	}
	dispatch_end();
}

void instance_cull::dispatch_second(command_stream *Commands, cull_occlusion const &Occlusion)
{
	if (!m_dispatched || !m_twophase)
	{
		return;
	}
	auto &shared{*m_shared};
	// the draws of the first phase read the output and the commands the second phase writes (other slots of them)
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT,
	               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
	auto const groups{(m_candidates + gl::INSTANCE_CULL_GROUP_SIZE - 1) / gl::INSTANCE_CULL_GROUP_SIZE};
	auto const groups_x{std::min<std::uint32_t>(groups, 65535)};
	auto const groups_y{(groups + groups_x - 1) / groups_x};
	shared.cull.dispatch(Commands, cull_descriptors(&Occlusion), glm::uvec4{static_cast<std::uint32_t>(m_joblist.size()), m_candidates, m_entries, 2u}, groups_x, groups_y);
	compute_barrier(Commands);
	if (m_commandcount > 0)
	{
		shared.commands.dispatch(Commands, {buffer_descriptor(m_counters->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_commands->handle(), 0, VK_WHOLE_SIZE)}, glm::uvec4{m_commandcount, m_entries, 1u, 0u},
		                         (m_commandcount + gl::INSTANCE_COMMAND_GROUP_SIZE - 1) / gl::INSTANCE_COMMAND_GROUP_SIZE);
	}
	// the commands go to indirect draws, the matrices to the vertex shaders, the counts to the copy for the cpu
	compute_barrier(Commands);
	read_counters(Commands);
}

void instance_cull::fill_commands(command_stream *Commands, std::vector<gl::instance_draw_command> const &Commands_)
{
	m_commandcount = static_cast<std::uint32_t>(Commands_.size());
	if (!m_dispatched || Commands_.empty())
	{
		return;
	}
	auto &shared{*m_shared};
	auto const &environment{shared.environment};
	// the first copy is sent, the others are made from it on the gpu
	auto const size{static_cast<VkDeviceSize>(Commands_.size() * sizeof(gl::instance_draw_command))};
	if (false == reserve(m_commands, 3 * size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, "instance commands"))
	{
		m_commandcount = 0;
		return;
	}
	before_writes(Commands);
	auto const slice{environment.ring->allocate(size, 16)};
	if (slice)
	{
		std::memcpy(slice.data, Commands_.data(), static_cast<std::size_t>(size));
		VkBufferCopy2 region{};
		region.sType = VK_STRUCTURE_TYPE_BUFFER_COPY_2;
		region.srcOffset = slice.offset;
		region.dstOffset = 0;
		region.size = size;
		VkCopyBufferInfo2 copy{};
		copy.sType = VK_STRUCTURE_TYPE_COPY_BUFFER_INFO_2;
		copy.srcBuffer = slice.buffer;
		copy.dstBuffer = m_commands->handle();
		copy.regionCount = 1;
		copy.pRegions = &region;
		Commands->copy_buffer2(&copy);
	}
	else
	{
		// the ring of the frame is full: the commands written in pieces with the commands of the frame
		auto const *data{reinterpret_cast<std::uint8_t const *>(Commands_.data())};
		for (VkDeviceSize offset = 0; offset < size; offset += 65536)
		{
			Commands->update_buffer(m_commands->handle(), offset, std::min<VkDeviceSize>(65536, size - offset), data + offset);
		}
	}
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
	shared.commands.dispatch(Commands, {buffer_descriptor(m_counters->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_commands->handle(), 0, VK_WHOLE_SIZE)}, glm::uvec4{m_commandcount, m_entries, 0u, 0u},
	                         (m_commandcount + gl::INSTANCE_COMMAND_GROUP_SIZE - 1) / gl::INSTANCE_COMMAND_GROUP_SIZE);
	// the commands go to indirect draws, the matrices to the vertex shaders
	compute_barrier(Commands);
}

// the depth pyramid

bool hiz_pyramid::init(VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout)
{
	try
	{
		return m_kernel.create(Device, Pipelines, Shaders.get("hiz_build.comp"), Passlayout, {}, "hi-z");
	}
	catch (gfx::shader_exception const &Exception)
	{
		log_error(Exception.what());
		return false;
	}
}

void hiz_pyramid::destroy(VkDevice Device, deletion_queue *Deletions, std::uint64_t const Value)
{
	if (Deletions != nullptr)
	{
		// the frames in flight may still read them
		Deletions->push(Value, [Device, levels = std::move(m_levels), pyramid = std::move(m_pyramid)]() {
			for (auto *view : levels)
			{
				vkDestroyImageView(Device, view, nullptr);
			}
		});
	}
	else
	{
		for (auto *view : m_levels)
		{
			vkDestroyImageView(Device, view, nullptr);
		}
		m_kernel.destroy(Device);
		m_pyramid.reset();
	}
	m_levels.clear();
	m_pyramid.reset();
	m_sourcesize = glm::ivec2{0};
	m_size = glm::ivec2{0};
}

bool hiz_pyramid::build(command_stream *Commands, VkDevice Device, allocator &Memory, deletion_queue &Deletions, std::uint64_t const Value, VkImageView const Depth, VkSampler const Sampler, int const Width, int const Height)
{
	if ((Width <= 0) || (Height <= 0) || (false == bool(m_kernel)))
	{
		return false;
	}
	if ((m_sourcesize != glm::ivec2{Width, Height}) || (false == bool(m_pyramid)))
	{
		if (m_pyramid)
		{
			destroy(Device, &Deletions, Value);
		}
		m_sourcesize = glm::ivec2{Width, Height};
		m_size = glm::ivec2{(Width + 1) / 2, (Height + 1) / 2};
		std::uint32_t levels{1};
		while ((std::max(m_size.x, m_size.y) >> levels) > 0)
		{
			++levels;
		}
		image_desc desc;
		desc.format = VK_FORMAT_R32_SFLOAT;
		desc.extent = {static_cast<std::uint32_t>(m_size.x), static_cast<std::uint32_t>(m_size.y), 1};
		desc.mips = levels;
		desc.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
		m_pyramid = std::make_shared<image>();
		if (false == m_pyramid->create(Memory, desc, "hi-z pyramid"))
		{
			m_pyramid.reset();
			m_sourcesize = glm::ivec2{0};
			return false;
		}
		for (std::uint32_t level = 0; level < levels; ++level)
		{
			VkImageViewCreateInfo info{};
			info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
			info.image = m_pyramid->handle();
			info.viewType = VK_IMAGE_VIEW_TYPE_2D;
			info.format = desc.format;
			info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, level, 1, 0, 1};
			VkImageView view{VK_NULL_HANDLE};
			VK14_VERIFY(vkCreateImageView(Device, &info, nullptr, &view));
			m_levels.push_back(view);
		}
		// the pyramid stays in the general layout: the kernel reads the levels it wrote, the culling all of them
		transition(Commands, *m_pyramid, VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
	}
	else
	{
		// the culling done with the pyramid made before
		memory_barrier(Commands, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
	}
	auto source{Depth};
	auto sourcelayout{VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
	auto sourcesize{m_sourcesize};
	for (std::size_t level = 0; level < m_levels.size(); ++level)
	{
		auto const targetsize{glm::max(m_size >> static_cast<int>(level), glm::ivec2{1})};
		hiz_constants constants{glm::ivec4{sourcesize, targetsize}, glm::ivec4{0}};
		m_kernel.dispatch(Commands, {image_descriptor(Sampler, source, sourcelayout), storage_image(m_levels[level])}, constants, (targetsize.x + gl::HIZ_GROUP_SIZE - 1) / gl::HIZ_GROUP_SIZE,
		                  (targetsize.y + gl::HIZ_GROUP_SIZE - 1) / gl::HIZ_GROUP_SIZE);
		memory_barrier(Commands, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
		source = m_levels[level];
		sourcelayout = VK_IMAGE_LAYOUT_GENERAL;
		sourcesize = targetsize;
	}
	return true;
}

cull_occlusion hiz_pyramid::occlusion(VkSampler const Sampler, glm::mat4 const &Projection) const
{
	cull_occlusion result;
	if (false == bool(m_pyramid))
	{
		return result;
	}
	result.pyramid = m_pyramid->view();
	result.sampler = Sampler;
	result.size = m_size;
	result.levels = static_cast<int>(m_levels.size());
	result.screen = glm::vec2{m_sourcesize};
	result.projection = Projection;
	return result;
}

// the cascades fitted to the depth of the view

namespace
{

struct fit_depth_constants
{
	glm::mat4 inv_projection;
};

struct fit_bounds_constants
{
	glm::mat4 inv_projection;
	glm::mat4 view_to_light;
	glm::vec4 split_params;
	glm::vec4 margin_params;
};

} // namespace

bool shadow_fitter::init(VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout, allocator const &Memory, std::uint32_t const Frames)
{
	try
	{
		if ((false == m_depthkernel.create(Device, Pipelines, Shaders.get("shadow_fit_depth.comp"), Passlayout, {}, "shadow fit depth")) ||
		    (false == m_boundskernel.create(Device, Pipelines, Shaders.get("shadow_fit_bounds.comp"), Passlayout, {}, "shadow fit bounds")))
		{
			return false;
		}
	}
	catch (gfx::shader_exception const &Exception)
	{
		log_error(Exception.what());
		return false;
	}
	return m_buffer.create(Memory, sizeof(gl::shadow_fit_data), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, memory_kind::device, "shadow fit") &&
	       m_readback.create(Memory, sizeof(gl::shadow_fit_data), Frames, "shadow fit readback");
}

void shadow_fitter::destroy(VkDevice Device)
{
	m_depthkernel.destroy(Device);
	m_boundskernel.destroy(Device);
	m_buffer.destroy();
	m_readback.destroy();
}

void shadow_fitter::measure(command_stream *Commands, VkImageView const Depth, VkSampler const Sampler, int const Width, int const Height, view const &View, unsigned int const Frame, std::uint32_t const Slot,
                            std::uint64_t const Value)
{
	if ((Width <= 0) || (Height <= 0) || (View.range <= 0.f) || (false == bool(*this)))
	{
		return;
	}
	auto const data{begin_measurement(View, Frame)};
	update_buffer(Commands, m_buffer, &data, sizeof(data), 0);
	auto const inverseprojection{glm::inverse(View.projection)};
	auto const viewtolight{glm::mat4{View.light * glm::inverse(View.rotation)}};
	auto const groupsx{(static_cast<std::uint32_t>(Width) + gl::SHADOW_FIT_GROUP_SIZE - 1) / gl::SHADOW_FIT_GROUP_SIZE};
	auto const groupsy{(static_cast<std::uint32_t>(Height) + gl::SHADOW_FIT_GROUP_SIZE - 1) / gl::SHADOW_FIT_GROUP_SIZE};
	std::vector<descriptor> const descriptors{image_descriptor(Sampler, Depth, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL), buffer_descriptor(m_buffer.handle(), 0, VK_WHOLE_SIZE)};
	m_depthkernel.dispatch(Commands, descriptors, fit_depth_constants{inverseprojection}, groupsx, groupsy);
	compute_barrier(Commands);
	m_boundskernel.dispatch(Commands, descriptors, fit_bounds_constants{inverseprojection, viewtolight, bounds_params(View), margin_params()}, groupsx, groupsy);
	compute_barrier(Commands);
	m_readback.push(Commands, Slot, Value, m_buffer.handle(), 0, sizeof(gl::shadow_fit_data));
}

shadow_fitter::result const *shadow_fitter::latest(std::uint64_t const Completed)
{
	gl::shadow_fit_data data;
	if (m_readback.pop(Completed, &data, sizeof(data)))
	{
		arrived(data);
	}
	return gfx::shadow_fit::latest();
}

} // namespace vk14
