/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14particles.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14context.h"
#include "utilities/Logs.h"

#include <cstring>

namespace vk14
{

namespace
{

// the push constants of particles_smoke_simulate.comp (std430)
struct simulate_constants
{
	std::uint32_t places;
	std::uint32_t pad0[3];
	glm::vec4 step;
	glm::vec4 wind;
	std::uint32_t seed;
	std::uint32_t pad1[3];
};
static_assert(offsetof(simulate_constants, step) == 16 && offsetof(simulate_constants, seed) == 48, "the layout of the constants of particles_smoke_simulate.comp");

// the push constants of particles_smoke_prepare.comp (std430)
struct prepare_constants
{
	std::uint32_t places;
	std::uint32_t pad0[3];
	glm::vec3 origin;
	float pad1;
	glm::mat4 view;
	glm::vec4 options;
	glm::vec4 planes[5];
};
static_assert(offsetof(prepare_constants, origin) == 16 && offsetof(prepare_constants, view) == 32 && offsetof(prepare_constants, options) == 96 && offsetof(prepare_constants, planes) == 112,
              "the layout of the constants of particles_smoke_prepare.comp");

// the block of particles_precipitation.comp (std140)
struct precipitation_block
{
	glm::uvec4 counts;
	glm::vec4 offsets[3];
	glm::vec4 sizes;
	glm::vec4 velocity;
	glm::vec4 timing;
	glm::mat4 view;
	glm::vec4 options;
	glm::vec4 planes[5];
};
static_assert(offsetof(precipitation_block, view) == 112 && offsetof(precipitation_block, planes) == 192 && sizeof(precipitation_block) == 272, "the layout of the block of particles_precipitation.comp");

std::uint32_t groups(std::uint32_t const Count)
{
	return (Count + gl::PARTICLE_GROUP_SIZE - 1) / gl::PARTICLE_GROUP_SIZE;
}

VkBufferUsageFlags constexpr storage_usage{VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT};

} // namespace

gpu_particles::~gpu_particles()
{
	destroy();
}

void gpu_particles::init(VkDevice const Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout const Passlayout, allocator &Memory, uploader &Uploads, deletion_queue &Deletions,
                         frame_ring &Ring, VkDeviceSize const Alignment, std::uint32_t const Frames, host Host)
{
	m_device = Device;
	m_pipelines = &Pipelines;
	m_shaders = &Shaders;
	m_passlayout = Passlayout;
	m_memory = &Memory;
	m_uploads = &Uploads;
	m_deletions = &Deletions;
	m_ring = &Ring;
	m_alignment = std::max<VkDeviceSize>(Alignment, 16);
	m_frames = Frames;
	m_host = std::move(Host);
}

void gpu_particles::destroy()
{
	if (m_device == VK_NULL_HANDLE)
	{
		return;
	}
	// the device is idle: everything goes at once
	m_deletions = nullptr;
	m_smokesimulate.destroy(m_device);
	m_smokeprepare.destroy(m_device);
	m_precipitationprepare.destroy(m_device);
	m_smokedraw.reset();
	m_precipitationdraw.reset();
	for (auto *buffer : {&m_particles, &m_sources, &m_owners, &m_smokeoutput, &m_smokecommand, &m_precipitationoutput, &m_precipitationcommand})
	{
		buffer->reset();
	}
	m_smokereadback.destroy();
	m_precipitationreadback.destroy();
	m_device = VK_NULL_HANDLE;
}

std::shared_ptr<buffer> gpu_particles::make_buffer(VkDeviceSize const Size, VkBufferUsageFlags const Usage, char const *Name)
{
	auto result{std::make_shared<buffer>()};
	if (false == result->create(*m_memory, std::max<VkDeviceSize>(Size, 16), Usage, memory_kind::device, Name))
	{
		ErrorLog(std::string("particles: no buffer for the ") + Name);
		return nullptr;
	}
	return result;
}

void gpu_particles::let_go(std::shared_ptr<buffer> &Buffer)
{
	if ((Buffer != nullptr) && (m_deletions != nullptr))
	{
		// the frames in flight may still read it
		m_deletions->push(m_uploads->current_value(), [old = std::move(Buffer)]() {});
	}
	Buffer.reset();
}

bool gpu_particles::gpu_init()
{
	if ((m_device == VK_NULL_HANDLE) || (false == bool(m_host.compute)) || (false == bool(m_host.draw)))
	{
		return false;
	}
	try
	{
		auto kernel = [&](compute_kernel &Kernel, char const *Shader, char const *Name) {
			if (false == Kernel.create(m_device, *m_pipelines, m_shaders->get(Shader), m_passlayout, {}, Name))
			{
				throw gfx::shader_exception(std::string("no compute pipeline of ") + Name);
			}
		};
		kernel(m_smokesimulate, "particles_smoke_simulate.comp", "smoke simulation");
		kernel(m_smokeprepare, "particles_smoke_prepare.comp", "smoke");
		kernel(m_precipitationprepare, "particles_precipitation.comp", "precipitation");
		auto program = [&](char const *Vertex, char const *Fragment) {
			auto result{std::make_shared<scene_program>()};
			result->vertex = m_shaders->get(Vertex);
			result->fragment = m_shaders->get(Fragment);
			result->fragment_name = Fragment;
			result->vertex_name = Vertex;
			result->vertices = vertex_kind::particle_instance;
			result->name = std::string(Vertex) + " + " + Fragment;
			return result;
		};
		m_smokedraw = program("particles_smoke.vert", "particles_smoke.frag");
		m_precipitationdraw = program("particles_precipitation.vert", "particles_precipitation.frag");
	}
	catch (gfx::shader_exception const &e)
	{
		ErrorLog("particles: can't use the programs of the particles of the gpu, the smoke stays on the cpu: " + std::string(e.what()));
		return false;
	}
	auto const commandusage{VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT};
	m_smokecommand = make_buffer(4 * sizeof(std::uint32_t), commandusage, "command of the smoke");
	m_precipitationcommand = make_buffer(4 * sizeof(std::uint32_t), commandusage, "command of the precipitation");
	if ((m_smokecommand == nullptr) || (m_precipitationcommand == nullptr) || (false == m_smokereadback.create(*m_memory, 4 * sizeof(std::uint32_t), m_frames, "smoke read back")) ||
	    (false == m_precipitationreadback.create(*m_memory, 4 * sizeof(std::uint32_t), m_frames, "precipitation read back")))
	{
		ErrorLog("particles: no buffers of the commands of the particles of the gpu, the smoke stays on the cpu");
		return false;
	}
	return true;
}

void gpu_particles::gpu_clear()
{
	for (auto *buffer : {&m_particles, &m_sources, &m_owners, &m_smokeoutput, &m_precipitationoutput})
	{
		let_go(*buffer);
	}
}

void gpu_particles::gpu_layout(std::uint32_t const Places, std::uint32_t const Kept, std::vector<std::uint32_t> const &Owners, std::size_t const Sources)
{
	auto const commands{m_host.compute()};
	auto particles{make_buffer(static_cast<VkDeviceSize>(Places) * sizeof(gl::particle_smoke), storage_usage, "smoke particles")};
	if (particles == nullptr)
	{
		return;
	}
	// the particles of the places kept go over to the new storage, the new places are dead
	memory_barrier(commands, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
	commands->fill_buffer(particles->handle(), 0, VK_WHOLE_SIZE, 0);
	if ((m_particles != nullptr) && (Kept > 0))
	{
		memory_barrier(commands, VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
		VkBufferCopy2 region{};
		region.sType = VK_STRUCTURE_TYPE_BUFFER_COPY_2;
		region.size = std::min<VkDeviceSize>(static_cast<VkDeviceSize>(Kept) * sizeof(gl::particle_smoke), m_particles->size());
		VkCopyBufferInfo2 copy{};
		copy.sType = VK_STRUCTURE_TYPE_COPY_BUFFER_INFO_2;
		copy.srcBuffer = m_particles->handle();
		copy.dstBuffer = particles->handle();
		copy.regionCount = 1;
		copy.pRegions = &region;
		commands->copy_buffer2(&copy);
	}
	memory_barrier(commands, VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
	let_go(m_particles);
	m_particles = std::move(particles);

	let_go(m_owners);
	let_go(m_smokeoutput);
	let_go(m_sources);
	m_owners = make_buffer(static_cast<VkDeviceSize>(Places) * sizeof(std::uint32_t), storage_usage, "smoke owners");
	m_smokeoutput = make_buffer(static_cast<VkDeviceSize>(Places) * sizeof(gl::particle_draw), storage_usage | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, "smoke draws");
	m_sources = make_buffer(static_cast<VkDeviceSize>(std::max<std::size_t>(Sources, 1)) * sizeof(gl::particle_smoke_source), storage_usage, "smoke sources");
	if (m_owners != nullptr)
	{
		// before the commands of the frame, the kernels of the frame read it
		m_uploads->upload_buffer(m_owners, Owners.data(), static_cast<VkDeviceSize>(Owners.size() * sizeof(std::uint32_t)));
	}
}

void gpu_particles::gpu_simulate(smoke_simulation const &Simulation, std::vector<gl::particle_smoke_source> const &Records)
{
	if ((m_particles == nullptr) || (m_sources == nullptr) || (m_owners == nullptr))
	{
		return;
	}
	if (false == Records.empty())
	{
		m_uploads->upload_buffer(m_sources, Records.data(), static_cast<VkDeviceSize>(Records.size() * sizeof(gl::particle_smoke_source)));
	}
	auto const commands{m_host.compute()};
	simulate_constants constants{};
	constants.places = Simulation.places;
	constants.step = Simulation.step;
	constants.wind = Simulation.wind;
	constants.seed = Simulation.seed;
	m_host.bind(m_smokesimulate);
	m_smokesimulate.dispatch(commands,
	                         {buffer_descriptor(m_particles->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_sources->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_owners->handle(), 0, VK_WHOLE_SIZE)},
	                         constants, groups(Simulation.places));
	compute_barrier(commands);
}

void gpu_particles::reset_command(command_stream *const Commands, buffer const &Command)
{
	std::uint32_t const command[4]{4, 0, 0, 0};
	update_buffer(Commands, Command, command, sizeof(command), 0);
}

void gpu_particles::collect(readback_ring &Readback, std::uint64_t const Completed, std::size_t &Drawn)
{
	std::uint32_t command[4];
	if (Readback.pop(Completed, command, sizeof(command)))
	{
		Drawn = command[1];
	}
}

void gpu_particles::gpu_render_smoke(smoke_draw const &Draw, std::size_t &Drawn)
{
	collect(m_smokereadback, m_host.completed(), Drawn);
	if ((m_particles == nullptr) || (m_sources == nullptr) || (m_owners == nullptr) || (m_smokeoutput == nullptr))
	{
		return;
	}
	auto const commands{m_host.compute()};
	reset_command(commands, *m_smokecommand);
	prepare_constants constants{};
	constants.places = Draw.places;
	constants.origin = Draw.origin;
	constants.view = Draw.view;
	constants.options = Draw.options;
	std::copy(Draw.planes.begin(), Draw.planes.end(), constants.planes);
	m_host.bind(m_smokeprepare);
	m_smokeprepare.dispatch(commands,
	                        {buffer_descriptor(m_particles->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_sources->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_owners->handle(), 0, VK_WHOLE_SIZE),
	                         buffer_descriptor(m_smokecommand->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_smokeoutput->handle(), 0, VK_WHOLE_SIZE)},
	                        constants, groups(Draw.places));
	compute_barrier(commands);
	// the count of the instances is what the draw takes, read back once the gpu is done with the frame
	m_smokereadback.push(commands, m_host.slot(), m_host.value(), m_smokecommand->handle(), 0, 4 * sizeof(std::uint32_t));
	m_host.draw(*m_smokedraw, m_smokeoutput->handle(), m_smokecommand->handle(), Draw.soft, glm::vec4{0.f});
}

void gpu_particles::gpu_collect_precipitation(std::size_t &Drawn)
{
	collect(m_precipitationreadback, m_host.completed(), Drawn);
}

void gpu_particles::gpu_precipitation_storage(std::uint32_t const Capacity)
{
	let_go(m_precipitationoutput);
	m_precipitationoutput = make_buffer(static_cast<VkDeviceSize>(Capacity) * sizeof(gl::particle_draw), storage_usage | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, "precipitation draws");
}

void gpu_particles::gpu_render_precipitation(precipitation_draw const &Draw)
{
	if (m_precipitationoutput == nullptr)
	{
		return;
	}
	precipitation_block block{};
	block.counts = Draw.counts;
	std::copy(Draw.offsets.begin(), Draw.offsets.end(), block.offsets);
	block.sizes = Draw.sizes;
	block.velocity = Draw.velocity;
	block.timing = Draw.timing;
	block.view = Draw.view;
	block.options = Draw.options;
	std::copy(Draw.planes.begin(), Draw.planes.end(), block.planes);
	auto const slice{m_ring->allocate(sizeof(block), m_alignment)};
	if (false == bool(slice))
	{
		return;
	}
	std::memcpy(slice.data, &block, sizeof(block));
	auto const commands{m_host.compute()};
	reset_command(commands, *m_precipitationcommand);
	m_host.bind(m_precipitationprepare);
	m_precipitationprepare.run(commands,
	                           {buffer_descriptor(m_precipitationcommand->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(m_precipitationoutput->handle(), 0, VK_WHOLE_SIZE),
	                            buffer_descriptor(slice.buffer, slice.offset, sizeof(block))},
	                           groups(Draw.count));
	compute_barrier(commands);
	m_precipitationreadback.push(commands, m_host.slot(), m_host.value(), m_precipitationcommand->handle(), 0, 4 * sizeof(std::uint32_t));
	m_host.draw(*m_precipitationdraw, m_precipitationoutput->handle(), m_precipitationcommand->handle(), Draw.soft, Draw.pixel);
}

} // namespace vk14
