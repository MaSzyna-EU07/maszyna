/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46water.h"

#include "model/Texture.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <array>
#include <iterator>

namespace
{

GLuint groups(std::size_t const Count)
{
	return static_cast<GLuint>((Count + gl::WATER_GROUP_SIZE - 1) / gl::WATER_GROUP_SIZE);
}

// a texture a kernel reads, made ready: read from its file first when nothing read it yet. the opengl renderer reads a texture
// when a draw binds it, and the glasses never draw the mask of their wipers (only the kernels read it), so without this the mask
// stayed unread and the glass never got its map
bool kernel_texture(ITexture &Texture)
{
	if (auto *source{dynamic_cast<texture_source *>(&Texture)}; (source != nullptr) && (source->data_state == resource_state::none) && (false == source->is_rendertarget))
		source->load();
	return Texture.create() && Texture.get_is_ready() && (Texture.get_id() != 0);
}

} // namespace

namespace
{
// the programs of the passes, in their order (opengl46_windscreen_water::pass). the passes of the snow (from pass::snow on) are
// optional: without them the glasses of the snow stay clear and the rain works on
constexpr std::array<char const *, 11> PROGRAM_NAMES{"water_trail.comp", "water_spawn.comp", "water_move.comp", "water_clear.comp", "water_grid.comp",
                                                     "water_merge.comp", "water_gather.comp", "water_settle.comp", "water_bake.comp", "water_snow.comp",
                                                     "water_snowlayer.comp"};
} // namespace

// the program of a pass made; false when it doesn't compile (a pass of the snow then leaves the snow out, both of its passes)
bool opengl46_windscreen_water::make_program(std::size_t const Index)
{
	try
	{
		gl::shader shader(PROGRAM_NAMES[Index]);
		m_programs[Index] = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
		return true;
	}
	catch (gl::shader_exception const &e)
	{
		if (Index >= pass::snow)
		{
			ErrorLog("windscreen water: can't use " + std::string(PROGRAM_NAMES[Index]) + ", the glasses of the snow stay clear: " + std::string(e.what()));
			m_nosnow = true;
			m_programs[pass::snow].reset();
			m_programs[pass::snowlayer].reset();
		}
		else
		{
			ErrorLog("windscreen water: " + std::string(PROGRAM_NAMES[Index]) + " not made: " + std::string(e.what()));
		}
		return false;
	}
}

// the program of the cells of the rain (rain_cells.comp) made, once; its storage waits for the first rain (update_rain_cells())
void opengl46_windscreen_water::make_rain_cells_program()
{
	if (m_raincelltried)
		return;
	m_raincelltried = true;
	try
	{
		gl::shader shader("rain_cells.comp");
		m_raincellprogram = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("rain cells: can't use rain_cells.comp, the glasses with rain stay dry: " + std::string(e.what()));
		m_raincellprogram.reset();
	}
}

bool opengl46_windscreen_water::prepare_next()
{
	static_assert(PROGRAM_NAMES.size() == pass::count, "a name for every pass of the water");
	for (std::size_t index = 0; index < PROGRAM_NAMES.size(); ++index)
	{
		if (m_programs[index] || ((index >= pass::snow) && m_nosnow))
			continue;
		// a pass of the water which doesn't compile is left to gpu_init(), which goes without the simulation
		return (make_program(index) || (index >= pass::snow));
	}
	// the cells of the rain made ahead as well, so the first rain doesn't compile them in the middle of a frame
	if (false == m_raincelltried)
	{
		make_rain_cells_program();
		return true;
	}
	return false;
}

bool opengl46_windscreen_water::gpu_init()
{
	// the ones made ahead (prepare_next()) are kept
	for (std::size_t i = 0; i < PROGRAM_NAMES.size(); ++i)
	{
		if (m_programs[i] || ((i >= pass::snow) && m_nosnow))
			continue;
		if ((false == make_program(i)) && (i < pass::snow))
		{
			ErrorLog("windscreen water: can't use the programs of the simulation");
			return false;
		}
	}
	m_storage = std::make_unique<gl::ssbo>(static_cast<GLsizeiptr>(gl::WATER_STORAGE_SIZE), gl::WATER_SSBO_BINDING);
	::glClearNamedBufferData(*m_storage, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
	m_block = std::make_unique<gl::ubo>(sizeof(gl::water_ubs), gl::WATER_UBO_BINDING);
	return true;
}

void opengl46_windscreen_water::bind() const
{
	if (m_storage)
		m_storage->bind_base();
	if (m_raincells)
		m_raincells->bind_base();
}

void opengl46_windscreen_water::update_rain_cells(float const Time, float const Rain)
{
	if (Rain <= 0.f)
		return;
	make_rain_cells_program();
	if (!m_raincellprogram)
		return;
	// the storage of the cells made at the first rain
	if (!m_raincells)
		m_raincells = std::make_unique<gl::ssbo>(static_cast<GLsizeiptr>(gl::RAIN_CELLS_COUNT * 16), gl::RAIN_CELLS_SSBO_BINDING);
	// the glasses of the frame before read the cells, this one writes them
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_raincells->bind_base();
	auto const groups = [](std::size_t const Count) { return static_cast<GLuint>((Count + gl::RAIN_CELLS_GROUP_SIZE - 1) / gl::RAIN_CELLS_GROUP_SIZE); };
	::glProgramUniform1f(*m_raincellprogram, 0, Time);
	::glProgramUniform1f(*m_raincellprogram, 1, Rain);
	// the cells, then the reach of the drops into the quarters of each drop cell, which reads them
	::glProgramUniform1i(*m_raincellprogram, 2, 0);
	m_raincellprogram->dispatch(groups(gl::RAIN_MASK_BASE));
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	::glProgramUniform1i(*m_raincellprogram, 2, 1);
	m_raincellprogram->dispatch(groups(gl::RAIN_CELLS_COUNT - gl::RAIN_MASK_BASE));
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	gl::program::unbind();
}

void opengl46_windscreen_water::gpu_setup(std::size_t const Slot, std::vector<glm::vec4> const &Axes)
{
	if (!m_storage)
		return;
	// the frames before may still draw with the storage
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
	// the drops, the grid, the paths, the header and the counts of the slot cleared, the axes of the map put in
	for (auto const &part : gl::water_slot_parts(Slot))
		::glClearNamedBufferSubData(*m_storage, GL_R32UI, static_cast<GLintptr>(part.first), static_cast<GLsizeiptr>(part.second), GL_RED_INTEGER, GL_UNSIGNED_INT, nullptr);
	auto const axes{gl::water_slot_parts(Slot)[1]};
	m_storage->update(Axes.data(), static_cast<GLintptr>(axes.first), static_cast<GLsizeiptr>(std::min(axes.second, Axes.size() * sizeof(glm::vec4))));
	// where the layer of its snow lies
	auto const info{layer_info(Slot, Axes)};
	m_storage->update(info.data(), static_cast<GLintptr>(gl::WATER_LAYER_INFO_OFFSET + gl::WATER_LAYER_INFO_SIZE * Slot), static_cast<GLsizeiptr>(sizeof(info)));
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
}

bool opengl46_windscreen_water::gpu_bake(std::size_t const Slot, texture_handle const Mask, gl::water_ubs const &Block)
{
	if ((!m_storage) || (Mask == null_handle))
		return false;
	auto &texture{GfxRenderer->Texture(Mask)};
	if (false == kernel_texture(texture))
		return false;
	m_storage->bind_base();
	m_block->update(Block);
	m_block->bind_uniform();
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_MASK_TEX), static_cast<GLuint>(texture.get_id()));
	m_programs[pass::bake]->dispatch(groups(static_cast<std::size_t>(gl::WATER_MAP) * gl::WATER_MAP));
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_MASK_TEX), 0);
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	gl::program::unbind();
	return true;
}

bool opengl46_windscreen_water::gpu_snow_layer(std::size_t const Slot, texture_handle const Atlas, texture_handle const Normal, gl::water_ubs const &Block)
{
	if ((!m_storage) || (!m_programs[pass::snowlayer]) || (Slot >= gl::WATER_SLOTS) || (Atlas == null_handle) || (Normal == null_handle))
		return false;
	auto &atlas{GfxRenderer->Texture(Atlas)};
	auto &normal{GfxRenderer->Texture(Normal)};
	if ((false == kernel_texture(atlas)) || (false == kernel_texture(normal)))
		return false;
	m_storage->bind_base();
	m_block->update(Block);
	m_block->bind_uniform();
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_SNOW_ATLAS_TEX), static_cast<GLuint>(atlas.get_id()));
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_SNOW_NORMAL_TEX), static_cast<GLuint>(normal.get_id()));
	m_programs[pass::snowlayer]->dispatch(groups(gl::WATER_LAYER_TEXELS));
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_SNOW_ATLAS_TEX), 0);
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_SNOW_NORMAL_TEX), 0);
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	gl::program::unbind();
	return true;
}

void opengl46_windscreen_water::gpu_step(gl::water_ubs const &Block, std::uint32_t const Tries)
{
	if (!m_storage)
		return;
	m_storage->bind_base();
	m_block->update(Block);
	m_block->bind_uniform();
	auto const slots{static_cast<GLuint>(gl::WATER_SLOTS)};
	auto const drops{groups(gl::WATER_DROPS)};
	auto const cells{groups(static_cast<std::size_t>(gl::WATER_GRID) * gl::WATER_GRID)};
	auto const run = [&](pass const Pass, GLuint const Groups) {
		if (Groups == 0)
			return;
		m_programs[Pass]->dispatch(Groups, slots);
		::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	};
	// the drops only where a glass of the water is (the glasses of the snow have passes of their own)
	auto const water{std::ranges::any_of(Block.slots, [](glm::vec4 const &Slot) { return Slot.z > 0.5f; })};
	auto const drop_groups{water ? drops : 0u};
	auto const cell_groups{water ? cells : 0u};
	// the wetted paths the blades went over in the step wiped
	if (water && (Block.blades != Block.blades_before))
		run(pass::trail, groups(static_cast<std::size_t>(gl::WATER_TRAIL) * gl::WATER_TRAIL));
	run(pass::spawn, water ? groups(Tries) : 0u);
	run(pass::move, drop_groups);
	run(pass::clear, cell_groups);
	run(pass::grid, drop_groups);
	run(pass::merge, drop_groups);
	run(pass::gather, drop_groups);
	run(pass::settle, drop_groups);
	run(pass::clear, cell_groups);
	run(pass::grid, drop_groups);
	// the snow of the glasses of the snow, a texel of their maps a thread
	if ((Block.snow_slots != glm::vec4{0.f}) && m_programs[pass::snow])
		run(pass::snow, groups(static_cast<std::size_t>(gl::WATER_MAP) * gl::WATER_MAP));
	gl::program::unbind();
}
