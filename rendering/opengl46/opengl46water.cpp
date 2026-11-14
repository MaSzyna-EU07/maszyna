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

#include "rendering/renderer.h"
#include "utilities/Logs.h"

#include <algorithm>

namespace
{

GLuint groups(std::size_t const Count)
{
	return static_cast<GLuint>((Count + gl::WATER_GROUP_SIZE - 1) / gl::WATER_GROUP_SIZE);
}

} // namespace

bool opengl46_windscreen_water::gpu_init()
{
	try
	{
		std::array<char const *, pass::count> const names{"water_trail.comp", "water_spawn.comp", "water_move.comp", "water_clear.comp", "water_grid.comp",
		                                                  "water_merge.comp", "water_gather.comp", "water_settle.comp", "water_bake.comp", "water_snow.comp",
		                                                  "water_snowlayer.comp"};
		for (std::size_t i = 0; i < names.size(); ++i)
		{
			gl::shader shader(names[i]);
			m_programs[i] = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
		}
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("windscreen water: can't use the programs of the simulation: " + std::string(e.what()));
		return false;
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
	if (false == m_raincelltried)
	{
		m_raincelltried = true;
		try
		{
			gl::shader shader("rain_cells.comp");
			m_raincellprogram = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
			m_raincells = std::make_unique<gl::ssbo>(static_cast<GLsizeiptr>(gl::RAIN_CELLS_COUNT * 16), gl::RAIN_CELLS_SSBO_BINDING);
		}
		catch (gl::shader_exception const &e)
		{
			ErrorLog("rain cells: can't use rain_cells.comp, the glasses with rain stay dry: " + std::string(e.what()));
			m_raincellprogram.reset();
		}
	}
	if ((!m_raincellprogram) || (!m_raincells))
		return;
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
	::glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
}

bool opengl46_windscreen_water::gpu_bake(std::size_t const Slot, texture_handle const Mask, gl::water_ubs const &Block)
{
	if ((!m_storage) || (Mask == null_handle))
		return false;
	auto &texture{GfxRenderer->Texture(Mask)};
	if ((false == texture.create()) || (false == texture.get_is_ready()) || (texture.get_id() == 0))
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
	if ((!m_storage) || (Slot >= gl::WATER_SLOTS) || (Atlas == null_handle) || (Normal == null_handle))
		return false;
	auto &atlas{GfxRenderer->Texture(Atlas)};
	auto &normal{GfxRenderer->Texture(Normal)};
	if ((false == atlas.create()) || (false == atlas.get_is_ready()) || (atlas.get_id() == 0) || (false == normal.create()) || (false == normal.get_is_ready()) ||
	    (normal.get_id() == 0))
		return false;
	m_storage->bind_base();
	m_block->update(Block);
	m_block->bind_uniform();
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_SNOW_ATLAS_TEX), static_cast<GLuint>(atlas.get_id()));
	::glBindTextureUnit(static_cast<GLuint>(gl::WATER_SNOW_NORMAL_TEX), static_cast<GLuint>(normal.get_id()));
	m_programs[pass::snowlayer]->dispatch(groups(static_cast<std::size_t>(gl::WATER_LAYER) * gl::WATER_LAYER));
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
	if (Block.snow_slots != glm::vec4{0.f})
		run(pass::snow, groups(static_cast<std::size_t>(gl::WATER_MAP) * gl::WATER_MAP));
	gl::program::unbind();
}
