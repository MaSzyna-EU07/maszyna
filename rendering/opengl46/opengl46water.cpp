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
		                                                  "water_merge.comp", "water_gather.comp", "water_settle.comp", "water_bake.comp"};
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
	// the wetted paths the blades went over in the step wiped
	if (Block.blades != Block.blades_before)
		run(pass::trail, groups(static_cast<std::size_t>(gl::WATER_TRAIL) * gl::WATER_TRAIL));
	run(pass::spawn, groups(Tries));
	run(pass::move, drops);
	run(pass::clear, cells);
	run(pass::grid, drops);
	run(pass::merge, drops);
	run(pass::gather, drops);
	run(pass::settle, drops);
	run(pass::clear, cells);
	run(pass::grid, drops);
	gl::program::unbind();
}
