/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <memory>

#include "rendering/common/windscreenwater.h"
#include "gl/shader.h"
#include "gl/ssbo.h"
#include "gl/ubo.h"

// the water on the windscreens for the opengl 4.6 renderer (gfx::windscreen_water): the storage of the water, the block of the
// step and the compute programs
class opengl46_windscreen_water : public gfx::windscreen_water
{
  public:
	opengl46_windscreen_water() = default;
	~opengl46_windscreen_water() override = default;

	// the storage bound to its point for the glasses drawn next (the bindings are kept by every context of its own)
	void bind() const;

  protected:
	bool gpu_init() override;
	void gpu_setup(std::size_t Slot, std::vector<glm::vec4> const &Axes) override;
	bool gpu_bake(std::size_t Slot, texture_handle Mask, gl::water_ubs const &Block) override;
	void gpu_step(gl::water_ubs const &Block, std::uint32_t Tries) override;

  private:
	// the passes of the step, in their order
	enum pass
	{
		trail,
		spawn,
		move,
		clear,
		grid,
		merge,
		gather,
		settle,
		bake,
		count
	};
	std::array<std::unique_ptr<gl::program>, pass::count> m_programs;
	std::unique_ptr<gl::ssbo> m_storage;
	std::unique_ptr<gl::ubo> m_block;
};
