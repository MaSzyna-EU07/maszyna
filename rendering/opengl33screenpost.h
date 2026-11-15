/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scripting/scriptscreens.h"
#include "gl/framebuffer.h"
#include "gl/shader.h"
#include "gl/vao.h"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

// the post-processing of the script screens (image:led_post, image:resize) on the gpu, for the OpenGL 3.3 renderer: a
// pass for a led_post and two for a resize (along the rows, then along the columns), the last pass into the screen's
// texture. shaders/screen_led.frag and shaders/screen_resize.frag give the pixels of scripting/scriptscreenkernels.cpp
class opengl33_screen_post final : public script_screens::post_processor
{
  public:
	opengl33_screen_post() = default;
	~opengl33_screen_post() override;

	bool apply(ITexture &Target, std::span<std::uint8_t const> Pixels, int Width, int Height, std::span<script_screens::post_step const> Steps) override;

  private:
	// a shader of the passes, with the places of its uniforms
	struct stage
	{
		std::unique_ptr<gl::program> program;
		GLint source_size{-1};
		GLint target_size{-1};
		GLint flip{-1};
		GLint mode{-1};
	};
	// a texture the passes read
	struct surface
	{
		GLuint texture{0};
		int width{0};
		int height{0};
	};
	struct pass
	{
		stage const *shader;
		int width;
		int height;
		int mode; // led: 1 with the glow; resize: 0 along the rows, 1 along the columns
	};

	// makes the shaders on the first use. false when they failed, then and from then on
	bool setup();
	std::vector<pass> passes_of(std::span<script_screens::post_step const> Steps, int Width, int Height) const;
	// draws a pass from the source into the texture; Last: the rows go bottom row first, as the screen's texture takes them
	bool draw(pass const &Pass, surface const &Source, GLuint Target, bool Last) const;

	static stage make_stage(gl::shader const &Vertex, std::string const &Fragment);
	// sizes the texture, and fills it when there are pixels
	static void store(surface &Surface, int Width, int Height, std::uint8_t const *Pixels);

	bool m_failed{false};
	stage m_led;
	stage m_resize;
	std::unique_ptr<gl::vao> m_vao;
	std::unique_ptr<gl::framebuffer> m_framebuffer;
	surface m_source; // the image as drawn
	std::array<surface, 2> m_between; // what the passes before the last one give, in turn
};
