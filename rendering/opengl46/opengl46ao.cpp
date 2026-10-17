/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46ao.h"

#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

namespace
{

// work group of the compute passes, 8x8 (gtao_*.comp)
int const GROUP_SIZE{8};

} // namespace

void opengl46_ao::init()
{
	auto const make_compute = [](std::string const &Name) {
		gl::shader shader(Name);
		return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	};
	m_main_program = make_compute("gtao_main.comp");
	m_denoise_program = make_compute("gtao_denoise.comp");
	{
		gl::shader vertex("quad.vert");
		gl::shader fragment("postfx_ssao_apply.frag");
		m_apply_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, fragment});
	}
	m_vao = std::make_unique<gl::vao>();
}

void opengl46_ao::compute(opengl_texture &Depth, opengl_texture &Raw, opengl_texture &Smooth, glm::mat4 const &Projection)
{
	m_timer.begin();
	m_timing = true;

	auto const inverseprojection{glm::inverse(Projection)};
	auto const groups_x{static_cast<GLuint>((Raw.get_width() + GROUP_SIZE - 1) / GROUP_SIZE)};
	auto const groups_y{static_cast<GLuint>((Raw.get_height() + GROUP_SIZE - 1) / GROUP_SIZE)};

	// horizon search
	Depth.bind(0);
	glProgramUniformMatrix4fv(*m_main_program, 0, 1, GL_FALSE, glm::value_ptr(inverseprojection));
	glProgramUniform2f(*m_main_program, 1, Global.gfx_postfx_ao_radius, Projection[0][0] * 0.5f * static_cast<float>(Raw.get_width()));
	glProgramUniform4f(*m_main_program, 2, Projection[0][3], Projection[1][3], Projection[2][3], Projection[3][3]);
	glProgramUniform2i(*m_main_program, 3, Global.gfx_postfx_ao_slices, Global.gfx_postfx_ao_steps);
	glBindImageTexture(0, Raw.id, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R8);
	m_main_program->dispatch(groups_x, groups_y);
	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

	// smoothing
	Raw.bind(1);
	glProgramUniformMatrix4fv(*m_denoise_program, 0, 1, GL_FALSE, glm::value_ptr(inverseprojection));
	glBindImageTexture(0, Smooth.id, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R8);
	m_denoise_program->dispatch(groups_x, groups_y);
	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

	glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R8);
	opengl_texture::reset_unit_cache();
}

void opengl46_ao::apply(int const Mode, opengl_texture &Depth, opengl_texture &Occlusion, gl::framebuffer &Target, glm::mat4 const &Projection)
{
	Target.bind();
	m_apply_program->bind();
	m_vao->bind();
	Depth.bind(0);
	Occlusion.bind(1);
	glProgramUniformMatrix4fv(*m_apply_program, 0, 1, GL_FALSE, glm::value_ptr(glm::inverse(Projection)));
	glProgramUniform4f(*m_apply_program, 1, (Mode == 1 ? 1.f : 0.f), Global.gfx_postfx_ao_strength, Global.gfx_postfx_ao_bounce, 0.f);

	// result = image * occlusion; the blending state is put back as it was
	auto const blending{glIsEnabled(GL_BLEND) == GL_TRUE};
	GLint blendfunc[4];
	glGetIntegerv(GL_BLEND_SRC_RGB, &blendfunc[0]);
	glGetIntegerv(GL_BLEND_DST_RGB, &blendfunc[1]);
	glGetIntegerv(GL_BLEND_SRC_ALPHA, &blendfunc[2]);
	glGetIntegerv(GL_BLEND_DST_ALPHA, &blendfunc[3]);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ZERO, GL_SRC_COLOR);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBlendFuncSeparate(blendfunc[0], blendfunc[1], blendfunc[2], blendfunc[3]);
	if (!blending)
		glDisable(GL_BLEND);

	opengl_texture::reset_unit_cache();

	if (m_timing)
	{
		m_timer.end();
		m_timing = false;
	}
}

std::string opengl46_ao::info_times()
{
	std::string text;
	if (auto const time = m_timer.result())
		text += " ambient occlusion: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}

void opengl46_ao::draw_debug_ui(bool const Available)
{
	if (!Available)
	{
		ImGui::TextUnformatted("ambient occlusion: off at the start, set gfx.postfx.ao in eu07.ini");
		return;
	}
	char const *modes[]{"off", "older ssao", "ground truth (gtao)"};
	ImGui::Combo("ambient occlusion (gfx.postfx.ao)", &Global.gfx_postfx_ao, modes, IM_ARRAYSIZE(modes));
	if (Global.gfx_postfx_ao != 2)
		return;
	ImGui::SliderFloat("ao radius (m)", &Global.gfx_postfx_ao_radius, 0.1f, 10.f, "%.2f", ImGuiSliderFlags_Logarithmic);
	ImGui::SliderFloat("ao strength", &Global.gfx_postfx_ao_strength, 0.f, 4.f);
	ImGui::SliderFloat("ao bounce albedo", &Global.gfx_postfx_ao_bounce, 0.f, 1.f);
	ImGui::SliderInt("ao slices", &Global.gfx_postfx_ao_slices, 1, 8);
	ImGui::SliderInt("ao steps per side", &Global.gfx_postfx_ao_steps, 1, 16);
}
