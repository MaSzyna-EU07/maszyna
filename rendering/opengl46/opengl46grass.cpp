/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46grass.h"

#include "rendering/openglcamera.h"
#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace
{

GLuint make_texture(GLenum const Format, int const Width, int const Height, GLenum const Filter)
{
	GLuint texture{0};
	::glCreateTextures(GL_TEXTURE_2D, 1, &texture);
	::glTextureStorage2D(texture, 1, Format, Width, Height);
	::glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, Filter);
	::glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, Filter);
	::glTextureParameteri(texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	::glTextureParameteri(texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	return texture;
}

} // namespace

opengl46_grass::~opengl46_grass()
{
	// the textures go with the context; deleted here only if it's still there
	if (::glfwGetCurrentContext() != nullptr)
		clear();
}

bool opengl46_grass::init()
{
	if (m_state != 0)
		return m_state > 0;
	m_state = -1;
	try
	{
		gl::shader const vertex("grass.vert");
		gl::shader const fragment("grass.frag");
		m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, fragment});
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("heightmap terrain: can't use the grass shader: " + std::string(e.what()));
		return false;
	}
	m_vao = std::make_unique<gl::vao>();
	// the locations grass.vert gives its uniforms (a program loaded from SPIR-V doesn't know their names)
	m_uniforms.tile = 0;
	m_uniforms.part = 1;
	m_uniforms.chunk = 2;
	m_uniforms.camera = 3;
	m_uniforms.lod = 4;
	m_uniforms.wind = 5;
	m_uniforms.base = 6;
	m_uniforms.tip = 10;
	m_uniforms.shape = 14;
	m_uniforms.share = 18;
	m_uniforms.dry = 19;
	m_uniforms.season = 23;
	m_state = 1;
	return true;
}

void opengl46_grass::gpu_release(std::uint64_t const Id)
{
	auto const lookup{m_textures.find(Id)};
	if (lookup == m_textures.end())
		return;
	auto &chunk{lookup->second};
	for (auto *texture : {&chunk.heights, &chunk.density, &chunk.holes})
		if (*texture != 0)
		{
			::glDeleteTextures(1, texture);
			*texture = 0;
		}
	m_textures.erase(lookup);
}

void opengl46_grass::gpu_upload(std::uint64_t const Id, gfx::grass_chunk const &Source)
{
	auto &chunk{m_textures[Id]};
	auto const side{Source.cells + 1};
	chunk.heights = make_texture(GL_R32F, side, side, GL_NEAREST);
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	::glTextureSubImage2D(chunk.heights, 0, 0, 0, side, side, GL_RED, GL_FLOAT, Source.heights);

	// the densities of the kinds side by side, as the channels of a texel
	auto const samples{Source.samples + 1};
	auto const plane{static_cast<std::size_t>(samples) * samples};
	std::size_t kinds{0};
	while (kinds < Source.kinds.size() && Source.kinds[kinds] >= 0)
		++kinds;
	std::vector<std::uint8_t> texels(plane * 4, 0);
	for (std::size_t kind = 0; kind < kinds; ++kind)
		for (std::size_t i = 0; i < plane; ++i)
			texels[i * 4 + kind] = Source.density[kind * plane + i];
	chunk.density = make_texture(GL_RGBA8, samples, samples, GL_LINEAR);
	::glTextureSubImage2D(chunk.density, 0, 0, 0, samples, samples, GL_RGBA, GL_UNSIGNED_BYTE, texels.data());

	std::uint8_t const none{0};
	auto const holeside{(Source.holes > 0 && Source.hole_mask != nullptr) ? Source.holes + 1 : 1};
	chunk.holes = make_texture(GL_R8, holeside, holeside, GL_LINEAR);
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	::glTextureSubImage2D(chunk.holes, 0, 0, 0, holeside, holeside, GL_RED, GL_UNSIGNED_BYTE, holeside > 1 ? Source.hole_mask : &none);
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
}

std::size_t opengl46_grass::render(opengl_camera const &Camera, glm::mat4 const &View, std::function<void(glm::mat4 const &)> const &Modelview)
{
	if (false == prepare(Camera.position(), [&](scene::bounding_area const &Area) { return Camera.visible(Area); }) || false == init())
		return 0;

	m_program->bind();
	m_vao->bind();
	::glDisable(GL_CULL_FACE);
	auto const &frame{this->frame()};
	::glUniform4fv(m_uniforms.lod, 1, glm::value_ptr(frame.lod));
	::glUniform4fv(m_uniforms.season, 1, glm::value_ptr(frame.season));
	::glUniform4fv(m_uniforms.wind, 1, glm::value_ptr(frame.wind));

	auto const &tiles{tile_draws()};
	for (auto const &chunk : chunk_draws())
	{
		// the chunk is drawn relative to its corner, the heights as they are
		Modelview(glm::translate(View, glm::vec3{chunk.offset}));
		::glUniform4fv(m_uniforms.chunk, 1, glm::value_ptr(chunk.chunk));
		::glUniform4fv(m_uniforms.camera, 1, glm::value_ptr(chunk.camera));
		::glUniform4fv(m_uniforms.base, 4, glm::value_ptr(chunk.base[0]));
		::glUniform4fv(m_uniforms.tip, 4, glm::value_ptr(chunk.tip[0]));
		::glUniform4fv(m_uniforms.shape, 4, glm::value_ptr(chunk.shape[0]));
		::glUniform4fv(m_uniforms.share, 1, glm::value_ptr(chunk.share));
		::glUniform4fv(m_uniforms.dry, 4, glm::value_ptr(chunk.dry[0]));
		auto const &textures{m_textures[chunk.id]};
		GLuint const units[]{textures.heights, textures.density, textures.holes};
		::glBindTextures(0, 3, units);
		for (auto i = chunk.first; i < chunk.first + chunk.count; ++i)
		{
			auto const &tile{tiles[i]};
			::glUniform4fv(m_uniforms.tile, 1, glm::value_ptr(tile.tile));
			::glUniform4fv(m_uniforms.part, 1, glm::value_ptr(tile.part));
			::glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, tile.segments * 2 + 1, tile.count);
		}
	}

	::glEnable(GL_CULL_FACE);
	gl::vao::unbind();
	// the units of the textures were taken behind the back of the texture cache
	opengl_texture::reset_unit_cache();
	return triangles();
}
