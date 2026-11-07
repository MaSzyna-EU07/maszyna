/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46terrain.h"

#include "rendering/opengl46/opengl46geometrybank.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace
{

void bind_for_upload(opengl_texture const &Texture)
{
	::glActiveTexture(GL_TEXTURE0);
	::glBindTexture(Texture.target, Texture.id);
}

void set_anisotropy(GLenum const Target)
{
	if (Global.AnisotropicFiltering >= 0)
		::glTexParameterf(Target, GL_TEXTURE_MAX_ANISOTROPY, Global.AnisotropicFiltering);
}

} // namespace

bool opengl46_terrain_materials::gpu_init()
{
	try
	{
		m_copy = std::make_unique<gl::postfx>(gl::shader("copy.frag"));
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("heightmap terrain: can't use the copy shader: " + std::string(e.what()));
		return false;
	}
	m_framebuffer = std::make_unique<gl::framebuffer>();
	return true;
}

void opengl46_terrain_materials::gpu_texture(texture_source &Texture, texture_kind const Kind)
{
	auto &texture{static_cast<opengl_texture &>(Texture)};
	if (texture.id == opengl_texture::invalid_id)
		::glGenTextures(1, &texture.id);
	texture.target = (Kind == texture_kind::layers ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D);
}

void opengl46_terrain_materials::gpu_white(texture_source &Texture)
{
	auto const &white{static_cast<opengl_texture &>(Texture)};
	std::uint8_t const texel[4]{255, 255, 255, 255};
	bind_for_upload(white);
	::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, texel);
}

void opengl46_terrain_materials::gpu_storage(texture_array const &Array, int const Layers, bool const Fresh)
{
	auto &texture{m_gltextures.texture(Array.handle)};
	if (Fresh)
	{
		::glDeleteTextures(1, &texture.id);
		::glGenTextures(1, &texture.id);
	}
	auto const format{Array.srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8};
	bind_for_upload(texture);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT);
	set_anisotropy(GL_TEXTURE_2D_ARRAY);
	::glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, format, Array.size, Array.size, Layers, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	texture.data_width = Array.size;
	texture.data_height = Array.size;
	texture.layers = Layers;
	texture.data_format = format;
	texture.data_components = GL_RGBA;
}

void opengl46_terrain_materials::gpu_blank(slot const &Slot)
{
	std::uint8_t const blank[4]{0, 0, 0, 0};
	for (auto const handle : Slot.weights)
	{
		bind_for_upload(m_gltextures.texture(handle));
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, blank);
		::glGenerateMipmap(GL_TEXTURE_2D);
	}
	{
		bind_for_upload(m_gltextures.texture(Slot.table));
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		std::array<float, max_layers * 2 * 4> const zeros{};
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, static_cast<GLsizei>(max_layers), 2, 0, GL_RGBA, GL_FLOAT, zeros.data());
	}
	{
		// led between the samples, so the edges of the holes run smoothly across the cells
		bind_for_upload(m_gltextures.texture(Slot.holes));
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		std::uint8_t const none{0};
		::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, 1, 1, 0, GL_RED, GL_UNSIGNED_BYTE, &none);
		::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	}
}

void opengl46_terrain_materials::gpu_weights(texture_handle const Texture, int const Side, std::uint8_t const *Texels)
{
	bind_for_upload(m_gltextures.texture(Texture));
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, Side, Side, 0, GL_RGBA, GL_UNSIGNED_BYTE, Texels);
	::glGenerateMipmap(GL_TEXTURE_2D);
}

void opengl46_terrain_materials::gpu_table(texture_handle const Texture, float const *Table)
{
	bind_for_upload(m_gltextures.texture(Texture));
	::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, static_cast<GLsizei>(max_layers), 2, 0, GL_RGBA, GL_FLOAT, Table);
}

void opengl46_terrain_materials::gpu_holes(texture_handle const Texture, int const Side, std::uint8_t const *Samples)
{
	bind_for_upload(m_gltextures.texture(Texture));
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	::glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, Side, Side, 0, GL_RED, GL_UNSIGNED_BYTE, Samples);
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
}

void opengl46_terrain_materials::gpu_changed()
{
	opengl_texture::reset_unit_cache();
}

void opengl46_terrain_materials::gpu_begin_copies()
{
	m_saved.blend = ::glIsEnabled(GL_BLEND);
	m_saved.scissor = ::glIsEnabled(GL_SCISSOR_TEST);
	m_saved.cull = ::glIsEnabled(GL_CULL_FACE);
	m_saved.depth = ::glIsEnabled(GL_DEPTH_TEST);
	m_saved.srgb = ::glIsEnabled(GL_FRAMEBUFFER_SRGB);
	::glDisable(GL_BLEND);
	::glDisable(GL_SCISSOR_TEST);
	::glDisable(GL_CULL_FACE);
}

void opengl46_terrain_materials::gpu_copy(texture_array const &Array, int const Layer, texture_source *Source, std::array<float, 4> const &Default)
{
	auto &target{m_gltextures.texture(Array.handle)};
	m_framebuffer->attach(target, GL_COLOR_ATTACHMENT0, Layer);
	m_framebuffer->setup_drawing(1);
	::glViewport(0, 0, Array.size, Array.size);
	if (Source == nullptr)
	{
		::glClearBufferfv(GL_COLOR, 0, Default.data());
		return;
	}
	if (Array.srgb)
		::glEnable(GL_FRAMEBUFFER_SRGB);
	else
		::glDisable(GL_FRAMEBUFFER_SRGB);
	m_copy->apply(static_cast<opengl_texture &>(*Source), m_framebuffer.get());
}

void opengl46_terrain_materials::gpu_end_copies(bool const Copied)
{
	if (Copied)
	{
		for (auto const &array : arrays())
		{
			bind_for_upload(m_gltextures.texture(array.handle));
			::glGenerateMipmap(GL_TEXTURE_2D_ARRAY);
		}
	}

	// leave the state as the rest of the renderer knows it
	gl::framebuffer::unbind();
	if (m_saved.blend)
		::glEnable(GL_BLEND);
	if (m_saved.scissor)
		::glEnable(GL_SCISSOR_TEST);
	if (m_saved.cull)
		::glEnable(GL_CULL_FACE);
	if (m_saved.depth)
		::glEnable(GL_DEPTH_TEST);
	else
		::glDisable(GL_DEPTH_TEST);
	if (m_saved.srgb)
		::glEnable(GL_FRAMEBUFFER_SRGB);
	else
		::glDisable(GL_FRAMEBUFFER_SRGB);
	::glDepthMask(GL_TRUE);
	opengl_texture::reset_unit_cache();
	gfx::opengl46_vaogeometrybank::reset();
}
