/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl33terrain.h"

#include "rendering/opengl33geometrybank.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

namespace
{

std::size_t constexpr max_layers{8};
int constexpr initial_capacity{8};
int constexpr copy_attempts{600}; // frames to wait for the textures of a layer to load before it's left with what it has
float constexpr default_repeat{8.f};

// array textures: diffuse, normal maps, specular/gloss maps
std::array<int, 3> constexpr array_sizes{1024, 512, 256};

void bind_for_upload(opengl_texture const &Texture)
{
	::glActiveTexture(GL_TEXTURE0);
	::glBindTexture(Texture.target, Texture.id);
}

void set_anisotropy(GLenum const Target)
{
	if (Global.AnisotropicFiltering >= 0 && (GLAD_GL_EXT_texture_filter_anisotropic || GLAD_GL_ARB_texture_filter_anisotropic))
		::glTexParameterf(Target, GL_TEXTURE_MAX_ANISOTROPY_EXT, Global.AnisotropicFiltering);
}

} // namespace

bool opengl33_terrain_materials::init()
{
	if (m_state != 0)
		return m_state > 0;
	m_state = -1;
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

	m_white = make_texture("internal_src:terrain/white", GL_TEXTURE_2D);
	{
		auto const &white{m_textures.texture(m_white)};
		std::uint8_t const texel[4]{255, 255, 255, 255};
		bind_for_upload(white);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, texel);
	}
	char const *names[]{"internal_src:terrain/layers_diffuse", "internal_src:terrain/layers_normal", "internal_src:terrain/layers_specgloss"};
	for (std::size_t i = 0; i < m_arrays.size(); ++i)
	{
		auto &array{m_arrays[i]};
		array.handle = make_texture(names[i], GL_TEXTURE_2D_ARRAY);
		array.size = array_sizes[i];
		array.srgb = (i == 0);
		array.format = (array.srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8);
		make_storage(array, initial_capacity);
	}
	m_capacity = initial_capacity;
	opengl_texture::reset_unit_cache();

	// the shader is checked now, so a broken one makes the terrain fall back on plain materials instead of the failure indicator
	try
	{
		GfxRenderer->Fetch_Shader("terrain");
	}
	catch (gl::shader_exception const &e)
	{
		ErrorLog("heightmap terrain: can't use the terrain shader: " + std::string(e.what()));
		return false;
	}
	m_state = 1;
	return true;
}

texture_handle opengl33_terrain_materials::make_texture(std::string const &Name, GLenum const Target)
{
	auto const handle{m_textures.create(Name, false, GL_RGBA)};
	auto &texture{m_textures.texture(handle)};
	if (texture.id == opengl_texture::invalid_id)
		::glGenTextures(1, &texture.id);
	texture.target = Target;
	// the content is supplied here, the texture manager isn't to load, release or replace it
	texture.is_ready = true;
	texture.is_static = true;
	texture.is_rendertarget = true;
	texture.is_texstub = false;
	texture.data_state = resource_state::good;
	return handle;
}

void opengl33_terrain_materials::make_storage(texture_array &Array, int const Layers)
{
	auto &texture{m_textures.texture(Array.handle)};
	bind_for_upload(texture);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT);
	::glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT);
	set_anisotropy(GL_TEXTURE_2D_ARRAY);
	::glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, Array.format, Array.size, Array.size, Layers, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	texture.data_width = Array.size;
	texture.data_height = Array.size;
	texture.layers = Layers;
	texture.data_format = Array.format;
	texture.data_components = GL_RGBA;
}

int opengl33_terrain_materials::layer_index(gfx::terrain_layer const &Layer)
{
	for (auto const &existing : m_layers)
		if (existing.material == Layer.material && existing.size == Layer.size)
			return existing.index;

	layer entry;
	entry.material = Layer.material;
	entry.size = Layer.size;
	entry.index = static_cast<int>(m_layers.size());
	auto const &material{m_materials.material(Layer.material)};
	entry.repeat = (Layer.size > 0.f ? Layer.size : (material.size.x > 0.f ? material.size.x : default_repeat));
	if (material.shader)
	{
		entry.normalmap = material.shader->texture_conf.count("normalmap") > 0;
		entry.specgloss = material.shader->texture_conf.count("specgloss") > 0;
	}
	entry.reflection = (std::isfinite(material.params[1].z) ? material.params[1].z : 0.f);
	m_layers.push_back(entry);
	m_pending = true;

	if (static_cast<int>(m_layers.size()) > m_capacity)
	{
		// more room: the arrays are made anew, and every layer copied again
		auto const capacity{m_capacity * 2};
		for (auto &array : m_arrays)
		{
			auto &texture{m_textures.texture(array.handle)};
			::glDeleteTextures(1, &texture.id);
			::glGenTextures(1, &texture.id);
			make_storage(array, capacity);
		}
		m_capacity = capacity;
		for (auto &existing : m_layers)
		{
			existing.copied = false;
			existing.attempts = 0;
		}
		opengl_texture::reset_unit_cache();
	}
	return entry.index;
}

std::size_t opengl33_terrain_materials::acquire_slot(material_handle const Reuse)
{
	if (Reuse != null_handle)
	{
		auto const lookup{m_slotof.find(Reuse)};
		if (lookup != m_slotof.end())
		{
			m_slots[lookup->second].used = true;
			return lookup->second;
		}
	}
	if (false == m_free.empty())
	{
		auto const index{m_free.back()};
		m_free.pop_back();
		m_slots[index].used = true;
		return index;
	}
	auto const index{m_slots.size()};
	auto const suffix{std::to_string(index)};
	slot entry;
	entry.weights[0] = make_texture("internal_src:terrain/weights0_" + suffix, GL_TEXTURE_2D);
	entry.weights[1] = make_texture("internal_src:terrain/weights1_" + suffix, GL_TEXTURE_2D);
	entry.table = make_texture("internal_src:terrain/table_" + suffix, GL_TEXTURE_2D);
	// the textures get their content before the material is made of them, which checks they're there
	std::uint8_t const blank[4]{0, 0, 0, 0};
	for (auto const handle : entry.weights)
	{
		bind_for_upload(m_textures.texture(handle));
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, blank);
		::glGenerateMipmap(GL_TEXTURE_2D);
	}
	{
		bind_for_upload(m_textures.texture(entry.table));
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		std::array<float, max_layers * 2 * 4> const zeros{};
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, static_cast<GLsizei>(max_layers), 2, 0, GL_RGBA, GL_FLOAT, zeros.data());
	}
	opengl_texture::reset_unit_cache();
	auto const name = [&](texture_handle const Handle) { return "internal_src:" + m_textures.texture(Handle).name; };
	std::string const definition{
	    "shader: terrain\n"
	    "texture1: " + name(m_white) + "\n"
	    "texture2: " + name(entry.weights[0]) + "\n"
	    "texture3: " + name(entry.weights[1]) + "\n"
	    "texture4: " + name(m_arrays[0].handle) + "\n"
	    "texture5: " + name(m_arrays[1].handle) + "\n"
	    "texture6: " + name(m_arrays[2].handle) + "\n"
	    "texture7: " + name(entry.table) + "\n"};
	entry.material = m_materials.create_from_text("internal_src:terrain/chunk_" + suffix, definition);
	if (entry.material == null_handle || false == m_materials.material(entry.material).is_good)
	{
		ErrorLog("heightmap terrain: can't make the material of a chunk");
		m_state = -1;
		return static_cast<std::size_t>(-1);
	}
	entry.used = true;
	m_slots.push_back(entry);
	m_slotof.emplace(entry.material, index);
	return index;
}

material_handle opengl33_terrain_materials::make(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement)
{
	if (Layers.empty() || false == init())
		return null_handle;
	auto const count{std::min(Layers.size(), max_layers)};
	std::array<int, max_layers> indices{};
	for (std::size_t i = 0; i < count; ++i)
	{
		if (Layers[i].material == null_handle)
			return null_handle;
		indices[i] = layer_index(Layers[i]);
	}
	auto const slotindex{acquire_slot(Reuse)};
	if (slotindex == static_cast<std::size_t>(-1))
		return null_handle;
	auto &entry{m_slots[slotindex]};

	// weights, four layers to a texture
	auto const painted{Weights != nullptr && Samples > 0 && count > 1};
	auto const side{painted ? Samples + 1 : 1};
	auto const samples{static_cast<std::size_t>(side) * side};
	std::vector<std::uint8_t> texels(samples * 4);
	for (std::size_t texture = 0; texture < 2; ++texture)
	{
		std::fill(texels.begin(), texels.end(), 0);
		for (std::size_t channel = 0; channel < 4; ++channel)
		{
			auto const layer{texture * 4 + channel};
			if (layer >= count)
				break;
			if (false == painted)
			{
				if (layer == 0)
					texels[channel] = 255;
				continue;
			}
			auto const *source{Weights + layer * samples};
			for (std::size_t i = 0; i < samples; ++i)
				texels[i * 4 + channel] = source[i];
		}
		bind_for_upload(m_textures.texture(entry.weights[texture]));
		::glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, side, side, 0, GL_RGBA, GL_UNSIGNED_BYTE, texels.data());
		::glGenerateMipmap(GL_TEXTURE_2D);
	}
	// what the layers are: index in the arrays, repeat, which maps they have, reflection
	std::array<float, max_layers * 2 * 4> table{};
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &layer{m_layers[indices[i]]};
		table[i * 4 + 0] = static_cast<float>(layer.index);
		table[i * 4 + 1] = 1.f / std::max(0.01f, layer.repeat);
		table[i * 4 + 2] = layer.normalmap ? 1.f : 0.f;
		table[i * 4 + 3] = layer.specgloss ? 1.f : 0.f;
		table[(max_layers + i) * 4 + 0] = layer.reflection;
		// the textures turned by the layer of the chunk (the same material can be turned differently in another layer)
		auto const angle{glm::radians(static_cast<double>(Layers[i].rotation))};
		table[(max_layers + i) * 4 + 1] = static_cast<float>(std::cos(angle));
		table[(max_layers + i) * 4 + 2] = static_cast<float>(std::sin(angle));
	}
	bind_for_upload(m_textures.texture(entry.table));
	::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, static_cast<GLsizei>(max_layers), 2, 0, GL_RGBA, GL_FLOAT, table.data());
	opengl_texture::reset_unit_cache();

	auto &material{m_materials.material(entry.material)};
	material.params[2] = glm::vec4(Placement.x, Placement.y, 1.f / std::max(1.f, Placement.z), static_cast<float>(side - 1));
	return entry.material;
}

void opengl33_terrain_materials::release(material_handle const Material)
{
	auto const lookup{m_slotof.find(Material)};
	if (lookup == m_slotof.end() || false == m_slots[lookup->second].used)
		return;
	m_slots[lookup->second].used = false;
	m_free.push_back(lookup->second);
}

bool opengl33_terrain_materials::copy_layer(layer &Layer)
{
	auto const &material{m_materials.material(Layer.material)};
	auto const slot = [&](char const *Name, int const Fallback) {
		if (material.shader)
		{
			auto const lookup{material.shader->texture_conf.find(Name)};
			if (lookup != material.shader->texture_conf.end())
				return static_cast<int>(lookup->second.id);
			return -1;
		}
		return Fallback;
	};
	std::array<int, 3> const slots{slot("diffuse", 0), Layer.normalmap ? slot("normalmap", 1) : -1, Layer.specgloss ? slot("specgloss", 2) : -1};
	// all maps of the layer have to be ready, so they go together
	std::array<opengl_texture *, 3> sources{};
	for (std::size_t i = 0; i < slots.size(); ++i)
	{
		if (slots[i] < 0 || material.textures[slots[i]] == null_handle)
			continue;
		auto &texture{m_textures.mark_as_used(material.textures[slots[i]])};
		if (false == texture.is_ready)
		{
			// a texture which is on the card already may have dropped its data after the upload (create() would refuse it then),
			// one released by the sweep or never read has to be read first
			if (texture.data_state == resource_state::none)
			{
				texture.reload_on_use = false;
				texture.load();
			}
			if (texture.data_state == resource_state::failed)
				continue;
			if (false == texture.create())
			{
				if (++Layer.attempts < copy_attempts)
					return false;
				WriteLog("heightmap terrain: texture \"" + texture.name + "\" isn't ready, the terrain goes without it");
				continue; // waited long enough, the layer goes without it
			}
		}
		sources[i] = &texture;
	}
	// defaults for maps the layer doesn't have: grey, flat, without specular reflections
	std::array<std::array<float, 4>, 3> const defaults{{{0.5f, 0.5f, 0.5f, 1.f}, {0.5f, 0.5f, 1.f, 1.f}, {0.f, 1.f, 0.f, 1.f}}};
	for (std::size_t i = 0; i < m_arrays.size(); ++i)
	{
		auto const &array{m_arrays[i]};
		auto &target{m_textures.texture(array.handle)};
		m_framebuffer->attach(target, GL_COLOR_ATTACHMENT0, Layer.index);
		m_framebuffer->setup_drawing(1);
		::glViewport(0, 0, array.size, array.size);
		if (sources[i] == nullptr)
		{
			::glClearBufferfv(GL_COLOR, 0, defaults[i].data());
			continue;
		}
		if (false == Global.gfx_usegles)
		{
			if (array.srgb)
				::glEnable(GL_FRAMEBUFFER_SRGB);
			else
				::glDisable(GL_FRAMEBUFFER_SRGB);
		}
		m_copy->apply(*sources[i], m_framebuffer.get());
	}
	Layer.copied = true;
	return true;
}

void opengl33_terrain_materials::update()
{
	if (false == m_pending || m_state <= 0)
		return;

	auto const blend{::glIsEnabled(GL_BLEND)};
	auto const scissor{::glIsEnabled(GL_SCISSOR_TEST)};
	auto const cull{::glIsEnabled(GL_CULL_FACE)};
	auto const depth{::glIsEnabled(GL_DEPTH_TEST)};
	auto const srgb{(false == Global.gfx_usegles) && ::glIsEnabled(GL_FRAMEBUFFER_SRGB)};
	::glDisable(GL_BLEND);
	::glDisable(GL_SCISSOR_TEST);
	::glDisable(GL_CULL_FACE);

	bool copied{false};
	m_pending = false;
	for (auto &layer : m_layers)
	{
		if (layer.copied)
			continue;
		if (copy_layer(layer))
			copied = true;
		else
			m_pending = true;
	}
	if (copied)
	{
		for (auto const &array : m_arrays)
		{
			bind_for_upload(m_textures.texture(array.handle));
			::glGenerateMipmap(GL_TEXTURE_2D_ARRAY);
		}
	}

	// leave the state as the rest of the renderer knows it
	gl::framebuffer::unbind();
	if (blend)
		::glEnable(GL_BLEND);
	if (scissor)
		::glEnable(GL_SCISSOR_TEST);
	if (cull)
		::glEnable(GL_CULL_FACE);
	if (depth)
		::glEnable(GL_DEPTH_TEST);
	else
		::glDisable(GL_DEPTH_TEST);
	if (false == Global.gfx_usegles)
	{
		if (srgb)
			::glEnable(GL_FRAMEBUFFER_SRGB);
		else
			::glDisable(GL_FRAMEBUFFER_SRGB);
	}
	::glDepthMask(GL_TRUE);
	opengl_texture::reset_unit_cache();
	gfx::opengl33_vaogeometrybank::reset();
}
