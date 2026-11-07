/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/terrainmaterials.h"

#include "rendering/common/shadersource.h"
#include "utilities/Logs.h"

namespace gfx
{

namespace
{

int constexpr initial_capacity{8};
int constexpr copy_attempts{600}; // frames to wait for the textures of a layer to load before it's left with what it has
float constexpr default_repeat{8.f};

// array textures: diffuse, normal maps, specular/gloss maps
std::array<int, 3> constexpr array_sizes{1024, 512, 256};

} // namespace

bool terrain_materials::init()
{
	if (m_state != 0)
		return m_state > 0;
	m_state = -1;
	if (false == gpu_init())
		return false;

	m_white = make_texture("internal_src:terrain/white", texture_kind::white);
	gpu_white(m_textures.texture(m_white));
	char const *names[]{"internal_src:terrain/layers_diffuse", "internal_src:terrain/layers_normal", "internal_src:terrain/layers_specgloss"};
	for (std::size_t i = 0; i < m_arrays.size(); ++i)
	{
		auto &array{m_arrays[i]};
		array.handle = make_texture(names[i], texture_kind::layers);
		array.size = array_sizes[i];
		array.srgb = (i == 0);
		gpu_storage(array, initial_capacity, false);
	}
	m_capacity = initial_capacity;
	gpu_changed();

	// the shader is checked now, so a broken one makes the terrain fall back on plain materials instead of the failure indicator
	try
	{
		m_program = GfxRenderer->Fetch_Shader("terrain");
	}
	catch (gfx::shader_exception const &e)
	{
		ErrorLog("heightmap terrain: can't use the terrain shader: " + std::string(e.what()));
		return false;
	}
	m_state = 1;
	return true;
}

texture_handle terrain_materials::make_texture(std::string const &Name, texture_kind const Kind)
{
	auto const handle{m_textures.create(Name, false, GL_RGBA)};
	auto &texture{m_textures.texture(handle)};
	gpu_texture(texture, Kind);
	// the content is supplied here, the texture manager isn't to load, release or replace it
	texture.is_ready = true;
	texture.is_static = true;
	texture.is_rendertarget = true;
	texture.is_texstub = false;
	texture.data_state = resource_state::good;
	return handle;
}

void terrain_materials::read_material(layer &Layer)
{
	auto const &material{m_materials.material(Layer.material)};
	Layer.repeat = (Layer.size > 0.f ? Layer.size : (material.size.x > 0.f ? material.size.x : default_repeat));
	Layer.normalmap = (material.shader && material.shader->texture_conf.count("normalmap") > 0);
	Layer.specgloss = (material.shader && material.shader->texture_conf.count("specgloss") > 0);
	Layer.reflection = (std::isfinite(material.params[1].z) ? material.params[1].z : 0.f);
}

int terrain_materials::layer_index(gfx::terrain_layer const &Layer)
{
	for (auto const &existing : m_layers)
		if (existing.material == Layer.material && existing.size == Layer.size)
			return existing.index;

	layer entry;
	entry.material = Layer.material;
	entry.size = Layer.size;
	entry.index = static_cast<int>(m_layers.size());
	read_material(entry);
	m_layers.push_back(entry);
	m_pending = true;

	if (static_cast<int>(m_layers.size()) > m_capacity)
	{
		// more room: the arrays are made anew, and every layer copied again
		auto const capacity{m_capacity * 2};
		for (auto &array : m_arrays)
			gpu_storage(array, capacity, true);
		m_capacity = capacity;
		for (auto &existing : m_layers)
		{
			existing.copied = false;
			existing.attempts = 0;
		}
		gpu_changed();
	}
	return entry.index;
}

std::size_t terrain_materials::acquire_slot(material_handle const Reuse)
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
	entry.weights[0] = make_texture("internal_src:terrain/weights0_" + suffix, texture_kind::weights);
	entry.weights[1] = make_texture("internal_src:terrain/weights1_" + suffix, texture_kind::weights);
	entry.table = make_texture("internal_src:terrain/table_" + suffix, texture_kind::table);
	entry.holes = make_texture("internal_src:terrain/holes_" + suffix, texture_kind::holes);
	// the textures get their content before the material is made of them, which checks they're there
	gpu_blank(entry);
	gpu_changed();
	auto const name = [&](texture_handle const Handle) { return "internal_src:" + m_textures.texture(Handle).name; };
	std::string const definition{
	    "shader: terrain\n"
	    "texture1: " + name(m_white) + "\n"
	    "texture2: " + name(entry.weights[0]) + "\n"
	    "texture3: " + name(entry.weights[1]) + "\n"
	    "texture4: " + name(m_arrays[0].handle) + "\n"
	    "texture5: " + name(m_arrays[1].handle) + "\n"
	    "texture6: " + name(m_arrays[2].handle) + "\n"
	    "texture7: " + name(entry.table) + "\n"
	    "texture8: " + name(entry.holes) + "\n"};
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

material_handle terrain_materials::make(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement,
                                        int const HoleSamples, std::uint8_t const *Holes)
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
		gpu_weights(entry.weights[texture], side, texels.data());
	}
	entry.count = count;
	entry.layers = indices;
	for (std::size_t i = 0; i < count; ++i)
		entry.rotations[i] = Layers[i].rotation;
	write_table(entry);

	// the holes: the cells cut out entirely are left out of the mesh, the program of the holes cuts the rest of the openings
	auto const cut{Holes != nullptr && HoleSamples > 0};
	{
		auto const holeside{cut ? HoleSamples + 1 : 1};
		std::uint8_t const none{0};
		gpu_holes(entry.holes, holeside, cut ? Holes : &none);
		gpu_changed();
	}
	auto &material{m_materials.material(entry.material)};
	material.params[2] = glm::vec4(Placement.x, Placement.y, 1.f / std::max(1.f, Placement.z), static_cast<float>(side - 1));
	if (m_holesprogram == nullptr && cut)
	{
		try
		{
			m_holesprogram = GfxRenderer->Fetch_Shader("terrain_holes");
		}
		catch (gfx::shader_exception const &e)
		{
			ErrorLog("heightmap terrain: can't use the shader of the holes: " + std::string(e.what()));
		}
	}
	// the program of the holes discards fragments, which the depth prepass of the plain one never does
	material.shader = (cut && m_holesprogram != nullptr) ? m_holesprogram : m_program;
	return entry.material;
}

void terrain_materials::write_table(slot const &Slot)
{
	// what the layers are: index in the arrays, repeat, which maps they have, reflection
	std::array<float, max_layers * 2 * 4> table{};
	for (std::size_t i = 0; i < Slot.count; ++i)
	{
		auto const &layer{m_layers[Slot.layers[i]]};
		table[i * 4 + 0] = static_cast<float>(layer.index);
		table[i * 4 + 1] = 1.f / std::max(0.01f, layer.repeat);
		table[i * 4 + 2] = layer.normalmap ? 1.f : 0.f;
		table[i * 4 + 3] = layer.specgloss ? 1.f : 0.f;
		table[(max_layers + i) * 4 + 0] = layer.reflection;
		// the textures turned by the layer of the chunk (the same material can be turned differently in another layer)
		auto const angle{glm::radians(static_cast<double>(Slot.rotations[i]))};
		table[(max_layers + i) * 4 + 1] = static_cast<float>(std::cos(angle));
		table[(max_layers + i) * 4 + 2] = static_cast<float>(std::sin(angle));
	}
	gpu_table(Slot.table, table.data());
	gpu_changed();
}

void terrain_materials::on_environment_change()
{
	if (m_state <= 0)
		return;
	bool changed{false};
	for (auto &layer : m_layers)
	{
		auto const &material{m_materials.material(layer.material)};
		if (false == (material.update_on_season_change || material.update_on_weather_change))
			continue;
		// the reloaded material can have other textures, other maps and another size
		read_material(layer);
		layer.copied = false;
		layer.attempts = 0;
		changed = true;
	}
	if (false == changed)
		return;
	m_pending = true;
	// the tables of the chunks hold what the layers have, freed slots get theirs anew when they're used again
	for (auto const &entry : m_slots)
		if (entry.used)
			write_table(entry);
}

void terrain_materials::release(material_handle const Material)
{
	auto const lookup{m_slotof.find(Material)};
	if (lookup == m_slotof.end() || false == m_slots[lookup->second].used)
		return;
	m_slots[lookup->second].used = false;
	m_free.push_back(lookup->second);
}

bool terrain_materials::copy_layer(layer &Layer)
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
	std::array<texture_source *, 3> sources{};
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
		gpu_copy(m_arrays[i], Layer.index, sources[i], defaults[i]);
	Layer.copied = true;
	return true;
}

void terrain_materials::update()
{
	if (false == m_pending || m_state <= 0)
		return;

	gpu_begin_copies();
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
	gpu_end_copies(copied);
}

} // namespace gfx
