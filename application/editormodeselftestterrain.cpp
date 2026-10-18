/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "editor/editorSettings.hpp"
#include "editor/editorGeometry.hpp"
#include "application/application.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"

#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "scene/scenelayers.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "world/MemCell.h"
#include "world/Event.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "world/Sweep.h"
#include "input/keyboardinput.h"
#include "launcher/textures_scanner.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "scene/sceneterrain.h"
#include "editor/editorSpeed.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include "application/editormodeselftest.h"

editor_mode::selftest_result editor_mode::selftest_terrain(std::string const &command, std::istringstream &words)
{
	if (command == "orthophoto")
	{
		auto config{m_orthophoto.settings()};
		int fit{1};
		int inscene{0};
		words >> config.north >> config.east >> config.radius >> fit >> inscene;
		config.drape = fit != 0;
		config.in_scene = inscene != 0;
		m_orthophoto.settings(config);
		m_orthophoto.enabled(true);
		WriteLog(format("SELFTEST orthophoto at %.0f %.0f, radius %d, %s%s", config.north, config.east, config.radius, config.drape ? "fitted" : "flat", config.in_scene ? ", in the scene" : ""));
		return selftest_result::handled;
	}
	if (command == "terrainmode")
	{
		m_terrain_open = true;
		WriteLog("SELFTEST terrainmode");
		return selftest_result::handled;
	}
	if (command == "terrainsculpt" || command == "terrainpaint" || command == "terrainlevel")
	{
		double x{0.0};
		double z{0.0};
		double radius{10.0};
		double value{1.0};
		int mode{0};
		words >> x >> z >> radius >> value >> mode;
		auto const changed{command == "terrainsculpt" ? m_streamer.sculpt(x, z, radius, value)
		                   : command == "terrainlevel" ? m_streamer.level(x, z, radius, value, 1.0, mode)
		                                               : m_streamer.paint(x, z, radius, 1.0, static_cast<std::uint16_t>(value))};
		forget_ground();
		WriteLog(format("SELFTEST %s %.1f %.1f: %s", command.c_str(), x, z, changed ? "changed" : "nothing changed"));
		return selftest_result::handled;
	}
	if (command == "terrainmodifier")
	{
		heightmap::modifier modifier;
		heightmap::corridor_point a, b;
		double formation{0.0};
		double halfwidth{3.5};
		words >> modifier.name >> a.position.x >> a.position.y >> b.position.x >> b.position.y >> formation >> halfwidth;
		b.formation = formation;
		a.formation = b.formation;
		b.half_width = halfwidth;
		a.half_width = b.half_width;
		modifier.points = {a, b};
		m_streamer.modifier(std::move(modifier));
		forget_ground();
		WriteLog(format("SELFTEST terrainmodifier: %zu modifiers", m_streamer.modifiers().size()));
		return selftest_result::handled;
	}
	if (command == "terrainfill")
	{
		int layer{0};
		words >> layer >> m_paint_feather >> m_paint_rotation;
		m_terrain_layer = layer;
		m_paint_outline.clear();
		double x;
		double z;
		while (words >> x >> z)
			m_paint_outline.emplace_back(x, 0.0, z);
		fill_terrain_area();
		std::string palette;
		for (auto const &entry : m_streamer.manifest().layers)
			palette += format(" [%s %.1f m %.0f deg]", entry.material.c_str(), entry.size, entry.rotation);
		WriteLog("SELFTEST terrainfill: " + m_terrain_status + ", palette" + palette);
		return selftest_result::handled;
	}
	if (command == "terrainturn")
	{
		int layer{0};
		float degrees{0.f};
		words >> layer >> degrees;
		m_streamer.layer_rotation(static_cast<std::uint16_t>(layer), degrees);
		WriteLog(format("SELFTEST terrainturn %d: %.1f deg", layer, m_streamer.manifest().layers.at(layer).rotation));
		return selftest_result::handled;
	}
	if (command == "terraintouchup")
	{
		int state{1};
		words >> state;
		m_streamer.touch_up(state != 0);
		WriteLog(format("SELFTEST terraintouchup %d", state));
		return selftest_result::handled;
	}
	if (command == "terrainrestore")
	{
		double x{0.0};
		double z{0.0};
		double radius{10.0};
		words >> x >> z >> radius;
		auto const changed{m_streamer.restore_stitching(x, z, radius, 1.0)};
		forget_ground();
		WriteLog(format("SELFTEST terrainrestore %.1f %.1f: %s, touch-ups left %zu", x, z, changed ? "changed" : "nothing changed", m_streamer.touched_up()));
		return selftest_result::handled;
	}
	if (command == "terrainbake")
	{
		std::string name;
		words >> name;
		auto const baked{m_streamer.bake_modifier(name)};
		forget_ground();
		WriteLog(format("SELFTEST terrainbake %s: %s, %zu modifiers left", name.c_str(), baked ? "baked" : "not found", m_streamer.modifiers().size()));
		return selftest_result::handled;
	}
	if (command == "terrainselect")
	{
		int cx{0};
		int cz{0};
		std::string add;
		words >> cx >> cz >> add;
		if (add != "add")
			m_terrain_selection.clear();
		if (m_streamer.exists({cx, cz}))
			m_terrain_selection.insert({cx, cz});
		WriteLog(format("SELFTEST terrainselect: %zu selected", m_terrain_selection.size()));
		return selftest_result::handled;
	}
	if (command == "terrainresolution")
	{
		float spacing{1.f};
		int paint{1};
		words >> spacing >> paint;
		m_terrain_convert_spacing = heightmap::valid_spacing(spacing);
		m_terrain_convert_paint = paint != 0;
		convert_selected_chunks();
		WriteLog("SELFTEST terrainresolution: " + m_terrain_status);
		return selftest_result::handled;
	}
	if (command == "terrainwater" || command == "terrainwaterdefault")
	{
		if (command == "terrainwaterdefault")
		{
			std::string material;
			words >> material;
			m_streamer.water_material(material);
		}
		else
		{
			std::size_t index{0};
			std::string material;
			float size{0.f};
			words >> index >> material >> size;
			auto water{m_streamer.water()};
			if (index < water.size())
			{
				water[index].material = material == "-" ? std::string{} : material;
				water[index].size = size;
				m_streamer.water(std::move(water));
			}
		}
		std::string list;
		for (auto const &body : m_streamer.water())
			list += format(" [%s: %s, %.1f m]", body.name.c_str(), body.material.empty() ? "default" : body.material.c_str(), body.size);
		WriteLog("SELFTEST " + command + ": default " + m_streamer.manifest().water_material + list);
		return selftest_result::handled;
	}
	if (command == "terrainadd")
	{
		int count{1};
		float spacing{m_terrain_spacing};
		words >> count >> spacing;
		m_terrain_spacing = heightmap::valid_spacing(spacing);
		add_chunks_around(std::clamp(count, 1, 8));
		WriteLog("SELFTEST terrainadd: " + m_terrain_status);
		return selftest_result::handled;
	}
	if (command == "terrainprobe")
	{
		double x{0.0};
		double z{0.0};
		double height{0.0};
		words >> x >> z;
		auto const *chunk{m_streamer.terrain_at(x, z)};
		std::string materials;
		if (chunk != nullptr)
			for (std::size_t slot = 0; slot < std::max<std::size_t>(1, chunk->data().layer_count()); ++slot)
			{
				auto const layer{chunk->data().layers[slot]};
				auto const &palette{m_streamer.manifest().layers};
				materials += (layer < palette.size() ? palette[layer].material : std::string{"?"}) + " ";
			}
		auto const found{m_streamer.height_at(x, z, height)};
		std::string grid;
		if (chunk != nullptr)
		{
			auto const &data{chunk->data()};
			double base{0.0};
			double adjust{0.0};
			if (false == data.base.empty())
				base = data.height_in(data.base, x, z);
			if (false == data.adjust.empty())
				adjust = data.height_in(data.adjust, x, z);
			grid = format(", spacing %.1f, paint %d, base %s %.3f, touch-up %.3f", data.spacing(), data.paint, data.base.empty() ? "none" : "", base, adjust);
			if (data.paint > 0)
			{
				// weights at the paint sample nearest the point
				auto const corner{data.corner()};
				auto const ix{static_cast<int>(std::lround((x - corner.x) / heightmap::chunk_size * data.paint))};
				auto const iz{static_cast<int>(std::lround((z - corner.y) / heightmap::chunk_size * data.paint))};
				grid += ", weights";
				for (std::size_t slot = 0; slot < data.layer_count(); ++slot)
					grid += format(" %d", data.weight(slot, std::clamp(ix, 0, data.paint), std::clamp(iz, 0, data.paint)));
			}
		}
		WriteLog(format("SELFTEST terrainprobe %.1f %.1f: %s %.3f, %zu chunks, materials %s%s", x, z, found ? "height" : "no terrain", height, m_streamer.resident(), materials.c_str(), grid.c_str()));
		return selftest_result::handled;
	}
	if (command == "terrainconvert")
	{
		load_ground_files();
		for (auto const &source : m_ground_conversion.files)
		{
			glm::dvec3 low{1e30};
			glm::dvec3 high{-1e30};
			for (auto index = source.first; index < source.first + source.count; ++index)
				for (auto const &vertex : m_ground_conversion.triangles[index])
				{
					low = glm::min(low, vertex);
					high = glm::max(high, vertex);
				}
			WriteLog("SELFTEST terrain file " + source.file + (source.text ? " (text): " : " (binary): ") + source.message +
			         format(", from %.1f %.1f %.1f to %.1f %.1f %.1f", low.x, low.y, low.z, high.x, high.y, high.z));
		}
		m_ground_conversion.status = convert_ground();
		WriteLog("SELFTEST terrainconvert: " + m_ground_conversion.status);
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
