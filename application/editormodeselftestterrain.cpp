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
		if (words.str().find(" off") != std::string::npos)
		{
			m_orthophoto.enabled(false);
			WriteLog("SELFTEST orthophoto off");
			return selftest_result::handled;
		}
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
	if (command == "nmtfit")
	{
		// the heights of the terrain model fitted to the tracks, as the button of the terrain panel does
		nmt_fit();
		WriteLog("SELFTEST nmtfit: " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "nmtmake")
	{
		// chunks from the terrain model along the tracks (with the band in metres) or around the camera
		std::string where;
		double band{0.0};
		words >> where >> band;
		if (band > 0.0)
			m_nmt_state.band = band;
		nmt_make(where == "around" ? nmt_keys_around() : nmt_keys_along_tracks());
		WriteLog("SELFTEST nmtmake " + where + ": " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "nmtunder")
	{
		// the terrain under the tracks, the tracks left as they are, as the button of the terrain panel does
		nmt_under_start();
		WriteLog("SELFTEST nmtunder: " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "nmtpending")
	{
		// the chunks still waiting for the model: their place and the squares of the model they wait for
		WriteLog(format("SELFTEST nmtpending: %zu chunks, %zu made, model waiting %zu, fitting %d, stage %d, under %d, status %s", m_nmt_state.pending.size(), m_nmt_state.made, m_nmt.waiting(), m_nmt_state.fitting ? 1 : 0, static_cast<int>(m_nmt_state.stage), m_nmt_state.under ? 1 : 0, m_nmt_state.status.c_str()));
		for (auto const &key : m_nmt_state.pending)
		{
			auto const low{heightmap::chunk_corner(key)};
			auto const high{low + glm::dvec2{heightmap::chunk_size}};
			double height{0.0};
			auto const known{m_nmt.height(0.5 * (low.x + high.x), 0.5 * (low.y + high.y), height)};
			WriteLog(format("SELFTEST nmtpending chunk %d,%d at %.0f..%.0f %.0f..%.0f: exists %d, model in %d, height in the middle %d %.2f", key.first, key.second, low.x, high.x, low.y, high.y, m_streamer.exists(key) ? 1 : 0, m_nmt.request(low - glm::dvec2{2.0}, high + glm::dvec2{2.0}) ? 1 : 0, known ? 1 : 0, height));
		}
		return selftest_result::handled;
	}
	if (command == "nmtcheck")
	{
		// the ground under the axis of the tracks against their formation (the foot of the ballast), every Step m: over the top of the
		// ballast it sticks out of the track, far below it the track floats; and the places with no terrain at all
		double step{10.0};
		int top{20};
		words >> step >> top;
		struct finding
		{
			double off;
			glm::dvec3 at;
			std::string track;
		};
		std::vector<finding> over, under;
		std::size_t places{0}, none{0}, bridges{0};
		std::vector<double> offs;
		for (auto const &place : nmt_track_places(step))
		{
			if (false == place.shapes)
			{
				++bridges;
				continue;
			}
			++places;
			double height{0.0};
			if (false == m_streamer.height_at(place.position.x, place.position.z, height))
			{
				++none;
				continue;
			}
			auto const off{height - place.formation};
			offs.push_back(off);
			auto const name{place.track->name().empty() ? std::string{"(noname)"} : place.track->name()};
			if (height > place.crown + 0.02)
				over.push_back({height - place.crown, place.position, name});
			else if (off < -0.3)
				under.push_back({off, place.position, name});
		}
		std::sort(offs.begin(), offs.end());
		auto const quantile = [&](double const Q) { return offs.empty() ? 0.0 : offs[static_cast<std::size_t>(Q * (offs.size() - 1))]; };
		WriteLog(format("SELFTEST nmtcheck: %zu places, %zu without terrain, %zu on bridges or in tunnels; ground less formation: 5%% %.2f, median %.2f, 95%% %.2f m; over the top of the ballast %zu, more than 0.3 m under the formation %zu; modifiers %zu",
		                places, none, bridges, quantile(0.05), quantile(0.5), quantile(0.95), over.size(), under.size(), m_streamer.manifest().modifiers.size()));
		for (auto *list : {&over, &under})
		{
			std::sort(list->begin(), list->end(), [](finding const &A, finding const &B) { return std::abs(A.off) > std::abs(B.off); });
			for (std::size_t i = 0; i < std::min<std::size_t>(list->size(), static_cast<std::size_t>(top)); ++i)
				WriteLog(format("SELFTEST nmtcheck %s %+.2f m at %.1f %.1f %.1f %s", list == &over ? "over the crown" : "under the formation", (*list)[i].off, (*list)[i].at.x, (*list)[i].at.y, (*list)[i].at.z, (*list)[i].track.c_str()));
		}
		return selftest_result::handled;
	}
	if (command == "nmtauto")
	{
		// the tracks put onto the terrain model, as the button of the terrain panel does; nmtwait waits for the end
		nmt_auto_start();
		WriteLog("SELFTEST nmtauto: " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "nmtprobe")
	{
		// heights across the tracks: the rail top, the top of the ballast under the rails, the model, the ground under the axis
		// and beside the track
		std::string name;
		double step{250.0};
		words >> name >> step;
		auto const places{nmt_track_places(step)};
		for (std::size_t i = 0; i + 1 < places.size(); ++i)
		{
			auto const &place{places[i]};
			auto const &next{places[i + 1]};
			if (place.track != next.track || place.path != next.path || (false == name.empty() && place.track->name() != name))
				continue;
			glm::dvec2 along{next.position.x - place.position.x, next.position.z - place.position.z};
			if (glm::length(along) < 1e-6)
				continue;
			along = glm::normalize(along);
			glm::dvec2 const across{-along.y, along.x};
			std::vector<glm::dvec3> points;
			for (double const offset : {0.0, -2.0, 2.0, -3.5, 3.5, -6.0, 6.0})
				points.emplace_back(place.position.x + across.x * offset, place.position.y, place.position.z + across.y * offset);
			std::vector<char> found;
			auto const ground{ground_heights(points, true, &found)};
			double model{std::numeric_limits<double>::quiet_NaN()};
			if (double height; m_nmt.height(place.position.x, place.position.z, height))
				model = height + m_nmt_state.offset;
			std::string text{format("SELFTEST nmtprobe %s at %.0f %.0f: rail top %.2f, ballast top %.2f, foot %.2f, model %.2f; ground/model at axis, -2, 2, -3.5, 3.5, -6, 6 m:", place.track->name().c_str(), place.position.x, place.position.z, place.position.y, place.crown, place.formation, model)};
			for (std::size_t k = 0; k < points.size(); ++k)
			{
				double side{std::numeric_limits<double>::quiet_NaN()};
				if (double height; m_nmt.height(points[k].x, points[k].z, height))
					side = height + m_nmt_state.offset;
				text += (found[k] != 0 ? format(" %.2f", ground[k]) : std::string{" -"}) + format("/%.2f", side);
			}
			WriteLog(text);
		}
		return selftest_result::handled;
	}
	if (command == "nmtoffer")
	{
		// the offer of the terrain to a scenery begun from nothing: logged, and taken with the place given, if any
		double north{0.0}, east{0.0};
		words >> north >> east;
		WriteLog(format("SELFTEST nmtoffer: shown %d", m_nmt_state.prompt ? 1 : 0));
		if (north > 0.0 && east > 0.0 && m_nmt_state.prompt)
		{
			m_nmt_state.north = north;
			m_nmt_state.east = east;
			nmt_accept_offer();
			WriteLog("SELFTEST nmtoffer taken: " + m_nmt_state.status);
		}
		return selftest_result::handled;
	}
	if (command == "nmtwait")
	{
		// till the model is in and the chunks waiting for it are made
		if (m_nmt_state.fitting || m_nmt_state.stage != nmt_state::automatic::idle || false == m_nmt_state.pending.empty())
		{
			--m_selftest.next;
			return selftest_result::pause;
		}
		WriteLog("SELFTEST nmtwait: " + m_nmt_state.status + (m_nmt_state.error.empty() ? std::string{} : " (" + m_nmt_state.error + ")"));
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
