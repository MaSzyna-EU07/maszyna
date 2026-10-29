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
#include <cstring>
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
			// as the user turns it off: the terrain model leaves it so
			choose_orthophoto(false);
			WriteLog("SELFTEST orthophoto off");
			return selftest_result::handled;
		}
		int fit{1};
		int inscene{0};
		float height{0.0f};
		words >> config.north >> config.east >> config.radius >> fit >> inscene;
		config.drape = fit != 0;
		config.in_scene = inscene != 0;
		if (words >> height)
			config.height = height;
		m_orthophoto.settings(config);
		m_orthophoto.enabled(true);
		WriteLog(format("SELFTEST orthophoto at %.0f %.0f, radius %d, %s%s", config.north, config.east, config.radius, config.drape ? "fitted" : "flat", config.in_scene ? ", in the scene" : ""));
		return selftest_result::handled;
	}
	if (command == "orthoarea")
	{
		// the download area through the points of the world (x, z), as drawn on the map; clear: none
		std::vector<glm::dvec2> polygon;
		if (words.str().find(" clear") == std::string::npos)
			for (glm::dvec2 point{0.0}; words >> point.x >> point.y;)
				polygon.push_back(m_orthophoto.to_puwg(point));
		m_orthophoto.area(polygon);
		WriteLog(format("SELFTEST orthoarea: %zu points, %d tiles", m_orthophoto.area().size(), m_orthophoto.stats().area_tiles));
		return selftest_result::handled;
	}
	if (command == "orthoareapause")
	{
		int paused{1};
		words >> paused;
		m_area_paused = paused != 0;
		m_orthophoto.area_paused(m_area_paused);
		m_nmt.prefetch_paused(m_area_paused);
		WriteLog(format("SELFTEST orthoareapause %d", paused));
		return selftest_result::handled;
	}
	if (command == "orthohires")
	{
		int hires{1};
		words >> hires;
		auto config{m_orthophoto.settings()};
		config.hires = hires != 0;
		m_orthophoto.settings(config);
		WriteLog(format("SELFTEST orthohires %d", hires));
		return selftest_result::handled;
	}
	if (command == "orthosettings")
	{
		// as the panel of the orthophoto changes them, kept with the editor
		auto config{m_orthophoto.settings()};
		std::string key;
		double value{0.0};
		auto changed{false};
		while (words >> key >> value)
		{
			changed = true;
			if (key == "radius")
				config.radius = std::clamp(static_cast<int>(value), 0, editor_orthophoto::max_radius);
			else if (key == "opacity")
				config.opacity = std::clamp(static_cast<float>(value), 0.0f, 1.0f);
			else if (key == "lift")
				config.lift = static_cast<float>(value);
			else if (key == "height")
				config.height = static_cast<float>(value);
			else if (key == "year")
				config.year = static_cast<int>(value);
			else if (key == "hires")
				config.hires = value != 0.0;
			else if (key == "drape")
				config.drape = value != 0.0;
			else if (key == "scene")
				config.in_scene = value != 0.0;
			else if (key == "on")
				choose_orthophoto(value != 0.0);
		}
		if (changed)
		{
			m_orthophoto.settings(config);
			save_orthophoto_settings();
		}
		config = m_orthophoto.settings();
		WriteLog(format("SELFTEST orthosettings on %d radius %d opacity %.2f lift %.2f height %.2f year %d hires %d drape %d scene %d origin %.2f %.2f", m_orthophoto.enabled() ? 1 : 0, config.radius, config.opacity, config.lift,
		                config.height, config.year, config.hires ? 1 : 0, config.drape ? 1 : 0, config.in_scene ? 1 : 0, config.north, config.east));
		return selftest_result::handled;
	}
	if (command == "nmtsettings")
	{
		// as the terrain panel changes them, kept with the editor the next frame (keep_nmt_settings); the terrain while laying at
		// once, as its chip keeps it
		auto &state{m_nmt_state};
		std::string key;
		double value{0.0};
		while (words >> key >> value)
		{
			if (key == "band")
				state.band = std::clamp(value, 50.0, 3000.0);
			else if (key == "around")
				m_terrain_add_count = std::clamp(static_cast<int>(value), 1, 8);
			else if (key == "shape")
				state.shape = value != 0.0;
			else if (key == "follow")
				state.follow = value != 0.0;
			else if (key == "offset")
				state.offset = value;
			else if (key == "fitted")
				state.fitted = value != 0.0;
			else if (key == "grow")
			{
				state.grow = value != 0.0;
				EditorSettings.terrain_while_laying(state.grow);
				EditorSettings.save();
			}
		}
		WriteLog(format("SELFTEST nmtsettings band %.0f around %d shape %d follow %d grow %d offset %+.2f fitted %d", state.band, m_terrain_add_count, state.shape ? 1 : 0, state.follow ? 1 : 0, state.grow ? 1 : 0, state.offset,
		                state.fitted ? 1 : 0));
		return selftest_result::handled;
	}
	if (command == "wmsdescribe")
	{
		// what a WMS tells of itself, as the layers window reads it before one is added
		std::string url;
		words >> url;
		auto const info{editor_orthophoto::describe(url)};
		WriteLog(format("SELFTEST wmsdescribe: \"%s\", %zu layers, EPSG:2180 %d%s", info.title.c_str(), info.layers.size(), info.puwg ? 1 : 0, info.error.empty() ? "" : (", error " + info.error).c_str()));
		WriteLog("SELFTEST wmsdescribe terms: " + info.terms);
		for (std::size_t i = 0; i < std::min<std::size_t>(info.layers.size(), 5); ++i)
			WriteLog("SELFTEST wmsdescribe layer " + info.layers[i].first + " \"" + info.layers[i].second + "\"");
		return selftest_result::handled;
	}
	if (command == "wms")
	{
		// a WMS layer of the user's on top of the others, as the layers window adds it: wms <address> <layers>
		editorSettings::wms_layer entry;
		words >> entry.url >> entry.layers;
		entry.title = entry.layers;
		EditorSettings.wms_layers().insert(EditorSettings.wms_layers().begin(), entry);
		WriteLog("SELFTEST wms " + entry.url + " " + entry.layers);
		return selftest_result::handled;
	}
	if (command == "wmsstats")
	{
		for (std::size_t i = 0; i < m_wms.size(); ++i)
		{
			auto const stats{m_wms[i]->stats()};
			WriteLog(format("SELFTEST wmsstats %zu %s: enabled %d, shown %d, loading %d, failed %d", i, m_wms[i]->settings().layers.c_str(), m_wms[i]->enabled() ? 1 : 0, stats.resident, stats.loading, stats.failed));
		}
		return selftest_result::handled;
	}
	if (command == "orthostats")
	{
		auto const stats{m_orthophoto.stats()};
		auto const terrain{m_nmt.prefetched()};
		WriteLog(format("SELFTEST orthostats: shown %d, loading %d, failed %d; area %d of %d in the cache, %d failed%s; requests %d, uploads %d, stale %d; terrain area %d of %d, %d failed",
		                stats.resident, stats.loading, stats.failed, stats.area_cached, stats.area_tiles, stats.area_failed, stats.area_paused ? ", paused" : "", stats.requests, stats.uploads, stats.stale, terrain.cached,
		                terrain.total, terrain.failed));
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
	if (command == "nmtoffset")
	{
		// the offset of the terrain model set, 0 as in an editor opened anew
		words >> m_nmt_state.offset;
		WriteLog(format("SELFTEST nmtoffset %+.2f", m_nmt_state.offset));
		return selftest_result::handled;
	}
	if (command == "nmtauto")
	{
		// the tracks put onto the terrain model, as the button of the panel of the tracks does, all or the ones not fitted yet;
		// nmtwait waits for the end
		std::string which;
		words >> which;
		nmt_auto_start(which == "unfitted");
		WriteLog("SELFTEST nmtauto " + which + ": " + m_nmt_state.status);
		return selftest_result::handled;
	}
	if (command == "turntables")
	{
		// each turntable: the height of its rails and of the model in its pit; of each track running up to it the heights out along
		// its line from the rim, the model there under the rails beside them, and how far from the rim the line keeps level
		for (auto const &table : nmt_turntables())
		{
			auto const &points{table.table->m_paths.front().points};
			double pit{std::numeric_limits<double>::quiet_NaN()};
			if (double height; m_nmt.height(table.centre.x, table.centre.z, height))
				pit = height + m_nmt_state.offset;
			std::string included;
			for (auto const &item : standing_templates([](std::string const &) { return true; }))
				if (false == item.values.empty() && ToLower(item.values.front()) == table.table->name())
					included += format(" %s at %.3f %.3f %.3f", item.file.c_str(), item.location.x, item.location.y, item.location.z);
			WriteLog(format("SELFTEST turntable %s: rails %.3f .. %.3f m, rim %.2f m, the model in the pit %.2f m, %zu tracks run up to it; its include%s", table.table->name().c_str(), points[segment_data::point::start].y,
			                points[segment_data::point::end].y, table.rim, pit, table.ends.size(), included.empty() ? " none" : included.c_str()));
			for (auto const &[track, path, atend] : table.ends)
			{
				auto route{editor_track::run_route(*track, 80.0, path)};
				auto const samples{editor_track::sample_route(route, 1.0)};
				if (samples.size() < 2)
					continue;
				auto const &rim{track->m_paths[path].points[atend ? segment_data::point::end : segment_data::point::start]};
				auto const reversed{geometry::plan_distance(samples.back().position, rim) < geometry::plan_distance(samples.front().position, rim)};
				auto const length{samples.back().chainage - samples.front().chainage};
				auto const chainage_at = [&](double const Off) { return reversed ? samples.back().chainage - Off : samples.front().chainage + Off; };
				auto const height_at = [&](double const Off) { return editor_track::sampled_elevation(samples, chainage_at(Off)); };
				auto const rails{static_cast<double>(track->rail_height())};
				std::string text;
				for (auto const off : {0.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0})
				{
					if (off > length)
						break;
					text += format(" %.0f m %.3f", off, height_at(off));
					for (auto const &sample : samples)
						if (std::abs(sample.chainage - chainage_at(off)) < 0.51)
						{
							if (double model; m_nmt.height(sample.position.x, sample.position.z, model))
								text += format(" (model %.2f)", model + m_nmt_state.offset + rails);
							break;
						}
				}
				// level off the rim within a centimetre
				double level{0.0};
				for (double off = 0.0; off <= length; off += 0.5)
					if (std::abs(height_at(off) - points[segment_data::point::start].y) <= 0.01)
						level = off;
					else
						break;
				WriteLog(format("SELFTEST turntable %s track %s: at the rim %.3f m, %+.2f per mille over its first 5 m, level within 1 cm of the table for %.1f m; out along it%s", table.table->name().c_str(), track->name().c_str(), rim.y,
				                1000.0 * (height_at(std::min(5.0, length)) - height_at(0.0)) / std::max(1e-6, std::min(5.0, length)), level, text.c_str()));
			}
		}
		return selftest_result::handled;
	}
	if (command == "fittedstate")
	{
		// the tracks whose grade line is fitted, as the panel of the tracks tells it; reload: the marks read again from the lines
		// they go to the files with
		std::string reload;
		words >> reload;
		if (reload == "reload")
		{
			m_fitted.clear();
			m_fitted_loaded = false;
		}
		std::size_t rails{0}, done{0};
		std::string names;
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && false == track->m_editorremoved && (track->iCategoryFlag & 1) != 0)
			{
				++rails;
				if (grade_fitted(*track))
				{
					++done;
					if (names.size() < 200)
						names += " " + track->name();
				}
			}
		std::size_t written{0};
		for (auto const layer : scene::Layers.marked_layers("//$f"))
			written += scene::Layers.marked(layer, "//$f").size();
		WriteLog(format("SELFTEST fittedstate: %zu of %zu tracks fitted, %zu marks kept, %zu lines for the files;", done, rails, m_fitted.size(), written) + names);
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
			m_nmt_map.pick({east, north});
			nmt_accept_offer();
			WriteLog("SELFTEST nmtoffer taken: " + m_nmt_state.status);
		}
		return selftest_result::handled;
	}
	if (command == "nmtplace")
	{
		// a place as typed into the terrain window: the PUWG 1992 and the degrees it's understood as
		std::string text;
		std::getline(words, text);
		glm::dvec2 point;
		if (false == editor_geo::parse_place(text, point))
		{
			WriteLog("SELFTEST nmtplace" + text + ": not understood");
			return selftest_result::handled;
		}
		auto const degrees{editor_geo::to_degrees(point)};
		WriteLog(format("SELFTEST nmtplace%s: X %.1f Y %.1f, %.6f N %.6f E", text.c_str(), point.y, point.x, degrees.x, degrees.y));
		return selftest_result::handled;
	}
	if (command == "nmtgrow")
	{
		// the cursor of a track laid at the place, its first point placed there unless "chosen" says the tool is only chosen: the terrain
		// grows around it once the laying begins
		auto const active{m_lay.active};
		auto const points{m_lay.points};
		std::string chosen;
		words >> m_lay.mouse.x >> m_lay.mouse.z >> chosen;
		m_lay.active = true;
		if (chosen != "chosen" && m_lay.points.empty())
			m_lay.points.push_back(m_lay.mouse);
		else if (chosen == "chosen")
			m_lay.points.clear();
		m_nmt_state.grow_time = -10.0;
		nmt_grow();
		m_lay.active = active;
		m_lay.points = points;
		WriteLog(format("SELFTEST nmtgrow: %zu chunks waiting, grown %d", m_nmt_state.pending.size(), m_nmt_state.grown ? 1 : 0));
		return selftest_result::handled;
	}
	if (command == "terrainseams")
	{
		// the steps of the ground where two chunks in memory meet
		std::vector<editor_terrain *> chunks;
		m_streamer.collect(chunks);
		std::set<heightmap::chunk_key> keys;
		for (auto const *chunk : chunks)
			keys.insert(chunk->key());
		std::size_t edges{0};
		double worst{0.0};
		glm::dvec2 where{0.0};
		for (auto const &key : keys)
			for (auto const [dx, dz] : {std::pair{1, 0}, std::pair{0, 1}})
			{
				if (false == keys.contains({key.first + dx, key.second + dz}))
					continue;
				++edges;
				auto const corner{heightmap::chunk_corner({key.first + dx, key.second + dz})};
				for (int i = 0; i <= 32; ++i)
				{
					auto const along{heightmap::chunk_size * i / 32.0};
					glm::dvec2 const point{corner.x + (dx == 0 ? along : 0.0), corner.y + (dz == 0 ? along : 0.0)};
					glm::dvec2 const inside{dx * 0.01, dz * 0.01};
					if (double a, b; m_streamer.height_at(point.x - inside.x, point.y - inside.y, a) && m_streamer.height_at(point.x + inside.x, point.y + inside.y, b) && std::abs(a - b) > worst)
					{
						worst = std::abs(a - b);
						where = point;
					}
				}
			}
		WriteLog(format("SELFTEST terrainseams: %zu chunks, %zu edges, the greatest step %.3f m at %.1f %.1f", keys.size(), edges, worst, where.x, where.y));
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
	if (command == "danglers")
	{
		// models the cells hold which aren't in the scene any more, or are about to be destroyed
		std::set<TAnimModel const *> const alive{simulation::Instances.sequence().begin(), simulation::Instances.sequence().end()};
		std::size_t held{0}, gone{0}, retired{0};
		for (auto const *section : simulation::Region->m_sections)
		{
			if (section == nullptr)
				continue;
			for (auto const &cell : section->m_cells)
				for (auto const *sequence : {&cell.m_instancesopaque, &cell.m_instancetranslucent})
					for (auto const *instance : *sequence)
					{
						++held;
						if (false == alive.contains(instance))
							++gone;
						else if (std::any_of(m_retired.begin(), m_retired.end(), [&](std::pair<TAnimModel *, int> const &Retired) { return Retired.first == instance; }))
							++retired;
					}
		}
		WriteLog(format("SELFTEST danglers: %zu held, %zu gone, %zu retired, %zu waiting", held, gone, retired, m_retired.size()));
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
	if (command == "terrainremove")
	{
		// the chunks selected taken out, as the button of the chunk panel and Del do
		remove_selected_chunks();
		WriteLog("SELFTEST terrainremove: " + m_terrain_status);
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
	if (command == "terraincut")
	{
		double x{0.0};
		double z{0.0};
		double radius{2.0};
		int cut{1};
		words >> x >> z >> radius >> cut;
		auto const changed{m_streamer.cut(x, z, radius, cut != 0)};
		forget_ground();
		WriteLog(format("SELFTEST terraincut %.1f %.1f: %s", x, z, changed ? "changed" : "nothing changed"));
		return selftest_result::handled;
	}
	if (command == "terraincutarea" || command == "terraingrassarea")
	{
		int value{1};
		words >> value;
		std::vector<glm::dvec2> outline;
		double x, z;
		while (words >> x >> z)
			outline.emplace_back(x, z);
		auto const changed{command == "terraincutarea" ? m_streamer.cut(outline, value != 0) : m_streamer.grass(outline, static_cast<std::uint16_t>(std::max(0, value)), value < 0)};
		forget_ground();
		WriteLog(format("SELFTEST %s: %s", command.c_str(), changed ? "changed" : "nothing changed"));
		return selftest_result::handled;
	}
	if (command == "terraingrasskind")
	{
		std::string name;
		std::getline(words >> std::ws, name);
		for (auto const &preset : heightmap::grass_presets())
			if (preset.name == name)
				m_grass_kind = m_streamer.add_grass_kind(preset);
		WriteLog(format("SELFTEST terraingrasskind %s: %zu kinds", name.c_str(), m_streamer.grass_kinds().size()));
		return selftest_result::handled;
	}
	if (command == "terraingrass")
	{
		double x{0.0};
		double z{0.0};
		double radius{5.0};
		int kind{0};
		int erase{0};
		words >> x >> z >> radius >> kind >> erase;
		auto const changed{m_streamer.grass(x, z, radius, 1.0, static_cast<std::uint16_t>(kind), erase != 0)};
		WriteLog(format("SELFTEST terraingrass %.1f %.1f: %s", x, z, changed ? "changed" : "nothing changed"));
		return selftest_result::handled;
	}
	if (command == "terraintool")
	{
		std::string name;
		words >> name;
		m_terrain_tool = name == "holes" ? terrain_tool::holes : name == "grass" ? terrain_tool::grass : name == "paint" ? terrain_tool::paint : name == "sculpt" ? terrain_tool::sculpt : terrain_tool::none;
		m_terrain_brush_radius = 3.0f;
		WriteLog("SELFTEST terraintool " + name);
		return selftest_result::handled;
	}
	if (command == "terraincursor")
	{
		glm::dvec3 world;
		auto const found{terrain_cursor(world)};
		WriteLog(format("SELFTEST terraincursor: %s %.2f %.2f %.2f", found ? "ground" : "none", world.x, world.y, world.z));
		return selftest_result::handled;
	}
	if (command == "trackterrain")
	{
		// the terrain drawn to the named tracks, or their formations taken out of it, as the buttons of the panel of the tracks
		std::string action;
		words >> action;
		std::vector<TTrack *> tracks;
		for (std::string name; words >> name;)
			if (auto *track{simulation::Paths.find(name)})
				tracks.push_back(track);
		if (action == "pull")
			nmt_pull_terrain(tracks);
		else
			nmt_restore_terrain(tracks);
		WriteLog(format("SELFTEST trackterrain %s %zu tracks: %s, %zu modifiers", action.c_str(), tracks.size(), m_nmt_state.status.c_str(), m_streamer.modifiers().size()));
		return selftest_result::handled;
	}
	if (command == "terrainhash")
	{
		// the heights of the chunks in memory hashed, to compare the ground two builds made
		std::vector<editor_terrain *> chunks;
		m_streamer.collect(chunks);
		std::sort(chunks.begin(), chunks.end(), [](editor_terrain const *A, editor_terrain const *B) { return A->key() < B->key(); });
		std::uint64_t hash{1469598103934665603ull};
		for (auto const *chunk : chunks)
			for (auto const height : chunk->heights())
			{
				std::uint32_t bits;
				std::memcpy(&bits, &height, sizeof(bits));
				hash = (hash ^ bits) * 1099511628211ull;
			}
		WriteLog(format("SELFTEST terrainhash: %zu chunks, %016llx", chunks.size(), static_cast<unsigned long long>(hash)));
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
			// the holes and the grass at the point
			grid += format(", holes %d%s, grass kinds %zu", data.holes, data.hole_at(x, z) ? " (cut here)" : "", data.grass_count());
			if (data.grass > 0)
			{
				auto const corner{data.corner()};
				auto const ix{std::clamp(static_cast<int>(std::lround((x - corner.x) / heightmap::chunk_size * data.grass)), 0, data.grass)};
				auto const iz{std::clamp(static_cast<int>(std::lround((z - corner.y) / heightmap::chunk_size * data.grass)), 0, data.grass)};
				auto const plane{static_cast<std::size_t>(data.grass + 1) * (data.grass + 1)};
				grid += ", grass";
				for (std::size_t slot = 0; slot < data.grass_count(); ++slot)
					grid += format(" %u:%d", data.grass_kinds[slot], data.grass_density[slot * plane + static_cast<std::size_t>(iz) * (data.grass + 1) + ix]);
			}
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
