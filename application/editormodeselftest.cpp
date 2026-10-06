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
#include "application/application.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"
#include "vehicle/DynObj.h"
#include "editor/editorFormat.hpp"

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

// the script is a text file, one action a line:
//   wait <frames>
//   camera <x> <y> <z> <x> <y> <z>        from the first point, looking at the second
//   select <track name>
//   sweep <key>=<value>...               model, params (separated by commas), lateral, height, side (origin, near, far,
//                                        centre), top (origin, bottom, top, edge), mode (bend, repeat), step,
//                                        length (original, all, or metres), from, to, widen (0, 1), face (0, 1), tilt (0, 1)
//   spread <track> <stop 0..2> <reach>   logs the paths the spread from the track takes
//   newscenery                           the editor starts again with the wizard of a new scenery
//   vehicles                             logs each vehicle: its head, facing, and what its couplers hold
//   screenshot
//   save
//   log <text>
//   quit
void editor_mode::selftest_step()
{
	auto &test{m_selftest};
	if (false == test.loaded)
	{
		test.loaded = true;
		auto const *file{std::getenv("EU07_EDITOR_SELFTEST")};
		if (file == nullptr)
			return;
		std::ifstream stream(file);
		std::string line;
		while (std::getline(stream, line))
			if (false == line.empty() && line.front() != '#')
				test.lines.push_back(line);
		WriteLog("SELFTEST: " + std::to_string(test.lines.size()) + " steps from " + file);
	}
	if (test.wait > 0)
	{
		--test.wait;
		return;
	}
	while (test.next < test.lines.size())
	{
		std::istringstream words(test.lines[test.next++]);
		std::string command;
		words >> command;
		if (command == "wait")
		{
			words >> test.wait;
			return;
		}
		if (command == "camera")
		{
			glm::dvec3 from, to;
			words >> from.x >> from.y >> from.z >> to.x >> to.y >> to.z;
			auto const look{glm::normalize(to - from)};
			Camera.Pos = from;
			Camera.Angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
			Global.pCamera = Camera;
			m_focus_active = false;
		}
		else if (command == "select")
		{
			std::string name;
			words >> name;
			m_node = simulation::Paths.find(name);
			m_track_point = {};
			ui()->set_node(m_node);
			WriteLog("SELFTEST select " + name + (m_node != nullptr ? "" : ": no such track"));
		}
		else if (command == "sweep")
		{
			std::map<std::string, std::string> values;
			std::string pair;
			while (words >> pair)
			{
				auto const equals{pair.find('=')};
				if (equals != std::string::npos)
					values[pair.substr(0, equals)] = pair.substr(equals + 1);
			}
			auto &tool{m_sweep};
			auto const get = [&](char const *Key, std::string const &Default) { auto const found{values.find(Key)}; return found != values.end() ? found->second : Default; };
			tool.edited = nullptr;
			tool.curve_for = nullptr;
			std::snprintf(tool.model, sizeof(tool.model), "%s", get("model", "").c_str());
			tool.parameters.clear();
			std::stringstream parameters(get("params", ""));
			std::string parameter;
			while (std::getline(parameters, parameter, ','))
			{
				tool.parameters.emplace_back();
				std::snprintf(tool.parameters.back().data(), tool.parameters.back().size(), "%s", parameter.c_str());
			}
			tool.parameters_for = tool.model;
			auto &settings{tool.settings};
			settings.lateral = std::stod(get("lateral", "0"));
			settings.height = std::stod(get("height", "0"));
			auto const side{get("side", "origin")};
			settings.side_anchor = side == "near" ? 1 : side == "far" ? 2 : side == "centre" ? 3 : 0;
			auto const top{get("top", "origin")};
			settings.height_anchor = top == "bottom" ? 1 : top == "top" ? 2 : top == "edge" ? 3 : 0;
			settings.bend = get("mode", "bend") != "repeat";
			settings.step = std::stod(get("step", "0"));
			settings.widen = get("widen", "0") == "1";
			settings.face = get("face", "1") == "1";
			settings.tilt = get("tilt", "0") == "1";
			settings.from = std::stod(get("from", "0"));
			settings.to = std::stod(get("to", "-1"));
			auto const length{get("length", "all")};
			tool.length_mode = length == "original" ? 0 : length == "all" ? 2 : 1;
			if (tool.length_mode == 1)
				tool.length = std::stod(length);
			tool.measured.clear();
			tool.open = true;
			sweep_curve();
			sweep_measure();
			if (get("create", "1") == "1")
			{
				sweep_create();
				WriteLog("SELFTEST sweep: " + tool.status);
			}
		}
		else if (command == "drag")
		{
			glm::dvec3 from{0.0}, to{0.0};
			words >> from.x >> from.z >> to.x >> to.z;
			m_sweep.open = true;
			if (false == sweep_press_at(from, false))
			{
				WriteLog("SELFTEST drag: the press wasn't taken");
				continue;
			}
			sweep_drag_at(to);
			sweep_release();
			WriteLog("SELFTEST drag: " + m_sweep.status);
		}
		else if (command == "probe")
		{
			// edge of the last laid model as it was made: its distance from the axis and the height of its top over the rail head
			std::string name;
			words >> name;
			auto *sweep{name.empty() ? m_sweep.edited : simulation::Sweeps.find(name)};
			if (sweep == nullptr)
			{
				WriteLog("SELFTEST probe: nothing laid");
				continue;
			}
			auto const &definition{sweep->definition()};
			double const sign{definition.lateral >= 0.0 ? 1.0 : -1.0};
			struct bin
			{
				double edge{std::numeric_limits<double>::max()};
				std::vector<std::pair<double, double>> points; // lateral, height over the axis
			};
			std::map<int, bin> bins;
			for (auto const &shape : sweep->create_shapes())
				for (auto const &vertex : shape.data().vertices)
				{
					double lateral;
					auto const station{sweep->project(vertex.position, lateral)};
					if (lateral * sign <= 0.0)
						continue;
					auto const axis{sweep->frame_at(station)};
					auto &entry{bins[static_cast<int>(station / 20.0)]};
					entry.edge = std::min(entry.edge, std::abs(lateral));
					entry.points.emplace_back(std::abs(lateral), vertex.position.y - axis.position.y);
				}
			WriteLog(format("SELFTEST probe %s: %zu stretches with the model, %.1f - %.1f m of %.1f m", sweep->name().c_str(), bins.size(), sweep->start(), sweep->end(), sweep->length()));
			for (auto const &[index, entry] : bins)
			{
				auto const station{index * 20.0 + 10.0};
				if (station < sweep->start() || station > sweep->end())
					continue;
				double top{-1000.0};
				for (auto const &point : entry.points)
					if (point.first < entry.edge + 0.15)
						top = std::max(top, point.second);
				auto const at{sweep->frame_at(station)};
				auto const head{0.18 + sweep_node::rail_head(at.roll, definition.lateral)};
				WriteLog(format("SELFTEST probe %s at %.0f m: edge %.3f m from the axis, its top %.3f m over the rail head (cant %.1f deg, gauge widening %.3f m, cant part %.3f m)", sweep->name().c_str(), station, entry.edge, top - head, at.roll,
				                at.widening[definition.lateral >= 0.0 ? 1 : 0], std::max(0.0, at.cant[definition.lateral >= 0.0 ? 1 : 0])));
			}
		}
		else if (command == "fouling")
		{
			std::map<std::string, std::string> values;
			std::string pair;
			while (words >> pair)
			{
				auto const equals{pair.find('=')};
				if (equals != std::string::npos)
					values[pair.substr(0, equals)] = pair.substr(equals + 1);
			}
			auto &tool{m_fouling};
			tool.scope = std::stoi(values.count("scope") ? values["scope"] : "2");
			tool.height_mode = values["mode"] == "ground" ? 0 : 1;
			tool.height = std::stod(values.count("height") ? values["height"] : "0");
			tool.dirty = true;
			fouling_update();
			WriteLog("SELFTEST fouling: " + tool.status);
			int shown{0};
			for (auto const &found : tool.found)
				if (found.wrong && shown++ < 5)
					WriteLog(format("SELFTEST fouling: %s %.2f m off, %+.3f m high", found.marker.file.c_str(), found.error, found.height_error));
			if (values["place"] == "1")
			{
				fouling_place();
				tool.dirty = true;
				fouling_update();
				WriteLog("SELFTEST fouling after placing: " + tool.status);
			}
			if (values["fix"] == "1")
			{
				fouling_fix();
				tool.dirty = true;
				fouling_update();
				WriteLog("SELFTEST fouling after the fix: " + tool.status);
			}
		}
		else if (command == "hekto")
		{
			std::map<std::string, std::string> values;
			std::string pair;
			while (words >> pair)
			{
				auto const equals{pair.find('=')};
				if (equals != std::string::npos)
					values[pair.substr(0, equals)] = pair.substr(equals + 1);
			}
			auto *track{simulation::Paths.find(values["track"])};
			if (track == nullptr)
			{
				WriteLog("SELFTEST hekto: no such track");
				continue;
			}
			hekto_start(*track, track->m_paths.front().points[segment_data::point::start]);
			auto &tool{m_hekto};
			tool.dirty = true;
			hekto_update();
			if (values.count("km"))
			{
				tool.km = std::stod(values["km"]);
				tool.growth = values["growth"] == "-1" ? -1 : 1;
			}
			tool.side = values["side"] == "alt" ? 0 : values["side"] == "left" ? -1 : 1;
			tool.dirty = true;
			hekto_update();
			auto const missing{std::count_if(tool.posts.begin(), tool.posts.end(), [](hekto_post const &Post) { return Post.standing < 0; })};
			WriteLog(format("SELFTEST hekto: km %.3f growth %d, %zu posts along %.0f m, %d to place, %zu standing. %s", tool.km, tool.growth, tool.posts.size(), tool.samples.empty() ? 0.0 : tool.samples.back().chainage, static_cast<int>(missing), tool.found.size(), tool.status.c_str()));
			for (std::size_t i = 0; i < tool.posts.size() && i < 4; ++i)
			{
				auto const &post{tool.posts[i]};
				double lateral;
				glm::dvec3 const axis{editor_track::sampled_position(tool.samples, post.chainage)};
				auto const direction{editor_track::sampled_position(tool.samples, post.chainage + 1.0) - axis};
				glm::dvec2 const offset{post.position.x - axis.x, post.position.z - axis.z};
				lateral = glm::length(offset);
				auto const right{(direction.x * offset.y - direction.z * offset.x) * tool.growth >= 0.0};
				WriteLog(format("SELFTEST hekto post %d.%d at %.1f m, %.2f m from the axis, %s of the growing kilometrage, at %.1f %.1f %.1f", post.hectometres / 10, post.hectometres % 10, post.chainage, lateral, right ? "right" : "left", post.position.x, post.position.y,
				                post.position.z));
			}
			if (values["place"] == "1")
			{
				hekto_place();
				WriteLog("SELFTEST hekto: " + tool.status);
			}
		}
		else if (command == "tracks")
		{
			int tab{0};
			words >> tab;
			show_track_tab(track_tab::lineside);
			m_sweep.expand = tab == 0;
			m_hekto.expand = tab == 1;
			m_fouling.expand = tab == 2;
			m_parallel.expand = tab == 3;
			m_vehicle.expand = tab == 4;
		}
		else if (command == "curve")
		{
			sweep_curve();
			for (auto const &piece : m_sweep.curve)
			{
				auto const &a{piece.points[segment_data::point::start]};
				auto const &b{piece.points[segment_data::point::end]};
				WriteLog(format("SELFTEST curve piece %.1f %.1f -> %.1f %.1f", a.x, a.z, b.x, b.z));
			}
			if (false == m_sweep.outline.empty())
				WriteLog(format("SELFTEST curve outline %zu points, %.1f %.1f -> %.1f %.1f, %.1f m", m_sweep.outline.size(), m_sweep.outline.front().x, m_sweep.outline.front().z, m_sweep.outline.back().x, m_sweep.outline.back().z, m_sweep.curve_length));
		}
		else if (command == "projtest")
		{
			screen_projection const projection;
			for (std::size_t i = 0; i < m_sweep.outline.size() && i < 45; i += 4)
			{
				ImVec2 screen;
				auto const shown{projection.project(m_sweep.outline[i], screen)};
				auto const clip{projection.clip(m_sweep.outline[i])};
				WriteLog(format("SELFTEST proj %zu: %.1f %.1f %.1f -> %d %.0f %.0f w %.2f", i, m_sweep.outline[i].x, m_sweep.outline[i].y, m_sweep.outline[i].z, shown ? 1 : 0, screen.x, screen.y, clip.w));
			}
		}
		else if (command == "spread")
		{
			std::string name;
			words >> name >> m_spread.stop >> m_spread.reach;
			if (auto *track{simulation::Paths.find(name)})
			{
				std::string names;
				for (auto const *other : track_spread_from(*track))
					names += ' ' + other->name();
				WriteLog("SELFTEST spread " + name + ":" + names + " | prev " + (track->trPrev ? track->trPrev->name() : std::string{"-"}) + " next " + (track->trNext ? track->trNext->name() : std::string{"-"}));
			}
		}
		else if (command == "newscenery")
		{
			WriteLog(restart_for_new_scenery() ? "SELFTEST newscenery: started" : "SELFTEST newscenery: failed");
			return;
		}
		else if (command == "vehicles")
		{
			for (auto *vehicle : simulation::Vehicles.sequence())
			{
				if (vehicle == nullptr)
					continue;
				auto const head{vehicle->HeadPosition()};
				auto const front{vehicle->VectorFront()};
				auto const coupler = [&](int const End) {
					auto const &coupler{vehicle->MoverParameters->Couplers[End]};
					return coupler.Connected != nullptr ? coupler.Connected->Name + "/" + std::to_string(coupler.CouplingFlag) : std::string{"-"};
				};
				WriteLog(format("SELFTEST vehicle %s head %.2f %.2f front %.2f %.2f dir %d coupler0 %s coupler1 %s", vehicle->name().c_str(), head.x, head.z, front.x, front.z, vehicle->iDirection, coupler(0).c_str(), coupler(1).c_str()));
			}
		}
		else if (command == "screenshot")
		{
			Application.queue_screenshot();
			test.wait = 3;
			return;
		}
		else if (command == "save")
		{
			save();
		}
		else if (command == "log")
		{
			std::string text;
			std::getline(words, text);
			WriteLog("SELFTEST" + text);
		}
		else if (command == "quit")
		{
			if (false == test.quitting)
			{
				WriteLog("SELFTEST done");
				test.quitting = true;
				test.wait = 30;
				--test.next;
				return;
			}
			Application.queue_quit(true);
			return;
		}
	}
}
