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
#include "world/MemCell.h"
#include "world/Event.h"
#include "vehicle/DynObj.h"
#include "model/AnimModel.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "world/Sweep.h"
#include "input/keyboardinput.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include "editor/editorFormat.hpp"

#include <cstdint>
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
//   selectmodel <x> <z>                  the model instance nearest the point
//   bend <key>=<value>...                the selected model bent along a track: track (index on the list), axis (auto, z, x),
//                                        turned (0, 1), along (start, origin, middle, end, point), side (origin, near, far,
//                                        middle, point), height (origin, bottom, top, edge, point), point=<x>,<y>,<z>,
//                                        pick=<x>,<y>,<z> (a point of the scene picked as the reference point), go (0, 1)
//   selectinclude <x> <z>                the include placed nearest the point
//   bendset <key>=<value>...             the bent model being edited: length, lateral, height, along, side, height
//   bendprobe                            logs the bent model being edited
//   undo, redo
//   key <S|W|CTRL|RMB> <press|release> [ctrl]  a key or right button event, as the window gives it
//   lineside <vehicle|hekto|fouling|parallel>  the tab of the objects along the track in the track window
//   event <name>                         queues the event
//   memcell <name>                       logs the memory cell
//   bendpanel                            unfolds the bend section of the toolset
//   openscenerypopup                     the window of File / Open scenery, its list searched for zz_
//   openscenery <scenery file>           the editor starts again with the scenery
//   checkspans                           logs whether the text the scenery files hold at each node's place starts and ends it
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
			settings.platform = get("platform", "0") == "1";
			settings.face = get("face", "1") == "1";
			settings.tilt = get("tilt", "0") == "1";
			settings.from = std::stod(get("from", "0"));
			settings.to = std::stod(get("to", "-1"));
			auto const length{get("length", "all")};
			tool.length_mode = length == "original" ? 0 : length == "all" ? 2 : 1;
			if (tool.length_mode == 1)
				tool.length = std::stod(length);
			if (values.count("edge"))
				EditorSettings.platform_edge(std::stod(values["edge"]));
			tool.measured.clear();
			tool.open = true;
			sweep_curve();
			sweep_measure();
			if (values.count("widen"))
				settings.widen = get("widen", "0") == "1";
			if (values.count("platform"))
				settings.platform = get("platform", "0") == "1";
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
			double step{20.0};
			words >> name >> step;
			if (name == "-")
				name.clear();
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
					auto &entry{bins[static_cast<int>(std::floor(station / step))]};
					entry.edge = std::min(entry.edge, std::abs(lateral));
					entry.points.emplace_back(std::abs(lateral), vertex.position.y - axis.position.y);
				}
			WriteLog(format("SELFTEST probe %s: %zu stretches with the model, %.1f - %.1f m of %.1f m", sweep->name().c_str(), bins.size(), sweep->start(), sweep->end(), sweep->length()));
			for (auto const &[index, entry] : bins)
			{
				auto const station{(index + 0.5) * step};
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
		else if (command == "profile")
		{
			std::string name;
			double station{0.0};
			words >> name >> station;
			auto *sweep{simulation::Sweeps.find(name)};
			if (sweep == nullptr)
			{
				WriteLog("SELFTEST profile: no such sweep");
				continue;
			}
			auto const &definition{sweep->definition()};
			std::map<int, std::pair<double, double>> columns;
			for (auto const &shape : sweep->create_shapes())
				for (auto const &vertex : shape.data().vertices)
				{
					double lateral;
					auto const at{sweep->project(vertex.position, lateral)};
					if (std::abs(at - station) > 1.0)
						continue;
					auto const axis{sweep->frame_at(at)};
					auto const height{vertex.position.y - axis.position.y - 0.18 - sweep_node::rail_head(axis.roll, definition.lateral)};
					auto &column{columns.try_emplace(static_cast<int>(std::floor(lateral / 0.1)), std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()).first->second};
					column.first = std::min(column.first, height);
					column.second = std::max(column.second, height);
				}
			std::string text;
			for (auto const &[index, column] : columns)
				text += format(" [%.1f %.2f..%.2f]", index * 0.1, column.first, column.second);
			WriteLog(format("SELFTEST profile %s at %.0f m:", name.c_str(), station) + text);
		}
		else if (command == "curvedswitch")
		{
			std::string name, label;
			glm::dvec3 at{0.0}, toward{0.0};
			words >> name >> at.x >> at.z >> toward.x >> toward.z >> label;
			m_node = simulation::Paths.find(name);
			ui()->set_node(m_node);
			show_track_tab(track_tab::turnout);
			arm_switch();
			auto &tool{m_switch};
			for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
				if (false == label.empty() && tool.templates[i].label.find(label) != std::string::npos)
				{
					tool.armed = i;
					break;
				}
			m_cursor_override = at;
			auto const started{start_switch_placement()};
			m_cursor_override.reset();
			if (false == started)
			{
				WriteLog("SELFTEST curvedswitch: not started, " + tool.status);
				continue;
			}
			tool.mouse = glm::dvec3{toward.x, tool.point.y, toward.z};
			finish_switch_placement();
			WriteLog("SELFTEST curvedswitch " + (tool.armed >= 0 ? tool.templates[tool.armed].label : std::string{}) + ": " + tool.status);
		}
		else if (command == "conecheck")
		{
			std::string name;
			words >> name;
			auto *track{simulation::Paths.find(name)};
			if (track == nullptr || track->m_paths.size() < 2)
			{
				WriteLog("SELFTEST conecheck: no switch " + name);
				continue;
			}
			geometry::bezier const main{track->m_paths[0]};
			geometry::bezier const other{track->m_paths[1]};
			std::string text;
			for (double t = 0.125; t < 1.01; t += 0.125)
			{
				auto const point{other.point(t)};
				double best{std::numeric_limits<double>::max()}, along{0.0};
				for (int k = 0; k <= 2000; ++k)
					if (auto const distance{geometry::plan_distance(main.point(k / 2000.0), point)}; distance < best)
					{
						best = distance;
						along = k / 2000.0;
					}
				auto const base{main.point(along)};
				auto const direction{glm::normalize(geometry::plan_of(main.first(along)))};
				auto const lateral{(point.x - base.x) * -direction.y + (point.z - base.z) * direction.x};
				auto const &path{track->m_paths[0]};
				auto const roll{glm::radians(path.rolls[0] + (path.rolls[1] - path.rolls[0]) * along)};
				auto const cone{base.y - lateral * std::tan(roll)};
				text += format(" [t %.3f off %.3f dy %.4f]", t, lateral, point.y - cone);
			}
			WriteLog("SELFTEST conecheck " + name + ":" + text);
		}
		else if (command == "frames")
		{
			std::string name;
			double from{0.0}, to{0.0}, step{0.1};
			words >> name >> from >> to >> step;
			auto *sweep{simulation::Sweeps.find(name)};
			if (sweep == nullptr)
				continue;
			auto const side{sweep->definition().lateral >= 0.0 ? 1 : 0};
			for (double station = from; station <= to + 1e-9; station += step)
			{
				auto const at{sweep->frame_at(station)};
				WriteLog(format("SELFTEST frame %s %.2f: %.3f %.3f %.3f widening %.4f cant %.4f setback %.4f", name.c_str(), station, at.position.x, at.position.y, at.position.z, at.widening[side], at.cant[side], sweep->setback(at)));
			}
		}
		else if (command == "switchdrive")
		{
			int index{-1}, side{0};
			words >> index >> side;
			scan_switch_drives();
			m_switch.drive = index;
			m_switch.drive_side = side;
			WriteLog("SELFTEST switchdrive: " + (index >= 0 && index < static_cast<int>(m_switch.drives.size()) ? m_switch.drives[index].name : std::string{"none"}));
		}
		else if (command == "drivetilt")
		{
			std::string name;
			words >> name;
			auto *model{simulation::Instances.find(name)};
			auto *track{simulation::Paths.find(name.substr(0, name.find('_')))};
			if (model == nullptr || track == nullptr)
			{
				WriteLog("SELFTEST drivetilt: no " + name);
				continue;
			}
			auto const &path{track->m_paths[0]};
			auto const transform{glm::dmat3(model->rotation_scale())};
			auto const across{transform * glm::dvec3{1.0, 0.0, 0.0}};
			auto const forward{transform * glm::dvec3{0.0, 0.0, 1.0}};
			auto const direction{glm::normalize(glm::dvec2{forward.x, forward.z})};
			auto const lateral{across.x * -direction.y + across.z * direction.x};
			auto const cone{-lateral * std::tan(glm::radians(static_cast<double>(path.rolls[0])))};
			auto const control{path.points[segment_data::point::control1]};
			auto const grade{control.y / std::hypot(control.x, control.z)};
			WriteLog(format("SELFTEST drivetilt %s: angles %.3f %.3f %.3f, +x rises %.4f (cone %.4f), +z rises %.4f (grade %.4f), origin %.3f %.3f %.3f, points %.3f", name.c_str(), model->Angles().x, model->Angles().y, model->Angles().z, across.y, cone, forward.y, grade,
			                model->location().x, model->location().y, model->location().z, path.points[segment_data::point::start].y));
		}
		else if (command == "weather")
		{
			auto const &time{simulation::Time.data()};
			WriteLog(format("SELFTEST weather: %02d:%02d, fog end %.0f m, overcast %.2f", time.wHour, time.wMinute, Global.fFogEnd, Global.Overcast));
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
		else if (command == "lay")
		{
			std::string name, side;
			words >> name >> side;
			auto *track{simulation::Paths.find(name)};
			if (track == nullptr || track->m_paths.empty())
			{
				WriteLog("SELFTEST lay: no track " + name);
				return;
			}
			auto const &path{track->m_paths.front()};
			auto const atend{side == "end"};
			editor_track::snap_target start;
			start.track = track;
			start.point = {0, atend ? editor_track::point_kind::end : editor_track::point_kind::start};
			start.position = path.points[atend ? segment_data::point::end : segment_data::point::start];
			auto const &control{path.points[atend ? segment_data::point::control2 : segment_data::point::control1]};
			start.direction = glm::normalize(control != glm::dvec3{} ? control : path.points[atend ? segment_data::point::start : segment_data::point::end] - start.position);
			m_lay.points = {start.position};
			m_lay.start = start;
			for (glm::dvec3 point{0.0, start.position.y, 0.0}; words >> point.x >> point.z;)
				m_lay.points.push_back(point);
			lay_finish({});
			WriteLog("SELFTEST lay: " + m_lay.status);
		}
		else if (command == "analyse")
		{
			std::string name;
			words >> name;
			editor_track::curve curve;
			if (auto *track{simulation::Paths.find(name)}; track != nullptr && editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve))
			{
				std::string arcs;
				for (auto const &arc : curve.arcs)
					arcs += format(" R%.0f/%.1fdeg/after %.1fm", arc.radius, glm::degrees(arc.turn), arc.transition);
				WriteLog(format("SELFTEST analyse %s: R %.0f, compound %d, in %.1f m, out %.1f m, turn %.1f deg,", name.c_str(), curve.radius, curve.compound ? 1 : 0, curve.transition_in, curve.transition_out, glm::degrees(curve.turn)) + arcs);
			}
			else
				WriteLog("SELFTEST analyse " + name + ": no curve");
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
		else if (command == "turntable")
		{
			glm::dvec3 centre{0.0};
			double yaw{0.0};
			words >> centre.x >> centre.y >> centre.z >> yaw;
			turntable_place(centre, yaw);
			for (double angle; words >> angle;)
			{
				m_turntable.heading = m_turntable.table_yaw + angle;
				m_turntable.exit_length = 10.0;
				track_intent intent;
				intent.what = track_intent::kind::turntable_exit;
				turntable_press(intent);
			}
			WriteLog("SELFTEST turntable: " + (m_turntable.error.empty() ? m_turntable.status : m_turntable.error));
		}
		else if (command == "selectmodel")
		{
			glm::dvec3 point{0.0};
			words >> point.x >> point.z;
			TAnimModel *nearest{nullptr};
			auto best{50.0};
			for (auto *model : simulation::Instances.sequence())
			{
				if (model == nullptr)
					continue;
				auto const distance{glm::length(glm::dvec2{model->location().x - point.x, model->location().z - point.z})};
				if (distance < best)
				{
					best = distance;
					nearest = model;
				}
			}
			m_node = nearest;
			select_include(0);
			ui()->set_node(m_node);
			WriteLog("SELFTEST selectmodel: " + (nearest != nullptr ? format("%s at %.2f %.2f %.2f", nearest->name().c_str(), nearest->location().x, nearest->location().y, nearest->location().z) : std::string{"none"}));
		}
		else if (command == "selectinclude")
		{
			glm::dvec3 point{0.0};
			words >> point.x >> point.z;
			scene::instance_handle nearest{0};
			auto best{50.0};
			m_node = nullptr;
			for (scene::instance_handle instance = 1; instance <= scene::Layers.instance_count(); ++instance)
			{
				if (false == scene::Layers.removable(instance))
					continue;
				select_include(instance);
				bend_source source;
				if (false == bend_source_of_selection(source))
					continue;
				auto const at{glm::dvec3(source.transform[3])};
				auto const distance{glm::length(glm::dvec2{at.x - point.x, at.z - point.z})};
				if (distance < best)
				{
					best = distance;
					nearest = instance;
				}
			}
			select_include(nearest);
			WriteLog("SELFTEST selectinclude: " + (nearest != 0 ? *scene::Layers.instance(nearest).file : std::string{"none"}));
		}
		else if (command == "bend" || command == "bendset")
		{
			std::map<std::string, std::string> values;
			std::string pair;
			while (words >> pair)
			{
				auto const equals{pair.find('=')};
				if (equals != std::string::npos)
					values[pair.substr(0, equals)] = pair.substr(equals + 1);
			}
			auto const vector = [](std::string Text) {
				std::replace(Text.begin(), Text.end(), ',', ' ');
				std::istringstream numbers(Text);
				glm::dvec3 result{0.0};
				numbers >> result.x >> result.y >> result.z;
				return result;
			};
			auto const index = [](std::string const &Text, std::vector<std::string> const &Names) {
				return static_cast<int>(std::find(Names.begin(), Names.end(), Text) - Names.begin());
			};
			auto &tool{m_bend};
			auto const along{values.count("along") ? index(values["along"], {"start", "origin", "middle", "end", "point"}) : tool.along_anchor};
			auto const side{values.count("side") ? index(values["side"], {"origin", "near", "far", "middle", "point"}) : tool.side_anchor};
			auto const height{values.count("height") ? index(values["height"], {"origin", "bottom", "top", "edge", "point"}) : tool.height_anchor};
			auto const point{values.count("point") ? vector(values["point"]) : tool.point};
			if (command == "bend")
			{
				tool.edited = nullptr;
				tool.along_anchor = along;
				tool.side_anchor = side;
				tool.height_anchor = height;
				tool.point = point;
				tool.axis = values.count("axis") ? index(values["axis"], {"auto", "z", "x"}) : 0;
				tool.turned = values["turned"] == "1";
				bend_source source;
				if (false == bend_source_of_selection(source))
				{
					std::string reason;
					auto const editable{m_node != nullptr && scene::Layers.editable(m_node, &reason)};
					WriteLog(format("SELFTEST bend: nothing to bend (node %p template %d editable %d %s, instance %d)", static_cast<void *>(m_node), m_node != nullptr && m_node->from_template() ? 1 : 0, editable ? 1 : 0, reason.c_str(), static_cast<int>(m_instance)));
					continue;
				}
				tool.tracks_for = nullptr;
				bend_find_tracks(glm::dvec3(source.transform[3]), source.model != nullptr ? static_cast<void const *>(source.model) : reinterpret_cast<void const *>(static_cast<std::uintptr_t>(source.instance)));
				tool.track = values.count("track") ? std::stoi(values["track"]) : 0;
				if (values.count("pick"))
					bend_pick(vector(values["pick"]));
				if (tool.tracks.empty())
				{
					WriteLog("SELFTEST bend: no track nearby");
					continue;
				}
				sweep_node::state state;
				std::vector<TTrack *> tracks;
				bend_definition(source, *tool.tracks[tool.track].first, state, tracks);
				WriteLog(format("SELFTEST bend: track %s, %zu paths, anchor before %.3f %.3f %.3f, lateral %.3f height %.3f from %.3f to %.3f, axis %s flip %d; %s", tool.tracks[tool.track].first->name().c_str(), tracks.size(),
				                tool.marker.x, tool.marker.y, tool.marker.z, state.lateral, state.height, state.from, state.to, state.along_x ? "x" : "z", state.flip ? 1 : 0, tool.status.c_str()));
				if (values["go"] == "1")
				{
					bend_selection();
					WriteLog("SELFTEST bend: " + tool.status);
				}
			}
			else if (auto *sweep{tool.edited})
			{
				auto state{sweep->definition()};
				tool.along_anchor = along;
				if (side != tool.side_anchor || height != tool.height_anchor || point != tool.point)
				{
					tool.side_anchor = side;
					tool.height_anchor = height;
					tool.point = point;
					bend_reanchor(state, side, height, point);
				}
				if (values.count("lateral"))
					state.lateral = std::stod(values["lateral"]);
				if (values.count("up"))
					state.height = std::stod(values["up"]);
				if (values.count("widen"))
					state.widen = values["widen"] == "1";
				if (values.count("pick"))
				{
					bend_pick(vector(values["pick"]));
					state = sweep->definition();
				}
				if (values.count("length"))
				{
					bend_stretch(*sweep, bend_fixed(*sweep, state.point), std::stod(values["length"]), state.point, state);
				}
				bend_edit(state);
				WriteLog("SELFTEST bendset done");
			}
		}
		else if (command == "bendprobe")
		{
			auto *sweep{m_bend.edited};
			if (sweep == nullptr)
			{
				WriteLog("SELFTEST bendprobe: nothing bent");
				continue;
			}
			glm::dvec3 low, high, shift;
			sweep->bounds(low, high, shift);
			auto const &state{sweep->definition()};
			auto const marker{bend_marker(*sweep)};
			glm::dvec3 boxlow{std::numeric_limits<double>::max()}, boxhigh{-std::numeric_limits<double>::max()};
			std::size_t vertices{0};
			for (auto const &shape : sweep->create_shapes())
				for (auto const &vertex : shape.data().vertices)
				{
					boxlow = glm::min(boxlow, vertex.position);
					boxhigh = glm::max(boxhigh, vertex.position);
					++vertices;
				}
			WriteLog(format("SELFTEST bendprobe %s: stations %.3f - %.3f, along the reference line %.3f m, model %.3f m, lateral %.3f height %.3f anchors %d/%d point %.3f %.3f %.3f, marker %.3f %.3f %.3f, box %.2f %.2f %.2f - %.2f %.2f %.2f, %zu vertices, removed %d",
			                sweep->name().c_str(), sweep->start(), sweep->end(), sweep->distance_at(sweep->end()) - sweep->distance_at(sweep->start()), high.x - low.x, state.lateral, state.height, state.side_anchor, state.height_anchor, state.point.x,
			                state.point.y, state.point.z, marker.x, marker.y, marker.z, boxlow.x, boxlow.y, boxlow.z, boxhigh.x, boxhigh.y, boxhigh.z, vertices, sweep->m_editorremoved ? 1 : 0));
		}
		else if (command == "modelbox")
		{
			auto *model{dynamic_cast<TAnimModel *>(m_node)};
			bend_source source;
			if (model == nullptr || false == bend_source_of_selection(source))
			{
				WriteLog("SELFTEST modelbox: no model selected");
				continue;
			}
			scene::node_data data;
			sweep_node probe{data};
			sweep_node::state state;
			state.model = source.file;
			state.skin = source.skin;
			state.scale = source.scale;
			probe.define(state);
			glm::dvec3 boxlow{std::numeric_limits<double>::max()}, boxhigh{-std::numeric_limits<double>::max()};
			for (auto const &item : probe.items())
			{
				auto const transform{source.transform * glm::scale(glm::dmat4(1.0), source.scale) * item.transform};
				std::vector<TSubModel *> pending{item.model->GetSMRoot()};
				while (false == pending.empty())
				{
					auto *submodel{pending.back()};
					pending.pop_back();
					for (; submodel != nullptr; submodel = submodel->Next)
					{
						if (submodel->Child != nullptr)
							pending.push_back(submodel->Child);
						if (submodel->eType != GL_TRIANGLES || submodel->fSquareMinDist > 0.f || (submodel->m_geometry.handle.bank == 0 && submodel->m_geometry.handle.chunk == 0))
							continue;
						glm::dmat4 local{1.0};
						for (auto *parent = submodel; parent != nullptr; parent = parent->Parent)
							if ((parent->iFlags & 0xC000) != 0 && parent->GetMatrix() != nullptr)
								local = glm::dmat4(glm::make_mat4(parent->GetMatrix()->readArray())) * local;
						for (auto const &vertex : GfxRenderer->Vertices(submodel->m_geometry.handle))
						{
							auto const world{glm::dvec3(transform * local * glm::dvec4(glm::dvec3(vertex.position), 1.0))};
							boxlow = glm::min(boxlow, world);
							boxhigh = glm::max(boxhigh, world);
						}
					}
				}
			}
			WriteLog(format("SELFTEST modelbox: %.2f %.2f %.2f - %.2f %.2f %.2f", boxlow.x, boxlow.y, boxlow.z, boxhigh.x, boxhigh.y, boxhigh.z));
		}
		else if (command == "key")
		{
			std::string key, action, mods;
			words >> key >> action >> mods;
			if (key == "RMB")
				m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT, action == "press" ? GLFW_PRESS : GLFW_RELEASE);
			else
			{
				auto const code{key == "S" ? GLFW_KEY_S : key == "W" ? GLFW_KEY_W : key == "CTRL" ? GLFW_KEY_LEFT_CONTROL : 0};
				on_key(code, 0, action == "press" ? GLFW_PRESS : GLFW_RELEASE, mods == "ctrl" ? GLFW_MOD_CONTROL : 0);
			}
			WriteLog(format("SELFTEST key %s %s %s: camera %.2f %.2f %.2f, key S %d, right button %d", key.c_str(), action.c_str(), mods.c_str(), Camera.Pos.x, Camera.Pos.y, Camera.Pos.z, static_cast<int>(input::keys[GLFW_KEY_S]), m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT)));
		}
		else if (command == "lineside")
		{
			std::string tab;
			words >> tab;
			show_track_tab(track_tab::lineside);
			(tab == "vehicle" ? m_vehicle.expand : tab == "hekto" ? m_hekto.expand : tab == "fouling" ? m_fouling.expand : m_parallel.expand) = true;
		}
		else if (command == "event")
		{
			std::string name;
			words >> name;
			auto *event{simulation::Events.FindEvent(name)};
			if (event != nullptr)
				simulation::Events.AddToQuery(event, nullptr);
			WriteLog("SELFTEST event " + name + (event != nullptr ? " queued" : ": no such event"));
		}
		else if (command == "memcell")
		{
			std::string name;
			words >> name;
			auto *cell{simulation::Memory.find(name)};
			WriteLog("SELFTEST memcell " + name + (cell != nullptr ? format(": %s %.1f %.1f", cell->Text().c_str(), cell->Value1(), cell->Value2()) : std::string{": none"}));
		}
		else if (command == "bendpanel")
		{
			ui()->expand_bend();
		}
		else if (command == "newscenerypopup")
		{
			m_newscenery_asked = true;
		}
		else if (command == "openscenerypopup")
		{
			m_openscenery_asked = true;
			std::snprintf(m_openscenery_search, sizeof(m_openscenery_search), "%s", "zz_");
		}
		else if (command == "openscenery")
		{
			std::string name;
			words >> name;
			WriteLog(restart_editor(name) ? "SELFTEST openscenery: started " + name : "SELFTEST openscenery: failed");
		}
		else if (command == "gauge")
		{
			std::string name;
			words >> name;
			if (auto *track{simulation::Paths.find(name)})
			{
				scan_gauge(*track);
				WriteLog(format("SELFTEST gauge %s: %d hits", name.c_str(), static_cast<int>(m_gauge.line.hits.size())));
				for (auto const &hit : m_gauge.line.hits)
					WriteLog(format("SELFTEST gauge hit %s depth %.3f at %.2f %.2f %.2f", hit.model.c_str(), hit.depth, hit.point.x, hit.point.y, hit.point.z));
			}
			else
				WriteLog("SELFTEST gauge " + name + ": no such track");
		}
		else if (command == "deletetrack")
		{
			delete_selected_track();
			WriteLog("SELFTEST deletetrack");
		}
		else if (command == "ttincludes")
		{
			std::string name;
			words >> name;
			auto const *track{simulation::Paths.find(name)};
			WriteLog(format("SELFTEST ttincludes %s: track %s, includes %d", name.c_str(), track == nullptr ? "none" : track->m_editorremoved ? "removed" : "present",
			                track != nullptr ? static_cast<int>(turntable_includes(*track).size()) : -1));
		}
		else if (command == "undo")
		{
			undo_last();
			WriteLog("SELFTEST undo");
		}
		else if (command == "redo")
		{
			redo_last();
			WriteLog("SELFTEST redo");
		}
		else if (command == "checkspans")
		{
			WriteLog("SELFTEST spans: " + scene::Layers.check_sources());
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
