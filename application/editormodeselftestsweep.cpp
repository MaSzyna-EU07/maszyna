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

editor_mode::selftest_result editor_mode::selftest_sweep(std::string const &command, std::istringstream &words)
{
	if (command == "sweep")
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
		return selftest_result::handled;
	}
	if (command == "drag")
	{
		glm::dvec3 from{0.0};
		glm::dvec3 to{0.0};
		words >> from.x >> from.z >> to.x >> to.z;
		m_sweep.open = true;
		if (false == sweep_press_at(from, false))
		{
			WriteLog("SELFTEST drag: the press wasn't taken");
			return selftest_result::handled;
		}
		sweep_drag_at(to);
		sweep_release();
		WriteLog("SELFTEST drag: " + m_sweep.status);
		return selftest_result::handled;
	}
	if (command == "fouling")
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
		if (values["show"] == "1")
		{
			show_track_tab(track_tab::lineside);
			tool.expand = true;
		}
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
		if (values["place"] == "one")
		{
			// the first fouling point without a marker alone, as a click on it in the view
			for (int index = 0; index < static_cast<int>(tool.points.size()); ++index)
				if (tool.points[index].standing < 0)
				{
					fouling_place(index);
					break;
				}
			tool.dirty = true;
			fouling_update();
			WriteLog("SELFTEST fouling one: " + tool.status);
		}
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
		return selftest_result::handled;
	}
	if (command == "hekto")
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
			return selftest_result::handled;
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
		return selftest_result::handled;
	}
	if (command == "spread")
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
		return selftest_result::handled;
	}
	if (command == "turntable")
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
		return selftest_result::handled;
	}
	if (command == "bend" || command == "bendset")
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
				return selftest_result::handled;
			}
			tool.tracks_for = nullptr;
			bend_find_tracks(glm::dvec3(source.transform[3]), source.model != nullptr ? static_cast<void const *>(source.model) : reinterpret_cast<void const *>(static_cast<std::uintptr_t>(source.instance)));
			tool.track = values.count("track") ? std::stoi(values["track"]) : 0;
			if (values.count("pick"))
				bend_pick(vector(values["pick"]));
			if (tool.tracks.empty())
			{
				WriteLog("SELFTEST bend: no track nearby");
				return selftest_result::handled;
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
		return selftest_result::handled;
	}
	if (command == "bendprobe")
	{
		auto *sweep{m_bend.edited};
		if (sweep == nullptr)
		{
			WriteLog("SELFTEST bendprobe: nothing bent");
			return selftest_result::handled;
		}
		glm::dvec3 low;
		glm::dvec3 high;
		glm::dvec3 shift;
		sweep->bounds(low, high, shift);
		auto const &state{sweep->definition()};
		auto const marker{bend_marker(*sweep)};
		glm::dvec3 boxlow{std::numeric_limits<double>::max()};
		glm::dvec3 boxhigh{-std::numeric_limits<double>::max()};
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
		return selftest_result::handled;
	}
	if (command == "lineside")
	{
		std::string tab;
		words >> tab;
		show_track_tab(tab == "parallel" ? track_tab::parallel : track_tab::lineside);
		(tab == "vehicle" ? m_vehicle.expand : tab == "hekto" ? m_hekto.expand : tab == "fouling" ? m_fouling.expand : m_parallel.expand) = true;
		return selftest_result::handled;
	}
	if (command == "bendpanel")
	{
		ui()->expand_bend();
		return selftest_result::handled;
	}
	if (command == "gauge")
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
		return selftest_result::handled;
	}
	if (command == "ttincludes")
	{
		std::string name;
		words >> name;
		auto const *track{simulation::Paths.find(name)};
		WriteLog(format("SELFTEST ttincludes %s: track %s, includes %d", name.c_str(), track == nullptr ? "none" : track->m_editorremoved ? "removed" : "present",
		                track != nullptr ? static_cast<int>(turntable_includes(*track).size()) : -1));
		return selftest_result::handled;
	}
	if (command == "checkspans")
	{
		WriteLog("SELFTEST spans: " + scene::Layers.check_sources());
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
