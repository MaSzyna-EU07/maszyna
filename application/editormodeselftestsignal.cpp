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

editor_mode::selftest_result editor_mode::selftest_signal(std::string const &command, std::istringstream &words)
{
	if (command == "signal" || command == "signalmove" || command == "signaldelete" || command == "signalrename" || command == "signalprobe")
	{
		std::map<std::string, std::string> values;
		std::string pair;
		while (words >> pair)
		{
			auto const equals{pair.find('=')};
			if (equals != std::string::npos)
				values[pair.substr(0, equals)] = pair.substr(equals + 1);
		}
		show_track_tab(track_tab::signals);
		signal_scan();
		signal_refresh(true);
		auto &tool{m_signal};
		auto const standing_named = [&](std::string const &Name) -> signal_standing const * {
			auto const found{std::find_if(tool.standing.begin(), tool.standing.end(), [&](signal_standing const &Standing) { return Standing.name == Name; })};
			return found != tool.standing.end() ? &*found : nullptr;
		};
		auto const spot_on = [&](signal_template const &Template, int const Facing, int const Side, signal_spot &Spot) {
			auto *track{simulation::Paths.find(values["track"])};
			if (track == nullptr)
				return false;
			auto const path{values.count("path") ? std::stoi(values["path"]) : 0};
			auto const at{values.count("at") ? std::stod(values["at"]) : 0.5};
			geometry::bezier const curve{track->m_paths[std::clamp(path, 0, static_cast<int>(track->m_paths.size()) - 1)]};
			return signal_spot_at(*track, path, curve.point(at), curve.point(at), Facing, Side, Template, Spot);
		};
		if (command == "signal")
		{
			auto const *item{signal_template_of(values["file"])};
			if (item == nullptr)
			{
				WriteLog("SELFTEST signal: no signal template " + values["file"]);
				return selftest_result::handled;
			}
			for (int k = 0; k < static_cast<int>(tool.templates.size()); ++k)
				if (&tool.templates[k] == item)
				{
					static char const *const kinds[] = {"main", "block", "warning", "shunt", "stop", "substitute", "repeater"};
					for (int i = 0; i < 7; ++i)
						if (item->kind == kinds[i])
						{
							tool.kind = i;
							tool.chosen[i] = k;
						}
				}
			if (values.count("name"))
				std::snprintf(tool.name, sizeof(tool.name), "%s", values["name"].c_str());
			if (values.count("plate"))
				std::snprintf(tool.plate, sizeof(tool.plate), "%s", values["plate"].c_str());
			auto const side{values["side"] == "left" ? -1 : 1};
			auto const facing{values["facing"] == "along" ? 1 : values["facing"] == "against" ? -1 : 0};
			signal_spot spot;
			if (false == spot_on(*signal_armed(), facing, side, spot))
			{
				WriteLog("SELFTEST signal: no such track " + values["track"]);
				return selftest_result::handled;
			}
			auto const placed{signal_place(spot, *signal_armed())};
			WriteLog(format("SELFTEST signal %s: %s", placed ? "placed" : "failed", tool.status.c_str()));
		}
		else if (command == "signalmove")
		{
			auto const *standing{standing_named(values["name"])};
			auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
			if (item == nullptr)
			{
				WriteLog("SELFTEST signalmove: no signal " + values["name"]);
				return selftest_result::handled;
			}
			auto const yaw{glm::radians(standing->include.yaw + 180.0)};
			glm::dvec2 const travel{std::sin(yaw), std::cos(yaw)};
			signal_spot probe;
			if (false == spot_on(*item, 1, 1, probe))
			{
				WriteLog("SELFTEST signalmove: no such track " + values["track"]);
				return selftest_result::handled;
			}
			auto const facing{glm::dot(probe.travel, travel) >= 0.0 ? 1 : -1};
			auto const sideoftravel{glm::dot(geometry::plan_of(standing->include.location) - geometry::plan_of(standing->spot.axis), glm::dvec2{-travel.y, travel.x}) >= 0.0 ? 1 : -1};
			signal_select(standing->include.instance);
			tool.moving = true;
			tool.moved_valid = spot_on(*item, facing, sideoftravel * facing, tool.moved);
			tool.pressed = {-1000.f, -1000.f};
			signal_release();
			WriteLog("SELFTEST signalmove: " + tool.status);
		}
		else if (command == "signaldelete")
		{
			auto const *standing{standing_named(values["name"])};
			auto const deleted{standing != nullptr && signal_delete(standing->include.instance)};
			WriteLog(std::string{"SELFTEST signaldelete "} + values["name"] + (deleted ? ": removed with its event" : ": not removed by the signal tool"));
		}
		else if (command == "signalrename")
		{
			auto const *standing{standing_named(values["name"])};
			if (standing == nullptr)
			{
				WriteLog("SELFTEST signalrename: no signal " + values["name"]);
				return selftest_result::handled;
			}
			select_include(standing->include.instance);
			m_include.values[0] = values["to"];
			apply_include();
			m_include_gesture = false;
			WriteLog("SELFTEST signalrename " + values["name"] + " -> " + values["to"]);
		}
		else
		{
			signal_refresh(true);
			WriteLog(format("SELFTEST signalprobe: %zu signals", tool.standing.size()));
			for (auto const &standing : tool.standing)
			{
				std::string where{"not read"};
				if (standing.track != nullptr)
				{
					auto const &curve{standing.track->m_paths.front()};
					geometry::bezier const bez{curve};
					auto best{std::numeric_limits<double>::max()};
					double parameter{0.0};
					for (int k = 0; k <= 1024; ++k)
					{
						auto const d{geometry::plan_distance(bez.point(k / 1024.0), standing.include.location)};
						if (d < best)
						{
							best = d;
							parameter = k / 1024.0;
						}
					}
					auto const axis{bez.point(parameter)};
					auto tangent{bez.first(parameter)};
					glm::dvec2 travel{tangent.x, tangent.z};
					travel = glm::normalize(travel) * (standing.list == 1 ? -1.0 : 1.0);
					glm::dvec2 const offset{standing.include.location.x - axis.x, standing.include.location.z - axis.z};
					auto const cross{travel.x * offset.y - travel.y * offset.x};
					auto heading{glm::degrees(std::atan2(travel.x, travel.y))};
					auto turn{std::fmod(standing.include.yaw - heading + 720.0, 360.0)};
					auto const &list{editor_track::events(*standing.track, standing.list)};
					auto const bound{std::any_of(list.begin(), list.end(), [&](auto const &Event) { return Event.second != nullptr && ToLower(Event.first) == ToLower(standing.read); })};
					where = std::string{bound ? "bound, " : "unbound, "} + format("%s of %s, %.3f m from the axis, %s of the trains, %.3f m over the axis, model turned %.1f deg from the travel", editor_track::event_keyword(standing.list), standing.track->name().c_str(), glm::length(offset), cross > 0.0 ? "right" : "left",
					               standing.include.location.y - axis.y, turn);
				}
				WriteLog(format("SELFTEST signalprobe %s %s at %.3f %.3f %.3f yaw %.2f read %s: %s", standing.name.c_str(), standing.include.file.c_str(), standing.include.location.x, standing.include.location.y, standing.include.location.z, standing.include.yaw,
				                standing.read.empty() ? "-" : standing.read.c_str(), where.c_str()));
			}
		}
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
