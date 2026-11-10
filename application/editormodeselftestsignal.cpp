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
#include <array>
#include <ranges>
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
	if (command == "signaltemplates")
	{
		// signaltemplates [file=<template>]: the templates of the signals with what their descriptions give, the one named chosen
		// for its kind in the palette, not placed
		auto values{selftest_values(words)};
		show_track_tab(track_tab::signals);
		signal_scan(true);
		auto &tool{m_signal};
		static std::array<char const *, 8> constexpr kinds{"main", "block", "warning", "shunt", "stop", "substitute", "repeater", "crossing"};
		for (int k = 0; k < static_cast<int>(tool.templates.size()); ++k)
		{
			auto const &item{tool.templates[k]};
			std::string aspects;
			std::string repeats;
			for (auto const &[name, called] : item.aspect_events)
				aspects += (aspects.empty() ? "" : " ") + name + "=" + called;
			for (auto const &[speed, called] : item.repeats)
				repeats += (repeats.empty() ? "" : " ") + speed + "=" + called;
			if (item.arms > 0 || false == item.aspect_events.empty() || false == item.repeats.empty() || values["all"] == "1")
				WriteLog(format("SELFTEST signaltemplates %s: %s %s lamps=%s arms=%d aspects=[%s] repeats=[%s] linked=%d read=%s", item.file.c_str(), item.kind.c_str(), item.mount.c_str(), item.lamps.empty() ? "-" : item.lamps.c_str(), item.arms, aspects.c_str(), repeats.c_str(), item.linked ? 1 : 0,
				                item.read.empty() ? "-" : item.read.c_str()));
			// the template named chosen for its kind
			if (auto const kind{std::ranges::find_if(kinds, [&item](char const *Kind) { return item.kind == Kind; })}; values.contains("file") && item.file == values["file"] && kind != kinds.end())
			{
				tool.kind = static_cast<int>(kind - kinds.begin());
				tool.chosen[tool.kind] = k;
			}
		}
		std::vector<signal_head> heads;
		signal_heads(heads, true);
		std::string known;
		for (auto const &head : heads)
			if (head.arms > 0 || false == head.repeats.empty())
				known += " " + head.name + (head.chosen ? "*" : "");
		WriteLog(format("SELFTEST signaltemplates: %zu templates, %zu for the signalling, of them by arms or repeats:", tool.templates.size(), heads.size()) + known);
		// what the signalling places of each kind, with no template asked for
		std::string placed;
		for (int i = 0; i < static_cast<int>(kinds.size()); ++i)
		{
			signal_order order;
			order.kind = kinds[i];
			auto const *item{signal_template_for(order)};
			auto const *chosen{tool.chosen[i] >= 0 ? &tool.templates[tool.chosen[i]] : nullptr};
			placed += format(" %s=%s(chosen %s)", kinds[i], item != nullptr ? item->file.c_str() : "-", chosen != nullptr ? chosen->file.c_str() : "-");
		}
		WriteLog("SELFTEST signaltemplates placed by the signalling:" + placed);
		return selftest_result::handled;
	}
	if (command == "signalaspect")
	{
		// signalaspect name=<signal> aspect=<name>: the signal set to the aspect its description lists, in the editor
		auto values{selftest_values(words)};
		show_track_tab(track_tab::signals);
		signal_scan(true);
		signal_refresh(true);
		auto const found{std::ranges::find_if(m_signal.standing, [&values](signal_standing const &Standing) { return Standing.name == values["name"]; })};
		auto const *item{found != m_signal.standing.end() ? signal_template_of(found->include.file) : nullptr};
		// of the aspects of the same name the last one, as the description lists them
		std::pair<std::string, std::string> const *aspect_event{nullptr};
		if (item != nullptr)
		{
			auto const backwards{item->aspect_events | std::views::reverse};
			if (auto const named{std::ranges::find_if(backwards, [&values](auto const &Aspect) { return Aspect.first == values["aspect"]; })}; named != backwards.end())
				aspect_event = &*named;
		}
		if (aspect_event == nullptr)
		{
			WriteLog("SELFTEST signalaspect: no signal " + values["name"] + " with the aspect " + values["aspect"]);
			return selftest_result::handled;
		}
		signal_select(found->include.instance);
		select_include(found->include.instance);
		auto const shown{signal_show_aspect(found->include.instance, aspect_event->first, aspect_event->second)};
		WriteLog(std::string{"SELFTEST signalaspect "} + values["name"] + " " + values["aspect"] + (shown ? ": " : ": failed, ") + m_signal.status);
		return selftest_result::handled;
	}
	if (command == "signal" || command == "signalmove" || command == "signaldelete" || command == "signalrename" || command == "signalprobe" || command == "signalflip" || command == "signalturn" || command == "signalselect" || command == "signalswap" ||
	    command == "signalread" || command == "signalreadflip" || command == "signalreaddrop" || command == "signalreaddefault" || command == "signalindicator" ||
	    command == "signalindicatordrop" || command == "signalgizmo" || command == "signalindicatorswap" || command == "signalacross")
	{
		auto values{selftest_values(words)};
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
					static char const *const kinds[] = {"main", "block", "warning", "shunt", "stop", "substitute", "repeater", "crossing"};
					for (int i = 0; i < 8; ++i)
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
			// index=<digits>, m=1: of the drawn plate
			std::snprintf(tool.plate_index, sizeof(tool.plate_index), "%s", values["index"].c_str());
			tool.plate_m = values["m"] == "1";
			auto const side{values["side"] == "left" ? -1 : 1};
			auto const facing{values["facing"] == "along" ? 1 : values["facing"] == "against" ? -1 : 0};
			// offset=<m>: from the axis, for this signal only
			auto const kept{EditorSettings.signal_offset()};
			if (values.count("offset"))
				EditorSettings.signal_offset(std::stod(values["offset"]));
			signal_spot spot;
			auto const found{spot_on(*signal_current(), facing, side, spot)};
			EditorSettings.signal_offset(kept);
			if (false == found)
			{
				WriteLog("SELFTEST signal: no such track " + values["track"]);
				return selftest_result::handled;
			}
			// snap=1: to a round distance to the switch ahead, as placing it by hand does
			if (values["snap"] == "1")
			{
				auto const before{signal_danger(spot)};
				signal_snap(spot, *signal_current());
				auto const after{signal_danger(spot)};
				WriteLog(format("SELFTEST signal snap: %.2f m -> %.2f m to %s, on %s path %d", before.distance, after.distance, after.track != nullptr ? after.track->name().c_str() : "-", spot.track != nullptr ? spot.track->name().c_str() : "-", spot.path));
			}
			auto const placed{signal_place(spot, *signal_current())};
			WriteLog(format("SELFTEST signal %s: %s", placed ? "placed" : "failed", tool.status.c_str()));
		}
		else if (command == "signalmove")
		{
			auto const *standing{standing_named(values["name"])};
			auto *track{simulation::Paths.find(values["track"])};
			if (standing == nullptr || track == nullptr || track->m_paths.empty())
			{
				WriteLog("SELFTEST signalmove: no signal " + values["name"] + " or no track " + values["track"]);
				return selftest_result::handled;
			}
			auto const path{std::clamp(values.count("path") ? std::stoi(values["path"]) : 0, 0, static_cast<int>(track->m_paths.size()) - 1)};
			auto const at{values.count("at") ? std::stod(values["at"]) : 0.5};
			geometry::bezier const curve{track->m_paths[path]};
			auto point{curve.point(at)};
			auto const sided{values.count("side") != 0};
			auto side{values["side"] == "left" ? -1 : 1};
			if (values.count("off"))
			{
				auto const tangent{glm::normalize(glm::dvec2{curve.first(at).x, curve.first(at).z})};
				auto const off{std::stod(values["off"])};
				point += glm::dvec3{-tangent.y * off, 0.0, tangent.x * off};
				side = 0;
			}
			else if (signal_spot home; false == sided && signal_standing_spot(*standing, home))
				side = home.side * (home.list == 2 ? 1 : -1);
			signal_select(standing->include.instance);
			tool.moving = true;
			tool.moved_valid = signal_drop_at(*standing, *track, path, point, side, tool.moved);
			// as a drag does: what it had by its own track carried
			if (signal_spot home; tool.moved_valid && signal_standing_spot(*standing, home))
				if (auto const *item{signal_template_of(standing->include.file)})
					signal_carry(home, tool.moved, *item);
			tool.pressed = {-1000.f, -1000.f};
			signal_release();
			WriteLog("SELFTEST signalmove: " + tool.status);
		}
		else if (command == "signalflip" || command == "signalturn")
		{
			auto const *standing{standing_named(values["name"])};
			auto const done{standing != nullptr && (command == "signalflip" ? signal_flip(standing->include.instance) : signal_turn(standing->include.instance))};
			WriteLog("SELFTEST " + command + " " + values["name"] + (done ? ": " : ": failed, ") + tool.status);
		}
		else if (command == "signalselect" || command == "signalswap")
		{
			auto const *standing{standing_named(values["name"])};
			if (standing == nullptr)
			{
				WriteLog("SELFTEST " + command + ": no signal " + values["name"]);
				return selftest_result::handled;
			}
			auto const instance{standing->include.instance};
			signal_select(instance);
			if (command == "signalswap")
			{
				auto const done{signal_swap(instance, values["file"])};
				WriteLog("SELFTEST signalswap " + values["name"] + (done ? ": " : ": failed, ") + tool.status);
			}
			else
				WriteLog("SELFTEST signalselect " + values["name"] + ": " + standing->include.file);
		}
		else if (command == "signalindicator" || command == "signalindicatordrop")
		{
			// signalindicator name=<signal> file=<indicator template>: the indicator put on it; signalindicatordrop name=<signal>
			// index=<n>: its indicator removed, the highest one 0
			auto const *standing{standing_named(values["name"])};
			if (standing == nullptr)
			{
				WriteLog("SELFTEST " + command + ": no signal " + values["name"]);
				return selftest_result::handled;
			}
			bool done{false};
			if (command == "signalindicator")
			{
				auto const *item{indicator_template_of(values["file"])};
				if (item == nullptr)
				{
					WriteLog("SELFTEST signalindicator: no indicator template " + values["file"]);
					return selftest_result::handled;
				}
				done = signal_indicator_add(*standing, *item);
			}
			else
			{
				auto const index{values.count("index") ? std::stoul(values["index"]) : 0ul};
				done = index < standing->indicators.size() && signal_indicator_remove(standing->indicators[index].instance);
			}
			WriteLog("SELFTEST " + command + " " + values["name"] + (done ? ": " : ": failed, ") + tool.status);
		}
		else if (command == "signalindicatorswap" || command == "signalacross")
		{
			// signalindicatorswap name=<signal> upper=<n> lower=<n>: two of its indicators under the lights swapped, the highest one 0;
			// signalacross name=<signal> [offset=<m>] [height=<m>]: set from the axis of its track and raised so. Each logs the signal
			// and its indicators from the top down after
			auto const *standing{standing_named(values["name"])};
			if (standing == nullptr)
			{
				WriteLog("SELFTEST " + command + ": no signal " + values["name"]);
				return selftest_result::handled;
			}
			auto const name{standing->name};
			bool done{false};
			if (command == "signalindicatorswap")
				done = signal_indicators_swap(*standing, std::stoul(values["upper"]), std::stoul(values["lower"]));
			else if (values.count("offset") > 0 || values.count("height") > 0)
				done = signal_set_across(*standing, values.count("offset") ? std::optional<double>{std::stod(values["offset"])} : std::nullopt, values.count("height") ? std::optional<double>{std::stod(values["height"])} : std::nullopt);
			WriteLog("SELFTEST " + command + " " + name + (done ? ": " : ": failed, ") + tool.status);
			signal_refresh(true);
			if (auto const *after{standing_named(name)})
			{
				signal_spot spot;
				auto const *item{signal_template_of(after->include.file)};
				if (signal_standing_spot(*after, spot) && item != nullptr)
					WriteLog(format("SELFTEST %s %s: %.3f m from the axis, zero %.3f m over the rail heads, raised %.3f m, ground %.3f m over the rail heads, at %.2f %.2f %.2f, the trains it faces run %.3f %.3f", command.c_str(), name.c_str(), geometry::plan_distance(after->include.location, spot.axis), after->include.location.y - spot.axis.y - 0.18, after->include.location.y - signal_default_height(*item, spot.axis, after->include.location), ground_at({after->include.location.x, spot.axis.y + 0.18, after->include.location.z}).y - spot.axis.y - 0.18, after->include.location.x, after->include.location.y, after->include.location.z, spot.travel.x, spot.travel.y));
				for (auto const &mark : after->indicators)
					WriteLog(format("SELFTEST %s %s indicator %s at %.3f m over the foot, %.3f m aside", command.c_str(), name.c_str(), mark.file.c_str(), mark.location.y - after->include.location.y, geometry::plan_distance(mark.location, after->include.location)));
			}
		}
		else if (command == "signalread" || command == "signalreadflip" || command == "signalreaddrop" || command == "signalreaddefault")
		{
			// signalread name=<signal> track=<path> [path=<n>] [add=1]: the path picked to read it, as a click (a Shift+click with add)
			// while picking; signalreadflip and signalreaddrop name=<signal> index=<n>: a read of its list turned the other way or
			// removed; signalreaddefault name=<signal>: the read where the editor puts it
			auto const *standing{standing_named(values["name"])};
			if (standing == nullptr)
			{
				WriteLog("SELFTEST " + command + ": no signal " + values["name"]);
				return selftest_result::handled;
			}
			signal_select(standing->include.instance);
			auto done{false};
			if (command == "signalread")
			{
				auto *track{simulation::Paths.find(values["track"])};
				tool.picking_read = true;
				done = track != nullptr && signal_read_pick(*track, values.count("path") ? std::stoi(values["path"]) : 0, values["add"] == "1");
			}
			else if (command == "signalreaddefault")
			{
				signal_spot spot;
				done = signal_standing_spot(*standing, spot) && spot.reader != nullptr && signal_read_set(*standing, {{spot.reader, spot.reader_list}});
			}
			else
			{
				auto reads{standing->reads};
				auto const index{values.count("index") ? static_cast<std::size_t>(std::stoul(values["index"])) : std::size_t{0}};
				if (index < reads.size())
				{
					if (command == "signalreaddrop")
						reads.erase(reads.begin() + static_cast<std::ptrdiff_t>(index));
					else
						reads[index].second = reads[index].second % 3 == 2 ? reads[index].second - 1 : reads[index].second + 1;
					done = signal_read_set(*standing, reads);
				}
			}
			WriteLog("SELFTEST " + command + " " + values["name"] + (done ? ": " : ": failed, ") + tool.status);
		}
		else if (command == "signaldelete")
		{
			auto const *standing{standing_named(values["name"])};
			auto const deleted{standing != nullptr && signal_delete(standing->include.instance)};
			WriteLog(std::string{"SELFTEST signaldelete "} + values["name"] + (deleted ? ": removed with its event" : ": not removed by the signal tool"));
		}
		else if (command == "signalgizmo")
		{
			// signalgizmo name=<signal> turn=<degrees> x=<metres> z=<metres>: the signal turned and moved by the gizmo of its include
			// as one drag, then let go
			auto const *standing{standing_named(values["name"])};
			if (standing == nullptr)
			{
				WriteLog("SELFTEST signalgizmo: no signal " + values["name"]);
				return selftest_result::handled;
			}
			select_include(standing->include.instance);
			m_include_gesture = false;
			signal_gizmo_begin();
			auto const shifted = [&](std::size_t const Index, char const *Key) {
				if (values.count(Key) != 0 && Index < m_include.values.size())
					m_include.values[Index] = editor_includes::number(std::strtod(m_include.values[Index].c_str(), nullptr) + std::stod(values[Key]));
			};
			shifted(1, "x");
			shifted(3, "z");
			shifted(4, "turn");
			apply_include();
			signal_gizmo_step();
			signal_gizmo_end();
			m_include_gesture = false;
			WriteLog("SELFTEST signalgizmo " + values["name"] + ": " + m_signal.status);
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
			auto const only{ToLower(values["name"])};
			for (auto const &standing : tool.standing)
			{
				if (false == only.empty() && ToLower(standing.name) != only)
					continue;
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
					// the path it stands by, which on a switch past the frog isn't the one reading it
					if (standing.reads.size() > 1)
					{
						where += "; also";
						for (std::size_t r = 1; r < standing.reads.size(); ++r)
							where += format(" %s of %s", editor_track::event_keyword(standing.reads[r].second), standing.reads[r].first->name().c_str());
					}
					if (signal_spot spot; signal_standing_spot(standing, spot) && spot.track != nullptr)
						where += format("; stands by %s path %d, %.3f m from its axis", spot.track->name().c_str(), spot.path, geometry::plan_distance(spot.axis, standing.include.location));
					// the switch ahead of it, as its card shows it
					if (auto const &metre{signal_metre_of(standing)}; metre.valid)
						where += format("; offset %.2f m; switch ahead: %s %s %.2f m at %.2f %.2f, axes %.3f m apart", metre.spot.offset,
						                metre.danger.what == track_trace::danger::kind::start ? "start" : metre.danger.what == track_trace::danger::kind::fouling ? "fouling" : "none",
						                metre.danger.track != nullptr ? metre.danger.track->name().c_str() : "-", metre.danger.distance, metre.danger.point.x, metre.danger.point.z, metre.danger.spacing);
				}
				// its indicators, over its foot and from its axis, and its lowest lamp
				for (auto const &mark : standing.indicators)
					where += format("; indicator %s %.3f m up, %.3f m off", mark.file.c_str(), mark.location.y - standing.include.location.y, geometry::plan_distance(mark.location, standing.include.location));
				if (auto const lamp{signal_lowest_lamp(standing.include.instance)}; lamp > 0.0)
					where += format("; lowest lamp %.3f m up", lamp - standing.include.location.y);
				// the gauge of the tracks around it
				if (auto const hit{signal_gauge(standing.include.instance)}; hit.depth > 0.0 && hit.track != nullptr)
					where += format("; in the gauge of %s path %d by %.3f m at %.2f %.2f %.2f", hit.track->name().c_str(), hit.path, hit.depth, hit.point.x, hit.point.y, hit.point.z);
				else
					where += "; out of the gauge";
				WriteLog(format("SELFTEST signalprobe %s %s at %.3f %.3f %.3f yaw %.2f read %s: %s", standing.name.c_str(), standing.include.file.c_str(), standing.include.location.x, standing.include.location.y, standing.include.location.z, standing.include.yaw,
				                standing.read.empty() ? "-" : standing.read.c_str(), where.c_str()));
			}
		}
		return selftest_result::handled;
	}
	return selftest_result::unhandled;
}
