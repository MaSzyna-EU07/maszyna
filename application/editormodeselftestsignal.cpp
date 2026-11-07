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

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <sstream>
#include "application/editormodeselftest.h"

namespace
{

// the commands of the signal self-test
bool signal_command(std::string const &Command)
{
	static std::array<char const *, 9> const commands{"signal", "signalmove", "signaldelete", "signalrename", "signalprobe", "signalflip", "signalturn", "signalselect", "signalswap"};
	return std::ranges::any_of(commands, [&Command](char const *Name) { return Command == Name; });
}

// the signal of the name among the ones standing, null when there's none
template <typename Standing>
Standing const *standing_named(std::vector<Standing> const &Standings, std::string const &Name)
{
	auto const found{std::ranges::find_if(Standings, [&Name](Standing const &Each) { return Each.name == Name; })};
	return found != Standings.end() ? std::to_address(found) : nullptr;
}

// the text into the buffer, as much of it as fits with the terminating zero
void copy_text(std::span<char> const Buffer, char const *Text)
{
	*std::format_to_n(Buffer.data(), static_cast<std::ptrdiff_t>(Buffer.size()) - 1, "{}", Text).out = '\0';
}

// the kind of a signal template as the tool counts them, -1 for none of them
int signal_kind_index(std::string const &Kind)
{
	static std::array<char const *, 7> const kinds{"main", "block", "warning", "shunt", "stop", "substitute", "repeater"};
	for (int i = 0; i < static_cast<int>(kinds.size()); ++i)
		if (Kind == kinds[i])
			return i;
	return -1;
}

// the pairs of a list as name=value, one space apart
std::string joined_pairs(std::vector<std::pair<std::string, std::string>> const &Pairs)
{
	std::string text;
	for (auto const &[name, value] : Pairs)
		text += (text.empty() ? "" : " ") + name + "=" + value;
	return text;
}

// a signal template as the self-test of the templates writes it out
template <typename Template>
std::string template_line(Template const &Item)
{
	return format("SELFTEST signaltemplates %s: %s %s lamps=%s arms=%d aspects=[%s] repeats=[%s] linked=%d read=%s", Item.file.c_str(), Item.kind.c_str(), Item.mount.c_str(), Item.lamps.empty() ? "-" : Item.lamps.c_str(), Item.arms, joined_pairs(Item.aspect_events).c_str(),
	              joined_pairs(Item.repeats).c_str(), Item.linked ? 1 : 0, Item.read.empty() ? "-" : Item.read.c_str());
}

// the facing a value names: 1 along the track, -1 against it, 0 the one the tool picks
int facing_named(std::string const &Facing)
{
	if (Facing == "along")
		return 1;
	if (Facing == "against")
		return -1;
	return 0;
}

// the parameter of the curve nearest to the point in the plan, out of 1024 steps
double nearest_parameter(geometry::bezier const &Curve, glm::dvec3 const &Point)
{
	auto best{std::numeric_limits<double>::max()};
	double parameter{0.0};
	for (int k = 0; k <= 1024; ++k)
	{
		auto const d{geometry::plan_distance(Curve.point(k / 1024.0), Point)};
		if (d < best)
		{
			best = d;
			parameter = k / 1024.0;
		}
	}
	return parameter;
}

// where a signal stands by the track it reads: whether its event is bound there, its side and its offset from the axis, the
// turn of its model from the travel
std::string probe_where(TTrack &Track, int const List, standing_template const &Include, std::string const &Read)
{
	auto const &curve{Track.m_paths.front()};
	geometry::bezier const bez{curve};
	auto const parameter{nearest_parameter(bez, Include.location)};
	auto const axis{bez.point(parameter)};
	auto tangent{bez.first(parameter)};
	glm::dvec2 travel{tangent.x, tangent.z};
	travel = glm::normalize(travel) * (List == 1 ? -1.0 : 1.0);
	glm::dvec2 const offset{Include.location.x - axis.x, Include.location.z - axis.z};
	auto const cross{travel.x * offset.y - travel.y * offset.x};
	auto heading{glm::degrees(std::atan2(travel.x, travel.y))};
	auto turn{std::fmod(Include.yaw - heading + 720.0, 360.0)};
	auto const &list{editor_track::events(Track, List)};
	auto const bound{std::ranges::any_of(list, [&Read](auto const &Event) { return Event.second != nullptr && ToLower(Event.first) == ToLower(Read); })};
	return std::string{bound ? "bound, " : "unbound, "} + format("%s of %s, %.3f m from the axis, %s of the trains, %.3f m over the axis, model turned %.1f deg from the travel", editor_track::event_keyword(List), Track.name().c_str(), glm::length(offset), cross > 0.0 ? "right" : "left",
	                                                             Include.location.y - axis.y, turn);
}

} // namespace

editor_mode::selftest_result editor_mode::selftest_signal_templates(std::istringstream &words)
{
	// signaltemplates [file=<template>]: the templates of the signals with what their descriptions give, the one named chosen
	// for its kind in the palette, not placed
	auto values{selftest_values(words)};
	show_track_tab(track_tab::srk);
	signal_scan(true);
	auto &tool{m_signal};
	static std::array<char const *, 7> constexpr kinds{"main", "block", "warning", "shunt", "stop", "substitute", "repeater"};
	for (int k = 0; k < static_cast<int>(tool.templates.size()); ++k)
	{
		auto const &item{tool.templates[k]};
		if (item.arms > 0 || false == item.aspect_events.empty() || false == item.repeats.empty() || values["all"] == "1")
			WriteLog(template_line(item));
		// the template named chosen for its kind
		if (auto const kind{signal_kind_index(item.kind)}; values.contains("file") && item.file == values["file"] && kind >= 0)
		{
			tool.kind = kind;
			tool.chosen[kind] = k;
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

editor_mode::selftest_result editor_mode::selftest_signal_aspect(std::istringstream &words)
{
	// signalaspect name=<signal> aspect=<name>: the signal set to the aspect its description lists, in the editor
	auto values{selftest_values(words)};
	show_track_tab(track_tab::srk);
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

editor_mode::selftest_result editor_mode::selftest_signal(std::string const &command, std::istringstream &words)
{
	if (command == "signaltemplates")
		return selftest_signal_templates(words);
	if (command == "signalaspect")
		return selftest_signal_aspect(words);
	if (false == signal_command(command))
		return selftest_result::unhandled;
	auto values{selftest_values(words)};
	show_track_tab(track_tab::srk);
	signal_scan();
	signal_refresh(true);
	auto &tool{m_signal};
	if (command == "signal")
		selftest_signal_place(values);
	else if (command == "signalmove")
		selftest_signal_move(values);
	else if (command == "signalflip" || command == "signalturn")
	{
		auto const *standing{standing_named(tool.standing, values["name"])};
		auto const done{standing != nullptr && (command == "signalflip" ? signal_flip(standing->include.instance) : signal_turn(standing->include.instance))};
		WriteLog("SELFTEST " + command + " " + values["name"] + (done ? ": " : ": failed, ") + tool.status);
	}
	else if (command == "signalselect" || command == "signalswap")
		selftest_signal_select(command, values);
	else if (command == "signaldelete")
	{
		auto const *standing{standing_named(tool.standing, values["name"])};
		auto const deleted{standing != nullptr && signal_delete(standing->include.instance)};
		WriteLog(std::string{"SELFTEST signaldelete "} + values["name"] + (deleted ? ": removed with its event" : ": not removed by the signal tool"));
	}
	else if (command == "signalrename")
	{
		auto const *standing{standing_named(tool.standing, values["name"])};
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
		selftest_signal_probe();
	return selftest_result::handled;
}

void editor_mode::selftest_signal_place(std::map<std::string, std::string, std::less<>> &values)
{
	auto &tool{m_signal};
	auto const *item{signal_template_of(values["file"])};
	if (item == nullptr)
	{
		WriteLog("SELFTEST signal: no signal template " + values["file"]);
		return;
	}
	// the template chosen as the one of its kind
	auto const found{std::ranges::find_if(tool.templates, [item](signal_template const &Template) { return &Template == item; })};
	if (auto const kind{signal_kind_index(item->kind)}; found != tool.templates.end() && kind >= 0)
	{
		tool.kind = kind;
		tool.chosen[kind] = static_cast<int>(std::distance(tool.templates.begin(), found));
	}
	if (values.contains("name"))
		copy_text(tool.name, values["name"].c_str());
	if (values.contains("plate"))
		copy_text(tool.plate, values["plate"].c_str());
	auto const side{values["side"] == "left" ? -1 : 1};
	auto const facing{facing_named(values["facing"])};
	auto const &chosen{*signal_current()};
	signal_spot spot;
	auto *track{simulation::Paths.find(values["track"])};
	if (track == nullptr)
	{
		WriteLog("SELFTEST signal: no such track " + values["track"]);
		return;
	}
	auto const path{values.contains("path") ? std::stoi(values["path"]) : 0};
	auto const at{values.contains("at") ? std::stod(values["at"]) : 0.5};
	geometry::bezier const curve{track->m_paths[std::clamp(path, 0, static_cast<int>(track->m_paths.size()) - 1)]};
	if (false == signal_spot_at(*track, path, curve.point(at), curve.point(at), facing, side, chosen, spot))
	{
		WriteLog("SELFTEST signal: no such track " + values["track"]);
		return;
	}
	auto const placed{signal_place(spot, *signal_current())};
	WriteLog(format("SELFTEST signal %s: %s", placed ? "placed" : "failed", tool.status.c_str()));
}

void editor_mode::selftest_signal_move(std::map<std::string, std::string, std::less<>> &values)
{
	auto &tool{m_signal};
	auto const *standing{standing_named(tool.standing, values["name"])};
	auto *track{simulation::Paths.find(values["track"])};
	if (standing == nullptr || track == nullptr || track->m_paths.empty())
	{
		WriteLog("SELFTEST signalmove: no signal " + values["name"] + " or no track " + values["track"]);
		return;
	}
	auto const path{std::clamp(values.contains("path") ? std::stoi(values["path"]) : 0, 0, static_cast<int>(track->m_paths.size()) - 1)};
	auto const at{values.contains("at") ? std::stod(values["at"]) : 0.5};
	geometry::bezier const curve{track->m_paths[path]};
	auto point{curve.point(at)};
	auto const sided{values.contains("side")};
	auto side{values["side"] == "left" ? -1 : 1};
	if (values.contains("off"))
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
	tool.pressed = {-1000.f, -1000.f};
	signal_release();
	WriteLog("SELFTEST signalmove: " + tool.status);
}

void editor_mode::selftest_signal_select(std::string const &command, std::map<std::string, std::string, std::less<>> &values)
{
	auto const *standing{standing_named(m_signal.standing, values["name"])};
	if (standing == nullptr)
	{
		WriteLog("SELFTEST " + command + ": no signal " + values["name"]);
		return;
	}
	auto const instance{standing->include.instance};
	signal_select(instance);
	if (command == "signalswap")
	{
		auto const done{signal_swap(instance, values["file"])};
		WriteLog("SELFTEST signalswap " + values["name"] + (done ? ": " : ": failed, ") + m_signal.status);
	}
	else
		WriteLog("SELFTEST signalselect " + values["name"] + ": " + standing->include.file);
}

void editor_mode::selftest_signal_probe()
{
	auto &tool{m_signal};
	signal_refresh(true);
	WriteLog(format("SELFTEST signalprobe: %zu signals", tool.standing.size()));
	for (auto const &standing : tool.standing)
	{
		std::string where{"not read"};
		if (standing.track != nullptr)
			where = probe_where(*standing.track, standing.list, standing.include, standing.read);
		WriteLog(format("SELFTEST signalprobe %s %s at %.3f %.3f %.3f yaw %.2f read %s: %s", standing.name.c_str(), standing.include.file.c_str(), standing.include.location.x, standing.include.location.y, standing.include.location.z, standing.include.yaw,
		                standing.read.empty() ? "-" : standing.read.c_str(), where.c_str()));
	}
}
