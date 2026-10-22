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
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"
#include "editor/editorSettings.hpp"

#include "rendering/renderer.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace
{

using geometry::bezier;
using geometry::heading_of;
using geometry::plan_of;

double const kRailTop{0.18};
double const kSignalRange{1500.0};
double const kSignalHead{4.0};
float const kSignalPick{12.0f};
float const kDragStart{6.0f};
double const kDragTravel{1.0};
double const kRefreshPeriod{2.0};
double const kSignalReach{10.0};

char const *const kKinds[] = {"main", "block", "warning", "shunt", "stop", "substitute", "repeater"};
char const *const kKindLabels[] = {"Semaphore", "Automatic block", "Warning signal", "Shunting signal", "Stop signal", "Substitute signal", "Repeater"};
int const kKindCount{7};

int kind_index(std::string const &Kind)
{
	for (int i = 0; i < kKindCount; ++i)
		if (Kind == kKinds[i])
			return i;
	return -1;
}

double nearest_on(TTrack const &Track, int const Path, glm::dvec3 const &Point)
{
	bezier const curve{Track.m_paths[Path]};
	auto parameter{0.0};
	auto best{std::numeric_limits<double>::max()};
	for (int k = 0; k <= 256; ++k)
	{
		auto const distance{geometry::plan_distance(curve.point(k / 256.0), Point)};
		if (distance < best)
		{
			best = distance;
			parameter = k / 256.0;
		}
	}
	return parameter;
}

glm::dvec2 forward_at(TTrack const &Track, int const Path, double const Parameter)
{
	auto const tangent{bezier{Track.m_paths[Path]}.first(Parameter)};
	glm::dvec2 const planar{tangent.x, tangent.z};
	return glm::length(planar) > 1e-9 ? glm::normalize(planar) : glm::dvec2{0.0, 1.0};
}

glm::dvec2 right_of(glm::dvec2 const &Direction)
{
	return {-Direction.y, Direction.x};
}

glm::dvec2 travel_of(double const Yaw)
{
	auto const yaw{glm::radians(Yaw + 180.0)};
	return {std::sin(yaw), std::cos(yaw)};
}

int travel_side(int const Side, int const List)
{
	return Side * (List == 2 ? 1 : -1);
}

int lean_wanted(int const Side)
{
	return Side > 0 ? 1 : 2;
}

int lean_of(std::string const &Lean)
{
	return Lean == "left" ? 1 : Lean == "right" ? 2 : 0;
}

std::size_t edit_distance(std::string const &A, std::string const &B)
{
	std::vector<std::size_t> row(B.size() + 1);
	for (std::size_t j = 0; j <= B.size(); ++j)
		row[j] = j;
	for (std::size_t i = 1; i <= A.size(); ++i)
	{
		auto diagonal{row[0]};
		row[0] = i;
		for (std::size_t j = 1; j <= B.size(); ++j)
		{
			auto const above{row[j]};
			row[j] = std::min({row[j] + 1, row[j - 1] + 1, diagonal + (A[i - 1] == B[j - 1] ? 0 : 1)});
			diagonal = above;
		}
	}
	return row[B.size()];
}

ImU32 lamp_colour(char const Lamp)
{
	switch (Lamp)
	{
	case 'z': return IM_COL32(40, 210, 80, 255);
	case 'c': return IM_COL32(230, 40, 40, 255);
	case 'p': return IM_COL32(255, 160, 30, 255);
	case 'b': return IM_COL32(235, 235, 235, 255);
	case 'n': return IM_COL32(60, 110, 255, 255);
	default: return IM_COL32(70, 70, 70, 255);
	}
}

std::string group_of(std::string const &Mount, std::string const &Lamps)
{
	return Mount + "|" + Lamps;
}

std::vector<std::string> const &plate_textures()
{
	static std::vector<std::string> plates;
	static bool scanned{false};
	if (scanned)
		return plates;
	scanned = true;
	std::error_code error;
	for (std::filesystem::directory_iterator file{"textures/tabl", error}, end; file != end; file.increment(error))
	{
		std::string stem;
		try
		{
			stem = ToLower(file->path().stem().string());
		}
		catch (std::exception const &)
		{
			continue;
		}
		if (std::find(plates.begin(), plates.end(), stem) == plates.end())
			plates.push_back(stem);
	}
	std::sort(plates.begin(), plates.end());
	return plates;
}

std::string plate_for(std::string const &Plate, std::string const &Name)
{
	if (false == Plate.empty())
		return Plate;
	auto const &plates{plate_textures()};
	auto const name{ToLower(Name)};
	return std::binary_search(plates.begin(), plates.end(), name) ? name : std::string{"0_nothing"};
}

void draw_signal_icon(ImDrawList *Draw, ImVec2 const Origin, ImVec2 const Size, std::string const &Mount, std::string const &Lamps, ImU32 const Frame)
{
	auto const lamps{std::max<int>(1, static_cast<int>(Lamps.size()))};
	auto const centre{Origin.x + Size.x * 0.5f};
	auto const bottom{Origin.y + Size.y - 4.0f};
	auto const lamp{std::min(7.0f, (Size.y - 26.0f) / (lamps * 2.2f))};
	auto const headwidth{lamp * 2.0f + 6.0f};
	auto const headheight{lamps * lamp * 2.2f + 4.0f};
	ImU32 const pole{IM_COL32(150, 150, 150, 255)};
	float headtop;
	if (Mount == "dwarf")
	{
		headtop = bottom - headheight;
		Draw->AddRectFilled(ImVec2(centre - headwidth * 0.7f, bottom - 3.0f), ImVec2(centre + headwidth * 0.7f, bottom), pole);
	}
	else if (Mount == "gantry")
	{
		headtop = Origin.y + 10.0f;
		Draw->AddLine(ImVec2(Origin.x + 4.0f, Origin.y + 6.0f), ImVec2(Origin.x + Size.x - 4.0f, Origin.y + 6.0f), pole, 3.0f);
		Draw->AddLine(ImVec2(centre, Origin.y + 6.0f), ImVec2(centre, headtop), pole, 2.0f);
	}
	else
	{
		headtop = Origin.y + 6.0f;
		Draw->AddLine(ImVec2(centre, headtop + headheight), ImVec2(centre, bottom), pole, 3.0f);
		Draw->AddLine(ImVec2(centre - 6.0f, bottom), ImVec2(centre + 6.0f, bottom), pole, 2.0f);
	}
	if (Lamps.empty())
	{
		Draw->AddLine(ImVec2(centre, headtop), ImVec2(centre, headtop + headheight), pole, 3.0f);
		Draw->AddLine(ImVec2(centre, headtop + 6.0f), ImVec2(centre + 14.0f, headtop - 2.0f), IM_COL32(230, 60, 60, 255), 4.0f);
		return;
	}
	Draw->AddRectFilled(ImVec2(centre - headwidth * 0.5f, headtop), ImVec2(centre + headwidth * 0.5f, headtop + headheight), IM_COL32(25, 25, 25, 255), 3.0f);
	Draw->AddRect(ImVec2(centre - headwidth * 0.5f, headtop), ImVec2(centre + headwidth * 0.5f, headtop + headheight), Frame, 3.0f);
	for (int i = 0; i < static_cast<int>(Lamps.size()); ++i)
		Draw->AddCircleFilled(ImVec2(centre, headtop + 2.0f + lamp * 1.1f + i * lamp * 2.2f), lamp, lamp_colour(Lamps[i]));
}

} // namespace

bool editor_mode::signal_scan(bool const Wait)
{
	auto &tool{m_signal};
	if (Wait)
		EditorIncludes.wait();
	else
		EditorIncludes.start();
	if (false == EditorIncludes.scanned() || tool.revision == EditorIncludes.revision())
		return false;
	tool.revision = EditorIncludes.revision();
	std::string chosen[kKindCount];
	for (int i = 0; i < kKindCount; ++i)
		if (tool.chosen[i] >= 0 && tool.chosen[i] < static_cast<int>(tool.templates.size()))
			chosen[i] = tool.templates[tool.chosen[i]].file;
	tool.templates.clear();
	for (auto const &entry : EditorIncludes.entries())
	{
		if (entry.category != "signal" || false == entry.complete)
			continue;
		include_info info;
		std::string error;
		if (false == editor_includes::load(entry.file, info, error) || kind_index(info.signal_kind) < 0)
			continue;
		signal_template item;
		item.file = entry.file;
		item.name = info.name.empty() ? entry.file : info.name;
		item.description = info.description;
		item.kind = info.signal_kind;
		item.mount = info.signal_mount.empty() ? std::string{"mast"} : info.signal_mount;
		item.lamps = info.signal_lamps;
		item.lean = info.signal_lean;
		item.read = info.signal_read;
		for (auto const &parameter : info.parameters)
		{
			item.parameters = std::max(item.parameters, parameter.id);
			if (parameter.id == 6 && parameter.role == "texture")
				item.plate = true;
			if (parameter.id == 7)
				item.linked = true;
		}
		editor_includes::signal_speeds(entry.file, item.speeds);
		if (item.kind == "block")
			item.aspects = editor_includes::block_aspects(entry.file);
		tool.templates.push_back(std::move(item));
	}
	std::sort(tool.templates.begin(), tool.templates.end(), [](signal_template const &A, signal_template const &B) {
		return std::make_tuple(kind_index(A.kind), A.lamps.empty(), A.mount != "mast", A.lamps.size(), A.lamps, A.name.size(), A.name) < std::make_tuple(kind_index(B.kind), B.lamps.empty(), B.mount != "mast", B.lamps.size(), B.lamps, B.name.size(), B.name);
	});
	for (int i = 0; i < kKindCount; ++i)
	{
		tool.chosen[i] = -1;
		std::unordered_map<std::string, int> sizes;
		for (auto const &item : tool.templates)
			if (item.kind == kKinds[i])
				++sizes[group_of(item.mount, item.lamps)];
		auto best{-1};
		for (int k = 0; k < static_cast<int>(tool.templates.size()); ++k)
		{
			auto const &item{tool.templates[k]};
			if (item.kind != kKinds[i])
				continue;
			if (item.file == chosen[i])
			{
				best = k;
				break;
			}
			if (best < 0 || sizes[group_of(item.mount, item.lamps)] > sizes[group_of(tool.templates[best].mount, tool.templates[best].lamps)])
				best = k;
		}
		tool.chosen[i] = best;
	}
	return true;
}

editor_mode::signal_template const *editor_mode::signal_armed() const
{
	auto const &tool{m_signal};
	if (tool.kind < 0 || tool.kind >= kKindCount)
		return nullptr;
	auto const index{tool.chosen[tool.kind]};
	return index >= 0 && index < static_cast<int>(tool.templates.size()) ? &tool.templates[index] : nullptr;
}

editor_mode::signal_template const *editor_mode::signal_template_of(std::string const &File) const
{
	auto const file{ToLower(File)};
	for (auto const &item : m_signal.templates)
		if (item.file == file)
			return &item;
	return nullptr;
}

bool editor_mode::signal_spot_at(TTrack &Track, int const Path, glm::dvec3 const &Point, glm::dvec3 const &Cursor, int const Facing, int const Side, signal_template const &Template, signal_spot &Spot) const
{
	if (Track.m_paths.empty() || Track.m_editorremoved)
		return false;
	auto const path{std::clamp(Path, 0, static_cast<int>(Track.m_paths.size()) - 1)};
	auto const parameter{nearest_on(Track, path, Point)};
	auto const axis{bezier{Track.m_paths[path]}.point(parameter)};
	auto const forward{forward_at(Track, path, parameter)};
	auto const right{right_of(forward)};
	auto side{Side};
	if (side == 0)
		side = glm::dot(plan_of(Cursor) - plan_of(axis), right) >= 0.0 ? 1 : -1;
	auto facing{Facing};
	if (facing == 0)
		facing = side;
	Spot.track = &Track;
	Spot.path = path;
	Spot.axis = axis;
	Spot.travel = forward * static_cast<double>(facing);
	Spot.side = side;
	Spot.list = facing > 0 ? 2 : 1;
	Spot.offset = Template.mount == "gantry" ? 0.0 : EditorSettings.signal_offset();
	auto const across{right * static_cast<double>(side) * Spot.offset};
	Spot.position = {axis.x + across.x, axis.y + kRailTop + (Template.mount == "gantry" ? static_cast<double>(m_signal.height) : 0.0), axis.z + across.y};
	auto const yaw{heading_of(Spot.travel) + 180.0};
	Spot.yaw = yaw >= 360.0 ? yaw - 360.0 : yaw;
	return true;
}

std::string editor_mode::signal_intent_label(signal_template const &Template, signal_spot const &Spot) const
{
	auto const name{m_signal.name[0] != '\0' ? std::string{m_signal.name} : std::string{"?"}};
	auto const right{glm::dot(right_of(Spot.travel), plan_of(Spot.position) - plan_of(Spot.axis)) >= 0.0};
	auto const where{right ? STR(" on the right of the trains") : STR(" on the left of the trains")};
	auto variant{signal_variant(Template, Spot)};
	variant = variant.substr(0, variant.rfind('.'));
	if (Template.read.empty())
		return format(STR_C("Click: %s %s (%s)%s, the train doesn't read it"), STR(kKindLabels[kind_index(Template.kind)]).c_str(), name.c_str(), variant.c_str(), where.c_str()) + "\n" + STR("Drag along: the trains it faces run the way of the drag");
	auto const event{editor_includes::substitute(Template.read, {name})};
	auto const path{name_of(*Spot.track)};
	return format(STR_C("Click: %s %s (%s)%s, read by %s %s of %s"), STR(kKindLabels[kind_index(Template.kind)]).c_str(), name.c_str(), variant.c_str(), where.c_str(), editor_track::event_keyword(Spot.list), event.c_str(), path.c_str()) + "\n" + STR("Drag along: the trains it faces run the way of the drag");
}

void editor_mode::signal_refresh(bool const Force)
{
	auto &tool{m_signal};
	auto const scanned{signal_scan(false)};
	auto const now{ImGui::GetTime()};
	if (false == Force && false == scanned && tool.standing_history == m_history.size() && tool.standing_redo == g_redo.size() && now - tool.standing_time < kRefreshPeriod)
		return;
	tool.standing_history = m_history.size();
	tool.standing_redo = g_redo.size();
	tool.standing_time = now;
	tool.standing.clear();
	if (scene::Layers.empty())
		return;
	auto const found{standing_templates([this](std::string const &File) { return signal_template_of(File) != nullptr; })};
	std::unordered_map<std::string, std::pair<TTrack *, int>> readers;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		for (int list = 0; list < 6; ++list)
			for (auto const &event : editor_track::events(*track, list))
				if (false == event.first.empty())
					readers.emplace(ToLower(event.first), std::make_pair(track, list));
	}
	for (auto const &include : found)
	{
		auto const *item{signal_template_of(include.file)};
		signal_standing standing;
		standing.include = include;
		standing.kind = kind_index(item->kind);
		standing.name = include.values.empty() ? std::string{} : include.values.front();
		if (false == item->read.empty())
		{
			standing.read = editor_includes::substitute(item->read, include.values);
			if (auto const reader{readers.find(ToLower(standing.read))}; reader != readers.end())
			{
				standing.track = reader->second.first;
				standing.list = reader->second.second;
			}
		}
		tool.standing.push_back(std::move(standing));
	}
	if (tool.selected != 0 && std::none_of(tool.standing.begin(), tool.standing.end(), [&](signal_standing const &Standing) { return Standing.include.instance == tool.selected; }))
		tool.selected = 0;
}

int editor_mode::signal_hit() const
{
	screen_projection const projection;
	auto const &mouse{ImGui::GetIO().MousePos};
	auto best{kSignalPick * kSignalPick};
	int result{-1};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (int i = 0; i < static_cast<int>(m_signal.standing.size()); ++i)
	{
		auto const &location{m_signal.standing[i].include.location};
		if (glm::distance(location, camera) > kSignalRange)
			continue;
		ImVec2 foot, head;
		if (false == projection.project(location, foot) || false == projection.project(location + glm::dvec3{0.0, kSignalHead, 0.0}, head))
			continue;
		ImVec2 const span{head.x - foot.x, head.y - foot.y};
		auto const length2{span.x * span.x + span.y * span.y};
		auto const t{length2 > 1e-6f ? std::clamp(((mouse.x - foot.x) * span.x + (mouse.y - foot.y) * span.y) / length2, 0.0f, 1.0f) : 0.0f};
		auto const dx{foot.x + span.x * t - mouse.x};
		auto const dy{foot.y + span.y * t - mouse.y};
		auto const distance2{dx * dx + dy * dy};
		if (distance2 < best)
		{
			best = distance2;
			result = i;
		}
	}
	return result;
}

void editor_mode::signal_press(track_intent const &Intent)
{
	auto &tool{m_signal};
	auto const &mouse{ImGui::GetIO().MousePos};
	tool.pressed = {mouse.x, mouse.y};
	tool.facing = 0;
	tool.status.clear();
	if (Intent.what == track_intent::kind::signal_move)
	{
		if (Intent.index < 0 || Intent.index >= static_cast<int>(tool.standing.size()))
			return;
		auto const &standing{tool.standing[Intent.index]};
		tool.grab = plan_of(standing.include.location) - plan_of(cursor_level(standing.include.location.y));
		tool.home_valid = signal_standing_spot(standing, tool.home);
		tool.moving = true;
		tool.moved_valid = false;
		signal_select(standing.include.instance);
		return;
	}
	auto const *armed{signal_armed()};
	if (armed == nullptr || Intent.track == nullptr)
		return;
	if (false == signal_spot_at(*Intent.track, Intent.path, m_hover.point, cursor_level(m_hover.point.y), 0, 0, *armed, tool.press))
		return;
	tool.placing = true;
}

void editor_mode::signal_move_update()
{
	auto &tool{m_signal};
	auto const &mouse{ImGui::GetIO().MousePos};
	auto const dragged{glm::length(glm::vec2{mouse.x, mouse.y} - tool.pressed) > kDragStart};
	if (tool.placing)
	{
		auto const *armed{signal_armed()};
		if (armed == nullptr || tool.press.track == nullptr || tool.press.track->m_editorremoved)
		{
			tool.placing = false;
			return;
		}
		if (false == dragged)
			return;
		auto const cursor{cursor_level(tool.press.axis.y)};
		auto const forward{tool.press.travel * static_cast<double>(tool.press.list == 2 ? 1 : -1)};
		auto const along{glm::dot(plan_of(cursor) - plan_of(tool.press.axis), forward)};
		if (std::abs(along) < kDragTravel)
			return;
		tool.facing = along > 0.0 ? 1 : -1;
		signal_spot spot;
		if (signal_spot_at(*tool.press.track, tool.press.path, tool.press.axis, cursor, tool.facing, tool.press.side, *armed, spot))
			tool.press = spot;
		return;
	}
	if (false == tool.moving)
		return;
	if (false == dragged)
		return;
	auto const *found{signal_standing_of(tool.selected)};
	if (found == nullptr)
	{
		tool.moved_valid = false;
		return;
	}
	auto const &location{found->include.location};
	auto const cursor{cursor_level(location.y)};
	glm::dvec3 const held{cursor.x + tool.grab.x, location.y, cursor.z + tool.grab.y};
	// the track under the cursor; off every track, as it is when the signal is dragged beside its track, the track the
	// signal stands by
	auto *track{m_hover.track};
	auto path{m_hover.path};
	if (track == nullptr && tool.home_valid)
	{
		track = tool.home.track;
		path = tool.home.path;
	}
	tool.moved_valid = track != nullptr && signal_drop_at(*found, *track, path, held, 0, tool.moved);
}

editor_mode::signal_standing const *editor_mode::signal_standing_of(scene::instance_handle const Instance) const
{
	auto const &standing{m_signal.standing};
	auto const found{std::find_if(standing.begin(), standing.end(), [&](signal_standing const &Standing) { return Standing.include.instance == Instance; })};
	return found != standing.end() ? &*found : nullptr;
}

bool editor_mode::signal_drop_at(signal_standing const &Standing, TTrack &Track, int const Path, glm::dvec3 const &Point, int const Side, signal_spot &Spot) const
{
	auto const *item{signal_template_of(Standing.include.file)};
	if (item == nullptr || Track.m_paths.empty() || Track.m_editorremoved)
		return false;
	auto const path{std::clamp(Path, 0, static_cast<int>(Track.m_paths.size()) - 1)};
	auto const forward{forward_at(Track, path, nearest_on(Track, path, Point))};
	auto const facing{glm::dot(forward, travel_of(Standing.include.yaw)) >= 0.0 ? 1 : -1};
	return signal_spot_at(Track, path, Point, Point, facing, Side * facing, *item, Spot);
}

bool editor_mode::signal_standing_spot(signal_standing const &Standing, signal_spot &Spot) const
{
	auto const &location{Standing.include.location};
	TTrack *track{nullptr};
	int path{0};
	auto best{kSignalReach};
	auto const consider = [&](TTrack *Track) {
		if (Track == nullptr || Track->m_editorremoved)
			return;
		for (int p = 0; p < static_cast<int>(Track->m_paths.size()); ++p)
		{
			bezier const curve{Track->m_paths[p]};
			auto const low{glm::min(glm::min(plan_of(curve.p0), plan_of(curve.p1)), glm::min(plan_of(curve.p2), plan_of(curve.p3)))};
			auto const high{glm::max(glm::max(plan_of(curve.p0), plan_of(curve.p1)), glm::max(plan_of(curve.p2), plan_of(curve.p3)))};
			if (glm::length(glm::max(glm::max(low - plan_of(location), plan_of(location) - high), glm::dvec2{0.0})) >= best)
				continue;
			auto const distance{geometry::plan_distance(curve.point(nearest_on(*Track, p, location)), location)};
			if (distance < best)
			{
				best = distance;
				track = Track;
				path = p;
			}
		}
	};
	consider(Standing.track);
	if (track == nullptr)
		for (auto *candidate : simulation::Paths.sequence())
			consider(candidate);
	if (track == nullptr)
		return false;
	auto const axis{bezier{track->m_paths[path]}.point(nearest_on(*track, path, location))};
	auto const side{glm::dot(plan_of(location) - plan_of(axis), right_of(travel_of(Standing.include.yaw))) >= 0.0 ? 1 : -1};
	if (false == signal_drop_at(Standing, *track, path, location, side, Spot))
		return false;
	Spot.offset = geometry::plan_distance(location, axis);
	Spot.position = location;
	Spot.yaw = Standing.include.yaw;
	return true;
}

void editor_mode::signal_release()
{
	auto &tool{m_signal};
	if (tool.placing)
	{
		tool.placing = false;
		if (auto const *armed{signal_armed()}; armed != nullptr && tool.press.track != nullptr && false == tool.press.track->m_editorremoved)
			signal_place(tool.press, *armed);
		return;
	}
	if (false == tool.moving)
		return;
	tool.moving = false;
	auto const &mouse{ImGui::GetIO().MousePos};
	if (glm::length(glm::vec2{mouse.x, mouse.y} - tool.pressed) <= kDragStart)
		return;
	if (false == tool.moved_valid)
	{
		tool.status = refused(STR_C("Drop the signal by a track"));
		return;
	}
	tool.moved_valid = false;
	if (auto const *found{signal_standing_of(tool.selected)})
		signal_move_to(*found, tool.moved, STR_C("Signal %s moved, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s moved, Ctrl+Z takes it back"));
}

bool editor_mode::signal_move_to(signal_standing const &Standing, signal_spot const &Spot, char const *Read, char const *Unread)
{
	auto &tool{m_signal};
	auto const instance{Standing.include.instance};
	if (false == scene::Layers.tracked(instance) || false == scene::Layers.removable(instance))
	{
		tool.status = refused(STR_C("This signal is in a file the editor doesn't change, it can't be moved"));
		return false;
	}
	auto const &included{scene::Layers.instance(instance)};
	auto file{*included.file};
	signal_spot before;
	auto const *item{signal_template_of(Standing.include.file)};
	if (item != nullptr && signal_standing_spot(Standing, before) && travel_side(before.side, before.list) != travel_side(Spot.side, Spot.list))
		file = signal_variant(*item, Spot);
	auto values{Standing.include.values};
	auto const local{Spot.position - included.context.offset};
	values[1] = editor_includes::number(local.x);
	values[2] = editor_includes::number(local.y);
	values[3] = editor_includes::number(local.z);
	values[4] = editor_includes::number(Spot.yaw);
	auto const *next{signal_template_of(file)};
	auto const read{next != nullptr && false == next->read.empty() ? editor_includes::substitute(next->read, values) : Standing.read};
	auto directive{scene::Layers.directive(instance)};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	if (false == Standing.read.empty() && Spot.track->Dynamics.empty())
	{
		signal_detach_events(Standing.read, states);
		if (std::none_of(states.begin(), states.end(), [&](auto const &State) { return State.first == Spot.track; }))
			states.emplace_back(Spot.track, editor_track::capture(*Spot.track));
		editor_track::events(*Spot.track, Spot.list).emplace_back(ToLower(read), nullptr);
		std::string missing;
		editor_track::bind_events(*Spot.track, missing);
		Spot.track->mark_dirty();
	}
	auto const name{Standing.name};
	auto const node{Standing.include.file};
	set_include_directive(instance, editor_includes::compose_directive(file, values));
	if (states.empty())
	{
		EditorSnapshot snap;
		snap.directives.emplace_back(instance, std::move(directive));
		snap.node_name = node;
		trim_history();
		m_history.push_back(std::move(snap));
		g_redo.clear();
	}
	else
	{
		push_track_snapshot(std::move(states));
		m_history.back().directives.emplace_back(instance, std::move(directive));
	}
	signal_refresh(true);
	signal_select(instance);
	auto const *moved{signal_standing_of(instance)};
	tool.status = moved != nullptr && moved->track != nullptr ? format(Read, name.c_str(), editor_track::event_keyword(moved->list), moved->read.c_str(), name_of(*moved->track).c_str()) : format(Unread, name.c_str());
	return true;
}

bool editor_mode::signal_flip(scene::instance_handle const Instance)
{
	auto const *standing{signal_standing_of(Instance)};
	signal_spot spot;
	if (standing == nullptr || false == signal_standing_spot(*standing, spot))
	{
		m_signal.status = refused(STR_C("No track by the signal to take it across"));
		return false;
	}
	auto const across{plan_of(spot.position) - plan_of(spot.axis)};
	auto const mirrored{plan_of(spot.axis) + spot.travel * glm::dot(across, spot.travel) - right_of(spot.travel) * glm::dot(across, right_of(spot.travel))};
	spot.side = -spot.side;
	spot.position = {mirrored.x, spot.position.y, mirrored.y};
	return signal_move_to(*standing, spot, STR_C("Signal %s moved to the other side of the track, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s moved to the other side of the track, Ctrl+Z takes it back"));
}

bool editor_mode::signal_turn(scene::instance_handle const Instance)
{
	auto const *standing{signal_standing_of(Instance)};
	signal_spot spot;
	if (standing == nullptr || false == signal_standing_spot(*standing, spot))
	{
		m_signal.status = refused(STR_C("No track by the signal to turn it to"));
		return false;
	}
	auto const *item{signal_template_of(standing->include.file)};
	auto const position{spot.position};
	if (item == nullptr || false == signal_spot_at(*spot.track, spot.path, spot.axis, spot.axis, spot.list == 2 ? -1 : 1, spot.side, *item, spot))
		return false;
	spot.position = position;
	spot.yaw = std::fmod(standing->include.yaw + 180.0, 360.0);
	return signal_move_to(*standing, spot, STR_C("Signal %s turned to the trains of the other direction, read by %s %s of %s, Ctrl+Z takes it back"), STR_C("Signal %s turned to the trains of the other direction, Ctrl+Z takes it back"));
}

bool editor_mode::signal_swap(scene::instance_handle const Instance, std::string const &File)
{
	auto &tool{m_signal};
	auto const *standing{signal_standing_of(Instance)};
	if (standing == nullptr)
		return false;
	auto const name{standing->name};
	auto const before{standing->include.file};
	signal_changes changes;
	changes.files.emplace_back(Instance, File);
	std::string error;
	signal_batch(changes, {}, {}, error);
	signal_refresh(true);
	auto const *swapped{signal_standing_of(Instance)};
	if (false == error.empty() || swapped == nullptr || swapped->include.file == before)
	{
		tool.status = error.empty() ? format(STR_C("Signal %s can't take the template %s"), name.c_str(), File.c_str()) : error;
		return false;
	}
	signal_select(Instance);
	tool.status = format(STR_C("Signal %s got the template %s, Ctrl+Z takes it back"), name.c_str(), swapped->include.file.c_str());
	return true;
}

std::string editor_mode::signal_variant(signal_template const &Template, signal_spot const &Spot) const
{
	auto file{Template.file};
	auto const wanted{lean_wanted(glm::dot(right_of(Spot.travel), plan_of(Spot.position) - plan_of(Spot.axis)) >= 0.0 ? 1 : -1)};
	if (lean_of(Template.lean) == 0 || lean_of(Template.lean) == wanted)
		return file;
	auto const group{group_of(Template.mount, Template.lamps)};
	auto best{std::numeric_limits<std::size_t>::max()};
	for (auto const &other : m_signal.templates)
	{
		if (other.kind != Template.kind || group_of(other.mount, other.lamps) != group || lean_of(other.lean) != wanted || other.speeds != Template.speeds)
			continue;
		auto const distance{edit_distance(other.name, Template.name)};
		if (distance < best)
		{
			best = distance;
			file = other.file;
		}
	}
	return file;
}

std::string editor_mode::signal_next_name(std::string const &Name) const
{
	if (Name.empty())
		return Name;
	auto digits{Name.size()};
	while (digits > 0 && std::isdigit(static_cast<unsigned char>(Name[digits - 1])))
		--digits;
	if (digits < Name.size() && Name.size() - digits < 9)
		return Name.substr(0, digits) + std::to_string(std::stoi(Name.substr(digits)) + 1);
	auto const last{static_cast<unsigned char>(Name.back())};
	if (std::isalpha(last) && last != 'z' && last != 'Z')
		return Name.substr(0, Name.size() - 1) + static_cast<char>(last + 1);
	return Name + "2";
}

bool editor_mode::signal_place(signal_spot const &Spot, signal_template const &Template)
{
	auto &tool{m_signal};
	std::string name{tool.name};
	if (name.empty() || name.find_first_of(" \t\";") != std::string::npos)
	{
		tool.status = refused(STR_C("Give the signal a name without spaces first"));
		return false;
	}
	signal_refresh(true);
	auto const taken = [&](std::string const &Name) {
		return std::any_of(tool.standing.begin(), tool.standing.end(), [&](signal_standing const &Standing) { return ToLower(Standing.name) == ToLower(Name); });
	};
	for (int guard = 0; taken(name) && guard < 1000; ++guard)
		name = signal_next_name(name);
	auto const file{signal_variant(Template, Spot)};
	template_item item;
	item.file = file;
	item.location = Spot.position;
	item.yaw = Spot.yaw;
	item.described = true;
	item.values["name"] = name;
	if (Template.plate)
		item.values["#6"] = plate_for(tool.plate, name);
	if (Template.linked)
		item.values["#7"] = tool.linked[0] != '\0' ? std::string{tool.linked} : std::string{"none"};
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		tool.status = refused(STR_C("The signal couldn't be placed: ") + error);
		return false;
	}
	auto const instance{m_history.back().instances.front()};
	auto const *placed{signal_template_of(file)};
	auto const read{placed != nullptr && false == placed->read.empty() ? editor_includes::substitute(placed->read, {name}) : std::string{}};
	auto const path{name_of(*Spot.track)};
	std::string reason;
	if (read.empty())
		tool.status = format(STR_C("Signal %s placed (%s), the train doesn't read it, Ctrl+Z takes it back"), name.c_str(), file.c_str());
	else if (false == Spot.track->Dynamics.empty())
		tool.status = format(STR_C("Signal %s placed (%s), but vehicles stand on the path: add %s to its %s by hand, Ctrl+Z takes it back"), name.c_str(), file.c_str(), read.c_str(), editor_track::event_keyword(Spot.list));
	else if (false == scene::Layers.editable(Spot.track, &reason))
		tool.status = format(STR_C("Signal %s placed (%s), but the path can't take its event: %s, Ctrl+Z takes it back"), name.c_str(), file.c_str(), reason.c_str());
	else
	{
		auto snap{std::move(m_history.back())};
		m_history.pop_back();
		auto const before{editor_track::capture(*Spot.track)};
		editor_track::events(*Spot.track, Spot.list).emplace_back(ToLower(read), nullptr);
		std::string missing;
		editor_track::bind_events(*Spot.track, missing);
		Spot.track->mark_dirty();
		push_track_snapshot({{Spot.track, before}});
		m_history.back().instances = std::move(snap.instances);
		tool.status = format(STR_C("Signal %s placed (%s), read by %s %s of %s, Ctrl+Z takes it back. Its logic works after the scenery is saved and opened again"), name.c_str(), file.c_str(), editor_track::event_keyword(Spot.list), read.c_str(), path.c_str());
	}
	std::snprintf(tool.name, sizeof(tool.name), "%s", signal_next_name(name).c_str());
	signal_refresh(true);
	signal_select(instance);
	first_step_done(first_step::signal);
	return true;
}

void editor_mode::signal_detach_events(std::string const &Read, std::vector<std::pair<TTrack *, editor_track::state>> &States)
{
	auto const name{ToLower(Read)};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		bool carries{false};
		for (int list = 0; list < 6 && false == carries; ++list)
			for (auto const &event : editor_track::events(*track, list))
				carries = carries || ToLower(event.first) == name;
		if (false == carries)
			continue;
		if (std::none_of(States.begin(), States.end(), [&](auto const &State) { return State.first == track; }))
			States.emplace_back(track, editor_track::capture(*track));
		for (int list = 0; list < 6; ++list)
		{
			auto &events{editor_track::events(*track, list)};
			events.erase(std::remove_if(events.begin(), events.end(), [&](auto const &Event) { return ToLower(Event.first) == name; }), events.end());
		}
		std::string missing;
		editor_track::bind_events(*track, missing);
		track->mark_dirty();
	}
}

bool editor_mode::signal_delete(scene::instance_handle const Instance)
{
	if (false == scene::Layers.tracked(Instance) || false == scene::Layers.removable(Instance))
		return false;
	signal_scan();
	auto const &included{scene::Layers.instance(Instance)};
	auto const *item{signal_template_of(*included.file)};
	if (item == nullptr || item->read.empty())
		return false;
	std::string file;
	std::vector<std::string> values;
	if (false == editor_includes::parse_directive(scene::Layers.directive(Instance), file, values))
		return false;
	auto const read{editor_includes::substitute(item->read, values)};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	signal_detach_events(read, states);
	if (states.empty())
		return false;
	scene::Layers.removed(Instance, true);
	push_track_snapshot(std::move(states));
	m_history.back().instances.push_back(Instance);
	ui()->set_status(format(STR_C("Signal %s removed with its event %s, Ctrl+Z brings both back"), values.empty() ? "" : values.front().c_str(), read.c_str()));
	select_include(0);
	m_signal.selected = 0;
	signal_refresh(true);
	return true;
}

void editor_mode::signal_renamed(std::string const &Before, std::string const &After, EditorSnapshot &Snapshot)
{
	auto const name{ToLower(Before)};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		bool carries{false};
		for (int list = 0; list < 6 && false == carries; ++list)
			for (auto const &event : editor_track::events(*track, list))
				carries = carries || ToLower(event.first) == name;
		if (false == carries)
			continue;
		if (std::none_of(Snapshot.tracks.begin(), Snapshot.tracks.end(), [&](auto const &State) { return State.first == track; }))
			Snapshot.tracks.emplace_back(track, editor_track::capture(*track));
		for (int list = 0; list < 6; ++list)
			for (auto &event : editor_track::events(*track, list))
				if (ToLower(event.first) == name)
					event = {ToLower(After), nullptr};
		std::string missing;
		editor_track::bind_events(*track, missing);
		track->mark_dirty();
	}
	if (Snapshot.tracks.empty() || Snapshot.instance == 0)
		return;
	Snapshot.directives.emplace_back(Snapshot.instance, std::move(Snapshot.serialized));
	Snapshot.instance = 0;
	Snapshot.serialized.clear();
	Snapshot.action = EditorSnapshot::Action::TrackEdit;
	Snapshot.node_ptr = Snapshot.tracks.front().first;
	Snapshot.node_name = Snapshot.tracks.front().first->name();
}

bool editor_mode::standing_signals(std::vector<standing_signal> &Signals, bool const Wait)
{
	Signals.clear();
	signal_scan(Wait);
	if (m_signal.templates.empty())
		return false;
	signal_refresh(true);
	for (auto const &standing : m_signal.standing)
	{
		if (standing.kind < 0)
			continue;
		std::string file;
		std::vector<std::string> values;
		auto const *item{signal_template_of(standing.include.file)};
		auto const linked{item != nullptr && item->linked && editor_includes::parse_directive(scene::Layers.directive(standing.include.instance), file, values) && values.size() > 6};
		Signals.push_back({standing.include.instance, standing.name, kKinds[standing.kind], standing.include.location, standing.include.yaw, linked ? values[6] : std::string{}});
		auto &signal{Signals.back()};
		signal.file = standing.include.file;
		signal.track = standing.track;
		signal.facing = standing.track == nullptr ? 0 : standing.list % 3 == 2 ? 1 : standing.list % 3 == 1 ? -1 : 0;
	}
	return true;
}

bool editor_mode::signal_heads(std::vector<signal_head> &Heads, bool const Wait)
{
	Heads.clear();
	signal_scan(Wait);
	if (m_signal.templates.empty())
		return false;
	for (int index = 0; index < static_cast<int>(m_signal.templates.size()); ++index)
	{
		auto const &item{m_signal.templates[index]};
		if (item.lamps.empty())
			continue;
		auto const kind{kind_index(item.kind)};
		Heads.push_back({item.file, item.name, item.kind, item.mount, item.lamps, item.speeds, kind >= 0 && m_signal.chosen[kind] == index, item.aspects, item.lean});
	}
	return true;
}

std::size_t editor_mode::signal_batch(signal_changes const &Changes, std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created, std::string &Error)
{
	auto &tool{m_signal};
	signal_scan();
	signal_refresh(true);
	std::vector<template_item> items;
	std::vector<std::pair<signal_spot, std::string>> reads;
	for (auto const &order : Changes.placed)
	{
		auto const kind{kind_index(order.kind)};
		auto const *item{kind >= 0 && tool.chosen[kind] >= 0 && tool.chosen[kind] < static_cast<int>(tool.templates.size()) ? &tool.templates[tool.chosen[kind]] : nullptr};
		if (auto const *wanted{order.file.empty() ? nullptr : signal_template_of(order.file)}; wanted != nullptr && wanted->kind == order.kind)
			item = wanted;
		if (item == nullptr)
		{
			Error = format(STR_C("no template of the kind %s"), STR(kKindLabels[std::max(0, kind)]).c_str());
			break;
		}
		signal_spot spot;
		if (order.track == nullptr || false == signal_spot_at(*order.track, order.path, order.point, order.point, order.facing, order.side, *item, spot))
			continue;
		template_item placing;
		placing.file = signal_variant(*item, spot);
		placing.location = spot.position;
		placing.yaw = spot.yaw;
		placing.described = true;
		placing.values["name"] = order.name;
		auto const *variant{signal_template_of(placing.file)};
		if (variant == nullptr)
			variant = item;
		if (variant->plate)
			placing.values["#6"] = plate_for(std::string{}, order.plate);
		if (variant->linked)
			placing.values["#7"] = order.linked.empty() ? std::string{"none"} : order.linked;
		items.push_back(std::move(placing));
		reads.emplace_back(spot, variant->read.empty() ? std::string{} : editor_includes::substitute(variant->read, {order.name}));
	}
	if (false == Error.empty())
	{
		// nothing changed but the tracks cut for them, a step of undo
		if (false == States.empty() || false == Created.empty())
			push_track_snapshot(std::move(States), std::move(Created));
		return 0;
	}
	// the names passed round among the signals go through ones of their own first, or a read event renamed would be renamed again
	items.insert(items.end(), Changes.others.begin(), Changes.others.end());
	EditorSnapshot step;
	step.tracks = std::move(States);
	std::vector<std::pair<scene::instance_handle, std::string>> directives;
	std::unordered_set<scene::instance_handle> changed;
	auto const record = [&](scene::instance_handle const Instance, std::string const &Before) {
		if (changed.insert(Instance).second)
			directives.emplace_back(Instance, Before);
	};
	for (auto const temporary : {true, false})
		for (auto const &rename : Changes.renamed)
		{
			auto const *standing{signal_standing_of(rename.instance)};
			auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
			auto const before{scene::Layers.directive(rename.instance)};
			std::string file;
			std::vector<std::string> values;
			if (item == nullptr || false == editor_includes::parse_directive(before, file, values) || values.empty())
				continue;
			if (temporary && ToLower(values[0]) == ToLower(rename.name))
				continue;
			auto const read_before{item->read.empty() ? std::string{} : editor_includes::substitute(item->read, values)};
			values[0] = rename.name + (temporary ? "~" : "");
			if (item->plate && values.size() > 5 && false == rename.plate.empty())
				values[5] = plate_for(std::string{}, rename.plate);
			if (item->linked && values.size() > 6 && false == rename.linked.empty())
				values[6] = rename.linked;
			auto const read_after{item->read.empty() ? std::string{} : editor_includes::substitute(item->read, values)};
			auto const directive{editor_includes::compose_directive(file, values)};
			if (directive == before)
				continue;
			record(rename.instance, before);
			set_include_directive(rename.instance, directive);
			if (read_before != read_after)
				signal_renamed(read_before, read_after, step);
		}
	for (auto const &[instance, values] : Changes.values)
	{
		auto const before{scene::Layers.directive(instance)};
		std::string file;
		std::vector<std::string> standing;
		if (false == editor_includes::parse_directive(before, file, standing))
			continue;
		if (auto const directive{editor_includes::compose_directive(file, values)}; directive != before)
		{
			record(instance, before);
			set_include_directive(instance, directive);
		}
	}
	for (auto const &[instance, wanted] : Changes.files)
	{
		auto const *standing{signal_standing_of(instance)};
		auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
		auto const *chosen{signal_template_of(wanted)};
		auto const before{scene::Layers.directive(instance)};
		std::string file;
		std::vector<std::string> values;
		if (item == nullptr || chosen == nullptr || chosen->kind != item->kind || false == editor_includes::parse_directive(before, file, values) || values.empty())
			continue;
		signal_spot spot;
		auto const *variant{signal_standing_spot(*standing, spot) ? signal_template_of(signal_variant(*chosen, spot)) : chosen};
		if (variant == nullptr)
			variant = chosen;
		if (variant->file == item->file)
			continue;
		include_info had;
		include_info info;
		std::string missing;
		editor_includes::load(item->file, had, missing);
		editor_includes::load(variant->file, info, missing);
		auto const role_of = [](include_info &Info, int const Id) { return Info.parameter(Id).role; };
		std::vector<std::string> after(std::max(variant->parameters, 1));
		for (int id = 1; id <= static_cast<int>(after.size()); ++id)
		{
			auto &value{after[id - 1]};
			auto const role{role_of(info, id)};
			for (int old = 1; old <= static_cast<int>(values.size()) && value.empty(); ++old)
				if (role != "free" && role != "text" && role_of(had, old) == role)
					value = values[old - 1];
			if (id == 7 && variant->linked)
				value = item->linked && values.size() > 6 ? values[6] : std::string{"none"};
			if (value.empty() && id == 6 && variant->plate)
				value = plate_for(std::string{}, values[0].substr(values[0].rfind('_') + 1));
			if (value.empty())
				value = info.parameter(id).value.empty() ? std::string{"none"} : info.parameter(id).value;
		}
		auto const read_before{item->read.empty() ? std::string{} : editor_includes::substitute(item->read, values)};
		auto const read_after{variant->read.empty() ? std::string{} : editor_includes::substitute(variant->read, after)};
		record(instance, before);
		set_include_directive(instance, editor_includes::compose_directive(variant->file, after));
		if (false == read_before.empty() && false == read_after.empty() && ToLower(read_before) != ToLower(read_after))
			signal_renamed(read_before, read_after, step);
	}
	std::unordered_set<TTrack const *> const created(Created.begin(), Created.end());
	std::vector<scene::instance_handle> removed;
	for (auto const instance : Changes.removed)
	{
		if (false == scene::Layers.tracked(instance) || false == scene::Layers.removable(instance) || scene::Layers.instance(instance).removed)
			continue;
		auto const *standing{signal_standing_of(instance)};
		auto const *item{standing != nullptr ? signal_template_of(standing->include.file) : nullptr};
		std::string file;
		std::vector<std::string> values;
		if (item != nullptr && false == item->read.empty() && editor_includes::parse_directive(scene::Layers.directive(instance), file, values))
		{
			std::vector<std::pair<TTrack *, editor_track::state>> states;
			signal_detach_events(editor_includes::substitute(item->read, values), states);
			for (auto &state : states)
				if (created.count(state.first) == 0 && std::none_of(step.tracks.begin(), step.tracks.end(), [&](auto const &Taken) { return Taken.first == state.first; }))
					step.tracks.push_back(std::move(state));
		}
		scene::Layers.removed(instance, true);
		removed.push_back(instance);
	}
	std::vector<scene::instance_handle> instances;
	if (Error.empty() && false == items.empty())
	{
		if (place_templates(items, Error) == 0)
			reads.clear();
		else
		{
			auto placed{std::move(m_history.back())};
			m_history.pop_back();
			instances = std::move(placed.instances);
		}
	}
	else
		reads.clear();
	for (auto const &[spot, read] : reads)
	{
		if (read.empty() || false == spot.track->Dynamics.empty() || false == scene::Layers.editable(spot.track))
			continue;
		if (created.count(spot.track) == 0 && std::none_of(step.tracks.begin(), step.tracks.end(), [&](auto const &State) { return State.first == spot.track; }))
			step.tracks.emplace_back(spot.track, editor_track::capture(*spot.track));
		editor_track::events(*spot.track, spot.list).emplace_back(ToLower(read), nullptr);
		std::string missing;
		editor_track::bind_events(*spot.track, missing);
		spot.track->mark_dirty();
	}
	auto const count{instances.size()};
	instances.insert(instances.end(), removed.begin(), removed.end());
	if (step.tracks.empty() && Created.empty() && instances.empty() && directives.empty())
		return 0;
	if (step.tracks.empty() && Created.empty())
	{
		EditorSnapshot only;
		auto const *file{scene::Layers.instance(directives.empty() ? instances.front() : directives.front().first).file};
		only.node_name = file != nullptr ? *file : std::string{};
		trim_history();
		m_history.push_back(std::move(only));
		g_redo.clear();
	}
	else
		push_track_snapshot(std::move(step.tracks), std::move(Created));
	auto &pushed{m_history.back()};
	pushed.instances = std::move(instances);
	pushed.directives.insert(pushed.directives.end(), directives.begin(), directives.end());
	signal_refresh(true);
	return count;
}

void editor_mode::signal_select(scene::instance_handle const Instance)
{
	m_signal.selected = Instance;
	if (Instance != 0)
		select_include(Instance);
}

void editor_mode::render_signal_palette()
{
	auto &tool{m_signal};
	ImGui::SetNextItemWidth(-1.0f);
	ImGui::InputTextWithHint("##signalfilter", STR_C("filter: name, w24, sbl..."), tool.filter, sizeof(tool.filter));
	auto const filter{ToLower(tool.filter)};
	auto const *armed{signal_armed()};
	std::vector<std::vector<int>> groups;
	std::vector<std::string> keys;
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
	{
		auto const &item{tool.templates[i]};
		if (item.kind != kKinds[tool.kind])
			continue;
		if (false == filter.empty() && item.name.find(filter) == std::string::npos && ToLower(item.description).find(filter) == std::string::npos)
			continue;
		auto const key{group_of(item.mount, item.lamps)};
		auto const found{std::find(keys.begin(), keys.end(), key)};
		if (found == keys.end())
		{
			keys.push_back(key);
			groups.push_back({i});
		}
		else
			groups[found - keys.begin()].push_back(i);
	}
	if (groups.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No template of this kind matches the filter"));
		return;
	}
	ImVec2 const tile{52.0f, 86.0f};
	auto const spacing{ImGui::GetStyle().ItemSpacing.x};
	ImGui::BeginChild("##signaltiles", ImVec2(0.0f, std::min(2.0f * (tile.y + 8.0f) + 20.0f, ImGui::GetContentRegionAvail().y * 0.45f)), true);
	auto const columns{std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x + spacing) / (tile.x + spacing)))};
	auto *draw{ImGui::GetWindowDrawList()};
	for (int g = 0; g < static_cast<int>(groups.size()); ++g)
	{
		auto const &group{groups[g]};
		auto const &first{tool.templates[group.front()]};
		auto const chosen{armed != nullptr && group_of(armed->mount, armed->lamps) == keys[g] && armed->kind == first.kind};
		if (g % columns != 0)
			ImGui::SameLine();
		ImGui::PushID(g);
		auto const origin{ImGui::GetCursorScreenPos()};
		if (ImGui::InvisibleButton("##tile", tile))
		{
			auto pick{group.front()};
			if (armed != nullptr && chosen)
				pick = tool.chosen[tool.kind];
			tool.chosen[tool.kind] = pick;
		}
		auto const hovered{ImGui::IsItemHovered()};
		draw->AddRectFilled(origin, ImVec2(origin.x + tile.x, origin.y + tile.y), chosen ? IM_COL32(60, 90, 140, 255) : hovered ? IM_COL32(60, 60, 60, 255) : IM_COL32(40, 40, 40, 255), 4.0f);
		draw_signal_icon(draw, origin, ImVec2(tile.x, tile.y - 14.0f), first.mount, first.lamps, chosen ? overlay_color::selected : IM_COL32(120, 120, 120, 255));
		auto const caption{first.lamps.empty() ? first.name : first.lamps};
		auto const captionsize{ImGui::CalcTextSize(caption.c_str())};
		draw->AddText(ImVec2(origin.x + std::max(2.0f, (tile.x - captionsize.x) * 0.5f), origin.y + tile.y - 15.0f), IM_COL32(220, 220, 220, 255), caption.c_str());
		if (group.size() > 1)
			draw->AddText(ImVec2(origin.x + 3.0f, origin.y + 2.0f), IM_COL32(170, 170, 170, 255), std::to_string(group.size()).c_str());
		if (hovered)
		{
			ImGui::BeginTooltip();
			ImGui::TextUnformatted(first.description.empty() ? first.name.c_str() : first.description.c_str());
			ImGui::TextDisabled(STR_C("%zu variants, e.g. %s"), group.size(), first.file.c_str());
			ImGui::TextDisabled("%s", first.read.empty() ? STR_C("The train doesn't read it") : STR_C("The train reads it by an event of the path"));
			ImGui::EndTooltip();
		}
		ImGui::PopID();
	}
	ImGui::EndChild();
	if (armed == nullptr)
		return;
	auto const key{group_of(armed->mount, armed->lamps)};
	ImGui::SetNextItemWidth(-1.0f);
	if (ImGui::BeginCombo("##signalvariant", armed->name.c_str()))
	{
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			auto const &item{tool.templates[i]};
			if (item.kind != armed->kind || group_of(item.mount, item.lamps) != key)
				continue;
			auto const label{item.name + (item.lean == "left" ? std::string{"  \xE2\x86\x96"} : item.lean == "right" ? std::string{"  \xE2\x86\x97"} : std::string{}) + "##" + item.file};
			if (ImGui::Selectable(label.c_str(), &item == armed))
				tool.chosen[tool.kind] = i;
			if (ImGui::IsItemHovered() && false == item.description.empty())
				ImGui::SetTooltip("%s\n%s", item.description.c_str(), item.file.c_str());
		}
		ImGui::EndCombo();
	}
	if (ImGui::IsItemHovered())
	{
		auto const read{armed->read.empty() ? std::string{armed->kind == "warning" || armed->kind == "repeater" ? STR_C("The train doesn't read it: it repeats what the signal ahead of it shows") : STR_C("The train doesn't read it")}
		                                    : format(STR_C("The train reads it by %s on the path it stands by"), editor_includes::substitute(armed->read, {tool.name[0] != '\0' ? std::string{tool.name} : std::string{"?"}}).c_str())};
		ImGui::SetTooltip("%s\n%s", STR_C("Variant of the chosen type. A head leaning to one side is swapped for its counterpart leaning towards the track on the other side"), read.c_str());
	}
	if (false == armed->description.empty())
		ImGui::TextWrapped("%s", armed->description.c_str());
}

void editor_mode::render_signal_selected()
{
	auto const *found{signal_standing_of(m_signal.selected)};
	if (found == nullptr)
		return;
	ImGui::Separator();
	ImGui::TextUnformatted(format(STR_C("Selected: %s (%s)"), found->name.c_str(), found->include.file.c_str()).c_str());
	item_tooltip("Name, plate and the linked signal: in the node properties. Del removes it with its event");
	if (found->read.empty())
		ImGui::TextDisabled("%s", STR_C("The train doesn't read it"));
	else if (found->track != nullptr)
		ImGui::TextDisabled(STR_C("Read by %s %s of %s"), editor_track::event_keyword(found->list), found->read.c_str(), name_of(*found->track).c_str());
	else
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), STR_C("No path has %s among its events: the train doesn't see it"), found->read.c_str());
	auto const instance{found->include.instance};
	if (auto const *item{signal_template_of(found->include.file)}; item != nullptr)
	{
		auto const shown = [](signal_template const &Template) { return Template.lamps.empty() ? Template.name : Template.name + "  " + Template.lamps; };
		std::string swap;
		ImGui::SetNextItemWidth(-1.0f);
		if (ImGui::BeginCombo("##signalswap", format(STR_C("Template: %s"), shown(*item).c_str()).c_str(), ImGuiComboFlags_HeightLarge))
		{
			for (auto const &other : m_signal.templates)
				if (other.kind == item->kind && other.mount == item->mount && ImGui::Selectable((shown(other) + "##" + other.file).c_str(), other.file == item->file) && other.file != item->file)
					swap = other.file;
			ImGui::EndCombo();
		}
		item_tooltip("Another template of its kind and mount for the signal: the name, the plate, the place and the event the train reads it by stay. A head leaning to one side is swapped for its counterpart leaning towards the track. Ctrl+Z takes it back");
		if (false == swap.empty())
		{
			signal_swap(instance, swap);
			return;
		}
	}
	auto const width{(ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f};
	if (ImGui::Button(STR_C("To the other side"), ImVec2(width, 0.0f)))
		signal_flip(instance);
	item_tooltip("Takes the signal across the track, it keeps facing the same trains. Dragging it over the track does the same");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Turn around"), ImVec2(width, 0.0f)))
		signal_turn(instance);
	item_tooltip("Turns the signal to the trains of the other direction, they read it by the other event of the path");
}

void editor_mode::render_signal_ui()
{
	auto &tool{m_signal};
	signal_scan(false);
	if (tool.templates.empty() && EditorIncludes.searching())
	{
		ImGui::TextDisabled("%s", STR_C("Looking for the templates..."));
		return;
	}
	if (tool.templates.empty())
	{
		ImGui::TextWrapped("%s", STR_C("No signal templates: no template of the scenery directory is described with category: signal. tools/editor/signal_headers.py writes the descriptions into the signal templates."));
		return;
	}
	std::vector<std::string> labels;
	labels.reserve(kKindCount);
	std::vector<segment_option> kinds;
	for (int k = 0; k < kKindCount; ++k)
		if (std::any_of(tool.templates.begin(), tool.templates.end(), [&](signal_template const &Item) { return Item.kind == kKinds[k]; }))
		{
			labels.push_back(STR(kKindLabels[k]));
			kinds.push_back({k, labels.back().c_str()});
		}
	if (false == kinds.empty() && std::none_of(kinds.begin(), kinds.end(), [&](segment_option const &Option) { return Option.value == tool.kind; }))
		tool.kind = kinds.front().value;
	choice_chips("signalkinds", tool.kind, kinds);
	render_signal_palette();
	auto const *armed{signal_armed()};
	ImGui::Separator();
	if (begin_fields("##signalnames", 0.3f))
	{
		field_row("Name");
		ImGui::InputText("##signalname", tool.name, sizeof(tool.name));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Name of the next signal; it counts up after each one placed. Its events and memory cell are named after it"));
		ImGui::EndTable();
	}
	ImGui::PushID("signalmore");
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		bool missing{false};
		if (begin_fields("##signalplace", 0.45f))
		{
			if (armed != nullptr && armed->plate)
			{
				auto const automatic{format(STR_C("auto: %s"), plate_for({}, tool.name).c_str())};
				field_row("Plate");
				ImGui::InputTextWithHint("##signalplate", automatic.c_str(), tool.plate, sizeof(tool.plate));
				auto const typing{ImGui::IsItemActive()};
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("Texture of the plate from textures/tabl, e.g. a-12 or tob. Left empty: the one named like the signal, if there is one"));
				auto const &plates{plate_textures()};
				auto const typed{ToLower(tool.plate)};
				auto const exists{typed.empty() || std::binary_search(plates.begin(), plates.end(), typed)};
				if (false == exists)
					ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", STR_C("no such texture"));
				missing = typing || (false == exists && false == typed.empty());
			}
			if (armed != nullptr && armed->linked)
			{
				field_row("Linked signal");
				ImGui::InputText("##linked", tool.linked, sizeof(tool.linked));
				item_tooltip("(p7) of the template: the warning signal a semaphore drives, the previous block signal, \"none\" for no link");
			}
			auto offset{static_cast<float>(EditorSettings.signal_offset())};
			field_row("From the axis (m)");
			if (ImGui::InputFloat("##signaloffset", &offset, 0.05f, 0.25f, "%.2f"))
			{
				EditorSettings.signal_offset(std::clamp(static_cast<double>(offset), 0.0, 10.0));
				EditorSettings.save();
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("Distance of the mast from the axis of the track. 2.25 m is what the signals of the sceneries stand at"));
			if (armed != nullptr && armed->mount == "gantry")
			{
				field_row("Over the rails (m)");
				ImGui::InputFloat("##signalheight", &tool.height, 0.1f, 0.5f, "%.2f");
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("A head for a gantry hangs over the axis of the track this high over the rail heads"));
			}
			ImGui::EndTable();
		}
		if (missing)
		{
			auto const &plates{plate_textures()};
			auto const typed{ToLower(tool.plate)};
			int shown{0};
			for (auto const &plate : plates)
			{
				if (typed.empty() || plate.compare(0, typed.size(), typed) != 0 || plate == typed)
					continue;
				if (shown % 6 != 0)
					ImGui::SameLine();
				if (ImGui::SmallButton(plate.c_str()))
					std::snprintf(tool.plate, sizeof(tool.plate), "%s", plate.c_str());
				if (++shown >= 18)
					break;
			}
		}
		ImGui::TreePop();
	}
	ImGui::PopID();
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
	render_signal_selected();
}

void editor_mode::draw_signal_overlay() const
{
	auto const &tool{m_signal};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const arrow = [&](glm::dvec3 const &From, glm::dvec2 const &Direction, ImU32 const Colour) {
		glm::dvec3 const tip{From.x + Direction.x * 6.0, From.y, From.z + Direction.y * 6.0};
		ImVec2 a, b;
		if (false == projection.project(From, a) || false == projection.project(tip, b))
			return;
		drawlist->AddLine(a, b, Colour, 2.5f);
		auto const dx{b.x - a.x};
		auto const dy{b.y - a.y};
		auto const size{std::sqrt(dx * dx + dy * dy)};
		if (size < 1.0f)
			return;
		ImVec2 const unit{dx / size, dy / size};
		ImVec2 const side{-unit.y, unit.x};
		drawlist->AddTriangleFilled(ImVec2(b.x + unit.x * 9.0f, b.y + unit.y * 9.0f), ImVec2(b.x + side.x * 6.0f, b.y + side.y * 6.0f), ImVec2(b.x - side.x * 6.0f, b.y - side.y * 6.0f), Colour);
	};
	auto const mast = [&](glm::dvec3 const &Foot, ImU32 const Colour, float const Width) { projection.line(drawlist, Foot, Foot + glm::dvec3{0.0, kSignalHead, 0.0}, Colour, Width); };
	for (int i = 0; i < static_cast<int>(tool.standing.size()); ++i)
	{
		auto const &standing{tool.standing[i]};
		auto const &location{standing.include.location};
		if (glm::distance(location, camera) > kSignalRange)
			continue;
		auto const selected{standing.include.instance == tool.selected};
		auto const hovered{m_intent.what == track_intent::kind::signal_move && m_intent.index == i};
		auto const unread{false == standing.read.empty() && standing.track == nullptr};
		auto const colour{selected ? overlay_color::selected : hovered ? overlay_color::highlight : unread ? overlay_color::invalid : IM_COL32(255, 210, 60, 160)};
		mast(location, colour, selected || hovered ? 3.0f : 1.5f);
		ImVec2 head;
		if ((selected || hovered || unread) && projection.project(location + glm::dvec3{0.0, kSignalHead, 0.0}, head))
		{
			auto const label{standing.name + (unread ? std::string{"  "} + STR(" not read") : std::string{})};
			drawlist->AddText(ImVec2(head.x + 6.0f, head.y - 6.0f), colour, label.c_str());
		}
		if (selected || hovered)
			arrow(location, travel_of(standing.include.yaw), colour);
	}
	auto const preview = [&](signal_spot const &Spot, ImU32 const Colour) {
		projection.line(drawlist, Spot.axis + glm::dvec3{0.0, kRailTop, 0.0}, Spot.position, Colour, 2.0f);
		mast(Spot.position, Colour, 3.0f);
		arrow(Spot.axis + glm::dvec3{0.0, kRailTop, 0.0}, Spot.travel, Colour);
		ImVec2 head;
		auto const *armed{signal_armed()};
		if (armed != nullptr && false == armed->lamps.empty() && projection.project(Spot.position + glm::dvec3{0.0, kSignalHead, 0.0}, head))
		{
			auto const lamps{static_cast<int>(armed->lamps.size())};
			drawlist->AddRectFilled(ImVec2(head.x - 6.0f, head.y - 4.0f), ImVec2(head.x + 6.0f, head.y + lamps * 10.0f), IM_COL32(20, 20, 20, 220), 3.0f);
			for (int i = 0; i < lamps; ++i)
				drawlist->AddCircleFilled(ImVec2(head.x, head.y + 1.0f + i * 10.0f), 4.0f, lamp_colour(armed->lamps[i]));
		}
	};
	if (tool.placing)
		preview(tool.press, overlay_color::grip);
	else if (tool.moving && tool.moved_valid)
		preview(tool.moved, overlay_color::selected);
	else if (m_intent.what == track_intent::kind::signal_place && m_hover.track != nullptr)
	{
		if (auto const *armed{signal_armed()}; armed != nullptr)
		{
			signal_spot spot;
			if (signal_spot_at(*m_hover.track, m_hover.path, m_hover.point, cursor_level(m_hover.point.y), 0, 0, *armed, spot))
				preview(spot, overlay_color::marked);
		}
	}
}
