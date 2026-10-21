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
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "world/Segment.h"
#include "world/Track.h"
#include "world/TrackRails.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>

namespace
{

std::string drive_file(std::string Text)
{
	std::replace(Text.begin(), Text.end(), '\\', '/');
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

int drive_side(std::string const &Side)
{
	if (Side == "left")
		return 0;
	return Side == "right" ? 1 : -1;
}

std::optional<std::string> pose_value(std::string const &Role, glm::dvec3 const &Local, editor_track::drive_spot const &Spot)
{
	if (Role == "pos.x")
		return editor_includes::number(Local.x);
	if (Role == "pos.y")
		return editor_includes::number(Local.y);
	if (Role == "pos.z")
		return editor_includes::number(Local.z);
	if (Role == "rot.y")
		return editor_includes::number(Spot.yaw);
	if (Role == "rot.x")
		return editor_includes::number(Spot.tilt.x);
	if (Role == "rot.z")
		return editor_includes::number(Spot.tilt.y);
	return std::nullopt;
}

double angle_value(std::vector<std::string> const &Values, std::size_t const Index)
{
	double value{0.0};
	if (Index < Values.size())
		std::sscanf(Values[Index].c_str(), "%lf", &value);
	return value;
}

} // namespace

bool editor_mode::scan_switch_drives(bool const Wait)
{
	auto &tool{m_switch};
	if (Wait)
		EditorIncludes.wait();
	else
		EditorIncludes.start();
	if (false == EditorIncludes.scanned() || tool.drives_revision == EditorIncludes.revision())
		return false;
	tool.drives_revision = EditorIncludes.revision();
	// the drive chosen stays chosen, also none
	auto const chosen{tool.drive >= 0 && tool.drive < static_cast<int>(tool.drives.size()) ? std::optional<std::string>{tool.drives[tool.drive].name} : std::nullopt};
	auto const none{false == tool.drives.empty() && tool.drive < 0};
	tool.drives.clear();
	for (auto const &entry : EditorIncludes.entries())
	{
		if (entry.category != "switch drive")
			continue;
		include_info info;
		std::string error;
		if (false == editor_includes::load(entry.file, info, error) || editor_includes::parameter_with_role(info, "track") == 0)
			continue;
		auto const hand{drive_side(info.switch_hand)};
		auto const drive{drive_side(info.switch_drive)};
		if (hand < 0 || drive < 0)
			continue;
		auto const name{info.name.empty() ? entry.file : info.name};
		auto family{std::find_if(tool.drives.begin(), tool.drives.end(), [&](switch_tool::drive_family const &Family) { return Family.name == name; })};
		if (family == tool.drives.end())
		{
			tool.drives.push_back({name, {}});
			family = std::prev(tool.drives.end());
		}
		family->files[hand * 2 + drive] = entry.file;
		family->heights[hand * 2 + drive] = info.switch_height;
		family->preferred |= info.switch_default;
	}
	std::sort(tool.drives.begin(), tool.drives.end(), [](switch_tool::drive_family const &A, switch_tool::drive_family const &B) { return A.name < B.name; });
	auto const preferred{std::find_if(tool.drives.begin(), tool.drives.end(), [](switch_tool::drive_family const &Family) { return Family.preferred; })};
	auto const kept{std::find_if(tool.drives.begin(), tool.drives.end(), [&](switch_tool::drive_family const &Family) { return chosen.has_value() && Family.name == *chosen; })};
	if (tool.drives.empty() || none)
		tool.drive = -1;
	else if (kept != tool.drives.end())
		tool.drive = static_cast<int>(kept - tool.drives.begin());
	else
		tool.drive = preferred != tool.drives.end() ? static_cast<int>(preferred - tool.drives.begin()) : 0;
	return true;
}

void editor_mode::switch_drive_captured(TTrack const &Track)
{
	if (Track.eType != tt_Switch || m_infra_suspended)
		return;
	auto const found{find_switch_drive(Track)};
	if (false == found.has_value())
		return;
	auto const instance{found->standing.instance};
	if (std::any_of(m_drive_buffer.begin(), m_drive_buffer.end(), [&](auto const &Entry) { return Entry.first == instance; }))
		return;
	m_drive_buffer.emplace_back(instance, scene::Layers.directive(instance));
}

void editor_mode::switch_drives_committed(std::vector<TTrack *> const &Tracks)
{
	if (m_infra_suspended)
		return;
	for (auto const *track : Tracks)
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch)
			continue;
		auto const found{find_switch_drive(*track)};
		auto const spot{editor_track::drive_spot_of(*track)};
		if (false == found.has_value() || false == spot.has_value())
			continue;
		auto const &drive{found->standing};
		auto const &included{scene::Layers.instance(drive.instance)};
		include_info info;
		std::string error;
		if (included.file == nullptr || false == editor_includes::load(*included.file, info, error))
			continue;
		auto const local{spot->location + glm::dvec3{0.0, m_switch.drives[found->family].heights[found->index], 0.0} - included.context.offset};
		auto values{drive.values};
		for (auto const &parameter : info.parameters)
		{
			if (parameter.id < 1 || parameter.id > static_cast<int>(values.size()))
				continue;
			if (auto const value{pose_value(parameter.role, local, *spot)})
				values[parameter.id - 1] = *value;
		}
		auto const directive{editor_includes::compose_directive(*included.file, values)};
		if (directive != scene::Layers.directive(drive.instance))
			set_include_directive(drive.instance, directive);
	}
}

std::string editor_mode::missing_drive_variant(int const Family, bool const Rightswitch, bool const Driveright) const
{
	return format(Rightswitch ? STR_C("The drive %s has no variant for a right-hand switch with the drive %s") : STR_C("The drive %s has no variant for a left-hand switch with the drive %s"), m_switch.drives[Family].name.c_str(), Driveright ? STR_C("on the right") : STR_C("on the left"));
}

bool editor_mode::place_switch_drive(TTrack &Switch)
{
	auto &tool{m_switch};
	scan_switch_drives();
	if (tool.drive < 0 || tool.drive >= static_cast<int>(tool.drives.size()) || Switch.eType != tt_Switch || Switch.m_paths.size() < 2)
		return false;
	auto const spot{editor_track::drive_spot_of(Switch)};
	if (false == spot.has_value())
		return false;
	bool const driveright{(tool.drive_side == 0) == spot->rightswitch};
	auto const &family{tool.drives[tool.drive]};
	auto const index{(spot->rightswitch ? 2 : 0) + (driveright ? 1 : 0)};
	template_item item;
	item.file = family.files[index];
	if (item.file.empty())
	{
		tool.status = refused(missing_drive_variant(tool.drive, spot->rightswitch, driveright));
		return false;
	}
	item.location = spot->location + glm::dvec3{0.0, family.heights[index], 0.0};
	item.yaw = spot->yaw;
	item.described = true;
	item.track = Switch.name();
	item.tilt = spot->tilt;
	if (auto const present{find_switch_drive(Switch)})
		return replace_switch_drive(Switch, tool.drive, (present->index & 1) != (driveright ? 1 : 0));
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		tool.status = refused(STR_C("The drive couldn't be placed: ") + error);
		return false;
	}
	tool.status = format(STR_C("Switch %s with the drive %s: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), item.file.c_str(), Switch.name().c_str(), Switch.name().c_str());
	return true;
}

std::optional<editor_mode::switch_drive> editor_mode::find_switch_drive(TTrack const &Switch, bool const Wait)
{
	auto const &tool{m_switch};
	scan_switch_drives(Wait);
	std::map<std::string, std::pair<int, int>, std::less<>> variants;
	for (int family = 0; family < static_cast<int>(tool.drives.size()); ++family)
		for (int index = 0; index < 4; ++index)
			if (false == tool.drives[family].files[index].empty())
				variants[drive_file(tool.drives[family].files[index])] = {family, index};
	auto const name{drive_file(Switch.name())};
	auto const standing{standing_templates([&](std::string const &File) { return variants.contains(File); })};
	auto const found{std::find_if(standing.begin(), standing.end(), [&](standing_template const &Standing) { return drive_file(Standing.values[0]) == name; })};
	if (found == standing.end())
		return std::nullopt;
	auto const &[family, index]{variants[found->file]};
	return switch_drive{*found, family, index};
}

void editor_mode::apply_switch_drive_choice(TTrack &Switch)
{
	if (m_switch.drive >= 0)
	{
		place_switch_drive(Switch);
		return;
	}
	if (auto const found{find_switch_drive(Switch)}; found.has_value() && remove_templates({found->standing.instance}) > 0)
		m_switch.status = format(STR_C("Switch %s without a drive now"), Switch.name().c_str());
}

bool editor_mode::flip_switch_drive(TTrack &Switch)
{
	return replace_switch_drive(Switch, -1, true);
}

bool editor_mode::replace_switch_drive(TTrack &Switch, int const Family, bool const Flip)
{
	auto &tool{m_switch};
	auto const present{find_switch_drive(Switch)};
	if (false == present.has_value())
	{
		tool.status = refused(format(STR_C("No drive of the switch %s among the described drives"), Switch.name().c_str()));
		return false;
	}
	auto const &found{present->standing};
	auto const family{Family >= 0 ? Family : present->family};
	auto const index{Flip ? present->index ^ 1 : present->index};
	template_item item;
	item.file = tool.drives[family].files[index];
	if (item.file.empty())
	{
		tool.status = refused(Flip ? format(STR_C("The drive %s has no variant with the drive on the other side"), tool.drives[family].name.c_str()) : missing_drive_variant(family, (index & 2) != 0, (index & 1) != 0));
		return false;
	}
	if (drive_file(item.file) == found.file)
		return false;
	auto const spot{editor_track::drive_spot_of(Switch)};
	item.location = spot.has_value() ? spot->location + glm::dvec3{0.0, tool.drives[family].heights[index], 0.0} : found.location;
	item.yaw = found.yaw;
	item.described = true;
	item.track = Switch.name();
	item.tilt = {angle_value(found.values, 5), angle_value(found.values, 6)};
	if (remove_templates({found.instance}) == 0)
	{
		tool.status = refused(STR_C("The drive of this switch can't be changed in its file"));
		return false;
	}
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		scene::Layers.removed(found.instance, false);
		m_history.pop_back();
		tool.status = refused(STR_C("The drive couldn't be placed: ") + error);
		return false;
	}
	join_last_steps();
	tool.status = format(index & 1 ? STR_C("Switch %s: the drive %s, on the right now") : STR_C("Switch %s: the drive %s, on the left now"), Switch.name().c_str(), item.file.c_str());
	return true;
}

bool editor_mode::render_switch_drive_choice()
{
	auto &tool{m_switch};
	scan_switch_drives(false);
	if (tool.drives.empty())
	{
		ImGui::TextDisabled("%s", EditorIncludes.searching() ? STR_C("Looking for the templates...") : STR_C("No switch drive models"));
		return false;
	}
	auto const label = [&](int const Index) { return Index < 0 ? std::string{STR_C("no drive")} : tool.drives[Index].name; };
	bool changed{false};
	if (false == begin_fields("##drivefields", 0.3f))
		return false;
	field_row("Drive");
	if (ImGui::BeginCombo("##drive", label(tool.drive).c_str()))
	{
		for (int i = -1; i < static_cast<int>(tool.drives.size()); ++i)
			if (ImGui::Selectable(label(i).c_str(), tool.drive == i))
			{
				tool.drive = i;
				changed = true;
			}
		ImGui::EndCombo();
	}
	ImGui::EndTable();
	item_tooltip("Templates described with category: switch drive, the side of the switch and of the drive, and a parameter with the role track for the name of the switch.\nIts events <name>+ and <name>- switch it with the keys by the switch and in the map during the game");
	if (tool.drive >= 0)
		changed |= segmented("driveside", tool.drive_side, {{0, STR_C("diverging side")}, {1, STR_C("other side")}});
	return changed;
}

// a key by the movable nose of each switch with a drive, the frog is far from the drive at the start of the switch
bool editor_mode::nose_controls(std::vector<std::string> &Rootstatements)
{
	if (scene::Layers.empty())
		return false;
	std::string root{scene::Layers.layer(scene::layer_handle{1}).name};
	if (auto const slash{root.find_last_of("/\\")}; slash != std::string::npos)
		root.erase(0, slash + 1);
	if (auto const dot{root.rfind('.')}; dot != std::string::npos)
		root.erase(dot);
	if (root.empty())
		return false;
	auto const file{root + "_dzioby.scm"};
	std::ostringstream text;
	auto noses{0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Switch || track->SwitchExtension == nullptr || false == track->SwitchExtension->m_movablenose || track->name().empty())
			continue;
		auto const &segments{track->SwitchExtension->Segments};
		auto const frog{track->frog()};
		if (false == frog.valid || segments[0] == nullptr || frog.angle <= 0.0 || false == find_switch_drive(*track).has_value())
			continue;
		auto const &name{track->name()};
		// where the inner rails cross, on the gauge line of the main path
		glm::dvec3 point, across;
		segments[0]->plan_frame(frog.at[0], point, across);
		auto const nose{point + across * (frog.side[0] * 0.5 * std::abs(track->fTrackWidth))};
		auto const range{5.0 + track_rails::movable_nose_length(std::tan(frog.angle))};
		text << "node -1 0 " << name << "_dziob eventlauncher " << format("%.3f %.3f %.3f %.1f", nose.x, nose.y, nose.z, range) << " t 0 " << name << "+ " << name << "- end\r\n";
		++noses;
	}
	auto const path{Global.asCurrentSceneryPath + file};
	std::error_code error;
	auto const exists{std::filesystem::exists(path, error)};
	if (noses == 0 && false == exists)
		return false;
	auto const content{"// made by the scenery editor on each save: T and Shift+T by the movable nose throw its switch, as by its drive\r\n" + text.str()};
	std::string current;
	{
		std::ifstream input{path, std::ios_base::binary};
		current.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
	}
	if (current != content)
	{
		std::ofstream output{path, std::ios_base::binary | std::ios_base::trunc};
		output << content;
	}
	auto included{m_nose_included};
	for (std::size_t handle = 1; handle <= scene::Layers.size() && false == included; ++handle)
	{
		auto name{scene::Layers.layer(static_cast<scene::layer_handle>(handle)).name};
		std::replace(name.begin(), name.end(), '\\', '/');
		included = ToLower(name) == ToLower(file) || ToLower(name).ends_with("/" + ToLower(file));
	}
	if (included)
		return false;
	Rootstatements.push_back("include " + file + " end");
	return true;
}
