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

#include "world/Track.h"
#include "scene/scenelayers.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <iterator>
#include <map>

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

void editor_mode::scan_switch_drives()
{
	auto &tool{m_switch};
	if (tool.drives_scanned)
		return;
	tool.drives_scanned = true;
	tool.drives.clear();
	if (false == EditorIncludes.scanned())
		EditorIncludes.scan();
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
	if (tool.drives.empty())
		tool.drive = -1;
	else
		tool.drive = preferred != tool.drives.end() ? static_cast<int>(preferred - tool.drives.begin()) : 0;
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
	return format(STR_C("The drive %s has no variant for a %s switch with the drive on the %s"), m_switch.drives[Family].name.c_str(), Rightswitch ? STR_C("right") : STR_C("left"), Driveright ? STR_C("right") : STR_C("left"));
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
		tool.status = missing_drive_variant(tool.drive, spot->rightswitch, driveright);
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
		tool.status = STR_C("The drive couldn't be placed: ") + error;
		return false;
	}
	tool.status = format(STR_C("Switch %s with the drive %s: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), item.file.c_str(), Switch.name().c_str(), Switch.name().c_str());
	return true;
}

std::optional<editor_mode::switch_drive> editor_mode::find_switch_drive(TTrack const &Switch)
{
	auto const &tool{m_switch};
	scan_switch_drives();
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
		tool.status = format(STR_C("No drive of the switch %s among the described drives"), Switch.name().c_str());
		return false;
	}
	auto const &found{present->standing};
	auto const family{Family >= 0 ? Family : present->family};
	auto const index{Flip ? present->index ^ 1 : present->index};
	template_item item;
	item.file = tool.drives[family].files[index];
	if (item.file.empty())
	{
		tool.status = Flip ? format(STR_C("The drive %s has no variant with the drive on the other side"), tool.drives[family].name.c_str()) : missing_drive_variant(family, (index & 2) != 0, (index & 1) != 0);
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
		tool.status = STR_C("The drive of this switch can't be changed in its file");
		return false;
	}
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		scene::Layers.removed(found.instance, false);
		m_history.pop_back();
		tool.status = STR_C("The drive couldn't be placed: ") + error;
		return false;
	}
	join_last_steps();
	tool.status = format(index & 1 ? STR_C("Switch %s: the drive %s, on the right now") : STR_C("Switch %s: the drive %s, on the left now"), Switch.name().c_str(), item.file.c_str());
	return true;
}

bool editor_mode::render_switch_drive_choice()
{
	auto &tool{m_switch};
	scan_switch_drives();
	if (tool.drives.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No switch drives: no template is described with category: switch drive"));
		return false;
	}
	auto const label = [&](int const Index) { return Index < 0 ? std::string{STR_C("no drive")} : tool.drives[Index].name; };
	ImGui::PushItemWidth(200.0f);
	bool changed{false};
	if (ImGui::BeginCombo(STR_C("Drive"), label(tool.drive).c_str()))
	{
		for (int i = -1; i < static_cast<int>(tool.drives.size()); ++i)
			if (ImGui::Selectable(label(i).c_str(), tool.drive == i))
			{
				tool.drive = i;
				changed = true;
			}
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();
	item_tooltip("Templates described with category: switch drive, the side of the switch and of the drive, and a parameter with the role track for the name of the switch.\nIts events <name>+ and <name>- switch it with the keys by the switch and in the map during the game");
	if (tool.drive >= 0)
	{
		ImGui::SameLine();
		changed |= ImGui::RadioButton(STR_C("diverging side"), &tool.drive_side, 0);
		ImGui::SameLine();
		changed |= ImGui::RadioButton(STR_C("other side"), &tool.drive_side, 1);
	}
	return changed;
}
