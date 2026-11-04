/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/


#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include "application/editormodetrackshared.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>

using namespace track_mode;

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
		auto const side = [](std::string const &Side) { return Side == "left" ? 0 : Side == "right" ? 1 : -1; };
		auto const hand{side(info.switch_hand)};
		auto const drive{side(info.switch_drive)};
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
	}
	std::sort(tool.drives.begin(), tool.drives.end(), [](switch_tool::drive_family const &A, switch_tool::drive_family const &B) { return A.name < B.name; });
	tool.drive = tool.drives.empty() ? -1 : 0;
}

bool editor_mode::place_switch_drive(TTrack &Switch)
{
	auto &tool{m_switch};
	if (tool.drive < 0 || tool.drive >= static_cast<int>(tool.drives.size()) || Switch.eType != tt_Switch || Switch.m_paths.size() < 2)
		return false;
	auto const &main{Switch.m_paths[0]};
	auto const &diverging{Switch.m_paths[1]};
	auto const start{main.points[segment_data::point::start]};
	auto const &control{main.points[segment_data::point::control1]};
	auto heading{plan_of(control != glm::dvec3{} ? control : main.points[segment_data::point::end] - start)};
	if (glm::length(heading) < 1e-9)
		return false;
	heading = glm::normalize(heading);
	glm::dvec2 const right{-heading.y, heading.x};
	bool const rightswitch{glm::dot(plan_of(diverging.points[segment_data::point::end] - start), right) > 0.0};
	bool const driveright{(tool.drive_side == 0) == rightswitch};
	auto const &family{tool.drives[tool.drive]};
	template_item item;
	item.file = family.files[(rightswitch ? 2 : 0) + (driveright ? 1 : 0)];
	if (item.file.empty())
	{
		tool.status = format(STR_C("The drive %s has no variant for a %s switch with the drive on the %s"), family.name.c_str(), rightswitch ? STR_C("right") : STR_C("left"), driveright ? STR_C("right") : STR_C("left"));
		return false;
	}
	item.location = start;
	auto yaw{glm::degrees(std::atan2(heading.x, heading.y))};
	item.yaw = yaw < 0.0 ? yaw + 360.0 : yaw;
	item.described = true;
	item.track = Switch.name();
	{
		auto const along{control != glm::dvec3{} ? control : main.points[segment_data::point::end] - start};
		auto const run{glm::length(plan_of(along))};
		auto const roll{static_cast<double>(main.rolls[0])};
		item.tilt = {run > 1e-9 ? -glm::degrees(std::atan(along.y / run)) : 0.0, roll};
		if (Global.bRollFix)
			item.location.y += 0.75 * std::abs(std::sin(glm::radians(roll)));
	}
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		tool.status = STR_C("The drive couldn't be placed: ") + error;
		return false;
	}
	tool.status = format(STR_C("Switch %s with the drive %s: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), item.file.c_str(), Switch.name().c_str(), Switch.name().c_str());
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
