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

void editor_mode::render_switch_ui()
{
	auto &tool{m_switch};
	auto const list = [&](bool const Slips) {
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			auto const &entry{tool.templates[i]};
			if (entry.double_slip != Slips)
				continue;
			auto label{entry.source != nullptr ? STR("Scenery: ") + entry.label + "  (" + std::to_string(entry.count) + "x)" : "PLK: " + entry.label};
			label += "##switch" + std::to_string(i);
			if (ImGui::Selectable(label.c_str(), tool.armed == i))
				tool.armed = tool.last = i;
		}
	};
	ImGui::TextDisabled("%s", STR_C("Switch template"));
	ImGui::BeginChild("##switchtemplates", ImVec2(0.0f, 240.0f), true);
	list(false);
	ImGui::EndChild();
	if (std::any_of(tool.templates.begin(), tool.templates.end(), [](editor_track::switch_template const &Template) { return Template.double_slip; }))
	{
		if (tool.armed >= 0 && tool.armed < static_cast<int>(tool.templates.size()) && tool.templates[tool.armed].double_slip)
			ImGui::SetNextItemOpen(true);
		if (ImGui::CollapsingHeader(STR_C("Double slip switches")))
		{
			ImGui::TextDisabled("%s", STR_C("Only at a crossing of two straights"));
			list(true);
		}
	}
	ImGui::PushItemWidth(120.0f);
	ImGui::InputTextWithHint(STR_C("Name##switchname"), switch_name_for_new().c_str(), tool.name, sizeof(tool.name));
	ImGui::PopItemWidth();
	item_tooltip("Name of the next switch, its drive events are named after it; empty: the next free z1, z2...; a number at the end goes up with each switch");
	render_switch_drive_choice();
	if (false == tool.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", tool.status.c_str());
	if (ImGui::SmallButton(STR_C("Collect from the scenery")))
	{
		tool.templates = editor_track::standard_switch_templates();
		auto const found{editor_track::find_switch_templates()};
		tool.templates.insert(tool.templates.end(), found.begin(), found.end());
		tool.collected = true;
		arm_switch();
	}
	item_tooltip("Looks for the switches of the scenery, to use their shapes as the templates");
}
