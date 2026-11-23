/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editorsetupui.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorNormsReport.hpp"

#include "imgui/imgui.h"
#include "utilities/translation.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <vector>

namespace setup_ui
{

namespace
{

// a translated name, what follows ## in it left out: the short names of the choices, told apart from the same words elsewhere
std::string shown(char const *Translated)
{
	std::string text{Translated};
	return text.substr(0, text.find("##"));
}

// a texture of the look, typed: empty for the one the lay tool has
bool texture_field(char const *Label, char const *Id, std::string &Value, char const *Hint)
{
	field_row(Label);
	std::array<char, 256> text{};
	std::strncpy(text.data(), Value.c_str(), text.size() - 1);
	if (false == ImGui::InputTextWithHint(Id, Hint, text.data(), text.size()))
		return false;
	Value = text.data();
	return true;
}

} // namespace

char const *profile_label(norms::profile_id const Profile)
{
	switch (Profile)
	{
	case norms::profile_id::tsi: return "TSI";
	case norms::profile_id::en13803: return "EN 13803";
	case norms::profile_id::ch: return "CH";
	default: return "PLK";
	}
}

std::string summary(project_setup::setup const &Setup)
{
	return format("%s · %.0f km/h · %s · %s · %d mm · %s", profile_label(Setup.profile), Setup.speed, norms_report::traffic_name(Setup.traffic),
	              Setup.track == norms::track_kind::side ? STR_C("siding") : STR_C("main track"), Setup.gauge, Setup.rail.c_str());
}

bool fields(char const *Id, project_setup::setup &Setup)
{
	bool changed{false};
	ImGui::PushID(Id);
	if (begin_fields("##setupfields", 0.3f))
	{
		field_row("Norms");
		if (int profile{static_cast<int>(Setup.profile)};
		    segmented("setupnorms", profile,
		              {{static_cast<int>(norms::profile_id::plk), "PLK", "PKP PLK Id-1 (ST-T1-A6): the values preferred, the normal and the extended limits"},
		               {static_cast<int>(norms::profile_id::tsi), "TSI", "TSI INF alone: the least the law of the EU allows, the loosest; its limits can't be passed either"},
		               {static_cast<int>(norms::profile_id::en13803), "EN 13803", "EN 13803: the normal limits warn, the exceptional ones can't be passed"},
		               {static_cast<int>(norms::profile_id::ch), "CH", "Swiss AB-EBV: the normal limits warn, the greatest ones can't be passed"}}))
		{
			Setup.profile = static_cast<norms::profile_id>(profile);
			changed = true;
		}
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("A map across two countries: the areas of the norms later, in the settings"));
		ImGui::PopTextWrapPos();

		field_row("Line speed");
		changed |= value_chips("setupspeeds", Setup.speed, {40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0, 200.0}, "%.0f");
		ImGui::SetNextItemWidth(std::round(ImGui::GetFontSize() * 5.0f));
		changed |= ImGui::DragScalar("##setupspeed", ImGuiDataType_Double, &Setup.speed, 1.0f, nullptr, nullptr, "%.0f km/h");
		Setup.speed = std::clamp(Setup.speed, 10.0, 350.0);

		field_row("Traffic");
		auto const passenger{shown(STR_C("passenger##traffic"))};
		auto const mixed{shown(STR_C("mixed##traffic"))};
		auto const freight{shown(STR_C("freight##traffic"))};
		if (int traffic{static_cast<int>(Setup.traffic)};
		    segmented("setuptraffic", traffic,
		              {{static_cast<int>(norms::traffic::passenger), passenger.c_str()}, {static_cast<int>(norms::traffic::mixed), mixed.c_str()}, {static_cast<int>(norms::traffic::freight), freight.c_str()}}))
		{
			Setup.traffic = static_cast<norms::traffic>(traffic);
			changed = true;
		}

		field_row("Track");
		auto const main{shown(STR_C("main##track"))};
		auto const side{shown(STR_C("side (station)##track"))};
		if (int track{static_cast<int>(Setup.track)};
		    segmented("setuptrack", track,
		              {{static_cast<int>(norms::track_kind::main), main.c_str(), "A running line or a main track of a station"},
		               {static_cast<int>(norms::track_kind::side), side.c_str(), "A siding of a station: the norms of the side tracks"}}))
		{
			Setup.track = static_cast<norms::track_kind>(track);
			changed = true;
		}

		field_row("Gauge");
		{
			std::vector<segment_option> options;
			std::vector<std::string> names;
			for (auto const gauge : project_setup::gauges)
				names.push_back(format("%d", gauge));
			for (std::size_t i = 0; i < names.size(); ++i)
				options.push_back({project_setup::gauges[i], names[i].c_str()});
			if (int gauge{Setup.gauge}; choice_chips("setupgauge", gauge, options))
			{
				Setup.gauge = gauge;
				changed = true;
			}
		}

		field_row("Rail");
		{
			int rail{0};
			for (std::size_t i = 0; i < project_setup::rail_kinds.size(); ++i)
				if (project_setup::rail_kinds[i] == Setup.rail)
					rail = static_cast<int>(i);
			if (segmented("setuprail", rail, {{0, "60E1"}, {1, "49E1"}, {2, "S42"}}))
			{
				Setup.rail = std::string{project_setup::rail_kinds[static_cast<std::size_t>(rail)]};
				changed = true;
			}
			ImGui::PushTextWrapPos(0.0f);
			ImGui::TextDisabled("%s", STR_C("the switches of this rail first"));
			ImGui::PopTextWrapPos();
		}
		ImGui::EndTable();
	}
	if (ImGui::TreeNode(STR_C("More")))
	{
		ImGui::TextDisabled("%s", STR_C("Look of the tracks laid: empty, rails and trackbed as now"));
		if (begin_fields("##setuplook", 0.3f))
		{
			changed |= texture_field("Rails##setuplook", "##setuprails", Setup.rails, "rail_screw_used1");
			changed |= texture_field("Trackbed##setuplook", "##setupballast", Setup.ballast, "1435mm/tpbps-new2");
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}
	ImGui::PopID();
	return changed;
}

} // namespace setup_ui
