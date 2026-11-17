/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the tracks whose names have letters outside of ASCII: listed, nothing renamed. The names the sceneries have go on working as they
// always did, in the editor and in the game; the editor gives no new name outside of ASCII (editor_track::name_valid)

#include "stdafx.h"

#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorTrack.hpp"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/asciiname.h"
#include "utilities/translation.h"

#include <algorithm>
#include <format>
#include <map>
#include <string>
#include <vector>

namespace
{

// the tracks listed: the lanes take their names from their road, the removed ones are gone
bool listed(TTrack const *Track)
{
	return Track != nullptr && false == Track->m_editorremoved && (Track->iCategoryFlag & 0x80) == 0 && Track->m_road == nullptr && false == Track->name().empty() &&
	       false == ascii_name::plain(Track->name());
}

char const *yes_no(bool const Value)
{
	return Value ? STR_C("yes") : STR_C("no");
}

} // namespace

void editor_mode::check_track_names(bool const Report)
{
	m_track_names.clear();
	std::vector<TTrack const *> found;
	std::map<std::string, int> folded; // how many of the names listed give each ASCII name
	for (auto const *track : simulation::Paths.sequence())
	{
		if (false == listed(track))
			continue;
		found.push_back(track);
		++folded[ascii_name::fold(track->name())];
	}
	for (auto const *track : found)
	{
		track_name_entry entry;
		entry.name = ascii_name::readable(track->name());
		entry.ascii = ascii_name::fold(track->name());
		// another path named so already, or another name listed giving the same
		auto const *other{simulation::Paths.find(entry.ascii)};
		entry.clash = (other != nullptr && other != track && false == other->m_editorremoved) || folded[entry.ascii] > 1;
		entry.include = track->from_template() && editor_track::uses_of_name(*track).includes > 0;
		entry.written = track->from_template() ? entry.include : scene::Layers.editable(track);
		m_track_names.push_back(std::move(entry));
	}
	if (false == Report || m_track_names.empty())
		return;
	for (auto const &entry : m_track_names)
		WriteLog(std::format(R"(Editor: track "{}" has characters outside of ASCII in its name; in ASCII "{}"{})", entry.name, entry.ascii, entry.clash ? ", a name taken already" : ""));
	ui()->set_status(format(STR_C("%zu track names have characters outside of ASCII; they work as they are, the list is in Tools"), m_track_names.size()));
}

void editor_mode::render_track_names_window()
{
	if (false == m_track_names_open)
		return;
	ImGui::SetNextWindowSize(ImVec2(620.0f * Global.ui_scale, 380.0f * Global.ui_scale), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin((std::string(STR_C("Track names outside of ASCII")) + "###tracknamesascii").c_str(), &m_track_names_open))
	{
		ImGui::End();
		return;
	}
	ImGui::TextWrapped("%s", STR_C("These names work as they are and nothing here changes them. New names given in the editor are ASCII only. To give one of these tracks an ASCII name, rename it in the track properties: the memory cells and the includes which name it go along, the events named after it are listed there."));
	if (ImGui::SmallButton(STR_C("Check again")))
		check_track_names(false);
	ImGui::SameLine();
	ImGui::TextDisabled(STR_C("Tracks: %zu"), m_track_names.size());
	auto constexpr flags{ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable};
	if (ImGui::BeginTable("##tracknamesascii", 5, flags))
	{
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableSetupColumn(STR_C("Name"));
		ImGui::TableSetupColumn(STR_C("In ASCII"));
		ImGui::TableSetupColumn(STR_C("Taken"));
		ImGui::TableSetupColumn(STR_C("From an include"));
		ImGui::TableSetupColumn(STR_C("In a file the editor writes"));
		ImGui::TableHeadersRow();
		for (auto const &entry : m_track_names)
		{
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(entry.name.c_str());
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(entry.ascii.c_str());
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(yes_no(entry.clash));
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(yes_no(entry.include));
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(yes_no(entry.written));
		}
		ImGui::EndTable();
	}
	ImGui::End();
}
