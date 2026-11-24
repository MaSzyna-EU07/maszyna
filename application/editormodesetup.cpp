/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// The assumptions of the project ("Co rysujesz?", editor-project-setup.md): what the user draws, asked for by a card of the inspector of
// the tracks in a scenery with none, or by the last step of the wizard of a new scenery, and changed later in a window of their own. Kept
// with the map, in its project file where it has one, else in the scenery as //$u setup ...; the defaults of the tools from then on. A
// change of them checks the lines laid before against them, the report in the window of the norms; nothing laid is changed by it

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodeselftest.h"
#include "application/editormodeshared.h"
#include "application/editorflow.h"
#include "application/editorsetupui.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/project/projectFile.hpp"
#include "editor/project/projectSession.hpp"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"
#include "world/Track.h"

#include "imgui/imgui.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

namespace
{

std::filesystem::path project_file_of_map()
{
	auto const map{scene::Layers.layer(scene::layer_handle{1}).name};
	return project::path_of(std::filesystem::path{paths::scenery} / map);
}

bool any_track()
{
	return std::ranges::any_of(simulation::Paths.sequence(), [](TTrack const *Track) { return Track != nullptr && false == Track->m_editorremoved && (Track->iCategoryFlag & 15) == 1; });
}

} // namespace

void editor_mode::setup_update()
{
	if (false == m_setup_read && Global.editor_session && false == scene::Layers.empty())
		setup_read();
	// the lines laid before checked against the assumptions changed: what it came to
	if (m_setup_checking && false == m_norms.checking())
	{
		m_setup_checking = false;
		auto const blocked{m_norms.lines_beyond(norms::tier::block)};
		auto const warned{m_norms.lines_beyond(norms::tier::warn)};
		m_setup_told = blocked + warned == 0 ? std::string{STR_C("The lines laid before meet the norms of these assumptions")}
		                                     : format(STR_C("%zu of the lines laid before don't meet these norms (%zu beyond the hard limits): the report is in the window of the norms. Nothing was changed"),
		                                              warned, blocked);
		WriteLog("Editor: the assumptions of the project changed: " + m_setup_told, logtype::generic);
	}
}

void editor_mode::setup_read()
{
	m_setup_read = true;
	std::optional<project_setup::setup> found;
	// the project file of the map first, then the scenery
	std::error_code error;
	if (auto const file{project_file_of_map()}; std::filesystem::exists(file, error))
		if (auto const loaded{project::load(file)}; loaded.project.has_value() && false == loaded.project->setup.empty())
			found = project_setup::read(std::string_view{loaded.project->setup});
	auto const marked{scene::Layers.marked(scene::layer_handle{1}, "//$u")};
	m_setup_marked = false == marked.empty();
	if (false == found.has_value())
		found = project_setup::read(marked);
	if (found.has_value())
	{
		setup_apply(*found, false);
		WriteLog("Editor: the assumptions of the project: " + project_setup::record(*found), logtype::generic);
	}
	else
		WriteLog("Editor: the scenery has no assumptions of the project yet, the defaults meanwhile", logtype::generic);
}

void editor_mode::setup_apply(project_setup::setup const &Setup, bool const Changed)
{
	// said for the first time, or changed: only a change checks the lines laid before
	auto const before{m_setup};
	m_setup = Setup;
	m_setup_edit = Setup;
	setup_defaults(Setup);
	model_setup();
	if (false == Changed)
		return;
	m_setup_changed = true;
	m_setup_told.clear();
	WriteLog("Editor: the assumptions of the project set: " + project_setup::record(Setup), logtype::generic);
	// the lines laid before: which of them don't meet the norms of these, nothing changed
	if (before.has_value() && *before != Setup && any_track())
	{
		m_norms.check_scenery();
		m_setup_checking = true;
	}
}

void editor_mode::setup_defaults(project_setup::setup const &Setup)
{
	// the speed of the laying and of the plan, the look and the gauge of the tracks laid, the norms
	m_lay.style.velocity = Setup.speed;
	m_lay.style.gauge = Setup.gauge / 1000.0;
	// a texture left empty is the lay tool's own: also where the field was cleared after another one was typed, which stayed
	editor_track::path_style const own;
	m_lay.style.rail = Setup.rails.empty() ? own.rail : Setup.rails;
	m_lay.style.ballast = Setup.ballast.empty() ? own.ballast : Setup.ballast;
	m_route.design.norms.gauge = project_setup::rail_axes(Setup);
	if (m_route.chain.tracks.empty())
		m_route.design.speed = Setup.speed;
	m_norms.defaults(Setup.profile, project_setup::line_class(Setup));
	// the switches: of the rail, the usual one for the speed of their diverging track
	auto &tool{m_switch};
	tool.rail_pick = Setup.rail;
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
	}
	if (auto const usual{usual_switch(Setup.rail, project_setup::diverging_speed(Setup))}; usual >= 0)
	{
		tool.last = usual;
		if (tool.armed >= 0)
			tool.armed = usual;
	}
}

void editor_mode::setup_store()
{
	if (false == m_setup.has_value() || false == m_setup_changed || scene::Layers.empty())
		return;
	m_setup_changed = false;
	auto const line{project_setup::record(*m_setup)};
	auto const moved_out = [this]() {
		// written into the project: the lines of the scenery go
		if (m_setup_marked)
			scene::Layers.mark(scene::layer_handle{1}, "//$u", {});
		m_setup_marked = false;
	};
	if (model_setup())
	{
		// the project of the model is written after the scenery, and may fail to be: model_saved() puts them back into the scenery then,
		// and they stay there till the project is saved
		m_setup_moving = true;
		if (m_setup_unsaved)
			setup_keep_in_scenery();
		else
			moved_out();
		return;
	}
	std::error_code error;
	if (auto const file{project_file_of_map()}; std::filesystem::exists(file, error))
	{
		auto loaded{project::load(file)};
		if (loaded.project.has_value())
		{
			loaded.project->setup = line;
			loaded.project->network.areas.scenery = m_setup->profile;
			if (std::string why; project::save(*loaded.project, file, why))
			{
				moved_out();
				return;
			}
			else
				WriteLog("Editor: the assumptions of the project not written into its file: " + why, logtype::generic);
		}
	}
	setup_keep_in_scenery();
}

void editor_mode::setup_keep_in_scenery()
{
	if (false == m_setup.has_value())
		return;
	scene::Layers.mark(scene::layer_handle{1}, "//$u", {"# the assumptions of the project: what the editor lays the tracks to", project_setup::record(*m_setup)});
	m_setup_marked = true;
}

void editor_mode::setup_project_saved(bool const Saved)
{
	if (false == m_setup_moving)
		return;
	m_setup_moving = false;
	m_setup_unsaved = false == Saved;
	// not saved: the assumptions go back into the scenery, written with its next save, rather than be in neither of them; saved while the
	// scenery holds them still: taken out of it with the next save
	if (false == Saved)
		setup_keep_in_scenery();
	if (false == Saved || m_setup_marked)
		m_setup_changed = true;
}

bool editor_mode::render_setup_card()
{
	if (m_setup.has_value() || false == m_setup_read)
		return false;
	auto const font{ImGui::GetFontSize()};
	ImGui::Spacing();
	ImGui::PushStyleColor(ImGuiCol_ChildBg, ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(std::round(font * 0.6f), std::round(font * 0.5f)));
	if (ImGui::BeginChild("##setupcard", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding))
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::SetWindowFontScale(1.12f);
		ImGui::TextUnformatted(STR_C("What do you draw?"));
		ImGui::SetWindowFontScale(1.0f);
		ImGui::TextDisabled("%s", STR_C("The tracks are laid to these norms and this speed: a curve they don't allow can't be laid"));
		ImGui::PopTextWrapPos();
		ImGui::Spacing();
		setup_ui::fields("card", m_setup_edit);
		ImGui::Spacing();
		if (main_button(STR_C("Start drawing"), ImVec2(-FLT_MIN, tall_button_height())))
			setup_apply(m_setup_edit, true);
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("Changed later in Assumptions of the project: Editor settings, or the line of them in Tracks"));
		ImGui::PopTextWrapPos();
	}
	ImGui::EndChild();
	ImGui::PopStyleVar();
	ImGui::PopStyleColor();
	return true;
}

void editor_mode::render_setup_line()
{
	if (false == m_setup.has_value())
		return;
	ImGui::Spacing();
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", (STR("Assumptions: ") + setup_ui::summary(*m_setup)).c_str());
	ImGui::PopTextWrapPos();
	same_line_if_fits(STR_C("change"));
	ImGui::PushID("setupline");
	if (ImGui::TextLink(STR_C("change")))
	{
		m_setup_edit = *m_setup;
		m_setup_window = true;
	}
	ImGui::PopID();
	item_tooltip("The assumptions of the project: the norms, the speed of the line, the traffic, the track, the gauge, the rail");
}

void editor_mode::render_setup_window()
{
	if (false == m_setup_window)
		return;
	// over the view, beside the window of the settings it's opened from, not over the panels
	auto const pad{std::round(8.0f * Global.ui_scale)};
	ImGui::SetNextWindowPos(ImVec2(editor_ui::view_min().x + pad * 4.0f, ImGui::GetMainViewport()->WorkPos.y + pad * 4.0f), ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(ImVec2(std::round(ImGui::GetFontSize() * 30.0f), 0.0f), ImGuiCond_FirstUseEver);
	if (ImGui::Begin((std::string{STR_C("Assumptions of the project")} + "###setupwindow").c_str(), &m_setup_window, ImGuiWindowFlags_NoCollapse))
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("The defaults of the laying, the plan, the switches and the profile; a field of a tool changes them for one line"));
		ImGui::PopTextWrapPos();
		ImGui::Spacing();
		setup_ui::fields("window", m_setup_edit);
		ImGui::Spacing();
		auto const same{m_setup.has_value() && *m_setup == m_setup_edit};
		ImGui::BeginDisabled(same);
		if (main_button(STR_C("Apply"), ImVec2(0.0f, 0.0f)))
			setup_apply(m_setup_edit, true);
		ImGui::EndDisabled();
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Close")))
			m_setup_window = false;
		ImGui::PushTextWrapPos(0.0f);
		if (m_setup_checking)
			ImGui::TextDisabled("%s", STR_C("Checking the lines laid before against them..."));
		else if (false == m_setup_told.empty())
		{
			editor_flow::message_line(m_setup_told, m_norms.lines_beyond(norms::tier::warn) > 0 ? editor_flow::kBelow : editor_flow::kGood);
			if (m_norms.lines_beyond(norms::tier::warn) > 0 && ImGui::Button(STR_C("Show the report")))
				m_norms.set_open(true);
		}
		ImGui::TextDisabled("%s", STR_C("A change doesn't alter what's laid: the tracks laid before stay as they are"));
		ImGui::PopTextWrapPos();
	}
	ImGui::End();
}

namespace
{

void expectation(bool const Passed, std::string const &What, std::string const &Got, std::string const &Wanted)
{
	++editor_selftest::expectations();
	if (Passed)
	{
		WriteLog("SELFTEST PASS " + What);
		return;
	}
	++editor_selftest::failures();
	WriteLog("SELFTEST FAIL " + What + ": got " + Got + ", expected " + Wanted);
}

std::string rest_of(std::istringstream &Words)
{
	std::string result;
	for (std::string word; Words >> word;)
		result += (result.empty() ? "" : " ") + word;
	return result;
}

} // namespace

// setup [<key> <value>]...                the assumptions of the project set as the card's Start sets them, the keys of their record over the
//                                         ones there are (norms, speed, traffic, track, gauge, rail)
// setupwindow [close]                     the window of the assumptions of the project opened, as from the settings, or closed
// expect setup none | expect setup <key> <value>   the assumptions of the scenery, none said yet, or the value of a key
// expect setupmark <present|absent>       the scenery holds the line of the assumptions (//$u), or not
// expect layspeed <km/h>                  the speed the lay tool lays at
editor_mode::selftest_result editor_mode::selftest_setup(std::string const &command, std::istringstream &words)
{
	if (command == "setup")
	{
		auto setup{setup_now()};
		auto const line{project_setup::read(std::string_view{project_setup::record(setup) + " " + rest_of(words)})};
		if (line.has_value())
			setup = *line;
		setup_apply(setup, true);
		WriteLog("SELFTEST setup: " + project_setup::record(setup));
		return selftest_result::handled;
	}
	if (command == "setupwindow")
	{
		std::string state;
		words >> state;
		m_setup_edit = setup_now();
		m_setup_window = state != "close";
		WriteLog(std::string{"SELFTEST setupwindow: "} + (m_setup_window ? "open" : "closed"));
		return selftest_result::handled;
	}
	if (command != "expect")
		return selftest_result::unhandled;
	auto const start{words.tellg()};
	std::string kind;
	words >> kind;
	auto what{words.str()};
	what.erase(0, std::min(what.size(), what.find("expect") + 6));
	what.erase(0, std::min(what.size(), what.find_first_not_of(' ')));
	auto const &lay{m_lay};
	if (kind == "setup")
	{
		std::string key;
		std::string wanted;
		words >> key >> wanted;
		if (key == "none")
		{
			expectation(false == m_setup.has_value(), what, m_setup.has_value() ? project_setup::record(*m_setup) : "none", "none");
			return selftest_result::handled;
		}
		if (false == m_setup.has_value())
		{
			expectation(false, what, "none", key + " " + wanted);
			return selftest_result::handled;
		}
		// the value of the key as the record writes it
		std::istringstream record{project_setup::record(*m_setup)};
		std::string got{"-"};
		for (std::string word; record >> word;)
			if (word == key)
				record >> got;
		expectation(got == wanted, what, got, wanted);
		return selftest_result::handled;
	}
	if (kind == "setupmark")
	{
		std::string wanted;
		words >> wanted;
		auto const marked{scene::Layers.empty() ? std::vector<std::string>{} : scene::Layers.marked(scene::layer_handle{1}, "//$u")};
		auto const present{std::ranges::any_of(marked, [](std::string const &Line) { return Line.starts_with("setup"); })};
		expectation(present == (wanted == "present"), what, present ? "present" : "absent", wanted);
		return selftest_result::handled;
	}
	if (kind == "layspeed")
	{
		double wanted{0.0};
		words >> wanted;
		expectation(std::abs(lay.style.velocity - wanted) < 0.5, what, format("%.0f", lay.style.velocity), format("%.0f", wanted));
		return selftest_result::handled;
	}
	// another kind: the expectations of the other groups
	words.clear();
	words.seekg(start);
	return selftest_result::unhandled;
}
