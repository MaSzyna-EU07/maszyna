/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodenorms.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <set>
#include <unordered_set>
#include <utility>

namespace
{

double constexpr kLineReach{5000.0}; // m each way, for the line through the selected path
double constexpr kSceneryReach{20000.0}; // m each way, for the lines of the scenery
double constexpr kBranchReach{300.0}; // m each way around the diverging track of a switch
double constexpr kOverlayRange{3000.0};

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == 1 && false == Track.m_editorremoved && Track.m_road == nullptr && editor_track::is_supported(Track);
}

std::vector<norms_report::span> spans_of(editor_track::route const &Route)
{
	std::vector<norms_report::span> result;
	result.reserve(Route.spans.size());
	for (auto const &span : Route.spans)
		result.push_back({span.track, span.path, span.forward});
	return result;
}

int rank(norms::tier const Tier)
{
	switch (Tier)
	{
	case norms::tier::block: return 0;
	case norms::tier::warn: return 1;
	default: return 2;
	}
}

ImVec4 tier_colour(norms::tier const Tier)
{
	switch (Tier)
	{
	case norms::tier::block: return {1.0f, 0.45f, 0.4f, 1.0f};
	case norms::tier::warn: return {1.0f, 0.75f, 0.3f, 1.0f};
	default: return ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled);
	}
}

std::string source_text(norms::finding const &Finding)
{
	if (Finding.from.document.empty())
		return "-";
	return std::string{Finding.from.document} + " " + std::string{Finding.from.place};
}

std::array<char const *, 4> constexpr kProfileKeys{"plk", "tsi", "en", "ch"};

} // namespace

void norms_tool::start()
{
	auto &state{m_state};
	state.queue.clear();
	state.next = 0;
	state.lines.clear();
	state.results.clear();
	state.selected = -1;
	state.status.clear();
	state.options.profile = static_cast<norms::profile_id>(std::clamp(state.profile, 0, static_cast<int>(norms::profile_id::count) - 1));
	if (state.scope == 0)
	{
		auto *track{m_selection.selected_track()};
		if (track == nullptr || false == rail(*track))
		{
			state.status = STR_C("Select a track first");
			return;
		}
		state.queue.push_back({spans_of(editor_track::run_route(*track, kLineReach)), -1});
	}
	else
	{
		// the lines through the plain tracks, then each path of a switch none of them ran along, with the line it lies in
		std::unordered_set<TTrack const *> covered;
		std::set<std::pair<TTrack const *, int>> switch_paths;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || false == rail(*track) || track->eType != tt_Normal || covered.contains(track))
				continue;
			auto const route{editor_track::run_route(*track, kSceneryReach)};
			for (auto const &span : route.spans)
			{
				if (span.track->eType == tt_Normal)
					covered.insert(span.track);
				else
					switch_paths.emplace(span.track, span.path);
			}
			state.queue.push_back({spans_of(route), -1});
		}
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || false == rail(*track) || track->eType != tt_Switch)
				continue;
			for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
			{
				if (switch_paths.contains({track, path}))
					continue;
				auto const route{editor_track::run_route(*track, kBranchReach, path)};
				auto const own{std::ranges::find_if(route.spans, [track, path](editor_track::route_span const &Span) { return Span.track == track && Span.path == path; })};
				if (own == route.spans.end())
					continue;
				state.queue.push_back({spans_of(route), static_cast<int>(std::distance(route.spans.begin(), own))});
			}
		}
	}
	state.history = m_history.history_revision();
}

void norms_tool::defaults(norms::profile_id const Profile, norms::line_class const &Line)
{
	m_state.profile = static_cast<int>(Profile);
	m_state.options.profile = Profile;
	m_state.options.line = Line;
}

void norms_tool::check_scenery()
{
	m_state.scope = 1;
	m_state.open = true;
	start();
}

std::size_t norms_tool::lines_beyond(norms::tier const Tier) const
{
	return static_cast<std::size_t>(std::ranges::count_if(m_state.lines, [Tier](norms_report::checked_line const &Line) {
		return std::ranges::any_of(Line.entries, [Tier](norms_report::entry const &Entry) { return static_cast<int>(Entry.finding.level) >= static_cast<int>(Tier); });
	}));
}

void norms_tool::update()
{
	auto &state{m_state};
	if (state.queue.empty())
		return;
	auto const deadline{std::chrono::steady_clock::now() + std::chrono::milliseconds(15)};
	while (state.next < state.queue.size() && std::chrono::steady_clock::now() < deadline)
	{
		auto const &line{state.queue[state.next++]};
		auto checked{norms_report::check(line.spans, state.options, nullptr, line.own)};
		for (auto const &entry : checked.entries)
		{
			// a place two lines run through is listed once
			if (std::ranges::none_of(state.results, [&entry](norms_report::entry const &Other) {
				    return Other.track == entry.track && Other.path == entry.path && Other.finding.what == entry.finding.what && Other.finding.level == entry.finding.level &&
				           std::abs(Other.finding.value - entry.finding.value) < 1e-6;
			    }))
				state.results.push_back(entry);
		}
		state.lines.push_back(std::move(checked));
	}
	if (state.next >= state.queue.size())
		finish();
}

void norms_tool::finish()
{
	auto &state{m_state};
	// the worst first: the hard limits, the warnings, the values above the recommended ones
	std::ranges::stable_sort(state.results, [](norms_report::entry const &A, norms_report::entry const &B) { return rank(A.finding.level) < rank(B.finding.level); });
	auto const count = [&state](norms::tier const Tier) { return std::ranges::count_if(state.results, [Tier](norms_report::entry const &Entry) { return Entry.finding.level == Tier; }); };
	state.status = format(STR_P("%zu line checked against %s: %d beyond the hard limits, %d warnings, %d above the recommended values",
	                            "%zu lines checked against %s: %d beyond the hard limits, %d warnings, %d above the recommended values", static_cast<long long>(state.lines.size())),
	                      state.lines.size(),
	                      std::string{norms::profile_of(state.options.profile).name}.c_str(), static_cast<int>(count(norms::tier::block)), static_cast<int>(count(norms::tier::warn)),
	                      static_cast<int>(count(norms::tier::prefer)));
	state.queue.clear();
	state.next = 0;
	WriteLog("Editor: norms - " + state.status, logtype::generic);
}

std::vector<int> norms_tool::listed() const
{
	std::vector<int> result;
	for (int i = 0; i < static_cast<int>(m_state.results.size()); ++i)
		if (m_state.notes || m_state.results[i].finding.level != norms::tier::prefer)
			result.push_back(i);
	return result;
}

void norms_tool::focus(int const Index)
{
	auto &state{m_state};
	if (Index < 0 || Index >= static_cast<int>(state.results.size()))
		return;
	state.selected = Index;
	auto const &entry{state.results[Index]};
	auto const &paths{simulation::Paths.sequence()};
	if (entry.track == nullptr || std::ranges::find(paths, entry.track) == paths.end() || entry.track->m_editorremoved)
		return;
	m_selection.set_current_node(entry.track);
	m_view.focus_track(*entry.track, entry.path, entry.position);
}

void norms_tool::forget(scene::basic_node const *Node)
{
	auto &state{m_state};
	auto const removed{std::erase_if(state.results, [Node](norms_report::entry const &Entry) { return Entry.track == Node; })};
	if (removed > 0)
		state.selected = -1;
	for (auto &line : state.lines)
		std::erase_if(line.entries, [Node](norms_report::entry const &Entry) { return Entry.track == Node; });
	// the span whose findings alone are kept moves down by the spans taken out before it; a line whose own span went has nothing left
	// to report
	auto const gone = [Node](norms_report::span const &Span) { return Span.track == Node; };
	auto line{state.queue.begin()};
	while (line != state.queue.end())
	{
		if (line->own >= 0 && line->own < static_cast<int>(line->spans.size()))
		{
			if (gone(line->spans[line->own]))
			{
				line = state.queue.erase(line);
				continue;
			}
			line->own -= static_cast<int>(std::count_if(line->spans.begin(), line->spans.begin() + line->own, gone));
		}
		std::erase_if(line->spans, gone);
		++line;
	}
}

void norms_tool::render_class()
{
	auto &options{m_state.options};
	auto &line{options.line};
	if (begin_fields_fitted("##normsprofile", 0.55f, {"Profile"}))
	{
		field_row("Profile");
		auto const current{std::string{norms::profile_of(static_cast<norms::profile_id>(m_state.profile)).name}};
		if (ImGui::BeginCombo("##normsprofile", current.c_str()))
		{
			for (int i = 0; i < static_cast<int>(norms::profile_id::count); ++i)
				if (ImGui::Selectable(std::string{norms::profile_of(static_cast<norms::profile_id>(i)).name}.c_str(), i == m_state.profile))
					m_state.profile = i;
			ImGui::EndCombo();
		}
		item_tooltip("PLK: P0 recommended, beyond P1 a warning, beyond P2 the hard limit. The limits of the TSI hold in every profile");
		ImGui::EndTable();
	}
	// the class of the line folded to a line of what it is
	std::array<char const *, 3> const kinds{norms_report::traffic_name(norms::traffic::passenger), norms_report::traffic_name(norms::traffic::mixed), norms_report::traffic_name(norms::traffic::freight)};
	std::array<char const *, 2> const tracks{STR_C("main track"), STR_C("siding")};
	auto const vmin{line.vmin > 0.0 ? line.vmin : norms::slow_speed(line.kind, line.vmax)};
	auto const summary{format(STR_C("Line class: %.0f km/h, slow trains %.0f km/h, %s, %s"), line.vmax, vmin, kinds[static_cast<std::size_t>(line.kind)], tracks[static_cast<std::size_t>(line.track)])};
	if (false == ImGui::TreeNode("##normsclass", "%s", summary.c_str()))
		return;
	if (begin_fields_fitted("##normsclassfields", 0.55f, {"Line speed (km/h)", "Slow trains (km/h)", "Traffic", "Track", "Load (Tg a year)"}))
	{
		field_row("Line speed (km/h)");
		ImGui::InputDouble("##normsvmax", &line.vmax, 0.0, 0.0, "%.0f");
		item_tooltip("For the tracks with no speed of their own, or for all of them");
		field_row("Slow trains (km/h)");
		ImGui::InputDouble("##normsvmin", &line.vmin, 0.0, 0.0, "%.0f");
		item_tooltip("The cant excess is checked at this speed. 0: by the type of the line, ST-T1-A6 tab. 8.1");
		field_row("Traffic");
		auto kind{static_cast<int>(line.kind)};
		if (ImGui::Combo("##normstraffic", &kind, kinds.data(), static_cast<int>(kinds.size())))
			line.kind = static_cast<norms::traffic>(kind);
		field_row("Track");
		auto track{static_cast<int>(line.track)};
		if (ImGui::Combo("##normstrack", &track, tracks.data(), static_cast<int>(tracks.size())))
			line.track = static_cast<norms::track_kind>(track);
		field_row("Load (Tg a year)");
		ImGui::InputDouble("##normsload", &line.load, 0.0, 0.0, "%.0f");
		item_tooltip("The tonnage over the track a year, for the cant excess of ST-T1-A6 tab. 9.8");
		ImGui::EndTable();
	}
	line.vmax = std::clamp(line.vmax, 10.0, 350.0);
	line.vmin = std::clamp(line.vmin, 0.0, line.vmax);
	line.load = std::clamp(line.load, 0.0, 100.0);
	ImGui::Checkbox(STR_C("The speed of the tracks where they have one"), &options.track_speeds);
	bool platform{(line.features & norms::feature::platform) != 0};
	if (ImGui::Checkbox(STR_C("Tracks at a platform"), &platform))
		line.features = platform ? line.features | norms::feature::platform : line.features & ~norms::feature::platform;
	ImGui::TextDisabled("%s", STR_C("Change the class, then Check again"));
	ImGui::TreePop();
}

void norms_tool::render_results()
{
	auto &state{m_state};
	auto const listed_rows{listed()};
	bool const chosen{state.selected >= 0 && state.selected < static_cast<int>(state.results.size())};
	auto const details{chosen ? ImGui::GetTextLineHeightWithSpacing() * 3.0f : 0.0f};
	// each column as wide as what's in it and its name and a little apart, the last one to the edge; the names stay at the top as the
	// list scrolls
	ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(std::round(ImGui::GetFontSize() * 0.6f), ImGui::GetStyle().CellPadding.y));
	// down to the bottom of the window, but some rows at least where the window is short (docked across the bottom), which scrolls then
	auto const height{std::max(ImGui::GetContentRegionAvail().y - details, ImGui::GetTextLineHeightWithSpacing() * 8.0f)};
	if (ImGui::BeginTable("normsresults", 6, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersOuter, ImVec2(0.0f, height)))
	{
		for (int column = 0; column < 6; ++column)
			ImGui::TableSetupColumn(nullptr, column == 5 ? ImGuiTableColumnFlags_WidthStretch : ImGuiTableColumnFlags_None);
		ImGui::TableSetupScrollFreeze(0, 1);
		ImGui::TableNextRow();
		for (auto const *header : {"Path", "Tier", "Parameter", "Value", "Limit", "Source"})
		{
			ImGui::TableNextColumn();
			ImGui::TextDisabled("%s", STR_C(header));
		}
		ImGuiListClipper clipper;
		clipper.Begin(static_cast<int>(listed_rows.size()));
		while (clipper.Step())
			for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
			{
				auto const index{listed_rows[row]};
				auto const &entry{state.results[index]};
				auto const &finding{entry.finding};
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::PushStyleColor(ImGuiCol_Text, tier_colour(finding.level));
				auto const name{entry.track != nullptr ? name_of(*entry.track) : std::string{"-"}};
				if (ImGui::Selectable((name + "##norms" + std::to_string(index)).c_str(), state.selected == index, ImGuiSelectableFlags_SpanAllColumns))
					focus(index);
				ImGui::PopStyleColor();
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", norms_report::message(finding, entry.kind).c_str());
				ImGui::TableNextColumn();
				ImGui::TextColored(tier_colour(finding.level), "%s", norms_report::tier_name(finding.level));
				ImGui::TableNextColumn();
				ImGui::TextUnformatted(norms_report::parameter_name(finding.what, entry.kind));
				ImGui::TableNextColumn();
				ImGui::TextUnformatted(finding.what == norms::parameter::profile_boundary ? "-" : norms_report::value_text(finding.what, finding.value).c_str());
				ImGui::TableNextColumn();
				ImGui::TextUnformatted(finding.what == norms::parameter::profile_boundary ? "-" : norms_report::value_text(finding.what, finding.limit).c_str());
				ImGui::TableNextColumn();
				ImGui::TextUnformatted(source_text(finding).c_str());
			}
		ImGui::EndTable();
	}
	ImGui::PopStyleVar();
	if (chosen)
	{
		auto const &entry{state.results[state.selected]};
		ImGui::TextWrapped("%s", norms_report::message(entry.finding, entry.kind).c_str());
	}
}

void norms_tool::render_options()
{
	auto &state{m_state};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", STR_C("Checks the geometry of the tracks against the norms of a profile: what is beyond them is listed, nothing is changed"));
	ImGui::PopTextWrapPos();
	segmented("normsscope", state.scope, {{0, STR_C("Line through the selected path")}, {1, STR_C("Whole scenery")}});
	render_class();
	if (false == state.queue.empty())
	{
		auto const progress{static_cast<float>(state.next) / static_cast<float>(state.queue.size())};
		ImGui::ProgressBar(progress, ImVec2(-70.0f, 0.0f), format(STR_C("Checking %zu / %zu"), state.next, state.queue.size()).c_str());
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Cancel")))
			state.queue.resize(state.next);
	}
	else
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertU32ToFloat4(family_colour(2, 0.55f)));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::ColorConvertU32ToFloat4(family_colour(2, 0.75f)));
		if (ImGui::Button(STR_C("Check"), ImVec2(-1.0f, tall_button_height())))
			start();
		ImGui::PopStyleColor(2);
		item_tooltip("The radius, the cant and its deficiency and excess, their changes along the ramps and the transitions and at the joints, the lengths of the straights, arcs and transitions, the vertical curves and the grades");
	}
	if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	if (state.results.empty())
		return;
	if (state.history != m_history.history_revision())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("The scenery changed since the check"));
	ImGui::Checkbox(STR_C("Above the recommended values too"), &state.notes);
	render_results();
}

void norms_tool::draw_overlay() const
{
	auto const &state{m_state};
	if (false == state.open || state.results.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (int i = 0; i < static_cast<int>(state.results.size()); ++i)
	{
		auto const &entry{state.results[i]};
		auto const level{entry.finding.level};
		if (level == norms::tier::prefer || glm::distance(entry.position, camera) > kOverlayRange)
			continue;
		ImVec2 screen;
		if (false == projection.project(entry.position, screen))
			continue;
		auto colour{level == norms::tier::block ? IM_COL32(255, 70, 60, 230) : IM_COL32(255, 180, 60, 200)};
		if (i == state.selected)
			colour = IM_COL32(255, 255, 80, 255);
		drawlist->AddCircle(screen, 12.0f, colour, 16, 2.5f);
		drawlist->AddText(ImVec2(screen.x + 14.0f, screen.y - 7.0f), colour, norms_report::parameter_name(entry.finding.what, entry.kind));
	}
}

editor_selftest_result norms_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command != "normsreport")
		return editor_selftest_result::unhandled;
	std::string file;
	int scope{0};
	std::string profile{"plk"};
	double speed{0.0};
	Words >> file >> scope >> profile >> speed;
	m_state.scope = scope;
	auto const key{std::ranges::find(kProfileKeys, profile)};
	m_state.profile = key != kProfileKeys.end() ? static_cast<int>(std::distance(kProfileKeys.begin(), key)) : 0;
	if (speed > 0.0)
		m_state.options.line.vmax = speed;
	start();
	while (false == m_state.queue.empty())
		update();
	std::string text{norms_report::header(m_state.options)};
	for (auto const &line : m_state.lines)
		text += norms_report::text(line, true);
	std::ofstream out{file, std::ios::binary};
	out << text;
	WriteLog(format("SELFTEST normsreport %s: %s", file.c_str(), m_state.status.c_str()));
	return editor_selftest_result::handled;
}
