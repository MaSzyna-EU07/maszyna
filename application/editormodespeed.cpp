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
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace
{

double const kSpeedLineReach{5000.0}; // m each way, for the line through the selected path
double const kSpeedOverlayRange{3000.0};

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == 1 && false == Track.m_editorremoved && Track.m_road == nullptr && editor_track::is_supported(Track);
}

std::string speed_text(double const Speed)
{
	return Speed > 0.0 ? format("%.0f", Speed) : std::string{"none"};
}

} // namespace

void editor_mode::speed_start()
{
	auto &state{m_speed};
	state.queue.clear();
	state.next = 0;
	state.results.clear();
	state.selected = -1;
	state.status.clear();
	state.options.norms = m_route.design.norms;
	auto const add = [&](TTrack *Track) {
		for (int i = 0; i < static_cast<int>(Track->m_paths.size()); ++i)
			state.queue.emplace_back(Track, i);
	};
	if (state.scope == 0)
	{
		auto *track{selected_track()};
		if (track == nullptr || false == rail(*track))
		{
			state.status = STR_C("Select a track first");
			return;
		}
		editor_track::chain line;
		std::string error;
		if (track->eType == tt_Normal && editor_track::find_run(*track, kSpeedLineReach, line, error))
		{
			for (auto *member : line.tracks)
				add(member);
			// the switches at the ends of the line too
			for (auto *end : {line.tracks.front(), line.tracks.back()})
				for (auto *neighbour : editor_track::neighbours(*end))
					if (neighbour->eType == tt_Switch && rail(*neighbour) && std::none_of(state.queue.begin(), state.queue.end(), [&](auto const &Entry) { return Entry.first == neighbour; }))
						add(neighbour);
		}
		else
		{
			add(track);
		}
	}
	else
	{
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && rail(*track))
				add(track);
	}
	state.history = m_history.size();
}

void editor_mode::speed_step()
{
	auto &state{m_speed};
	if (state.next >= state.queue.size())
		return;
	auto const deadline{std::chrono::steady_clock::now() + std::chrono::milliseconds(15)};
	while (state.next < state.queue.size() && std::chrono::steady_clock::now() < deadline)
	{
		auto const &entry{state.queue[state.next++]};
		if (entry.first->m_editorremoved)
			continue;
		state.results.push_back(speed_check::check(*entry.first, entry.second, state.options));
	}
	if (state.next < state.queue.size())
		return;
	// the worst first: the violations, then the warnings, each by the largest excess of the speed set over the safe one
	auto const excess = [&](speed_check::verdict const &Verdict) { return Verdict.set > 0.0 ? Verdict.set - Verdict.safe : state.options.top - Verdict.safe; };
	auto const rank = [&](speed_check::verdict const &Verdict) { return Verdict.violated(state.options) ? 0 : Verdict.warned(state.options) ? 1 : 2; };
	std::stable_sort(state.results.begin(), state.results.end(), [&](auto const &A, auto const &B) {
		auto const a{rank(A)};
		auto const b{rank(B)};
		if (a != b)
			return a < b;
		return a < 2 ? excess(A) > excess(B) : A.safe < B.safe;
	});
	auto const count{std::count_if(state.results.begin(), state.results.end(), [&](auto const &Verdict) { return Verdict.violated(state.options); })};
	auto const warnings{std::count_if(state.results.begin(), state.results.end(), [&](auto const &Verdict) { return Verdict.warned(state.options); })};
	state.status = format(STR_C("%zu paths checked, %d with the speed above what the geometry allows, %d switch branch warnings"), state.results.size(), static_cast<int>(count), static_cast<int>(warnings));
	WriteLog("Editor: speed check - " + state.status, logtype::generic);
}

std::vector<int> editor_mode::speed_listed() const
{
	std::vector<int> listed;
	for (int i = 0; i < static_cast<int>(m_speed.results.size()); ++i)
	{
		auto const &verdict{m_speed.results[i]};
		if ((m_speed.limits == 1 && verdict.set <= 0.0) || (m_speed.limits == 2 && verdict.set > 0.0))
			continue;
		if (m_speed.show_all || verdict.violated(m_speed.options) || verdict.warned(m_speed.options))
			listed.push_back(i);
	}
	return listed;
}

void editor_mode::speed_focus(int const Index)
{
	auto &state{m_speed};
	if (Index < 0 || Index >= static_cast<int>(state.results.size()))
		return;
	state.selected = Index;
	auto const &verdict{state.results[Index]};
	auto const &paths{simulation::Paths.sequence()};
	if (std::find(paths.begin(), paths.end(), verdict.track) == paths.end() || verdict.track->m_editorremoved)
		return;
	m_node = verdict.track;
	ui()->set_node(m_node);
	focus_track(*verdict.track, verdict.path, verdict.position);
}

void editor_mode::speed_apply(std::vector<int> const &Indices, bool const Branches)
{
	auto &state{m_speed};
	struct change
	{
		TTrack *track;
		double limits[2]; // km/h for the path, and the diverging track of a switch, -1: none
	};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<change> changes;
	std::size_t locked{0};
	for (auto const index : Indices)
	{
		auto const &verdict{state.results[index]};
		auto const safe{Branches ? verdict.safe : verdict.hard};
		if (safe >= state.options.top || verdict.track->m_editorremoved)
			continue;
		if (false == scene::Layers.editable(verdict.track))
		{
			++locked;
			continue;
		}
		auto existing{std::find_if(changes.begin(), changes.end(), [&](change const &Change) { return Change.track == verdict.track; })};
		if (existing == changes.end())
		{
			changes.push_back({verdict.track, {speed_check::path_velocity(*verdict.track, 0), speed_check::path_velocity(*verdict.track, 1)}});
			states.emplace_back(verdict.track, editor_track::capture(*verdict.track));
			existing = std::prev(changes.end());
		}
		auto &limit{existing->limits[verdict.track->eType == tt_Switch ? std::clamp(verdict.path, 0, 1) : 0]};
		auto const speed{speed_check::suggested(safe)};
		limit = limit > 0.0 ? std::min(limit, speed) : speed;
	}
	auto const skipped{locked > 0 ? format(", %zu in the files which can't be changed are left as they are", locked) : std::string{}};
	if (changes.empty())
	{
		if (locked > 0)
			state.status = STR_C("Nothing changed") + skipped;
		return;
	}
	push_track_snapshot(std::move(states));
	for (auto const &entry : changes)
	{
		auto const velocity{entry.track->eType == tt_Switch ? speed_check::switch_velocity(entry.limits[0], entry.limits[1]) : entry.limits[0]};
		editor_track::velocity(*entry.track, velocity);
		entry.track->mark_dirty();
	}
	for (auto &verdict : state.results)
		verdict.set = speed_check::path_velocity(*verdict.track, verdict.path);
	state.history = m_history.size();
	state.status = format(STR_C("Speed limits of %zu paths lowered to the safe ones, Ctrl+Z brings them back"), changes.size()) + skipped;
	WriteLog("Editor: speed check - " + state.status, logtype::generic);
}

void editor_mode::render_speed_body()
{
	auto &state{m_speed};
	ImGui::TextDisabled(STR_C("Speed limits of the paths against the speed their geometry allows:\nradius and cant, cant ramps, curves without the transitions, switch branches"));
	ImGui::RadioButton(STR_C("Line through the selected path"), &state.scope, 0);
	ImGui::SameLine();
	ImGui::RadioButton(STR_C("Whole scenery"), &state.scope, 1);
	bool const running{state.next < state.queue.size()};
	if (running)
	{
		auto const progress{static_cast<float>(state.next) / static_cast<float>(state.queue.size())};
		ImGui::ProgressBar(progress, ImVec2(-70.0f, 0.0f), format(STR_C("Checking %zu / %zu"), state.next, state.queue.size()).c_str());
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Cancel")))
			state.queue.resize(state.next);
	}
	else if (ImGui::Button(STR_C("Check")))
	{
		speed_start();
	}
	if (ImGui::TreeNode(STR_C("Limits")))
	{
		auto &options{state.options};
		ImGui::PushItemWidth(100.0f);
		ImGui::InputDouble(STR_C("Unbalanced acceleration (m/s2)"), &options.norms.unbalanced, 0.0, 0.0, "%.2f");
		ImGui::InputDouble(STR_C("Switch branches (m/s2)"), &options.branch_unbalanced, 0.0, 0.0, "%.2f");
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("Unbalanced acceleration allowed on the diverging tracks of the switches, which have no cant.\n0.65 m/s2 gives 40 km/h for R 190 m and 100 km/h for R 1200 m"));
		ImGui::InputDouble(STR_C("Cant change rate (mm/s)"), &options.norms.cant_rate, 0.0, 0.0, "%.0f");
		ImGui::InputDouble(STR_C("Cant ramp 1:(k*V), k"), &options.norms.ramp_factor, 0.0, 0.0, "%.1f");
		ImGui::InputDouble(STR_C("Unbalanced acc. change rate (m/s3)"), &options.norms.jerk, 0.0, 0.0, "%.2f");
		ImGui::InputDouble(STR_C("Reference length (m)"), &options.reference_length, 0.0, 0.0, "%.0f");
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("Length over which an abrupt change of the curvature or the cant is felt,\nabout the length of a car. Shorter: stricter for the curves without the transitions"));
		ImGui::PopItemWidth();
		options.norms.unbalanced = std::max(0.05, options.norms.unbalanced);
		options.branch_unbalanced = std::max(0.05, options.branch_unbalanced);
		options.norms.cant_rate = std::max(1.0, options.norms.cant_rate);
		options.norms.ramp_factor = std::max(1.0, options.norms.ramp_factor);
		options.norms.jerk = std::max(0.01, options.norms.jerk);
		options.reference_length = std::clamp(options.reference_length, 2.0, 200.0);
		ImGui::TextDisabled(STR_C("Change the limits, then Check again"));
		ImGui::TreePop();
	}
	if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	if (state.results.empty())
		return;
	if (state.history != m_history.size())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), STR_C("The scenery changed since the check"));
	ImGui::SetNextItemWidth(190.0f);
	ImGui::Combo("##speedlimits", &state.limits, "All the paths\0With a speed limit set\0Without a speed limit\0");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("Paths without a speed limit of their own run at the line speed,\nthose are listed whenever the geometry limits the speed at all"));
	ImGui::SameLine();
	ImGui::Checkbox(STR_C("Within their limits too"), &state.show_all);
	auto const listed{speed_listed()};
	std::vector<int> violated;
	for (auto const index : listed)
		if (state.results[index].violated(state.options))
			violated.push_back(index);
	if (false == violated.empty())
	{
		ImGui::SameLine();
		if (ImGui::Button(format(STR_C("Set the safe speed on %zu paths"), violated.size()).c_str()))
			speed_apply(violated, false);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("Lowers the speed limit of each listed path to its safe speed, rounded down to 10 km/h (5 km/h below 40).\n"
			                  "The limits of the switch branches are only warnings and aren't applied here, the button of a single path does.\n"
			                  "A switch has one velocity for both its tracks: negative limits only the diverging one, the main one keeping no limit.\n"
			                  "When the main track needs a limit of its own, the lower of the two applies to both"));
	}
	auto const namewidth{std::max(120.0f, ImGui::GetContentRegionAvail().x * 0.3f)};
	ImGui::Columns(6, STR_C("speedcolumns"), false);
	ImGui::SetColumnWidth(0, namewidth + 8.0f);
	for (auto const *header : {"Path", "Set", "Safe", "Suggested", "Cause", "R / cant"})
	{
		ImGui::TextDisabled("%s", header);
		ImGui::NextColumn();
	}
	ImGui::Columns(1);
	bool const chosen{state.selected >= 0 && state.selected < static_cast<int>(state.results.size())};
	auto const details{chosen ? ImGui::GetFrameHeightWithSpacing() + ImGui::GetTextLineHeightWithSpacing() * 5.0f : 0.0f};
	ImGui::BeginChild("speedresults", ImVec2(0.0f, -details), true);
	ImGui::Columns(6, STR_C("speedrows"), false);
	ImGui::SetColumnWidth(0, namewidth);
	ImGuiListClipper clipper;
	clipper.Begin(static_cast<int>(listed.size()));
	while (clipper.Step())
		for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
		{
			auto const index{listed[row]};
			auto const &verdict{state.results[index]};
			bool const violated{verdict.violated(state.options)};
			bool const bad{violated || verdict.warned(state.options)};
			auto const colour{violated ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f) : ImVec4(1.0f, 0.75f, 0.3f, 1.0f)};
			auto name{verdict.track->name().empty() ? std::string{"(noname)"} : verdict.track->name()};
			if (verdict.track->eType == tt_Switch)
				name += verdict.path == 0 ? " (main)" : " (branch)";
			if (bad)
				ImGui::PushStyleColor(ImGuiCol_Text, colour);
			if (ImGui::Selectable((name + "##speed" + std::to_string(index)).c_str(), state.selected == index, ImGuiSelectableFlags_SpanAllColumns))
				speed_focus(index);
			if (ImGui::IsItemHovered())
			{
				if (bad)
					ImGui::PopStyleColor();
				ImGui::SetTooltip("%s", speed_check::explain(verdict, state.options).c_str());
				if (bad)
					ImGui::PushStyleColor(ImGuiCol_Text, colour);
			}
			ImGui::NextColumn();
			ImGui::TextUnformatted(speed_text(verdict.set).c_str());
			ImGui::NextColumn();
			ImGui::TextUnformatted(verdict.safe >= state.options.top ? STR_C("any") : format("%.0f", verdict.safe).c_str());
			ImGui::NextColumn();
			ImGui::TextUnformatted(verdict.safe >= state.options.top ? "-" : format("%.0f", speed_check::suggested(verdict.safe)).c_str());
			ImGui::NextColumn();
			ImGui::TextUnformatted(speed_check::describe(verdict.why));
			ImGui::NextColumn();
			if (verdict.radius > 0.0)
				ImGui::Text(STR_C("%.0f m / %.0f mm"), verdict.radius, verdict.cant);
			else
				ImGui::TextUnformatted("-");
			ImGui::NextColumn();
			if (bad)
				ImGui::PopStyleColor();
		}
	ImGui::Columns(1);
	ImGui::EndChild();
	if (chosen)
	{
		auto const &verdict{state.results[state.selected]};
		ImGui::TextWrapped("%s", speed_check::explain(verdict, state.options).c_str());
		if ((verdict.violated(state.options) || verdict.warned(state.options)) && ImGui::Button(STR_C("Set the safe speed on this path")))
			speed_apply({state.selected}, true);
	}
}

void editor_mode::draw_speed_overlay() const
{
	auto const &state{m_speed};
	if (false == state.open || state.results.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (int i = 0; i < static_cast<int>(state.results.size()); ++i)
	{
		auto const &verdict{state.results[i]};
		bool const violated{verdict.violated(state.options)};
		if ((false == violated && false == verdict.warned(state.options)) || glm::distance(verdict.position, camera) > kSpeedOverlayRange)
			continue;
		ImVec2 screen;
		if (false == projection.project(verdict.position, screen))
			continue;
		auto const colour{i == state.selected ? IM_COL32(255, 255, 80, 255) : violated ? IM_COL32(255, 70, 60, 230) : IM_COL32(255, 180, 60, 200)};
		drawlist->AddCircle(screen, 12.0f, colour, 16, 2.5f);
		auto const text{format("%s > %.0f", speed_text(verdict.set).c_str(), verdict.safe)};
		drawlist->AddText(ImVec2(screen.x + 14.0f, screen.y - 7.0f), colour, text.c_str());
	}
}
