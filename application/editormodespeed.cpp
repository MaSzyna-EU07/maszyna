/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodespeed.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <array>
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
	return Speed > 0.0 ? format("%.0f", Speed) : std::string{STR_C("(none)")};
}

// the thresholds the paths are checked against, in a tree node
void render_thresholds(speed_check::options &Options)
{
	if (false == ImGui::TreeNode(STR_C("Thresholds")))
		return;
	if (begin_fields("##speedlimits", 0.6f))
	{
		auto const limit = [](char const *Label, char const *Id, double &Value, char const *Format) {
			field_row(Label);
			ImGui::InputDouble(Id, &Value, 0.0, 0.0, Format);
		};
		limit("Unbalanced acceleration (m/s2)", "##speedunbalanced", Options.norms.unbalanced, "%.2f");
		limit("Switch branches (m/s2)", "##speedbranch", Options.branch_unbalanced, "%.2f");
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Unbalanced acceleration allowed on the diverging tracks of the switches, which have no cant.\n0.65 m/s2 gives 40 km/h for R 190 m and 100 km/h for R 1200 m"));
		limit("Cant change rate (mm/s)", "##speedcantrate", Options.norms.cant_rate, "%.0f");
		limit("Cant ramp 1:(k*V), k", "##speedramp", Options.norms.ramp_factor, "%.1f");
		limit("Unbalanced acc. change rate (m/s3)", "##speedjerk", Options.norms.jerk, "%.2f");
		limit("Reference length (m)", "##speedreference", Options.reference_length, "%.0f");
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Length over which an abrupt change of the curvature or the cant is felt,\nabout the length of a car. Shorter: stricter for the curves without the transitions"));
		ImGui::EndTable();
	}
	Options.norms.unbalanced = std::max(0.05, Options.norms.unbalanced);
	Options.branch_unbalanced = std::max(0.05, Options.branch_unbalanced);
	Options.norms.cant_rate = std::max(1.0, Options.norms.cant_rate);
	Options.norms.ramp_factor = std::max(1.0, Options.norms.ramp_factor);
	Options.norms.jerk = std::max(0.01, Options.norms.jerk);
	Options.reference_length = std::clamp(Options.reference_length, 2.0, 200.0);
	ImGui::TextDisabled("%s", STR_C("Change the thresholds, then Check again"));
	ImGui::TreePop();
}

// of the paths listed, the ones above the speed their geometry allows
std::vector<int> violated_of(std::vector<int> const &Listed, std::vector<speed_check::verdict> const &Results, speed_check::options const &Options)
{
	std::vector<int> violated;
	for (auto const index : Listed)
		if (Results[index].violated(Options))
			violated.push_back(index);
	return violated;
}

// the headers of the columns of the list of the paths checked
void result_headers(float const NameWidth)
{
	ImGui::Columns(6, "speedcolumns", false);
	ImGui::SetColumnWidth(0, NameWidth + 8.0f);
	for (auto const *header : {"Path", "Limit", "Safe", "Suggested", "Cause", "R / cant"})
	{
		ImGui::TextDisabled("%s", STR_C(header));
		ImGui::NextColumn();
	}
	ImGui::Columns(1);
}

// the columns of a row of the list after the name of the path
void result_cells(speed_check::verdict const &Verdict, speed_check::options const &Options)
{
	ImGui::NextColumn();
	ImGui::TextUnformatted(speed_text(Verdict.set).c_str());
	ImGui::NextColumn();
	ImGui::TextUnformatted(Verdict.safe >= Options.top ? STR_C("any") : format("%.0f", Verdict.safe).c_str());
	ImGui::NextColumn();
	ImGui::TextUnformatted(Verdict.safe >= Options.top ? "-" : format("%.0f", speed_check::suggested(Verdict.safe)).c_str());
	ImGui::NextColumn();
	ImGui::TextUnformatted(STR_C(speed_check::describe(Verdict.why)));
	ImGui::NextColumn();
	if (Verdict.radius > 0.0)
		ImGui::Text(STR_C("%.0f m / %.0f mm"), Verdict.radius, Verdict.cant);
	else
		ImGui::TextUnformatted("-");
	ImGui::NextColumn();
}

// a row of the list of the paths checked, Selected: the index of the path chosen; Focus takes the index of the path when its name is clicked
template <typename Focus_>
void result_row(speed_check::verdict const &Verdict, speed_check::options const &Options, int const Index, int const &Selected, Focus_ const &Focus)
{
	bool const violated{Verdict.violated(Options)};
	bool const bad{violated || Verdict.warned(Options)};
	auto const colour{violated ? ImVec4(1.0f, 0.45f, 0.4f, 1.0f) : ImVec4(1.0f, 0.75f, 0.3f, 1.0f)};
	auto name{name_of(*Verdict.track)};
	if (Verdict.track->eType == tt_Switch)
		name += Verdict.path == 0 ? STR_C(" (main)") : STR_C(" (branch)");
	if (bad)
		ImGui::PushStyleColor(ImGuiCol_Text, colour);
	if (ImGui::Selectable((name + "##speed" + std::to_string(Index)).c_str(), Selected == Index, ImGuiSelectableFlags_SpanAllColumns))
		Focus(Index);
	if (ImGui::IsItemHovered())
	{
		if (bad)
			ImGui::PopStyleColor();
		ImGui::SetTooltip("%s", speed_check::explain(Verdict, Options).c_str());
		if (bad)
			ImGui::PushStyleColor(ImGuiCol_Text, colour);
	}
	result_cells(Verdict, Options);
	if (bad)
		ImGui::PopStyleColor();
}

// what the check found about the path chosen, true when the button setting its safe speed was clicked
bool result_details(speed_check::verdict const &Verdict, speed_check::options const &Options)
{
	ImGui::TextWrapped("%s", speed_check::explain(Verdict, Options).c_str());
	return (Verdict.violated(Options) || Verdict.warned(Options)) && ImGui::Button(STR_C("Set the safe speed on this path"));
}


// every path of the track
template <typename Queue>
void queue_paths(Queue &Paths, TTrack *Track)
{
	for (int i = 0; i < static_cast<int>(Track->m_paths.size()); ++i)
		Paths.emplace_back(Track, i);
}

// the paths of the line through the track and of the switches at its ends; of the track alone where it lies on no line
template <typename Queue>
void queue_line(Queue &Paths, TTrack *Track)
{
	editor_track::chain line;
	std::string error;
	if (Track->eType != tt_Normal || false == editor_track::find_run(*Track, kSpeedLineReach, line, error))
	{
		queue_paths(Paths, Track);
		return;
	}
	for (auto *member : line.tracks)
		queue_paths(Paths, member);
	// the switches at the ends of the line too
	for (auto *end : {line.tracks.front(), line.tracks.back()})
		for (auto *neighbour : editor_track::neighbours(*end))
			if (neighbour->eType == tt_Switch && rail(*neighbour) && std::ranges::none_of(Paths, [&](auto const &Entry) { return Entry.first == neighbour; }))
				queue_paths(Paths, neighbour);
}

// the speed limits a track is to get: km/h for the path, and the diverging track of a switch, -1: none
struct speed_change
{
	TTrack *track;
	std::array<double, 2> limits;
};

// the limit of the path of the verdict lowered to the safe speed given, the state of its track kept before its first change
void lower_limit(speed_check::verdict const &Verdict, double const Safe, std::vector<speed_change> &Changes, std::vector<std::pair<TTrack *, editor_track::state>> &States)
{
	auto existing{std::ranges::find_if(Changes, [&Verdict](speed_change const &Change) { return Change.track == Verdict.track; })};
	if (existing == Changes.end())
	{
		Changes.push_back({Verdict.track, {speed_check::path_velocity(*Verdict.track, 0), speed_check::path_velocity(*Verdict.track, 1)}});
		States.emplace_back(Verdict.track, editor_track::capture(*Verdict.track));
		existing = std::prev(Changes.end());
	}
	auto &limit{existing->limits[Verdict.track->eType == tt_Switch ? std::clamp(Verdict.path, 0, 1) : 0]};
	auto const speed{speed_check::suggested(Safe)};
	limit = limit > 0.0 ? std::min(limit, speed) : speed;
}

// the button setting the safe speed on the tracks listed over it, with what it does told
template <typename Apply>
void render_safe_speed_button(std::size_t const Count, Apply const &Lower)
{
	if (ImGui::Button(format(STR_P("Set the safe speed on %zu track", "Set the safe speed on %zu tracks", static_cast<long long>(Count)), Count).c_str(), ImVec2(-FLT_MIN, 0.0f)))
		Lower();
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("Lowers the speed limit of each listed path to its safe speed, rounded down to 10 km/h (5 km/h below 40).\n"
		                  "The limits of the switch branches are only warnings and aren't applied here, the button of a single path does.\n"
		                  "A switch has one velocity for both its tracks: negative limits only the diverging one, the main one keeping no limit.\n"
		                  "When the main track needs a limit of its own, the lower of the two applies to both"));
}

} // namespace

void speed_tool::speed_start()
{
	auto &state{m_speed};
	state.queue.clear();
	state.next = 0;
	state.results.clear();
	state.selected = -1;
	state.status.clear();
	state.options.norms = m_design.route_norms();
	if (state.scope == 0)
	{
		auto *track{m_selection.selected_track()};
		if (track == nullptr || false == rail(*track))
		{
			state.status = STR_C("Select a track first");
			return;
		}
		queue_line(state.queue, track);
	}
	else
	{
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && rail(*track))
				queue_paths(state.queue, track);
	}
	state.history = m_history.history_revision();
}

void speed_tool::update()
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

std::vector<int> speed_tool::speed_listed() const
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

void speed_tool::speed_focus(int const Index)
{
	auto &state{m_speed};
	if (Index < 0 || Index >= static_cast<int>(state.results.size()))
		return;
	state.selected = Index;
	auto const &verdict{state.results[Index]};
	auto const &paths{simulation::Paths.sequence()};
	if (std::find(paths.begin(), paths.end(), verdict.track) == paths.end() || verdict.track->m_editorremoved)
		return;
	m_selection.set_current_node(verdict.track);
	m_view.focus_track(*verdict.track, verdict.path, verdict.position);
}

void speed_tool::speed_apply(std::vector<int> const &Indices, bool const Branches)
{
	auto &state{m_speed};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<speed_change> changes;
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
		lower_limit(verdict, safe, changes, states);
	}
	auto const skipped{locked > 0 ? format(STR_C(", %zu in the files which can't be changed are left as they are"), locked) : std::string{}};
	if (changes.empty())
	{
		if (locked > 0)
			state.status = STR_C("Nothing changed") + skipped;
		return;
	}
	m_history.push_track_snapshot(std::move(states));
	for (auto const &entry : changes)
	{
		auto const velocity{entry.track->eType == tt_Switch ? speed_check::switch_velocity(entry.limits[0], entry.limits[1]) : entry.limits[0]};
		editor_track::velocity(*entry.track, velocity);
		entry.track->mark_dirty();
	}
	for (auto &verdict : state.results)
		verdict.set = speed_check::path_velocity(*verdict.track, verdict.path);
	state.history = m_history.history_revision();
	state.status = format(STR_C("Speed limits of %zu paths lowered to the safe ones, Ctrl+Z brings them back"), changes.size()) + skipped;
	WriteLog("Editor: speed check - " + state.status, logtype::generic);
}

void speed_tool::render_options()
{
	auto &state{m_speed};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", STR_C("Checks that the speed on the tracks isn't above what their geometry allows"));
	ImGui::PopTextWrapPos();
	segmented("speedscope", state.scope, {{0, STR_C("Line through the selected path")}, {1, STR_C("Whole scenery")}});
	bool const running{state.next < state.queue.size()};
	if (running)
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
			speed_start();
		ImGui::PopStyleColor(2);
		item_tooltip("Looks at the radius and the cant, the cant ramps, the curves without the transitions and the switch branches");
	}
	render_thresholds(state.options);
	if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	if (state.results.empty())
		return;
	if (state.history != m_history.history_revision())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), STR_C("The scenery changed since the check"));
	auto const *const within{STR_C("Within their limits too")};
	auto const checkbox{ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(within).x};
	ImGui::SetNextItemWidth(std::max(120.0f, ImGui::GetContentRegionAvail().x - checkbox - ImGui::GetStyle().ItemSpacing.x));
	std::array<char const *, 3> const limits{STR_C("All the paths"), STR_C("With a speed limit set"), STR_C("Without a speed limit")};
	ImGui::Combo("##speedlimits", &state.limits, limits.data(), static_cast<int>(limits.size()));
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Paths without a speed limit of their own run at the line speed,\nthose are listed whenever the geometry limits the speed at all"));
	ImGui::SameLine();
	ImGui::Checkbox(within, &state.show_all);
	auto const listed{speed_listed()};
	auto const violated{violated_of(listed, state.results, state.options)};
	if (false == violated.empty())
		render_safe_speed_button(violated.size(), [this, &violated] { speed_apply(violated, false); });
	auto const namewidth{std::max(120.0f, ImGui::GetContentRegionAvail().x * 0.3f)};
	result_headers(namewidth);
	bool const chosen{state.selected >= 0 && state.selected < static_cast<int>(state.results.size())};
	auto const details{chosen ? ImGui::GetFrameHeightWithSpacing() + ImGui::GetTextLineHeightWithSpacing() * 5.0f : 0.0f};
	ImGui::BeginChild("speedresults", ImVec2(0.0f, -details), true);
	ImGui::Columns(6, "speedrows", false);
	ImGui::SetColumnWidth(0, namewidth);
	ImGuiListClipper clipper;
	clipper.Begin(static_cast<int>(listed.size()));
	while (clipper.Step())
		for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
		{
			auto const index{listed[row]};
			result_row(state.results[index], state.options, index, state.selected, [this](int const Index) { speed_focus(Index); });
		}
	ImGui::Columns(1);
	ImGui::EndChild();
	if (chosen && result_details(state.results[state.selected], state.options))
		speed_apply({state.selected}, true);
}

void speed_tool::draw_overlay() const
{
	auto const &state{m_speed};
	if (false == state.open || state.results.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
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

void speed_tool::check_line()
{
	m_speed.scope = 0;
	speed_start();
}

editor_selftest_result speed_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "speedcheck")
	{
		int scope{0};
		Words >> scope;
		m_speed.scope = scope;
		speed_start();
		while (m_speed.next < m_speed.queue.size())
			update();
		WriteLog("SELFTEST speedcheck: " + m_speed.status);
		for (auto const index : speed_listed())
		{
			auto const &verdict{m_speed.results[index]};
			WriteLog(format("SELFTEST speedcheck %s path %d: set %.0f safe %.0f", verdict.track->name().c_str(), verdict.path, verdict.set, verdict.safe));
		}
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}
