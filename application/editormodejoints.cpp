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

#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <tuple>

namespace
{

using geometry::grade_of;
using geometry::plan_of;

double const kLinked{0.02}; // m, ends this close are joined by the simulation
double const kOverlayRange{3000.0};
double const kGauge{1.5}; // m between the rails, for the cant
int const kRailCategory{1};

using kind = editor_mode::joint_issue::kind;

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == kRailCategory && false == Track.m_editorremoved && editor_track::is_supported(Track);
}

bool at_start(editor_track::point_ref const &Point)
{
	return Point.kind == editor_track::point_kind::start;
}

// direction of the path at its end, into the path
glm::dvec3 into(TTrack const &Track, editor_track::point_ref const &Point)
{
	auto const &path{Track.m_paths[Point.path]};
	bool const start{at_start(Point)};
	auto const &point{path.points[start ? segment_data::point::start : segment_data::point::end]};
	auto const &control{path.points[start ? segment_data::point::control1 : segment_data::point::control2]};
	glm::dvec3 const direction{control != glm::dvec3{} ? control : path.points[start ? segment_data::point::end : segment_data::point::start] - point};
	return glm::length(direction) > 1e-9 ? glm::normalize(direction) : glm::dvec3{0.0, 0.0, 1.0};
}

// roll of the path at its end, for the travel into the path, in degrees
double roll_into(TTrack const &Track, editor_track::point_ref const &Point)
{
	auto const &path{Track.m_paths[Point.path]};
	return at_start(Point) ? path.rolls[0] : -path.rolls[1];
}

double plan_angle(glm::dvec3 const &A, glm::dvec3 const &B)
{
	auto const a{plan_of(A)};
	auto const b{plan_of(B)};
	if (glm::length(a) < 1e-9 || glm::length(b) < 1e-9)
		return 0.0;
	return glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(a), glm::normalize(b)), -1.0, 1.0)));
}

using end_key = std::tuple<TTrack const *, int, int>;

end_key key_of(TTrack const *Track, editor_track::point_ref const &Point)
{
	return {Track, Point.path, static_cast<int>(Point.kind)};
}

// pairs of the ends already looked at in the check under way
std::set<std::pair<end_key, end_key>> reported;

char const *kind_name(kind const Kind)
{
	switch (Kind)
	{
	case kind::gap: return "gap";
	case kind::step: return "step";
	case kind::kink: return "kink";
	case kind::cant: return "cant jump";
	case kind::grade: return "grade break";
	default: return "free end";
	}
}

std::string value_text(editor_mode::joint_issue const &Issue)
{
	switch (Issue.type)
	{
	case kind::gap: return format("%.3f m", Issue.value);
	case kind::step: return format("%.1f mm", Issue.value * 1000.0);
	case kind::kink: return format("%.3f deg", Issue.value);
	case kind::cant: return format("%.1f mm", Issue.value);
	case kind::grade: return format("%.1f %%o", Issue.value);
	default: return "-";
	}
}

ImU32 kind_colour(kind const Kind)
{
	switch (Kind)
	{
	case kind::gap: return IM_COL32(255, 70, 60, 235);
	case kind::step: return IM_COL32(255, 140, 50, 230);
	case kind::kink: return IM_COL32(255, 190, 60, 225);
	case kind::cant: return IM_COL32(200, 120, 255, 225);
	case kind::grade: return IM_COL32(90, 200, 255, 220);
	default: return IM_COL32(200, 200, 200, 200);
	}
}

std::string name_of(TTrack const *Track)
{
	if (Track == nullptr)
		return "-";
	return Track->name().empty() ? std::string{"(noname)"} : Track->name();
}

} // namespace

void editor_mode::joints_start()
{
	auto &state{m_joints};
	state.queue.clear();
	state.next = 0;
	state.issues.clear();
	state.selected = -1;
	state.status.clear();
	reported.clear();
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track))
			continue;
		if (state.scope == 0 && glm::distance(track->location(), camera) > state.reach)
			continue;
		state.queue.push_back(track);
	}
	state.history = m_history.size();
}

void editor_mode::joints_check(TTrack &Track)
{
	auto &state{m_joints};
	auto const once = [&](end_key const &A, end_key const &B) { return reported.insert(A < B ? std::make_pair(A, B) : std::make_pair(B, A)).second; };
	for (int i = 0; i < static_cast<int>(Track.m_paths.size()); ++i)
	{
		for (auto const kind_of_end : {editor_track::point_kind::start, editor_track::point_kind::end})
		{
			editor_track::point_ref const point{i, kind_of_end};
			auto const position{editor_track::point_position(Track, point)};
			auto const connections{editor_track::connected_points(Track, position)};
			if (connections.empty())
			{
				joint_issue nearest;
				nearest.value = std::numeric_limits<double>::max();
				for (auto *section : simulation::Region->sections(position, static_cast<float>(state.gap)))
					for (auto const &cell : section->m_cells)
						for (auto *other : cell.m_directories.paths)
						{
							if (other == nullptr || other == &Track || false == rail(*other))
								continue;
							for (int k = 0; k < static_cast<int>(other->m_paths.size()); ++k)
								for (auto const theirkind : {editor_track::point_kind::start, editor_track::point_kind::end})
								{
									editor_track::point_ref const theirs{k, theirkind};
									auto const distance{glm::distance(position, editor_track::point_position(*other, theirs))};
									if (distance > state.gap || distance >= nearest.value)
										continue;
									nearest.track = &Track;
									nearest.point = point;
									nearest.other = other;
									nearest.theirs = theirs;
									nearest.value = distance;
								}
						}
				if (nearest.other != nullptr)
				{
					if (once(key_of(&Track, point), key_of(nearest.other, nearest.theirs)))
					{
						nearest.type = kind::gap;
						nearest.position = position;
						state.issues.push_back(nearest);
					}
				}
				else if (state.free_ends)
				{
					joint_issue open;
					open.type = kind::open;
					open.track = &Track;
					open.point = point;
					open.position = position;
					state.issues.push_back(open);
				}
				continue;
			}
			for (auto const &connection : connections)
			{
				auto *other{connection.first};
				auto const &theirs{connection.second};
				if (false == rail(*other) || false == once(key_of(&Track, point), key_of(other, theirs)))
					continue;
				joint_issue issue;
				issue.track = &Track;
				issue.point = point;
				issue.other = other;
				issue.theirs = theirs;
				issue.position = position;
				auto const step{glm::distance(position, editor_track::point_position(*other, theirs))};
				auto const ours{into(Track, point)};
				auto const their{into(*other, theirs)};
				auto const angle{plan_angle(ours, -their)};
				auto const cant{kGauge * 1000.0 * std::abs(std::sin(glm::radians(roll_into(Track, point) + roll_into(*other, theirs))))};
				auto const grade{std::abs(grade_of(ours) + grade_of(their)) * 1000.0};
				auto const add = [&](kind const Kind, double const Value) {
					issue.type = Kind;
					issue.value = Value;
					state.issues.push_back(issue);
				};
				if (step > state.step)
					add(kind::step, step);
				if (angle > state.angle)
					add(kind::kink, angle);
				if (cant > state.cant)
					add(kind::cant, cant);
				if (grade > state.grade)
					add(kind::grade, grade);
			}
		}
	}
}

void editor_mode::joints_step()
{
	auto &state{m_joints};
	if (state.next >= state.queue.size())
		return;
	auto const deadline{std::chrono::steady_clock::now() + std::chrono::milliseconds(15)};
	while (state.next < state.queue.size() && std::chrono::steady_clock::now() < deadline)
	{
		auto *track{state.queue[state.next++]};
		if (track->m_editorremoved)
			continue;
		joints_check(*track);
	}
	if (state.next < state.queue.size())
		return;
	std::stable_sort(state.issues.begin(), state.issues.end(), [](joint_issue const &A, joint_issue const &B) {
		if (A.type != B.type)
			return A.type < B.type;
		return A.value > B.value;
	});
	std::array<int, 6> counts{};
	for (auto const &issue : state.issues)
		++counts[static_cast<int>(issue.type)];
	state.status = format(STR_C("%zu paths checked: %d gaps, %d steps, %d kinks, %d cant jumps, %d grade breaks"), state.queue.size(), counts[0], counts[1], counts[2], counts[3], counts[4]);
	if (state.free_ends)
		state.status += format(STR_C(", %d free ends"), counts[5]);
	WriteLog("Editor: joints check - " + state.status, logtype::generic);
}

void editor_mode::joints_focus(int const Index)
{
	auto &state{m_joints};
	if (Index < 0 || Index >= static_cast<int>(state.issues.size()))
		return;
	state.selected = Index;
	auto const &issue{state.issues[Index]};
	auto const &paths{simulation::Paths.sequence()};
	if (std::find(paths.begin(), paths.end(), issue.track) == paths.end() || issue.track->m_editorremoved)
		return;
	m_node = issue.track;
	m_track_point = {};
	ui()->set_node(m_node);
	focus_track(*issue.track, issue.point.path, issue.position);
}

bool editor_mode::joint_fix(joint_issue &Issue, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Changed)
{
	if (Issue.fixed || Issue.type == kind::open || Issue.track == nullptr || Issue.other == nullptr || Issue.track->m_editorremoved || Issue.other->m_editorremoved)
		return false;
	auto const movable = [](TTrack const *Track) {
		std::string reason;
		return Track->eType == tt_Normal && editor_track::can_edit_geometry(*Track, reason) && scene::Layers.editable(Track);
	};
	TTrack *mover{nullptr};
	editor_track::point_ref moved;
	TTrack *fixed{nullptr};
	editor_track::point_ref held;
	if (movable(Issue.track))
	{
		mover = Issue.track;
		moved = Issue.point;
		fixed = Issue.other;
		held = Issue.theirs;
	}
	else if (movable(Issue.other) && (Issue.type != kind::gap || false == editor_track::is_connected(*Issue.other, Issue.theirs)))
	{
		mover = Issue.other;
		moved = Issue.theirs;
		fixed = Issue.track;
		held = Issue.point;
	}
	if (mover == nullptr)
		return false;
	if (std::none_of(States.begin(), States.end(), [&](auto const &Entry) { return Entry.first == mover; }))
		States.emplace_back(mover, editor_track::capture(*mover));
	if (std::find(Changed.begin(), Changed.end(), mover) == Changed.end())
		Changed.push_back(mover);
	editor_track::snap_target target;
	target.track = fixed;
	target.point = held;
	target.position = editor_track::point_position(*fixed, held);
	target.direction = into(*fixed, held);
	switch (Issue.type)
	{
	case kind::gap:
	case kind::kink:
	case kind::grade:
		editor_track::snap_point(*mover, moved, target, true);
		break;
	case kind::step:
		editor_track::snap_point(*mover, moved, target, false);
		break;
	case kind::cant:
	{
		auto &path{mover->m_paths[moved.path]};
		auto const roll{static_cast<float>(roll_into(*fixed, held))};
		if (at_start(moved))
			path.rolls[0] = -roll;
		else
			path.rolls[1] = roll;
		break;
	}
	default:
		break;
	}
	Issue.fixed = true;
	return true;
}

void editor_mode::joints_fix(std::vector<int> const &Indices)
{
	auto &state{m_joints};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> changed;
	std::vector<joint_issue *> chosen;
	for (auto const index : Indices)
		if (index >= 0 && index < static_cast<int>(state.issues.size()))
			chosen.push_back(&state.issues[index]);
	std::size_t count{0};
	for (auto *issue : chosen)
		if (joint_fix(*issue, states, changed))
			++count;
	if (count == 0)
	{
		state.status = STR_C("Nothing could be fixed: the paths are switches or in the files which can't be changed");
		return;
	}
	push_track_snapshot(std::move(states));
	editor_track::commit(changed);
	for (auto *track : changed)
		track->mark_dirty();
	state.history = m_history.size();
	state.status = format(STR_C("%zu joints fixed, Ctrl+Z takes them back"), count);
	WriteLog("Editor: joints check - " + state.status, logtype::generic);
}

void editor_mode::render_joints_body()
{
	auto &state{m_joints};
	ImGui::TextDisabled("%s", STR_C("Ends of the paths which almost meet, steps, kinks,\njumps of the cant and of the grade at the joints"));
	ImGui::RadioButton(STR_C("Around the camera"), &state.scope, 0);
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
		joints_start();
	}
	if (ImGui::TreeNode(STR_C("Limits")))
	{
		ImGui::PushItemWidth(100.0f);
		if (state.scope == 0)
			ImGui::InputDouble(STR_C("Around the camera (m)"), &state.reach, 0.0, 0.0, "%.0f");
		ImGui::InputDouble(STR_C("Gap up to (m)"), &state.gap, 0.0, 0.0, "%.2f");
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("A free end closer than this to the end of another path is a gap: the trains don't go across it.\nThe simulation joins the ends no more than 2 cm apart"));
		ImGui::InputDouble(STR_C("Step (m)"), &state.step, 0.0, 0.0, "%.3f");
		ImGui::InputDouble(STR_C("Kink (deg)"), &state.angle, 0.0, 0.0, "%.2f");
		ImGui::InputDouble(STR_C("Cant jump (mm)"), &state.cant, 0.0, 0.0, "%.1f");
		ImGui::InputDouble(STR_C("Grade break (per mille)"), &state.grade, 0.0, 0.0, "%.1f");
		ImGui::Checkbox(STR_C("Free ends too"), &state.free_ends);
		ImGui::PopItemWidth();
		state.reach = std::clamp(state.reach, 50.0, 50000.0);
		state.gap = std::clamp(state.gap, 0.03, 20.0);
		state.step = std::max(0.001, state.step);
		state.angle = std::max(0.01, state.angle);
		state.cant = std::max(0.5, state.cant);
		state.grade = std::max(0.5, state.grade);
		ImGui::TextDisabled("%s", STR_C("Change the limits, then Check again"));
		ImGui::TreePop();
	}
	if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	if (state.issues.empty())
		return;
	if (state.history != m_history.size())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("The scenery changed since the check"));
	int filter{state.filter + 1};
	ImGui::SetNextItemWidth(150.0f);
	auto const filters{std::string{STR_C("All")} + '\0' + STR_C("gap") + '\0' + STR_C("step") + '\0' + STR_C("kink") + '\0' + STR_C("cant jump") + '\0' + STR_C("grade break") + '\0' + STR_C("free end") + '\0' + '\0'};
	if (ImGui::Combo("##jointfilter", &filter, filters.c_str()))
		state.filter = filter - 1;
	std::vector<int> listed;
	std::vector<int> fixable;
	for (int i = 0; i < static_cast<int>(state.issues.size()); ++i)
	{
		auto const &issue{state.issues[i]};
		if (state.filter >= 0 && static_cast<int>(issue.type) != state.filter)
			continue;
		listed.push_back(i);
		if (false == issue.fixed && issue.type != kind::open)
			fixable.push_back(i);
	}
	if (false == fixable.empty())
	{
		ImGui::SameLine();
		if (ImGui::Button(format(STR_C("Fix %zu listed"), fixable.size()).c_str()))
			joints_fix(fixable);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Gaps: the free end goes to the other end, along it. Steps: the end goes to the other one.\n"
			                              "Kinks and grade breaks: the end is turned in line with the other path. Cant: the cant of the other path.\n"
			                              "The plain path moves, a switch stays as it is. One Ctrl+Z takes it all back"));
	}
	auto const namewidth{std::max(110.0f, ImGui::GetContentRegionAvail().x * 0.32f)};
	bool const chosen{state.selected >= 0 && state.selected < static_cast<int>(state.issues.size())};
	auto const details{chosen ? ImGui::GetFrameHeightWithSpacing() + ImGui::GetTextLineHeightWithSpacing() * 2.0f : 0.0f};
	ImGui::BeginChild("jointresults", ImVec2(0.0f, -details), true);
	ImGui::Columns(4, "jointrows", false);
	ImGui::SetColumnWidth(0, 90.0f);
	ImGui::SetColumnWidth(1, 80.0f);
	ImGui::SetColumnWidth(2, namewidth);
	ImGuiListClipper clipper(static_cast<int>(listed.size()));
	while (clipper.Step())
		for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
		{
			auto const index{listed[row]};
			auto const &issue{state.issues[index]};
			auto const colour{ImGui::ColorConvertU32ToFloat4(kind_colour(issue.type))};
			ImGui::PushStyleColor(ImGuiCol_Text, issue.fixed ? ImVec4(0.5f, 0.8f, 0.5f, 1.0f) : colour);
			if (ImGui::Selectable(format("%s##joint%d", STR_C(kind_name(issue.type)), index).c_str(), state.selected == index, ImGuiSelectableFlags_SpanAllColumns))
				joints_focus(index);
			ImGui::PopStyleColor();
			ImGui::NextColumn();
			ImGui::TextUnformatted(issue.fixed ? STR_C("fixed") : value_text(issue).c_str());
			ImGui::NextColumn();
			ImGui::TextUnformatted(name_of(issue.track).c_str());
			ImGui::NextColumn();
			ImGui::TextUnformatted(name_of(issue.other).c_str());
			ImGui::NextColumn();
		}
	ImGui::Columns(1);
	ImGui::EndChild();
	if (chosen)
	{
		auto &issue{state.issues[state.selected]};
		ImGui::Text("%s %s: %s / %s", STR_C(kind_name(issue.type)), value_text(issue).c_str(), name_of(issue.track).c_str(), name_of(issue.other).c_str());
		if (false == issue.fixed && issue.type != kind::open && ImGui::Button(STR_C("Fix this one")))
			joints_fix({state.selected});
	}
}

void editor_mode::draw_joints_overlay() const
{
	auto const &state{m_joints};
	if (false == state.open || state.issues.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (int i = 0; i < static_cast<int>(state.issues.size()); ++i)
	{
		auto const &issue{state.issues[i]};
		if (issue.fixed || (state.filter >= 0 && static_cast<int>(issue.type) != state.filter) || glm::distance(issue.position, camera) > kOverlayRange)
			continue;
		ImVec2 screen;
		if (false == projection.project(issue.position, screen))
			continue;
		auto const colour{i == state.selected ? IM_COL32(255, 255, 80, 255) : kind_colour(issue.type)};
		drawlist->AddCircle(screen, 11.0f, colour, 16, 2.5f);
		auto const text{std::string{STR_C(kind_name(issue.type))} + " " + value_text(issue)};
		drawlist->AddText(ImVec2(screen.x + 13.0f, screen.y - 7.0f), colour, text.c_str());
	}
}
