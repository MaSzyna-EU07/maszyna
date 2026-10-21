/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodejoints.h"
#include "editor/editorNmt.hpp"
#include "application/editormodeshared.h"
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
#include <limits>
#include <map>
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
double const kModelStep{5.0}; // m between the places of the tracks compared with the terrain model
std::size_t const kModelRun{200}; // places of a path whose model is asked for at once
double const kModelPoll{0.25}; // s between the looks whether the model is in
double const kUnfitted{30.0}; // m; a scenery whose tracks lie this far from the model as a whole isn't fitted to it

using kind = joints_tool::joint_issue::kind;

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

std::tuple<TTrack const *, int, int> key_of(TTrack const *Track, editor_track::point_ref const &Point)
{
	return {Track, Point.path, static_cast<int>(Point.kind)};
}

char const *kind_name(kind const Kind)
{
	switch (Kind)
	{
	case kind::gap: return "gap";
	case kind::step: return "step";
	case kind::kink: return "kink";
	case kind::cant: return "cant jump";
	case kind::grade: return "grade break";
	case kind::model: return "off the terrain model";
	default: return "free end";
	}
}

std::string value_text(joints_tool::joint_issue const &Issue)
{
	switch (Issue.type)
	{
	case kind::gap: return format("%.3f m", Issue.value);
	case kind::step: return format("%.1f mm", Issue.value * 1000.0);
	case kind::kink: return format("%.3f\xc2\xb0", Issue.value);
	case kind::cant: return format("%.1f mm", Issue.value);
	case kind::grade: return format("%.1f \xe2\x80\xb0", Issue.value);
	case kind::model: return format("%+.2f m", Issue.value);
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
	case kind::model: return IM_COL32(70, 200, 190, 235);
	default: return IM_COL32(200, 200, 200, 200);
	}
}

std::string label_of(TTrack const *Track)
{
	return Track == nullptr ? std::string{"-"} : ::name_of(*Track);
}

} // namespace

void joints_tool::joints_start()
{
	auto &state{m_joints};
	state.queue.clear();
	state.next = 0;
	state.issues.clear();
	state.selected = -1;
	state.status.clear();
	m_reported.clear();
	glm::dvec3 const camera{Global.pCamera.Pos};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track))
			continue;
		if (state.scope == 0 && glm::distance(track->location(), camera) > state.reach)
			continue;
		state.queue.push_back(track);
	}
	state.history = m_history.history_size();
	state.places.clear();
	state.runs.clear();
	state.run_next = 0;
	state.model_waiting = false;
	state.model_asked = false;
	state.model_polled = 0.0;
	state.model_unplaced = false;
	if (false == state.model_check || state.queue.empty())
		return;
	m_ground.nmt_locate();
	if (false == m_ground.terrain_model().placed())
	{
		state.model_unplaced = true;
		return;
	}
	std::set<TTrack const *> const queued(state.queue.begin(), state.queue.end());
	for (auto const &place : m_ground.nmt_track_places(kModelStep))
		if (place.shapes && queued.count(place.track) > 0)
		{
			if (state.places.empty() || state.places.back().track != place.track || state.places.back().path != place.path || state.places.size() - state.runs.back().first >= kModelRun)
				state.runs.emplace_back(state.places.size(), state.places.size());
			state.places.push_back(place);
			state.runs.back().second = state.places.size();
		}
	state.model_waiting = false == state.places.empty();
}

bool joints_tool::joints_model()
{
	auto &state{m_joints};
	auto const now{ImGui::GetTime()};
	if (state.model_asked && now - state.model_polled < kModelPoll)
		return false;
	state.model_polled = now;
	auto const box = [&](std::pair<std::size_t, std::size_t> const &Run, glm::dvec2 &Min, glm::dvec2 &Max) {
		Min = glm::dvec2{std::numeric_limits<double>::max()};
		Max = glm::dvec2{std::numeric_limits<double>::lowest()};
		for (auto i = Run.first; i < Run.second; ++i)
		{
			glm::dvec2 const at{state.places[i].position.x, state.places[i].position.z};
			Min = glm::min(Min, at - 2.0);
			Max = glm::max(Max, at + 2.0);
		}
	};
	glm::dvec2 low, high;
	// all of it asked for once, then the runs looked over in turn as their squares come
	if (false == state.model_asked)
	{
		for (auto const &run : state.runs)
		{
			box(run, low, high);
			m_ground.terrain_model().request(low, high);
		}
		state.model_asked = true;
	}
	while (state.run_next < state.runs.size())
	{
		box(state.runs[state.run_next], low, high);
		if (false == m_ground.terrain_model().request(low, high))
			return false;
		++state.run_next;
	}
	std::vector<double> differences(state.places.size(), std::numeric_limits<double>::quiet_NaN());
	std::vector<double> known;
	for (std::size_t i = 0; i < state.places.size(); ++i)
		if (double height; m_ground.terrain_model().height(state.places[i].position.x, state.places[i].position.z, height))
		{
			differences[i] = state.places[i].crown - (height + m_ground.terrain_model_offset());
			known.push_back(differences[i]);
		}
	auto shift{0.0};
	if (false == m_ground.terrain_model_fitted() && false == known.empty())
	{
		std::nth_element(known.begin(), known.begin() + known.size() / 2, known.end());
		if (std::abs(known[known.size() / 2]) > kUnfitted)
			shift = known[known.size() / 2];
	}
	// the place of each path the farthest from the model
	std::map<std::pair<TTrack *, int>, std::size_t> worst;
	for (std::size_t i = 0; i < state.places.size(); ++i)
	{
		if (false == std::isfinite(differences[i]))
			continue;
		differences[i] -= shift;
		if (std::abs(differences[i]) <= state.model)
			continue;
		auto const key{std::make_pair(state.places[i].track, state.places[i].path)};
		auto const found{worst.find(key)};
		if (found == worst.end())
			worst.emplace(key, i);
		else if (std::abs(differences[i]) > std::abs(differences[found->second]))
			found->second = i;
	}
	for (auto const &entry : worst)
	{
		auto const &place{state.places[entry.second]};
		joint_issue issue;
		issue.type = kind::model;
		issue.track = place.track;
		issue.point = {place.path, editor_track::point_kind::start};
		issue.position = place.position;
		issue.value = differences[entry.second];
		state.issues.push_back(issue);
	}
	state.places.clear();
	state.runs.clear();
	state.model_waiting = false;
	return true;
}

void joints_tool::joints_check(TTrack &Track, std::vector<joint_issue> &Issues, std::set<std::pair<end_key, end_key>> &Seen, bool const Open) const
{
	auto const &state{m_joints};
	auto const once = [&](end_key const &A, end_key const &B) { return Seen.insert(A < B ? std::make_pair(A, B) : std::make_pair(B, A)).second; };
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
						Issues.push_back(nearest);
					}
				}
				else if (Open)
				{
					joint_issue open;
					open.type = kind::open;
					open.track = &Track;
					open.point = point;
					open.position = position;
					Issues.push_back(open);
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
					Issues.push_back(issue);
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

void joints_tool::tracks_committed(std::vector<TTrack *> const &Tracks)
{
	for (auto *track : Tracks)
		if (track != nullptr)
			m_live.pending.insert(track);
}

void joints_tool::live_update()
{
	auto &live{m_live};
	auto const gone = [](TTrack const *Track) { return Track != nullptr && Track->m_editorremoved; };
	std::erase_if(live.issues, [&](joint_issue const &Issue) { return gone(Issue.track) || gone(Issue.other); });
	if (live.pending.empty() || ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseDown(ImGuiMouseButton_Right))
		return;
	// the joints of a track changed are looked at again, from its side and from the side of the tracks it joins
	std::set<TTrack *> checked;
	for (auto *track : live.pending)
	{
		if (track->m_editorremoved || false == rail(*track))
			continue;
		checked.insert(track);
		for (auto const &path : track->m_paths)
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				for (auto const &connection : editor_track::connected_points(*track, path.points[index]))
					if (rail(*connection.first))
						checked.insert(connection.first);
	}
	live.pending.clear();
	std::erase_if(live.issues, [&](joint_issue const &Issue) { return checked.contains(Issue.track) || checked.contains(Issue.other); });
	std::set<std::pair<end_key, end_key>> seen;
	for (auto *track : checked)
		joints_check(*track, live.issues, seen, false);
}

void joints_tool::update()
{
	auto &state{m_joints};
	live_update();
	if (state.next >= state.queue.size() && false == state.model_waiting)
		return;
	auto const deadline{std::chrono::steady_clock::now() + std::chrono::milliseconds(15)};
	while (state.next < state.queue.size() && std::chrono::steady_clock::now() < deadline)
	{
		auto *track{state.queue[state.next++]};
		if (track->m_editorremoved)
			continue;
		joints_check(*track, state.issues, m_reported, state.free_ends);
	}
	if (state.next < state.queue.size())
		return;
	if (state.model_waiting && false == joints_model())
		return;
	std::stable_sort(state.issues.begin(), state.issues.end(), [](joint_issue const &A, joint_issue const &B) {
		if (A.type != B.type)
			return A.type < B.type;
		return std::abs(A.value) > std::abs(B.value);
	});
	std::array<int, 7> counts{};
	for (auto const &issue : state.issues)
		++counts[static_cast<int>(issue.type)];
	state.status = format(STR_C("%zu paths checked: %d gaps, %d steps, %d kinks, %d cant jumps, %d grade breaks"), state.queue.size(), counts[0], counts[1], counts[2], counts[3], counts[4]);
	if (state.model_check && state.model_unplaced)
		state.status += STR_C(", the terrain model left out: the scenery isn't placed on the map (the orthophoto settings)");
	else if (state.model_check)
		state.status += format(STR_C(", %d paths off the terrain model"), counts[5]);
	if (state.free_ends)
		state.status += format(STR_C(", %d free ends"), counts[6]);
	WriteLog("Editor: joints check - " + state.status, logtype::generic);
}

void joints_tool::joints_focus(int const Index)
{
	auto &state{m_joints};
	if (Index < 0 || Index >= static_cast<int>(state.issues.size()))
		return;
	state.selected = Index;
	auto const &issue{state.issues[Index]};
	auto const &paths{simulation::Paths.sequence()};
	if (std::find(paths.begin(), paths.end(), issue.track) == paths.end() || issue.track->m_editorremoved)
		return;
	m_selection.set_current_node(issue.track);
	m_selection.clear_track_point();
	m_view.focus_track(*issue.track, issue.point.path, issue.position);
}

bool joints_tool::joint_fix(joint_issue &Issue, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Changed)
{
	if (Issue.fixed || Issue.type == kind::open || Issue.type == kind::model || Issue.track == nullptr || Issue.other == nullptr || Issue.track->m_editorremoved || Issue.other->m_editorremoved)
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

void joints_tool::joints_fix(std::vector<int> const &Indices)
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
	m_history.push_track_snapshot(std::move(states));
	editor_track::commit(changed);
	for (auto *track : changed)
		track->mark_dirty();
	state.history = m_history.history_size();
	state.status = format(STR_C("%zu joints fixed, Ctrl+Z takes them back"), count);
	WriteLog("Editor: joints check - " + state.status, logtype::generic);
}

void joints_tool::render_options()
{
	auto &state{m_joints};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", STR_C("Looks for faults at the joints of the tracks"));
	ImGui::PopTextWrapPos();
	segmented("jointscope", state.scope, {{0, STR_C("Around the camera")}, {1, STR_C("Whole scenery")}});
	bool const running{state.next < state.queue.size() || state.model_waiting};
	if (running)
	{
		if (state.next < state.queue.size())
		{
			auto const progress{static_cast<float>(state.next) / static_cast<float>(state.queue.size())};
			ImGui::ProgressBar(progress, ImVec2(-70.0f, 0.0f), format(STR_C("Checking %zu / %zu"), state.next, state.queue.size()).c_str());
		}
		else
		{
			auto const progress{static_cast<float>(state.run_next) / static_cast<float>(std::max<std::size_t>(1, state.runs.size()))};
			ImGui::ProgressBar(progress, ImVec2(-70.0f, 0.0f), format(STR_C("Terrain model %zu / %zu"), state.run_next, state.runs.size()).c_str());
		}
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Cancel")))
		{
			state.queue.resize(state.next);
			state.model_waiting = false;
		}
	}
	else
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertU32ToFloat4(family_colour(2, 0.55f)));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::ColorConvertU32ToFloat4(family_colour(2, 0.75f)));
		if (ImGui::Button(STR_C("Check"), ImVec2(-1.0f, tall_button_height())))
			joints_start();
		ImGui::PopStyleColor(2);
		item_tooltip("Finds the gaps between the ends of the tracks, the steps, the kinks and the jumps of the cant and of the grade");
	}
	if (ImGui::TreeNode(STR_C("Thresholds")))
	{
		if (begin_fields("##jointlimits", 0.55f))
		{
			auto const limit = [](char const *Label, char const *Id, double &Value, char const *Format) {
				field_row(Label);
				ImGui::InputDouble(Id, &Value, 0.0, 0.0, Format);
			};
			if (state.scope == 0)
				limit("Around the camera (m)", "##jointreach", state.reach, "%.0f");
			limit("Gap up to (m)", "##jointgap", state.gap, "%.2f");
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("A free end closer than this to the end of another path is a gap: the trains don't go across it.\nThe simulation joins the ends no more than 2 cm apart"));
			limit("Step (m)", "##jointstep", state.step, "%.3f");
			limit("Kink (deg)", "##jointkink", state.angle, "%.2f");
			limit("Cant jump (mm)", "##jointcant", state.cant, "%.1f");
			limit("Grade break (per mille)", "##jointgrade", state.grade, "%.1f");
			limit("Off the terrain model (m)", "##jointmodel", state.model, "%.2f");
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", STR_C("The track lies higher or lower than the terrain model (NMT) says the ground is, by more than this.\nEach path is listed once, at its place the farthest off; bridges and tunnels are left out"));
			ImGui::EndTable();
		}
		toggle_chips("jointflags", {{STR_C("Free ends too"), &state.free_ends}, {STR_C("Against the terrain model"), &state.model_check}});
		state.reach = std::clamp(state.reach, 50.0, 50000.0);
		state.gap = std::clamp(state.gap, 0.03, 20.0);
		state.step = std::max(0.001, state.step);
		state.angle = std::max(0.01, state.angle);
		state.cant = std::max(0.5, state.cant);
		state.grade = std::max(0.5, state.grade);
		state.model = std::max(0.05, state.model);
		ImGui::TextDisabled("%s", STR_C("Change the thresholds, then Check again"));
		ImGui::TreePop();
	}
	if (false == state.status.empty())
		ImGui::TextWrapped("%s", state.status.c_str());
	if (state.issues.empty())
		return;
	if (state.history != m_history.history_size())
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("The scenery changed since the check"));
	int filter{state.filter + 1};
	ImGui::SetNextItemWidth(150.0f);
	auto const filters{std::string{STR_C("All")} + '\0' + STR_C("gap") + '\0' + STR_C("step") + '\0' + STR_C("kink") + '\0' + STR_C("cant jump") + '\0' + STR_C("grade break") + '\0' + STR_C("off the terrain model") + '\0' +
	                   STR_C("free end") + '\0' + '\0'};
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
		if (false == issue.fixed && issue.type != kind::open && issue.type != kind::model)
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
	ImGuiListClipper clipper;
	clipper.Begin(static_cast<int>(listed.size()));
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
			ImGui::TextUnformatted(label_of(issue.track).c_str());
			ImGui::NextColumn();
			ImGui::TextUnformatted(label_of(issue.other).c_str());
			ImGui::NextColumn();
		}
	ImGui::Columns(1);
	ImGui::EndChild();
	if (chosen)
	{
		auto &issue{state.issues[state.selected]};
		ImGui::Text("%s %s: %s / %s", STR_C(kind_name(issue.type)), value_text(issue).c_str(), label_of(issue.track).c_str(), label_of(issue.other).c_str());
		if (false == issue.fixed && issue.type != kind::open && issue.type != kind::model && ImGui::Button(STR_C("Fix this one")))
			joints_fix({state.selected});
	}
}

void joints_tool::draw_overlay() const
{
	auto const &state{m_joints};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	glm::dvec3 const camera{Global.pCamera.Pos};
	if (false == state.open)
	{
		for (auto const &issue : m_live.issues)
		{
			ImVec2 screen;
			if ((false == Global.EditorOrtho && glm::distance(issue.position, camera) > kOverlayRange) || false == projection.project(issue.position, screen))
				continue;
			auto const colour{kind_colour(issue.type)};
			drawlist->AddCircle(screen, 9.0f, colour, 16, 2.0f);
			auto const text{std::string{STR_C(kind_name(issue.type))} + " " + value_text(issue)};
			drawlist->AddRectFilled(ImVec2(screen.x + 11.0f, screen.y - 9.0f), ImVec2(screen.x + 17.0f + ImGui::CalcTextSize(text.c_str()).x, screen.y + 9.0f), IM_COL32(0, 0, 0, 150), 3.0f);
			drawlist->AddText(ImVec2(screen.x + 14.0f, screen.y - 7.0f), colour, text.c_str());
		}
		return;
	}
	if (state.issues.empty())
		return;
	for (int i = 0; i < static_cast<int>(state.issues.size()); ++i)
	{
		auto const &issue{state.issues[i]};
		if (issue.fixed || (state.filter >= 0 && static_cast<int>(issue.type) != state.filter) || (false == Global.EditorOrtho && glm::distance(issue.position, camera) > kOverlayRange))
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

editor_selftest_result joints_tool::selftest(std::string const &Command, std::istringstream &Words)
{
	if (Command == "jointsmodel")
	{
		// the whole scenery against the terrain model, as the check of the panel does it, waiting for the model
		static bool started{false};
		if (false == started)
		{
			double threshold{0.5};
			Words >> threshold;
			m_joints.scope = 1;
			m_joints.model_check = true;
			m_joints.model = threshold;
			joints_start();
			started = true;
		}
		update();
		if (m_joints.next < m_joints.queue.size() || m_joints.model_waiting)
			return editor_selftest_result::again;
		started = false;
		std::size_t count{0};
		for (auto const &issue : m_joints.issues)
			if (issue.type == joint_issue::kind::model && count++ < 40)
				WriteLog(format("SELFTEST jointsmodel: %+.2f m at %.1f %.1f, %s path %d", issue.value, issue.position.x, issue.position.z, issue.track != nullptr ? issue.track->name().c_str() : "-", issue.point.path));
		WriteLog("SELFTEST jointsmodel: " + m_joints.status);
		return editor_selftest_result::handled;
	}
	if (Command == "jointslive")
	{
		live_update();
		std::string text;
		for (auto const &issue : m_live.issues)
			text += format(" [%s %s at %.1f %.1f]", kind_name(issue.type), value_text(issue).c_str(), issue.position.x, issue.position.z);
		WriteLog(format("SELFTEST jointslive: %zu pending, %zu marked", m_live.pending.size(), m_live.issues.size()) + text);
		return editor_selftest_result::handled;
	}
	if (Command == "joints")
	{
		std::vector<glm::dvec3> points;
		for (glm::dvec3 point{0.0}; Words >> point.x >> point.z;)
			points.push_back(point);
		auto const scope{m_joints.scope};
		auto const model{m_joints.model_check};
		m_joints.scope = 1;
		m_joints.model_check = false;
		joints_start();
		while (m_joints.next < m_joints.queue.size())
			update();
		m_joints.scope = scope;
		m_joints.model_check = model;
		char const *const kinds[]{"gap", "step", "kink", "cant jump", "grade break", "off the terrain model", "free end"};
		int around{0};
		for (auto const &issue : m_joints.issues)
		{
			if (false == points.empty() && issue.type == joint_issue::kind::open)
				continue;
			if (false == points.empty() && std::none_of(points.begin(), points.end(), [&](glm::dvec3 const &Point) { return geometry::plan_distance(Point, issue.position) < 5.0; }))
				continue;
			++around;
			auto const name = [](TTrack const *Track) { return Track == nullptr ? std::string{"-"} : Track->name() + (Track->eType == tt_Switch ? "(switch)" : ""); };
			auto const rise{issue.other != nullptr ? editor_track::point_position(*issue.other, issue.theirs).y - issue.position.y : 0.0};
			WriteLog(format("SELFTEST joints: %s %.3f at %.1f %.1f, %s / %s, rise %.3f", kinds[static_cast<int>(issue.type)], issue.value, issue.position.x, issue.position.z, name(issue.track).c_str(), name(issue.other).c_str(), rise));
		}
		WriteLog(format("SELFTEST joints: %d issues around the points, %zu in all", around, m_joints.issues.size()));
		return editor_selftest_result::handled;
	}
	return editor_selftest_result::unhandled;
}
