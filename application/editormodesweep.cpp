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
#include "editor/editorIncludeInfo.hpp"

#include "rendering/renderer.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Sweep.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <algorithm>
#include <optional>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

namespace
{

using geometry::bezier;
using geometry::plan_distance;
using geometry::plan_of;

double const kSweepRouteReach{2000.0}; // m each way through the switches
double const kSweepOutlineStep{1.0};
double const kSweepNear{4.0}; // m, a model along a curve this close to the selected path belongs to its line
double const kSweepGrab{25.0}; // m from the track, a press this close marks the stretch of the model
std::size_t const kSweepModelList{400};
double const kRailTop{0.18}; // m from the points of a track to the top of its rails

segment_data reversed(segment_data const &Path)
{
	segment_data result{Path};
	result.points[segment_data::point::start] = Path.points[segment_data::point::end];
	result.points[segment_data::point::end] = Path.points[segment_data::point::start];
	result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
	result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
	result.rolls = {-Path.rolls[1], -Path.rolls[0]};
	return result;
}

bool same_piece(segment_data const &A, segment_data const &B)
{
	for (int i = 0; i < 4; ++i)
		if (glm::distance(A.points[i], B.points[i]) > 1e-3)
			return false;
	return std::abs(A.rolls[0] - B.rolls[0]) < 1e-3f && std::abs(A.rolls[1] - B.rolls[1]) < 1e-3f;
}

std::string lower(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

} // namespace

void editor_mode::sweep_scan_models()
{
	auto &tool{m_sweep};
	if (tool.scanned)
		return;
	tool.scanned = true;
	std::error_code error;
	std::set<std::string> found;
	for (auto const &entry : std::filesystem::recursive_directory_iterator("models", std::filesystem::directory_options::skip_permission_denied, error))
	{
		if (false == entry.is_regular_file(error))
			continue;
		auto const extension{lower(entry.path().extension().string())};
		if (extension != ".e3d" && extension != ".t3d")
			continue;
		auto name{std::filesystem::relative(entry.path(), "models", error).generic_string()};
		name.erase(name.size() - extension.size());
		found.insert(name);
	}
	for (auto const &entry : std::filesystem::recursive_directory_iterator("scenery", std::filesystem::directory_options::skip_permission_denied, error))
	{
		if (entry.is_regular_file(error) && lower(entry.path().extension().string()) == ".inc")
			found.insert(std::filesystem::relative(entry.path(), "scenery", error).generic_string());
	}
	tool.models.assign(found.begin(), found.end());
}


void editor_mode::sweep_curve()
{
	auto &tool{m_sweep};
	auto *track{selected_track()};
	if (track == tool.curve_for && tool.curve_history == m_history.size())
		return;
	tool.curve_for = track;
	tool.curve_history = m_history.size();
	tool.curve.clear();
	tool.curve_pieces.clear();
	if (track != nullptr && false == track->m_paths.empty() && editor_track::is_supported(*track))
	{
		tool.curve.push_back(track->m_paths.front());
		tool.curve_pieces.push_back({track, 0, true});
	}
	sweep_outline();
}

void editor_mode::sweep_outline()
{
	auto &tool{m_sweep};
	tool.outline.clear();
	tool.stations.clear();
	tool.curve_length = 0.0;
	for (auto const &piece : tool.curve)
	{
		bezier const curve{piece};
		auto const count{std::clamp(static_cast<int>(curve.plan_length() / kSweepOutlineStep), 2, 2000)};
		for (int k = tool.outline.empty() ? 0 : 1; k <= count; ++k)
		{
			auto const point{curve.point(static_cast<double>(k) / count)};
			if (false == tool.outline.empty())
				tool.curve_length += plan_distance(tool.outline.back(), point);
			tool.outline.push_back(point);
			tool.stations.push_back(tool.curve_length);
		}
	}
}

bool editor_mode::sweep_extend(bool const Atend, glm::dvec3 const &Cursor)
{
	auto &tool{m_sweep};
	if (tool.curve_pieces.empty() || tool.curve_length > kSweepRouteReach)
		return false;
	auto const &edge{Atend ? tool.curve_pieces.back() : tool.curve_pieces.front()};
	auto const &edgepiece{Atend ? tool.curve.back() : tool.curve.front()};
	auto const point{Atend ? edgepiece.points[segment_data::point::end] : edgepiece.points[segment_data::point::start]};
	std::optional<sweep_tool::piece> best;
	segment_data bestpiece;
	auto bestdistance{std::numeric_limits<double>::max()};
	for (auto const &[other, end] : editor_track::connected_points(*edge.track, point))
	{
		if (other == nullptr || other->m_editorremoved || other->eType == tt_Table || false == editor_track::is_supported(*other) || end.path >= static_cast<int>(other->m_paths.size()))
			continue;
		if (std::any_of(tool.curve_pieces.begin(), tool.curve_pieces.end(), [&](sweep_tool::piece const &Piece) { return Piece.track == other && Piece.path == end.path; }))
			continue;
		auto const startshere{end.kind == editor_track::point_kind::start};
		auto const forward{Atend ? startshere : false == startshere};
		auto const piece{forward ? other->m_paths[end.path] : reversed(other->m_paths[end.path])};
		bezier const curve{piece};
		auto distance{std::numeric_limits<double>::max()};
		for (int k = 0; k <= 32; ++k)
			distance = std::min(distance, plan_distance(curve.point(k / 32.0), Cursor));
		if (distance < bestdistance)
		{
			bestdistance = distance;
			best = sweep_tool::piece{other, end.path, forward};
			bestpiece = piece;
		}
	}
	if (false == best.has_value())
		return false;
	if (Atend)
	{
		tool.curve_pieces.push_back(*best);
		tool.curve.push_back(bestpiece);
		sweep_outline();
	}
	else
	{
		auto const before{tool.curve_length};
		tool.curve_pieces.insert(tool.curve_pieces.begin(), *best);
		tool.curve.insert(tool.curve.begin(), bestpiece);
		sweep_outline();
		tool.anchor += tool.curve_length - before;
	}
	return true;
}

double editor_mode::sweep_station(glm::dvec3 const &Point, double &Lateral) const
{
	auto const &tool{m_sweep};
	double best{std::numeric_limits<double>::max()};
	double station{0.0};
	Lateral = 0.0;
	auto const point{plan_of(Point)};
	for (std::size_t i = 1; i < tool.outline.size(); ++i)
	{
		auto const a{plan_of(tool.outline[i - 1])};
		auto const ab{plan_of(tool.outline[i]) - a};
		auto const length2{glm::dot(ab, ab)};
		if (length2 < 1e-12)
			continue;
		auto const t{std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0)};
		auto const distance{glm::length(point - (a + ab * t))};
		if (distance < best)
		{
			best = distance;
			station = tool.stations[i - 1] + (tool.stations[i] - tool.stations[i - 1]) * t;
			Lateral = geometry::cross(glm::normalize(ab), point - a) >= 0.0 ? distance : -distance;
		}
	}
	return station;
}

void editor_mode::sweep_measure()
{
	auto &tool{m_sweep};
	std::string const model{tool.model};
	if (model == tool.measured)
		return;
	tool.measured = model;
	tool.model_length = 0.0;
	if (model.empty())
		return;
	scene::node_data data;
	sweep_node probe{data};
	auto state{sweep_definition()};
	state.pieces.clear();
	state.along_x = false;
	probe.define(state);
	auto const alongz{probe.model_length()};
	state.along_x = true;
	probe.define(state);
	auto const alongx{probe.model_length()};
	tool.settings.along_x = tool.axis == 2 || (tool.axis == 0 && alongx > alongz);
	tool.model_length = tool.settings.along_x ? alongx : alongz;
}

sweep_node::state editor_mode::sweep_definition() const
{
	auto const &tool{m_sweep};
	auto state{tool.settings};
	state.model = tool.model;
	state.skin = tool.skin[0] != '\0' ? tool.skin : "none";
	state.pieces = tool.curve;
	state.height = tool.settings.height + kRailTop;
	if (tool.length_mode == 0 && tool.model_length > 0.0)
		state.to = state.from + tool.model_length;
	else if (tool.length_mode == 1)
		state.to = state.from + std::max(0.5, tool.length);
	state.parameters.clear();
	for (auto const &value : tool.parameters)
		state.parameters.emplace_back(value[0] != '\0' ? value.data() : "none");
	return state;
}

void editor_mode::sweep_create()
{
	auto &tool{m_sweep};
	sweep_curve();
	if (tool.curve.empty() || tool.model[0] == '\0')
		return;
	static int counter{0};
	scene::node_data data;
	data.type = "sweep";
	data.range_max = -1.0;
	data.layer = scene::Layers.active();
	do
	{
		data.name = "sweep_" + std::to_string(++counter);
	} while (simulation::Sweeps.find(data.name) != nullptr);
	auto *sweep{new sweep_node(data)};
	sweep->define(sweep_definition());
	sweep->mark_dirty();
	simulation::Sweeps.insert(sweep);
	scene::Layers.count(sweep->layer(), scene::layer_item::model);
	sweep->show();
	// the pieces are the paths of the line, so the model follows them when they change
	for (auto const &piece : tool.curve_pieces)
		if (piece.track != nullptr && false == piece.track->m_editorremoved)
			sweeps_captured(*piece.track);
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = sweep->name();
	snap.sweeps_toggled.push_back(sweep);
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	tool.edited = sweep;
	tool.status = format(STR_C("%s laid along %.1f m, Ctrl+Z takes it back"), tool.model, sweep->end() - sweep->start());
	if (scene::Layers.active() == null_handle)
		tool.status += "\n" + std::string{STR_C("The scenery isn't opened for editing, it won't be saved")};
	WriteLog("Editor: " + tool.status, logtype::generic);
}

void editor_mode::sweep_apply()
{
	auto &tool{m_sweep};
	auto *sweep{tool.edited};
	if (sweep == nullptr || sweep->m_editorremoved)
		return;
	auto state{sweep_definition()};
	state.pieces = sweep->definition().pieces;
	if (state == sweep->definition())
		return;
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = sweep->name();
	snap.sweeps.emplace_back(sweep, sweep->definition());
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	sweep->define(state);
	sweep->mark_dirty();
}

void editor_mode::sweep_delete(sweep_node &Sweep)
{
	Sweep.m_editorremoved = true;
	Sweep.hide();
	Sweep.mark_dirty();
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = Sweep.name();
	snap.sweeps_toggled.push_back(&Sweep);
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	if (m_sweep.edited == &Sweep)
		m_sweep.edited = nullptr;
}

void editor_mode::sweep_edit(sweep_node &Sweep)
{
	auto &tool{m_sweep};
	tool.edited = &Sweep;
	auto const &state{Sweep.definition()};
	tool.settings = state;
	tool.settings.height = state.height - kRailTop;
	tool.settings.pieces.clear();
	std::snprintf(tool.model, sizeof(tool.model), "%s", state.model.c_str());
	std::snprintf(tool.skin, sizeof(tool.skin), "%s", state.skin.c_str());
	tool.axis = state.along_x ? 2 : 1;
	tool.measured.clear();
	tool.parameters_for.clear();
	tool.parameters.clear();
	for (auto const &value : state.parameters)
	{
		tool.parameters.emplace_back();
		std::snprintf(tool.parameters.back().data(), tool.parameters.back().size(), "%s", value.c_str());
	}
	tool.parameters_for = state.model;
}

void editor_mode::restore_sweeps(EditorSnapshot &Snapshot)
{
	for (auto &entry : Snapshot.sweeps)
	{
		auto current{entry.first->definition()};
		entry.first->define(entry.second);
		entry.first->mark_dirty();
		entry.second = std::move(current);
	}
	for (auto *sweep : Snapshot.sweeps_toggled)
	{
		sweep->m_editorremoved = false == sweep->m_editorremoved;
		if (sweep->m_editorremoved)
			sweep->hide();
		else
			sweep->show();
		sweep->mark_dirty();
		if (m_sweep.edited == sweep && sweep->m_editorremoved)
			m_sweep.edited = nullptr;
	}
}

std::vector<sweep_node *> editor_mode::sweeps_near(TTrack const &Track) const
{
	std::vector<sweep_node *> result;
	std::vector<glm::dvec3> points;
	for (auto const &path : Track.m_paths)
	{
		bezier const curve{path};
		for (int k = 0; k <= 8; ++k)
			points.push_back(curve.point(k / 8.0));
	}
	for (auto *sweep : simulation::Sweeps.sequence())
	{
		if (sweep == nullptr || sweep->m_editorremoved)
			continue;
		auto const &definition{sweep->definition()};
		bool close{false};
		for (auto const &piece : definition.pieces)
		{
			bezier const curve{piece};
			for (int k = 0; k <= 8 && false == close; ++k)
			{
				auto const point{curve.point(k / 8.0)};
				close = std::any_of(points.begin(), points.end(), [&](glm::dvec3 const &Other) { return plan_distance(point, Other) < kSweepNear; });
			}
			if (close)
				break;
		}
		if (close)
			result.push_back(sweep);
	}
	return result;
}

bool editor_mode::sweep_press()
{
	return sweep_press_at(Global.pCamera.Pos + GfxRenderer->Mouse_Position(), false);
}

bool editor_mode::sweep_grabs(glm::dvec3 const &Point)
{
	auto &tool{m_sweep};
	if (false == tool.open || tool.model[0] == '\0')
		return false;
	sweep_curve();
	if (tool.outline.size() < 2)
		return false;
	sweep_measure();
	if (tool.model_length <= 0.0)
		return false;
	double lateral;
	sweep_station(Point, lateral);
	return std::abs(lateral) <= kSweepGrab;
}

bool editor_mode::sweep_press_at(glm::dvec3 const &Point, bool const Anywhere)
{
	auto &tool{m_sweep};
	sweep_curve();
	if (false == tool.open || tool.outline.size() < 2 || tool.model[0] == '\0')
		return false;
	sweep_measure();
	if (tool.model_length <= 0.0)
		return false;
	auto *track{selected_track()};
	if (track != nullptr && track->eType == tt_Switch && track->m_paths.size() > 1 && tool.curve_pieces.size() == 1)
	{
		auto nearest{0};
		auto best{std::numeric_limits<double>::max()};
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			bezier const curve{track->m_paths[i]};
			for (int k = 0; k <= 32; ++k)
				if (auto const distance{plan_distance(curve.point(k / 32.0), Point)}; distance < best)
				{
					best = distance;
					nearest = i;
				}
		}
		tool.curve = {track->m_paths[nearest]};
		tool.curve_pieces = {{track, nearest, true}};
		sweep_outline();
	}
	double lateral;
	auto const station{sweep_station(Point, lateral)};
	// a press away from the track is left to the selection of the tracks
	if (false == Anywhere && std::abs(lateral) > kSweepGrab)
		return false;
	// the press tells the side, the distance from the track is the one set
	auto const distance{std::abs(tool.settings.lateral) > 1e-6 ? std::abs(tool.settings.lateral) : std::round(std::abs(lateral) / 0.05) * 0.05};
	tool.settings.lateral = lateral >= 0.0 ? distance : -distance;
	if (tool.settings.side_anchor == 0)
		tool.settings.side_anchor = 1;
	tool.anchor = std::round(station * 10.0) / 10.0;
	tool.settings.from = tool.settings.to = tool.anchor;
	tool.dragging = true;
	return true;
}

void editor_mode::sweep_drag()
{
	if (m_sweep.dragging)
		sweep_drag_at(Global.pCamera.Pos + GfxRenderer->Mouse_Position());
}

void editor_mode::sweep_drag_at(glm::dvec3 const &Point)
{
	auto &tool{m_sweep};
	if (false == tool.dragging)
		return;
	double lateral;
	for (int i = 0; i < 4; ++i)
	{
		auto const raw{sweep_station(Point, lateral)};
		if (raw >= tool.curve_length - 0.5 && sweep_extend(true, Point))
			continue;
		if (raw <= 0.5 && sweep_extend(false, Point))
			continue;
		break;
	}
	auto const station{std::round(sweep_station(Point, lateral) * 10.0) / 10.0};
	tool.settings.from = std::min(tool.anchor, station);
	tool.settings.to = std::max(tool.anchor, station);
}

void editor_mode::sweep_release()
{
	auto &tool{m_sweep};
	if (false == tool.dragging)
		return;
	tool.dragging = false;
	// a click lays one copy as long as the original from the place clicked, a drag the stretch dragged over
	if (tool.settings.to - tool.settings.from < 1.0)
	{
		tool.length_mode = 0;
		tool.settings.from = tool.anchor;
	}
	else
	{
		tool.length_mode = 1;
		tool.length = tool.settings.to - tool.settings.from;
	}
	if (tool.edited == nullptr)
	{
		sweep_create();
		return;
	}
	// the model being edited is laid again along the line, over the stretch marked
	auto state{sweep_definition()};
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = tool.edited->name();
	snap.sweeps.emplace_back(tool.edited, tool.edited->definition());
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	tool.edited->define(state);
	tool.edited->mark_dirty();
	if (auto *track{selected_track()})
		sweeps_captured(*track);
	tool.status = format(STR_C("%s laid along %.1f m, Ctrl+Z takes it back"), tool.model, tool.edited->end() - tool.edited->start());
}

void editor_mode::sweeps_captured(TTrack const &Track)
{
	for (auto *sweep : simulation::Sweeps.sequence())
	{
		if (sweep == nullptr || sweep->m_editorremoved)
			continue;
		auto const &pieces{sweep->definition().pieces};
		for (std::size_t i = 0; i < pieces.size(); ++i)
			for (int path = 0; path < static_cast<int>(Track.m_paths.size()); ++path)
			{
				bool const along{same_piece(pieces[i], Track.m_paths[path])};
				bool const against{false == along && same_piece(pieces[i], reversed(Track.m_paths[path]))};
				if (false == along && false == against)
					continue;
				if (std::none_of(m_sweep_links.begin(), m_sweep_links.end(), [&](sweep_link const &Link) { return Link.sweep == sweep && Link.piece == i; }))
					m_sweep_links.push_back({sweep, i, const_cast<TTrack *>(&Track), path, against});
			}
	}
}

void editor_mode::sweeps_committed(std::vector<TTrack *> const &Tracks)
{
	std::vector<sweep_node *> changed;
	std::vector<std::pair<sweep_node *, sweep_node::state>> states;
	for (auto const &link : m_sweep_links)
	{
		if (link.track == nullptr || link.track->m_editorremoved || link.sweep->m_editorremoved || std::find(Tracks.begin(), Tracks.end(), link.track) == Tracks.end() ||
		    link.path >= static_cast<int>(link.track->m_paths.size()))
			continue;
		auto entry{std::find_if(states.begin(), states.end(), [&](auto const &State) { return State.first == link.sweep; })};
		if (entry == states.end())
		{
			states.emplace_back(link.sweep, link.sweep->definition());
			entry = std::prev(states.end());
		}
		if (link.piece >= entry->second.pieces.size())
			continue;
		auto const &path{link.track->m_paths[link.path]};
		entry->second.pieces[link.piece] = link.reversed ? reversed(path) : path;
	}
	for (auto &entry : states)
	{
		if (entry.second == entry.first->definition())
			continue;
		entry.first->define(entry.second);
		entry.first->mark_dirty();
	}
}

void editor_mode::sweeps_split(TTrack &Original, TTrack &Created)
{
	if (Created.m_paths.empty())
		return;
	std::vector<sweep_link> added;
	for (auto &link : m_sweep_links)
	{
		if (link.track != &Original || link.path != 0)
			continue;
		auto state{link.sweep->definition()};
		if (link.piece >= state.pieces.size())
			continue;
		// the second part goes after the first one along the curve, or ahead of it on a curve laid against the path
		auto const at{link.piece + (link.reversed ? 0 : 1)};
		state.pieces.insert(state.pieces.begin() + at, link.reversed ? reversed(Created.m_paths.front()) : Created.m_paths.front());
		for (auto &other : m_sweep_links)
			if (other.sweep == link.sweep && other.piece >= at && &other != &link)
				++other.piece;
		if (link.reversed)
			++link.piece;
		added.push_back({link.sweep, at, &Created, 0, link.reversed});
		link.sweep->define(state);
	}
	m_sweep_links.insert(m_sweep_links.end(), added.begin(), added.end());
}

void editor_mode::render_sweep_ui()
{
	auto &tool{m_sweep};
	auto *track{selected_track()};
	sweep_curve();
	sweep_scan_models();
	ImGui::TextDisabled("%s", STR_C("A model bent to follow the track, or repeated along it: platforms, walls, fences, barriers, lamps"));

	// models along curves by this line
	if (track != nullptr)
	{
		auto const alongside{sweeps_near(*track)};
		if (false == alongside.empty())
		{
			ImGui::TextDisabled("%s", STR_C("By this track:"));
			for (auto *sweep : alongside)
			{
				ImGui::PushID(sweep);
				auto const &definition{sweep->definition()};
				if (ImGui::Selectable(format("%s  %s, %.0f m", sweep->name().c_str(), definition.model.c_str(), sweep->end() - sweep->start()).c_str(), tool.edited == sweep, 0, ImVec2(ImGui::GetContentRegionAvail().x - 60.0f, 0.0f)))
					sweep_edit(*sweep);
				ImGui::SameLine();
				if (ImGui::SmallButton(STR_C("Delete")))
					sweep_delete(*sweep);
				ImGui::PopID();
			}
			if (tool.edited != nullptr && ImGui::SmallButton(STR_C("New one")))
				tool.edited = nullptr;
			ImGui::Separator();
		}
	}
	bool changed{false};
	auto const edited = [&](bool const Changed) { changed |= Changed; };

	ImGui::SetNextItemWidth(-1.0f);
	if (ImGui::InputTextWithHint("##sweepmodel", STR_C("model, e.g. peronowe/1k96w200m"), tool.model, sizeof(tool.model)))
		changed = true;
	ImGui::SetNextItemWidth(-1.0f);
	ImGui::InputTextWithHint("##sweepsearch", STR_C("search the models"), tool.search, sizeof(tool.search));
	std::string const text{lower(tool.search)};
	if (false == text.empty())
	{
		ImGui::BeginChild("##sweepmodels", ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * 7.0f + 6.0f), true);
		std::size_t listed{0};
		for (auto const &model : tool.models)
		{
			if (lower(model).find(text) == std::string::npos)
				continue;
			if (++listed > kSweepModelList)
				break;
			if (ImGui::Selectable(model.c_str(), model == tool.model))
			{
				std::snprintf(tool.model, sizeof(tool.model), "%s", model.c_str());
				changed = true;
			}
		}
		ImGui::EndChild();
	}
	// a template takes its parameters, the ones which place it are zeros: it's laid out in its own space
	std::string const model{tool.model};
	bool const istemplate{model.size() > 4 && lower(model.substr(model.size() - 4)) == ".inc"};
	if (istemplate && model != tool.parameters_for)
	{
		tool.parameters_for = model;
		include_info info;
		std::string error;
		editor_includes::load(model, info, error);
		editor_includes::suggest(model, info);
		auto const count{editor_includes::parameter_count(model)};
		tool.parameters.assign(static_cast<std::size_t>(count), {});
		tool.parameter_labels.assign(static_cast<std::size_t>(count), {});
		tool.parameter_placement.assign(static_cast<std::size_t>(count), false);
		tool.parameter_choices.assign(static_cast<std::size_t>(count), {});
		// a parameter which makes a part of the name of a texture gets the variants of that texture to choose from
		{
			std::ifstream file(Global.asCurrentSceneryPath + model);
			std::stringstream content;
			content << file.rdbuf();
			std::string word;
			while (content >> word)
			{
				auto const open{word.find("(p")};
				auto const close{open != std::string::npos ? word.find(')', open) : std::string::npos};
				if (close == std::string::npos || word.find('/') == std::string::npos || word.find('/') > open)
					continue;
				auto const id{std::atoi(word.substr(open + 2, close - open - 2).c_str())};
				if (id < 1 || id > count || false == tool.parameter_choices[id - 1].empty())
					continue;
				auto const prefix{lower(word.substr(0, open))};
				auto suffix{word.substr(close + 1)};
				suffix = suffix.substr(0, suffix.find(':'));
				auto const slash{prefix.rfind('/')};
				auto const folder{std::string{"textures/"} + prefix.substr(0, slash)};
				auto const start{prefix.substr(slash + 1)};
				std::set<std::string> variants;
				std::error_code error;
				for (auto const &entry : std::filesystem::directory_iterator(folder, error))
				{
					auto const stem{lower(entry.path().stem().string())};
					if (stem.size() > start.size() + suffix.size() && stem.compare(0, start.size(), start) == 0 && (suffix.empty() || stem.compare(stem.size() - suffix.size(), suffix.size(), lower(suffix)) == 0))
						variants.insert(stem.substr(start.size(), stem.size() - start.size() - suffix.size()));
				}
				tool.parameter_choices[id - 1].assign(variants.begin(), variants.end());
			}
		}
		for (int id = 1; id <= count; ++id)
		{
			auto const &parameter{info.parameter(id)};
			auto const placement{parameter.role.rfind("pos.", 0) == 0 || parameter.role.rfind("rot.", 0) == 0};
			auto const value{placement ? std::string{"0"} : parameter.value.empty() ? std::string{"none"} : parameter.value};
			std::snprintf(tool.parameters[id - 1].data(), tool.parameters[id - 1].size(), "%s", value.c_str());
			tool.parameter_labels[id - 1] = parameter.label.empty() ? "p" + std::to_string(id) + (parameter.role != "free" ? " (" + parameter.role + ")" : std::string{}) : parameter.label;
			tool.parameter_placement[id - 1] = placement;
		}
		tool.measured.clear();
		changed = true;
	}
	else if (false == istemplate && false == tool.parameters.empty())
	{
		tool.parameters.clear();
		tool.parameters_for.clear();
	}
	if (false == tool.parameters.empty() && ImGui::TreeNodeEx(STR_C("Parameters of the template"), ImGuiTreeNodeFlags_DefaultOpen))
	{
		for (std::size_t i = 0; i < tool.parameters.size(); ++i)
		{
			ImGui::PushID(static_cast<int>(i));
			ImGui::SetNextItemWidth(160.0f);
			auto const label{i < tool.parameter_labels.size() ? tool.parameter_labels[i] : "p" + std::to_string(i + 1)};
			auto const *choices{i < tool.parameter_choices.size() && false == tool.parameter_choices[i].empty() ? &tool.parameter_choices[i] : nullptr};
			if (choices != nullptr && ImGui::BeginCombo(label.c_str(), tool.parameters[i].data()))
			{
				for (auto const &choice : *choices)
					if (ImGui::Selectable(choice.c_str(), choice == tool.parameters[i].data()))
					{
						std::snprintf(tool.parameters[i].data(), tool.parameters[i].size(), "%s", choice.c_str());
						tool.measured.clear();
						changed = true;
					}
				ImGui::EndCombo();
			}
			else if (choices == nullptr)
			{
				ImGui::InputText(label.c_str(), tool.parameters[i].data(), tool.parameters[i].size());
				if (ImGui::IsItemDeactivatedAfterEdit())
				{
					tool.measured.clear();
					changed = true;
				}
			}
			ImGui::PopID();
		}
		ImGui::TextDisabled("%s", STR_C("The location and the rotation are zeros, the template is laid out along the track"));
		ImGui::TreePop();
	}
	sweep_measure();
	if (tool.model_length > 0.0)
		ImGui::TextDisabled(STR_C("The model is %.2f m long along the track"), tool.model_length);
	else if (tool.model[0] != '\0')
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", STR_C("The model has no triangles to lay, or it can't be loaded"));

	auto &settings{tool.settings};
	int mode{settings.bend ? 0 : 1};
	edited(ImGui::RadioButton(STR_C("Bent along the track"), &mode, 0));
	ImGui::SameLine();
	edited(ImGui::RadioButton(STR_C("Repeated"), &mode, 1));
	settings.bend = mode == 0;
	if (false == settings.bend)
	{
		ImGui::SameLine();
		ImGui::SetNextItemWidth(80.0f);
		edited(ImGui::InputDouble(STR_C("every (m, 0: its length)"), &settings.step, 0.0, 0.0, "%.2f"));
		settings.step = std::max(0.0, settings.step);
	}
	ImGui::TextDisabled("%s", STR_C("Platform edge:"));
	for (auto const height : {0.55, 0.76, 0.96})
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(format("%.2f m##platform", height).c_str()))
		{
			settings.lateral = std::copysign(1.725, settings.lateral == 0.0 ? 1.0 : settings.lateral);
			settings.side_anchor = 1;
			settings.height = height;
			settings.height_anchor = 3;
			settings.face = true;
			settings.widen = true;
			changed = true;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The edge of the platform 1.725 m from the axis of the track, its top this high over the rail top"));
	}
	ImGui::TextDisabled(STR_C("%.2f m %s of the axis, %.2f m over the rail top"), std::abs(settings.lateral), settings.lateral >= 0.0 ? STR_C("right") : STR_C("left"), settings.height);
	ImGui::TextWrapped("%s", STR_C("Press beside the track where the model begins and drag along to where it ends; the side of the press is the side of the model. "
	                               "Past the end of the path it goes on along the joined ones. A click lays one copy."));

	if (tool.edited != nullptr && changed)
		sweep_apply();
	if (tool.edited == nullptr && false == tool.curve.empty() && tool.model_length > 0.0 && ImGui::Button(STR_C("Lay it along the selected path")))
	{
		tool.length_mode = 2;
		settings.from = 0.0;
		settings.to = -1.0;
		sweep_create();
	}
	if (tool.edited != nullptr && false == tool.curve.empty() && ImGui::Button(STR_C("Lay it again along the selected line")))
	{
		auto state{sweep_definition()};
		EditorSnapshot snap;
		snap.action = EditorSnapshot::Action::Other;
		snap.node_name = tool.edited->name();
		snap.sweeps.emplace_back(tool.edited, tool.edited->definition());
		trim_history();
		m_history.push_back(std::move(snap));
		g_redo.clear();
		tool.edited->define(state);
		tool.edited->mark_dirty();
		if (track != nullptr)
			sweeps_captured(*track);
	}

	changed = false;
	if (ImGui::CollapsingHeader(STR_C("More settings")))
	{
		ImGui::SetNextItemWidth(200.0f);
		edited(ImGui::InputText(STR_C("Skin"), tool.skin, sizeof(tool.skin), ImGuiInputTextFlags_EnterReturnsTrue));
		edited(ImGui::IsItemDeactivatedAfterEdit());
		ImGui::PushItemWidth(90.0f);
		edited(ImGui::InputDouble(STR_C("Sideways (m, + right)"), &settings.lateral, 0.0, 0.0, "%.3f"));
		ImGui::SameLine();
		ImGui::SetNextItemWidth(150.0f);
		char const *sides[] = {STR_C("model origin"), STR_C("side facing the track"), STR_C("far side"), STR_C("middle")};
		edited(ImGui::Combo("##sweepsideanchor", &settings.side_anchor, sides, IM_ARRAYSIZE(sides)));
		edited(ImGui::InputDouble(STR_C("Up (m, from the rail top)"), &settings.height, 0.0, 0.0, "%.3f"));
		ImGui::SameLine();
		ImGui::SetNextItemWidth(150.0f);
		char const *heights[] = {STR_C("model origin"), STR_C("bottom"), STR_C("top"), STR_C("top of the edge by the track")};
		edited(ImGui::Combo("##sweepheightanchor", &settings.height_anchor, heights, IM_ARRAYSIZE(heights)));
		if (settings.bend)
		{
			edited(ImGui::InputDouble(STR_C("Copy length (m, 0: the model's)"), &settings.step, 0.0, 0.0, "%.2f"));
			settings.step = std::max(0.0, settings.step);
		}
		ImGui::PopItemWidth();
		char const *axes[] = {"auto", "z", "x"};
		ImGui::SetNextItemWidth(70.0f);
		if (ImGui::Combo(STR_C("Axis of the model along the track"), &tool.axis, axes, IM_ARRAYSIZE(axes)))
		{
			tool.measured.clear();
			changed = true;
		}
		edited(ImGui::Checkbox(STR_C("Edge to the track"), &settings.face));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The model is turned so its -x side faces the track, the way the platforms and the walls of the scenery are made"));
		ImGui::SameLine();
		edited(ImGui::Checkbox(STR_C("Turned around"), &settings.flip));
		ImGui::SameLine();
		edited(ImGui::Checkbox(STR_C("Mirrored"), &settings.mirror));
		ImGui::SameLine();
		edited(ImGui::Checkbox(STR_C("Leans with the cant"), &settings.tilt));
		edited(ImGui::Checkbox(STR_C("Set back by the widening of the structure gauge in the curves"), &settings.widen));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The offset grows by the widening of the gauge below the platforms: 3.75/R in the curves of R >= 250 m,\n"
			                              "more in the sharper ones, and on the inner side of a canted curve by the cant times the height of the edge / 1.5;\n"
			                              "it begins 20 m (inner side) and 26 m (outer side) ahead of the curve, as in the gauge check"));
		ImGui::TextDisabled("%s", STR_C("Length"));
		ImGui::SameLine();
		edited(ImGui::RadioButton(STR_C("as the original"), &tool.length_mode, 0));
		ImGui::SameLine();
		edited(ImGui::RadioButton(STR_C("given"), &tool.length_mode, 1));
		ImGui::SameLine();
		edited(ImGui::RadioButton(STR_C("the whole stretch"), &tool.length_mode, 2));
		ImGui::PushItemWidth(90.0f);
		edited(ImGui::InputDouble(STR_C("From (m)"), &settings.from, 0.0, 0.0, "%.1f"));
		ImGui::SameLine();
		if (tool.length_mode == 1)
			edited(ImGui::InputDouble(STR_C("Length (m)"), &tool.length, 0.0, 0.0, "%.1f"));
		else if (tool.length_mode == 2)
			edited(ImGui::InputDouble(STR_C("To (m, -1: the end)"), &settings.to, 0.0, 0.0, "%.1f"));
		ImGui::PopItemWidth();
		settings.from = std::max(0.0, settings.from);
		if (tool.model_length > 0.0)
		{
			auto const state{sweep_definition()};
			auto const end{state.to < 0.0 ? tool.curve_length : std::min(state.to, tool.curve_length)};
			auto const stretch{std::max(0.0, end - state.from)};
			auto const own{settings.step > 0.01 ? settings.step : tool.model_length};
			auto const copies{settings.bend ? std::max(1, static_cast<int>(std::round(stretch / own))) : static_cast<int>(std::floor(stretch / own + 1e-6)) + 1};
			if (settings.bend)
				ImGui::TextDisabled(STR_C("%.1f m: %d copies of %.2f m (the original %.2f m)"), stretch, copies, stretch / copies, tool.model_length);
			else
				ImGui::TextDisabled(STR_C("%.1f m: %d copies every %.2f m"), stretch, copies, own);
		}
		if (tool.edited != nullptr && changed)
			sweep_apply();
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

void editor_mode::draw_sweep_overlay() const
{
	auto const &tool{m_sweep};
	if (false == tool.open || tool.outline.size() < 2)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	auto const &settings{tool.settings};
	auto const from{std::max(0.0, settings.from)};
	auto const to{settings.to < 0.0 ? tool.curve_length : std::min(settings.to, tool.curve_length)};
	glm::dvec3 previous{0.0};
	bool started{false};
	for (std::size_t i = 1; i < tool.outline.size(); ++i)
	{
		if (tool.stations[i] < from || tool.stations[i - 1] > to)
			continue;
		auto const direction{plan_of(tool.outline[i] - tool.outline[i - 1])};
		if (glm::length(direction) < 1e-9)
			continue;
		auto const unit{glm::normalize(direction)};
		glm::dvec3 const left{-unit.y, 0.0, unit.x};
		auto const point{tool.outline[i] + left * settings.lateral + glm::dvec3{0.0, settings.height + kRailTop, 0.0}};
		if (started)
			projection.line(drawlist, previous, point, IM_COL32(255, 160, 40, 230), 3.0f);
		else
		{
			ImVec2 screen;
			if (projection.project(point, screen))
				drawlist->AddCircleFilled(screen, 5.0f, IM_COL32(255, 160, 40, 230));
		}
		previous = point;
		started = true;
	}
	ImVec2 screen;
	if (started && projection.project(previous, screen))
		drawlist->AddCircle(screen, 6.0f, IM_COL32(255, 160, 40, 230), 12, 2.0f);
}
