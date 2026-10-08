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
#include "editor/editorSettings.hpp"

#include "rendering/renderer.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "world/Sweep.h"
#include "model/AnimModel.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "utilities/translation.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <regex>
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

glm::dvec3 flat(glm::dvec3 Vector)
{
	Vector.y = 0.0;
	return glm::length(Vector) > 1e-9 ? glm::normalize(Vector) : glm::dvec3{0.0, 0.0, 1.0};
}

// the edges of the platforms in peronowe/ are named by the count of the edges and the height: 1k96w200m, 2k30w..., 1kraw200m55cm,
// old_1k76w..., old2_peron1k30w..., old_peron1k_prosty_h70...; the rest there are the benches, the lamps and such. their side facing the track is the edge
double platform_height(std::string File)
{
	std::transform(File.begin(), File.end(), File.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	std::replace(File.begin(), File.end(), '\\', '/');
	auto const slash{File.rfind('/')};
	if (slash == std::string::npos || File.substr(0, slash).find("peron") == std::string::npos)
		return 0.0;
	static std::regex const pattern{R"(^(?:old2?_)?(?:(?:peron)?\dk(\d\d)|\dkraw\d+m(\d\d)cm|peron.*_h(\d\d))(?:[^\d]|$))"};
	std::smatch match;
	auto const name{File.substr(slash + 1)};
	if (false == std::regex_search(name, match, pattern))
		return 0.0;
	for (std::size_t i = 1; i < match.size(); ++i)
		if (match[i].matched)
			return std::stoi(match[i].str()) / 100.0;
	return 0.0;
}

bool platform_model(std::string const &File)
{
	return platform_height(File) > 0.0;
}

// the edge of the platform 1.725 m from the axis on the side it stands, set back further by the widening of the gauge
void platform_edge(sweep_node::state &State, std::string const &Model, double const Base)
{
	State.lateral = std::copysign(EditorSettings.platform_edge(), State.lateral == 0.0 ? 1.0 : State.lateral);
	State.side_anchor = 1;
	State.height_anchor = 3;
	if (auto const height{platform_height(Model)}; height > 0.0)
		State.height = Base + height;
	State.widen = true;
	State.platform = true;
}

bool platform_distance()
{
	double const distances[]{1.725, 1.675, 1.650};
	char const *labels[]{STR_C("1.725 m: lines of out-of-gauge loads, older platforms"), STR_C("1.675 m: nominal by the 0.55 and 0.76 m platforms"), STR_C("1.650 m: the least allowed in service")};
	auto const current{EditorSettings.platform_edge()};
	int chosen{0};
	for (int i = 0; i < IM_ARRAYSIZE(distances); ++i)
		if (std::abs(current - distances[i]) < std::abs(current - distances[chosen]))
			chosen = i;
	ImGui::SetNextItemWidth(300.0f);
	if (false == ImGui::Combo(STR_C("edge of the platform from the axis"), &chosen, labels, IM_ARRAYSIZE(labels)))
		return false;
	EditorSettings.platform_edge(distances[chosen]);
	EditorSettings.save();
	return true;
}

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
	if (tool.edited == nullptr || tool.edited->definition().model != model)
	{
		if (platform_model(model))
			platform_edge(tool.settings, model, 0.0);
		else
			tool.settings.platform = tool.settings.widen = false;
	}
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
	for (auto wanted{sweep_definition().to}; wanted > tool.curve_length && tool.outline.size() > 1;)
	{
		auto const end{tool.outline.back()};
		auto const ahead{end + flat(end - tool.outline[tool.outline.size() - 2]) * (wanted - tool.curve_length)};
		if (false == sweep_extend(true, ahead))
			break;
	}
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
	m_bend.edited = sweep;
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
	for (auto &entry : states)
	{
		auto *sweep{entry.first};
		if (false == sweep->bridge())
			continue;
		sweep->mark_dirty();
		m_sweep_links.erase(std::remove_if(m_sweep_links.begin(), m_sweep_links.end(), [&](sweep_link const &Link) { return Link.sweep == sweep; }), m_sweep_links.end());
		auto const &pieces{sweep->definition().pieces};
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved)
				continue;
			for (std::size_t i = 0; i < pieces.size(); ++i)
				for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
				{
					auto const along{same_piece(pieces[i], track->m_paths[path])};
					if (along || same_piece(pieces[i], reversed(track->m_paths[path])))
						m_sweep_links.push_back({sweep, i, track, path, false == along});
				}
		}
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
	sweep_curve();
	sweep_scan_models();
	ImGui::TextDisabled("%s", STR_C("A model bent to follow the track, or repeated along it: platforms, walls, fences, barriers, lamps"));

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
			auto const &choices{tool.parameter_choices[id - 1]};
			auto const value{placement ? std::string{"0"} : false == parameter.value.empty() ? parameter.value : choices.empty() ? std::string{"none"} : choices.front()};
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
			platform_edge(settings, tool.model, 0.0);
			settings.height = height;
			settings.face = true;
			changed = true;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("The edge of the platform %.3f m from the axis of the track, its top this high over the rail top"), EditorSettings.platform_edge());
	}
	if (platform_distance() && settings.platform)
	{
		settings.lateral = std::copysign(EditorSettings.platform_edge(), settings.lateral == 0.0 ? 1.0 : settings.lateral);
		changed = true;
	}
	ImGui::TextDisabled(STR_C("%.2f m %s of the axis, %.2f m over the rail top"), std::abs(settings.lateral), settings.lateral >= 0.0 ? STR_C("right") : STR_C("left"), settings.height);
	if (false == tool.curve.empty() && tool.model_length > 0.0 && ImGui::Button(STR_C("Lay it along the selected path")))
		sweep_create();

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
		if (ImGui::Checkbox(STR_C("Platform"), &settings.platform))
		{
			if (settings.platform)
				platform_edge(settings, tool.model, 0.0);
			changed = true;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Puts the edge facing the track at the distance chosen from its axis, set back by the widening of the gauge in the curves.\n"
			                              "Stands on the plane of the rail heads carried on to its edge: lower on the inner side of a canted curve,\n"
			                              "higher on the outer one; on the inner side also set back by the cant times the height of the edge / 1.5"));
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

namespace
{

double const kBendReach{60.0};
double const kBendMargin{1.0};

glm::dmat3 axes_matrix(sweep_node const &Sweep)
{
	return glm::dmat3(Sweep.axes({1.0, 0.0, 0.0}), Sweep.axes({0.0, 1.0, 0.0}), Sweep.axes({0.0, 0.0, 1.0}));
}

double along_anchor(sweep_node const &Sweep, int const Anchor, glm::dvec3 const &Low, glm::dvec3 const &High, glm::dvec3 const &Point)
{
	auto const &state{Sweep.definition()};
	switch (Anchor)
	{
	case 0: return Low.x;
	case 2: return 0.5 * (Low.x + High.x);
	case 3: return High.x;
	case 4: return Sweep.axes(state.scale * Point).x;
	default: return 0.0;
	}
}

int copies_of(sweep_node const &Sweep, double const Own)
{
	auto const &state{Sweep.definition()};
	auto const stretch{Sweep.distance_at(Sweep.end()) - Sweep.distance_at(Sweep.start())};
	auto const length{state.step > 0.01 ? state.step : Own};
	if (length < 1e-3)
		return 1;
	return state.bend ? std::max(1, static_cast<int>(std::round(stretch / length))) : static_cast<int>(std::floor(stretch / length + 1e-6)) + 1;
}

} // namespace

bool editor_mode::bend_source_of_selection(bend_source &Source) const
{
	Source = bend_source{};
	if (auto *model = dynamic_cast<TAnimModel *>(m_node))
	{
		if (model->from_template() || false == scene::Layers.editable(model))
			return false;
		std::string text;
		model->export_as_text(text);
		std::istringstream words(text);
		std::string token;
		double number;
		while (words >> token && token != "model")
			;
		words >> number >> number >> number >> number >> Source.file;
		words >> std::ws;
		if (words.peek() == '"')
		{
			words.get();
			std::getline(words, Source.skin, '"');
		}
		else
			words >> Source.skin;
		if (Source.file.empty() || Source.file == "none")
			return false;
		auto const angles{glm::dvec3(model->Angles())};
		Source.transform = glm::translate(glm::dmat4(1.0), model->location());
		Source.transform = glm::rotate(Source.transform, glm::radians(angles.y), glm::dvec3{0.0, 1.0, 0.0});
		Source.transform = glm::rotate(Source.transform, glm::radians(angles.x), glm::dvec3{1.0, 0.0, 0.0});
		Source.transform = glm::rotate(Source.transform, glm::radians(angles.z), glm::dvec3{0.0, 0.0, 1.0});
		Source.scale = glm::dvec3(model->Scale());
		Source.layer = model->layer();
		Source.model = model;
		return true;
	}
	if (m_instance == 0 || false == m_include.placement || false == m_include.issue.empty() || false == scene::Layers.removable(m_instance))
		return false;
	auto info{m_include.info};
	editor_includes::suggest(m_include.target, info);
	glm::dvec3 position{0.0}, rotation{0.0};
	Source.parameters = m_include.values;
	for (std::size_t i = 0; i < Source.parameters.size(); ++i)
	{
		auto const &role{info.parameter(static_cast<int>(i) + 1).role};
		if (role.rfind("pos.", 0) != 0 && role.rfind("rot.", 0) != 0)
			continue;
		auto const value{std::atof(Source.parameters[i].c_str())};
		auto const axis{role.back() == 'x' ? 0 : role.back() == 'y' ? 1 : 2};
		(role[0] == 'p' ? position : rotation)[axis] = value;
		Source.parameters[i] = "0";
	}
	auto const &included{scene::Layers.instance(m_instance)};
	Source.file = m_include.target;
	Source.transform = glm::translate(glm::dmat4(1.0), included.context.offset + position);
	Source.transform = glm::rotate(Source.transform, glm::radians(rotation.y), glm::dvec3{0.0, 1.0, 0.0});
	Source.transform = glm::rotate(Source.transform, glm::radians(rotation.x), glm::dvec3{1.0, 0.0, 0.0});
	Source.transform = glm::rotate(Source.transform, glm::radians(rotation.z), glm::dvec3{0.0, 0.0, 1.0});
	Source.layer = included.layer;
	Source.instance = m_instance;
	return true;
}

void editor_mode::bend_find_tracks(glm::dvec3 const &Point, void const *Key)
{
	auto &tool{m_bend};
	if (Key == tool.tracks_for && glm::distance(Point, tool.tracks_at) < 1.0)
		return;
	tool.tracks_for = Key;
	tool.tracks_at = Point;
	tool.tracks.clear();
	tool.track = 0;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType == tt_Table || track->m_paths.empty())
			continue;
		auto best{std::numeric_limits<double>::max()};
		for (auto const &path : track->m_paths)
		{
			if (plan_distance(path.points[segment_data::point::start], Point) > 2000.0 + kBendReach)
				continue;
			bezier const curve{path};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 2.0), 8, 400)};
			for (int k = 0; k <= count; ++k)
				best = std::min(best, plan_distance(curve.point(static_cast<double>(k) / count), Point));
		}
		if (best <= kBendReach)
			tool.tracks.emplace_back(track, best);
	}
	std::sort(tool.tracks.begin(), tool.tracks.end(), [](auto const &A, auto const &B) { return A.second < B.second; });
	if (tool.tracks.size() > 12)
		tool.tracks.resize(12);
}

bool editor_mode::bend_definition(bend_source const &Source, TTrack &Track, sweep_node::state &State, std::vector<TTrack *> &Tracks)
{
	auto &tool{m_bend};
	State = sweep_node::state{};
	State.model = Source.file;
	State.skin = Source.skin.empty() ? "none" : Source.skin;
	State.parameters = Source.parameters;
	State.scale = Source.scale;
	State.face = false;
	State.platform = State.widen = platform_model(Source.file);
	State.side_anchor = tool.side_anchor;
	State.height_anchor = tool.height_anchor;
	State.point = tool.point;
	auto const origin{glm::dvec3(Source.transform[3])};
	int path{0};
	{
		auto best{std::numeric_limits<double>::max()};
		for (int i = 0; i < static_cast<int>(Track.m_paths.size()); ++i)
		{
			bezier const curve{Track.m_paths[i]};
			for (int k = 0; k <= 64; ++k)
				if (auto const distance{plan_distance(curve.point(k / 64.0), origin)}; distance < best)
				{
					best = distance;
					path = i;
				}
		}
	}
	std::vector<sweep_tool::piece> records{{&Track, path, true}};
	State.pieces = {Track.m_paths[path]};
	scene::node_data data;
	sweep_node probe{data};
	auto const define = [&]() { probe.define(State); };
	define();
	double lateral;
	auto const forward{flat(probe.frame_at(probe.project(origin, lateral)).forward)};
	glm::dmat3 const rotation{Source.transform};
	auto const ex{flat(rotation * glm::dvec3{1.0, 0.0, 0.0})};
	auto const ez{flat(rotation * glm::dvec3{0.0, 0.0, 1.0})};
	State.along_x = tool.axis == 2 || (tool.axis == 0 && std::abs(glm::dot(ex, forward)) > std::abs(glm::dot(ez, forward)));
	auto const direction{State.along_x ? ex : ez};
	State.flip = (glm::dot(direction, forward) < 0.0) != tool.turned;
	State.lateral = lateral != 0.0 ? lateral : 1e-3;
	define();
	glm::dvec3 low, high, shift;
	if (false == probe.bounds(low, high, shift))
	{
		tool.status = STR("The model has no triangles to bend, or it can't be loaded");
		return false;
	}
	if (State.platform && probe.edge_far())
	{
		State.flip = false == State.flip;
		define();
		probe.bounds(low, high, shift);
	}
	auto const inverse{glm::inverse(axes_matrix(probe))};
	auto const world = [&](glm::dvec3 const &Local) { return glm::dvec3(Source.transform * glm::dvec4(inverse * Local, 1.0)); };
	auto const ends{std::array<glm::dvec3, 2>{world({low.x, 0.0, 0.0}), world({high.x, 0.0, 0.0})}};
	auto const beyond = [&](glm::dvec3 const &Point, bool const Atend) {
		auto const at{probe.frame_at(Atend ? probe.length() : 0.0)};
		auto const along{glm::dot(glm::dvec3{Point.x - at.position.x, 0.0, Point.z - at.position.z}, flat(at.forward))};
		return Atend ? along > -kBendMargin : along < kBendMargin;
	};
	auto const extend = [&](bool const Atend, glm::dvec3 const &Target) {
		auto const &edge{Atend ? records.back() : records.front()};
		auto const &edgepiece{Atend ? State.pieces.back() : State.pieces.front()};
		auto const point{Atend ? edgepiece.points[segment_data::point::end] : edgepiece.points[segment_data::point::start]};
		std::optional<sweep_tool::piece> best;
		segment_data bestpiece;
		auto bestdistance{std::numeric_limits<double>::max()};
		for (auto const &[other, end] : editor_track::connected_points(*edge.track, point))
		{
			if (other == nullptr || other->m_editorremoved || other->eType == tt_Table || false == editor_track::is_supported(*other) || end.path >= static_cast<int>(other->m_paths.size()))
				continue;
			if (std::any_of(records.begin(), records.end(), [&](sweep_tool::piece const &Piece) { return Piece.track == other && Piece.path == end.path; }))
				continue;
			auto const startshere{end.kind == editor_track::point_kind::start};
			auto const forward{Atend ? startshere : false == startshere};
			auto const piece{forward ? other->m_paths[end.path] : reversed(other->m_paths[end.path])};
			bezier const curve{piece};
			auto distance{std::numeric_limits<double>::max()};
			for (int k = 0; k <= 32; ++k)
				distance = std::min(distance, plan_distance(curve.point(k / 32.0), Target));
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
			records.push_back(*best);
			State.pieces.push_back(bestpiece);
		}
		else
		{
			records.insert(records.begin(), *best);
			State.pieces.insert(State.pieces.begin(), bestpiece);
		}
		define();
		return true;
	};
	for (int i = 0; i < 40 && probe.length() < kSweepRouteReach; ++i)
	{
		auto grown{false};
		for (auto const &point : ends)
		{
			if (beyond(point, false) && extend(false, point))
				grown = true;
			else if (beyond(point, true) && extend(true, point))
				grown = true;
		}
		if (false == grown)
			break;
	}
	glm::dvec3 anchor{0.0};
	double station{0.0};
	auto const place = [&]() {
		for (int pass = 0; pass < 3; ++pass)
		{
			define();
			probe.bounds(low, high, shift);
			auto const a{along_anchor(probe, tool.along_anchor, low, high, State.point)};
			anchor = world({a, -shift.y, -shift.z});
			double side;
			station = probe.project(anchor, side);
			for (int k = 0; k < 2; ++k)
			{
				auto const at{probe.frame_at(station)};
				station += glm::dot(glm::dvec3{anchor.x - at.position.x, 0.0, anchor.z - at.position.z}, flat(at.forward));
			}
			auto const at{probe.frame_at(station)};
			side = glm::dot(glm::dvec3{anchor.x - at.position.x, 0.0, anchor.z - at.position.z}, flat(at.left));
			auto const flipped{(side >= 0.0) != (State.lateral >= 0.0)};
			State.lateral = side != 0.0 ? side : 1e-3;
			State.height = anchor.y - at.position.y - probe.rise(at);
			if (false == flipped)
				break;
		}
		define();
		probe.bounds(low, high, shift);
		auto const a{along_anchor(probe, tool.along_anchor, low, high, State.point)};
		auto const first{probe.distance_at(station) - (a - low.x)};
		State.from = probe.station_at(first);
		State.to = probe.station_at(first + high.x - low.x);
	};
	place();
	// the straight model leaves the track where it curves on, short of the length it takes bent along it
	for (int i = 0; i < 40 && probe.length() < kSweepRouteReach; ++i)
	{
		auto const ahead = [&](bool const Atend) {
			auto const at{probe.frame_at(Atend ? probe.length() : 0.0)};
			auto const reach{Atend ? State.to - probe.length() : State.from};
			return at.position + flat(at.forward) * reach;
		};
		if ((State.to > probe.length() + 0.01 && extend(true, ahead(true))) || (State.from < -0.01 && extend(false, ahead(false))))
			place();
		else
			break;
	}
	if (State.platform)
	{
		platform_edge(State, Source.file, kRailTop);
		define();
		anchor = bend_marker(probe);
	}
	if (State.to <= 0.0)
	{
		tool.status = STR("The model lies before the start of the track");
		return false;
	}
	tool.marker = anchor;
	tool.marker_valid = true;
	Tracks.clear();
	for (auto const &record : records)
		Tracks.push_back(record.track);
	auto const residual{glm::degrees(std::acos(std::clamp(std::abs(glm::dot(direction, forward)), 0.0, 1.0)))};
	tool.status = residual > 1.0 ? format(STR_C("The model stands %.1f deg off the track, bent it'll follow the track"), residual) : std::string{};
	return true;
}

void editor_mode::bend_selection()
{
	auto &tool{m_bend};
	bend_source source;
	if (false == bend_source_of_selection(source) || tool.tracks.empty())
		return;
	auto *track{tool.tracks[std::clamp(tool.track, 0, static_cast<int>(tool.tracks.size()) - 1)].first};
	sweep_node::state state;
	std::vector<TTrack *> tracks;
	if (false == bend_definition(source, *track, state, tracks))
		return;
	static int counter{0};
	scene::node_data data;
	data.type = "sweep";
	data.range_max = -1.0;
	data.layer = scene::Layers.accepts(source.layer) ? source.layer : scene::Layers.active();
	do
	{
		data.name = "sweep_" + std::to_string(++counter);
	} while (simulation::Sweeps.find(data.name) != nullptr);
	auto *sweep{new sweep_node(data)};
	sweep->define(state);
	sweep->mark_dirty();
	simulation::Sweeps.insert(sweep);
	scene::Layers.count(sweep->layer(), scene::layer_item::model);
	sweep->show();
	for (auto *path : tracks)
		if (path != nullptr && false == path->m_editorremoved)
			sweeps_captured(*path);
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Bend;
	snap.sweeps_toggled.push_back(sweep);
	snap.layer = source.layer;
	if (source.model != nullptr)
	{
		auto *model{source.model};
		model->export_as_text(snap.serialized);
		snap.node_name = model->name();
		snap.position = model->location();
		snap.rotation = model->Angles();
		snap.scale = model->Scale();
		snap.uuid = model->uuid;
		nullify_history_pointers(model);
		remove_from_hierarchy(model);
		m_node = nullptr;
		m_dragging = false;
		ui()->set_node(nullptr);
		simulation::State.delete_model(model);
	}
	else
	{
		snap.instance = source.instance;
		snap.node_name = source.file;
		scene::Layers.removed(source.instance, true);
		select_include(0);
	}
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	tool.edited = sweep;
	tool.signature.clear();
	tool.status = format(STR_C("%s bent along %.1f m of the track, Ctrl+Z takes it back"), source.file.c_str(), sweep->distance_at(sweep->end()) - sweep->distance_at(sweep->start()));
	WriteLog("Editor: " + tool.status, logtype::generic);
}

void editor_mode::restore_bend(EditorSnapshot &Snapshot, bool const Undo)
{
	if (Snapshot.sweeps_toggled.empty())
		return;
	auto *sweep{Snapshot.sweeps_toggled.front()};
	sweep->m_editorremoved = Undo;
	if (Undo)
		sweep->hide();
	else
		sweep->show();
	sweep->mark_dirty();
	if (Snapshot.instance != 0)
	{
		if (scene::Layers.tracked(Snapshot.instance) && false == scene::Layers.instance(Snapshot.instance).dead)
			scene::Layers.removed(Snapshot.instance, false == Undo);
		select_include(0);
	}
	else if (Undo)
	{
		auto *created{simulation::State.create_model(Snapshot.serialized, Snapshot.node_name, Snapshot.position)};
		if (created != nullptr)
		{
			scene::Layers.move(created, Snapshot.layer);
			created->location(Snapshot.position);
			created->Angles(Snapshot.rotation);
			created->Scale(Snapshot.scale);
			created->uuid = Snapshot.uuid;
			add_to_hierarchy(created);
			Snapshot.node_ptr = created;
			m_node = created;
			ui()->set_node(m_node);
		}
	}
	else if (auto *model{dynamic_cast<TAnimModel *>(find_node_by_any(Snapshot.node_ptr, Snapshot.uuid.to_string(), Snapshot.node_name))})
	{
		nullify_history_pointers(model);
		remove_from_hierarchy(model);
		simulation::State.delete_model(model);
		Snapshot.node_ptr = nullptr;
		m_node = nullptr;
		ui()->set_node(nullptr);
	}
	m_bend.edited = Undo ? nullptr : sweep;
	m_bend.signature.clear();
}

void editor_mode::bend_edit(sweep_node::state const &State)
{
	auto *sweep{m_bend.edited};
	if (sweep == nullptr || sweep->m_editorremoved || State == sweep->definition())
		return;
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = sweep->name();
	snap.sweeps.emplace_back(sweep, sweep->definition());
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	sweep->define(State);
	sweep->mark_dirty();
}

double editor_mode::bend_fixed(sweep_node const &Sweep, glm::dvec3 const &Point) const
{
	auto const from{Sweep.distance_at(Sweep.start())};
	auto const stretch{Sweep.distance_at(Sweep.end()) - from};
	glm::dvec3 low, high, shift;
	if (false == Sweep.bounds(low, high, shift))
		return from;
	switch (m_bend.along_anchor)
	{
	case 0: return from;
	case 2: return from + 0.5 * stretch;
	case 3: return from + stretch;
	default: break;
	}
	auto const own{high.x - low.x};
	auto const scale{Sweep.definition().bend && own > 1e-3 ? stretch / copies_of(Sweep, own) / own : 1.0};
	return from + (along_anchor(Sweep, m_bend.along_anchor, low, high, Point) - low.x) * scale;
}

void editor_mode::bend_stretch(sweep_node const &Sweep, double const Fixed, double const Length, glm::dvec3 const &Point, sweep_node::state &State) const
{
	glm::dvec3 low, high, shift;
	auto first{Fixed};
	switch (m_bend.along_anchor)
	{
	case 0: break;
	case 2: first = Fixed - 0.5 * Length; break;
	case 3: first = Fixed - Length; break;
	default:
		if (Sweep.bounds(low, high, shift))
		{
			auto const own{high.x - low.x};
			auto const step{State.step > 0.01 ? State.step : own};
			auto const copies{State.bend && step > 1e-3 ? std::max(1, static_cast<int>(std::round(Length / step))) : 1};
			auto const scale{State.bend && own > 1e-3 ? Length / copies / own : 1.0};
			first = Fixed - (along_anchor(Sweep, m_bend.along_anchor, low, high, Point) - low.x) * scale;
		}
		break;
	}
	State.from = Sweep.station_at(first);
	State.to = Sweep.station_at(first + Length);
}

void editor_mode::bend_reanchor(sweep_node::state &State, int const Side, int const Height, glm::dvec3 const &Point) const
{
	scene::node_data data;
	sweep_node probe{data};
	probe.define(State);
	glm::dvec3 low, high, before;
	if (false == probe.bounds(low, high, before))
		return;
	auto const stretch{probe.distance_at(probe.end()) - probe.distance_at(probe.start())};
	auto const fixed{probe.station_at(bend_fixed(probe, Point))};
	auto next{State};
	next.side_anchor = Side;
	next.height_anchor = Height;
	next.point = Point;
	probe.define(next);
	glm::dvec3 after;
	if (false == probe.bounds(low, high, after))
		return;
	next.lateral = State.lateral + before.y - after.y;
	next.height = State.height + before.z - after.z;
	probe.define(next);
	auto const whole{State.to < 0.0 && State.from == 0.0};
	bend_stretch(probe, probe.distance_at(fixed), stretch, Point, next);
	if (whole)
	{
		next.from = 0.0;
		next.to = -1.0;
	}
	State = next;
}

glm::dvec3 editor_mode::bend_marker(sweep_node const &Sweep) const
{
	glm::dvec3 low, high, shift;
	if (false == Sweep.bounds(low, high, shift))
		return glm::dvec3{0.0};
	auto const &state{Sweep.definition()};
	auto const from{Sweep.distance_at(Sweep.start())};
	auto const stretch{Sweep.distance_at(Sweep.end()) - from};
	auto const own{high.x - low.x};
	auto const copies{copies_of(Sweep, own)};
	auto const scale{state.bend && own > 1e-3 ? stretch / copies / own : 1.0};
	auto const at{Sweep.frame_at(Sweep.station_at(bend_fixed(Sweep, state.point)))};
	return at.position + at.left * (state.lateral + Sweep.setback(at)) + glm::dvec3{0.0, state.height + Sweep.rise(at), 0.0};
}

void editor_mode::bend_pick(glm::dvec3 const &Point)
{
	auto &tool{m_bend};
	tool.picking = false;
	if (auto *sweep{tool.edited}; sweep != nullptr && false == sweep->m_editorremoved)
	{
		glm::dvec3 low, high, shift;
		if (false == sweep->bounds(low, high, shift))
			return;
		auto const &state{sweep->definition()};
		double lateral;
		auto const station{sweep->project(Point, lateral)};
		auto const at{sweep->frame_at(station)};
		auto const from{sweep->distance_at(sweep->start())};
		auto const stretch{sweep->distance_at(sweep->end()) - from};
		auto const own{high.x - low.x};
		auto const copies{copies_of(*sweep, own)};
		auto const copy{stretch / copies};
		auto const scale{state.bend && own > 1e-3 ? copy / own : 1.0};
		auto const along{std::fmod(std::max(0.0, sweep->distance_at(station) - from), copy)};
		glm::dvec3 const local{low.x + along / scale, lateral - state.lateral - sweep->setback(at) - shift.y,
		                       Point.y - at.position.y - state.height - sweep->rise(at) - shift.z};
		auto const raw{glm::inverse(axes_matrix(*sweep)) * local / state.scale};
		tool.point = raw;
		tool.along_anchor = tool.side_anchor = tool.height_anchor = 4;
		auto next{state};
		bend_reanchor(next, 4, 4, raw);
		bend_edit(next);
	}
	else
	{
		bend_source source;
		if (false == bend_source_of_selection(source))
			return;
		auto const local{glm::dvec3(glm::inverse(source.transform) * glm::dvec4(Point, 1.0))};
		tool.point = local / source.scale;
		tool.along_anchor = tool.side_anchor = tool.height_anchor = 4;
		tool.signature.clear();
	}
	tool.status = format(STR_C("Reference point %.2f %.2f %.2f in the model"), tool.point.x, tool.point.y, tool.point.z);
}

void editor_mode::render_bend()
{
	auto &tool{m_bend};
	if (tool.edited != nullptr && tool.edited->m_editorremoved)
		tool.edited = nullptr;
	ImGui::PushID("bend");
	char const *alongs[] = {STR_C("start"), STR_C("model origin"), STR_C("middle"), STR_C("end"), STR_C("picked point")};
	char const *sides[] = {STR_C("model origin"), STR_C("side facing the track"), STR_C("far side"), STR_C("middle"), STR_C("picked point")};
	char const *heights[] = {STR_C("model origin"), STR_C("bottom"), STR_C("top"), STR_C("top of the edge by the track"), STR_C("picked point")};
	auto anchors = [&](bool &Changed) {
		ImGui::SetNextItemWidth(170.0f);
		Changed |= ImGui::Combo(STR_C("along the track"), &tool.along_anchor, alongs, IM_ARRAYSIZE(alongs));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Reference point: the model bends about it, the line through it keeps its length, the offsets are its place"));
		ImGui::SetNextItemWidth(170.0f);
		Changed |= ImGui::Combo(STR_C("across"), &tool.side_anchor, sides, IM_ARRAYSIZE(sides));
		ImGui::SetNextItemWidth(170.0f);
		Changed |= ImGui::Combo(STR_C("up"), &tool.height_anchor, heights, IM_ARRAYSIZE(heights));
		if (tool.along_anchor == 4 || tool.side_anchor == 4 || tool.height_anchor == 4)
		{
			ImGui::SetNextItemWidth(220.0f);
			glm::vec3 point{tool.point};
			if (ImGui::DragFloat3(STR_C("point (x, y, z of the model)"), &point.x, 0.01f, 0.0f, 0.0f, "%.3f"))
			{
				tool.point = glm::dvec3(point);
				Changed = true;
			}
		}
		if (ImGui::Button(tool.picking ? STR_C("Click the point on the model...") : STR_C("Pick the point on the model")))
			tool.picking = false == tool.picking;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The next click in the view sets the reference point where it lands on the model"));
	};

	if (auto *sweep{tool.edited})
	{
		auto state{sweep->definition()};
		glm::dvec3 low, high, shift;
		auto const measured{sweep->bounds(low, high, shift)};
		auto const own{measured ? high.x - low.x : 0.0};
		auto const from{sweep->distance_at(sweep->start())};
		auto const stretch{sweep->distance_at(sweep->end()) - from};
		ImGui::Text("%s  %s", sweep->name().c_str(), state.model.c_str());
		if (own > 0.0)
			ImGui::TextDisabled(STR_C("%.2f m along the reference line, %d copies (the model is %.2f m)"), stretch, copies_of(*sweep, own), own);
		bool changed{false};
		tool.side_anchor = state.side_anchor;
		tool.height_anchor = state.height_anchor;
		tool.point = state.point;
		auto const side{tool.side_anchor}, height{tool.height_anchor};
		auto const point{tool.point};
		bool anchored{false};
		anchors(anchored);
		if (anchored && (side != tool.side_anchor || height != tool.height_anchor || point != tool.point))
		{
			bend_reanchor(state, tool.side_anchor, tool.height_anchor, tool.point);
			changed = true;
		}
		ImGui::PushItemWidth(110.0f);
		changed |= ImGui::InputDouble(STR_C("from the track axis (m, + right)"), &state.lateral, 0.05, 0.5, "%.3f");
		double overhead{state.height - kRailTop};
		if (ImGui::InputDouble(STR_C("over the rail head (m)"), &overhead, 0.01, 0.1, "%.3f"))
		{
			state.height = overhead + kRailTop;
			changed = true;
		}
		double along{sweep->distance_at(bend_fixed(*sweep, state.point))};
		if (ImGui::InputDouble(STR_C("along the track (m)"), &along, 0.1, 1.0, "%.2f"))
		{
			bend_stretch(*sweep, along, stretch, state.point, state);
			changed = true;
		}
		double length{stretch};
		if (ImGui::InputDouble(STR_C("length (m)"), &length, 0.5, 5.0, "%.2f") && length > 0.1)
		{
			bend_stretch(*sweep, bend_fixed(*sweep, state.point), length, state.point, state);
			changed = true;
		}
		ImGui::PopItemWidth();
		if (own > 0.0 && ImGui::SmallButton(STR_C("As long as the model")))
		{
			bend_stretch(*sweep, bend_fixed(*sweep, state.point), own, state.point, state);
			changed = true;
		}
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Along the whole track")))
		{
			state.from = 0.0;
			state.to = -1.0;
			changed = true;
		}
		int mode{state.bend ? 0 : 1};
		changed |= ImGui::RadioButton(STR_C("Bent"), &mode, 0);
		ImGui::SameLine();
		changed |= ImGui::RadioButton(STR_C("Repeated"), &mode, 1);
		state.bend = mode == 0;
		ImGui::SameLine();
		ImGui::SetNextItemWidth(80.0f);
		changed |= ImGui::InputDouble(STR_C("copy every (m, 0: its length)"), &state.step, 0.0, 0.0, "%.2f");
		state.step = std::max(0.0, state.step);
		changed |= ImGui::Checkbox(STR_C("Turned around"), &state.flip);
		ImGui::SameLine();
		changed |= ImGui::Checkbox(STR_C("Mirrored"), &state.mirror);
		ImGui::SameLine();
		changed |= ImGui::Checkbox(STR_C("Leans with the cant"), &state.tilt);
		changed |= ImGui::Checkbox(STR_C("Set back by the widening of the structure gauge in the curves"), &state.widen);
		if (ImGui::Checkbox(STR_C("Platform"), &state.platform))
		{
			if (state.platform)
			{
				bend_reanchor(state, 1, 3, state.point);
				platform_edge(state, state.model, kRailTop);
			}
			changed = true;
		}
		if (state.platform && platform_distance())
		{
			state.lateral = std::copysign(EditorSettings.platform_edge(), state.lateral);
			changed = true;
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Puts the edge facing the track at the distance chosen from its axis, set back by the widening of the gauge in the curves.\n"
			                              "Stands on the plane of the rail heads carried on to its edge: lower on the inner side of a canted curve,\n"
			                              "higher on the outer one; on the inner side also set back by the cant times the height of the edge / 1.5"));
		ImGui::TextDisabled("%s", STR_C("In the view: drag the blue cross along the track and aside; Shift: height; Ctrl: only along"));
		if (changed)
			bend_edit(state);
		if (ImGui::Button(STR_C("Done")))
			tool.edited = nullptr;
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Delete")))
		{
			sweep_delete(*sweep);
			tool.edited = nullptr;
		}
	}
	else
	{
		bend_source source;
		if (bend_source_of_selection(source))
		{
			void const *key{source.model != nullptr ? static_cast<void const *>(source.model) : reinterpret_cast<void const *>(static_cast<std::uintptr_t>(source.instance))};
			bend_find_tracks(glm::dvec3(source.transform[3]), key);
			ImGui::Text("%s", source.file.c_str());
			if (tool.tracks.empty())
				ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), STR_C("No track within %.0f m of the model"), kBendReach);
			else
			{
				tool.track = std::clamp(tool.track, 0, static_cast<int>(tool.tracks.size()) - 1);
				auto const label = [](std::pair<TTrack *, double> const &Entry) { return format("%s  (%.1f m)", Entry.first->name().empty() ? "(unnamed)" : Entry.first->name().c_str(), Entry.second); };
				ImGui::SetNextItemWidth(220.0f);
				if (ImGui::BeginCombo(STR_C("track"), label(tool.tracks[tool.track]).c_str()))
				{
					for (int i = 0; i < static_cast<int>(tool.tracks.size()); ++i)
						if (ImGui::Selectable(label(tool.tracks[i]).c_str(), i == tool.track))
							tool.track = i;
					ImGui::EndCombo();
				}
				char const *axes[] = {STR_C("the one nearest the track"), "z", "x"};
				ImGui::SetNextItemWidth(170.0f);
				ImGui::Combo(STR_C("axis of the model along the track"), &tool.axis, axes, IM_ARRAYSIZE(axes));
				ImGui::Checkbox(STR_C("Turned around"), &tool.turned);
				if (platform_model(source.file))
					platform_distance();
				bool anchored{false};
				anchors(anchored);
				auto const signature{format("%p %d %d %d %d %d %d %.4f %.4f %.4f %.3f", key, tool.track, tool.axis, tool.turned ? 1 : 0, tool.along_anchor, tool.side_anchor, tool.height_anchor, tool.point.x, tool.point.y, tool.point.z, EditorSettings.platform_edge())};
				if (signature != tool.signature)
				{
					tool.signature = signature;
					tool.marker_valid = false;
					sweep_node::state state;
					std::vector<TTrack *> tracks;
					bend_definition(source, *tool.tracks[tool.track].first, state, tracks);
				}
				if (ImGui::Button(STR_C("Bend it along the track  (B)")))
					bend_selection();
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", STR_C("The model stays where it stands and bends with the track; Ctrl+Z gives it back"));
			}
		}
		else
		{
			tool.marker_valid = false;
			tool.picking = false;
			ImGui::TextDisabled("%s", STR_C("Select a model by a track, or click a bent one"));
		}
		auto *track{selected_track()};
		auto const nearby{track != nullptr ? sweeps_near(*track) : std::vector<sweep_node *>{}};
		std::vector<std::pair<sweep_node *, double>> listed;
		if (track != nullptr)
			for (auto *sweep : nearby)
				listed.emplace_back(sweep, 0.0);
		else
			for (auto *sweep : simulation::Sweeps.sequence())
				if (sweep != nullptr && false == sweep->m_editorremoved)
				{
					auto const distance{glm::distance(sweep->location(), glm::dvec3{Global.pCamera.Pos})};
					if (distance < 0.5 * (sweep->end() - sweep->start()) + 300.0)
						listed.emplace_back(sweep, distance);
				}
		if (false == listed.empty())
		{
			ImGui::Separator();
			ImGui::TextDisabled("%s", track != nullptr ? STR_C("Bent models by this track:") : STR_C("Bent models nearby:"));
			for (auto const &entry : listed)
			{
				auto *sweep{entry.first};
				ImGui::PushID(sweep);
				if (ImGui::Selectable(format("%s  %s, %.0f m", sweep->name().c_str(), sweep->definition().model.c_str(), sweep->end() - sweep->start()).c_str(), false))
				{
					tool.edited = sweep;
					tool.picking = false;
				}
				ImGui::PopID();
			}
		}
		if (track != nullptr && ImGui::TreeNodeEx(STR_C("New model along the selected track")))
		{
			m_sweep.open = true;
			render_sweep_ui();
			ImGui::TreePop();
		}
		else
			m_sweep.open = false;
	}
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
	ImGui::PopID();
}

void editor_mode::draw_bend_overlay() const
{
	auto const &tool{m_bend};
	if (false == tool.label.empty())
	{
		auto const &io{ImGui::GetIO()};
		auto *foreground{ImGui::GetForegroundDrawList()};
		auto const labelsize{ImGui::CalcTextSize(tool.label.c_str())};
		auto const detailsize{tool.details.empty() ? ImVec2(0.0f, 0.0f) : ImGui::CalcTextSize(tool.details.c_str())};
		ImVec2 const size{std::max(labelsize.x, detailsize.x), labelsize.y + (tool.details.empty() ? 0.0f : detailsize.y + 4.0f)};
		ImVec2 position{io.MousePos.x + 18.0f, io.MousePos.y + 18.0f};
		position.x = std::min(position.x, io.DisplaySize.x - size.x - 10.0f);
		position.y = std::min(position.y, io.DisplaySize.y - size.y - 10.0f);
		foreground->AddRectFilled(ImVec2(position.x - 6.0f, position.y - 4.0f), ImVec2(position.x + size.x + 6.0f, position.y + size.y + 4.0f), IM_COL32(0, 0, 0, 170), 4.0f);
		foreground->AddText(position, IM_COL32(40, 220, 255, 255), tool.label.c_str());
		if (false == tool.details.empty())
			foreground->AddText(ImVec2(position.x, position.y + labelsize.y + 4.0f), IM_COL32(255, 255, 255, 235), tool.details.c_str());
	}
	if (auto const *model{dynamic_cast<TAnimModel const *>(m_node)}; model != nullptr && tool.edited == nullptr && false == tool.tracks.empty() && tool.tracks_for == static_cast<void const *>(model))
	{
		screen_projection const projection;
		ImVec2 at;
		if (projection.project(model->location(), at))
		{
			auto const text{format(STR_C("B: bend along %s"), tool.tracks.front().first->name().c_str())};
			auto *foreground{ImGui::GetForegroundDrawList()};
			auto const size{ImGui::CalcTextSize(text.c_str())};
			foreground->AddRectFilled(ImVec2(at.x + 10.0f, at.y - size.y - 14.0f), ImVec2(at.x + size.x + 22.0f, at.y - 6.0f), IM_COL32(0, 0, 0, 150), 4.0f);
			foreground->AddText(ImVec2(at.x + 16.0f, at.y - size.y - 10.0f), IM_COL32(40, 220, 255, 255), text.c_str());
		}
	}
	glm::dvec3 marker;
	if (tool.edited != nullptr && false == tool.edited->m_editorremoved)
		marker = bend_marker(*tool.edited);
	else if (tool.marker_valid)
		marker = tool.marker;
	else
		return;
	screen_projection const projection;
	ImVec2 screen;
	if (false == projection.project(marker, screen))
		return;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	if (tool.over_marker || tool.dragging != 0)
		drawlist->AddCircleFilled(screen, 9.0f, IM_COL32(40, 220, 255, 120), 16);
	drawlist->AddCircle(screen, 9.0f, IM_COL32(40, 220, 255, 255), 16, 2.5f);
	drawlist->AddLine(ImVec2(screen.x - 14.0f, screen.y), ImVec2(screen.x + 14.0f, screen.y), IM_COL32(40, 220, 255, 255), 2.0f);
	drawlist->AddLine(ImVec2(screen.x, screen.y - 14.0f), ImVec2(screen.x, screen.y + 14.0f), IM_COL32(40, 220, 255, 255), 2.0f);
}

bool editor_mode::bend_drag_start(int const Mods)
{
	auto &tool{m_bend};
	auto *sweep{tool.edited};
	if (sweep == nullptr || sweep->m_editorremoved || ImGui::GetIO().WantCaptureMouse)
		return false;
	auto const marker{bend_marker(*sweep)};
	screen_projection const projection;
	ImVec2 screen;
	if (false == projection.project(marker, screen))
		return false;
	auto const &mouse{ImGui::GetIO().MousePos};
	if ((mouse.x - screen.x) * (mouse.x - screen.x) + (mouse.y - screen.y) * (mouse.y - screen.y) > 18.0f * 18.0f)
		return false;
	tool.dragging = (Mods & GLFW_MOD_SHIFT) != 0 ? 2 : (Mods & GLFW_MOD_CONTROL) != 0 ? 3 : 1;
	tool.drag_state = sweep->definition();
	tool.drag_anchor = marker;
	double lateral;
	tool.drag_station = sweep->project(marker, lateral);
	tool.drag_label.clear();
	return true;
}

void editor_mode::bend_drag_update()
{
	auto &tool{m_bend};
	tool.label.clear();
	tool.details.clear();
	tool.hover = nullptr;
	tool.over_marker = false;
	auto const &io{ImGui::GetIO()};
	if (auto *model{dynamic_cast<TAnimModel *>(m_node)}; model != nullptr && tool.edited == nullptr && false == model->from_template() && scene::Layers.editable(model))
		bend_find_tracks(model->location(), model);
	if (tool.dragging == 0 && false == io.WantCaptureMouse)
	{
		if (tool.picking)
			tool.label = STR("Click: the reference point, where it lands on the model");
		else
		{
			if (auto *sweep{tool.edited}; sweep != nullptr && false == sweep->m_editorremoved)
			{
				screen_projection const projection;
				ImVec2 screen;
				if (projection.project(bend_marker(*sweep), screen))
					tool.over_marker = (io.MousePos.x - screen.x) * (io.MousePos.x - screen.x) + (io.MousePos.y - screen.y) * (io.MousePos.y - screen.y) < 14.0f * 14.0f;
			}
			if (tool.over_marker)
			{
				tool.label = STR("Drag: along the track and aside");
				tool.details = STR("Shift: height   Ctrl: only along the track");
			}
			else if (ui()->mode() == nodebank_panel::MODIFY && m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) != GLFW_PRESS)
			{
				tool.hover = sweep_under(Global.pCamera.Pos + GfxRenderer->Mouse_Position());
				if (tool.hover != nullptr && tool.hover != tool.edited)
				{
					tool.label = STR("Click: edit the bent model");
					tool.details = format("%s  %s, %.1f m", tool.hover->name().c_str(), tool.hover->definition().model.c_str(), tool.hover->distance_at(tool.hover->end()) - tool.hover->distance_at(tool.hover->start()));
				}
			}
		}
	}
	if (tool.dragging == 0)
		return;
	if (false == tool.drag_label.empty())
		tool.label = tool.drag_label;
	auto *sweep{tool.edited};
	if (sweep == nullptr || sweep->m_editorremoved)
	{
		tool.dragging = 0;
		return;
	}
	if (false == ImGui::GetIO().MouseDown[0])
	{
		tool.dragging = 0;
		tool.drag_label.clear();
		if (sweep->definition() == tool.drag_state)
			return;
		EditorSnapshot snap;
		snap.action = EditorSnapshot::Action::Other;
		snap.node_name = sweep->name();
		snap.sweeps.emplace_back(sweep, tool.drag_state);
		trim_history();
		m_history.push_back(std::move(snap));
		g_redo.clear();
		sweep->mark_dirty();
		return;
	}
	screen_projection const projection;
	auto const &mouse{ImGui::GetIO().MousePos};
	auto const step{0.01};
	auto const snap = [&](double const Value) { return std::round(Value / step) * step; };
	auto state{tool.drag_state};
	auto const slide = [&](double const Shift) {
		auto const span{(tool.drag_state.to < 0.0 ? sweep->length() : tool.drag_state.to) - tool.drag_state.from};
		auto from{tool.drag_state.from + Shift};
		auto to{from + span};
		if (span > 0.1 && span <= sweep->length() + 1e-6)
		{
			if (from < 0.0)
			{
				to -= from;
				from = 0.0;
			}
			if (to > sweep->length())
			{
				from = std::max(0.0, sweep->length() - span);
				to = from + span;
			}
		}
		state.from = from;
		state.to = to;
	};
	if (tool.dragging == 2)
	{
		glm::dvec3 origin, direction;
		projection.ray(mouse, origin, direction);
		glm::dvec3 normal{Global.pCamera.Pos.x - tool.drag_anchor.x, 0.0, Global.pCamera.Pos.z - tool.drag_anchor.z};
		if (glm::length(normal) < 1e-6)
			return;
		normal = glm::normalize(normal);
		auto const facing{glm::dot(direction, normal)};
		if (std::abs(facing) < 1e-6)
			return;
		auto const hit{origin + direction * (glm::dot(tool.drag_anchor - origin, normal) / facing)};
		state.height = kRailTop + snap(tool.drag_state.height - kRailTop + hit.y - tool.drag_anchor.y);
		tool.drag_label = format(STR_C("%.2f m over the rail head"), state.height - kRailTop);
	}
	else
	{
		glm::dvec3 point;
		if (false == projection.on_level(mouse, tool.drag_anchor.y, point))
			return;
		double lateral;
		auto const station{sweep->project(point, lateral)};
		auto const shift{snap(station - tool.drag_station)};
		slide(shift);
		if (tool.dragging == 1)
		{
			auto const at{sweep->frame_at(station)};
			state.lateral = snap(lateral - sweep->setback(at));
			if (std::abs(state.lateral) < step)
				state.lateral = std::copysign(step, tool.drag_state.lateral);
			tool.drag_label = format(STR_C("%.2f m from the track, %+.2f m along"), std::abs(state.lateral), shift);
		}
		else
			tool.drag_label = format(STR_C("%+.2f m along the track"), shift);
	}
	if (false == (state == sweep->definition()))
		sweep->define(state);
}

sweep_node *editor_mode::sweep_under(glm::dvec3 const &Point) const
{
	sweep_node *best{nullptr};
	auto bestdistance{std::numeric_limits<double>::max()};
	for (auto *sweep : simulation::Sweeps.sequence())
	{
		if (sweep == nullptr || sweep->m_editorremoved)
			continue;
		auto const &state{sweep->definition()};
		if (plan_distance(Point, sweep->location()) > 0.5 * sweep->length() + std::abs(state.lateral) + 30.0)
			continue;
		double lateral;
		auto const station{sweep->project(Point, lateral)};
		if (station < sweep->start() - 0.5 || station > sweep->end() + 0.5)
			continue;
		glm::dvec3 low, high, shift;
		if (false == sweep->bounds(low, high, shift))
			continue;
		auto const at{sweep->frame_at(station)};
		auto const across{lateral - state.lateral - sweep->setback(at) - shift.y};
		auto const up{Point.y - at.position.y - state.height - sweep->rise(at) - shift.z};
		if (across < low.y - 0.3 || across > high.y + 0.3 || up < low.z - 0.3 || up > high.z + 0.3)
			continue;
		auto const distance{std::abs(across - 0.5 * (low.y + high.y))};
		if (distance < bestdistance)
		{
			bestdistance = distance;
			best = sweep;
		}
	}
	return best;
}

bool editor_mode::bend_click()
{
	auto &tool{m_bend};
	if (tool.hover == nullptr || tool.hover == tool.edited || tool.over_marker)
		return false;
	tool.edited = tool.hover;
	tool.picking = false;
	tool.signature.clear();
	m_node = nullptr;
	m_dragging = false;
	select_include(0);
	ui()->set_node(nullptr);
	ui()->expand_bend();
	return true;
}

bool editor_mode::bend_shortcut()
{
	auto &tool{m_bend};
	bend_source source;
	if (false == bend_source_of_selection(source))
		return false;
	void const *key{source.model != nullptr ? static_cast<void const *>(source.model) : reinterpret_cast<void const *>(static_cast<std::uintptr_t>(source.instance))};
	bend_find_tracks(glm::dvec3(source.transform[3]), key);
	if (tool.tracks.empty())
	{
		ui()->set_status(format(STR_C("No track within %.0f m of the model"), kBendReach), true);
		return true;
	}
	bend_selection();
	ui()->expand_bend();
	return true;
}
