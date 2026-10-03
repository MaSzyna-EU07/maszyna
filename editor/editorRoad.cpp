/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorRoad.hpp"

#include "editor/editorTrack.hpp"
#include "world/Track.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>

namespace
{

double const kSamePoint{0.25}; // ends of two pieces this close are taken for joined
double const kPieceLength{50.0}; // longest piece the planner makes
double const kPieceTurn{glm::radians(30.0)}; // largest turn of a single piece
double const kShortestRoad{2.0};
double const kTightestRadius{6.0};
double const kCornerRoom{4.0}; // length of an arm of a junction past the point where its edge meets the edge of the next one, taken by the rounded corner
double const kSharpestBranch{glm::radians(30.0)}; // smallest angle between two roads leaving a junction
double const kWidestCorner{glm::radians(120.0)}; // arms further apart than this don't have to make room for each other
double const kTransitionLength{30.0}; // length of the road taken by a change of its lanes

void add_unique(std::vector<road_node *> &Roads, road_node *Road)
{
	if (Road != nullptr && std::find(Roads.begin(), Roads.end(), Road) == Roads.end())
		Roads.emplace_back(Road);
}

glm::dvec3 axis_end(road_node::state const &State, bool const Atend)
{
	return State.axis.points[Atend ? segment_data::point::end : segment_data::point::start];
}

// roads other than the specified one, with an end at specified point
std::vector<road_node *> roads_at(glm::dvec3 const &Point, road_node const *Exclude)
{
	std::vector<road_node *> result;
	for (auto *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road == Exclude || road->m_editorremoved)
			continue;
		for (auto const atend : {false, true})
		{
			if (glm::distance(axis_end(road->definition(), atend), Point) < kSamePoint)
			{
				result.emplace_back(road);
				break;
			}
		}
	}
	return result;
}

// junctions with an arm at specified point
std::vector<junction_node *> junctions_at(glm::dvec3 const &Point)
{
	std::vector<junction_node *> result;
	for (auto *junction : simulation::Junctions.sequence())
	{
		if (junction == nullptr || junction->m_editorremoved)
			continue;
		for (auto const &arm : junction->definition().arms)
		{
			if (glm::distance(arm.position, Point) < kSamePoint)
			{
				result.emplace_back(junction);
				break;
			}
		}
	}
	return result;
}

void add_unique(std::vector<junction_node *> &Junctions, junction_node *Junction)
{
	if (Junction != nullptr && std::find(Junctions.begin(), Junctions.end(), Junction) == Junctions.end())
		Junctions.emplace_back(Junction);
}

// part of a path between two values of its curve parameter
segment_data subpath(segment_data const &Path, double const From, double const To)
{
	segment_data part{Path};
	auto const &p0{Path.points[segment_data::point::start]};
	auto const &p3{Path.points[segment_data::point::end]};
	part.rolls = {static_cast<float>(Path.rolls[0] + (Path.rolls[1] - Path.rolls[0]) * From), static_cast<float>(Path.rolls[0] + (Path.rolls[1] - Path.rolls[0]) * To)};
	if (Path.points[segment_data::point::control1] == glm::dvec3{0.0} && Path.points[segment_data::point::control2] == glm::dvec3{0.0})
	{
		part.points[segment_data::point::start] = glm::mix(p0, p3, From);
		part.points[segment_data::point::end] = glm::mix(p0, p3, To);
		return part;
	}
	auto const p1{p0 + Path.points[segment_data::point::control1]};
	auto const p2{p3 + Path.points[segment_data::point::control2]};
	auto const point = [&](double const T) {
		auto const u{1.0 - T};
		return u * u * u * p0 + 3.0 * u * u * T * p1 + 3.0 * u * T * T * p2 + T * T * T * p3;
	};
	auto const derivative = [&](double const T) {
		auto const u{1.0 - T};
		return 3.0 * u * u * (p1 - p0) + 6.0 * u * T * (p2 - p1) + 3.0 * T * T * (p3 - p2);
	};
	// a part of a cubic curve is a cubic curve, with the tangents scaled to the part of the parameter range it takes
	part.points[segment_data::point::start] = point(From);
	part.points[segment_data::point::end] = point(To);
	part.points[segment_data::point::control1] = derivative(From) * ((To - From) / 3.0);
	part.points[segment_data::point::control2] = derivative(To) * (-(To - From) / 3.0);
	return part;
}

// true if there's a vehicle on any of the paths turning the traffic back at the loose ends of the road
bool turning(road_node const &Road)
{
	for (auto const *turn : Road.turns())
		if (turn != nullptr && false == turn->Dynamics.empty())
			return true;
	return false;
}

// definition of a part of a piece, between two values of the curve parameter of its axis
road_node::state part(road_node::state const &State, double const From, double const To)
{
	auto result{State};
	result.axis = subpath(State.axis, From, To);
	result.taper = {static_cast<float>(State.scale(From)), static_cast<float>(State.scale(To))};
	return result;
}

// points of a path as a bezier curve, and a point of that curve. unlike the methods of a road definition
// these don't measure the whole axis for each call, which matters for what's done every frame
std::array<glm::dvec3, 4> curve_points(segment_data const &Path)
{
	auto const &p0{Path.points[segment_data::point::start]};
	auto const &p3{Path.points[segment_data::point::end]};
	if (Path.points[segment_data::point::control1] == glm::dvec3{0.0} && Path.points[segment_data::point::control2] == glm::dvec3{0.0})
		return {p0, p0 + (p3 - p0) / 3.0, p3 - (p3 - p0) / 3.0, p3};
	return {p0, p0 + Path.points[segment_data::point::control1], p3 + Path.points[segment_data::point::control2], p3};
}

glm::dvec3 curve_point(std::array<glm::dvec3, 4> const &Points, double const T)
{
	auto const u{1.0 - T};
	return u * u * u * Points[0] + 3.0 * u * u * T * Points[1] + 3.0 * u * T * T * Points[2] + T * T * T * Points[3];
}

int const kStations{64};

// distances along a path for evenly spaced values of its curve parameter
std::vector<double> stations(road_node::state const &State)
{
	std::vector<double> result(kStations + 1, 0.0);
	auto const points{curve_points(State.axis)};
	auto previous{points[0]};
	for (int i = 1; i <= kStations; ++i)
	{
		auto const current{curve_point(points, static_cast<double>(i) / kStations)};
		result[i] = result[i - 1] + glm::distance(previous, current);
		previous = current;
	}
	return result;
}

// adds to a record what a piece or a junction is like, ahead of changing it. the first entry stays, as it's what taking the lot back has to end with
void remember(editor_road::record &Record, road_node *Road)
{
	if (std::none_of(Record.roads.begin(), Record.roads.end(), [Road](std::pair<road_node *, road_node::state> const &Entry) { return Entry.first == Road; }))
		Record.roads.emplace_back(Road, Road->definition());
}

void remember(editor_road::record &Record, junction_node *Junction)
{
	if (std::none_of(Record.junctions.begin(), Record.junctions.end(), [Junction](std::pair<junction_node *, junction_node::state> const &Entry) { return Entry.first == Junction; }))
		Record.junctions.emplace_back(Junction, Junction->definition());
}

// distance along a path for specified value of its curve parameter, and the other way around
double station_at(std::vector<double> const &Stations, double const T)
{
	auto const position{std::clamp(T, 0.0, 1.0) * kStations};
	auto const index{std::min(kStations - 1, static_cast<int>(position))};
	return glm::mix(Stations[index], Stations[index + 1], position - index);
}

double parameter_at(std::vector<double> const &Stations, double const Station)
{
	if (Station <= 0.0)
		return 0.0;
	if (Station >= Stations.back())
		return 1.0;
	auto const index{static_cast<int>(std::distance(Stations.begin(), std::upper_bound(Stations.begin(), Stations.end(), Station)))};
	auto const span{Stations[index] - Stations[index - 1]};
	return (index - 1 + (span > 0.0 ? (Station - Stations[index - 1]) / span : 0.0)) / kStations;
}

// direction seen from above
glm::dvec2 planar(glm::dvec3 const &Direction)
{
	glm::dvec2 const result{Direction.x, Direction.z};
	return glm::length(result) > 1e-9 ? glm::normalize(result) : glm::dvec2{0.0, 1.0};
}

// where a junction made in the middle of a road ends on one of its sides
struct road_cut
{
	road_node *road{nullptr};
	double t{0.0};
	bool keepstart{false}; // what stays of the piece: from its start up to the cut, or from the cut to its end
};

// follows a road from a point of a piece for specified distance, to find where a junction made at that point ends.
// Consumed: receives the pieces past the first one which lie within the distance whole
bool follow(road_node &Road, double const T, bool const Forward, double const Distance, road_cut &Cut, std::vector<road_node *> &Consumed, std::string &Error)
{
	auto *piece{&Road};
	auto from{T};
	auto forward{Forward};
	auto remaining{Distance};
	for (int guard = 0; guard < 64; ++guard)
	{
		auto const &state{piece->definition()};
		auto const lengths{stations(state)};
		auto const here{station_at(lengths, from)};
		auto const available{forward ? lengths.back() - here : here};
		if (remaining <= 0.0 || available >= remaining + kShortestRoad)
		{
			// the cut is put here; with nothing left to go the piece is kept whole
			Cut.road = piece;
			Cut.t = (remaining <= 0.0 ? from : parameter_at(lengths, forward ? here + remaining : here - remaining));
			Cut.keepstart = (false == forward);
			return true;
		}
		// the junction takes this piece up to its end, as what would be left of it is too short to be a road
		remaining = std::max(0.0, remaining - available);
		if (piece != &Road)
			Consumed.emplace_back(piece);
		auto const end{axis_end(state, forward)};
		if (false == junctions_at(end).empty())
		{
			Error = "Too close to a junction";
			return false;
		}
		auto const next{roads_at(end, piece)};
		if (next.size() != 1)
		{
			Error = (next.empty() ? "Too close to the end of the road" : "Too close to a place where roads meet");
			return false;
		}
		if (next.front() == &Road || std::find(Consumed.begin(), Consumed.end(), next.front()) != Consumed.end())
		{
			Error = "The road is too short for a junction";
			return false;
		}
		piece = next.front();
		forward = (glm::distance(axis_end(piece->definition(), false), end) < kSamePoint);
		from = (forward ? 0.0 : 1.0);
	}
	Error = "The road is cut into too many pieces here";
	return false;
}

// arm of a junction for the part of a road left on one side of it. returns: false if the lanes there differ in width
bool cut_arm(road_cut const &Cut, junction_node::arm_data &Arm)
{
	auto const &state{Cut.road->definition()};
	Arm.position = state.point(Cut.t);
	// the way out of the junction is the way the kept part of the piece goes from the cut
	Arm.direction = planar(state.tangent(Cut.t)) * (Cut.keepstart ? -1.0 : 1.0);
	// lanes heading for the cut lead into the junction
	Arm.incoming = (Cut.keepstart ? state.forward : state.backward);
	Arm.outgoing = (Cut.keepstart ? state.backward : state.forward);
	return editor_road::lane_width(state, Cut.t, Arm.width);
}

// how far from the centre of a junction the edges of two of its roads meet, measured along the first one.
// Halfwidth, Otherhalfwidth: of the two roads, Angle: between the ways they leave
double edge_reach(double const Halfwidth, double const Otherhalfwidth, double const Angle)
{
	return (Otherhalfwidth + Halfwidth * std::cos(Angle)) / std::sin(Angle);
}

// the look of a junction goes after the road it's made on
void dress(junction_node::state &Junction, road_node::state const &Road)
{
	Junction.surface = Road.surface;
	Junction.texturelength = Road.texturelength;
	Junction.markings = Road.markings;
	Junction.friction = Road.friction;
	Junction.sounddistance = Road.sounddistance;
	Junction.quality = Road.quality;
	Junction.environment = Road.environment;
}

void retire(std::vector<TTrack *> const &Tracks)
{
	for (auto *track : Tracks)
		if (track != nullptr)
			editor_track::retire(*track);
}

// smallest radius of the bend made by a bezier curve, seen from above; 0 if the curve is practically straight
double tightest_radius(segment_data const &Path)
{
	auto const &p0{Path.points[segment_data::point::start]};
	auto const &p3{Path.points[segment_data::point::end]};
	auto const p1{p0 + Path.points[segment_data::point::control1]};
	auto const p2{p3 + Path.points[segment_data::point::control2]};
	double curvature{0.0};
	for (int i = 0; i <= 16; ++i)
	{
		auto const t{i / 16.0};
		auto const u{1.0 - t};
		auto const first{3.0 * u * u * (p1 - p0) + 6.0 * u * t * (p2 - p1) + 3.0 * t * t * (p3 - p2)};
		auto const second{6.0 * u * (p2 - 2.0 * p1 + p0) + 6.0 * t * (p3 - 2.0 * p2 + p1)};
		auto const speedsquared{first.x * first.x + first.z * first.z};
		if (speedsquared < 1e-9)
			continue;
		curvature = std::max(curvature, std::abs(second.x * first.z - second.z * first.x) / (speedsquared * std::sqrt(speedsquared)));
	}
	return curvature > 1e-5 ? 1.0 / curvature : 0.0;
}

}

bool editor_road::available(std::string *Reason)
{
	if (Global.NvRenderer)
	{
		if (Reason != nullptr)
			*Reason = "Geometry of the roads can't be rebuilt with the experimental renderer";
		return false;
	}
	if (simulation::Region == nullptr)
	{
		if (Reason != nullptr)
			*Reason = "No scenery is loaded";
		return false;
	}
	return true;
}

bool editor_road::can_edit(road_node const &Road, std::string *Reason)
{
	std::string reason;
	if (false == available(&reason))
	{
		// reason was filled by the call
	}
	else if (Road.merged())
	{
		reason = "the scenery wasn't opened for editing (-edit), geometry of the roads it came with can't be taken apart";
	}
	else if (false == scene::Layers.editable(&Road, &reason))
	{
		// reason was filled by the call
	}
	else if (Road.occupied())
	{
		reason = "there are vehicles on it";
	}
	else
	{
		// the paths made to turn the vehicles back at the loose ends of the neighbours go away too
		for (auto const *neighbour : neighbours(Road))
			for (auto const *turn : neighbour->turns())
				if (turn != nullptr && false == turn->Dynamics.empty())
					reason = "there are vehicles turning back next to it";
	}
	if (reason.empty())
	{
		// so do the ways through the junctions it leads to
		for (auto const atend : {false, true})
			for (auto const *junction : junctions_at(axis_end(Road.definition(), atend)))
				if (junction->occupied())
					reason = "there are vehicles on the junction it leads to";
	}
	if (false == reason.empty() && Reason != nullptr)
		*Reason = reason;
	return reason.empty();
}

bool editor_road::can_edit(junction_node const &Junction, std::string *Reason)
{
	std::string reason;
	if (false == available(&reason))
	{
		// reason was filled by the call
	}
	else if (Junction.merged())
	{
		reason = "the scenery wasn't opened for editing (-edit), geometry of the junctions it came with can't be taken apart";
	}
	else if (false == scene::Layers.editable(&Junction, &reason))
	{
		// reason was filled by the call
	}
	else if (Junction.occupied())
	{
		reason = "there are vehicles on it";
	}
	else
	{
		// the roads attached to it get their loose ends closed anew
		for (auto const &arm : Junction.definition().arms)
			for (auto const *road : roads_at(arm.position, nullptr))
				if (turning(*road))
					reason = "there are vehicles turning back on the roads it leads to";
	}
	if (false == reason.empty() && Reason != nullptr)
		*Reason = reason;
	return reason.empty();
}

bool editor_road::can_join(loose_end const &End, std::string *Reason)
{
	// what gets attached takes the place of the paths made for the end as it was
	std::string reason;
	if (End.road != nullptr && turning(*End.road))
		reason = "there are vehicles turning back on the road";
	if (End.junction != nullptr)
	{
		if (End.junction->occupied())
			reason = "there are vehicles on the junction";
		for (auto const &arm : End.junction->definition().arms)
			for (auto const *road : roads_at(arm.position, nullptr))
				if (turning(*road))
					reason = "there are vehicles turning back on the roads the junction leads to";
	}
	if (false == reason.empty() && Reason != nullptr)
		*Reason = reason;
	return reason.empty();
}

std::vector<road_node *> editor_road::neighbours(road_node const &Road)
{
	std::vector<road_node *> result;
	if (Road.m_editorremoved)
		return result;
	for (auto const atend : {false, true})
		for (auto *road : roads_at(axis_end(Road.definition(), atend), &Road))
			add_unique(result, road);
	return result;
}

template <class Change_>
void editor_road::refresh(std::vector<road_node *> const &Roads, std::vector<junction_node *> const &Junctions, Change_ Change)
{
	// the pieces and junctions next to the changed ones keep their shape, but what they're joined with changes.
	// this takes down the paths made for the joints, so they can be made again once the change is done
	std::vector<road_node *> affected;
	std::vector<junction_node *> crossings;
	auto const gather = [&]() {
		std::vector<road_node *> roads{affected};
		std::vector<junction_node *> junctions{crossings};
		for (auto *road : Roads)
		{
			add_unique(roads, road);
			if (road->m_editorremoved)
				continue;
			for (auto *neighbour : neighbours(*road))
				add_unique(roads, neighbour);
			for (auto const atend : {false, true})
				for (auto *junction : junctions_at(axis_end(road->definition(), atend)))
					add_unique(junctions, junction);
		}
		for (auto *junction : Junctions)
			add_unique(junctions, junction);
		// a junction changes what the roads attached to it do at their ends
		for (auto *junction : junctions)
			if (false == junction->m_editorremoved)
				for (auto const &arm : junction->definition().arms)
					for (auto *road : roads_at(arm.position, nullptr))
						add_unique(roads, road);
		for (auto *road : roads)
		{
			if (std::find(affected.begin(), affected.end(), road) == affected.end())
			{
				retire(road->release_turns());
				affected.emplace_back(road);
			}
		}
		for (auto *junction : junctions)
		{
			if (std::find(crossings.begin(), crossings.end(), junction) == crossings.end())
			{
				retire(junction->release_links());
				crossings.emplace_back(junction);
			}
		}
	};
	gather();
	for (auto *road : Roads)
	{
		road->hide();
		retire(road->release_lanes());
	}
	for (auto *junction : Junctions)
		junction->hide();

	Change();

	// the change could have moved things next to other ones
	gather();
	std::vector<TTrack *> lanes;
	for (auto *road : Roads)
	{
		if (road->m_editorremoved)
			continue;
		road->create_lanes();
		road->show();
		lanes.insert(lanes.end(), road->tracks().begin(), road->tracks().end());
	}
	// joins the lanes with the lanes of the neighbours, by the ends they share
	editor_track::commit(lanes);
	for (auto *junction : Junctions)
		if (false == junction->m_editorremoved)
			junction->show();
	// the junctions take the lanes which end at them, what's left loose after that is closed by the roads
	for (auto *junction : crossings)
		if (false == junction->m_editorremoved)
			junction->create_links();
	for (auto *road : affected)
		if (false == road->m_editorremoved)
			road->close_ends();
}

std::vector<road_node *> editor_road::create(std::vector<road_node::state> const &States)
{
	static int counter{0};
	std::vector<road_node *> created;
	if (false == available())
		return created;
	for (auto const &state : States)
	{
		scene::node_data data;
		data.type = "road";
		data.range_max = -1.0;
		data.layer = scene::Layers.active(); // null_handle unless the scenery was opened for editing
		// the name is what the lanes are known by to the vehicles put on them, so each piece needs one of its own
		do
		{
			data.name = "road_" + std::to_string(++counter);
		} while (simulation::Roads.find(data.name) != nullptr);
		auto *road{new road_node(data)};
		road->define(state);
		road->mark_dirty();
		simulation::Roads.insert(road);
		scene::Layers.count(road->layer(), scene::layer_item::track);
		created.emplace_back(road);
	}
	refresh(created, {}, []() {});
	return created;
}

void editor_road::apply(std::vector<std::pair<road_node *, road_node::state>> const &Changes)
{
	std::vector<road_node *> roads;
	for (auto const &change : Changes)
		add_unique(roads, change.first);
	if (roads.empty())
		return;
	refresh(roads, {}, [&]() {
		for (auto const &change : Changes)
		{
			change.first->define(change.second);
			change.first->mark_dirty();
		}
	});
}

void editor_road::remove(std::vector<road_node *> const &Roads)
{
	std::vector<road_node *> roads;
	for (auto *road : Roads)
		if (road != nullptr && false == road->m_editorremoved)
			add_unique(roads, road);
	if (roads.empty())
		return;
	refresh(roads, {}, [&]() {
		for (auto *road : roads)
		{
			road->m_editorremoved = true;
			scene::Layers.count(road->layer(), scene::layer_item::track, -1);
		}
	});
}

void editor_road::revive(std::vector<road_node *> const &Roads)
{
	std::vector<road_node *> roads;
	for (auto *road : Roads)
		if (road != nullptr && road->m_editorremoved)
			add_unique(roads, road);
	if (roads.empty())
		return;
	refresh(roads, {}, [&]() {
		for (auto *road : roads)
		{
			road->m_editorremoved = false;
			road->mark_dirty();
			scene::Layers.count(road->layer(), scene::layer_item::track);
		}
	});
}

void editor_road::apply(std::vector<std::pair<junction_node *, junction_node::state>> const &Changes)
{
	std::vector<junction_node *> junctions;
	for (auto const &change : Changes)
		add_unique(junctions, change.first);
	if (junctions.empty())
		return;
	refresh({}, junctions, [&]() {
		for (auto const &change : Changes)
		{
			if (change.first == nullptr)
				continue;
			change.first->define(change.second);
			change.first->mark_dirty();
		}
	});
}

std::vector<junction_node *> editor_road::create(std::vector<junction_node::state> const &States)
{
	static int counter{0};
	std::vector<junction_node *> created;
	if (false == available())
		return created;
	for (auto const &state : States)
	{
		scene::node_data data;
		data.type = "junction";
		data.range_max = -1.0;
		data.layer = scene::Layers.active(); // null_handle unless the scenery was opened for editing
		do
		{
			data.name = "junction_" + std::to_string(++counter);
		} while (simulation::Junctions.find(data.name) != nullptr);
		auto *junction{new junction_node(data)};
		junction->define(state);
		junction->mark_dirty();
		simulation::Junctions.insert(junction);
		scene::Layers.count(junction->layer(), scene::layer_item::track);
		created.emplace_back(junction);
	}
	refresh({}, created, []() {});
	return created;
}

void editor_road::remove(std::vector<junction_node *> const &Junctions)
{
	std::vector<junction_node *> junctions;
	for (auto *junction : Junctions)
		if (junction != nullptr && false == junction->m_editorremoved)
			add_unique(junctions, junction);
	if (junctions.empty())
		return;
	refresh({}, junctions, [&]() {
		for (auto *junction : junctions)
		{
			junction->m_editorremoved = true;
			scene::Layers.count(junction->layer(), scene::layer_item::track, -1);
		}
	});
}

void editor_road::revive(std::vector<junction_node *> const &Junctions)
{
	std::vector<junction_node *> junctions;
	for (auto *junction : Junctions)
		if (junction != nullptr && junction->m_editorremoved)
			add_unique(junctions, junction);
	if (junctions.empty())
		return;
	refresh({}, junctions, [&]() {
		for (auto *junction : junctions)
		{
			junction->m_editorremoved = false;
			junction->mark_dirty();
			scene::Layers.count(junction->layer(), scene::layer_item::track);
		}
	});
}

road_node *editor_road::split(road_node &Road, double const T)
{
	if (T <= 0.001 || T >= 0.999)
		return nullptr;
	auto first{Road.definition()};
	auto second{first};
	auto const &path{Road.definition().axis};
	auto const p0{path.points[segment_data::point::start]};
	auto const p3{path.points[segment_data::point::end]};
	if (path.points[segment_data::point::control1] == glm::dvec3{0.0} && path.points[segment_data::point::control2] == glm::dvec3{0.0})
	{
		auto const middle{glm::mix(p0, p3, T)};
		first.axis.points[segment_data::point::end] = middle;
		second.axis.points[segment_data::point::start] = middle;
	}
	else
	{
		auto const p1{p0 + path.points[segment_data::point::control1]};
		auto const p2{p3 + path.points[segment_data::point::control2]};
		auto const p01{glm::mix(p0, p1, T)};
		auto const p12{glm::mix(p1, p2, T)};
		auto const p23{glm::mix(p2, p3, T)};
		auto const p012{glm::mix(p01, p12, T)};
		auto const p123{glm::mix(p12, p23, T)};
		auto const middle{glm::mix(p012, p123, T)};
		first.axis.points[segment_data::point::control1] = p01 - p0;
		first.axis.points[segment_data::point::control2] = p012 - middle;
		first.axis.points[segment_data::point::end] = middle;
		second.axis.points[segment_data::point::start] = middle;
		second.axis.points[segment_data::point::control1] = p123 - middle;
		second.axis.points[segment_data::point::control2] = p23 - p3;
	}
	auto const roll{static_cast<float>(path.rolls[0] + (path.rolls[1] - path.rolls[0]) * T)};
	first.axis.rolls[1] = roll;
	second.axis.rolls[0] = roll;
	// the width at the cut is what it was there
	auto const scale{static_cast<float>(Road.definition().scale(T))};
	first.taper[1] = scale;
	second.taper[0] = scale;
	apply(std::vector<std::pair<road_node *, road_node::state>>{{&Road, first}});
	auto const created{create(std::vector<road_node::state>{second})};
	return created.empty() ? nullptr : created.front();
}

std::vector<road_node *> editor_road::chain(road_node &Road)
{
	std::vector<road_node *> result{&Road};
	for (std::size_t idx = 0; idx < result.size(); ++idx)
	{
		for (auto const atend : {false, true})
		{
			// a road goes on only where exactly two pieces meet
			auto const next{roads_at(axis_end(result[idx]->definition(), atend), result[idx])};
			if (next.size() == 1)
				add_unique(result, next.front());
		}
	}
	return result;
}

double editor_road::nearest_parameter(road_node::state const &State, glm::dvec3 const &Point)
{
	auto const points{curve_points(State.axis)};
	auto const distance = [&](double const T) {
		auto const position{curve_point(points, T)};
		return glm::distance(glm::dvec2{position.x, position.z}, glm::dvec2{Point.x, Point.z});
	};
	double best{0.0};
	for (int i = 1; i <= 64; ++i)
		if (distance(i / 64.0) < distance(best))
			best = i / 64.0;
	double step{1.0 / 64.0};
	for (int i = 0; i < 24; ++i)
	{
		step *= 0.5;
		if (best - step >= 0.0 && distance(best - step) < distance(best))
			best -= step;
		else if (best + step <= 1.0 && distance(best + step) < distance(best))
			best += step;
	}
	return best;
}

road_node *editor_road::nearest(glm::dvec3 const &Point, double const Margin)
{
	road_node *result{nullptr};
	double best{std::numeric_limits<double>::max()};
	for (auto *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->m_editorremoved)
			continue;
		auto const &state{road->definition()};
		auto const reach{0.5 * state.width() * std::max(state.taper[0], state.taper[1]) + std::max(state.sides[0].width, state.sides[1].width) + Margin};
		// the axis is no longer than the lines joining its points, which is good enough to tell the road is nowhere near
		auto const points{curve_points(state.axis)};
		if (glm::distance(road->location(), Point) > 0.5 * (glm::distance(points[0], points[1]) + glm::distance(points[1], points[2]) + glm::distance(points[2], points[3])) + reach + 10.0)
			continue;
		auto const position{curve_point(points, nearest_parameter(state, Point))};
		auto const distance{glm::distance(glm::dvec2{position.x, position.z}, glm::dvec2{Point.x, Point.z})};
		// roads passing over or under the point don't count
		if (distance > reach || std::abs(position.y - Point.y) > 3.0)
			continue;
		if (distance < best)
		{
			best = distance;
			result = road;
		}
	}
	return result;
}

junction_node *editor_road::nearest_junction(glm::dvec3 const &Point)
{
	junction_node *result{nullptr};
	double best{std::numeric_limits<double>::max()};
	for (auto *junction : simulation::Junctions.sequence())
	{
		if (junction == nullptr || junction->m_editorremoved)
			continue;
		auto const &state{junction->definition()};
		// the junction takes the ground up to the ends of the roads it's made for
		double reach{0.0};
		for (auto const &arm : state.arms)
			reach = std::max(reach, glm::distance(glm::dvec2{arm.position.x, arm.position.z}, glm::dvec2{state.centre.x, state.centre.z}));
		auto const distance{glm::distance(glm::dvec2{state.centre.x, state.centre.z}, glm::dvec2{Point.x, Point.z})};
		if (distance > reach || std::abs(state.centre.y - Point.y) > 3.0 || distance > best)
			continue;
		best = distance;
		result = junction;
	}
	return result;
}

editor_road::loose_end editor_road::find_end(glm::dvec3 const &Point, double const Radius, bool const Junctions)
{
	loose_end result;
	double best{Radius};
	for (auto *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->m_editorremoved)
			continue;
		auto const &state{road->definition()};
		for (auto const atend : {false, true})
		{
			auto const position{axis_end(state, atend)};
			auto const distance{glm::distance(glm::dvec2{position.x, position.z}, glm::dvec2{Point.x, Point.z})};
			if (distance > best || std::abs(position.y - Point.y) > 3.0 || false == roads_at(position, road).empty() || false == junctions_at(position).empty())
				continue;
			best = distance;
			result = loose_end{};
			result.road = road;
			result.atend = atend;
			result.position = position;
			result.outwards = (atend ? state.tangent(1.0) : -state.tangent(0.0));
		}
	}
	if (false == Junctions)
		return result;
	for (auto *junction : simulation::Junctions.sequence())
	{
		if (junction == nullptr || junction->m_editorremoved)
			continue;
		for (auto const &arm : junction->definition().arms)
		{
			auto const distance{glm::distance(glm::dvec2{arm.position.x, arm.position.z}, glm::dvec2{Point.x, Point.z})};
			if (distance > best || std::abs(arm.position.y - Point.y) > 3.0 || false == roads_at(arm.position, nullptr).empty())
				continue;
			best = distance;
			result = loose_end{};
			result.junction = junction;
			result.position = arm.position;
			result.outwards = glm::dvec3{arm.direction.x, 0.0, arm.direction.y};
			// a road leading away from the junction has the lanes going out of it as the ones going along its axis
			result.forward = arm.outgoing;
			result.backward = arm.incoming;
			result.width = arm.width;
		}
	}
	return result;
}

std::vector<segment_data> editor_road::plan(glm::dvec3 const &Start, glm::dvec2 const *Direction, glm::dvec3 const &Target, glm::dvec2 const *Enddirection, bool const Straight, std::string &Error)
{
	std::vector<segment_data> pieces;
	glm::dvec2 const chord{Target.x - Start.x, Target.z - Start.z};
	auto const span{glm::length(chord)};
	if (span < kShortestRoad)
	{
		Error = "Too short";
		return pieces;
	}
	// straight road, cut into pieces of reasonable length
	auto const straight = [&](glm::dvec3 const &End) {
		auto const count{std::max(1, static_cast<int>(std::ceil(glm::distance(Start, End) / kPieceLength)))};
		for (int i = 0; i < count; ++i)
		{
			segment_data piece;
			piece.points[segment_data::point::start] = glm::mix(Start, End, static_cast<double>(i) / count);
			piece.points[segment_data::point::end] = glm::mix(Start, End, static_cast<double>(i + 1) / count);
			pieces.emplace_back(piece);
		}
	};

	if (Direction != nullptr && Enddirection != nullptr)
	{
		// both ends are tied, which leaves a single curve with the tangents set
		if (glm::dot(*Direction, chord) < 0.1 * span || glm::dot(*Enddirection, chord) < 0.1 * span)
		{
			Error = "The road would have to turn back to meet the end";
			return pieces;
		}
		auto const rise{(Target.y - Start.y) / 3.0};
		segment_data piece;
		piece.points[segment_data::point::start] = Start;
		piece.points[segment_data::point::end] = Target;
		piece.points[segment_data::point::control1] = glm::dvec3{Direction->x * span / 3.0, rise, Direction->y * span / 3.0};
		piece.points[segment_data::point::control2] = glm::dvec3{-Enddirection->x * span / 3.0, -rise, -Enddirection->y * span / 3.0};
		auto const radius{tightest_radius(piece)};
		if (radius == 0.0)
		{
			pieces.clear();
			straight(Target);
			return pieces;
		}
		if (radius < kTightestRadius)
		{
			Error = "The bend is too tight";
			return pieces;
		}
		piece.radius = static_cast<float>(radius);
		pieces.emplace_back(piece);
		return pieces;
	}
	if (Direction == nullptr && Enddirection != nullptr)
	{
		// only the far end is tied: the same road as the one leading back from there, turned around
		glm::dvec2 const back{-*Enddirection};
		auto reversed{plan(Target, &back, Start, nullptr, Straight, Error)};
		for (auto &piece : reversed)
		{
			std::swap(piece.points[segment_data::point::start], piece.points[segment_data::point::end]);
			std::swap(piece.points[segment_data::point::control1], piece.points[segment_data::point::control2]);
		}
		std::reverse(reversed.begin(), reversed.end());
		return reversed;
	}
	if (Direction == nullptr)
	{
		straight(Target);
		return pieces;
	}

	// the road leaves the start in set direction, and reaches the target with an arc
	auto const forward{*Direction};
	glm::dvec2 const left{forward.y, -forward.x};
	auto const along{glm::dot(chord, forward)};
	auto const side{glm::dot(chord, left)};
	if (Straight || std::abs(side) < 0.02 * span)
	{
		if (along < kShortestRoad)
		{
			Error = "The target is behind the start";
			return pieces;
		}
		straight(glm::dvec3{Start.x + forward.x * along, Target.y, Start.z + forward.y * along});
		return pieces;
	}
	auto const turn{2.0 * std::atan2(side, along)}; // positive to the left
	auto const radius{span * span / (2.0 * side)}; // likewise
	if (std::abs(turn) > glm::radians(170.0))
	{
		Error = "The road would have to turn back to reach the target";
		return pieces;
	}
	if (std::abs(radius) < kTightestRadius)
	{
		Error = "The bend is too tight";
		return pieces;
	}
	glm::dvec2 const start{Start.x, Start.z};
	auto const centre{start + left * radius};
	auto const count{std::max(1, static_cast<int>(std::ceil(std::max(std::abs(turn) / kPieceTurn, std::abs(radius * turn) / kPieceLength))))};
	auto const handle{4.0 / 3.0 * std::tan(std::abs(turn) / count / 4.0) * std::abs(radius)};
	auto const rise{(Target.y - Start.y) / count};
	for (int i = 0; i < count; ++i)
	{
		auto const angle0{turn * i / count};
		auto const angle1{turn * (i + 1) / count};
		auto const point0{centre - left * (radius * std::cos(angle0)) + forward * (radius * std::sin(angle0))};
		auto const point1{centre - left * (radius * std::cos(angle1)) + forward * (radius * std::sin(angle1))};
		auto const tangent0{forward * std::cos(angle0) + left * std::sin(angle0)};
		auto const tangent1{forward * std::cos(angle1) + left * std::sin(angle1)};
		segment_data piece;
		piece.points[segment_data::point::start] = glm::dvec3{point0.x, Start.y + rise * i, point0.y};
		piece.points[segment_data::point::end] = glm::dvec3{point1.x, Start.y + rise * (i + 1), point1.y};
		piece.points[segment_data::point::control1] = glm::dvec3{tangent0.x * handle, rise / 3.0, tangent0.y * handle};
		piece.points[segment_data::point::control2] = glm::dvec3{-tangent1.x * handle, -rise / 3.0, -tangent1.y * handle};
		piece.radius = static_cast<float>(std::abs(radius));
		pieces.emplace_back(piece);
	}
	// the arc is worked out from the start, make sure it ends exactly at the target
	pieces.back().points[segment_data::point::end] = Target;
	return pieces;
}

double editor_road::planar_length(segment_data const &Path)
{
	auto const whole{subpath(Path, 0.0, 1.0)};
	auto const &p0{whole.points[segment_data::point::start]};
	auto const &p3{whole.points[segment_data::point::end]};
	if (whole.points[segment_data::point::control1] == glm::dvec3{0.0} && whole.points[segment_data::point::control2] == glm::dvec3{0.0})
		return glm::distance(glm::dvec2{p0.x, p0.z}, glm::dvec2{p3.x, p3.z});
	auto const p1{p0 + whole.points[segment_data::point::control1]};
	auto const p2{p3 + whole.points[segment_data::point::control2]};
	double length{0.0};
	glm::dvec2 previous{p0.x, p0.z};
	for (int i = 1; i <= 32; ++i)
	{
		auto const t{i / 32.0};
		auto const u{1.0 - t};
		auto const point{u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3};
		glm::dvec2 const current{point.x, point.z};
		length += glm::distance(previous, current);
		previous = current;
	}
	return length;
}

double editor_road::end_grade(segment_data const &Path)
{
	auto const &control{Path.points[segment_data::point::control2]};
	if (control != glm::dvec3{0.0})
	{
		auto const run{glm::length(glm::dvec2{control.x, control.z})};
		return run > 1e-6 ? -control.y / run : 0.0;
	}
	auto const length{planar_length(Path)};
	return length > 1e-6 ? (Path.points[segment_data::point::end].y - Path.points[segment_data::point::start].y) / length : 0.0;
}

std::vector<segment_data> editor_road::refine(std::vector<segment_data> const &Pieces, double const Length)
{
	std::vector<segment_data> result;
	for (auto const &piece : Pieces)
	{
		auto const count{std::max(1, static_cast<int>(std::ceil(planar_length(piece) / std::max(1.0, Length) - 0.01)))};
		for (int i = 0; i < count; ++i)
			result.emplace_back(count == 1 ? piece : subpath(piece, static_cast<double>(i) / count, static_cast<double>(i + 1) / count));
	}
	return result;
}

void editor_road::profile(std::vector<segment_data> &Pieces, std::vector<double> Heights, double const *Startgrade, double const *Endgrade)
{
	auto const count{Pieces.size()};
	if (count == 0 || Heights.size() != count + 1)
		return;
	std::vector<double> lengths(count);
	for (std::size_t i = 0; i < count; ++i)
		lengths[i] = std::max(0.01, planar_length(Pieces[i]));
	// the heights in between are evened out, so the road doesn't ride every dip. it's never lowered though,
	// as that would let the ground the heights were taken from come through the surface
	{
		auto const sampled{Heights};
		for (std::size_t i = 1; i < count; ++i)
			Heights[i] = std::max(sampled[i], 0.25 * sampled[i - 1] + 0.5 * sampled[i] + 0.25 * sampled[i + 1]);
	}
	std::vector<double> slopes(count);
	for (std::size_t i = 0; i < count; ++i)
		slopes[i] = (Heights[i + 1] - Heights[i]) / lengths[i];
	// slope at each joint is shared by the pieces on both its sides, which is what rounds the profile
	std::vector<double> grades(count + 1);
	grades.front() = (Startgrade != nullptr ? *Startgrade : slopes.front());
	grades.back() = (Endgrade != nullptr ? *Endgrade : slopes.back());
	for (std::size_t i = 1; i < count; ++i)
	{
		auto const before{slopes[i - 1]};
		auto const after{slopes[i]};
		if (before * after <= 0.0)
		{
			// a crest or a bottom
			grades[i] = 0.0;
			continue;
		}
		auto grade{(before * lengths[i] + after * lengths[i - 1]) / (lengths[i - 1] + lengths[i])};
		// kept from overshooting the heights the road is led through
		auto const limit{3.0 * std::min(std::abs(before), std::abs(after))};
		grades[i] = std::clamp(grade, -limit, limit);
	}
	for (std::size_t i = 0; i < count; ++i)
	{
		auto &piece{Pieces[i]};
		auto &start{piece.points[segment_data::point::start]};
		auto &end{piece.points[segment_data::point::end]};
		auto &control1{piece.points[segment_data::point::control1]};
		auto &control2{piece.points[segment_data::point::control2]};
		start.y = Heights[i];
		end.y = Heights[i + 1];
		if (control1 == glm::dvec3{0.0} && control2 == glm::dvec3{0.0})
		{
			if (std::abs(grades[i] - slopes[i]) < 1e-4 && std::abs(grades[i + 1] - slopes[i]) < 1e-4)
				continue; // even slope, the piece stays a plain straight
			// a straight needs its control points spelled out to bend up or down
			control1 = (end - start) / 3.0;
			control2 = (start - end) / 3.0;
		}
		control1.y = grades[i] * glm::length(glm::dvec2{control1.x, control1.z});
		control2.y = -grades[i + 1] * glm::length(glm::dvec2{control2.x, control2.z});
	}
}

bool editor_road::nearest_joint(glm::dvec3 const &Point, double const Radius, glm::dvec3 &Joint)
{
	auto best{Radius};
	bool found{false};
	for (auto const *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->m_editorremoved)
			continue;
		for (auto const atend : {false, true})
		{
			auto const position{axis_end(road->definition(), atend)};
			auto const distance{glm::distance(glm::dvec2{position.x, position.z}, glm::dvec2{Point.x, Point.z})};
			if (distance > best || std::abs(position.y - Point.y) > 3.0)
				continue;
			best = distance;
			Joint = position;
			found = true;
		}
	}
	return found;
}

std::vector<editor_road::piece_end> editor_road::ends_at(glm::dvec3 const &Point)
{
	std::vector<piece_end> result;
	for (auto *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->m_editorremoved)
			continue;
		for (auto const atend : {false, true})
			if (glm::distance(axis_end(road->definition(), atend), Point) < kSamePoint)
				result.push_back({road, atend});
	}
	return result;
}

std::vector<std::pair<junction_node *, std::size_t>> editor_road::arms_at(glm::dvec3 const &Point)
{
	std::vector<std::pair<junction_node *, std::size_t>> result;
	for (auto *junction : simulation::Junctions.sequence())
	{
		if (junction == nullptr || junction->m_editorremoved)
			continue;
		auto const &arms{junction->definition().arms};
		for (std::size_t arm = 0; arm < arms.size(); ++arm)
			if (glm::distance(arms[arm].position, Point) < kSamePoint)
				result.emplace_back(junction, arm);
	}
	return result;
}

void editor_road::settle(road_node &Road, record &Record, std::string &Notes)
{
	for (auto const atend : {false, true})
	{
		if (Road.m_editorremoved)
			return;
		auto const &layout{Road.definition()};
		auto const position{axis_end(layout, atend)};
		// lanes heading for this end of the piece, and the ones leading away from it
		auto const arriving{atend ? layout.forward : layout.backward};
		auto const leaving{atend ? layout.backward : layout.forward};
		float lanewidth{0.f};
		auto const evenlanes{lane_width(layout, atend ? 1.0 : 0.0, lanewidth)};
		char const *unevenlanes{" differ in width, and a junction takes roads with lanes of the same width. "};
		std::string reason;

		auto const junctions{arms_at(position)};
		if (false == junctions.empty())
		{
			// a junction is made for the lanes of the road at each arm
			for (auto const &entry : junctions)
			{
				auto *junction{entry.first};
				auto state{junction->definition()};
				auto &arm{state.arms[entry.second]};
				if (arm.incoming == arriving && arm.outgoing == leaving && std::abs(arm.width - lanewidth) < 0.001f)
					continue;
				if (false == evenlanes)
				{
					Notes += "Lanes of \"" + Road.name() + "\"" + unevenlanes;
					continue;
				}
				if (false == can_edit(*junction, &reason))
				{
					Notes += "\"" + junction->name() + "\" wasn't fitted: " + reason + ". ";
					continue;
				}
				arm.incoming = arriving;
				arm.outgoing = leaving;
				arm.width = lanewidth;
				remember(Record, junction);
				apply(std::vector<std::pair<junction_node *, junction_node::state>>{{junction, state}});
			}
			continue;
		}

		auto const neighbours{roads_at(position, &Road)};
		if (neighbours.size() != 1)
			continue;
		auto *other{neighbours.front()};
		auto const otherlayout{other->definition()};
		auto const otheratend{glm::distance(axis_end(otherlayout, true), position) < kSamePoint};
		// lanes of the neighbour heading for the place they meet, and the ones leading away from it
		auto const otherarriving{otheratend ? otherlayout.forward : otherlayout.backward};
		auto const otherleaving{otheratend ? otherlayout.backward : otherlayout.forward};
		if (otherarriving == leaving && otherleaving == arriving)
		{
			// the same lanes: the neighbour is made as wide as the piece where they meet, and gets to that width along its length
			auto const width{layout.width() * layout.taper[atend ? 1 : 0]};
			if (std::abs(width - otherlayout.width() * otherlayout.taper[otheratend ? 1 : 0]) < 0.001)
				continue;
			if (false == can_edit(*other, &reason))
			{
				Notes += "\"" + other->name() + "\" wasn't fitted: " + reason + ". ";
				continue;
			}
			auto state{otherlayout};
			state.taper[otheratend ? 1 : 0] = std::clamp(static_cast<float>(width / otherlayout.width()), 0.25f, 4.f);
			remember(Record, other);
			apply(std::vector<std::pair<road_node *, road_node::state>>{{other, state}});
			continue;
		}

		// other lanes: the end of the neighbour is given up for a junction of the two, where the lanes of one are led to the lanes of the other
		if (false == evenlanes)
		{
			Notes += "Lanes of \"" + Road.name() + "\"" + unevenlanes;
			continue;
		}
		road_cut cut;
		std::vector<road_node *> consumed;
		std::string error;
		bool room{false};
		for (auto const length : {kTransitionLength, 0.5 * kTransitionLength})
		{
			consumed.clear();
			if (follow(*other, otheratend ? 1.0 : 0.0, false == otheratend, length, cut, consumed, error))
			{
				room = true;
				break;
			}
		}
		if (false == room)
		{
			Notes += "There's no room next to \"" + Road.name() + "\" for the lanes to change (" + error + "), so they don't lead on there. ";
			continue;
		}
		if (cut.road == &Road)
		{
			Notes += "The road is too short for its lanes to change. ";
			continue;
		}
		std::vector<road_node *> removed{consumed};
		if (cut.road != other)
			removed.emplace_back(other);
		// a cut at the very end the piece is kept from leaves it whole
		auto const trimmed{cut.keepstart ? cut.t < 0.999 : cut.t > 0.001};
		bool allowed{false == trimmed || can_edit(*cut.road, &reason)};
		for (auto const *piece : removed)
			allowed = allowed && can_edit(*piece, &reason);
		junction_node::state junction;
		junction_node::arm_data arms[2];
		if (false == allowed)
		{
			Notes += "The lanes next to \"" + Road.name() + "\" weren't fitted: " + reason + ". ";
			continue;
		}
		if (false == cut_arm(cut, arms[0]))
		{
			Notes += "Lanes of \"" + cut.road->name() + "\"" + unevenlanes;
			continue;
		}
		arms[1].position = position;
		arms[1].direction = planar(layout.tangent(atend ? 1.0 : 0.0)) * (atend ? -1.0 : 1.0);
		arms[1].incoming = arriving;
		arms[1].outgoing = leaving;
		arms[1].width = lanewidth;
		junction.arms.assign(std::begin(arms), std::end(arms));
		junction.centre = 0.5 * (arms[0].position + arms[1].position);
		dress(junction, layout);
		// it's a stretch of the road, with the speed limit the road has
		junction.velocity = std::min(layout.velocity > 0.f ? layout.velocity : std::numeric_limits<float>::max(), otherlayout.velocity > 0.f ? otherlayout.velocity : std::numeric_limits<float>::max());
		if (junction.velocity == std::numeric_limits<float>::max())
			junction.velocity = -1.f;
		junction.normalize();

		// room is made first, so nothing gets joined with what's about to go
		remove(removed);
		Record.roads_removed.insert(Record.roads_removed.end(), removed.begin(), removed.end());
		if (trimmed)
		{
			remember(Record, cut.road);
			apply(std::vector<std::pair<road_node *, road_node::state>>{{cut.road, cut.keepstart ? part(cut.road->definition(), 0.0, cut.t) : part(cut.road->definition(), cut.t, 1.0)}});
		}
		auto const created{create(std::vector<junction_node::state>{junction})};
		Record.junctions_created.insert(Record.junctions_created.end(), created.begin(), created.end());
	}
}

bool editor_road::lane_width(road_node::state const &State, double const T, float &Width)
{
	if (State.lanes.empty())
		return false;
	for (auto const &lane : State.lanes)
		if (std::abs(lane.width - State.lanes.front().width) > 0.005f)
			return false;
	Width = static_cast<float>(State.lanes.front().width * State.scale(T));
	return true;
}

bool editor_road::branch_from(road_node &Road, double const T, glm::dvec2 const &Direction, int const Incoming, int const Outgoing, float const Width, branch &Result, std::string &Error)
{
	Result = branch{};
	auto const &layout{Road.definition()};
	auto const centre{layout.point(T)};
	auto const along{planar(layout.tangent(T))};
	auto const direction{glm::length(Direction) > 1e-9 ? glm::normalize(Direction) : glm::dvec2{0.0, 1.0}};
	// the sharper the angle the road leaves at, the further from the centre the edges of the roads part
	auto const angle{std::acos(std::clamp(std::abs(glm::dot(direction, along)), 0.0, 1.0))};
	if (angle < kSharpestBranch)
	{
		Error = "The road would leave at too sharp an angle";
		return false;
	}
	float lanewidth{0.f};
	if (false == lane_width(layout, T, lanewidth))
	{
		Error = "Lanes of \"" + Road.name() + "\" differ in width, and a junction takes roads with lanes of the same width";
		return false;
	}
	auto const halfwidth{0.5 * (layout.forward + layout.backward) * lanewidth};
	auto const branchhalfwidth{0.5 * (Incoming + Outgoing) * Width};
	auto const reach{edge_reach(halfwidth, branchhalfwidth, angle) + kCornerRoom};
	auto const branchreach{edge_reach(branchhalfwidth, halfwidth, angle) + kCornerRoom};

	// where the road is cut on both sides of the centre
	road_cut cuts[2];
	std::vector<road_node *> consumed;
	if (false == follow(Road, T, false, reach, cuts[0], consumed, Error) || false == follow(Road, T, true, reach, cuts[1], consumed, Error))
		return false;
	if (cuts[0].road == cuts[1].road && cuts[0].road != &Road)
	{
		Error = "The road is too short for a junction";
		return false;
	}
	// what's left of the pieces
	if (cuts[0].road == &Road && cuts[1].road == &Road)
	{
		Result.changes.emplace_back(&Road, part(layout, 0.0, cuts[0].t));
		Result.added.emplace_back(part(layout, cuts[1].t, 1.0));
	}
	else if (cuts[0].road != &Road && cuts[1].road != &Road)
	{
		Result.removed.emplace_back(&Road);
	}
	for (auto const &cut : cuts)
	{
		if (cut.road == &Road && cuts[0].road == cuts[1].road)
			continue;
		// a cut at the very end the piece is kept from leaves it whole
		if (cut.keepstart ? cut.t < 0.999 : cut.t > 0.001)
			Result.changes.emplace_back(cut.road, cut.keepstart ? part(cut.road->definition(), 0.0, cut.t) : part(cut.road->definition(), cut.t, 1.0));
	}
	for (auto *piece : consumed)
		Result.removed.emplace_back(piece);
	std::string reason;
	for (auto const &change : Result.changes)
	{
		if (false == can_edit(*change.first, &reason))
		{
			Error = "\"" + change.first->name() + "\" can't be changed: " + reason;
			return false;
		}
	}
	for (auto const *piece : Result.removed)
	{
		if (false == can_edit(*piece, &reason))
		{
			Error = "\"" + piece->name() + "\" can't be changed: " + reason;
			return false;
		}
	}

	// the junction: the road on both its sides, and the one leaving
	auto &junction{Result.layout};
	junction.centre = centre;
	for (auto const &cut : cuts)
	{
		junction_node::arm_data arm;
		if (false == cut_arm(cut, arm))
		{
			Error = "Lanes of \"" + cut.road->name() + "\" differ in width, and a junction takes roads with lanes of the same width";
			return false;
		}
		junction.arms.emplace_back(arm);
	}
	junction_node::arm_data arm;
	arm.position = centre + glm::dvec3{direction.x, 0.0, direction.y} * branchreach;
	arm.direction = direction;
	arm.incoming = Incoming;
	arm.outgoing = Outgoing;
	arm.width = Width;
	junction.arms.emplace_back(arm);
	Result.arm = junction.arms.size() - 1;
	dress(junction, layout);
	junction.normalize();
	return true;
}

bool editor_road::branch_from(junction_node &Junction, glm::dvec2 const &Direction, int const Incoming, int const Outgoing, float const Width, branch &Result, std::string &Error)
{
	Result = branch{};
	auto const &layout{Junction.definition()};
	if (layout.arms.size() >= 4)
	{
		Error = "The junction has four roads already";
		return false;
	}
	std::string reason;
	if (false == can_edit(Junction, &reason))
	{
		Error = "\"" + Junction.name() + "\" can't be changed: " + reason;
		return false;
	}
	auto const direction{glm::length(Direction) > 1e-9 ? glm::normalize(Direction) : glm::dvec2{0.0, 1.0}};
	auto const halfwidth{0.5 * (Incoming + Outgoing) * Width};
	// the roads which are there stay where they are, so the new one has to fit between them
	auto reach{kCornerRoom};
	for (std::size_t idx = 0; idx < layout.arms.size(); ++idx)
	{
		auto const &other{layout.arms[idx]};
		auto const angle{std::acos(std::clamp(glm::dot(direction, other.direction), -1.0, 1.0))};
		if (angle < kSharpestBranch)
		{
			Error = "The road would leave too close to another road of the junction";
			return false;
		}
		if (angle > kWidestCorner)
			continue;
		auto const otherhalfwidth{0.5 * layout.arm_width(idx)};
		auto const otherreach{glm::distance(glm::dvec2{other.position.x, other.position.z}, glm::dvec2{layout.centre.x, layout.centre.z})};
		if (edge_reach(otherhalfwidth, halfwidth, angle) > otherreach - 0.5)
		{
			Error = "There's no room for the road between the roads the junction has; lead it out at a wider angle to them";
			return false;
		}
		reach = std::max(reach, edge_reach(halfwidth, otherhalfwidth, angle) + kCornerRoom);
	}
	Result.junction = &Junction;
	Result.layout = layout;
	junction_node::arm_data arm;
	arm.position = layout.centre + glm::dvec3{direction.x, 0.0, direction.y} * reach;
	arm.direction = direction;
	arm.incoming = Incoming;
	arm.outgoing = Outgoing;
	arm.width = Width;
	Result.layout.arms.emplace_back(arm);
	Result.arm = Result.layout.arms.size() - 1;
	Result.layout.normalize();
	return true;
}

junction_node *editor_road::carry_out(branch const &Branch, record &Record)
{
	if (Branch.junction != nullptr)
	{
		remember(Record, Branch.junction);
		apply(std::vector<std::pair<junction_node *, junction_node::state>>{{Branch.junction, Branch.layout}});
		return Branch.junction;
	}
	// room is made first, so nothing gets joined with what's about to go
	remove(Branch.removed);
	Record.roads_removed.insert(Record.roads_removed.end(), Branch.removed.begin(), Branch.removed.end());
	for (auto const &change : Branch.changes)
		remember(Record, change.first);
	apply(Branch.changes);
	auto const added{create(Branch.added)};
	Record.roads_created.insert(Record.roads_created.end(), added.begin(), added.end());
	auto const junctions{create(std::vector<junction_node::state>{Branch.layout})};
	if (junctions.empty())
		return nullptr;
	Record.junctions_created.emplace_back(junctions.front());
	return junctions.front();
}

std::vector<std::string> const &editor_road::materials()
{
	static std::vector<std::string> names;
	static bool scanned{false};
	if (scanned)
		return names;
	scanned = true;
	std::set<std::string> found;
	std::error_code error;
	for (std::filesystem::directory_iterator file{Global.asCurrentTexturePath, std::filesystem::directory_options::skip_permission_denied, error}, end; file != end; file.increment(error))
	{
		if (error || false == file->is_regular_file(error))
			continue;
		auto extension{file->path().extension().string()};
		std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		if (extension != ".mat" && extension != ".dds" && extension != ".tga" && extension != ".png" && extension != ".bmp" && extension != ".jpg" && extension != ".ktx" && extension != ".tex")
			continue;
		auto name{file->path().stem().string()};
		std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		// names with spaces can't be put in a scenery file
		if (name.empty() || name.find(' ') != std::string::npos)
			continue;
		found.emplace(name);
	}
	names.assign(found.begin(), found.end());
	return names;
}
