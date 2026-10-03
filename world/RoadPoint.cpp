/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/RoadPoint.h"

#include "world/Track.h"
#include "world/Event.h"
#include "world/RoadOrder.h"
#include "vehicle/DynObj.h"
#include "vehicle/Driver.h"
#include "vehicle/Train.h"
#include "model/Model3d.h"
#include "simulation/simulation.h"
#include "rendering/particles.h"
#include "rendering/renderer.h"
#include "scene/scene.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"

namespace
{

constexpr double stopline_width{0.5};
constexpr double stopline_inset{0.15}; // gap between the ends of a stop line and the edges of its lane
constexpr double line_lift{0.02}; // the line is raised over the surface, so the two don't fight for depth
constexpr double scan_period{0.2}; // seconds between looks at the rails of a crossing
constexpr double hold_time{4.0}; // seconds a crossing stays closed after the last train was seen
constexpr double near_zone{30.0}; // a rail vehicle this close to the stop lines keeps the crossing closed whichever way it goes
constexpr double clear_zone{20.0}; // a vehicle isn't put on the road with another one this close
constexpr double retry_period{1.0}; // seconds to the next try, when a vehicle can't be put on the road
constexpr double front_room{2.5}; // half the length a vehicle is presumed to have before it's made
constexpr double queue_reach{250.0}; // how far back from a closed crossing the vehicles standing on the lanes are taken to wait for it
constexpr double stuck_time{15.0}; // seconds a vehicle of a spawn point can stand still before it's taken off the road
constexpr double stuck_speed{0.1}; // m/s, slower than this is standing still
constexpr double reverse_speed{0.5}; // m/s, a vehicle going backwards faster than this is taken off the road
constexpr double stripe_width{0.5}; // of a pedestrian crossing; the gaps are as wide

// a path as a curve: points along it, and distances along it
class path_curve
{

  public:
	explicit path_curve(segment_data const &Path)
	{
		auto const &start{Path.points[segment_data::point::start]};
		auto const &end{Path.points[segment_data::point::end]};
		auto const &control1{Path.points[segment_data::point::control1]};
		auto const &control2{Path.points[segment_data::point::control2]};
		m_points[0] = start;
		m_points[3] = end;
		if (control1 == glm::dvec3{0.0} && control2 == glm::dvec3{0.0})
		{
			m_points[1] = start + (end - start) / 3.0;
			m_points[2] = end - (end - start) / 3.0;
		}
		else
		{
			m_points[1] = start + control1;
			m_points[2] = end + control2;
		}
		m_stations[0] = 0.0;
		auto previous{m_points[0]};
		for (int idx = 1; idx <= samples; ++idx)
		{
			auto const current{point(static_cast<double>(idx) / samples)};
			m_stations[idx] = m_stations[idx - 1] + glm::distance(previous, current);
			previous = current;
		}
	}
	double length() const
	{
		return m_stations[samples];
	}
	glm::dvec3 point(double const T) const
	{
		auto const u{1.0 - T};
		return u * u * u * m_points[0] + 3.0 * u * u * T * m_points[1] + 3.0 * u * T * T * m_points[2] + T * T * T * m_points[3];
	}
	glm::dvec3 direction(double const T) const
	{
		auto const u{1.0 - T};
		auto const result{3.0 * u * u * (m_points[1] - m_points[0]) + 6.0 * u * T * (m_points[2] - m_points[1]) + 3.0 * T * T * (m_points[3] - m_points[2])};
		return glm::length2(result) > 1e-12 ? glm::normalize(result) : glm::dvec3{0.0, 0.0, 1.0};
	}
	// value of the curve parameter for specified distance from the start, and the other way around
	double parameter(double const Station) const
	{
		if (Station <= 0.0)
		{
			return 0.0;
		}
		if (Station >= length())
		{
			return 1.0;
		}
		auto const idx{std::distance(m_stations.begin(), std::upper_bound(m_stations.begin(), m_stations.end(), Station))};
		auto const span{m_stations[idx] - m_stations[idx - 1]};
		return (static_cast<double>(idx - 1) + (span > 0.0 ? (Station - m_stations[idx - 1]) / span : 0.0)) / samples;
	}
	double station(double const T) const
	{
		auto const position{std::clamp(T, 0.0, 1.0) * samples};
		auto const idx{std::min(samples - 1, static_cast<int>(position))};
		return glm::mix(m_stations[idx], m_stations[idx + 1], position - idx);
	}
	// value of the curve parameter for the point nearest to specified one, seen from above; Distance receives how far it is
	double nearest(glm::dvec3 const &Point, double &Distance) const
	{
		auto const distance = [&](double const T) {
			auto const position{point(T)};
			return glm::distance(glm::dvec2{position.x, position.z}, glm::dvec2{Point.x, Point.z});
		};
		double best{0.0};
		for (int idx = 1; idx <= samples; ++idx)
		{
			if (distance(static_cast<double>(idx) / samples) < distance(best))
			{
				best = static_cast<double>(idx) / samples;
			}
		}
		auto step{1.0 / samples};
		for (int pass = 0; pass < 16; ++pass)
		{
			step *= 0.5;
			if (best - step >= 0.0 && distance(best - step) < distance(best))
			{
				best -= step;
			}
			else if (best + step <= 1.0 && distance(best + step) < distance(best))
			{
				best += step;
			}
		}
		Distance = distance(best);
		return best;
	}

  private:
	static constexpr int samples{32};
	std::array<glm::dvec3, 4> m_points;
	std::array<double, samples + 1> m_stations{};
};

// true if the path can't have a point within specified distance from specified one; a cheap way to tell most of the scenery apart
bool out_of_reach(segment_data const &Path, glm::dvec3 const &Point, double const Reach)
{
	auto const &start{Path.points[segment_data::point::start]};
	auto const &end{Path.points[segment_data::point::end]};
	// the curve lies within its control points, which are no further from the start than this
	auto const span{glm::length(Path.points[segment_data::point::control1]) + glm::distance(start, end) + glm::length(Path.points[segment_data::point::control2])};
	return glm::distance(glm::dvec2{start.x, start.z}, glm::dvec2{Point.x, Point.z}) > span + Reach;
}

// an order for the drivers heading for a closed crossing
using stop_event = road_order;

// the road passing under specified point: the one whose axis is nearest. T: receives the value of the curve parameter of its axis there
road_node const *road_under(glm::dvec3 const &Point, double &T)
{
	road_node const *result{nullptr};
	auto best{std::numeric_limits<double>::max()};
	for (auto const *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->m_editorremoved || out_of_reach(road->definition().axis, Point, 50.0))
		{
			continue;
		}
		auto const &layout{road->definition()};
		path_curve const axis{layout.axis};
		double distance{0.0};
		auto const t{axis.nearest(Point, distance)};
		if (distance > 0.5 * layout.span(t) + 1.0 || std::abs(axis.point(t).y - Point.y) > 3.0 || distance >= best)
		{
			continue;
		}
		best = distance;
		result = road;
		T = t;
	}
	return result;
}

// lane of a road with the point nearest to specified one, within specified distance from it. Station: receives the distance of that point from the start of the lane
TTrack *nearest_lane(glm::dvec3 const &Point, double const Reach, double &Station)
{
	TTrack *result{nullptr};
	auto best{Reach};
	for (auto const *road : simulation::Roads.sequence())
	{
		if (road == nullptr || road->m_editorremoved)
		{
			continue;
		}
		for (auto *lane : road->tracks())
		{
			if (lane == nullptr || lane->m_paths.empty() || out_of_reach(lane->m_paths.front(), Point, Reach))
			{
				continue;
			}
			path_curve const curve{lane->m_paths.front()};
			double distance{0.0};
			auto const t{curve.nearest(Point, distance)};
			if (distance >= best || std::abs(curve.point(t).y - Point.y) > 3.0)
			{
				continue;
			}
			best = distance;
			result = lane;
			Station = curve.station(t);
		}
	}
	return result;
}

double planar_distance(glm::dvec3 const &Left, glm::dvec3 const &Right)
{
	return glm::distance(glm::dvec2{Left.x, Left.z}, glm::dvec2{Right.x, Right.z});
}

} // namespace

// brings the content to a usable form
void roadpoint_node::state::normalize()
{
	clearance = std::clamp(clearance, 1.f, 100.f);
	warning = std::clamp(warning, 50.f, 5000.f);
	interval = std::clamp(interval, 1.f, 3600.f);
	variation = std::clamp(variation, 0.f, 1.f);
	velocity = std::clamp(velocity, 5.f, 200.f);
	count = std::clamp(count, 1, 64);
	radius = std::clamp(radius, 0.5f, 50.f);
	length = std::clamp(length, 1.f, 20.f);
}

roadpoint_node::roadpoint_node(scene::node_data const &Nodedata) : basic_node(Nodedata)
{
	m_state.kind = (Nodedata.type == "spawn" ? kind_type::spawn : Nodedata.type == "despawn" ? kind_type::despawn : Nodedata.type == "crosswalk" ? kind_type::crosswalk : kind_type::crossing);
}

// scenery keyword for specified kind of point
std::string roadpoint_node::keyword(kind_type const Kind)
{
	return Kind == kind_type::spawn ? "spawn" : Kind == kind_type::despawn ? "despawn" : Kind == kind_type::crosswalk ? "crosswalk" : "crossing";
}

bool roadpoint_node::is_keyword(std::string const &Type)
{
	return Type == "crossing" || Type == "spawn" || Type == "despawn" || Type == "crosswalk";
}

// reads vehicles out of a text
std::vector<roadpoint_node::vehicle_data> roadpoint_node::parse_vehicles(std::string const &Text)
{
	std::vector<vehicle_data> result;
	auto const add = [&result](std::vector<std::string> const &Tokens, std::size_t const First, std::string const &Load, std::string const &Loadtype) {
		vehicle_data vehicle;
		vehicle.folder = Tokens[First];
		vehicle.skin = Tokens[First + 1];
		vehicle.type = Tokens[First + 2];
		replace_slashes(vehicle.folder);
		replace_slashes(vehicle.skin);
		replace_slashes(vehicle.type);
		vehicle.load = std::max(0, std::atoi(Load.c_str()));
		if (vehicle.load > 0)
		{
			vehicle.loadtype = Loadtype;
		}
		result.emplace_back(vehicle);
	};
	std::vector<std::string> tokens;
	{
		std::istringstream stream{Text};
		std::string token;
		while (stream >> token)
		{
			tokens.emplace_back(token);
		}
	}
	if (std::find(tokens.begin(), tokens.end(), "dynamic") != tokens.end())
	{
		// node <max> <min> <name> dynamic <folder> <skin> <type> <path> <offset> <driver> <velocity> <load> [<load type>] enddynamic
		for (std::size_t idx = 0; idx < tokens.size(); ++idx)
		{
			if (tokens[idx] != "dynamic" || idx + 3 >= tokens.size())
			{
				continue;
			}
			auto last{idx + 4};
			while (last < tokens.size() && tokens[last] != "enddynamic")
			{
				++last;
			}
			// past the vehicle itself come the path, the offset, the driver and the velocity, then the load
			auto const load{idx + 8};
			add(tokens, idx + 1, load < last ? tokens[load] : std::string{}, load + 1 < last ? tokens[load + 1] : std::string{});
			idx = last;
		}
		return result;
	}
	std::istringstream lines{Text};
	std::string line;
	while (std::getline(lines, line))
	{
		std::istringstream stream{line};
		std::vector<std::string> parts;
		std::string part;
		while (stream >> part)
		{
			parts.emplace_back(part);
		}
		if (parts.size() >= 3)
		{
			add(parts, 0, parts.size() > 3 ? parts[3] : std::string{}, parts.size() > 4 ? parts[4] : std::string{});
		}
	}
	return result;
}

// restores content of the node from provided input stream; reads up to and including the closing statement
void roadpoint_node::import(cParser &Input, glm::dvec3 const &Offset)
{
	auto const type{keyword(m_state.kind)};
	auto const label{type + " \"" + m_name + "\""};
	auto const kind{m_state.kind};
	m_state = state{};
	m_state.kind = kind;
	Input.getTokens(3);
	Input >> m_state.position.x >> m_state.position.y >> m_state.position.z;
	m_state.position += Offset;
	int copies{0}; // of each vehicle, the way the first sceneries with spawn points gave the size of the set
	auto counted{false};

	auto token{Input.getToken<std::string>()};
	while (false == token.empty() && token != "end" + type)
	{
		if (token == "clearance")
		{
			Input.getTokens();
			Input >> m_state.clearance;
		}
		else if (token == "warning")
		{
			Input.getTokens();
			Input >> m_state.warning;
		}
		else if (token == "stoplines")
		{
			m_state.stoplines = (Input.getToken<std::string>() != "no");
		}
		else if (token == "interval")
		{
			Input.getTokens();
			Input >> m_state.interval;
		}
		else if (token == "variation")
		{
			Input.getTokens();
			Input >> m_state.variation;
		}
		else if (token == "velocity")
		{
			Input.getTokens();
			Input >> m_state.velocity;
		}
		else if (token == "count")
		{
			Input.getTokens();
			Input >> m_state.count;
			counted = true;
		}
		else if (token == "copies")
		{
			Input.getTokens();
			Input >> copies;
		}
		else if (token == "length")
		{
			Input.getTokens();
			Input >> m_state.length;
		}
		else if (token == "radius")
		{
			Input.getTokens();
			Input >> m_state.radius;
		}
		else if (token == "node")
		{
			// a vehicle, given the way vehicles are put in a scenery:
			// node <max> <min> <name> dynamic <folder> <skin> <type> <path> <offset> <driver> <velocity> <load> [<load type>] enddynamic
			std::string discard;
			std::string nodetype;
			Input.getTokens(4);
			Input >> discard >> discard >> discard >> nodetype;
			vehicle_data vehicle;
			if (nodetype == "dynamic")
			{
				vehicle.folder = Input.getToken<std::string>();
				vehicle.skin = Input.getToken<std::string>();
				vehicle.type = Input.getToken<std::string>();
				replace_slashes(vehicle.folder);
				replace_slashes(vehicle.skin);
				replace_slashes(vehicle.type);
			}
			else
			{
				ErrorLog("Bad road point: " + label + " lists a \"" + nodetype + "\" where it takes vehicles only");
			}
			// where the vehicle is put and how it's driven is up to the point, what's left to take is its load
			std::vector<std::string> rest;
			auto part{Input.getToken<std::string>()};
			while (false == part.empty() && part != "enddynamic" && part != "end" + type)
			{
				rest.emplace_back(part);
				part = Input.getToken<std::string>();
			}
			if (rest.size() > 4)
			{
				vehicle.load = std::atoi(rest[4].c_str());
			}
			if (rest.size() > 5 && vehicle.load > 0)
			{
				vehicle.loadtype = rest[5];
			}
			if (false == vehicle.type.empty())
			{
				m_state.vehicles.emplace_back(vehicle);
			}
			if (part != "enddynamic")
			{
				ErrorLog("Bad road point: a vehicle of " + label + " isn't closed with \"enddynamic\"");
				if (part == "end" + type)
				{
					break;
				}
			}
		}
		else
		{
			ErrorLog("Bad road point: unknown property: \"" + token + "\" defined for " + label);
		}
		token = Input.getToken<std::string>();
	}
	if (false == counted && copies > 0)
	{
		m_state.count = copies * static_cast<int>(std::max<std::size_t>(1, m_state.vehicles.size()));
	}
	m_state.normalize();
	location(m_state.position);
}

// replaces what the node knows about itself
void roadpoint_node::define(state const &State)
{
	auto const vehicles{m_state.vehicles};
	auto const count{m_state.count};
	m_state = State;
	m_state.normalize();
	if (m_prepared && (m_state.vehicles != vehicles || m_state.count != count))
	{
		// vehicles can't be unmade, there's too much holding on to them. the old ones are left alone:
		// those which wait stay where they are, those on the road drive on until something takes them away
		m_pool.clear();
		m_prepared = false;
	}
	location(m_state.position);
	m_area.radius = -1.f;
}

// ties the point to the lanes and the rails as they are at the moment
void roadpoint_node::bind()
{
	unbind();
	m_bound = true;
	if (m_editorremoved)
	{
		return;
	}
	switch (m_state.kind)
	{
	case kind_type::crossing:
	{
		bind_crossing();
		break;
	}
	case kind_type::spawn:
	{
		// the vehicles are put on the lane the point is on
		double station{0.0};
		auto *lane{nearest_lane(m_state.position, 3.0, station)};
		if (lane != nullptr)
		{
			m_lanes.emplace_back(lane);
			// at the very start of a lane which leads from nowhere the vehicle would hang over its end
			m_offset = (lane->trPrev == nullptr ? std::max(station, 2.0 * front_room) : station);
		}
		break;
	}
	case kind_type::crosswalk:
	{
		// the stripes go across the road the point is on
		double t{0.0};
		auto const *road{road_under(m_state.position, t)};
		if (road != nullptr)
		{
			auto const &layout{road->definition()};
			path_curve const axis{layout.axis};
			m_onroad = true;
			m_along = axis.direction(t);
			m_halfwidth = 0.5 * layout.span(t);
			// the point itself may be off the axis, the stripes aren't
			auto const centre{axis.point(t)};
			m_state.position = centre;
		}
		break;
	}
	case kind_type::despawn:
	{
		// the vehicles are taken from every lane which passes close enough
		for (auto const *road : simulation::Roads.sequence())
		{
			if (road == nullptr || road->m_editorremoved)
			{
				continue;
			}
			for (auto *lane : road->tracks())
			{
				if (lane == nullptr || lane->m_paths.empty() || out_of_reach(lane->m_paths.front(), m_state.position, m_state.radius))
				{
					continue;
				}
				path_curve const curve{lane->m_paths.front()};
				double distance{0.0};
				auto const t{curve.nearest(m_state.position, distance)};
				if (distance <= m_state.radius && std::abs(curve.point(t).y - m_state.position.y) <= 3.0)
				{
					m_lanes.emplace_back(lane);
				}
			}
		}
		break;
	}
	}
}

// finds the rails of a crossing, and the places the vehicles of each lane are to stop at
void roadpoint_node::bind_crossing()
{
	// the road the crossing is on: the one whose axis passes nearest
	double where{0.0};
	auto const *crossed{road_under(m_state.position, where)};
	if (crossed != nullptr)
	{
		auto const &lanes{crossed->tracks()};
		for (std::size_t idx = 0; idx < lanes.size(); ++idx)
		{
			auto *lane{lanes[idx]};
			if (lane == nullptr || lane->m_paths.empty())
			{
				continue;
			}
			// the lanes are laid out the way the traffic goes, so the place to stop at is back along the lane from the crossing
			double distance{0.0};
			path_curve curve{lane->m_paths.front()};
			auto station{curve.station(curve.nearest(m_state.position, distance))};
			double remaining{m_state.clearance};
			for (int guard = 0; guard < 16; ++guard)
			{
				if (station >= remaining)
				{
					station -= remaining;
					break;
				}
				// it's on the piece before this one, if there's any: the same lane of the road, not a way through a junction
				auto *previous{lane->trPrev};
				auto const *owner{previous != nullptr ? dynamic_cast<road_node const *>(previous->m_road) : nullptr};
				if (owner == nullptr || previous->m_paths.empty() || previous->trNext != lane || std::find(owner->tracks().begin(), owner->tracks().end(), previous) == owner->tracks().end())
				{
					station = 0.0;
					break;
				}
				remaining -= station;
				lane = previous;
				curve = path_curve{lane->m_paths.front()};
				station = curve.length();
			}
			auto const t{curve.parameter(station)};
			stop_data stop;
			stop.track = lane;
			stop.position = curve.point(t);
			stop.direction = curve.direction(t);
			stop.width = (idx < crossed->definition().lanes.size() ? crossed->definition().lanes[idx].width : crossed->definition().lanewidth);
			m_stops.emplace_back(stop);
		}
	}
	// orders for the drivers. the ones made before are used again, as a driver may still hold on to one
	while (m_events.size() < m_stops.size())
	{
		auto *event{new stop_event()};
		event->m_name = m_name + ":stop" + std::to_string(m_events.size() + 1);
		m_events.emplace_back(event);
	}
	for (std::size_t idx = 0; idx < m_stops.size(); ++idx)
	{
		auto *event{static_cast<stop_event *>(m_events[idx])};
		event->m_location = m_stops[idx].position;
		// lanes are driven from their first point to the second, which is the way events of the second kind are for
		m_stops[idx].track->m_events2.emplace_back(event->m_name, event);
	}

	// rails: the tracks passing between the stop lines, and what leads to them within the warning distance
	std::vector<std::pair<TTrack *, double>> reached;
	auto const known = [&reached](TTrack const *Track) { return std::any_of(reached.begin(), reached.end(), [Track](std::pair<TTrack *, double> const &Entry) { return Entry.first == Track; }); };
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 1) == 0)
		{
			continue;
		}
		for (auto const &path : track->m_paths)
		{
			if (out_of_reach(path, m_state.position, m_state.clearance))
			{
				continue;
			}
			path_curve const curve{path};
			double distance{0.0};
			auto const t{curve.nearest(m_state.position, distance)};
			if (distance <= m_state.clearance && std::abs(curve.point(t).y - m_state.position.y) <= 3.0)
			{
				reached.emplace_back(track, 0.0);
				break;
			}
		}
	}
	for (std::size_t idx = 0; idx < reached.size(); ++idx)
	{
		auto *track{reached[idx].first};
		double length{0.0};
		for (auto const &path : track->m_paths)
		{
			length = std::max(length, path_curve{path}.length());
		}
		auto const travelled{reached[idx].second + length};
		if (travelled > m_state.warning)
		{
			continue;
		}
		std::vector<TTrack *> neighbours{track->trPrev, track->trNext};
		if (track->SwitchExtension != nullptr)
		{
			// both ways of a switch count, as the train can come by either
			neighbours.insert(neighbours.end(), {track->SwitchExtension->pPrevs[0], track->SwitchExtension->pPrevs[1], track->SwitchExtension->pNexts[0], track->SwitchExtension->pNexts[1]});
		}
		for (auto *neighbour : neighbours)
		{
			if (neighbour != nullptr && (neighbour->iCategoryFlag & 1) != 0 && false == known(neighbour))
			{
				reached.emplace_back(neighbour, travelled);
			}
		}
	}
	for (auto const &entry : reached)
	{
		m_rails.emplace_back(entry.first);
	}
	// lanes the vehicles wait on while the crossing is closed: the ones with the stops, and what leads to them
	for (auto const &stop : m_stops)
	{
		auto *lane{stop.track};
		double reach{0.0};
		for (int guard = 0; lane != nullptr && guard < 64 && reach < queue_reach; ++guard)
		{
			if (std::find(m_queue.begin(), m_queue.end(), lane) == m_queue.end())
			{
				m_queue.emplace_back(lane);
			}
			if (false == lane->m_paths.empty())
			{
				reach += glm::distance(lane->m_paths.front().points[segment_data::point::start], lane->m_paths.front().points[segment_data::point::end]);
			}
			lane = lane->trPrev;
		}
	}
	close(m_closed);
}

// lets go of the lanes and the rails
void roadpoint_node::unbind()
{
	if (false == m_bound)
	{
		return;
	}
	m_bound = false;
	for (std::size_t idx = 0; idx < m_stops.size() && idx < m_events.size(); ++idx)
	{
		auto &events{m_stops[idx].track->m_events2};
		auto const *event{m_events[idx]};
		events.erase(std::remove_if(events.begin(), events.end(), [event](std::pair<std::string, basic_event *> const &Entry) { return Entry.second == event; }), events.end());
	}
	// a driver who has one of the orders in sight keeps it; it's not to hold anyone anymore
	for (auto *event : m_events)
	{
		static_cast<stop_event *>(event)->m_velocity = -1.0;
	}
	m_stops.clear();
	m_rails.clear();
	m_queue.clear();
	m_lanes.clear();
	m_seen.clear();
	m_onroad = false;
}

// makes the vehicles of a spawn point
void roadpoint_node::prepare()
{
	if (m_prepared || m_state.kind != kind_type::spawn || m_editorremoved)
	{
		return;
	}
	if (false == m_bound)
	{
		bind();
	}
	if (m_lanes.empty())
	{
		ErrorLog("Bad road point: spawn \"" + m_name + "\" isn't on a lane of a road, there's nowhere to put its vehicles");
		return;
	}
	m_prepared = true;
	// the first vehicle shows up after a part of the interval, so points with the same interval don't all start at once
	m_timer = LocalRandom(0.0, m_state.interval);
	// vehicles are known by their names, so a point without one makes something up for them
	static int nameless{0};
	auto const basename{m_name.empty() ? "roadspawn" + std::to_string(++nameless) : m_name};
	// the listed vehicles are gone through as many times as it takes, from a random one on, so the set gets each kind in turn
	auto const kinds{m_state.vehicles.size()};
	auto const start{kinds > 0 ? std::min(kinds - 1, static_cast<std::size_t>(LocalRandom(0.0, static_cast<double>(kinds)))) : 0};
	for (int index = 0; index < m_state.count && kinds > 0; ++index)
	{
		auto const &definition{m_state.vehicles[(start + static_cast<std::size_t>(index)) % kinds]};
		// a vehicle like any other of the scenery, made where the point is and driven at the speed of the point...
		auto *vehicle{new TDynamicObject()};
		auto const length{vehicle->Init(basename + ":" + std::to_string(++m_made), definition.folder, definition.skin, definition.type, m_lanes.front(), m_offset + front_room, "headdriver",
		                                m_state.velocity, "", static_cast<float>(definition.load), definition.loadtype, false, "")};
		if (length == 0.0)
		{
			ErrorLog("Bad road point: vehicle \"" + definition.folder + "/" + definition.type + "\" of spawn \"" + m_name + "\" couldn't be made");
			if (vehicle->MyTrack != nullptr)
			{
				vehicle->MyTrack->RemoveDynamicObject(vehicle);
			}
			delete vehicle;
			continue;
		}
		if (vehicle->mdModel != nullptr)
		{
			for (auto const &smokesource : vehicle->mdModel->smoke_sources())
			{
				simulation::Particles.insert(smokesource.first, vehicle, smokesource.second);
			}
		}
		if (false == simulation::Vehicles.insert(vehicle))
		{
			ErrorLog("Bad road point: vehicle name \"" + vehicle->name() + "\" made for spawn \"" + m_name + "\" is taken already");
		}
		pooled_vehicle entry;
		entry.vehicle = vehicle;
		entry.velocity = vehicle->MoverParameters->V;
		m_pool.emplace_back(entry);
		// ...which is taken off the road right away, the way the vehicles leaving the scenery are, to wait for its turn
		if (vehicle->MyTrack != nullptr)
		{
			vehicle->MyTrack->RemoveDynamicObject(vehicle);
			vehicle->MyTrack = nullptr;
		}
		vehicle->bEnabled = false;
	}
	if (m_pool.empty())
	{
		ErrorLog("Bad road point: spawn \"" + m_name + "\" has no vehicles to put on the road");
	}
}

// true if specified vehicle stands in the line of vehicles held by the crossing
bool roadpoint_node::holds(TDynamicObject const &Vehicle) const
{
	return m_state.kind == kind_type::crossing && m_closed && false == m_editorremoved && std::find(m_queue.begin(), m_queue.end(), Vehicle.MyTrack) != m_queue.end();
}

std::size_t roadpoint_node::waiting() const
{
	return static_cast<std::size_t>(std::count_if(m_pool.begin(), m_pool.end(), [](pooled_vehicle const &Entry) { return false == Entry.vehicle->bEnabled && Entry.vehicle->MyTrack == nullptr; }));
}

// to be called with each step of the simulation
void roadpoint_node::update(double const Deltatime)
{
	if (m_editorremoved)
	{
		return;
	}
	if (false == m_bound)
	{
		bind();
	}
	switch (m_state.kind)
	{
	case kind_type::crossing:
	{
		update_crossing(Deltatime);
		break;
	}
	case kind_type::spawn:
	{
		update_spawn(Deltatime);
		break;
	}
	case kind_type::despawn:
	{
		update_despawn();
		break;
	}
	case kind_type::crosswalk:
	{
		// paint only
		break;
	}
	}
}

// closes the crossing while there's a rail vehicle coming, or standing right at it
void roadpoint_node::update_crossing(double const Deltatime)
{
	m_scan -= Deltatime;
	m_hold -= Deltatime;
	if (m_scan <= 0.0)
	{
		m_scan = scan_period;
		std::map<TDynamicObject const *, double> seen;
		for (auto const *track : m_rails)
		{
			for (auto const *vehicle : track->Dynamics)
			{
				if (vehicle == nullptr)
				{
					continue;
				}
				auto const distance{planar_distance(vehicle->GetPosition(), m_state.position)};
				seen.emplace(vehicle, distance);
				auto const before{m_seen.find(vehicle)};
				// a vehicle getting closer is on its way here; one which stands still or goes away isn't, unless it's at the crossing itself
				if (distance < m_state.clearance + near_zone || (before != m_seen.end() && distance < before->second - 0.01))
				{
					m_hold = hold_time;
				}
			}
		}
		m_seen.swap(seen);
	}
	if ((m_hold > 0.0) != m_closed)
	{
		close(m_hold > 0.0);
	}
}

// tells the drivers whether they can go
void roadpoint_node::close(bool const Closed)
{
	m_closed = Closed;
	for (std::size_t idx = 0; idx < m_stops.size() && idx < m_events.size(); ++idx)
	{
		static_cast<stop_event *>(m_events[idx])->m_velocity = (Closed ? 0.0 : -1.0);
	}
}

// puts a waiting vehicle on the road when its time comes
void roadpoint_node::update_spawn(double const Deltatime)
{
	if (false == m_prepared)
	{
		// the point was made in the editor, or the scenery was loaded for editing
		prepare();
	}
	// vehicles of the point which got stuck are taken off the road, to wait for their next turn: the ones standing still
	// for too long, unless it's a closed level crossing they wait at, and the ones which started to go backwards
	for (auto &entry : m_pool)
	{
		auto *vehicle{entry.vehicle};
		if (false == vehicle->bEnabled || vehicle->MyTrack == nullptr || (simulation::Train != nullptr && simulation::Train->Dynamic() == vehicle))
		{
			// not on the road, or taken over by the user
			entry.standing = 0.0;
			continue;
		}
		auto const speed{vehicle->MoverParameters->V};
		// forward is the way it went when it was put on the road
		auto const backwards{speed * entry.velocity < 0.0 && std::abs(speed) > reverse_speed};
		if (std::abs(speed) > stuck_speed || simulation::Roadpoints.holds(*vehicle))
		{
			entry.standing = 0.0;
		}
		else
		{
			entry.standing += Deltatime;
		}
		if (backwards || entry.standing > stuck_time)
		{
			WriteLog("Road traffic: vehicle \"" + vehicle->name() + "\" taken off the road, " + (backwards ? "it was going backwards" : "it stood still for too long"));
			entry.standing = 0.0;
			vehicle->bEnabled = false;
			TDynamicObject::bDynamicRemove = true;
		}
	}
	if (m_pool.empty() || m_lanes.empty())
	{
		return;
	}
	m_timer -= Deltatime;
	if (m_timer > 0.0)
	{
		return;
	}
	// not with something in the way
	auto const *lane{m_lanes.front()};
	for (auto const *track : {lane, static_cast<TTrack const *>(lane->trPrev), static_cast<TTrack const *>(lane->trNext)})
	{
		if (track == nullptr)
		{
			continue;
		}
		for (auto const *other : track->Dynamics)
		{
			if (other != nullptr && planar_distance(other->GetPosition(), m_state.position) < clear_zone)
			{
				m_timer = retry_period;
				return;
			}
		}
	}
	// one of the vehicles which wait, picked at random
	std::vector<pooled_vehicle const *> ready;
	for (auto const &entry : m_pool)
	{
		if (false == entry.vehicle->bEnabled && entry.vehicle->MyTrack == nullptr)
		{
			ready.emplace_back(&entry);
		}
	}
	if (ready.empty())
	{
		m_timer = retry_period;
		return;
	}
	auto const &entry{*ready[std::min(ready.size() - 1, static_cast<std::size_t>(LocalRandom(0.0, static_cast<double>(ready.size()))))]};
	auto *vehicle{entry.vehicle};
	// it comes back the way it was made: whole, at speed, with the driver told to drive on
	vehicle->bEnabled = true;
	vehicle->MoverParameters->DamageFlag = 0;
	vehicle->MoverParameters->EngDmgFlag = 0;
	vehicle->MoverParameters->V = entry.velocity;
	vehicle->place_on_track(m_lanes.front(), m_offset + 0.5 * vehicle->MoverParameters->Dim.L, false);
	if (vehicle->Mechanik != nullptr)
	{
		// the same order a vehicle put in the scenery gets; it has the driver forget the road they knew
		vehicle->Mechanik->PutCommand("Timetable:", m_state.velocity, 0, nullptr);
	}
	m_timer = m_state.interval * (1.0 + m_state.variation * LocalRandom(-1.0, 1.0));
}

// takes away the road vehicles which got to the point
void roadpoint_node::update_despawn()
{
	for (auto const *lane : m_lanes)
	{
		for (auto *vehicle : lane->Dynamics)
		{
			if (vehicle == nullptr || false == vehicle->bEnabled || vehicle->MoverParameters->CategoryFlag != 2)
			{
				continue;
			}
			if (simulation::Train != nullptr && simulation::Train->Dynamic() == vehicle)
			{
				// not the one the user sits in
				continue;
			}
			if (planar_distance(vehicle->GetPosition(), m_state.position) > m_state.radius + 0.5 * vehicle->MoverParameters->Dim.L)
			{
				continue;
			}
			// the way vehicles reaching the end of the scenery go: marked here, taken off the road once all vehicles made their move
			vehicle->bEnabled = false;
			TDynamicObject::bDynamicRemove = true;
		}
	}
}

// generates geometry of the stop lines
std::vector<scene::shape_node> roadpoint_node::create_shapes() const
{
	std::vector<scene::shape_node> shapes;
	std::vector<world_vertex> vertices;
	glm::dvec3 const lift{0.0, line_lift, 0.0};
	glm::vec3 const upwards{0.f, 1.f, 0.f};
	if (m_state.kind == kind_type::crosswalk && m_onroad)
	{
		// stripes laid along the road, side by side from one edge of it to the other
		glm::dvec3 across{m_along.z, 0.0, -m_along.x};
		if (glm::length2(across) > 1e-12)
		{
			across = glm::normalize(across);
			auto const reach{m_halfwidth - stopline_inset};
			auto const stripes{std::max(1, static_cast<int>(std::floor((2.0 * reach + stripe_width) / (2.0 * stripe_width))))};
			// the stripes are spread evenly over the width, with one at each edge
			auto const first{-0.5 * (stripes * 2 - 1) * stripe_width};
			auto const back{-m_along * static_cast<double>(m_state.length)};
			for (int stripe = 0; stripe < stripes; ++stripe)
			{
				auto const offset{first + stripe * 2.0 * stripe_width};
				auto const right{m_state.position + across * offset + m_along * (0.5 * m_state.length) + lift};
				auto const left{right + across * stripe_width};
				vertices.push_back({right + back, upwards, {0.f, 0.f}});
				vertices.push_back({right, upwards, {1.f, 0.f}});
				vertices.push_back({left + back, upwards, {0.f, 1.f}});
				vertices.push_back({right, upwards, {1.f, 0.f}});
				vertices.push_back({left, upwards, {1.f, 1.f}});
				vertices.push_back({left + back, upwards, {0.f, 1.f}});
			}
		}
	}
	for (auto const &stop : m_stops)
	{
		if (m_state.kind != kind_type::crossing || false == m_state.stoplines)
		{
			break;
		}
		// a line across the lane, ending where the vehicles are to stop
		glm::dvec3 across{stop.direction.z, 0.0, -stop.direction.x};
		if (glm::length2(across) < 1e-12)
		{
			continue;
		}
		across = glm::normalize(across);
		glm::dvec3 back{-stop.direction.x, 0.0, -stop.direction.z};
		back = glm::normalize(back) * stopline_width;
		auto const reach{std::max(0.25, 0.5 * stop.width - stopline_inset)};
		auto const right{stop.position - across * reach + lift};
		auto const left{stop.position + across * reach + lift};
		vertices.push_back({right + back, upwards, {0.f, 0.f}});
		vertices.push_back({right, upwards, {1.f, 0.f}});
		vertices.push_back({left + back, upwards, {0.f, 1.f}});
		vertices.push_back({right, upwards, {1.f, 0.f}});
		vertices.push_back({left, upwards, {1.f, 1.f}});
		vertices.push_back({left + back, upwards, {0.f, 1.f}});
	}
	if (false == vertices.empty())
	{
		lighting_data paint;
		paint.diffuse = glm::vec4{0.92f, 0.92f, 0.92f, 1.f};
		paint.ambient = paint.diffuse;
		scene::shape_node shape;
		shape.make_terrain(GfxRenderer->Fetch_Material("colored"), std::move(vertices), glm::dvec3{0.0});
		shape.lighting(paint);
		shapes.emplace_back(std::move(shape));
	}
	return shapes;
}

// puts geometry of the point in the scene as shapes of its own, which can be taken back
void roadpoint_node::show()
{
	if (m_shapes.shown() || m_editorremoved || simulation::Region == nullptr || Global.NvRenderer)
	{
		// NOTE: the experimental renderer draws only what the scenery had when it was loaded
		return;
	}
	m_shapes.show(create_shapes(), m_state.position);
}

void roadpoint_node::hide()
{
	m_shapes.hide();
}

// radius() subclass details, calculates node's bounding radius
float roadpoint_node::radius_()
{
	return m_state.kind == kind_type::crossing ? m_state.clearance + 5.f : m_state.kind == kind_type::crosswalk ? static_cast<float>(m_halfwidth) + m_state.length : m_state.radius;
}

// serialize() subclass details, sends content of the subclass to provided stream
void roadpoint_node::serialize_(std::ostream &Output) const
{
	// TODO: implement
}

// deserialize() subclass details, restores content of the subclass from provided stream
void roadpoint_node::deserialize_(std::istream &Input)
{
	// TODO: implement
}

// export() subclass details, sends basic content of the class in legacy (text) format to provided stream
void roadpoint_node::export_as_text_(std::ostream &Output) const
{
	auto const type{keyword(m_state.kind)};
	Output << type << ' ';
	auto const precision{Output.precision(std::numeric_limits<double>::digits10)};
	Output << m_state.position.x << ' ' << m_state.position.y << ' ' << m_state.position.z << ' ';
	Output.precision(precision);
	switch (m_state.kind)
	{
	case kind_type::crossing:
	{
		Output << "clearance " << m_state.clearance << ' ' << "warning " << m_state.warning << ' ' << "stoplines " << (m_state.stoplines ? "yes" : "no") << ' ';
		break;
	}
	case kind_type::spawn:
	{
		Output << "interval " << m_state.interval << ' ' << "variation " << m_state.variation << ' ' << "velocity " << m_state.velocity << ' ' << "count " << m_state.count << ' ';
		for (auto const &vehicle : m_state.vehicles)
		{
			// the way vehicles are put in a scenery. the path and the offset are there to keep that form, the point doesn't use them
			Output << "node -1 0 none dynamic " << vehicle.folder << ' ' << vehicle.skin << ' ' << vehicle.type << " none 0 headdriver " << m_state.velocity << ' ' << vehicle.load << ' ';
			if (vehicle.load > 0 && false == vehicle.loadtype.empty())
			{
				Output << vehicle.loadtype << ' ';
			}
			Output << "enddynamic ";
		}
		break;
	}
	case kind_type::despawn:
	{
		Output << "radius " << m_state.radius << ' ';
		break;
	}
	case kind_type::crosswalk:
	{
		Output << "length " << m_state.length << ' ';
		break;
	}
	}
	Output << "end" << type << "\n";
}

// legacy style initialization, to be performed when the lanes of the roads are tied together
void roadpoint_table::InitRoadpoints()
{
	for (auto *point : m_items)
	{
		if (point == nullptr)
		{
			continue;
		}
		point->bind();
		if (false == Global.editor_session)
		{
			// the vehicles are made now, so they don't hold the simulation up later. a scenery opened for editing
			// does without them until the simulation is run
			point->prepare();
		}
	}
}

// lets go of the lanes and the rails, ahead of a change of the roads
void roadpoint_table::unbind()
{
	for (auto *point : m_items)
	{
		if (point != nullptr)
		{
			point->unbind();
		}
	}
}

// ties all points anew, after the roads were changed
void roadpoint_table::bind()
{
	for (auto *point : m_items)
	{
		if (point == nullptr)
		{
			continue;
		}
		// the stop lines go where the lanes are now
		point->hide();
		point->bind();
		if (m_geometry)
		{
			point->show();
		}
	}
}

// puts geometry of the points in the scene
void roadpoint_table::create_geometry()
{
	m_geometry = true;
	for (auto *point : m_items)
	{
		if (point != nullptr)
		{
			point->show();
		}
	}
}

// true if specified vehicle stands in the line of vehicles held by a closed level crossing
bool roadpoint_table::holds(TDynamicObject const &Vehicle) const
{
	return std::any_of(m_items.begin(), m_items.end(), [&Vehicle](roadpoint_node const *Point) { return Point != nullptr && Point->holds(Vehicle); });
}

// to be called with each step of the simulation
void roadpoint_table::update(double const Deltatime)
{
	for (auto *point : m_items)
	{
		if (point != nullptr)
		{
			point->update(Deltatime);
		}
	}
}

//---------------------------------------------------------------------------
