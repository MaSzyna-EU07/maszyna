/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorStation.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorKilometrage.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorTrack.hpp"
#include "simulation/simulation.h"
#include "world/Track.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace station
{

using geometry::bezier;
using geometry::plan_distance;
using geometry::same_point;

namespace
{

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == 1 && false == Track.m_editorremoved && false == Track.m_paths.empty();
}

// switches, crossings and turntables: where the plain track between them ends
bool node(TTrack const &Track)
{
	return Track.eType != tt_Normal;
}

glm::dvec3 end_of(TTrack const &Track, int const Path, bool const Atend)
{
	return Track.m_paths[Path].points[Atend ? segment_data::point::end : segment_data::point::start];
}

TTrack *neighbour(TTrack const &Track, int const Path, bool const Atend)
{
	if (Track.SwitchExtension != nullptr && (Track.eType == tt_Switch || Track.eType == tt_Cross))
		return Atend ? Track.SwitchExtension->pNexts[Path & 1] : Track.SwitchExtension->pPrevs[Path & 1]; // NOSONAR
	return Atend ? Track.trNext : Track.trPrev;
}

// the end of the track at the point
bool end_at(TTrack const &Track, glm::dvec3 const &Point, int &Path, bool &Atend)
{
	for (int path = 0; path < static_cast<int>(Track.m_paths.size()); ++path)
		for (auto const atend : {false, true})
			if (glm::distance(end_of(Track, path, atend), Point) < same_point)
			{
				Path = path;
				Atend = atend;
				return true;
			}
	return false;
}

// the ends of the paths of a switch, each place once: its start, the ends of its paths
std::vector<std::pair<int, bool>> ends_of(TTrack const &Track)
{
	std::vector<std::pair<int, bool>> result;
	for (int path = 0; path < static_cast<int>(Track.m_paths.size()); ++path)
		for (auto const atend : {false, true})
		{
			auto const point{end_of(Track, path, atend)};
			if (std::none_of(result.begin(), result.end(), [&](auto const &End) { return glm::distance(end_of(Track, End.first, End.second), point) < same_point; }))
				result.emplace_back(path, atend);
		}
	return result;
}

double plan_length(TTrack const &Track, int const Path)
{
	return bezier{Track.m_paths[Path]}.plan_length(16);
}

// plain track from an end of a switch on, up to the next switch, a free end or past the limit
struct chain
{
	std::vector<TTrack *> tracks;
	double length{0.0};
	TTrack *reached{nullptr}; // the switch met at its other end
	bool ended{false}; // at a free end
	TTrack const *from{nullptr}; // the switch it leads out of, with the end of it
	int path{0};
	bool atend{false};
};

chain follow(TTrack const &From, int const Path, bool const Atend, double const Limit)
{
	chain result;
	result.from = &From;
	result.path = Path;
	result.atend = Atend;
	auto point{end_of(From, Path, Atend)};
	auto *next{neighbour(From, Path, Atend)};
	while (next != nullptr && result.length <= Limit)
	{
		if (false == rail(*next))
			break;
		if (node(*next))
		{
			result.reached = next;
			return result;
		}
		int path{0};
		bool atend{false};
		if (false == end_at(*next, point, path, atend) || std::find(result.tracks.begin(), result.tracks.end(), next) != result.tracks.end())
			break;
		result.tracks.push_back(next);
		result.length += plan_length(*next, 0);
		point = end_of(*next, 0, false == atend);
		next = neighbour(*next, 0, false == atend);
	}
	result.ended = next == nullptr || false == rail(*next);
	return result;
}

// the switches and the plain tracks of the station, and the lines leading out of it
struct area
{
	std::vector<TTrack *> switches;
	std::vector<TTrack *> plain;
	std::vector<chain> exits;
	std::vector<chain> ends; // tracks ending free: sidings, or lines cut off by the edge of the scenery
};

area collect(TTrack &From, options const &Options)
{
	area result;
	if (false == rail(From))
		return result;
	auto *first{node(From) ? &From : nullptr};
	if (first == nullptr)
	{
		auto nearest{Options.link};
		for (auto const atend : {false, true})
			if (auto const line{follow(From, 0, atend, Options.link)}; line.reached != nullptr && line.length < nearest)
			{
				nearest = line.length;
				first = line.reached;
			}
	}
	if (first == nullptr)
		return result;
	std::unordered_set<TTrack const *> known{first};
	result.switches.push_back(first);
	for (std::size_t i = 0; i < result.switches.size(); ++i)
	{
		auto const &from{*result.switches[i]};
		for (auto const &[path, atend] : ends_of(from))
		{
			auto line{follow(from, path, atend, Options.link)};
			if (line.reached == nullptr)
			{
				(line.ended ? result.ends : result.exits).push_back(std::move(line));
				continue;
			}
			for (auto *track : line.tracks)
				if (known.insert(track).second)
					result.plain.push_back(track);
			if (line.reached != nullptr && known.insert(line.reached).second)
				result.switches.push_back(line.reached);
		}
	}
	// a link longer than the limit between two switches of the station is a track of it, not two lines out
	auto exits{std::move(result.exits)};
	result.exits.clear();
	for (auto &line : exits)
	{
		if (false == line.ended)
			if (auto const whole{follow(*line.from, line.path, line.atend, std::numeric_limits<double>::max())}; whole.reached != nullptr && known.count(whole.reached) > 0)
			{
				for (auto *track : whole.tracks)
					if (known.insert(track).second)
						result.plain.push_back(track);
				continue;
			}
		if (std::none_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Track) { return known.count(Track) > 0; }))
			result.exits.push_back(std::move(line));
	}
	return result;
}

std::vector<glm::dvec3> points_of(TTrack const &Track, int const Path, double &Length)
{
	bezier const curve{Track.m_paths[Path]};
	Length = curve.plan_length(16);
	auto const count{std::clamp(static_cast<int>(Length / 2.0), 2, 64)};
	std::vector<glm::dvec3> result;
	for (int i = 0; i <= count; ++i)
		result.push_back(curve.point(static_cast<double>(i) / count));
	return result;
}

// the points of the path a metre or so apart, for the joints to come out the same however the tracks are cut
std::vector<glm::dvec3> dense_points(TTrack const &Track, int const Path, double &Length)
{
	bezier const curve{Track.m_paths[Path]};
	Length = curve.plan_length(64);
	auto const count{std::clamp(static_cast<int>(std::ceil(Length)), 2, 4000)};
	std::vector<glm::dvec3> result;
	for (int i = 0; i <= count; ++i)
		result.push_back(curve.point(static_cast<double>(i) / count));
	return result;
}

// the map shows the east to the right and the north up: the x of the scenery runs to the west
glm::dvec2 map_of(glm::dvec3 const &Point)
{
	return {-Point.x, Point.z};
}

double constexpr kStart{1.5}; // m, a joint before the start of a switch
double constexpr kFouling{3.5}; // m, between the axes at the fouling point of two tracks
double constexpr kBeyond{4.2}; // m, from a fouling point on to the joint beyond it, Ie-4 § 33
double constexpr kBranches{10.0}; // m, the ends of a switch closer together are the ends of its paths at the frog
double constexpr kReach{120.0}; // m, along a branch looked for its fouling point
double constexpr kSection{30.0}; // m, the shortest stretch between two joints made a track circuit of its own
double constexpr kSnap{0.3}; // m, a joint this close to the end of a track lies at the end
double constexpr kThrough{400.0}; // m, of the switches a station track is followed through for its circuits beyond
double constexpr kEntry{300.0}; // m, from the joint of the station out to its entry signal
double constexpr kJoint{3.5}; // m, from a shunting signal to the joint of the circuit it guards
double constexpr kStopping{200.0}; // m, the shortest circuit of a station track a train is sent out of by an exit signal
double constexpr kSignalSide{2.25}; // m, from the axis to a signal
double constexpr kSignalClear{2.5}; // m, the least from a signal to the axis of another track
double constexpr kSignalApart{3.0}; // m, the least between two signals

// the points of the way out of an end of the track, as a train runs: along the main path of a switch run into from its start,
// back along the path of one run into from the end of it
std::vector<glm::dvec3> ahead(TTrack const &From, int const Path, bool const Atend, double const Length)
{
	std::vector<glm::dvec3> result{end_of(From, Path, Atend)};
	auto *next{neighbour(From, Path, Atend)};
	std::unordered_set<TTrack const *> seen{&From};
	double length{0.0};
	while (next != nullptr && length < Length && rail(*next) && seen.insert(next).second)
	{
		int path{0};
		bool atend{false};
		if (false == end_at(*next, result.back(), path, atend))
			break;
		if (next->eType == tt_Switch && false == atend)
			path = 0;
		double plan{0.0};
		auto points{dense_points(*next, path, plan)};
		if (atend)
			std::reverse(points.begin(), points.end());
		result.insert(result.end(), points.begin() + 1, points.end());
		length += plan;
		auto const *track{next};
		next = neighbour(*track, path, false == atend);
	}
	return result;
}

// the path of a switch a train run into from its start goes on along: the one turning the least
int straighter(TTrack const &Switch)
{
	if (Switch.eType != tt_Switch || Switch.m_paths.size() < 2)
		return 0;
	auto const turn = [&](int const Path) {
		bezier const curve{Switch.m_paths[Path]};
		auto const from{curve.first(0.0)};
		auto const to{curve.first(1.0)};
		return std::abs(std::atan2(from.x * to.z - from.z * to.x, from.x * to.x + from.z * to.z));
	};
	return turn(1) < turn(0) ? 1 : 0;
}

// the tracks of the run from the end of the track at the point on, through the switches along their straighter paths
std::vector<TTrack *> straight_run(TTrack &Track, glm::dvec3 Point, double const Limit)
{
	std::vector<TTrack *> result{&Track};
	std::unordered_set<TTrack const *> seen{&Track};
	int path{0};
	bool atend{false};
	if (false == end_at(Track, Point, path, atend))
		return result;
	auto *next{neighbour(Track, path, atend)};
	double length{0.0};
	while (next != nullptr && length < Limit && rail(*next) && seen.insert(next).second)
	{
		if (false == end_at(*next, Point, path, atend))
			break;
		if (next->eType == tt_Switch && false == atend)
			path = straighter(*next);
		result.push_back(next);
		length += plan_length(*next, path);
		Point = end_of(*next, path, false == atend);
		auto const *track{next};
		next = neighbour(*track, path, false == atend);
	}
	return result;
}

double plan_gap(glm::dvec3 const &Point, std::vector<glm::dvec3> const &Line)
{
	glm::dvec2 const point{Point.x, Point.z};
	auto best{glm::distance(point, glm::dvec2{Line.front().x, Line.front().z})};
	for (std::size_t i = 1; i < Line.size(); ++i)
	{
		glm::dvec2 const a{Line[i - 1].x, Line[i - 1].z};
		glm::dvec2 const along{glm::dvec2{Line[i].x, Line[i].z} - a};
		auto const length2{glm::dot(along, along)};
		auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, along) / length2, 0.0, 1.0) : 0.0};
		best = std::min(best, glm::distance(point, a + along * t));
	}
	return best;
}

// how far out from the end of the switch along the track leading on its fouling point lies, below 0 within the switch: where the
// axes of the path there and of the other one are 3.5 m apart; none at the start of a switch
std::optional<double> fouling(TTrack const &Switch, int const Path, bool const Atend)
{
	auto const point{end_of(Switch, Path, Atend)};
	std::optional<std::pair<int, bool>> other;
	auto nearest{kBranches};
	for (auto const &[path, atend] : ends_of(Switch))
		if (auto const distance{plan_distance(end_of(Switch, path, atend), point)}; distance > same_point && distance < nearest)
		{
			nearest = distance;
			other = std::make_pair(path, atend);
		}
	if (false == other.has_value())
		return std::nullopt;
	auto const theirs{ahead(Switch, other->first, other->second, kReach + 50.0)};
	if (plan_gap(point, theirs) >= kFouling)
	{
		// the fouling point within the switch: back along its paths to it
		double length{0.0};
		auto mine{dense_points(Switch, Path, length)};
		auto const own{dense_points(Switch, other->first, length)};
		if (Atend)
			std::reverse(mine.begin(), mine.end());
		double back{0.0};
		for (std::size_t i = 1; i < mine.size() && plan_gap(mine[i], own) >= kFouling; ++i)
			back += plan_distance(mine[i - 1], mine[i]);
		return -back;
	}
	auto const mine{ahead(Switch, Path, Atend, kReach)};
	double along{0.0};
	auto gap{plan_gap(mine.front(), theirs)};
	for (std::size_t i = 1; i < mine.size(); ++i)
	{
		auto const step{plan_distance(mine[i - 1], mine[i])};
		auto const next{plan_gap(mine[i], theirs)};
		if (next >= kFouling)
			return along + (next > gap ? step * std::clamp((kFouling - gap) / (next - gap), 0.0, 1.0) : step);
		along += step;
		gap = next;
	}
	return along - kBeyond;
}

// how far from the end of the switch its joint lies: 4.2 m beyond the fouling point at a branch, before the start elsewhere
double clearance(TTrack const &Switch, int const Path, bool const Atend)
{
	auto const point{fouling(Switch, Path, Atend)};
	return point.has_value() ? std::max(0.0, *point + kBeyond) : kStart;
}

} // namespace

std::optional<layout> find(TTrack &From, options const &Options)
{
	auto const found{collect(From, Options)};
	if (found.switches.empty())
		return std::nullopt;
	std::unordered_set<TTrack const *> switches(found.switches.begin(), found.switches.end());
	// the axis: the run through the station along the main paths of its switches which passes the most of them
	std::vector<TTrack *> starts;
	for (auto const *lines : {&found.exits, &found.ends})
		for (auto const &line : *lines)
			if (false == line.tracks.empty())
				starts.push_back(line.tracks.front());
	if (starts.empty())
		starts.push_back(found.switches.front());
	editor_track::route best;
	std::size_t most{0};
	double longest{0.0};
	for (auto *start : starts)
	{
		auto route{editor_track::run_route(*start, 20000.0)};
		std::size_t passed{0};
		double length{0.0};
		for (auto const &span : route.spans)
		{
			passed += switches.count(span.track);
			length += plan_length(*span.track, span.path);
		}
		if (passed > most || (passed == most && length > longest))
		{
			most = passed;
			longest = length;
			best = std::move(route);
		}
	}
	layout result;
	for (auto const &span : best.spans)
	{
		double length{0.0};
		auto points{points_of(*span.track, span.path, length)};
		if (false == span.forward)
			std::reverse(points.begin(), points.end());
		for (auto const &point : points)
			if (result.axis.empty() || plan_distance(result.axis.back(), point) > 0.01)
				result.axis.push_back(point);
	}
	if (result.axis.size() < 2)
		return std::nullopt;
	auto const measure = [&]() {
		result.chainage.assign(1, 0.0);
		for (std::size_t i = 1; i < result.axis.size(); ++i)
			result.chainage.push_back(result.chainage.back() + plan_distance(result.axis[i - 1], result.axis[i]));
	};
	measure();
	// the chainage runs with the km of the switches, when they have one and most of them agree on it: of the kilometrage of the
	// line, of their records, of the alignment
	double constexpr kAgree{50.0}; // m
	std::vector<std::pair<double, double>> marks; // the chainage and the km, in m
	for (auto *track : found.switches)
	{
		auto const start{end_of(*track, 0, false)};
		std::optional<double> km;
		double line_km{0.0};
		if (auto const line{kilometrage::line_at(start)}; false == line.empty() && kilometrage::km_at(line, start, line_km))
			km = line_km * 1000.0;
		else if (auto const record{switch_store::named(*track)}; record && record->km)
			km = record->km;
		else
			km = alignment_store::chainage_at(start);
		if (km)
			marks.emplace_back(project(result, start).x, *km);
	}
	auto const length{result.chainage.back()};
	struct fit
	{
		std::size_t agree{0};
		double origin{0.0};
	};
	auto const fit_of = [&](bool const Reversed) {
		std::vector<double> origins;
		for (auto const &[chainage, km] : marks)
			origins.push_back(km - (Reversed ? length - chainage : chainage));
		std::nth_element(origins.begin(), origins.begin() + origins.size() / 2, origins.end());
		fit result{0, origins[origins.size() / 2]};
		for (auto const origin : origins)
			result.agree += std::abs(origin - result.origin) <= kAgree ? 1 : 0;
		return result;
	};
	if (false == marks.empty())
	{
		auto const along{fit_of(false)};
		auto const against{fit_of(true)};
		auto const reversed{against.agree > along.agree};
		auto const &best{reversed ? against : along};
		if (best.agree * 2 >= marks.size())
		{
			if (reversed)
			{
				std::reverse(result.axis.begin(), result.axis.end());
				measure();
			}
			result.kilometrage = true;
			result.km = best.origin / 1000.0;
		}
	}
	// without the km the station runs from the west to the east, as the map shows it
	if (false == result.kilometrage && map_of(result.axis.back()).x < map_of(result.axis.front()).x)
	{
		std::reverse(result.axis.begin(), result.axis.end());
		measure();
	}
	// a long track ending free far beyond the switches is a line cut off by the edge of the scenery, else a siding
	std::vector<chain> exits{found.exits};
	std::vector<TTrack *> plain{found.plain};
	{
		auto low{std::numeric_limits<double>::max()};
		auto high{std::numeric_limits<double>::lowest()};
		for (auto const *track : found.switches)
		{
			auto const along{project(result, end_of(*track, 0, false)).x};
			low = std::min(low, along);
			high = std::max(high, along);
		}
		for (auto const &line : found.ends)
		{
			auto beyond{0.0};
			for (auto const *track : line.tracks)
				for (auto const atend : {false, true})
				{
					auto const at{project(result, end_of(*track, 0, atend))};
					beyond = std::max({beyond, low - at.x, at.x - high});
				}
			if (beyond > Options.approach * 0.5 && line.length > Options.approach)
				exits.push_back(line);
			else
				plain.insert(plain.end(), line.tracks.begin(), line.tracks.end());
		}
	}
	// the paths, laid out along the axis
	auto const add = [&](TTrack &Track, bool const Approach) {
		for (int index = 0; index < static_cast<int>(Track.m_paths.size()); ++index)
		{
			path entry;
			entry.track = &Track;
			entry.index = index;
			entry.approach = Approach;
			entry.points = points_of(Track, index, entry.length);
			for (auto const &point : entry.points)
				entry.schematic.push_back(project(result, point));
			result.paths.push_back(std::move(entry));
		}
	};
	for (auto *track : found.switches)
		add(*track, false);
	for (auto *track : plain)
		add(*track, false);
	for (auto const &line : exits)
	{
		double length{0.0};
		result.lines.emplace_back();
		for (auto *track : line.tracks)
		{
			if (length >= Options.approach)
				break;
			add(*track, true);
			result.lines.back().push_back(track);
			length += plan_length(*track, 0);
		}
	}
	result.from = std::numeric_limits<double>::max();
	result.to = std::numeric_limits<double>::lowest();
	result.low = std::numeric_limits<double>::max();
	result.high = std::numeric_limits<double>::lowest();
	for (auto const &entry : result.paths)
	{
		if (entry.approach)
			continue;
		for (auto const &point : entry.schematic)
		{
			if (switches.count(entry.track) > 0)
			{
				result.from = std::min(result.from, point.x);
				result.to = std::max(result.to, point.x);
			}
			result.low = std::min(result.low, point.y);
			result.high = std::max(result.high, point.y);
		}
	}
	result.switches = found.switches;
	std::sort(result.switches.begin(), result.switches.end(), [&](TTrack const *A, TTrack const *B) { return project(result, end_of(*A, 0, false)).x < project(result, end_of(*B, 0, false)).x; });
	// a double-track line: another line leaves the station by the one along the axis a few metres aside; the run from it along
	// the main paths of the switches is the other main track
	for (auto const &span : best.spans)
		result.main.push_back(span.track);
	std::unordered_set<TTrack const *> const axis(result.main.begin(), result.main.end());
	std::vector<std::pair<std::size_t, glm::dvec2>> leaving;
	for (std::size_t index = 0; index < result.lines.size(); ++index)
	{
		std::vector<glm::dvec2> points;
		for (auto const &path : result.paths)
			if (path.approach && std::find(result.lines[index].begin(), result.lines[index].end(), path.track) != result.lines[index].end())
				points.insert(points.end(), path.schematic.begin(), path.schematic.end());
		auto const beyond = [&](glm::dvec2 const &Point) { return std::abs(std::max(result.from - Point.x, Point.x - result.to) - 60.0); };
		if (false == points.empty())
			leaving.emplace_back(index, *std::min_element(points.begin(), points.end(), [&](glm::dvec2 const &A, glm::dvec2 const &B) { return beyond(A) < beyond(B); }));
	}
	for (auto const &[along, at] : leaving)
	{
		if (axis.count(result.lines[along].front()) == 0 || false == result.second.empty())
			continue;
		for (auto const &[aside, there] : leaving)
		{
			if (aside == along || axis.count(result.lines[aside].front()) > 0 || (at.x < result.from) != (there.x < result.from) || std::abs(there.y - at.y) < 3.0 || std::abs(there.y - at.y) > 7.0)
				continue;
			// the run goes in from the end of the line at the station
			auto &first{*result.lines[aside].front()};
			auto const *start{neighbour(first, 0, false)};
			auto const inwards{start != nullptr && switches.count(start) > 0 ? false : true};
			for (auto *track : straight_run(first, end_of(first, 0, inwards), result.chainage.back() + 2.0 * Options.approach))
				if (axis.count(track) == 0 && std::find(result.second.begin(), result.second.end(), track) == result.second.end())
					result.second.push_back(track);
			break;
		}
	}
	return result;
}

glm::dvec2 project(layout const &Layout, glm::dvec3 const &Point)
{
	auto const point{map_of(Point)};
	auto best{std::numeric_limits<double>::max()};
	glm::dvec2 result{0.0};
	for (std::size_t i = 1; i < Layout.axis.size(); ++i)
	{
		auto const a{map_of(Layout.axis[i - 1])};
		auto const b{map_of(Layout.axis[i])};
		auto const along{b - a};
		auto const length{glm::length(along)};
		if (length < 1e-9)
			continue;
		auto const unit{along / length};
		// beyond the ends of the axis it goes on straight
		auto const t{std::clamp(glm::dot(point - a, unit), i == 1 ? std::numeric_limits<double>::lowest() : 0.0, i + 1 == Layout.axis.size() ? std::numeric_limits<double>::max() : length)};
		auto const foot{a + unit * t};
		auto const distance{glm::distance(point, foot)};
		if (distance >= best)
			continue;
		best = distance;
		auto const side{point - foot};
		result = {Layout.chainage[i - 1] + t, unit.x * side.y - unit.y * side.x};
	}
	return result;
}

double km_of(layout const &Layout, double const Chainage)
{
	return Layout.km + Chainage / 1000.0;
}

std::vector<TTrack *> stations(options const &Options)
{
	std::vector<TTrack *> result;
	std::unordered_set<TTrack const *> taken;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || false == node(*track) || taken.count(track) > 0)
			continue;
		auto const found{collect(*track, Options)};
		for (auto const *member : found.switches)
			taken.insert(member);
		result.push_back(track);
	}
	return result;
}

std::vector<unit> numbering(layout const &Layout, bool const Reversed)
{
	std::unordered_set<TTrack const *> members;
	for (auto const &entry : Layout.paths)
		if (false == entry.approach)
			members.insert(entry.track);
	std::unordered_set<TTrack const *> taken;
	std::vector<unit> result;
	for (auto *track : Layout.switches)
	{
		if (track->eType == tt_Table || taken.count(track) > 0)
			continue;
		unit entry;
		if (auto const record{switch_store::named(*track)})
		{
			entry.record = record->name;
			if (record->kind != "switch")
				for (auto const &name : record->parts)
					if (auto *part{simulation::Paths.find(name)}; part != nullptr && members.count(part) > 0 && taken.insert(part).second)
						entry.parts.push_back(part);
		}
		if (taken.insert(track).second)
			entry.parts.push_back(track);
		entry.chainage = std::numeric_limits<double>::max();
		for (auto const *part : entry.parts)
			entry.chainage = std::min(entry.chainage, project(Layout, end_of(*part, 0, false)).x);
		result.push_back(std::move(entry));
	}
	std::stable_sort(result.begin(), result.end(), [&](unit const &A, unit const &B) { return Reversed ? A.chainage > B.chainage : A.chainage < B.chainage; });
	return result;
}

std::string prefix(std::string const &Code)
{
	std::string result;
	for (auto const c : Code)
		result += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return result + '_';
}

std::string switch_name(std::string const &Code, int const Number, std::size_t const Part, std::size_t const Parts)
{
	auto result{prefix(Code) + std::to_string(Number)};
	if (Parts > 1)
		result += static_cast<char>('a' + std::min<std::size_t>(Part, 25));
	return result;
}

std::vector<joint> joints(layout const &Layout, std::vector<unit> const &Units, options const &Options)
{
	std::unordered_map<TTrack const *, std::size_t> unit_of;
	for (std::size_t index = 0; index < Units.size(); ++index)
		for (auto const *part : Units[index].parts)
			unit_of[part] = index;
	auto const one_unit = [&](TTrack const *A, TTrack const *B) {
		auto const a{unit_of.find(A)};
		auto const b{unit_of.find(B)};
		return a != unit_of.end() && b != unit_of.end() && a->second == b->second;
	};
	std::unordered_set<TTrack const *> through;
	std::vector<glm::dvec3> meets;
	std::vector<joint> result;
	for (auto *from : Layout.switches)
		for (auto const &[path, atend] : ends_of(*from))
		{
			auto line{follow(*from, path, atend, std::numeric_limits<double>::max())};
			auto const start{end_of(*from, path, atend)};
			// out of the station: to a switch of another one, along a line cut off by the edge of the scenery
			if ((line.reached != nullptr && std::find(Layout.switches.begin(), Layout.switches.end(), line.reached) == Layout.switches.end()) ||
			    (line.ended && false == line.tracks.empty() && std::any_of(Layout.lines.begin(), Layout.lines.end(), [&](std::vector<TTrack *> const &Line) { return false == Line.empty() && Line.front() == line.tracks.front(); })))
			{
				line.reached = nullptr;
				line.ended = false;
			}
			if (line.reached == nullptr && false == line.ended)
			{
				double length{0.0};
				auto const kept{std::find_if(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Track) { return (length += plan_length(*Track, 0)) > kReach * 2.0; })};
				line.tracks.erase(kept == line.tracks.end() ? kept : std::next(kept), line.tracks.end());
			}
			if (line.tracks.empty())
			{
				// two switches meeting: the joint lies where they meet, unless they're parts of one
				if (line.reached == nullptr || one_unit(from, line.reached) || std::any_of(meets.begin(), meets.end(), [&](glm::dvec3 const &Meet) { return glm::distance(Meet, start) < same_point; }))
					continue;
				meets.push_back(start);
				result.push_back({nullptr, start});
				continue;
			}
			if (through.count(line.tracks.front()) > 0)
				continue;
			through.insert(line.tracks.begin(), line.tracks.end());
			// the link as one line from the switch on, and where each of its tracks ends along it
			std::vector<glm::dvec3> points{start};
			std::vector<double> lengths{0.0};
			std::vector<std::pair<double, TTrack *>> pieces;
			for (auto *track : line.tracks)
			{
				double plan{0.0};
				auto samples{dense_points(*track, 0, plan)};
				if (plan_distance(samples.back(), points.back()) < plan_distance(samples.front(), points.back()))
					std::reverse(samples.begin(), samples.end());
				for (std::size_t i = 1; i < samples.size(); ++i)
				{
					lengths.push_back(lengths.back() + plan_distance(points.back(), samples[i]));
					points.push_back(samples[i]);
				}
				pieces.emplace_back(lengths.back(), track);
			}
			auto const length{lengths.back()};
			auto const own{clearance(*from, path, atend)};
			std::vector<double> places;
			if (line.reached != nullptr)
			{
				int far_path{0};
				bool far_atend{false};
				if (one_unit(from, line.reached) || false == end_at(*line.reached, points.back(), far_path, far_atend))
					continue;
				auto const theirs{clearance(*line.reached, far_path, far_atend)};
				if (length - own - theirs >= kSection)
					places = {own, length - theirs};
				else
					places = {(own + length - theirs) * 0.5};
			}
			else if (false == line.ended || length - own >= kSection)
				places = {own};
			for (auto const place : places)
			{
				auto distance{std::clamp(place, 0.0, length)};
				TTrack *track{nullptr};
				auto previous{0.0};
				for (auto const &[end, piece] : pieces)
				{
					if (std::abs(distance - previous) <= kSnap || std::abs(distance - end) <= kSnap)
					{
						distance = std::abs(distance - previous) <= kSnap ? previous : end;
						track = nullptr;
						break;
					}
					if (distance < end)
					{
						track = piece;
						break;
					}
					previous = end;
				}
				auto const after{std::min<std::size_t>(std::upper_bound(lengths.begin(), lengths.end(), distance) - lengths.begin(), lengths.size() - 1)};
				auto const before{after > 0 ? after - 1 : 0};
				auto const span{lengths[after] - lengths[before]};
				result.push_back({track, glm::mix(points[before], points[after], span > 1e-9 ? (distance - lengths[before]) / span : 0.0)});
			}
		}
	return result;
}

std::vector<circuit> circuits(layout const &Layout, std::vector<unit> const &Units, std::vector<glm::dvec3> const &Joints)
{
	std::unordered_map<TTrack const *, std::size_t> unit_of;
	for (std::size_t index = 0; index < Units.size(); ++index)
		for (auto const *part : Units[index].parts)
			unit_of[part] = index;
	std::vector<TTrack *> tracks{Layout.switches};
	std::unordered_set<TTrack const *> members(tracks.begin(), tracks.end());
	std::unordered_set<TTrack const *> approach;
	for (auto const &entry : Layout.paths)
	{
		if (members.insert(entry.track).second)
			tracks.push_back(entry.track);
		if (entry.approach)
			approach.insert(entry.track);
	}
	auto const barrier = [&](glm::dvec3 const &Point) { return std::any_of(Joints.begin(), Joints.end(), [&](glm::dvec3 const &Joint) { return glm::distance(Joint, Point) < 0.05; }); };
	std::unordered_set<TTrack const *> taken;
	std::vector<circuit> result;
	for (auto *seed : tracks)
	{
		if (taken.count(seed) > 0)
			continue;
		taken.insert(seed);
		circuit entry;
		entry.tracks.push_back(seed);
		for (std::size_t i = 0; i < entry.tracks.size(); ++i)
		{
			auto const &track{*entry.tracks[i]};
			auto const ends{node(track) ? ends_of(track) : std::vector<std::pair<int, bool>>{{0, false}, {0, true}}};
			for (auto const &[path, atend] : ends)
			{
				if (barrier(end_of(track, path, atend)))
					continue;
				auto *next{neighbour(track, path, atend)};
				if (next != nullptr && members.count(next) > 0 && taken.insert(next).second)
					entry.tracks.push_back(next);
			}
		}
		for (auto const *track : entry.tracks)
			if (auto const found{unit_of.find(track)}; found != unit_of.end())
			{
				entry.unit = static_cast<int>(found->second);
				break;
			}
		if (entry.unit < 0 && std::any_of(entry.tracks.begin(), entry.tracks.end(), [&](TTrack const *Track) { return approach.count(Track) > 0; }))
			continue;
		entry.from = std::numeric_limits<double>::max();
		entry.to = std::numeric_limits<double>::lowest();
		double weighted{0.0};
		std::unordered_set<TTrack const *> own(entry.tracks.begin(), entry.tracks.end());
		for (auto const &path : Layout.paths)
		{
			if (own.count(path.track) == 0)
				continue;
			for (std::size_t i = 1; i < path.schematic.size(); ++i)
			{
				auto const step{glm::distance(path.points[i - 1], path.points[i])};
				weighted += step * (path.schematic[i - 1].y + path.schematic[i].y) * 0.5;
				entry.length += step;
			}
			for (auto const &point : path.schematic)
			{
				entry.from = std::min(entry.from, point.x);
				entry.to = std::max(entry.to, point.x);
			}
		}
		entry.offset = entry.length > 0.0 ? weighted / entry.length : 0.0;
		result.push_back(std::move(entry));
	}
	return result;
}

std::vector<std::string> track_numbers(layout const &Layout, std::vector<circuit> const &Circuits)
{
	std::unordered_set<TTrack const *> const main(Layout.main.begin(), Layout.main.end());
	std::unordered_set<TTrack const *> const second(Layout.second.begin(), Layout.second.end());
	auto const on = [&](circuit const &Circuit, std::unordered_set<TTrack const *> const &Tracks) { return std::any_of(Circuit.tracks.begin(), Circuit.tracks.end(), [&](TTrack const *Track) { return Tracks.count(Track) > 0; }); };
	std::vector<std::size_t> axis;
	std::vector<std::size_t> other;
	std::vector<std::size_t> rest;
	std::unordered_map<TTrack const *, std::size_t> circuit_of;
	for (std::size_t index = 0; index < Circuits.size(); ++index)
	{
		for (auto const *track : Circuits[index].tracks)
			circuit_of[track] = index;
		if (Circuits[index].unit >= 0)
			continue;
		else if (on(Circuits[index], main))
			axis.push_back(index);
		else if (on(Circuits[index], second))
			other.push_back(index);
		else
			rest.push_back(index);
	}
	// a station track goes on through the switches along their straighter paths: its circuits beyond them share its number
	std::vector<std::size_t> group(Circuits.size());
	std::iota(group.begin(), group.end(), 0);
	auto const root = [&](std::size_t Index) {
		while (group[Index] != Index)
			Index = group[Index] = group[group[Index]];
		return Index;
	};
	std::unordered_set<std::size_t> const plain(rest.begin(), rest.end());
	for (auto const index : rest)
		for (auto const *track : Circuits[index].tracks)
			for (auto const atend : {false, true})
			{
				if (node(*track))
					break;
				auto point{end_of(*track, 0, atend)};
				auto *next{neighbour(*track, 0, atend)};
				double length{0.0};
				while (next != nullptr && length < kThrough && rail(*next))
				{
					auto const there{circuit_of.find(next)};
					if (there == circuit_of.end() || there->second == index)
						break;
					if (Circuits[there->second].unit < 0)
					{
						if (plain.count(there->second) > 0)
							group[root(there->second)] = root(index);
						break;
					}
					int path{0};
					bool end{false};
					if (false == end_at(*next, point, path, end))
						break;
					if (next->eType == tt_Switch)
					{
						if (end && path != straighter(*next))
							break;
						path = straighter(*next);
					}
					length += plan_length(*next, path);
					point = end_of(*next, path, false == end);
					auto const *through{next};
					next = neighbour(*through, path, false == end);
				}
			}
	std::vector<std::pair<std::vector<std::size_t>, double>> right;
	std::vector<std::pair<std::vector<std::size_t>, double>> left;
	{
		std::map<std::size_t, std::vector<std::size_t>> groups;
		for (auto const index : rest)
			groups[root(index)].push_back(index);
		for (auto &[first, members] : groups)
		{
			double weighted{0.0};
			double length{0.0};
			for (auto const index : members)
			{
				weighted += Circuits[index].offset * Circuits[index].length;
				length += Circuits[index].length;
			}
			auto const offset{length > 0.0 ? weighted / length : Circuits[first].offset};
			(offset < 0.0 ? right : left).emplace_back(std::move(members), offset);
		}
	}
	auto const mean = [&](std::vector<std::size_t> const &Group) {
		double weighted{0.0};
		double length{0.0};
		for (auto const index : Group)
		{
			weighted += Circuits[index].offset * Circuits[index].length;
			length += Circuits[index].length;
		}
		return length > 0.0 ? weighted / length : 0.0;
	};
	// track 1 on the right along the chainage: the other main track of a double-track line when it lies to the right of the axis
	auto const swapped{false == other.empty() && mean(other) < mean(axis)};
	std::vector<std::pair<std::vector<std::size_t>, int>> tracks{{axis, swapped ? 2 : 1}};
	if (false == other.empty())
		tracks.emplace_back(other, swapped ? 1 : 2);
	// beyond the main tracks the others side by side, the nearest first, the odd numbers to the right, the even ones to the left
	for (auto *side : {&right, &left})
	{
		std::sort(side->begin(), side->end(), [&](auto const &A, auto const &B) { return std::abs(A.second) < std::abs(B.second); });
		auto number{side == &right ? 3 : other.empty() ? 2 : 4};
		for (auto &[members, offset] : *side)
		{
			tracks.emplace_back(std::move(members), number);
			number += 2;
		}
	}
	std::vector<std::string> result(Circuits.size());
	for (auto &[members, number] : tracks)
	{
		std::sort(members.begin(), members.end(), [&](std::size_t A, std::size_t B) { return Circuits[A].from < Circuits[B].from; });
		for (std::size_t index = 0; index < members.size(); ++index)
			result[members[index]] = std::to_string(number) + (members.size() > 1 ? std::string(1, static_cast<char>('a' + std::min<std::size_t>(index, 25))) : std::string{});
	}
	return result;
}

std::vector<TTrack *> switches_of(TTrack &Switch, options const &Options)
{
	return collect(Switch, Options).switches;
}

namespace
{

// the point the distance in from the end of the plain track, and the way to the end there
std::pair<glm::dvec3, glm::dvec3> in_from(TTrack const &Track, bool const Atend, double const Distance)
{
	double length{0.0};
	auto points{dense_points(Track, 0, length)};
	if (Atend)
		std::reverse(points.begin(), points.end());
	double along{0.0};
	for (std::size_t i = 1; i < points.size(); ++i)
	{
		auto const step{plan_distance(points[i - 1], points[i])};
		if (along + step >= Distance || i + 1 == points.size())
		{
			auto const t{step > 1e-9 ? std::clamp((Distance - along) / step, 0.0, 1.0) : 0.0};
			return {glm::mix(points[i - 1], points[i], t), points[i - 1] - points[i]};
		}
		along += step;
	}
	return {points.front(), glm::dvec3{0.0, 0.0, 1.0}};
}

// the point the distance in from the end of the track, on into the tracks before it while they're shorter, short of a switch: the
// track, the end of it the trains running out reach, the point and the way to that end
std::tuple<TTrack *, bool, glm::dvec3, glm::dvec3> back_from(TTrack *Track, bool Atend, double Distance)
{
	std::unordered_set<TTrack const *> seen{Track};
	while (true)
	{
		auto const length{plan_length(*Track, 0)};
		if (Distance < length - 1.0)
			break;
		auto const inner{end_of(*Track, 0, false == Atend)};
		auto *before{neighbour(*Track, 0, false == Atend)};
		int path{0};
		bool atend{false};
		if (before == nullptr || node(*before) || false == rail(*before) || false == seen.insert(before).second || false == end_at(*before, inner, path, atend))
		{
			Distance = std::min(Distance, length * 0.5);
			break;
		}
		Distance -= length;
		Track = before;
		Atend = atend;
	}
	auto const [at, way] = in_from(*Track, Atend, Distance);
	return {Track, Atend, at, way};
}

// where a signal on the side of the track at the point stands, in the plan; Way: of the path there
glm::dvec2 side_spot(glm::dvec3 const &Point, glm::dvec3 const &Way, int const Side)
{
	glm::dvec2 const way{glm::normalize(glm::dvec2{Way.x, Way.z} + glm::dvec2{1e-12, 0.0})};
	return glm::dvec2{Point.x, Point.z} + glm::dvec2{-way.y, way.x} * (Side * kSignalSide);
}

// how far a signal on the side of the track at the point keeps from the other tracks, none where another signal stands already
double side_gap(layout const &Layout, glm::dvec3 const &Point, glm::dvec3 const &Way, int const Side, std::vector<glm::dvec2> const &Taken)
{
	auto const spot{side_spot(Point, Way, Side)};
	if (std::any_of(Taken.begin(), Taken.end(), [&](glm::dvec2 const &Other) { return glm::distance(Other, spot) < kSignalApart; }))
		return 0.0;
	glm::dvec3 const at{spot.x, Point.y, spot.y};
	auto gap{std::numeric_limits<double>::max()};
	for (auto const &path : Layout.paths)
		if (plan_gap(Point, path.points) > 1.0)
			gap = std::min(gap, plan_gap(at, path.points));
	return gap;
}

// on the right of the trains, unless another track runs too close by there or another signal stands there, and less so on the left;
// the spot taken
int side_for(layout const &Layout, glm::dvec3 const &Point, glm::dvec3 const &Way, int const Facing, std::vector<glm::dvec2> &Taken)
{
	auto const right{side_gap(Layout, Point, Way, Facing, Taken)};
	auto side{Facing};
	if (right < kSignalClear)
	{
		auto const left{side_gap(Layout, Point, Way, -Facing, Taken)};
		if (left >= kSignalClear || left > right)
			side = -Facing;
	}
	Taken.push_back(side_spot(Point, Way, side));
	return side;
}

// the ends of the tracks out of the set, Free where no track goes on
struct track_end
{
	TTrack *track{nullptr};
	bool atend{false};
	glm::dvec3 point{0.0};
	bool free{false};
};

std::vector<track_end> ends_out(std::vector<TTrack *> const &Tracks)
{
	std::unordered_set<TTrack const *> const own(Tracks.begin(), Tracks.end());
	std::vector<track_end> result;
	for (auto *track : Tracks)
	{
		if (node(*track))
			continue;
		for (auto const atend : {false, true})
		{
			auto const *next{neighbour(*track, 0, atend)};
			if (next != nullptr && rail(*next) && own.count(next) > 0)
				continue;
			result.push_back({track, atend, end_of(*track, 0, atend), next == nullptr || false == rail(*next)});
		}
	}
	return result;
}

char next_letter(char Letter, int const Step)
{
	do
		Letter = static_cast<char>(Letter + Step);
	while (Letter == 'I' || Letter == 'Q');
	return Letter;
}

} // namespace

std::vector<signal> signals(layout const &Layout, std::vector<numbered> const &Tracks, std::function<bool(TTrack const &)> const &Inside)
{
	std::vector<signal> result;
	auto const middle{(Layout.from + Layout.to) * 0.5};
	std::vector<glm::dvec2> taken; // the spots of the signals placed
	// the entry signals: out along each line from where it leaves the circuits of the station
	struct entry
	{
		signal placed;
		double offset{0.0};
		bool start{false};
	};
	std::vector<entry> entries;
	for (auto const &line : Layout.lines)
	{
		auto const outside{std::find_if(line.begin(), line.end(), [&](TTrack const *Track) { return false == Inside(*Track); })};
		if (outside == line.end() || node(**outside))
			continue;
		// the joint: the end of the first track out of the station by the track before it
		auto *track{*outside};
		bool atend{false};
		if (outside != line.begin())
		{
			int path{0};
			bool end{false};
			atend = false == (end_at(**std::prev(outside), end_of(*track, 0, false), path, end));
		}
		else
		{
			auto const start{end_of(*track, 0, false)};
			atend = std::none_of(Layout.switches.begin(), Layout.switches.end(), [&](TTrack const *Switch) {
				return std::any_of(Switch->m_paths.begin(), Switch->m_paths.end(), [&](auto const &Path) { return glm::distance(Path.points[segment_data::point::start], start) < same_point || glm::distance(Path.points[segment_data::point::end], start) < same_point; });
			});
		}
		// out from the joint: along the path when the joint is its start
		entry found;
		auto const joint{end_of(*track, 0, atend)};
		found.offset = project(Layout, joint).y;
		found.start = project(Layout, joint).x < middle;
		double along{0.0};
		auto point{joint};
		auto *next{track};
		std::unordered_set<TTrack const *> seen;
		while (next != nullptr && rail(*next) && false == node(*next) && seen.insert(next).second)
		{
			int path{0};
			bool end{false};
			if (false == end_at(*next, point, path, end))
				break;
			auto const outward{false == end};
			auto const length{plan_length(*next, 0)};
			found.placed.run.emplace_back(next, outward);
			if (along + length >= kEntry)
			{
				auto const [at, way] = in_from(*next, end, kEntry - along);
				found.placed.track = next;
				found.placed.point = at;
				found.placed.facing = outward ? -1 : 1;
				found.placed.side = side_for(Layout, at, outward ? -way : way, found.placed.facing, taken);
				found.placed.travel = way;
				break;
			}
			along += length;
			point = end_of(*next, 0, outward);
			next = neighbour(*next, 0, outward);
		}
		if (found.placed.track == nullptr && false == found.placed.run.empty() && along > 30.0)
		{
			// the line ends sooner, by the edge of the scenery: as far out as it goes
			auto const &[last, outward] = found.placed.run.back();
			auto const length{plan_length(*last, 0)};
			auto const [at, way] = in_from(*last, outward, std::min(10.0, length * 0.5));
			found.placed.track = last;
			found.placed.point = at;
			found.placed.facing = outward ? -1 : 1;
			found.placed.side = side_for(Layout, at, outward ? way : -way, found.placed.facing, taken);
			found.placed.travel = -way;
		}
		if (found.placed.track != nullptr)
			entries.push_back(std::move(found));
	}
	std::sort(entries.begin(), entries.end(), [](entry const &A, entry const &B) { return A.offset > B.offset; });
	char from_start{'A'};
	char from_end{'Z'};
	char last_start{'@'};
	char first_end{'['};
	for (auto &found : entries)
	{
		auto &letter{found.start ? from_start : from_end};
		found.placed.kind = signal::role::entry;
		found.placed.name = std::string(1, letter);
		(found.start ? last_start : first_end) = letter;
		letter = next_letter(letter, found.start ? 1 : -1);
		result.push_back(std::move(found.placed));
	}
	// the exit signals, a letter for the odd tracks and one for the even ones at each end, clear of the entry ones
	auto letter_after = [](char Letter, char const Limit, int const Step) {
		while (Step > 0 ? Letter <= Limit : Letter >= Limit)
			Letter = next_letter(Letter, Step);
		return Letter;
	};
	auto const odd_start{letter_after('F', last_start, 1)};
	auto const even_start{next_letter(odd_start, 1)};
	auto const odd_end{letter_after('R', first_end, -1)};
	auto const even_end{next_letter(odd_end, -1)};
	auto const lines_at = [&](bool const Start) { return std::any_of(entries.begin(), entries.end(), [&](entry const &Entry) { return Entry.start == Start; }); };
	auto const starts{lines_at(true)};
	auto const ends{lines_at(false)};
	std::vector<signal> shunting;
	for (auto const &track : Tracks)
	{
		auto const digit{std::find_if(track.number.rbegin(), track.number.rend(), [](char const Letter) { return std::isdigit(static_cast<unsigned char>(Letter)) != 0; })};
		auto const odd{digit == track.number.rend() || (*digit - '0') % 2 == 1};
		auto upper{track.number};
		std::transform(upper.begin(), upper.end(), upper.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
		// the ends of the longest circuit long enough, for the longest useful length of the track both ways (Ie-4 § 13 ust. 8), else
		// the outer ends of the outermost ones
		struct reach
		{
			track_end low;
			track_end high;
			double low_x{std::numeric_limits<double>::max()};
			double high_x{std::numeric_limits<double>::lowest()};
		};
		reach longest;
		reach any;
		double best{0.0};
		for (auto const &circuit : track.circuits)
		{
			double length{0.0};
			for (auto const *part : circuit)
				length += node(*part) ? 0.0 : plan_length(*part, 0);
			auto const chosen{length >= kStopping && length > best};
			if (chosen)
			{
				best = length;
				longest = reach{};
			}
			for (auto const &end : ends_out(circuit))
			{
				auto const x{project(Layout, end.point).x};
				for (auto *found : {&any, chosen ? &longest : static_cast<reach *>(nullptr)})
				{
					if (found == nullptr)
						continue;
					if (x < found->low_x)
					{
						found->low_x = x;
						found->low = end;
					}
					if (x > found->high_x)
					{
						found->high_x = x;
						found->high = end;
					}
				}
			}
		}
		auto const stopping{longest.low.track != nullptr};
		auto const &chosen{stopping ? longest : any};
		for (auto const start : {true, false})
		{
			auto const &end{start ? chosen.low : chosen.high};
			if (end.track == nullptr || end.free)
				continue;
			signal placed;
			auto const exit{stopping && (start ? starts : ends)};
			auto const [track, atend, at, way] = back_from(end.track, end.atend, exit ? signal_drop : kJoint);
			placed.track = track;
			placed.point = at;
			placed.facing = atend ? 1 : -1;
			placed.side = side_for(Layout, at, atend ? way : -way, placed.facing, taken);
			placed.travel = way;
			if (exit)
			{
				placed.kind = signal::role::exit;
				placed.name = std::string(1, start ? (odd ? odd_start : even_start) : (odd ? odd_end : even_end)) + upper;
				result.push_back(std::move(placed));
			}
			else
			{
				placed.kind = signal::role::shunting;
				shunting.push_back(std::move(placed));
			}
		}
	}
	std::sort(shunting.begin(), shunting.end(), [&](signal const &A, signal const &B) { return project(Layout, A.point).x < project(Layout, B.point).x; });
	for (std::size_t index = 0; index < shunting.size(); ++index)
	{
		shunting[index].name = "Tm" + std::to_string(index + 1);
		result.push_back(std::move(shunting[index]));
	}
	return result;
}

std::vector<line> lines_out(layout const &Layout, std::vector<signal> const &Signals, double const Limit)
{
	std::vector<line> result;
	for (std::size_t index = 0; index < Signals.size(); ++index)
	{
		auto const &entry{Signals[index]};
		if (entry.kind != signal::role::entry || entry.track == nullptr || node(*entry.track))
			continue;
		line found;
		found.entry = index;
		std::unordered_set<TTrack const *> seen;
		auto const add = [&](TTrack *Track, bool const Outward, std::vector<glm::dvec3> Points) {
			if (false == found.points.empty())
				Points.erase(Points.begin());
			seen.insert(Track);
			found.tracks.push_back(Track);
			found.outward.push_back(Outward);
			for (auto const &point : Points)
			{
				found.distances.push_back(found.points.empty() ? 0.0 : found.distances.back() + plan_distance(found.points.back(), point));
				found.points.push_back(point);
				found.owners.push_back(found.tracks.size() - 1);
			}
		};
		// in from the signal to the joint of the station first: its trains run on there beyond it
		for (auto const &[part, out] : entry.run)
		{
			if (part == entry.track)
				break;
			double length{0.0};
			auto points{dense_points(*part, 0, length)};
			if (false == out)
				std::reverse(points.begin(), points.end());
			add(part, out, std::move(points));
		}
		// out where the trains it faces come from
		auto *track{entry.track};
		auto outward{entry.facing < 0};
		auto first{true};
		auto origin{0.0};
		while (true)
		{
			double length{0.0};
			auto points{dense_points(*track, 0, length)};
			if (false == outward)
				std::reverse(points.begin(), points.end());
			std::size_t at{0};
			if (first)
			{
				auto const nearest{std::min_element(points.begin(), points.end(), [&](glm::dvec3 const &A, glm::dvec3 const &B) { return plan_distance(A, entry.point) < plan_distance(B, entry.point); })};
				*nearest = entry.point;
				at = static_cast<std::size_t>(nearest - points.begin());
				if (found.points.empty())
				{
					points.erase(points.begin(), nearest);
					at = 0;
				}
				else
					at += found.points.size() - 1;
			}
			add(track, outward, std::move(points));
			if (first)
				origin = found.distances[at];
			first = false;
			if (found.length() - origin > Limit)
				break;
			auto const end{end_of(*track, 0, outward)};
			auto *next{neighbour(*track, 0, outward)};
			if (next == nullptr || false == rail(*next) || seen.count(next) > 0)
				break;
			if (node(*next))
			{
				found.station = true;
				break;
			}
			int path{0};
			bool atend{false};
			if (false == end_at(*next, end, path, atend))
				break;
			track = next;
			outward = false == atend;
		}
		for (auto &distance : found.distances)
			distance -= origin;
		if (found.length() > 1.0)
			result.push_back(std::move(found));
	}
	// a double-track line: two lines out of the same end 3 to 6.5 m apart all along their first kilometre
	auto const middle{(Layout.from + Layout.to) * 0.5};
	auto const start = [&](line const &Line) { return project(Layout, Line.points.front()).x < middle; };
	auto const gap_between = [&](line const &A, line const &B) {
		auto const reach{std::min({A.length(), B.length(), 1000.0})};
		auto worst{0.0};
		for (auto const fraction : {0.2, 0.6, 1.0})
		{
			double gap{0.0};
			distance_on(B, point_on(A, reach * fraction).point, gap);
			worst = std::max(worst, std::abs(gap - 4.5));
		}
		return reach < 100.0 ? std::numeric_limits<double>::max() : worst;
	};
	// the pairs running side by side the longest first: a siding along a main track for a while is no line
	std::vector<std::tuple<double, double, std::size_t, std::size_t>> pairs;
	for (std::size_t i = 0; i < result.size(); ++i)
		for (std::size_t j = i + 1; j < result.size(); ++j)
			if (start(result[i]) == start(result[j]))
				if (auto const gap{gap_between(result[i], result[j])}; gap <= 2.0)
					pairs.emplace_back(-std::min(result[i].length(), result[j].length()), gap, i, j);
	std::sort(pairs.begin(), pairs.end());
	std::vector<char> paired(result.size(), 0);
	for (auto const &[reach_order, gap, i, partner] : pairs)
	{
		if (paired[i] != 0 || paired[partner] != 0)
			continue;
		paired[i] = paired[partner] = 1;
		result[i].partner = static_cast<int>(partner);
		result[partner].partner = static_cast<int>(i);
		// the trains towards the station on the track on their right; along the chainage the right one is track 1
		auto const &travel{Signals[result[i].entry].travel};
		glm::dvec2 const right{-travel.z, travel.x};
		auto const reach{std::min({result[i].length(), result[partner].length(), 200.0})};
		auto const here{point_on(result[i], reach).point};
		auto const there{point_on(result[partner], reach).point};
		auto const partner_inbound{glm::dot(glm::dvec2{there.x - here.x, there.z - here.z}, right) > 0.0};
		result[i].inbound = false == partner_inbound;
		result[partner].inbound = partner_inbound;
		for (auto *found : {&result[i], &result[partner]})
			found->number = found->inbound == start(*found) ? 1 : 2;
	}
	return result;
}

line run_line(signal const &Entry)
{
	line result;
	for (auto const &[track, outward] : Entry.run)
	{
		double length{0.0};
		auto points{dense_points(*track, 0, length)};
		if (false == outward)
			std::reverse(points.begin(), points.end());
		if (false == result.points.empty())
			points.erase(points.begin());
		result.tracks.push_back(track);
		result.outward.push_back(outward);
		for (auto const &point : points)
		{
			result.distances.push_back(result.points.empty() ? 0.0 : result.distances.back() + plan_distance(result.points.back(), point));
			result.points.push_back(point);
			result.owners.push_back(result.tracks.size() - 1);
		}
	}
	return result;
}

on_line point_on(line const &Line, double const Distance)
{
	on_line result;
	if (Line.points.empty())
		return result;
	auto const after{std::lower_bound(Line.distances.begin(), Line.distances.end(), Distance)};
	auto const i{std::clamp<std::size_t>(static_cast<std::size_t>(after - Line.distances.begin()), 1, std::max<std::size_t>(1, Line.points.size() - 1))};
	if (Line.points.size() < 2)
	{
		result.track = Line.tracks[Line.owners.front()];
		result.outward = Line.outward[Line.owners.front()];
		result.point = Line.points.front();
		return result;
	}
	auto const step{Line.distances[i] - Line.distances[i - 1]};
	auto const t{step > 1e-9 ? std::clamp((Distance - Line.distances[i - 1]) / step, 0.0, 1.0) : 0.0};
	result.track = Line.tracks[Line.owners[i]];
	result.outward = Line.outward[Line.owners[i]];
	result.point = glm::mix(Line.points[i - 1], Line.points[i], t);
	auto const way{Line.points[i] - Line.points[i - 1]};
	if (glm::length(way) > 1e-9)
		result.way = glm::normalize(way);
	return result;
}

double distance_on(line const &Line, glm::dvec3 const &Point, double &Gap)
{
	Gap = std::numeric_limits<double>::max();
	auto result{0.0};
	glm::dvec2 const point{Point.x, Point.z};
	for (std::size_t i = 1; i < Line.points.size(); ++i)
	{
		glm::dvec2 const a{Line.points[i - 1].x, Line.points[i - 1].z};
		glm::dvec2 const b{Line.points[i].x, Line.points[i].z};
		auto const along{b - a};
		auto const length{glm::length(along)};
		if (length < 1e-9)
			continue;
		auto const t{std::clamp(glm::dot(point - a, along) / (length * length), 0.0, 1.0)};
		if (auto const gap{glm::distance(point, a + along * t)}; gap < Gap)
		{
			Gap = gap;
			result = Line.distances[i - 1] + length * t;
		}
	}
	return result;
}

std::string block_number(double const Km, int const Track, bool const Rising)
{
	auto const hectometres{std::max(Km, 0.0) * 10.0};
	auto number{static_cast<long long>(Rising ? std::ceil(hectometres - 1e-6) : std::floor(hectometres + 1e-6))};
	if ((number % 2 == 1) != (Track % 2 == 1))
		number += Rising ? 1 : -1;
	if (number < 1)
		number = Track % 2 == 1 ? 1 : 2;
	return std::to_string(number);
}

namespace
{

double constexpr kBeside{4.5}; // m, from the axis of a station track to a signal of it, at most
double constexpr kFacing{0.5}; // of the cosine between the way of a signal and the track, at least
double constexpr kLevel{1.5}; // m, the most a road crossing a track lies above or below it at a level crossing
double constexpr kSkew{0.2}; // of the sine between a road and a track, the least a level crossing is taken at
int constexpr kSearch{4096}; // of the steps of the way through the switches within a station track, the most tried

// the points of the path from its end, Length: in the plan
std::vector<glm::dvec3> from_end(TTrack const &Track, int const Path, bool const Atend, double &Length)
{
	auto points{dense_points(Track, Path, Length)};
	if (Atend)
		std::reverse(points.begin(), points.end());
	return points;
}

// a path of a track along a station track, run along from its end
struct step
{
	TTrack *track{nullptr};
	int path{0};
	bool atend{false};
};

// the shortest way out of the end of the track through the tracks of others, the switches within a station track, on to one of
// its own not taken yet, no longer than the limit; its last step on that one, empty for none
std::vector<step> way_on(TTrack const &Track, bool const Atend, std::unordered_set<TTrack const *> const &Own, std::unordered_set<TTrack const *> const &Taken, double const Limit)
{
	std::vector<step> best;
	auto shortest{Limit};
	std::vector<step> current;
	auto budget{kSearch};
	std::function<void(TTrack const &, int, bool, double)> search = [&](TTrack const &From, int const Path, bool const End, double const Length) {
		auto const point{end_of(From, Path, End)};
		auto *next{neighbour(From, Path, End)};
		if (--budget < 0 || next == nullptr || false == rail(*next) || Length >= shortest || std::any_of(current.begin(), current.end(), [&](step const &Step) { return Step.track == next; }))
			return;
		if (Own.count(next) > 0)
		{
			int path{0};
			bool atend{false};
			if (Taken.count(next) == 0 && end_at(*next, point, path, atend))
			{
				best = current;
				best.push_back({next, 0, atend});
				shortest = Length;
			}
			return;
		}
		for (int path = 0; path < static_cast<int>(next->m_paths.size()); ++path)
			for (auto const atend : {false, true})
				if (glm::distance(end_of(*next, path, atend), point) < same_point)
				{
					current.push_back({next, path, atend});
					search(*next, path, false == atend, Length + plan_length(*next, path));
					current.pop_back();
				}
	};
	search(Track, 0, Atend, 0.0);
	return best;
}

// out of an end of a station track to the switch beyond it or to the end of the tracks
struct beyond
{
	TTrack const *node{nullptr}; // the switch, none at the end of the tracks
	bool open{false}; // neither within reach: the circuit of the track ends by another of plain track
	double out{0.0}; // from the end of the track to the end of the switch, or to the end of the tracks
	std::optional<double> fouling; // from the end of the track out to the fouling point of the switch
	std::vector<glm::dvec3> points; // out from the end of the track, into the switch
	std::vector<double> distances;
};

beyond out_of(TTrack const &Track, bool const Atend)
{
	beyond result;
	auto point{end_of(Track, 0, Atend)};
	result.points.push_back(point);
	result.distances.push_back(0.0);
	auto *next{neighbour(Track, 0, Atend)};
	std::unordered_set<TTrack const *> seen{&Track};
	while (next != nullptr && rail(*next) && seen.insert(next).second)
	{
		int path{0};
		bool atend{false};
		if (false == end_at(*next, point, path, atend) || result.out > kReach)
			break;
		double length{0.0};
		auto const points{from_end(*next, path, atend, length)};
		for (std::size_t i = 1; i < points.size(); ++i)
		{
			result.distances.push_back(result.distances.back() + plan_distance(result.points.back(), points[i]));
			result.points.push_back(points[i]);
		}
		if (node(*next))
		{
			result.node = next;
			if (auto const at{fouling(*next, path, atend)})
				result.fouling = result.out - *at;
			return result;
		}
		result.out = result.distances.back();
		point = end_of(*next, 0, false == atend);
		next = neighbour(*next, 0, false == atend);
	}
	result.open = next != nullptr && rail(*next);
	return result;
}

glm::dvec3 point_along(std::vector<glm::dvec3> const &Points, std::vector<double> const &Distances, double const Along)
{
	auto const after{std::min<std::size_t>(std::upper_bound(Distances.begin(), Distances.end(), Along) - Distances.begin(), Distances.size() - 1)};
	auto const before{after > 0 ? after - 1 : 0};
	auto const span{Distances[after] - Distances[before]};
	return glm::mix(Points[before], Points[after], span > 1e-9 ? std::clamp((Along - Distances[before]) / span, 0.0, 1.0) : 0.0);
}

// an end of a stretch of a station track between the switches: at a switch within the track or at an end of it, the distances along
// the track
struct bound
{
	TTrack const *node{nullptr}; // the switch, none at the end of the tracks
	bool open{false}; // no switch: the circuit ends by another of plain track
	double joint{0.0}; // the end of the circuit of the stretch, the edge of a level crossing
	std::optional<double> fouling; // of the switch
	double start{0.0}; // of the switch, where the stretch meets it at its start; else the end of the tracks
	bool crossing{false}; // a level crossing
};

// the limit of a stretch at its end, Inward: the way into it along the track. Behind the trains stopped by a signal the joint, or the
// fouling point where it lies within; else the fouling point, the start of the switch or the end of the tracks
limit limit_at(bound const &Bound, double const Inward, bool const Signalled)
{
	limit found;
	found.along = Bound.joint;
	if (Bound.crossing)
		found.what = limit::kind::crossing;
	if (Bound.open || Bound.crossing)
		return found;
	if (Bound.node == nullptr)
	{
		found.what = limit::kind::buffer;
		found.along = Bound.start;
		return found;
	}
	found.name = Bound.node->name();
	if (Bound.fouling.has_value())
	{
		if (false == Signalled || (*Bound.fouling - Bound.joint) * Inward > 0.0)
		{
			found.what = limit::kind::fouling;
			found.along = *Bound.fouling;
		}
	}
	else if (false == Signalled)
	{
		found.what = limit::kind::start;
		found.along = Bound.start;
	}
	return found;
}

// the roads by the station, each as points along its axis with its width
struct road
{
	std::vector<glm::dvec3> points;
	double width{0.0};
	glm::dvec2 low{0.0}; // the corners of the plan around it
	glm::dvec2 high{0.0};
};

std::vector<road> roads_by(layout const &Layout)
{
	std::vector<road> result;
	glm::dvec2 low{std::numeric_limits<double>::max()};
	glm::dvec2 high{std::numeric_limits<double>::lowest()};
	for (auto const &path : Layout.paths)
		for (auto const &point : path.points)
		{
			low = glm::min(low, glm::dvec2{point.x, point.z});
			high = glm::max(high, glm::dvec2{point.x, point.z});
		}
	low -= glm::dvec2{50.0};
	high += glm::dvec2{50.0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || (track->iCategoryFlag & 2) == 0 || track->m_editorremoved)
			continue;
		for (auto const &segment : track->m_paths)
		{
			bezier const curve{segment};
			auto const length{curve.plan_length(16)};
			if (false == std::isfinite(length))
				continue;
			auto const count{std::clamp(static_cast<int>(length / 2.0), 2, 64)};
			road entry;
			entry.width = std::max(track->fTrackWidth, track->fTrackWidth2);
			entry.low = glm::dvec2{std::numeric_limits<double>::max()};
			entry.high = glm::dvec2{std::numeric_limits<double>::lowest()};
			for (int i = 0; i <= count; ++i)
			{
				entry.points.push_back(curve.point(static_cast<double>(i) / count));
				entry.low = glm::min(entry.low, glm::dvec2{entry.points.back().x, entry.points.back().z});
				entry.high = glm::max(entry.high, glm::dvec2{entry.points.back().x, entry.points.back().z});
			}
			if (entry.low.x <= high.x && entry.high.x >= low.x && entry.low.y <= high.y && entry.high.y >= low.y)
				result.push_back(std::move(entry));
		}
	}
	return result;
}

double cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

// the stretches along the track the roads crossing it on its level take, from the edge to the edge of each, in order
std::vector<std::pair<double, double>> crossings_of(std::vector<glm::dvec3> const &Points, std::vector<double> const &Distances, std::vector<road> const &Roads)
{
	std::vector<std::pair<double, double>> result;
	glm::dvec2 low{std::numeric_limits<double>::max()};
	glm::dvec2 high{std::numeric_limits<double>::lowest()};
	for (auto const &point : Points)
	{
		low = glm::min(low, glm::dvec2{point.x, point.z});
		high = glm::max(high, glm::dvec2{point.x, point.z});
	}
	for (auto const &road : Roads)
	{
		if (road.low.x > high.x || road.high.x < low.x || road.low.y > high.y || road.high.y < low.y)
			continue;
		for (std::size_t k = 1; k < road.points.size(); ++k)
		{
			glm::dvec2 const r0{road.points[k - 1].x, road.points[k - 1].z};
			glm::dvec2 const across{glm::dvec2{road.points[k].x, road.points[k].z} - r0};
			if (glm::dot(across, across) < 1e-9)
				continue;
			for (std::size_t i = 1; i < Points.size(); ++i)
			{
				glm::dvec2 const p0{Points[i - 1].x, Points[i - 1].z};
				glm::dvec2 const along{glm::dvec2{Points[i].x, Points[i].z} - p0};
				auto const determinant{cross(along, across)};
				if (std::abs(determinant) < 1e-9)
					continue;
				auto const u{cross(r0 - p0, across) / determinant};
				auto const t{cross(r0 - p0, along) / determinant};
				if (u < 0.0 || u > 1.0 || t < 0.0 || t > 1.0)
					continue;
				if (std::abs(glm::mix(road.points[k - 1].y, road.points[k].y, t) - glm::mix(Points[i - 1].y, Points[i].y, u)) > kLevel)
					continue;
				auto const sine{std::abs(determinant) / (glm::length(along) * glm::length(across))};
				auto const half{road.width * 0.5 / std::max(sine, kSkew)};
				auto const at{Distances[i - 1] + (Distances[i] - Distances[i - 1]) * u};
				result.emplace_back(at - half, at + half);
			}
		}
	}
	std::sort(result.begin(), result.end());
	std::vector<std::pair<double, double>> merged;
	for (auto const &part : result)
		if (false == merged.empty() && part.first <= merged.back().second)
			merged.back().second = std::max(merged.back().second, part.second);
		else
			merged.push_back(part);
	return merged;
}

} // namespace

std::vector<lengths> track_lengths(layout const &Layout, std::vector<numbered> const &Tracks, std::vector<signal> const &Signals)
{
	std::vector<lengths> result;
	auto const roads{roads_by(Layout)};
	for (auto const &track : Tracks)
	{
		std::vector<TTrack *> own;
		std::unordered_map<TTrack const *, std::size_t> circuit_of;
		for (std::size_t circuit = 0; circuit < track.circuits.size(); ++circuit)
			for (auto *part : track.circuits[circuit])
				if (false == node(*part) && circuit_of.emplace(part, circuit).second)
					own.push_back(part);
		auto const ends{ends_out(own)};
		if (ends.empty())
			continue;
		// from the end at the lower chainage along its tracks, through the switches between them: the stretches between the switches
		// and the joints, each between the end of one bound and the start of the next
		auto const first{std::min_element(ends.begin(), ends.end(), [&](track_end const &A, track_end const &B) { return project(Layout, A.point).x < project(Layout, B.point).x; })};
		std::unordered_set<TTrack const *> const members(own.begin(), own.end());
		std::unordered_set<TTrack const *> taken;
		std::vector<glm::dvec3> points;
		std::vector<double> distances;
		std::vector<std::pair<TTrack const *, int>> paths;
		std::vector<std::pair<bound, bound>> bounds; // of the stretch before, of the one after
		double within{0.0};
		auto const add = [&](TTrack const &Track, int const Path, bool const Atend) {
			double length{0.0};
			auto const along{from_end(Track, Path, Atend, length)};
			for (std::size_t i = points.empty() ? 0 : 1; i < along.size(); ++i)
			{
				distances.push_back(points.empty() ? 0.0 : distances.back() + plan_distance(points.back(), along[i]));
				points.push_back(along[i]);
			}
			paths.emplace_back(&Track, Path);
		};
		auto const low{out_of(*first->track, first->atend)};
		{
			bound after;
			after.node = low.node;
			after.open = low.open;
			if (low.fouling.has_value())
				after.fouling = -*low.fouling;
			after.start = -low.out;
			bounds.emplace_back(bound{}, after);
		}
		auto *at{first->track};
		auto atend{first->atend};
		taken.insert(at);
		add(*at, 0, atend);
		while (true)
		{
			auto const way{way_on(*at, false == atend, members, taken, kReach)};
			if (way.empty())
				break;
			if (way.size() == 1 && circuit_of[way.front().track] == circuit_of[at])
			{
				at = way.front().track;
				atend = way.front().atend;
				taken.insert(at);
				add(*at, 0, atend);
				continue;
			}
			bound before;
			bound after;
			before.joint = distances.back();
			for (std::size_t i = 0; i < way.size(); ++i)
			{
				auto const &step{way[i]};
				auto const from{distances.back()};
				if (i + 1 == way.size())
					after.joint = from;
				add(*step.track, step.path, step.atend);
				if (false == node(*step.track))
					continue;
				within += distances.back() - from;
				if (before.node == nullptr)
				{
					before.node = step.track;
					before.start = from;
					if (auto const out{fouling(*step.track, step.path, step.atend)})
						before.fouling = from - *out;
				}
				after.node = step.track;
				after.start = distances.back();
				after.fouling.reset();
				if (auto const out{fouling(*step.track, step.path, false == step.atend)})
					after.fouling = distances.back() + *out;
			}
			before.open = after.open = before.node == nullptr;
			bounds.emplace_back(before, after);
			at = way.back().track;
			atend = way.back().atend;
			taken.insert(at);
		}
		auto const length{distances.back()};
		auto const high{out_of(*at, false == atend)};
		{
			bound before;
			before.node = high.node;
			before.open = high.open;
			before.joint = length;
			if (high.fouling.has_value())
				before.fouling = length + *high.fouling;
			before.start = length + high.out;
			bounds.emplace_back(before, bound{});
		}
		// the level crossings within the stretches part them
		for (auto const &[from, to] : crossings_of(points, distances, roads))
		{
			auto const middle{(from + to) * 0.5};
			for (std::size_t index = 0; index + 1 < bounds.size(); ++index)
				if (middle > bounds[index].second.joint && middle < bounds[index + 1].first.joint)
				{
					bound before;
					before.crossing = true;
					before.joint = from;
					bound after;
					after.crossing = true;
					after.joint = to;
					bounds.insert(bounds.begin() + index + 1, {before, after});
					break;
				}
		}
		lengths entry;
		entry.number = track.number;
		entry.construction = length + (low.open ? 0.0 : low.out) + (high.open ? 0.0 : high.out) - within;
		for (std::size_t i = low.points.size(); i-- > 1;)
		{
			entry.points.push_back(low.points[i]);
			entry.distances.push_back(-low.distances[i]);
		}
		entry.points.insert(entry.points.end(), points.begin(), points.end());
		entry.distances.insert(entry.distances.end(), distances.begin(), distances.end());
		for (std::size_t i = 1; i < high.points.size(); ++i)
		{
			entry.points.push_back(high.points[i]);
			entry.distances.push_back(length + high.distances[i]);
		}
		// the signals by it: nearer to it than to any other track, facing along it
		struct stop
		{
			double along{0.0};
			int facing{1};
			std::string name;
		};
		std::vector<stop> stops;
		for (auto const &signal : Signals)
		{
			glm::dvec2 const point{signal.point.x, signal.point.z};
			auto gap{std::numeric_limits<double>::max()};
			double along{0.0};
			glm::dvec2 tangent{0.0, 1.0};
			for (std::size_t i = 1; i < points.size(); ++i)
			{
				glm::dvec2 const a{points[i - 1].x, points[i - 1].z};
				glm::dvec2 const segment{glm::dvec2{points[i].x, points[i].z} - a};
				auto const length2{glm::dot(segment, segment)};
				if (length2 < 1e-12)
					continue;
				auto const t{std::clamp(glm::dot(point - a, segment) / length2, 0.0, 1.0)};
				if (auto const distance{glm::distance(point, a + segment * t)}; distance < gap)
				{
					gap = distance;
					along = distances[i - 1] + (distances[i] - distances[i - 1]) * t;
					tangent = segment / std::sqrt(length2);
				}
			}
			if (gap > kBeside || std::any_of(Layout.paths.begin(), Layout.paths.end(), [&](path const &Path) {
				    return std::none_of(paths.begin(), paths.end(), [&](auto const &Own) { return Own.first == Path.track && Own.second == Path.index; }) && plan_gap(signal.point, Path.points) < gap - 0.01;
			    }))
				continue;
			auto const cosine{glm::dot(glm::normalize(glm::dvec2{signal.travel.x, signal.travel.z} + glm::dvec2{1e-12, 0.0}), tangent)};
			if (std::abs(cosine) >= kFacing)
				stops.push_back({along, cosine > 0.0 ? 1 : -1, signal.name});
		}
		// each way the longest: from a signal facing the trains back to the end of its stretch behind them, else of the stretches
		// between the fouling points
		for (int way = 0; way < 2; ++way)
		{
			auto const forward{way == 0};
			auto const inward{forward ? 1.0 : -1.0};
			auto &part{entry.ways[way]};
			part.length = -1.0;
			for (std::size_t index = 0; index + 1 < bounds.size(); ++index)
			{
				auto const &before{bounds[index].second};
				auto const &after{bounds[index + 1].first};
				auto const rear{forward ? limit_at(before, inward, true) : limit_at(after, inward, true)};
				for (auto const &stop : stops)
				{
					if (stop.facing != (forward ? 1 : -1) || stop.along < before.joint || stop.along > after.joint)
						continue;
					auto const length{(stop.along - rear.along) * inward};
					if (length > 0.0 && length > part.length)
					{
						part.rear = rear;
						part.head.what = limit::kind::signal;
						part.head.name = stop.name;
						part.head.along = stop.along;
						part.length = length;
					}
				}
			}
			if (part.length < 0.0)
				for (std::size_t index = 0; index + 1 < bounds.size(); ++index)
				{
					auto const lower{limit_at(bounds[index].second, 1.0, false)};
					auto const upper{limit_at(bounds[index + 1].first, -1.0, false)};
					if (auto const length{upper.along - lower.along}; length > part.length)
					{
						part.rear = forward ? lower : upper;
						part.head = forward ? upper : lower;
						part.length = length;
					}
				}
			part.length = std::max(0.0, part.length);
			part.head.point = point_along(entry.points, entry.distances, part.head.along);
			part.rear.point = point_along(entry.points, entry.distances, part.rear.along);
		}
		result.push_back(std::move(entry));
	}
	return result;
}

namespace
{

double constexpr kStraight{5000.0}; // m, a path curved less goes for a straight one
int constexpr kPieces{20000}; // of the tracks passed by the routes of a signal, at most

struct reach
{
	TTrack *track{nullptr};
	int path{0};
	int facing{1};
	double from{0.0}; // m, along the path from its start, where the route comes in
	double length{0.0}; // of the route up to there
	double speed{0.0};
	int warning{-1};
	std::vector<TTrack *> turnouts;
	std::vector<std::pair<TTrack const *, int>> passed;
};

} // namespace

double path_speed(TTrack const &Track, int const Path)
{
	bezier const curve{Track.m_paths[Path]};
	int constexpr count{32};
	int constexpr span{4};
	auto least{std::numeric_limits<double>::max()};
	for (int i = 0; i + 2 * span <= count; ++i)
	{
		auto const a{map_of(curve.point(static_cast<double>(i) / count))};
		auto const b{map_of(curve.point(static_cast<double>(i + span) / count))};
		auto const c{map_of(curve.point(static_cast<double>(i + 2 * span) / count))};
		auto const twice{std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))};
		if (twice > 1e-9)
			least = std::min(least, glm::distance(a, b) * glm::distance(b, c) * glm::distance(c, a) / (2.0 * twice));
	}
	return least > kStraight ? 0.0 : std::sqrt(8.424 * least);
}

std::vector<std::vector<route>> routes(std::vector<post> const &Posts, std::vector<std::size_t> const &From, double const Limit)
{
	std::vector<std::vector<route>> all;
	struct along
	{
		std::vector<glm::dvec3> points;
		std::vector<double> distances;
	};
	std::map<std::pair<TTrack const *, int>, along> paths;
	auto const along_of = [&](TTrack const &Track, int const Path) -> along const & {
		auto &found{paths[{&Track, Path}]};
		if (found.points.empty())
		{
			double length{0.0};
			found.points = points_of(Track, Path, length);
			found.distances.assign(1, 0.0);
			for (std::size_t k = 1; k < found.points.size(); ++k)
				found.distances.push_back(found.distances.back() + plan_distance(found.points[k - 1], found.points[k]));
		}
		return found;
	};
	auto const project_on = [&](along const &Path, glm::dvec3 const &Point, double &Gap) {
		auto best{std::numeric_limits<double>::max()};
		double at{0.0};
		for (std::size_t k = 0; k + 1 < Path.points.size(); ++k)
		{
			auto const a{map_of(Path.points[k])};
			auto const ab{map_of(Path.points[k + 1]) - a};
			auto const length2{glm::dot(ab, ab)};
			auto const t{length2 > 1e-12 ? std::clamp(glm::dot(map_of(Point) - a, ab) / length2, 0.0, 1.0) : 0.0};
			if (auto const distance{glm::distance(a + ab * t, map_of(Point))}; distance < best)
			{
				best = distance;
				at = Path.distances[k] + (Path.distances[k + 1] - Path.distances[k]) * t;
			}
		}
		Gap = best;
		return at;
	};
	std::unordered_map<TTrack const *, std::vector<std::size_t>> by_track;
	for (std::size_t index = 0; index < Posts.size(); ++index)
		if (Posts[index].track != nullptr)
			by_track[Posts[index].track].push_back(index);
	auto const routes_from = [&](std::size_t const Start) {
		std::vector<route> result;
		auto const &start{Posts[Start]};
		auto path{0};
		auto nearest{std::numeric_limits<double>::max()};
		double from{0.0};
		for (int candidate = 0; candidate < static_cast<int>(start.track->m_paths.size()); ++candidate)
		{
			double gap{0.0};
			auto const at{project_on(along_of(*start.track, candidate), start.point, gap)};
			if (gap < nearest - 1e-3)
			{
				nearest = gap;
				path = candidate;
				from = at;
			}
		}
		std::vector<reach> stack{{start.track, path, start.facing, from, 0.0, 0.0, -1, {}, {}}};
		auto pieces{0};
		while (false == stack.empty() && pieces++ < kPieces)
		{
			auto here{std::move(stack.back())};
			stack.pop_back();
			auto const &track{*here.track};
			auto const &line{along_of(track, here.path)};
			auto const total{line.distances.back()};
			here.passed.emplace_back(&track, here.path * 2 + (here.facing > 0 ? 1 : 0));
			if (node(track))
			{
				auto const speed{path_speed(track, here.path)};
				auto limit{speed};
				auto const set{track.SwitchExtension != nullptr ? std::abs(static_cast<double>(track.SwitchExtension->fVelocity)) : here.track->VelocityGet()};
				if (speed > 0.0 && set >= 2.0)
					limit = std::min(speed, set);
				if (limit > 0.0 && (here.speed <= 0.0 || limit < here.speed))
					here.speed = limit;
				if (speed > 0.0 && (track.eType != tt_Switch || here.path != straighter(track)))
					here.turnouts.push_back(here.track);
			}
			// the signals ahead on the path, the nearest first
			std::vector<std::pair<double, std::size_t>> ahead;
			if (auto const found{by_track.find(&track)}; found != by_track.end())
				for (auto const index : found->second)
				{
					auto const &other{Posts[index]};
					if (index == Start || other.facing != here.facing || other.kind == signal::role::shunting)
						continue;
					double gap{0.0};
					auto const at{project_on(line, other.point, gap)};
					auto const distance{(at - here.from) * here.facing};
					if (gap < 4.5 && distance > 0.1)
						ahead.emplace_back(distance, index);
				}
			std::sort(ahead.begin(), ahead.end());
			auto stopped{false};
			for (auto const &[distance, index] : ahead)
			{
				if (here.length + distance > Limit)
					break;
				if (Posts[index].kind == signal::role::warning)
				{
					if (here.warning < 0)
						here.warning = static_cast<int>(index);
					continue;
				}
				result.push_back({static_cast<int>(index), here.warning, here.length + distance, here.speed, here.turnouts, false});
				stopped = true;
				break;
			}
			if (stopped)
				continue;
			auto const length{here.length + (here.facing > 0 ? total - here.from : here.from)};
			if (length > Limit)
			{
				result.push_back({-1, here.warning, Limit, here.speed, here.turnouts, false});
				continue;
			}
			auto const atend{here.facing > 0};
			auto const point{end_of(track, here.path, atend)};
			auto *next{neighbour(track, here.path, atend)};
			auto went{false};
			if (next != nullptr && rail(*next))
				for (int other = 0; other < static_cast<int>(next->m_paths.size()); ++other)
					for (auto const end : {false, true})
					{
						if (glm::distance(end_of(*next, other, end), point) >= same_point)
							continue;
						auto const facing{end ? -1 : 1};
						if (std::find(here.passed.begin(), here.passed.end(), std::make_pair(static_cast<TTrack const *>(next), other * 2 + (facing > 0 ? 1 : 0))) != here.passed.end())
							continue;
						auto const &entered{along_of(*next, other)};
						stack.push_back({next, other, facing, end ? entered.distances.back() : 0.0, length, here.speed, here.warning, here.turnouts, here.passed});
						went = true;
					}
			if (false == went)
				result.push_back({-1, here.warning, length, here.speed, here.turnouts, true});
		}
		return result;
	};
	for (auto const from : From)
		all.push_back(from < Posts.size() && Posts[from].track != nullptr && false == Posts[from].track->m_paths.empty() ? routes_from(from) : std::vector<route>{});
	return all;
}

} // namespace station
