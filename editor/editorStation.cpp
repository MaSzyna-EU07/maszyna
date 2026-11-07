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
#include <format>
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
			if (std::ranges::none_of(result, [&](auto const &End) { return glm::distance(end_of(Track, End.first, End.second), point) < same_point; }))
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
	while (next != nullptr && result.length <= Limit && rail(*next))
	{
		if (node(*next))
		{
			result.reached = next;
			return result;
		}
		int path{0};
		bool atend{false};
		if (false == end_at(*next, point, path, atend) || std::ranges::find(result.tracks, next) != result.tracks.end())
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

// the switch the station of the track is looked for from: the track itself, or the nearest switch along it within the link
TTrack *first_switch(TTrack &From, options const &Options)
{
	if (node(From))
		return &From;
	TTrack *first{nullptr};
	auto nearest{Options.link};
	for (auto const atend : {false, true})
		if (auto const line{follow(From, 0, atend, Options.link)}; line.reached != nullptr && line.length < nearest)
		{
			nearest = line.length;
			first = line.reached;
		}
	return first;
}

// the tracks not known yet taken for plain tracks of the station
void add_plain(std::vector<TTrack *> const &Tracks, std::unordered_set<TTrack const *> &Known, std::vector<TTrack *> &Plain)
{
	for (auto *track : Tracks)
		if (Known.insert(track).second)
			Plain.push_back(track);
}

// a link longer than the limit between two switches of the station is a track of it, not two lines out
void drop_links(area &Result, std::unordered_set<TTrack const *> &Known)
{
	auto exits{std::move(Result.exits)};
	Result.exits.clear();
	for (auto &line : exits)
	{
		if (false == line.ended)
			if (auto const whole{follow(*line.from, line.path, line.atend, std::numeric_limits<double>::max())}; whole.reached != nullptr && Known.contains(whole.reached))
			{
				add_plain(whole.tracks, Known, Result.plain);
				continue;
			}
		if (std::ranges::none_of(line.tracks, [&](TTrack const *Track) { return Known.contains(Track); }))
			Result.exits.push_back(std::move(line));
	}
}

area collect(TTrack &From, options const &Options)
{
	area result;
	if (false == rail(From))
		return result;
	auto *first{first_switch(From, Options)};
	if (first == nullptr)
		return result;
	std::unordered_set<TTrack const *> known{first};
	result.switches.push_back(first);
	std::size_t index{0};
	while (index < result.switches.size())
	{
		auto const &from{*result.switches[index]};
		++index;
		for (auto const &[path, atend] : ends_of(from))
		{
			auto line{follow(from, path, atend, Options.link)};
			if (line.reached == nullptr)
			{
				(line.ended ? result.ends : result.exits).push_back(std::move(line));
				continue;
			}
			add_plain(line.tracks, known, result.plain);
			if (line.reached != nullptr && known.insert(line.reached).second)
				result.switches.push_back(line.reached);
		}
	}
	drop_links(result, known);
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
	std::vector result{end_of(From, Path, Atend)};
	auto const *next{neighbour(From, Path, Atend)};
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
			std::ranges::reverse(points);
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
	std::vector result{&Track};
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
			std::ranges::reverse(mine);
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

// the tracks the runs through the station are tried from: the first ones of the lines out and of the tracks ending free, else a
// switch
std::vector<TTrack *> starts_of(area const &Found)
{
	std::vector<TTrack *> starts;
	for (auto const *lines : {&Found.exits, &Found.ends})
		for (auto const &line : *lines)
			if (false == line.tracks.empty())
				starts.push_back(line.tracks.front());
	if (starts.empty())
		starts.push_back(Found.switches.front());
	return starts;
}

// the run through the station along the main paths of its switches which passes the most of them, the longest of those
editor_track::route main_route(std::vector<TTrack *> const &Starts, std::unordered_set<TTrack const *> const &Switches)
{
	editor_track::route best;
	std::size_t most{0};
	double longest{0.0};
	for (auto *start : Starts)
	{
		auto route{editor_track::run_route(*start, 20000.0)};
		std::size_t passed{0};
		double length{0.0};
		for (auto const &span : route.spans)
		{
			passed += Switches.count(span.track);
			length += plan_length(*span.track, span.path);
		}
		if (passed > most || (passed == most && length > longest))
		{
			most = passed;
			longest = length;
			best = std::move(route);
		}
	}
	return best;
}

// the points of the axis along the route, each place once
std::vector<glm::dvec3> axis_of(editor_track::route const &Route)
{
	std::vector<glm::dvec3> axis;
	for (auto const &span : Route.spans)
	{
		double length{0.0};
		auto points{points_of(*span.track, span.path, length)};
		if (false == span.forward)
			std::ranges::reverse(points);
		for (auto const &point : points)
			if (axis.empty() || plan_distance(axis.back(), point) > 0.01)
				axis.push_back(point);
	}
	return axis;
}

// the chainage of the points of the axis
void measure(layout &Layout)
{
	Layout.chainage.assign(1, 0.0);
	for (std::size_t i = 1; i < Layout.axis.size(); ++i)
		Layout.chainage.push_back(Layout.chainage.back() + plan_distance(Layout.axis[i - 1], Layout.axis[i]));
}

// the km of the switch, in m: of the kilometrage of the line, of its record, of the alignment
std::optional<double> km_of_switch(TTrack const &Switch)
{
	auto const start{end_of(Switch, 0, false)};
	double line_km{0.0};
	if (auto const line{kilometrage::line_at(start)}; false == line.empty() && kilometrage::km_at(line, start, line_km))
		return line_km * 1000.0;
	if (auto const record{switch_store::named(Switch)}; record.has_value() && record->km.has_value())
		return record->km;
	return alignment_store::chainage_at(start);
}

// the chainage and the km of the switches which have one, in m
std::vector<std::pair<double, double>> km_marks(layout const &Layout, std::vector<TTrack *> const &Switches)
{
	std::vector<std::pair<double, double>> marks;
	for (auto const *track : Switches)
		if (auto const km{km_of_switch(*track)}; km.has_value())
			marks.emplace_back(project(Layout, end_of(*track, 0, false)).x, *km);
	return marks;
}

// the km at chainage 0 the middle one of the marks gives, and how many of the marks agree with it
struct fit
{
	std::size_t agree{0};
	double origin{0.0};
};

// Reversed: the chainage runs against the km
fit fit_of(std::vector<std::pair<double, double>> const &Marks, double const Length, bool const Reversed)
{
	double constexpr kAgree{50.0}; // m
	std::vector<double> origins;
	for (auto const &[chainage, km] : Marks)
		origins.push_back(km - (Reversed ? Length - chainage : chainage));
	std::ranges::nth_element(origins, origins.begin() + origins.size() / 2);
	fit best_fit{0, origins[origins.size() / 2]};
	for (auto const origin : origins)
		best_fit.agree += std::abs(origin - best_fit.origin) <= kAgree ? 1 : 0;
	return best_fit;
}

// the chainage runs with the km of the switches, when they have one and most of them agree on it
void follow_km(layout &Layout, std::vector<std::pair<double, double>> const &Marks)
{
	if (Marks.empty())
		return;
	auto const length{Layout.chainage.back()};
	auto const along{fit_of(Marks, length, false)};
	auto const against{fit_of(Marks, length, true)};
	auto const reversed{against.agree > along.agree};
	auto const &chosen{reversed ? against : along};
	if (chosen.agree * 2 < Marks.size())
		return;
	if (reversed)
	{
		std::ranges::reverse(Layout.axis);
		measure(Layout);
	}
	Layout.kilometrage = true;
	Layout.km = chosen.origin / 1000.0;
}

// a long track ending free far beyond the switches is a line cut off by the edge of the scenery, else a siding
void sort_ends(layout const &Layout, area const &Found, options const &Options, std::vector<chain> &Exits, std::vector<TTrack *> &Plain)
{
	auto low{std::numeric_limits<double>::max()};
	auto high{std::numeric_limits<double>::lowest()};
	for (auto const *track : Found.switches)
	{
		auto const along{project(Layout, end_of(*track, 0, false)).x};
		low = std::min(low, along);
		high = std::max(high, along);
	}
	for (auto const &line : Found.ends)
	{
		auto beyond{0.0};
		for (auto const *track : line.tracks)
			for (auto const atend : {false, true})
			{
				auto const at{project(Layout, end_of(*track, 0, atend))};
				beyond = std::max({beyond, low - at.x, at.x - high});
			}
		if (beyond > Options.approach * 0.5 && line.length > Options.approach)
			Exits.push_back(line);
		else
			Plain.insert(Plain.end(), line.tracks.begin(), line.tracks.end());
	}
}

// the paths of the track, laid out along the axis
void add_paths(layout &Layout, TTrack &Track, bool const Approach)
{
	for (int index = 0; index < static_cast<int>(Track.m_paths.size()); ++index)
	{
		path entry;
		entry.track = &Track;
		entry.index = index;
		entry.approach = Approach;
		entry.points = points_of(Track, index, entry.length);
		for (auto const &point : entry.points)
			entry.schematic.push_back(project(Layout, point));
		Layout.paths.push_back(std::move(entry));
	}
}

// the lines leading out, as far as the approach
void add_lines(layout &Layout, std::vector<chain> const &Exits, options const &Options)
{
	for (auto const &line : Exits)
	{
		double line_length{0.0};
		Layout.lines.emplace_back();
		for (auto *track : line.tracks)
		{
			if (line_length >= Options.approach)
				break;
			add_paths(Layout, *track, true);
			Layout.lines.back().push_back(track);
			line_length += plan_length(*track, 0);
		}
	}
}

// the chainage over which the switches lie, the offsets of the outermost tracks of the station
void span_of(layout &Layout, std::unordered_set<TTrack const *> const &Switches)
{
	Layout.from = std::numeric_limits<double>::max();
	Layout.to = std::numeric_limits<double>::lowest();
	Layout.low = std::numeric_limits<double>::max();
	Layout.high = std::numeric_limits<double>::lowest();
	for (auto const &entry : Layout.paths)
	{
		if (entry.approach)
			continue;
		for (auto const &point : entry.schematic)
		{
			if (Switches.contains(entry.track))
			{
				Layout.from = std::min(Layout.from, point.x);
				Layout.to = std::max(Layout.to, point.x);
			}
			Layout.low = std::min(Layout.low, point.y);
			Layout.high = std::max(Layout.high, point.y);
		}
	}
}

// where each line leaves the station: its point in the schematic nearest to 60 m beyond the switches
std::vector<std::pair<std::size_t, glm::dvec2>> leaving_of(layout const &Layout)
{
	std::vector<std::pair<std::size_t, glm::dvec2>> leaving;
	auto const beyond = [&](glm::dvec2 const &Point) { return std::abs(std::max(Layout.from - Point.x, Point.x - Layout.to) - 60.0); };
	for (std::size_t index = 0; index < Layout.lines.size(); ++index)
	{
		auto const &tracks{Layout.lines[index]};
		std::vector<glm::dvec2> points;
		for (auto const &path : Layout.paths)
			if (path.approach && std::ranges::find(tracks, path.track) != tracks.end())
				points.insert(points.end(), path.schematic.begin(), path.schematic.end());
		if (false == points.empty())
			leaving.emplace_back(index, *std::ranges::min_element(points, [&](glm::dvec2 const &A, glm::dvec2 const &B) { return beyond(A) < beyond(B); }));
	}
	return leaving;
}

// the other main track: the run in from the end of its line at the station, along the main paths of the switches
void add_second(layout &Layout, TTrack &First, std::unordered_set<TTrack const *> const &Switches, std::unordered_set<TTrack const *> const &Axis, options const &Options)
{
	auto const *start{neighbour(First, 0, false)};
	auto const inwards{start == nullptr || false == Switches.contains(start)};
	for (auto *track : straight_run(First, end_of(First, 0, inwards), Layout.chainage.back() + 2.0 * Options.approach))
		if (false == Axis.contains(track) && std::ranges::find(Layout.second, track) == Layout.second.end())
			Layout.second.push_back(track);
}

// a double-track line: another line leaves the station by the one along the axis a few metres aside; the run from it along the
// main paths of the switches is the other main track
void find_second(layout &Layout, std::unordered_set<TTrack const *> const &Switches, options const &Options)
{
	std::unordered_set<TTrack const *> const axis(Layout.main.begin(), Layout.main.end());
	auto const leaving{leaving_of(Layout)};
	for (auto const &[along, at] : leaving)
	{
		if (false == axis.contains(Layout.lines[along].front()) || false == Layout.second.empty())
			continue;
		for (auto const &[aside, there] : leaving)
		{
			if (aside == along || axis.contains(Layout.lines[aside].front()) || (at.x < Layout.from) != (there.x < Layout.from) || std::abs(there.y - at.y) < 3.0 || std::abs(there.y - at.y) > 7.0)
				continue;
			// the run goes in from the end of the line at the station
			add_second(Layout, *Layout.lines[aside].front(), Switches, axis, Options);
			break;
		}
	}
}

} // namespace

std::optional<layout> find(TTrack &From, options const &Options)
{
	auto const found{collect(From, Options)};
	if (found.switches.empty())
		return std::nullopt;
	std::unordered_set<TTrack const *> const switches(found.switches.begin(), found.switches.end());
	// the axis: the run through the station along the main paths of its switches which passes the most of them
	auto const best{main_route(starts_of(found), switches)};
	layout result;
	result.axis = axis_of(best);
	if (result.axis.size() < 2)
		return std::nullopt;
	measure(result);
	// the chainage runs with the km of the switches, when they have one and most of them agree on it: of the kilometrage of the
	// line, of their records, of the alignment
	follow_km(result, km_marks(result, found.switches));
	// without the km the station runs from the west to the east, as the map shows it
	if (false == result.kilometrage && map_of(result.axis.back()).x < map_of(result.axis.front()).x)
	{
		std::ranges::reverse(result.axis);
		measure(result);
	}
	// a long track ending free far beyond the switches is a line cut off by the edge of the scenery, else a siding
	std::vector<chain> exits{found.exits};
	std::vector<TTrack *> plain{found.plain};
	sort_ends(result, found, Options, exits, plain);
	// the paths, laid out along the axis
	for (auto *track : found.switches)
		add_paths(result, *track, false);
	for (auto *track : plain)
		add_paths(result, *track, false);
	add_lines(result, exits, Options);
	span_of(result, switches);
	result.switches = found.switches;
	std::ranges::sort(result.switches, [&](TTrack const *A, TTrack const *B) { return project(result, end_of(*A, 0, false)).x < project(result, end_of(*B, 0, false)).x; });
	// a double-track line: another line leaves the station by the one along the axis a few metres aside; the run from it along
	// the main paths of the switches is the other main track
	for (auto const &span : best.spans)
		result.main.push_back(span.track);
	find_second(result, switches, Options);
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

namespace
{

// a track rising more than this to the chainage runs across the rows
double constexpr kAcross{0.04};
// m, two tracks this far apart lie side by side
double constexpr kSideBySide{1.0};
// m of the chainage, points of two tracks compared side by side
double constexpr kAbreast{3.0};
// m, a plain stretch between the switches keeps this much of its length in the plot, the rest shortened to kKept of it
double constexpr kWhole{60.0};
double constexpr kKept{0.3};
// m, how far a slant is followed over the tracks to the row it ends on
double constexpr kSlantSearch{400.0};
// m, the plot puts a point of the schematic on a path no farther from it
double constexpr kNearPath{15.0};

double shortened(double const Length)
{
	return Length <= kWhole ? Length : std::lerp(kWhole, Length, kKept);
}

double lengthened(double const Length)
{
	return Length <= kWhole ? Length : kWhole + (Length - kWhole) / kKept; // NOSONAR: std::lerp would take 1 / kKept, which rounds otherwise than dividing by it
}

// rows, for the offset gone off a row: at most three rows further
double rows_off(double const Steps)
{
	auto const size{std::abs(Steps)};
	return std::copysign(size <= 1.0 ? size : 1.0 + 2.0 * (1.0 - std::exp((1.0 - size) * 0.5)), Steps);
}

// the row of a point off the rows
int constexpr kOffRow{std::numeric_limits<int>::min()};

// the ends of the paths which meet: of each path, at its start and at its end, the other paths with their ends
using met_ends = std::vector<std::array<std::vector<std::pair<std::size_t, bool>>, 2>>;

bool plain_path(std::vector<path> const &Paths, std::size_t const Path)
{
	return Paths[Path].track == nullptr || false == node(*Paths[Path].track);
}

bool usable_path(std::vector<path> const &Paths, std::size_t const Path)
{
	return Paths[Path].points.size() >= 2 && Paths[Path].points.size() == Paths[Path].schematic.size();
}

glm::dvec3 path_end(std::vector<path> const &Paths, std::size_t const Path, bool const Atend)
{
	return Atend ? Paths[Path].points.back() : Paths[Path].points.front();
}

// the ends of the two paths which meet, noted with both
void note_meeting(std::vector<path> const &Paths, std::size_t const P, std::size_t const Q, met_ends &Met)
{
	for (auto const pend : {false, true})
		for (auto const qend : {false, true})
			if (usable_path(Paths, Q) && plan_distance(path_end(Paths, P, pend), path_end(Paths, Q, qend)) < 0.05)
			{
				Met[P][pend].emplace_back(Q, qend);
				Met[Q][qend].emplace_back(P, pend);
			}
}

met_ends met_of(std::vector<path> const &Paths)
{
	auto const count{Paths.size()};
	met_ends met(count);
	for (std::size_t p = 0; p < count; ++p)
		for (std::size_t q = p + 1; q < count && usable_path(Paths, p); ++q)
			note_meeting(Paths, p, q, met);
	return met;
}

// of each path, its points running along the axis rather than across it
std::vector<std::vector<char>> along_axis(std::vector<path> const &Paths)
{
	std::vector<std::vector<char>> along(Paths.size());
	for (std::size_t p = 0; p < Paths.size(); ++p)
	{
		auto const &points{Paths[p].schematic};
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			auto const &a{points[i > 0 ? i - 1 : i]};
			auto const &b{points[i + 1 < points.size() ? i + 1 : i]};
			along[p].push_back(std::abs(b.x - a.x) > 1e-3 && std::abs(b.y - a.y) < std::abs(b.x - a.x) * kAcross);
		}
	}
	return along;
}

// the run of the station track from the path on: the plain paths joined to it end to end where nothing else meets them; Owner: the
// run of each path, -1 for none yet
void gather_run(std::vector<path> const &Paths, met_ends const &Met, std::size_t const From, int const Index, std::vector<int> &Owner, drawing::run &Run)
{
	std::vector stack{From};
	while (false == stack.empty())
	{
		auto const current{stack.back()};
		stack.pop_back();
		Run.paths.push_back(current);
		for (auto const atend : {false, true})
		{
			if (Met[current][atend].size() != 1)
				continue;
			auto const [other, otherend]{Met[current][atend].front()};
			if (plain_path(Paths, other) && Owner[other] < 0 && Met[other][otherend].size() == 1)
			{
				Owner[other] = Index;
				stack.push_back(other);
			}
		}
	}
}

// the station tracks: plain paths joined end to end where nothing else meets them, each on a row of its own; the run of each path,
// -1 for none
std::vector<int> runs_of(std::vector<path> const &Paths, met_ends const &Met, std::vector<drawing::run> &Runs)
{
	std::vector owner(Paths.size(), -1);
	for (std::size_t p = 0; p < Paths.size(); ++p)
	{
		if (owner[p] >= 0 || false == plain_path(Paths, p) || false == usable_path(Paths, p))
			continue;
		auto const index{static_cast<int>(Runs.size())};
		Runs.emplace_back();
		owner[p] = index;
		gather_run(Paths, Met, p, index, owner, Runs.back());
	}
	return owner;
}

// the points of the run along the axis, by the chainage
std::vector<glm::dvec2> samples_of(std::vector<path> const &Paths, std::vector<std::vector<char>> const &Along, drawing::run const &Run)
{
	std::vector<glm::dvec2> samples;
	for (auto const p : Run.paths)
		for (std::size_t i = 0; i < Paths[p].schematic.size(); ++i)
			if (Along[p][i])
				samples.push_back(Paths[p].schematic[i]);
	std::ranges::sort(samples, [](glm::dvec2 const &A, glm::dvec2 const &B) { return A.x < B.x; });
	return samples;
}

// the middle one of the offsets of the samples
double middle_offset(std::vector<glm::dvec2> const &Samples)
{
	std::vector<double> offsets;
	for (auto const &sample : Samples)
		offsets.push_back(sample.y);
	std::ranges::nth_element(offsets, offsets.begin() + offsets.size() / 2);
	return offsets[offsets.size() / 2];
}

// how the first run lies by the second where they run side by side: above 0 on its left, below 0 on its right; the gaps between the
// tracks next to each other noted
int votes_between(std::vector<glm::dvec2> const &A, std::vector<glm::dvec2> const &B, std::vector<double> &Gaps)
{
	auto votes{0};
	for (auto const &sample : A)
	{
		auto const found{std::ranges::lower_bound(B, sample.x - kAbreast, {}, [](glm::dvec2 const &Sample) { return Sample.x; })};
		if (found == B.end() || found->x > sample.x + kAbreast || std::abs(sample.y - found->y) <= kSideBySide)
			continue;
		votes += sample.y > found->y ? 1 : -1;
		// of the tracks next to each other
		if (auto const gap{std::abs(sample.y - found->y)}; gap > 3.0 && gap < 8.0)
			Gaps.push_back(gap);
	}
	return votes;
}

// of each run the runs lying beside it on its right; the gaps between the tracks next to each other noted
std::vector<std::vector<std::size_t>> lying_beside(std::vector<std::vector<glm::dvec2>> const &Samples, std::vector<char> const &Level, std::vector<double> &Gaps)
{
	auto const count{Samples.size()};
	std::vector<std::vector<std::size_t>> below(count);
	for (std::size_t a = 0; a < count; ++a)
		for (std::size_t b = a + 1; b < count; ++b)
		{
			if (0 == Level[a] || 0 == Level[b] || Samples[a].back().x + kAbreast < Samples[b].front().x || Samples[b].back().x + kAbreast < Samples[a].front().x)
				continue;
			auto const votes{votes_between(Samples[a], Samples[b], Gaps)};
			if (votes > 0)
				below[a].push_back(b);
			else if (votes < 0)
				below[b].push_back(a);
		}
	return below;
}

// the rows pushed apart away from the axis where a track lies beside another on the same row or beyond, once: whether any moved
bool push_apart(std::vector<drawing::run> &Runs, std::vector<std::vector<std::size_t>> const &Below)
{
	auto changed{false};
	for (std::size_t a = 0; a < Runs.size(); ++a)
		for (auto const b : Below[a])
		{
			if (Runs[a].row > Runs[b].row)
				continue;
			if (Runs[a].offset + Runs[b].offset >= 0.0)
				Runs[a].row = Runs[b].row + 1;
			else
				Runs[b].row = Runs[a].row - 1;
			changed = true;
		}
	return changed;
}

// the main track through the station on the middle row
void centre_main(layout const &Layout, std::vector<int> const &Owner, std::vector<char> const &Level, std::vector<drawing::run> &Runs)
{
	if (Layout.main.empty())
		return;
	for (std::size_t p = 0; p < Owner.size(); ++p)
	{
		if (Owner[p] < 0 || 0 == Level[Owner[p]] || Layout.paths[p].track != Layout.main.front())
			continue;
		auto const shift{Runs[Owner[p]].row};
		for (auto &run : Runs)
			run.row -= shift;
		return;
	}
}

// a group of tracks far aside, as a yard, closer: no more than one row left empty before it
void close_up(std::vector<drawing::run> &Runs, std::vector<char> const &Level)
{
	std::vector<int> used;
	for (std::size_t r = 0; r < Runs.size(); ++r)
		if (Level[r])
			used.push_back(Runs[r].row);
	used.push_back(0);
	std::ranges::sort(used);
	used.erase(std::ranges::unique(used).begin(), used.end());
	std::map<int, int> closer{{0, 0}};
	auto const middle{static_cast<std::size_t>(std::ranges::find(used, 0) - used.begin())};
	for (auto i{middle + 1}; i < used.size(); ++i)
		closer[used[i]] = closer[used[i - 1]] + std::min(used[i] - used[i - 1], 2);
	for (auto i{middle}; i > 0; --i)
		closer[used[i - 1]] = closer[used[i]] - std::min(used[i] - used[i - 1], 2);
	for (std::size_t r = 0; r < Runs.size(); ++r)
		if (Level[r])
			Runs[r].row = closer[Runs[r].row];
}

// the breakpoints at the ends of the switches, the stretches between them shortened
std::vector<glm::dvec2> warp_of(layout const &Layout)
{
	auto const &paths{Layout.paths};
	std::vector<double> stops;
	for (std::size_t p = 0; p < paths.size(); ++p)
		if (false == paths[p].approach && false == plain_path(paths, p) && false == paths[p].schematic.empty())
		{
			stops.push_back(paths[p].schematic.front().x);
			stops.push_back(paths[p].schematic.back().x);
		}
	if (stops.empty())
		stops = {Layout.from, Layout.to};
	std::ranges::sort(stops);
	std::vector<glm::dvec2> warp;
	for (auto const stop : stops)
		if (warp.empty())
			warp.emplace_back(stop, stop);
		else if (stop - warp.back().x > 1.0)
			warp.emplace_back(stop, warp.back().y + shortened(stop - warp.back().x));
	return warp;
}

// the row of each point: the row of its track where it runs along it, kOffRow elsewhere
std::vector<std::vector<int>> rows_on_runs(std::vector<path> const &Paths, std::vector<std::vector<char>> const &Along, std::vector<int> const &Owner, std::vector<char> const &Level, std::vector<drawing::run> const &Runs)
{
	std::vector<std::vector<int>> rows(Paths.size());
	for (std::size_t p = 0; p < Paths.size(); ++p)
	{
		rows[p].assign(Paths[p].schematic.size(), kOffRow);
		if (Owner[p] < 0 || 0 == Level[Owner[p]])
			continue;
		for (std::size_t i = 0; i < rows[p].size(); ++i)
			if (Along[p][i])
				rows[p][i] = Runs[Owner[p]].row;
	}
	return rows;
}

// the row the end of the path takes from the paths meeting it there, kOffRow for none
int joined_row(met_ends const &Met, std::vector<std::vector<int>> const &Rows, std::size_t const Path, bool const Atend)
{
	for (auto const &[other, otherend] : Met[Path][Atend])
		if (auto const value{otherend ? Rows[other].back() : Rows[other].front()}; value != kOffRow)
			return value;
	return kOffRow;
}

// the rows of the points of a switch: going straight on it takes the row of the track it joins, its curved path keeps it at its ends
// only
std::vector<int> wanted_rows(met_ends const &Met, std::vector<std::vector<int>> const &Rows, std::vector<char> const &Along, std::size_t const Path)
{
	auto const straight{std::ranges::all_of(Along, [](char const Point) { return Point != 0; })};
	auto const first{Along.front() ? joined_row(Met, Rows, Path, false) : kOffRow};
	auto const last{Along.back() ? joined_row(Met, Rows, Path, true) : kOffRow};
	std::vector wanted(Rows[Path].size(), kOffRow);
	if (straight && first != kOffRow && (last == kOffRow || last == first))
		std::ranges::fill(wanted, first);
	else if (straight && last != kOffRow && first == kOffRow)
		std::ranges::fill(wanted, last);
	else
	{
		wanted.front() = first;
		wanted.back() = last;
	}
	return wanted;
}

// the rows of the switches taken from the tracks they join, once: whether any changed
bool settle_rows(std::vector<path> const &Paths, met_ends const &Met, std::vector<std::vector<char>> const &Along, std::vector<std::vector<int>> &Rows)
{
	auto changed{false};
	for (std::size_t p = 0; p < Paths.size(); ++p)
	{
		if (plain_path(Paths, p) || false == usable_path(Paths, p))
			continue;
		if (auto wanted{wanted_rows(Met, Rows, Along[p], p)}; wanted != Rows[p])
		{
			Rows[p] = std::move(wanted);
			changed = true;
		}
	}
	return changed;
}

double row_y(int const Row)
{
	return static_cast<double>(Row) * row_height;
}

// the way along the path from its end, the chord of its first metres
glm::dvec2 leaving_way(std::vector<path> const &Paths, std::size_t const Path, bool const Atend)
{
	auto const &points{Paths[Path].points};
	auto const reach{std::min<std::size_t>(5, points.size() - 1)};
	auto const way{Atend ? points[points.size() - 1 - reach] - points.back() : points[reach] - points.front()};
	return glm::normalize(glm::dvec2{way.x, way.z} + glm::dvec2{1e-12, 0.0});
}

// a point on a row in the plot
struct on_row
{
	glm::dvec2 drawn{0.0};
	double offset{0.0}; // in the schematic
};

// the path not seen yet meeting the end of the path which goes on from it the straightest, with the end it is entered at; none
// going on ahead
std::optional<std::pair<std::size_t, bool>> straightest(std::vector<path> const &Paths, met_ends const &Met, std::unordered_set<std::size_t> const &Seen, std::size_t const Path, bool const Atend)
{
	auto const away{-leaving_way(Paths, Path, Atend)};
	auto best{0.0};
	std::optional<std::pair<std::size_t, bool>> found;
	for (auto const &[other, otherend] : Met[Path][Atend])
	{
		if (Seen.contains(other))
			continue;
		if (auto const straightness{glm::dot(leaving_way(Paths, other, otherend), away)}; straightness > best)
		{
			best = straightness;
			found = std::make_pair(other, otherend);
		}
	}
	return found;
}

// the first point on a row along the path from the end it is entered at, the way walked added up; none within the reach
std::optional<on_row> first_on_row(drawing const &Drawing, std::vector<glm::dvec2> const &Schematic, std::vector<int> const &Row, bool const Entered, double &Walked)
{
	auto const size{Row.size()};
	for (std::size_t k = 0; k < size; ++k)
	{
		auto const index{Entered ? size - 1 - k : k};
		if (k > 0)
			Walked += glm::distance(Schematic[index], Schematic[Entered ? index + 1 : index - 1]);
		if (Walked > kSlantSearch)
			return std::nullopt;
		if (Row[index] != kOffRow)
			return on_row{{Drawing.x(Schematic[index].x), row_y(Row[index])}, Schematic[index].y};
	}
	return std::nullopt;
}

// the first point on a row along the tracks on from the end of the path, away from it: at a switch along the path going on the
// straightest, so a slant over a ladder of switches runs to its far end
std::optional<on_row> outward(drawing const &Drawing, std::vector<path> const &Paths, met_ends const &Met, std::vector<std::vector<int>> const &Rows, std::size_t const From, bool const Atend)
{
	auto current{From};
	auto exit{Atend};
	auto walked{0.0};
	std::unordered_set<std::size_t> seen{From};
	while (walked <= kSlantSearch)
	{
		auto const next{straightest(Paths, Met, seen, current, exit)};
		if (false == next.has_value())
			return std::nullopt;
		auto const [onto, entered]{*next};
		seen.insert(onto);
		if (auto const found{first_on_row(Drawing, Paths[onto].schematic, Rows[onto], entered, walked)}; found.has_value())
			return found;
		current = onto;
		exit = false == entered;
	}
	return std::nullopt;
}

// the points of the stretch off the rows between two points on them: on a straight slant from the one to the other; off a row
// towards no other, a line out leaving the station aside
void slant(std::vector<glm::dvec2> &Drawn, std::vector<glm::dvec2> const &Points, std::pair<std::size_t, std::size_t> const Stretch, std::optional<on_row> const &Before, std::optional<on_row> const &After, double const Spacing)
{
	for (auto k{Stretch.first}; k <= Stretch.second; ++k)
	{
		if (Before.has_value() && After.has_value() && std::abs(After->drawn.x - Before->drawn.x) > 0.5)
		{
			auto const t{std::clamp((Drawn[k].x - Before->drawn.x) / (After->drawn.x - Before->drawn.x), 0.0, 1.0)};
			Drawn[k].y = Before->drawn.y + (After->drawn.y - Before->drawn.y) * t;
		}
		// off a row towards no other, a line out leaving the station aside
		else if (Before.has_value() || After.has_value())
		{
			auto const &known{Before.has_value() ? *Before : *After};
			Drawn[k].y = known.drawn.y + row_height * rows_off((Points[k].y - known.offset) / Spacing);
		}
	}
}

// the points of the path off the rows: between them on a straight slant from the last row to the next
void slants(drawing &Drawing, std::vector<path> const &Paths, met_ends const &Met, std::vector<std::vector<int>> const &Rows, std::size_t const Path)
{
	auto const &points{Paths[Path].schematic};
	auto const &row{Rows[Path]};
	auto &drawn{Drawing.paths[Path]};
	std::size_t i{0};
	while (i < points.size())
	{
		if (row[i] != kOffRow)
		{
			++i;
			continue;
		}
		auto end{i};
		while (end + 1 < points.size() && row[end + 1] == kOffRow)
			++end;
		auto const before{i > 0 ? std::optional<on_row>{on_row{drawn[i - 1], points[i - 1].y}} : outward(Drawing, Paths, Met, Rows, Path, false)};
		auto const after{end + 1 < points.size() ? std::optional<on_row>{on_row{drawn[end + 1], points[end + 1].y}} : outward(Drawing, Paths, Met, Rows, Path, true)};
		slant(drawn, points, {i, end}, before, after, Drawing.spacing);
		i = end + 1;
	}
}

// the points in the plot: on their rows, between them on a straight slant from the last row to the next; far from any row a few
// rows beyond the outer ones at most
void draw_paths(drawing &Drawing, std::vector<path> const &Paths, met_ends const &Met, std::vector<std::vector<int>> const &Rows, std::vector<char> const &Level)
{
	auto lowest{0};
	auto highest{0};
	for (std::size_t r = 0; r < Drawing.runs.size(); ++r)
		if (Level[r])
		{
			lowest = std::min(lowest, Drawing.runs[r].row);
			highest = std::max(highest, Drawing.runs[r].row);
		}
	auto const loose = [&](double const Offset) { return std::clamp(Offset / Drawing.spacing, lowest - 3.0, highest + 3.0) * row_height; };
	Drawing.paths.resize(Paths.size());
	for (std::size_t p = 0; p < Paths.size(); ++p)
	{
		auto const &points{Paths[p].schematic};
		auto const &row{Rows[p]};
		auto &drawn{Drawing.paths[p]};
		for (std::size_t i = 0; i < points.size(); ++i)
			drawn.emplace_back(Drawing.x(points[i].x), row[i] != kOffRow ? row_y(row[i]) : loose(points[i].y));
		if (usable_path(Paths, p))
			slants(Drawing, Paths, Met, Rows, p);
	}
}

} // namespace

double drawing::x(double const Chainage) const
{
	if (warp.empty())
		return Chainage;
	if (Chainage <= warp.front().x)
		return warp.front().y - shortened(warp.front().x - Chainage);
	if (Chainage >= warp.back().x)
		return warp.back().y + shortened(Chainage - warp.back().x);
	auto const next{std::ranges::upper_bound(warp, Chainage, {}, [](glm::dvec2 const &Point) { return Point.x; })};
	auto const &a{*(next - 1)};
	auto const &b{*next};
	return a.y + (b.y - a.y) * (Chainage - a.x) / std::max(1e-9, b.x - a.x);
}

double drawing::chainage(double const X) const
{
	if (warp.empty())
		return X;
	if (X <= warp.front().y)
		return warp.front().x - lengthened(warp.front().y - X);
	if (X >= warp.back().y)
		return warp.back().x + lengthened(X - warp.back().y);
	auto const next{std::ranges::upper_bound(warp, X, {}, [](glm::dvec2 const &Point) { return Point.y; })};
	auto const &a{*(next - 1)};
	auto const &b{*next};
	return a.x + (b.x - a.x) * (X - a.y) / std::max(1e-9, b.y - a.y);
}

glm::dvec2 drawing::at(glm::dvec2 const &Schematic) const
{
	auto best{kNearPath * kNearPath};
	glm::dvec2 result{x(Schematic.x), Schematic.y / spacing * row_height};
	if (false == runs.empty())
	{
		auto const [low, high]{std::ranges::minmax_element(runs, [](run const &A, run const &B) { return A.row < B.row; })};
		result.y = std::clamp(result.y, (low->row - 3.0) * row_height, (high->row + 3.0) * row_height);
	}
	for (std::size_t p = 0; p < schematic.size() && p < paths.size(); ++p)
	{
		auto const &points{schematic[p]};
		if (points.empty() || std::max(points.front().x, points.back().x) + 2.0 * kNearPath < Schematic.x || std::min(points.front().x, points.back().x) - 2.0 * kNearPath > Schematic.x)
			continue;
		for (std::size_t i = 1; i < points.size(); ++i)
		{
			auto const &a{points[i - 1]};
			auto const along{points[i] - a};
			auto const length2{glm::dot(along, along)};
			auto const t{length2 > 1e-12 ? std::clamp(glm::dot(Schematic - a, along) / length2, 0.0, 1.0) : 0.0};
			auto const gap{a + along * t - Schematic};
			if (auto const distance2{glm::dot(gap, gap)}; distance2 < best)
			{
				best = distance2;
				result = paths[p][i - 1] + (paths[p][i] - paths[p][i - 1]) * t;
			}
		}
	}
	return result;
}

drawing generalize(layout const &Layout)
{
	drawing result;
	auto const &paths{Layout.paths};
	for (auto const &path : paths)
		result.schematic.push_back(path.schematic);
	auto const met{met_of(paths)};
	auto const along{along_axis(paths)};
	auto const run_of{runs_of(paths, met, result.runs)};
	// of each run the points along the axis, by the chainage: the middle offset of them, which other runs lie beside it on the right
	auto &runs{result.runs};
	std::vector<std::vector<glm::dvec2>> samples(runs.size());
	std::vector<char> level(runs.size(), 0);
	for (std::size_t r = 0; r < runs.size(); ++r)
	{
		samples[r] = samples_of(paths, along, runs[r]);
		if (samples[r].empty())
			continue;
		level[r] = 1;
		runs[r].offset = middle_offset(samples[r]);
	}
	std::vector<double> gaps;
	auto const below{lying_beside(samples, level, gaps)};
	if (false == gaps.empty())
	{
		std::ranges::nth_element(gaps, gaps.begin() + gaps.size() / 2);
		result.spacing = std::clamp(gaps[gaps.size() / 2], 3.5, 12.0);
	}
	// the rows: by the offsets, then pushed apart away from the axis where a track lies beside another on the same row or beyond
	for (auto &run : runs)
		run.row = static_cast<int>(std::lround(run.offset / result.spacing));
	for (std::size_t pass = 0; pass < runs.size() + 1; ++pass)
		if (false == push_apart(runs, below))
			break;
	centre_main(Layout, run_of, level, runs);
	close_up(runs, level);
	result.warp = warp_of(Layout);
	// the row of each point: the row of its track where it runs along it; a switch going straight on takes the row of the track it
	// joins, its curved path keeps it at its ends only
	auto rows{rows_on_runs(paths, along, run_of, level, runs)};
	for (int pass = 0; pass < 32; ++pass)
		if (false == settle_rows(paths, met, along, rows))
			break;
	draw_paths(result, paths, met, rows, level);
	return result;
}

std::vector<TTrack *> stations(options const &Options)
{
	std::vector<TTrack *> result;
	std::unordered_set<TTrack const *> taken;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || false == node(*track) || taken.contains(track))
			continue;
		auto const found{collect(*track, Options)};
		for (auto const *member : found.switches)
			taken.insert(member);
		result.push_back(track);
	}
	return result;
}

namespace
{

// the parts of a slip, a double switch or a crossing its record names, of the station and not taken yet
void record_parts(switch_store::record const &Record, std::unordered_set<TTrack const *> const &Members, std::unordered_set<TTrack const *> &Taken, std::vector<TTrack *> &Parts)
{
	if (Record.kind == "switch")
		return;
	for (auto const &name : Record.parts)
		if (auto *part{simulation::Paths.find(name)}; part != nullptr && Members.contains(part) && Taken.insert(part).second)
			Parts.push_back(part);
}

} // namespace

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
		if (track->eType == tt_Table || taken.contains(track))
			continue;
		unit entry;
		if (auto const record{switch_store::named(*track)}; record.has_value())
		{
			entry.record = record->name;
			record_parts(*record, members, taken, entry.parts);
		}
		if (taken.insert(track).second)
			entry.parts.push_back(track);
		entry.chainage = std::numeric_limits<double>::max();
		for (auto const *part : entry.parts)
			entry.chainage = std::min(entry.chainage, project(Layout, end_of(*part, 0, false)).x);
		result.push_back(std::move(entry));
	}
	std::ranges::stable_sort(result, [&](unit const &A, unit const &B) { return Reversed ? A.chainage > B.chainage : A.chainage < B.chainage; });
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
	auto result{std::format("{}{}", prefix(Code), Number)};
	if (Parts > 1)
		result += static_cast<char>('a' + std::min<std::size_t>(Part, 25));
	return result;
}

namespace
{

// an index of each track: of its unit, of its circuit
using track_map = std::unordered_map<TTrack const *, std::size_t>;

// the units of the station by their parts
track_map units_by_part(std::vector<unit> const &Units)
{
	track_map unit_of;
	for (std::size_t index = 0; index < Units.size(); ++index)
		for (auto const *part : Units[index].parts)
			unit_of[part] = index;
	return unit_of;
}

// the two tracks are parts of the same unit
bool one_unit(track_map const &Units, TTrack const *A, TTrack const *B)
{
	auto const a{Units.find(A)};
	auto const b{Units.find(B)};
	return a != Units.end() && b != Units.end() && a->second == b->second;
}

// the line runs out of the station: to a switch of another one, or along a line cut off by the edge of the scenery
bool leaves(layout const &Layout, chain const &Line)
{
	if (Line.reached != nullptr && std::ranges::find(Layout.switches, Line.reached) == Layout.switches.end())
		return true;
	return Line.ended && false == Line.tracks.empty() && std::ranges::any_of(Layout.lines, [&](auto const &Out) { return false == Out.empty() && Out.front() == Line.tracks.front(); });
}

// the line out kept as far as the reach of the joints, the track reaching beyond it the last
void trim(chain &Line)
{
	double length{0.0};
	auto const kept{std::ranges::find_if(Line.tracks, [&](TTrack const *Track) {
		length += plan_length(*Track, 0);
		return length > kReach * 2.0;
	})};
	Line.tracks.erase(kept == Line.tracks.end() ? kept : std::next(kept), Line.tracks.end());
}

// a link from a switch on as one line, and where each of its tracks ends along it
struct link_line
{
	std::vector<glm::dvec3> points;
	std::vector<double> lengths;
	std::vector<std::pair<double, TTrack *>> pieces;
};

link_line link_of(glm::dvec3 const &Start, std::vector<TTrack *> const &Tracks)
{
	link_line result{{Start}, {0.0}, {}};
	for (auto *track : Tracks)
	{
		double plan{0.0};
		auto samples{dense_points(*track, 0, plan)};
		if (plan_distance(samples.back(), result.points.back()) < plan_distance(samples.front(), result.points.back()))
			std::ranges::reverse(samples);
		for (std::size_t i = 1; i < samples.size(); ++i)
		{
			result.lengths.push_back(result.lengths.back() + plan_distance(result.points.back(), samples[i]));
			result.points.push_back(samples[i]);
		}
		result.pieces.emplace_back(result.lengths.back(), track);
	}
	return result;
}

// how far along the link out of the end of the switch its joints lie: past the clearance of the switch at each end, in the middle of
// a link too short for both; none on a link between the parts of one unit
std::vector<double> places_on(TTrack const &From, int const Path, bool const Atend, chain const &Line, link_line const &Link, track_map const &Units)
{
	auto const length{Link.lengths.back()};
	auto const own{clearance(From, Path, Atend)};
	if (Line.reached != nullptr)
	{
		int far_path{0};
		bool far_atend{false};
		if (one_unit(Units, &From, Line.reached) || false == end_at(*Line.reached, Link.points.back(), far_path, far_atend))
			return {};
		auto const theirs{clearance(*Line.reached, far_path, far_atend)};
		if (length - own - theirs >= kSection)
			return {own, length - theirs};
		return {(own + length - theirs) * 0.5};
	}
	if (false == Line.ended || length - own >= kSection)
		return {own};
	return {};
}

// the track of the link the joint at the distance cuts, none where the joint falls at the end of one; the distance snapped to that end
TTrack *cut_at(std::vector<std::pair<double, TTrack *>> const &Pieces, double &Distance)
{
	auto previous{0.0};
	for (auto const &[end, piece] : Pieces)
	{
		if (std::abs(Distance - previous) <= kSnap || std::abs(Distance - end) <= kSnap)
		{
			Distance = std::abs(Distance - previous) <= kSnap ? previous : end;
			return nullptr;
		}
		if (Distance < end)
			return piece;
		previous = end;
	}
	return nullptr;
}

// the joint at the distance along the link
joint joint_on(link_line const &Link, double const Place)
{
	auto distance{std::clamp(Place, 0.0, Link.lengths.back())};
	auto *track{cut_at(Link.pieces, distance)};
	auto const after{std::min<std::size_t>(std::ranges::upper_bound(Link.lengths, distance) - Link.lengths.begin(), Link.lengths.size() - 1)};
	auto const before{after > 0 ? after - 1 : 0};
	auto const span{Link.lengths[after] - Link.lengths[before]};
	return {track, glm::mix(Link.points[before], Link.points[after], span > 1e-9 ? (distance - Link.lengths[before]) / span : 0.0)};
}

// the joints found so far, with the links they lie on and the places where two switches meet
struct joints_found
{
	std::unordered_set<TTrack const *> through;
	std::vector<glm::dvec3> meets;
	std::vector<joint> joints;
};

// the joints of the link out of the end of the switch
void add_joints(layout const &Layout, track_map const &Units, TTrack const &From, int const Path, bool const Atend, joints_found &Found)
{
	auto line{follow(From, Path, Atend, std::numeric_limits<double>::max())};
	auto const start{end_of(From, Path, Atend)};
	// out of the station: to a switch of another one, along a line cut off by the edge of the scenery
	if (leaves(Layout, line))
	{
		line.reached = nullptr;
		line.ended = false;
	}
	if (line.reached == nullptr && false == line.ended)
		trim(line);
	if (line.tracks.empty())
	{
		// two switches meeting: the joint lies where they meet, unless they're parts of one
		if (line.reached == nullptr || one_unit(Units, &From, line.reached) || std::ranges::any_of(Found.meets, [&](glm::dvec3 const &Meet) { return glm::distance(Meet, start) < same_point; }))
			return;
		Found.meets.push_back(start);
		Found.joints.emplace_back(nullptr, start);
		return;
	}
	if (Found.through.contains(line.tracks.front()))
		return;
	Found.through.insert(line.tracks.begin(), line.tracks.end());
	// the link as one line from the switch on, and where each of its tracks ends along it
	auto const link{link_of(start, line.tracks)};
	for (auto const place : places_on(From, Path, Atend, line, link, Units))
		Found.joints.push_back(joint_on(link, place));
}

// a joint lies at the point
bool barrier(std::vector<glm::dvec3> const &Joints, glm::dvec3 const &Point)
{
	return std::ranges::any_of(Joints, [&](glm::dvec3 const &Joint) { return glm::distance(Joint, Point) < 0.05; });
}

// the tracks of the circuit of the seed: on from it through the ends no joint lies at, among the members not taken yet
std::vector<TTrack *> circuit_tracks(TTrack *Seed, std::unordered_set<TTrack const *> const &Members, std::vector<glm::dvec3> const &Joints, std::unordered_set<TTrack const *> &Taken)
{
	std::vector tracks{Seed};
	std::size_t index{0};
	while (index < tracks.size())
	{
		auto const &track{*tracks[index]};
		++index;
		auto const ends{node(track) ? ends_of(track) : std::vector<std::pair<int, bool>>{{0, false}, {0, true}}};
		for (auto const &[path, atend] : ends)
		{
			if (barrier(Joints, end_of(track, path, atend)))
				continue;
			auto *next{neighbour(track, path, atend)};
			if (next != nullptr && Members.contains(next) && Taken.insert(next).second)
				tracks.push_back(next);
		}
	}
	return tracks;
}

// the unit of the first of the tracks which is a part of one, -1 for none
int unit_among(std::vector<TTrack *> const &Tracks, track_map const &Units)
{
	for (auto const *track : Tracks)
		if (auto const found{Units.find(track)}; found != Units.end())
			return static_cast<int>(found->second);
	return -1;
}

// the chainage the circuit spans, its length and its mean offset in the schematic
void span_circuit(layout const &Layout, circuit &Circuit)
{
	Circuit.from = std::numeric_limits<double>::max();
	Circuit.to = std::numeric_limits<double>::lowest();
	double weighted{0.0};
	std::unordered_set<TTrack const *> const own(Circuit.tracks.begin(), Circuit.tracks.end());
	for (auto const &path : Layout.paths)
	{
		if (false == own.contains(path.track))
			continue;
		for (std::size_t i = 1; i < path.schematic.size(); ++i)
		{
			auto const step{glm::distance(path.points[i - 1], path.points[i])};
			weighted += step * (path.schematic[i - 1].y + path.schematic[i].y) * 0.5;
			Circuit.length += step;
		}
		for (auto const &point : path.schematic)
		{
			Circuit.from = std::min(Circuit.from, point.x);
			Circuit.to = std::max(Circuit.to, point.x);
		}
	}
	Circuit.offset = Circuit.length > 0.0 ? weighted / Circuit.length : 0.0;
}

} // namespace

std::vector<joint> joints(layout const &Layout, std::vector<unit> const &Units, [[maybe_unused]] options const &Options)
{
	auto const unit_of{units_by_part(Units)};
	joints_found found;
	for (auto const *from : Layout.switches)
		for (auto const &[path, atend] : ends_of(*from))
			add_joints(Layout, unit_of, *from, path, atend, found);
	return std::move(found.joints);
}

std::vector<circuit> circuits(layout const &Layout, std::vector<unit> const &Units, std::vector<glm::dvec3> const &Joints)
{
	auto const unit_of{units_by_part(Units)};
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
	std::unordered_set<TTrack const *> taken;
	std::vector<circuit> result;
	for (auto *seed : tracks)
	{
		if (taken.contains(seed))
			continue;
		taken.insert(seed);
		circuit entry;
		entry.tracks = circuit_tracks(seed, members, Joints, taken);
		entry.unit = unit_among(entry.tracks, unit_of);
		if (entry.unit < 0 && std::ranges::any_of(entry.tracks, [&](TTrack const *Track) { return approach.contains(Track); }))
			continue;
		span_circuit(Layout, entry);
		result.push_back(std::move(entry));
	}
	return result;
}

namespace
{

// the circuit has a track of the set
bool on_tracks(circuit const &Circuit, std::unordered_set<TTrack const *> const &Tracks)
{
	return std::ranges::any_of(Circuit.tracks, [&](TTrack const *Track) { return Tracks.contains(Track); });
}

// the root of the group of the circuit, the way to it halved
std::size_t root_of(std::vector<std::size_t> &Group, std::size_t Index)
{
	while (Group[Index] != Index)
	{
		Group[Index] = Group[Group[Index]];
		Index = Group[Index];
	}
	return Index;
}

// the circuit of plain track the station track of the circuit given runs on into from the end of its track, through the switches
// along their straighter paths no further than kThrough; none when it runs into none, or into one of a switch first
std::optional<std::size_t> through_to(TTrack const &Track, bool const Atend, std::size_t const Index, std::vector<circuit> const &Circuits, track_map const &Owners)
{
	auto point{end_of(Track, 0, Atend)};
	auto const *next{neighbour(Track, 0, Atend)};
	double length{0.0};
	while (next != nullptr && length < kThrough && rail(*next))
	{
		auto const there{Owners.find(next)};
		if (there == Owners.end() || there->second == Index)
			return std::nullopt;
		if (Circuits[there->second].unit < 0)
			return there->second;
		int path{0};
		bool end{false};
		if (false == end_at(*next, point, path, end) || (next->eType == tt_Switch && end && path != straighter(*next)))
			return std::nullopt;
		if (next->eType == tt_Switch)
			path = straighter(*next);
		length += plan_length(*next, path);
		point = end_of(*next, path, false == end);
		next = neighbour(*next, path, false == end);
	}
	return std::nullopt;
}

// the circuits of the station tracks the plain track of the circuit runs on into at its ends join its group
void join_through(TTrack const &Track, std::size_t const Index, std::vector<circuit> const &Circuits, track_map const &Owners, std::unordered_set<std::size_t> const &Plain, std::vector<std::size_t> &Group)
{
	if (node(Track))
		return;
	for (auto const atend : {false, true})
		if (auto const there{through_to(Track, atend, Index, Circuits, Owners)}; there.has_value() && Plain.contains(*there))
			Group[root_of(Group, *there)] = root_of(Group, Index);
}

// the mean offset of the circuits, by their lengths; Otherwise: when they have none
double mean_offset(std::vector<circuit> const &Circuits, std::vector<std::size_t> const &Group, double const Otherwise)
{
	double weighted{0.0};
	double length{0.0};
	for (auto const index : Group)
	{
		weighted += Circuits[index].offset * Circuits[index].length;
		length += Circuits[index].length;
	}
	return length > 0.0 ? weighted / length : Otherwise;
}

// the circuits of the station tracks by their groups, each with its mean offset
using side_groups = std::vector<std::pair<std::vector<std::size_t>, double>>;

// the groups of the circuits of the other station tracks, on the right of the axis and on its left
std::pair<side_groups, side_groups> sides_of(std::vector<circuit> const &Circuits, std::vector<std::size_t> const &Rest, std::vector<std::size_t> &Group)
{
	std::map<std::size_t, std::vector<std::size_t>> groups;
	for (auto const index : Rest)
		groups[root_of(Group, index)].push_back(index);
	side_groups on_right;
	side_groups on_left;
	for (auto &[first, members] : groups)
	{
		auto const offset{mean_offset(Circuits, members, Circuits[first].offset)};
		(offset < 0.0 ? on_right : on_left).emplace_back(std::move(members), offset);
	}
	return {std::move(on_right), std::move(on_left)};
}

// the tracks of the side numbered side by side, the nearest first, from the number given on by two
void number_side(side_groups &Side, int Number, std::vector<std::pair<std::vector<std::size_t>, int>> &Tracks)
{
	std::ranges::sort(Side, [](auto const &A, auto const &B) { return std::abs(A.second) < std::abs(B.second); });
	for (auto &[members, offset] : Side)
	{
		Tracks.emplace_back(std::move(members), Number);
		Number += 2;
	}
}

// the number of each circuit by the station track it belongs to, empty for those of switches; the circuits of a track lettered along
// the chainage where it has several
std::vector<std::string> numbers_of(std::vector<circuit> const &Circuits, std::vector<std::pair<std::vector<std::size_t>, int>> &Tracks)
{
	std::vector<std::string> result(Circuits.size());
	for (auto &[members, number] : Tracks)
	{
		std::ranges::sort(members, [&](std::size_t A, std::size_t B) { return Circuits[A].from < Circuits[B].from; });
		for (std::size_t index = 0; index < members.size(); ++index)
		{
			auto const letter{members.size() > 1 ? std::string(1, static_cast<char>('a' + std::min<std::size_t>(index, 25))) : std::string{}};
			result[members[index]] = std::format("{}{}", number, letter);
		}
	}
	return result;
}

} // namespace

std::vector<std::string> track_numbers(layout const &Layout, std::vector<circuit> const &Circuits)
{
	std::unordered_set<TTrack const *> const main(Layout.main.begin(), Layout.main.end());
	std::unordered_set<TTrack const *> const second(Layout.second.begin(), Layout.second.end());
	std::vector<std::size_t> axis;
	std::vector<std::size_t> other;
	std::vector<std::size_t> rest;
	track_map circuit_of;
	for (std::size_t index = 0; index < Circuits.size(); ++index)
	{
		for (auto const *track : Circuits[index].tracks)
			circuit_of[track] = index;
		if (Circuits[index].unit >= 0)
			continue;
		else if (on_tracks(Circuits[index], main))
			axis.push_back(index);
		else if (on_tracks(Circuits[index], second))
			other.push_back(index);
		else
			rest.push_back(index);
	}
	// a station track goes on through the switches along their straighter paths: its circuits beyond them share its number
	std::vector<std::size_t> group(Circuits.size());
	std::iota(group.begin(), group.end(), 0);
	std::unordered_set<std::size_t> const plain(rest.begin(), rest.end());
	for (auto const index : rest)
		for (auto const *track : Circuits[index].tracks)
			join_through(*track, index, Circuits, circuit_of, plain, group);
	auto [on_right, on_left]{sides_of(Circuits, rest, group)};
	// track 1 on the right along the chainage: the other main track of a double-track line when it lies to the right of the axis
	auto const swapped{false == other.empty() && mean_offset(Circuits, other, 0.0) < mean_offset(Circuits, axis, 0.0)};
	std::vector<std::pair<std::vector<std::size_t>, int>> tracks{{axis, swapped ? 2 : 1}};
	if (false == other.empty())
		tracks.emplace_back(other, swapped ? 1 : 2);
	// beyond the main tracks the others side by side, the nearest first, the odd numbers to the right, the even ones to the left
	number_side(on_right, 3, tracks);
	number_side(on_left, other.empty() ? 2 : 4, tracks);
	return numbers_of(Circuits, tracks);
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
		std::ranges::reverse(points);
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

// the plain track before the end of the track the trains running out reach, with the end of it they reach; none at a switch, at the
// end of the tracks or at a track seen already
std::optional<std::pair<TTrack *, bool>> track_before(TTrack const &Track, bool const Atend, std::unordered_set<TTrack const *> &Seen)
{
	auto const inner{end_of(Track, 0, false == Atend)};
	auto *before{neighbour(Track, 0, false == Atend)};
	int path{0};
	bool atend{false};
	if (before == nullptr || node(*before) || false == rail(*before) || false == Seen.insert(before).second || false == end_at(*before, inner, path, atend))
		return std::nullopt;
	return std::make_pair(before, atend);
}

// the point the distance in from the end of the track, with the track, the end of it and the way to that end
std::tuple<TTrack *, bool, glm::dvec3, glm::dvec3> in_from_end(TTrack *Track, bool const Atend, double const Distance)
{
	auto const [at, way] = in_from(*Track, Atend, Distance);
	return {Track, Atend, at, way};
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
		auto const before{track_before(*Track, Atend, seen)};
		if (false == before.has_value())
			return in_from_end(Track, Atend, std::min(Distance, length * 0.5));
		Distance -= length;
		Track = before->first;
		Atend = before->second;
	}
	return in_from_end(Track, Atend, Distance);
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
	if (std::ranges::any_of(Taken, [&](glm::dvec2 const &Other) { return glm::distance(Other, spot) < kSignalApart; }))
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
	auto const right_gap{side_gap(Layout, Point, Way, Facing, Taken)};
	auto side{Facing};
	if (right_gap < kSignalClear)
	{
		auto const left_gap{side_gap(Layout, Point, Way, -Facing, Taken)};
		if (left_gap >= kSignalClear || left_gap > right_gap)
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
			if (next != nullptr && rail(*next) && own.contains(next))
				continue;
			result.emplace_back(track, atend, end_of(*track, 0, atend), next == nullptr || false == rail(*next));
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

// the next letter beyond the limit, the step given
char letter_after(char Letter, char const Limit, int const Step)
{
	while (Step > 0 ? Letter <= Limit : Letter >= Limit)
		Letter = next_letter(Letter, Step);
	return Letter;
}

// an entry signal, with the offset and the end of the station its line leaves at
struct entry_signal
{
	signal placed;
	double offset{0.0};
	bool start{false};
};

// the end of the first track of the line out of the station by the track before it, the joint: true its end
bool joint_end(layout const &Layout, std::vector<TTrack *> const &Line, std::vector<TTrack *>::const_iterator const Outside)
{
	auto const &track{**Outside};
	if (Outside != Line.begin())
	{
		int path{0};
		bool end{false};
		return false == (end_at(**std::prev(Outside), end_of(track, 0, false), path, end));
	}
	auto const start{end_of(track, 0, false)};
	return std::ranges::none_of(Layout.switches, [&](TTrack const *Switch) {
		return std::ranges::any_of(Switch->m_paths, [&](auto const &Path) { return glm::distance(Path.points[segment_data::point::start], start) < same_point || glm::distance(Path.points[segment_data::point::end], start) < same_point; });
	});
}

// out from the joint along the line, Point: the joint, the tracks passed noted in the run of the signal; the signal placed kEntry out:
// how far out the line went before it
double run_out(layout const &Layout, TTrack *Track, glm::dvec3 Point, signal &Placed, std::vector<glm::dvec2> &Taken)
{
	double along{0.0};
	auto *next{Track};
	std::unordered_set<TTrack const *> seen;
	while (next != nullptr && rail(*next) && false == node(*next) && seen.insert(next).second)
	{
		int path{0};
		bool end{false};
		if (false == end_at(*next, Point, path, end))
			return along;
		auto const outward{false == end};
		auto const length{plan_length(*next, 0)};
		Placed.run.emplace_back(next, outward);
		if (along + length >= kEntry)
		{
			auto const [at, way] = in_from(*next, end, kEntry - along);
			Placed.track = next;
			Placed.point = at;
			Placed.facing = outward ? -1 : 1;
			Placed.side = side_for(Layout, at, outward ? -way : way, Placed.facing, Taken);
			Placed.travel = way;
			return along;
		}
		along += length;
		Point = end_of(*next, 0, outward);
		next = neighbour(*next, 0, outward);
	}
	return along;
}

// the entry signal of the line out of the station from the joint at the end of the track: out from the joint, along the path when
// the joint is its start; none placed for a line ending within 30 m
entry_signal entry_on(layout const &Layout, TTrack *Track, bool const Atend, std::vector<glm::dvec2> &Taken)
{
	entry_signal found;
	auto const joint{end_of(*Track, 0, Atend)};
	found.offset = project(Layout, joint).y;
	found.start = project(Layout, joint).x < (Layout.from + Layout.to) * 0.5;
	auto const along{run_out(Layout, Track, joint, found.placed, Taken)};
	if (found.placed.track == nullptr && false == found.placed.run.empty() && along > 30.0)
	{
		// the line ends sooner, by the edge of the scenery: as far out as it goes
		auto const &[last, outward] = found.placed.run.back();
		auto const length{plan_length(*last, 0)};
		auto const [at, way] = in_from(*last, outward, std::min(10.0, length * 0.5));
		found.placed.track = last;
		found.placed.point = at;
		found.placed.facing = outward ? -1 : 1;
		found.placed.side = side_for(Layout, at, outward ? way : -way, found.placed.facing, Taken);
		found.placed.travel = -way;
	}
	return found;
}

// the entry signals named, A, B... from the start of the chainage, Z, Y... from its end, in the order given, and put in the result:
// the last letter given at the start and at the end
std::pair<char, char> name_entries(std::vector<entry_signal> &Entries, std::vector<signal> &Result)
{
	char from_start{'A'};
	char from_end{'Z'};
	char last_start{'@'};
	char first_end{'['};
	for (auto &found : Entries)
	{
		auto &letter{found.start ? from_start : from_end};
		found.placed.kind = signal::role::entry;
		found.placed.name = std::string(1, letter);
		(found.start ? last_start : first_end) = letter;
		letter = next_letter(letter, found.start ? 1 : -1);
		Result.push_back(std::move(found.placed));
	}
	return {last_start, first_end};
}

// how the exit signals are named: a letter for the odd tracks and one for the even ones at each end; at an end no line leaves, the
// station tracks get shunting signals instead
struct exit_naming
{
	char odd_start{'F'};
	char even_start{'G'};
	char odd_end{'R'};
	char even_end{'P'};
	bool starts{false}; // lines leave at the start of the chainage
	bool ends{false}; // and at its end
};

char exit_letter(exit_naming const &Naming, bool const Start, bool const Odd)
{
	if (Start)
		return Odd ? Naming.odd_start : Naming.even_start;
	return Odd ? Naming.odd_end : Naming.even_end;
}

// the outer ends of circuits of a station track, by the chainage
struct track_reach
{
	track_end low;
	track_end high;
	double low_x{std::numeric_limits<double>::max()};
	double high_x{std::numeric_limits<double>::lowest()};
};

// the reach widened to the end at the chainage, where it lies beyond it
void widen(track_reach &Reach, track_end const &End, double const X)
{
	if (X < Reach.low_x)
	{
		Reach.low_x = X;
		Reach.low = End;
	}
	if (X > Reach.high_x)
	{
		Reach.high_x = X;
		Reach.high = End;
	}
}

// the ends of the longest circuit long enough, for the longest useful length of the track both ways (Ie-4 § 13 ust. 8), else the
// outer ends of the outermost ones; whether the circuit is long enough
std::pair<track_reach, bool> exit_ends(layout const &Layout, numbered const &Track)
{
	track_reach longest;
	track_reach any;
	double best{0.0};
	for (auto const &circuit : Track.circuits)
	{
		double length{0.0};
		for (auto const *part : circuit)
			length += node(*part) ? 0.0 : plan_length(*part, 0);
		auto const chosen{length >= kStopping && length > best};
		if (chosen)
		{
			best = length;
			longest = track_reach{};
		}
		for (auto const &end : ends_out(circuit))
		{
			auto const x{project(Layout, end.point).x};
			widen(any, end, x);
			if (chosen)
				widen(longest, end, x);
		}
	}
	auto const stopping{longest.low.track != nullptr};
	return {stopping ? longest : any, stopping};
}

// the exit or the shunting signal at the end of a station track, facing out
signal end_signal(layout const &Layout, track_end const &End, bool const Exit, std::vector<glm::dvec2> &Taken)
{
	signal placed;
	auto const [track, atend, at, way] = back_from(End.track, End.atend, Exit ? signal_drop : kJoint);
	placed.track = track;
	placed.point = at;
	placed.facing = atend ? 1 : -1;
	placed.side = side_for(Layout, at, atend ? way : -way, placed.facing, Taken);
	placed.travel = way;
	placed.kind = Exit ? signal::role::exit : signal::role::shunting;
	return placed;
}

// the signals at the ends of the station track: the exit ones into the result, the shunting ones aside
void track_signals(layout const &Layout, numbered const &Track, exit_naming const &Naming, std::vector<glm::dvec2> &Taken, std::vector<signal> &Result, std::vector<signal> &Shunting)
{
	auto const digit{std::find_if(Track.number.rbegin(), Track.number.rend(), [](char const Letter) { return std::isdigit(static_cast<unsigned char>(Letter)) != 0; })};
	auto const odd{digit == Track.number.rend() || (*digit - '0') % 2 == 1};
	auto upper{Track.number};
	std::ranges::transform(upper, upper.begin(), [](char const Letter) { return static_cast<char>(std::toupper(static_cast<unsigned char>(Letter))); });
	auto const [chosen, stopping]{exit_ends(Layout, Track)};
	for (auto const start : {true, false})
	{
		auto const &end{start ? chosen.low : chosen.high};
		if (end.track == nullptr || end.free)
			continue;
		auto const exit{stopping && (start ? Naming.starts : Naming.ends)};
		auto placed{end_signal(Layout, end, exit, Taken)};
		if (false == exit)
		{
			Shunting.push_back(std::move(placed));
			continue;
		}
		placed.name = std::string(1, exit_letter(Naming, start, odd)) + upper;
		Result.push_back(std::move(placed));
	}
}

} // namespace

std::vector<signal> signals_beyond(layout const &Layout, std::vector<numbered> const &Tracks, std::vector<std::size_t> const &Outside)
{
	std::vector<signal> result;
	std::vector<glm::dvec2> taken; // the spots of the signals placed
	// the entry signals: out along each line from where it leaves the circuits of the station
	std::vector<entry_signal> entries;
	for (std::size_t index = 0; index < Layout.lines.size(); ++index)
	{
		auto const &line{Layout.lines[index]};
		auto const outside{std::next(line.begin(), static_cast<std::ptrdiff_t>(Outside[index]))};
		if (outside == line.end() || node(**outside))
			continue;
		// the joint: the end of the first track out of the station by the track before it
		if (auto found{entry_on(Layout, *outside, joint_end(Layout, line, outside), taken)}; found.placed.track != nullptr)
			entries.push_back(std::move(found));
	}
	std::ranges::sort(entries, [](entry_signal const &A, entry_signal const &B) { return A.offset > B.offset; });
	auto const [last_start, first_end]{name_entries(entries, result)};
	// the exit signals, a letter for the odd tracks and one for the even ones at each end, clear of the entry ones
	exit_naming naming;
	naming.odd_start = letter_after('F', last_start, 1);
	naming.even_start = next_letter(naming.odd_start, 1);
	naming.odd_end = letter_after('R', first_end, -1);
	naming.even_end = next_letter(naming.odd_end, -1);
	naming.starts = std::ranges::any_of(entries, [](entry_signal const &Entry) { return Entry.start; });
	naming.ends = std::ranges::any_of(entries, [](entry_signal const &Entry) { return false == Entry.start; });
	std::vector<signal> shunting;
	for (auto const &track : Tracks)
		track_signals(Layout, track, naming, taken, result, shunting);
	std::ranges::sort(shunting, [&](signal const &A, signal const &B) { return project(Layout, A.point).x < project(Layout, B.point).x; });
	for (std::size_t index = 0; index < shunting.size(); ++index)
	{
		shunting[index].name = std::format("Tm{}", index + 1);
		result.push_back(std::move(shunting[index]));
	}
	return result;
}

namespace
{

// the points of the track added to the line, Outward: the line runs out towards the end of its path
void add_to_line(line &Line, std::unordered_set<TTrack const *> &Seen, TTrack *Track, bool const Outward, std::vector<glm::dvec3> Points)
{
	if (false == Line.points.empty())
		Points.erase(Points.begin());
	Seen.insert(Track);
	Line.tracks.push_back(Track);
	Line.outward.push_back(Outward);
	for (auto const &point : Points)
	{
		Line.distances.push_back(Line.points.empty() ? 0.0 : Line.distances.back() + plan_distance(Line.points.back(), point));
		Line.points.push_back(point);
		Line.owners.push_back(Line.tracks.size() - 1);
	}
}

// the track added to the line along it
void add_out(line &Line, std::unordered_set<TTrack const *> &Seen, TTrack *Track, bool const Outward)
{
	double length{0.0};
	auto points{dense_points(*Track, 0, length)};
	if (false == Outward)
		std::ranges::reverse(points);
	add_to_line(Line, Seen, Track, Outward, std::move(points));
}

// the track of the entry signal added to the line, its point nearest to the signal put at the signal: the distance of the signal
// along the line
double add_first(line &Line, std::unordered_set<TTrack const *> &Seen, signal const &Entry, bool const Outward)
{
	double length{0.0};
	auto points{dense_points(*Entry.track, 0, length)};
	if (false == Outward)
		std::ranges::reverse(points);
	auto const nearest{std::ranges::min_element(points, [&](glm::dvec3 const &A, glm::dvec3 const &B) { return plan_distance(A, Entry.point) < plan_distance(B, Entry.point); })};
	*nearest = Entry.point;
	auto at{static_cast<std::size_t>(nearest - points.begin())};
	if (Line.points.empty())
	{
		points.erase(points.begin(), nearest);
		at = 0;
	}
	else
		at += Line.points.size() - 1;
	add_to_line(Line, Seen, Entry.track, Outward, std::move(points));
	return Line.distances[at];
}

// the track the line goes on into beyond the end of the track, with the way out along it; none at the end of the line, a switch of
// the next station noted
std::optional<std::pair<TTrack *, bool>> next_out(line &Line, std::unordered_set<TTrack const *> const &Seen, TTrack const &Track, bool const Outward)
{
	auto const end{end_of(Track, 0, Outward)};
	auto *next{neighbour(Track, 0, Outward)};
	if (next == nullptr || false == rail(*next) || Seen.contains(next))
		return std::nullopt;
	if (node(*next))
	{
		Line.station = true;
		return std::nullopt;
	}
	int path{0};
	bool atend{false};
	if (false == end_at(*next, end, path, atend))
		return std::nullopt;
	return std::make_pair(next, false == atend);
}

// the line on out from the track, until it goes beyond the limit past the distance of the signal or ends
void run_on(line &Line, std::unordered_set<TTrack const *> &Seen, TTrack *Track, bool Outward, double const Origin, double const Limit)
{
	while (true)
	{
		if (Line.length() - Origin > Limit)
			return;
		auto const next{next_out(Line, Seen, *Track, Outward)};
		if (false == next.has_value())
			return;
		Track = next->first;
		Outward = next->second;
		add_out(Line, Seen, Track, Outward);
	}
}

// the line beyond the entry signal, no longer than the limit
line line_out(signal const &Entry, std::size_t const Index, double const Limit)
{
	line found;
	found.entry = Index;
	std::unordered_set<TTrack const *> seen;
	// in from the signal to the joint of the station first: its trains run on there beyond it
	for (auto const &[part, out] : Entry.run)
	{
		if (part == Entry.track)
			break;
		add_out(found, seen, part, out);
	}
	// out where the trains it faces come from
	auto const outward{Entry.facing < 0};
	auto const origin{add_first(found, seen, Entry, outward)};
	run_on(found, seen, Entry.track, outward, origin, Limit);
	for (auto &distance : found.distances)
		distance -= origin;
	return found;
}

// the line starts at the start of the chainage
bool at_start(layout const &Layout, line const &Line)
{
	return project(Layout, Line.points.front()).x < (Layout.from + Layout.to) * 0.5;
}

// how far two lines keep from 4.5 m apart along their first kilometre, at most; the most for lines shorter than 100 m
double gap_between(line const &A, line const &B)
{
	auto const reach{std::min({A.length(), B.length(), 1000.0})};
	auto worst{0.0};
	for (auto const fraction : {0.2, 0.6, 1.0})
	{
		double gap{0.0};
		distance_on(B, point_on(A, reach * fraction).point, gap);
		worst = std::max(worst, std::abs(gap - 4.5));
	}
	return reach < 100.0 ? std::numeric_limits<double>::max() : worst;
}

// the pairs of lines running side by side, the longest first: a siding along a main track for a while is no line
std::vector<std::tuple<double, double, std::size_t, std::size_t>> side_by_side(layout const &Layout, std::vector<line> const &Lines)
{
	std::vector<std::tuple<double, double, std::size_t, std::size_t>> pairs;
	for (std::size_t i = 0; i < Lines.size(); ++i)
		for (std::size_t j = i + 1; j < Lines.size(); ++j)
		{
			if (at_start(Layout, Lines[i]) != at_start(Layout, Lines[j]))
				continue;
			if (auto const gap{gap_between(Lines[i], Lines[j])}; gap <= 2.0)
				pairs.emplace_back(-std::min(Lines[i].length(), Lines[j].length()), gap, i, j);
		}
	std::ranges::sort(pairs);
	return pairs;
}

// a double-track line: two lines out of the same end 3 to 6.5 m apart all along their first kilometre
void pair_lines(layout const &Layout, std::vector<signal> const &Signals, std::vector<line> &Lines)
{
	std::vector<char> paired(Lines.size(), 0);
	for (auto const &[reach_order, gap, i, partner] : side_by_side(Layout, Lines))
	{
		if (paired[i] != 0 || paired[partner] != 0)
			continue;
		paired[partner] = 1;
		paired[i] = 1;
		Lines[i].partner = static_cast<int>(partner);
		Lines[partner].partner = static_cast<int>(i);
		// the trains towards the station on the track on their right; along the chainage the right one is track 1
		auto const &travel{Signals[Lines[i].entry].travel};
		glm::dvec2 const rightwards{-travel.z, travel.x};
		auto const reach{std::min({Lines[i].length(), Lines[partner].length(), 200.0})};
		auto const here{point_on(Lines[i], reach).point};
		auto const there{point_on(Lines[partner], reach).point};
		auto const partner_inbound{glm::dot(glm::dvec2{there.x - here.x, there.z - here.z}, rightwards) > 0.0};
		Lines[i].inbound = false == partner_inbound;
		Lines[partner].inbound = partner_inbound;
		for (auto *found : {&Lines[i], &Lines[partner]})
			found->number = found->inbound == at_start(Layout, *found) ? 1 : 2;
	}
}

} // namespace

std::vector<line> lines_out(layout const &Layout, std::vector<signal> const &Signals, double const Limit)
{
	std::vector<line> result;
	for (std::size_t index = 0; index < Signals.size(); ++index)
	{
		auto const &entry{Signals[index]};
		if (entry.kind != signal::role::entry || entry.track == nullptr || node(*entry.track))
			continue;
		if (auto found{line_out(entry, index, Limit)}; found.length() > 1.0)
			result.push_back(std::move(found));
	}
	pair_lines(Layout, Signals, result);
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
			std::ranges::reverse(points);
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
	auto const after{std::ranges::lower_bound(Line.distances, Distance)};
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
	if (auto const way{Line.points[i] - Line.points[i - 1]}; glm::length(way) > 1e-9)
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
		std::ranges::reverse(points);
	return points;
}

// a path of a track along a station track, run along from its end
struct step
{
	TTrack *track{nullptr};
	int path{0};
	bool atend{false};
};

// the end of the track at the point, true its end; none when it has none there
std::optional<bool> atend_at(TTrack const &Track, glm::dvec3 const &Point)
{
	bool atend{false};
	if (int path{0}; false == end_at(Track, Point, path, atend))
		return std::nullopt;
	return atend;
}

// the search for the way out of a station track: the tracks of its own and those taken, the shortest way found and the way tried
struct way_search
{
	std::unordered_set<TTrack const *> const *own{nullptr};
	std::unordered_set<TTrack const *> const *taken{nullptr};
	std::vector<step> best;
	double shortest{0.0};
	std::vector<step> current;
	int budget{kSearch};
};

// on from the end of the path of the track, the way up to it Length long
void search_on(way_search &Search, TTrack const &From, int const Path, bool const End, double const Length)
{
	auto const point{end_of(From, Path, End)};
	auto *next{neighbour(From, Path, End)};
	if (--Search.budget < 0 || next == nullptr || false == rail(*next) || Length >= Search.shortest || std::ranges::any_of(Search.current, [&](step const &Step) { return Step.track == next; }))
		return;
	if (Search.own->contains(next))
	{
		if (Search.taken->contains(next))
			return;
		if (auto const atend{atend_at(*next, point)}; atend.has_value())
		{
			Search.best = Search.current;
			Search.best.push_back({next, 0, *atend});
			Search.shortest = Length;
		}
		return;
	}
	for (int path = 0; path < static_cast<int>(next->m_paths.size()); ++path)
		for (auto const atend : {false, true})
			if (glm::distance(end_of(*next, path, atend), point) < same_point)
			{
				Search.current.push_back({next, path, atend});
				search_on(Search, *next, path, false == atend, Length + plan_length(*next, path));
				Search.current.pop_back();
			}
}

// the shortest way out of the end of the track through the tracks of others, the switches within a station track, on to one of
// its own not taken yet, no longer than the limit; its last step on that one, empty for none
std::vector<step> way_on(TTrack const &Track, bool const Atend, std::unordered_set<TTrack const *> const &Own, std::unordered_set<TTrack const *> const &Taken, double const Limit)
{
	way_search search;
	search.own = &Own;
	search.taken = &Taken;
	search.shortest = Limit;
	search_on(search, Track, 0, Atend, 0.0);
	return std::move(search.best);
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
	auto const *next{neighbour(Track, 0, Atend)};
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
			if (auto const at{fouling(*next, path, atend)}; at.has_value())
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
	auto const after{std::min<std::size_t>(std::ranges::upper_bound(Distances, Along) - Distances.begin(), Distances.size() - 1)};
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
	for (auto const *track : simulation::Paths.sequence())
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

// the stretches along the track the segment of the road ending at the point given takes, crossing it on its level
void crossings_with(std::vector<glm::dvec3> const &Points, std::vector<double> const &Distances, road const &Road, std::size_t const K, std::vector<std::pair<double, double>> &Result)
{
	glm::dvec2 const r0{Road.points[K - 1].x, Road.points[K - 1].z};
	glm::dvec2 const across{glm::dvec2{Road.points[K].x, Road.points[K].z} - r0};
	if (glm::dot(across, across) < 1e-9)
		return;
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
		if (std::abs(glm::mix(Road.points[K - 1].y, Road.points[K].y, t) - glm::mix(Points[i - 1].y, Points[i].y, u)) > kLevel)
			continue;
		auto const sine{std::abs(determinant) / (glm::length(along) * glm::length(across))};
		auto const half{Road.width * 0.5 / std::max(sine, kSkew)};
		auto const at{Distances[i - 1] + (Distances[i] - Distances[i - 1]) * u};
		Result.emplace_back(at - half, at + half);
	}
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
			crossings_with(Points, Distances, road, k, result);
	}
	std::ranges::sort(result);
	std::vector<std::pair<double, double>> merged;
	for (auto const &part : result)
		if (false == merged.empty() && part.first <= merged.back().second)
			merged.back().second = std::max(merged.back().second, part.second);
		else
			merged.push_back(part);
	return merged;
}

// a station track walked along from its end at the lower chainage, through the switches within it: its points with their distances
// from that end, its paths, the bounds of its stretches between the switches, how much of it the switches take
struct track_walk
{
	std::vector<glm::dvec3> points;
	std::vector<double> distances;
	std::vector<std::pair<TTrack const *, int>> paths;
	std::vector<std::pair<bound, bound>> bounds; // of the stretch before, of the one after
	double within{0.0};
};

// the points of the path from its end added to the walk
void walk_on(track_walk &Walk, TTrack const &Track, int const Path, bool const Atend)
{
	double length{0.0};
	auto const along{from_end(Track, Path, Atend, length)};
	for (std::size_t i = Walk.points.empty() ? 0 : 1; i < along.size(); ++i)
	{
		Walk.distances.push_back(Walk.points.empty() ? 0.0 : Walk.distances.back() + plan_distance(Walk.points.back(), along[i]));
		Walk.points.push_back(along[i]);
	}
	Walk.paths.emplace_back(&Track, Path);
}

// the bound of the stretch before the switch the step runs into, Along the walk where it does
void switch_ahead(bound &Bound, step const &Step, double const Along)
{
	Bound.node = Step.track;
	Bound.start = Along;
	if (auto const out{fouling(*Step.track, Step.path, Step.atend)}; out.has_value())
		Bound.fouling = Along - *out;
}

// the bound of the stretch after the switch the step runs out of, Along the walk where it does
void switch_behind(bound &Bound, step const &Step, double const Along)
{
	Bound.node = Step.track;
	Bound.start = Along;
	Bound.fouling.reset();
	if (auto const out{fouling(*Step.track, Step.path, false == Step.atend)}; out.has_value())
		Bound.fouling = Along + *out;
}

// the way through the switches within the track added to the walk: the bounds of the stretches before and after them
std::pair<bound, bound> through_switches(track_walk &Walk, std::vector<step> const &Way)
{
	bound before;
	bound after;
	before.joint = Walk.distances.back();
	for (std::size_t i = 0; i < Way.size(); ++i)
	{
		auto const &part{Way[i]};
		auto const from{Walk.distances.back()};
		if (i + 1 == Way.size())
			after.joint = from;
		walk_on(Walk, *part.track, part.path, part.atend);
		if (false == node(*part.track))
			continue;
		Walk.within += Walk.distances.back() - from;
		if (before.node == nullptr)
			switch_ahead(before, part, from);
		switch_behind(after, part, Walk.distances.back());
	}
	after.open = before.node == nullptr;
	before.open = after.open;
	return {before, after};
}

// the walk along the station track from the end of its track, through the switches between its tracks: the end of the track it
// ends on; Owners: the circuit of each of its tracks
std::pair<TTrack const *, bool> walk_along(track_walk &Walk, TTrack const *Track, bool Atend, std::unordered_set<TTrack const *> const &Members, std::unordered_map<TTrack const *, std::size_t> &Owners)
{
	std::unordered_set<TTrack const *> taken;
	taken.insert(Track);
	walk_on(Walk, *Track, 0, Atend);
	while (true)
	{
		auto const way{way_on(*Track, false == Atend, Members, taken, kReach)};
		if (way.empty())
			break;
		if (way.size() == 1 && Owners[way.front().track] == Owners[Track])
			walk_on(Walk, *way.front().track, 0, way.front().atend);
		else
			Walk.bounds.emplace_back(through_switches(Walk, way));
		Track = way.back().track;
		Atend = way.back().atend;
		taken.insert(Track);
	}
	return {Track, Atend};
}

// the bound of the first stretch, at the end of the track at the lower chainage
bound low_bound(beyond const &Low)
{
	bound after;
	after.node = Low.node;
	after.open = Low.open;
	if (Low.fouling.has_value())
		after.fouling = -*Low.fouling;
	after.start = -Low.out;
	return after;
}

// the bound of the last stretch, at the other end of the track Length along it
bound high_bound(beyond const &High, double const Length)
{
	bound before;
	before.node = High.node;
	before.open = High.open;
	before.joint = Length;
	if (High.fouling.has_value())
		before.fouling = Length + *High.fouling;
	before.start = Length + High.out;
	return before;
}

// the stretch the point Middle along the walk lies within: the index of the bounds after it; none when it lies within none
std::optional<std::size_t> stretch_at(std::vector<std::pair<bound, bound>> const &Bounds, double const Middle)
{
	for (std::size_t index = 1; index < Bounds.size(); ++index)
		if (Middle > Bounds[index - 1].second.joint && Middle < Bounds[index].first.joint)
			return index;
	return std::nullopt;
}

// the level crossings within the stretches part them
void add_crossings(track_walk &Walk, std::vector<road> const &Roads)
{
	for (auto const &[from, to] : crossings_of(Walk.points, Walk.distances, Roads))
	{
		auto const at{stretch_at(Walk.bounds, (from + to) * 0.5)};
		if (false == at.has_value())
			continue;
		bound before;
		before.crossing = true;
		before.joint = from;
		bound after;
		after.crossing = true;
		after.joint = to;
		Walk.bounds.emplace(Walk.bounds.begin() + *at, before, after);
	}
}

// the points of the track from its end at the lower chainage, the switches beyond its ends taken in
void lay_points(lengths &Entry, beyond const &Low, track_walk const &Walk, beyond const &High)
{
	auto const length{Walk.distances.back()};
	for (std::size_t i = Low.points.size(); i-- > 1;)
	{
		Entry.points.push_back(Low.points[i]);
		Entry.distances.push_back(-Low.distances[i]);
	}
	Entry.points.insert(Entry.points.end(), Walk.points.begin(), Walk.points.end());
	Entry.distances.insert(Entry.distances.end(), Walk.distances.begin(), Walk.distances.end());
	for (std::size_t i = 1; i < High.points.size(); ++i)
	{
		Entry.points.push_back(High.points[i]);
		Entry.distances.push_back(length + High.distances[i]);
	}
}

// where along the walk a point lies nearest to it: how far from it, how far along, the way of the walk there
struct nearest_on
{
	double gap{std::numeric_limits<double>::max()};
	double along{0.0};
	glm::dvec2 tangent{0.0, 1.0};
};

nearest_on nearest_along(track_walk const &Walk, glm::dvec2 const &Point)
{
	nearest_on result;
	auto const &points{Walk.points};
	auto const &distances{Walk.distances};
	for (std::size_t i = 1; i < points.size(); ++i)
	{
		glm::dvec2 const a{points[i - 1].x, points[i - 1].z};
		glm::dvec2 const segment{glm::dvec2{points[i].x, points[i].z} - a};
		auto const length2{glm::dot(segment, segment)};
		if (length2 < 1e-12)
			continue;
		auto const t{std::clamp(glm::dot(Point - a, segment) / length2, 0.0, 1.0)};
		if (auto const distance{glm::distance(Point, a + segment * t)}; distance < result.gap)
		{
			result.gap = distance;
			result.along = distances[i - 1] + (distances[i] - distances[i - 1]) * t;
			result.tangent = segment / std::sqrt(length2);
		}
	}
	return result;
}

// a path of another track runs nearer to the point than the walk, Gap away
bool nearer_other(layout const &Layout, track_walk const &Walk, glm::dvec3 const &Point, double const Gap)
{
	return std::ranges::any_of(Layout.paths, [&](path const &Path) {
		return std::ranges::none_of(Walk.paths, [&](auto const &Own) { return Own.first == Path.track && Own.second == Path.index; }) && plan_gap(Point, Path.points) < Gap - 0.01;
	});
}

// a signal by a station track, how far along it and the way of the trains it faces along it
struct stop_signal
{
	double along{0.0};
	int facing{1};
	std::string name;
};

// the signals by the track: nearer to it than to any other track, facing along it
std::vector<stop_signal> stops_by(layout const &Layout, track_walk const &Walk, std::vector<signal> const &Signals)
{
	std::vector<stop_signal> stops;
	for (auto const &signal : Signals)
	{
		auto const nearest{nearest_along(Walk, glm::dvec2{signal.point.x, signal.point.z})};
		if (nearest.gap > kBeside || nearer_other(Layout, Walk, signal.point, nearest.gap))
			continue;
		auto const cosine{glm::dot(glm::normalize(glm::dvec2{signal.travel.x, signal.travel.z} + glm::dvec2{1e-12, 0.0}), nearest.tangent)};
		if (std::abs(cosine) >= kFacing)
			stops.emplace_back(nearest.along, cosine > 0.0 ? 1 : -1, signal.name);
	}
	return stops;
}

// the longest part the trains running the way stand on: from a signal facing them back to the end of its stretch behind them
void signalled_stand(stand &Part, std::vector<std::pair<bound, bound>> const &Bounds, std::vector<stop_signal> const &Stops, bool const Forward)
{
	auto const inward{Forward ? 1.0 : -1.0};
	for (std::size_t index = 0; index + 1 < Bounds.size(); ++index)
	{
		auto const &before{Bounds[index].second};
		auto const &after{Bounds[index + 1].first};
		auto const behind{Forward ? limit_at(before, inward, true) : limit_at(after, inward, true)};
		for (auto const &stop : Stops)
		{
			if (stop.facing != (Forward ? 1 : -1) || stop.along < before.joint || stop.along > after.joint)
				continue;
			if (auto const useful{(stop.along - behind.along) * inward}; useful > 0.0 && useful > Part.length)
			{
				Part.rear = behind;
				Part.head.what = limit::kind::signal;
				Part.head.name = stop.name;
				Part.head.along = stop.along;
				Part.length = useful;
			}
		}
	}
}

// the longest stretch between the fouling points, where no signal stands by the track
void plain_stand(stand &Part, std::vector<std::pair<bound, bound>> const &Bounds, bool const Forward)
{
	for (std::size_t index = 0; index + 1 < Bounds.size(); ++index)
	{
		auto const lower{limit_at(Bounds[index].second, 1.0, false)};
		auto const upper{limit_at(Bounds[index + 1].first, -1.0, false)};
		if (auto const stretch{upper.along - lower.along}; stretch > Part.length)
		{
			Part.rear = Forward ? lower : upper;
			Part.head = Forward ? upper : lower;
			Part.length = stretch;
		}
	}
}

// each way the longest: from a signal facing the trains back to the end of its stretch behind them, else of the stretches between
// the fouling points
void stands_of(lengths &Entry, std::vector<std::pair<bound, bound>> const &Bounds, std::vector<stop_signal> const &Stops)
{
	for (int way = 0; way < 2; ++way)
	{
		auto const forward{way == 0};
		auto &part{Entry.ways[way]};
		part.length = -1.0;
		signalled_stand(part, Bounds, Stops, forward);
		if (part.length < 0.0)
			plain_stand(part, Bounds, forward);
		part.length = std::max(0.0, part.length);
		part.head.point = point_along(Entry.points, Entry.distances, part.head.along);
		part.rear.point = point_along(Entry.points, Entry.distances, part.rear.along);
	}
}

// the lengths of the station track, none for a track without an end
std::optional<lengths> track_length(layout const &Layout, numbered const &Track, std::vector<signal> const &Signals, std::vector<road> const &Roads)
{
	std::vector<TTrack *> own;
	std::unordered_map<TTrack const *, std::size_t> circuit_of;
	for (std::size_t circuit = 0; circuit < Track.circuits.size(); ++circuit)
		for (auto *part : Track.circuits[circuit])
			if (false == node(*part) && circuit_of.emplace(part, circuit).second)
				own.push_back(part);
	auto const ends{ends_out(own)};
	if (ends.empty())
		return std::nullopt;
	// from the end at the lower chainage along its tracks, through the switches between them: the stretches between the switches
	// and the joints, each between the end of one bound and the start of the next
	auto const first{std::ranges::min_element(ends, [&](track_end const &A, track_end const &B) { return project(Layout, A.point).x < project(Layout, B.point).x; })};
	std::unordered_set<TTrack const *> const members(own.begin(), own.end());
	track_walk walk;
	auto const low{out_of(*first->track, first->atend)};
	walk.bounds.emplace_back(bound{}, low_bound(low));
	auto const [at, atend]{walk_along(walk, first->track, first->atend, members, circuit_of)};
	auto const length{walk.distances.back()};
	auto const high{out_of(*at, false == atend)};
	walk.bounds.emplace_back(high_bound(high, length), bound{});
	add_crossings(walk, Roads);
	lengths entry;
	entry.number = Track.number;
	entry.construction = length + (low.open ? 0.0 : low.out) + (high.open ? 0.0 : high.out) - walk.within;
	lay_points(entry, low, walk, high);
	stands_of(entry, walk.bounds, stops_by(Layout, walk, Signals));
	return entry;
}

} // namespace

std::vector<lengths> track_lengths(layout const &Layout, std::vector<numbered> const &Tracks, std::vector<signal> const &Signals)
{
	std::vector<lengths> result;
	auto const roads{roads_by(Layout)};
	for (auto const &track : Tracks)
		if (auto entry{track_length(Layout, track, Signals, roads)}; entry.has_value())
			result.push_back(std::move(*entry));
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

namespace
{

// the points of a path of a track, with the distances along it to them in the plan
struct path_points
{
	std::vector<glm::dvec3> points;
	std::vector<double> distances;
};

using path_cache = std::map<std::pair<TTrack const *, int>, path_points>;

// the points of the path of the track, worked out on the first call for it
path_points const &along_of(path_cache &Paths, TTrack const &Track, int const Path)
{
	auto &found{Paths[{&Track, Path}]};
	if (found.points.empty())
	{
		double length{0.0};
		found.points = points_of(Track, Path, length);
		found.distances.assign(1, 0.0);
		for (std::size_t k = 1; k < found.points.size(); ++k)
			found.distances.push_back(found.distances.back() + plan_distance(found.points[k - 1], found.points[k]));
	}
	return found;
}

// how far along the path its point nearest to the point given lies, Gap: how far from it
double project_on(path_points const &Path, glm::dvec3 const &Point, double &Gap)
{
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
}

// the search for the routes from the posts: the posts, those by each track, the paths run along, the limit of the routes
struct route_search
{
	std::vector<post> const *posts{nullptr};
	std::unordered_map<TTrack const *, std::vector<std::size_t>> by_track;
	path_cache paths;
	double limit{0.0};
};

// the route through the switch it is on: the least speed it allows, the turnout taken off the straight
void pass_switch(reach &Here)
{
	auto const &track{*Here.track};
	auto const speed{path_speed(track, Here.path)};
	auto allowed{speed};
	if (auto const set{track.SwitchExtension != nullptr ? std::abs(static_cast<double>(track.SwitchExtension->fVelocity)) : Here.track->VelocityGet()}; speed > 0.0 && set >= 2.0)
		allowed = std::min(speed, set);
	if (allowed > 0.0 && (Here.speed <= 0.0 || allowed < Here.speed))
		Here.speed = allowed;
	if (speed > 0.0 && (track.eType != tt_Switch || Here.path != straighter(track)))
		Here.turnouts.push_back(Here.track);
}

// the signals ahead on the path of the route facing it, but those for shunting and the one it starts at: the nearest first
std::vector<std::pair<double, std::size_t>> posts_ahead(route_search const &Search, reach const &Here, path_points const &Line, std::size_t const Start)
{
	std::vector<std::pair<double, std::size_t>> ahead;
	if (auto const found{Search.by_track.find(Here.track)}; found != Search.by_track.end())
		for (auto const index : found->second)
		{
			auto const &other{(*Search.posts)[index]};
			if (index == Start || other.facing != Here.facing || other.kind == signal::role::shunting)
				continue;
			double gap{0.0};
			auto const at{project_on(Line, other.point, gap)};
			auto const distance{(at - Here.from) * Here.facing};
			if (gap < 4.5 && distance > 0.1)
				ahead.emplace_back(distance, index);
		}
	std::ranges::sort(ahead);
	return ahead;
}

// the route up to the first of the signals ahead within the limit, the warning disc passed before it noted: whether one stops it
bool stop_ahead(route_search const &Search, reach &Here, std::vector<std::pair<double, std::size_t>> const &Ahead, std::vector<route> &Result)
{
	for (auto const &[distance, index] : Ahead)
	{
		if (Here.length + distance > Search.limit)
			return false;
		if ((*Search.posts)[index].kind == signal::role::warning)
		{
			if (Here.warning < 0)
				Here.warning = static_cast<int>(index);
			continue;
		}
		Result.emplace_back(static_cast<int>(index), Here.warning, Here.length + distance, Here.speed, Here.turnouts, false);
		return true;
	}
	return false;
}

// the route on into the paths beyond the end of its path not passed yet, Length long up to there: whether it goes on into any
bool go_on(route_search &Search, reach const &Here, double const Length, std::vector<reach> &Stack)
{
	auto const atend{Here.facing > 0};
	auto const point{end_of(*Here.track, Here.path, atend)};
	auto *next{neighbour(*Here.track, Here.path, atend)};
	if (next == nullptr || false == rail(*next))
		return false;
	auto went{false};
	for (int other = 0; other < static_cast<int>(next->m_paths.size()); ++other)
		for (auto const end : {false, true})
		{
			if (glm::distance(end_of(*next, other, end), point) >= same_point)
				continue;
			auto const facing{end ? -1 : 1};
			if (std::ranges::find(Here.passed, std::make_pair(static_cast<TTrack const *>(next), other * 2 + (facing > 0 ? 1 : 0))) != Here.passed.end())
				continue;
			auto const &entered{along_of(Search.paths, *next, other)};
			Stack.emplace_back(next, other, facing, end ? entered.distances.back() : 0.0, Length, Here.speed, Here.warning, Here.turnouts, Here.passed);
			went = true;
		}
	return went;
}

// the start of the routes from the post, on the path of its track it lies nearest to
reach start_reach(route_search &Search, post const &Start)
{
	auto path{0};
	auto nearest{std::numeric_limits<double>::max()};
	double from{0.0};
	for (int candidate = 0; candidate < static_cast<int>(Start.track->m_paths.size()); ++candidate)
	{
		double gap{0.0};
		auto const at{project_on(along_of(Search.paths, *Start.track, candidate), Start.point, gap)};
		if (gap < nearest - 1e-3)
		{
			nearest = gap;
			path = candidate;
			from = at;
		}
	}
	return {Start.track, path, Start.facing, from, 0.0, 0.0, -1, {}, {}};
}

// the route from the post Start along the path it has come to: stopped by a signal on it, ended at the limit or at the end of
// the tracks, else gone on into the paths beyond it
void step_route(route_search &Search, std::size_t const Start, reach Here, std::vector<reach> &Stack, std::vector<route> &Result)
{
	auto const &track{*Here.track};
	auto const &line{along_of(Search.paths, track, Here.path)};
	auto const total{line.distances.back()};
	Here.passed.emplace_back(&track, Here.path * 2 + (Here.facing > 0 ? 1 : 0));
	if (node(track))
		pass_switch(Here);
	// the signals ahead on the path, the nearest first
	if (stop_ahead(Search, Here, posts_ahead(Search, Here, line, Start), Result))
		return;
	auto const length{Here.length + (Here.facing > 0 ? total - Here.from : Here.from)};
	if (length > Search.limit)
	{
		Result.emplace_back(-1, Here.warning, Search.limit, Here.speed, Here.turnouts, false);
		return;
	}
	if (false == go_on(Search, Here, length, Stack))
		Result.emplace_back(-1, Here.warning, length, Here.speed, Here.turnouts, true);
}

// the routes from the post, the paths they pass kPieces at most
std::vector<route> routes_from(route_search &Search, std::size_t const Start)
{
	std::vector<route> result;
	std::vector stack{start_reach(Search, (*Search.posts)[Start])};
	auto pieces{0};
	while (false == stack.empty() && pieces < kPieces)
	{
		++pieces;
		auto here{std::move(stack.back())};
		stack.pop_back();
		step_route(Search, Start, std::move(here), stack, result);
	}
	return result;
}

} // namespace

std::vector<std::vector<route>> routes(std::vector<post> const &Posts, std::vector<std::size_t> const &From, double const Limit)
{
	std::vector<std::vector<route>> all;
	route_search search;
	search.posts = &Posts;
	search.limit = Limit;
	for (std::size_t index = 0; index < Posts.size(); ++index)
		if (Posts[index].track != nullptr)
			search.by_track[Posts[index].track].push_back(index);
	for (auto const from : From)
		all.push_back(from < Posts.size() && Posts[from].track != nullptr && false == Posts[from].track->m_paths.empty() ? routes_from(search, from) : std::vector<route>{});
	return all;
}

namespace
{

// the path of the track nearest to the point, its point nearest to it, its points a metre or so apart
struct dense_path
{
	int path{0};
	long at{0};
	std::vector<glm::dvec3> points;
};

dense_path nearest_path(TTrack const &Track, glm::dvec3 const &Point)
{
	dense_path result;
	auto nearest{std::numeric_limits<double>::max()};
	for (int candidate = 0; candidate < static_cast<int>(Track.m_paths.size()); ++candidate)
	{
		double length{0.0};
		auto dense{dense_points(Track, candidate, length)};
		for (std::size_t k = 0; k < dense.size(); ++k)
			if (auto const gap{plan_distance(dense[k], Point)}; gap < nearest - 1e-6)
			{
				nearest = gap;
				result.path = candidate;
				result.at = static_cast<long>(k);
			}
		if (result.path == candidate)
			result.points = std::move(dense);
	}
	return result;
}

// the way of the trains walked back along the tracks: the path it is on and the way the trains run along it, its points, the one
// it goes back from, the last one passed and how far back that lies
struct approach_walk
{
	TTrack *track{nullptr};
	int path{0};
	int facing{1};
	long at{0};
	std::vector<glm::dvec3> points;
	glm::dvec3 last{0.0};
	double distance{0.0};
};

// the points of the path walked back to its end the trains come from: whether the walk reaches the distance on it
bool walk_back(approach_walk &Walk, double const Distance, std::vector<approach_point> &Result)
{
	auto const &points{Walk.points};
	auto const count{static_cast<long>(points.size())};
	auto const stride{Walk.facing > 0 ? -1L : 1L};
	for (auto k{Walk.at}; k >= 0 && k < count; k += stride)
	{
		Walk.distance += plan_distance(Walk.last, points[k]);
		Walk.last = points[k];
		auto const ahead{points[std::clamp(k + 1, 0L, count - 1)] - points[std::clamp(k - 1, 0L, count - 1)]};
		glm::dvec3 way{ahead.x * Walk.facing, 0.0, ahead.z * Walk.facing};
		way = glm::length(way) > 1e-9 ? glm::normalize(way) : glm::dvec3{0.0, 0.0, 1.0};
		Result.emplace_back(Walk.track, Walk.path, Walk.facing, points[k], way, Walk.distance);
		if (Walk.distance >= Distance)
			return true;
	}
	return false;
}

// the walk on back into the track the trains come from, into a switch from its start along its straighter path: whether there is
// one not walked yet
bool step_back(approach_walk &Walk, std::unordered_set<TTrack const *> &Seen)
{
	auto const atend{Walk.facing < 0};
	auto const end{end_of(*Walk.track, Walk.path, atend)};
	auto *before{neighbour(*Walk.track, Walk.path, atend)};
	int entered{0};
	bool from_end{false};
	if (before == nullptr || false == rail(*before) || false == Seen.insert(before).second || false == end_at(*before, end, entered, from_end))
		return false;
	if (before->eType == tt_Switch && false == from_end)
		entered = straighter(*before);
	Walk.track = before;
	Walk.path = entered;
	double length{0.0};
	Walk.points = dense_points(*Walk.track, Walk.path, length);
	Walk.facing = from_end ? 1 : -1;
	Walk.at = from_end ? static_cast<long>(Walk.points.size()) - 2 : 1;
	return true;
}

} // namespace

std::vector<approach_point> approach(TTrack &Track, glm::dvec3 const &Point, int const Facing, double const Distance)
{
	std::vector<approach_point> result;
	if (false == rail(Track))
		return result;
	auto nearest{nearest_path(Track, Point)};
	approach_walk walk;
	walk.track = &Track;
	walk.path = nearest.path;
	walk.at = nearest.at;
	walk.points = std::move(nearest.points);
	walk.facing = Facing > 0 ? 1 : -1;
	walk.last = walk.points[walk.at];
	std::unordered_set<TTrack const *> seen{walk.track};
	while (false == walk_back(walk, Distance, result))
		if (false == step_back(walk, seen))
			return result;
	return result;
}

} // namespace station
