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
#include "editor/editorSwitchStore.hpp"
#include "editor/editorTrack.hpp"
#include "simulation/simulation.h"
#include "world/Track.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
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
};

chain follow(TTrack const &From, int const Path, bool const Atend, double const Limit)
{
	chain result;
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
			if (line.reached == nullptr && false == line.ended)
			{
				result.exits.push_back(std::move(line));
				continue;
			}
			for (auto *track : line.tracks)
				if (known.insert(track).second)
					result.plain.push_back(track);
			if (line.reached != nullptr && known.insert(line.reached).second)
				result.switches.push_back(line.reached);
		}
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

// the map shows the east to the right and the north up: the x of the scenery runs to the west
glm::dvec2 map_of(glm::dvec3 const &Point)
{
	return {-Point.x, Point.z};
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
	for (auto const &line : found.exits)
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
	// the chainage runs with the km of the switches, when they have one and most of them agree on it
	double constexpr kAgree{50.0}; // m
	std::vector<std::pair<double, double>> marks; // the chainage and the km, in m
	for (auto *track : found.switches)
	{
		auto const start{end_of(*track, 0, false)};
		std::optional<double> km;
		if (auto const record{switch_store::named(*track)}; record && record->km)
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
	for (auto *track : found.plain)
		add(*track, false);
	for (auto const &line : found.exits)
	{
		double length{0.0};
		for (auto *track : line.tracks)
		{
			if (length >= Options.approach)
				break;
			add(*track, true);
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
		auto const t{std::clamp(glm::dot(point - a, unit), 0.0, length)};
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

std::string switch_name(std::string const &Code, int const Number, std::size_t const Part, std::size_t const Parts)
{
	std::string result;
	for (auto const c : Code)
		result += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	result += '_' + std::to_string(Number);
	if (Parts > 1)
		result += static_cast<char>('a' + std::min<std::size_t>(Part, 25));
	return result;
}

std::vector<TTrack *> switches_of(TTrack &Switch, options const &Options)
{
	return collect(Switch, Options).switches;
}

} // namespace station
