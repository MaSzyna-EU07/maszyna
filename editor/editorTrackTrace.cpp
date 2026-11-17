/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTrackTrace.hpp"
#include "editor/editorFouling.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorTrack.hpp"

#include "world/Track.h"

#include <algorithm>
#include <functional>
#include <limits>

namespace track_trace
{

namespace
{

using geometry::bezier;
using geometry::plan_distance;
using geometry::plan_of;

int const kRailCategory{1};
// m the track is followed past the farthest switch looked at for one whose fouling point lies nearer: it lies that far before the
// end of the switch at most
double const kFoulingReach{200.0};
// m from the axis of the track a fouling point is taken as lying on it
double const kOnAxis{1.0};
// m the tracks of a switch are followed from its start for its fouling point
double const kFoulingTrace{250.0};

// the track a train runs onto from the end of a path at Exit: entering a switch at its start it takes the main track
struct next_track
{
	TTrack *track{nullptr};
	int path{0};
	bool start{true}; // run onto at its start, along its path
};

next_track next_of(TTrack &Track, glm::dvec3 const &Exit)
{
	next_track next;
	for (auto const &connection : editor_track::connected_points(Track, Exit))
	{
		auto *other{connection.first};
		if (other == &Track || other->m_editorremoved || (other->iCategoryFlag & 15) != kRailCategory)
			continue;
		bool const start{connection.second.kind == editor_track::point_kind::start};
		if (next.track == nullptr || (other == next.track && start && connection.second.path == 0))
		{
			next.track = other;
			next.path = connection.second.path;
			next.start = start;
		}
	}
	return next;
}

// the axis from the parameter From of a path on, the way a train runs, for Limit m: each point with the distance to it and the
// path it lies on. Enter is told of each track run onto, with the distance to the end of it run onto; it stops the walk
// returning false
void walk(TTrack &Track, int const Path, double const From, bool const Forward, double const Limit, std::vector<glm::dvec3> &Points, std::vector<double> &Along, std::vector<std::pair<TTrack *, int>> &Paths,
          std::function<bool(TTrack &, int, bool, double)> const &Enter, std::vector<double> *Cant = nullptr)
{
	auto *track{&Track};
	auto path{Path};
	auto forward{Forward};
	auto from{From};
	double length{0.0};
	for (int guard = 0; guard < 128 && track != nullptr && length < Limit; ++guard)
	{
		bezier const curve{track->m_paths[path]};
		auto const span{forward ? 1.0 - from : from};
		auto const count{std::clamp(static_cast<int>(curve.plan_length() * span), 2, 400)};
		for (int k = Points.empty() ? 0 : 1; k <= count; ++k)
		{
			auto const t{forward ? from + span * k / count : from - span * k / count};
			auto const point{curve.point(t)};
			if (false == Points.empty())
				length += plan_distance(Points.back(), point);
			Points.push_back(point);
			Along.push_back(length);
			Paths.emplace_back(track, path);
			// the raise of one rail over the other from the roll of the path there, as the gauge of the editor takes it: a roll
			// above 0 lowers the left rail of the path, the cars lean to the left of it; turned to the way it's walked
			if (Cant != nullptr)
			{
				auto const &rolls{track->m_paths[path].rolls};
				auto const roll{static_cast<double>(rolls[0]) + static_cast<double>(rolls[1] - rolls[0]) * t};
				Cant->push_back(1.5 * std::sin(glm::radians(roll)) * (forward ? 1.0 : -1.0));
			}
		}
		auto const next{next_of(*track, Points.back())};
		if (next.track == nullptr || (Enter && false == Enter(*next.track, next.path, next.start, length)))
			return;
		track = next.track;
		path = next.path;
		forward = next.start;
		from = forward ? 0.0 : 1.0;
	}
}

// the distance along the line to the point of it nearest the point given; Off: of the point from it
double along_on(std::vector<glm::dvec3> const &Line, std::vector<double> const &Along, glm::dvec3 const &Point, double &Off)
{
	Off = std::numeric_limits<double>::max();
	double result{0.0};
	auto const point{plan_of(Point)};
	for (std::size_t i = 1; i < Line.size(); ++i)
	{
		auto const a{plan_of(Line[i - 1])};
		auto const ab{plan_of(Line[i]) - a};
		auto const length2{glm::dot(ab, ab)};
		auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0) : 0.0};
		auto const off{glm::length(point - (a + ab * t))};
		if (off < Off)
		{
			Off = off;
			result = Along[i - 1] + (Along[i] - Along[i - 1]) * t;
		}
	}
	return result;
}

} // namespace

std::vector<glm::dvec3> trace(TTrack &Track, int const Path, bool const Forward, double const Limit)
{
	std::vector<double> cant;
	return trace(Track, Path, Forward, Limit, cant);
}

std::vector<glm::dvec3> trace(TTrack &Track, int const Path, bool const Forward, double const Limit, std::vector<double> &Cant)
{
	std::vector<glm::dvec3> points;
	std::vector<double> along;
	std::vector<std::pair<TTrack *, int>> paths;
	Cant.clear();
	walk(Track, Path, Forward ? 0.0 : 1.0, Forward, Limit, points, along, paths, {}, &Cant);
	return points;
}

double parameter_of(TTrack const &Track, int const Path, glm::dvec3 const &Point)
{
	bezier const curve{Track.m_paths[Path]};
	auto const distance = [&](double const T) { return plan_distance(curve.point(T), Point); };
	auto best{0.0};
	auto nearest{std::numeric_limits<double>::max()};
	for (int k = 0; k <= 256; ++k)
		if (auto const d{distance(k / 256.0)}; d < nearest)
		{
			nearest = d;
			best = k / 256.0;
		}
	// then closer, between the neighbours of the nearest
	auto low{std::max(0.0, best - 1.0 / 256.0)};
	auto high{std::min(1.0, best + 1.0 / 256.0)};
	for (int i = 0; i < 40; ++i)
	{
		auto const a{low + (high - low) / 3.0};
		auto const b{high - (high - low) / 3.0};
		if (distance(a) < distance(b))
			high = b;
		else
			low = a;
	}
	return (low + high) * 0.5;
}

bool fouling_of(TTrack &Switch, double const Spacing, double const Limit, fouling &Point)
{
	if (Switch.m_paths.size() < 2)
		return false;
	std::vector<double> maincant;
	std::vector<double> branchcant;
	auto const main{trace(Switch, 0, true, Limit, maincant)};
	auto const branch{trace(Switch, 1, true, Limit, branchcant)};
	::fouling::place place;
	if (false == ::fouling::find(main, branch, Spacing, place, maincant, branchcant))
		return false;
	Point.main = place.main;
	Point.branch = place.branch;
	Point.back = place.back;
	Point.spacing = place.spacing;
	return true;
}

glm::dvec3 danger::at(double const Distance, std::pair<TTrack *, int> &Path) const
{
	if (line.empty())
		return glm::dvec3{0.0};
	auto const found{std::lower_bound(along.begin(), along.end(), Distance)};
	if (found == along.begin())
	{
		Path = paths.front();
		return line.front();
	}
	if (found == along.end())
	{
		Path = paths.back();
		return line.back();
	}
	auto const i{static_cast<std::size_t>(found - along.begin())};
	auto const span{along[i] - along[i - 1]};
	// a segment lies on the track of its farther end: ahead the one after it, behind, where the line runs reversed, the one before
	Path = Distance < 0.0 ? paths[i - 1] : paths[i];
	return glm::mix(line[i - 1], line[i], span > 1e-9 ? (Distance - along[i - 1]) / span : 1.0);
}

danger danger_ahead(TTrack &Track, int const Path, glm::dvec3 const &Point, bool const Forward, double const Limit, double const Behind, double const Spacing)
{
	danger result;
	if (Track.m_paths.empty())
		return result;
	auto const path{std::clamp(Path, 0, static_cast<int>(Track.m_paths.size()) - 1)};
	auto const from{parameter_of(Track, path, Point)};
	// behind the point: a fouling point there puts it in the fouling zone
	std::vector<glm::dvec3> back;
	std::vector<double> backalong;
	std::vector<std::pair<TTrack *, int>> backpaths;
	walk(Track, path, from, false == Forward, Behind, back, backalong, backpaths, {});
	for (auto i{back.size()}; i-- > 1;)
	{
		result.line.push_back(back[i]);
		result.along.push_back(-backalong[i]);
		result.paths.push_back(backpaths[i]);
	}
	for (auto &step : backalong)
		step = -step;
	std::vector<glm::dvec3> ahead;
	std::vector<double> along;
	std::vector<std::pair<TTrack *, int>> paths;
	auto best{std::numeric_limits<double>::max()};
	// the fouling point of a switch run onto from the end of the path given, on that path, where it lies along the track so far
	auto const trailing = [&](TTrack &Switch, int const SwitchPath) {
		fouling found;
		if (false == fouling_of(Switch, Spacing, kFoulingTrace, found))
			return;
		auto const point{SwitchPath == 0 ? found.main : found.branch};
		double off;
		double offback;
		auto const onahead{along_on(ahead, along, point, off)};
		auto const onback{along_on(back, backalong, point, offback)};
		auto const distance{offback < off ? onback : onahead};
		if (std::min(off, offback) > kOnAxis || distance >= best)
			return;
		best = distance;
		result.what = danger::kind::fouling;
		result.track = &Switch;
		result.distance = distance;
		result.point = point;
		result.spacing = found.spacing;
	};
	// on a switch the train runs towards its start, its own fouling point, once the track ahead is known
	auto own{Track.eType == tt_Switch && Track.m_paths.size() >= 2 && false == Forward};
	// a switch run onto from behind: its fouling point once its path is walked too, it may lie on it
	std::pair<TTrack *, int> pending{nullptr, 0};
	auto const settle = [&]() {
		if (pending.first != nullptr)
			trailing(*pending.first, pending.second);
		pending.first = nullptr;
	};
	walk(Track, path, from, Forward, Limit, ahead, along, paths, [&](TTrack &Next, int const NextPath, bool const Start, double const At) {
		if (std::exchange(own, false))
			trailing(Track, path);
		settle();
		if (At > best + kFoulingReach)
			return false;
		if (Next.eType != tt_Switch || Next.m_paths.size() < 2)
			return true;
		if (false == Start)
		{
			pending = {&Next, NextPath};
			return true;
		}
		// run onto at its start: nothing farther is nearer
		if (At < best)
		{
			best = At;
			result.what = danger::kind::start;
			result.track = &Next;
			result.distance = At;
			result.point = ahead.back();
		}
		return false;
	});
	if (own)
		trailing(Track, path);
	settle();
	result.line.insert(result.line.end(), ahead.begin(), ahead.end());
	result.along.insert(result.along.end(), along.begin(), along.end());
	result.paths.insert(result.paths.end(), paths.begin(), paths.end());
	return result;
}

} // namespace track_trace
