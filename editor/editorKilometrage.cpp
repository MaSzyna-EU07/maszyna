/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorKilometrage.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorTrack.hpp"

#include "world/Track.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <unordered_set>

namespace kilometrage
{

namespace
{

using geometry::plan_distance;
using geometry::plan_of;

std::string const kMark{"//$k"};
double const kReach{60000.0}; // m, of the tracks followed each way from a mark
double const kStep{2.0}; // m between the points of an axis
double const kOnAxis{12.0}; // m, of a mark from the axis of its line
double const kNear{6.0}; // m, of a mark from the track it stands on
double const kBetweenLow{3.0}; // m between the axes of the tracks of a double-track line
double const kBetweenHigh{6.5};
double const kCell{10.0}; // m, of the squares the track beside is sorted into
double const kApart{15.0}; // m between the tracks, of a double-track line on two formations
double const kBlend{100.0}; // m, the axis takes to go over from between the tracks to one of them
double const kLonger{1.0}; // m, of the track beside longer than the track of the line, for the axis to follow it
double const kDetour{20000.0}; // m, of the tracks followed each way from a via back onto the line

std::vector<mark> g_marks;
std::vector<via> g_vias;
state g_base; // as it was before the changes not yet in the undo history
bool g_changed{false};
bool g_loaded{false};
std::size_t g_edits{0};
std::map<std::string, std::optional<axis>> g_axes;
std::size_t g_revision{0};

void drop_axes()
{
	g_axes.clear();
	++g_revision;
}

using span = editor_track::route_span;

std::string word(std::string const &Text)
{
	return Text.empty() ? std::string{"-"} : Text;
}

// format of the lines, coordinates in the space of the file:
//   mark <line> <x y z> way <x z> km <km> [beyond <km>]
//   via <line> <x y z>
void store(scene::layer_handle const Layer)
{
	if (false == scene::Layers.valid(Layer))
		return;
	auto const &context{scene::Layers.layer(Layer).context_insert()};
	std::vector<std::string> lines;
	auto const heading = [&]() {
		if (lines.empty())
			lines.emplace_back("# kilometrage of the lines set in the track editor, the simulation skips these lines");
	};
	for (auto const &entry : g_marks)
	{
		if (entry.layer != Layer)
			continue;
		heading();
		auto const point{context.to_local(entry.point)};
		auto const ahead{context.to_local(entry.point + glm::dvec3{entry.way.x, 0.0, entry.way.y})};
		auto way{plan_of(ahead - point)};
		way = glm::length(way) > 1e-12 ? glm::normalize(way) : entry.way;
		auto text{format("mark %s %.4f %.4f %.4f way %.9f %.9f km %.3f", word(entry.line).c_str(), point.x, point.y, point.z, way.x, way.y, entry.km)};
		if (entry.beyond.has_value())
			text += format(" beyond %.3f", *entry.beyond);
		lines.push_back(text);
	}
	for (auto const &entry : g_vias)
	{
		if (entry.layer != Layer)
			continue;
		heading();
		auto const point{context.to_local(entry.point)};
		lines.push_back(format("via %s %.4f %.4f %.4f", word(entry.line).c_str(), point.x, point.y, point.z));
	}
	scene::Layers.mark(Layer, kMark, std::move(lines));
}

void changed(std::set<scene::layer_handle> const &Layers)
{
	g_changed = true;
	drop_axes();
	for (auto const layer : Layers)
		store(layer);
}

std::vector<span> flipped(std::vector<span> Spans)
{
	std::reverse(Spans.begin(), Spans.end());
	for (auto &entry : Spans)
		entry.forward = false == entry.forward;
	return Spans;
}

std::size_t index_of(std::vector<span> const &Spans, TTrack const *Track)
{
	auto const found{std::find_if(Spans.begin(), Spans.end(), [&](span const &Entry) { return Entry.track == Track; })};
	return found == Spans.end() ? Spans.size() : static_cast<std::size_t>(found - Spans.begin());
}

// the run on from the end of the span the way it goes, along the main paths of the switches
std::vector<span> onward(span const &Last)
{
	auto const run{editor_track::run_route(*Last.track, kReach)};
	auto const at{index_of(run.spans, Last.track)};
	if (at == run.spans.size())
		return {};
	if (run.spans[at].forward == Last.forward)
		return {run.spans.begin() + at + 1, run.spans.end()};
	return flipped({run.spans.begin(), run.spans.begin() + at});
}

// the spans with each track once, the first time it comes
std::vector<span> once(std::vector<span> const &Spans)
{
	std::vector<span> result;
	std::unordered_set<TTrack const *> seen;
	for (auto const &entry : Spans)
		if (seen.insert(entry.track).second)
			result.push_back(entry);
	return result;
}

// the route runs along the track of the via, along the path of a switch it stands on
bool runs_by(editor_track::route const &Route, std::size_t const Via)
{
	int path{0};
	auto const *track{track_near(g_vias[Via].point, kNear, path)};
	return track != nullptr && std::any_of(Route.spans.begin(), Route.spans.end(), [&](span const &Entry) { return Entry.track == track && Entry.path == path; });
}

// the route led along the tracks of the vias: from the track of each the run along the straight paths of the switches both ways
// back onto the route takes the place of the stretch of the route between where it comes back; where it comes back one way only,
// as at a junction, the route goes on along it from there
void through(editor_track::route &Route, std::vector<std::size_t> const &Vias)
{
	for (auto const index : Vias)
	{
		int path{0};
		auto *track{track_near(g_vias[index].point, kNear, path)};
		if (track == nullptr || runs_by(Route, index))
			continue;
		auto const run{editor_track::run_route(*track, kDetour, path)};
		auto const at{index_of(run.spans, track)};
		if (at == run.spans.size())
			continue;
		auto const none{Route.spans.size()};
		auto const on = [&](std::size_t const Index) { return index_of(Route.spans, run.spans[Index].track); };
		std::optional<std::size_t> low;
		for (auto i = at; i-- > 0;)
			if (on(i) != none)
			{
				low = i;
				break;
			}
		std::optional<std::size_t> high;
		for (auto i = at + 1; i < run.spans.size(); ++i)
			if (on(i) != none)
			{
				high = i;
				break;
			}
		auto const part = [&](std::size_t const From, std::size_t const To) { return std::vector<span>{run.spans.begin() + From, run.spans.begin() + To}; };
		auto const head = [&](std::size_t const To) { return std::vector<span>{Route.spans.begin(), Route.spans.begin() + To}; };
		auto const tail = [&](std::size_t const From) { return std::vector<span>{Route.spans.begin() + From, Route.spans.end()}; };
		auto const joined = [](std::vector<std::vector<span>> const &Parts) {
			std::vector<span> result;
			for (auto const &entry : Parts)
				result.insert(result.end(), entry.begin(), entry.end());
			return result;
		};
		std::vector<span> spans;
		if (low && high)
		{
			auto const from{on(*low)};
			auto const to{on(*high)};
			if (from == to)
				continue;
			spans = from < to ? joined({head(from), part(*low, *high + 1), tail(to + 1)}) : joined({head(to), flipped(part(*low, *high + 1)), tail(from + 1)});
		}
		else if (low)
		{
			auto const from{on(*low)};
			spans = run.spans[*low].forward == Route.spans[from].forward ? joined({head(from), part(*low, run.spans.size())}) : joined({flipped(part(*low, run.spans.size())), tail(from + 1)});
		}
		else if (high)
		{
			auto const to{on(*high)};
			spans = run.spans[*high].forward == Route.spans[to].forward ? joined({part(0, *high + 1), tail(to + 1)}) : joined({head(to), flipped(part(0, *high + 1))});
		}
		else
			continue;
		Route.spans = once(spans);
	}
}

double plan_gap(std::vector<editor_track::route_sample> const &Samples, glm::dvec3 const &Point)
{
	auto best{std::numeric_limits<double>::max()};
	for (auto const &sample : Samples)
		best = std::min(best, plan_distance(sample.position, Point));
	return best;
}

// the track beside the line: of each sample of the line the sample of it beside, where they run side by side
struct pairing
{
	editor_track::route route; // of the track beside
	std::vector<editor_track::route_sample> partner; // along the track beside, empty for none
	std::vector<int> match; // of each sample, -1 where the track beside isn't there
	std::vector<double> across; // to the sample beside, along the normal of the sample
};

pairing pair_with(std::vector<editor_track::route_sample> const &Samples, TTrack &Beside, std::vector<std::size_t> const &Vias)
{
	pairing result;
	result.match.assign(Samples.size(), -1);
	result.across.assign(Samples.size(), 0.0);
	result.route = editor_track::run_route(Beside, kReach);
	through(result.route, Vias);
	result.partner = editor_track::sample_route(result.route, kStep);
	auto const &partner{result.partner};
	auto const cell = [](glm::dvec3 const &Point) { return std::make_pair(static_cast<long>(std::floor(Point.x / kCell)), static_cast<long>(std::floor(Point.z / kCell))); };
	std::map<std::pair<long, long>, std::vector<std::size_t>> grid;
	for (std::size_t j = 0; j < partner.size(); ++j)
		grid[cell(partner[j].position)].push_back(j);
	for (std::size_t i = 0; i < Samples.size(); ++i)
	{
		auto const &sample{Samples[i]};
		glm::dvec2 const normal{-sample.direction.y, sample.direction.x};
		auto const [cx, cz]{cell(sample.position)};
		auto best{std::numeric_limits<double>::max()};
		for (auto x = cx - 1; x <= cx + 1; ++x)
			for (auto z = cz - 1; z <= cz + 1; ++z)
			{
				auto const found{grid.find({x, z})};
				if (found == grid.end())
					continue;
				for (auto const j : found->second)
				{
					auto const offset{plan_of(partner[j].position) - plan_of(sample.position)};
					auto const across{glm::dot(offset, normal)};
					auto const along{glm::dot(offset, sample.direction)};
					if (std::abs(across) < kBetweenLow || std::abs(across) > kBetweenHigh || std::abs(along) >= kStep * 1.5 || std::abs(glm::dot(sample.direction, partner[j].direction)) <= 0.97 || std::abs(along) >= best)
						continue;
					best = std::abs(along);
					result.match[i] = static_cast<int>(j);
					result.across[i] = across;
				}
			}
	}
	return result;
}

// the track beside comes back onto the line between the stretches it runs beside it, or both ways, as a track of a station or a
// passing loop does: not the other track
bool looped(pairing const &Pairs, std::unordered_set<TTrack const *> const &Own)
{
	auto low{Pairs.partner.size()};
	std::size_t high{0};
	for (auto const match : Pairs.match)
		if (match >= 0)
		{
			low = std::min(low, static_cast<std::size_t>(match));
			high = std::max(high, static_cast<std::size_t>(match));
		}
	auto const on_line = [&](editor_track::route_sample const &Sample) { return Sample.span < Pairs.route.spans.size() && Own.count(Pairs.route.spans[Sample.span].track) > 0; };
	if (low >= Pairs.partner.size())
		return false;
	auto const first{Pairs.partner.begin() + static_cast<std::ptrdiff_t>(low)};
	auto const last{Pairs.partner.begin() + static_cast<std::ptrdiff_t>(high) + 1};
	return std::any_of(first, last, on_line) || (std::any_of(Pairs.partner.begin(), first, on_line) && std::any_of(last, Pairs.partner.end(), on_line));
}

// the track beside the line, looked for by the first mark and then every 500 m along the line: a track the line doesn't run
// through, the spacing of a double-track line away, beside it for 100 m at least
pairing pair_up(std::vector<editor_track::route_sample> const &Samples, editor_track::route const &Route, std::size_t const Origin, std::vector<std::size_t> const &Vias)
{
	std::unordered_set<TTrack const *> own;
	for (auto const &entry : Route.spans)
		own.insert(entry.track);
	std::unordered_set<TTrack const *> tried;
	auto const step{static_cast<std::size_t>(500.0 / kStep)};
	for (std::size_t k = 0; k * step < Samples.size() && tried.size() < 4; ++k)
		for (auto const way : {1, -1})
		{
			if (k == 0 && way < 0)
				continue;
			auto const index{static_cast<long>(Origin) + way * static_cast<long>(k * step)};
			if (index < 0 || index >= static_cast<long>(Samples.size()))
				continue;
			auto const &at{Samples[static_cast<std::size_t>(index)]};
			glm::dvec2 const left{-at.direction.y, at.direction.x};
			for (auto const side : {1.0, -1.0})
			{
				auto const probe{plan_of(at.position) + left * (side * (kBetweenLow + kBetweenHigh) * 0.5)};
				auto *track{track_near({probe.x, at.position.y, probe.y}, 1.6)};
				if (track == nullptr || own.count(track) > 0 || false == tried.insert(track).second)
					continue;
				auto result{pair_with(Samples, *track, Vias)};
				if (std::count_if(result.match.begin(), result.match.end(), [](int const Match) { return Match >= 0; }) * kStep >= 100.0 && false == looped(result, own))
					return result;
			}
		}
	pairing none;
	none.match.assign(Samples.size(), -1);
	none.across.assign(Samples.size(), 0.0);
	return none;
}

// the axis: between the tracks where they run side by side, along the longer one where they part and come together again
// (PKP GK-1 6.1.12), the other one getting the km by the share of its length, on the track of the line where the other one ends
void compose(axis &Result, std::vector<editor_track::route_sample> const &Samples, pairing const &Pairs)
{
	using sample = editor_track::route_sample;
	auto const emit = [&](glm::dvec3 const &Point, glm::dvec2 const &Direction) {
		Result.chainage.push_back(Result.points.empty() ? 0.0 : Result.chainage.back() + plan_distance(Result.points.back(), Point));
		Result.points.push_back(Point);
		Result.directions.push_back(Direction);
	};
	auto const shifted = [](glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Offset) {
		auto const planar{plan_of(Point) + glm::dvec2{-Direction.y, Direction.x} * Offset};
		return glm::dvec3{planar.x, Point.y, planar.y};
	};
	std::vector<std::size_t> matched;
	for (std::size_t i = 0; i < Samples.size(); ++i)
		if (Pairs.match[i] >= 0)
			matched.push_back(i);
	if (matched.empty())
	{
		for (auto const &entry : Samples)
			emit(entry.position, entry.direction);
		return;
	}
	Result.between = true;
	auto const half = [&](std::size_t const Index) { return Pairs.across[Index] * 0.5; };
	// from between the tracks over to one of them, within kBlend of the ends of the stretch apart
	auto const eased = [&](double const From, double const To, double const Low, double const High) {
		return From < To ? Low * (1.0 - std::clamp(From / kBlend, 0.0, 1.0)) : High * (1.0 - std::clamp(To / kBlend, 0.0, 1.0));
	};
	auto const first{matched.front()};
	for (std::size_t i = 0; i < first; ++i)
		emit(shifted(Samples[i].position, Samples[i].direction, half(first) * std::clamp(1.0 - (Samples[first].chainage - Samples[i].chainage) / kBlend, 0.0, 1.0)), Samples[i].direction);
	std::optional<axis::side> pending;
	std::vector<double> shares;
	for (std::size_t k = 0; k < matched.size(); ++k)
	{
		auto const a{matched[k]};
		emit(shifted(Samples[a].position, Samples[a].direction, half(a)), Samples[a].direction);
		if (pending)
		{
			pending->to = Result.chainage.back();
			for (auto const share : shares)
				pending->chainage.push_back(pending->from + share * (pending->to - pending->from));
			Result.sides.push_back(std::move(*pending));
			pending.reset();
		}
		if (k + 1 == matched.size())
			break;
		auto const b{matched[k + 1]};
		if (b == a + 1)
			continue;
		// the tracks apart between a and b: the track beside from the sample by a to the one by b, turned along the line
		auto const ja{Pairs.match[a]};
		auto const jb{Pairs.match[b]};
		std::vector<sample> beside;
		for (auto j = ja; j != jb; j += ja < jb ? 1 : -1)
		{
			auto entry{Pairs.partner[static_cast<std::size_t>(j)]};
			if (ja > jb)
				entry.direction = -entry.direction;
			beside.push_back(entry);
		}
		beside.push_back(Pairs.partner[static_cast<std::size_t>(jb)]);
		if (ja > jb)
			beside.back().direction = -beside.back().direction;
		std::vector<double> along{0.0};
		for (std::size_t q = 1; q < beside.size(); ++q)
			along.push_back(along.back() + plan_distance(beside[q - 1].position, beside[q].position));
		auto const own_length{Samples[b].chainage - Samples[a].chainage};
		auto const beside_length{along.back()};
		auto const at_share = [&](double const Share) {
			auto const target{Share * beside_length};
			auto const next{std::min<std::size_t>(beside.size() - 1, static_cast<std::size_t>(std::lower_bound(along.begin(), along.end(), target) - along.begin()))};
			if (next == 0)
				return beside.front().position;
			auto const span{along[next] - along[next - 1]};
			return glm::mix(beside[next - 1].position, beside[next].position, span > 1e-9 ? (target - along[next - 1]) / span : 0.0);
		};
		auto widest{0.0};
		for (auto i = a + 1; i < b; ++i)
			widest = std::max(widest, plan_distance(Samples[i].position, at_share((Samples[i].chainage - Samples[a].chainage) / own_length)));
		if (widest <= kApart || own_length < 1e-6 || beside_length < 1e-6)
		{
			// spread a little, as through a station: between them still
			for (auto i = a + 1; i < b; ++i)
			{
				auto const t{(Samples[i].chainage - Samples[a].chainage) / own_length};
				emit(shifted(Samples[i].position, Samples[i].direction, half(a) + (half(b) - half(a)) * t), Samples[i].direction);
			}
			continue;
		}
		pending = axis::side{};
		shares.clear();
		pending->from = Result.chainage.back();
		if (beside_length > own_length + kLonger)
		{
			for (std::size_t q = 1; q + 1 < beside.size(); ++q)
				emit(shifted(beside[q].position, beside[q].direction, -eased(along[q], beside_length - along[q], half(a), half(b))), beside[q].direction);
			pending->length = own_length;
			for (auto i = a + 1; i < b; ++i)
			{
				pending->points.push_back(Samples[i].position);
				pending->along.push_back(Samples[i].chainage - Samples[a].chainage);
				shares.push_back(pending->along.back() / own_length);
			}
		}
		else
		{
			for (auto i = a + 1; i < b; ++i)
				emit(shifted(Samples[i].position, Samples[i].direction, eased(Samples[i].chainage - Samples[a].chainage, Samples[b].chainage - Samples[i].chainage, half(a), half(b))), Samples[i].direction);
			pending->length = beside_length;
			for (std::size_t q = 1; q + 1 < beside.size(); ++q)
			{
				pending->points.push_back(beside[q].position);
				pending->along.push_back(along[q]);
				shares.push_back(along[q] / beside_length);
			}
		}
	}
	auto const last{matched.back()};
	for (auto i = last + 1; i < Samples.size(); ++i)
		emit(shifted(Samples[i].position, Samples[i].direction, half(last) * std::clamp(1.0 - (Samples[i].chainage - Samples[last].chainage) / kBlend, 0.0, 1.0)), Samples[i].direction);
}

std::optional<axis> build(std::string const &Line)
{
	std::vector<std::size_t> own;
	for (std::size_t index = 0; index < g_marks.size(); ++index)
		if (g_marks[index].line == Line)
			own.push_back(index);
	if (own.empty())
		return std::nullopt;
	std::vector<TTrack *> tracks;
	for (auto const index : own)
		tracks.push_back(track_near(g_marks[index].point, kNear));
	auto const start{std::find_if(tracks.begin(), tracks.end(), [](TTrack const *Track) { return Track != nullptr; })};
	if (start == tracks.end())
		return std::nullopt;
	auto *first{*start};
	auto const first_mark{own[static_cast<std::size_t>(start - tracks.begin())]};
	std::vector<std::size_t> passes;
	for (std::size_t index = 0; index < g_vias.size(); ++index)
		if (g_vias[index].line == Line)
			passes.push_back(index);
	auto route{editor_track::run_route(*first, kReach)};
	through(route, passes);
	if (route.spans.empty())
		return std::nullopt;
	// the marks the main paths of the switches lead away from: the line goes their way
	std::set<std::size_t> tried;
	std::vector<editor_track::route_sample> samples;
	pairing pairs;
	auto const origin_of = [&](std::vector<editor_track::route_sample> const &Samples) {
		std::size_t origin{0};
		auto nearest{std::numeric_limits<double>::max()};
		for (std::size_t i = 0; i < Samples.size(); ++i)
			if (auto const distance{plan_distance(Samples[i].position, g_marks[first_mark].point)}; distance < nearest)
			{
				nearest = distance;
				origin = i;
			}
		return origin;
	};
	for (std::size_t round = 0; round <= own.size(); ++round)
	{
		samples = editor_track::sample_route(route, kStep);
		// the track beside goes along the vias the line doesn't
		std::vector<std::size_t> beside;
		std::copy_if(passes.begin(), passes.end(), std::back_inserter(beside), [&](std::size_t const Via) { return false == runs_by(route, Via); });
		pairs = pair_up(samples, route, origin_of(samples), beside);
		std::size_t away{own.size()};
		for (std::size_t k = 0; k < own.size() && away == own.size(); ++k)
			if (tracks[k] != nullptr && tried.count(k) == 0 && plan_gap(samples, g_marks[own[k]].point) > kOnAxis && plan_gap(pairs.partner, g_marks[own[k]].point) > kOnAxis)
				away = k;
		if (away == own.size())
			break;
		tried.insert(away);
		editor_track::route found;
		std::string error;
		if (false == editor_track::find_route(first, tracks[away], found, error) || found.spans.size() < 2)
			continue;
		auto const at{index_of(route.spans, first)};
		if (at == route.spans.size())
			break;
		std::vector<span> spans;
		if (found.spans.front().forward == route.spans[at].forward)
		{
			spans.assign(route.spans.begin(), route.spans.begin() + at);
			spans.insert(spans.end(), found.spans.begin(), found.spans.end());
			auto const more{onward(found.spans.back())};
			spans.insert(spans.end(), more.begin(), more.end());
		}
		else
		{
			spans = flipped(onward(found.spans.back()));
			auto const back{flipped(found.spans)};
			spans.insert(spans.end(), back.begin(), back.end());
			spans.insert(spans.end(), route.spans.begin() + at + 1, route.spans.end());
		}
		route.spans = once(spans);
		through(route, passes);
	}
	if (samples.size() < 2)
		return std::nullopt;
	axis result;
	result.line = Line;
	compose(result, samples, pairs);
	for (auto const &entry : route.spans)
		result.paths.emplace_back(entry.track, entry.path);
	auto const paired{std::any_of(pairs.match.begin(), pairs.match.end(), [](int const Match) { return Match >= 0; })};
	if (paired)
		for (auto const &entry : pairs.route.spans)
			result.paths.emplace_back(entry.track, entry.path);
	for (auto const index : passes)
		if (false == runs_by(route, index) && (false == paired || false == runs_by(pairs.route, index)))
			result.blocked.push_back(index);
	for (auto const index : own)
	{
		axis::anchor anchor;
		anchor.mark = index;
		anchor.chainage = chainage_at(result, g_marks[index].point, anchor.gap);
		if (anchor.gap > kOnAxis)
		{
			result.astray.push_back(index);
			continue;
		}
		anchor.before = g_marks[index].km;
		anchor.after = g_marks[index].beyond.value_or(g_marks[index].km);
		result.anchors.push_back(anchor);
	}
	if (result.anchors.empty())
		return std::nullopt;
	std::sort(result.anchors.begin(), result.anchors.end(), [](axis::anchor const &A, axis::anchor const &B) { return A.chainage < B.chainage; });
	// the km rise the way the marks grow, the way the only one says
	auto const &low{result.anchors.front()};
	auto const &high{result.anchors.back()};
	if (result.anchors.size() > 1 && std::abs(high.before - low.before) > 1e-6)
		result.growth = high.before > low.before ? 1 : -1;
	else
		result.growth = glm::dot(g_marks[low.mark].way, direction_on(result, low.chainage)) >= 0.0 ? 1 : -1;
	return result;
}

} // namespace

void reset()
{
	g_marks.clear();
	g_vias.clear();
	g_base = {};
	drop_axes();
	g_changed = false;
	g_loaded = false;
}

void load()
{
	if (g_loaded || scene::Layers.empty())
		return;
	g_loaded = true;
	g_marks.clear();
	g_vias.clear();
	drop_axes();
	for (auto const layer : scene::Layers.marked_layers(kMark))
	{
		auto const &context{scene::Layers.layer(layer).context_insert()};
		for (auto const &text : scene::Layers.marked(layer, kMark))
		{
			std::istringstream line{text};
			std::string keyword;
			mark entry;
			entry.layer = layer;
			glm::dvec3 point{0.0};
			glm::dvec2 way{0.0, 1.0};
			line >> keyword;
			if (keyword == "via")
			{
				via entry;
				entry.layer = layer;
				glm::dvec3 point{0.0};
				line >> entry.line >> point.x >> point.y >> point.z;
				if (line.fail() || entry.line.empty())
					continue;
				entry.point = context.to_world(point);
				g_vias.push_back(std::move(entry));
				continue;
			}
			if (keyword != "mark")
				continue;
			line >> entry.line >> point.x >> point.y >> point.z >> keyword >> way.x >> way.y >> keyword >> entry.km;
			if (line.fail() || entry.line.empty() || false == std::isfinite(entry.km))
				continue;
			while (line >> keyword)
				if (keyword == "beyond")
				{
					double beyond{0.0};
					if (line >> beyond)
						entry.beyond = beyond;
				}
			entry.point = context.to_world(point);
			entry.way = plan_of(context.to_world(point + glm::dvec3{way.x, 0.0, way.y}) - entry.point);
			entry.way = glm::length(entry.way) > 1e-12 ? glm::normalize(entry.way) : glm::dvec2{0.0, 1.0};
			g_marks.push_back(std::move(entry));
		}
	}
	g_base = current();
	g_changed = false;
	if (false == g_marks.empty())
		WriteLog("Editor: " + std::to_string(g_marks.size()) + " kilometrage mark(s) and " + std::to_string(g_vias.size()) + " via(s) read from the scenery", logtype::generic);
}

std::vector<mark> const &marks()
{
	load();
	return g_marks;
}

std::vector<via> const &vias()
{
	load();
	return g_vias;
}

state current()
{
	return {g_marks, g_vias};
}

void put(mark Entry)
{
	load();
	auto *track{track_near(Entry.point, kNear)};
	Entry.layer = track != nullptr ? alignment_store::layer_of(*track) : scene::Layers.active();
	if (false == scene::Layers.valid(Entry.layer))
		Entry.layer = 1;
	auto const layer{Entry.layer};
	g_marks.push_back(std::move(Entry));
	changed({layer});
}

void change(std::size_t const Index, mark Entry)
{
	load();
	if (Index >= g_marks.size())
		return;
	Entry.layer = g_marks[Index].layer;
	g_marks[Index] = std::move(Entry);
	changed({g_marks[Index].layer});
}

void remove(std::size_t const Index)
{
	load();
	if (Index >= g_marks.size())
		return;
	auto const layer{g_marks[Index].layer};
	g_marks.erase(g_marks.begin() + static_cast<std::ptrdiff_t>(Index));
	changed({layer});
}

void put(via Entry)
{
	load();
	int path{0};
	auto *track{track_near(Entry.point, kNear, path)};
	Entry.layer = track != nullptr ? alignment_store::layer_of(*track) : scene::Layers.active();
	if (false == scene::Layers.valid(Entry.layer))
		Entry.layer = 1;
	auto const layer{Entry.layer};
	g_vias.push_back(std::move(Entry));
	changed({layer});
}

void remove_via(std::size_t const Index)
{
	load();
	if (Index >= g_vias.size())
		return;
	auto const layer{g_vias[Index].layer};
	g_vias.erase(g_vias.begin() + static_cast<std::ptrdiff_t>(Index));
	changed({layer});
}

std::optional<state> take_changes()
{
	if (false == g_changed)
		return std::nullopt;
	auto before{std::move(g_base)};
	g_base = current();
	g_changed = false;
	return before;
}

void settle()
{
	g_base = current();
	g_changed = false;
}

void restore(state State)
{
	std::set<scene::layer_handle> layers;
	for (auto const *list : {&g_marks, &State.marks})
		for (auto const &entry : *list)
			layers.insert(entry.layer);
	for (auto const *list : {&g_vias, &State.vias})
		for (auto const &entry : *list)
			layers.insert(entry.layer);
	g_marks = std::move(State.marks);
	g_vias = std::move(State.vias);
	changed(layers);
	settle();
}

void follow(std::size_t const Edits)
{
	if (Edits == g_edits)
		return;
	g_edits = Edits;
	drop_axes();
}

std::size_t revision()
{
	return g_revision;
}

std::vector<std::string> lines()
{
	load();
	std::vector<std::string> result;
	for (auto const &entry : g_marks)
		if (std::find(result.begin(), result.end(), entry.line) == result.end())
			result.push_back(entry.line);
	std::sort(result.begin(), result.end());
	return result;
}

axis const *axis_of(std::string const &Line)
{
	load();
	auto found{g_axes.find(Line)};
	if (found == g_axes.end())
		found = g_axes.emplace(Line, build(Line)).first;
	return found->second.has_value() ? &*found->second : nullptr;
}

double km_on(axis const &Axis, double const Chainage)
{
	auto const &anchors{Axis.anchors};
	if (anchors.empty())
		return 0.0;
	auto const growth{static_cast<double>(Axis.growth)};
	// the km of an anchor on the side of the lower chainage, and of the higher one
	auto const lower = [&](axis::anchor const &Anchor) { return Axis.growth > 0 ? Anchor.before : Anchor.after; };
	auto const higher = [&](axis::anchor const &Anchor) { return Axis.growth > 0 ? Anchor.after : Anchor.before; };
	if (Chainage <= anchors.front().chainage)
		return lower(anchors.front()) - growth * (anchors.front().chainage - Chainage) / 1000.0;
	if (Chainage >= anchors.back().chainage)
		return higher(anchors.back()) + growth * (Chainage - anchors.back().chainage) / 1000.0;
	auto const next{std::upper_bound(anchors.begin(), anchors.end(), Chainage, [](double const Value, axis::anchor const &Anchor) { return Value < Anchor.chainage; })};
	auto const &to{*next};
	auto const &from{*std::prev(next)};
	auto const span{to.chainage - from.chainage};
	if (span < 1e-6)
		return higher(from);
	return higher(from) + (lower(to) - higher(from)) * (Chainage - from.chainage) / span;
}

double km_on(axis const &Axis, axis::side const &Side, double const Along)
{
	auto const low{km_on(Axis, Side.from + 1e-6)};
	auto const high{km_on(Axis, Side.to - 1e-6)};
	auto const length{Side.length};
	if (length < 1e-6 || Side.to - Side.from < 1e-6)
		return low;
	// along the rising km: from the km the stretch starts at, the irregular hectometre before the one it ends at
	auto const rising{Axis.growth > 0};
	auto const start{rising ? low : high};
	auto const end{rising ? high : low};
	auto const s{std::clamp(rising ? Along : length - Along, 0.0, length)};
	auto const scale{std::abs(end - start) / ((Side.to - Side.from) / 1000.0)}; // km of the axis to a km of the scenery
	auto const regular{start + scale * length / 1000.0};
	auto const last{(std::ceil(std::min(regular, end) * 10.0 - 1e-9) - 1.0) / 10.0};
	if (end <= start || last <= start || scale <= 1e-9)
		return start + (end - start) * s / length;
	auto const at{(last - start) / scale * 1000.0};
	if (s <= at || at >= length)
		return start + scale * s / 1000.0;
	return last + (end - last) * (s - at) / (length - at);
}

double chainage_of(axis const &Axis, glm::dvec3 const &Point, double &Gap)
{
	Gap = std::numeric_limits<double>::max();
	auto result{0.0};
	auto const point{plan_of(Point)};
	for (std::size_t i = 1; i < Axis.points.size(); ++i)
	{
		auto const a{plan_of(Axis.points[i - 1])};
		auto const along{plan_of(Axis.points[i]) - a};
		auto const length2{glm::dot(along, along)};
		auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, along) / length2, 0.0, 1.0) : 0.0};
		if (auto const gap{glm::distance(point, a + along * t)}; gap < Gap)
		{
			Gap = gap;
			result = Axis.chainage[i - 1] + (Axis.chainage[i] - Axis.chainage[i - 1]) * t;
		}
	}
	return result;
}

double chainage_at(axis const &Axis, glm::dvec3 const &Point, double &Gap)
{
	auto result{chainage_of(Axis, Point, Gap)};
	auto const point{plan_of(Point)};
	for (auto const &entry : Axis.sides)
		for (std::size_t i = 1; i < entry.points.size(); ++i)
		{
			auto const a{plan_of(entry.points[i - 1])};
			auto const along{plan_of(entry.points[i]) - a};
			auto const length2{glm::dot(along, along)};
			auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, along) / length2, 0.0, 1.0) : 0.0};
			if (auto const gap{glm::distance(point, a + along * t)}; gap < Gap)
			{
				Gap = gap;
				result = entry.chainage[i - 1] + (entry.chainage[i] - entry.chainage[i - 1]) * t;
			}
		}
	return result;
}

glm::dvec3 point_on(axis const &Axis, double const Chainage)
{
	if (Axis.points.empty())
		return glm::dvec3{0.0};
	auto const after{std::lower_bound(Axis.chainage.begin(), Axis.chainage.end(), Chainage)};
	if (after == Axis.chainage.begin())
		return Axis.points.front();
	if (after == Axis.chainage.end())
		return Axis.points.back();
	auto const i{static_cast<std::size_t>(after - Axis.chainage.begin())};
	auto const step{Axis.chainage[i] - Axis.chainage[i - 1]};
	return glm::mix(Axis.points[i - 1], Axis.points[i], step > 1e-9 ? (Chainage - Axis.chainage[i - 1]) / step : 0.0);
}

glm::dvec2 direction_on(axis const &Axis, double const Chainage)
{
	if (Axis.directions.empty())
		return {0.0, 1.0};
	auto const after{std::lower_bound(Axis.chainage.begin(), Axis.chainage.end(), Chainage)};
	auto const i{std::min(Axis.directions.size() - 1, static_cast<std::size_t>(after - Axis.chainage.begin()))};
	return Axis.directions[i];
}

bool km_at(std::string const &Line, glm::dvec3 const &Point, double &Km, double const Reach)
{
	auto const *found{axis_of(Line)};
	if (found == nullptr)
		return false;
	double gap{0.0};
	auto const chainage{chainage_of(*found, Point, gap)};
	auto const point{plan_of(Point)};
	axis::side const *apart{nullptr};
	auto along{0.0};
	for (auto const &entry : found->sides)
		for (std::size_t i = 1; i < entry.points.size(); ++i)
		{
			auto const a{plan_of(entry.points[i - 1])};
			auto const ab{plan_of(entry.points[i]) - a};
			auto const length2{glm::dot(ab, ab)};
			auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0) : 0.0};
			if (auto const distance{glm::distance(point, a + ab * t)}; distance < gap)
			{
				gap = distance;
				apart = &entry;
				along = entry.along[i - 1] + (entry.along[i] - entry.along[i - 1]) * t;
			}
		}
	if (gap > Reach)
		return false;
	Km = apart != nullptr ? km_on(*found, *apart, along) : km_on(*found, chainage);
	return true;
}

TTrack *track_near(glm::dvec3 const &Point, double const Reach)
{
	TTrack *best{nullptr};
	auto nearest{Reach};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 1) == 0 || false == editor_track::is_supported(*track) || track->m_paths.empty())
			continue;
		auto const &ends{track->m_paths.front().points};
		auto const chord{plan_distance(ends[segment_data::point::start], ends[segment_data::point::end])};
		if (std::min(plan_distance(ends[segment_data::point::start], Point), plan_distance(ends[segment_data::point::end], Point)) > chord * 0.8 + nearest + 1.0)
			continue;
		auto const on{editor_track::point_at(*track, editor_track::nearest_parameter(*track, Point))};
		if (auto const distance{plan_distance(on, Point)}; distance < nearest)
		{
			nearest = distance;
			best = track;
		}
	}
	return best;
}

int path_near(TTrack const &Track, glm::dvec3 const &Point, glm::dvec3 &On)
{
	int result{0};
	auto nearest{std::numeric_limits<double>::max()};
	On = Point;
	for (int path = 0; path < static_cast<int>(Track.m_paths.size()); ++path)
	{
		geometry::bezier const curve{Track.m_paths[path]};
		auto previous{curve.point(0.0)};
		for (int i = 1; i <= 64; ++i)
		{
			auto const next{curve.point(i / 64.0)};
			auto const ab{plan_of(next - previous)};
			auto const length2{glm::dot(ab, ab)};
			auto const t{length2 > 1e-12 ? std::clamp(glm::dot(plan_of(Point - previous), ab) / length2, 0.0, 1.0) : 0.0};
			auto const at{glm::mix(previous, next, t)};
			// where the paths of a switch run together, the first one
			if (auto const distance{plan_distance(at, Point)}; distance < nearest - 1e-3)
			{
				nearest = distance;
				result = path;
				On = at;
			}
			previous = next;
		}
	}
	return result;
}

TTrack *track_near(glm::dvec3 const &Point, double const Reach, int &Path)
{
	TTrack *best{nullptr};
	auto nearest{Reach};
	Path = 0;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 1) == 0 || false == editor_track::is_supported(*track) || track->m_paths.empty())
			continue;
		auto const &ends{track->m_paths.front().points};
		auto const chord{plan_distance(ends[segment_data::point::start], ends[segment_data::point::end])};
		if (std::min(plan_distance(ends[segment_data::point::start], Point), plan_distance(ends[segment_data::point::end], Point)) > chord * 0.8 + nearest + 1.0)
			continue;
		glm::dvec3 on{0.0};
		auto const path{path_near(*track, Point, on)};
		if (auto const distance{plan_distance(on, Point)}; distance < nearest)
		{
			nearest = distance;
			best = track;
			Path = path;
		}
	}
	return best;
}

std::string text(double const Km)
{
	auto result{format("%.3f", Km)};
	std::replace(result.begin(), result.end(), '.', ',');
	return result;
}

} // namespace kilometrage
