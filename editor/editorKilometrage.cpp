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
#include <format>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string_view>
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

// the kilometrage set, and the axes found from it
struct kept_state
{
	std::vector<mark> marks;
	std::vector<via> vias;
	state base; // as it was before the changes not yet in the undo history
	bool changed{false};
	bool loaded{false};
	std::size_t edits{0};
	std::map<std::string, std::optional<axis>, std::less<>> axes;
	std::size_t revision{0};
};

kept_state &kept()
{
	static kept_state value;
	return value;
}

void drop_axes()
{
	kept().axes.clear();
	++kept().revision;
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
	for (auto const &entry : kept().marks)
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
	for (auto const &entry : kept().vias)
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
	kept().changed = true;
	drop_axes();
	for (auto const layer : Layers)
		store(layer);
}

std::vector<span> flipped(std::vector<span> Spans)
{
	std::ranges::reverse(Spans);
	for (auto &entry : Spans)
		entry.forward = false == entry.forward;
	return Spans;
}

std::size_t index_of(std::vector<span> const &Spans, TTrack const *Track)
{
	auto const found{std::ranges::find_if(Spans, [Track](span const &Entry) { return Entry.track == Track; })};
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
	auto const *track{track_near(kept().vias[Via].point, kNear, path)};
	return track != nullptr && std::ranges::any_of(Route.spans, [track, path](span const &Entry) { return Entry.track == track && Entry.path == path; });
}

// joined in the order given
std::vector<span> joined(std::vector<std::vector<span>> const &Parts)
{
	std::vector<span> result;
	for (auto const &entry : Parts)
		result.insert(result.end(), entry.begin(), entry.end());
	return result;
}

// the last span of the run before At, of a track the route runs through
std::optional<std::size_t> rejoins_before(std::vector<span> const &Route, std::vector<span> const &Run, std::size_t const At)
{
	for (auto i = At; i-- > 0;)
		if (index_of(Route, Run[i].track) != Route.size())
			return i;
	return std::nullopt;
}

// the first span of the run after At, of a track the route runs through
std::optional<std::size_t> rejoins_after(std::vector<span> const &Route, std::vector<span> const &Run, std::size_t const At)
{
	for (auto i = At + 1; i < Run.size(); ++i)
		if (index_of(Route, Run[i].track) != Route.size())
			return i;
	return std::nullopt;
}

// the spans of the route with the run from the track of a via in place of the stretch between where the run comes back onto it,
// Low before the via and High after it; none where the route stays as it is
std::optional<std::vector<span>> rerouted(std::vector<span> const &Route, std::vector<span> const &Run, std::optional<std::size_t> const Low, std::optional<std::size_t> const High)
{
	auto const on = [&Route, &Run](std::size_t const Index) { return index_of(Route, Run[Index].track); };
	auto const part = [&Run](std::size_t const From, std::size_t const To) { return std::vector<span>{Run.begin() + From, Run.begin() + To}; };
	auto const head = [&Route](std::size_t const To) { return std::vector<span>{Route.begin(), Route.begin() + To}; };
	auto const tail = [&Route](std::size_t const From) { return std::vector<span>{Route.begin() + From, Route.end()}; };
	if (Low.has_value() && High.has_value())
	{
		auto const from{on(*Low)};
		auto const to{on(*High)};
		if (from == to)
			return std::nullopt;
		return from < to ? joined({head(from), part(*Low, *High + 1), tail(to + 1)}) : joined({head(to), flipped(part(*Low, *High + 1)), tail(from + 1)});
	}
	if (Low.has_value())
	{
		auto const from{on(*Low)};
		return Run[*Low].forward == Route[from].forward ? joined({head(from), part(*Low, Run.size())}) : joined({flipped(part(*Low, Run.size())), tail(from + 1)});
	}
	if (High.has_value())
	{
		auto const to{on(*High)};
		return Run[*High].forward == Route[to].forward ? joined({part(0, *High + 1), tail(to + 1)}) : joined({head(to), flipped(part(0, *High + 1))});
	}
	return std::nullopt;
}

// the route led along the tracks of the vias: from the track of each the run along the straight paths of the switches both ways
// back onto the route takes the place of the stretch of the route between where it comes back; where it comes back one way only,
// as at a junction, the route goes on along it from there
void through(editor_track::route &Route, std::vector<std::size_t> const &Vias)
{
	for (auto const index : Vias)
	{
		int path{0};
		auto *track{track_near(kept().vias[index].point, kNear, path)};
		if (track == nullptr || runs_by(Route, index))
			continue;
		auto const run{editor_track::run_route(*track, kDetour, path)};
		auto const at{index_of(run.spans, track)};
		if (at == run.spans.size())
			continue;
		auto const low{rejoins_before(Route.spans, run.spans, at)};
		auto const high{rejoins_after(Route.spans, run.spans, at)};
		if (auto const spans{rerouted(Route.spans, run.spans, low, high)}; spans.has_value())
			Route.spans = once(*spans);
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

// the samples of the track beside by the squares of kCell they lie in
using sample_grid = std::map<std::pair<long, long>, std::vector<std::size_t>>;

std::pair<long, long> cell_of(glm::dvec3 const &Point)
{
	return std::make_pair(static_cast<long>(std::floor(Point.x / kCell)), static_cast<long>(std::floor(Point.z / kCell)));
}

// the sample beside the one of the line, a double-track spacing away and running its way: kept in Match and Across when it lies
// nearer along it than Best, of the ones in a square of the grid
void match_in_cell(editor_track::route_sample const &Sample, std::vector<editor_track::route_sample> const &Partner, std::vector<std::size_t> const &Cell, double &Best, int &Match, double &Across)
{
	glm::dvec2 const normal{-Sample.direction.y, Sample.direction.x};
	for (auto const j : Cell)
	{
		auto const offset{plan_of(Partner[j].position) - plan_of(Sample.position)};
		auto const across{glm::dot(offset, normal)};
		auto const along{glm::dot(offset, Sample.direction)};
		if (std::abs(across) < kBetweenLow || std::abs(across) > kBetweenHigh || std::abs(along) >= kStep * 1.5 || std::abs(glm::dot(Sample.direction, Partner[j].direction)) <= 0.97 || std::abs(along) >= Best)
			continue;
		Best = std::abs(along);
		Match = static_cast<int>(j);
		Across = across;
	}
}

// the sample beside the one of the line nearest along it, of the squares of the grid around it: kept in Match and Across
void match_around(editor_track::route_sample const &Sample, std::vector<editor_track::route_sample> const &Partner, sample_grid const &Grid, int &Match, double &Across)
{
	auto const [cx, cz]{cell_of(Sample.position)};
	auto best{std::numeric_limits<double>::max()};
	for (auto x = cx - 1; x <= cx + 1; ++x)
		for (auto z = cz - 1; z <= cz + 1; ++z)
			if (auto const found{Grid.find({x, z})}; found != Grid.end())
				match_in_cell(Sample, Partner, found->second, best, Match, Across);
}

pairing pair_with(std::vector<editor_track::route_sample> const &Samples, TTrack &Beside, std::vector<std::size_t> const &Vias)
{
	pairing result;
	result.match.assign(Samples.size(), -1);
	result.across.assign(Samples.size(), 0.0);
	result.route = editor_track::run_route(Beside, kReach);
	through(result.route, Vias);
	result.partner = editor_track::sample_route(result.route, kStep);
	auto const &partner{result.partner};
	sample_grid grid;
	for (std::size_t j = 0; j < partner.size(); ++j)
		grid[cell_of(partner[j].position)].push_back(j);
	for (std::size_t i = 0; i < Samples.size(); ++i)
		match_around(Samples[i], partner, grid, result.match[i], result.across[i]);
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
	auto const on_line = [&Pairs, &Own](editor_track::route_sample const &Sample) { return Sample.span < Pairs.route.spans.size() && Own.contains(Pairs.route.spans[Sample.span].track); };
	if (low >= Pairs.partner.size())
		return false;
	auto const first{Pairs.partner.begin() + static_cast<std::ptrdiff_t>(low)};
	auto const last{Pairs.partner.begin() + static_cast<std::ptrdiff_t>(high) + 1};
	return std::any_of(first, last, on_line) || (std::any_of(Pairs.partner.begin(), first, on_line) && std::any_of(last, Pairs.partner.end(), on_line));
}

// the track beside the line looked for on both sides of the sample, of the tracks not tried yet: a track the line doesn't run
// through, the spacing of a double-track line away, beside it for 100 m at least
std::optional<pairing> pair_by(std::vector<editor_track::route_sample> const &Samples, std::size_t const Index, std::unordered_set<TTrack const *> const &Own, std::unordered_set<TTrack const *> &Tried, std::vector<std::size_t> const &Vias)
{
	auto const &at{Samples[Index]};
	glm::dvec2 const normal{-at.direction.y, at.direction.x};
	for (auto const side : {1.0, -1.0})
	{
		auto const probe{plan_of(at.position) + normal * (side * (kBetweenLow + kBetweenHigh) * 0.5)};
		auto *track{track_near({probe.x, at.position.y, probe.y}, 1.6)};
		if (track == nullptr || Own.contains(track) || false == Tried.insert(track).second)
			continue;
		auto result{pair_with(Samples, *track, Vias)};
		if (static_cast<double>(std::ranges::count_if(result.match, [](int const Match) { return Match >= 0; })) * kStep >= 100.0 && false == looped(result, Own))
			return result;
	}
	return std::nullopt;
}

// the track beside the line, looked for by the first mark and then every 500 m along the line
pairing pair_up(std::vector<editor_track::route_sample> const &Samples, editor_track::route const &Route, std::size_t const Origin, std::vector<std::size_t> const &Vias)
{
	std::unordered_set<TTrack const *> own;
	for (auto const &entry : Route.spans)
		own.insert(entry.track);
	std::unordered_set<TTrack const *> tried;
	auto const step{static_cast<std::size_t>(500.0 / kStep)};
	std::size_t k{0};
	while (k * step < Samples.size() && tried.size() < 4)
	{
		for (auto const way : {1, -1})
		{
			if (k == 0 && way < 0)
				continue;
			auto const index{static_cast<long>(Origin) + way * static_cast<long>(k * step)};
			if (index < 0 || index >= static_cast<long>(Samples.size()))
				continue;
			if (auto found{pair_by(Samples, static_cast<std::size_t>(index), own, tried, Vias)}; found.has_value())
				return std::move(*found);
		}
		++k;
	}
	pairing unpaired;
	unpaired.match.assign(Samples.size(), -1);
	unpaired.across.assign(Samples.size(), 0.0);
	return unpaired;
}

// the next point of the axis, its chainage on from the one before
void emit(axis &Result, glm::dvec3 const &Point, glm::dvec2 const &Direction)
{
	Result.chainage.push_back(Result.points.empty() ? 0.0 : Result.chainage.back() + plan_distance(Result.points.back(), Point));
	Result.points.push_back(Point);
	Result.directions.push_back(Direction);
}

// the point moved by the offset to the left of the direction
glm::dvec3 shifted(glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Offset)
{
	auto const planar{plan_of(Point) + glm::dvec2{-Direction.y, Direction.x} * Offset};
	return glm::dvec3{planar.x, Point.y, planar.y};
}

// from between the tracks over to one of them, within kBlend of the ends of the stretch apart
double eased(double const From, double const To, double const Low, double const High)
{
	return From < To ? Low * (1.0 - std::clamp(From / kBlend, 0.0, 1.0)) : High * (1.0 - std::clamp(To / kBlend, 0.0, 1.0));
}

// the tracks apart between the samples A and B of the line: the track beside from the sample by A to the one by B, turned along
// the line
std::vector<editor_track::route_sample> beside_between(pairing const &Pairs, std::size_t const A, std::size_t const B)
{
	auto const ja{Pairs.match[A]};
	auto const jb{Pairs.match[B]};
	std::vector<editor_track::route_sample> beside;
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
	return beside;
}

// of each sample the length from the first one along them
std::vector<double> lengths_along(std::vector<editor_track::route_sample> const &Samples)
{
	std::vector along{0.0};
	for (std::size_t q = 1; q < Samples.size(); ++q)
		along.push_back(along.back() + plan_distance(Samples[q - 1].position, Samples[q].position));
	return along;
}

// the point at the share of the length along the samples, Along: their lengths from the first one
glm::dvec3 at_share(std::vector<editor_track::route_sample> const &Samples, std::vector<double> const &Along, double const Share)
{
	auto const target{Share * Along.back()};
	auto const next{std::min<std::size_t>(Samples.size() - 1, static_cast<std::size_t>(std::ranges::lower_bound(Along, target) - Along.begin()))};
	if (next == 0)
		return Samples.front().position;
	auto const piece{Along[next] - Along[next - 1]};
	return glm::mix(Samples[next - 1].position, Samples[next].position, piece > 1e-9 ? (target - Along[next - 1]) / piece : 0.0);
}

// the axis between the samples A and B of the line, where the track beside isn't matched to the samples between: between the
// tracks still where they spread a little, else along the longer one. returns: the other one, Shares: of its length, the ones its
// points lie at, for their chainage once the axis reaches B; none where the axis runs between the tracks
std::optional<axis::side> apart(axis &Result, std::vector<editor_track::route_sample> const &Samples, pairing const &Pairs, std::size_t const A, std::size_t const B, std::vector<double> &Shares)
{
	auto const beside{beside_between(Pairs, A, B)};
	auto const along{lengths_along(beside)};
	auto const own_length{Samples[B].chainage - Samples[A].chainage};
	auto const beside_length{along.back()};
	auto const half_a{Pairs.across[A] * 0.5};
	auto const half_b{Pairs.across[B] * 0.5};
	auto widest{0.0};
	for (auto i = A + 1; i < B; ++i)
		widest = std::max(widest, plan_distance(Samples[i].position, at_share(beside, along, (Samples[i].chainage - Samples[A].chainage) / own_length)));
	if (widest <= kApart || own_length < 1e-6 || beside_length < 1e-6)
	{
		// spread a little, as through a station: between them still
		for (auto i = A + 1; i < B; ++i)
		{
			auto const t{(Samples[i].chainage - Samples[A].chainage) / own_length};
			emit(Result, shifted(Samples[i].position, Samples[i].direction, half_a + (half_b - half_a) * t), Samples[i].direction);
		}
		return std::nullopt;
	}
	axis::side stretch;
	Shares.clear();
	stretch.from = Result.chainage.back();
	if (beside_length > own_length + kLonger)
	{
		for (std::size_t q = 1; q + 1 < beside.size(); ++q)
			emit(Result, shifted(beside[q].position, beside[q].direction, -eased(along[q], beside_length - along[q], half_a, half_b)), beside[q].direction);
		stretch.length = own_length;
		for (auto i = A + 1; i < B; ++i)
		{
			stretch.points.push_back(Samples[i].position);
			stretch.along.push_back(Samples[i].chainage - Samples[A].chainage);
			Shares.push_back(stretch.along.back() / own_length);
		}
	}
	else
	{
		for (auto i = A + 1; i < B; ++i)
			emit(Result, shifted(Samples[i].position, Samples[i].direction, eased(Samples[i].chainage - Samples[A].chainage, Samples[B].chainage - Samples[i].chainage, half_a, half_b)), Samples[i].direction);
		stretch.length = beside_length;
		for (std::size_t q = 1; q + 1 < beside.size(); ++q)
		{
			stretch.points.push_back(beside[q].position);
			stretch.along.push_back(along[q]);
			Shares.push_back(along[q] / beside_length);
		}
	}
	return stretch;
}

// the axis: between the tracks where they run side by side, along the longer one where they part and come together again
// (PKP GK-1 6.1.12), the other one getting the km by the share of its length, on the track of the line where the other one ends
void compose(axis &Result, std::vector<editor_track::route_sample> const &Samples, pairing const &Pairs)
{
	std::vector<std::size_t> matched;
	for (std::size_t i = 0; i < Samples.size(); ++i)
		if (Pairs.match[i] >= 0)
			matched.push_back(i);
	if (matched.empty())
	{
		for (auto const &entry : Samples)
			emit(Result, entry.position, entry.direction);
		return;
	}
	Result.between = true;
	auto const half = [&Pairs](std::size_t const Index) { return Pairs.across[Index] * 0.5; };
	auto const first{matched.front()};
	for (std::size_t i = 0; i < first; ++i)
		emit(Result, shifted(Samples[i].position, Samples[i].direction, half(first) * std::clamp(1.0 - (Samples[first].chainage - Samples[i].chainage) / kBlend, 0.0, 1.0)), Samples[i].direction);
	std::optional<axis::side> pending;
	std::vector<double> shares;
	for (std::size_t k = 0; k < matched.size(); ++k)
	{
		auto const a{matched[k]};
		emit(Result, shifted(Samples[a].position, Samples[a].direction, half(a)), Samples[a].direction);
		if (pending.has_value())
		{
			pending->to = Result.chainage.back();
			for (auto const share : shares)
				pending->chainage.push_back(pending->from + share * (pending->to - pending->from));
			Result.sides.push_back(std::move(*pending));
			pending.reset();
		}
		if (k + 1 == matched.size())
			break;
		if (auto const b{matched[k + 1]}; b != a + 1)
			pending = apart(Result, Samples, Pairs, a, b, shares);
	}
	auto const last{matched.back()};
	for (auto i = last + 1; i < Samples.size(); ++i)
		emit(Result, shifted(Samples[i].position, Samples[i].direction, half(last) * std::clamp(1.0 - (Samples[i].chainage - Samples[last].chainage) / kBlend, 0.0, 1.0)), Samples[i].direction);
}

// the marks and the vias of a line, with the tracks of the marks
struct line_marks
{
	std::vector<std::size_t> own; // of the marks
	std::vector<TTrack *> tracks; // of each of them, nullptr where none is near
	std::vector<std::size_t> passes; // of the vias
	TTrack *first{nullptr}; // of the first mark on a track
	std::size_t first_mark{0};
};

// none where no mark of the line stands on a track
std::optional<line_marks> marks_of(std::string_view const Line)
{
	auto const &placed{kept().marks};
	line_marks result;
	for (std::size_t index = 0; index < placed.size(); ++index)
		if (placed[index].line == Line)
			result.own.push_back(index);
	if (result.own.empty())
		return std::nullopt;
	for (auto const index : result.own)
		result.tracks.push_back(track_near(placed[index].point, kNear));
	auto const start{std::ranges::find_if(result.tracks, [](TTrack const *Track) { return Track != nullptr; })};
	if (start == result.tracks.end())
		return std::nullopt;
	result.first = *start;
	result.first_mark = result.own[static_cast<std::size_t>(start - result.tracks.begin())];
	auto const &passing{kept().vias};
	for (std::size_t index = 0; index < passing.size(); ++index)
		if (passing[index].line == Line)
			result.passes.push_back(index);
	return result;
}

// the sample nearest to the point, in the plan
std::size_t nearest_sample(std::vector<editor_track::route_sample> const &Samples, glm::dvec3 const &Point)
{
	std::size_t origin{0};
	auto nearest{std::numeric_limits<double>::max()};
	for (std::size_t i = 0; i < Samples.size(); ++i)
		if (auto const distance{plan_distance(Samples[i].position, Point)}; distance < nearest)
		{
			nearest = distance;
			origin = i;
		}
	return origin;
}

// the first mark on a track, of the ones not tried yet, away from the axis of the line and from the track beside: the count of
// the marks for none
std::size_t astray_mark(line_marks const &Marks, std::set<std::size_t> const &Tried, std::vector<editor_track::route_sample> const &Samples, std::vector<editor_track::route_sample> const &Beside)
{
	auto const &placed{kept().marks};
	for (std::size_t k = 0; k < Marks.own.size(); ++k)
		if (Marks.tracks[k] != nullptr && false == Tried.contains(k) && plan_gap(Samples, placed[Marks.own[k]].point) > kOnAxis && plan_gap(Beside, placed[Marks.own[k]].point) > kOnAxis)
			return k;
	return Marks.own.size();
}

// the route led along the one found from the track of the first mark, then on from its end along the main paths of the
// switches. returns: false where the route doesn't run through the track of the first mark
bool led_along(editor_track::route &Route, TTrack const *First, editor_track::route const &Found)
{
	auto const at{index_of(Route.spans, First)};
	if (at == Route.spans.size())
		return false;
	std::vector<span> spans;
	if (Found.spans.front().forward == Route.spans[at].forward)
	{
		spans.assign(Route.spans.begin(), Route.spans.begin() + at);
		spans.insert(spans.end(), Found.spans.begin(), Found.spans.end());
		auto const more{onward(Found.spans.back())};
		spans.insert(spans.end(), more.begin(), more.end());
	}
	else
	{
		spans = flipped(onward(Found.spans.back()));
		auto const back{flipped(Found.spans)};
		spans.insert(spans.end(), back.begin(), back.end());
		spans.insert(spans.end(), Route.spans.begin() + at + 1, Route.spans.end());
	}
	Route.spans = once(spans);
	return true;
}

// the route led to a mark away from it, of the ones not tried yet, and through the vias again: the marks the main paths of the
// switches lead away from, the line goes their way. returns: false once no mark is left away from it, or it can't be led
bool lead_to_mark(editor_track::route &Route, line_marks const &Marks, std::set<std::size_t> &Tried, std::vector<editor_track::route_sample> const &Samples, pairing const &Pairs)
{
	auto const away{astray_mark(Marks, Tried, Samples, Pairs.partner)};
	if (away == Marks.own.size())
		return false;
	Tried.insert(away);
	editor_track::route found;
	if (std::string error; false == editor_track::find_route(Marks.first, Marks.tracks[away], found, error) || found.spans.size() < 2)
		return true;
	if (false == led_along(Route, Marks.first, found))
		return false;
	through(Route, Marks.passes);
	return true;
}

// the paths of the tracks the line runs along, of the track beside too, and the vias it doesn't run along
void list_paths(axis &Result, editor_track::route const &Route, pairing const &Pairs, std::vector<std::size_t> const &Passes)
{
	for (auto const &entry : Route.spans)
		Result.paths.emplace_back(entry.track, entry.path);
	auto const paired{std::ranges::any_of(Pairs.match, [](int const Match) { return Match >= 0; })};
	if (paired)
		for (auto const &entry : Pairs.route.spans)
			Result.paths.emplace_back(entry.track, entry.path);
	for (auto const index : Passes)
		if (false == runs_by(Route, index) && (false == paired || false == runs_by(Pairs.route, index)))
			Result.blocked.push_back(index);
}

// the marks of the line on its axis anchor its km, the rest are astray
void anchor_marks(axis &Result, std::vector<std::size_t> const &Own)
{
	auto const &placed{kept().marks};
	for (auto const index : Own)
	{
		axis::anchor anchor;
		anchor.mark = index;
		anchor.chainage = chainage_at(Result, placed[index].point, anchor.gap);
		if (anchor.gap > kOnAxis)
		{
			Result.astray.push_back(index);
			continue;
		}
		anchor.before = placed[index].km;
		anchor.after = placed[index].beyond.value_or(placed[index].km);
		Result.anchors.push_back(anchor);
	}
}

std::optional<axis> build(std::string_view const Line)
{
	auto const found{marks_of(Line)};
	if (false == found.has_value())
		return std::nullopt;
	auto const &of_line{*found};
	auto route{editor_track::run_route(*of_line.first, kReach)};
	through(route, of_line.passes);
	if (route.spans.empty())
		return std::nullopt;
	std::set<std::size_t> tried;
	std::vector<editor_track::route_sample> samples;
	pairing pairs;
	for (std::size_t round = 0; round <= of_line.own.size(); ++round)
	{
		samples = editor_track::sample_route(route, kStep);
		// the track beside goes along the vias the line doesn't
		std::vector<std::size_t> beside;
		std::ranges::copy_if(of_line.passes, std::back_inserter(beside), [&route](std::size_t const Via) { return false == runs_by(route, Via); });
		pairs = pair_up(samples, route, nearest_sample(samples, kept().marks[of_line.first_mark].point), beside);
		if (false == lead_to_mark(route, of_line, tried, samples, pairs))
			break;
	}
	if (samples.size() < 2)
		return std::nullopt;
	axis result;
	result.line = Line;
	compose(result, samples, pairs);
	list_paths(result, route, pairs, of_line.passes);
	anchor_marks(result, of_line.own);
	if (result.anchors.empty())
		return std::nullopt;
	std::ranges::sort(result.anchors, [](axis::anchor const &A, axis::anchor const &B) { return A.chainage < B.chainage; });
	// the km rise the way the marks grow, the way the only one says
	auto const &low{result.anchors.front()};
	if (auto const &high{result.anchors.back()}; result.anchors.size() > 1 && std::abs(high.before - low.before) > 1e-6)
		result.growth = high.before > low.before ? 1 : -1;
	else
		result.growth = glm::dot(kept().marks[low.mark].way, direction_on(result, low.chainage)) >= 0.0 ? 1 : -1;
	return result;
}

// a via read from the scenery file of the layer, after its keyword: kept unless the line is broken
void read_via(std::istringstream &Line, scene::layer_handle const Layer, scene::layer_context const &Context)
{
	via entry;
	entry.layer = Layer;
	glm::dvec3 point{0.0};
	Line >> entry.line >> point.x >> point.y >> point.z;
	if (Line.fail() || entry.line.empty())
		return;
	entry.point = Context.to_world(point);
	kept().vias.push_back(std::move(entry));
}

// a mark read from the scenery file of the layer, after its keyword: kept unless the line is broken
void read_mark(std::istringstream &Line, scene::layer_handle const Layer, scene::layer_context const &Context)
{
	mark entry;
	entry.layer = Layer;
	glm::dvec3 point{0.0};
	glm::dvec2 way{0.0, 1.0};
	std::string keyword;
	Line >> entry.line >> point.x >> point.y >> point.z >> keyword >> way.x >> way.y >> keyword >> entry.km;
	if (Line.fail() || entry.line.empty() || false == std::isfinite(entry.km))
		return;
	while (Line >> keyword)
		if (keyword == "beyond")
		{
			double beyond{0.0};
			if (Line >> beyond)
				entry.beyond = beyond;
		}
	entry.point = Context.to_world(point);
	entry.way = plan_of(Context.to_world(point + glm::dvec3{way.x, 0.0, way.y}) - entry.point);
	entry.way = glm::length(entry.way) > 1e-12 ? glm::normalize(entry.way) : glm::dvec2{0.0, 1.0};
	kept().marks.push_back(std::move(entry));
}

// a line of the kilometrage read from the scenery file of the layer: a mark or a via
void read_line(std::string const &Text, scene::layer_handle const Layer, scene::layer_context const &Context)
{
	std::istringstream line{Text};
	std::string keyword;
	line >> keyword;
	if (keyword == "via")
		read_via(line, Layer, Context);
	else if (keyword == "mark")
		read_mark(line, Layer, Context);
}

} // namespace

void reset()
{
	kept().marks.clear();
	kept().vias.clear();
	kept().base = {};
	drop_axes();
	kept().changed = false;
	kept().loaded = false;
}

void load()
{
	if (kept().loaded || scene::Layers.empty())
		return;
	kept().loaded = true;
	kept().marks.clear();
	kept().vias.clear();
	drop_axes();
	for (auto const layer : scene::Layers.marked_layers(kMark))
	{
		auto const &context{scene::Layers.layer(layer).context_insert()};
		for (auto const &text : scene::Layers.marked(layer, kMark))
			read_line(text, layer, context);
	}
	kept().base = current();
	kept().changed = false;
	if (false == kept().marks.empty())
		WriteLog(std::format("Editor: {} kilometrage mark(s) and {} via(s) read from the scenery", kept().marks.size(), kept().vias.size()), logtype::generic);
}

std::vector<mark> const &marks()
{
	load();
	return kept().marks;
}

std::vector<via> const &vias()
{
	load();
	return kept().vias;
}

state current()
{
	return {kept().marks, kept().vias};
}

void put(mark Entry)
{
	load();
	auto const *track{track_near(Entry.point, kNear)};
	Entry.layer = track != nullptr ? alignment_store::layer_of(*track) : scene::Layers.active();
	if (false == scene::Layers.valid(Entry.layer))
		Entry.layer = 1;
	auto const layer{Entry.layer};
	kept().marks.push_back(std::move(Entry));
	changed({layer});
}

void change(std::size_t const Index, mark Entry)
{
	load();
	if (Index >= kept().marks.size())
		return;
	Entry.layer = kept().marks[Index].layer;
	kept().marks[Index] = std::move(Entry);
	changed({kept().marks[Index].layer});
}

void remove(std::size_t const Index)
{
	load();
	if (Index >= kept().marks.size())
		return;
	auto const layer{kept().marks[Index].layer};
	kept().marks.erase(kept().marks.begin() + static_cast<std::ptrdiff_t>(Index));
	changed({layer});
}

void put(via Entry)
{
	load();
	int path{0};
	auto const *track{track_near(Entry.point, kNear, path)};
	Entry.layer = track != nullptr ? alignment_store::layer_of(*track) : scene::Layers.active();
	if (false == scene::Layers.valid(Entry.layer))
		Entry.layer = 1;
	auto const layer{Entry.layer};
	kept().vias.push_back(std::move(Entry));
	changed({layer});
}

void remove_via(std::size_t const Index)
{
	load();
	if (Index >= kept().vias.size())
		return;
	auto const layer{kept().vias[Index].layer};
	kept().vias.erase(kept().vias.begin() + static_cast<std::ptrdiff_t>(Index));
	changed({layer});
}

std::optional<state> take_changes()
{
	if (false == kept().changed)
		return std::nullopt;
	auto before{std::move(kept().base)};
	kept().base = current();
	kept().changed = false;
	return before;
}

void settle()
{
	kept().base = current();
	kept().changed = false;
}

void restore(state State)
{
	std::set<scene::layer_handle> layers;
	for (auto const *list : {&kept().marks, &State.marks})
		for (auto const &entry : *list)
			layers.insert(entry.layer);
	for (auto const *list : {&kept().vias, &State.vias})
		for (auto const &entry : *list)
			layers.insert(entry.layer);
	kept().marks = std::move(State.marks);
	kept().vias = std::move(State.vias);
	changed(layers);
	settle();
}

void follow(std::size_t const Edits)
{
	if (Edits == kept().edits)
		return;
	kept().edits = Edits;
	drop_axes();
}

std::size_t revision()
{
	return kept().revision;
}

std::vector<std::string> lines()
{
	load();
	std::vector<std::string> result;
	for (auto const &entry : kept().marks)
		if (std::ranges::find(result, entry.line) == result.end())
			result.push_back(entry.line);
	std::ranges::sort(result);
	return result;
}

axis const *axis_of(std::string const &Line)
{
	load();
	auto found{kept().axes.find(Line)};
	if (found == kept().axes.end())
		found = kept().axes.try_emplace(Line, build(Line)).first;
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
	auto const next{std::ranges::upper_bound(anchors, Chainage, std::ranges::less{}, &axis::anchor::chainage)};
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
	auto const after{std::ranges::lower_bound(Axis.chainage, Chainage)};
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
	auto const after{std::ranges::lower_bound(Axis.chainage, Chainage)};
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

std::string line_at(glm::dvec3 const &Point, double const Reach)
{
	std::string result;
	auto nearest{Reach};
	for (auto const &line : lines())
		if (auto const *found{axis_of(line)}; found != nullptr)
		{
			double gap{0.0};
			chainage_at(*found, Point, gap);
			if (gap < nearest)
			{
				nearest = gap;
				result = line;
			}
		}
	return result;
}

int growth_at(std::string const &Line, glm::dvec3 const &Point, glm::dvec2 const &Way, double const Reach)
{
	auto const *found{axis_of(Line)};
	if (found == nullptr)
		return 0;
	double gap{0.0};
	auto const chainage{chainage_at(*found, Point, gap)};
	if (gap > Reach)
		return 0;
	return glm::dot(direction_on(*found, chainage) * static_cast<double>(found->growth), Way) >= 0.0 ? 1 : -1;
}

namespace
{

// the km falls on a whole hectometre
bool whole_hectometre(double const Km)
{
	return std::abs(Km * 10.0 - std::round(Km * 10.0)) < 1e-4;
}

// a jump lies between the chainages, or by one of them
bool jump_between(std::vector<double> const &Jumps, double const A, double const B)
{
	auto const from{std::min(A, B)};
	auto const to{std::max(A, B)};
	return std::ranges::any_of(Jumps, [from, to](double const Jump) { return Jump >= from - 1e-6 && Jump <= to + 1e-6; });
}

// the hectometres the km given at the points pass, but on the pieces with a jump
void hectometres_along(std::vector<hectometre> &Result, std::vector<double> const &Jumps, std::vector<glm::dvec3> const &Points, std::vector<double> const &Km, std::vector<double> const &Chainage)
{
	for (std::size_t i = 1; i < Points.size(); ++i)
	{
		if (jump_between(Jumps, Chainage[i - 1], Chainage[i]))
			continue;
		auto const k0{Km[i - 1]};
		auto const k1{Km[i]};
		if (std::abs(k1 - k0) < 1e-9)
			continue;
		auto way{plan_of(Points[i] - Points[i - 1])};
		way = glm::length(way) > 1e-9 ? glm::normalize(way) : glm::dvec2{0.0, 1.0};
		auto const first{static_cast<int>(std::floor(std::min(k0, k1) * 10.0 + 1e-9)) + 1};
		auto const last{static_cast<int>(std::floor(std::max(k0, k1) * 10.0 + 1e-9))};
		for (auto number = first; number <= last && number < first + 10; ++number)
			Result.emplace_back(glm::mix(Points[i - 1], Points[i], (number / 10.0 - k0) / (k1 - k0)), k1 > k0 ? way : -way, number);
	}
}

// at a jump the post of the km on from it, as the posts of a line built anew begin there, else of the km the jump is reached at
void jump_posts(std::vector<hectometre> &Result, axis const &Axis)
{
	for (auto const &anchor : Axis.anchors)
	{
		if (std::abs(anchor.after - anchor.before) < 1e-6)
			continue;
		auto const km{whole_hectometre(anchor.after) ? anchor.after : anchor.before};
		if (whole_hectometre(km))
			Result.emplace_back(point_on(Axis, anchor.chainage), direction_on(Axis, anchor.chainage) * static_cast<double>(Axis.growth), static_cast<int>(std::lround(km * 10.0)));
	}
}

} // namespace

std::vector<hectometre> hectometres(axis const &Axis)
{
	std::vector<hectometre> result;
	std::vector<double> jumps;
	for (auto const &anchor : Axis.anchors)
		if (std::abs(anchor.after - anchor.before) >= 1e-6)
			jumps.push_back(anchor.chainage);
	jump_posts(result, Axis);
	std::vector<double> km;
	for (auto const chainage : Axis.chainage)
		km.push_back(km_on(Axis, chainage));
	hectometres_along(result, jumps, Axis.points, km, Axis.chainage);
	for (auto const &side : Axis.sides)
	{
		km.clear();
		for (auto const distance : side.along)
			km.push_back(km_on(Axis, side, distance));
		hectometres_along(result, jumps, side.points, km, side.chainage);
	}
	return result;
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
		if (auto const chord{plan_distance(ends[segment_data::point::start], ends[segment_data::point::end])}; std::min(plan_distance(ends[segment_data::point::start], Point), plan_distance(ends[segment_data::point::end], Point)) > chord * 0.8 + nearest + 1.0)
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
		if (auto const chord{plan_distance(ends[segment_data::point::start], ends[segment_data::point::end])}; std::min(plan_distance(ends[segment_data::point::start], Point), plan_distance(ends[segment_data::point::end], Point)) > chord * 0.8 + nearest + 1.0)
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
	std::ranges::replace(result, '.', ',');
	return result;
}

} // namespace kilometrage
