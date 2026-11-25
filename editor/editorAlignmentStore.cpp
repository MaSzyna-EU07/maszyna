/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorRecords.hpp"

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

namespace alignment_store
{

namespace
{

using geometry::plan_of;

std::string const kMark{"//$h"};
double const kStep{1.0};

std::vector<record> g_records;
std::vector<record> g_base; // as they were before the changes not yet in the undo history
bool g_changed{false};
bool g_loaded{false};

// the lines and their format: editor_records::read_alignments() and alignment_lines()

// the points of the line every kStep from the start of the stretch it's kept for
void sample(record &Record)
{
	Record.samples.clear();
	auto const from{std::clamp(Record.from, 0.0, Record.result.length)};
	auto const to{std::clamp(Record.kept_to(), from, Record.result.length)};
	auto const count{static_cast<std::size_t>(std::ceil((to - from) / kStep))};
	for (std::size_t i = 0; i <= count; ++i)
		Record.samples.push_back(plan_of(alignment::evaluate(Record.result, std::min(to, from + i * kStep)).position));
	Record.high = Record.samples.empty() ? glm::dvec2{0.0} : Record.samples.front();
	Record.low = Record.high;
	for (auto const &point : Record.samples)
	{
		Record.low = glm::min(Record.low, point);
		Record.high = glm::max(Record.high, point);
	}
}

bool agrees(std::vector<stored_element> const &Stored, std::vector<alignment::element> const &Made)
{
	std::vector<stored_element> stored;
	std::vector<stored_element> made;
	std::copy_if(Stored.begin(), Stored.end(), std::back_inserter(stored), [](stored_element const &E) { return E.length > 0.01; });
	for (auto const &e : Made)
		if (e.length > 0.01)
			made.push_back(describe(e));
	if (stored.size() != made.size())
		return false;
	auto const close = [](double const A, double const B) { return std::abs(A - B) <= 0.01 + 1e-3 * std::abs(A); };
	for (std::size_t i = 0; i < stored.size(); ++i)
		if (stored[i].kind != made[i].kind || std::abs(stored[i].chainage - made[i].chainage) > 0.01 || std::abs(stored[i].length - made[i].length) > 0.01 ||
		    false == close(stored[i].radius_start, made[i].radius_start) || false == close(stored[i].radius_end, made[i].radius_end))
			return false;
	return true;
}

void mark(scene::layer_handle const Layer)
{
	if (false == scene::Layers.valid(Layer))
		return;
	std::vector<record> records;
	std::copy_if(g_records.begin(), g_records.end(), std::back_inserter(records), [&](record const &Entry) { return Entry.layer == Layer; });
	scene::Layers.mark(Layer, kMark, editor_records::alignment_lines(records, scene::Layers.layer(Layer).context_insert()));
}

// the vertex as seen from the other end of the line
alignment::vertex reversed_vertex(alignment::vertex Vertex)
{
	std::swap(Vertex.transition_in, Vertex.transition_out);
	std::swap(Vertex.ramp_in_offset, Vertex.ramp_out_offset);
	std::swap(Vertex.ramp_in_length, Vertex.ramp_out_length);
	if (Vertex.compound && false == Vertex.arcs.empty())
	{
		struct part
		{
			double radius;
			double share;
			double transition;
			double cant;
		};
		std::vector<part> parts{{Vertex.radius, Vertex.share, 0.0, Vertex.first_cant}};
		for (auto const &arc : Vertex.arcs)
			parts.push_back({arc.radius, arc.share, arc.transition, arc.cant});
		auto const count{parts.size()};
		Vertex.radius = parts.back().radius;
		Vertex.share = parts.back().share;
		Vertex.first_cant = parts.back().cant;
		for (std::size_t j = 1; j < count; ++j)
			Vertex.arcs[j - 1] = {parts[count - 1 - j].radius, parts[count - j].transition, parts[count - 1 - j].share, parts[count - 1 - j].cant};
	}
	return Vertex;
}

void finish(record &Record)
{
	Record.result = alignment::compute(Record.design);
	sample(Record);
}

// the points clicked added to the ones there, each once
void add_clicks(std::vector<glm::dvec3> &To, std::vector<glm::dvec3> const &From)
{
	for (auto const &click : From)
		if (std::none_of(To.begin(), To.end(), [&](glm::dvec3 const &Other) { return glm::distance(Other, click) < 0.01; }))
			To.push_back(click);
}

// the curves of a design whose vertices are vertices of a stored alignment are taken into that alignment, in place of its own
int merge(record const &Entry)
{
	auto const &vertices{Entry.result.vertices};
	if (vertices.empty())
		return -1;
	for (std::size_t r = 0; r < g_records.size(); ++r)
	{
		auto const &others{g_records[r].result.vertices};
		if (others.size() < vertices.size() || false == g_records[r].whole())
			continue;
		for (auto const backwards : {false, true})
			for (std::size_t first = 0; first + vertices.size() <= others.size(); ++first)
			{
				auto const other = [&](std::size_t const K) { return backwards ? first + vertices.size() - 1 - K : first + K; };
				bool same{true};
				for (std::size_t k = 0; k < vertices.size() && same; ++k)
					same = glm::distance(vertices[k], others[other(k)]) <= 0.1;
				if (false == same)
					continue;
				auto merged{g_records[r]};
				add_clicks(merged.clicks, Entry.clicks);
				for (std::size_t k = 0; k < vertices.size(); ++k)
				{
					auto const shape{backwards ? reversed_vertex(Entry.design.vertices[k]) : Entry.design.vertices[k]};
					auto &target{merged.design.vertices[other(k)]};
					auto const position{target.position};
					auto const offset{target.offset};
					target = shape;
					target.position = position;
					target.offset = offset;
				}
				finish(merged);
				if (false == merged.result.valid)
					return -1;
				merged.elements.clear();
				for (auto const &e : merged.result.elements)
					merged.elements.push_back(describe(e));
				merged.consistent = true;
				g_records[r] = std::move(merged);
				return static_cast<int>(r);
			}
	}
	return -1;
}

} // namespace

// describe(): with the lines, editor/editorRecords.cpp

void reset()
{
	g_records.clear();
	g_base.clear();
	g_changed = false;
	g_loaded = false;
}

namespace
{

void close_record(record &Entry, std::string const &Layer)
{
	finish(Entry);
	if (false == Entry.result.valid || Entry.result.elements.empty())
	{
		WriteLog("Editor: a horizontal alignment in " + Layer + " doesn't make a line, left out", logtype::generic);
		return;
	}
	Entry.consistent = agrees(Entry.elements, Entry.result.elements);
	if (false == Entry.consistent)
		WriteLog("Editor: the elements of a horizontal alignment in " + Layer + " differ from the ones its design makes now", logtype::generic);
	g_records.push_back(std::move(Entry));
}

void load_layer(scene::layer_handle const Layer)
{
	for (auto &entry : editor_records::read_alignments(scene::Layers.marked(Layer, kMark), scene::Layers.layer(Layer).context_insert(), Layer))
		close_record(entry, scene::Layers.layer(Layer).name);
}

} // namespace

void load()
{
	if (g_loaded || scene::Layers.empty())
		return;
	g_loaded = true;
	g_records.clear();
	for (auto const layer : scene::Layers.marked_layers(kMark))
		load_layer(layer);
	g_base = g_records;
	g_changed = false;
	if (false == g_records.empty())
		WriteLog("Editor: " + std::to_string(g_records.size()) + " horizontal alignment(s) read from the scenery", logtype::generic);
}

std::vector<record> const &records()
{
	load();
	return g_records;
}

namespace
{

double approximation(geometry::bezier const &Curve)
{
	auto const begin{plan_of(Curve.first(0.0))};
	auto const finish{plan_of(Curve.first(1.0))};
	if (glm::length(begin) < 1e-9 || glm::length(finish) < 1e-9)
		return 0.0;
	auto const turn{std::abs(geometry::signed_angle(glm::normalize(begin), glm::normalize(finish)))};
	if (turn < 1e-6)
		return 0.0;
	auto const radius{geometry::plan_distance(Curve.point(0.0), Curve.point(1.0)) / (2.0 * std::sin(turn * 0.5))};
	return 3.0e-4 * radius * std::pow(turn / (0.5 * 3.14159265358979323846), 6.0);
}

// the point on the line of the stored alignment, if it lies by it within the reach and not beyond its ends
std::optional<hit> project(record const &Entry, std::size_t const Index, glm::dvec2 const &Point, double const Reach)
{
	auto const &entry{Entry};
	auto const reach{Reach + entry.fit};
	auto const margin{reach + kStep};
	if (entry.samples.empty() || Point.x < entry.low.x - margin || Point.y < entry.low.y - margin || Point.x > entry.high.x + margin || Point.y > entry.high.y + margin)
		return std::nullopt;
	std::size_t nearest{0};
	double distance{std::numeric_limits<double>::max()};
	for (std::size_t i = 0; i < entry.samples.size(); ++i)
	{
		auto const d{glm::distance(entry.samples[i], Point)};
		if (d < distance)
		{
			distance = d;
			nearest = i;
		}
	}
	if (distance > margin)
		return std::nullopt;
	auto const away = [&](double const S) { return glm::distance(plan_of(alignment::evaluate(entry.result, S).position), Point); };
	auto const from{entry.from};
	auto const to{entry.kept_to()};
	auto low{std::max(from, from + nearest * kStep - kStep)};
	auto high{std::min(to, from + nearest * kStep + kStep)};
	for (int i = 0; i < 60; ++i)
	{
		auto const a{low + (high - low) / 3.0};
		auto const b{high - (high - low) / 3.0};
		if (away(a) < away(b))
			high = b;
		else
			low = a;
	}
	auto const chainage{(low + high) * 0.5};
	auto const offset{away(chainage)};
	if (offset > reach)
		return std::nullopt;
	if (chainage < from + 1e-3 || chainage > to - 1e-3)
	{
		auto const end{alignment::evaluate(entry.result, chainage)};
		auto const outward{glm::dot(Point - plan_of(end.position), glm::normalize(plan_of(end.direction))) * (chainage < from + 1e-3 ? -1.0 : 1.0)};
		if (outward > tolerance + entry.fit)
			return std::nullopt;
	}
	auto const &elements{entry.result.elements};
	auto const found{std::upper_bound(elements.begin(), elements.end(), chainage, [](double const Value, alignment::element const &E) { return Value < E.chainage; })};
	return hit{static_cast<int>(Index), static_cast<int>(std::max<std::ptrdiff_t>(0, std::distance(elements.begin(), found) - 1)), chainage, offset};
}

std::optional<hit> project(std::size_t const Index, glm::dvec2 const &Point, double const Reach)
{
	return project(g_records[Index], Index, Point, Reach);
}

bool lies_along(record const &Entry, segment_data const &Path)
{
	geometry::bezier const curve{Path};
	auto const reach{tolerance + approximation(curve)};
	for (auto const t : {0.0, 0.25, 0.5, 0.75, 1.0})
		if (false == project(Entry, 0, plan_of(curve.point(t)), t == 0.25 || t == 0.75 ? reach : tolerance).has_value())
			return false;
	return true;
}

bool lies_along(record const &Entry, TTrack const &Track)
{
	return false == Track.m_paths.empty() && lies_along(Entry, Track.m_paths.front());
}

std::vector<std::pair<double, double>> bare(std::size_t const Index)
{
	auto const &entry{g_records[Index]};
	auto const margin{tolerance + kStep};
	auto const inside = [&](glm::dvec2 const &Point) { return Point.x >= entry.low.x - margin && Point.y >= entry.low.y - margin && Point.x <= entry.high.x + margin && Point.y <= entry.high.y + margin; };
	std::vector<std::pair<double, double>> covered;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved)
			continue;
		for (auto const &path : track->m_paths)
		{
			auto const start{plan_of(path.points[segment_data::point::start])};
			auto const end{plan_of(path.points[segment_data::point::end])};
			if (false == inside(start) || false == inside(end) || false == lies_along(entry, path))
				continue;
			auto const from{project(entry, Index, start, tolerance)};
			auto const to{project(entry, Index, end, tolerance)};
			if (from.has_value() && to.has_value())
				covered.emplace_back(std::min(from->chainage, to->chainage), std::max(from->chainage, to->chainage));
		}
	}
	std::sort(covered.begin(), covered.end());
	std::vector<std::pair<double, double>> gaps;
	double reached{entry.from};
	for (auto const &[from, to] : covered)
	{
		if (from - reached > 0.5)
			gaps.emplace_back(reached, from);
		reached = std::max(reached, to);
	}
	if (entry.kept_to() - reached > 0.5)
		gaps.emplace_back(reached, entry.kept_to());
	return gaps;
}

record temporary(alignment::result const &Result)
{
	record entry;
	entry.result = Result;
	sample(entry);
	return entry;
}

} // namespace

std::optional<hit> at(glm::dvec3 const &Point, double const Reach)
{
	load();
	std::optional<hit> best;
	for (std::size_t r = 0; r < g_records.size(); ++r)
	{
		auto const found{project(r, plan_of(Point), Reach)};
		if (found.has_value() && (false == best.has_value() || found->offset < best->offset))
			best = found;
	}
	return best;
}

std::optional<hit> on_track(TTrack const &Track, double const Reach)
{
	if (Track.m_paths.empty() || Track.m_editorremoved)
		return std::nullopt;
	load();
	geometry::bezier const curve{Track.m_paths.front()};
	auto const reach{Reach + approximation(curve)};
	std::optional<hit> best;
	for (std::size_t r = 0; r < g_records.size(); ++r)
	{
		auto const middle{project(r, plan_of(curve.point(0.5)), Reach)};
		if (false == middle.has_value() || (best.has_value() && middle->offset >= best->offset))
			continue;
		bool along{true};
		for (auto const t : {0.0, 0.25, 0.75, 1.0})
			along = along && project(r, plan_of(curve.point(t)), reach).has_value();
		if (along)
			best = middle;
	}
	return best;
}

std::optional<designed_vertex> vertex_of(std::vector<std::pair<TTrack *, bool>> const &Run)
{
	if (Run.empty() || records().empty())
		return std::nullopt;
	auto const entering = [&](std::pair<TTrack *, bool> const &Member, bool const Atend) {
		auto const &path{Member.first->m_paths.front()};
		return plan_of(path.points[(Member.second == Atend) ? segment_data::point::end : segment_data::point::start]);
	};
	for (std::size_t r = 0; r < g_records.size(); ++r)
	{
		auto const &entry{g_records[r]};
		std::set<int> vertices;
		bool along{true};
		for (auto const &member : Run)
		{
			if (member.first == nullptr || member.first->m_paths.empty())
				return std::nullopt;
			geometry::bezier const curve{member.first->m_paths.front()};
			auto const reach{tolerance + approximation(curve)};
			std::vector<hit> hits;
			for (auto const t : {0.0, 0.25, 0.5, 0.75, 1.0})
				if (auto const found{project(r, plan_of(curve.point(t)), t == 0.25 || t == 0.75 ? reach : tolerance)})
					hits.push_back(*found);
			if (hits.size() < 5)
			{
				along = false;
				break;
			}
			auto const from{std::min(hits.front().chainage, hits.back().chainage) + 0.01};
			auto const to{std::max(hits.front().chainage, hits.back().chainage) - 0.01};
			for (auto const &e : entry.result.elements)
				if (e.kind != alignment::element_kind::straight && e.chainage < to && e.chainage + e.length > from && e.vertex >= 0)
					vertices.insert(e.vertex);
		}
		if (false == along || vertices.size() != 1 || *vertices.begin() >= static_cast<int>(entry.design.vertices.size()))
			continue;
		// a curve cut by the deletion of tracks along it is no designed one any more: the tracks left of it are recognized as they lie
		if (std::any_of(entry.result.elements.begin(), entry.result.elements.end(), [&](alignment::element const &E) {
			    return E.vertex == *vertices.begin() && E.kind != alignment::element_kind::straight && (E.chainage < entry.from - 0.01 || E.chainage + E.length > entry.kept_to() + 0.01);
		    }))
			continue;
		auto const start{project(r, entering(Run.front(), false), tolerance)};
		auto const end{project(r, entering(Run.back(), true), tolerance)};
		if (false == start.has_value() || false == end.has_value())
			continue;
		// the tracks of a part of the curve only, as on either side of a switch laid into it, aren't laid by its design: it's theirs only
		// where they reach over the whole curve, else they're recognized as they lie
		auto const low{std::min(start->chainage, end->chainage) - 0.5};
		auto const high{std::max(start->chainage, end->chainage) + 0.5};
		if (std::any_of(entry.result.elements.begin(), entry.result.elements.end(), [&](alignment::element const &E) {
			    return E.vertex == *vertices.begin() && E.kind != alignment::element_kind::straight && (E.chainage < low || E.chainage + E.length > high);
		    }))
			continue;
		designed_vertex result;
		result.record = static_cast<int>(r);
		result.vertex = *vertices.begin();
		result.shape = entry.design.vertices[result.vertex];
		result.reversed = end->chainage < start->chainage;
		if (result.reversed)
			result.shape = reversed_vertex(result.shape);
		return result;
	}
	return std::nullopt;
}

int find(glm::dvec3 const &Start, glm::dvec3 const &End, bool &Reversed)
{
	load();
	Reversed = false;
	for (std::size_t i = 0; i < g_records.size(); ++i)
	{
		auto const &design{g_records[i].design};
		if (false == g_records[i].whole())
			continue;
		if (geometry::plan_distance(design.start, Start) <= 0.1 && geometry::plan_distance(design.end, End) <= 0.1)
			return static_cast<int>(i);
		if (geometry::plan_distance(design.start, End) <= 0.1 && geometry::plan_distance(design.end, Start) <= 0.1)
		{
			Reversed = true;
			return static_cast<int>(i);
		}
	}
	return -1;
}

glm::dvec3 kept_start(record const &Record)
{
	return alignment::evaluate(Record.result, Record.from).position;
}

glm::dvec3 kept_end(record const &Record)
{
	return alignment::evaluate(Record.result, Record.kept_to()).position;
}

std::size_t trim(std::vector<TTrack *> const &Removed)
{
	load();
	// the records the tracks deleted lay along, the ones they lay along before they were retired
	std::set<std::size_t> touched;
	for (auto const *track : Removed)
		if (track != nullptr)
			for (std::size_t r = 0; r < g_records.size(); ++r)
				if (std::any_of(track->m_paths.begin(), track->m_paths.end(), [&](segment_data const &Path) { return lies_along(g_records[r], Path); }))
					touched.insert(r);
	std::vector<record> kept;
	std::set<scene::layer_handle> layers;
	std::size_t cut{0};
	for (auto r{touched.rbegin()}; r != touched.rend(); ++r)
	{
		auto const gaps{bare(*r)};
		if (gaps.empty())
			continue;
		auto const entry{g_records[*r]};
		g_records.erase(g_records.begin() + static_cast<std::ptrdiff_t>(*r));
		layers.insert(entry.layer);
		++cut;
		// the stretches between the gaps, with the points clicked along them; none too short for a track
		std::vector<glm::dvec2> plan;
		for (auto const &click : entry.clicks)
			plan.push_back(plan_of(click));
		auto const projections{alignment::project(entry.result, plan)};
		auto from{entry.from};
		auto const keep = [&](double const To) {
			if (To - from < 1.0)
				return;
			auto piece{entry};
			piece.from = from;
			piece.to = To;
			piece.clicks.clear();
			for (std::size_t i = 0; i < entry.clicks.size() && i < projections.size(); ++i)
				if (projections[i].chainage >= from - 1.0 && projections[i].chainage <= To + 1.0)
					piece.clicks.push_back(entry.clicks[i]);
			sample(piece);
			kept.push_back(std::move(piece));
		};
		for (auto const &[start, end] : gaps)
		{
			keep(start);
			from = end;
		}
		keep(entry.kept_to());
	}
	if (cut == 0)
		return 0;
	g_records.insert(g_records.end(), kept.begin(), kept.end());
	g_changed = true;
	for (auto const layer : layers)
		mark(layer);
	return cut;
}

int departed(std::vector<TTrack *> const &Tracks, double &Departure)
{
	load();
	Departure = 0.0;
	double const reach{5.0};
	int result{-1};
	std::map<std::size_t, std::vector<std::pair<double, double>>> gaps;
	for (auto *track : Tracks)
	{
		if (track == nullptr || track->m_paths.empty() || on_track(*track).has_value())
			continue;
		geometry::bezier const curve{track->m_paths.front()};
		for (auto const t : {0.0, 0.25, 0.5, 0.75, 1.0})
			for (std::size_t r = 0; r < g_records.size(); ++r)
			{
				auto const found{project(r, plan_of(curve.point(t)), reach)};
				if (false == found.has_value() || found->offset <= tolerance + g_records[r].fit || found->offset <= Departure)
					continue;
				auto known{gaps.find(r)};
				if (known == gaps.end())
					known = gaps.emplace(r, bare(r)).first;
				if (std::none_of(known->second.begin(), known->second.end(), [&](auto const &Gap) { return found->chainage >= Gap.first - tolerance && found->chainage <= Gap.second + tolerance; }))
					continue;
				Departure = found->offset;
				result = found->record;
			}
	}
	return result;
}

double deviation(alignment::result const &Result, std::vector<TTrack *> const &Tracks, TTrack *&Worst)
{
	Worst = nullptr;
	if (false == Result.valid || Result.elements.empty())
		return std::numeric_limits<double>::max();
	auto const entry{temporary(Result)};
	double largest{0.0};
	for (auto *track : Tracks)
	{
		if (track == nullptr || track->m_paths.empty())
			continue;
		geometry::bezier const curve{track->m_paths.front()};
		auto const allowance{approximation(curve)};
		for (auto const t : {0.0, 0.25, 0.5, 0.75, 1.0})
		{
			auto const point{plan_of(curve.point(t))};
			auto const found{project(entry, 0, point, 1000.0)};
			auto offset{found.has_value() ? found->offset : std::numeric_limits<double>::max()};
			if (false == found.has_value())
				for (auto const &sample : entry.samples)
					offset = std::min(offset, glm::distance(sample, point));
			if (t == 0.25 || t == 0.75)
				offset = std::max(0.0, offset - allowance);
			if (offset > largest)
			{
				largest = offset;
				Worst = track;
			}
		}
	}
	return largest;
}

bool follows(alignment::result const &Result, std::vector<TTrack *> const &Tracks)
{
	if (false == Result.valid || Result.elements.empty())
		return false;
	auto const entry{temporary(Result)};
	return std::all_of(Tracks.begin(), Tracks.end(), [&](TTrack const *Track) { return Track != nullptr && lies_along(entry, *Track); });
}

std::optional<double> chainage_on(alignment::result const &Result, glm::dvec3 const &Point)
{
	if (false == Result.valid)
		return std::nullopt;
	auto const found{project(temporary(Result), 0, plan_of(Point), tolerance)};
	if (false == found.has_value())
		return std::nullopt;
	return found->chainage;
}

std::vector<glm::dvec3> clicks_along(alignment::result const &Result, double const Reach)
{
	load();
	std::vector<glm::dvec3> along;
	if (false == Result.valid || Result.elements.empty())
		return along;
	std::vector<glm::dvec3> all;
	for (auto const &entry : g_records)
		add_clicks(all, entry.clicks);
	if (all.empty())
		return along;
	std::vector<glm::dvec2> plan;
	for (auto const &click : all)
		plan.push_back(plan_of(click));
	auto const projections{alignment::project(Result, plan)};
	for (std::size_t i = 0; i < all.size() && i < projections.size(); ++i)
		if (projections[i].chainage > 0.0 && projections[i].chainage < Result.length && std::abs(projections[i].offset) <= Reach)
			along.push_back(all[i]);
	return along;
}

int store(alignment::design const &Design, alignment::result const &Result, double const Origin, scene::layer_handle const Layer, std::string const &Source, double const Fit, std::vector<glm::dvec3> const &Clicks)
{
	load();
	record entry;
	entry.source = Source;
	entry.fit = Fit;
	entry.clicks = Clicks;
	entry.design = Design;
	entry.design.start_reserve = 0.0;
	entry.design.end_reserve = 0.0;
	entry.origin = Origin;
	finish(entry);
	if (false == entry.result.valid)
	{
		entry.result = Result;
		sample(entry);
	}
	entry.elements.clear();
	for (auto const &e : entry.result.elements)
		entry.elements.push_back(describe(e));
	bool reversed;
	auto index{find(Design.start, Design.end, reversed)};
	if (index >= 0)
	{
		entry.layer = g_records[index].layer;
		if (entry.clicks.empty())
			entry.clicks = g_records[index].clicks;
		g_records[index] = std::move(entry);
	}
	else if ((index = merge(entry)) >= 0)
	{
	}
	else
	{
		entry.layer = Layer;
		g_records.push_back(std::move(entry));
		index = static_cast<int>(g_records.size()) - 1;
	}
	g_changed = true;
	mark(g_records[index].layer);
	return index;
}

void forget(int const Index)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(g_records.size()))
		return;
	auto const layer{g_records[Index].layer};
	g_records.erase(g_records.begin() + Index);
	g_changed = true;
	mark(layer);
}

std::optional<hit> on_record(int const Index, glm::dvec3 const &Point, double const Reach)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(g_records.size()))
		return std::nullopt;
	return project(static_cast<std::size_t>(Index), plan_of(Point), Reach);
}

double cant_at(int const Index, double const Chainage)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(g_records.size()))
		return 0.0;
	return alignment::evaluate(g_records[Index].result, Chainage).cant;
}

void transform(int const Index, glm::dvec3 const &From, glm::dvec2 const &Fromdirection, glm::dvec3 const &To, glm::dvec2 const &Todirection)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(g_records.size()))
		return;
	auto &entry{g_records[Index]};
	auto const angle{geometry::signed_angle(glm::normalize(Fromdirection), glm::normalize(Todirection))};
	auto const lift{To.y - From.y};
	auto const place = [&](glm::dvec3 const &Point) {
		auto const planar{plan_of(To) + geometry::turned(plan_of(Point) - plan_of(From), angle)};
		return glm::dvec3{planar.x, Point.y + lift, planar.y};
	};
	auto &design{entry.design};
	design.start = place(design.start);
	design.end = place(design.end);
	design.start_direction = geometry::turned(design.start_direction, angle);
	design.end_direction = geometry::turned(design.end_direction, angle);
	for (auto &vertex : design.vertices)
		vertex.position = plan_of(place({vertex.position.x, 0.0, vertex.position.y}));
	for (auto &click : entry.clicks)
		click = place(click);
	finish(entry);
	entry.elements.clear();
	for (auto const &e : entry.result.elements)
		entry.elements.push_back(describe(e));
	g_changed = true;
	mark(entry.layer);
}

std::optional<double> chainage_at(glm::dvec3 const &Point)
{
	auto const found{at(Point)};
	if (false == found.has_value())
		return std::nullopt;
	return g_records[found->record].origin + found->chainage;
}

std::optional<std::vector<record>> take_changes()
{
	if (false == g_changed)
		return std::nullopt;
	auto before{std::move(g_base)};
	g_base = g_records;
	g_changed = false;
	return before;
}

void settle()
{
	g_base = g_records;
	g_changed = false;
}

void restore(std::vector<record> Records)
{
	std::set<scene::layer_handle> layers;
	for (auto const *list : {&g_records, &Records})
		for (auto const &entry : *list)
			layers.insert(entry.layer);
	g_records = std::move(Records);
	for (auto const layer : layers)
		mark(layer);
	settle();
}

scene::layer_handle layer_of(TTrack const &Track)
{
	auto const usable = [](scene::layer_handle const Layer) {
		if (false == scene::Layers.valid(Layer) || false == scene::Layers.accepts(Layer))
			return false;
		auto const &name{scene::Layers.layer(Layer).name};
		return false == name.ends_with(".txtf") && false == name.ends_with("_ter.scm");
	};
	auto layer{scene::Layers.resolve(Track.layer())};
	if (false == usable(layer))
		layer = scene::Layers.active();
	if (false == usable(layer))
		layer = 1;
	return layer;
}

} // namespace alignment_store
