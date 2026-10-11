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

#include "world/Track.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <cmath>
#include <limits>
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

// format of the lines, coordinates in the space of the file:
//   alignment <start x y z> <end x y z>
//   source <designed> origin <chainage of the start> shape <parabola|clothoid> speed <km/h>
//   ends <start direction x z> <end direction x z> <start grade> <end grade> <start R> <end R>
//   norms <gauge> <unbalanced> <cant max> <cant rate> <ramp factor> <jerk> <straight between curves, s> <arc min> <arc min factor>
//   pieces <pieces of a transition> <angle of an arc piece>
//   vertex <x> <z> <offset> <R> <L in> <L out> <cant> <reverse 0/1> <compound 0/1> <share> <arcs> [<R> <L> <share>]...
//   element <straight|transition|arc> <vertex> <chainage> <length> <R at the start> <R at the end> <cant> <start x> <start z> <direction x> <direction z>
//     radii signed with the turn, 0 for no curvature
//   end
char const *shape_name(alignment::transition_shape const Shape)
{
	return Shape == alignment::transition_shape::clothoid ? "clothoid" : "parabola";
}

glm::dvec2 local_direction(scene::layer_context const &Context, glm::dvec3 const &Point, glm::dvec2 const &Direction)
{
	auto const from{Context.to_local(Point)};
	auto const to{Context.to_local(Point + glm::dvec3{Direction.x, 0.0, Direction.y})};
	auto const direction{plan_of(to - from)};
	return glm::length(direction) > 1e-12 ? glm::normalize(direction) : Direction;
}

glm::dvec2 world_direction(scene::layer_context const &Context, glm::dvec3 const &Local, glm::dvec2 const &Direction)
{
	auto const from{Context.to_world(Local)};
	auto const to{Context.to_world(Local + glm::dvec3{Direction.x, 0.0, Direction.y})};
	auto const direction{plan_of(to - from)};
	return glm::length(direction) > 1e-12 ? glm::normalize(direction) : Direction;
}

void sample(record &Record)
{
	Record.samples.clear();
	auto const length{Record.result.length};
	auto const count{static_cast<std::size_t>(std::ceil(length / kStep))};
	for (std::size_t i = 0; i <= count; ++i)
		Record.samples.push_back(plan_of(alignment::evaluate(Record.result, std::min(length, i * kStep)).position));
	Record.low = Record.high = Record.samples.empty() ? glm::dvec2{0.0} : Record.samples.front();
	for (auto const &point : Record.samples)
	{
		Record.low = glm::min(Record.low, point);
		Record.high = glm::max(Record.high, point);
	}
}

bool agrees(std::vector<stored_element> const &Stored, std::vector<alignment::element> const &Made)
{
	if (Stored.size() != Made.size())
		return false;
	for (std::size_t i = 0; i < Stored.size(); ++i)
	{
		auto const made{describe(Made[i])};
		if (Stored[i].kind != made.kind || std::abs(Stored[i].chainage - made.chainage) > 0.01 || std::abs(Stored[i].length - made.length) > 0.01 ||
		    std::abs(Stored[i].radius_start - made.radius_start) > 0.01 || std::abs(Stored[i].radius_end - made.radius_end) > 0.01)
			return false;
	}
	return true;
}

void mark(scene::layer_handle const Layer)
{
	if (false == scene::Layers.valid(Layer))
		return;
	auto const &context{scene::Layers.layer(Layer).context_insert()};
	auto const local = [&](glm::dvec3 const &Point) {
		auto const point{context.to_local(Point)};
		return format("%.4f %.4f %.4f", point.x, point.y, point.z);
	};
	std::vector<std::string> lines;
	for (auto const &entry : g_records)
	{
		if (entry.layer != Layer)
			continue;
		if (lines.empty())
			lines.emplace_back("# horizontal alignments designed in the track editor, the simulation skips these lines");
		auto const &design{entry.design};
		lines.push_back("alignment " + local(design.start) + ' ' + local(design.end));
		lines.push_back(format("source %s origin %.4f shape %s speed %.1f", entry.source.c_str(), entry.origin, shape_name(design.shape), design.speed));
		auto const startdirection{local_direction(context, design.start, design.start_direction)};
		auto const enddirection{local_direction(context, design.end, design.end_direction)};
		lines.push_back(format("ends %.9f %.9f %.9f %.9f %.6f %.6f %.3f %.3f", startdirection.x, startdirection.y, enddirection.x, enddirection.y, design.start_grade, design.end_grade, design.start_radius, design.end_radius));
		auto const &norms{design.norms};
		lines.push_back(format("norms %.1f %.3f %.1f %.1f %.2f %.3f %.2f %.2f %.3f", norms.gauge, norms.unbalanced, norms.cant_max, norms.cant_rate, norms.ramp_factor, norms.jerk, norms.tangent_min_time, norms.arc_min, norms.arc_min_factor));
		lines.push_back(format("pieces %d %.3f", design.transition_pieces, design.arc_piece_angle));
		for (auto const &vertex : design.vertices)
		{
			auto const position{context.to_local({vertex.position.x, 0.0, vertex.position.y})};
			auto text{format("vertex %.4f %.4f %.4f %.3f %.3f %.3f %.1f %d %d %.9f %zu", position.x, position.z, vertex.offset, vertex.radius, vertex.transition_in, vertex.transition_out, vertex.cant, vertex.reverse_turn ? 1 : 0, vertex.compound ? 1 : 0,
			                  vertex.share, vertex.arcs.size())};
			for (auto const &arc : vertex.arcs)
				text += format(" %.3f %.3f %.9f", arc.radius, arc.transition, arc.share);
			lines.push_back(text);
		}
		for (auto const &e : entry.result.elements)
		{
			auto const station{alignment::evaluate(entry.result, e.chainage)};
			auto const start{context.to_local(station.position)};
			auto const direction{local_direction(context, station.position, glm::normalize(plan_of(station.direction)))};
			auto const element{describe(e)};
			lines.push_back(format("element %s %d %.4f %.4f %.3f %.3f %.1f %.4f %.4f %.9f %.9f", element.kind.c_str(), element.vertex, element.chainage, element.length, element.radius_start, element.radius_end, element.cant, start.x, start.z, direction.x, direction.y));
		}
		lines.emplace_back("end");
	}
	scene::Layers.mark(Layer, kMark, std::move(lines));
}

// the vertex as seen from the other end of the line
alignment::vertex reversed_vertex(alignment::vertex Vertex)
{
	std::swap(Vertex.transition_in, Vertex.transition_out);
	if (Vertex.compound && false == Vertex.arcs.empty())
	{
		struct part
		{
			double radius, share, transition;
		};
		std::vector<part> parts{{Vertex.radius, Vertex.share, 0.0}};
		for (auto const &arc : Vertex.arcs)
			parts.push_back({arc.radius, arc.share, arc.transition});
		auto const count{parts.size()};
		Vertex.radius = parts.back().radius;
		Vertex.share = parts.back().share;
		for (std::size_t j = 1; j < count; ++j)
			Vertex.arcs[j - 1] = {parts[count - 1 - j].radius, parts[count - j].transition, parts[count - 1 - j].share};
	}
	return Vertex;
}

void finish(record &Record)
{
	Record.result = alignment::compute(Record.design);
	sample(Record);
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
		if (others.size() < vertices.size())
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

stored_element describe(alignment::element const &Element)
{
	stored_element result;
	result.vertex = Element.vertex;
	result.chainage = Element.chainage;
	result.length = Element.length;
	result.cant = Element.cant;
	auto const sign{Element.turn < 0 ? -1.0 : 1.0};
	auto const radius = [&](double const Curvature) { return Curvature > 1e-12 ? sign / Curvature : 0.0; };
	switch (Element.kind)
	{
	case alignment::element_kind::transition_in:
		result.radius_end = sign * Element.radius;
		break;
	case alignment::element_kind::arc:
		result.radius_start = result.radius_end = sign * Element.radius;
		break;
	case alignment::element_kind::transition_out:
		result.radius_start = sign * Element.radius;
		break;
	case alignment::element_kind::spiral:
		result.radius_start = radius(Element.curvature_start);
		result.radius_end = radius(Element.curvature_end);
		break;
	default:
		result.cant = 0.0;
		break;
	}
	if (result.radius_start == 0.0 && result.radius_end == 0.0)
		result.kind = "straight";
	else if (std::abs(result.radius_start - result.radius_end) < 1e-6)
		result.kind = "arc";
	else
		result.kind = "transition";
	return result;
}

void reset()
{
	g_records.clear();
	g_base.clear();
	g_changed = false;
	g_loaded = false;
}

void load()
{
	if (g_loaded || scene::Layers.empty())
		return;
	g_loaded = true;
	g_records.clear();
	for (auto const layer : scene::Layers.marked_layers(kMark))
	{
		auto const &context{scene::Layers.layer(layer).context_insert()};
		std::optional<record> current;
		std::vector<std::string> made;
		for (auto const &text : scene::Layers.marked(layer, kMark))
		{
			std::istringstream line{text};
			std::string keyword, word;
			line >> keyword;
			if (keyword == "alignment")
			{
				record entry;
				entry.layer = layer;
				glm::dvec3 start, end;
				line >> start.x >> start.y >> start.z >> end.x >> end.y >> end.z;
				if (line.fail())
				{
					current.reset();
					continue;
				}
				entry.design.start = context.to_world(start);
				entry.design.end = context.to_world(end);
				current = entry;
				continue;
			}
			if (false == current.has_value())
				continue;
			auto &entry{*current};
			auto &design{entry.design};
			if (keyword == "source")
			{
				std::string shape;
				line >> entry.source >> word >> entry.origin >> word >> shape >> word >> design.speed;
				design.shape = shape == "clothoid" ? alignment::transition_shape::clothoid : alignment::transition_shape::cubic_parabola;
			}
			else if (keyword == "ends")
			{
				glm::dvec2 startdirection, enddirection;
				line >> startdirection.x >> startdirection.y >> enddirection.x >> enddirection.y >> design.start_grade >> design.end_grade >> design.start_radius >> design.end_radius;
				design.start_direction = world_direction(context, context.to_local(design.start), startdirection);
				design.end_direction = world_direction(context, context.to_local(design.end), enddirection);
			}
			else if (keyword == "norms")
			{
				auto &norms{design.norms};
				line >> norms.gauge >> norms.unbalanced >> norms.cant_max >> norms.cant_rate >> norms.ramp_factor >> norms.jerk >> norms.tangent_min_time >> norms.arc_min >> norms.arc_min_factor;
			}
			else if (keyword == "pieces")
			{
				line >> design.transition_pieces >> design.arc_piece_angle;
			}
			else if (keyword == "vertex")
			{
				alignment::vertex vertex;
				glm::dvec3 position{0.0};
				int reverse{0}, compound{0};
				std::size_t arcs{0};
				line >> position.x >> position.z >> vertex.offset >> vertex.radius >> vertex.transition_in >> vertex.transition_out >> vertex.cant >> reverse >> compound >> vertex.share >> arcs;
				for (std::size_t i = 0; i < arcs && false == line.fail(); ++i)
				{
					alignment::compound_arc arc;
					line >> arc.radius >> arc.transition >> arc.share;
					vertex.arcs.push_back(arc);
				}
				if (line.fail())
					continue;
				position = context.to_world(position);
				vertex.position = {position.x, position.z};
				vertex.reverse_turn = (reverse != 0);
				vertex.compound = (compound != 0);
				design.vertices.push_back(vertex);
			}
			else if (keyword == "element")
			{
				stored_element e;
				line >> e.kind >> e.vertex >> e.chainage >> e.length >> e.radius_start >> e.radius_end >> e.cant;
				if (false == line.fail())
					entry.elements.push_back(e);
			}
			else if (keyword == "end")
			{
				finish(entry);
				if (false == entry.result.valid || entry.result.elements.empty())
				{
					WriteLog("Editor: a horizontal alignment in " + scene::Layers.layer(layer).name + " doesn't make a line, left out", logtype::generic);
					current.reset();
					continue;
				}
				entry.consistent = agrees(entry.elements, entry.result.elements);
				if (false == entry.consistent)
					WriteLog("Editor: the elements of a horizontal alignment in " + scene::Layers.layer(layer).name + " differ from the ones its design makes now", logtype::generic);
				g_records.push_back(std::move(entry));
				current.reset();
			}
		}
	}
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

// the point on the line of the stored alignment, if it lies by it within the reach and not beyond its ends
std::optional<hit> project(std::size_t const Index, glm::dvec2 const &Point, double const Reach)
{
	auto const &entry{g_records[Index]};
	auto const margin{Reach + kStep};
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
	auto low{std::max(0.0, nearest * kStep - kStep)};
	auto high{std::min(entry.result.length, nearest * kStep + kStep)};
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
	if (offset > Reach)
		return std::nullopt;
	if (chainage < 1e-3 || chainage > entry.result.length - 1e-3)
	{
		auto const end{alignment::evaluate(entry.result, chainage)};
		auto const outward{glm::dot(Point - plan_of(end.position), glm::normalize(plan_of(end.direction))) * (chainage < 1e-3 ? -1.0 : 1.0)};
		if (outward > tolerance)
			return std::nullopt;
	}
	auto const &elements{entry.result.elements};
	auto const found{std::upper_bound(elements.begin(), elements.end(), chainage, [](double const Value, alignment::element const &E) { return Value < E.chainage; })};
	return hit{static_cast<int>(Index), static_cast<int>(std::max<std::ptrdiff_t>(0, std::distance(elements.begin(), found) - 1)), chainage, offset};
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
	std::optional<hit> best;
	for (std::size_t r = 0; r < g_records.size(); ++r)
	{
		auto const middle{project(r, plan_of(curve.point(0.5)), Reach)};
		if (false == middle.has_value() || (best.has_value() && middle->offset >= best->offset))
			continue;
		bool along{true};
		for (auto const t : {0.0, 0.25, 0.75, 1.0})
			along = along && project(r, plan_of(curve.point(t)), Reach).has_value();
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
			std::vector<hit> hits;
			for (auto const t : {0.0, 0.25, 0.5, 0.75, 1.0})
				if (auto const found{project(r, plan_of(curve.point(t)), tolerance)})
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
		auto const start{project(r, entering(Run.front(), false), tolerance)};
		auto const end{project(r, entering(Run.back(), true), tolerance)};
		if (false == start.has_value() || false == end.has_value())
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

int departed(std::vector<TTrack *> const &Tracks, double &Departure)
{
	load();
	Departure = 0.0;
	double const reach{5.0};
	int result{-1};
	for (auto *track : Tracks)
	{
		if (track == nullptr || track->m_paths.empty() || on_track(*track).has_value())
			continue;
		geometry::bezier const curve{track->m_paths.front()};
		for (auto const t : {0.0, 0.25, 0.5, 0.75, 1.0})
		{
			auto const found{at(curve.point(t), reach)};
			if (found.has_value() && found->offset > tolerance && found->offset > Departure)
			{
				Departure = found->offset;
				result = found->record;
			}
		}
	}
	return result;
}

int store(alignment::design const &Design, alignment::result const &Result, double const Origin, scene::layer_handle const Layer)
{
	load();
	record entry;
	entry.design = Design;
	entry.design.start_reserve = entry.design.end_reserve = 0.0;
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

} // namespace alignment_store
