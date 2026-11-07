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
#include "simulation/simulation.h"
#include "utilities/Logs.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <iterator>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <sstream>

namespace alignment_store
{

namespace
{

using geometry::plan_of;

std::string const kMark{"//$h"};
double const kStep{1.0};
// the parameters of the points of a path compared with a line
std::array<double, 5> const kChecked{0.0, 0.25, 0.5, 0.75, 1.0};

// the alignments of the scenery as the editor keeps them
struct store_state
{
	std::vector<record> records;
	std::vector<record> base; // as they were before the changes not yet in the undo history
	bool changed{false};
	bool loaded{false};
};

store_state &state()
{
	static store_state kept;
	return kept;
}

// format of the lines, coordinates in the space of the file:
//   alignment <start x y z> <end x y z>
//   source <designed> origin <chainage of the start> shape <parabola|clothoid> speed <km/h> [fit <m>]
//   ends <start direction x z> <end direction x z> <start grade> <end grade> <start R> <end R>
//   norms <gauge> <unbalanced> <cant max> <cant rate> <ramp factor> <jerk> <straight between curves, s> <arc min> <arc min factor>
//   pieces <pieces of a transition> <angle of an arc piece>
//   vertex <x> <z> <offset> <R> <L in> <L out> <cant> <reverse 0/1> <compound 0/1> <share> <arcs> [<R> <L> <share>]... [spirals 1]
//     spirals: two transition curves with no arc between them, their lengths following from R
//     [cants <first> <arc>...]: the cants of the radii of a compound curve set one by one, below 0 the cant in proportion to the curvature
//     [ramps <in offset> <in length> <out offset> <out length>]: the cant ramps of the curve where they don't go along its transitions,
//       their outer ends from the start and from the end of the curve, into it above 0; a length below 0 keeps the ramp along the transition
//   click <x y z>: a point clicked when the line was laid, its curves fitted to them
//   kept <from> <to>: the stretch of the line the design is kept for, chainages from its start, where tracks along it were deleted
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

// the points of the line every kStep from the start of the stretch it's kept for
void sample(record &Record)
{
	Record.samples.clear();
	auto const from{std::clamp(Record.from, 0.0, Record.result.length)};
	auto const to{std::clamp(Record.kept_to(), from, Record.result.length)};
	auto const count{static_cast<std::size_t>(std::ceil((to - from) / kStep))};
	for (std::size_t i = 0; i <= count; ++i)
		Record.samples.push_back(plan_of(alignment::evaluate(Record.result, std::min(to, from + static_cast<double>(i) * kStep)).position));
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
	std::ranges::copy_if(Stored, std::back_inserter(stored), [](stored_element const &E) { return E.length > 0.01; });
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

std::string local_text(scene::layer_context const &Context, glm::dvec3 const &Point)
{
	auto const point{Context.to_local(Point)};
	return format("%.4f %.4f %.4f", point.x, point.y, point.z);
}

std::string ends_line(alignment::design const &Design, scene::layer_context const &Context)
{
	auto const startdirection{local_direction(Context, Design.start, Design.start_direction)};
	auto const enddirection{local_direction(Context, Design.end, Design.end_direction)};
	auto ends{format("ends %.9f %.9f %.9f %.9f %.6f %.6f %.3f %.3f", startdirection.x, startdirection.y, enddirection.x, enddirection.y, Design.start_grade, Design.end_grade, Design.start_radius, Design.end_radius)};
	// the sides the adjoining tracks turn to, and the curves going on from them with their cants, written since there are such
	if (Design.start_turn != 0 || Design.end_turn != 0)
		ends += format(" turns %d %d", Design.start_turn, Design.end_turn);
	if (Design.start_curved || Design.end_curved)
		ends += format(" curved %d %d %.1f %.1f", Design.start_curved ? 1 : 0, Design.end_curved ? 1 : 0, Design.start_cant, Design.end_cant);
	return ends;
}

std::string vertex_line(alignment::vertex const &Vertex, scene::layer_context const &Context)
{
	auto const position{Context.to_local({Vertex.position.x, 0.0, Vertex.position.y})};
	auto text{format("vertex %.4f %.4f %.4f %.3f %.3f %.3f %.1f %d %d %.9f %zu", position.x, position.z, Vertex.offset, Vertex.radius, Vertex.transition_in, Vertex.transition_out, Vertex.cant, Vertex.reverse_turn ? 1 : 0, Vertex.compound ? 1 : 0,
	                  Vertex.share, Vertex.arcs.size())};
	for (auto const &arc : Vertex.arcs)
		text += format(" %.3f %.3f %.9f", arc.radius, arc.transition, arc.share);
	if (Vertex.spirals)
		text += " spirals 1";
	if (Vertex.first_cant >= 0.0 || std::ranges::any_of(Vertex.arcs, [](alignment::compound_arc const &Arc) { return Arc.cant >= 0.0; }))
	{
		text += format(" cants %.1f", Vertex.first_cant);
		for (auto const &arc : Vertex.arcs)
			text += format(" %.1f", arc.cant);
	}
	if (Vertex.own_ramps())
		text += format(" ramps %.3f %.3f %.3f %.3f", Vertex.ramp_in_offset, Vertex.ramp_in_length, Vertex.ramp_out_offset, Vertex.ramp_out_length);
	return text;
}

// the lines a record is kept in, in the file of its layer
void record_lines(record const &Entry, scene::layer_context const &Context, std::vector<std::string> &Lines)
{
	auto const &design{Entry.design};
	Lines.push_back("alignment " + local_text(Context, design.start) + ' ' + local_text(Context, design.end));
	Lines.push_back(format("source %s origin %.4f shape %s speed %.1f", Entry.source.c_str(), Entry.origin, shape_name(design.shape), design.speed) + (Entry.fit > 0.0 ? format(" fit %.3f", Entry.fit) : std::string{}));
	Lines.push_back(ends_line(design, Context));
	auto const &norms{design.norms};
	Lines.push_back(format("norms %.1f %.3f %.1f %.1f %.2f %.3f %.2f %.2f %.3f", norms.gauge, norms.unbalanced, norms.cant_max, norms.cant_rate, norms.ramp_factor, norms.jerk, norms.tangent_min_time, norms.arc_min, norms.arc_min_factor));
	Lines.push_back(format("pieces %d %.3f", design.transition_pieces, design.arc_piece_angle));
	if (false == Entry.whole())
		Lines.push_back(format("kept %.4f %.4f", Entry.from, Entry.kept_to()));
	for (auto const &vertex : design.vertices)
		Lines.push_back(vertex_line(vertex, Context));
	for (auto const &click : Entry.clicks)
		Lines.push_back("click " + local_text(Context, click));
	for (auto const &e : Entry.result.elements)
	{
		auto const station{alignment::evaluate(Entry.result, e.chainage)};
		auto const start{Context.to_local(station.position)};
		auto const direction{local_direction(Context, station.position, glm::normalize(plan_of(station.direction)))};
		auto const element{describe(e)};
		Lines.push_back(format("element %s %d %.4f %.4f %.3f %.3f %.1f %.4f %.4f %.9f %.9f", element.kind.c_str(), element.vertex, element.chainage, element.length, element.radius_start, element.radius_end, element.cant, start.x, start.z, direction.x, direction.y));
	}
	Lines.emplace_back("end");
}

void mark(scene::layer_handle const Layer)
{
	if (false == scene::Layers.valid(Layer))
		return;
	auto const &context{scene::Layers.layer(Layer).context_insert()};
	std::vector<std::string> lines;
	for (auto const &entry : state().records)
	{
		if (entry.layer != Layer)
			continue;
		if (lines.empty())
			lines.emplace_back("# horizontal alignments designed in the track editor, the simulation skips these lines");
		record_lines(entry, context, lines);
	}
	scene::Layers.mark(Layer, kMark, std::move(lines));
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
			parts.emplace_back(arc.radius, arc.share, arc.transition, arc.cant);
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
		if (std::ranges::none_of(To, [&click](glm::dvec3 const &Other) { return glm::distance(Other, click) < 0.01; }))
			To.push_back(click);
}

// the vertex of the other alignment the one of the design at K lies on, when the design lies on it from the first one on
std::size_t matched_vertex(std::size_t const First, std::size_t const Count, bool const Backwards, std::size_t const K)
{
	return Backwards ? First + Count - 1 - K : First + K;
}

// whether the vertices of the design lie on the ones of the other alignment, from the first one on, either way
bool vertices_match(std::vector<glm::dvec2> const &Vertices, std::vector<glm::dvec2> const &Others, std::size_t const First, bool const Backwards)
{
	bool same{true};
	std::size_t k{0};
	while (k < Vertices.size() && same)
	{
		same = glm::distance(Vertices[k], Others[matched_vertex(First, Vertices.size(), Backwards, k)]) <= 0.1;
		++k;
	}
	return same;
}

// where the vertices of the design lie on the ones of the other alignment: the first of them and whether backwards, the first
// such place found
std::optional<std::pair<std::size_t, bool>> vertices_on(std::vector<glm::dvec2> const &Vertices, std::vector<glm::dvec2> const &Others)
{
	for (auto const backwards : {false, true})
		for (std::size_t first = 0; first + Vertices.size() <= Others.size(); ++first)
			if (vertices_match(Vertices, Others, first, backwards))
				return std::make_pair(first, backwards);
	return std::nullopt;
}

// the stored alignment with the curves of the design taken in at the vertices it lies on; its result isn't valid when they don't
// make a line
record merged_record(record const &Into, record const &Entry, std::size_t const First, bool const Backwards)
{
	auto const &vertices{Entry.result.vertices};
	auto merged{Into};
	add_clicks(merged.clicks, Entry.clicks);
	for (std::size_t k = 0; k < vertices.size(); ++k)
	{
		auto const shape{Backwards ? reversed_vertex(Entry.design.vertices[k]) : Entry.design.vertices[k]};
		auto &target{merged.design.vertices[matched_vertex(First, vertices.size(), Backwards, k)]};
		auto const position{target.position};
		auto const offset{target.offset};
		target = shape;
		target.position = position;
		target.offset = offset;
	}
	finish(merged);
	if (false == merged.result.valid)
		return merged;
	merged.elements.clear();
	for (auto const &e : merged.result.elements)
		merged.elements.push_back(describe(e));
	merged.consistent = true;
	return merged;
}

// the curves of a design whose vertices are vertices of a stored alignment are taken into that alignment, in place of its own
int merge(record const &Entry)
{
	auto const &vertices{Entry.result.vertices};
	if (vertices.empty())
		return -1;
	auto &entries{state().records};
	for (std::size_t r = 0; r < entries.size(); ++r)
	{
		auto const &others{entries[r].result.vertices};
		if (others.size() < vertices.size() || false == entries[r].whole())
			continue;
		auto const place{vertices_on(vertices, others)};
		if (false == place.has_value())
			continue;
		auto merged{merged_record(entries[r], Entry, place->first, place->second)};
		if (false == merged.result.valid)
			return -1;
		entries[r] = std::move(merged);
		return static_cast<int>(r);
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
		result.radius_end = sign * Element.radius;
		result.radius_start = result.radius_end;
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
	state().records.clear();
	state().base.clear();
	state().changed = false;
	state().loaded = false;
}

namespace
{

std::optional<record> read_start(std::istringstream &Line, scene::layer_context const &Context, scene::layer_handle const Layer)
{
	record entry;
	entry.layer = Layer;
	glm::dvec3 start;
	glm::dvec3 end;
	Line >> start.x >> start.y >> start.z >> end.x >> end.y >> end.z;
	if (Line.fail())
		return std::nullopt;
	entry.design.start = Context.to_world(start);
	entry.design.end = Context.to_world(end);
	return entry;
}

std::optional<alignment::vertex> read_vertex(std::istringstream &Line, scene::layer_context const &Context)
{
	alignment::vertex vertex;
	glm::dvec3 position{0.0};
	int reverse{0};
	int compound{0};
	std::size_t arcs{0};
	Line >> position.x >> position.z >> vertex.offset >> vertex.radius >> vertex.transition_in >> vertex.transition_out >> vertex.cant >> reverse >> compound >> vertex.share >> arcs;
	for (std::size_t i = 0; i < arcs && false == Line.fail(); ++i)
	{
		alignment::compound_arc arc;
		Line >> arc.radius >> arc.transition >> arc.share;
		vertex.arcs.push_back(arc);
	}
	if (Line.fail())
		return std::nullopt;
	// written since the bends of the straights are eased with two transition curves, the lines of before end with the arcs
	for (std::string word; Line >> word;)
	{
		if (int value{0}; word == "spirals" && Line >> value)
			vertex.spirals = (value != 0);
		// the cants of the arcs of a compound curve, set one by one: of the first arc and of the ones after it, below 0 in proportion
		if (word == "cants")
		{
			Line >> vertex.first_cant;
			for (auto &arc : vertex.arcs)
				Line >> arc.cant;
		}
		if (word == "ramps")
			Line >> vertex.ramp_in_offset >> vertex.ramp_in_length >> vertex.ramp_out_offset >> vertex.ramp_out_length;
	}
	position = Context.to_world(position);
	vertex.position = {position.x, position.z};
	vertex.reverse_turn = (reverse != 0);
	vertex.compound = (compound != 0);
	return vertex;
}

void read_source(record &Entry, std::istringstream &Line)
{
	auto &design{Entry.design};
	std::string word;
	std::string shape;
	Line >> Entry.source >> word >> Entry.origin >> word >> shape >> word >> design.speed;
	design.shape = shape == "clothoid" ? alignment::transition_shape::clothoid : alignment::transition_shape::cubic_parabola;
	if (Line >> word && word == "fit")
		Line >> Entry.fit;
}

void read_ends(alignment::design &Design, std::istringstream &Line, scene::layer_context const &Context)
{
	glm::dvec2 startdirection;
	glm::dvec2 enddirection;
	Line >> startdirection.x >> startdirection.y >> enddirection.x >> enddirection.y >> Design.start_grade >> Design.end_grade >> Design.start_radius >> Design.end_radius;
	Design.start_direction = world_direction(Context, Context.to_local(Design.start), startdirection);
	Design.end_direction = world_direction(Context, Context.to_local(Design.end), enddirection);
	for (std::string key; Line >> key;)
	{
		if (key == "turns")
			Line >> Design.start_turn >> Design.end_turn;
		if (int start{0}, end{0}; key == "curved" && Line >> start >> end >> Design.start_cant >> Design.end_cant)
		{
			Design.start_curved = (start != 0);
			Design.end_curved = (end != 0);
		}
	}
}

void read_part(record &Entry, std::string_view const Keyword, std::istringstream &Line, scene::layer_context const &Context)
{
	auto &design{Entry.design};
	if (Keyword == "source")
		read_source(Entry, Line);
	else if (Keyword == "ends")
		read_ends(design, Line, Context);
	else if (Keyword == "norms")
	{
		auto &norms{design.norms};
		Line >> norms.gauge >> norms.unbalanced >> norms.cant_max >> norms.cant_rate >> norms.ramp_factor >> norms.jerk >> norms.tangent_min_time >> norms.arc_min >> norms.arc_min_factor;
	}
	else if (Keyword == "pieces")
		Line >> design.transition_pieces >> design.arc_piece_angle;
	else if (Keyword == "vertex")
	{
		if (auto const vertex{read_vertex(Line, Context)})
			design.vertices.push_back(*vertex);
	}
	else if (Keyword == "element")
	{
		stored_element e;
		Line >> e.kind >> e.vertex >> e.chainage >> e.length >> e.radius_start >> e.radius_end >> e.cant;
		if (false == Line.fail())
			Entry.elements.push_back(e);
	}
	else if (Keyword == "click")
	{
		glm::dvec3 point;
		if (Line >> point.x >> point.y >> point.z)
			Entry.clicks.push_back(Context.to_world(point));
	}
	else if (Keyword == "kept")
	{
		double from;
		double to;
		if (Line >> from >> to && to > from)
		{
			Entry.from = std::max(0.0, from);
			Entry.to = to;
		}
	}
}

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
	state().records.push_back(std::move(Entry));
}

void load_layer(scene::layer_handle const Layer)
{
	auto const &context{scene::Layers.layer(Layer).context_insert()};
	std::optional<record> current;
	for (auto const &text : scene::Layers.marked(Layer, kMark))
	{
		std::istringstream line{text};
		std::string keyword;
		line >> keyword;
		if (keyword == "alignment")
			current = read_start(line, context, Layer);
		else if (false == current.has_value())
			continue;
		else if (keyword == "end")
		{
			close_record(*current, scene::Layers.layer(Layer).name);
			current.reset();
		}
		else
			read_part(*current, keyword, line, context);
	}
}

} // namespace

void load()
{
	if (state().loaded || scene::Layers.empty())
		return;
	state().loaded = true;
	state().records.clear();
	for (auto const layer : scene::Layers.marked_layers(kMark))
		load_layer(layer);
	state().base = state().records;
	state().changed = false;
	if (false == state().records.empty())
		WriteLog(std::format("Editor: {} horizontal alignment(s) read from the scenery", state().records.size()), logtype::generic);
}

std::vector<record> const &records()
{
	load();
	return state().records;
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
	return 3.0e-4 * radius * std::pow(turn / (0.5 * std::numbers::pi), 6.0);
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
		auto const a{low + (high - low) / 3.0}; // NOSONAR: std::lerp(low, high, 1.0 / 3.0) rounds differently from the third taken by the division
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
	auto const found{std::ranges::upper_bound(elements, chainage, std::ranges::less{}, &alignment::element::chainage)};
	return hit{static_cast<int>(Index), static_cast<int>(std::max<std::ptrdiff_t>(0, std::distance(elements.begin(), found) - 1)), chainage, offset};
}

std::optional<hit> project(std::size_t const Index, glm::dvec2 const &Point, double const Reach)
{
	return project(state().records[Index], Index, Point, Reach);
}

bool lies_along(record const &Entry, segment_data const &Path)
{
	geometry::bezier const curve{Path};
	auto const reach{tolerance + approximation(curve)};
	return std::ranges::all_of(kChecked, [&Entry, &curve, reach](double const T) { return project(Entry, 0, plan_of(curve.point(T)), T == 0.25 || T == 0.75 ? reach : tolerance).has_value(); });
}

bool lies_along(record const &Entry, TTrack const &Track)
{
	return false == Track.m_paths.empty() && lies_along(Entry, Track.m_paths.front());
}

std::vector<std::pair<double, double>> bare(std::size_t const Index)
{
	auto const &entry{state().records[Index]};
	auto const margin{tolerance + kStep};
	auto const inside = [&](glm::dvec2 const &Point) { return Point.x >= entry.low.x - margin && Point.y >= entry.low.y - margin && Point.x <= entry.high.x + margin && Point.y <= entry.high.y + margin; };
	std::vector<std::pair<double, double>> covered;
	for (auto const *track : simulation::Paths.sequence())
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
	std::ranges::sort(covered);
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
	for (std::size_t r = 0; r < state().records.size(); ++r)
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
	for (std::size_t r = 0; r < state().records.size(); ++r)
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

namespace
{

glm::dvec2 entering(std::pair<TTrack *, bool> const &Member, bool const Atend)
{
	auto const &path{Member.first->m_paths.front()};
	return plan_of(path.points[(Member.second == Atend) ? segment_data::point::end : segment_data::point::start]);
}

// the vertices of the stored alignment whose curves lie under the tracks of the run; none when a track lies off its line
std::optional<std::set<int>> run_vertices(std::size_t const Index, std::vector<std::pair<TTrack *, bool>> const &Run)
{
	auto const &entry{state().records[Index]};
	std::set<int> vertices;
	for (auto const &[track, reversed] : Run)
	{
		geometry::bezier const curve{track->m_paths.front()};
		auto const reach{tolerance + approximation(curve)};
		std::vector<hit> hits;
		for (auto const t : kChecked)
			if (auto const found{project(Index, plan_of(curve.point(t)), t == 0.25 || t == 0.75 ? reach : tolerance)})
				hits.push_back(*found);
		if (hits.size() < 5)
			return std::nullopt;
		auto const from{std::min(hits.front().chainage, hits.back().chainage) + 0.01};
		auto const to{std::max(hits.front().chainage, hits.back().chainage) - 0.01};
		for (auto const &e : entry.result.elements)
			if (e.kind != alignment::element_kind::straight && e.chainage < to && e.chainage + e.length > from && e.vertex >= 0)
				vertices.insert(e.vertex);
	}
	return vertices;
}

// whether a curve of the vertex reaches beyond the stretch of the line between the chainages
bool curve_beyond(record const &Entry, int const Vertex, double const Low, double const High)
{
	return std::ranges::any_of(Entry.result.elements, [Vertex, Low, High](alignment::element const &E) { return E.vertex == Vertex && E.kind != alignment::element_kind::straight && (E.chainage < Low || E.chainage + E.length > High); });
}

// the vertex of the stored alignment the tracks of the run are laid by, if they are
std::optional<designed_vertex> designed_by(std::size_t const Index, std::vector<std::pair<TTrack *, bool>> const &Run)
{
	auto const &entry{state().records[Index]};
	auto const vertices{run_vertices(Index, Run)};
	if (false == vertices.has_value() || vertices->size() != 1 || *vertices->begin() >= static_cast<int>(entry.design.vertices.size()))
		return std::nullopt;
	auto const vertex{*vertices->begin()};
	// a curve cut by the deletion of tracks along it is no designed one any more: the tracks left of it are recognized as they lie
	if (curve_beyond(entry, vertex, entry.from - 0.01, entry.kept_to() + 0.01))
		return std::nullopt;
	auto const start{project(Index, entering(Run.front(), false), tolerance)};
	auto const end{project(Index, entering(Run.back(), true), tolerance)};
	if (false == start.has_value() || false == end.has_value())
		return std::nullopt;
	// the tracks of a part of the curve only, as on either side of a switch laid into it, aren't laid by its design: it's theirs only
	// where they reach over the whole curve, else they're recognized as they lie
	if (curve_beyond(entry, vertex, std::min(start->chainage, end->chainage) - 0.5, std::max(start->chainage, end->chainage) + 0.5))
		return std::nullopt;
	designed_vertex result;
	result.record = static_cast<int>(Index);
	result.vertex = vertex;
	result.shape = entry.design.vertices[result.vertex];
	result.reversed = end->chainage < start->chainage;
	if (result.reversed)
		result.shape = reversed_vertex(result.shape);
	return result;
}

} // namespace

std::optional<designed_vertex> vertex_of(std::vector<std::pair<TTrack *, bool>> const &Run)
{
	if (Run.empty() || records().empty())
		return std::nullopt;
	// a track with no path in the run lets none be found, wherever it is in it
	if (std::ranges::any_of(Run, [](std::pair<TTrack *, bool> const &Member) { return Member.first == nullptr || Member.first->m_paths.empty(); }))
		return std::nullopt;
	for (std::size_t r = 0; r < state().records.size(); ++r)
		if (auto const found{designed_by(r, Run)})
			return found;
	return std::nullopt;
}

int find(glm::dvec3 const &Start, glm::dvec3 const &End, bool &Reversed)
{
	load();
	Reversed = false;
	for (std::size_t i = 0; i < state().records.size(); ++i)
	{
		auto const &design{state().records[i].design};
		if (false == state().records[i].whole())
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

namespace
{

// the stored alignments the paths of the track lie along
void records_along(TTrack const &Track, std::set<std::size_t> &Touched)
{
	auto const &entries{state().records};
	for (std::size_t r = 0; r < entries.size(); ++r)
		if (std::ranges::any_of(Track.m_paths, [&entries, r](segment_data const &Path) { return lies_along(entries[r], Path); }))
			Touched.insert(r);
}

// the stretch of the line kept as a record of its own, with the points clicked along it; none too short for a track
void keep_piece(record const &Entry, std::vector<alignment::plan_projection> const &Projections, double const From, double const To, std::vector<record> &Kept)
{
	if (To - From < 1.0)
		return;
	auto piece{Entry};
	piece.from = From;
	piece.to = To;
	piece.clicks.clear();
	for (std::size_t i = 0; i < Entry.clicks.size() && i < Projections.size(); ++i)
		if (Projections[i].chainage >= From - 1.0 && Projections[i].chainage <= To + 1.0)
			piece.clicks.push_back(Entry.clicks[i]);
	sample(piece);
	Kept.push_back(std::move(piece));
}

// the stretches of the line between the gaps, each kept as a record of its own
void keep_pieces(record const &Entry, std::vector<std::pair<double, double>> const &Gaps, std::vector<record> &Kept)
{
	std::vector<glm::dvec2> plan;
	for (auto const &click : Entry.clicks)
		plan.push_back(plan_of(click));
	auto const projections{alignment::project(Entry.result, plan)};
	auto from{Entry.from};
	for (auto const &[start, end] : Gaps)
	{
		keep_piece(Entry, projections, from, start, Kept);
		from = end;
	}
	keep_piece(Entry, projections, from, Entry.kept_to(), Kept);
}

} // namespace

std::size_t trim(std::span<TTrack *const> const Removed)
{
	load();
	// the records the tracks deleted lay along, the ones they lay along before they were retired
	std::set<std::size_t> touched;
	for (auto const *track : Removed)
		if (track != nullptr)
			records_along(*track, touched);
	std::vector<record> kept;
	std::set<scene::layer_handle> layers;
	std::size_t cut{0};
	for (auto r{touched.rbegin()}; r != touched.rend(); ++r)
	{
		auto const gaps{bare(*r)};
		if (gaps.empty())
			continue;
		auto const entry{state().records[*r]};
		state().records.erase(state().records.begin() + static_cast<std::ptrdiff_t>(*r));
		layers.insert(entry.layer);
		++cut;
		// the stretches between the gaps, with the points clicked along them; none too short for a track
		keep_pieces(entry, gaps, kept);
	}
	if (cut == 0)
		return 0;
	state().records.insert(state().records.end(), kept.begin(), kept.end());
	state().changed = true;
	for (auto const layer : layers)
		mark(layer);
	return cut;
}

namespace
{

// the point of a track lying off the stored alignments by more than the departure so far, along a stretch of a line no track lies
// along any more: the departure and the record it lies off, the gaps of the lines found kept
void departed_at(glm::dvec2 const &Point, std::map<std::size_t, std::vector<std::pair<double, double>>> &Gaps, double &Departure, int &Result)
{
	double const reach{5.0};
	for (std::size_t r = 0; r < state().records.size(); ++r)
	{
		auto const found{project(r, Point, reach)};
		if (false == found.has_value() || found->offset <= tolerance + state().records[r].fit || found->offset <= Departure)
			continue;
		auto known{Gaps.find(r)};
		if (known == Gaps.end())
			known = Gaps.try_emplace(r, bare(r)).first;
		if (std::ranges::none_of(known->second, [&found](auto const &Gap) { return found->chainage >= Gap.first - tolerance && found->chainage <= Gap.second + tolerance; }))
			continue;
		Departure = found->offset;
		Result = found->record;
	}
}

// the distance from the point to the nearest of the points sampled along the line
double sampled_distance(record const &Entry, glm::dvec2 const &Point)
{
	auto offset{std::numeric_limits<double>::max()};
	for (auto const &sample : Entry.samples)
		offset = std::min(offset, glm::distance(sample, Point));
	return offset;
}

} // namespace

int departed(std::vector<TTrack *> const &Tracks, double &Departure)
{
	load();
	Departure = 0.0;
	int result{-1};
	std::map<std::size_t, std::vector<std::pair<double, double>>> gaps;
	for (auto *track : Tracks)
	{
		if (track == nullptr || track->m_paths.empty() || on_track(*track).has_value())
			continue;
		geometry::bezier const curve{track->m_paths.front()};
		for (auto const t : kChecked)
			departed_at(plan_of(curve.point(t)), gaps, Departure, result);
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
		for (auto const t : kChecked)
		{
			auto const point{plan_of(curve.point(t))};
			auto const found{project(entry, 0, point, 1000.0)};
			auto offset{found.has_value() ? found->offset : sampled_distance(entry, point)};
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
	return std::ranges::all_of(Tracks, [&entry](TTrack const *Track) { return Track != nullptr && lies_along(entry, *Track); });
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
	for (auto const &entry : state().records)
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

int store(alignment::design const &Design, alignment::result const &Result, double const Origin, scene::layer_handle const Layer, std::string_view const Source, double const Fit, std::vector<glm::dvec3> const &Clicks)
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
		entry.layer = state().records[index].layer;
		if (entry.clicks.empty())
			entry.clicks = state().records[index].clicks;
		state().records[index] = std::move(entry);
	}
	else
	{
		// taken into the stored alignment it lies on, else kept as one of its own
		index = merge(entry);
		if (index < 0)
		{
			entry.layer = Layer;
			state().records.push_back(std::move(entry));
			index = static_cast<int>(state().records.size()) - 1;
		}
	}
	state().changed = true;
	mark(state().records[index].layer);
	return index;
}

void forget(int const Index)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(state().records.size()))
		return;
	auto const layer{state().records[Index].layer};
	state().records.erase(state().records.begin() + Index);
	state().changed = true;
	mark(layer);
}

std::optional<hit> on_record(int const Index, glm::dvec3 const &Point, double const Reach)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(state().records.size()))
		return std::nullopt;
	return project(static_cast<std::size_t>(Index), plan_of(Point), Reach);
}

double cant_at(int const Index, double const Chainage)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(state().records.size()))
		return 0.0;
	return alignment::evaluate(state().records[Index].result, Chainage).cant;
}

void transform(int const Index, glm::dvec3 const &From, glm::dvec2 const &Fromdirection, glm::dvec3 const &To, glm::dvec2 const &Todirection)
{
	load();
	if (Index < 0 || Index >= static_cast<int>(state().records.size()))
		return;
	auto &entry{state().records[Index]};
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
	state().changed = true;
	mark(entry.layer);
}

std::optional<double> chainage_at(glm::dvec3 const &Point)
{
	auto const found{at(Point)};
	if (false == found.has_value())
		return std::nullopt;
	return state().records[found->record].origin + found->chainage;
}

std::optional<std::vector<record>> take_changes()
{
	if (false == state().changed)
		return std::nullopt;
	auto before{std::move(state().base)};
	state().base = state().records;
	state().changed = false;
	return before;
}

void settle()
{
	state().base = state().records;
	state().changed = false;
}

void restore(std::vector<record> Records)
{
	std::set<scene::layer_handle> layers;
	for (auto const *list : {&state().records, &Records})
		for (auto const &entry : *list)
			layers.insert(entry.layer);
	state().records = std::move(Records);
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
