/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorRecords.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"

#include "utilities/utilities.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace editor_records
{

namespace
{

using geometry::plan_of;

// //$h -----------------------------------------------------------------------------------------------------------------------------
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

std::optional<alignment_store::record> read_start(std::istringstream &Line, scene::layer_context const &Context, scene::layer_handle const Layer)
{
	alignment_store::record entry;
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
		// the speed of a curve slowed down below the one of the line
		if (word == "speed")
			Line >> vertex.speed;
	}
	position = Context.to_world(position);
	vertex.position = {position.x, position.z};
	vertex.reverse_turn = (reverse != 0);
	vertex.compound = (compound != 0);
	return vertex;
}

void read_part(alignment_store::record &Entry, std::string const &Keyword, std::istringstream &Line, scene::layer_context const &Context)
{
	auto &design{Entry.design};
	std::string word;
	if (Keyword == "source")
	{
		std::string shape;
		Line >> Entry.source >> word >> Entry.origin >> word >> shape >> word >> design.speed;
		design.shape = shape == "clothoid" ? alignment::transition_shape::clothoid : alignment::transition_shape::cubic_parabola;
		if (Line >> word && word == "fit")
			Line >> Entry.fit;
	}
	else if (Keyword == "ends")
	{
		glm::dvec2 startdirection;
		glm::dvec2 enddirection;
		Line >> startdirection.x >> startdirection.y >> enddirection.x >> enddirection.y >> design.start_grade >> design.end_grade >> design.start_radius >> design.end_radius;
		design.start_direction = world_direction(Context, Context.to_local(design.start), startdirection);
		design.end_direction = world_direction(Context, Context.to_local(design.end), enddirection);
		for (std::string key; Line >> key;)
		{
			if (key == "turns")
				Line >> design.start_turn >> design.end_turn;
			if (int start{0}, end{0}; key == "curved" && Line >> start >> end >> design.start_cant >> design.end_cant)
			{
				design.start_curved = (start != 0);
				design.end_curved = (end != 0);
			}
		}
	}
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
		alignment_store::stored_element e;
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

// //$r -----------------------------------------------------------------------------------------------------------------------------
// format of the lines, coordinates in the space of the file:
//   <switch|slip|crossing|double|connection> <name> <type> <rail> <R> <n of 1:n> <hand L|R|-> <a> <b> <R of the main track, + right, - left, 0 straight>
//     parts <count> <names>... [shape <main end along, across> <diverging end along, across>] start <x y z> dir <x z>
//     km <km|-> number <label|-> source <designed|measured> [middle <R in the middle of the slip curves of a double slip>]
//     [lift <m> <from x y z> <to x y z> ramp <m> [liftend <m>]]
std::string switch_word(std::string const &Text)
{
	if (Text.empty())
		return "-";
	auto result{Text};
	std::replace(result.begin(), result.end(), ' ', '_');
	return result;
}

// //$k -----------------------------------------------------------------------------------------------------------------------------
// format of the lines, coordinates in the space of the file:
//   mark <line> <x y z> way <x z> km <km> [beyond <km>]
//   via <line> <x y z>
std::string kilometrage_word(std::string const &Text)
{
	return Text.empty() ? std::string{"-"} : Text;
}

// //$s -----------------------------------------------------------------------------------------------------------------------------
// a word of the record line: blanks taken out, - for none
std::string station_word(std::string const &Text)
{
	std::string result;
	for (auto const c : Text)
		if (false == std::isspace(static_cast<unsigned char>(c)))
			result += c;
	return result.empty() ? std::string{"-"} : result;
}

std::string shown(std::string const &Word)
{
	return Word == "-" ? std::string{} : Word;
}

} // namespace

// //$h -------------------------------------------------------------------------------------------------------------------------------

std::vector<alignment_store::record> read_alignments(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer)
{
	std::vector<alignment_store::record> result;
	std::optional<alignment_store::record> current;
	for (auto const &text : Lines)
	{
		std::istringstream line{text};
		std::string keyword;
		line >> keyword;
		if (keyword == "alignment")
			current = read_start(line, Context, Layer);
		else if (false == current.has_value())
			continue;
		else if (keyword == "end")
		{
			result.push_back(std::move(*current));
			current.reset();
		}
		else
			read_part(*current, keyword, line, Context);
	}
	return result;
}

std::vector<std::string> alignment_lines(std::vector<alignment_store::record> const &Records, scene::layer_context const &Context)
{
	auto const local = [&](glm::dvec3 const &Point) {
		auto const point{Context.to_local(Point)};
		return format("%.4f %.4f %.4f", point.x, point.y, point.z);
	};
	std::vector<std::string> lines;
	for (auto const &entry : Records)
	{
		if (lines.empty())
			lines.emplace_back("# horizontal alignments designed in the track editor, the simulation skips these lines");
		auto const &design{entry.design};
		lines.push_back("alignment " + local(design.start) + ' ' + local(design.end));
		lines.push_back(format("source %s origin %.4f shape %s speed %.1f", entry.source.c_str(), entry.origin, shape_name(design.shape), design.speed) + (entry.fit > 0.0 ? format(" fit %.3f", entry.fit) : std::string{}));
		auto const startdirection{local_direction(Context, design.start, design.start_direction)};
		auto const enddirection{local_direction(Context, design.end, design.end_direction)};
		auto ends{format("ends %.9f %.9f %.9f %.9f %.6f %.6f %.3f %.3f", startdirection.x, startdirection.y, enddirection.x, enddirection.y, design.start_grade, design.end_grade, design.start_radius, design.end_radius)};
		// the sides the adjoining tracks turn to, and the curves going on from them with their cants, written since there are such
		if (design.start_turn != 0 || design.end_turn != 0)
			ends += format(" turns %d %d", design.start_turn, design.end_turn);
		if (design.start_curved || design.end_curved)
			ends += format(" curved %d %d %.1f %.1f", design.start_curved ? 1 : 0, design.end_curved ? 1 : 0, design.start_cant, design.end_cant);
		lines.push_back(ends);
		auto const &norms{design.norms};
		lines.push_back(format("norms %.1f %.3f %.1f %.1f %.2f %.3f %.2f %.2f %.3f", norms.gauge, norms.unbalanced, norms.cant_max, norms.cant_rate, norms.ramp_factor, norms.jerk, norms.tangent_min_time, norms.arc_min, norms.arc_min_factor));
		lines.push_back(format("pieces %d %.3f", design.transition_pieces, design.arc_piece_angle));
		if (false == entry.whole())
			lines.push_back(format("kept %.4f %.4f", entry.from, entry.kept_to()));
		for (auto const &vertex : design.vertices)
		{
			auto const position{Context.to_local({vertex.position.x, 0.0, vertex.position.y})};
			auto text{format("vertex %.4f %.4f %.4f %.3f %.3f %.3f %.1f %d %d %.9f %zu", position.x, position.z, vertex.offset, vertex.radius, vertex.transition_in, vertex.transition_out, vertex.cant, vertex.reverse_turn ? 1 : 0, vertex.compound ? 1 : 0,
			                  vertex.share, vertex.arcs.size())};
			for (auto const &arc : vertex.arcs)
				text += format(" %.3f %.3f %.9f", arc.radius, arc.transition, arc.share);
			if (vertex.spirals)
				text += " spirals 1";
			if (vertex.first_cant >= 0.0 || std::any_of(vertex.arcs.begin(), vertex.arcs.end(), [](alignment::compound_arc const &Arc) { return Arc.cant >= 0.0; }))
			{
				text += format(" cants %.1f", vertex.first_cant);
				for (auto const &arc : vertex.arcs)
					text += format(" %.1f", arc.cant);
			}
			if (vertex.own_ramps())
				text += format(" ramps %.3f %.3f %.3f %.3f", vertex.ramp_in_offset, vertex.ramp_in_length, vertex.ramp_out_offset, vertex.ramp_out_length);
			if (vertex.speed > 0.0)
				text += format(" speed %.1f", vertex.speed);
			lines.push_back(text);
		}
		for (auto const &click : entry.clicks)
			lines.push_back("click " + local(click));
		for (auto const &e : entry.result.elements)
		{
			auto const station{alignment::evaluate(entry.result, e.chainage)};
			auto const start{Context.to_local(station.position)};
			auto const direction{local_direction(Context, station.position, glm::normalize(plan_of(station.direction)))};
			auto const element{alignment_store::describe(e)};
			lines.push_back(format("element %s %d %.4f %.4f %.3f %.3f %.1f %.4f %.4f %.9f %.9f", element.kind.c_str(), element.vertex, element.chainage, element.length, element.radius_start, element.radius_end, element.cant, start.x, start.z, direction.x, direction.y));
		}
		lines.emplace_back("end");
	}
	return lines;
}

} // namespace editor_records

// the element as the //$h lines give it
alignment_store::stored_element alignment_store::describe(alignment::element const &Element)
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

namespace editor_records
{

// //$r -------------------------------------------------------------------------------------------------------------------------------

std::vector<switch_store::record> read_switches(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer)
{
	std::vector<switch_store::record> result;
	for (auto const &text : Lines)
	{
		std::istringstream line{text};
		switch_store::record entry;
		entry.layer = Layer;
		std::string keyword;
		std::string hand;
		std::string km;
		std::size_t count{0};
		line >> entry.kind;
		if (entry.kind != "switch" && entry.kind != "slip" && entry.kind != "crossing" && entry.kind != "double" && entry.kind != "connection")
			continue;
		line >> entry.name >> entry.type >> entry.rail >> entry.radius >> entry.ratio >> hand >> entry.a >> entry.b >> entry.main_radius >> keyword >> count;
		if (line.fail() || keyword != "parts" || count == 0 || count > 64)
			continue;
		entry.hand = hand.empty() ? '-' : hand.front();
		entry.parts.resize(count);
		for (auto &part : entry.parts)
			line >> part;
		glm::dvec3 start{0.0};
		glm::dvec3 ahead{0.0};
		glm::dvec2 direction{0.0, 1.0};
		line >> keyword;
		if (keyword == "shape")
		{
			line >> entry.shape.x >> entry.shape.y >> entry.shape.z >> entry.shape.w >> keyword;
			entry.shaped = true;
		}
		line >> start.x >> start.y >> start.z >> keyword >> direction.x >> direction.y >> keyword >> km >> keyword >> entry.number >> keyword >> entry.source;
		if (line.fail())
			continue;
		while (line >> keyword)
		{
			if (keyword == "middle")
				line >> entry.middle;
			else if (keyword == "lift")
			{
				glm::dvec3 from{0.0}, to{0.0};
				line >> entry.lift >> from.x >> from.y >> from.z >> to.x >> to.y >> to.z >> keyword >> entry.ramp;
				entry.lift_end = entry.lift;
				entry.lift_from = Context.to_world(from);
				entry.lift_to = Context.to_world(to);
			}
			else if (keyword == "liftend")
				line >> entry.lift_end;
		}
		entry.start = Context.to_world(start);
		ahead = Context.to_world(start + glm::dvec3{direction.x, 0.0, direction.y});
		entry.direction = plan_of(ahead - entry.start);
		entry.direction = glm::length(entry.direction) > 1e-12 ? glm::normalize(entry.direction) : glm::dvec2{0.0, 1.0};
		if (km != "-")
		{
			char *end{nullptr};
			auto const value{std::strtod(km.c_str(), &end)};
			if (end != km.c_str() && *end == '\0' && std::isfinite(value))
				entry.km = value * 1000.0;
		}
		result.push_back(std::move(entry));
	}
	return result;
}

std::vector<std::string> switch_lines(std::vector<switch_store::record> const &Records, scene::layer_context const &Context)
{
	std::vector<std::string> lines;
	for (auto const &entry : Records)
	{
		if (lines.empty())
			lines.emplace_back("# switches placed in the track editor, the simulation skips these lines");
		auto const start{Context.to_local(entry.start)};
		auto const ahead{Context.to_local(entry.start + glm::dvec3{entry.direction.x, 0.0, entry.direction.y})};
		auto direction{plan_of(ahead - start)};
		direction = glm::length(direction) > 1e-12 ? glm::normalize(direction) : entry.direction;
		auto text{format("%s %s %s %s %.3f %.3f %c %.3f %.3f %.3f parts %zu", entry.kind.c_str(), switch_word(entry.name).c_str(), switch_word(entry.type).c_str(), switch_word(entry.rail).c_str(), entry.radius, entry.ratio, entry.hand, entry.a, entry.b,
		                 entry.main_radius, entry.parts.size())};
		for (auto const &part : entry.parts)
			text += ' ' + switch_word(part);
		if (entry.shaped)
			text += format(" shape %.4f %.4f %.4f %.4f", entry.shape.x, entry.shape.y, entry.shape.z, entry.shape.w);
		text += format(" start %.4f %.4f %.4f dir %.9f %.9f km ", start.x, start.y, start.z, direction.x, direction.y);
		text += entry.km.has_value() ? format("%.3f", *entry.km / 1000.0) : std::string{"-"};
		text += " number " + switch_word(entry.number) + " source " + switch_word(entry.source);
		if (entry.middle > 0.0)
			text += format(" middle %.3f", entry.middle);
		if (entry.lift != 0.0 || entry.lift_end != 0.0)
		{
			auto const from{Context.to_local(entry.lift_from)};
			auto const to{Context.to_local(entry.lift_to)};
			text += format(" lift %.4f %.4f %.4f %.4f %.4f %.4f %.4f ramp %.3f", entry.lift, from.x, from.y, from.z, to.x, to.y, to.z, entry.ramp);
			if (entry.lift_end != entry.lift)
				text += format(" liftend %.4f", entry.lift_end);
		}
		lines.push_back(text);
	}
	return lines;
}

// //$k -------------------------------------------------------------------------------------------------------------------------------

void read_kilometrage(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer, std::vector<kilometrage::mark> &Marks, std::vector<kilometrage::via> &Vias)
{
	for (auto const &text : Lines)
	{
		std::istringstream line{text};
		std::string keyword;
		kilometrage::mark entry;
		entry.layer = Layer;
		glm::dvec3 point{0.0};
		glm::dvec2 way{0.0, 1.0};
		line >> keyword;
		if (keyword == "via")
		{
			kilometrage::via via;
			via.layer = Layer;
			glm::dvec3 at{0.0};
			line >> via.line >> at.x >> at.y >> at.z;
			if (line.fail() || via.line.empty())
				continue;
			via.point = Context.to_world(at);
			Vias.push_back(std::move(via));
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
		entry.point = Context.to_world(point);
		entry.way = plan_of(Context.to_world(point + glm::dvec3{way.x, 0.0, way.y}) - entry.point);
		entry.way = glm::length(entry.way) > 1e-12 ? glm::normalize(entry.way) : glm::dvec2{0.0, 1.0};
		Marks.push_back(std::move(entry));
	}
}

std::vector<std::string> kilometrage_lines(std::vector<kilometrage::mark> const &Marks, std::vector<kilometrage::via> const &Vias, scene::layer_context const &Context)
{
	std::vector<std::string> lines;
	auto const heading = [&]() {
		if (lines.empty())
			lines.emplace_back("# kilometrage of the lines set in the track editor, the simulation skips these lines");
	};
	for (auto const &entry : Marks)
	{
		heading();
		auto const point{Context.to_local(entry.point)};
		auto const ahead{Context.to_local(entry.point + glm::dvec3{entry.way.x, 0.0, entry.way.y})};
		auto way{plan_of(ahead - point)};
		way = glm::length(way) > 1e-12 ? glm::normalize(way) : entry.way;
		auto text{format("mark %s %.4f %.4f %.4f way %.9f %.9f km %.3f", kilometrage_word(entry.line).c_str(), point.x, point.y, point.z, way.x, way.y, entry.km)};
		if (entry.beyond.has_value())
			text += format(" beyond %.3f", *entry.beyond);
		lines.push_back(text);
	}
	for (auto const &entry : Vias)
	{
		heading();
		auto const point{Context.to_local(entry.point)};
		lines.push_back(format("via %s %.4f %.4f %.4f", kilometrage_word(entry.line).c_str(), point.x, point.y, point.z));
	}
	return lines;
}

// //$p -------------------------------------------------------------------------------------------------------------------------------
// format of the lines, coordinates in the space of the file:
//   profile <length> <start x y z> <end x y z> <middle x y z>
//   speed <km/h> origin <chainage of the start> join <start 0/1> <end 0/1>
//   norms <grade max> <grade over switches max> <k of R >= k*V^2> <R min> <change needing a curve> <constant grade min> <fixed tolerance>
//   [departure <m>]
//   point <chainage> <elevation> <vertical curve R> <automatic R 0/1>
//   end

std::vector<grade_profile> read_profiles(std::vector<std::string> const &Lines, scene::layer_context const &Context, scene::layer_handle const Layer)
{
	std::vector<grade_profile> result;
	auto const height = [&](double const Local) { return Context.to_world({0.0, Local, 0.0}).y; };
	std::optional<grade_profile> current;
	for (auto const &text : Lines)
	{
		std::istringstream line{text};
		std::string keyword;
		line >> keyword;
		if (keyword == "profile")
		{
			grade_profile entry;
			entry.layer = Layer;
			glm::dvec3 start;
			glm::dvec3 end;
			glm::dvec3 middle;
			line >> entry.length >> start.x >> start.y >> start.z >> end.x >> end.y >> end.z >> middle.x >> middle.y >> middle.z;
			if (line.fail())
			{
				current.reset();
				continue;
			}
			entry.start = Context.to_world(start);
			entry.end = Context.to_world(end);
			entry.middle = Context.to_world(middle);
			current = entry;
			continue;
		}
		if (false == current.has_value())
			continue;
		auto &entry{*current};
		std::string word;
		if (keyword == "speed")
		{
			int start{1};
			int end{1};
			line >> entry.line.speed >> word >> entry.origin >> word >> start >> end;
			entry.line.join_start = (start != 0);
			entry.line.join_end = (end != 0);
		}
		else if (keyword == "norms")
		{
			auto &norms{entry.line.norms};
			line >> norms.grade_max >> norms.grade_switch_max >> norms.radius_factor >> norms.radius_min >> norms.curve_threshold >> norms.element_min >> norms.fixed_tolerance;
		}
		else if (keyword == "departure")
			line >> entry.departure;
		else if (keyword == "point")
		{
			profile::pvi point;
			int automatic{0};
			line >> point.chainage >> point.elevation >> point.radius >> automatic;
			if (line.fail())
				continue;
			point.elevation = height(point.elevation);
			point.automatic = (automatic != 0);
			entry.line.points.push_back(point);
		}
		else if (keyword == "end")
		{
			if (entry.line.points.size() >= 2 && entry.length > 0.0)
				result.push_back(entry);
			current.reset();
		}
	}
	return result;
}

std::vector<std::string> profile_lines(std::vector<grade_profile> const &Profiles, scene::layer_context const &Context)
{
	auto const local = [&](glm::dvec3 const &Point) {
		auto const point{Context.to_local(Point)};
		return format("%.4f %.4f %.4f", point.x, point.y, point.z);
	};
	std::vector<std::string> lines;
	for (auto const &entry : Profiles)
	{
		if (lines.empty())
			lines.emplace_back("# vertical profiles (grade lines) designed in the track editor, the simulation skips these lines");
		lines.push_back(format("profile %.4f ", entry.length) + local(entry.start) + ' ' + local(entry.end) + ' ' + local(entry.middle));
		lines.push_back(format("speed %.1f origin %.4f join %d %d", entry.line.speed, entry.origin, entry.line.join_start ? 1 : 0, entry.line.join_end ? 1 : 0));
		auto const &norms{entry.line.norms};
		lines.push_back(format("norms %.4f %.4f %.4f %.1f %.4f %.2f %.4f", norms.grade_max, norms.grade_switch_max, norms.radius_factor, norms.radius_min, norms.curve_threshold, norms.element_min, norms.fixed_tolerance));
		if (entry.departure >= 0.0)
			lines.push_back(format("departure %.4f", entry.departure));
		for (auto const &point : entry.line.points)
			lines.push_back(format("point %.4f %.4f %.1f %d", point.chainage, Context.to_local({0.0, point.elevation, 0.0}).y, point.radius, point.automatic ? 1 : 0));
		lines.emplace_back("end");
	}
	return lines;
}

// //$t -------------------------------------------------------------------------------------------------------------------------------
// format of the lines: fitted <name> <shape, 16 hex digits> [hand]

std::vector<fitted_track> read_fitted(std::vector<std::string> const &Lines)
{
	std::vector<fitted_track> result;
	for (auto const &text : Lines)
	{
		std::istringstream line{text};
		std::string keyword, name, shape, kind;
		line >> keyword >> name >> shape;
		if (keyword == "fitted" && false == line.fail())
		{
			line >> kind;
			result.push_back({name, std::strtoull(shape.c_str(), nullptr, 16), kind == "hand"});
		}
	}
	return result;
}

std::vector<std::string> fitted_lines(std::vector<fitted_track> const &Tracks)
{
	std::vector<std::string> lines;
	for (auto const &track : Tracks)
	{
		if (lines.empty())
			lines.emplace_back("# the tracks whose grade line was fitted in the track editor, with their shape then; the simulation skips these lines");
		lines.push_back(format("fitted %s %016llx%s", track.name.c_str(), static_cast<unsigned long long>(track.shape), track.hand ? " hand" : ""));
	}
	return lines;
}

// //$w -------------------------------------------------------------------------------------------------------------------------------
// format of the lines: gradeline <key> <number> [hand]

std::vector<grade_line> read_grade_lines(std::vector<std::string> const &Lines)
{
	std::vector<grade_line> result;
	for (auto const &text : Lines)
	{
		std::istringstream line{text};
		std::string keyword, name, hand;
		int number{0};
		line >> keyword >> name >> number;
		if (keyword == "gradeline" && false == line.fail())
			result.push_back({name, number, (line >> hand) && hand == "hand"});
	}
	return result;
}

std::vector<std::string> grade_line_lines(std::vector<grade_line> const &Lines)
{
	std::vector<std::string> lines;
	for (auto const &entry : Lines)
	{
		if (lines.empty())
			lines.emplace_back("# the tracks sharing a grade line, each with the number of it; the simulation skips these lines");
		lines.push_back(format("gradeline %s %d%s", entry.key.c_str(), entry.number, entry.hand ? " hand" : ""));
	}
	return lines;
}

// //$b -------------------------------------------------------------------------------------------------------------------------------
// format of the lines, coordinates in the space of the file:
//   <kind> <category> <x y z of the object> <file or name, - if none> <turns 0/1> <anchor count> { <path> <start x y z> <end x y z> }
// the object is the model, the include (position from its parameters), the traction piece (its first point), the memory
// cell or the event launcher found at the location; the anchors name the paths by their ends

std::optional<binding> read_binding(std::string const &Line, scene::layer_context const &Context)
{
	if (Line.empty() || Line.front() == '#')
		return std::nullopt;
	std::istringstream line{Line};
	binding result;
	int turns{0};
	int count{0};
	line >> result.kind >> result.category >> result.location.x >> result.location.y >> result.location.z >> result.key >> turns >> count;
	if (line.fail() || count < 1 || count > 4)
		return std::nullopt;
	result.location = Context.to_world(result.location);
	result.turns = (turns != 0);
	for (int i = 0; i < count; ++i)
	{
		binding::anchor anchor;
		line >> anchor.path >> anchor.start.x >> anchor.start.y >> anchor.start.z >> anchor.end.x >> anchor.end.y >> anchor.end.z;
		if (line.fail())
			return std::nullopt;
		anchor.start = Context.to_world(anchor.start);
		anchor.end = Context.to_world(anchor.end);
		result.anchors.push_back(anchor);
	}
	return result;
}

std::string binding_line(binding const &Binding, scene::layer_context const &Context)
{
	auto const local = [&](glm::dvec3 const &Point) {
		auto const point{Context.to_local(Point)};
		return format("%.4f %.4f %.4f", point.x, point.y, point.z);
	};
	auto text{Binding.kind + ' ' + std::to_string(Binding.category) + ' ' + local(Binding.location) + ' ' + Binding.key + ' ' + (Binding.turns ? "1 " : "0 ") + std::to_string(Binding.anchors.size())};
	for (auto const &anchor : Binding.anchors)
		text += ' ' + std::to_string(anchor.path) + ' ' + local(anchor.start) + ' ' + local(anchor.end);
	return text;
}

std::string binding_heading()
{
	return "# objects bound to the paths in the track editor, they follow changes of the track; the simulation skips these lines";
}

// //$s -------------------------------------------------------------------------------------------------------------------------------
// format of the lines:
//   station <code> line <number> km <km> from <left|right> anchor <switch> [braking <m>] [blocks <signal>=<semi|sbl|sbl4>,...]
//     [ways <signal>=<1|2>,...] [joints <+|-><x>/<y>/<z>,...] name <the rest of the line, windows-1250>
// - for an empty word, the joints in the world

std::optional<station> read_station(std::string const &Line)
{
	std::istringstream words{Line};
	std::string word;
	if ((words >> word).fail() || word != "station")
		return std::nullopt;
	station entry;
	words >> entry.code;
	while (words >> word)
	{
		if (word == "line")
			words >> entry.line;
		else if (word == "km")
			words >> entry.km;
		else if (word == "from")
			entry.reversed = (words >> word) && word == "right";
		else if (word == "braking")
			words >> entry.braking;
		else if (word == "blocks" && words >> word)
			for (std::size_t from = 0; from < word.size();)
			{
				auto const comma{std::min(word.find(',', from), word.size())};
				auto const item{word.substr(from, comma - from)};
				if (auto const equals{item.find('=')}; equals != std::string::npos && equals > 0)
				{
					auto const kind{item.substr(equals + 1)};
					entry.blocks[item.substr(0, equals)] = kind == "sbl4" ? block_kind::four : kind == "sbl" || kind == "sbl3" ? block_kind::three : block_kind::semi;
				}
				from = comma + 1;
			}
		else if (word == "ways" && words >> word)
			for (std::size_t from = 0; from < word.size();)
			{
				auto const comma{std::min(word.find(',', from), word.size())};
				auto const item{word.substr(from, comma - from)};
				if (auto const equals{item.find('=')}; equals != std::string::npos && equals > 0)
					entry.ways[item.substr(0, equals)] = item.substr(equals + 1) == "2";
				from = comma + 1;
			}
		else if (word == "joints" && words >> word)
			// +x/y/z for one put by hand, -x/y/z for a planned one taken away
			for (std::size_t from = 0; from < word.size();)
			{
				auto const comma{std::min(word.find(',', from), word.size())};
				auto const item{word.substr(from, comma - from)};
				glm::dvec3 point{0.0};
				if (item.size() > 1 && (item.front() == '+' || item.front() == '-') && std::sscanf(item.c_str() + 1, "%lf/%lf/%lf", &point.x, &point.y, &point.z) == 3)
					(item.front() == '+' ? entry.joints_added : entry.joints_removed).push_back(point);
				from = comma + 1;
			}
		else if (word == "anchor")
			words >> entry.anchor;
		else if (word == "name")
		{
			std::getline(words, entry.name);
			entry.name.erase(0, entry.name.find_first_not_of(' '));
			// windows-1250 like the rest of the scenery file; earlier builds wrote it in utf-8, which stays as it is
			if (std::u32string decoded; false == utf8_to_utf32(entry.name, decoded))
				entry.name = win1250_to_utf8(entry.name);
			break;
		}
	}
	entry.code = shown(entry.code);
	entry.line = shown(entry.line);
	entry.km = shown(entry.km);
	if (entry.anchor.empty())
		return std::nullopt;
	return entry;
}

std::vector<std::string> station_lines(std::vector<station> const &Stations)
{
	std::vector<std::string> lines;
	for (auto const &entry : Stations)
	{
		if (lines.empty())
			lines.emplace_back("# stations described in the track editor, the simulation skips these lines");
		auto const braking{entry.braking > 0 ? format(" braking %d", entry.braking) : std::string{}};
		std::string blocks;
		for (auto const &[signal, kind] : entry.blocks)
			if (false == signal.empty() && signal.find_first_of(" ,=\t") == std::string::npos)
				blocks += (blocks.empty() ? " blocks " : ",") + signal + (kind == block_kind::four ? "=sbl4" : kind == block_kind::three ? "=sbl" : "=semi");
		std::string ways;
		for (auto const &[signal, both] : entry.ways)
			if (false == signal.empty() && signal.find_first_of(" ,=\t") == std::string::npos)
				ways += (ways.empty() ? " ways " : ",") + signal + (both ? "=2" : "=1");
		std::string joints;
		for (auto const *list : {&entry.joints_added, &entry.joints_removed})
			for (auto const &point : *list)
				joints += (joints.empty() ? " joints " : ",") + format("%c%.3f/%.3f/%.3f", list == &entry.joints_added ? '+' : '-', point.x, point.y, point.z);
		lines.push_back(format("station %s line %s km %s from %s anchor %s%s%s%s%s name %s", station_word(entry.code).c_str(), station_word(entry.line).c_str(), station_word(entry.km).c_str(), entry.reversed ? "right" : "left", entry.anchor.c_str(),
		                       braking.c_str(), blocks.c_str(), ways.c_str(), joints.c_str(), utf8_to_win1250(entry.name).c_str()));
	}
	return lines;
}

// //$c -------------------------------------------------------------------------------------------------------------------------------
// format of the line: <x y z> <pitch> <yaw>

std::optional<camera> read_camera(std::vector<std::string> const &Lines)
{
	if (Lines.empty())
		return std::nullopt;
	std::istringstream values(Lines.front());
	camera result;
	if (values >> result.position.x >> result.position.y >> result.position.z >> result.angle.x >> result.angle.y)
		return result;
	return std::nullopt;
}

std::string camera_line(camera const &Camera)
{
	return format("%.3f %.3f %.3f %.5f %.5f", Camera.position.x, Camera.position.y, Camera.position.z, Camera.angle.x, Camera.angle.y);
}

// //$o -------------------------------------------------------------------------------------------------------------------------------
// format of the line: on <0/1> [origin <north> <east>]

std::string orthophoto_line(bool const Enabled, bool const Origin, double const North, double const East, bool const Aligned)
{
	// how the imagery is shown is the user's, kept with the editor (EditorSettings); the scenery keeps whether it's on and where the
	// scenery lies
	char text[256];
	std::snprintf(text, sizeof(text), "on %d", Enabled ? 1 : 0);
	std::string result{text};
	if (Origin)
	{
		std::snprintf(text, sizeof(text), " origin %.2f %.2f", North, East);
		result += text;
		if (Aligned)
			result += " aligned";
	}
	return result;
}

bool read_orthophoto(std::string const &Line, bool &Enabled, double &North, double &East, bool *Aligned)
{
	std::istringstream words{Line};
	std::string key;
	auto found{false};
	// the year, opacity, radius and the rest the lines of the older sceneries carry are passed over: the user's settings stand
	while (words >> key)
	{
		if (key == "on")
			words >> Enabled;
		else if (key == "origin")
			words >> North >> East;
		else if (key == "aligned" && Aligned != nullptr)
			*Aligned = true;
		else
			continue;
		if (words.fail())
			return found;
		found = true;
	}
	return found;
}

// //$z -------------------------------------------------------------------------------------------------------------------------------
// format of the lines: point <easting> <northing>

std::vector<std::string> area_lines(std::vector<glm::dvec2> const &Polygon)
{
	std::vector<std::string> lines;
	if (Polygon.size() < 3)
		return lines;
	lines.emplace_back("# the area the editor downloads the orthophoto and the terrain model in, PUWG 1992 easting northing");
	for (auto const &point : Polygon)
	{
		char text[64];
		std::snprintf(text, sizeof(text), "point %.2f %.2f", point.x, point.y);
		lines.emplace_back(text);
	}
	return lines;
}

std::vector<glm::dvec2> read_area(std::vector<std::string> const &Lines)
{
	std::vector<glm::dvec2> polygon;
	for (auto const &line : Lines)
	{
		std::istringstream words{line};
		std::string keyword;
		glm::dvec2 point{0.0};
		if (words >> keyword && keyword == "point" && words >> point.x >> point.y)
			polygon.push_back(point);
	}
	if (polygon.size() < 3)
		polygon.clear();
	return polygon;
}

} // namespace editor_records
