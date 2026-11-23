/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/projectFile.hpp"
#include "editor/layout/layoutTurnout.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdio>
#include <format>
#include <fstream>
#include <iterator>
#include <map>
#include <system_error>

namespace project
{

namespace
{

using layout::line_id;
using layout::node_id;
using layout::turnout_id;
using layout::vertex_id;

// the shortest form which reads back to the same double
std::string number(double const Value)
{
	return std::format("{}", Value);
}

std::array<std::pair<norms::profile_id, std::string_view>, 4> const profile_names{{
    {norms::profile_id::plk, "plk"},
    {norms::profile_id::tsi, "tsi"},
    {norms::profile_id::en13803, "en13803"},
    {norms::profile_id::ch, "ch"},
}};

std::string_view profile_name(norms::profile_id const Profile)
{
	for (auto const &[id, name] : profile_names)
		if (id == Profile)
			return name;
	return "plk";
}

std::string_view traffic_name(norms::traffic const Kind)
{
	switch (Kind)
	{
	case norms::traffic::passenger: return "passenger";
	case norms::traffic::freight: return "freight";
	default: return "mixed";
	}
}

std::string_view transition_name(layout::transition_shape const Shape)
{
	switch (Shape)
	{
	case layout::transition_shape::cubic_parabola: return "parabola";
	case layout::transition_shape::bloss: return "bloss";
	default: return "clothoid";
	}
}

// the classes of the lines as the file numbers them: each one once, in the order of the lines which have it
std::vector<norms::line_class> classes_of(layout::network const &Network)
{
	std::vector<norms::line_class> result;
	for (auto const &[id, line] : Network.lines())
		if (std::ranges::find(result, line.category) == result.end())
			result.push_back(line.category);
	return result;
}

std::size_t class_number(std::vector<norms::line_class> const &Classes, norms::line_class const &Class)
{
	return static_cast<std::size_t>(std::ranges::find(Classes, Class) - Classes.begin()) + 1;
}

std::string hex(std::uint64_t const Value)
{
	return std::format("{:016x}", Value);
}

// the words of a line of the file, a blank or more between them
std::vector<std::string_view> words_of(std::string_view const Line)
{
	std::vector<std::string_view> result;
	std::size_t at{0};
	while (at < Line.size())
	{
		auto const begin{Line.find_first_not_of(" \t", at)};
		if (begin == std::string_view::npos)
			break;
		auto end{Line.find_first_of(" \t", begin)};
		end = (end == std::string_view::npos ? Line.size() : end);
		result.push_back(Line.substr(begin, end - begin));
		at = end;
	}
	return result;
}

// the words of a record read one after another; the first one which isn't what's expected of it is the error
class record
{
  public:
	explicit record(std::string_view const Line) : m_words{words_of(Line)} {}

	bool empty() const { return m_words.empty(); }
	bool done() const { return m_at >= m_words.size(); }
	bool failed() const { return false == m_error.empty(); }
	std::string const &error() const { return m_error; }
	std::string_view peek() const { return done() ? std::string_view{} : m_words[m_at]; }

	std::string_view next(std::string_view const What)
	{
		if (failed())
			return {};
		if (done())
		{
			fail(std::format("{} missing", What));
			return {};
		}
		return m_words[m_at++];
	}
	void expect(std::string_view const Keyword)
	{
		if (auto const got{next(Keyword)}; false == failed() && got != Keyword)
			fail(std::format("'{}' where '{}' was expected", got, Keyword));
	}
	double real(std::string_view const What)
	{
		auto const text{next(What)};
		double value{0.0};
		if (false == failed())
		{
			auto const [end, error]{std::from_chars(text.data(), text.data() + text.size(), value)};
			if (error != std::errc{} || end != text.data() + text.size())
				fail(std::format("'{}' isn't a number ({})", text, What));
		}
		return value;
	}
	std::uint64_t whole(std::string_view const What, int const Base = 10)
	{
		auto const text{next(What)};
		std::uint64_t value{0};
		if (false == failed())
		{
			auto const [end, error]{std::from_chars(text.data(), text.data() + text.size(), value, Base)};
			if (error != std::errc{} || end != text.data() + text.size())
				fail(std::format("'{}' isn't a whole number ({})", text, What));
		}
		return value;
	}
	std::uint32_t id(std::string_view const What)
	{
		auto const value{whole(What)};
		if (false == failed() && value > 0xffffffffu)
			fail(std::format("{} {} is out of range", What, value));
		return static_cast<std::uint32_t>(value);
	}
	// one of the names, the index of it
	template <std::size_t Count>
	std::size_t choice(std::string_view const What, std::array<std::string_view, Count> const &Names)
	{
		auto const text{next(What)};
		if (failed())
			return 0;
		for (std::size_t i = 0; i < Count; ++i)
			if (Names[i] == text)
				return i;
		fail(std::format("'{}' isn't a {}", text, What));
		return 0;
	}
	norms::profile_id profile(std::string_view const What)
	{
		auto const text{next(What)};
		for (auto const &[id, name] : profile_names)
			if (name == text)
				return id;
		if (false == failed())
			fail(std::format("'{}' isn't a profile of the norms", text));
		return norms::profile_id::plk;
	}
	// the rest of the line as it is written, the blanks between the words kept as one
	std::string rest()
	{
		std::string result;
		for (; false == done(); ++m_at)
			result += (result.empty() ? "" : " ") + std::string{m_words[m_at]};
		return result;
	}
	void finish()
	{
		if (false == failed() && false == done())
			fail(std::format("'{}' left over", m_words[m_at]));
	}
	void fail(std::string Error)
	{
		if (m_error.empty())
			m_error = std::move(Error);
	}

  private:
	std::vector<std::string_view> m_words;
	std::size_t m_at{0};
	std::string m_error;
};

bool same_areas(norms::area_map const &A, norms::area_map const &B)
{
	if (A.scenery != B.scenery || A.lines != B.lines || A.areas.size() != B.areas.size())
		return false;
	for (std::size_t i = 0; i < A.areas.size(); ++i)
	{
		auto const &a{A.areas[i]};
		auto const &b{B.areas[i]};
		if (a.name != b.name || a.profile != b.profile || a.outline.size() != b.outline.size())
			return false;
		for (std::size_t p = 0; p < a.outline.size(); ++p)
			if (a.outline[p].x != b.outline[p].x || a.outline[p].z != b.outline[p].z)
				return false;
	}
	return true;
}

// the sections of the file
enum class section
{
	top,
	line,
	turnouts,
	anchors,
	generated,
	legacy
};

} // namespace

std::string word(std::string_view const Text)
{
	if (Text.empty())
		return "-";
	if (Text == "-")
		return "%2D";
	std::string result;
	for (auto const c : Text)
	{
		if (c == ' ')
			result += "%20";
		else if (c == '%')
			result += "%25";
		else if (c == '\t')
			result += "%09";
		else
			result += c;
	}
	return result;
}

std::string unword(std::string_view const Word)
{
	if (Word == "-")
		return {};
	std::string result;
	for (std::size_t i = 0; i < Word.size(); ++i)
	{
		if (Word[i] == '%' && i + 2 < Word.size())
		{
			unsigned value{0};
			auto const [end, error]{std::from_chars(Word.data() + i + 1, Word.data() + i + 3, value, 16)};
			if (error == std::errc{} && end == Word.data() + i + 3)
			{
				result += static_cast<char>(value);
				i += 2;
				continue;
			}
		}
		result += Word[i];
	}
	return result;
}

bool same(document const &A, document const &B)
{
	return A.network.same(B.network) && A.network.handed() == B.network.handed() && same_areas(A.network.areas, B.network.areas) && A.generated == B.generated && A.hand_changed == B.hand_changed && A.anchors == B.anchors &&
	       A.legacy == B.legacy;
}

std::string write(document const &Project)
{
	auto const &network{Project.network};
	std::string text;
	auto const put = [&](std::string const &Line) {
		text += Line;
		text += '\n';
	};
	put(std::format("eu07proj {}", schema_version));
	put("# the project of the track editor for the scenery next to it: the lines, the nodes and the norms of its tracks; the editor");
	put("# reads and writes it whole, the simulator doesn't read it");
	auto const handed{network.handed()};
	put(std::format("next node {} line {} vertex {} turnout {}", handed.node + 1ull, handed.line + 1ull, handed.vertex + 1ull, handed.turnout + 1ull));
	put(std::format("norms {}", profile_name(network.areas.scenery)));
	for (auto const &area : network.areas.areas)
	{
		auto line{std::format("area {} {} {}", word(area.name), profile_name(area.profile), area.outline.size())};
		for (auto const &point : area.outline)
			line += ' ' + number(point.x) + ' ' + number(point.z);
		put(line);
	}
	for (auto const &[name, profile] : network.areas.lines)
		put(std::format("lineprofile {} {}", word(name), profile_name(profile)));
	auto const classes{classes_of(network)};
	for (std::size_t i = 0; i < classes.size(); ++i)
	{
		auto const &kind{classes[i]};
		put(std::format("class {} vmax {} vmin {} traffic {} track {} load {} features {}", i + 1, number(kind.vmax), number(kind.vmin), traffic_name(kind.kind), kind.track == norms::track_kind::side ? "side" : "main", number(kind.load), kind.features));
	}
	for (auto const &[id, node] : network.nodes())
	{
		auto line{std::format("node {} at {} {} {} heading {} {} curvature {} cant {} grade {} ends {}", layout::number(id), number(node.position.x), number(node.position.y), number(node.position.z), number(node.heading.x), number(node.heading.y),
		                      number(node.curvature), number(node.cant), number(node.grade), node.ends.size())};
		for (auto const &end : node.ends)
			line += std::format(" {} {} {}", layout::number(end.line), end.end == layout::line_end::start ? "start" : "end", end.sense);
		put(line);
	}
	for (auto const &[id, line] : network.lines())
	{
		put(std::format("line {} from {} to {} class {} chainage {} transition {} ramp {} continues {} {} turnout {} profile {}", layout::number(id), layout::number(line.start), layout::number(line.end), class_number(classes, line.category),
		                number(line.chainage_start), transition_name(line.shape.transition), line.shape.ramp == layout::ramp_shape::bloss ? "bloss" : "linear", line.start_continues ? 1 : 0, line.end_continues ? 1 : 0, layout::number(line.turnout),
		                line.profile.has_value() ? profile_name(*line.profile) : "-"));
		for (auto const &vertex : line.shape.vertices)
			put(std::format("vertex {} at {} {} radius {} in {} out {} cant {}", layout::number(vertex.id), number(vertex.point.x), number(vertex.point.y), number(vertex.radius), number(vertex.transition_in), number(vertex.transition_out), number(vertex.cant)));
		for (auto const &grade : line.shape.grades)
			put(std::format("grade {} {} {}", number(grade.chainage), number(grade.elevation), number(grade.vertical_radius)));
		put("end");
	}
	put("turnouts");
	for (auto const &[id, turnout] : network.turnouts())
	{
		auto text{std::format("turnout {} type {} hand {} direction {} start {} main {} {} diverging {} {} chainage {} radii {} {} cant {} speeds {} {} velocity {} movable {}", layout::number(id), word(turnout.type), turnout.hand,
		                      turnout.direction, layout::number(turnout.start), layout::number(turnout.main), layout::number(turnout.main_end), layout::number(turnout.diverging), layout::number(turnout.diverging_end),
		                      number(turnout.chainage), number(turnout.main_radius), number(turnout.diverging_radius), number(turnout.cant), number(turnout.speed_main), number(turnout.speed_diverging),
		                      number(turnout.velocity), turnout.movable ? 1 : 0)};
		// the tracks and the legs of the double switches, the slips and the crossings, only where the turnout has them
		if (turnout.second != node_id::none)
			text += std::format(" second {} {} {} {}", layout::number(turnout.second), layout::number(turnout.main2), layout::number(turnout.diverging2), layout::number(turnout.diverging2_end));
		if (turnout.crossing != line_id::none)
			text += std::format(" crossing {} {} {} slip {}", layout::number(turnout.crossing), layout::number(turnout.cross_start), layout::number(turnout.cross_end), layout::number(turnout.diverging2));
		put(text);
	}
	put("end");
	put("anchors");
	for (auto const &line : Project.anchors)
		put(line);
	put("end");
	put("generated");
	for (auto const &node : Project.generated)
		put(std::format("{} {} line {} file {} sum {}", node.kind, word(node.name), layout::number(node.line), word(node.file), hex(node.sum)));
	for (auto const id : Project.hand_changed)
		put(std::format("hand {}", layout::number(id)));
	put("end");
	for (auto const &record : Project.legacy)
	{
		put(std::format("legacy {} {} line {}", record.mark, word(record.file), layout::number(record.line)) + (record.why.empty() ? std::string{} : " why " + record.why));
		for (auto const &line : record.lines)
			put(line.empty() ? std::string{"|"} : "| " + line);
		put("end");
	}
	return text;
}

namespace
{

// the tracks of the turnout there and its, its legs there, the tracks it lays itself between its legs
bool whole_turnout(layout::network const &Network, layout::turnout const &Turnout)
{
	auto const held = [&](node_id const Node) { return Node == node_id::none || Network.find(Node) != nullptr; };
	auto const between = [&](line_id const Track, node_id const From, node_id const To) {
		auto const *track{Network.find(Track)};
		return Track == line_id::none || (track != nullptr && track->start == From && track->end == To);
	};
	auto const tracks{layout::tracks_of(Turnout)};
	if (Network.find(Turnout.main) == nullptr || (Turnout.diverging == line_id::none && Turnout.crossing == line_id::none))
		return false;
	if (false == std::ranges::all_of(tracks, [&](line_id const Track) { return Network.find(Track) != nullptr && Network.find(Track)->turnout == Turnout.id; }))
		return false;
	auto const slip{Turnout.crossing != line_id::none};
	return held(Turnout.start) && held(Turnout.main_end) && held(Turnout.diverging_end) && held(Turnout.second) && held(Turnout.diverging2_end) && held(Turnout.cross_start) &&
	       held(Turnout.cross_end) && between(Turnout.diverging, Turnout.start, slip ? Turnout.cross_end : Turnout.diverging_end) &&
	       between(Turnout.diverging2, slip ? Turnout.cross_start : Turnout.second, Turnout.diverging2_end) && between(Turnout.crossing, Turnout.cross_start, Turnout.cross_end);
}

} // namespace

read_result read(std::string_view const Text)
{
	read_result result;
	document project;
	layout::transaction change{project.network};
	std::map<std::size_t, norms::line_class> classes;
	std::map<line_id, std::size_t> line_classes;
	std::optional<layout::network::handed_out> next;
	section in{section::top};
	std::optional<layout::line> line;
	bool versioned{false};
	std::size_t number{0};
	auto const fail = [&](std::string const &Error) {
		result.error = std::format("line {}: {}", number, Error);
		return result;
	};

	std::size_t at{0};
	while (at <= Text.size())
	{
		auto end{Text.find('\n', at)};
		end = (end == std::string_view::npos ? Text.size() : end);
		auto text{Text.substr(at, end - at)};
		at = end + 1;
		++number;
		if (false == text.empty() && text.back() == '\r')
			text.remove_suffix(1);
		if (number == 1 && text.starts_with("\xEF\xBB\xBF"))
			text.remove_prefix(3);

		// the lines kept as they are come first: the //$ lines, which may be anything, and the sections reserved for later stages
		if (in == section::legacy && false == text.empty() && text.front() == '|')
		{
			project.legacy.back().lines.emplace_back(text.size() > 2 ? text.substr(2) : std::string_view{});
			continue;
		}
		record words{text};
		if (words.empty() || words.peek().front() == '#')
			continue;
		if (false == versioned)
		{
			// the schema first: a newer one isn't read at all, since what it holds can't be told
			if (words.peek() != "eu07proj")
			{
				result.error = "not a project file of the track editor: it doesn't start with eu07proj";
				return result;
			}
			words.next("");
			auto const version{words.whole("the version of the schema")};
			if (words.failed() || version < 1)
				return fail("not a project file of the track editor: " + (words.failed() ? words.error() : std::string{"version 0"}));
			if (version > static_cast<std::uint64_t>(schema_version))
			{
				result.error = std::format("the project file is of version {} of the schema, newer than {} this editor reads; it was written by a newer editor", version, schema_version);
				return result;
			}
			versioned = true;
			continue;
		}
		auto const keyword{words.peek()};
		if (in == section::anchors)
		{
			if (keyword == "end" && words_of(text).size() == 1)
				in = section::top;
			else
				project.anchors.emplace_back(text);
			continue;
		}
		words.next("");
		if (in == section::turnouts)
		{
			if (keyword == "turnout")
			{
				layout::turnout turnout;
				auto const sign = [&](std::string_view const What) {
					auto const text{words.next(What)};
					if (text != "1" && text != "-1" && false == words.failed())
						words.fail(std::format("'{}' isn't 1 or -1 ({})", text, What));
					return text == "-1" ? -1 : 1;
				};
				turnout.id = static_cast<turnout_id>(words.id("the turnout"));
				words.expect("type");
				turnout.type = unword(words.next("the type"));
				words.expect("hand");
				turnout.hand = sign("the hand");
				words.expect("direction");
				turnout.direction = sign("the direction");
				words.expect("start");
				turnout.start = static_cast<node_id>(words.id("the node at the start"));
				words.expect("main");
				turnout.main = static_cast<line_id>(words.id("the main track"));
				turnout.main_end = static_cast<node_id>(words.id("the node at the end of the main track"));
				words.expect("diverging");
				turnout.diverging = static_cast<line_id>(words.id("the diverging track"));
				turnout.diverging_end = static_cast<node_id>(words.id("the node at the end of the diverging track"));
				words.expect("chainage");
				turnout.chainage = words.real("the chainage");
				words.expect("radii");
				turnout.main_radius = words.real("the radius of the main track");
				turnout.diverging_radius = words.real("the radius of the diverging track");
				words.expect("cant");
				turnout.cant = words.real("the cant");
				words.expect("speeds");
				turnout.speed_main = words.real("the speed of the main track");
				turnout.speed_diverging = words.real("the speed of the diverging track");
				words.expect("velocity");
				turnout.velocity = words.real("the velocity");
				words.expect("movable");
				turnout.movable = words.choice("movable", std::array<std::string_view, 2>{"0", "1"}) == 1;
				if (words.peek() == "second")
				{
					words.next("second");
					turnout.second = static_cast<node_id>(words.id("the node of the second switch"));
					turnout.main2 = static_cast<line_id>(words.id("the main track of the second switch"));
					turnout.diverging2 = static_cast<line_id>(words.id("the diverging track of the second switch"));
					turnout.diverging2_end = static_cast<node_id>(words.id("the node at the end of the diverging track of the second switch"));
				}
				if (words.peek() == "crossing")
				{
					words.next("crossing");
					turnout.crossing = static_cast<line_id>(words.id("the crossing track"));
					turnout.cross_start = static_cast<node_id>(words.id("the node at the start of the crossing track"));
					turnout.cross_end = static_cast<node_id>(words.id("the node at the end of the crossing track"));
					words.expect("slip");
					turnout.diverging2 = static_cast<line_id>(words.id("the second slip curve"));
					if (turnout.diverging2 != line_id::none)
						turnout.diverging2_end = turnout.main_end;
				}
				words.finish();
				if (false == words.failed())
				{
					if (turnout.id == turnout_id::none)
						return fail("a turnout numbered 0");
					if (project.network.find(turnout.id) != nullptr)
						return fail(std::format("turnout {} given twice", layout::number(turnout.id)));
					change.add(std::move(turnout));
				}
			}
			else if (keyword == "end")
			{
				words.finish();
				in = section::top;
			}
			else
				words.fail(std::format("'{}' among the turnouts", keyword));
			if (words.failed())
				return fail(words.error());
			continue;
		}
		if (in == section::line)
		{
			if (keyword == "vertex")
			{
				layout::vertex vertex;
				vertex.id = static_cast<vertex_id>(words.id("the vertex"));
				words.expect("at");
				vertex.point.x = words.real("x");
				vertex.point.y = words.real("z");
				words.expect("radius");
				vertex.radius = words.real("the radius");
				words.expect("in");
				vertex.transition_in = words.real("the transition in");
				words.expect("out");
				vertex.transition_out = words.real("the transition out");
				words.expect("cant");
				vertex.cant = words.real("the cant");
				words.finish();
				if (false == words.failed() && vertex.id == vertex_id::none)
					words.fail("a vertex numbered 0");
				line->shape.vertices.push_back(vertex);
			}
			else if (keyword == "grade")
			{
				layout::grade_break grade;
				grade.chainage = words.real("the chainage");
				grade.elevation = words.real("the elevation");
				grade.vertical_radius = words.real("the vertical radius");
				words.finish();
				line->shape.grades.push_back(grade);
			}
			else if (keyword == "end")
			{
				words.finish();
				if (false == words.failed())
				{
					if (project.network.find(line->id) != nullptr)
						return fail(std::format("line {} given twice", layout::number(line->id)));
					change.add(std::move(*line));
					line.reset();
					in = section::top;
				}
			}
			else
				words.fail(std::format("'{}' in a line", keyword));
			if (words.failed())
				return fail(words.error());
			continue;
		}
		if (in == section::generated)
		{
			if (keyword == "track" || keyword == "switch")
			{
				generated_node node;
				node.kind = std::string{keyword};
				node.name = unword(words.next("the name"));
				words.expect("line");
				node.line = static_cast<line_id>(words.id("the line"));
				words.expect("file");
				node.file = unword(words.next("the file"));
				words.expect("sum");
				node.sum = words.whole("the checksum", 16);
				words.finish();
				project.generated.push_back(std::move(node));
			}
			else if (keyword == "hand")
			{
				project.hand_changed.insert(static_cast<line_id>(words.id("the line")));
				words.finish();
			}
			else if (keyword == "end")
			{
				words.finish();
				in = section::top;
			}
			else
				words.fail(std::format("'{}' among the generated nodes", keyword));
			if (words.failed())
				return fail(words.error());
			continue;
		}
		if (in == section::legacy)
		{
			if (keyword != "end")
				return fail(std::format("'{}' among the //$ lines, which start with |", keyword));
			words.finish();
			if (words.failed())
				return fail(words.error());
			in = section::top;
			continue;
		}

		if (keyword == "next")
		{
			layout::network::handed_out last;
			words.expect("node");
			last.node = words.id("the next node") - 1u;
			words.expect("line");
			last.line = words.id("the next line") - 1u;
			words.expect("vertex");
			last.vertex = words.id("the next vertex") - 1u;
			words.expect("turnout");
			last.turnout = words.id("the next turnout") - 1u;
			words.finish();
			next = last;
		}
		else if (keyword == "norms")
		{
			project.network.areas.scenery = words.profile("the profile of the scenery");
			words.finish();
		}
		else if (keyword == "area")
		{
			norms::area area;
			area.name = unword(words.next("the name"));
			area.profile = words.profile("the profile");
			auto const count{words.whole("the count of the points")};
			for (std::uint64_t i = 0; i < count && false == words.failed(); ++i)
			{
				norms::plan_point point;
				point.x = words.real("x");
				point.z = words.real("z");
				area.outline.push_back(point);
			}
			words.finish();
			project.network.areas.areas.push_back(std::move(area));
		}
		else if (keyword == "lineprofile")
		{
			auto name{unword(words.next("the name of the line"))};
			auto const profile{words.profile("the profile")};
			words.finish();
			project.network.areas.lines.emplace_back(std::move(name), profile);
		}
		else if (keyword == "class")
		{
			auto const id{static_cast<std::size_t>(words.whole("the class"))};
			norms::line_class kind;
			words.expect("vmax");
			kind.vmax = words.real("the highest speed");
			words.expect("vmin");
			kind.vmin = words.real("the speed of the slow trains");
			words.expect("traffic");
			kind.kind = static_cast<norms::traffic>(words.choice("traffic", std::array<std::string_view, 3>{"passenger", "mixed", "freight"}));
			words.expect("track");
			kind.track = static_cast<norms::track_kind>(words.choice("track", std::array<std::string_view, 2>{"main", "side"}));
			words.expect("load");
			kind.load = words.real("the load");
			words.expect("features");
			kind.features = static_cast<unsigned>(words.whole("the features"));
			words.finish();
			if (false == words.failed() && false == classes.emplace(id, kind).second)
				return fail(std::format("class {} given twice", id));
		}
		else if (keyword == "node")
		{
			layout::node node;
			node.id = static_cast<node_id>(words.id("the node"));
			words.expect("at");
			node.position.x = words.real("x");
			node.position.y = words.real("the height");
			node.position.z = words.real("z");
			words.expect("heading");
			node.heading.x = words.real("x of the heading");
			node.heading.y = words.real("z of the heading");
			words.expect("curvature");
			node.curvature = words.real("the curvature");
			words.expect("cant");
			node.cant = words.real("the cant");
			words.expect("grade");
			node.grade = words.real("the grade");
			words.expect("ends");
			auto const count{words.whole("the count of the ends")};
			for (std::uint64_t i = 0; i < count && false == words.failed(); ++i)
			{
				layout::end_ref end;
				end.line = static_cast<line_id>(words.id("the line of an end"));
				end.end = words.choice("end", std::array<std::string_view, 2>{"start", "end"}) == 0 ? layout::line_end::start : layout::line_end::end;
				auto const sense{words.next("the sense")};
				if (sense != "1" && sense != "-1" && false == words.failed())
					words.fail(std::format("'{}' isn't a sense, 1 or -1", sense));
				end.sense = (sense == "-1" ? -1 : 1);
				node.ends.push_back(end);
			}
			words.finish();
			if (false == words.failed())
			{
				if (node.id == node_id::none)
					return fail("a node numbered 0");
				if (project.network.find(node.id) != nullptr)
					return fail(std::format("node {} given twice", layout::number(node.id)));
				change.add(std::move(node));
			}
		}
		else if (keyword == "line")
		{
			layout::line laid;
			laid.id = static_cast<line_id>(words.id("the line"));
			words.expect("from");
			laid.start = static_cast<node_id>(words.id("the node at the start"));
			words.expect("to");
			laid.end = static_cast<node_id>(words.id("the node at the end"));
			words.expect("class");
			line_classes[laid.id] = static_cast<std::size_t>(words.whole("the class"));
			words.expect("chainage");
			laid.chainage_start = words.real("the chainage");
			words.expect("transition");
			laid.shape.transition = static_cast<layout::transition_shape>(words.choice("transition", std::array<std::string_view, 3>{"clothoid", "parabola", "bloss"}));
			words.expect("ramp");
			laid.shape.ramp = words.choice("ramp", std::array<std::string_view, 2>{"linear", "bloss"}) == 0 ? layout::ramp_shape::linear : layout::ramp_shape::bloss;
			words.expect("continues");
			laid.start_continues = words.choice("continuation", std::array<std::string_view, 2>{"0", "1"}) == 1;
			laid.end_continues = words.choice("continuation", std::array<std::string_view, 2>{"0", "1"}) == 1;
			words.expect("turnout");
			laid.turnout = static_cast<turnout_id>(words.id("the turnout"));
			words.expect("profile");
			if (words.peek() == "-")
				words.next("the profile");
			else
				laid.profile = words.profile("the profile");
			words.finish();
			if (false == words.failed() && laid.id == line_id::none)
				return fail("a line numbered 0");
			line = std::move(laid);
			in = section::line;
		}
		else if (keyword == "turnouts" || keyword == "anchors" || keyword == "generated")
		{
			words.finish();
			in = keyword == "turnouts" ? section::turnouts : keyword == "anchors" ? section::anchors : section::generated;
		}
		else if (keyword == "legacy")
		{
			legacy_record record;
			auto const mark{words.next("the mark")};
			if (mark.size() != 1 && false == words.failed())
				words.fail(std::format("'{}' isn't the letter of a mark", mark));
			record.mark = mark.empty() ? 'h' : mark.front();
			record.file = unword(words.next("the file"));
			words.expect("line");
			record.line = static_cast<line_id>(words.id("the line"));
			if (false == words.done())
			{
				words.expect("why");
				record.why = words.rest();
			}
			project.legacy.push_back(std::move(record));
			in = section::legacy;
		}
		else
			words.fail(std::format("'{}' isn't a record of version {}", keyword, schema_version));
		if (words.failed())
			return fail(words.error());
	}
	if (false == versioned)
	{
		result.error = "not a project file of the track editor: it's empty";
		return result;
	}
	if (in != section::top)
	{
		result.error = "the file ends inside a section, its end missing";
		return result;
	}

	// what refers to what
	auto &network{project.network};
	for (auto const &[id, laid] : network.lines())
	{
		auto const kind{classes.find(line_classes[id])};
		if (kind == classes.end())
		{
			result.error = std::format("line {}: there's no class {}", layout::number(id), line_classes[id]);
			return result;
		}
		change.change(id).category = kind->second;
		for (auto const end : {layout::line_end::start, layout::line_end::end})
		{
			auto const *at{network.find(laid.node_at(end))};
			auto const held{at == nullptr ? 0 : std::ranges::count_if(at->ends, [&](layout::end_ref const &Ref) { return Ref.line == id && Ref.end == end; })};
			if (held != 1)
			{
				result.error = std::format("line {}: its {} isn't held once by node {}", layout::number(id), end == layout::line_end::start ? "start" : "end", layout::number(laid.node_at(end)));
				return result;
			}
		}
	}
	for (auto const &[id, node] : network.nodes())
		for (auto const &end : node.ends)
			if (auto const *owner{network.find(end.line)}; owner == nullptr || owner->node_at(end.end) != id)
			{
				result.error = std::format("node {}: line {} doesn't end at it", layout::number(id), layout::number(end.line));
				return result;
			}
	// the turnouts and their tracks, each of them of the other; the tracks it lays itself between its legs as written
	for (auto const &[id, turnout] : network.turnouts())
	{
		if (false == whole_turnout(network, turnout))
		{
			result.error = std::format("turnout {}: its tracks or its legs aren't the ones written", layout::number(id));
			return result;
		}
	}
	for (auto const &[id, laid] : network.lines())
	{
		auto const *owner{network.find(laid.turnout)};
		auto const tracks{owner != nullptr ? layout::tracks_of(*owner) : std::vector<line_id>{}};
		if (laid.turnout != turnout_id::none && std::ranges::find(tracks, id) == tracks.end())
		{
			result.error = std::format("line {}: turnout {} hasn't it", layout::number(id), layout::number(laid.turnout));
			return result;
		}
	}
	// the identifiers: none handed out again, whatever the record of them says
	layout::network::handed_out last;
	for (auto const &[id, node] : network.nodes())
		last.node = std::max(last.node, layout::number(id));
	for (auto const &[id, laid] : network.lines())
	{
		last.line = std::max(last.line, layout::number(id));
		for (auto const &vertex : laid.shape.vertices)
			last.vertex = std::max(last.vertex, layout::number(vertex.id));
	}
	for (auto const &[id, turnout] : network.turnouts())
		last.turnout = std::max(last.turnout, layout::number(id));
	if (false == next.has_value())
	{
		result.error = "the next identifiers (next) missing";
		return result;
	}
	if (next->node < last.node || next->line < last.line || next->vertex < last.vertex || next->turnout < last.turnout)
		result.notes.push_back("the next identifiers written are lower than the ones in use, taken past them");
	network.hand_out(*next);
	network.hand_out(last);
	// the lines laid between their nodes as read; the tracks the turnouts lay themselves then, off their main tracks
	for (auto const &[id, laid] : network.lines())
	{
		if (layout::laid_by_turnout(network, id))
			continue;
		auto solved{layout::solve(laid.shape, network.condition(laid, layout::line_end::start), network.condition(laid, layout::line_end::end))};
		for (auto const &error : solved.errors)
			result.notes.push_back(std::format("line {}: {}", layout::number(id), error));
		change.change(id).solved = std::move(solved);
	}
	for (auto const &[id, turnout] : network.turnouts())
	{
		std::string why;
		auto const type{layout::turnout_type_of(turnout.type, &why)};
		if (false == type.has_value())
		{
			result.notes.push_back(std::format("turnout {}: {}", layout::number(id), why.empty() ? "no type " + turnout.type : why));
			continue;
		}
		auto const laid{layout::lay_tracks(network, turnout, *type)};
		for (auto const &error : laid.errors)
			result.notes.push_back(std::format("turnout {}: {}", layout::number(id), error));
		for (auto const &track : laid.tracks)
			change.change(track.line).solved = track.solved;
	}
	result.project = std::move(project);
	return result;
}

std::filesystem::path path_of(std::filesystem::path const &Map)
{
	auto result{Map};
	result.replace_extension(extension);
	return result;
}

read_result load(std::filesystem::path const &Path)
{
	std::ifstream file{Path, std::ios::binary};
	if (false == file.is_open())
	{
		read_result result;
		result.error = "there's no project file " + Path.generic_string();
		return result;
	}
	std::string const text{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
	return read(text);
}

bool save(document const &Project, std::filesystem::path const &Path, std::string &Error)
{
	auto const text{write(Project)};
	auto temporary{Path};
	temporary += ".new";
	{
		std::ofstream file{temporary, std::ios::binary | std::ios::trunc};
		if (false == file.is_open())
		{
			Error = "can't write " + temporary.generic_string();
			return false;
		}
		file.write(text.data(), static_cast<std::streamsize>(text.size()));
		if (false == file.good())
		{
			Error = "can't write " + temporary.generic_string();
			return false;
		}
	}
	std::error_code error;
	std::filesystem::rename(temporary, Path, error);
	if (error)
	{
		Error = "can't put " + temporary.generic_string() + " in the place of the project file: " + error.message();
		std::filesystem::remove(temporary, error);
		return false;
	}
	return true;
}

} // namespace project
