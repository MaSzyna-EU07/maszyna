/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/projectMigration.hpp"
#include "editor/editorAlignment.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorRecords.hpp"
#include "editor/project/sceneryText.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <iterator>
#include <optional>

namespace project
{

namespace
{

using geometry::plan_of;
using layout::node_id;

double constexpr kLength{0.005}; // m: the domain lays a design as the editor laid it, the lengths within this
double constexpr kEnds{0.05}; // m: a grade line runs along a line, its ends and its length within this
double constexpr kJoint{0.0005}; // m: the ends of two lines this close are one node
double constexpr kKink{1e-3}; // rad: the ends of two lines turning more than this aren't joined
double constexpr kHeading{1e-9}; // rad: the turn at a node the domain takes for none (layoutNetwork.cpp)

// the marks of the //$ lines taken into the project, in their order there; the camera, the orthophoto and the area aren't the tracks
constexpr std::string_view kept_marks{"hprktwbs"};
constexpr std::string_view passed_marks{"coz"};

// the marks some kinds had before, read by the word the editor wrote in their lines or the comment before them (scenelayers.cpp)
struct moved_mark
{
	char before;
	char now;
	std::string_view word;
	std::string_view note;
};
constexpr std::array<moved_mark, 3> moved_marks{{
    {'a', 'z', "point", "# the area the editor downloads"},
    {'n', 'w', "gradeline", "# the tracks sharing a grade line"},
    {'f', 't', "fitted", "# the tracks whose grade line was fitted"},
}};

std::string_view first_word(std::string_view const Text)
{
	return Text.substr(0, Text.find_first_of(" \t"));
}

// other tools put lines of their own after some marks, Rainsted its includes after //$r: the editor's lines are told by their first
// words, as the editor tells them (scenelayers.cpp, editor_line())
bool editor_line(char const Mark, std::string_view const Text)
{
	static std::array<std::pair<char, std::array<std::string_view, 11>>, 7> const words{{
	    {'h', {"#", "alignment", "source", "ends", "norms", "pieces", "kept", "vertex", "click", "element", "end"}},
	    {'r', {"#", "switch", "slip", "crossing", "double", "connection"}},
	    {'s', {"#", "station"}},
	    {'k', {"#", "mark", "via"}},
	    {'z', {"#", "point"}},
	    {'w', {"#", "gradeline"}},
	    {'t', {"#", "fitted"}},
	}};
	auto const lookup{std::ranges::find_if(words, [&](auto const &Entry) { return Entry.first == Mark; })};
	if (lookup == words.end())
		return true;
	auto const word{first_word(Text)};
	return std::ranges::any_of(lookup->second, [&](std::string_view const Known) { return false == Known.empty() && Known == word; });
}

// the records of a kind written between a first line and an end line: //$h alignment ... end, //$p profile ... end
std::vector<std::vector<std::string>> records_of(std::vector<std::string> const &Lines, std::string_view const First)
{
	std::vector<std::vector<std::string>> result;
	bool inside{false};
	for (auto const &line : Lines)
	{
		auto const word{first_word(line)};
		if (word == First)
		{
			result.emplace_back();
			inside = true;
		}
		if (false == inside)
			continue;
		result.back().push_back(line);
		if (word == "end")
			inside = false;
	}
	return result;
}

struct alignment_candidate
{
	std::size_t legacy{0}; // the record kept as it was
	std::optional<alignment_store::record> entry;
	alignment::result result;
	std::string why; // it doesn't become a line
	// a grade line along it
	std::optional<std::size_t> profile;
	std::vector<layout::grade_break> grades;
	double start_height{0.0};
	double end_height{0.0};
};

struct profile_candidate
{
	std::size_t legacy{0};
	std::optional<editor_records::grade_profile> entry;
	std::string why;
};

// what in the design the domain hasn't a place for, or lays otherwise; empty if nothing
std::string unfit(alignment_candidate const &Candidate)
{
	if (false == Candidate.entry.has_value())
		return "a record which can't be read";
	auto const &entry{*Candidate.entry};
	if (false == Candidate.result.valid)
		return "a design the editor can't lay" + (Candidate.result.errors.empty() ? std::string{} : ": " + Candidate.result.errors.front());
	if (false == entry.whole())
		return "a design kept for a part of its line";
	for (auto const &vertex : entry.design.vertices)
	{
		if (vertex.compound)
			return "a compound curve";
		if (vertex.spirals)
			return "two transition curves with no arc between them";
		if (vertex.own_ramps())
			return "cant ramps of its own, not along the transitions";
		if (vertex.reverse_turn)
			return "a curve turning the longer way";
	}
	if (entry.design.start_held || entry.design.end_held)
		return "a curve going on from the diverging track of a switch";
	if ((entry.design.start_curved && entry.design.start_radius <= 0.0) || (entry.design.end_curved && entry.design.end_radius <= 0.0))
		return "a curve going on from an adjoining curve of no radius";
	if (Candidate.result.vertices.size() != entry.design.vertices.size())
		return "a design whose vertices don't match its curves";
	return {};
}

// the line the design is in the domain, laid between its ends as nodes would hold them
struct laid_line
{
	layout::line_shape shape;
	layout::end_condition start;
	layout::end_condition end;
	layout::solved_line solved;
};

laid_line lay(alignment_candidate const &Candidate, bool const Graded)
{
	auto const &design{Candidate.entry->design};
	laid_line result;
	for (std::size_t i = 0; i < design.vertices.size(); ++i)
	{
		auto const &source{design.vertices[i]};
		layout::vertex vertex;
		vertex.point = Candidate.result.vertices[i];
		vertex.radius = source.radius;
		vertex.transition_in = source.transition_in;
		vertex.transition_out = source.transition_out;
		vertex.cant = source.cant;
		result.shape.vertices.push_back(vertex);
	}
	result.shape.transition = design.shape == alignment::transition_shape::clothoid ? layout::transition_shape::clothoid : layout::transition_shape::cubic_parabola;
	result.shape.ramp = layout::ramp_shape::linear;
	result.start.position = design.start;
	result.end.position = design.end;
	if (Graded)
	{
		result.shape.grades = Candidate.grades;
		result.start.position.y = Candidate.start_height;
		result.end.position.y = Candidate.end_height;
	}
	// a curve going on from the adjoining track: the curvature and the cant of it as the line runs, the side it turns to giving the sign
	if (design.start_curved)
	{
		result.start.continues = true;
		result.start.curvature = design.start_turn / design.start_radius;
		result.start.cant = design.start_cant * design.start_turn;
	}
	if (design.end_curved)
	{
		result.end.continues = true;
		result.end.curvature = design.end_turn / design.end_radius;
		result.end.cant = design.end_cant * design.end_turn;
	}
	result.solved = layout::solve(result.shape, result.start, result.end);
	return result;
}

// why the line laid isn't the design, empty if it is
std::string differs(alignment_candidate const &Candidate, laid_line const &Line)
{
	if (false == Line.solved.valid())
		return "the domain can't lay it" + (Line.solved.errors.empty() ? std::string{} : ": " + Line.solved.errors.front());
	auto const difference{Line.solved.length - Candidate.result.length};
	if (std::abs(difference) > kLength)
		return std::format("the domain lays it {:.1f} mm {}", std::abs(difference) * 1000.0, difference > 0.0 ? "longer" : "shorter");
	return {};
}

// the grade line along the design: its breaks and the heights of its ends, or why it isn't one
std::string grade_line(alignment_candidate &Candidate, editor_records::grade_profile const &Profile)
{
	auto const &design{Candidate.entry->design};
	auto const close_by = [](glm::dvec3 const &A, glm::dvec3 const &B) { return glm::distance(plan_of(A), plan_of(B)) <= kEnds; };
	bool reversed{false};
	if (close_by(Profile.start, design.start) && close_by(Profile.end, design.end))
		reversed = false;
	else if (close_by(Profile.start, design.end) && close_by(Profile.end, design.start))
		reversed = true;
	else
		return "-";
	if (std::abs(Profile.length - Candidate.result.length) > kEnds)
		return "-";
	auto const &points{Profile.line.points};
	if (points.size() < 2 || std::abs(points.front().chainage) > kEnds || std::abs(points.back().chainage - Profile.length) > kEnds)
		return "a grade line which doesn't reach the ends of its line";
	std::vector<layout::grade_break> grades;
	for (std::size_t i = 1; i + 1 < points.size(); ++i)
	{
		auto const &point{points[i]};
		if (point.radius <= 0.0)
			return std::format("a break of the grade with no vertical curve at {:.1f} m", point.chainage);
		grades.push_back({reversed ? Profile.length - point.chainage : point.chainage, point.elevation, point.radius});
	}
	if (reversed)
		std::ranges::reverse(grades);
	Candidate.grades = std::move(grades);
	Candidate.start_height = reversed ? points.back().elevation : points.front().elevation;
	Candidate.end_height = reversed ? points.front().elevation : points.back().elevation;
	return {};
}

double angle(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return std::atan2(std::abs(A.x * B.y - A.y * B.x), glm::dot(A, B));
}

// the free ends of the lines at one place joined at a node, where they run on from one another: the free nodes swept along x, each
// pair close enough looked at once
void join_ends(layout::network &Network, std::vector<std::string> &Notes)
{
	std::vector<std::pair<double, node_id>> loose;
	for (auto const &[id, node] : Network.nodes())
		if (node.ends.size() == 1)
			loose.emplace_back(node.position.x, id);
	std::ranges::sort(loose);
	for (std::size_t i = 0; i < loose.size(); ++i)
		for (std::size_t j = i + 1; j < loose.size() && loose[j].first - loose[i].first <= kJoint; ++j)
		{
			auto const kept{std::min(loose[i].second, loose[j].second)};
			auto const gone{std::max(loose[i].second, loose[j].second)};
			auto const *a{Network.find(kept)};
			auto const *b{Network.find(gone)};
			// each node joined once: one which holds two ends now, or is gone, isn't loose any more
			if (a == nullptr || b == nullptr || a->ends.size() != 1 || b->ends.size() != 1 || b->ends.front().line == a->ends.front().line || glm::distance(a->position, b->position) > kJoint)
				continue;
			auto const along{glm::dot(a->heading, b->heading) >= 0.0};
			auto moved{b->ends.front()};
			moved.sense = along ? moved.sense : -moved.sense;
			auto const turn{angle(a->heading, along ? b->heading : -b->heading)};
			if (moved.side() == a->ends.front().side() || turn > kKink)
				continue;
			if (turn > kHeading)
				Notes.push_back(std::format("node {}: the ends of lines {} and {} joined there turn by {:.2g} rad", layout::number(kept), layout::number(a->ends.front().line), layout::number(moved.line), turn));
			layout::transaction change{Network};
			change.change(kept).ends.push_back(moved);
			auto &line{change.change(moved.line)};
			(moved.end == layout::line_end::start ? line.start : line.end) = kept;
			change.erase(gone);
			layout::settle(change);
		}
}

} // namespace

std::map<char, std::vector<std::string>> marked_lines(std::string_view const Text)
{
	std::map<char, std::vector<std::string>> result;
	std::size_t at{0};
	while (at < Text.size())
	{
		auto end{Text.find('\n', at)};
		end = (end == std::string_view::npos ? Text.size() : end);
		auto line{Text.substr(at, end - at)};
		at = end + 1;
		auto const first{line.find_first_not_of(" \t")};
		if (first == std::string_view::npos)
			continue;
		line.remove_prefix(first);
		while (false == line.empty() && (line.back() == '\r' || line.back() == '\n'))
			line.remove_suffix(1);
		if (line.size() < 4 || false == line.starts_with("//$") || (line.size() > 4 && line[4] != ' ' && line[4] != '\t'))
			continue;
		auto mark{line[3]};
		auto text{line.substr(4)};
		auto const begin{text.find_first_not_of(" \t")};
		text = (begin == std::string_view::npos ? std::string_view{} : text.substr(begin));
		auto const moved{std::ranges::find_if(moved_marks, [&](moved_mark const &Moved) { return Moved.before == mark; })};
		if (moved != moved_marks.end())
			mark = (first_word(text) == moved->word || text.starts_with(moved->note)) ? moved->now : '\0';
		else if (false == editor_line(mark, text))
			mark = '\0';
		if (mark != '\0')
			result[mark].emplace_back(text);
	}
	return result;
}

migration migrate(std::vector<marked_file> const &Files, norms::area_map const &Areas)
{
	migration result;
	auto &project{result.project};
	project.network.areas = Areas;
	std::vector<alignment_candidate> alignments;
	std::vector<profile_candidate> profiles;

	// the records kept as they were, each kind of each file; the alignments and the grade lines one by one
	for (auto const &file : Files)
	{
		for (auto const mark : kept_marks)
		{
			auto const found{file.lines.find(mark)};
			if (found == file.lines.end() || found->second.empty())
				continue;
			auto const &lines{found->second};
			if (mark == 'h')
				for (auto &record : records_of(lines, "alignment"))
				{
					alignment_candidate candidate;
					candidate.legacy = project.legacy.size();
					auto entries{editor_records::read_alignments(record, file.context, null_handle)};
					if (false == entries.empty())
					{
						candidate.entry = std::move(entries.front());
						candidate.result = alignment::compute(candidate.entry->design);
					}
					project.legacy.push_back({'h', file.file, layout::line_id::none, {}, std::move(record)});
					alignments.push_back(std::move(candidate));
				}
			else if (mark == 'p')
				for (auto &record : records_of(lines, "profile"))
				{
					profile_candidate candidate;
					candidate.legacy = project.legacy.size();
					auto entries{editor_records::read_profiles(record, file.context, null_handle)};
					if (false == entries.empty())
						candidate.entry = std::move(entries.front());
					else
						candidate.why = "a record which can't be read";
					project.legacy.push_back({'p', file.file, layout::line_id::none, {}, std::move(record)});
					profiles.push_back(std::move(candidate));
				}
			else
				project.legacy.push_back({mark, file.file, layout::line_id::none, {}, lines});
		}
		for (auto const &[mark, lines] : file.lines)
		{
			if (passed_marks.find(mark) != std::string_view::npos)
				result.notes.push_back(std::format("{}: the //${} lines are the editor's own, not the tracks: left in the scenery file", file.file, mark));
			else if (kept_marks.find(mark) == std::string_view::npos)
				result.notes.push_back(std::format("{}: the //${} lines aren't known: left in the scenery file", file.file, mark));
		}
	}

	// what of the designs the domain lays as the editor laid them
	for (auto &candidate : alignments)
		candidate.why = unfit(candidate);
	// the grade lines along them
	for (std::size_t p = 0; p < profiles.size(); ++p)
	{
		auto &profile{profiles[p]};
		if (false == profile.entry.has_value())
			continue;
		for (auto &candidate : alignments)
		{
			if (false == candidate.why.empty() || candidate.profile.has_value())
				continue;
			auto const why{grade_line(candidate, *profile.entry)};
			if (why == "-")
				continue;
			if (why.empty())
				candidate.profile = p;
			else
				profile.why = why;
			break;
		}
		if (profile.why.empty() && std::ranges::none_of(alignments, [&](alignment_candidate const &Candidate) { return Candidate.profile == p; }))
			profile.why = "along no line of an alignment taken into the project";
	}

	// the lines and their nodes
	for (auto &candidate : alignments)
	{
		if (false == candidate.why.empty())
			continue;
		auto line{lay(candidate, candidate.profile.has_value())};
		auto why{differs(candidate, line)};
		if (false == why.empty() && candidate.profile.has_value())
		{
			// laid with no grade line of its own then, the grade line kept as it was
			profiles[*candidate.profile].why = "the line can't be laid with it: " + why;
			candidate.profile.reset();
			line = lay(candidate, false);
			why = differs(candidate, line);
		}
		if (false == why.empty())
		{
			candidate.why = why;
			continue;
		}
		auto &network{project.network};
		layout::transaction change{network};
		auto const id{network.new_line()};
		layout::node first;
		first.id = network.new_node();
		first.position = line.start.position;
		first.heading = line.solved.start.heading;
		first.curvature = line.start.curvature;
		first.cant = line.start.cant;
		first.ends.push_back({id, layout::line_end::start, 1});
		layout::node last;
		last.id = network.new_node();
		last.position = line.end.position;
		last.heading = line.solved.end.heading;
		last.curvature = line.end.curvature;
		last.cant = line.end.cant;
		last.ends.push_back({id, layout::line_end::end, 1});
		layout::line laid;
		laid.id = id;
		laid.start = first.id;
		laid.end = last.id;
		laid.shape = line.shape;
		for (auto &vertex : laid.shape.vertices)
			vertex.id = network.new_vertex();
		laid.start_continues = line.start.continues;
		laid.end_continues = line.end.continues;
		laid.chainage_start = candidate.entry->origin;
		laid.category.vmax = candidate.entry->design.speed;
		change.add(std::move(first));
		change.add(std::move(last));
		change.add(std::move(laid));
		layout::settle(change);
		project.legacy[candidate.legacy].line = id;
		if (candidate.profile.has_value())
			project.legacy[profiles[*candidate.profile].legacy].line = id;
	}
	join_ends(project.network, result.notes);

	for (auto const &candidate : alignments)
		if (false == candidate.why.empty())
		{
			project.legacy[candidate.legacy].why = candidate.why;
			result.notes.push_back(std::format("{}: an alignment kept as it was: {}", project.legacy[candidate.legacy].file, candidate.why));
		}
	for (auto const &profile : profiles)
		if (false == profile.why.empty())
		{
			project.legacy[profile.legacy].why = profile.why;
			result.notes.push_back(std::format("{}: a grade line kept as it was: {}", project.legacy[profile.legacy].file, profile.why));
		}
	return result;
}

std::vector<marked_file> marked_files(std::filesystem::path const &Folder, std::string const &Map)
{
	std::vector<marked_file> result;
	scenery_text::read_options options;
	options.map = Map;
	options.keep = [](std::string_view, std::string_view) { return false; };
	auto const scenery{scenery_text::read(Folder, Map, options)};
	for (auto const &file : scenery.files)
	{
		if (file.missing || file.skipped || std::ranges::any_of(result, [&](marked_file const &Marked) { return scenery_text::same_file(Marked.file, file.name); }))
			continue;
		// the templates (*.inc) are a part of the layer including them, their own lines are the descriptions (//$e)
		if (scenery_text::lowered(file.name).ends_with(".inc"))
			continue;
		auto const text{scenery_text::file_text(Folder / file.name)};
		auto lines{marked_lines(text.value_or(std::string{}))};
		if (lines.empty())
			continue;
		result.push_back({file.name, file.context, std::move(lines)});
	}
	return result;
}

opened open_project(std::filesystem::path const &Folder, std::string const &Map, norms::area_map const &Areas)
{
	opened result;
	auto const path{path_of(Folder / Map)};
	if (std::error_code error; std::filesystem::exists(path, error))
	{
		result.project = load(path);
		return result;
	}
	auto made{migrate(marked_files(Folder, Map), Areas)};
	result.notes = std::move(made.notes);
	// no //$ lines: no file is written next to the map, which would be one more stray file in a working copy of the data. The project,
	// empty, is written as the editor saves the scenery once it has something
	if (made.project.network.lines().empty() && made.project.network.nodes().empty())
	{
		result.project.project = std::move(made.project);
		return result;
	}
	std::string error;
	if (false == save(made.project, path, error))
	{
		result.project.error = error;
		return result;
	}
	result.migrated = true;
	// as it's read from now on
	result.project = load(path);
	return result;
}

} // namespace project
