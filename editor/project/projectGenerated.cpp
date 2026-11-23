/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/project/projectGenerated.hpp"
#include "editor/project/sceneryText.hpp"

#include <algorithm>
#include <cstdlib>
#include <format>
#include <map>

namespace project
{

std::uint64_t checksum(std::string_view const Text)
{
	auto const text{scenery_text::joined(scenery_text::split(Text))};
	std::uint64_t result{14695981039346656037ull};
	for (auto const c : text)
	{
		result ^= static_cast<unsigned char>(c);
		result *= 1099511628211ull;
	}
	return result;
}

void record_generated(document &Project, layout::generated_line const &Line, std::string const &File)
{
	std::erase_if(Project.generated, [&](generated_node const &Node) { return Node.kind == "track" && Node.line == Line.line; });
	for (auto const &piece : Line.pieces)
	{
		generated_node node;
		node.name = piece.name;
		node.line = Line.line;
		node.file = File;
		node.sum = checksum(piece.text);
		Project.generated.push_back(std::move(node));
	}
	// written again by the editor: the author decided
	Project.hand_changed.erase(Line.line);
}

void record_generated(document &Project, layout::generated_turnout const &Turnout, std::string const &File)
{
	auto const *turnout{Project.network.find(Turnout.turnout)};
	if (turnout == nullptr)
		return;
	auto const main{turnout->main};
	std::erase_if(Project.generated, [&](generated_node const &Node) { return Node.line == main; });
	// each track of the simulator it's made of, by its main track: the switch tracks, and the plain tracks of a double switch, a slip or a
	// crossing piece by piece
	auto const keep = [&](std::string const &Kind, std::string const &Name, std::string const &Text) {
		generated_node node;
		node.kind = Kind;
		node.name = Name;
		node.line = main;
		node.file = File;
		node.sum = checksum(Text);
		Project.generated.push_back(std::move(node));
	};
	if (Turnout.parts.empty())
		keep("switch", Turnout.name, Turnout.text);
	for (auto const &part : Turnout.parts)
	{
		if (part.is_switch)
		{
			keep("switch", part.name, part.text);
			continue;
		}
		for (auto const &piece : part.pieces[0])
			keep("track", piece.name, piece.text);
	}
	Project.hand_changed.erase(main);
}

std::vector<hand_change> find_hand_changes(document const &Project, std::function<std::optional<std::string>(std::string const &)> const &Files)
{
	std::vector<hand_change> result;
	std::map<std::string, std::optional<std::string>> texts;
	for (auto const &node : Project.generated)
	{
		auto found{texts.find(node.file)};
		if (found == texts.end())
			found = texts.emplace(node.file, Files(node.file)).first;
		auto const &text{found->second};
		auto const now{text.has_value() ? scenery_text::node_text(*text, node.name) : std::nullopt};
		if (now.has_value() && checksum(*now) == node.sum)
			continue;
		result.push_back({node.line, node.name, node.file, false == now.has_value()});
	}
	return result;
}

std::set<layout::line_id> mark_hand_changes(document &Project, std::vector<hand_change> const &Changes)
{
	std::set<layout::line_id> result;
	for (auto const &change : Changes)
		if (Project.hand_changed.insert(change.line).second)
			result.insert(change.line);
	return result;
}

bool may_overwrite(document const &Project, layout::line_id const Line)
{
	return false == Project.hand_changed.contains(Line);
}

std::optional<std::string> moved_entry(std::string_view const Text, glm::dvec3 const &Shift)
{
	std::vector<std::string> words;
	for (auto const &token : scenery_text::split(Text))
		words.emplace_back(token.text);
	// node <range> <range> <name> track <normal|switch> <length> <surface> <paths> ... endtrack; the surface of 13 words with the materials of
	// a track seen (vis), of 7 of one not (unvis)
	if (words.size() < 14 || words[0] != "node" || words[4] != "track" || (words[5] != "normal" && words[5] != "switch"))
		return std::nullopt;
	auto const surface{words[13] == "vis" ? std::size_t{13} : words[13] == "unvis" ? std::size_t{7} : std::size_t{0}};
	if (surface == 0)
		return std::nullopt;
	// a path: the start and its roll, two control vectors, the end and its roll, the radius
	std::size_t constexpr kPath{15};
	auto const paths{words[5] == "switch" ? std::size_t{2} : std::size_t{1}};
	auto const first{6 + 1 + surface};
	if (words.size() < first + paths * kPath)
		return std::nullopt;
	// as the generator writes the numbers: the shortest form which reads back to the same double
	auto const number = [](double const Value) { return std::format("{}", Value); };
	for (std::size_t path = 0; path < paths; ++path)
		for (auto const point : {std::size_t{0}, std::size_t{10}})
			for (std::size_t axis = 0; axis < 3; ++axis)
			{
				auto &word{words[first + path * kPath + point + axis]};
				char *end{nullptr};
				auto const value{std::strtod(word.c_str(), &end)};
				if (end == word.c_str() || *end != '\0')
					return std::nullopt;
				word = number(value - Shift[static_cast<glm::length_t>(axis)]);
			}
	std::string result;
	for (auto const &word : words)
		result += (result.empty() ? "" : " ") + word;
	return result + "\n";
}

} // namespace project
