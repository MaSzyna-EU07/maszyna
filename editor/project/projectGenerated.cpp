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
	std::erase_if(Project.generated, [&](generated_node const &Node) { return Node.kind == "switch" && Node.line == main; });
	generated_node node;
	node.kind = "switch";
	node.name = Turnout.name;
	node.line = main;
	node.file = File;
	node.sum = checksum(Turnout.text);
	Project.generated.push_back(std::move(node));
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

} // namespace project
