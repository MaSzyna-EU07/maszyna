/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorModelSets.hpp"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <filesystem>
#include <fstream>

editorModelSets EditorModelSets;

// file layout:
//   set <name>
//   <node template, single line>
//   ...
//   endset
namespace
{
namespace fs = std::filesystem;

fs::path sets_path()
{
	fs::path p = user_config_path("eu07_editor_sets.txt");
	return p.empty() ? fs::path("eu07_editor_sets.txt") : p;
}

std::string trimmed(std::string Text)
{
	auto const first = Text.find_first_not_of(" \t");
	if (first == std::string::npos)
		return {};
	auto const last = Text.find_last_not_of(" \t");
	return Text.substr(first, last - first + 1);
}

// the file stores one template per line, so line breaks inside a template (e.g. from export_as_text) become plain separators
std::string single_line(std::string Text)
{
	std::replace_if(Text.begin(), Text.end(), [](char const Character) { return Character == '\r' || Character == '\n' || Character == '\t'; }, ' ');
	return trimmed(Text);
}
} // namespace

bool editorModelSets::load()
{
	m_sets.clear();

	const fs::path path = sets_path();
	std::error_code ec;
	if (!fs::exists(path, ec))
		return false;

	std::ifstream stream(path, std::ios_base::in | std::ios_base::binary);
	if (!stream.is_open())
		return false;

	editor_model_set *current{nullptr};
	std::string line;
	while (std::getline(stream, line))
	{
		line = single_line(line);
		if (line.empty() || line.rfind("//", 0) == 0)
			continue;

		// a set header also closes a set with missing "endset"; templates always start with "node"
		if (line.rfind("set", 0) == 0 && (line.size() == 3 || line[3] == ' '))
		{
			editor_model_set set;
			set.id = m_nextid++;
			set.name = unique_name(line.substr(3));
			m_sets.push_back(std::move(set));
			current = &m_sets.back();
			continue;
		}
		if (line == "endset")
		{
			current = nullptr;
			continue;
		}
		if (current == nullptr)
			continue;
		current->labels.push_back(label(line));
		current->templates.push_back(std::move(line));
	}
	return true;
}

bool editorModelSets::save() const
{
	const fs::path path = sets_path();
	std::error_code ec;
	fs::create_directories(path.parent_path(), ec);

	std::ofstream stream(path, std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);
	if (!stream.is_open())
	{
		ErrorLog("failed to save editor model sets");
		return false;
	}

	stream << "// MaSzyna editor: user-defined model sets\n";
	for (auto const &set : m_sets)
	{
		stream << "\nset " << set.name << "\n";
		for (auto const &entry : set.templates)
			stream << entry << "\n";
		stream << "endset\n";
	}
	return true;
}

editor_model_set const *editorModelSets::find(int const Id) const
{
	auto const lookup = std::find_if(m_sets.begin(), m_sets.end(), [Id](auto const &Set) { return Set.id == Id; });
	return lookup != m_sets.end() ? &(*lookup) : nullptr;
}

editor_model_set *editorModelSets::find_(int const Id)
{
	auto const lookup = std::find_if(m_sets.begin(), m_sets.end(), [Id](auto const &Set) { return Set.id == Id; });
	return lookup != m_sets.end() ? &(*lookup) : nullptr;
}

int editorModelSets::create(std::string const &Name, std::vector<std::string> const &Templates)
{
	editor_model_set set;
	set.id = m_nextid++;
	set.name = unique_name(Name);
	for (auto const &entry : Templates)
	{
		auto line = single_line(entry);
		if (line.empty())
			continue;
		set.labels.push_back(label(line));
		set.templates.push_back(std::move(line));
	}
	m_sets.push_back(std::move(set));
	save();
	return m_sets.back().id;
}

void editorModelSets::remove(int const Id)
{
	auto const lookup = std::find_if(m_sets.begin(), m_sets.end(), [Id](auto const &Set) { return Set.id == Id; });
	if (lookup == m_sets.end())
		return;
	m_sets.erase(lookup);
	save();
}

void editorModelSets::rename(int const Id, std::string const &Name)
{
	auto *set = find_(Id);
	if (set == nullptr)
		return;
	auto const name = unique_name(Name, Id);
	if (name == set->name)
		return;
	set->name = name;
	save();
}

void editorModelSets::add(int const Id, std::vector<std::string> const &Templates)
{
	auto *set = find_(Id);
	if (set == nullptr)
		return;
	auto const count{set->templates.size()};
	for (auto const &entry : Templates)
	{
		auto line = single_line(entry);
		if (line.empty())
			continue;
		set->labels.push_back(label(line));
		set->templates.push_back(std::move(line));
	}
	if (set->templates.size() != count)
		save();
}

void editorModelSets::erase(int const Id, std::size_t const Index)
{
	auto *set = find_(Id);
	if (set == nullptr || Index >= set->templates.size())
		return;
	set->templates.erase(set->templates.begin() + Index);
	set->labels.erase(set->labels.begin() + Index);
	save();
}

void editorModelSets::clear(int const Id)
{
	auto *set = find_(Id);
	if (set == nullptr || set->templates.empty())
		return;
	set->templates.clear();
	set->labels.clear();
	save();
}

std::string editorModelSets::unique_name(std::string const &Base, int const IgnoreId) const
{
	auto base = single_line(Base);
	if (base.empty())
		base = "New set";
	auto const taken = [&](std::string const &Name) {
		return std::any_of(m_sets.begin(), m_sets.end(), [&](auto const &Set) { return Set.id != IgnoreId && Set.name == Name; });
	};
	auto name = base;
	for (int suffix = 2; taken(name); ++suffix)
		name = base + " " + std::to_string(suffix);
	return name;
}

std::string editorModelSets::label(std::string const &Template)
{
	cParser tokenizer(Template);
	tokenizer.getTokens(9, false); // node, ranges, name, type, position, rotation
	auto model{tokenizer.getToken<std::string>(false)};
	auto texture{tokenizer.getToken<std::string>(false)};
	replace_slashes(model);
	erase_extension(model);
	replace_slashes(texture);
	return texture == "none" || texture.empty() ? model : model + " (" + texture + ")";
}
