/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTrackPresets.hpp"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

editorTrackPresets EditorTrackPresets;

// file layout:
//   preset <name>
//   from <track name>
//   groups <group key> <group key>...
//   value <property key> <value, the rest of the line>
//   ...
//   endpreset
namespace
{
namespace fs = std::filesystem;

fs::path presets_path()
{
	fs::path p = user_config_path("eu07_editor_track_presets.txt");
	return p.empty() ? fs::path("eu07_editor_track_presets.txt") : p;
}

std::string trimmed(std::string Text)
{
	auto const first = Text.find_first_not_of(" \t");
	if (first == std::string::npos)
		return {};
	auto const last = Text.find_last_not_of(" \t");
	return Text.substr(first, last - first + 1);
}

// one value per line in the file
std::string single_line(std::string Text)
{
	std::replace_if(Text.begin(), Text.end(), [](char const Character) { return Character == '\r' || Character == '\n' || Character == '\t'; }, ' ');
	return trimmed(Text);
}

// the line past its first word, when it starts with the word
bool keyword(std::string const &Line, char const *Word, std::string &Rest)
{
	auto const length{std::char_traits<char>::length(Word)};
	if (Line.compare(0, length, Word) != 0 || (Line.size() > length && Line[length] != ' '))
		return false;
	Rest = Line.size() > length ? trimmed(Line.substr(length + 1)) : std::string{};
	return true;
}
} // namespace

bool editorTrackPresets::load()
{
	m_presets.clear();

	const fs::path path = presets_path();
	std::error_code ec;
	if (!fs::exists(path, ec))
		return false;

	std::ifstream stream(path, std::ios_base::in | std::ios_base::binary);
	if (!stream.is_open())
		return false;

	editor_track_preset *current{nullptr};
	std::string line;
	std::string rest;
	while (std::getline(stream, line))
	{
		line = single_line(line);
		if (line.empty() || line.rfind("//", 0) == 0)
			continue;

		// a preset header also closes a preset with missing "endpreset"
		if (keyword(line, "preset", rest))
		{
			editor_track_preset preset;
			preset.id = m_nextid++;
			preset.name = unique_name(rest);
			m_presets.push_back(std::move(preset));
			current = &m_presets.back();
			continue;
		}
		if (line == "endpreset")
		{
			current = nullptr;
			continue;
		}
		if (current == nullptr)
			continue;
		if (keyword(line, "from", rest))
			current->from = rest;
		else if (keyword(line, "groups", rest))
		{
			std::istringstream words{rest};
			current->groups.clear();
			for (std::string group; words >> group;)
				current->groups.push_back(group);
		}
		else if (keyword(line, "value", rest) && false == rest.empty())
		{
			auto const split{rest.find(' ')};
			if (split == std::string::npos)
				current->values.emplace_back(rest, std::string{});
			else
				current->values.emplace_back(rest.substr(0, split), trimmed(rest.substr(split + 1)));
		}
	}
	return true;
}

bool editorTrackPresets::save() const
{
	const fs::path path = presets_path();
	std::error_code ec;
	fs::create_directories(path.parent_path(), ec);

	std::ofstream stream(path, std::ios_base::out | std::ios_base::trunc | std::ios_base::binary);
	if (!stream.is_open())
	{
		ErrorLog("failed to save editor track presets");
		return false;
	}

	stream << "// MaSzyna editor: user-defined presets of the track properties\n";
	for (auto const &preset : m_presets)
	{
		stream << "\npreset " << preset.name << "\n";
		if (false == preset.from.empty())
			stream << "from " << preset.from << "\n";
		stream << "groups";
		for (auto const &group : preset.groups)
			stream << ' ' << group;
		stream << "\n";
		for (auto const &[key, value] : preset.values)
			stream << "value " << key << ' ' << single_line(value) << "\n";
		stream << "endpreset\n";
	}
	return true;
}

editor_track_preset const *editorTrackPresets::find(int const Id) const
{
	auto const lookup = std::find_if(m_presets.begin(), m_presets.end(), [Id](auto const &Preset) { return Preset.id == Id; });
	return lookup != m_presets.end() ? &(*lookup) : nullptr;
}

editor_track_preset *editorTrackPresets::find_(int const Id)
{
	auto const lookup = std::find_if(m_presets.begin(), m_presets.end(), [Id](auto const &Preset) { return Preset.id == Id; });
	return lookup != m_presets.end() ? &(*lookup) : nullptr;
}

int editorTrackPresets::create(std::string const &Name, std::string const &From, std::vector<std::pair<std::string, std::string>> const &Values, std::vector<std::string> const &Groups)
{
	editor_track_preset preset;
	preset.id = m_nextid++;
	preset.name = unique_name(Name);
	preset.from = single_line(From);
	preset.values = Values;
	preset.groups = Groups;
	m_presets.push_back(std::move(preset));
	save();
	return m_presets.back().id;
}

void editorTrackPresets::remove(int const Id)
{
	auto const lookup = std::find_if(m_presets.begin(), m_presets.end(), [Id](auto const &Preset) { return Preset.id == Id; });
	if (lookup == m_presets.end())
		return;
	m_presets.erase(lookup);
	save();
}

void editorTrackPresets::rename(int const Id, std::string const &Name)
{
	auto *preset = find_(Id);
	if (preset == nullptr)
		return;
	auto const name = unique_name(Name, Id);
	if (name == preset->name)
		return;
	preset->name = name;
	save();
}

void editorTrackPresets::update(int const Id, std::string const &From, std::vector<std::pair<std::string, std::string>> const &Values)
{
	auto *preset = find_(Id);
	if (preset == nullptr)
		return;
	preset->from = single_line(From);
	preset->values = Values;
	save();
}

void editorTrackPresets::set_groups(int const Id, std::vector<std::string> const &Groups)
{
	auto *preset = find_(Id);
	if (preset == nullptr || preset->groups == Groups)
		return;
	preset->groups = Groups;
	save();
}

std::string editorTrackPresets::unique_name(std::string const &Base, int const IgnoreId) const
{
	auto base = single_line(Base);
	if (base.empty())
		base = "New preset";
	auto const taken = [&](std::string const &Name) {
		return std::any_of(m_presets.begin(), m_presets.end(), [&](auto const &Preset) { return Preset.id != IgnoreId && Preset.name == Name; });
	};
	auto name = base;
	for (int suffix = 2; taken(name); ++suffix)
		name = base + " " + std::to_string(suffix);
	return name;
}
