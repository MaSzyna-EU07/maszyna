/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorSwitchDrives.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

namespace switch_drives
{

std::string game_name(std::string Name)
{
	std::ranges::replace(Name, '\\', '/');
	std::ranges::transform(Name, Name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Name;
}

standing_switches::standing_switches(finder Find, lister Names) : m_find{std::move(Find)}, m_names{std::move(Names)} {}

bool standing_switches::stands(std::string const &Name)
{
	if (auto const removed{m_find(Name)}; removed.has_value())
		return false == *removed;
	if (false == m_standing.has_value())
	{
		m_standing.emplace();
		for (auto const &name : m_names())
			m_standing->insert(game_name(name));
	}
	return m_standing->contains(game_name(Name));
}

bool standing_switches::any_stands(std::vector<std::string> const &Names)
{
	return std::ranges::any_of(Names, [this](std::string const &Name) { return stands(Name); });
}

std::vector<std::string> const &template_switches::of(std::string const &File, int const Revision, int const Frame, reader const &Read)
{
	if (m_revision != Revision)
	{
		m_files.clear();
		m_revision = Revision;
	}
	auto const [found, fresh]{m_files.try_emplace(File)};
	auto &entry{found->second};
	// read once; one that couldn't be read again in each frame at most, as each include of it asks
	if (false == fresh && (false == entry.failed.has_value() || *entry.failed == Frame))
		return entry.suffixes;
	entry.failed.reset();
	if (false == Read(File, entry.suffixes))
	{
		entry.suffixes.clear();
		entry.failed = Frame;
	}
	return entry.suffixes;
}

} // namespace switch_drives
