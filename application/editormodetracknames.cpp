/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "editor/editorFormat.hpp"
#include "editor/editorTrack.hpp"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/asciiname.h"
#include "utilities/translation.h"

#include <algorithm>
#include <format>
#include <string>
#include <vector>

namespace
{

// how long the warning about the names stays in the middle of the screen
constexpr double kWarningSeconds{20.0};

// the name in ASCII which no other path has yet
std::string free_name(TTrack const &Track)
{
	auto const base{ascii_name::fold(Track.name())};
	auto name{base};
	for (int number = 2;; ++number)
	{
		auto const *taken{simulation::Paths.find(name)};
		if (taken == nullptr || taken == &Track)
			return name;
		name = base + "_" + std::to_string(number);
	}
}

} // namespace

void editor_mode::ascii_track_names()
{
	struct change
	{
		std::string from;
		std::string to;
	};
	std::vector<change> changes;
	int unsaved{0};
	// collected first: the renaming changes the map the names are looked up in, not the sequence of the paths
	std::vector<TTrack *> named;
	for (auto *track : simulation::Paths.sequence())
	{
		// the lanes take their names from their road
		if (track == nullptr || track->m_editorremoved || (track->iCategoryFlag & 0x80) != 0 || track->m_road != nullptr)
			continue;
		if (false == ascii_name::plain(track->name()))
			named.push_back(track);
	}
	for (auto *track : named)
	{
		auto const old{track->name()};
		auto const name{free_name(*track)};
		// a path of a template is written with its new name only if the name is given to it by a parameter of the include
		auto const saved{track->from_template() ? editor_track::uses_of_name(*track).includes > 0 : scene::Layers.editable(track)};
		// the include directives which give the path its name, the memory cells which point to it are changed along with it
		editor_track::rename(*track, name);
		if (false == saved)
			++unsaved;
		WriteLog(std::format("Editor: track \"{}\" renamed to \"{}\", its name had characters outside of ASCII{}", ascii_name::readable(old), name, saved ? "" : "; its file isn't written by the editor"));
		changes.push_back({ascii_name::readable(old), name});
	}
	if (changes.empty())
		return;
	auto text{format(STR_C("The names of %zu tracks had characters outside of ASCII (Polish letters and the like), they were changed to ASCII:"), changes.size())};
	constexpr std::size_t shown{6};
	for (std::size_t i = 0; i < std::min(shown, changes.size()); ++i)
		text += "\n  " + changes[i].from + "  ->  " + changes[i].to;
	if (changes.size() > shown)
		text += "\n  " + format(STR_C("... and %zu more, the whole list is in the log"), changes.size() - shown);
	text += "\n\n";
	text += STR_C("The events and the rest still find the tracks by their old names. Save the scenery to write the new names into its files.");
	if (unsaved > 0)
		text += "\n" + format(STR_C("%d of them are in files the editor doesn't write; there they stay as they were."), unsaved);
	ui()->set_warning(text, kWarningSeconds);
	ui()->set_status(format(STR_C("Track names changed to ASCII: %zu"), changes.size()));
}
