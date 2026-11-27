/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

// what the switch tool of the editor tells of the drives of the switches, with no track and no window of its own
namespace switch_drives
{

// a name as the game finds a track by it: '/' for '\', lower case
std::string game_name(std::string Name);

// whether the switches a drive throws stand in the scenery, told by their names as the events of the drive find them in the game
class standing_switches
{
  public:
	// nullopt when the name finds no track, else whether the track it finds is removed
	using finder = std::function<std::optional<bool>(std::string const &Name)>;
	// the names of all the tracks of the scenery not removed, as they're written
	using lister = std::function<std::vector<std::string>()>;

	standing_switches(finder Find, lister Names);

	// the track the name finds stands while it isn't removed. an older scenery may have two tracks of a name, the name finding the later
	// one; deleted, it takes the name away, and the earlier one left still stands, so with no track found by the name any track of this
	// name not removed counts
	bool stands(std::string const &Name);
	// a drive of a double switch throws four of them: it still throws one while one of them stands
	bool any_stands(std::vector<std::string> const &Names);

  private:
	finder m_find;
	lister m_names;
	// the game names of the tracks not removed, read once, at the first name finding none
	std::optional<std::unordered_set<std::string>> m_standing;
};

// the switches each template of a drive throws, read once from each file while the templates found stay the same: a template changed
// on the disk is found again by a rescan or by its save in the editor, either changing the revision of the templates. a file which
// can't be read is read again, in a later frame, so a template missing for a moment isn't taken for one that isn't a drive until the
// next rescan
class template_switches
{
  public:
	// what follows the first parameter in the names of the switches File throws, as editor_includes::switched_tracks() tells it.
	// returns: false if the file can't be read
	using reader = std::function<bool(std::string const &File, std::vector<std::string> &Suffixes)>;

	// the switches File throws, none for a template which isn't a drive or can't be read. Revision: of the templates found, Frame: the
	// frame asked in. the reference holds until a call with another revision
	std::vector<std::string> const &of(std::string const &File, int Revision, int Frame, reader const &Read);

  private:
	struct entry
	{
		std::vector<std::string> suffixes;
		std::optional<int> failed; // the frame the file couldn't be read in
	};
	std::map<std::string, entry, std::less<>> m_files;
	std::optional<int> m_revision;
};

} // namespace switch_drives
