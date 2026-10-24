/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <utility>
#include <vector>

// user-defined presets of the properties of tracks (rails, trackbed, sleepers...), taken from a track and put on others
// with the choice of what they overwrite. stored next to the user's ini files, like the model sets
struct editor_track_preset
{
	int id{0}; // runtime identifier, stable while the editor runs (renames don't break references)
	std::string name;
	std::string from; // name of the track the values were taken from
	// the values of the properties as the track inspector reads and writes them, by their keys
	std::vector<std::pair<std::string, std::string>> values;
	// the groups of the properties the preset overwrites, by their keys; the other values of it are kept but left out
	std::vector<std::string> groups;
};

class editorTrackPresets
{
  public:
	bool load();
	bool save() const;

	std::vector<editor_track_preset> const &presets() const
	{
		return m_presets;
	}
	editor_track_preset const *find(int Id) const;
	// all modifying methods save the presets right away
	// returns: id of the new preset
	int create(std::string const &Name, std::string const &From, std::vector<std::pair<std::string, std::string>> const &Values, std::vector<std::string> const &Groups);
	void remove(int Id);
	void rename(int Id, std::string const &Name);
	// the values taken again, from another track or the same one changed
	void update(int Id, std::string const &From, std::vector<std::pair<std::string, std::string>> const &Values);
	void set_groups(int Id, std::vector<std::string> const &Groups);
	// name not used by any other preset, built from specified base
	std::string unique_name(std::string const &Base, int IgnoreId = 0) const;

  private:
	editor_track_preset *find_(int Id);

	std::vector<editor_track_preset> m_presets;
	int m_nextid{1};
};

extern editorTrackPresets EditorTrackPresets;
