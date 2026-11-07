/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"
#include "application/editormoderailservices.h"
#include "application/editormodetrackprops.h"

#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

class TTrack;

// properties taken from a track and set on the selected ones, and the presets of the properties (editorTrackPresets), made from a
// track and put on the selected ones; see editormodetrackprops.cpp
class track_style_tool final : public editor_tool
{
  public:
	track_style_tool(editor_selection &Selection, editor_rail_services &Rail, track_sheet_tool &Sheet) : m_selection{Selection}, m_rail{Rail}, m_sheet{Sheet} {}
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void copy_track_style(TTrack const &Track);
	std::size_t paste_track_style(std::vector<TTrack *> const &Tracks);
	void render_track_style(std::vector<TTrack *> const &Tracks);
	void render_track_presets_menu();
	// the next click on a track in the view takes its properties
	bool picking() const { return m_track_style.picking; }
	void stop_picking() { m_track_style.picking = false; }
	// properties were taken from a track, the one named
	bool taken() const { return false == m_track_style.values.empty(); }
	std::string const &taken_from() const { return m_track_style.from; }

  private:
	// which groups of the properties are pasted or put on the tracks by a preset, in the order of the groups of editormodetrackprops.cpp
	using track_groups = std::array<bool, 11>;
	// properties taken from a track, to be set on the selected ones
	struct track_style
	{
		std::string from;
		std::vector<std::pair<std::string, std::string>> values;
		// ticked at first by their groups: speed, environment, overhead line, track circuits, rails, trackbed, its shape, sleepers, ride and
		// sound, frogs, sleeper placement
		track_groups groups{false, false, false, false, true, true, true, true, false, true, true};
		std::map<std::string, bool, std::less<>> ticks; // changed by hand, by the keys of the properties
		bool picking{false}; // the next click on a track in the view takes its properties
		bool reveal{false};  // its section of the inspector opens, a track having been taken
	};
	// a property of the track taken which the selected ones have otherwise
	struct track_style_line
	{
		int property{-1};
		std::string now;       // as the selected tracks have it, or how many values they have
		std::string taken;     // as the track taken has it
		std::size_t differ{0}; // of the selected tracks, the ones it changes
	};
	struct track_presets_panel
	{
		int selected{0}; // id of the preset chosen in the inspector
		int nameid{0}; // the preset whose name is in the field
		std::array<char, 128> name{};
	};

	std::vector<track_style_line> track_style_lines(std::vector<TTrack *> const &Tracks) const;
	bool track_style_takes(std::string const &Key) const;
	// what the track taken changes on the selected tracks, each with a tick, and the button setting it
	void render_style_taken(std::vector<TTrack *> const &Tracks);
	// a line of what the track taken changes: its tick and how it changes. returns: true when it's ticked
	bool render_style_line(int const Property, std::string const &Now, std::string const &Taken, std::size_t const Differ, std::size_t const Selected);
	// the values of the properties of the chosen groups put on the tracks, in one step of the undo. returns: the number of the
	// tracks changed, none when no value is of the chosen groups
	std::optional<std::size_t> apply_track_values(std::vector<std::pair<std::string, std::string>> const &Values, track_groups const &Groups, std::vector<TTrack *> const &Tracks, char const *What);
	// presets of the properties of the tracks (editorTrackPresets), made from a track and put on the selected ones
	void render_track_presets(std::vector<TTrack *> const &Tracks);
	// the combo choosing the preset
	void render_preset_combo();
	// the buttons applying the preset and making a new one from the track
	void render_preset_new(std::vector<TTrack *> const &Tracks, bool const None, float const Width);
	// the name of the preset of the id, renamed by Enter or the button
	void render_preset_name(int const Id);
	// the ticks of what the preset of the id overwrites
	void render_preset_groups(int const Id);
	// the buttons updating the preset of the id from the track and deleting it
	void render_preset_update(std::vector<TTrack *> const &Tracks, int const Id, float const Width);
	std::size_t apply_track_preset(int const Id, std::vector<TTrack *> const &Tracks);
	editor_selftest_result selftest_trackstyle(std::istringstream &Words);
	void selftest_trackstyle_list();

	editor_selection &m_selection;
	editor_rail_services &m_rail;
	track_sheet_tool &m_sheet;
	track_style m_track_style;
	track_presets_panel m_track_presets;
};
