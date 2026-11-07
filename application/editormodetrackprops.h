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

#include <array>
#include <cstddef>
#include <functional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

class TTrack;

// the properties of the selected tracks in the inspector, one sheet for one track and for many, the list of the selected tracks and
// the tools selecting more of them; see editormodetrackprops.cpp
class track_sheet_tool final : public editor_tool
{
  public:
	track_sheet_tool(editor_selection &Selection, editor_view &View, editor_history &History, editor_rail_services &Rail) : m_selection{Selection}, m_view{View}, m_history{History}, m_rail{Rail} {}
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	// changes each track which can be changed, in one step of the undo; false from Change: the track stays as it was
	std::size_t tracks_apply(std::vector<TTrack *> const &Tracks, std::function<bool(TTrack &)> const &Change, bool const Rebuild, char const *What);
	int track_property_index(std::string_view const Key) const;
	std::size_t track_property_apply(int const Property, std::vector<TTrack *> const &Tracks, std::string const &Text);
	void track_field_finish();
	void track_field_settle();
	void render_track_properties(std::vector<TTrack *> const &Tracks);
	void render_track_set_header(std::vector<TTrack *> const &Tracks);
	void render_track_select_tools(std::vector<TTrack *> const &Tracks);
	std::vector<TTrack *> similar_tracks(int const Property, std::vector<TTrack *> const &Tracks) const;
	// the values of the properties over the tracks, when the sheet was drawn for the tracks of the signature
	std::vector<std::vector<std::pair<std::string, int>>> const *known_values(std::size_t const Signature, std::size_t const Count) const;
	// the track under the mouse in the list of the selected tracks, in this frame or the one before; nullptr: none
	TTrack const *list_hover() const;

  private:
	// values of the properties over the selected tracks, each with the number of the tracks having it
	struct track_values
	{
		std::size_t signature{0};
		int generation{0};
		int generation_seen{-1};
		std::size_t history{0};
		int frame{-100};
		std::vector<std::vector<std::pair<std::string, int>>> values;
		std::vector<std::size_t> applies;
		std::size_t unsaved{0}; // of the tracks, changed since the file was written
	};
	// how the last value given in the sheet went: on how many tracks, and why not on the others
	struct track_outcome
	{
		int property{-1};
		std::string text;
		bool refused{false}; // some tracks couldn't take it
		std::size_t signature{0}; // of the tracks it was given to
		std::size_t history{0};
		double time{0.0};
	};
	// the lines under the rows of the sheet
	struct track_note
	{
		enum class kind
		{
			typing,
			done,
			refused
		};
		int property{-1};
		std::string text;
		kind type{kind::done};
	};
	// the field being typed in, with the tracks it was started on
	struct track_field
	{
		int property{-1};
		std::vector<TTrack *> tracks;
		std::array<char, 256> text{};
		std::string initial;
		int frame{0};
	};

	// the values of the properties over the tracks gathered again when the selection or the tracks changed
	void property_values_gather(std::vector<TTrack *> const &Tracks, std::size_t const Signature);
	// the notes on the values of the properties made again when the selection or the tracks changed
	void property_notes(std::size_t const Signature);
	// the tooltip of a row of the properties
	void property_row_help(std::size_t const Index, std::size_t const Selected) const;
	// the label of a row of the properties, marked when the selected tracks differ in it
	void property_row_label(std::size_t const Index, std::size_t const Selected) const;
	// the rows of the properties by their kind: a text, a choice, a tick, a texture
	void property_text_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	void property_choice_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	void property_check_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	void property_texture_row(std::size_t const Index, std::vector<TTrack *> const &Tracks);
	// whether any property of the part is shown, More: of those under "more"
	bool property_rows_any(int const Part, bool const More) const;
	// the rows of the properties of the part
	void property_rows(int const Part, bool const More, std::vector<TTrack *> const &Tracks);
	// the notes under a row of the properties
	void property_row_notes(std::size_t const Index) const;
	// the rows under "more" of the part, in a tree node of the id
	void property_more_rows(char const *Id, int const Part, std::vector<TTrack *> const &Tracks);
	// the rows of the look of the track
	void property_look_rows(std::vector<TTrack *> const &Tracks);
	// a row of the list of the selected tracks: Removed or Alone set to the track when it's taken out or kept alone
	void render_selected_track_row(TTrack *Track, int const Index, float const Column, TTrack *&Removed, TTrack *&Alone);
	// the popup selecting the tracks of the scenery alike the selected ones
	void render_similar_popup(std::vector<TTrack *> const &Tracks);
	// the commands of the self-test, one each
	editor_selftest_result selftest_tracksel(std::istringstream &Words);
	editor_selftest_result selftest_trackprop(std::istringstream &Words);
	editor_selftest_result selftest_trackprops();
	editor_selftest_result selftest_tracksimilar(std::istringstream &Words);

	editor_selection &m_selection;
	editor_view &m_view;
	editor_history &m_history;
	editor_rail_services &m_rail;
	track_values m_track_values;
	track_outcome m_track_outcome;
	std::vector<track_note> m_track_notes;
	track_field m_track_field;
	std::vector<std::vector<TTrack *>> m_track_similar;
	TTrack const *m_track_list_hover{nullptr}; // in the list of the selected tracks
	int m_track_list_hover_frame{-1};
};
