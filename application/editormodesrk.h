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
#include "editor/editorStation.hpp"
#include "scene/scenelayers.h"

#include "imgui/imgui.h"

#include <optional>
#include <string>
#include <utility>
#include <vector>

// the signalling of a station (SRK): the station found from a switch of it, its schematic, its name and code
class srk_tool final : public editor_tool
{
  public:
	srk_tool(editor_selection &Selection, editor_view &View, editor_history &History) : m_selection{Selection}, m_view{View}, m_history{History} {}
	void update() override;
	void draw_overlay() const override;
	void render_options() override;
	void render_windows() override;
	void forget(scene::basic_node const *Node) override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void set_open(bool const Open) { m_open = Open; }
	// the stations described go to the files of their layers, as "//$s" lines
	void store();

  private:
	editor_selection &m_selection;
	editor_view &m_view;
	editor_history &m_history;
	// a station described: found again by a switch of it
	struct record
	{
		scene::layer_handle layer{null_handle};
		std::string anchor; // the name of a switch of the station
		TTrack *track{nullptr}; // the switch, once found: it stays the anchor when it gets renamed
		std::string code; // the abbreviation of the station, as MMz
		std::string name;
		std::string line; // the number of the railway line
	};
	// the schematic: the chainage across, the offsets upwards, exaggerated
	struct view
	{
		double from{0.0};
		double to{1000.0};
		double centre{0.0};
		float exaggeration{0.0f}; // 0: fit at the next draw
	};
	bool m_open{false};
	bool m_loaded{false};
	bool m_changed{false};
	std::vector<record> m_records;
	std::optional<station::layout> m_layout;
	int m_record{-1}; // of the open station, -1 while it has none
	TTrack *m_anchor{nullptr}; // the switch the open station was found from
	bool m_reversed{false}; // the switches numbered from the right end of the schematic
	std::size_t m_history_size{0};
	TTrack const *m_picked{nullptr}; // the track selected when the station was looked for
	view m_view_state;
	int m_hover{-1}; // path of the layout under the mouse in the schematic
	bool m_dragging{false};
	std::vector<std::pair<std::string, std::size_t>> m_found; // the stations of the scenery: a switch and the count of them
	struct circuit_mark
	{
		glm::dvec2 at{0.0}; // in the schematic
		std::string text;
		ImU32 colour{0};
		double length{0.0};
		bool labelled{true}; // the circuits of the switches go without, named after them
	};
	std::vector<int> m_circuit_of; // of each path of the layout, an index into m_circuits, -1 for none
	std::vector<circuit_mark> m_circuits;
	std::vector<glm::dvec2> m_ticks; // the joints between the circuits, in the schematic
	std::string m_status;

	void load();
	void open(TTrack &Track);
	void refresh();
	record &record_of_open();
	std::string title() const;
	void render_found();
	void render_canvas(float const Height);
	void fit_view(float const Width, float const Height);
	void find_all();
	// the switches numbered along the station and named after its code; one step of the history
	void number_switches();
	// the station cut at the joints of its track circuits and the circuits named; one step of the history
	void set_circuits();
	// the track circuits as the tracks have them, for the schematic
	void survey();
	// the names of the switches and the circuits of the open station begin with it, empty while it has no code
	std::string stem_of_open() const;
	// the circuits of the station track of the track renamed after another number; one step of the history
	void renumber_track(TTrack const &Track, std::string const &Number);
};
