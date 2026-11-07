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

#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// the signalling of a station (SRK): the station found from a switch of it, its schematic, its name and code
class srk_tool final : public editor_tool
{
  public:
	srk_tool(editor_selection &Selection, editor_view &View, editor_history &History, editor_templates &Templates, editor_signals &Signals)
	    : m_selection{Selection}, m_view{View}, m_history{History}, m_templates{Templates}, m_signals{Signals}
	{
	}
	void update() override;
	void draw_overlay() const override;
	void render_options() override;
	void render_windows() override;
	void forget(scene::basic_node const *Node) override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void set_open(bool const Open) { m_open = Open; }
	// the block boundaries drawn by hand on the lines out of the open station
	bool drawing() const { return m_open && m_drawing && m_layout.has_value(); }
	// what a click at the point of a track does while the block boundaries are drawn, for the label at the mouse; Valid: it does
	// something
	std::string boundary_hover(glm::dvec3 const &Point, bool &Valid);
	void boundary_click();
	// the stations described go to the files of their layers, as "//$s" lines
	void store();

  private:
	editor_selection &m_selection;
	editor_view &m_view;
	editor_history &m_history;
	editor_templates &m_templates;
	editor_signals &m_signals;
	// the block of a line out of the station
	enum class block_kind
	{
		semi, // semi-automatic: a warning disc before the entry signal
		three, // the automatic one of three aspects: the block sections no shorter than the braking distance
		four // of four aspects, the warning two signals ahead: the sections no shorter than half of it
	};
	// where the block of a line out of the station comes from
	enum class block_reason
	{
		hand, // set by hand
		standing, // block signals stand along it
		double_track, // the automatic one on a double-track line
		single // the semi-automatic one on a single-track line
	};
	// a station described: found again by a switch of it
	struct record
	{
		scene::layer_handle layer{null_handle};
		std::string anchor; // the name of a switch of the station
		TTrack *track{nullptr}; // the switch, once found: it stays the anchor when it gets renamed
		std::string code; // the abbreviation of the station, as MMz
		std::string name;
		std::string line; // the number of the railway line
		std::string km; // of the axis of the station along the line, empty while unknown
		bool reversed{false}; // the km rise to the left of the schematic, the switches numbered from there
		int braking{0}; // m, the braking distance of the line set by hand, 0: after its speed
		std::map<std::string, block_kind> blocks; // of the lines by their entry signals, as set by hand
		std::map<std::string, bool> ways; // the automatic block of the lines by their entry signals both ways, as set by hand
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
	bool m_strip{true}; // the schematic shown, off for the screenshots of the self-tests
	bool m_loaded{false};
	bool m_changed{false};
	std::vector<record> m_records;
	std::optional<station::layout> m_layout;
	station::drawing m_plot; // the schematic of the layout as the plot draws it
	int m_record{-1}; // of the open station, -1 while it has none
	TTrack *m_anchor{nullptr}; // the switch the open station was found from
	bool m_reversed{false}; // the switches numbered from the right end of the schematic
	std::size_t m_history_revision{0};
	TTrack const *m_picked{nullptr}; // the track selected when the station was looked for
	view m_view_state;
	bool m_fit_lines{false}; // the schematic fits the lines out to the next stations too
	int m_hover{-1}; // path of the layout under the mouse in the schematic
	bool m_dragging{false};
	std::vector<std::pair<std::string, std::size_t>> m_found; // the stations of the scenery: a switch and the count of them
	struct circuit_mark
	{
		glm::dvec2 at{0.0}; // in the schematic
		glm::dvec3 world{0.0};
		std::string text;
		ImU32 colour{0};
		double length{0.0};
		bool labelled{true}; // the circuits of the switches go without, named after them
		glm::dvec2 drawn{0.0}; // in the plot
	};
	std::vector<int> m_circuit_of; // of each path of the layout, an index into m_circuits, -1 for none
	std::vector<circuit_mark> m_circuits;
	std::vector<glm::dvec2> m_ticks; // the joints between the circuits, in the schematic
	std::vector<glm::dvec2> m_ticks_drawn; // the same in the plot
	struct joint_mark
	{
		glm::dvec3 at{0.0};
		glm::dvec3 across{0.0}; // of the track there, in the plan
	};
	std::vector<joint_mark> m_joints; // the same in the scenery
	struct signal_mark
	{
		glm::dvec2 at{0.0}; // in the schematic
		bool ahead{true}; // the trains it faces run along the chainage
		std::string text;
		glm::dvec3 head{0.0}; // above its lamps, in the scenery
		bool unfit{false}; // its lights don't show what its routes need
		glm::dvec2 drawn{0.0}; // on its track in the plot
		float side{0.0f}; // 1: it stands to the left of the track in the plot, -1: to the right
	};
	std::vector<signal_mark> m_signal_marks;
	bool m_signals_known{false}; // the signals standing surveyed, once their templates are found
	std::vector<station::lengths> m_lengths; // of the station tracks, by their numbers
	bool m_measured{false};
	bool m_numbered{false}; // the station has tracks numbered by their circuits
	int m_length_hover{-1}; // of the lengths, the row under the mouse
	int m_length_canvas{-1}; // of the lengths, of the track under the mouse in the schematic
	int m_length_shown{-1}; // of the lengths, the one drawn: of the row or the track under the mouse, else of the track selected
	// how the SHP track resonator of a signal stands
	enum class shp_state
	{
		needless, // the signal needs none
		missing,
		beyond, // missing, the tracks before the signal end short of its place
		off, // farther from its place than allowed
		placed
	};
	// a signal of the station or of its lines, its lights as the routes of its trains need them: Ie-1, Ie-4 § 8
	struct head_check
	{
		scene::instance_handle instance{0};
		std::string name; // shown, without the code of the station
		std::string kind;
		glm::dvec3 position{0.0};
		glm::dvec2 travel{0.0, 1.0};
		std::string file; // of its template
		std::string lamps;
		std::vector<int> own; // the speeds of its template
		bool station{false}; // stands by the station, not out on a line
		std::vector<station::route> routes;
		std::vector<std::string> nexts; // of the routes, the signals they end at, shown
		std::vector<int> speeds; // km/h, the lower ones it must show: 40, 60, 100
		bool informs{false}; // it must tell the next signal shows stop
		bool white{false}; // it must give the substitute signal
		int shortened{0}; // km/h, it slows the trains to, the next signal nearer than the braking distance: Ie-4 § 8 ust. 5
		bool indicator{false}; // a W21 "14" beside it, for 140 km/h so
		bool fits{true}; // its template shows all of it
		bool more{false}; // its template has more than it needs
		std::string fit; // the template that fits best, empty for none
		std::vector<std::string> problems; // of its place
		// m before it the SHP track resonator belongs, -1 for none: 200 m before an entry signal, a block signal and a warning disc,
		// at an exit signal onto a line of the automatic block
		double shp{-1.0};
		double shp_found{-1.0}; // m before it the trains run into the event of the SHP resonator standing, -1 for none
		bool shp_short{false}; // the tracks before it end short of the place of its resonator
		shp_state shp_of() const;
	};
	std::vector<head_check> m_heads;
	bool m_heads_known{false}; // the templates found
	int m_head_hover{-1}; // of the heads, the row under the mouse
	// a line out of the open station, from an entry signal of it
	struct line_row
	{
		std::string entry; // the entry signal, shown
		glm::dvec3 position{0.0}; // of the entry signal
		glm::dvec2 travel{0.0, 1.0}; // the trains it faces run, in the plan
		int number{0}; // of the track of a double-track line, 0 on a single-track one
		bool inbound{true}; // its trains run towards the station
		int partner{-1}; // the other track of a double-track line, of the lines
		double length{0.0}; // to the entry signal of the next station, or to the end of the tracks
		std::string next; // the entry signal of the next station facing out along it, shown; empty for none
		bool station{false}; // ends at the next station rather than at the end of the tracks
		std::size_t blocks{0}; // the block signals standing along it
		// the block boundaries of the same facing its trains, the joints they turn them to stop at, out along it, with the signal shown
		std::vector<std::pair<double, std::string>> stops;
		std::vector<std::pair<double, std::string>> wrong; // the block signals facing its wrong-track trains, out along it, shown
		block_kind block{block_kind::semi};
		block_reason why{block_reason::single}; // the block it has, so
		block_kind own{block_kind::semi}; // the block it has by itself, unless set by hand
		block_reason own_why{block_reason::single};
		// the automatic block both ways: at each block boundary a block signal of each track either way, the wrong-track one with N
		bool both{false};
		bool both_hand{false}; // so set by hand
		bool own_both{false}; // so by itself
		// block signals stand along it, none switched by block sections of the editor: it keeps the block of the scenery as it stands
		bool legacy{false};
		double braking{0.0}; // m, the braking distance on it
		std::string km_number; // the line of the kilometrage it runs along
		bool rising{true}; // its trains run towards the higher km
		bool km_known{false}; // the km out along it are known
		// in the schematic, straightened: the chainage of the entry signal, 1 or -1 the way out along the chainage, the offset of the
		// track there
		double chainage{0.0};
		double outward{1.0};
		double offset{0.0};
		struct run
		{
			double from{0.0}; // out along the line
			double to{0.0};
			std::string name;
			double km_from{0.0};
			double km_to{0.0};
		};
		std::vector<run> sections; // the track circuits along it, out from the station
		std::vector<std::pair<double, double>> bare; // out along it, tracks between its block sections in none, of the automatic block
		struct post
		{
			double distance{0.0}; // out along the line
			std::string name; // shown
			bool wrong{false}; // facing the wrong-track trains
			double km{0.0};
		};
		std::vector<post> posts; // the block signals along it
		// the signals of the other block standing along it: warning discs on a line of the automatic block, block signals on one of the
		// semi-automatic one, the first of them
		std::size_t leftovers{0};
		glm::dvec3 leftover{0.0};
		glm::dvec2 leftover_travel{0.0, 1.0};
	};
	std::vector<line_row> m_lines;
	std::vector<station::line> m_line_ways; // of the lines listed
	std::unordered_set<std::string> m_signal_names; // of the signals of the scenery, lowercase: a block signal drawn on one's number takes a letter
	std::string m_lines_problem; // why the station has no lines listed
	// what the check of the open station finds wrong: signals no track reads, lines broken off, tracks between the block sections of
	// a line in none, block sections without a track
	struct problem
	{
		std::string text;
		glm::dvec3 at{0.0};
		scene::instance_handle signal{0}; // Repair has a track read it again
		bool lines{false}; // Repair sets the line signals again
	};
	std::vector<problem> m_problems;
	bool m_drawing{false}; // the block boundaries drawn by hand
	// a block boundary under the mouse while they are drawn: block signals across the line there, or the ones standing taken away
	struct boundary
	{
		bool valid{false};
		bool remove{false};
		std::string entry; // of the line under the mouse, its entry signal shown
		glm::dvec3 point{0.0}; // the joint, on its axis
		std::vector<std::pair<glm::dvec3, glm::dvec3>> joints; // of each track, on its axis, the way out along it
		struct post
		{
			glm::dvec3 point{0.0}; // on the axis
			glm::dvec3 side{1.0, 0.0, 0.0}; // the signal stands off the axis so
			std::string name;
		};
		std::vector<post> posts; // of the block signals
	};
	boundary m_boundary;
	// a block boundary drawn on a line of the automatic block of the open station
	struct block_edit
	{
		std::string entry; // of the line, its entry signal shown
		glm::dvec3 point{0.0}; // on its axis
		bool remove{false}; // the block signals standing there taken away, else placed
	};
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
	// the signals placed where the station has none and the standing ones named, the circuits before the entry signals set too; one
	// step of the history
	void set_signals();
	// the signals standing by the station, for the schematic
	void survey_signals();
	// the lengths of the station tracks, the signals given by them
	void measure(std::vector<station::signal> const &Signals);
	// km/h, the highest of the speeds the tracks of the open station are set to, 0 while none is
	double set_speed_of_open() const;
	// km/h, the trains run the station at: the highest set on its tracks, else after the braking distance set by hand, else 160
	double speed_of_open() const;
	// m, the braking distance of the line of the open station: set by hand, else after its speed, as Ie-4 § 8 gives it
	double braking_of_open() const;
	// the lights of the signals checked against the routes of their trains and the places of the signals they tell of
	void check_heads(std::vector<standing_signal> const &Standing);
	void render_heads();
	// the signals given the templates whose lights fit their routes; one step of the history
	void fit_heads();
	// the lengths of the station tracks as the list of the tracks gives them, the useful ones each way
	void render_lengths();
	// the lengths as a table to paste into a spreadsheet, a line a track, the columns parted by tabs
	std::string lengths_table() const;
	// of the lengths, the one of the station track the track is a part of, -1 for none
	int length_of(TTrack const *Track) const;
	// the signals of the lines out of the station: on a double-track line the block signals to the next station and the block
	// sections between them, on a single-track one a warning disc before the entry signal; one step of the history
	void set_lines(block_edit const *Edit = nullptr);
	// the lines out of the open station from its entry signals standing, Entries: the signals of the plan they begin at, Owners:
	// the standing ones; false with the reason while it has none
	bool lines_of_open(std::vector<standing_signal> const &Standing, std::vector<station::line> &Lines, std::vector<station::signal> &Entries, std::vector<std::size_t> &Owners, std::string &Problem) const;
	// the block of the line: as set by hand for its entry signal, else the automatic one where block signals stand along it, of four
	// aspects where one of them has, else on a double-track line, of the aspects of the block signal chosen in Signals, else
	// the semi-automatic one; Own: the block it has by itself
	block_kind block_of(station::line const &Line, std::string const &Entry, std::vector<standing_signal> const &Standing, std::vector<signal_head> const &Heads, block_reason &Why, block_kind &Own, block_reason &Own_why, std::size_t &Blocks) const;
	// the automatic block of the double-track line both ways: as set by hand for its entry signal, else where block signals stand
	// along it, when some of them face its wrong-track trains, else on a new line; Own: so by itself
	bool both_ways(std::vector<station::line> const &Lines, std::size_t Index, std::string const &Entry, std::vector<standing_signal> const &Standing, bool &Hand, bool &Own) const;
	// the line of the kilometrage a line out of the open station runs along: of the station, at a junction another one
	std::string km_line(station::line const &Line) const;
	// the km out along the line, Number: of the line of its kilometrage
	double km_along(station::line const &Line, std::string const &Number, double const Distance) const;
	// the trains of the line run towards the higher km
	bool km_rising(station::line const &Line, std::string const &Number) const;
	// m, the braking distance at the highest speed out along the line, at 160 km/h while its tracks have none, unless set by hand
	double line_braking(station::line const &Line) const;
	// the lines out of the open station, for the panel
	void survey_lines(std::vector<standing_signal> const &Standing);
	void render_lines();
	void survey_problems(std::vector<standing_signal> const &Standing);
	void render_problems();
	// the signals no track reads read by the tracks they stand by, the lines with tracks in no block section set again; a step of
	// the history each
	void fix_problems();
	// the SHP track resonators placed where the signals checked need them and have none: 200 m before an entry signal, a block
	// signal and a warning disc, at an exit signal onto a line of the automatic block, the track cut where none of its joints lies
	// within 5 m, its event put on the track the trains run into there; one step of the history
	void set_shp();
	// the names of the switches and the circuits of the open station begin with it, empty while it has no code
	std::string stem_of_open() const;
	// the km at the chainage of the open station: of the kilometrage of its line, else of its record, else of its switches; false
	// while unknown
	bool km_of_open(double const Chainage, double &Km) const;
	// the km at the chainage of the open station, as the Kilometrage tool sets them for its line; false where it doesn't
	bool km_of_line(double const Chainage, double &Km) const;
	// the line of the open station: of its record, else the one the Kilometrage tool sets the km of through it; empty for none
	std::string line_of_open() const;
	// the station tracks by their numbers with their circuits as the tracks have them, Inside: the tracks of the circuits; false
	// while it has none
	bool station_tracks(std::vector<station::numbered> &Tracks, std::unordered_set<TTrack const *> &Inside) const;
	// the circuits of the station track of the track renamed after another number; one step of the history
	void renumber_track(TTrack const &Track, std::string const &Number);
};
