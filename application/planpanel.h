/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"

#include "editor/orthophoto.h"
#include "maj0sted/editor/layout.hpp"
#include "maj0sted/editor/model.hpp"
#include "maj0sted/editor/render.hpp"
#include "maj0sted/editor/solution.hpp"
#include "maj0sted/io/scn_export.hpp"

#include <string>
#include <vector>

// track layout tool. the drawing happens in the viewport, which the tool puts into a top-down
// orthographic plan view; the panel holds only the controls. none of the geometry is computed here:
// maj0sted solves the whole document and the panel paints what comes back.
//
// a track is a chain of elements, and every one of them says outright what it is: prosta of so many
// metres, luk of radius R turning so far, krzywa przejsciowa easing to R over so many metres. there
// is no second language of "run on until you reach that line" - laying by clicking along the ground
// works out the corner at the click and writes the numbers down.
//
// what one track keeps against another is said as a relation, not redrawn by hand: an element may be
// held parallel to another track's element at a given miedzytorze, and the solver moves the whole
// track until it really is.
class plan_panel : public ui_panel
{

  public:
	plan_panel(std::string const &Name, bool const Isopen);

	void update() override;
	void render_contents() override;

  private:
	using Document = maj0sted::editor::Document;
	using Element = maj0sted::editor::Element;
	using ElementId = maj0sted::editor::ElementId;
	using Solution = maj0sted::editor::Solution;
	using Track = maj0sted::editor::Track;
	using TrackId = maj0sted::editor::TrackId;
	using TurnoutId = maj0sted::editor::TurnoutId;

	// panels
	void render_toolbar();
	void render_elements();
	void render_turnouts();
	void render_turnout_type(maj0sted::editor::TurnoutType &Type_);
	// the catalogue drawing of the chosen type, in its own window: the diverging path piece by
	// piece, both tracks' rails, and the points a drawing is dimensioned from
	void render_template_window();
	void render_diagnostics();
	void render_storage();
	void export_scn();
	void seed_catalogue();
	void render_newmap_dialog();
	void render_location_dialog();

	// scene
	void handle_scene();
	void draw_on_scene();
	void go_to_plan();

	// document
	void solve();
	void start_map(bool const Georeferenced, double const Originx, double const Originy);
	Track *current_track();
	Track const *current_track() const;
	void new_track();
	void delete_current_track();
	void clear_selection();
	// carries the track on to (X,Y). the track ends where it was last clicked, so that end is the
	// corner: it gets rounded with an arc of m_corner_radius, the straight behind it gives up the
	// tangent, and the new straight runs out to (X,Y) exactly. all of it lands in the document as
	// ordinary elements with ordinary lengths
	void append_point_to(double const X, double const Y);
	// takes back one click: the trailing straight, its corner arc, and the tangent the arc had
	// taken off the straight before it - which leaves the track ending on the click before
	void drop_last_vertex();
	// appends one element to the end of the current track, exactly as asked for
	void append_element(maj0sted::editor::Kind const Kind);
	void drop_last_element();
	void delete_selected_element();
	void place_turnout_at(TrackId const On, double const Wx, double const Wy);
	// re-lays the edited track along the lines its straights sat on before the change, so a new
	// radius (or a new transition length) is absorbed by the corner it belongs to and nothing
	// further down the track moves. false, with a reason, when it cannot be laid that way
	bool refit_current_track(std::string &Why);
	// swings the grabbed straight onto the line through its far end and (X,Y), then re-fits the
	// arcs on either side of it. leaves the track untouched when that cannot be laid
	void drag_straight_to(double const X, double const Y);

	// hit tests, in screen pixels
	// the end of a straight of the edited track: OutIndex is its element index, OutEnd 0 for the
	// end it starts on and 1 for the end it runs out to
	bool hit_straight_end(ImVec2 const &Mouse, std::size_t &OutIndex, int &OutEnd) const;
	bool hit_element(ImVec2 const &Mouse, ElementId &OutElement, TrackId &OutTrack) const;
	bool hit_turnout(ImVec2 const &Mouse, TurnoutId &OutTurnout) const;
	// nearest station along a solved track's axis, for placing and sliding turnouts
	bool station_on(TrackId const Track, double const Wx, double const Wy, double &OutStation) const;
	TrackId nearest_track_axis(ImVec2 const &Mouse, float const Tolerance) const;

	// members
	Document m_document;
	Solution m_solution;
	std::vector<maj0sted::editor::PlanPolyline> m_rails;

	TrackId m_track{};
	ElementId m_sel_element{};
	TurnoutId m_sel_turnout{};

	// a press on the ground does whatever is under it: picks up what is drawn there, or carries the
	// current track on to there. every transient errand (placing a turnout, picking the element to
	// run parallel to) is started from its own button, and Esc drops back out of it
	bool m_dragging_turnout{false};

	// grabbing a straight by one of its ends: which one, and the far end it swings about. the far
	// end is captured once, at the grab, because re-fitting the arc behind it moves it
	bool m_dragging_straight{false};
	std::size_t m_drag_index{0};
	int m_drag_end{0};
	double m_pivot_x{0.0};
	double m_pivot_y{0.0};

	// an empty track needs two clicks before there is anything to carry on: the first only marks
	// where it starts. after that the track always ends on the last click, which is the corner the
	// next one is rounded at
	bool m_pending{false};
	double m_startx{0.0};
	double m_starty{0.0};

	// 0 idle, 1 waiting for a click on the through track
	int m_pick_turnout{0};
	// 0 idle, 1 waiting for the element the selected one is to be held parallel to
	int m_pick_parallel{0};

	// what the next appended element is made of
	double m_new_radius{300.0};
	double m_new_length{100.0};
	int m_new_hand{1};
	// the radius a corner is rounded with when laying by clicking
	double m_corner_radius{300.0};
	// miedzytorze a new hold is given, metres
	double m_new_offset{4.75};

	int m_turnout_type{0};
	// which piece of the chosen type's geometry list is open for editing; -1 = none
	int m_sel_piece{-1};

	// the template drawing. a switch is thirty metres long and a metre and a half wide, so like
	// every catalogue drawing this one is stretched across the track - the ratio is said on screen
	bool m_show_template{false};
	float m_tpl_zoom{1.0f};
	float m_tpl_pan{0.0f};    // metres off centre, along the through track
	float m_tpl_exagg{6.0f};  // przewyzszenie poprzeczne
	bool m_tpl_rails{true};
	bool m_tpl_marks{true};
	// lukowanie in the drawing: the radius the tor zasadniczy is bent to, and which way
	double m_tpl_bend_radius{0.0};
	int m_tpl_bend_hand{0};

	char m_namebuf[128]{};
	std::string m_status;
	char m_path[256]{"editor/plan.m0s"};
	char m_scn_path[256]{"scenery/plan_export.scn"};

	bool m_pickingplace{false};
	double m_pickx{0.0};
	double m_picky{0.0};
	double m_mapviewx{0.0};
	double m_mapviewy{0.0};
	double m_mapscale{0.0};

	editor::wms_image m_topomap{maj0sted::editor::WmsConfig::geoportal_topo()};
	editor::orthophoto_source m_ortho;
	// 1 km topo cells for views wider than a kilometre (orto's 100 m grid is too dense there)
	editor::orthophoto_source m_topo{maj0sted::editor::WmsConfig::geoportal_topo(), 1000, "topo"};
	bool m_showortho{true};
};
