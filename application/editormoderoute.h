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
#include "application/editormodebuildservices.h"
#include "application/editormodebuildshared.h"
#include "imgui/imgui.h"

#include <cstddef>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class route_designer;
class route_relay_tool;
class route_overlay;
class route_window;

// the line designed between two paths: read from them or from the alignment kept in the scenery, stored, laid and applied
class route_designer final : public editor_tool
{
  public:
	route_designer(editor_services &Services, editor_build_services &Build, build_shared &Shared, route_relay_tool &Route_relay) : m_services{Services}, m_build{Build}, m_route_relay{Route_relay}, m_route{Build.route_state()}, m_straights{Build.straight_lines()}, m_infra_carried{Shared.infra_carried} {}
	bool route_active() const;
	void route_recommend(alignment::vertex &Vertex) const;
	void route_bind_ends();
	void route_reset();
	alignment::vertex route_vertex_of(editor_track::curve const &Curve) const;
	bool route_load(TTrack &Track);
	void route_take_stored(int const Index);
	void render_route_stored();
	void route_apply();
	glm::dvec3 route_vertex_position(int const Vertex) const;
	bool route_hit(int &Vertex, int &Grip) const;
	editor_track::straight const &current_straight();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	route_relay_tool &m_route_relay;
	route_design &m_route;
	straights_state &m_straights;
	std::set<std::pair<scene::basic_node const *, scene::instance_handle>> &m_infra_carried;
	bool route_from_curve(TTrack &Track);
	// the first curve of the design and the last one going on from the curves of the tracks adjoining, where they turn their way: a
	// curve cut by a switch bent into it, say. Lays the design
	void route_curved_ends();
	// the first curve of the design, or the last one, goes on from the curve of the track adjoining at that end
	bool route_goes_on(bool const Atend) const;
	void route_read_stored(TTrack &Track);
	// the note on what reading the design kept did to the status: the line digitized whole or in parts, the paths Left out of it
	std::string route_stored_note(bool const Digitized, int const Parts, int const Skipped, std::string const &Failure, bool const Left) const;
	void route_store();
	bool route_from_line(TTrack &Track);
	void route_lay();
	// the ends of the design bound to the line laid, its first vertex and its last one moved in by what the curve took over of the
	// straights there
	void route_extend_ends(double const Startextension, double const Endextension);
};

// the tracks drawn made into designed lines, and the switches and crossovers on them taken out and laid again with the lines
class route_relay_tool final : public editor_tool
{
  public:
	route_relay_tool(editor_services &Services, editor_build_services &Build, route_designer &Designer, profile_library &Library) : m_services{Services}, m_build{Build}, m_designer{Designer}, m_library{Library}, m_profiles{Build.stored_profiles()}, m_route{Build.route_state()}, m_straights{Build.straight_lines()}, m_infra{Build.infra_state()} {}
	bool digitize_drawn(std::vector<TTrack *> const &Created, bool const Alone = true);
	// the lines kept, the paths left out
	void digitize_all(int *Kept = nullptr, int *Failed = nullptr);
	int digitize_parts(editor_track::chain const &Chain, std::string *Reason, int &Skipped);
	// the crossovers on the switches of the chain taken out, the line laid again and the crossovers laid back onto it; false when
	// there are none to take out, true when the line is done with, laid or refused
	bool route_lay_crossovers(editor_track::chain const &Chain);
	// the adjoining straights cut where the curve takes them over, the chain found again between its new ends From and To; false
	// when that fails, what was done taken back
	bool route_cut_straights(editor_track::chain &Chain, TTrack *&From, TTrack *&To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created);
	// the crossovers taken out before laid again where they fit the lines now
	void route_crossovers_again();
	bool route_relay_through_switches(editor_track::chain const &Chain, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, TTrack *&Last);
	// the lines, the switches, the crossovers and the objects along the tracks at one go; without Write only counts them
	std::string digitize_scenery(bool const Write);

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	route_designer &m_designer;
	profile_library &m_library;
	std::vector<stored_profile> &m_profiles;
	route_design &m_route;
	straights_state &m_straights;
	infra_window &m_infra;
	bool digitize_chain(editor_track::chain const &Chain, std::string *Reason = nullptr, TTrack **Culprit = nullptr) const;
	// the vertices of the design recognized from the paths of the chain in the shape of its transitions; false where two straights meet
	// with no curve, why and at which path given
	bool digitize_vertices(editor_track::chain const &Chain, alignment::design &Design, std::string *Reason, TTrack **Culprit) const;
	// the switch of a crossover a track of the chain joins at an end of the chain, none when there's no such
	TTrack const *route_end_crossover(editor_track::chain const &Chain) const;
	// the crossovers on the switches, each one once, with where they go back
	std::vector<route_design::crossover_out> route_crossovers_on(std::vector<TTrack *> const &Switches);
	void route_take_out_crossovers(std::vector<TTrack *> const &Switches);
	// the crossovers laid back onto the new line, those which don't fit it kept to be laid later; the status tells which
	void route_crossovers_back(std::vector<route_design::crossover_out> const &Crossovers, editor_track_tab const Tab);
	// the switches without a type kept get the one of the catalogue they are of, or the size measured from their shape
	void digitize_switches(digitize_report &Report, bool const Write) const;
	// two switches facing each other from two tracks, their diverging tracks joined by a plain one, are kept as a crossover
	void digitize_crossovers(digitize_report &Report, bool const Write) const;
};

// the designed line in the view: its curves, the points clicked when it was laid, the trace along the photo and the gizmo of its vertices
class route_overlay final : public editor_tool
{
  public:
	route_overlay(editor_services &Services, editor_build_services &Build, build_shared &Shared, route_designer &Designer) : m_services{Services}, m_build{Build}, m_designer{Designer}, m_route{Build.route_state()}, m_route_dragging{Build.route_dragging()}, m_route_drag_from{Shared.route_drag_from} {}
	// the camera over the stretch of the line between the chainages, a curve seen whole
	void route_show_curve(double const From, double const To);
	// the departures of the points clicked from the line and the curves they belong to, measured again when the line changed
	void route_measure_clicks();
	// the radii of the curve of the vertex fitted again to the points clicked along it, its transitions and cants as they are
	bool route_hold_clicks(int const Vertex);
	void draw_route_overlay() const;
	void render_route_trace();
	void route_trace_start();
	void route_trace_fit();
	void route_trace_end(bool const Keep);
	glm::dvec3 route_trace_cursor() const;
	int route_trace_hit() const;
	void render_route_gizmo();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	route_designer &m_designer;
	route_design &m_route;
	bool &m_route_dragging;
	alignment::design &m_route_drag_from;
	// the elements of the part of the curve in focus in the panel, in their order along the line
	std::vector<std::size_t> route_focused() const;
	// the element of the curve in focus, one of its Own elements, is of the part in focus; Radii those of the arcs of the curve
	bool route_focus_takes(std::size_t const Element, std::vector<std::size_t> const &Own, std::vector<double> const &Radii) const;
	// the points clicked along the curve of the vertex, in the plan, the bends of straights left out
	std::vector<glm::dvec2> route_clicks_of(int const Vertex) const;
	// the line designed in its colours, with what the panel points at along it
	void draw_route_line(screen_projection const &Projection, ImDrawList *Drawlist) const;
	// the part of the curve in focus in the panel lit up, its Focused elements, and named
	void draw_route_focus(screen_projection const &Projection, ImDrawList *Drawlist, std::vector<std::size_t> const &Focused) const;
	// the name of the part of the curve in focus, its first element and its last one that long; empty when the line has no such curve
	std::string route_focus_name(double const Firstlength, double const Lastlength) const;
	// the vertex and the grip of its curve, with its radii
	void draw_route_vertex(screen_projection const &Projection, ImDrawList *Drawlist, int const Index) const;
	void draw_route_clicks(screen_projection const &Projection, ImDrawList *Drawlist) const;
	void draw_route_traced(screen_projection const &Projection, ImDrawList *Drawlist) const;
	// the grip of a curve dragged to the point: the radius of the curve passing through it
	void route_drag_grip(glm::dvec3 const &Moved);
};

// the panel of the designed line: the band of its curvature, its parameters, its vertices with their curves and cants, and the result
class route_window final : public editor_tool
{
  public:
	route_window(editor_services &Services, editor_build_services &Build, route_designer &Designer, route_relay_tool &Route_relay, route_overlay &Route_view) : m_services{Services}, m_build{Build}, m_designer{Designer}, m_route_relay{Route_relay}, m_route_view{Route_view}, m_route{Build.route_state()} {}
	void render_route_ui();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	route_designer &m_designer;
	route_relay_tool &m_route_relay;
	route_overlay &m_route_view;
	route_design &m_route;
	bool render_route_band();
	route_band_view route_band_layout(ImVec2 const Top, ImVec2 const Size) const;
	static std::vector<route_band_view::role> route_band_roles(alignment::result const &Result, std::size_t const Vertices, std::vector<int> &Arcs);
	// the element under the mouse and what of it a drag changes, Arcs the number of the arcs of each curve
	void route_band_pick(route_band_view &View, std::vector<route_band_view::role> const &Roles, std::vector<int> const &Arcs) const;
	// a drag in the band begun, gone on or let go of; whether it changed the design
	bool route_band_drag(route_band_view const &View, bool const Active);
	// the radius the drag of an arc in the band asks for
	double route_band_radius(route_band_view const &View) const;
	bool route_band_wheel(route_band_view const &View);
	void draw_route_band(route_band_view const &View, ImDrawList *List) const;
	void draw_route_band_labels(route_band_view const &View, ImDrawList *List) const;
	void draw_band_radius(route_band_view const &View, ImDrawList *List, alignment::element const &Element) const;
	// the length of the transition along the axis, or with the km at the bottom where none taken there is too close
	static void draw_band_length(route_band_view const &View, ImDrawList *List, alignment::element const &Element, std::vector<std::pair<float, float>> &Taken);
	void draw_route_band_value(route_band_view const &View, ImDrawList *List) const;
	void route_band_tooltip(route_band_view const &View, std::vector<int> const &Arcs) const;
	bool render_route_parameters();
	bool render_route_vertices();
	bool render_route_vertex(int const Index);
	// the part of the curve of the vertex the field just drawn is of in focus while it's edited or the mouse is over it
	void route_mark(int const Index, route_design::curve_focus::part const What, int const Arc);
	// the name of the curve of the vertex with where it lies along the line, a button to show it, how far it is from the points clicked
	bool route_vertex_heading(int const Index);
	bool route_vertex_clicks(int const Index);
	bool route_vertex_curved_ends(int const Index);
	// the radius, the cant and the transitions of the curve, two fields Half the panel wide in a row; Settled when an edit is done
	bool route_vertex_fields(int const Index, float const Half, bool &Settled);
	// the arcs of a compound curve, with their shares in the Total and their Cants as they are
	bool route_vertex_arcs(int const Index, bool &Settled);
	bool route_vertex_arc(int const Index, int const K, double const Total, std::vector<double> const &Cants, bool &Settled, int &Removed);
	bool route_arc_cant(int const Index, int const Arc, double &Cant, std::vector<double> const &Cants, bool &Settled);
	// the cant and the transitions as the norms recommend them, the radii fitted to the points clicked again after
	bool route_vertex_reshape(int const Index);
	bool route_vertex_ramps(int const Index, float const Half);
	// the cant ramp in, or Out, of the curve, as the line has it laid
	bool route_cant_ramp(int const Index, bool const Out, alignment::result::cant_ramp const *Ramp, float const Half);
	bool route_vertex_details(int const Index);
	void render_route_result();
};
