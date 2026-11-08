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
class extend_editor;
class lay_tracer;
class nmt_auto_fit;
class nmt_panel;
class straights_editor;

// the track laid through the points clicked worked out: the preview at the cursor, the pieces of the line, its speed and its norms, the
// notes on it and the wheel changing its radius
class lay_planner final : public editor_tool
{
  public:
	lay_planner(editor_services &Services, editor_build_services &Build, build_shared &Shared, extend_editor &Extender, lay_tracer &Tracer) : m_services{Services}, m_build{Build}, m_extender{Extender}, m_tracer{Tracer}, m_track_edits{Build.track_edits()}, m_lay{Build.lay_state()}, m_extend{Build.extend_state()}, m_route{Build.route_state()}, m_straights{Build.straight_lines()}, m_fits_waiting{Shared.fits_waiting} {}
	void update_lay_preview(glm::dvec3 const &Ground, glm::vec2 const &Cursor, bool const Pointing);
	// the layout laid on from the end of From with the transition curves of the norms; none where it doesn't fit
	std::optional<alignment::lay_result> eased_layout(TTrack const *From, glm::dvec3 const &Point, glm::dvec2 const &Direction, glm::dvec3 const &Target, double const Radius) const;
	void give_own_speed(std::vector<TTrack *> const &Created) const;
	// the track through the points clicked, each the way its mode says; End: the free end it finishes at, the last point then
	std::vector<segment_data> lay_pieces(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, editor_track::snap_target const &End, double &Length, std::string &Error,
	                                     lay_line_plan *Plan = nullptr) const;
	std::vector<segment_data> join_pieces(TTrack const *From, editor_track::point_ref const &Frompoint, glm::dvec3 const &Start, glm::dvec2 const &Direction, editor_track::snap_target const &End, double &Length,
	                                      std::string &Error) const;
	double design_speed(TTrack const *From, TTrack const *To) const;
	// the tracks the laying under way goes on from and to
	std::pair<TTrack const *, TTrack const *> laying_ends() const;
	std::string design_speed_of(TTrack const *From, TTrack const *To) const;
	// what ST-T1-A6 says of the sudden changes of the cant deficiency right behind a switch the pieces laid come out of at Start (From),
	// or go into at End (To): of the end of its curve and the first sudden change of the pieces (9.11, tables 9.15, 9.16)
	std::vector<std::string> switch_exit_warnings(TTrack const *From, glm::dvec3 const &Start, TTrack const *To, glm::dvec3 const &End, std::vector<segment_data> const &Pieces) const;
	alignment::limits laying_norms() const;
	// the points the same in number as their modes, a point with none a bend of straights
	void lay_trim();
	// the straight track Alt turns the laid straight to: the nearest one the straight from From to To crosses, or one along From
	TTrack const *skew_reference(glm::dvec3 const &From, glm::dvec3 const &To, double const Reach) const;
	track_hover split_spot() const;
	int wheel_step(double const Offset, bool const Gesture);
	void wheel_speed(int const Step);
	void wheel_radius(bool const Joined, int const Step);
	// the notes told of the track laid; the compound curve it makes with the tracks of the free ends
	void lay_notes(std::vector<TTrack *> const &Created, std::vector<segment_data> const &Pieces, editor_track::snap_target const &End, lay_line_plan const &Plan,
	               std::optional<alignment::lay_result> const &Eased, std::string const &Error, TTrack const *Style);

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	extend_editor &m_extender;
	lay_tracer &m_tracer;
	int const &m_track_edits;
	lay_tool &m_lay;
	extend_tool &m_extend;
	route_design &m_route;
	straights_state &m_straights;
	bool &m_fits_waiting;
	// the curves eased lately, kept for the same request; worked out in const methods, so the cache is mutable
	mutable std::optional<std::pair<alignment::lay_request, std::optional<alignment::lay_result>>> m_eased_cache;
	// a touchpad sends fractions of a notch: a choice moves a step for each whole notch gathered, and only one in a gesture when
	// Gesture; a pause, another key or the other way starts over
	struct wheel_state
	{
		double rest{0.0};
		double time{-10.0};
		int keys{0};
		bool spent{false};
	};
	wheel_state m_wheel;
	void lay_settle();
	void lay_work_out(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, editor_track::snap_target const &End);
	// the track whose speed the design takes, none where it's the one set for the laying
	TTrack const *speed_track(TTrack const *From, TTrack const *To) const;
	// the parts of it: the track from a point to a free end, the second point; the heights of the pieces and the cant at the free ends
	std::vector<segment_data> lay_to_end(std::vector<glm::dvec3> const &Points, editor_track::snap_target const &End, double &Length, std::string &Error) const;
	glm::dvec3 lay_angle_snap(glm::dvec3 const &Mouse, std::string &Label) const;
	void compound_note(std::vector<TTrack *> const &Created, editor_track::snap_target const &End);
	void lay_heights(std::vector<segment_data> &Pieces, std::vector<glm::dvec3> const &Points, editor_track::snap_target const &End, lay_line_plan const &Plan) const;
};

// the laying of the track driven: the points clicked, taken back and finished, the keys, and the panel of the tool
class lay_controls final : public editor_tool
{
  public:
	lay_controls(editor_services &Services, editor_build_services &Build, lay_planner &Planner, extend_editor &Extender, nmt_auto_fit &Nmt_auto, nmt_panel &Nmt_window, straights_editor &Straight_edit) : m_services{Services}, m_build{Build}, m_planner{Planner}, m_extender{Extender}, m_nmt_auto{Nmt_auto}, m_nmt_window{Nmt_window}, m_straight_edit{Straight_edit}, m_nmt_state{Build.terrain_state()}, m_lay{Build.lay_state()} {}
	// Tab: the bends of straights or the curve traced, for the points clicked from now on
	void lay_switch_mode();
	void lay_click();
	void lay_finish(editor_track::snap_target const &End);
	void lay_enter();
	void lay_cancel();
	void lay_escape();
	void lay_after_history(bool const Undo);
	void lay_back();
	void render_lay_ui();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	lay_planner &m_planner;
	extend_editor &m_extender;
	nmt_auto_fit &m_nmt_auto;
	nmt_panel &m_nmt_window;
	straights_editor &m_straight_edit;
	nmt_state &m_nmt_state;
	lay_tool &m_lay;
	std::size_t lay_pending() const; // the points clicked and not laid yet
	// the parts of it: the points not laid yet, the settings of the mode, the curves of the line shown, the settings folded away, a
	// texture of the new track
	void render_lay_pending();
	void render_lay_shape();
	void render_lay_curves() const;
	void render_lay_more();
	void render_texture_row(int const Kind, char const *Label, std::string &Value, float const Column);
	void render_lay_status() const;
};
