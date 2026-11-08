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

// the line laid along the points clicked: its straights and the curves fitted to the stretches of points traced along them
class lay_tracer final : public editor_tool
{
  public:
	lay_tracer(editor_build_services &Build, build_shared &Shared) : m_build{Build}, m_lay{Build.lay_state()}, m_route{Build.route_state()}, m_fits_waiting{Shared.fits_waiting} {}
	// Adjoining: the curve next to a free end on a curve goes on from its curvature, where it turns its way; else from a straight there
	bool lay_line(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, editor_track::snap_target const &End, lay_line_plan &Plan, std::string &Error, bool const Adjoining = true) const;
	// the curve through a row of bends clicked close along one, by traced_stretch
	alignment::stretch_fit bends_fit() const
	{
		return [this](alignment::design const &Base, std::vector<glm::dvec2> const &Points) { return traced_stretch(Base, Points, 0, false, false); };
	}
	// the curves fitted lately forgotten, and the turn of the curve at the cursor tried first
	void forget_stretches()
	{
		m_stretch_cache.clear();
	}
	void forget_turn()
	{
		m_turned_last = 0.0;
	}

  private:
	editor_build_services &m_build;
	lay_tool &m_lay;
	route_design &m_route;
	bool &m_fits_waiting;
	// the curves fitted lately, kept for the same points; the fits are made in const methods, so the caches are mutable
	mutable std::vector<std::pair<std::vector<double>, alignment::trace_stretch>> m_stretch_cache;
	// the stretch the cursor moves along is fitted on a thread, the frames drawn meanwhile with the fit done last
	mutable alignment::stretch_fitter m_stretch_fitter;
	// how much more the curve at the cursor was turned last to reach it, the first one tried for the next line
	mutable double m_turned_last{0.0};
	// the parts of the line, on its state and a stretch of points along a curve, defined where they're used: the points gone through, the curve
	// fitted to a stretch of them, the line closed at its end; the fit of a stretch leaving that way, going on from the curve of the free
	// end, from a straight at a switch, into the curve of the free end at the end, its end turned onto the straight clicked after it or
	// to reach the last point
	struct lay_line_state;
	struct lay_stretch;
	// whether the points next to the start of the line, or to its end, AtEnd, a free end joined there, are clicked along a curve
	static bool traced_from(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, bool const AtEnd);
	bool lay_line_points(lay_line_state &State) const;
	bool lay_line_stretch(lay_line_state &State, std::size_t const First, std::size_t const Last, bool const Reaching, bool const Fromend) const;
	bool lay_stretch_fit(lay_line_state &State, lay_stretch &Stretch) const;
	std::optional<alignment::trace_stretch> stretch_fit_to(lay_stretch const &Stretch, glm::dvec2 const &Leaving, bool const Meanwhile, bool &Exact) const;
	std::optional<alignment::trace_stretch> stretch_going(lay_line_state &State, lay_stretch &Stretch) const;
	bool straight_first_closer(lay_stretch const &Stretch, alignment::trace_stretch const &Going) const;
	std::optional<alignment::trace_stretch> stretch_into(lay_stretch &Stretch) const;
	void stretch_end_turn(lay_stretch &Stretch, std::optional<alignment::trace_stretch> &Fitted) const;
	std::optional<std::pair<alignment::trace_stretch, glm::dvec2>> clicked_straight(lay_stretch const &Stretch, bool &Waiting) const;
	std::optional<alignment::trace_stretch> heading_fit(lay_stretch const &Stretch, glm::dvec2 const &Heading, bool &Waiting) const;
	void closest_heading(lay_stretch const &Stretch, glm::dvec2 const &Chorded, double const Step, std::optional<alignment::trace_stretch> &Fit, glm::dvec2 &Heading, bool &Waiting) const;
	void turn_to_point(lay_stretch &Stretch, std::optional<alignment::trace_stretch> &Fitted, double const First, double const Ending) const;
	bool lay_line_close(lay_line_state &State) const;
	// the curves of a stretch traced, the ones fitted lately kept for the same points
	// none while the stretch the cursor moves along is still being fitted; Meanwhile: the fit done last of the same points but the one
	// at the cursor until then
	// Exact receives whether the fit is the one of these points, not one shown meanwhile; Going: the curve going on from the curve of the
	// free end Base starts at, alignment::fit_going_on
	std::optional<alignment::trace_stretch> traced_stretch(alignment::design const &Base, std::vector<glm::dvec2> const &Points, int const Radii, bool const Straights, bool const Meanwhile = true,
	                                                       bool *Exact = nullptr, bool const Going = false) const;
};
