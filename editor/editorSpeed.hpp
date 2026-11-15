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
#include <vector>
#include <glm/glm.hpp>

#include "editor/editorAlignment.hpp"

class TTrack;

// speed the geometry of the paths allows, set against the speed limits written in them
namespace speed_check
{

enum class cause
{
	none,
	radius, // unbalanced acceleration in the curve
	cant_rate, // cant changes too fast for the passengers
	cant_ramp, // the cant ramp is too steep for the wheels
	jerk, // the unbalanced acceleration changes too fast, curves without the transitions
	branch // diverging track of a switch, without cant
};
char const *describe(cause const Cause);

struct options
{
	alignment::limits norms;
	double branch_unbalanced{0.65}; // m/s2, for the diverging tracks of the switches
	double reference_length{20.0}; // m, over which an abrupt change of the curvature or of the cant is felt
	double step{2.0}; // m, between the checked points
	double top{300.0}; // km/h, nothing above is looked into
};

struct verdict
{
	TTrack *track{nullptr};
	int path{0};
	double set{-1.0}; // km/h, -1: no limit of its own
	double safe{0.0}; // km/h, Options.top when the geometry allows any speed
	cause why{cause::none};
	glm::dvec3 position{0.0}; // where the geometry is the tightest
	double radius{0.0}; // m, there
	double cant{0.0}; // mm, there
	// over the reference length around the point: the radius and the cant at its ends, 0 m for a straight.
	// negative where the curve turns the other way than at the point
	double radius_from{0.0}, radius_to{0.0};
	double cant_from{0.0}, cant_to{0.0}; // mm
	double cant_slope{0.0}; // mm per m
	// km/h, the safe speed without the limit of the switch branches, which is only a warning
	double hard{0.0};
	cause hard_why{cause::none};
	// true when the speed set is above the safe one, or isn't set over a tight geometry; the switch branches not counted
	bool violated(options const &Options) const;
	// true when only the limit of a switch branch is exceeded
	bool warned(options const &Options) const;
};
// what limits the speed, with the numbers which do
std::string explain(verdict const &Verdict, options const &Options);

verdict check(TTrack &Track, int const Path, options const &Options);
// the safe speed rounded down to the usual steps of the speed limits
double suggested(double const Safe);
// speed limit in force on the path, -1 for none. a switch keeps one for both its tracks: positive applies to both,
// -2 and below only to the diverging one, its main track having no limit then
double path_velocity(TTrack const &Track, int const Path);
// velocity of a switch which keeps its tracks to the limits given, -1 for none. one written value can't hold
// a limit on each track, the main one gets the diverging one's limit too when that's lower
double switch_velocity(double const Main, double const Diverging);

} // namespace speed_check
