/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

// a switch laid into a curve or into a transition curve, as the standard of PKP PLK ST-T1-A6 sees it: the curvatures of its tracks
// (9.7 pkt 12, 11.4, table 11.9) and the cant along it, which is to stay the same over it or be ramped from its ends (9.6 pkt
// 16-17, 11.4 pkt 7, table 9.6)
namespace switch_curve
{

// a point of a track in the plan: how far along it, which way it heads (radians, growing as it turns to the left), its cant (mm)
struct track_point
{
	double station{0.0};
	double heading{0.0};
	double cant{0.0};
};

// the curvature of the track at the station, from its headings a metre either side of it, 1/m positive to the left
double curvature_at(std::vector<track_point> const &Track, double const Station);

// the stations where the cant of the track starts or stops changing: the ends of its cant ramps
std::vector<double> ramp_ends(std::vector<track_point> const &Track);

struct switch_report
{
	// at the start and at the end of the switch, 1/m, positive turning the way the line does
	double main_start{0.0};
	double main_end{0.0};
	double branch_start{0.0};
	double branch_end{0.0};
	// of the main track, mm
	double cant_start{0.0};
	double cant_end{0.0};
	bool curved{false}; // the main track bent
	bool transition{false}; // its curvature changing along the switch
	std::vector<std::string> notes;
	std::vector<std::string> warnings;
	// the hard rules broken (11.4 pkt 4 and 7, 9.6 pkt 16, 9.7 pkt 12): the switch isn't laid nor moved so
	std::vector<std::string> refusals;
};

// Main, Branch: the tracks of the switch from its start, the stations from 0 there; Line: the track it's laid into, From and To the
// stations of the start and the end of the switch along it, From after To where it goes back along the line. Radius and Ratio:
// of the diverging track of the basic switch it's bent from; Straight: how far both tracks keep straight at the end of the switch, over
// a straight frog, the switch bent up to it as the DB bends such switches; Double: a double switch, which isn't bent (11.4 pkt 4a)
switch_report check(std::vector<track_point> const &Main, std::vector<track_point> const &Branch, std::vector<track_point> const &Line, double const From, double const To, double const Radius,
                    double const Ratio, double const Straight = 0.0, bool const Double = false);
// a switch laid into a straight: the cant of the line over it alone, as check() judges it
switch_report check_line(std::vector<track_point> const &Line, double const From, double const To);

// the switches bent one-sided from the basic ones, table 11.9: the radius of the basic switch, the limiting curvature of the diverging
// track (1/m) and the least radius of the main track (m); the values in its brackets are allowed only by way of exception, after the
// agreement of the Zaklad Linii Kolejowych, the same as the others where it has none
struct bent_limit
{
	double radius;
	double branch;
	double main;
	double branch_exception;
	double main_exception;
};
std::optional<bent_limit> bent_limit_of(double const Radius);

// the radius of the diverging track of a switch bent from the basic one of Radius and 1:Ratio, its main track bent to Main, table 11.8:
// Main above 0 bent the way the diverging track turns off it (one-sided), below 0 the other way (two-sided), 0 not bent. The tangent of
// the diverging curve of the basic switch, t = r tan(a/2), corrects the plain sum of the curvatures. Above 0 the diverging track turns
// off the way it does in the basic switch, below 0 the other way (bent past straight by a main track sharper than its own curve), 0 for
// a straight one
double bent_radius(double const Main, double const Radius, double const Ratio);

// the cant deficiency over a switch, table 9.7, mm: recommended (P0), normal (P1) and the limit (P2); of a fixed frog up to 160 km/h,
// of a movable nose up to 230 km/h, which take in any diverging track of the catalogue
struct deficiency
{
	double recommended;
	double normal;
	double limit;
};
deficiency switch_deficiency(bool const Movable);

// the speed with the cant deficiency I up to Deficiency (mm) on a track of Radius (m) under the Cant (mm, above 0 leaning into its
// turn, below 0 against it), formula 9.9 the other way round: V = sqrt((I + D) R / 11.8), km/h. Infinite for a straight, 0 where the
// cant against the turn takes up all the deficiency already
double curve_speed(double const Radius, double const Cant, double const Deficiency);

// a slip switch bent into a curve, as ST-T1-A6 sees it: none is bent (11.4 pkt 4b, the editor telling no main track from a side
// one), the curvature of each track through it of one sign all along it (9.7 pkt 12), its slip curves no sharper than a side track may
// be (table 9.2), the cant as over a switch bent, none recommended in a side track (9.3 pkt 5)
struct slip_report
{
	// of the two tracks through the slip in its middle, 1/m, positive turning the way the line does
	double first{0.0};
	double second{0.0};
	// of the slip curves at their sharpest, 1/m
	std::vector<double> curves;
	// of the line, mm
	double cant_start{0.0};
	double cant_end{0.0};
	bool curved{false}; // the tracks through it bent
	std::vector<std::string> notes;
	std::vector<std::string> warnings;
	// as of a switch: a slip isn't bent (11.4 pkt 4b)
	std::vector<std::string> refusals;
};

// Tracks: the two tracks through the slip, Curves: its slip curves, each from its start, the stations from 0 there; Line as for a switch,
// From and To the stations of the ends of the first track through the slip along it
slip_report check_slip(std::vector<std::vector<track_point>> const &Tracks, std::vector<std::vector<track_point>> const &Curves, std::vector<track_point> const &Line, double const From, double const To);

// a line laid on from the end of the diverging track of a switch, as ST-T1-A6 sees the sudden changes of the cant deficiency right behind
// the switch (9.11): Curvature the switch ends with (1/m, positive to the left as the line runs on), Straight metres before its end
// over a straight frog, then the pieces of the line, each its length and its curvature at its start and at its end. The first two
// sudden changes nearer than the intermediate element of table 9.16 taken together, else each one alone, against table 9.15 for track
// connections at the Speed (km/h); Gauge between the axes of the rails, mm
struct exit_report
{
	double apart{-1.0}; // m, from the end of the curve of the switch to the next sudden change, -1 none
	double change{0.0}; // mm, the largest sudden change of the cant deficiency judged
	double allowed{0.0}; // mm, table 9.15
	double element{0.0}; // m, the intermediate element of table 9.16
	std::vector<std::string> warnings;
};
exit_report check_exit(double const Curvature, double const Straight, std::vector<std::array<double, 3>> const &Line, double const Speed, double const Gauge = 1500.0);

// the start of a switch Length long, going Direction along the line, drawn to where its start or its end meets an end of a cant ramp,
// or to where it keeps the 6 m off it, when the station pressed is within Reach of that; What receives which of them, empty for none
double snapped_start(std::vector<double> const &Ramps, double const Pressed, double const Length, int const Direction, double const Reach, std::string *What = nullptr);

} // namespace switch_curve
