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
};

// Main, Branch: the tracks of the switch from its start, the stations from 0 there; Line: the track it's laid into, From and To the
// stations of the start and the end of the switch along it, From after To where it goes back along the line. Radius and Ratio:
// of the diverging track of the basic switch it's bent from
switch_report check(std::vector<track_point> const &Main, std::vector<track_point> const &Branch, std::vector<track_point> const &Line, double const From, double const To, double const Radius,
                    double const Ratio);

// the start of a switch Length long, going Direction along the line, drawn to where its start or its end meets an end of a cant ramp,
// or to where it keeps the 6 m off it, when the station pressed is within Reach of that; What receives which of them, empty for none
double snapped_start(std::vector<double> const &Ramps, double const Pressed, double const Length, int const Direction, double const Reach, std::string *What = nullptr);

} // namespace switch_curve
