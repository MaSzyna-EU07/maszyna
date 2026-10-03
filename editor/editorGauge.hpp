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

// structure gauge (skrajnia budowli): the space along the track which has to stay clear of the objects
namespace gauge
{

// right half of the outline on a straight track, in metres: half-width from the axis and height above the rail top,
// listed from the bottom up; the left half is the mirror
struct profile
{
	std::string name;
	struct point
	{
		double half_width{0.0};
		double height{0.0};
	};
	std::vector<point> outline;
};

// starting points of the outlines, approximations to verify against PN-69/K-02057
std::vector<profile> default_profiles();
// outlines kept in the file, the defaults when there's none
std::vector<profile> load_profiles(std::string const &File);
void save_profiles(std::string const &File, std::vector<profile> const &Profiles);

// widening of both sides in a curve, from the radius (table 1 of the D1 annex 11), metres
double curve_widening(double Radius);
// widening of the inner side of a curve from the tilt of the vehicles on the cant, at the height above the rail top, metres
double cant_widening(double Cant, double Height);

// place of the gauge across the track
struct section
{
	double radius{0.0}; // 0: straight
	int inner{0}; // side of the curve centre, +1 left or -1 right, 0 on a straight
	double cant{0.0}; // metres
};
// half-width of the gauge at the height on the side (+1 left, -1 right), negative above or below the outline
double half_width(profile const &Profile, section const &Section, double Height, int Side);
// depth of the point inside the gauge, metres, not positive outside of it. Lateral positive to the left
double intrusion(profile const &Profile, section const &Section, double Lateral, double Height);

} // namespace gauge
