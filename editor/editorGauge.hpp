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
#include <string>
#include <vector>

// structure gauge (skrajnia budowli) of the PKP PLK technical standards, volume II: the space along the track
// which has to stay clear of the objects
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

// GPL-1 and GPL-2 over the limit installation gauge below 1170 mm, with and without the pantograph gauge
std::vector<profile> default_profiles();
// outlines kept in the file, the defaults when there's none
std::vector<profile> load_profiles(std::string const &File);
void save_profiles(std::string const &File, std::vector<profile> const &Profiles);

// the points of the paths in the scenery lie at the rail foot, the gauge is measured from the rail head
double constexpr rail_height{0.18};

// below this height the outline is the limit installation gauge, which leaves the room for the platforms
double constexpr lower_part{1.17};

// widening of the sides of the gauge at a place of the track, metres; index 0 is the right side, 1 the left one
struct section
{
	std::array<double, 2> upper{}; // from the radius, above the lower part
	std::array<double, 2> lower{}; // from the radius, in the lower part
	std::array<double, 2> cant{}; // cant in metres, positive on the inner side of the curve and negative on the outer one
};
// sections at the points of a route, from their chainage, curvature (1/m, positive turning left) and cant (m).
// The widening begins and ends ahead of the changes of the curvature and cant, as the vehicles enter a curve
std::vector<section> sections(std::vector<double> const &Chainage, std::vector<double> const &Curvature, std::vector<double> const &Cant);

// widening of the side (+1 left, -1 right) at the height
double widening(section const &Section, double Height, int Side);
// half-width of the gauge at the height on the side, negative above or below the outline
double half_width(profile const &Profile, section const &Section, double Height, int Side);
// depth of the point inside the gauge, metres, not positive outside of it. Lateral positive to the left
double intrusion(profile const &Profile, section const &Section, double Lateral, double Height);

} // namespace gauge
