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

// how the outline widens along the track
enum class kind
{
	unified, // skrajnia budowli ujednolicona GPL: the curves of R >= 250 m are included above the lower part
	installation, // graniczna or nominalna skrajnia zabudowy: widened by 3750/R in all curves
	road // skrajnia drogi: measured from the edge of the carriageway, no widening
};

// right half of the outline on a straight track, in metres: half-width from the axis (for the roads from the edge
// of the carriageway) and height above the rail top or the road surface, listed from the bottom up; the left half
// is the mirror
struct profile
{
	std::string name;
	gauge::kind kind{kind::unified};
	struct point
	{
		double half_width{0.0};
		double height{0.0};
	};
	std::vector<point> outline;
};

// GPL-1 and GPL-2 over the limit installation gauge below 1170 mm, with and without the pantograph gauge, the limit
// and nominal installation gauges G1, G2, GA, GB, GC of the PKP PLK technical standards, and the road gauges
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
	double base{0.0}; // added to the half-width of the outline: half of the carriageway of a road
	std::array<double, 2> upper{}; // from the radius, above the lower part
	std::array<double, 2> lower{}; // from the radius, in the lower part
	std::array<double, 2> cant{}; // cant in metres, positive on the inner side of the curve and negative on the outer one
	double tilt{0.0}; // sine of the slope of the plane of the rail heads, positive with the left rail raised
};
// point of the cross-section, metres: lateral positive to the left, height above the rail top
struct place
{
	double lateral{0.0};
	double height{0.0};
};
// sections at the points of a route, from their chainage, curvature (1/m, positive turning left) and cant (m).
// The widening begins and ends ahead of the changes of the curvature and cant, as the vehicles enter a curve
std::vector<section> sections(kind Kind, std::vector<double> const &Chainage, std::vector<double> const &Curvature, std::vector<double> const &Cant);

// point of the outline on the side (+1 left, -1 right) at the section, with the widening: the cant shifts the part
// above the lower one, the lower part turns with the plane of the rail heads instead
place outline_point(profile const &Profile, section const &Section, std::size_t Index, int Side);
// distance of the point from the outline in any direction, metres, positive inside and negative outside.
// Lateral positive to the left
double intrusion(profile const &Profile, section const &Section, double Lateral, double Height);

} // namespace gauge
