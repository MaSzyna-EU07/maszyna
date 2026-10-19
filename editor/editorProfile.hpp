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
#include <utility>
#include <vector>

// vertical alignment (grade line) of the rail top along the chainage of a route
namespace profile
{

// point of intersection of two grades, with a parabolic vertical curve of radius R (0: none)
struct pvi
{
	double chainage{0.0};
	double elevation{0.0};
	double radius{0.0};
	bool automatic{true}; // radius follows the limits
	bool joint{false}; // placed by complete(), takes the grade over from the adjoining track
};

struct limits
{
	double grade_max{6.0}; // per mille
	double grade_switch_max{0.0}; // per mille, over switches and their shared timbers; 0: the limit of the line, as a switch may lie on
	                              // the grade of the line. A lower one for the tracks of a station where vehicles stand
	double radius_factor{0.4}; // R >= k * V^2
	double radius_min{2000.0};
	double curve_threshold{2.0}; // per mille, change of grade which needs a vertical curve
	double element_min{0.0}; // m, constant grade between vertical curves
	double fixed_tolerance{0.005}; // m, allowed departure from fixed points
};

struct line
{
	std::vector<pvi> points; // ordered by chainage; the first and the last are the ends of the route
	double speed{100.0};
	limits norms;
	// at the joined ends: keep the elevation and take over the grade of the adjoining track
	bool join_start{true};
	bool join_end{true};
};

double elevation(line const &Line, double const Chainage);
// dy/ds
double grade(line const &Line, double const Chainage);
// grade of the element between points Index and Index + 1
double grade_after(line const &Line, std::size_t const Index);
double required_radius(line const &Line);
// radii of the automatic points: the required one where the change of grade needs a curve, none elsewhere
void assign_radii(line &Line);

struct curve
{
	std::size_t point{0};
	double start{0.0};
	double end{0.0};
	double tangent{0.0};
	double sagitta{0.0};
	double change{0.0}; // grade after - grade before
};
std::vector<curve> curves(line const &Line);

// radius which gives specified elevation of the curve at the point of intersection
double radius_for_sagitta(line const &Line, std::size_t const Point, double const Sagitta);

// facts about the route which the checks take into account
struct context
{
	double length{0.0};
	struct zone
	{
		double from{0.0};
		double to{0.0};
		std::string name;
	};
	std::vector<zone> switches;
	// elevations which the grade line has to keep (chainage, elevation)
	std::vector<std::pair<double, double>> fixed;
	// adjoining track at the ends; grades rise along the route
	double start_elevation{0.0}, end_elevation{0.0};
	double start_grade{0.0}, end_grade{0.0};
	bool start_joined{false}, end_joined{false};
};

// locks the joined ends, places the joint points and sets the automatic radii
void complete(line &Line, context const &Context);

struct issue
{
	bool error{false};
	int point{-1};
	double chainage{0.0};
	std::string text;
};
std::vector<issue> check(line const &Line, context const &Context);

struct recognition
{
	double grade_tolerance{0.0004};
	double minimum_length{15.0};
	double sharp_gap{2.0};
	double step{0.02};
	// where the line read departs from the track by more than this, the bends of the track itself; 0 leaves the line as read
	double detail{0.0};
};
// grade line of existing track, from its elevations and grades sampled along the chainage
line recognize(std::vector<double> const &Chainages, std::vector<double> const &Elevations, std::vector<double> const &Grades, recognition const &Options);

struct ground_fit
{
	double tolerance{1.0}; // m, the ground this near the line agrees with it; farther, the line doesn't lie on it there
	double grade_max{0.0}; // per mille, the steepest grade the line may take; 0: the limit of the line
	// heights the line keeps between before it lies on the ground, as where the tracks off a switch of it lead into another line:
	// they can't climb more than their length lets them
	struct bound
	{
		double chainage{0.0};
		double low{0.0};
		double high{0.0};
	};
	std::vector<bound> bounds;
};
// grade line which lies on the ground where it agrees, within the limits of Settings, with the fewest and least changes of
// grade; where the ground parts from it, at a bridge, a viaduct, a cutting wall, the line goes on past it. Chainages ascend,
// Ground is NaN where there's none. Each sample may have its own tolerance, how closely it tells the height, and its own
// reach, how far from the line it still belongs to it (empty: the tolerance of Options). The joined ends of the route keep
// the elevation of the adjoining track, and its grade where the line can take it over
line fit_ground(std::vector<double> const &Chainages, std::vector<double> const &Ground, context const &Context, line const &Settings, ground_fit const &Options, std::vector<double> const &Tolerances = {},
                std::vector<double> const &Reaches = {});
// stretches where the ground departs from the line by over Tolerance, one way all along: depth below the line (a gap, under
// a bridge, a viaduct), above it (a wall, a structure over the track)
struct departure
{
	double from{0.0};
	double to{0.0};
	double depth{0.0}; // m, the ground against the line where they part the most
};
std::vector<departure> departures(line const &Line, std::vector<double> const &Chainages, std::vector<double> const &Ground, double const Tolerance);

// chainages where the paths have to end so that each of them carries a single grade or a single curve
std::vector<double> breaks(line const &Line);

std::string format_chainage(double const Chainage);

}
