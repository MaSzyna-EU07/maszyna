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
	// the humps of a marshalling yard: the switches of their fans lie on curves sharper than anywhere else
	std::vector<zone> humps;
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

// the hump of a marshalling yard: a crest with the fan of the switches of the sorting tracks past it, the ground falling away from
// it both ways, one of them more steeply than a line or a station is laid, and the grade changing across it more sharply than
// any vertical curve of the norms bends. Nothing else has all of it
struct hump
{
	double from{0.0};
	double crest{0.0};
	double to{0.0};
	double height{0.0}; // m, of the crest over the lower of its feet
};
// the humps of the ground; the samples which tell the height more closely than Tolerance, the rail tops beside, don't count
std::vector<hump> humps(std::vector<double> const &Chainages, std::vector<double> const &Ground, context const &Context, double const Tolerance, std::vector<double> const &Tolerances = {});

struct ground_fit
{
	double tolerance{1.0}; // m, the ground this near the line agrees with it; farther, the line doesn't lie on it there
	double grade_max{0.0}; // per mille, the steepest grade the line may take; 0: the limit of the line
	// the ground parting from the line longer than this, by less than the height and nowhere steeper than the steepest, is no
	// structure: no bridge or viaduct with the banks and the walls under it, no hill over a tunnel, but a hump, a rise, a
	// hollow. The line follows it, as closely as the tolerance. 0: the line goes past whatever it parts from
	double follow_length{0.0}; // m
	double follow_height{0.0}; // m
	double follow_steepest{0.0}; // per mille
	double follow_tolerance{0.0}; // m, 0: the tolerance
	// on the humps the line follows the ground all along, as closely as the follow tolerance, and its vertical curves are as sharp
	// as the ground bends, down to the hump radius. Nowhere else is a curve sharper than the norms require
	std::vector<hump> humps;
	double hump_radius{0.0}; // m, 0: the required one
	// heights the line keeps between before it lies on the ground, as where the tracks off a switch of it lead into another line:
	// they can't climb more than their length lets them. A yielding one, told over tracks which still bend, gives way to the
	// curves of the norms, if to no ground
	struct bound
	{
		double chainage{0.0};
		double low{0.0};
		double high{0.0};
		bool yielding{false};
	};
	std::vector<bound> bounds;
	// the joined ends keep the grade of the adjoining track; false: their height only, the grade breaks at the joint
	bool hold_grades{true};
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
// stretches a track goes over on a bridge or a viaduct the ground doesn't have: the ground under the heights of the track by over
// Depth, with a bank about them steeper than Steepest (per mille), as it falls by the abutments into what the bridge spans
std::vector<departure> bridged(std::vector<double> const &Chainages, std::vector<double> const &Heights, std::vector<double> const &Ground, double const Depth, double const Steepest);

// chainages where the paths have to end so that each of them carries a single grade or a single curve
std::vector<double> breaks(line const &Line);
// of the breaks those the paths between Joints can't do without: a path from one joint or break kept to the next follows the line
// within Tolerance with the heights and the grades of its ends
std::vector<double> fewest_breaks(line const &Line, std::vector<double> Joints, double const Tolerance);

std::string format_chainage(double const Chainage);

}
