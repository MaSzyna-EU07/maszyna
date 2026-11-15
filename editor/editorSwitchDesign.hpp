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

#include "world/Segment.h"

// switches described by their geometry, of the catalogue and made by the user in the template editor: the main track is a
// straight, the diverging track a chain of parts of linearly changing curvature from the start of the switch; a symmetric
// switch (Rls) has both tracks turning by the same parts to both sides; a double switch (Rpd to both sides, Rpj to one side)
// is two switches, the second starting on the main track of the first
namespace switch_design
{

struct part
{
	double length{0.0};
	double radius1{0.0}; // at the start of the part, 0 straight
	double radius2{0.0}; // at its end; the same as radius1 for an arc, different for a transition curve
};

struct design
{
	std::string name;
	std::string type{"Rz"}; // Rz, Rls, Rpd, Rpj
	std::string rail{"-"};
	double main{0.0}; // length of the main track, 0: as far as the diverging tracks reach
	std::vector<part> parts;
	double lead{0.0}; // the pre-blade stretch: the diverging track runs along the main one up to the tip of the blade
	double entry{0.0}; // degrees, the angle of the blade to the stock rail at its tip, 0: the diverging track leaves tangentially
	double tip{0.0};
	double heel{0.0}; // where the moving blades end, from the start of the switch, 0: as long as the flangeway behind them needs
	double bladethrow{0.0}; // how far the tips of the blades move over, 0: the usual 160 mm
	double stagger{0.0}; // of a double switch: where its second switch starts on the main track
	std::vector<part> parts2; // of the second switch, empty: the same as the first
	double lead2{-1.0}; // of the second switch, negative: the same as the first
	double entry2{-1.0};
	double tip2{-1.0};
	double a{0.0}; // as the catalogue gives them, 0: from the geometry
	double b{0.0};
	std::string note; // shown after the name on the list
	bool own{false}; // made in the template editor, kept in the file of the user
	bool movable{false}; // the frog with a movable nose
};

bool double_switch(std::string const &Type);
bool symmetric_switch(std::string const &Type);

// diverging track of the chain of parts from the start, with the first direction along +z and turning to +x
struct path
{
	segment_data whole; // the classic bezier, with the parts as its pieces when one bezier doesn't make them
	double angle{0.0}; // at the end, radians
	double radius{0.0}; // the sharpest
};
path diverging(std::vector<part> const &Parts, double Lead = 0.0, double Entry = 0.0, double Tip = 0.0);
double blade_tip(std::vector<part> const &Parts, double Lead, double Entry, double Tip, bool Symmetric);
// the tip of a blade with a cut tip from the start of its curve, as the tables of the rail give it, 0: none for the radius
double cut_tip(std::string const &Rail, double Radius);
// the pre-blade stretch and the angle in radians of blades cutting into the stock rails, as the ordinary switch of the catalogue
// with the same rail, radius and ratio has them; 0 and 0 for blades tangent to them
std::pair<double, double> secant_blade(std::string const &Rail, double Radius, double Ratio);

// the paths of the switch in its own space: the main and the diverging track of the first switch, then of the second one of a
// double switch and the plain track joining the two; empty with the reason in Error when the design can't make a switch
struct shape
{
	std::vector<segment_data> paths;
	double length{0.0};
	double ratio{0.0};
	double radius{0.0};
	double a{0.0};
	double b{0.0};
	double offset{0.0}; // of the end of the diverging track from the main one
	double stagger{0.0};
	double tip{0.0}; // of the blade, along the main track
	double tip2{0.0};
};
shape build(design const &Design, std::string &Error);
// what makes the difference between the tangents of a switch and the tangent of its arc
enum class tangent_fit
{
	straights, // a straight before the arc and after it, or the blade meeting the stock rail at an angle where a is too short
	transition, // a transition curve on the side of the longer tangent, a straight on the other one
	two_arcs, // a flatter arc on the side of the longer tangent
	arc_straight_arc // an arc, a straight, an arc: as far as it turns on the first one, as long as the straight, tell the tangents
};
// the diverging track of the design fitted to the tangents a and b, from the start of the switch to their meeting and on from there to
// its end, at the angle 1:Ratio with the sharpest radius Radius, 0 for a tangent: the tangent of the arc. False with the reason in Error
// when that doesn't make them. First: the radius of the first arc of an arc, a straight and an arc, 0: Radius
bool fit_tangents(design &Design, double Radius, double Ratio, double A, double B, tangent_fit Kind, std::string &Error, double First = 0.0);
// a switch measured from points clicked in the plan: its start, the end of its main track, the end of its diverging track, a point
// further on that track for its way on, then any number of points on the arc, of which three at least tell its radius
struct measured
{
	double a{0.0};
	double b{0.0}; // along the diverging track
	double main{0.0}; // the length of the main track, from the start to its end clicked
	double past{0.0}; // how far past the end of the main track the end of the diverging one lies along it
	double ratio{0.0};
	double radius{0.0}; // 0: not told
};
bool measure_clicks(std::vector<glm::dvec2> const &Clicks, measured &Result, std::string &Error);
// the design fitted to the points clicked: the tangents and the angle measured, of the shapes and radii the one keeping closest to the
// points along the diverging track, of those about as close the shape Kind. The radius the first arc of the shape has its own with First
struct clicks_fit
{
	tangent_fit kind{tangent_fit::straights};
	double radius{0.0};
	double worst{0.0}; // m, of the points from the track
};
bool fit_clicks(design &Design, std::vector<glm::dvec2> const &Clicks, tangent_fit Kind, double First, clicks_fit &Result, std::string &Error);

std::vector<design> catalogue();
// templates of the user, kept in eu07_switch_templates.txt in the configuration directory
std::vector<design> own();
bool save_own(std::vector<design> const &Designs, std::string &Error);

} // namespace switch_design
