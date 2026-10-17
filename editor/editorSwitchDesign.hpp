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

std::vector<design> catalogue();
// templates of the user, kept in eu07_switch_templates.txt in the configuration directory
std::vector<design> own();
bool save_own(std::vector<design> const &Designs, std::string &Error);

} // namespace switch_design
