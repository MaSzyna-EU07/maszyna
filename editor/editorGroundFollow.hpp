/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <vector>
#include <glm/glm.hpp>

#include "editor/editorProfile.hpp"
#include "world/Segment.h"

namespace ground_follow
{

using bed_query = std::function<std::vector<double>(std::vector<glm::dvec3> const &, std::vector<glm::dvec2> const &)>;

struct ends
{
	double height0{0.0};
	double grade0{0.0};
	double height1{0.0};
	double grade1{0.0};
};

struct knot_line
{
	std::vector<double> knots;
	std::vector<double> heights;
	std::vector<double> slopes;
	double grade0{0.0};
	double grade1{0.0};

	void refresh();
	double at(double const Chainage, double &Grade) const;
};

// the grade line the track is laid on where there's ground under it: the solver of the grade lines fits it to the ground
struct grading
{
	profile::line settings; // the speed and the norms of the line
	profile::ground_fit fit; // how closely it keeps to the ground
	double reach{100.0}; // m, how far the ground may part from the line and still be followed
};

// where Bed finds no ground, the heights lead from one end to the other
void follow(std::vector<segment_data> &Pieces, ends const &Ends, bed_query const &Bed, grading const &Grading = {});

} // namespace ground_follow
