/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <vector>

#include <glm/glm.hpp>

// the fouling point of two tracks running apart from a switch, where the W17 marker stands (Standardy techniczne PKP PLK,
// tom II, 6.5-6.6): where their axes are nominally 3.50 m apart, more where a track lies there in a curve under 250 m, by the
// widening of the gauge from its radius on the side of the other track, the widenings of both summed when both do; and as
// Ie-1 wants the cant taken in, by the leaning of the cars of a track on its cant when they lean towards the other one (bD of
// the same table)
namespace fouling
{

// m between the axes of the tracks at the fouling point on the straight
double constexpr kNominal{3.5};
// m over the rail top the leaning of the cars on the cant is taken at: the top of the widest part of the cars
double constexpr kLeanHeight{3.25};

// the nearest point of the polyline in the plan; Distance: to it, Segment: the index of the point the segment it lies on
// starts from
glm::dvec3 nearest_on(std::vector<glm::dvec3> const &Line, glm::dvec3 const &Point, double &Distance, std::size_t &Segment);

// the radius in the plan of the polyline about the point at Index, 0 where it is straight; Inner: Other lies on the side of
// the centre of the curve
double radius_at(std::vector<glm::dvec3> const &Line, std::size_t const Index, glm::dvec3 const &Other, bool &Inner);

// the widening of the gauge of the polyline at Index towards Other, m: from its radius there, none on the straight, and from its
// cant there when the cars lean towards Other. Cant: m, the raise of one rail over the other, positive with the cars leaning
// to the left of the way the polyline runs
double widening_at(std::vector<glm::dvec3> const &Line, std::size_t const Index, glm::dvec3 const &Other, double const Cant = 0.0);

struct place
{
	glm::dvec3 main{0.0}; // on the axis of the main track
	glm::dvec3 branch{0.0}; // on the axis of the branch
	glm::dvec2 back{0.0, 1.0}; // the plan direction along the branch towards the start of the switch
	double spacing{kNominal}; // m between the axes there: the nominal spacing with the widenings
	double cant{0.0}; // m of it from the cant
};

// Main and Branch: the axes of the tracks from the start of the switch on, points a metre or so apart. Nominal: the spacing
// on the straight. MainCant, BranchCant: the cant at each point of them as widening_at takes it, none when empty
bool find(std::vector<glm::dvec3> const &Main, std::vector<glm::dvec3> const &Branch, double const Nominal, place &Place, std::vector<double> const &MainCant = {},
          std::vector<double> const &BranchCant = {});

} // namespace fouling
