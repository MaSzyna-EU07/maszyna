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
// widening of the gauge from its radius on the side of the other track, the widenings of both summed when both do
namespace fouling
{

// m between the axes of the tracks at the fouling point on the straight
double constexpr kNominal{3.5};

// the nearest point of the polyline in the plan; Distance: to it, Segment: the index of the point the segment it lies on
// starts from
glm::dvec3 nearest_on(std::vector<glm::dvec3> const &Line, glm::dvec3 const &Point, double &Distance, std::size_t &Segment);

// the radius in the plan of the polyline about the point at Index, 0 where it is straight; Inner: Other lies on the side of
// the centre of the curve
double radius_at(std::vector<glm::dvec3> const &Line, std::size_t const Index, glm::dvec3 const &Other, bool &Inner);

// the widening of the gauge of the polyline at Index towards Other, m: from its radius there, none on the straight
double widening_at(std::vector<glm::dvec3> const &Line, std::size_t const Index, glm::dvec3 const &Other);

struct place
{
	glm::dvec3 main{0.0}; // on the axis of the main track
	glm::dvec3 branch{0.0}; // on the axis of the branch
	glm::dvec2 back{0.0, 1.0}; // the plan direction along the branch towards the start of the switch
	double spacing{kNominal}; // m between the axes there: the nominal spacing with the widenings
};

// Main and Branch: the axes of the tracks from the start of the switch on, points a metre or so apart. Nominal: the spacing
// on the straight
bool find(std::vector<glm::dvec3> const &Main, std::vector<glm::dvec3> const &Branch, double const Nominal, place &Place);

} // namespace fouling
