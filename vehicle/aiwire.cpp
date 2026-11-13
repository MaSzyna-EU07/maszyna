/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/aiwire.h"

#include <cmath>

namespace ai_wire {

contact span_contact(glm::dvec3 const &Point1, glm::dvec3 const &Parametric, glm::dvec3 const &Point,
                     glm::dvec3 const &Front, glm::dvec3 const &Up, glm::dvec3 const &Left, double const Halfwidth) {
	// Winger 170204: the parameter of the span's point in the plane of the pantograph's movement (the plane normal to
	// Front through Point). the operations and their order are those the vehicles always had, so they find the same wire
	auto const paramfrontdot = glm::dot(Parametric, Front);
	auto const parameter = -(glm::dot(Point1, Front) - glm::dot(Point, Front)) / (paramfrontdot != 0.0 ? paramfrontdot : 0.001);

	contact result;
	if (parameter < -0.001 || parameter > 1.001) {
		return result;
	}
	// the point where the wire crosses the plane, against the pantograph
	auto const crossing = Point1 + parameter * Parametric;
	auto const offset = crossing - Point;
	result.within = true;
	result.vertical = glm::dot(offset, Up);
	result.horizontal = std::abs(glm::dot(offset, Left)) - Halfwidth;
	return result;
}

bool reaches(contact const &Contact, double const Minheight, double const Maxheight) {
	return Contact.within && Contact.horizontal <= 0.0 && Contact.vertical >= Minheight && Contact.vertical <= Maxheight;
}

double wire_shortfall(std::function<bool(double)> const &Covered, double const Pantograph, double const Stop,
                      double const Margin, double const Step, double const Walkback) {
	if (!Covered || Step <= 0.0) {
		return 0.0;
	}
	auto const fine{[&](double const Place) { return Covered(Place) && Covered(Place + Margin); }};
	// the pantograph at the target of the head
	auto const target{Stop - Pantograph};
	if (fine(target)) {
		return 0.0;
	}
	// the places back are counted from the target, not added up, so the steps don't drift
	auto const steps{static_cast<int>(std::floor(Walkback / Step + 1e-9))};
	for (int step = 1; step <= steps; ++step) {
		auto const back{step * Step};
		if (fine(target - back)) {
			return back;
		}
	}
	return 0.0;
}

bool stops_under_wire(bool const Collector, bool const Livetraction, bool const Overheadrestricted, stop_kind const Kind) {
	if (!Collector || !Livetraction || Overheadrestricted) {
		return false;
	}
	switch (Kind) {
	case stop_kind::track_end:
	case stop_kind::timetable_stop:
		return true;
	case stop_kind::signal:
	case stop_kind::coupling:
	default:
		return false;
	}
}

} // namespace ai_wire
