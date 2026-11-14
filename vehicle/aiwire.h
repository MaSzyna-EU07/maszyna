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

#include <glm/glm.hpp>

// where the overhead contact wire is, for the AI driver to stop its train with the pantograph under the wire, not past
// the end of it. the geometry of a pantograph against a span of the wire is the one the vehicles use to find their wire
// (basic_cell::update_traction), so the AI sees the wire where the pantographs do
namespace ai_wire {

// a pantograph against one span of the wire
struct contact {
	// the plane of the pantograph's movement (across the vehicle, at the pantograph) cuts the span, between its ends
	bool within{false};
	// height of the wire above the pantograph's base point, along the up vector. negative for a wire below it
	double vertical{0.0};
	// sideways distance of the wire from the working part of the slide: |offset along the left vector| less the slide's
	// half width. the wire is on the slide when it's not positive
	double horizontal{0.0};
};

// the pantograph with its base point at Point, on a vehicle whose front, up and left vectors are Front, Up and Left, and
// a slide of half width Halfwidth, against the span from Point1 along Parametric (the span's other end is Point1 +
// Parametric). for a span parallel to the plane of the pantograph's movement the distance to the plane is divided by
// 0.001 in place of 0 (the division by zero trap the vehicles always had), which puts the crossing outside the span
// unless the span lies in the plane
contact span_contact(glm::dvec3 const &Point1, glm::dvec3 const &Parametric, glm::dvec3 const &Point,
                     glm::dvec3 const &Front, glm::dvec3 const &Up, glm::dvec3 const &Left, double Halfwidth);

// the span is a wire the pantograph can reach: it crosses the plane of the pantograph's movement on the slide, between
// Minheight and Maxheight above the pantograph's base point. the heights keep out the wires of the bridges above the
// line and of the lines below it
bool reaches(contact const &Contact, double Minheight, double Maxheight);

// the default margin of wire ahead of the pantograph at a stop, the step and the farthest distance back the stop is
// looked for under the wire, in metres
double const stop_margin{3.0};
double const stop_step{2.0};
double const stop_walk_back{300.0};

// how far short of its target a train has to stop so its pantograph stays under the wire, in metres along the route.
// the distances are along the route from the head of the train: Stop the target of the head, Pantograph the pantograph
// the train relies on (the rearmost one it raises) behind the head. Covered tells whether there's a wire over the
// route at the given distance. the pantograph at the stop is fine when there's wire over it and Margin ahead of it;
// otherwise it's moved back by Step at a time, up to Walkback, to the first place it's fine at. 0 when it's fine at the
// target, and when no place within Walkback is (not an electrified stretch, or a gap too long to tell about: stopping
// short won't help there)
double wire_shortfall(std::function<bool(double)> const &Covered, double Pantograph, double Stop,
                      double Margin = stop_margin, double Step = stop_step, double Walkback = stop_walk_back);

// what makes the train stop at a place
enum class stop_kind {
	track_end, // the end of the track, e.g. a buffer stop, in train or shunting mode
	timetable_stop, // a stop of the timetable (W4 indicator)
	signal, // a signal at stop
	coupling, // a vehicle to couple to or to uncouple from
};

// the train stops with its pantograph under the wire. not when it runs without the wire: without a current collector,
// with the voltage made up (livetraction off), on a stretch the scenery tells to coast or to run with the pantographs
// down. not for a signal (a wait, the wire ending before a signal is a fault of the scenery) and not for a coupling
// (the vehicle to couple to may stand on a siding without the wire)
bool stops_under_wire(bool Collector, bool Livetraction, bool Overheadrestricted, stop_kind Kind);

} // namespace ai_wire
