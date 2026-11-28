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
#include <cstddef>
#include <optional>
#include <utility>

// a switch thrown under a train (forked running, "jazda widłowa"). the wheels past the tips of the blades stay on the
// way they ran onto, those that come to the points after the throw take the other way: the train is split between the
// two ways of the switch. an axle (TTrackFollower) keeps the way of the switch it ran onto and leaves the switch at the
// end of that way, not at the end of the way the switch is set to now; the axles behind it take the way set now. where
// two axles that follow each other in the train took two ways, the vehicle between them, or the coupling of two
// vehicles, runs forked, and it derails once the ways are too far apart for it (McZapkie/derailment.h, physics.derailment
// new), or the log tells (monitor)
namespace switch_fork {

// ---- the way of a switch an axle keeps

// the way to leave a switch by [0, 1]: the axle ran onto the Path, the switch is set to the Set one now. -1 while the
// switch is set the way the axle ran onto it: it is left as it is set, as it always was
int leaving_path(int Path, int Set);

// the ways of the switches an axle ran over last, the newest last. the switch is a key only, never read
class path_memory {
public:
	static constexpr std::size_t size{4};
	// the axle ran onto the switch, the way Path; an older record of the switch is dropped
	void record(void const *Switch, int Path);
	// the way the axle ran over the switch the last time, none if it isn't remembered
	std::optional<int> path_of(void const *Switch) const;
	void clear() { m_count = 0; }
	std::size_t count() const { return m_count; }

private:
	std::array<std::pair<void const *, int>, size> m_entries{};
	std::size_t m_count{0};
};

// an axle ran onto a switch from its points and took the Path. the axle ahead of it in the train (the other one of the
// vehicle, or the nearest one of the vehicle coupled ahead) ran over the same switch by the Ahead way, none if it
// didn't (or it isn't known): the two run on two ways, forked
bool forked(int Path, std::optional<int> Ahead);

// ---- how much forked running a vehicle takes

// the tightest curve a rail vehicle runs through [m], when its definition gives none (Wheels: Rmin): what every rail
// vehicle has to run through, TSI LOC&PAS (Regulation (EU) No 1302/2014) 4.2.3.6
inline constexpr double minimum_curve{150.0};
// the angle [rad] a bogie turns under the body by in the tightest curve the vehicle runs through, the Radius [m], with
// the bogie pivots (or the axles of a vehicle without bogies) the Pivots [m] apart: asin(a / 2R). in a curve the two bogies turn the opposite ways; forked, both
// turn the same way, by the angle of the body to the two tracks. the bogie isn't built to turn farther, its frame or the
// stops of the secondary suspension meet the body, and the body turns the wheelsets against the rails: the flange of
// the leading one climbs
double bogie_limit(double Pivots, double Radius = minimum_curve);
// the buffer heads of coaches and locomotives are 450 mm wide (UIC 527-1, EN 15551): two coupled ends further apart
// sideways than that have no buffer face against the other one, they lock or ride past each other, and the screw
// coupling pulls the end of the vehicle across [m]
inline constexpr double buffer_head_width{0.45};

// the angle [rad] the line of a track, the heading TrackHeading [rad], is turned by from the body along the heading
// BodyHeading, either way round (-pi/2, pi/2]: positive the way the heading grows. a track runs either way, so the
// angle is taken between the two lines, not between the two directions
double turn(double TrackHeading, double BodyHeading);

// the sideways distance [m] between the ends of two coupled vehicles, of the coupler at (x, z) and the one at
// (OtherX, OtherZ), across the mean of the directions of the two vehicles (the headings Heading and OtherHeading [rad],
// the lines either way round)
double end_offset(double X, double Z, double Heading, double OtherX, double OtherZ, double OtherHeading);

// the vehicle, or the coupling of two vehicles, running forked: what it went by and its limit
struct state {
	double value{0.0}; // how far the bogie turned [rad], or the ends are apart [m]
	double limit{0.0};
	bool past() const { return value > limit; }
};
// the bogies the Pivots [m] apart, turned by Front and Rear [rad] (turn()) under the body, of a vehicle built for curves
// down to the Radius [m]
state bogies(double Front, double Rear, double Pivots, double Radius = minimum_curve);
// the coupled ends the Offset [m] apart sideways (end_offset())
state coupling(double Offset);

// a heading (x, z) of the plane of the scenery as an angle [rad], the way DynObj and TTrackFollower have it:
// atan2(-x, z)
double heading(double X, double Z);

} // namespace switch_fork
