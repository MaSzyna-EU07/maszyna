/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <utility>
#include <vector>
#include <glm/glm.hpp>

// The outside views of the user's train that go with one of its vehicles (eu07/maszyna#551): the orbit around a
// vehicle, the shunting view from above, the view beside a coupling and the trackside view of a railfan. Each one
// is worked out here from the place of the vehicle and the keys of the user alone, so it can be tested without the
// game; application/consistviews.cpp hands over the vehicles of the train and turns the results into the camera.
// The world has y up; the rails are at the height of the place of a vehicle (TDynamicObject::GetPosition).
namespace cameraviews {

// a vehicle of the train, as the views see it
struct vehicle_pose {
	glm::dvec3 position{}; // its middle, at the height of the rails
	glm::dvec3 front{0.0, 0.0, -1.0}; // unit, towards its front coupler
	double length{20.0};
	double width{3.0};
	double height{4.0};
};

// where the camera stands, the point it looks at and its up
struct placement {
	glm::dvec3 eye{};
	glm::dvec3 target{};
	glm::dvec3 up{0.0, 1.0, 0.0};
};

// the movement keys of the camera as TCamera::Velocity has them outside the cab: x to the right, y up, z backwards
// (the up arrow gives a negative z), 1 for a key held, 5 with ctrl
using move_keys = glm::dvec3;

// the Front of a vehicle flattened on the ground, unit; a vehicle standing upright (no heading) faces -z
glm::dvec3 flat_front(glm::dvec3 const &Front);
// to the left of the flat Front, unit
glm::dvec3 flat_left(glm::dvec3 const &Flatfront);

// the index after Step vehicles (or couplings) from Index, among Count of them, held at the first and the last
int step_index(int Index, int Count, int Step);

// the two ends of a train whose vehicles stand at Positions, going in the Direction: the index of the vehicle
// at its head (furthest along the Direction) and at its tail
std::pair<int, int> train_ends(std::vector<glm::dvec3> const &Positions, glm::dvec3 const &Direction);

// +1 when the front of the vehicle the view goes with points the way the user drives, -1 when it points back:
// the Front of that vehicle, the Front of the vehicle the user drives and its occupied cab (1 the front one,
// -1 the rear one, 0 none)
int driving_facing(glm::dvec3 const &Front, glm::dvec3 const &Drivenfront, int Cab);

// orbit: around the middle of the vehicle, the left and right keys (and the mouse) turn the camera around it,
// the up and down arrows bring it closer and further away, the page keys (and the mouse) raise and lower it
struct orbit {
	double azimuth{0.0}; // radians around the vehicle from its front, towards its left
	double elevation{0.0}; // radians above the horizon of the middle of the vehicle
	double radius{0.0}; // metres from the middle of the vehicle, 0 for the default one of the vehicle
};

inline constexpr double orbit_max_radius{250.0};
inline constexpr double orbit_max_elevation{1.4}; // radians, about 80 degrees
inline constexpr double orbit_min_elevation{-0.5}; // radians, about -30 degrees, if the rails let it
inline constexpr double rail_clearance{0.5}; // metres the camera keeps above the rails

// the first orbit around a vehicle: ahead of it on its left, a little above it, as far as the vehicle is long
orbit orbit_start(vehicle_pose const &Vehicle);
// the point the orbit goes around: the middle of the vehicle, a little lower than half of its height
glm::dvec3 orbit_centre(vehicle_pose const &Vehicle);
// the nearest the camera comes to the middle of the Vehicle: outside its body whichever way it looks
double orbit_min_radius(vehicle_pose const &Vehicle);
// the lowest elevation at the Radius which keeps the camera above the rails
double orbit_lowest_elevation(vehicle_pose const &Vehicle, double Radius);
// the Orbit held within its limits around the Vehicle (radius 0 becomes the default one)
orbit orbit_clamped(orbit Orbit, vehicle_pose const &Vehicle);
// the Orbit after Seconds of the Keys held and the mouse moved by Turn (radians: x up and down, y to the left and
// right, as TCamera::Angle takes them), within its limits around the Vehicle
orbit orbit_moved(orbit Orbit, move_keys const &Keys, glm::dvec2 const &Turn, double Seconds, vehicle_pose const &Vehicle);
// the camera of the Orbit around the Vehicle, looking at its middle
placement orbit_placement(orbit const &Orbit, vehicle_pose const &Vehicle);

// shunting view: straight down from above the vehicle, the way the user drives up the screen; it goes with the
// vehicle, the up and down arrows move it along the train, left and right across, the page keys up and down
struct topdown {
	double height{0.0}; // metres above the rails, 0 for the default one of the vehicle
	double along{0.0}; // metres from the middle of the vehicle along it, the way the user drives
	double across{0.0}; // metres from the middle of the vehicle across it, to the right of the screen
};

inline constexpr double topdown_max_height{300.0};
inline constexpr double topdown_max_across{60.0};

// the lowest the camera comes: above the roof of the Vehicle
double topdown_min_height(vehicle_pose const &Vehicle);
// the View held within its limits: Alonglimit metres along the vehicle either way
topdown topdown_clamped(topdown View, vehicle_pose const &Vehicle, double Alonglimit);
// the View after Seconds of the Keys held; the further the camera, the faster it moves
topdown topdown_moved(topdown View, move_keys const &Keys, double Seconds, vehicle_pose const &Vehicle, double Alonglimit);
// the camera of the View over the Vehicle; Facing +1 puts the front of the vehicle up the screen, -1 its rear
placement topdown_placement(topdown const &View, vehicle_pose const &Vehicle, int Facing);

// coupling view: beside a coupler, low, looking along the train at it; when another vehicle comes closer to it
// the camera steps back from the track to keep both couplers in sight, looking between them
inline constexpr double buffer_height{1.06}; // metres above the rails

struct coupler {
	glm::dvec3 position{}; // the coupler at the height of the rails (TDynamicObject::CouplerPosition)
	glm::dvec3 outward{0.0, 0.0, -1.0}; // unit, from its vehicle out through the coupler
	double width{3.0}; // of its vehicle
};

struct coupler_view {
	double side{1.0}; // 1 on the left of the outward way, -1 on the right
	double height{0.4}; // metres above the buffers
	double distance{2.5}; // metres from the side of the vehicle, before the vehicle coming closer moves it further
};

inline constexpr double coupler_min_distance{1.0};
inline constexpr double coupler_max_distance{40.0};
inline constexpr double coupler_min_height{-0.8}; // metres above the buffers: a little above the rails
inline constexpr double coupler_max_height{10.0};
inline constexpr double coupler_watch_range{150.0}; // metres to a vehicle that still counts as coming closer

// the View after Seconds of the Keys held: left and right move it to that side of the track, the arrows closer
// and further, the page keys up and down
coupler_view coupler_moved(coupler_view View, move_keys const &Keys, double Seconds);
// the camera beside the Coupler; Other is the facing coupler of the vehicle coming closer, when there is one
placement coupler_placement(coupler_view const &View, coupler const &Coupler, glm::dvec3 const *Other);
// the coupling of a train of Count vehicles the view starts at, numbered from 0 (its front end) to Count (its rear
// end): the end with another vehicle, not coupled, closer than coupler_watch_range (the closer one of the two),
// otherwise the front. Frontgap and Reargap are the distances to such vehicles, negative for none
int first_coupling(double Frontgap, double Reargap, int Count);

// trackside view: a railfan standing beside the line ahead of the train, looking at it as it comes and goes by;
// once the whole train has gone past (or gone far away) the railfan walks to a new spot ahead of it, on the other
// side of the line
struct trackside {
	glm::dvec3 spot{}; // the eye of the railfan
	double side{-1.0}; // 1 on the left of the line, -1 on the right
	bool placed{false};
};

// the train as the trackside view sees it
struct train_motion {
	glm::dvec3 head{}; // the vehicle at its head, the way it goes, at the height of the rails
	glm::dvec3 tail{}; // the vehicle at its tail
	glm::dvec3 direction{0.0, 0.0, -1.0}; // unit, flat, the way it goes
	double speed{0.0}; // m/s
	double curvature{0.0}; // 1/m of the line at the head, positive turning left
	double width{3.0}; // of the vehicles
};

inline constexpr double trackside_eye_height{1.7};
inline constexpr double trackside_distance{8.0}; // metres from the side of the train
inline constexpr double trackside_passed{40.0}; // metres the tail goes past the spot before the railfan moves on
inline constexpr double trackside_max_distance{1000.0}; // metres to the head before the railfan moves on anyway
inline constexpr double trackside_lead_time{15.0}; // seconds the train needs to reach the new spot
inline constexpr double trackside_min_lead{80.0};
inline constexpr double trackside_max_lead{500.0};

// whether the railfan at the View has seen the Train go by and moves on
bool trackside_expired(trackside const &View, train_motion const &Train);
// the next spot of the railfan, ahead of the Train along its line, on the other side than the View had
trackside trackside_next(trackside const &View, train_motion const &Train);
// the camera of the railfan at the View, looking at the Target
placement trackside_placement(trackside const &View, glm::dvec3 const &Target);

// the point and the way Distance metres on from Start going in the (flat, unit) Direction along a line which
// turns by Curvature (1/m, positive to the left)
std::pair<glm::dvec3, glm::dvec3> along_arc(glm::dvec3 const &Start, glm::dvec3 const &Direction, double Curvature, double Distance);
// the curvature of the line under two vehicles of a train going its way: A ahead of B, each at its Position with
// its front (or rear) towards the way of the train; 0 when they stand too close to tell
double line_curvature(glm::dvec3 const &Positiona, glm::dvec3 const &Directiona, glm::dvec3 const &Positionb, glm::dvec3 const &Directionb);

} // namespace cameraviews
