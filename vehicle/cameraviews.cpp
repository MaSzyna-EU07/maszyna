/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/cameraviews.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace cameraviews {

namespace {

glm::dvec3 const world_up{0.0, 1.0, 0.0};

// radians per second of the left and right keys, and of the page keys, for a key held
double constexpr orbit_turn_rate{1.2};
double constexpr orbit_climb_rate{0.8};
// the radius grows (or shrinks) e times in a second of a key held
double constexpr orbit_zoom_rate{1.0};
// the height of the shunting view grows (or shrinks) e times in a second of a key held
double constexpr topdown_zoom_rate{0.8};
// metres per second of the shunting view moved along and across, for each metre of its height
double constexpr topdown_pan_rate{0.6};
// metres per second of the coupling view moved closer, further, up and down
double constexpr coupler_move_rate{3.0};
// metres the coupling view stands back from the coupler, towards its vehicle
double constexpr coupler_step_back{2.0};
// the share of the gap to the vehicle coming closer the coupling view keeps from the side of the train
double constexpr coupler_gap_share{0.7};

double flat_distance(glm::dvec3 const &A, glm::dvec3 const &B) {
	return std::hypot(A.x - B.x, A.z - B.z);
}

} // namespace

glm::dvec3 flat_front(glm::dvec3 const &Front) {
	glm::dvec3 const flat{Front.x, 0.0, Front.z};
	auto const length{glm::length(flat)};
	if (length < 1e-6) {
		return {0.0, 0.0, -1.0};
	}
	return flat / length;
}

glm::dvec3 flat_left(glm::dvec3 const &Flatfront) {
	return glm::cross(world_up, Flatfront);
}

int step_index(int const Index, int const Count, int const Step) {
	if (Count <= 0) {
		return 0;
	}
	return std::clamp(Index + Step, 0, Count - 1);
}

std::pair<int, int> train_ends(std::vector<glm::dvec3> const &Positions, glm::dvec3 const &Direction) {
	if (Positions.empty()) {
		return {-1, -1};
	}
	auto const way{flat_front(Direction)};
	auto const along{[&way](glm::dvec3 const &Position) { return glm::dot(Position, way); }};
	auto const [tail, head]{std::ranges::minmax_element(Positions, {}, along)};
	return {static_cast<int>(head - Positions.begin()), static_cast<int>(tail - Positions.begin())};
}

int driving_facing(glm::dvec3 const &Front, glm::dvec3 const &Drivenfront, int const Cab) {
	auto const cab{Cab < 0 ? -1 : 1};
	auto const aligned{glm::dot(flat_front(Front), flat_front(Drivenfront)) < 0.0 ? -1 : 1};
	return cab * aligned;
}

// orbit

glm::dvec3 orbit_centre(vehicle_pose const &Vehicle) {
	return Vehicle.position + world_up * (0.4 * Vehicle.height);
}

double orbit_min_radius(vehicle_pose const &Vehicle) {
	return 0.5 * std::hypot(Vehicle.length, Vehicle.width) + 1.0;
}

orbit orbit_start(vehicle_pose const &Vehicle) {
	orbit start;
	start.azimuth = 0.6;
	start.elevation = 0.25;
	start.radius = std::max(orbit_min_radius(Vehicle), Vehicle.length + 5.0);
	return orbit_clamped(start, Vehicle);
}

double orbit_lowest_elevation(vehicle_pose const &Vehicle, double const Radius) {
	auto const centre{0.4 * Vehicle.height};
	auto const sine{std::clamp((rail_clearance - centre) / std::max(Radius, 1e-3), -1.0, 1.0)};
	return std::max(orbit_min_elevation, std::asin(sine));
}

orbit orbit_clamped(orbit Orbit, vehicle_pose const &Vehicle) {
	if (Orbit.radius <= 0.0) {
		Orbit.radius = orbit_start(Vehicle).radius;
	}
	Orbit.radius = std::clamp(Orbit.radius, orbit_min_radius(Vehicle), std::max(orbit_min_radius(Vehicle), orbit_max_radius));
	Orbit.elevation = std::clamp(Orbit.elevation, orbit_lowest_elevation(Vehicle, Orbit.radius), orbit_max_elevation);
	Orbit.azimuth = std::remainder(Orbit.azimuth, 2.0 * std::numbers::pi);
	return Orbit;
}

orbit orbit_moved(orbit Orbit, move_keys const &Keys, glm::dvec2 const &Turn, double const Seconds, vehicle_pose const &Vehicle) {
	Orbit = orbit_clamped(Orbit, Vehicle);
	// the left key takes the camera to the left around the vehicle, as a step to the left would while looking at it:
	// clockwise seen from above. so does the mouse moved to the right, which turns the view to the right
	Orbit.azimuth += Keys.x * orbit_turn_rate * Seconds + Turn.y;
	// the mouse moved down looks down at the vehicle, from higher up
	Orbit.elevation += Keys.y * orbit_climb_rate * Seconds - Turn.x;
	// the up arrow (a negative z) brings the camera closer
	Orbit.radius *= std::exp(Keys.z * orbit_zoom_rate * Seconds);
	return orbit_clamped(Orbit, Vehicle);
}

placement orbit_placement(orbit const &Orbit, vehicle_pose const &Vehicle) {
	auto const front{flat_front(Vehicle.front)};
	auto const left{flat_left(front)};
	auto const around{std::cos(Orbit.azimuth) * front + std::sin(Orbit.azimuth) * left};
	auto const way{std::cos(Orbit.elevation) * around + std::sin(Orbit.elevation) * world_up};
	placement place;
	place.target = orbit_centre(Vehicle);
	place.eye = place.target + Orbit.radius * way;
	place.up = world_up;
	return place;
}

// shunting view

double topdown_min_height(vehicle_pose const &Vehicle) {
	return Vehicle.height + 3.0;
}

topdown topdown_clamped(topdown View, vehicle_pose const &Vehicle, double const Alonglimit) {
	auto const lowest{topdown_min_height(Vehicle)};
	if (View.height <= 0.0) {
		View.height = 1.5 * Vehicle.length;
	}
	View.height = std::clamp(View.height, lowest, std::max(lowest, topdown_max_height));
	auto const along{std::max(0.0, Alonglimit)};
	View.along = std::clamp(View.along, -along, along);
	View.across = std::clamp(View.across, -topdown_max_across, topdown_max_across);
	return View;
}

topdown topdown_moved(topdown View, move_keys const &Keys, double const Seconds, vehicle_pose const &Vehicle, double const Alonglimit) {
	View = topdown_clamped(View, Vehicle, Alonglimit);
	auto const pan{View.height * topdown_pan_rate * Seconds};
	// the up arrow (a negative z) moves the view up the screen, the way the user drives
	View.along -= Keys.z * pan;
	View.across += Keys.x * pan;
	View.height *= std::exp(Keys.y * topdown_zoom_rate * Seconds);
	return topdown_clamped(View, Vehicle, Alonglimit);
}

placement topdown_placement(topdown const &View, vehicle_pose const &Vehicle, int const Facing) {
	auto const up{flat_front(Vehicle.front) * (Facing < 0 ? -1.0 : 1.0)};
	// looking down with the screen up the way the user drives, the right of the screen is the right of that way
	auto const right{-flat_left(up)};
	placement place;
	auto const ground{Vehicle.position + up * View.along + right * View.across};
	place.target = ground;
	place.eye = ground + world_up * View.height;
	place.up = up;
	return place;
}

// coupling view

coupler_view coupler_moved(coupler_view View, move_keys const &Keys, double const Seconds) {
	if (Keys.x < 0.0) {
		View.side = 1.0;
	}
	else if (Keys.x > 0.0) {
		View.side = -1.0;
	}
	View.distance = std::clamp(View.distance + Keys.z * coupler_move_rate * Seconds, coupler_min_distance, coupler_max_distance);
	View.height = std::clamp(View.height + Keys.y * coupler_move_rate * Seconds, coupler_min_height, coupler_max_height);
	return View;
}

placement coupler_placement(coupler_view const &View, coupler const &Coupler, glm::dvec3 const *Other) {
	auto const outward{flat_front(Coupler.outward)};
	auto const aside{flat_left(outward) * (View.side < 0.0 ? -1.0 : 1.0)};
	auto const buffers{Coupler.position + world_up * buffer_height};
	auto distance{View.distance};
	placement place;
	if (Other != nullptr) {
		auto const otherbuffers{*Other + world_up * buffer_height};
		distance = std::max(distance, coupler_gap_share * flat_distance(Coupler.position, *Other));
		place.target = 0.5 * (buffers + otherbuffers);
	}
	else {
		place.target = buffers + outward;
	}
	distance = std::min(distance, coupler_max_distance);
	place.eye = buffers - outward * coupler_step_back + aside * (0.5 * Coupler.width + distance) + world_up * View.height;
	place.up = world_up;
	return place;
}

int first_coupling(double const Frontgap, double const Reargap, int const Count) {
	auto const watched{[](double const Gap) { return Gap >= 0.0 && Gap <= coupler_watch_range; }};
	auto const front{watched(Frontgap)};
	auto const rear{watched(Reargap)};
	if (rear && (false == front || Reargap < Frontgap)) {
		return std::max(Count, 0);
	}
	return 0;
}

// trackside view

std::pair<glm::dvec3, glm::dvec3> along_arc(glm::dvec3 const &Start, glm::dvec3 const &Direction, double const Curvature, double const Distance) {
	auto const way{flat_front(Direction)};
	if (std::abs(Curvature) < 1e-6) {
		return {Start + way * Distance, way};
	}
	auto const left{flat_left(way)};
	auto const angle{Curvature * Distance};
	auto const point{Start + way * (std::sin(angle) / Curvature) + left * ((1.0 - std::cos(angle)) / Curvature)};
	auto const heading{way * std::cos(angle) + left * std::sin(angle)};
	return {point, heading};
}

double line_curvature(glm::dvec3 const &Positiona, glm::dvec3 const &Directiona, glm::dvec3 const &Positionb, glm::dvec3 const &Directionb) {
	auto const distance{flat_distance(Positiona, Positionb)};
	if (distance < 1.0) {
		return 0.0;
	}
	auto const ahead{flat_front(Directiona)};
	auto const behind{flat_front(Directionb)};
	// the vehicle ahead turned to the left of the one behind it: a turn about the up axis, anticlockwise from above
	auto const angle{std::atan2(glm::cross(behind, ahead).y, glm::dot(behind, ahead))};
	return angle / distance;
}

bool trackside_expired(trackside const &View, train_motion const &Train) {
	if (false == View.placed) {
		return true;
	}
	auto const way{flat_front(Train.direction)};
	auto const offset{View.spot - Train.tail};
	auto const passed{glm::dot(glm::dvec3{offset.x, 0.0, offset.z}, way) < -trackside_passed};
	return passed || flat_distance(View.spot, Train.head) > trackside_max_distance;
}

trackside trackside_next(trackside const &View, train_motion const &Train) {
	auto lead{std::clamp(Train.speed * trackside_lead_time, trackside_min_lead, trackside_max_lead)};
	// along a sharp curve the line comes round: the spot stays within a quarter of the circle
	if (auto const quarter{0.5 * std::numbers::pi}; std::abs(Train.curvature) * lead > quarter) {
		lead = quarter / std::abs(Train.curvature);
	}
	auto const [point, heading]{along_arc(Train.head, Train.direction, Train.curvature, lead)};
	trackside next;
	next.side = View.placed ? -View.side : View.side;
	next.spot = point + flat_left(heading) * (next.side * (0.5 * Train.width + trackside_distance)) + world_up * trackside_eye_height;
	next.placed = true;
	return next;
}

placement trackside_placement(trackside const &View, glm::dvec3 const &Target) {
	placement place;
	place.eye = View.spot;
	place.target = Target;
	place.up = world_up;
	return place;
}

} // namespace cameraviews
