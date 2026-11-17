/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/consistviews.h"

#include <algorithm>
#include <cmath>

#include "vehicle/Camera.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"

namespace {

// more vehicles than any train has: a guard against a loop in the couplings
std::size_t constexpr train_limit{1000};
// metres the vehicle picked moves before the trackside view takes the way it went for the way of the train
double constexpr direction_step{0.5};

cameraviews::vehicle_pose pose_of(TDynamicObject const *Vehicle) {
	cameraviews::vehicle_pose pose;
	pose.position = Vehicle->GetPosition();
	pose.front = Vehicle->VectorFront();
	pose.length = Vehicle->MoverParameters->Dim.L;
	pose.width = Vehicle->MoverParameters->Dim.W;
	pose.height = Vehicle->MoverParameters->Dim.H;
	return pose;
}

// the end of the Vehicle the Other vehicle is beyond, -1 when it's beyond neither
int end_towards(TDynamicObject const *Vehicle, TDynamicObject const *Other) {
	for (auto const side : {end::front, end::rear}) {
		if (Vehicle->MoverParameters->Neighbours[side].vehicle == Other) {
			return side;
		}
	}
	return -1;
}

// the end of the Vehicle at an end of the train, away from the Inner vehicle next to it (nullptr when it's alone)
int outer_end(TDynamicObject const *Vehicle, TDynamicObject const *Inner, int const Alone) {
	if (Inner == nullptr) {
		return Alone;
	}
	auto const inner{end_towards(Vehicle, Inner)};
	return inner < 0 ? Alone : 1 - inner;
}

cameraviews::coupler coupler_of(TDynamicObject const *Vehicle, int const End) {
	cameraviews::coupler coupler;
	coupler.position = Vehicle->CouplerPosition(static_cast<end>(End));
	coupler.outward = Vehicle->VectorFront() * (End == end::front ? 1.0 : -1.0);
	coupler.width = Vehicle->MoverParameters->Dim.W;
	return coupler;
}

// the driver's way: the front (1) or the rear (-1) of the Driven vehicle, as the user drives it
int driven_way(TDynamicObject const *Driven) {
	auto const cab{Driven->MoverParameters->CabOccupied < 0 ? -1 : 1};
	auto const direction{Driven->MoverParameters->DirActive < 0 ? -1 : 1};
	return cab * direction;
}

// the coupler of the Vehicle furthest along the Way (or furthest back, with a negative Way)
glm::dvec3 end_along(TDynamicObject const *Vehicle, glm::dvec3 const &Way) {
	auto const front{Vehicle->CouplerPosition(end::front)};
	auto const rear{Vehicle->CouplerPosition(end::rear)};
	return glm::dot(front - rear, Way) >= 0.0 ? front : rear;
}

// the front of the Vehicle, or its rear when that faces the Way
glm::dvec3 facing_along(TDynamicObject const *Vehicle, glm::dvec3 const &Way) {
	auto const front{cameraviews::flat_front(Vehicle->VectorFront())};
	return glm::dot(front, Way) < 0.0 ? -front : front;
}

bool usable(cameraviews::placement const &Place) {
	auto const finite{[](glm::dvec3 const &Value) { return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z); }};
	if (false == (finite(Place.eye) && finite(Place.target) && finite(Place.up))) {
		return false;
	}
	auto const look{Place.target - Place.eye};
	if (glm::length(look) < 0.01 || glm::length(Place.up) < 0.5) {
		return false;
	}
	// a camera looking along its own up has no way to tell left from right
	return std::abs(glm::dot(glm::normalize(look), glm::normalize(Place.up))) < 0.999;
}

} // namespace

std::vector<TDynamicObject *> consist_views::train_of(TDynamicObject *Driven) {
	std::vector<TDynamicObject *> train;
	if (Driven == nullptr) {
		return train;
	}
	auto const *controller{Driven->Mechanik != nullptr ? Driven->Mechanik : Driven->ctOwner};
	auto *first{controller != nullptr ? controller->Vehicle(end::front) : nullptr};
	if (first == nullptr) {
		first = Driven;
	}
	for (auto *vehicle{first}; vehicle != nullptr && train.size() < train_limit; vehicle = vehicle->Next()) {
		if (std::ranges::find(train, vehicle) != train.end()) {
			break;
		}
		train.push_back(vehicle);
	}
	if (std::ranges::find(train, Driven) == train.end()) {
		// the controller's list is behind the couplings: the driven vehicle alone, until it catches up
		train.assign(1, Driven);
	}
	return train;
}

TDynamicObject *consist_views::pick(std::vector<TDynamicObject *> const &Train, TDynamicObject *Driven) {
	auto picked{std::ranges::find(Train, m_vehicle)};
	if (m_vehicle == nullptr || picked == Train.end()) {
		picked = std::ranges::find(Train, Driven);
	}
	if (picked == Train.end()) {
		picked = Train.begin();
	}
	auto index{static_cast<int>(picked - Train.begin())};
	if (m_view != kind::coupling) {
		index = cameraviews::step_index(index, static_cast<int>(Train.size()), m_step);
		m_step = 0;
	}
	m_vehicle = Train[index];
	return m_vehicle;
}

double consist_views::gap_beyond(TDynamicObject const *Vehicle, int const End) {
	auto const &neighbour{Vehicle->MoverParameters->Neighbours[End]};
	if (neighbour.vehicle == nullptr || Vehicle->MoverParameters->Couplers[End].CouplingFlag != 0) {
		return -1.0;
	}
	if (neighbour.distance > cameraviews::coupler_watch_range) {
		return -1.0;
	}
	return std::max(0.0, static_cast<double>(neighbour.distance));
}

consist_views::coupling_place consist_views::coupling_at(std::vector<TDynamicObject *> const &Train, int const Index) {
	auto const count{static_cast<int>(Train.size())};
	coupling_place place;
	if (Index > 0 && Index < count) {
		// between two vehicles of the train
		auto *ahead{Train[Index - 1]};
		auto const *behind{Train[Index]};
		auto const aheadend{end_towards(ahead, behind)};
		auto const behindend{end_towards(behind, ahead)};
		place.vehicle = ahead;
		place.coupler = coupler_of(ahead, aheadend < 0 ? end::rear : aheadend);
		place.other = true;
		place.otherposition = behind->CouplerPosition(static_cast<end>(behindend < 0 ? end::front : behindend));
		return place;
	}
	// an end of the train, perhaps with another vehicle coming closer to it
	auto const front{Index <= 0};
	auto *vehicle{front ? Train.front() : Train.back()};
	auto const *inner{count > 1 ? Train[front ? 1 : count - 2] : nullptr};
	auto const outer{outer_end(vehicle, inner, front ? end::front : end::rear)};
	place.vehicle = vehicle;
	place.coupler = coupler_of(vehicle, outer);
	auto const &neighbour{vehicle->MoverParameters->Neighbours[outer]};
	place.gap = gap_beyond(vehicle, outer);
	if (place.gap >= 0.0 && (neighbour.vehicle_end == end::front || neighbour.vehicle_end == end::rear)) {
		place.other = true;
		place.otherposition = neighbour.vehicle->CouplerPosition(static_cast<end>(neighbour.vehicle_end));
	}
	return place;
}

cameraviews::train_motion consist_views::motion_of(std::vector<TDynamicObject *> const &Train, TDynamicObject const *Driven) {
	auto const position{m_vehicle->GetPosition()};
	if (false == m_moved) {
		// standing, the train is to go the way the user drives
		m_direction = cameraviews::flat_front(m_vehicle->VectorFront()) *
		              static_cast<double>(cameraviews::driving_facing(m_vehicle->VectorFront(), Driven->VectorFront(), driven_way(Driven)));
		m_lastposition = position;
		m_moved = true;
	}
	else if (std::hypot(position.x - m_lastposition.x, position.z - m_lastposition.z) > direction_step) {
		m_direction = cameraviews::flat_front(position - m_lastposition);
		m_lastposition = position;
	}
	std::vector<glm::dvec3> positions;
	positions.reserve(Train.size());
	for (auto const *vehicle : Train) {
		positions.emplace_back(vehicle->GetPosition());
	}
	auto const [head, tail]{cameraviews::train_ends(positions, m_direction)};
	cameraviews::train_motion motion;
	motion.direction = m_direction;
	motion.head = end_along(Train[head], m_direction);
	motion.tail = end_along(Train[tail], -m_direction);
	motion.speed = std::abs(m_vehicle->GetVelocity()) / 3.6;
	motion.width = m_vehicle->MoverParameters->Dim.W;
	if (Train.size() > 1) {
		auto const *behind{Train[head == 0 ? 1 : head - 1]};
		motion.curvature = cameraviews::line_curvature(Train[head]->GetPosition(), facing_along(Train[head], m_direction), behind->GetPosition(), facing_along(behind, m_direction));
	}
	return motion;
}

cameraviews::placement consist_views::place_orbit(TCamera &Camera, double const Seconds) {
	auto const pose{pose_of(m_vehicle)};
	if (m_orbit.radius <= 0.0) {
		m_orbit = cameraviews::orbit_start(pose);
	}
	// the mouse turns the camera around the vehicle rather than the view away from it
	glm::dvec2 const turn{Camera.Angle.x, Camera.Angle.y};
	Camera.Angle = {};
	m_orbit = cameraviews::orbit_moved(m_orbit, Camera.Velocity, turn, Seconds, pose);
	return cameraviews::orbit_placement(m_orbit, pose);
}

cameraviews::placement consist_views::place_shunting(TCamera const &Camera, std::vector<TDynamicObject *> const &Train, TDynamicObject const *Driven, double const Seconds) {
	auto const pose{pose_of(m_vehicle)};
	auto length{0.0};
	for (auto const *vehicle : Train) {
		length += vehicle->MoverParameters->Dim.L;
	}
	m_topdown = cameraviews::topdown_moved(m_topdown, Camera.Velocity, Seconds, pose, 0.5 * length + 50.0);
	auto const facing{cameraviews::driving_facing(m_vehicle->VectorFront(), Driven->VectorFront(), Driven->MoverParameters->CabOccupied)};
	return cameraviews::topdown_placement(m_topdown, pose, facing);
}

cameraviews::placement consist_views::place_coupling(TCamera const &Camera, std::vector<TDynamicObject *> const &Train, double const Seconds) {
	auto const count{static_cast<int>(Train.size())};
	if (m_coupling < 0) {
		auto const front{coupling_at(Train, 0)};
		auto const rear{coupling_at(Train, count)};
		m_coupling = cameraviews::first_coupling(front.gap, rear.gap, count);
	}
	m_coupling = cameraviews::step_index(std::min(m_coupling, count), count + 1, m_step);
	m_step = 0;
	auto const place{coupling_at(Train, m_coupling)};
	m_vehicle = place.vehicle;
	m_coupler = cameraviews::coupler_moved(m_coupler, Camera.Velocity, Seconds);
	return cameraviews::coupler_placement(m_coupler, place.coupler, place.other ? &place.otherposition : nullptr);
}

cameraviews::placement consist_views::place_trackside(std::vector<TDynamicObject *> const &Train, TDynamicObject const *Driven) {
	auto const motion{motion_of(Train, Driven)};
	if (cameraviews::trackside_expired(m_trackside, motion)) {
		m_trackside = cameraviews::trackside_next(m_trackside, motion);
	}
	return cameraviews::trackside_placement(m_trackside, cameraviews::orbit_centre(pose_of(m_vehicle)));
}

TDynamicObject *consist_views::enter(kind const View, TDynamicObject *Driven) {
	m_view = View;
	m_step = 0;
	if (View == kind::coupling) {
		// each time the view starts at the end of the train something comes closer to, or at its front
		m_coupling = -1;
	}
	if (View == kind::trackside) {
		// and the railfan finds a new spot ahead of the train
		m_trackside.placed = false;
		m_moved = false;
	}
	auto const train{train_of(Driven)};
	if (train.empty()) {
		return nullptr;
	}
	return pick(train, Driven);
}

void consist_views::select(int const Step) {
	m_step += Step;
}

TDynamicObject *consist_views::update(TCamera &Camera, TDynamicObject *Driven, double const Seconds) {
	auto const train{train_of(Driven)};
	if (train.empty()) {
		return nullptr;
	}
	pick(train, Driven);
	cameraviews::placement place;
	switch (m_view) {
	case kind::orbit:
		place = place_orbit(Camera, Seconds);
		break;
	case kind::shunting:
		// straight down, the mouse would only tilt it off the train
		Camera.Angle = {};
		place = place_shunting(Camera, train, Driven, Seconds);
		break;
	case kind::coupling:
		place = place_coupling(Camera, train, Seconds);
		break;
	case kind::trackside:
		place = place_trackside(train, Driven);
		break;
	}
	if (usable(place)) {
		Camera.Pos = place.eye;
		Camera.LookAt = place.target;
		Camera.vUp = place.up;
	}
	return m_vehicle;
}
