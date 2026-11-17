/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "collision.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "derailment.h"

namespace collision {

namespace {

// the hit is integrated in slices of this length at most, and in no more than so many of them [s]
constexpr double slice{0.0005};
constexpr double slices_max{200.0};

// the force of a curve over a range of crush within one of its parts, the force linear in it there [J]
double linear_work(double const ForceFrom, double const ForceTo, double const Length) {
	return 0.5 * (ForceFrom + ForceTo) * std::max(Length, 0.0);
}

} // namespace

double crush_curve::force(double const Crush) const {
	if (Crush >= limit) {
		return std::numeric_limits<double>::infinity();
	}
	if (Crush < onset) {
		return peak;
	}
	if (Crush < densify) {
		return plateau;
	}
	return plateau * (1.0 + hardening * (Crush - densify));
}

double crush_curve::energy(double const From, double const To) const {
	auto const from{std::max(From, 0.0)};
	auto const to{std::min(To, limit)};
	if (to <= from) {
		return 0.0;
	}
	auto work{0.0};
	// the peak
	work += peak * std::max(0.0, std::min(to, onset) - from);
	// the plateau
	auto const plateaufrom{std::max(from, onset)};
	auto const plateauto{std::min(to, std::max(densify, onset))};
	work += plateau * std::max(0.0, plateauto - plateaufrom);
	// packing down
	auto const densefrom{std::max(from, std::max(densify, onset))};
	if (to > densefrom) {
		work += linear_work(force(densefrom), plateau * (1.0 + hardening * (to - densify)), to - densefrom);
	}
	return work;
}

crush_curve default_crush(vehicle_kind const Kind) {
	switch (Kind) {
	case vehicle_kind::locomotive:
		return {4.0e6, 3.0e6, 0.05, 0.6, 2.0, 1.2};
	case vehicle_kind::crash_zone:
		return {2.0e6, 2.0e6, 0.0, 0.8, 2.5, 1.2};
	case vehicle_kind::wagon:
	case vehicle_kind::coach:
	default:
		return {3.0e6, 2.0e6, 0.05, 0.8, 2.0, 1.5};
	}
}

vehicle_kind kind_of(bool const Powered, bool const Passenger, bool const CrashZone) {
	if (CrashZone) {
		return vehicle_kind::crash_zone;
	}
	if (Passenger) {
		return vehicle_kind::coach;
	}
	return Powered ? vehicle_kind::locomotive : vehicle_kind::wagon;
}

crush_curve crush_of(end_definition const &Definition, vehicle_kind const Kind) {
	auto curve{default_crush(Definition.crash_zone.value_or(false) ? vehicle_kind::crash_zone : Kind)};
	curve.peak = std::max(Definition.peak.value_or(curve.peak), 1.0);
	curve.plateau = std::max(Definition.plateau.value_or(curve.plateau), 1.0);
	curve.onset = std::max(Definition.onset.value_or(curve.onset), 0.0);
	curve.densify = std::max(Definition.densify.value_or(curve.densify), curve.onset);
	curve.hardening = std::max(Definition.hardening.value_or(curve.hardening), 0.0);
	curve.limit = std::max(Definition.limit.value_or(curve.limit), curve.onset);
	return curve;
}

bool anticlimber_of(end_definition const &Definition, vehicle_kind const Kind) {
	return Definition.anticlimber.value_or(Definition.crash_zone.value_or(Kind == vehicle_kind::crash_zone));
}

double buffer_height(end_definition const &Definition, vehicle_kind const Kind, double const LoadShare) {
	auto const wagon{Kind == vehicle_kind::wagon};
	auto const empty{Definition.buffer_height.value_or(wagon ? 1.065 : 1.040)};
	// the springs of a wagon give 125 mm under its full load
	return wagon ? empty - 0.125 * std::clamp(LoadShare, 0.0, 1.0) : empty;
}

buffer_rating rating(buffer_category const Category) {
	switch (Category) {
	case buffer_category::B:
		return {0.105, 50.0e3};
	case buffer_category::C:
		return {0.105, 70.0e3};
	case buffer_category::L:
		return {0.150, 70.0e3};
	case buffer_category::passenger:
		return {0.110, 30.0e3};
	case buffer_category::A:
	default:
		return {0.105, 30.0e3};
	}
}

coupler_physics::spring_curve buffer_curve(buffer_category const Category, double const Progression) {
	auto const buffer{rating(Category)};
	auto const progression{std::clamp(Progression, 0.0, 0.9)};
	// the energy of the curve to the end of the stroke is F_end * stroke * ((1 - p) / 2 + p / 4), of two buffers
	auto const endforce{2.0 * buffer.energy / (buffer.stroke * ((1.0 - progression) / 2.0 + progression / 4.0))};
	return {endforce / buffer.stroke, buffer.stroke, progression};
}

std::optional<buffer_category> buffer_category_of(char const Name) {
	switch (Name) {
	case 'A':
	case 'a':
		return buffer_category::A;
	case 'B':
	case 'b':
		return buffer_category::B;
	case 'C':
	case 'c':
		return buffer_category::C;
	case 'L':
	case 'l':
		return buffer_category::L;
	case 'P':
	case 'p':
		return buffer_category::passenger;
	default:
		return std::nullopt;
	}
}

buffer_category default_buffers(vehicle_kind const Kind) {
	return Kind == vehicle_kind::coach ? buffer_category::passenger : buffer_category::A;
}

double buffer_capacity(end_definition const &Definition, vehicle_kind const Kind) {
	// two buffers side by side
	return 2.0 * rating(Definition.buffers.value_or(default_buffers(Kind))).energy;
}

void contact_energy::update(bool const Pressed, bool const Solid, double const Closing, double const Force, double const Compression, double const Dt) {
	if (false == Pressed) {
		// the buffers apart, the contact is over
		reset();
		return;
	}
	if (Solid || m_inside) {
		// pressed all the way, the buffers take in nothing more; nor do those of vehicles inside each other
		return;
	}
	auto const force{std::max(Force, 0.0)};
	if (Closing > 0.0) {
		m_absorbed += force * Closing * std::max(Dt, 0.0);
	}
	else if (Closing < 0.0) {
		// springing back, they hold no more than their springs pressed in so far
		m_absorbed = std::min(m_absorbed, 0.5 * force * std::max(Compression, 0.0));
	}
}

void contact_energy::inside() {
	m_absorbed = 0.0;
	m_inside = true;
}

void contact_energy::crushed(double const Work) {
	m_crushed += std::max(Work, 0.0);
}

double contact_energy::budget(double const Mass, double const Closing, double const Capacity) const {
	auto const closing{std::max(Closing, 0.0)};
	// what the buffers of EN 15551 would still take in: the springs of the .fiz taking in more leave no more energy past
	// the buffers than the hit has
	return 0.5 * std::max(Mass, 0.0) * closing * closing - std::max(0.0, Capacity - m_absorbed);
}

void contact_energy::reset() {
	m_absorbed = 0.0;
	m_crushed = 0.0;
	m_inside = false;
}

hit_result hit(end_state const &First, end_state const &Second, hit_input const &Input) {
	hit_result result;
	if (Input.velocity <= 0.0 || Input.mass1 <= 0.0 || Input.mass2 <= 0.0 || Input.dt <= 0.0) {
		return result;
	}
	auto const mass{Input.mass1 * Input.mass2 / (Input.mass1 + Input.mass2)};
	auto const restitution{std::clamp(Input.restitution, 0.0, 0.95)};
	// the elastic hit on the steel at the closing velocity adds root * velocity to the force of the springs
	auto const root{std::sqrt(std::max(Input.stiffness, 1.0) * mass)};
	auto const springs{std::max(Input.springs, 0.0)};
	// the closing velocity the step started with: the springs slow the pair over the step as well, the hit adds what
	// they don't carry of the force the end gives at
	auto velocity{Input.start_velocity > 0.0 ? std::max(Input.start_velocity, Input.velocity) : Input.velocity + springs * Input.dt / mass};
	// where the forces of the step alone would take the pair
	auto const unhit{velocity - springs * Input.dt / mass};

	auto crush1{First.crush};
	auto crush2{Second.crush};
	auto const elastic_limit = [&](double const Force) { return std::max(0.0, Force - springs) / root; };
	{
		auto const force{std::min(First.curve.force(crush1), Second.curve.force(crush2))};
		if (false == std::isfinite(force) || velocity <= elastic_limit(force) || false == (Input.budget > 0.0)) {
			// the steel takes it all: the legacy elastic exchange
			return result;
		}
	}
	result.crushing = true;

	auto const slices{std::clamp(std::ceil(Input.dt / slice), 1.0, slices_max)};
	auto const length{Input.dt / slices};
	auto time{0.0};
	auto rebound{false};
	auto crushtime{0.0};
	auto work{0.0};
	auto cap{0.0}; // the force the crush went at last
	while (Input.dt - time > 1e-12) {
		auto const force1{First.curve.force(crush1)};
		auto const force2{Second.curve.force(crush2)};
		auto const force{std::min(force1, force2)};
		if (false == std::isfinite(force) || velocity <= elastic_limit(force)) {
			// both ends crushed all the way, or the pair slowed down to what the steel takes back
			rebound = true;
			break;
		}
		auto const limit{elastic_limit(force)};
		auto step{std::min(length, Input.dt - time)};
		auto change{force * step / mass};
		if (velocity - change < limit) {
			step = (velocity - limit) * mass / force;
			change = velocity - limit;
		}
		auto crush{(velocity - 0.5 * change) * step};
		auto spent{false};
		if (work + force * crush > Input.budget) {
			// the energy of the hit past the buffers runs out in the slice: the crush goes as far as it takes, the
			// buffers of EN 15551 would take in the rest
			auto const share{std::clamp((Input.budget - work) / std::max(force * crush, 1e-12), 0.0, 1.0)};
			step *= share;
			change *= share;
			crush *= share;
			spent = true;
		}
		work += force * crush;
		cap = force;
		// the weaker end gives, both alike when they're alike
		if (std::abs(force1 - force2) <= 1e-9 * force) {
			crush1 += 0.5 * crush;
			crush2 += 0.5 * crush;
		}
		else if (force1 < force2) {
			crush1 += crush;
		}
		else {
			crush2 += crush;
		}
		velocity -= change;
		time += step;
		crushtime += step;
		result.force = std::max(result.force, force);
		if (spent) {
			result.spent = true;
			rebound = true;
			break;
		}
	}

	auto after{velocity};
	auto duration{crushtime};
	if (rebound) {
		// the steel springs back with the restitution, the springs push the pair apart over the rest of the step
		// under the force the weaker end gives at, as the slices stopped there. a crush stopped at the budget: the
		// buffers of EN 15551 would have taken in the rest, its force no more than what crushes the end
		result.force = std::max(result.force, result.spent ? std::min(springs + velocity * root, cap) : springs + velocity * root);
		if (velocity > 0.0) {
			duration += (1.0 + restitution) * velocity * mass / std::max(velocity * root, 1.0);
		}
		after = -restitution * velocity - springs * (Input.dt - time) / mass;
	}
	result.velocity = after;
	result.impulse = mass * (unhit - after);
	result.duration = std::min(duration, Input.dt);
	result.crush1 = crush1 - First.crush;
	result.crush2 = crush2 - Second.crush;
	result.work = work;
	return result;
}

override_result override_of(override_input const &Input) {
	override_result result;
	if (Input.anticlimbers) {
		// interlocked, they keep the ends level until the crush zone of one of them is used up
		auto const spent1{Input.zone1 <= 0.0 || Input.crush1 >= Input.zone1};
		auto const spent2{Input.zone2 <= 0.0 || Input.crush2 >= Input.zone2};
		if (false == spent1 && false == spent2) {
			return result;
		}
	}
	auto const crush{Input.crush1 + Input.crush2};
	if (crush > override_both) {
		result.first = true;
		result.second = true;
		return result;
	}
	if (crush <= override_crush) {
		return result;
	}
	if (Input.mass1 * override_mass_ratio < Input.mass2) {
		result.first = true;
	}
	else if (Input.mass2 * override_mass_ratio < Input.mass1) {
		result.second = true;
	}
	else if (false == Input.anticlimbers && std::abs(Input.height1 - Input.height2) >= override_height - 1e-9) {
		// the higher one rides up
		result.first = Input.height1 > Input.height2;
		result.second = false == result.first;
	}
	return result;
}

double pushed_wheel::ratio() const {
	if (q > 0.0) {
		return y / q;
	}
	return y > 0.0 ? 1e9 : 0.0;
}

double pushed_ratio(pushed_vehicle const &Vehicle, pushed_end const &End, double const Radius) {
	return pushed_forces(Vehicle, End, Radius).ratio();
}

pushed_wheel pushed_forces(pushed_vehicle const &Vehicle, pushed_end const &End, double const Radius) {
	pushed_wheel result;
	if (Vehicle.mass <= 0.0 || End.force <= 0.0) {
		return result;
	}
	auto const axles{static_cast<double>(std::max(Vehicle.axles, 1))};
	// the axles of the half of the vehicle at the end: its bogie, or its axle
	auto const half{std::max(axles / 2.0, 1.0)};
	auto const wheelload{Vehicle.mass * gravity / (2.0 * axles)};
	auto const halflength{std::max(0.5 * Vehicle.length, 1.0)};
	auto const angle{straight_angle + derailment::coupling_angle(Vehicle.length, End.other_length, Radius) + crush_angle * std::max(End.crush, 0.0) / halflength};
	// in a curve the leading axle meets the rail at an angle, the creep force of its inner wheel at most the friction
	auto guiding{0.0};
	if (std::abs(Radius) > 1e-3) {
		auto const attack{Vehicle.axle_base / (2.0 * std::abs(Radius))};
		guiding = derailment::tread_friction * wheelload * std::min(1.0, attack / derailment::saturated_attack);
	}
	auto const lateral{End.force * angle / half + guiding};
	// the force off the height it's carried at pitches the vehicle: the bogie at the end pressed in takes the load off
	auto const pivots{Vehicle.pivots > 0.0 ? Vehicle.pivots : halflength};
	result.y = lateral;
	result.q = wheelload - End.force * eccentricity / (2.0 * half * pivots);
	return result;
}

bool pushed_monitor::update(bool const Climbs, double const Distance, bool const Lifted, double const Duration) {
	auto const climbed{m_climbing.update(Climbs, Distance, climb_distance)};
	if (false == Lifted) {
		m_lifted = 0.0;
		return climbed;
	}
	m_lifted += std::max(Duration, 0.0);
	return climbed || m_lifted >= lift_time - 1e-9;
}

void pushed_monitor::reset() {
	m_climbing.reset();
	m_lifted = 0.0;
}

} // namespace collision
