/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "derailment.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace derailment {

namespace {

// a curve with a radius under this is taken for a straight track [m]
constexpr double straight{1e-3};

bool curved(double const Radius) {
	return std::abs(Radius) > straight;
}

// the height the lateral force on the body acts at, with the lean of the body on its suspension [m]
double leaning_height(double const CentreOfGravity, suspension const &Suspension) {
	return CentreOfGravity + Suspension.flexibility * std::max(0.0, CentreOfGravity - Suspension.roll_centre);
}

bool has(int const Flags, int const Flag) {
	return (Flags & Flag) != 0;
}

// the acceleration [m/s2] of what moves with the track sideways, besides the curve: the sway of the body and the
// short waves of an uneven track
double dynamic_acceleration(track_state const &Track, double const Velocity) {
	return body_vibration(Velocity) + uneven_share * Velocity * Velocity * std::abs(Track.uneven_curvature);
}

// the sideways force of the couplings of both ends on the vehicle, positive out of the curve [N]: pushed in, the
// ends go out of the curve, pulled out, they go into it
double coupling_force(vehicle_state const &Vehicle, track_state const &Track) {
	auto force{0.0};
	for (auto const &end : Vehicle.ends) {
		force += end.compression * std::sin(coupling_angle(Vehicle.length, end.other_length, Track.radius));
	}
	return force;
}

} // namespace

double contact_spacing(double const Gauge) {
	return Gauge + rail_head_width;
}

double default_centre_of_gravity(double const BodyHeight, vehicle_class const Class) {
	auto const body{BodyHeight > 2.0 ? BodyHeight : 4.0};
	return body * (Class == vehicle_class::powered ? 0.38 : 0.35);
}

double loaded_centre_of_gravity(double const EmptyHeight, double const EmptyMass, double const LoadMass, double const LoadShare, double const Floor, double const BodyHeight) {
	if (LoadMass <= 0.0 || EmptyMass + LoadMass <= 0.0) {
		return EmptyHeight;
	}
	auto const body{BodyHeight > 2.0 ? BodyHeight : 4.0};
	// a floor not given (or the older meaning of the keys) is that of a wagon
	auto const floor{Floor > 0.3 ? Floor : 1.2};
	auto const load{floor + 0.5 * std::max(0.0, body - floor) * std::clamp(LoadShare, 0.0, 1.0)};
	return (EmptyHeight * EmptyMass + load * LoadMass) / (EmptyMass + LoadMass);
}

double unbalanced_acceleration(double const Velocity, double const Radius, double const Cant) {
	auto const cant{gravity * std::sin(std::abs(Cant))};
	if (false == curved(Radius)) {
		return -cant;
	}
	return Velocity * Velocity / std::abs(Radius) - cant;
}

double line_curvature(track_state const &Track) {
	auto const buckle{buckle_share * Track.buckle_curvature};
	if (false == curved(Track.radius)) {
		return std::abs(buckle);
	}
	return 1.0 / std::abs(Track.radius) + buckle;
}

double line_acceleration(track_state const &Track, double const Velocity) {
	if (Track.buckle_curvature == 0.0) {
		return unbalanced_acceleration(Velocity, Track.radius, Track.cant);
	}
	// bent against the curve, out of the bend is into the curve, where the cant leans the vehicle too
	return Velocity * Velocity * line_curvature(Track) - gravity * std::sin(std::abs(Track.cant));
}

double unloading_ratio(double const Acceleration, double const CentreOfGravity, double const ContactSpacing, suspension const &Suspension) {
	return 2.0 / ContactSpacing * std::abs(Acceleration) / gravity * leaning_height(CentreOfGravity, Suspension);
}

double overturning_acceleration(double const CentreOfGravity, double const ContactSpacing, suspension const &Suspension) {
	return gravity * ContactSpacing / (2.0 * leaning_height(CentreOfGravity, Suspension));
}

double body_vibration(double const Velocity) {
	return vibration * std::min(1.0, std::abs(Velocity) / (100.0 / 3.6));
}

double overturning_speed(double const Radius, double const Cant, double const CentreOfGravity, double const Gauge) {
	if (false == curved(Radius)) {
		return 0.0;
	}
	vehicle_state vehicle;
	vehicle.mass = 1.0;
	vehicle.centre_of_gravity = CentreOfGravity;
	track_state track;
	track.radius = Radius;
	track.cant = Cant;
	track.gauge = Gauge;
	auto const spacing{contact_spacing(Gauge)};
	// the unloading grows with the speed: halve the range until the speed of D = 1 is found
	auto low{0.0};
	auto high{200.0};
	for (int iteration = 0; iteration < 60; ++iteration) {
		auto const middle{0.5 * (low + high)};
		auto const acceleration{body_acceleration(vehicle, track, middle)};
		// below the balancing speed the cant leans the vehicle the other way
		auto const outwards{unbalanced_acceleration(middle, Radius, Cant) > 0.0};
		if (outwards && unloading_ratio(acceleration, CentreOfGravity, spacing) >= 1.0) {
			high = middle;
		}
		else {
			low = middle;
		}
	}
	return high;
}

double nadal_limit(double const FlangeAngle, double const Friction) {
	auto const slope{std::tan(FlangeAngle)};
	return (slope - Friction) / (1.0 + Friction * slope);
}

track_condition condition_of(track_state const &Track) {
	track_condition condition;
	// the flags make the ride rougher: the sway of the vehicles and the short waves of the track grow, the curve and
	// the cant stay what they are. sceneries set them for the jolts of a worn track (damage 31 on all of line 107),
	// not for broken rails. a loose rail sways the vehicles 1.4 times as much, as the legacy criterion had it
	condition.lateral = 1.0 + 0.4 * has(Track.damage, track_damage::freerail) + 0.5 * has(Track.damage, track_damage::railbend) + 0.1 * has(Track.damage, track_damage::railwear) +
	                    0.1 * has(Track.damage, track_damage::thinrail);
	// the twist of the track lifts a wheel: 1 % of its load for each mm of cross level, up to the 0.6 the twist test
	// of EN 14363 lets the load drop by
	condition.unloading = std::min(0.6, 0.01 * std::max(0.0, Track.uneven_cross));
	return condition;
}

double coupling_angle(double const Length, double const OtherLength, double const Radius) {
	if (false == curved(Radius) || OtherLength <= 0.0) {
		return 0.0;
	}
	return (Length + OtherLength) / (4.0 * std::abs(Radius));
}

wheel_forces flange_forces(vehicle_state const &Vehicle, track_state const &Track, double const Velocity) {
	wheel_forces result;
	if (Vehicle.mass <= 0.0) {
		return result;
	}
	auto const axles{static_cast<double>(std::max(Vehicle.axles, 1))};
	auto const condition{condition_of(Track)};
	auto const spacing{contact_spacing(Track.gauge)};
	auto const height{leaning_height(Vehicle.centre_of_gravity, suspension{})};
	auto const staticload{Vehicle.mass * gravity / (2.0 * axles)};

	auto const inertia{Vehicle.mass * line_acceleration(Track, Velocity)};
	auto const couplings{coupling_force(Vehicle, Track)};
	auto const quasistatic{inertia + couplings};
	// the sway and the uneven track push the worst way: towards the rail the vehicle already leans on
	auto const direction{quasistatic >= 0.0 ? 1.0 : -1.0};
	auto const dynamic{Vehicle.mass * dynamic_acceleration(Track, Velocity) * condition.lateral};
	result.lateral = direction * (std::abs(quasistatic) + dynamic);

	// the leading axle meets the rail at an angle in a curve, the creep force of its inner wheel at most the friction
	auto guiding{0.0};
	if (auto const curvature{std::abs(line_curvature(Track))}; curvature > 0.0) {
		auto const attack{0.5 * Vehicle.axle_base * curvature};
		guiding = tread_friction * staticload * std::min(1.0, attack / saturated_attack);
	}
	result.y = std::abs(result.lateral) / axles + guiding;

	// the lateral forces at their heights move the load onto the wheels they push towards
	auto const moment{direction * (inertia * height + couplings * buffer_height) + dynamic * height};
	auto const shift{moment / (spacing * axles)};
	result.q = std::max(0.0, (staticload + shift) * (1.0 - condition.unloading));
	return result;
}

double body_acceleration(vehicle_state const &Vehicle, track_state const &Track, double const Velocity) {
	if (Vehicle.mass <= 0.0) {
		return 0.0;
	}
	auto const height{leaning_height(Vehicle.centre_of_gravity, suspension{})};
	// the couplings pull at the height of the buffers: as much of a lean as their moment gives
	auto const moment{Vehicle.mass * line_acceleration(Track, Velocity) * height + coupling_force(Vehicle, Track) * buffer_height};
	return std::abs(moment) / (Vehicle.mass * height) + dynamic_acceleration(Track, Velocity) * condition_of(Track).lateral;
}

bool stretch::update(bool const Holds, double const Distance, double const Window) {
	if (false == Holds) {
		m_distance = 0.0;
		return false;
	}
	m_distance += std::abs(Distance);
	return m_distance >= Window;
}

verdict monitor::update(vehicle_state const &Vehicle, track_state const &Track, double const Velocity, double const Distance) {
	verdict result;
	result.acceleration = body_acceleration(Vehicle, Track, Velocity);
	result.unloading = unloading_ratio(result.acceleration, Vehicle.centre_of_gravity, contact_spacing(Track.gauge));
	result.flange_ratio = flange_forces(Vehicle, Track, Velocity).ratio();
	auto const overturns{m_overturning.update(result.unloading >= 1.0, Distance)};
	auto const climbs{m_climbing.update(result.flange_ratio >= climbing_limit, Distance)};
	if (overturns) {
		result.result = outcome::overturned;
	}
	else if (climbs) {
		result.result = outcome::off_rails;
	}
	return result;
}

void monitor::reset() {
	m_overturning.reset();
	m_climbing.reset();
}

double sliding_friction(outcome const Outcome) {
	switch (Outcome) {
	case outcome::off_rails:
		return off_rails_friction;
	case outcome::overturned:
		return overturned_friction;
	default:
		return 0.0;
	}
}

std::optional<double> climbing_speed(vehicle_state const &Vehicle, track_state const &Track) {
	auto const climbs = [&](double const Velocity) { return flange_forces(Vehicle, Track, Velocity).ratio() >= climbing_limit; };
	// from the speed the cant balances, where the vehicle starts to lean on the outer rail
	auto const balancing{curved(Track.radius) ? std::sqrt(gravity * std::abs(Track.radius) * std::sin(std::abs(Track.cant))) : 0.0};
	if (climbs(balancing)) {
		return balancing;
	}
	// in steps of 1 m/s up to the first speed it climbs at, which needn't hold above it, then halving the step
	constexpr double fastest{200.0};
	auto const steps{static_cast<int>(std::ceil(fastest - balancing))};
	for (auto step = 0; step < steps; ++step) {
		auto low{balancing + step};
		auto high{std::min(low + 1.0, fastest)};
		if (false == climbs(high)) {
			continue;
		}
		for (int iteration = 0; iteration < 30; ++iteration) {
			auto const middle{0.5 * (low + high)};
			(climbs(middle) ? high : low) = middle;
		}
		return high;
	}
	return std::nullopt;
}

std::string report(verdict const &Verdict, vehicle_state const &Vehicle, track_state const &Track, double const Velocity, std::string_view const TrackName) {
	auto const cant{1000.0 * contact_spacing(Track.gauge) * std::sin(std::abs(Track.cant))};
	return std::format("{} at {:.0f} km/h on track {}, curve radius {:.0f} m, cant {:.0f} mm: lateral acceleration {:.2f} m/s2, wheel unloading {:.2f}, Y/Q {:.2f}"
	                   " (mass {:.1f} t, centre of gravity {:.2f} m, couplings pushing {:.0f} / {:.0f} kN, track damage {}, cross level {:.0f} mm)",
	                   Verdict.result == outcome::overturned ? "overturned" : "derailed, the flange climbed the rail", std::abs(Velocity) * 3.6, TrackName.empty() ? "(none)" : TrackName,
	                   std::abs(Track.radius), cant, Verdict.acceleration, Verdict.unloading, Verdict.flange_ratio, Vehicle.mass / 1000.0, Vehicle.centre_of_gravity,
	                   Vehicle.ends[0].compression / 1000.0, Vehicle.ends[1].compression / 1000.0, Track.damage, Track.uneven_cross);
}

} // namespace derailment
