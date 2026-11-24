/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/brakingcurve.h"

#include <algorithm>
#include <limits>

namespace braking_curve {

double brake_table::full_braking(double const Kmh) const {
	auto const index{std::clamp(static_cast<int>(size * Kmh / std::max(1.0, vmax)), 1, size)};
	return std::max(0.01, a0[index] + 12.0 * a1[index]);
}

bool brake_table::known() const {
	// a worked out table holds at least the rolling resistance term at every speed
	return a0[size] > 0.0;
}

double shaped_demand(double const Velocity, double const Target, double const Distance, double const Gravity, brake_table const &Table) {

	auto const travel = [Velocity, Target, Gravity, &Table](double const Ratio) {
		auto const steps{24};
		auto const step{(Velocity - Target) / steps};
		auto distance{0.0};
		for (auto i{0}; i < steps; ++i) {
			auto const kmh{Target + (i + 0.5) * step};
			auto const deceleration{Ratio * Table.full_braking(kmh) - Gravity};
			if (deceleration <= 0.001) {
				return std::numeric_limits<double>::max();
			}
			distance += (kmh / 3.6) * (step / 3.6) / deceleration;
		}
		return distance;
	};

	auto low{0.0};
	auto high{3.0};
	if (travel(high) > Distance) {
		return Gravity - high * Table.full_braking(Velocity);
	}
	for (auto i{0}; i < 30; ++i) {
		auto const middle{0.5 * (low + high)};
		(travel(middle) > Distance ? low : high) = middle;
	}
	return Gravity - high * Table.full_braking(Velocity);
}

double demand(approach const &Approach, brake_table const *Pneumatictable) {

	// a pneumatic brake acts on the state the train will be in once it has responded; while the
	// train is still speeding up, plan the deceleration from there, so more is asked for at once
	// both plans are made from the brake table of the consist; with none (in shunting it isn't worked out, and planning
	// from its zeroes would ask for no braking at all) the braking curve is the constant deceleration over the distance left
	auto const *table{(Pneumatictable != nullptr) && Pneumatictable->known() ? Pneumatictable : nullptr};
	auto velocity{Approach.speed};
	auto distance{Approach.distance};
	if ((table != nullptr) && (Approach.acceleration > 0.0)) {
		auto const horizon{Approach.speedup_horizon};
		velocity = Approach.speed + 3.6 * horizon * Approach.acceleration;
		distance = std::max(0.5 * Approach.distance, Approach.distance - (Approach.speed + velocity) * 0.5 / 3.6 * horizon);
	}
	auto const brakingroom{std::max(distance - Approach.offset, std::min(distance, Approach.offset))};
	if ((table == nullptr) || (velocity <= Approach.target)) {
		return (Approach.target * Approach.target - velocity * velocity) / (25.92 * brakingroom + 0.1);
	}
	// follow the shape of the consist brake force over speed rather than assume a constant
	// deceleration, over the distance left once the brake has had time to fill (while the
	// train is still speeding up, the prediction above has taken that off already)
	auto room{brakingroom};
	if ((Approach.acceleration <= 0.0) && (false == Approach.brake_applied)) {
		// only while the brake is still to be applied: once it acts, the room it needs to respond is behind
		room = std::max(0.5 * brakingroom, brakingroom - Approach.speed / 3.6 * Approach.fill_horizon);
	}
	return shaped_demand(velocity, Approach.target, room + 0.1 / 25.92, Approach.gravity, *table);
}

} // namespace braking_curve
