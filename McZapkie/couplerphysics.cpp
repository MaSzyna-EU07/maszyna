/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "couplerphysics.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <tuple>

namespace coupler_physics {

namespace {

// the transition between the loading and the unloading curve of a device, to the secant stiffness of the pair: the
// friction holds the device stiffer than its springs until the force changes enough to slip it
constexpr double transition_factor{3.0};
// the force, the deflection and the energy of a curve are computed within this many iterations of the inverse
constexpr int inverse_iterations{50};
constexpr int series_iterations{60};

// a device which doesn't give at all, standing in for the buffers of an end with an adapter
constexpr spring_curve rigid{1.0, 0.0, 0.0};

double series_stiffness(double First, double Second) {
	return First * Second / (First + Second);
}

// the buffers of the pair: those of both ends in series, or with an adapter on either end, the device with the shorter
// travel alone, the one of the automatic coupler which takes the push
struct pressed_devices {
	spring_curve first;
	spring_curve second;
	double unloading{0.5};
	double damping{0.0};
};

bool shorter(spring_curve const &First, spring_curve const &Second) {
	return std::tie(First.travel, First.stiffness, First.progression) <= std::tie(Second.travel, Second.stiffness, Second.progression);
}

pressed_devices buffers_of(end_parameters const &First, end_parameters const &Second, bool const Adapter) {
	auto const unloading{0.5 * (First.buffers_unloading + Second.buffers_unloading)};
	auto const damping{0.5 * (First.damping + Second.damping)};
	if (false == Adapter) {
		return {First.buffers, Second.buffers, unloading, damping};
	}
	auto const &coupler{shorter(First.buffers, Second.buffers) ? First.buffers : Second.buffers};
	return {coupler, rigid, unloading, damping};
}

pressed_devices draw_gears_of(end_parameters const &First, end_parameters const &Second) {
	return {First.draw_gear, Second.draw_gear, 0.5 * (First.draw_gear_unloading + Second.draw_gear_unloading), 0.5 * (First.damping + Second.damping)};
}

struct pressed_force {
	double force{0.0}; // with the friction and the damping [N]
	double curve_force{0.0}; // of the loading curve [N]
	double first_deflection{0.0}; // of the first device [m]
	bool solid{false}; // both devices at the end of their travel
};

// the force of the devices pressed (or pulled) by the deflection growing at the rate
pressed_force press(pressed_devices const &Devices, hysteresis &State, double const Deflection, double const Rate) {
	if (Deflection <= 0.0) {
		State.reset();
		return {};
	}
	auto const split{series(Devices.first, Devices.second, Deflection)};
	auto const travel{Devices.first.travel + Devices.second.travel};
	auto const top{std::max(Devices.first.end_force(), Devices.second.end_force())};
	auto const transition{transition_factor * top / std::max(travel, 1e-3)};
	auto const friction{State.update(Deflection, split.force, Devices.unloading * split.force, transition)};
	// a buffer pushes, a draw gear pulls, neither the other way round however fast they spring back
	return {std::max(0.0, friction + Devices.damping * Rate), split.force, split.first, Deflection >= travel};
}

bool weaker(link_strength const &First, link_strength const &Second) {
	return std::tie(First.ultimate, First.yield, First.hardening) <= std::tie(Second.ultimate, Second.yield, Second.hardening);
}

} // namespace

double spring_curve::force(double const Deflection) const {
	if (travel <= 0.0 || Deflection <= 0.0) {
		return 0.0;
	}
	auto const u{std::min(Deflection / travel, 1.0)};
	return end_force() * ((1.0 - progression) * u + progression * u * u * u);
}

double spring_curve::deflection(double const Force) const {
	if (travel <= 0.0 || Force <= 0.0) {
		return 0.0;
	}
	auto const f{Force / end_force()};
	if (f >= 1.0) {
		return travel;
	}
	if (progression <= 0.0) {
		return f * travel;
	}
	// the curve is rising and convex, so Newton's method from the end of the travel closes in from above
	auto u{1.0};
	auto step{1.0};
	for (int iteration = 0; iteration < inverse_iterations && std::abs(step) > 1e-15; ++iteration) {
		auto const value{progression * u * u * u + (1.0 - progression) * u - f};
		auto const slope{3.0 * progression * u * u + (1.0 - progression)};
		step = value / slope;
		u -= step;
	}
	return std::clamp(u, 0.0, 1.0) * travel;
}

double spring_curve::energy(double const Deflection) const {
	if (travel <= 0.0 || Deflection <= 0.0) {
		return 0.0;
	}
	auto const u{std::min(Deflection / travel, 1.0)};
	return end_force() * travel * ((1.0 - progression) * u * u / 2.0 + progression * u * u * u * u / 4.0);
}

series_split series(spring_curve const &First, spring_curve const &Second, double const Deflection) {
	if (Deflection <= 0.0) {
		return {};
	}
	if (Second.travel <= 0.0) {
		return {First.force(Deflection), std::min(Deflection, First.travel), 0.0};
	}
	if (First.travel <= 0.0) {
		return {Second.force(Deflection), 0.0, std::min(Deflection, Second.travel)};
	}
	if (std::tie(First.stiffness, First.travel, First.progression) == std::tie(Second.stiffness, Second.travel, Second.progression)) {
		auto const half{std::min(Deflection / 2.0, First.travel)};
		return {First.force(half), half, half};
	}
	auto const top{std::max(First.end_force(), Second.end_force())};
	if (Deflection >= First.travel + Second.travel) {
		return {top, First.travel, Second.travel};
	}
	// the force at which the deflections of both add up to the given one. the sum doesn't depend on the order of
	// the devices, so neither does the result
	auto low{0.0};
	auto high{top};
	for (int iteration = 0; iteration < series_iterations; ++iteration) {
		auto const middle{0.5 * (low + high)};
		if (First.deflection(middle) + Second.deflection(middle) < Deflection) {
			low = middle;
		}
		else {
			high = middle;
		}
	}
	auto const force{0.5 * (low + high)};
	return {force, First.deflection(force), Second.deflection(force)};
}

double hysteresis::update(double const Deflection, double const Loading, double const Unloading, double const Transition) {
	force = std::clamp(force + Transition * (Deflection - deflection), std::min(Unloading, Loading), Loading);
	deflection = Deflection;
	return force;
}

void hysteresis::reset() {
	force = 0.0;
	deflection = 0.0;
}

double plastic_elongation(link_strength const &Link, double const Force, double const Elongation) {
	if (false == Link.breakable || Force <= Link.yield) {
		return Elongation;
	}
	return std::max(Elongation, (Force - Link.yield) / Link.hardening);
}

double impact_force(double const Velocity, double const Stiffness, double const Mass1, double const Mass2) {
	if (Velocity <= 0.0 || Mass1 <= 0.0 || Mass2 <= 0.0) {
		return 0.0;
	}
	auto const mass{Mass1 * Mass2 / (Mass1 + Mass2)};
	return Velocity * std::sqrt(Stiffness * mass);
}

double strength_scatter(std::string_view const Name, int const End) {
	// FNV-1a, which std::hash isn't on every compiler
	std::uint64_t hash{14695981039346656037ULL};
	auto const mix = [&hash](unsigned char const Byte) {
		hash ^= Byte;
		hash *= 1099511628211ULL;
	};
	for (auto const character : Name) {
		mix(static_cast<unsigned char>(character));
	}
	mix(static_cast<unsigned char>(End));
	// the 53 high bits, as a fraction
	auto const fraction{static_cast<double>(hash >> 11) / 9007199254740992.0};
	return 1.0 + 0.2 * fraction;
}

link_strength weaker_link(end_parameters const &First, end_parameters const &Second) {
	auto link{weaker(First.link, Second.link) ? First.link : Second.link};
	// an articulated joint holds whatever the end of the other section is set up to
	link.breakable = First.link.breakable && Second.link.breakable;
	return link;
}

void pair_state::couple() {
	draw_gear.reset();
	elongation = 0.0;
}

pair_result update_pair(end_parameters const &First, end_parameters const &Second, pair_state &State, pair_input const &Input) {
	pair_result result;
	State.stop = 0;
	State.stop_force = 0.0;

	pressed_force pulled;
	if (Input.coupled) {
		auto const play{First.slack + Second.slack + State.elongation};
		pulled = press(draw_gears_of(First, Second), State.draw_gear, Input.gap - play, Input.gap_rate);
	}
	else {
		State.draw_gear.reset();
	}
	auto const pushed{press(buffers_of(First, Second, Input.adapter), State.buffers, -Input.gap, -Input.gap_rate)};

	result.tension = pulled.force;
	result.compression = pushed.force;
	result.buffers_deflection = Input.adapter ? 0.0 : pushed.first_deflection;
	if (pulled.solid) {
		State.stop = 1;
		State.stop_force = pulled.curve_force;
	}
	else if (pushed.solid) {
		State.stop = -1;
		State.stop_force = pushed.curve_force;
	}

	if (Input.coupled) {
		auto const link{weaker_link(First, Second)};
		result.overloaded = result.tension > link.yield;
		if (Input.damage && link.breakable && result.tension >= link.ultimate) {
			result.broken = true;
			result.tension = 0.0;
			State.draw_gear.reset();
			State.stop = std::min(State.stop, 0);
		}
		else if (Input.damage) {
			State.elongation = plastic_elongation(link, result.tension, State.elongation);
		}
	}

	result.force = result.tension - result.compression;
	State.gap = Input.gap;
	return result;
}

impact_result impact(end_parameters const &First, end_parameters const &Second, pair_state &State, double const Velocity, double const Mass1, double const Mass2) {
	impact_result result;
	if (State.stop == 0 || Velocity <= 0.0) {
		return result;
	}
	auto const stiffness{series_stiffness(First.link_stiffness, Second.link_stiffness)};
	result.force = State.stop_force + impact_force(Velocity, stiffness, Mass1, Mass2);
	if (State.stop > 0) {
		auto const link{weaker_link(First, Second)};
		result.broken = link.breakable && result.force >= link.ultimate;
		if (false == result.broken) {
			State.elongation = plastic_elongation(link, result.force, State.elongation);
		}
	}
	else {
		result.crushed = result.force >= std::min(First.crush_force, Second.crush_force);
	}
	return result;
}

} // namespace coupler_physics
