/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/TrackCheck.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace track_check {

namespace {

constexpr double kmh{3.6};

reference make_reference(std::string const &Name, double const Mass, double const Length, double const AxleBase, double const BodyHeight, derailment::vehicle_class const Class) {
	reference result;
	result.name = Name;
	result.vehicle.mass = Mass;
	result.vehicle.axles = 4;
	result.vehicle.length = Length;
	result.vehicle.axle_base = AxleBase;
	result.vehicle.centre_of_gravity = derailment::default_centre_of_gravity(BodyHeight, Class);
	return result;
}

// the cant [rad] of the given height of the outer rail [mm]
double cant_angle(double const Cant, double const Gauge) {
	return std::asin(std::clamp(std::abs(Cant) / (1000.0 * derailment::contact_spacing(Gauge)), 0.0, 1.0));
}

void append(std::string &Text, std::string const &Part) {
	if (false == Text.empty()) {
		Text += "; ";
	}
	Text += Part;
}

// the values of the unevenness past the range, as "side 40, cross 30"
std::string uneven_excess(track_uneven::attribute const &Uneven, settings const &Settings) {
	std::string result;
	auto const test = [&](char const *Name, float const Value, double const Most) {
		if (Value > Most) {
			if (false == result.empty()) {
				result += ", ";
			}
			result += std::format("{} {:g}", Name, Value);
		}
	};
	test("value", Uneven.value, Settings.most_uneven);
	test("up", Uneven.up, Settings.most_uneven);
	test("side", Uneven.side, Settings.most_uneven);
	test("cross", Uneven.cross, Settings.most_cross);
	test("dip", Uneven.dip, Settings.most_uneven);
	test("branch", Uneven.branch, Settings.most_uneven);
	test("branch up", Uneven.branchup, Settings.most_uneven);
	test("branch side", Uneven.branchside, Settings.most_uneven);
	return result;
}

} // namespace

std::vector<reference> const &default_references() {
	static std::vector<reference> const references{
	    make_reference("EU07", 80000.0, 15.915, 3.0, 4.348, derailment::vehicle_class::powered),
	    make_reference("coach", 40000.0, 24.5, 2.5, 4.05, derailment::vehicle_class::unpowered),
	};
	return references;
}

std::string_view name_of(kind const Kind) {
	switch (Kind) {
	case kind::overturning:
		return "overturning";
	case kind::climbing:
		return "flange climbing";
	case kind::overturning_default:
		return "overturning at the default speed";
	case kind::climbing_default:
		return "flange climbing at the default speed";
	case kind::cant_on_straight:
		return "cant on a straight";
	case kind::cant_excess:
		return "cant too high";
	case kind::tight_curve:
		return "tight curve too fast";
	case kind::uneven:
		return "uneven out of range";
	default:
		return "?";
	}
}

findings check(track const &Track, std::vector<reference> const &References, settings const &Settings) {
	findings result;
	auto const found = [&](kind const Kind, std::string const &Text) {
		if (std::find(result.kinds.begin(), result.kinds.end(), Kind) == result.kinds.end()) {
			result.kinds.push_back(Kind);
		}
		append(result.text, Text);
	};
	auto const curved{Track.radius > 0.0};
	auto const given{Track.velocity >= 0.0};
	auto const velocity{given ? Track.velocity : Settings.default_velocity};

	if (Track.plain) {
		// the speed in the curve, with the cant both ends have at the least
		if (curved && velocity > 0.0) {
			auto const cant{std::min(std::abs(Track.cant[0]), std::abs(Track.cant[1]))};
			derailment::track_state track;
			track.radius = Track.radius;
			track.cant = cant_angle(cant, Track.gauge);
			track.gauge = Track.gauge;
			for (auto const &reference : References) {
				auto const overturning{derailment::overturning_speed(track.radius, track.cant, reference.vehicle.centre_of_gravity, track.gauge) * kmh};
				if (overturning > 0.0 && velocity > overturning) {
					found(given ? kind::overturning : kind::overturning_default,
					      std::format("{} {:.0f} km/h over the overturning speed of {} {:.0f} km/h", given ? "speed limit" : "default speed", velocity, reference.name, overturning));
				}
				if (auto const climbing{derailment::climbing_speed(reference.vehicle, track)}; climbing.has_value() && velocity > *climbing * kmh) {
					found(given ? kind::climbing : kind::climbing_default,
					      std::format("{} {:.0f} km/h over the flange climbing speed of {} {:.0f} km/h", given ? "speed limit" : "default speed", velocity, reference.name, *climbing * kmh));
				}
			}
		}
		// cant held over the whole of a straight track: a transition has it at one end only
		if (false == curved && std::abs(Track.cant[0]) > Settings.straight_cant && std::abs(Track.cant[1]) > Settings.straight_cant && Track.cant[0] * Track.cant[1] > 0.0) {
			found(kind::cant_on_straight, std::format("cant {:.0f} / {:.0f} mm on a straight track", Track.cant[0], Track.cant[1]));
		}
		if (curved && velocity > Settings.tight_velocity && Track.radius < Settings.tight_radius) {
			found(kind::tight_curve, std::format("radius {:.0f} m with a {} of {:.0f} km/h", Track.radius, given ? "speed limit" : "default speed", velocity));
		}
	}
	if (auto const most{std::max(std::abs(Track.cant[0]), std::abs(Track.cant[1]))}; most > Settings.most_cant) {
		found(kind::cant_excess, std::format("cant {:.0f} mm over {:.0f} mm", most, Settings.most_cant));
	}
	if (auto const excess{uneven_excess(Track.uneven, Settings)}; false == excess.empty()) {
		found(kind::uneven, std::format("uneven {} mm out of range", excess));
	}
	if (false == result.empty()) {
		result.text += std::format(" (radius {:.0f} m, cant {:.0f} / {:.0f} mm, speed limit {}, damage {})", Track.radius, Track.cant[0], Track.cant[1],
		                           given ? std::format("{:.0f} km/h", Track.velocity) : std::string{"none"}, Track.damage);
	}
	return result;
}

summary scan(std::vector<track> const &Tracks, sink const &Log, std::vector<reference> const &References, settings const &Settings) {
	summary result;
	for (auto const &track : Tracks) {
		++result.tracks;
		if (track.damage != 0) {
			++result.damage[track.damage];
		}
		auto const found{check(track, References, Settings)};
		if (found.empty()) {
			continue;
		}
		++result.suspicious;
		for (auto const kind : found.kinds) {
			++result.counts[static_cast<std::size_t>(kind)];
		}
		Log(std::format("trackcheck: {}: {}", track.name.empty() ? std::string{"(none)"} : track.name, found.text));
	}
	std::string counts;
	for (std::size_t kind = 0; kind < result.counts.size(); ++kind) {
		append(counts, std::format("{} {}", name_of(static_cast<track_check::kind>(kind)), result.counts[kind]));
	}
	Log(std::format("trackcheck: {} rail tracks checked, {} suspicious: {}", result.tracks, result.suspicious, counts));
	std::string damage;
	for (auto const &[flags, count] : result.damage) {
		append(damage, std::format("{} on {}", flags, count));
	}
	Log(std::format("trackcheck: damage flags: {}", damage.empty() ? std::string{"none"} : damage));
	return result;
}

} // namespace track_check
