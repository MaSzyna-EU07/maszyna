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

// what the check of a track found so far
struct collector {
	findings &result;
	void add(kind const Kind, std::string const &Text) const {
		if (std::ranges::find(result.kinds, Kind) == result.kinds.end()) {
			result.kinds.push_back(Kind);
		}
		append(result.text, Text);
	}
};

derailment::track_state curve_of(track const &Track) {
	// with the cant both ends have at the least
	auto const cant{std::min(std::abs(Track.cant[0]), std::abs(Track.cant[1]))};
	derailment::track_state result;
	result.radius = Track.radius;
	result.cant = cant_angle(cant, Track.gauge);
	result.gauge = Track.gauge;
	return result;
}

// a speed limit over the speed a reference vehicle overturns or climbs the rail at in the curve
void check_limit(track const &Track, std::vector<reference> const &References, collector const &Found) {
	auto const curve{curve_of(Track)};
	for (auto const &reference : References) {
		auto const overturning{derailment::overturning_speed(curve.radius, curve.cant, reference.vehicle.centre_of_gravity, curve.gauge) * kmh};
		if (overturning > 0.0 && Track.velocity > overturning) {
			Found.add(kind::overturning, std::format("speed limit {:.0f} km/h over the overturning speed of {} {:.0f} km/h", Track.velocity, reference.name, overturning));
		}
		if (auto const climbing{derailment::climbing_speed(reference.vehicle, curve)}; climbing.has_value() && Track.velocity > *climbing * kmh) {
			Found.add(kind::climbing, std::format("speed limit {:.0f} km/h over the flange climbing speed of {} {:.0f} km/h", Track.velocity, reference.name, *climbing * kmh));
		}
	}
}

// a curve without a speed limit: how fast it takes the reference vehicles, when that's under the unlimited velocity
void check_unlimited(track const &Track, std::vector<reference> const &References, settings const &Settings, collector const &Found) {
	auto const curve{curve_of(Track)};
	auto slowest{Settings.unlimited_velocity};
	std::string what;
	for (auto const &reference : References) {
		auto const overturning{derailment::overturning_speed(curve.radius, curve.cant, reference.vehicle.centre_of_gravity, curve.gauge) * kmh};
		if (overturning > 0.0 && overturning < slowest) {
			slowest = overturning;
			what = std::format("the overturning speed of {}", reference.name);
		}
		if (auto const climbing{derailment::climbing_speed(reference.vehicle, curve)}; climbing.has_value() && *climbing * kmh < slowest) {
			slowest = *climbing * kmh;
			what = std::format("the flange climbing speed of {}", reference.name);
		}
	}
	if (what.empty()) {
		return;
	}
	Found.result.unlimited_speed = slowest;
	Found.add(kind::unlimited_curve, std::format("no speed limit set (not a defect): the curve takes at most {:.0f} km/h, {}", slowest, what));
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
	case kind::cant_on_straight:
		return "cant on a straight";
	case kind::cant_excess:
		return "cant too high";
	case kind::tight_curve:
		return "tight curve too fast";
	case kind::uneven:
		return "uneven out of range";
	case kind::unlimited_curve:
		return "curve without a speed limit";
	default:
		return "?";
	}
}

bool is_defect(kind const Kind) {
	return Kind != kind::unlimited_curve;
}

bool findings::defective() const {
	return std::ranges::any_of(kinds, [](kind const Kind) { return is_defect(Kind); });
}

findings check(track const &Track, std::vector<reference> const &References, settings const &Settings) {
	findings result;
	collector const found{result};
	auto const curved{Track.radius > 0.0};
	auto const given{Track.velocity >= 0.0};

	if (Track.plain) {
		// a closed track (speed 0) isn't run on
		if (curved && given && Track.velocity > 0.0) {
			check_limit(Track, References, found);
		}
		if (curved && false == given) {
			check_unlimited(Track, References, Settings, found);
		}
		// cant held over the whole of a straight track: a transition has it at one end only
		if (false == curved && std::abs(Track.cant[0]) > Settings.straight_cant && std::abs(Track.cant[1]) > Settings.straight_cant && Track.cant[0] * Track.cant[1] > 0.0) {
			found.add(kind::cant_on_straight, std::format("cant {:.0f} / {:.0f} mm on a straight track", Track.cant[0], Track.cant[1]));
		}
		if (curved && given && Track.velocity > Settings.tight_velocity && Track.radius < Settings.tight_radius) {
			found.add(kind::tight_curve, std::format("radius {:.0f} m with a speed limit of {:.0f} km/h", Track.radius, Track.velocity));
		}
	}
	if (auto const most{std::max(std::abs(Track.cant[0]), std::abs(Track.cant[1]))}; most > Settings.most_cant) {
		found.add(kind::cant_excess, std::format("cant {:.0f} mm over {:.0f} mm", most, Settings.most_cant));
	}
	if (auto const excess{uneven_excess(Track.uneven, Settings)}; false == excess.empty()) {
		found.add(kind::uneven, std::format("uneven {} mm out of range", excess));
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
		if (found.defective()) {
			++result.suspicious;
		}
		if (found.unlimited_speed > 0.0) {
			++result.unlimited;
			result.slowest = result.slowest > 0.0 ? std::min(result.slowest, found.unlimited_speed) : found.unlimited_speed;
		}
		for (auto const kind : found.kinds) {
			++result.counts[static_cast<std::size_t>(kind)];
		}
		Log(std::format("trackcheck: {}: {}", track.name.empty() ? std::string{"(none)"} : track.name, found.text));
	}
	std::string counts;
	for (std::size_t index = 0; index < result.counts.size(); ++index) {
		if (auto const kind{static_cast<track_check::kind>(index)}; is_defect(kind)) {
			append(counts, std::format("{} {}", name_of(kind), result.counts[index]));
		}
	}
	Log(std::format("trackcheck: {} rail tracks checked, {} suspicious: {}", result.tracks, result.suspicious, counts));
	Log(std::format("trackcheck: curves without a speed limit: {} taking the reference vehicles under {:.0f} km/h (not a defect){}", result.unlimited, Settings.unlimited_velocity,
	                result.unlimited > 0 ? std::format(", the slowest at {:.0f} km/h", result.slowest) : std::string{}));
	std::string damage;
	for (auto const &[flags, count] : result.damage) {
		append(damage, std::format("{} on {}", flags, count));
	}
	Log(std::format("trackcheck: damage flags: {}", damage.empty() ? std::string{"none"} : damage));
	return result;
}

} // namespace track_check
