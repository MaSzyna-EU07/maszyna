#include "stdafx.h"
#include "doctest.h"
#include "McZapkie/derailment.h"

#include <cmath>
#include <optional>

// the criteria of derailment (McZapkie/derailment.h): overturning by the lateral acceleration the cant doesn't
// balance, the flange climbing the rail by Nadal's Y/Q, either held over 2 m

namespace dr = derailment;

namespace {

constexpr double kmh{1.0 / 3.6};

// the cant [rad] of the height of the outer rail [mm] on the standard gauge
double cant(double const Millimetres) {
	return std::asin(Millimetres / 1000.0 / dr::contact_spacing(1.435));
}

// the centres of gravity the vehicles of data/fiz get without a CoGH of their own
// EU07 (4E): 80 t, a body 4.348 m high
double const eu07{dr::default_centre_of_gravity(4.348, dr::vehicle_class::powered)};
// the 430W coal wagon: 20.5 t, a body 3.235 m high, 59.5 t of coal
double const coal_wagon{dr::default_centre_of_gravity(3.235, dr::vehicle_class::unpowered)};
double const coal_wagon_loaded{dr::loaded_centre_of_gravity(coal_wagon, 20500.0, 59500.0, 1.0, 0.0, 3.235)};

} // namespace

TEST_CASE("derailment: Nadal's limit of Y/Q") {
	// 70 degrees of the flange: 1.34 with a friction of 0.3, 1.2 with 0.36, the limit the criterion takes
	CHECK(dr::nadal_limit(dr::flange_angle, 0.3) == doctest::Approx(1.342).epsilon(0.001));
	CHECK(dr::nadal_limit(dr::flange_angle, 0.36) == doctest::Approx(dr::climbing_limit).epsilon(0.01));
	// a steeper flange holds more, a higher friction less
	CHECK(dr::nadal_limit(dr::flange_angle + 0.05, 0.3) > dr::nadal_limit(dr::flange_angle, 0.3));
	CHECK(dr::nadal_limit(dr::flange_angle, 0.5) < dr::nadal_limit(dr::flange_angle, 0.3));
}

TEST_CASE("derailment: the centre of gravity") {
	CHECK(eu07 == doctest::Approx(1.65).epsilon(0.01));
	CHECK(coal_wagon == doctest::Approx(1.13).epsilon(0.01));
	// the load raises it: a full coal wagon is near 2 m, the limit of the railways for a loaded wagon is 2 to 2.5 m
	CHECK(coal_wagon_loaded == doctest::Approx(1.94).epsilon(0.01));
	CHECK(dr::loaded_centre_of_gravity(coal_wagon, 20500.0, 30000.0, 0.5, 0.0, 3.235) > coal_wagon);
	CHECK(dr::loaded_centre_of_gravity(coal_wagon, 20500.0, 30000.0, 0.5, 0.0, 3.235) < coal_wagon_loaded);
	CHECK(dr::loaded_centre_of_gravity(coal_wagon, 20500.0, 0.0, 0.0, 0.0, 3.235) == coal_wagon);
	// a body height of 2 m or less is the floor, the older meaning of the key: a 4 m high body then
	CHECK(dr::default_centre_of_gravity(1.2, dr::vehicle_class::unpowered) == doctest::Approx(1.4));
}

TEST_CASE("derailment: the overturning speed by the radius and the cant") {
	struct expected {
		char const *vehicle;
		double centre;
		double radius;
		double cant; // mm
		double speed; // km/h
	};
	// EU07 overturns at 108 km/h in a 300 m curve without cant, as the legacy criterion had it near 112 km/h; at
	// 119 km/h with a cant of 100 mm. a full coal wagon at 98 km/h, an empty one at 137 km/h
	for (auto const &[name, centre, radius, height, speed] : {
	         expected{"EU07", eu07, 150.0, 0.0, 77.6},
	         expected{"EU07", eu07, 300.0, 0.0, 107.8},
	         expected{"EU07", eu07, 300.0, 100.0, 119.0},
	         expected{"EU07", eu07, 600.0, 150.0, 175.8},
	         expected{"430W empty", coal_wagon, 300.0, 0.0, 136.9},
	         expected{"430W empty", coal_wagon, 300.0, 100.0, 145.9},
	         expected{"430W loaded", coal_wagon_loaded, 300.0, 0.0, 97.5},
	         expected{"430W loaded", coal_wagon_loaded, 300.0, 100.0, 109.6},
	         expected{"430W loaded", coal_wagon_loaded, 1200.0, 150.0, 230.5},
	     }) {
		CAPTURE(name);
		CAPTURE(radius);
		CAPTURE(height);
		CHECK(dr::overturning_speed(radius, cant(height), centre, 1.435) / kmh == doctest::Approx(speed).epsilon(0.002));
	}
	// on the faster side of 100 km/h, where the vibration is all there: v^2 / R - g sin(cant) + 0.05 g = g s / 2 h'
	auto const leaning{eu07 + 0.4 * (eu07 - 0.5)};
	auto const limit{dr::gravity * 1.5 / (2.0 * leaning)};
	CHECK(dr::overturning_acceleration(eu07, 1.5) == doctest::Approx(limit));
	auto const speed{std::sqrt((limit - dr::vibration + dr::gravity * std::sin(cant(100.0))) * 300.0)};
	CHECK(dr::overturning_speed(300.0, cant(100.0), eu07, 1.435) == doctest::Approx(speed).epsilon(0.001));
	// the legacy criterion, the lateral acceleration over g times the gauge to the height of the vehicle, without cant
	auto const legacy{std::sqrt(dr::gravity * 1.435 / 4.348 * 300.0)};
	CHECK(dr::overturning_speed(300.0, 0.0, eu07, 1.435) == doctest::Approx(legacy).epsilon(0.05));
	// faster on a wider curve, as its square root
	CHECK(dr::overturning_speed(1200.0, 0.0, eu07, 1.435) / dr::overturning_speed(300.0, 0.0, eu07, 1.435) == doctest::Approx(2.0).epsilon(0.001));
	// the cant only helps
	CHECK(dr::overturning_speed(300.0, cant(150.0), eu07, 1.435) > dr::overturning_speed(300.0, cant(60.0), eu07, 1.435));
	// no overturning on a straight track
	CHECK(dr::overturning_speed(0.0, 0.0, eu07, 1.435) == 0.0);
}

TEST_CASE("derailment: below the balancing speed the cant leans the vehicle into the curve") {
	// 100 mm on the 1.5 m between the contact points of the wheels
	CHECK(dr::unbalanced_acceleration(0.0, 300.0, cant(100.0)) == doctest::Approx(-dr::gravity * 0.1 / 1.5));
	// at the balancing speed nothing's left
	auto const balanced{std::sqrt(dr::gravity * 0.1 / 1.5 * 300.0)};
	CHECK(dr::unbalanced_acceleration(balanced, 300.0, cant(100.0)) == doctest::Approx(0.0).epsilon(1e-9));
	// the sign of the radius and of the cant doesn't matter
	CHECK(dr::unbalanced_acceleration(30.0, -300.0, -cant(100.0)) == doctest::Approx(dr::unbalanced_acceleration(30.0, 300.0, cant(100.0))));
}

TEST_CASE("derailment: the decision and the place don't depend on the step") {
	// EU07 runs at a steady speed into a 300 m curve without cant 100 m ahead, the track checked at the end of each step
	dr::vehicle_state eu07vehicle;
	eu07vehicle.mass = 80000.0;
	eu07vehicle.axles = 4;
	eu07vehicle.length = 15.915;
	eu07vehicle.axle_base = 3.05;
	eu07vehicle.centre_of_gravity = eu07;
	auto const critical{dr::overturning_speed(300.0, 0.0, eu07, 1.435)};
	constexpr double curve{100.0};

	// where the vehicle overturns, if it does within 400 m
	auto const run = [&](double const Speed, double const Step) -> std::optional<double> {
		dr::monitor monitor;
		dr::track_state track;
		auto distance{0.0};
		while (distance < 400.0) {
			auto const travelled{Speed * Step};
			distance += travelled;
			track.radius = distance >= curve ? 300.0 : 0.0;
			if (monitor.update(eu07vehicle, track, Speed, travelled).result == dr::outcome::overturned) {
				return distance;
			}
		}
		return std::nullopt;
	};

	for (auto const step : {0.01, 0.05, 0.5}) {
		CAPTURE(step);
		// 5 % too fast: overturns 2 m into the curve, give or take the step it got there in
		auto const fast{run(1.05 * critical, step)};
		REQUIRE(fast.has_value());
		CHECK(*fast >= curve + dr::window - 1.05 * critical * step - 1e-6);
		CHECK(*fast <= curve + dr::window + 1.05 * critical * step + 1e-6);
		// 5 % below it: never
		CHECK_FALSE(run(0.95 * critical, step).has_value());
	}
	auto const fine{run(1.05 * critical, 0.01)};
	auto const coarse{run(1.05 * critical, 0.5)};
	REQUIRE(fine.has_value());
	REQUIRE(coarse.has_value());
	CHECK(std::abs(*coarse - *fine) <= 1.05 * critical * 0.5);
}

TEST_CASE("derailment: a peak shorter than the window doesn't derail") {
	dr::stretch stretch;
	CHECK_FALSE(stretch.update(true, 1.5));
	CHECK_FALSE(stretch.update(false, 0.5));
	CHECK_FALSE(stretch.update(true, 1.5));
	CHECK(stretch.update(true, 0.5));
	CHECK(stretch.distance() == doctest::Approx(2.0));
}

TEST_CASE("derailment: a vehicle without a mass never derails") {
	// a definition without M (or with all of it in the rotating masses) gives no forces, not a lifted wheel
	dr::vehicle_state massless;
	massless.mass = 0.0;
	dr::track_state track;
	track.radius = 150.0;
	CHECK(dr::flange_forces(massless, track, 30.0).ratio() == 0.0);
	dr::monitor monitor;
	for (int step = 0; step < 10; ++step) {
		CHECK(monitor.update(massless, track, 30.0, 1.0).result == dr::outcome::none);
	}
}

TEST_CASE("derailment: pushing a light wagon between heavy vehicles in a curve") {
	// the empty 3WL (10.5 t, 2 axles 6 m apart, 9.87 m long) between two EU07 in a 150 m curve: the more the train
	// pushes, the more the couplings push the wagon out of the curve, the higher its Y/Q
	dr::vehicle_state wagon;
	wagon.mass = 10500.0;
	wagon.axles = 2;
	wagon.length = 9.87;
	wagon.axle_base = 6.0;
	wagon.centre_of_gravity = dr::default_centre_of_gravity(3.0, dr::vehicle_class::unpowered);
	dr::track_state track;
	track.radius = 150.0;
	auto const speed{20.0 * kmh};

	auto const ratio = [&](dr::vehicle_state Vehicle, double const Compression) {
		for (auto &end : Vehicle.ends) {
			end.compression = Compression;
			end.other_length = 15.915;
		}
		return dr::flange_forces(Vehicle, track, speed).ratio();
	};
	auto previous{ratio(wagon, 0.0)};
	// the leading axle guides the wagon with the friction of its inner wheel: over 0.3 even unpushed
	CHECK(previous > dr::tread_friction);
	CHECK(previous < 0.4);
	for (auto const compression : {100e3, 300e3, 600e3, 1000e3}) {
		CAPTURE(compression);
		auto const pushed{ratio(wagon, compression)};
		CHECK(pushed > previous);
		previous = pushed;
	}
	// a loaded 430W in its place is pushed out less
	dr::vehicle_state loaded;
	loaded.mass = 80000.0;
	loaded.axles = 4;
	loaded.length = 14.028;
	loaded.axle_base = 2.0;
	loaded.centre_of_gravity = coal_wagon_loaded;
	CHECK(ratio(loaded, 600e3) < ratio(wagon, 600e3));
	// pulled, the couplings draw the wagon into the curve: the flange of the inner wheel then
	auto pulled{wagon};
	for (auto &end : pulled.ends) {
		end.compression = -600e3;
		end.other_length = 15.915;
	}
	CHECK(dr::flange_forces(pulled, track, speed).lateral < 0.0);
	// on a twisted track the wheel is lighter, so the same push comes nearer the limit
	auto const twisted{[&] {
		auto const flat{ratio(wagon, 600e3)};
		track.uneven_cross = 30.0;
		auto const result{ratio(wagon, 600e3)};
		track.uneven_cross = 0.0;
		return result > flat;
	}()};
	CHECK(twisted);
	// on a bent rail, with a twist of 50 mm, the wagon pushed with 300 kN climbs the rail over 2 m, without falling over
	track.damage = dr::track_damage::railbend;
	track.uneven_cross = 50.0;
	dr::monitor monitor;
	auto pushed{wagon};
	for (auto &end : pushed.ends) {
		end.compression = 300e3;
		end.other_length = 15.915;
	}
	CHECK(monitor.update(pushed, track, speed, 1.0).result == dr::outcome::none);
	auto const verdict{monitor.update(pushed, track, speed, 1.0)};
	CAPTURE(verdict.flange_ratio);
	CAPTURE(verdict.unloading);
	CHECK(verdict.unloading < 1.0);
	CHECK(verdict.result == dr::outcome::off_rails);
}

TEST_CASE("derailment: the damage flags of the track make the ride rougher, not the curve sharper") {
	// line 107 has damage 31 on every track, for the jolts. a 401Ka wagon creeping at 28 km/h through a 282 m curve
	// with a cant of 179 mm, pulled by a freight train, stays on: in the 2026-11-15 run with the flags scaling the
	// whole lateral acceleration it overturned into the curve
	dr::vehicle_state wagon;
	wagon.mass = 23000.0;
	wagon.axles = 4;
	wagon.length = 14.7;
	wagon.axle_base = 1.8;
	wagon.centre_of_gravity = dr::default_centre_of_gravity(4.2, dr::vehicle_class::unpowered);
	for (auto &end : wagon.ends) {
		end.compression = -200e3;
		end.other_length = 14.7;
	}
	dr::track_state track;
	track.radius = 282.0;
	track.cant = std::asin(0.179 / 1.5);
	auto const speed{28.0 * kmh};
	auto const smooth{dr::body_acceleration(wagon, track, speed)};
	track.damage = 31;
	auto const rough{dr::body_acceleration(wagon, track, speed)};
	// the sway grows, by what a rough track adds to the sway at that speed
	CHECK(rough > smooth);
	CHECK(rough - smooth == doctest::Approx(1.1 * dr::body_vibration(speed)));
	CHECK(dr::unloading_ratio(rough, wagon.centre_of_gravity, 1.5) < 0.5);
	CHECK(dr::flange_forces(wagon, track, speed).ratio() < 0.8);
	// at speed the rough track still lowers the overturning speed some, as the legacy criterion's factor did
	{
		dr::vehicle_state eu07vehicle;
		eu07vehicle.mass = 80000.0;
		eu07vehicle.centre_of_gravity = eu07;
		dr::track_state good;
		good.radius = 300.0;
		auto worn{good};
		worn.damage = 31;
		CHECK(dr::body_acceleration(eu07vehicle, worn, 30.0) > dr::body_acceleration(eu07vehicle, good, 30.0));
		CHECK(dr::body_acceleration(eu07vehicle, worn, 30.0) - dr::body_acceleration(eu07vehicle, good, 30.0) == doctest::Approx(1.1 * dr::vibration));
	}
}
