#include "stdafx.h"
#include "doctest.h"

#include "McZapkie/hamulce.h"
#include "vehicle/brakesetting.h"

// the brake settings the AI wagon inspector (TController::AutoRewident) chooses for a train, by the
// brake instruction Cw-1 (Mw56) §18 and §19 (eu07/maszyna#350)

namespace {

using brake_setting::train_setting;

constexpr int G{bdelay_G};
constexpr int P{bdelay_P};
constexpr int R{bdelay_R};
constexpr int M{bdelay_M};

brake_setting::vehicle locomotive(int const Delays = G | P | R) {
	return {Delays, true, 17.0, 80000.0};
}
// a freight wagon of the given length [m] and mass [t]
brake_setting::vehicle freight_wagon(double const Length = 15.0, double const Tonnes = 80.0) {
	return {G | P, false, Length, Tonnes * 1000.0};
}
brake_setting::vehicle passenger_car(int const Delays = P | R) {
	return {Delays, false, 24.5, 45000.0};
}

std::vector<brake_setting::vehicle> train(brake_setting::vehicle const &Locomotive, int const Count, brake_setting::vehicle const &Wagon) {
	std::vector<brake_setting::vehicle> consist{Locomotive};
	consist.insert(consist.end(), Count, Wagon);
	return consist;
}

} // namespace

TEST_CASE("brake setting: a freight train runs on P only up to 500 m and 1300 t of wagons (Cw-1 §19.2)") {
	// 25 wagons of 20 m and 52 t: 500 m and 1300 t exactly, the limits included
	CHECK(brake_setting::choose_train_setting(train(locomotive(), 25, freight_wagon(20.0, 52.0))) == train_setting::freight_gp);
	// one metre longer
	CHECK(brake_setting::choose_train_setting(train(locomotive(), 25, freight_wagon(20.04, 52.0))) == train_setting::freight_g);
	// one tonne heavier
	CHECK(brake_setting::choose_train_setting(train(locomotive(), 25, freight_wagon(20.0, 52.04))) == train_setting::freight_g);
}

TEST_CASE("brake setting: the locomotives don't count into the length and mass of a freight train's wagons") {
	// 300 m and 600 t of wagons behind two locomotives of 80 t: all the wagons on P (Cw-1 §19.3a)
	auto consist{train(locomotive(), 20, freight_wagon(15.0, 30.0))};
	consist.insert(consist.begin(), locomotive());
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::freight_p);
	auto const settings{brake_setting::choose(consist)};
	CHECK(settings[0] == G);
	CHECK(settings[1] == G);
	for (std::size_t i = 2; i < settings.size(); ++i) {
		CAPTURE(i);
		CHECK(settings[i] == P);
	}
	// a wagon more makes it the long locomotive setting (Cw-1 §19.3b)
	consist.emplace_back(freight_wagon(15.0, 30.0));
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::freight_gp);
}

TEST_CASE("brake setting: the locomotive and the first five wagons on G, the rest on P (Cw-1 §19.3b)") {
	auto const consist{train(locomotive(), 12, freight_wagon(15.0, 60.0))};
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_gp);
	auto const settings{brake_setting::choose(consist)};
	REQUIRE(settings.size() == consist.size());
	for (std::size_t i = 0; i < settings.size(); ++i) {
		CAPTURE(i);
		CHECK(settings[i] == (i <= 5 ? G : P));
	}
}

TEST_CASE("brake setting: a locomotive in the middle of the train doesn't put five more wagons on G") {
	auto consist{train(locomotive(), 8, freight_wagon(15.0, 60.0))};
	consist.emplace_back(locomotive());
	consist.insert(consist.end(), 8, freight_wagon(15.0, 60.0));
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_gp);
	auto const settings{brake_setting::choose(consist)};
	CHECK(settings[1] == G);
	CHECK(settings[5] == G);
	CHECK(settings[6] == P);
	CHECK(settings[9] == G); // the locomotive
	CHECK(settings[10] == P);

	// the inspector of the earlier releases started counting anew at it
	auto const legacy{brake_setting::choose_legacy(consist)};
	CHECK(legacy[10] == G);
	CHECK(legacy[14] == G);
	CHECK(legacy[15] == P);
}

TEST_CASE("brake setting: a heavy freight train runs on G, a vehicle without G on P") {
	auto consist{train(locomotive(), 30, freight_wagon(15.0, 80.0))};
	consist.emplace_back(passenger_car(P));
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_g);
	auto const settings{brake_setting::choose(consist)};
	for (std::size_t i = 0; i + 1 < settings.size(); ++i) {
		CAPTURE(i);
		CHECK(settings[i] == G);
	}
	CHECK(settings.back() == P);
}

TEST_CASE("brake setting: a vehicle without the setting the train needs gets the other one of G and P") {
	// a light freight train with a wagon which has G only
	auto consist{train(locomotive(), 4, freight_wagon(15.0, 20.0))};
	consist.emplace_back(G, false, 15.0, 20000.0);
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_p);
	CHECK(brake_setting::choose(consist).back() == G);
	// and a vehicle with neither is left alone
	consist.emplace_back(R, false, 15.0, 20000.0);
	CHECK(brake_setting::choose(consist).back() == 0);
}

TEST_CASE("brake setting: a passenger train on R carries at most three cars on P, fewer than those on R (Cw-1 §18.3)") {
	auto consist{train(locomotive(), 4, passenger_car())};
	consist.insert(consist.end(), 3, passenger_car(P));
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::passenger_r);
	auto const settings{brake_setting::choose(consist)};
	CHECK(settings[0] == R);
	CHECK(settings[1] == R);
	CHECK(settings.back() == P);

	// as many cars on P as on R: all on P
	consist.emplace_back(passenger_car(P));
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::passenger_p);
	for (auto const setting : brake_setting::choose(consist)) {
		CHECK(setting == P);
	}
	// the inspector of the earlier releases kept such a train on R
	CHECK(brake_setting::choose_legacy(consist)[1] == R);

	// four cars on P are too many even with more on R
	consist.insert(consist.end(), 6, passenger_car());
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::passenger_p);
}

TEST_CASE("brake setting: freight wagons in a passenger train on R count as cars on P (Cw-1 §18.4)") {
	auto consist{train(locomotive(), 6, passenger_car())};
	consist.emplace_back(freight_wagon());
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::passenger_r);
	CHECK(brake_setting::choose(consist).back() == P);
}

TEST_CASE("brake setting: cars with a magnetic rail brake run on R+Mg in a train on R (Cw-1 §18.2)") {
	auto consist{train(locomotive(G | P | R | M), 3, passenger_car(P | R | M))};
	consist.emplace_back(passenger_car());
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::passenger_r);
	auto const settings{brake_setting::choose(consist)};
	CHECK(settings[0] == R); // the locomotive stays on R
	CHECK(settings[1] == (R | M));
	CHECK(settings.back() == R);
}

TEST_CASE("brake setting: a light engine runs on R where it has it") {
	std::vector<brake_setting::vehicle> const consist{locomotive(), locomotive(G | P)};
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::light_engine);
	auto const settings{brake_setting::choose(consist)};
	CHECK(settings[0] == R);
	CHECK(settings[1] == P);
}

TEST_CASE("brake setting: the inspector of the earlier releases counted the locomotives into the train's mass") {
	// 300 m and 600 t of wagons: P by Cw-1, the long locomotive setting before
	auto const consist{train(locomotive(), 20, freight_wagon(15.0, 30.0))};
	CHECK(brake_setting::choose(consist)[1] == P);
	CHECK(brake_setting::choose_legacy(consist)[1] == G);
	CHECK(brake_setting::choose_legacy(consist)[6] == P);
}
