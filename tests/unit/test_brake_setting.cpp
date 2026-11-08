#include "stdafx.h"
#include "doctest.h"

#include "McZapkie/MOVER.h"
#include "McZapkie/hamulce.h"
#include "vehicle.h"
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

TEST_CASE("brake setting: a heavy freight train runs on G, a wagon without G has its brake cut out (Cw-1 §19.9)") {
	auto consist{train(locomotive(), 30, freight_wagon(15.0, 80.0))};
	consist.emplace_back(passenger_car(P));
	consist.emplace_back(passenger_car());
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_g);
	auto const settings{brake_setting::choose(consist)};
	for (std::size_t i = 0; i + 2 < settings.size(); ++i) {
		CAPTURE(i);
		CHECK(settings[i] == G);
	}
	CHECK(settings[settings.size() - 2] == brake_setting::cut_out);
	CHECK(settings.back() == brake_setting::cut_out);

	// the inspector of the earlier releases put them on P
	auto const legacy{brake_setting::choose_legacy(consist)};
	CHECK(legacy[settings.size() - 2] == P);
	CHECK(legacy.back() == P);
}

TEST_CASE("brake setting: in a train on G a locomotive and the vehicle it's driven from keep a working brake") {
	auto consist{train(locomotive(P | R), 30, freight_wagon(15.0, 80.0))};
	// a dead locomotive without G at the end, and a car which tells no settings at all
	consist.emplace_back(locomotive(P | R));
	consist.emplace_back(0, false, 20.0, 40000.0);
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_g);
	auto settings{brake_setting::choose(consist)};
	CHECK(settings.front() == P);
	CHECK(settings[settings.size() - 2] == P);
	CHECK(settings.back() == 0);

	// a control car without G, the train driven from it, pushed by the locomotive at the rear
	brake_setting::vehicle controlcar{P | R, false, 24.5, 45000.0};
	controlcar.driven = true;
	consist.front() = controlcar;
	consist.back() = locomotive();
	settings = brake_setting::choose(consist);
	CHECK(settings.front() == P);
	CHECK(settings.back() == G);
}

TEST_CASE("brake setting: in the long locomotive setting a vehicle without G brakes on P, not cut out (Cw-1 §19.3b)") {
	// the train runs on P, so §19.9 doesn't apply to it, among the first five wagons kept on G or behind them
	auto consist{train(locomotive(), 2, freight_wagon(15.0, 60.0))};
	consist.emplace_back(passenger_car());
	consist.insert(consist.end(), 8, freight_wagon(15.0, 60.0));
	consist.emplace_back(passenger_car());
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_gp);
	auto const settings{brake_setting::choose(consist)};
	CHECK(settings[2] == G);
	CHECK(settings[3] == P);
	CHECK(settings[4] == G);
	CHECK(settings[6] == P);
	CHECK(settings.back() == P);
}

TEST_CASE("brake setting: a car with its brake cut out counts as one on P in a passenger train (Cw-1 §18.3)") {
	auto consist{train(locomotive(), 5, passenger_car())};
	consist.insert(consist.end(), 3, passenger_car(P));
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::passenger_r);
	// one car with R cut out makes four on P
	consist[1].cut_out = true;
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::passenger_p);
	// with fewer P-only cars, three on P and four on R still run on R
	consist.pop_back();
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::passenger_r);
	// a cut out brake changes nothing about a train being a passenger or a freight one
	auto freight{train(locomotive(), 4, freight_wagon())};
	freight.emplace_back(passenger_car());
	freight.back().cut_out = true;
	CHECK(brake_setting::choose_train_setting(freight) == train_setting::freight_p);
}

namespace {

// a passenger car with P and R, with the KE distributor of the test wagon
test::definition passenger_car_definition() {
	return test::definition::load("wagon").set("Cntrl", "BrakeDelays", "PR");
}

// charges the brake of the vehicle from a brake pipe at 5 bar, then makes a full service application
void charge_and_apply(test::vehicle &Vehicle) {
	Vehicle.hold_brake_pipe(5.0);
	Vehicle.run(90.0);
	Vehicle.hold_brake_pipe(3.5);
	Vehicle.run(20.0);
}

// the inspector at work on a heavy train on G with the vehicle at its end, as AutoRewident() does it
int inspect_heavy_train(test::vehicle &Vehicle) {
	auto consist{train(locomotive(), 30, freight_wagon(15.0, 80.0))};
	consist.emplace_back(brake_setting::describe(Vehicle.mover(), false));
	auto const setting{brake_setting::choose(consist).back()};
	brake_setting::apply(Vehicle.mover(), setting);
	return setting;
}

// the same on a light train, which runs on P
int inspect_light_train(test::vehicle &Vehicle) {
	auto consist{train(locomotive(), 4, freight_wagon())};
	consist.emplace_back(brake_setting::describe(Vehicle.mover(), false));
	auto const setting{brake_setting::choose(consist).back()};
	brake_setting::apply(Vehicle.mover(), setting);
	return setting;
}

} // namespace

TEST_CASE("brake setting: the inspector cuts the brake out in a train on G and puts it back in a train on P") {
	test::vehicle car{passenger_car_definition()};
	charge_and_apply(car);
	REQUIRE(car->BrakePress > 1.0);

	// made up into a heavy train on G: the brake is cut out and released
	REQUIRE(inspect_heavy_train(car) == brake_setting::cut_out);
	CHECK(brake_setting::is_cut_out(car.mover()));
	CHECK(car->BrakeCutOutByInspector);
	car.run(30.0);
	CHECK(car->BrakePress < 0.1);
	// and it stays off whatever the brake pipe does
	car.hold_brake_pipe(5.0);
	car.run(60.0);
	car.hold_brake_pipe(3.5);
	car.run(20.0);
	CHECK(car->BrakePress < 0.1);
	car.hold_brake_pipe(0.0);
	car.run(20.0);
	CHECK(car->BrakePress < 0.1);
	// to the inspector looking at the train again it's a car like any other, and the train needs it cut out still
	CHECK(false == brake_setting::describe(car.mover(), false).cut_out);
	REQUIRE(inspect_heavy_train(car) == brake_setting::cut_out);
	CHECK(car->BrakeCutOutByInspector);

	// made up into a light train: the brake is back on P, charges and brakes again
	CHECK(inspect_light_train(car) == P);
	CHECK(false == brake_setting::is_cut_out(car.mover()));
	CHECK(false == car->BrakeCutOutByInspector);
	CHECK(car->BrakeDelayFlag == P);
	// the reservoirs emptied when it was cut out take some five minutes to charge from the brake pipe
	car.hold_brake_pipe(5.0);
	car.run(300.0);
	CHECK(car->Hamulec->GetBRP() > 4.9);
	charge_and_apply(car);
	CHECK(car->BrakePress > 1.0);
}

TEST_CASE("brake setting: the inspector never puts back a brake cut out for another reason") {
	test::vehicle car{passenger_car_definition()};
	charge_and_apply(car);
	// cut out the way the scenery's '0' does it
	car->Hamulec->ForceEmptiness();
	car->Hamulec->SetBrakeStatus(car->Hamulec->GetBrakeStatus() | b_dmg);
	CHECK(brake_setting::describe(car.mover(), false).cut_out);

	// a train on G wants it cut out anyway: it isn't the inspector's to put back
	CHECK(inspect_heavy_train(car) == brake_setting::cut_out);
	CHECK(false == car->BrakeCutOutByInspector);
	// a train on P would want it working: it stays cut out
	CHECK(inspect_light_train(car) == P);
	CHECK(brake_setting::is_cut_out(car.mover()));
	charge_and_apply(car);
	CHECK(car->BrakePress < 0.1);

	// a brake the inspector cut out, which someone else puts back and cuts out again, is theirs from then on
	test::vehicle other{passenger_car_definition()};
	REQUIRE(inspect_heavy_train(other) == brake_setting::cut_out);
	other->Hamulec->SetBrakeStatus(other->Hamulec->GetBrakeStatus() & ~b_dmg);
	CHECK(inspect_heavy_train(other) == brake_setting::cut_out); // the inspector cuts it out again, and it's its own again
	CHECK(other->BrakeCutOutByInspector);
	other->Hamulec->SetBrakeStatus(other->Hamulec->GetBrakeStatus() & ~b_dmg);
	CHECK(inspect_light_train(other) == P);
	CHECK(false == other->BrakeCutOutByInspector);
	other->Hamulec->SetBrakeStatus(other->Hamulec->GetBrakeStatus() | b_dmg);
	CHECK(inspect_light_train(other) == P);
	CHECK(brake_setting::is_cut_out(other.mover()));
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
