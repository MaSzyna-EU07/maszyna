#include "stdafx.h"
#include "doctest.h"

#include "McZapkie/MOVER.h"
#include "McZapkie/hamulce.h"
#include "vehicle.h"
#include "vehicle/brakesetting.h"
#include "vehicle/Driver.h"

// the brake settings the AI wagon inspector (TController::AutoRewident) chooses for a train, by the
// brake instruction Cw-1 (Mw56) §18 and §19 (eu07/maszyna#350)

namespace {

using brake_setting::train_setting;

constexpr int G{bdelay_G};
constexpr int P{bdelay_P};
constexpr int R{bdelay_R};
constexpr int M{bdelay_M};

// how the AI runs the consist the inspector looks at: as a train (Obey_train or Bank), or shunting it
constexpr bool as_train{true};
constexpr bool shunting{false};
// whether the train stands, as AutoRewident() tells the inspector
constexpr bool standstill{true};
constexpr bool moving{false};

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
	auto const settings{brake_setting::choose(consist, as_train)};
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
	auto const settings{brake_setting::choose(consist, as_train)};
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
	auto const settings{brake_setting::choose(consist, as_train)};
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
	auto const settings{brake_setting::choose(consist, as_train)};
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
	auto settings{brake_setting::choose(consist, as_train)};
	CHECK(settings.front() == P);
	CHECK(settings[settings.size() - 2] == P);
	CHECK(settings.back() == 0);

	// a control car without G, the train driven from it, pushed by the locomotive at the rear
	brake_setting::vehicle controlcar{P | R, false, 24.5, 45000.0};
	controlcar.driven = true;
	consist.front() = controlcar;
	consist.back() = locomotive();
	settings = brake_setting::choose(consist, as_train);
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
	auto const settings{brake_setting::choose(consist, as_train)};
	CHECK(settings[2] == G);
	CHECK(settings[3] == P);
	CHECK(settings[4] == G);
	CHECK(settings[6] == P);
	CHECK(settings.back() == P);
}

TEST_CASE("brake setting: a heavy consist being shunted has nothing cut out, a wagon without G brakes on P") {
	// §19.9 is about trains: the same consist the AI shunts rather than runs as a train
	auto consist{train(locomotive(), 30, freight_wagon(15.0, 80.0))};
	consist.emplace_back(passenger_car(P));
	consist.emplace_back(passenger_car());
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_g);
	auto const settings{brake_setting::choose(consist, shunting)};
	for (std::size_t i = 0; i + 2 < settings.size(); ++i) {
		CAPTURE(i);
		CHECK(settings[i] == G);
	}
	CHECK(settings[settings.size() - 2] == P);
	CHECK(settings.back() == P);
	// as a train they're cut out
	CHECK(brake_setting::choose(consist, as_train).back() == brake_setting::cut_out);
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

// the inspector at work on a heavy train on G with the vehicle at its end, as AutoRewident() does it, the AI
// running it as a train or shunting it, standing or on the run
int inspect_heavy_train(test::vehicle &Vehicle, bool const Train = as_train, bool const Standstill = standstill) {
	auto consist{train(locomotive(), 30, freight_wagon(15.0, 80.0))};
	consist.emplace_back(brake_setting::describe(Vehicle.mover(), false));
	auto const setting{brake_setting::choose(consist, Train).back()};
	brake_setting::apply(Vehicle.mover(), setting, Standstill);
	return setting;
}

// the same on a light train, which runs on P
int inspect_light_train(test::vehicle &Vehicle) {
	auto consist{train(locomotive(), 4, freight_wagon())};
	consist.emplace_back(brake_setting::describe(Vehicle.mover(), false));
	auto const setting{brake_setting::choose(consist, as_train).back()};
	brake_setting::apply(Vehicle.mover(), setting, standstill);
	return setting;
}

// the inspector at work on a whole heavy train on G, the vehicles at its end, the way AutoRewident() goes through
// the train. tells whether it left a cut-out for the next standstill, as AutoRewident() notes for reinspect_brakes()
bool inspection_pending(std::vector<test::vehicle *> const &Vehicles, bool const Train, bool const Standstill) {
	auto consist{train(locomotive(), 30, freight_wagon(15.0, 80.0))};
	for (auto *vehicle : Vehicles) {
		consist.emplace_back(brake_setting::describe(vehicle->mover(), false));
	}
	auto const settings{brake_setting::choose(consist, Train)};
	auto setting{settings.end() - static_cast<std::ptrdiff_t>(Vehicles.size())};
	auto pending{false};
	for (auto *vehicle : Vehicles) {
		if (brake_setting::apply(vehicle->mover(), *setting++, Standstill)) {
			pending = true;
		}
	}
	return pending;
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

TEST_CASE("brake setting: shunting puts back a brake the inspector cut out, and cuts none out") {
	test::vehicle car{passenger_car_definition()};
	charge_and_apply(car);
	// shunted in a heavy consist: nothing is cut out, the car is on P and brakes
	CHECK(inspect_heavy_train(car, shunting) == P);
	CHECK(false == brake_setting::is_cut_out(car.mover()));
	CHECK(car->BrakeDelayFlag == P);
	CHECK(car->BrakePress > 1.0);

	// cut out when the consist became a train, then shunted again (e.g. left in a yard): the brake is back
	REQUIRE(inspect_heavy_train(car) == brake_setting::cut_out);
	REQUIRE(car->BrakeCutOutByInspector);
	CHECK(inspect_heavy_train(car, shunting) == P);
	CHECK(false == brake_setting::is_cut_out(car.mover()));
	CHECK(false == car->BrakeCutOutByInspector);
	car.hold_brake_pipe(5.0);
	car.run(300.0);
	charge_and_apply(car);
	CHECK(car->BrakePress > 1.0);

	// a brake cut out for a reason of its own stays cut out in shunting too
	test::vehicle other{passenger_car_definition()};
	other->Hamulec->SetBrakeStatus(other->Hamulec->GetBrakeStatus() | b_dmg);
	CHECK(inspect_heavy_train(other, shunting) == P);
	CHECK(brake_setting::is_cut_out(other.mover()));
}

TEST_CASE("brake setting: the inspector doesn't cut a brake out on the run, it waits for a standstill") {
	test::vehicle car{passenger_car_definition()};
	charge_and_apply(car);
	REQUIRE(car->BrakePress > 1.0);
	// the train checked on the run (the order changed mid-run): the car keeps its brake and its setting
	auto const setting{car->BrakeDelayFlag};
	CHECK(inspect_heavy_train(car, as_train, moving) == brake_setting::cut_out);
	CHECK(false == brake_setting::is_cut_out(car.mover()));
	CHECK(false == car->BrakeCutOutByInspector);
	CHECK(car->BrakeDelayFlag == setting);
	car.run(5.0);
	CHECK(car->BrakePress > 1.0);
	// the next check, standing, cuts it out
	CHECK(inspect_heavy_train(car) == brake_setting::cut_out);
	CHECK(brake_setting::is_cut_out(car.mover()));
	CHECK(car->BrakeCutOutByInspector);
	// a brake cut out already stays cut out on the run, and one put back on the run is back
	CHECK(inspect_heavy_train(car, as_train, moving) == brake_setting::cut_out);
	CHECK(car->BrakeCutOutByInspector);
	CHECK(inspect_heavy_train(car, shunting, moving) == P);
	CHECK(false == brake_setting::is_cut_out(car.mover()));
	CHECK(false == car->BrakeCutOutByInspector);
}

TEST_CASE("brake setting: a cut-out skipped on the run is reported, and done at the next inspection standing") {
	// a train formed by coupling: the AI looks at it as the buffers meet, above 1 km/h. two cars without G and
	// a freight wagon at the end of a heavy train on G
	test::vehicle first{passenger_car_definition()};
	test::vehicle second{passenger_car_definition()};
	test::vehicle wagon{test::definition::load("wagon").set("Cntrl", "BrakeDelays", "GP")};
	std::vector<test::vehicle *> const vehicles{&first, &second, &wagon};
	for (auto *vehicle : vehicles) {
		charge_and_apply(*vehicle);
	}
	auto const setting{first->BrakeDelayFlag};
	// moving: nothing is cut out, and the inspector tells it has to come back
	CHECK(inspection_pending(vehicles, as_train, brake_setting::at_standstill(3.0)));
	for (auto *vehicle : vehicles) {
		CHECK(false == brake_setting::is_cut_out(vehicle->mover()));
		CHECK(false == vehicle->mover().BrakeCutOutByInspector);
	}
	CHECK(first->BrakeDelayFlag == setting);
	CHECK(wagon->BrakeDelayFlag == G);
	// still moving: still pending
	CHECK(inspection_pending(vehicles, as_train, moving));
	// the train stands (its brake pipe charging after the coupling): the cars get cut out, nothing is left
	CHECK(false == inspection_pending(vehicles, as_train, brake_setting::at_standstill(0.0)));
	CHECK(brake_setting::is_cut_out(first.mover()));
	CHECK(first->BrakeCutOutByInspector);
	CHECK(brake_setting::is_cut_out(second.mover()));
	CHECK(second->BrakeCutOutByInspector);
	CHECK(false == brake_setting::is_cut_out(wagon.mover()));
	// and nothing is pending when the train moves off
	CHECK(false == inspection_pending(vehicles, as_train, moving));
	CHECK(brake_setting::is_cut_out(first.mover()));
}

TEST_CASE("brake setting: an inspection on the run with nothing to cut out leaves nothing pending") {
	// a wagon with G in a heavy train on G
	test::vehicle wagon{test::definition::load("wagon").set("Cntrl", "BrakeDelays", "GP")};
	CHECK(false == inspection_pending({&wagon}, as_train, moving));
	CHECK(wagon->BrakeDelayFlag == G);
	// a car without G in a consist being shunted: it keeps its brake, on P
	test::vehicle car{passenger_car_definition()};
	CHECK(false == inspection_pending({&car}, shunting, moving));
	CHECK(false == brake_setting::is_cut_out(car.mover()));
	// a car whose brake is cut out already, for a reason of its own or by the inspector
	test::vehicle damaged{passenger_car_definition()};
	damaged->Hamulec->SetBrakeStatus(damaged->Hamulec->GetBrakeStatus() | b_dmg);
	CHECK(false == inspection_pending({&damaged}, as_train, moving));
	CHECK(false == damaged->BrakeCutOutByInspector);
	test::vehicle done{passenger_car_definition()};
	REQUIRE(false == inspection_pending({&done}, as_train, standstill));
	REQUIRE(done->BrakeCutOutByInspector);
	CHECK(false == inspection_pending({&done}, as_train, moving));
	// a brake put back on the run (the train became a consist being shunted) leaves nothing either
	CHECK(false == inspection_pending({&done}, shunting, moving));
	CHECK(false == brake_setting::is_cut_out(done.mover()));
	// a single vehicle's apply() tells the same
	test::vehicle single{passenger_car_definition()};
	CHECK(brake_setting::apply(single.mover(), brake_setting::cut_out, moving));
	CHECK(false == brake_setting::apply(single.mover(), P, moving));
	CHECK(false == brake_setting::apply(single.mover(), 0, moving));
	CHECK(false == brake_setting::apply(single.mover(), brake_setting::cut_out, standstill));
	CHECK(false == brake_setting::apply(single.mover(), brake_setting::cut_out, moving));
}

TEST_CASE("brake setting: a train creeping to a stop counts as standing, one on the run or coupling doesn't") {
	// the AI looks at the train at a W4 and when it switches from shunting to the train run below 1 km/h: the
	// inspector takes those for a standstill. it couples faster than that (it drives onto the standing cut at up
	// to 4 km/h and looks at the new train at the first buffer contact), so the brakes to cut out then wait for
	// the look the AI takes once the train stands (the test above)
	CHECK(brake_setting::at_standstill(0.0));
	CHECK(brake_setting::at_standstill(0.04));
	CHECK(brake_setting::at_standstill(0.9));
	CHECK(brake_setting::at_standstill(-0.9));
	CHECK(false == brake_setting::at_standstill(EU07_AI_MOVEMENT));
	CHECK(false == brake_setting::at_standstill(30.0));

	// checked at 0.9 km/h: the brake is cut out
	test::vehicle creeping{passenger_car_definition()};
	charge_and_apply(creeping);
	CHECK(inspect_heavy_train(creeping, as_train, brake_setting::at_standstill(0.9)) == brake_setting::cut_out);
	CHECK(brake_setting::is_cut_out(creeping.mover()));
	CHECK(creeping->BrakeCutOutByInspector);
	// checked at 30 km/h: the car keeps its brake
	test::vehicle running{passenger_car_definition()};
	charge_and_apply(running);
	CHECK(inspect_heavy_train(running, as_train, brake_setting::at_standstill(30.0)) == brake_setting::cut_out);
	CHECK(false == brake_setting::is_cut_out(running.mover()));
	CHECK(false == running->BrakeCutOutByInspector);
	running.run(5.0);
	CHECK(running->BrakePress > 1.0);
}

TEST_CASE("brake setting: a bank engine's run counts as a train, shunting doesn't") {
	CHECK(brake_setting::runs_as_train(Obey_train));
	CHECK(brake_setting::runs_as_train(Bank));
	CHECK(brake_setting::runs_as_train(Obey_train | Connect));
	CHECK(false == brake_setting::runs_as_train(Shunt));
	CHECK(false == brake_setting::runs_as_train(Loose_shunt));
	CHECK(false == brake_setting::runs_as_train(Connect));
	CHECK(false == brake_setting::runs_as_train(Wait_for_orders));

	// a heavy consist on G banked: the wagon without G has its brake cut out, as in a train
	test::vehicle car{passenger_car_definition()};
	charge_and_apply(car);
	CHECK(inspect_heavy_train(car, brake_setting::runs_as_train(Bank)) == brake_setting::cut_out);
	CHECK(brake_setting::is_cut_out(car.mover()));
	// shunted: it's back on P
	CHECK(inspect_heavy_train(car, brake_setting::runs_as_train(Shunt)) == P);
	CHECK(false == brake_setting::is_cut_out(car.mover()));
}

TEST_CASE("brake setting: the AI looks at its train under the orders it drives it under, Connect included") {
	CHECK(brake_setting::inspects_train(Shunt, false));
	CHECK(brake_setting::inspects_train(Loose_shunt, false));
	CHECK(brake_setting::inspects_train(Obey_train, false));
	CHECK(brake_setting::inspects_train(Bank, false));
	// the run up to the wagons: a locomotive standing cold has its hand brake to release first
	CHECK(brake_setting::inspects_train(Connect, false));
	CHECK(brake_setting::inspects_train(Connect | Change_direction, false));
	// but not at the coupling: the wagons just coupled wait for the next order, which may leave some of them
	CHECK(false == brake_setting::inspects_train(Connect, true));
	CHECK(false == brake_setting::inspects_train(Connect | Change_direction, true));
	// a train run with a coupling on it is still a train run
	CHECK(brake_setting::inspects_train(Obey_train | Connect, true));
	CHECK(brake_setting::inspects_train(Shunt, true));
	// standing, starting or stopping the engine, turning round, uncoupling: nothing to look at
	CHECK(false == brake_setting::inspects_train(Wait_for_orders, false));
	CHECK(false == brake_setting::inspects_train(Prepare_engine, false));
	CHECK(false == brake_setting::inspects_train(Release_engine, false));
	CHECK(false == brake_setting::inspects_train(Change_direction, false));
	CHECK(false == brake_setting::inspects_train(Disconnect, false));
	CHECK(false == brake_setting::inspects_train(Jump_to_first_order, false));
}

TEST_CASE("brake setting: R+Mg brakes as hard as R at speed, the rapid stage of an LSt kept") {
	// the inspector sets R+Mg (Cw-1 §18.2), the bits of R and Mg together: the valves test the bit of R, not
	// the whole setting, so the high stage of R at speed stays on with Mg. the EU07's LSt given an Mg here
	auto const definition{test::definition::fiz("4e_v1/4e").set("Cntrl", "BrakeDelays", "GPR+Mg")};
	auto const service = [&](int const Setting) {
		test::reseed_random();
		test::vehicle real{definition, 160.0};
		real->DecManualBrakeLevel(ManualBrakePosNo);
		real->BrakeDelaySwitch(Setting); // false when already set
		REQUIRE(real->BrakeDelayFlag == Setting);
		real.hold_brake_pipe(5.0);
		real.run(60.0);
		real.hold_brake_pipe(3.5);
		real.run(10.0);
		REQUIRE(real->Vel > 100.0);
		return real->BrakePress;
	};
	auto const rapid{service(R)};
	CHECK(service(R | M) == doctest::Approx(rapid).epsilon(0.01));
	// and P has no high stage
	CHECK(service(P) < rapid - 1.0);
}

TEST_CASE("brake setting: a vehicle without the setting the train needs gets the other one of G and P") {
	// a light freight train with a wagon which has G only
	auto consist{train(locomotive(), 4, freight_wagon(15.0, 20.0))};
	consist.emplace_back(G, false, 15.0, 20000.0);
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::freight_p);
	CHECK(brake_setting::choose(consist, as_train).back() == G);
	// and a vehicle with neither is left alone
	consist.emplace_back(R, false, 15.0, 20000.0);
	CHECK(brake_setting::choose(consist, as_train).back() == 0);
}

TEST_CASE("brake setting: a passenger train on R carries at most three cars on P, fewer than those on R (Cw-1 §18.3)") {
	auto consist{train(locomotive(), 4, passenger_car())};
	consist.insert(consist.end(), 3, passenger_car(P));
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::passenger_r);
	auto const settings{brake_setting::choose(consist, as_train)};
	CHECK(settings[0] == R);
	CHECK(settings[1] == R);
	CHECK(settings.back() == P);

	// as many cars on P as on R: all on P
	consist.emplace_back(passenger_car(P));
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::passenger_p);
	for (auto const setting : brake_setting::choose(consist, as_train)) {
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
	CHECK(brake_setting::choose(consist, as_train).back() == P);
}

TEST_CASE("brake setting: cars with a magnetic rail brake run on R+Mg in a train on R (Cw-1 §18.2)") {
	auto consist{train(locomotive(G | P | R | M), 3, passenger_car(P | R | M))};
	consist.emplace_back(passenger_car());
	REQUIRE(brake_setting::choose_train_setting(consist) == train_setting::passenger_r);
	auto const settings{brake_setting::choose(consist, as_train)};
	CHECK(settings[0] == R); // the locomotive stays on R
	CHECK(settings[1] == (R | M));
	CHECK(settings.back() == R);
}

TEST_CASE("brake setting: a light engine runs on R where it has it") {
	std::vector<brake_setting::vehicle> const consist{locomotive(), locomotive(G | P)};
	CHECK(brake_setting::choose_train_setting(consist) == train_setting::light_engine);
	auto const settings{brake_setting::choose(consist, as_train)};
	CHECK(settings[0] == R);
	CHECK(settings[1] == P);
}

TEST_CASE("brake setting: the inspector of the earlier releases counted the locomotives into the train's mass") {
	// 300 m and 600 t of wagons: P by Cw-1, the long locomotive setting before
	auto const consist{train(locomotive(), 20, freight_wagon(15.0, 30.0))};
	CHECK(brake_setting::choose(consist, as_train)[1] == P);
	CHECK(brake_setting::choose_legacy(consist)[1] == G);
	CHECK(brake_setting::choose_legacy(consist)[6] == P);
}
