#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"

#include "McZapkie/hamulce.h"
#include "vehicle/crew.h"

// a driver put in a vehicle or taken off it by a "Crew:<vehicle>" putvalues event (eu07/maszyna#510): reading the
// command, what gets done about it in each state of the vehicle, and the state a train is left in when the driver
// in charge of it is taken off. the game's side (finding the vehicle, creating and deleting the driver) is run by
// the scenario cases, tests/scenarios/cases/*crew*

using test::consist;
using test::definition;
using test::track;
using test::vehicle;

namespace {

track const line{0.15, 0.0, test::catenary_3kv};

// the train started up by its driver, brakes released, then under power for Seconds: a notch a second up to the
// given position of the main controller
void get_under_way(consist &Train, int const Position, double const Seconds) {
	auto driver{test::driving(Train)};
	REQUIRE(test::start_up(driver, line));
	test::release_brakes(driver.cab);
	for (int i = 0; i < 3 && driver.cab.DirActive <= 0; ++i) {
		driver.cab.DirectionForward();
	}
	test::drive(driver, line, 20.0);
	driver.controlling.IncMainCtrl(1);
	for (int i = 0; i < 90 && false == driver.controlling.StLinFlag; ++i) {
		test::drive(driver, line, 1.0);
	}
	for (int i = 0; i < 60 && driver.controlling.MainCtrlPos < Position; ++i) {
		driver.controlling.IncMainCtrl(1);
		test::drive(driver, line, 1.0);
	}
	test::drive(driver, line, Seconds);
}

// seconds until the train stands, up to Limit. nobody attends to it: the driver is gone
double seconds_to_stop(consist &Train, double const Limit) {
	auto seconds{0.0};
	while (seconds < Limit && std::abs(Train[0].speed()) > 0.01) {
		Train.run(1.0, line);
		seconds += 1.0;
	}
	return seconds;
}

crew::vehicle_state const empty_locomotive{.has_cab = true};
crew::vehicle_state const ai_driven{.has_cab = true, .has_driver = true, .driver_is_ai = true};

} // namespace

TEST_CASE("crew: the command is told by its prefix, in this case only") {
	CHECK(crew::is_command("Crew:EP07-424"));
	CHECK(crew::is_command("Crew:"));
	CHECK_FALSE(crew::is_command("crew:ep07-424"));
	CHECK_FALSE(crew::is_command("CREW:EP07-424"));
	CHECK_FALSE(crew::is_command("Crew"));
	CHECK_FALSE(crew::is_command("Timetable:Crew:1"));
	CHECK_FALSE(crew::is_command("SetVelocity"));
	CHECK_FALSE(crew::is_command(""));
}

TEST_CASE("crew: the vehicle is named in the text, as the scenery keeps its names") {
	// the examples of the issue: Shift+4 a crew in cab A, Shift+5 in cab B, Shift+6 the crew off
	auto const front{crew::parse("Crew:EP07-424", 1.0, 0.0)};
	CHECK(front.vehicle == "ep07-424");
	CHECK(front.cab_valid);
	CHECK(front.cab == 1);
	CHECK(front.flags == 0);

	auto const rear{crew::parse("Crew:EP07-424", -1.0, 0.0)};
	CHECK(rear.cab_valid);
	CHECK(rear.cab == -1);

	auto const off{crew::parse("Crew:EP07-424", 0.0, 0.0)};
	CHECK(off.cab_valid);
	CHECK(off.cab == 0);

	SUBCASE("only the ASCII letters are folded, as the scenery parser folds them") {
		// "Ś" of windows-1250 (0x8C) and the two bytes of "Ł" in UTF-8 stay as they are
		std::string const name{"SM42-\x8C\xC5\x81" "A"};
		CHECK(crew::parse("Crew:" + name, 1.0, 0.0).vehicle == "sm42-\x8C\xC5\x81" "a");
	}
	SUBCASE("no name: the vehicle which set the event off") {
		auto const activator{crew::parse("Crew:", 1.0, 0.0)};
		CHECK(activator.vehicle.empty());
		CHECK(activator.cab_valid);
	}
	SUBCASE("a name with a colon in it is kept whole") {
		CHECK(crew::parse("Crew:a:b", 1.0, 0.0).vehicle == "a:b");
	}
	SUBCASE("the second value is kept for later, nothing reads it yet") {
		CHECK(crew::parse("Crew:x", 0.0, 1.0).flags == 1);
		CHECK(crew::parse("Crew:x", 0.0, 3.0).flags == 3);
	}
}

TEST_CASE("crew: a cab other than 1, -1 or 0 is not guessed at") {
	for (auto const cab : {2.0, -2.0, 0.5, -0.5, 1.5, 100.0}) {
		CAPTURE(cab);
		auto const request{crew::parse("Crew:ep07-424", cab, 0.0)};
		CHECK_FALSE(request.cab_valid);
		CHECK(crew::decide(request, empty_locomotive) == crew::action::refuse_cab);
		CHECK(crew::decide(request, ai_driven) == crew::action::refuse_cab);
	}
}

TEST_CASE("crew: text which is not the command asks for nothing") {
	auto const request{crew::parse("SetVelocity", 1.0, 0.0)};
	CHECK(request.vehicle.empty());
	CHECK_FALSE(request.cab_valid);
	CHECK_FALSE(crew::changes_crew(crew::decide(request, empty_locomotive)));
}

TEST_CASE("crew: a vehicle without a driver gets an AI one in the cab asked for") {
	CHECK(crew::decide(crew::parse("Crew:x", 1.0, 0.0), empty_locomotive) == crew::action::add_front);
	CHECK(crew::decide(crew::parse("Crew:x", -1.0, 0.0), empty_locomotive) == crew::action::add_rear);
}

TEST_CASE("crew: no second driver in a vehicle, and none in a vehicle without a cab") {
	CHECK(crew::decide(crew::parse("Crew:x", 1.0, 0.0), ai_driven) == crew::action::refuse_crewed);
	CHECK(crew::decide(crew::parse("Crew:x", -1.0, 0.0), ai_driven) == crew::action::refuse_crewed);
	// a wagon, or a car of a unit without a cab
	CHECK(crew::decide(crew::parse("Crew:x", 1.0, 0.0), crew::vehicle_state{}) == crew::action::refuse_no_cab);
}

TEST_CASE("crew: no second driver in charge of a consist") {
	auto const second{crew::vehicle_state{.has_cab = true, .consist_driven = true}};
	CHECK(crew::decide(crew::parse("Crew:x", 1.0, 0.0), second) == crew::action::refuse_consist_driven);
	CHECK(crew::decide(crew::parse("Crew:x", -1.0, 0.0), second) == crew::action::refuse_consist_driven);
}

TEST_CASE("crew: the AI driver is taken off, in charge of the consist or not") {
	CHECK(crew::decide(crew::parse("Crew:x", 0.0, 0.0), ai_driven) == crew::action::remove);
	auto helper{ai_driven};
	helper.consist_driven = true;
	CHECK(crew::decide(crew::parse("Crew:x", 0.0, 0.0), helper) == crew::action::remove);
	// AI drivers ride vehicles without a cab of their own too (a car of a unit set as driven in the scenery)
	auto const nocab{crew::vehicle_state{.has_driver = true, .driver_is_ai = true}};
	CHECK(crew::decide(crew::parse("Crew:x", 0.0, 0.0), nocab) == crew::action::remove);
}

TEST_CASE("crew: taking the driver off a vehicle which has none does nothing") {
	auto const decision{crew::decide(crew::parse("Crew:x", 0.0, 0.0), empty_locomotive)};
	CHECK(decision == crew::action::nothing);
	CHECK_FALSE(crew::changes_crew(decision));
}

TEST_CASE("crew: a vehicle with a user in it is left alone") {
	// the user drives; the user sits in the cab while the autopilot drives; the user sits in a vehicle whose
	// controller is the AI's (a cab of the game bound to the vehicle is what counts)
	auto const driving{crew::vehicle_state{.has_cab = true, .has_driver = true, .driver_is_ai = false, .occupied = true}};
	auto const autopilot{crew::vehicle_state{.has_cab = true, .has_driver = true, .driver_is_ai = true, .occupied = true}};
	auto const human{crew::vehicle_state{.has_cab = true, .has_driver = true, .driver_is_ai = false}};
	for (auto const &state : {driving, autopilot, human}) {
		for (auto const cab : {1.0, -1.0, 0.0}) {
			CAPTURE(cab);
			auto const decision{crew::decide(crew::parse("Crew:x", cab, 0.0), state)};
			CHECK(decision == crew::action::refuse_user);
			CHECK_FALSE(crew::changes_crew(decision));
		}
	}
}

TEST_CASE("crew: only adding and removing change the crew, and every outcome has its words for the log") {
	using enum crew::action;
	for (auto const action : {add_front, add_rear, remove}) {
		CHECK(crew::changes_crew(action));
		CHECK_FALSE(crew::describe(action).empty());
	}
	for (auto const action : {nothing, refuse_cab, refuse_user, refuse_crewed, refuse_no_cab, refuse_consist_driven}) {
		CHECK_FALSE(crew::changes_crew(action));
		CHECK_FALSE(crew::describe(action).empty());
	}
	// the scenario cases look for these in the log
	CHECK(crew::describe(add_front) == "an AI driver takes cab A");
	CHECK(crew::describe(add_rear) == "an AI driver takes cab B");
	CHECK(crew::describe(remove) == "the AI driver leaves");
	CHECK(crew::describe(refuse_user).starts_with("refused"));
}

TEST_CASE("crew: a locomotive under power left by its driver stops and stays braked") {
	test::reseed_random();
	consist train;
	train.add(definition::fiz("4e_v1/4e"), 1);
	get_under_way(train, 20, 30.0);
	auto &eu07{train[0].mover()};
	REQUIRE(train[0].speed() > 20.0);
	REQUIRE(eu07.MainCtrlPos > 0);
	REQUIRE(eu07.Mains);

	crew::secure(eu07, eu07);

	CHECK(eu07.MainCtrlPos == eu07.MainCtrlNoPowerPos());
	CHECK(eu07.ScndCtrlPos == 0);
	CHECK(eu07.BrakeCtrlPos == static_cast<int>(eu07.Handle->GetPos(bh_EB)));
	CHECK(eu07.CabActive == 0);
	CHECK(eu07.CabOccupied == 0);
	CHECK_FALSE(eu07.CabMaster);

	auto const stoppedafter{seconds_to_stop(train, 120.0)};
	CAPTURE(stoppedafter);
	CHECK(std::abs(train[0].speed()) <= 0.01);
	CHECK(eu07.PipePress < 1.0); // vented by the emergency position
	CHECK(eu07.BrakePress > 2.0);
	// it keeps standing, also on a slope, with the engine as the driver left it
	train.run(120.0, track{0.15, 0.02, test::catenary_3kv});
	CHECK(std::abs(train[0].speed()) <= 0.01);
	CHECK(eu07.Mains);
	CHECK(eu07.Im == doctest::Approx(0.0));
}

TEST_CASE("crew: what the AI held is let go when the driver leaves") {
	test::reseed_random();
	vehicle cab{definition::load("cab_vehicle")};
	cab->DirActive = 1;
	cab->WarningSignal = 1;
	cab->signal_departure(true);
	cab->Sandbox(true);
	REQUIRE(cab->DepartureSignal);
	REQUIRE(cab->SandDose);

	crew::secure(cab.mover(), cab.mover());

	CHECK(cab->WarningSignal == 0);
	CHECK_FALSE(cab->DepartureSignal);
	CHECK_FALSE(cab->SandDose);
}

TEST_CASE("crew: a train left by its driver is braked through the whole of it") {
	test::reseed_random();
	consist train;
	train.add(definition::fiz("4e_v1/4e"), 1);
	train.add(definition::fiz("11xa_v2/93a"));
	train.add(definition::fiz("11xa_v2/93a"));
	get_under_way(train, 20, 40.0);
	auto &eu07{train[0].mover()};
	auto &last{train[2].mover()};
	REQUIRE(train[0].speed() > 15.0);
	REQUIRE(last.BrakePress < 0.1);

	crew::secure(eu07, eu07);

	auto const stoppedafter{seconds_to_stop(train, 180.0)};
	CAPTURE(stoppedafter);
	CHECK(std::abs(train[0].speed()) <= 0.01);
	CHECK(std::abs(train[2].speed()) <= 0.01);
	CHECK(last.PipePress < 1.0);
	CHECK(last.BrakePress > 1.0);
}

TEST_CASE("crew: a standing locomotive left by its driver is braked too") {
	test::reseed_random();
	consist train;
	train.add(definition::fiz("4e_v1/4e"), 1);
	auto driver{test::driving(train)};
	REQUIRE(test::start_up(driver, line));
	test::release_brakes(driver.cab);
	auto &eu07{train[0].mover()};
	// the cylinders of a locomotive started up cold take their time to empty
	for (int i = 0; i < 180 && eu07.BrakePress >= 0.1; ++i) {
		test::drive(driver, line, 1.0);
	}
	REQUIRE(eu07.BrakePress < 0.1);

	crew::secure(eu07, eu07);

	// on a slope it would roll away with the brakes off
	train.run(120.0, track{0.15, 0.02, test::catenary_3kv});
	CHECK(eu07.BrakePress > 2.0);
	CHECK(std::abs(train[0].speed()) <= 0.01);
}

TEST_CASE("crew: the power an integrated handle asked for goes when the driver leaves") {
	test::reseed_random();
	// the handle's demand (eimic) is worked out only while a driver is in charge of the vehicle; left as it
	// was, the unit would go on pulling with nobody in it
	vehicle impuls{definition::fiz("impuls_v1/31we-a_pr"), 60.0, "impuls", 1};
	impuls->CabActivisation();
	impuls->MainCtrlPos = impuls->MainCtrlPosNo;
	impuls->eimic = 0.8;
	impuls->eimic_real = 0.8;

	crew::secure(impuls.mover(), impuls.mover());

	CHECK(impuls->MainCtrlPos == impuls->MainCtrlNoPowerPos());
	CHECK(impuls->eimic == 0.0);
	CHECK(impuls->eimic_real == 0.0);
	CHECK(impuls->BrakeCtrlPos == static_cast<int>(impuls->Handle->GetPos(bh_EB)));
	CHECK(impuls->CabOccupied == 0);
}

TEST_CASE("crew: a road vehicle left by its driver gets the brake the driver stops it with") {
	test::reseed_random();
	vehicle bus{definition::fiz("road/man/nl293"), 40.0, "bus", 1};
	REQUIRE(bus->BrakeSystem == TBrakeSystem::Individual);
	REQUIRE(bus->LocalBrakePosA == 0.0);
	bus->CabActivisation();

	crew::secure(bus.mover(), bus.mover());

	CHECK(bus->LocalBrakePosA == 1.0);
	CHECK(bus->MainCtrlPos == bus->MainCtrlNoPowerPos());
	bus.run(60.0, track{0.6, 0.0, 0.0, 2});
	CHECK(std::abs(bus.speed()) <= 0.01);
}
