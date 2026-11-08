#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the Cntrl. section of the vehicle definition, as read by TMoverParameters::LoadFIZ_Cntrl()

TEST_CASE("cntrl: the brake delays of the settings come from BDelay1 to BDelay4") {
	vehicle wagon{definition::load("wagon").set("Cntrl", "BDelay1", 10.0).set("Cntrl", "BDelay2", 20.0).set("Cntrl", "BDelay3", 30.0).set("Cntrl", "BDelay4", 40.0)};
	CHECK(wagon->BrakeDelay[0] == doctest::Approx(10.0));
	// the vehicle varies the others a little, by 0.83 to 0.9 (CheckLocomotiveParameters())
	for (auto setting{1}; setting < 4; ++setting) {
		CAPTURE(setting);
		auto const given{10.0 * (setting + 1)};
		CHECK(wagon->BrakeDelay[setting] >= 0.83 * given);
		CHECK(wagon->BrakeDelay[setting] <= 0.9 * given);
	}
}

TEST_CASE("cntrl: the relays of a multiple unit start on their own, of other vehicles by hand") {
	SUBCASE("multiple unit") {
		vehicle unit{definition::load("wagon").set("Param", "Type", "ezt")};
		REQUIRE(unit->TrainType == dt_EZT);
		CHECK(unit->GroundRelayStart == start_t::automatic);
		CHECK(unit->ConverterOverloadRelayStart == start_t::converter); // activates when the converter is switched off
	}
	SUBCASE("other vehicle") {
		vehicle wagon{definition::load("wagon")};
		CHECK(wagon->GroundRelayStart == start_t::manual);
		CHECK(wagon->ConverterOverloadRelayStart == start_t::manual);
	}
	SUBCASE("given in the definition") {
		vehicle unit{definition::load("wagon").set("Param", "Type", "ezt").set("Cntrl", "GroundRelayStart", "Manual").set("Cntrl", "ConverterOverloadRelayStart", "Automatic")};
		CHECK(unit->GroundRelayStart == start_t::manual);
		CHECK(unit->ConverterOverloadRelayStart == start_t::automatic);
	}
}

namespace {

// the first position of the master controller after Position without the starting resistors in the circuit
int resistance_free_position(TMoverParameters const &Vehicle, int Position = 0) {
	for (auto position{Position + 1}; position <= Vehicle.MainCtrlPosNo; ++position) {
		if (Vehicle.RList[position].R < 0.01) {
			return position;
		}
	}
	return 0;
}

// the vehicle with its cab occupied, the master controller set to Position and the shunt controller off
void controllers_at(vehicle &Vehicle, int Position) {
	Vehicle->CabActive = 1;
	Vehicle->MainCtrlPos = Position;
	Vehicle->ScndCtrlPos = 0;
}

} // namespace

TEST_CASE("cntrl: ScndCtrlResistiveLock keeps the field shunting off at zero and on the resistive positions (eu07/maszyna#402)") {
	for (auto const *path : {"et22_v3/201e-rn", "ep09_v2/104e"}) {
		CAPTURE(path);
		vehicle locked{definition::fiz(path).set("Cntrl", "ScndCtrlResistiveLock", "Yes"), 0.0, "test", 1};
		REQUIRE(locked->ScndCtrlResistiveLock);
		REQUIRE(locked->ScndCtrlPosNo > 0);
		auto const resistancefree{resistance_free_position(locked.mover())};
		REQUIRE(resistancefree > 1);
		if (resistancefree <= 1) {
			continue; // not reached, the REQUIRE above ends the test; tells the static analysis the index below is in range
		}
		REQUIRE(locked->RList[resistancefree - 1].R >= 0.01);

		SUBCASE("zero") {
			controllers_at(locked, 0);
			CHECK(locked->IsScndCtrlLockedByMainCtrl());
			CHECK_FALSE(locked->IncScndCtrl(1));
			CHECK_FALSE(locked->IncScndCtrl(2));
			CHECK(locked->ScndCtrlPos == 0);
		}
		SUBCASE("resistive positions") {
			for (auto position{1}; position < resistancefree; ++position) {
				CAPTURE(position);
				controllers_at(locked, position);
				CHECK_FALSE(locked->IncScndCtrl(1));
				CHECK(locked->ScndCtrlPos == 0);
			}
		}
		SUBCASE("every resistance-free position") {
			for (auto position{resistancefree}; position > 0; position = resistance_free_position(locked.mover(), position)) {
				CAPTURE(position);
				controllers_at(locked, position);
				CHECK_FALSE(locked->IsScndCtrlLockedByMainCtrl());
				CHECK(locked->IncScndCtrl(1));
				CHECK(locked->ScndCtrlPos == 1);
			}
		}
		SUBCASE("the shunt goes down on a resistance-free position") {
			controllers_at(locked, resistancefree);
			REQUIRE(locked->IncScndCtrl(1));
			REQUIRE(locked->IncScndCtrl(1));
			CHECK(locked->DecScndCtrl(1));
			CHECK(locked->ScndCtrlPos == 1);
			CHECK(locked->DecScndCtrl(2));
			CHECK(locked->ScndCtrlPos == 0);
		}
		SUBCASE("the shunt doesn't go down on a resistive position either") {
			// a state the lock doesn't let arise (see below), set by hand to check the controller alone
			controllers_at(locked, resistancefree - 1);
			locked->ScndCtrlPos = 1;
			CHECK(locked->IsScndCtrlLockedByMainCtrl());
			CHECK_FALSE(locked->DecScndCtrl(1));
			CHECK_FALSE(locked->DecScndCtrl(2));
			CHECK(locked->ScndCtrlPos == 1);
		}
		SUBCASE("the master controller taken back from a shunt step drops the shunt") {
			controllers_at(locked, resistancefree);
			REQUIRE(locked->IncScndCtrl(1));
			if (locked->TrainType == dt_ET22) {
				// the ET22's master controller doesn't turn with the shunt on: the shunt goes off first, as before
				CHECK_FALSE(locked->DecMainCtrl(1));
				CHECK(locked->MainCtrlPos == resistancefree);
				CHECK(locked->DecScndCtrl(1));
			}
			CHECK(locked->DecMainCtrl(1));
			CHECK(locked->MainCtrlPos == resistancefree - 1);
			CHECK(locked->ScndCtrlPos == 0);
			// and the master controller goes on down to zero, nothing left locked
			while (locked->MainCtrlPos > 0 && locked->DecMainCtrl(1)) {
			}
			CHECK(locked->MainCtrlPos == 0);
		}
		SUBCASE("the master controller set directly (AI, consist) drops the shunt in the next update") {
			controllers_at(locked, resistancefree);
			REQUIRE(locked->IncScndCtrl(1));
			locked->MainCtrlPos = resistancefree - 1;
			locked.step();
			CHECK(locked->ScndCtrlPos == 0);
			CHECK(locked->MainCtrlPos == resistancefree - 1);
		}
		SUBCASE("the master controller moved up from a shunt step onto a resistive position drops the shunt") {
			// the last series position, followed by the resistive parallel ones
			auto lastseries{0};
			for (auto position{resistancefree}; position > 0 && position < locked->MainCtrlPosNo; position = resistance_free_position(locked.mover(), position)) {
				if (locked->RList[position + 1].R >= 0.01) {
					lastseries = position;
					break;
				}
			}
			REQUIRE(lastseries > 0);
			controllers_at(locked, lastseries);
			locked->DirActive = 1;
			REQUIRE(locked->IncScndCtrl(1));
			if (locked->TrainType == dt_ET22) {
				CHECK_FALSE(locked->IncMainCtrl(1)); // the ET22's interlock, as before
				CHECK(locked->ScndCtrlPos == 1);
			}
			else {
				CHECK(locked->IncMainCtrl(1));
				CHECK(locked->MainCtrlPos == lastseries + 1);
				CHECK(locked->ScndCtrlPos == 0);
			}
		}
	}
}

TEST_CASE("cntrl: without ScndCtrlResistiveLock the field shunting engages on any position, as before") {
	for (auto const *path : {"et22_v3/201e-rn", "ep09_v2/104e"}) {
		CAPTURE(path);
		vehicle unlocked{definition::fiz(path), 0.0, "test", 1};
		CHECK_FALSE(unlocked->ScndCtrlResistiveLock);
		for (auto const position : {0, 1}) {
			CAPTURE(position);
			controllers_at(unlocked, position);
			CHECK_FALSE(unlocked->IsScndCtrlLockedByMainCtrl());
			CHECK(unlocked->IncScndCtrl(1));
			CHECK(unlocked->ScndCtrlPos == 1);
		}
	}
}
