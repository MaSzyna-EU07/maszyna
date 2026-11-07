#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"

#include "McZapkie/hamulce.h"

// the master controller of the Elf (EIMCtrlType 2): its positions, and the emergency brake position below
// them which EIMCtrlMainEmergency adds (eu07/maszyna#413)

using test::definition;
using test::vehicle;

namespace {

// a three-car Elf ready to run, standing with its brakes released and the driver in the front cab of the
// head car, which also drives (the controller's eimic is worked out there)
struct elf {
	explicit elf(bool Emergency) {
		for (auto const *car : {"elf_v1/21we-a", "elf_v1/21we-b", "elf_v1/21we-c"}) {
			auto cardefinition{definition::fiz(car)};
			if (Emergency) {
				cardefinition.set("Cntrl", "EIMCtrlMainEmergency", "Yes").set("Cntrl", "MCPN", 5);
			}
			unit.add(cardefinition, unit.size() == 0 ? 1 : 0);
		}
		ready = test::start_up(test::driving(unit), line);
		// the start-up of the tests leaves the controller at its position 0 (close_line_breaker() in drive.cpp),
		// the emergency brake one with EIMCtrlMainEmergency; the driver sets it to neutral, as the AI does. a unit
		// moves its controller only with a direction set
		for (int i = 0; i < 3 && cab().DirActive <= 0; ++i) {
			cab().DirectionForward();
		}
		while (cab().MainCtrlPos < cab().MainCtrlNoPowerPos() && cab().IncMainCtrl(1)) {
			// up to neutral
		}
		test::release_brakes(cab());
		recharge();
		charged = cab().PipePress > 4.5;
	}
	// the pipe charged again after an emergency stop: the Elf locks its emptied pipe off the brake handle (LPOn,
	// LPOff) until the unlock button (UBB1, the first universal brake button) is held, the handle at running
	void recharge() {
		cab().BrakeLevelSet(cab().Handle->GetPos(bh_RP));
		cab().UniversalBrakeButton(0, 1);
		wait(30.0);
		cab().UniversalBrakeButton(0, 0);
		wait(30.0);
	}
	TMoverParameters &cab() { return unit[0].mover(); }
	void wait(double Seconds) { test::drive(test::driving(unit), line, Seconds); }
	test::consist unit;
	test::track line{0.15, 0.0, test::catenary_3kv};
	bool ready{false};
	bool charged{false};
};

} // namespace

TEST_CASE("elf controller: without EIMCtrlMainEmergency the positions stay as they were") {
	elf unit{false};
	REQUIRE(unit.ready);
	REQUIRE(unit.charged);
	auto &mover{unit.cab()};
	CHECK_FALSE(mover.EIMCtrlMainEmergency);
	CHECK(mover.EIMCtrlEmergencyPositions() == 0);
	CHECK(mover.MainCtrlPosNo == 4);
	CHECK(mover.MainCtrlNoPowerPos() == 3);
	CHECK(mover.MainCtrlPos == 3);
	// the lowest position brakes fully, without emptying the pipe
	mover.MainCtrlPos = 0;
	CHECK_FALSE(mover.IsMainCtrlEmergencyPos());
	unit.wait(10.0);
	CHECK(mover.eimic == doctest::Approx(-1.0));
	CHECK(mover.PipePress > 4.5);
}

TEST_CASE("elf controller: EIMCtrlMainEmergency shifts the positions up by the emergency brake one") {
	elf unit{true};
	REQUIRE(unit.ready);
	REQUIRE(unit.charged);
	auto &mover{unit.cab()};
	CHECK(mover.EIMCtrlMainEmergency);
	CHECK(mover.EIMCtrlEmergencyPositions() == 1);
	CHECK(mover.MainCtrlPosNo == 5);
	CHECK(mover.MainCtrlNoPowerPos() == 4);
	CHECK(mover.MainCtrlPos == 4);
	CHECK_FALSE(mover.IsMainCtrlEmergencyPos());
	SUBCASE("the neutral position neither drives nor brakes") {
		unit.wait(5.0);
		CHECK(mover.eimic == doctest::Approx(0.0));
	}
	SUBCASE("the full braking position, formerly 0, brakes fully without emptying the pipe") {
		mover.MainCtrlPos = 1;
		unit.wait(10.0);
		CHECK(mover.eimic == doctest::Approx(-1.0));
		CHECK(mover.PipePress > 4.5);
	}
	SUBCASE("the braking position, formerly 1, ramps the braking up") {
		mover.MainCtrlPos = 2;
		unit.wait(1.0);
		CHECK(mover.eimic < 0.0);
		CHECK(mover.eimic > -1.0);
		CHECK(mover.PipePress > 4.5);
	}
	SUBCASE("the emergency brake position brakes fully at once and empties the pipe") {
		mover.MainCtrlPos = 0;
		CHECK(mover.IsMainCtrlEmergencyPos());
		// a single step of the physics
		unit.unit.step(unit.line);
		CHECK(mover.eimic == doctest::Approx(-1.0));
		unit.wait(10.0);
		CHECK(mover.PipePress < 0.5);
		CHECK(mover.BrakePress > 1.0);
		// out of it the pipe charges again
		mover.MainCtrlPos = mover.MainCtrlNoPowerPos();
		CHECK_FALSE(mover.IsMainCtrlEmergencyPos());
		unit.recharge();
		CHECK(mover.PipePress > 4.5);
	}
}

TEST_CASE("elf controller: EIMCtrlMainEmergency with the old MCPN gets its positions all the same") {
	vehicle car{definition::fiz("elf_v1/21we-a").set("Cntrl", "EIMCtrlMainEmergency", "Yes")};
	CHECK(car->MainCtrlPosNo == 5);
	CHECK(car->MainCtrlNoPowerPos() == 4);
}

TEST_CASE("elf controller: EIMCtrlMainEmergency leaves the other controllers alone") {
	// the Impuls (EIMCtrlType 0) has its emergency brake in the brake range of its controller (EIMCtrlEmergency)
	vehicle impuls{definition::fiz("impuls_v1/31we-a_pr").set("Cntrl", "EIMCtrlMainEmergency", "Yes"), 60.0, "impuls", 1};
	CHECK(impuls->EIMCtrlEmergencyPositions() == 0);
	CHECK(impuls->MainCtrlPosNo == 10);
	impuls->MainCtrlPos = 0;
	CHECK_FALSE(impuls->IsMainCtrlEmergencyPos());
}
