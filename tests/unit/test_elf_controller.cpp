#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"

#include "McZapkie/hamulce.h"

#include <initializer_list>
#include <string>

// the master controller of the Elf (EIMCtrlType 2): its positions, and the emergency brake position below
// them which EIMCtrlMainEmergency adds (eu07/maszyna#413)

using test::definition;
using test::vehicle;

namespace {

// the data of an Elf: without the emergency brake position as now, with it in the files updated for it
// (EIMCtrlMainEmergency, MCPN=5, DirChangeMaxPos=4), or with the key alone added to the old files, which the
// loader fixes up
enum class data { plain, emergency, emergency_old_layout };

// three-car Elfs coupled into one train, of the given data each, ready to run, standing with their brakes
// released and the driver in the front cab of the head car of the first one, which also drives (the
// controller's eimic is worked out there)
struct elf {
	explicit elf(bool Emergency) : elf({Emergency ? data::emergency : data::plain}) {}
	explicit elf(std::initializer_list<data> Units) {
		for (auto const units : Units) {
			for (auto const *car : {"elf_v1/21we-a", "elf_v1/21we-b", "elf_v1/21we-c"}) {
				auto cardefinition{definition::fiz(car)};
				if (units != data::plain) {
					cardefinition.set("Cntrl", "EIMCtrlMainEmergency", "Yes");
				}
				if (units == data::emergency) {
					cardefinition.set("Cntrl", "MCPN", 5);
					if (std::string{car} != "elf_v1/21we-b") {
						// the cars with a cab have it, at neutral: 3 before, one up with the key
						cardefinition.set("Cntrl", "DirChangeMaxPos", 4);
					}
				}
				unit.add(cardefinition, unit.size() == 0 ? 1 : 0);
			}
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
	TMoverParameters &car(std::size_t Index) { return unit[Index].mover(); }
	// the driver moves the controller notch by notch, which the cab sends on to the coupled vehicles
	void set_controller(int Position) {
		while (cab().MainCtrlPos < Position && cab().IncMainCtrl(1)) {
		}
		while (cab().MainCtrlPos > Position && cab().DecMainCtrl(1)) {
		}
	}
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
	// DirChangeMaxPos=3 of the file, the old neutral, moves up with the positions
	CHECK(car->MainCtrlMaxDirChangePos == 4);
}

TEST_CASE("elf controller: EIMCtrlMainEmergency leaves DirChangeMaxPos of the files updated for it alone") {
	vehicle car{definition::fiz("elf_v1/21we-a").set("Cntrl", "EIMCtrlMainEmergency", "Yes").set("Cntrl", "MCPN", 5).set("Cntrl", "DirChangeMaxPos", 4)};
	CHECK(car->MainCtrlPosNo == 5);
	CHECK(car->MainCtrlMaxDirChangePos == 4);
}

TEST_CASE("elf controller: the reverser works at the neutral shifted by EIMCtrlMainEmergency") {
	// the key added to the old files, DirChangeMaxPos=3 in them
	elf unit{{data::emergency_old_layout}};
	REQUIRE(unit.ready);
	auto &mover{unit.cab()};
	REQUIRE(mover.MainCtrlPos == mover.MainCtrlNoPowerPos());
	REQUIRE(mover.DirActive == 1);
	CHECK(mover.DirectionBackward());
	CHECK(mover.DirActive == 0);
	CHECK(mover.DirectionForward());
	CHECK(mover.DirActive == 1);
	// not in the power position
	unit.set_controller(mover.MainCtrlNoPowerPos() + 1);
	REQUIRE(mover.MainCtrlPos == 5);
	CHECK_FALSE(mover.DirectionBackward());
	CHECK(mover.DirActive == 1);
}

TEST_CASE("elf controller: the emergency brake position vents the pipe only from the vehicle driven from") {
	elf unit{true};
	REQUIRE(unit.ready);
	REQUIRE(unit.charged);
	// the rear car, with its own cab, has its controller in the emergency brake position, the driven one not
	auto &rear{unit.car(2)};
	rear.MainCtrlPos = 0;
	CHECK(rear.IsMainCtrlEmergencyPos());
	CHECK_FALSE(rear.IsCabMaster());
	unit.wait(10.0);
	CHECK(unit.cab().PipePress > 4.5);
	CHECK(rear.PipePress > 4.5);
}

TEST_CASE("elf controller: an Elf with the emergency brake position leading one without it") {
	elf unit{{data::emergency, data::plain}};
	REQUIRE(unit.ready);
	REQUIRE(unit.charged);
	auto &lead{unit.cab()};
	auto &other{unit.car(3)};
	REQUIRE_FALSE(other.EIMCtrlMainEmergency);
	// neutral of the lead is neutral of the other
	CHECK(lead.MainCtrlPos == 4);
	CHECK(other.MainCtrlPos == other.MainCtrlNoPowerPos());
	SUBCASE("power") {
		unit.set_controller(5);
		CHECK(other.MainCtrlPos == 4);
	}
	SUBCASE("full braking") {
		unit.set_controller(1);
		CHECK(other.MainCtrlPos == 0);
		unit.wait(10.0);
		CHECK(lead.PipePress > 4.5);
	}
	SUBCASE("the emergency brake: the other one at its full braking, the pipe vented by the lead") {
		unit.set_controller(0);
		REQUIRE(lead.IsMainCtrlEmergencyPos());
		CHECK(other.MainCtrlPos == 0);
		unit.wait(10.0);
		CHECK(lead.PipePress < 0.5);
		CHECK(other.PipePress < 0.5);
	}
}

TEST_CASE("elf controller: an Elf without the emergency brake position leading one with it") {
	elf unit{{data::plain, data::emergency}};
	REQUIRE(unit.ready);
	REQUIRE(unit.charged);
	auto &lead{unit.cab()};
	auto &other{unit.car(3)};
	REQUIRE(other.EIMCtrlMainEmergency);
	CHECK(lead.MainCtrlPos == 3);
	CHECK(other.MainCtrlPos == other.MainCtrlNoPowerPos());
	SUBCASE("power") {
		unit.set_controller(4);
		CHECK(other.MainCtrlPos == 5);
	}
	SUBCASE("the full braking of the lead is the full braking of the other, not its emergency brake") {
		unit.set_controller(0);
		CHECK(other.MainCtrlPos == 1);
		CHECK_FALSE(other.IsMainCtrlEmergencyPos());
		unit.wait(10.0);
		CHECK(lead.PipePress > 4.5);
		CHECK(other.PipePress > 4.5);
	}
}

TEST_CASE("elf controller: EIMCtrlMainEmergency leaves the other controllers alone") {
	// the Impuls (EIMCtrlType 0) has its emergency brake in the brake range of its controller (EIMCtrlEmergency)
	vehicle impuls{definition::fiz("impuls_v1/31we-a_pr").set("Cntrl", "EIMCtrlMainEmergency", "Yes"), 60.0, "impuls", 1};
	CHECK(impuls->EIMCtrlEmergencyPositions() == 0);
	CHECK(impuls->MainCtrlPosNo == 10);
	impuls->MainCtrlPos = 0;
	CHECK_FALSE(impuls->IsMainCtrlEmergencyPos());
}
