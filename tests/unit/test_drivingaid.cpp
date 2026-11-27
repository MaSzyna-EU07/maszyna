#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "real_vehicles.h"

#include "vehicle/drivingaid.h"
#include "McZapkie/hamulce.h"

#include <algorithm>
#include <cmath>
#include <tuple>

// the numbers of the driving aid panel's throttle and brakes lines: the ED braking shows (eu07/maszyna#558) and
// the local brake is left out where the cab has no control of it (eu07/maszyna#555)

using test::definition;
using test::vehicle;

namespace {

// the cab controls of the vehicles here, as their cab definitions (.mmd) have them: the Vectron has a local brake
// lever and a joint controller whose braking range is its ED lever, the Traxx and the EU07 a local brake lever,
// the SM42 a joint controller whose braking range is the local brake, none of the 45 Elf cabs either of them
constexpr drivingaid::cab_controls vectron_cab{true, true};
constexpr drivingaid::cab_controls lever_cab{true, false};
constexpr drivingaid::cab_controls jointctrl_cab{false, true};
constexpr drivingaid::cab_controls elf_cab{false, false};

drivingaid::readout read(vehicle &Vehicle, drivingaid::cab_controls const &Cab) {
	return drivingaid::compute(drivingaid::from_movers(Vehicle.mover(), Vehicle.mover(), Cab));
}

// what the panel showed before: the throttle and brakes numbers, and the local brake always
drivingaid::readout before(TMoverParameters const &Mover) {
	drivingaid::readout result;
	result.throttle_first = Mover.EIMCtrlType > 0 ? std::max(0, static_cast<int>(100.4 * Mover.eimic_real)) : Mover.MainCtrlPos;
	result.throttle_second = Mover.EIMCtrlType > 0 ? Mover.MainCtrlPos : Mover.ScndCtrlPos;
	result.brake_first = Mover.UniCtrlIntegratedBrakeCtrl ? std::max(0.0, -100.0 * Mover.eimic_real) : Mover.fBrakeCtrlPos;
	result.local_brake = Mover.LocalBrakePosA * LocalBrakePosNo;
	return result;
}

test::track const line{0.15, 0.0, test::catenary_3kv};

} // namespace

TEST_CASE("driving aid: the Vectron's ED lever shows as a negative throttle step, and its ED braking") {
	vehicle vectron{definition::fiz("vectron_v1/wariant_7w"), 80.0 /* km/h */, "vectron", 1};
	REQUIRE(vectron->SplitEDPneumaticBrake);
	REQUIRE(vectron->DynamicBrakeCtrlPosNo == 15);
	REQUIRE(vectron->EIMCtrlType == 0);

	// lever at zero: the master controller, as before, and no ED braking
	auto const idle{read(vectron, vectron_cab)};
	CHECK(idle.throttle_first == vectron->MainCtrlPos);
	CHECK_FALSE(idle.ed_percent.has_value());
	CHECK(idle.show_local_brake);

	// six steps of the ED lever, as the cab's joint controller shows them
	for (int step = 0; step < 6; ++step) {
		REQUIRE(vectron->IncDynamicBrakeLevel(1.f));
	}
	vectron.run(1.0, line);
	REQUIRE(vectron->Vel > 15.0); // above Vh1, where the ED brake has its full demand
	auto const braking{read(vectron, vectron_cab)};
	CHECK(braking.throttle_first == -6);
	REQUIRE(vectron->eimic_real == doctest::Approx(-0.4));
	REQUIRE(braking.ed_percent.has_value());
	CHECK(*braking.ed_percent == 40);
	// the train brake handle and the local brake, as before
	CHECK(braking.brake_first == doctest::Approx(vectron->fBrakeCtrlPos));
	CHECK(braking.show_local_brake);
	CHECK(braking.local_brake == doctest::Approx(0.0));

	// the joint controller's braking range is the ED lever, so it isn't a local brake control
	CHECK_FALSE(read(vectron, jointctrl_cab).show_local_brake);
}

TEST_CASE("driving aid: the Traxx's ED braking from its master controller") {
	vehicle traxx{definition::fiz("e186_v2/eu47"), 80.0 /* km/h */, "traxx", 1};
	REQUIRE(traxx->EIMCtrlType == 1);
	REQUIRE_FALSE(traxx->SplitEDPneumaticBrake);

	CHECK_FALSE(read(traxx, lever_cab).ed_percent.has_value());
	// B+: the braking demand ramps towards full
	traxx->MainCtrlPos = 0;
	traxx.run(3.0, line);
	REQUIRE(traxx->eimic_real < -0.2);
	auto const braking{read(traxx, lever_cab)};
	REQUIRE(braking.ed_percent.has_value());
	CHECK(*braking.ed_percent == std::lround(-100.0 * traxx->eimic_real));
	// the drive demand stays at zero while braking, with the controller position next to it, as before
	CHECK(braking.throttle_first == 0);
	CHECK(braking.throttle_second == 0);
	CHECK(braking.show_local_brake);
}

TEST_CASE("driving aid: the Elf leaves out the local brake it has no control of, and shows its ED braking") {
	vehicle elf{definition::fiz("elf_v1/21we-a"), 60.0 /* km/h */, "elf", 1};
	REQUIRE(elf->EIMCtrlType == 2);
	// the definition has a local brake (LocalBrake=PneumaticBrake, LocBrakeHandle=FD1), the cab no control of it
	REQUIRE(elf->LocalBrake == TLocalBrake::PneumaticBrake);

	auto const idle{read(elf, elf_cab)};
	CHECK_FALSE(idle.show_local_brake);
	CHECK_FALSE(idle.ed_percent.has_value());

	// the controller at its lowest braking position: the demand ramps towards full braking
	elf->MainCtrlPos = 0;
	elf.run(2.0, line);
	REQUIRE(elf->eimic_real < -0.1);
	auto const braking{read(elf, elf_cab)};
	REQUIRE(braking.ed_percent.has_value());
	CHECK(*braking.ed_percent == std::lround(-100.0 * elf->eimic_real));
	CHECK(braking.throttle_first == 0);
	CHECK(braking.throttle_second == 0);
	CHECK_FALSE(braking.show_local_brake);

	// applied after all (the AI, or the keys), it shows
	elf->LocalBrakePosA = 0.3;
	auto const applied{read(elf, elf_cab)};
	CHECK(applied.show_local_brake);
	CHECK(applied.local_brake == doctest::Approx(3.0));
}

TEST_CASE("driving aid: a vehicle without an EIM controller shows what it did") {
	vehicle eu07{definition::fiz("4e_v1/4e"), 60.0 /* km/h */, "eu07", 1};
	eu07->MainCtrlPos = 5;
	eu07->LocalBrakePosA = 0.2;
	auto const readout{read(eu07, lever_cab)};
	CHECK(readout.throttle_first == 5);
	CHECK(readout.throttle_second == eu07->ScndCtrlPos);
	CHECK(readout.local_brake == doctest::Approx(2.0));
	CHECK(readout.show_local_brake);
	CHECK_FALSE(readout.ed_percent.has_value());

	// the SM42's local brake is the braking range of its joint controller
	vehicle sm42{definition::fiz("sm42_v1/6d1"), 0.0, "sm42", 1};
	CHECK(read(sm42, jointctrl_cab).show_local_brake);
	CHECK_FALSE(read(sm42, elf_cab).show_local_brake);
}

TEST_CASE("driving aid: the integrated universal controller shows the ED demand as its brake number, once") {
	drivingaid::input input;
	input.eim = true;
	input.eimctrltype = 3;
	input.integratedbrake = true;
	input.eimic_real = -0.35;
	input.brakectrlpos = 2.0;
	auto const readout{drivingaid::compute(input)};
	CHECK(readout.brake_first == doctest::Approx(35.0));
	CHECK_FALSE(readout.ed_percent.has_value());
	CHECK(readout.throttle_first == 0);
}

TEST_CASE("driving aid: every real vehicle with a local brake control shows the numbers it showed before") {
	// with the ED lever of a split ED brake at zero, the change only adds the ED part and leaves out the local brake
	// of a cab without a control of it
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path), 40.0 /* km/h */, "real", 1};
		for (auto const &[mainctrl, localbrake, eimic] : {std::tuple{0, 0.0, 0.0}, std::tuple{1, 0.4, -0.25}, std::tuple{1, 0.0, 0.6}}) {
			real->MainCtrlPos = std::min(mainctrl, real->MainCtrlPosNo);
			real->LocalBrakePosA = localbrake;
			real->eimic_real = eimic;
			auto const now{read(real, lever_cab)};
			auto const was{before(real.mover())};
			CHECK(now.throttle_first == was.throttle_first);
			CHECK(now.throttle_second == was.throttle_second);
			CHECK(now.brake_first == doctest::Approx(was.brake_first));
			CHECK(now.local_brake == doctest::Approx(was.local_brake));
			CHECK(now.show_local_brake);
		}
	}
}
