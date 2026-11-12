#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "drive.h"

#include <cmath>
#include <vector>

// the 6Dg shunter as the game ships it (data/fiz/6dg_v1/6dg.json): the power demand of its universal controller
// (EIMCtrlType 3, UCList) moves in steps of 5% (UCList CtrlValStep=0.05, eu07/maszyna#412), and its diesel-electric
// drive follows the demand. the controller's positions in the 6Dg's table: 3 holds no power, 4 lowers the power
// to 0, 5 holds it, 6 raises it to 1, each at 0.15 a second after IniCDelay (0.5 s)

using test::definition;
using test::vehicle;

namespace {

constexpr int no_power{3};
constexpr int lower{4};
constexpr int hold{5};
constexpr int raise{6};

definition shipped() {
	return definition::fiz("6dg_v1/6dg");
}

void move_to(vehicle &Loco, int const Position) {
	// one notch at a time, as far as the controller goes
	while (Loco->MainCtrlPos < Position) {
		if (false == Loco->IncMainCtrl(1)) {
			break;
		}
	}
	while (Loco->MainCtrlPos > Position) {
		if (false == Loco->DecMainCtrl(1)) {
			break;
		}
	}
	REQUIRE(Loco->MainCtrlPos == Position);
}

// the 6Dg started up (engine running), its brakes released and its direction set, its controller holding no power
void get_ready(vehicle &Loco, test::track const &Track) {
	auto driven{test::driving(Loco)};
	REQUIRE(test::start_up(driven, Track));
	test::release_brakes(driven.cab);
	move_to(Loco, no_power);
	for (int i = 0; i < 3 && Loco->DirActive <= 0; ++i) {
		Loco->DirectionForward();
	}
	REQUIRE(Loco->DirActive > 0);
	// the brake pipe was charged during the start-up: the brakes release slowly (about a minute), and the pressure
	// switch keeps the line contactors open while the brake cylinders hold over 2 bar
	for (int second = 0; second < 180 && Loco->BrakePress > 0.1; ++second) {
		test::drive(driven, Track, 1.0);
	}
	REQUIRE(Loco->BrakePress <= 0.1);
	// the line contactors close on the first position past the ones with no power, as the driver passes it
	move_to(Loco, lower);
	test::drive(driven, Track, 1.0);
	move_to(Loco, hold);
	test::drive(driven, Track, 5.0);
	REQUIRE(Loco->Mains); // the engine running
	REQUIRE(Loco->StLinFlag); // the line contactors closed
	REQUIRE(false == Loco->SpeedCtrlUnit.IsActive); // the demand of the controller drives, not the speed control
	REQUIRE(Loco->eimic == doctest::Approx(0.0));
}

// a tap of the controller onto the given position and back onto holding, then a second of running
void tap(vehicle &Loco, int const Position, test::track const &Track) {
	move_to(Loco, Position);
	Loco.step(Track);
	move_to(Loco, hold);
	test::drive(test::driving(Loco), Track, 1.0);
}

// the changes of the demand while the controller stays on the given position for Seconds
std::vector<double> changes_held(vehicle &Loco, int const Position, double const Seconds, test::track const &Track) {
	std::vector<double> changes;
	move_to(Loco, Position);
	auto previous{Loco->eimic};
	for (int i = 0; i < std::lround(Seconds / 0.01); ++i) {
		if (Loco->SecuritySystem.is_blinking()) {
			Loco->SecuritySystemReset(); // the driver acknowledging the vigilance
		}
		Loco.step(Track);
		if (std::abs(Loco->eimic - previous) > 1e-9) {
			changes.push_back(Loco->eimic - previous);
			previous = Loco->eimic;
		}
	}
	return changes;
}

} // namespace

TEST_CASE("6Dg: the shipped definition sets the controller's power demand in steps of 5%") {
	test::track const line{0.15, 0.0};
	vehicle loco{shipped(), 0.0, "test", 1};
	// the key as the game data has it, on the controller it acts on
	REQUIRE(loco->EIMCtrlType == 3);
	CHECK(loco->UniCtrlValStep == doctest::Approx(0.05));
	// the ends of the table: raising goes up to 1, lowering down to 0, 20 steps of 0.05 between them
	auto const top{loco->UniCtrlList[raise].SetCtrlVal};
	auto const bottom{loco->UniCtrlList[lower].SetCtrlVal};
	REQUIRE(top == doctest::Approx(1.0));
	REQUIRE(bottom == doctest::Approx(0.0));
	REQUIRE(loco->UniCtrlList[raise].MaxCtrlVal == doctest::Approx(top));
	REQUIRE(loco->UniCtrlList[lower].MinCtrlVal == doctest::Approx(bottom));
	auto const steps{static_cast<int>(std::lround((top - bottom) / 0.05))};
	CHECK(steps == 20);
	get_ready(loco, line);

	SUBCASE("tapped: every tap moves the demand by one step, up to the top and down to the bottom, and no further") {
		auto changed{0};
		for (int i = 1; i <= steps + 3; ++i) {
			auto const before{loco->eimic};
			tap(loco, raise, line);
			CAPTURE(i);
			CHECK(loco->eimic == doctest::Approx(std::min(top, before + 0.05)));
			CHECK(loco->eimic <= top + 1e-9);
			CHECK(loco->eimic_real == doctest::Approx(loco->eimic)); // the demand reaching the drive
			changed += std::abs(loco->eimic - before) > 1e-9 ? 1 : 0;
		}
		CHECK(changed == steps);
		CHECK(loco->eimic == doctest::Approx(top));

		changed = 0;
		for (int i = 1; i <= steps + 3; ++i) {
			auto const before{loco->eimic};
			tap(loco, lower, line);
			CAPTURE(i);
			CHECK(loco->eimic == doctest::Approx(std::max(bottom, before - 0.05)));
			CHECK(loco->eimic >= bottom - 1e-9);
			changed += std::abs(loco->eimic - before) > 1e-9 ? 1 : 0;
		}
		CHECK(changed == steps);
		CHECK(loco->eimic == doctest::Approx(bottom));
	}

	SUBCASE("held: the demand goes over the whole range in steps of 0.05, never in between nor past the ends") {
		// 20 steps of 0.05 at 0.15 a second take under 7 s, after the initial delay
		auto const up{changes_held(loco, raise, 12.0, line)};
		CHECK(up.size() == static_cast<std::size_t>(steps));
		for (auto const change : up) {
			CHECK(change == doctest::Approx(0.05));
		}
		CHECK(loco->eimic == doctest::Approx(top));
		auto const down{changes_held(loco, lower, 12.0, line)};
		CHECK(down.size() == static_cast<std::size_t>(steps));
		for (auto const change : down) {
			CHECK(change == doctest::Approx(-0.05));
		}
		CHECK(loco->eimic == doctest::Approx(bottom));
	}
}

TEST_CASE("6Dg: the diesel-electric drive follows the stepped demand") {
	test::track const line{0.15, 0.0};
	struct reading {
		double demand;
		double revolutions; // of the engine, after the demand is set
		double force; // tractive
		double power; // of the generator
		double speed; // reached after the same time
	};
	std::vector<reading> readings;
	for (auto const taps : {5, 10, 15, 20}) {
		CAPTURE(taps);
		test::reseed_random();
		vehicle loco{shipped(), 0.0, "test", 1};
		get_ready(loco, line);
		for (int i = 0; i < taps; ++i) {
			move_to(loco, raise);
			loco.step(line);
			move_to(loco, hold);
			loco.step(line);
		}
		REQUIRE(loco->eimic == doctest::Approx(0.05 * taps));
		REQUIRE(loco->eimic_real == doctest::Approx(0.05 * taps));
		test::drive(test::driving(loco), line, 10.0);
		readings.emplace_back(loco->eimic_real, loco->enrot, std::abs(loco->Ft), loco->EnginePower, std::abs(loco.speed()));
	}
	// at 0.25, 0.5, 0.75 and 1: the engine at 897, 1198, 1499 and (still running up to) 1800 rpm as the WWList table
	// gives, about 28, 37, 48 and 59 kN, 87, 261, 445 and 663 kW, 14, 24, 28 and 30 km/h
	for (std::size_t i = 1; i < readings.size(); ++i) {
		CAPTURE(i);
		INFO("demand " << readings[i].demand << ": " << readings[i].revolutions * 60.0 << " rpm, " << readings[i].force << " N, " << readings[i].power << " kW, " << readings[i].speed
		               << " km/h; before it " << readings[i - 1].revolutions * 60.0 << " rpm, " << readings[i - 1].force << " N, " << readings[i - 1].power << " kW, "
		               << readings[i - 1].speed << " km/h");
		CHECK(readings[i].demand > readings[i - 1].demand);
		CHECK(readings[i].revolutions > readings[i - 1].revolutions);
		CHECK(readings[i].force > readings[i - 1].force);
		CHECK(readings[i].power > readings[i - 1].power);
		CHECK(readings[i].speed > readings[i - 1].speed);
	}
}

TEST_CASE("6Dg: without CtrlValStep the demand moves continuously between the rows of the table, as before") {
	test::track const line{0.15, 0.0};
	vehicle loco{shipped().remove("UCList", "CtrlValStep"), 0.0, "test", 1};
	REQUIRE(loco->UniCtrlValStep == 0.0);
	get_ready(loco, line);
	// a tap moves it by 0.01 only
	tap(loco, raise, line);
	CHECK(loco->eimic == doctest::Approx(0.01));
	// held, it goes on at the row's 0.15 a second after the initial delay, a little every step
	auto const up{changes_held(loco, raise, 12.0, line)};
	CHECK(up.size() > 100);
	for (auto const change : up) {
		CHECK(change > 0.0);
		CHECK(change < 0.05 - 1e-6);
	}
	CHECK(loco->eimic == doctest::Approx(1.0)); // up to the top of the table, as with the steps
	auto const down{changes_held(loco, lower, 12.0, line)};
	CHECK(down.size() > 100);
	CHECK(loco->eimic == doctest::Approx(0.0));
}
