#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "drive.h"

#include "physics/brakes.h"

#include <cmath>
#include <initializer_list>
#include <optional>
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
	// HoldingBrakeDelay, when given, goes to every car as SBDReleaseDelay
	explicit elf(std::initializer_list<data> Units, std::optional<double> HoldingBrakeDelay = std::nullopt) {
		for (auto const units : Units) {
			for (auto const *car : {"elf_v1/21we-a", "elf_v1/21we-b", "elf_v1/21we-c"}) {
				auto cardefinition{definition::fiz(car)};
				if (HoldingBrakeDelay) {
					cardefinition.set("Cntrl", "SBDReleaseDelay", *HoldingBrakeDelay);
				}
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

// the holding brake of the Elf (SBD) lets go only once power has been demanded for SBDReleaseDelay (eu07/maszyna#556):
// the release waits the initial delay of the controller (IniCDelay) instead of going at the first percent of the
// demand. TDynamicObject::Update() works the holding brake out, its MED isn't in the tests, so they do here what it
// does with it: TMoverParameters::UpdateStopBrakeRelease() of the cab with its demand after every step and, where the
// brake matters, the brake's share of every car on its cylinders (LocalBrakePosAEIM)

namespace {

constexpr double step_time{0.01}; // of a step of the tests, see test::consist::step()
constexpr double elf_delay{1.0}; // IniCDelay of the Elf with a cab

// the time from the start to the first step with the holding brake let go, the brake worked out after every step;
// Act runs before every step with the time so far. negative when it holds all the time
template <typename Action> double release_time(elf &Unit, double Seconds, Action Act) {
	auto &cab{Unit.cab()};
	auto const steps{std::lround(Seconds / step_time)};
	for (long i = 0; i < steps; ++i) {
		auto const time{i * step_time};
		Act(time);
		Unit.unit.step(Unit.line);
		if (cab.UpdateStopBrakeRelease(cab.eimic_real, step_time)) {
			return time;
		}
	}
	return -1.0;
}

// the holding brake of TDynamicObject::Update() at standstill: StopBrakeDecc times the mass of the train, shared out
// by the axles, each car's share made a position of its brake valve (LocalBrakePosAEIM) as there. at standstill the
// electric brake makes no part of it
void hold(elf &Unit, bool Hold) {
	auto mass{0.0};
	auto axles{0};
	for (std::size_t i = 0; i < Unit.unit.size(); ++i) {
		mass += Unit.car(i).TotalMass;
		axles += Unit.car(i).NAxles;
	}
	auto const force{Unit.cab().StopBrakeDecc * mass};
	for (std::size_t i = 0; i < Unit.unit.size(); ++i) {
		auto &car{Unit.car(i)};
		auto const nmax{((car.P2FTrans * car.MaxBrakePress[0] - car.BrakeCylSpring) * car.BrakeCylMult[0] - car.BrakeSlckAdj) * car.BrakeCylNo * car.BrakeRigEff};
		auto const fmax{nmax * car.Hamulec->GetFC(nmax / (car.NAxles * car.NBpA), car.MED_Vref) * 1000};
		auto position{Hold ? force * car.NAxles / axles / fmax : 0.0};
		if (position > 0.009) {
			auto const slack{(car.BrakeSlckAdj / car.BrakeCylMult[0] + car.BrakeCylSpring) / (car.P2FTrans * car.MaxBrakePress[0])};
			position = slack + (1 - slack) * position;
		}
		else {
			position = 0.0;
		}
		car.LocalBrakePosAEIM = position;
	}
}

// how a start of the Elf goes with its holding brake
struct start {
	double release{-1.0}; // the time the holding brake let go
	double moving{-1.0}; // the time the train got above 0.5 km/h
	double running{-1.0}; // the time it got above 10 km/h
	double demand{0.0}; // of the controller when the brake let go
	double pressure{0.0}; // in the brake cylinders of the cab car then
};

// the brake applied at standstill, then Act before every step with the time from the start; the brake let go as
// TDynamicObject::Update() lets it go (in ShuntMode, until above 1 km/h)
template <typename Action> start start_off(elf &Unit, double Seconds, Action Act) {
	auto &cab{Unit.cab()};
	// the train stands held, its cylinders filled
	hold(Unit, true);
	Unit.wait(10.0);
	start result;
	result.pressure = cab.BrakePress;
	auto shuntmode{true};
	auto const steps{std::lround(Seconds / step_time)};
	for (long i = 0; i < steps && result.running < 0.0; ++i) {
		auto const time{i * step_time};
		Act(time);
		Unit.unit.step(Unit.line);
		auto const released{cab.UpdateStopBrakeRelease(cab.eimic_real, step_time)};
		if (cab.Vel > 1.0) {
			shuntmode = false;
		}
		hold(Unit, shuntmode && false == released);
		if (released && result.release < 0.0) {
			result.release = time;
			result.demand = cab.eimic_real;
		}
		if (cab.Vel > 0.5 && result.moving < 0.0) {
			result.moving = time;
		}
		if (cab.Vel > 10.0) {
			result.running = time;
		}
	}
	return result;
}

void flick(elf &Unit, double Time) {
	Unit.set_controller(Time < 0.5 * step_time ? 4 : 3);
}

} // namespace

TEST_CASE("elf holding brake: without SBDReleaseDelay it lets go at the first demand, as it always did") {
	vehicle car{definition::fiz("elf_v1/21we-a")};
	CHECK(car->StopBrakeDecc == doctest::Approx(0.6));
	CHECK(car->StopBrakeReleaseDelay == 0.0);
	CHECK_FALSE(car->UpdateStopBrakeRelease(0.0, step_time));
	CHECK_FALSE(car->UpdateStopBrakeRelease(-0.5, step_time));
	CHECK(car->UpdateStopBrakeRelease(0.001, step_time));
	CHECK(car->UpdateStopBrakeRelease(0.01, step_time));
	// and holds again at no demand
	CHECK_FALSE(car->UpdateStopBrakeRelease(0.0, step_time));
}

TEST_CASE("elf holding brake: SBDReleaseDelay holds it for that long of a demand") {
	vehicle car{definition::fiz("elf_v1/21we-a").set("Cntrl", "SBDReleaseDelay", 1.0)};
	REQUIRE(car->StopBrakeReleaseDelay == doctest::Approx(1.0));
	// the demand rises above 0 in the step of time 0
	long held{0};
	while (held < 500 && false == car->UpdateStopBrakeRelease(0.01, step_time)) {
		++held;
	}
	CHECK(held * step_time == doctest::Approx(1.0).epsilon(0.015));
	// it holds again once there's no demand, and waits the whole delay again for the next one
	CHECK_FALSE(car->UpdateStopBrakeRelease(0.0, step_time));
	CHECK(car->StopBrakeReleaseTimer == 0.0);
	CHECK_FALSE(car->UpdateStopBrakeRelease(0.01, step_time));
}

TEST_CASE("elf holding brake: the controller held at power") {
	SUBCASE("without SBDReleaseDelay the brake lets go at the first step of the power position, at 1%") {
		elf unit{{data::plain}};
		REQUIRE(unit.ready);
		auto const release{release_time(unit, 5.0, [&unit](double) { unit.set_controller(4); })};
		CHECK(release == doctest::Approx(0.0));
		CHECK(unit.cab().eimic_real == doctest::Approx(0.01));
	}
	SUBCASE("with SBDReleaseDelay=1.0 it lets go as the demand starts to ramp up, about 1 s on") {
		elf unit{{data::plain}, elf_delay};
		REQUIRE(unit.ready);
		auto &cab{unit.cab()};
		auto const release{release_time(unit, 5.0, [&unit](double) { unit.set_controller(4); })};
		// the 1% of the first step is all there is until then
		auto const demand{cab.eimic_real};
		// the step the ramp of the demand starts at
		auto rampstart{release};
		while (cab.eimic_real < 0.0101 && rampstart < 5.0) {
			rampstart += step_time;
			unit.unit.step(unit.line);
		}
		MESSAGE("held: release " << release << " s at " << demand << ", ramp from " << rampstart << " s");
		CHECK(release == doctest::Approx(1.0).epsilon(0.015));
		CHECK(demand == doctest::Approx(0.01).epsilon(0.02));
		// it lets go within a step of the ramp's start
		CHECK(std::abs(release - rampstart) <= 1.5 * step_time);
	}
}

TEST_CASE("elf holding brake: the controller flicked to power once and back to holding the power") {
	elf unit{{data::plain}, elf_delay};
	REQUIRE(unit.ready);
	auto &cab{unit.cab()};
	auto const release{release_time(unit, 5.0, [&unit](double Time) { flick(unit, Time); })};
	MESSAGE("flick: release " << release << " s, demand " << cab.eimic_real);
	CHECK(release == doctest::Approx(1.0).epsilon(0.015));
	// the demand stays at the 1% of the flick
	CHECK(cab.eimic_real == doctest::Approx(0.01).epsilon(0.02));
}

TEST_CASE("elf holding brake: a demand dropped to zero before the delay starts the count again") {
	elf unit{{data::plain}, elf_delay};
	REQUIRE(unit.ready);
	auto &cab{unit.cab()};
	// power for 0.5 s, the braking position for 0.2 s (no demand), then power again
	auto const release{release_time(unit, 5.0, [&unit](double Time) { unit.set_controller(Time > 0.495 && Time < 0.695 ? 2 : 4); })};
	MESSAGE("restart: release " << release << " s");
	CHECK(release == doctest::Approx(1.7).epsilon(0.01));
	CHECK(cab.eimic_real > 0.0);
}

TEST_CASE("elf holding brake: it comes back when the demand drops to zero at standstill") {
	elf unit{{data::plain}, elf_delay};
	REQUIRE(unit.ready);
	auto &cab{unit.cab()};
	auto const release{release_time(unit, 5.0, [&unit](double) { unit.set_controller(4); })};
	REQUIRE(release > 0.0);
	// the braking position takes the demand off: the brake holds from the next step
	unit.set_controller(2);
	unit.unit.step(unit.line);
	CHECK(cab.eimic_real <= 0.0);
	CHECK_FALSE(cab.UpdateStopBrakeRelease(cab.eimic_real, step_time));
	// and power demanded again waits the delay again
	auto const again{release_time(unit, 5.0, [&unit](double) { unit.set_controller(4); })};
	CHECK(again == doctest::Approx(1.0).epsilon(0.015));
}

TEST_CASE("elf holding brake: two Elfs coupled let it go as one does") {
	elf one{{data::plain}, elf_delay};
	elf two{{data::plain, data::plain}, elf_delay};
	REQUIRE(one.ready);
	REQUIRE(two.ready);
	REQUIRE(two.unit.size() == 6);
	auto const oneheld{release_time(one, 5.0, [&one](double) { one.set_controller(4); })};
	auto const twoheld{release_time(two, 5.0, [&two](double) { two.set_controller(4); })};
	CHECK(oneheld == doctest::Approx(1.0).epsilon(0.015));
	CHECK(twoheld == doctest::Approx(oneheld));
	// no demand for a while, which holds the brake, then the flick
	CHECK(release_time(one, 5.0, [&one](double) { one.set_controller(2); }) < 0.0);
	CHECK(release_time(two, 5.0, [&two](double) { two.set_controller(2); }) < 0.0);
	auto const oneflicked{release_time(one, 5.0, [&one](double Time) { flick(one, Time); })};
	auto const twoflicked{release_time(two, 5.0, [&two](double Time) { flick(two, Time); })};
	CHECK(oneflicked == doctest::Approx(1.0).epsilon(0.015));
	CHECK(twoflicked == doctest::Approx(oneflicked));
}

TEST_CASE("elf holding brake: the train doesn't move before the brake lets go, and starts off after it") {
	// the driver and the AI both put the controller to power and keep it there: TController::CheckTimeControllers()
	// drops it to holding the power once there's a demand, but IncSpeedEIM() puts it back to power in the same update
	// of the AI, so the physics sees it held
	elf now{{data::plain}};
	elf delayed{{data::plain}, elf_delay};
	REQUIRE(now.ready);
	REQUIRE(delayed.ready);
	auto const before{start_off(now, 60.0, [&now](double) { now.set_controller(4); })};
	auto const after{start_off(delayed, 60.0, [&delayed](double) { delayed.set_controller(4); })};
	MESSAGE("held brake " << before.pressure << " bar in the cab car");
	MESSAGE("without the delay: release " << before.release << " s at " << before.demand << ", moving " << before.moving << " s, 10 km/h " << before.running << " s");
	MESSAGE("with the delay: release " << after.release << " s at " << after.demand << ", moving " << after.moving << " s, 10 km/h " << after.running << " s");
	CHECK(before.pressure > 1.0);
	CHECK(before.release == doctest::Approx(0.0));
	CHECK(after.release == doctest::Approx(1.0).epsilon(0.015));
	// held until then, at the 1% of the first step
	CHECK(after.demand == doctest::Approx(0.01).epsilon(0.05));
	CHECK(after.moving > after.release);
	// both start off, the delayed one at most the delay later
	REQUIRE(before.running > 0.0);
	REQUIRE(after.running > 0.0);
	CHECK(after.running >= before.running);
	CHECK(after.running <= before.running + elf_delay + 0.05);
}

TEST_CASE("elf holding brake: the flick to power doesn't move the train before the brake lets go") {
	elf unit{{data::plain}, elf_delay};
	REQUIRE(unit.ready);
	auto const flicked{start_off(unit, 10.0, [&unit](double Time) { flick(unit, Time); })};
	MESSAGE("flick: release " << flicked.release << " s, moving " << flicked.moving << " s, speed " << unit.cab().Vel << " km/h after 10 s");
	CHECK(flicked.release == doctest::Approx(1.0).epsilon(0.015));
	CHECK((flicked.moving < 0.0 || flicked.moving > flicked.release));
}

TEST_CASE("elf holding brake: the force of the motors at 1% against the holding brake") {
	// the numbers for the report: the holding force of the train (SBD times its mass), and what the motors of the cab
	// car make at the 1% of the first step of the controller with no brake holding them back
	elf unit{{data::plain}};
	REQUIRE(unit.ready);
	auto &cab{unit.cab()};
	auto mass{0.0};
	auto powered{0};
	for (std::size_t i = 0; i < unit.unit.size(); ++i) {
		mass += unit.car(i).TotalMass;
		powered += unit.car(i).NPoweredAxles > 0 ? 1 : 0;
	}
	unit.set_controller(4);
	unit.unit.step(unit.line);
	unit.set_controller(3);
	unit.wait(5.0);
	auto const holding{cab.StopBrakeDecc * mass};
	MESSAGE("mass " << mass << " kg, holding " << holding << " N, cars with motors " << powered << ", demand " << cab.eimic_real << ", cab car Ft " << cab.Ft
	                << " N, Fful " << cab.eimv[eimv_Fful] << " kN, speed " << cab.Vel << " km/h after 5 s");
	CHECK(cab.eimic_real == doctest::Approx(0.01).epsilon(0.02));
	CHECK(cab.Ft * powered < 0.1 * holding);
}
